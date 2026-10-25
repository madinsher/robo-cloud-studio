/**
 * MuRKA allocation model (agromarca `murca_experiment`: baseline_hungarian.py, murca_allocator.py,
 * article_experiments._compose_minimal_fleet) as studio allocators.
 *
 * Cost of robot i for task j (article eq. 1): c_ij = w1·d_ij + w2·(B − b_i) + w3·(1 − match_ij), ∞ (1e9) when the robot
 * lacks a required capability; d in metres, b = battery ratio, defaults w = (1, 10, 20), B = 1.
 *  - `hungarianAllocate`: one-to-one optimum on the square matrix padded with 1e9 (SciPy LSAP port → same ties);
 *  - `murkaAllocate`: greedy priority-first contract net — tasks announced by descending priority (stable), every robot
 *    with < K contracts and battery ≥ 0.15 bids its cost; a robot missing exactly one capability may bid a RECONFIGURE
 *    contract (cost with battery − 5 %, + 35); the winner moves to the task, pays 2 % battery (+5 % when reconfigured);
 *  - `hybridAllocate`: Hungarian on the feasible pairs, MuRKA on the leftovers;
 *  - `composeMinimalFleet`: greedy set cover of the required capabilities over a shuffled pool.
 * The arithmetic follows the Python code operation by operation (`pyHypot` for distances), so the experiments in
 * src/murka/agro reproduce the Python results exactly. The same contract net is registered as the FleetManager allocator
 * `murka` (distance from the robot's queue end, reconfiguration through the depot reconfigurer = capabilities missing
 * but a finite fleet cost).
 */
import { pyHypot, npStd, PyRandom } from '../murka/agro/pycompat';
import { linearSumAssignment } from './lsap';
import { registerAllocator, Assignment, FleetTask } from './fleet';
import type { MobileRobot } from '../mobile/items';

export const MURKA_INF = 1e9;
export const RECONFIG_COST = 35.0;
export const RECONFIG_ENERGY_PENALTY = 5.0;
export const MAX_CONTRACTS_PER_ROBOT = 2;

export interface MurkaRobot { id: string; x: number; y: number; battery: number; caps: ReadonlySet<string> }
export interface MurkaTask { id: string; x: number; y: number; caps: ReadonlySet<string>; priority: number }
export type MurkaWeights = readonly [number, number, number];
export const MURKA_WEIGHTS: MurkaWeights = [1.0, 10.0, 20.0];

export interface AllocationResult {
  assignments: Array<[string, string]>;
  totalCost: number;
  algorithmTimeMs: number;
  unassignedTasks: string[];
  unassignedRobots: string[];
  costMatrix?: number[][];
}
export interface MurkaAllocation { base: AllocationResult; reconfigurations: Array<[string, string]> }

const now = () => (typeof performance !== 'undefined' ? performance.now() : Date.now());
const isSubset = (a: ReadonlySet<string>, b: ReadonlySet<string>) => { for (const x of a) if (!b.has(x)) return false; return true; };

/** c_ij of the article (compute_cost). */
export function murkaCost(robot: MurkaRobot, task: MurkaTask, weights: MurkaWeights = MURKA_WEIGHTS, batteryReference = 1.0): number {
  if (task.caps.size && !isSubset(task.caps, robot.caps)) return MURKA_INF;
  const [w1, w2, w3] = weights;
  const distance = pyHypot(robot.x - task.x, robot.y - task.y);
  const batteryPenalty = Math.max(0.0, batteryReference - robot.battery);
  let have = 0; for (const c of task.caps) if (robot.caps.has(c)) have++;
  const capabilityMatch = task.caps.size ? have / task.caps.size : 1.0;
  const capabilityGap = 1.0 - capabilityMatch;
  return w1 * distance + w2 * batteryPenalty + w3 * capabilityGap;
}

/** Hungarian (one task per robot) on the 1e9-padded square matrix, infeasible pairs dropped (hungarian_allocate). */
export function hungarianAllocate(robots: readonly MurkaRobot[], tasks: readonly MurkaTask[], weights: MurkaWeights = MURKA_WEIGHTS, batteryReference = 1.0): AllocationResult {
  if (!robots.length || !tasks.length) return { assignments: [], totalCost: 0.0, algorithmTimeMs: 0.0, unassignedTasks: tasks.map((t) => t.id), unassignedRobots: robots.map((r) => r.id) };
  const n = robots.length, m = tasks.length, size = Math.max(n, m);
  const matrix = Array.from({ length: size }, (_, i) => Array.from({ length: size }, (_, j) => (i < n && j < m ? murkaCost(robots[i], tasks[j], weights, batteryReference) : MURKA_INF)));
  const t0 = now();
  const { rows, cols } = linearSumAssignment(matrix);
  const elapsed = now() - t0;
  const assignments: Array<[string, string]> = [];
  let totalCost = 0.0;
  const ar = new Set<number>(), at = new Set<number>();
  rows.forEach((i, k) => {
    const j = cols[k];
    if (i >= n || j >= m || matrix[i][j] >= MURKA_INF / 2) return;
    assignments.push([robots[i].id, tasks[j].id]);
    totalCost += matrix[i][j];
    ar.add(i); at.add(j);
  });
  return {
    assignments, totalCost, algorithmTimeMs: elapsed,
    unassignedTasks: tasks.filter((_, j) => !at.has(j)).map((t) => t.id),
    unassignedRobots: robots.filter((_, i) => !ar.has(i)).map((r) => r.id),
    costMatrix: matrix.slice(0, n).map((row) => row.slice(0, m)),
  };
}

export interface MurkaOptions {
  weights?: MurkaWeights; batteryReference?: number; allowReconfig?: boolean;
  maxContracts?: number; minBattery?: number; reconfigCost?: number; reconfigEnergyPenalty?: number; moveBatteryCost?: number;
  /** Observer of the contract-net messages (announce / bid / award / no_bid) — used for logs and message counts. */
  onMessage?: (m: { kind: 'announce' | 'bid' | 'award' | 'no_bid'; task: string; robot?: string; cost?: number; reconfig?: boolean }) => void;
}

/** One announcement: the best bid (robot, cost, reconfigure?) or null (_best_robot_for_task). */
export function murkaBestBid(task: MurkaTask, robots: readonly MurkaRobot[], load: ReadonlyMap<string, number>, o: MurkaOptions = {}): { robot: MurkaRobot; cost: number; reconfig: boolean } | null {
  const weights = o.weights ?? MURKA_WEIGHTS, bref = o.batteryReference ?? 1.0, allow = o.allowReconfig ?? true;
  const K = o.maxContracts ?? MAX_CONTRACTS_PER_ROBOT, minB = o.minBattery ?? 0.15, pen = o.reconfigEnergyPenalty ?? RECONFIG_ENERGY_PENALTY;
  let best: MurkaRobot | null = null, bestCost = MURKA_INF, bestReconfig = false;
  for (const robot of robots) {
    if ((load.get(robot.id) ?? 0) >= K) continue;
    if (robot.battery < minB) continue;
    let cost = murkaCost(robot, task, weights, bref);
    let needs = false;
    if (cost >= MURKA_INF / 2 && allow) {
      const missing = [...task.caps].filter((c) => !robot.caps.has(c));
      if (missing.length && missing.length <= 1) {
        const patched: MurkaRobot = { id: robot.id, x: robot.x, y: robot.y, battery: Math.max(0.0, robot.battery - pen / 100.0), caps: new Set([...robot.caps, ...missing]) };
        cost = murkaCost(patched, task, weights, bref) + (o.reconfigCost ?? RECONFIG_COST);
        needs = true;
      }
    }
    if (cost < MURKA_INF / 2) o.onMessage?.({ kind: 'bid', task: task.id, robot: robot.id, cost, reconfig: needs });
    if (cost < bestCost) { best = robot; bestCost = cost; bestReconfig = needs; }
  }
  if (!best || bestCost >= MURKA_INF / 2) return null;
  return { robot: best, cost: bestCost, reconfig: bestReconfig };
}

/** MuRKA greedy priority-first contract allocation (murca_allocate). */
export function murkaAllocate(robots: readonly MurkaRobot[], tasks: readonly MurkaTask[], o: MurkaOptions = {}): MurkaAllocation {
  const t0 = now();
  const pen = o.reconfigEnergyPenalty ?? RECONFIG_ENERGY_PENALTY, move = o.moveBatteryCost ?? 0.02;
  const sorted = tasks.map((t, i) => [t, i] as const).sort((a, b) => b[0].priority - a[0].priority || a[1] - b[1]).map(([t]) => t);
  const load = new Map(robots.map((r) => [r.id, 0]));
  let state = [...robots];
  const assignments: Array<[string, string]> = [], reconfigurations: Array<[string, string]> = [], unassigned: string[] = [];
  let totalCost = 0.0;
  for (const task of sorted) {
    o.onMessage?.({ kind: 'announce', task: task.id });
    const bid = murkaBestBid(task, state, load, o);
    if (!bid) { unassigned.push(task.id); o.onMessage?.({ kind: 'no_bid', task: task.id }); continue; }
    const best = bid.robot;
    assignments.push([best.id, task.id]);
    totalCost += bid.cost;
    load.set(best.id, (load.get(best.id) ?? 0) + 1);
    o.onMessage?.({ kind: 'award', task: task.id, robot: best.id, cost: bid.cost, reconfig: bid.reconfig });
    let caps = best.caps;
    if (bid.reconfig) { caps = new Set([...best.caps, ...task.caps]); reconfigurations.push([best.id, task.id]); }
    const updated: MurkaRobot = { id: best.id, x: task.x, y: task.y, battery: Math.max(0.0, best.battery - (!bid.reconfig ? move : move + pen / 100.0)), caps };
    state = state.map((r) => (r.id === best.id ? updated : r));
  }
  const elapsed = now() - t0;
  const assigned = new Set(assignments.map(([r]) => r));
  return { base: { assignments, totalCost, algorithmTimeMs: elapsed, unassignedTasks: unassigned, unassignedRobots: robots.filter((r) => !assigned.has(r.id)).map((r) => r.id) }, reconfigurations };
}

/** Hungarian on feasible pairs first, MuRKA (with reconfiguration) on the leftovers (hybrid_allocate). */
export function hybridAllocate(robots: readonly MurkaRobot[], tasks: readonly MurkaTask[], o: MurkaOptions = {}): MurkaAllocation {
  const h = hungarianAllocate(robots, tasks, o.weights, o.batteryReference);
  const done = new Set(h.assignments.map(([, t]) => t));
  const left = tasks.filter((t) => !done.has(t.id));
  if (!left.length) return { base: h, reconfigurations: [] };
  const second = murkaAllocate(robots, left, { ...o, allowReconfig: true });
  return {
    base: { assignments: [...h.assignments, ...second.base.assignments], totalCost: h.totalCost + second.base.totalCost, algorithmTimeMs: h.algorithmTimeMs + second.base.algorithmTimeMs, unassignedTasks: second.base.unassignedTasks, unassignedRobots: second.base.unassignedRobots },
    reconfigurations: [...second.reconfigurations],
  };
}

/** σ of the number of tasks per robot (numpy std). */
export function loadImbalance(robots: readonly MurkaRobot[], assignments: ReadonlyArray<readonly [string, string]>): number {
  const by = new Map(robots.map((r) => [r.id, 0]));
  for (const [rid] of assignments) by.set(rid, (by.get(rid) ?? 0) + 1);
  const values = [...by.values()];
  return values.length ? npStd(values) : 0.0;
}

/** Greedy minimum-fleet composition covering `required` (exp1 morphogenesis): robots shuffled with the seed's PRNG. */
export function composeMinimalFleet(pool: readonly MurkaRobot[], required: ReadonlySet<string>, rng: PyRandom): { chosen: MurkaRobot[]; covered: Set<string>; ms: number } {
  const remainingAll = rng.shuffle([...pool]);
  let remaining = remainingAll;
  const chosen: MurkaRobot[] = [];
  const covered = new Set<string>();
  const gain = (r: MurkaRobot) => { let g = 0; for (const c of r.caps) if (required.has(c) && !covered.has(c)) g++; return g; };
  const t0 = now();
  const missing = () => [...required].some((c) => !covered.has(c));
  while (missing() && remaining.length) {
    let best = remaining[0], bg = gain(best);
    for (const r of remaining) { const g = gain(r); if (g > bg) { best = r; bg = g; } }
    if (!bg) break;
    chosen.push(best);
    for (const c of best.caps) covered.add(c);
    remaining = remaining.filter((r) => r.id !== best.id);
  }
  return { chosen, covered, ms: now() - t0 };
}

// ------------------------------------------------------------------------------------------------------------------
// FleetManager allocator `murka`: the same contract net on the live fleet
// ------------------------------------------------------------------------------------------------------------------

const capsOfTask = (t: FleetTask) => new Set(t.requiredCapabilities);

/**
 * Tasks are announced in the fleet's priority order; every robot with fewer than K queued contracts and ≥ 15 % battery
 * bids w1·d + w2·(1 − b) (d in m from where its queue ends); a robot lacking capabilities bids only when the fleet can
 * reconfigure it (finite fleet cost via the depot reconfigurer), + 35 and the 5 % battery penalty; the lowest bid wins.
 */
registerAllocator('murka', (ctx) => {
  const K = Math.max(1, Math.min(ctx.queueLimit, MAX_CONTRACTS_PER_ROBOT));
  const ends = new Map(ctx.robots.map((r) => [r.id, ctx.queueEnd(r)]));
  const battery = new Map(ctx.robots.map((r) => [r.id, r.batteryLevel()]));
  const out: Assignment[] = [];
  for (const t of ctx.tasks) {
    let best: MobileRobot | null = null, bc = MURKA_INF, reconf = false;
    for (const r of ctx.robots) {
      const e = ends.get(r.id)!;
      if (e.n >= K + (r.state.taskId ? 1 : 0)) continue;
      const b = battery.get(r.id)!;
      if (b < 0.15) continue;
      const fleetCost = ctx.cost(r, t, e);
      if (!Number.isFinite(fleetCost)) continue;
      const lacks = t.requiredCapabilities.some((c) => !r.capabilities.includes(c));
      const me: MurkaRobot = { id: r.id, x: e.x / 1000, y: e.y / 1000, battery: lacks ? Math.max(0, b - RECONFIG_ENERGY_PENALTY / 100) : b, caps: lacks ? new Set([...r.capabilities, ...t.requiredCapabilities]) : new Set(r.capabilities) };
      const c = murkaCost(me, { id: t.id, x: t.location[0] / 1000, y: t.location[1] / 1000, caps: capsOfTask(t), priority: t.priority }) + (lacks ? RECONFIG_COST : 0);
      if (c < bc) { bc = c; best = r; reconf = lacks; }
    }
    if (!best) continue;
    out.push({ task: t, robot: best });
    const e = ends.get(best.id)!;
    const end = t.workPath?.length ? t.workPath[t.workPath.length - 1] : t.location;
    const until = ctx.fm.time + ctx.cost(best, t, e);
    ends.set(best.id, { x: end[0], y: end[1], t: until, n: e.n + 1 });
    battery.set(best.id, Math.max(0, battery.get(best.id)! - (reconf ? 0.07 : 0.02)));
    ctx.claim(t, best, until);
  }
  return out;
});
