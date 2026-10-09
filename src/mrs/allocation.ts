/**
 * Task allocation in a robot group (course chapter 10 and §9.7, practicum ПР3): greedy vs optimal (Hungarian)
 * assignment, sequential single-item auction (SSI, MiniSum) with best insertion, CBBA (bundle building and
 * consensus with release of lost tasks), a second-price (Vickrey) sequential auction and the contract-net protocol
 * message count.
 */
import { Vec2, dist } from './rng';
import { Matrix, edgeCount, neighbors } from './graph';
import { hungarian } from '../ctl/mrta';

export function costMatrix(robots: Vec2[], tasks: Vec2[]): Matrix { return robots.map((r) => tasks.map((t) => dist(r, t))); }
/** Optimal assignment (rectangular matrices are padded); returns {robot: task} and the total cost. */
export function optimalAssignment(C: Matrix): { assignment: Record<number, number>; cost: number } {
  const n = C.length, m = C[0]?.length ?? 0; const N = Math.max(n, m); const big = 0;
  const sq = Array.from({ length: N }, (_, i) => Array.from({ length: N }, (_, j) => (i < n && j < m ? C[i][j] : big)));
  const h = hungarian(sq); const assignment: Record<number, number> = {}; let cost = 0;
  h.assignment.forEach((j, i) => { if (i < n && j < m && j >= 0) { assignment[i] = j; cost += C[i][j]; } });
  return { assignment, cost };
}
/** Greedy: repeatedly take the cheapest free (robot, task) pair; ties → smaller robot, then smaller task index. */
export function greedyAssignment(C: Matrix): { assignment: Record<number, number>; cost: number } {
  const n = C.length, m = C[0]?.length ?? 0; const fr = new Set([...Array(n).keys()]), ft = new Set([...Array(m).keys()]); const assignment: Record<number, number> = {}; let cost = 0;
  while (fr.size && ft.size) { let bi = -1, bj = -1, bc = Infinity; for (const i of fr) for (const j of ft) if (C[i][j] < bc) { bc = C[i][j]; bi = i; bj = j; } assignment[bi] = bj; cost += bc; fr.delete(bi); ft.delete(bj); }
  return { assignment, cost };
}

export function routeLength(start: Vec2, route: number[], tasks: Vec2[]): number { let s = 0, pos = start; for (const j of route) { s += dist(pos, tasks[j]); pos = tasks[j]; } return s; }
/** Best position to insert task j into a route and the length increase (ties → the smaller position). */
export function bestInsertion(start: Vec2, route: number[], tasks: Vec2[], j: number): { pos: number; inc: number } {
  const base = routeLength(start, route, tasks); let best = { pos: 0, inc: Infinity };
  for (let k = 0; k <= route.length; k++) { const r = [...route.slice(0, k), j, ...route.slice(k)]; const inc = routeLength(start, r, tasks) - base; if (inc < best.inc - 1e-12) best = { pos: k, inc }; }
  return best;
}
/** SSI auction (MiniSum): each round every robot bids its best-insertion increase on every free task; the smallest bid wins. */
export function ssiAuction(robots: Vec2[], tasks: Vec2[]): { routes: number[][]; rounds: number; bids: number; log: Array<{ round: number; task: number; winner: number; bid: number }> } {
  const routes: number[][] = robots.map(() => []); const free = new Set([...Array(tasks.length).keys()]); let bids = 0, rounds = 0; const log: Array<{ round: number; task: number; winner: number; bid: number }> = [];
  while (free.size) {
    rounds++; let best = { i: -1, j: -1, bid: Infinity, pos: 0 };
    for (let i = 0; i < robots.length; i++) for (const j of free) { const b = bestInsertion(robots[i], routes[i], tasks, j); bids++; if (b.inc < best.bid - 1e-12) best = { i, j, bid: b.inc, pos: b.pos }; }
    routes[best.i].splice(best.pos, 0, best.j); free.delete(best.j); log.push({ round: rounds, task: best.j, winner: best.i, bid: best.bid });
  }
  return { routes, rounds, bids, log };
}
export function totalLength(robots: Vec2[], routes: number[][], tasks: Vec2[]): number { return routes.reduce((s, r, i) => s + routeLength(robots[i], r, tasks), 0); }
export function makespan(robots: Vec2[], routes: number[][], tasks: Vec2[]): number { return Math.max(...routes.map((r, i) => routeLength(robots[i], r, tasks))); }
/** Exhaustive optimum of the MiniSum routing for tiny instances (≤ 2 robots × 6 tasks). */
export function bruteForceRouting(robots: Vec2[], tasks: Vec2[]): number {
  const m = tasks.length; if (robots.length > 3 || m > 7) return NaN;
  const perms = (xs: number[]): number[][] => (xs.length <= 1 ? [xs] : xs.flatMap((x, i) => perms([...xs.slice(0, i), ...xs.slice(i + 1)]).map((p) => [x, ...p])));
  const bestRoute = (start: Vec2, set: number[]) => (set.length ? Math.min(...perms(set).map((p) => routeLength(start, p, tasks))) : 0);
  let best = Infinity; const k = robots.length; const total = k ** m;
  for (let code = 0; code < total; code++) { const groups: number[][] = robots.map(() => []); let c = code; for (let j = 0; j < m; j++) { groups[c % k].push(j); c = Math.floor(c / k); } best = Math.min(best, groups.reduce((s, g, i) => s + bestRoute(robots[i], g), 0)); }
  return best;
}

/** Route score Σ R_j λ^{t_j} (discounted rewards by arrival time). */
export function pathScore(start: Vec2, path: number[], tasks: Vec2[], rewards: number[], speed = 1, lam = 0.95): number { let t = 0, pos = start, s = 0; for (const j of path) { t += dist(pos, tasks[j]) / speed; s += rewards[j] * lam ** t; pos = tasks[j]; } return s; }
export class CBBAAgent {
  bundle: number[] = []; path: number[] = []; y: number[]; z: number[];
  constructor(public idx: number, public start: Vec2, nTasks: number, public capacity: number) { this.y = Array(nTasks).fill(0); this.z = Array(nTasks).fill(-1); }
}
/** CBBA phase 1 (§10.4.2): add tasks with the largest marginal gain while they outbid the known winners and the bundle has room. */
export function cbbaBuildBundle(agent: CBBAAgent, tasks: Vec2[], rewards: number[], speed = 1, lam = 0.95): number[] {
  const added: number[] = [];
  while (agent.bundle.length < agent.capacity) {
    const base = pathScore(agent.start, agent.path, tasks, rewards, speed, lam); let bj = -1, bc = -Infinity, bpos = 0;
    for (let j = 0; j < tasks.length; j++) { if (agent.path.includes(j)) continue; let c = -Infinity, pos = 0; for (let k = 0; k <= agent.path.length; k++) { const s = pathScore(agent.start, [...agent.path.slice(0, k), j, ...agent.path.slice(k)], tasks, rewards, speed, lam) - base; if (s > c + 1e-12) { c = s; pos = k; } } if (c > agent.y[j] + 1e-12 && c > bc + 1e-12) { bc = c; bj = j; bpos = pos; } }
    if (bj < 0) break;
    agent.path.splice(bpos, 0, bj); agent.bundle.push(bj); agent.y[bj] = bc; agent.z[bj] = agent.idx; added.push(bj);
  }
  return added;
}
/** CBBA phase 2 (synchronous max-consensus over neighbours; ties → smaller winner index; −1 never wins) with release of lost tasks. Returns true when anything changed. */
export function cbbaConsensus(agents: CBBAAgent[], A: Matrix): boolean {
  const Y = agents.map((a) => a.y.slice()), Z = agents.map((a) => a.z.slice()); let changed = false; const m = Y[0]?.length ?? 0;
  for (const ag of agents) {
    const i = ag.idx; const group = [i, ...neighbors(A, i)];
    for (let j = 0; j < m; j++) {
      let by = Y[i][j], bz = Z[i][j];
      // the believed winner speaks for itself: when neighbour k (= my winner for j) no longer claims j, adopt its view (CBBA "update / reset" rows)
      for (const k of group) if (k !== i && bz === k && Z[k][j] !== k) { by = Y[k][j]; bz = Z[k][j]; }
      for (const k of group) { const y = Y[k][j], z = Z[k][j]; if (z < 0) continue; if (z === i && k !== i) continue; if (y > by + 1e-12 || (Math.abs(y - by) <= 1e-12 && bz >= 0 && z < bz) || bz < 0) { by = y; bz = z; } }
      if (by !== ag.y[j] || bz !== ag.z[j]) { ag.y[j] = by; ag.z[j] = bz; changed = true; }
    }
    const k0 = ag.bundle.findIndex((j) => ag.z[j] !== i);
    if (k0 >= 0) { const removed = ag.bundle.slice(k0); for (const j of ag.bundle.slice(k0 + 1)) if (ag.z[j] === i) { ag.y[j] = 0; ag.z[j] = -1; } ag.bundle = ag.bundle.slice(0, k0); ag.path = ag.path.filter((j) => !removed.includes(j)); changed = true; }
  }
  return changed;
}
export function runCbba(starts: Vec2[], tasks: Vec2[], rewards: number[], A: Matrix, o: { capacity?: number; maxRounds?: number; speed?: number; lam?: number } = {}): { agents: CBBAAgent[]; iterations: number; messages: number } {
  const cap = o.capacity ?? 3, maxRounds = o.maxRounds ?? 200, speed = o.speed ?? 1, lam = o.lam ?? 0.95; const agents = starts.map((s, i) => new CBBAAgent(i, s, tasks.length, cap)); const e = edgeCount(A);
  for (let it = 1; it <= maxRounds; it++) { for (const ag of agents) cbbaBuildBundle(ag, tasks, rewards, speed, lam); if (!cbbaConsensus(agents, A)) return { agents, iterations: it, messages: it * e * 2 }; }
  return { agents, iterations: maxRounds, messages: maxRounds * e * 2 };
}
export function isConflictFree(agents: CBBAAgent[]): boolean { const owner = new Map<number, number>(); for (const a of agents) for (const j of a.path) { if (owner.has(j)) return false; owner.set(j, a.idx); } for (const a of agents) for (const [j, w] of owner) if (a.z[j] !== w) return false; return true; }
export function totalScore(agents: CBBAAgent[], tasks: Vec2[], rewards: number[], speed = 1, lam = 0.95): number { return agents.reduce((s, a) => s + pathScore(a.start, a.path, tasks, rewards, speed, lam), 0); }

/** Sequential second-price (Vickrey) auction (§9.7.2): utility = value − distance; the winner pays the second-best bid on that task. Truthful bidding is dominant. */
export function vickreyAuction(robots: Vec2[], tasks: Vec2[], values: number[]): { assignment: Record<number, number>; payments: Record<number, number>; rounds: Array<{ task: number; winner: number; bid: number; price: number }> } {
  const free = new Set([...Array(tasks.length).keys()]); const busy = new Set<number>(); const assignment: Record<number, number> = {}; const payments: Record<number, number> = {}; const rounds: Array<{ task: number; winner: number; bid: number; price: number }> = [];
  while (free.size && busy.size < robots.length) {
    let best = { i: -1, j: -1, bid: -Infinity };
    for (let i = 0; i < robots.length; i++) { if (busy.has(i)) continue; for (const j of free) { const b = values[j] - dist(robots[i], tasks[j]); if (b > best.bid) best = { i, j, bid: b }; } }
    const others = [...Array(robots.length).keys()].filter((i) => i !== best.i && !busy.has(i)).map((i) => values[best.j] - dist(robots[i], tasks[best.j])); const price = others.length ? Math.max(...others) : 0;
    assignment[best.i] = best.j; payments[best.i] = price; busy.add(best.i); free.delete(best.j); rounds.push({ task: best.j, winner: best.i, bid: best.bid, price });
  }
  return { assignment, payments, rounds };
}
/** Contract-net protocol (chapter 6): announce → bid → award messages for m tasks announced to n robots, one task per round. */
export function contractNetMessages(nRobots: number, nTasks: number): { announce: number; bids: number; awards: number; total: number } { const announce = nTasks * nRobots, bids = nTasks * nRobots, awards = nTasks; return { announce, bids, awards, total: announce + bids + awards }; }
