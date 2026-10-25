/**
 * Benchmark S1 «scouting → harvest → crate logistics» (agromarca article/BENCHMARK_SCENARIO.md) on the imported
 * orchard_demo.wbt with the studio's own models — contract C2 `runS1Benchmark(station, opts)`.
 *
 *  - Automata: MissionFSM + Drone / Harvester / Logistics FSMs are studio `fsm` documents (s1_fsm.ts) run by the Control
 *    tab's FsmExecutor; their `skill=s1_*` states are bound here to the robots of the station.
 *  - Time: discrete-event semantics on a tick executor — every robot keeps a logical clock; a skill starts at the robot's
 *    logical time (or the time of the event it waits for) and ends exactly at start + duration, so event times do not
 *    depend on the executor step (§9 «дискретно-событийно»).
 *  - Motion: the drone flies the boustrophedon at 5 m / 5 m/s (aerial drive); the UTP-G platforms drive Ackermann
 *    paths (Hybrid A* + Reeds–Shepp, R_min = 7.08 m, cusps allowed, no turning in place) between lane entrances and
 *    straight lanes at 0.4 m/s (harvesters 0.6 m/s off the rows, the logist 1.0 m/s), with constant speeds per segment;
 *    a robot starts a leg only when it does not meet another robot's committed motion (it waits or plans around
 *    a standing robot). Contacts are replayed afterwards with the studio collision module (s1_world.ts).
 *  - Decisions: D1 rows → harvesters (Hungarian = SciPy LSAP on the article's cost (1), recomputed at every row_done;
 *    contract net = MuRKA contracts, ≤ 2 per robot, re-announced when a queue empties; hybrid), D2 swap requests →
 *    logists (Hungarian over {requests}×{idle logists}; contracts with score E_ratio − 0.1·n_contracts), D3 failure
 *    (level C: global recomputation vs re-contracting the failed robot's contracts).
 *  - Levels: A nominal, B sector 2 yields 2× the survey estimate, C harvester 1 fails at 50 % of its second row
 *    (its crate is lost, the rest of the row is harvested from the other side of the row).
 *  - Log: events (events.jsonl lines), logical FSM state traces, metrics of §8.
 */
import { Station, ItemType, Item } from '../../core/items/item';
import { invert, multiply, mul as mulM, transl as translM, rotz as rotzM } from '../../core/math/pose';
import { MobileRobot } from '../../mobile/items';
import { rectsOverlap } from '../../mobile/footprint';
import { parseFsm, FsmExecutor, checkFsm } from '../../ctl/fsm';
import type { ExecutorWorld } from '../../ctl/exec';
import type { WorldBindings } from '../../ctl/runtime';
import type { BTStatus, TickContext } from '../../ctl/bt';
import { PyRandom } from './pycompat';
import { linearSumAssignment } from '../../fleet/lsap';
import { murkaCost, MURKA_INF, MAX_CONTRACTS_PER_ROBOT, MurkaRobot, MurkaTask } from '../../fleet/murka_alloc';
import { S1World, Leg, Pose, S1Row, S1ContactChecker, S1Contact } from './s1_world';
import { S1_FSMS } from './s1_fsm';
import { treeProfile, TreeProfile } from './orchard';
import type { AssetStore } from '../../scene/assets';

export type S1Level = 'A' | 'B' | 'C' | 'D';
export type S1Controller = 'hungarian' | 'contract' | 'hybrid';

export interface S1Params {
  boxKg: number; pickSPerKg: number; swapS: number; unloadS: number;
  /** Speeds (mm/s): harvester off-row / in-row, logist, drone; drone altitude and climb rate (mm, mm/s), swath (mm). */
  vH: number; vHRow: number; vL: number; vD: number; altitude: number; climb: number; swath: number;
  yieldMin: number; yieldMax: number;
  /** Mission timeout (s). */
  tMax: number;
  /** Battery (u), consumption per metre and per kg harvested (u). */
  battery: number; ePerM: number; ePerKg: number;
  /** Executor step (s): only when decisions are observed, event times are exact. */
  dt: number;
  /** Level C: failure point (fraction of the failing harvester's second row). */
  failFraction: number;
  /** Footprint gap kept between robots in conflict checks (mm) and behind a harvester when swapping (mm). */
  conflictMargin: number; swapGap: number;
  /** Message model of the decisions (D1 / D2): one-way latency (s), loss probability per message (seeded), retransmission
   *  timeout (s) of unacknowledged assignments; lost announcements / bids drop the bidder from that round. */
  msgLatency: number; msgLoss: number; msgTimeout: number;
  /** Level D: reconfiguration time at the depot (s). */
  reconfigS: number;
}
export const S1_DEFAULTS: S1Params = {
  boxKg: 30, pickSPerKg: 6, swapS: 45, unloadS: 30, vH: 600, vHRow: 400, vL: 1000, vD: 5000, altitude: 5000, climb: 1000, swath: 10000,
  yieldMin: 8, yieldMax: 12, tMax: 7200, battery: 100, ePerM: 0.2, ePerKg: 0.5, dt: 0.1, failFraction: 0.5, conflictMargin: 150, swapGap: 500,
  msgLatency: 0, msgLoss: 0, msgTimeout: 2, reconfigS: 180,
};

export interface S1Options {
  level?: S1Level;
  controller?: S1Controller;
  /** Run seed (yields y_i ~ U(8, 12) kg drawn with CPython's PRNG in row / tree order). */
  seed?: number;
  params?: Partial<S1Params>;
  harvesters?: string[];
  logists?: string[];
  drone?: string;
  /** Platforms that stay parked (obstacles). */
  parked?: string[];
  /** Replay the timeline and count contacts (default true) every `contactStep` s (default 0.5). */
  contacts?: boolean;
  contactStep?: number;
  assets?: AssetStore;
  /** Reuse a world (plans are cached per world). */
  world?: S1World;
  log?: (m: string) => void;
}

export interface S1Event { t: number; type: string; robot?: string; data?: Record<string, unknown> }

export interface S1Metrics {
  outcome: 'done' | 'aborted';
  missionStart: number; surveyDone: number; makespan: number; missionEnd: number;
  kgTotal: number; kgDelivered: number; kgLost: number; completeness: number;
  boxesDelivered: number; boxesExpected: number;
  waitSwap: Record<string, number>; waitSwapTotal: number; waitSwapPercent: number;
  pickTime: Record<string, number>;
  distance: Record<string, number>; distanceByRole: Record<string, number>;
  energy: Record<string, number>; energyByRole: Record<string, number>; battery: Record<string, number>;
  trafficWait: Record<string, number>;
  rowsByHarvester: Record<string, string[]>;
  reaction: { failedRobot: string; failedAt: number; reassignedAt: number | null; reactionS: number | null; recoveredFraction: number; lostKg: number } | null;
  /** Level D: harvester → logist reconfigurations (decision, arrival at the depot slot, done). */
  reconfigurations: Array<{ robot: string; decidedAt: number; startAt: number; doneAt: number; slot: string }>;
  comm: { decisions: number; matrixCells: number; announce: number; bids: number; awards: number; messages: number; lost: number; retransmissions: number };
  contacts: { trunk: number; crown: number; static: number; robot: number; flat: number; samples: number; minCrownClearance: Record<string, number> };
  kinematics: { maxSpeed: Record<string, number>; minTurnRadius: Record<string, number>; inPlaceRotations: number; maxStep: Record<string, number> };
  planner: { calls: number; ms: number; failures: string[] };
}

export interface S1Timeline {
  end: number;
  agents: string[];
  poseAt(name: string, t: number): Pose;
  /** Pose every robot of the run at time t (playback). */
  apply(t: number): void;
}

export interface S1Result {
  level: S1Level; controller: S1Controller; seed: number;
  rows: Array<{ id: string; sector: string; laneY: number; heading: number; trees: Array<{ x: number; y: number }>; yields: number[]; estimate: number }>;
  events: S1Event[];
  /** Logical state traces (t = robot logical time) and the executors' own traces (tick times). */
  states: Record<string, Array<{ t: number; state: string }>>;
  fsmTraces: Record<string, Array<{ t: number; path: string; outcome: string; to: string }>>;
  fsmDocs: Record<string, string>;
  metrics: S1Metrics;
  contacts: S1Contact[];
  timeline: S1Timeline;
  /** events.jsonl text. */
  eventsJsonl(): string;
}

type Role = 'drone' | 'harvester' | 'logistics';
interface Lane { y: number; heading: number; entrance: Pose; stops: Pose[] }
interface RowTask { id: string; row: S1Row; lane: Lane; /** indices into row.trees, in harvest order */ trees: number[]; yields: number[]; estimate: number; remainderOf?: string; failedFrom?: string; assignedAt?: number }
interface SwapReq { id: string; h: Agent; t: number; pose: Pose; lane: Lane; kg: number; partial: boolean; logist: Agent | null; assignedAt?: number; done?: number; delivered?: number }
interface Agent {
  name: string; role: Role; robot: MobileRobot; fp: [number, number, number]; radius: number;
  lt: number; home: Pose; start: Pose; legs: Leg[]; dist: number; kg: number; fill: number; failed: boolean;
  exec: FsmExecutor | null; runs: Map<string, { end: number; onEnd?: () => void; fail?: boolean }>;
  states: Array<{ t: number; state: string }>;
  // harvester
  task: RowTask | null; queue: RowTask[]; stopIdx: number; rowsStarted: number; rowsDone: string[]; lane: Lane | null; pickS: number;
  // logist
  req: SwapReq | null; reqs: SwapReq[]; contracts: number;
  // waiting
  waitSince: number | null; trafficWait: number; homeAt: number | null; requested: boolean;
  /** harvester: logical time of the swap_done of its pending box; failure time (level C). */
  swapAt: number | null; failAt: number | null;
  /** level D: RECONFIGURE contract received at (logical time); the platform became a logist. */
  reconfigAt: number | null; reconfigured: boolean;
}

const DEG = Math.PI / 180;
const wrap180 = (a: number) => ((((a + 180) % 360) + 360) % 360) - 180;
const r4 = (v: number) => Math.round(v * 1e4) / 1e4;

const worldCache = new WeakMap<Station, S1World>();

/** S1 world of a station (cached; parked platforms and the carried crate excluded from the static obstacles). */
export function s1World(station: Station, opts: { parked?: string[]; assets?: AssetStore; footprintOf?: string } = {}): S1World {
  const c = worldCache.get(station);
  if (c) return c;
  const robots = station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT);
  const ref = robots.find((r) => r.name === (opts.footprintOf ?? 'UTP-G-01')) ?? robots.find((r) => r.kin.drive === 'ackermann');
  if (!ref) throw new Error('S1: no UTP-G platform on the station (import orchard_demo.wbt)');
  const trees = station.children.filter((it) => /tree/i.test(it.name));
  if (!trees.length) throw new Error('S1: no trees on the station');
  const profile: TreeProfile = treeProfile(trees[0] as never, opts.assets);
  const parked = (opts.parked ?? ['UTP-G-02']).map((n) => robots.find((r) => r.name === n)).filter((r): r is MobileRobot => !!r);
  const movable = station.children.filter((it) => /^crate_work/i.test(it.name));
  const w = new S1World(station, { profile, footprint: ref.kin.footprint as [number, number, number], assets: opts.assets, parked, movable });
  worldCache.set(station, w);
  return w;
}

/** Run benchmark S1 on an orchard_demo station (C2). */
export function runS1Benchmark(station: Station, opts: S1Options = {}): S1Result {
  const level: S1Level = opts.level ?? 'A';
  const controller: S1Controller = opts.controller ?? 'hungarian';
  const seed = opts.seed ?? 0;
  const P: S1Params = { ...S1_DEFAULTS, ...(opts.params ?? {}) };
  const log = opts.log ?? (() => {});
  const robots = new Map(station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT).map((r) => [r.name, r]));
  const parkedNames = opts.parked ?? ['UTP-G-02'];
  const world = opts.world ?? s1World(station, { parked: parkedNames, assets: opts.assets });
  const need = (n: string) => { const r = robots.get(n); if (!r) throw new Error(`S1: robot ${n} is not on the station`); return r; };
  const hNames = opts.harvesters ?? ['UTP-G-01', 'UTP-G-03'];
  const lNames = opts.logists ?? ['UTP-G-04'];
  const dName = opts.drone ?? 'UTP-A-01';

  // ---------------------------------------------------------------------------------------------- yields & tasks
  const rng = new PyRandom(seed);
  const rowYields = world.rows.map((r) => r.trees.map(() => rng.uniform(P.yieldMin, P.yieldMax)));
  const estimate = rowYields.map((ys) => ys.reduce((a, b) => a + b, 0));
  const actual = world.rows.map((r, i) => rowYields[i].map((y) => ((level === 'B' || level === 'D') && r.sector === 'sector_2' ? 2 * y : y)));
  const kgTotal = actual.flat().reduce((a, b) => a + b, 0);
  const laneOf = (r: S1Row): Lane => ({ y: r.laneY, heading: r.heading, entrance: r.entrance, stops: r.stops });
  const pool: RowTask[] = world.rows.map((r, i) => ({ id: r.id, row: r, lane: laneOf(r), trees: r.trees.map((_, k) => k), yields: actual[i], estimate: estimate[i] }));

  // ---------------------------------------------------------------------------------------------- agents
  const events: S1Event[] = [];
  let T = 0;
  const emit = (t: number, type: string, robot?: string, data?: Record<string, unknown>) => { events.push({ t, type, ...(robot ? { robot } : {}), ...(data ? { data } : {}) }); log(`[${t.toFixed(1)}] ${type}${robot ? ` ${robot}` : ''}${data ? ` ${JSON.stringify(data)}` : ''}`); };
  const mk = (name: string, role: Role): Agent => {
    const r = need(name);
    const z = role === 'drone' ? (r.state.z ?? r.pose()[14]) : undefined;
    const start: Pose = { x: r.state.x, y: r.state.y, th: wrap180(r.state.theta), ...(z !== undefined ? { z } : {}) };
    return { name, role, robot: r, fp: [...r.kin.footprint] as [number, number, number], radius: r.kin.minTurnRadius ?? 7000, lt: 0, home: { ...start }, start, legs: [], dist: 0, kg: 0, fill: 0, failed: false, exec: null, runs: new Map(), states: [], task: null, queue: [], stopIdx: 0, rowsStarted: 0, rowsDone: [], lane: null, pickS: 0, req: null, reqs: [], contracts: 0, waitSince: null, trafficWait: 0, homeAt: null, requested: false, swapAt: null, failAt: null, reconfigAt: null, reconfigured: false };
  };
  const drone = mk(dName, 'drone');
  const H = hNames.map((n) => mk(n, 'harvester'));
  const L = lNames.map((n) => mk(n, 'logistics'));
  const agents = [drone, ...H, ...L];
  const ground = [...H, ...L];

  // ---------------------------------------------------------------------------------------------- timelines
  const lastPose = (a: Agent): Pose => (a.legs.length ? a.legs[a.legs.length - 1].end : a.start);
  const poseAt = (a: Agent, t: number): Pose => {
    let p = a.start;
    for (const l of a.legs) { if (t < l.t0) break; p = t <= l.t1 ? l.poseAt(t) : l.end; }
    return p;
  };
  const movingAt = (a: Agent, t: number) => a.legs.some((l) => t >= l.t0 && t <= l.t1);
  const lastEnd = (a: Agent) => (a.legs.length ? a.legs[a.legs.length - 1].t1 : 0);
  const rectOf = (a: Agent, p: Pose, grow = 0) => ({ x: p.x, y: p.y, theta: p.th, L: a.fp[0] + grow, W: a.fp[1] + grow });

  /** First conflict of `legs` (starting at their t0) with the committed motion of the other ground robots. */
  const conflict = (a: Agent, legs: Leg[]): { moving?: number; blocker?: Agent } | null => {
    const t0 = legs[0].t0, tA = legs[legs.length - 1].t1;
    const mine = (t: number) => { let p = legs[0].poseAt(t0); for (const l of legs) { if (t < l.t0) break; p = t <= l.t1 ? l.poseAt(t) : l.end; } return p; };
    for (const b of ground) {
      if (b === a) continue;
      const tB = Math.max(tA, lastEnd(b));
      for (let t = t0; t <= tB + 1e-9; t += 0.25) {
        const pa = mine(t), pb = poseAt(b, t);
        if (Math.hypot(pa.x - pb.x, pa.y - pb.y) > 6000) continue;
        if (rectsOverlap(rectOf(a, pa), rectOf(b, pb), P.conflictMargin)) {
          if (movingAt(b, t) || t < lastEnd(b)) { const l = b.legs.find((x) => t >= x.t0 && t <= x.t1); return { moving: l ? l.t1 : lastEnd(b) }; }
          return { blocker: b };
        }
      }
    }
    return null;
  };

  type LegSpec = { kind: 'straight'; to: Pose; speed: number; dir?: number } | { kind: 'transit'; to: Pose; speed: number } | { kind: 'fly'; to: Pose; speed: number } | { kind: 'vertical'; z: number };
  /** Build legs from a spec list starting at (pose, t0); transit legs are planned (extra polygons: standing robots). */
  const build = (a: Agent, from: Pose, t0: number, specs: LegSpec[], extraPolys: number[][][] = []): Leg[] | null => {
    const out: Leg[] = []; let p = from, t = t0;
    for (const s of specs) {
      let leg: Leg;
      if (s.kind === 'straight') {
        const dx = s.to.x - p.x, dy = s.to.y - p.y;
        const dir = s.dir ?? (Math.cos(p.th * DEG) * dx + Math.sin(p.th * DEG) * dy >= 0 ? 1 : -1);
        if (Math.hypot(dx, dy) < 1) continue;
        leg = new Leg(a.name, [[p.x, p.y, p.th, dir], [s.to.x, s.to.y, p.th, dir]], s.speed, t, dir > 0 ? 'straight' : 'reverse');
      } else if (s.kind === 'transit') {
        if (Math.hypot(s.to.x - p.x, s.to.y - p.y) < 1 && Math.abs(wrap180(s.to.th - p.th)) < 0.5) continue;
        const path = world.plan(a.fp, a.radius, p, s.to, extraPolys);
        if (!path) return null;
        path[path.length - 1] = [s.to.x, s.to.y, s.to.th, path[path.length - 1][3]];
        leg = new Leg(a.name, path, s.speed, t, 'transit');
      } else if (s.kind === 'fly') {
        if (Math.hypot(s.to.x - p.x, s.to.y - p.y) < 1) continue;
        const th = Math.atan2(s.to.y - p.y, s.to.x - p.x) / DEG;
        leg = new Leg(a.name, [[p.x, p.y, th, 1], [s.to.x, s.to.y, th, 1]], s.speed, t, 'fly', p.z, p.z);
      } else {
        leg = new Leg(a.name, [[p.x, p.y, p.th, 1]], 1, t, 'vertical', p.z, s.z, P.climb);
      }
      out.push(leg); p = leg.end; t = leg.t1;
    }
    return out;
  };

  /**
   * Commit motion legs: start at the robot's logical time (or later when they meet committed motion of others); plan
   * around a robot that stands in the way; 'wait' when it can only be resolved by that robot moving later.
   */
  const commit = (a: Agent, specs: LegSpec[], kind: string): number | 'wait' => {
    const from = lastPose(a);
    let t0 = Math.max(a.lt, a.waitSince !== null ? T : a.lt);
    let extra: number[][][] = [];
    for (let attempt = 0; attempt < 600; attempt++) {
      const legs = build(a, from, t0, specs, extra);
      if (!legs) { if (extra.length) return 'wait'; throw new Error(`S1: no path for ${a.name} (${kind}) from (${from.x.toFixed(0)}, ${from.y.toFixed(0)})`); }
      if (!legs.length) { a.waitSince = null; return t0; }
      const c = a.role === 'drone' ? null : conflict(a, legs);
      if (!c) {
        a.trafficWait += t0 - a.lt; a.waitSince = null;
        for (const l of legs) { a.legs.push(l); a.dist += l.length; }
        emit(t0, 'move', a.name, { kind, legs: legs.length, length_m: r4(legs.reduce((s, l) => s + l.length, 0) / 1000), until: r4(legs[legs.length - 1].t1) });
        return legs[legs.length - 1].t1;
      }
      if (c.moving !== undefined) { t0 = Math.max(t0 + 0.25, c.moving + 0.25); continue; }
      if (c.blocker && !extra.length && specs.some((s) => s.kind === 'transit')) { extra = [S1World.robotPoly(lastPose(c.blocker), c.blocker.fp, 200)]; continue; }
      if (a.waitSince === null) a.waitSince = a.lt;
      return 'wait';
    }
    return 'wait';
  };

  // ---------------------------------------------------------------------------------------------- dispatcher
  const comm = { decisions: 0, matrixCells: 0, announce: 0, bids: 0, awards: 0, messages: 0, lost: 0, retransmissions: 0 };
  // message model: latency, seeded losses, retransmission of assignments; lost announcements / bids drop a bidder
  const netRng = new PyRandom(seed * 7919 + 17);
  const lost = () => P.msgLoss > 0 && netRng.random() < P.msgLoss;
  /** An acknowledged message sent at t: arrival time (retransmitted every msgTimeout s while lost). */
  const deliver = (t: number): number => { let k = 0; comm.messages++; while (lost() && k < 100) { comm.lost++; comm.retransmissions++; comm.messages++; k++; } return t + k * P.msgTimeout + P.msgLatency; };
  /** An unacknowledged message (announce, bid): false when lost. */
  const send = (): boolean => { comm.messages++; if (lost()) { comm.lost++; return false; } return true; };
  let reannounceAt = Infinity;
  let failure: S1Metrics['reaction'] = null;
  const failedTasks = new Set<string>();
  const assigned: RowTask[] = [];
  const posM = (p: Pose) => ({ x: p.x / 1000, y: p.y / 1000 });
  const battery = (a: Agent) => Math.max(0, P.battery - P.ePerM * a.dist / 1000 - P.ePerKg * a.kg);
  const hRobot = (a: Agent, at: Pose): MurkaRobot => ({ id: a.name, ...posM(at), battery: battery(a) / P.battery, caps: new Set(['harvest']) });
  const rowTask = (t: RowTask): MurkaTask => ({ id: t.id, ...posM(t.lane.entrance), caps: new Set(['harvest']), priority: 1 });
  const giveRow = (h: Agent, t: RowTask, decided: number) => {
    const at = deliver(decided);
    t.assignedAt = at; h.queue.push(t); assigned.push(t);
    const i = pool.indexOf(t); if (i >= 0) pool.splice(i, 1);
    emit(at, 'row_assigned', h.name, { row: t.id, queue: h.queue.map((q) => q.id), estimate_kg: r4(t.estimate), ...(at !== decided ? { decided_at: r4(decided) } : {}) });
    if (failure && failure.reassignedAt === null && (t.remainderOf || t.failedFrom)) failure.reassignedAt = at;
  };
  /** Hungarian over the free harvesters and the open rows. */
  const hungarianRows = (free: Agent[], at: number) => {
    const open = pool.slice();
    if (!free.length || !open.length) return;
    const C = free.map((h) => open.map((t) => murkaCost(hRobot(h, lastPose(h)), rowTask(t))));
    comm.decisions++; comm.matrixCells += free.length * open.length;
    const { rows, cols } = linearSumAssignment(C);
    rows.forEach((i, k) => { if (C[i][cols[k]] < MURKA_INF / 2) giveRow(free[i], open[cols[k]], at); });
  };
  /** MuRKA contract net over the open rows: announce, bids (cost (1)), award; ≤ K contracts per harvester. */
  const contractRows = (bidders: Agent[], at: number) => {
    const state = new Map(bidders.map((h) => [h.name, { pose: h.queue.length ? h.queue[h.queue.length - 1].lane.entrance : lastPose(h), load: h.queue.length }]));
    for (const t of pool.slice()) {
      let best: Agent | null = null, bc = MURKA_INF, eligible = 0;
      for (const h of bidders) {
        const s = state.get(h.name)!;
        if (h.failed || s.load >= MAX_CONTRACTS_PER_ROBOT) continue;
        eligible++; comm.announce++;
        if (!send()) continue; // announcement lost
        const c = murkaCost(hRobot(h, s.pose), rowTask(t));
        comm.bids++;
        if (!send()) continue; // bid lost
        if (c < bc) { bc = c; best = h; }
      }
      if (!best) { if (eligible) reannounceAt = Math.min(reannounceAt, at + 2 * P.msgLatency + P.msgTimeout); continue; }
      comm.awards++;
      const s = state.get(best.name)!; s.load++; s.pose = t.lane.entrance;
      giveRow(best, t, at + 2 * P.msgLatency);
    }
  };
  let planned = false;
  const liveH = () => H.filter((x) => !x.failed && !x.reconfigured);
  const reconfigs: S1Metrics['reconfigurations'] = [];
  const requestRow = (h: Agent, at: number) => {
    if (!planned || h.queue.length || h.failed) return;
    if (controller === 'hungarian') hungarianRows(liveH().filter((x) => !x.task && !x.queue.length), at);
    else contractRows(liveH(), at);
    // D4 (level D, MuRKA): a harvester without rows while the other still harvests takes a RECONFIGURE contract and
    // becomes a second logist (depot, reconfigS); the Hungarian baseline keeps fixed roles
    if (level === 'D' && controller !== 'hungarian' && !h.queue.length && h.reconfigAt === null && !pool.length && liveH().some((x) => x !== h && (x.task || x.queue.length))) {
      comm.announce++; comm.messages++; comm.bids++; comm.messages++; comm.awards++;
      h.reconfigAt = deliver(at);
      if (h.exec) h.exec.bb.reconfigure = true;
      emit(h.reconfigAt, 'reconfigure_assigned', h.name, { from: 'harvester', to: 'logistics', reason: 'free platform, swap requests to serve' });
    }
  };
  const initialAssignment = (at: number) => {
    planned = true;
    if (controller === 'hungarian') hungarianRows(H, at);
    else if (controller === 'contract') contractRows(H, at);
    else { hungarianRows(H, at); contractRows(H, at); } // hybrid: Hungarian first, MuRKA contracts for the rest
  };
  const pending: SwapReq[] = [];
  const reqs: SwapReq[] = [];
  const lRobot = (a: Agent): MurkaRobot => ({ id: a.name, ...posM(lastPose(a)), battery: battery(a) / P.battery, caps: new Set(['transport']) });
  const reqTask = (r: SwapReq): MurkaTask => ({ id: r.id, ...posM(r.pose), caps: new Set(['transport']), priority: 1 });
  const giveReq = (l: Agent, r: SwapReq, decided: number) => {
    const at = deliver(decided);
    r.logist = l; r.assignedAt = at; l.reqs.push(r); l.contracts++;
    const i = pending.indexOf(r); if (i >= 0) pending.splice(i, 1);
    emit(at, 'swap_request', l.name, { box: r.id, harvester: r.h.name });
  };
  const dispatchSwaps = (at: number) => {
    if (!pending.length) return;
    if (controller === 'hungarian') {
      const idle = L.filter((l) => !l.failed && !l.req && !l.reqs.length && l.requested);
      if (!idle.length) return;
      const C = idle.map((l) => pending.map((r) => murkaCost(lRobot(l), reqTask(r))));
      comm.decisions++; comm.matrixCells += idle.length * pending.length;
      const open = pending.slice();
      const { rows, cols } = linearSumAssignment(C);
      rows.forEach((i, k) => giveReq(idle[i], open[cols[k]], at));
    } else {
      for (const r of pending.slice()) {
        let best: Agent | null = null, bs = -Infinity, eligible = 0;
        for (const l of L) { if (l.failed) continue; eligible++; comm.announce++; if (!send()) continue; const score = battery(l) / P.battery - 0.1 * (l.reqs.length + (l.req ? 1 : 0)); comm.bids++; if (!send()) continue; if (score > bs) { bs = score; best = l; } }
        if (best) { comm.awards++; giveReq(best, r, at + 2 * P.msgLatency); } else if (eligible) reannounceAt = Math.min(reannounceAt, at + 2 * P.msgLatency + P.msgTimeout);
      }
    }
  };

  // ---------------------------------------------------------------------------------------------- skills
  const flags = { start: -1, surveyCmd: -1, surveyDone: -1, returnHome: -1 };
  let surveyMap: unknown = null;
  type Handler = (ctx: TickContext, args: Record<string, unknown>, node: { id: string }) => BTStatus;
  /** A skill that runs from the robot's logical time for a computed duration (or 'wait'). */
  const timed = (a: Agent, state: string, start: (ctx: TickContext) => number | 'wait' | 'fail', onEnd?: (ctx: TickContext) => void): Handler => (ctx, _args, node) => {
    let run = a.runs.get(node.id);
    if (!run) {
      if (a.failed) return 'running';
      const end = start(ctx);
      if (end === 'wait') return 'running';
      if (end === 'fail') return 'failure';
      run = { end };
      a.runs.set(node.id, run);
      a.states.push({ t: r4(a.lt), state });
    }
    if (a.failed) return 'running';
    if (T + 1e-9 >= run.end) { a.lt = Math.max(a.lt, run.end); a.runs.delete(node.id); onEnd?.(ctx); return 'success'; }
    return 'running';
  };
  /** A skill that completes when `ready()` returns a logical time (≥ the robot's), or fails when `fail()` does. */
  const waitFor = (a: Agent, state: string, ready: () => number | null, fail?: () => number | null, onEnter?: () => void): Handler => (_ctx, _args, node) => {
    if (!a.runs.has(node.id)) { a.runs.set(node.id, { end: Infinity }); a.states.push({ t: r4(a.lt), state }); onEnter?.(); }
    if (a.failed) return 'running';
    const t = ready();
    if (t !== null && t <= T + 1e-9) { a.lt = Math.max(a.lt, t); a.runs.delete(node.id); return 'success'; }
    const f = fail?.() ?? null;
    if (f !== null && f <= T + 1e-9) { a.lt = Math.max(a.lt, f); a.runs.delete(node.id); return 'failure'; }
    return 'running';
  };
  const instant = (a: Agent, state: string, fn: () => void): Handler => () => { a.states.push({ t: r4(a.lt), state }); fn(); return 'success'; };
  const homePre = (a: Agent): Pose => { const d = 4000; const p = { x: a.home.x + d * Math.cos(a.home.th * DEG), y: a.home.y + d * Math.sin(a.home.th * DEG), th: a.home.th }; return world.free(a.fp, p) ? p : a.home; };
  const outOfLane = (a: Agent, speed: number): LegSpec[] => (a.lane ? [{ kind: 'straight', to: a.lane.entrance, speed, dir: -1 }] : []);
  const leaveHome = (a: Agent, speed: number): LegSpec[] => { const p = lastPose(a); return Math.hypot(p.x - a.home.x, p.y - a.home.y) < 10 ? [{ kind: 'straight', to: homePre(a), speed, dir: 1 }] : []; };
  const goHome = (a: Agent, vOff: number, vRow: number): LegSpec[] => { const pre = homePre(a); return [...outOfLane(a, vRow), { kind: 'transit', to: pre, speed: vOff }, ...(pre === a.home ? [] : [{ kind: 'straight' as const, to: a.home, speed: vOff, dir: -1 }])]; };
  const allHome = () => { const alive = [drone, ...ground].filter((a) => !a.failed); return alive.every((a) => a.homeAt !== null) ? Math.max(...alive.map((a) => a.homeAt!)) : null; };

  // drone survey plan: boustrophedon over the trees' bounding box (+ crown radius), legs along x
  const prof = world.o.profile;
  const tx = world.trees.map((t) => t.poseAbs()[12]), ty = world.trees.map((t) => t.poseAbs()[13]);
  const ax0 = Math.min(...tx) - prof.crownRadius, ax1 = Math.max(...tx) + prof.crownRadius, ay0 = Math.min(...ty) - prof.crownRadius, ay1 = Math.max(...ty) + prof.crownRadius;
  const nLegs = Math.max(1, Math.ceil((ay1 - ay0) / P.swath));
  const legY = Array.from({ length: nLegs }, (_, k) => (nLegs === 1 ? (ay0 + ay1) / 2 : ay0 + P.swath / 2 + (k * (ay1 - ay0 - P.swath)) / (nLegs - 1)));
  const legPts = legY.map((y, k) => (k % 2 === 0 ? [[ax0, y], [ax1, y]] : [[ax1, y], [ax0, y]]));
  const altZ = P.altitude;

  const droneActions: WorldBindings['actions'] = {
    s1_wait_start: waitFor(drone, 'Wait', () => (flags.start >= 0 ? flags.start : null)),
    s1_await_survey: waitFor(drone, 'Start', () => (flags.surveyCmd >= 0 ? flags.surveyCmd : null), undefined, () => emit(drone.lt, 'o_rob_Ready', drone.name)),
    s1_takeoff: timed(drone, 'Takeoff', () => commit(drone, [{ kind: 'vertical', z: altZ }], 'takeoff')),
    s1_fly_leg: timed(drone, 'SurveyLeg/FlyLeg', (ctx) => { const k = Number(ctx.bb.leg ?? 0); const [s, e] = legPts[k]; return commit(drone, [{ kind: 'fly', to: { x: s[0], y: s[1], th: 0, z: altZ }, speed: P.vD }, { kind: 'fly', to: { x: e[0], y: e[1], th: 0, z: altZ }, speed: P.vD }], `survey leg ${k + 1}`); }, (ctx) => { ctx.bb.leg = Number(ctx.bb.leg ?? 0) + 1; ctx.bb.legs = nLegs; }),
    s1_turn_leg: timed(drone, 'SurveyLeg/TurnNextLeg', (ctx) => { const k = Number(ctx.bb.leg ?? 0); const s = legPts[k][0]; return commit(drone, [{ kind: 'fly', to: { x: s[0], y: s[1], th: 0, z: altZ }, speed: P.vD }], 'turn'); }),
    s1_report_map: instant(drone, 'ReportMap', () => {
      flags.surveyDone = drone.lt;
      surveyMap = world.rows.map((r, i) => ({ row: r.id, start: [r4(r.trees[0].x / 1000), r4(r.trees[0].y / 1000)], end: [r4(r.trees[r.trees.length - 1].x / 1000), r4(r.trees[r.trees.length - 1].y / 1000)], yield_kg: r4(estimate[i]) }));
      emit(drone.lt, 'survey_done', drone.name, { rows: surveyMap });
    }),
    s1_return_home: timed(drone, 'ReturnHome', () => commit(drone, [{ kind: 'fly', to: { ...drone.home, z: altZ }, speed: P.vD }], 'return'), () => emit(drone.lt, 'o_rob_Arrived', drone.name)),
    s1_land: timed(drone, 'Land', () => commit(drone, [{ kind: 'vertical', z: drone.home.z ?? 0 }], 'land')),
    s1_end: waitFor(drone, 'End', () => (flags.returnHome >= 0 ? flags.returnHome : null), undefined, () => {}),
  };
  // the drone's End completes on i_rob_End; o_rob_WD then
  const droneEnd = droneActions.s1_end;
  droneActions.s1_end = (ctx, args, node) => { const st = droneEnd(ctx, args, node) as BTStatus; if (st === 'success') { drone.homeAt = drone.lt; emit(drone.lt, 'o_rob_WD', drone.name); } return st; };

  const harvesterActions = (h: Agent): WorldBindings['actions'] => ({
    s1_wait_start: waitFor(h, 'Wait', () => (flags.start >= 0 ? flags.start : null)),
    s1_ready: instant(h, 'Start', () => emit(h.lt, 'o_rob_Ready', h.name)),
    s1_await_row: waitFor(h, 'AwaitRow', () => (h.queue.length ? Math.max(h.lt, h.queue[0].assignedAt ?? 0) : null), () => (h.reconfigAt !== null && !h.queue.length ? h.reconfigAt : flags.returnHome >= 0 && !h.queue.length ? flags.returnHome : null), () => { requestRow(h, h.lt); }),
    s1_reconfigure: timed(h, 'Reconfigure', () => {
      // nearest free depot slot pose (planned), then the module swap (reconfigS)
      const from = lastPose(h);
      const slots = station.children.filter((c) => /^depot_slot_/i.test(c.name)).map((c) => ({ name: c.name, x: c.poseAbs()[12], y: c.poseAbs()[13] })).sort((a, b) => Math.hypot(a.x - from.x, a.y - from.y) - Math.hypot(b.x - from.x, b.y - from.y));
      let goal: Pose | null = null, slotName = '';
      for (const s of slots) { for (const th of [180, 0, 90, -90]) { const g = { x: s.x, y: s.y, th }; if (world.free(h.fp, g) && world.plan(h.fp, h.radius, h.lane ? h.lane.entrance : from, g)) { goal = g; slotName = s.name; break; } } if (goal) break; }
      if (!goal) return 'fail';
      const end = commit(h, [...outOfLane(h, P.vHRow), { kind: 'transit', to: goal, speed: P.vH }], `to ${slotName}`);
      if (end === 'wait') return end;
      h.lane = null;
      emit(end, 'reconfigure_start', h.name, { slot: slotName, recipe: 'transporter' });
      reconfigs.push({ robot: h.name, decidedAt: h.reconfigAt ?? end, startAt: end, doneAt: end + P.reconfigS, slot: slotName });
      return end + P.reconfigS;
    }, () => emit(h.lt, 'reconfigured', h.name, { from: 'harvester', to: 'logistics' })),
    s1_goto_row: timed(h, 'GotoWaypoints/FollowWaypoints', () => {
      if (!h.task) h.task = h.queue[0];
      const t = h.task!;
      const end = commit(h, [...outOfLane(h, P.vHRow), ...leaveHome(h, P.vH), { kind: 'transit', to: t.lane.entrance, speed: P.vH }], `to ${t.id}`);
      if (end !== 'wait') h.lane = null;
      return end;
    }),
    s1_arrived: instant(h, 'GotoWaypoints/ReachedGoal', () => emit(h.lt, 'o_rob_Arrived', h.name, { row: h.task!.id })),
    s1_align_row: timed(h, 'AlignRow', (ctx) => {
      const t = h.task!;
      h.rowsStarted++; h.stopIdx = 0; h.lane = t.lane;
      ctx.bb.trees_left = t.trees.length; ctx.bb.box_full = false;
      emit(h.lt, 'i_rob_Work', h.name, { row: t.id });
      return commit(h, [{ kind: 'straight', to: t.lane.stops[t.trees[0]], speed: P.vHRow, dir: 1 }], `align ${t.id}`);
    }),
    s1_pick_tree: timed(h, 'HarvestLoop/PickTree', () => {
      const t = h.task!; const k = t.trees[h.stopIdx]; const y = t.yields[k];
      const d = y * P.pickSPerKg;
      if (level === 'C' && h === H[0] && h.rowsStarted === 2 && !failure && (h.stopIdx + 0.5) / t.trees.length >= P.failFraction - 1e-9) {
        // failure at 50 % of the second row: half of this tree picked, the robot stops for good
        const tf = h.lt + d / 2;
        h.failAt = tf;
        return tf + 1e9; // never ends normally; the runner fails the robot at tf
      }
      return h.lt + d;
    }, (ctx) => {
      const t = h.task!; const k = t.trees[h.stopIdx]; const y = t.yields[k];
      h.fill += y; h.kg += y; h.pickS += y * P.pickSPerKg;
      emit(h.lt, 'tree_picked', h.name, { row: t.id, tree: k, kg: r4(y), fill_kg: r4(h.fill) });
      const left = t.trees.length - h.stopIdx - 1;
      ctx.bb.trees_left = left;
      ctx.bb.box_full = left > 0 ? h.fill + t.yields[t.trees[h.stopIdx + 1]] > P.boxKg : h.fill > 0;
    }),
    s1_wait_swap: waitFor(h, 'HarvestLoop/WaitSwap', () => h.swapAt, undefined, () => {
      const t = h.task!;
      const id = `box${reqs.length + 1}`;
      const left = t.trees.length - h.stopIdx - 1;
      const r: SwapReq = { id, h, t: h.lt, pose: t.lane.stops[t.trees[h.stopIdx]], lane: t.lane, kg: h.fill, partial: left === 0 && h.fill < P.boxKg, logist: null };
      reqs.push(r); pending.push(r);
      h.swapAt = null;
      emit(h.lt, 'box_full', h.name, { box: id, kg: r4(h.fill), partial: r.partial, row: t.id });
      dispatchSwaps(h.lt);
    }),
    s1_advance_tree: timed(h, 'HarvestLoop/AdvanceTree', () => { const t = h.task!; h.stopIdx++; return commit(h, [{ kind: 'straight', to: t.lane.stops[t.trees[h.stopIdx]], speed: P.vHRow, dir: 1 }], `advance ${t.id}`); }),
    s1_row_done: instant(h, 'RowDone', () => {
      const t = h.task!;
      h.rowsDone.push(t.id); h.queue.shift(); h.task = null;
      emit(h.lt, 'row_done', h.name, { row: t.id });
      // contract net: open rows are re-announced whenever a harvester has a free contract slot again (K = 2)
      if (controller !== 'hungarian' && planned && pool.length) contractRows(liveH(), h.lt);
    }),
    s1_return_home: timed(h, 'ReturnHome', () => commit(h, goHome(h, P.vH, P.vHRow), 'home'), () => { h.lane = null; h.homeAt = h.lt; emit(h.lt, 'o_rob_Arrived', h.name, { home: true }); }),
    s1_end: waitFor(h, 'End', () => (flags.returnHome >= 0 ? Math.max(flags.returnHome, h.homeAt ?? 0) : null), undefined, () => {}),
  });

  const logistActions = (l: Agent): WorldBindings['actions'] => ({
    s1_wait_start: waitFor(l, 'Wait', () => (flags.start >= 0 ? flags.start : null)),
    s1_ready: instant(l, 'Start', () => emit(l.lt, 'o_rob_Ready', l.name)),
    s1_await_request: waitFor(l, 'Idle', () => (l.reqs.length ? Math.max(l.lt, l.reqs[0].assignedAt ?? 0) : null), () => (flags.returnHome >= 0 && !l.reqs.length && !pending.length ? flags.returnHome : null), () => { l.requested = true; dispatchSwaps(l.lt); }),
    s1_goto_harvester: timed(l, 'GotoHarvester', () => {
      if (!l.req) { l.req = l.reqs.shift()!; l.requested = false; }
      const r = l.req;
      const u = [Math.cos(r.lane.heading * DEG), Math.sin(r.lane.heading * DEG)];
      const back = r.h.fp[0] / 2 + l.fp[0] / 2 + P.swapGap;
      const swap: Pose = { x: r.pose.x - u[0] * back, y: r.pose.y - u[1] * back, th: r.lane.heading };
      const inside = (swap.x - r.lane.entrance.x) * u[0] + (swap.y - r.lane.entrance.y) * u[1] > 0;
      const specs: LegSpec[] = [...outOfLane(l, P.vL), ...leaveHome(l, P.vL), ...(inside ? [{ kind: 'transit' as const, to: r.lane.entrance, speed: P.vL }, { kind: 'straight' as const, to: swap, speed: P.vL, dir: 1 }] : [{ kind: 'transit' as const, to: swap, speed: P.vL }])];
      const end = commit(l, specs, `to ${r.h.name}`);
      if (end !== 'wait') l.lane = inside ? r.lane : null;
      return end;
    }, () => emit(l.lt, 'o_rob_Arrived', l.name, { at: l.req!.h.name })),
    s1_swap_box: timed(l, 'SwapBox', () => { emit(l.lt, 'i_rob_Work', l.name, { box: l.req!.id }); return l.lt + P.swapS; }, () => {
      const r = l.req!; r.done = l.lt;
      r.h.fill = 0; r.h.swapAt = l.lt;
      emit(l.lt, 'swap_done', l.name, { box: r.id, harvester: r.h.name });
    }),
    s1_goto_cp: timed(l, 'GotoCP', () => { const end = commit(l, [...outOfLane(l, P.vL), { kind: 'transit', to: world.unloadPose, speed: P.vL }], 'to CP'); if (end !== 'wait') l.lane = null; return end; }, () => emit(l.lt, 'o_rob_Arrived', l.name, { at: 'CP' })),
    s1_unload: timed(l, 'Unload', () => l.lt + P.unloadS, () => {
      const r = l.req!; r.delivered = l.lt; l.req = null; l.contracts = Math.max(0, l.contracts - 1);
      emit(l.lt, 'box_delivered', l.name, { box: r.id, kg: r4(r.kg), harvester: r.h.name, partial: r.partial });
    }),
    s1_return_home: timed(l, 'ReturnHome', () => commit(l, goHome(l, P.vL, P.vL), 'home'), () => { l.lane = null; l.homeAt = l.lt; emit(l.lt, 'o_rob_Arrived', l.name, { home: true }); }),
    s1_end: waitFor(l, 'End', () => (flags.returnHome >= 0 ? Math.max(flags.returnHome, l.homeAt ?? 0) : null), undefined, () => {}),
  });
  // End of ground robots: o_rob_WD
  for (const a of ground) void a;

  // mission
  const mission = { lt: 0, states: [] as Array<{ t: number; state: string }> };
  const execDone = (): number | null => {
    if (pool.length || ground.some((a) => a.role === 'harvester' && !a.failed && (a.task || a.queue.length))) return null;
    if (reqs.some((r) => r.delivered === undefined && !r.h.failed)) return null;
    const ts = [...events.filter((e) => e.type === 'box_delivered' || e.type === 'row_done').map((e) => e.t), flags.surveyDone];
    return Math.max(...ts);
  };
  const mWait = (state: string, ready: () => number | null): Handler => (_c, _a, node) => {
    const key = `m${node.id}`;
    if (!(mission as Record<string, unknown>)[key]) { (mission as Record<string, unknown>)[key] = true; mission.states.push({ t: r4(mission.lt), state }); }
    if (T > P.tMax) return 'failure';
    const t = ready();
    if (t !== null && t <= T + 1e-9) { mission.lt = Math.max(mission.lt, t); return 'success'; }
    return 'running';
  };
  const missionActions: WorldBindings['actions'] = {
    s1_mission_start: () => { mission.states.push({ t: 0, state: 'IDLE' }); flags.start = 0; emit(0, 'mission_start', undefined, { level, controller, seed, harvesters: hNames, logists: lNames, drone: dName }); emit(0, 'i_rob_S'); return 'success'; },
    s1_survey: mWait('SURVEY', () => { if (flags.surveyCmd < 0) { flags.surveyCmd = mission.lt; emit(mission.lt, 'i_rob_GW', drone.name, { command: 'survey' }); } return flags.surveyDone >= 0 ? flags.surveyDone : null; }),
    s1_plan: mWait('PLAN', () => { initialAssignment(mission.lt); emit(mission.lt, 'assignments_ready', undefined, { rows: H.map((h) => ({ robot: h.name, rows: h.queue.map((q) => q.id) })) }); return mission.lt; }),
    s1_execute: mWait('EXECUTE', execDone),
    s1_return: mWait('RETURN', () => { if (flags.returnHome < 0) { flags.returnHome = mission.lt; emit(mission.lt, 'return_home'); emit(mission.lt, 'i_rob_End'); } return allHome(); }),
  };

  // ---------------------------------------------------------------------------------------------- executors
  const fsmDocs = { mission: S1_FSMS.mission, drone: S1_FSMS.drone, harvester: S1_FSMS.harvester, logistics: S1_FSMS.logistics };
  const docs = Object.fromEntries(Object.entries(fsmDocs).map(([k, v]) => [k, parseFsm(v)]));
  for (const [k, d] of Object.entries(docs)) { const errs = checkFsm(d).filter((i) => i.level === 'error'); if (errs.length) throw new Error(`S1 ${k} FSM: ${errs.map((e) => `${e.where}: ${e.message}`).join('; ')}`); }
  const worldOf = (actions: WorldBindings['actions']): ExecutorWorld => { const host: WorldBindings = { sense: () => ({}), actions }; return { host, forRobot: () => null, events: () => [] }; };
  const mExec = new FsmExecutor({ doc: docs.mission, world: worldOf(missionActions) });
  drone.exec = new FsmExecutor({ doc: docs.drone, world: worldOf(droneActions) });
  for (const h of H) h.exec = new FsmExecutor({ doc: docs.harvester, world: worldOf(harvesterActions(h)) });
  for (const l of L) l.exec = new FsmExecutor({ doc: docs.logistics, world: worldOf(logistActions(l)) });
  // o_rob_WD of ground robots at their End
  const wdDone = new Set<string>();

  const failRobot = (h: Agent, tf: number) => {
    h.failed = true; h.lt = tf;
    const t = h.task!; const k = t.trees[h.stopIdx];
    const half = t.yields[k] / 2;
    h.fill += half; h.kg += half; h.pickS += half * P.pickSPerKg;
    h.states.push({ t: r4(tf), state: 'FAILED' });
    h.exec?.stop('failed');
    // the half-picked tree and the rest of the row are harvested from the other side of the row (alternate lane)
    const rest = t.trees.slice(h.stopIdx);
    const restYields = t.yields.map((y, i) => (i === k ? y - half : y));
    const otherY = 2 * t.row.trees[0].y - t.lane.y;
    const alt: Lane = { y: otherY, heading: t.lane.heading, entrance: { ...t.lane.entrance, y: otherY }, stops: t.lane.stops.map((s) => ({ ...s, y: otherY })) };
    const lanePoly = S1World.robotPoly({ x: (alt.entrance.x + alt.stops[alt.stops.length - 1].x) / 2, y: otherY, th: 0 }, [Math.abs(alt.entrance.x - alt.stops[alt.stops.length - 1].x) + h.fp[0], h.fp[1], h.fp[2]]);
    const free = !world.blocked().some((q) => polysTouch(lanePoly, q)) && !world.rows.some((r) => r !== t.row && Math.abs(r.laneY - otherY) < 10 && Math.abs(r.entrance.x - alt.entrance.x) < 10 && (pool.some((p) => p.row === r) || assigned.some((p) => p.row === r && !H.some((x) => x.rowsDone.includes(p.id)))));
    const lost = h.fill;
    failure = { failedRobot: h.name, failedAt: tf, reassignedAt: null, reactionS: null, recoveredFraction: 0, lostKg: lost };
    emit(tf, 'robot_failed', h.name, { row: t.id, tree: k, fill_kg: r4(h.fill), trees_left: rest.length, alt_lane_y_m: free ? r4(otherY / 1000) : null });
    // D3: the failed robot's contracts / rows go back to the pool
    const back = h.queue.filter((q) => q !== t);
    h.queue = []; h.task = null;
    // the failed robot's contracts are announced first (higher priority than the open rows)
    for (const q of [...back].reverse()) { q.failedFrom = h.name; pool.unshift(q); const i = assigned.indexOf(q); if (i >= 0) assigned.splice(i, 1); }
    if (free) { const rem: RowTask = { id: `${t.id}b`, row: t.row, lane: alt, trees: rest, yields: restYields, estimate: rest.reduce((a, i) => a + restYields[i], 0), remainderOf: t.id }; pool.unshift(rem); failedTasks.add(rem.id); }
    for (const q of back) failedTasks.add(q.id);
    // pending swap of the failed robot is void
    for (const r of reqs) if (r.h === h && r.done === undefined) { r.done = tf; r.delivered = undefined; const i = pending.indexOf(r); if (i >= 0) pending.splice(i, 1); }
    if (controller === 'hungarian') { comm.decisions++; hungarianRows(liveH().filter((x) => !x.task && !x.queue.length), tf); }
    else contractRows(liveH(), tf);
  };

  const dt = P.dt;
  const tStop = P.tMax + 1200;
  for (; T <= tStop; T += dt) {
    if (reannounceAt <= T + 1e-9) { const t = reannounceAt; reannounceAt = Infinity; if (planned && pool.length) contractRows(liveH(), t); dispatchSwaps(t); }
    mExec.tick(dt);
    drone.exec!.tick(dt);
    for (const h of H) {
      if (h.reconfigured) continue;
      if (h.failAt !== null && !h.failed && T + 1e-9 >= h.failAt) failRobot(h, h.failAt);
      if (!h.failed) h.exec!.tick(dt);
      if (!h.failed && h.exec?.outcome === 'reconfigured') {
        // the same platform continues with the Logistics automaton
        h.reconfigured = true; h.role = 'logistics'; h.lane = null;
        h.states.push({ t: r4(h.lt), state: 'role: logistics' });
        h.exec = new FsmExecutor({ doc: docs.logistics, world: worldOf(logistActions(h)) });
        L.push(h);
      }
    }
    for (const l of L) l.exec!.tick(dt);
    for (const a of ground) if (!a.failed && a.exec?.outcome === 'end' && !wdDone.has(a.name)) { wdDone.add(a.name); emit(a.lt, 'o_rob_WD', a.name); }
    if (mExec.status !== 'running') break;
  }
  const outcome: 'done' | 'aborted' = mExec.outcome === 'done' ? 'done' : 'aborted';
  emit(mission.lt, outcome === 'done' ? 'mission_done' : 'mission_aborted');
  events.sort((a, b) => a.t - b.t);

  // ---------------------------------------------------------------------------------------------- metrics
  const ev = (type: string) => events.filter((e) => e.type === type);
  const delivered = ev('box_delivered');
  const kgDelivered = reqs.filter((r) => r.delivered !== undefined).reduce((a, r) => a + r.kg, 0);
  const tStart = 0;
  const makespan = delivered.length ? Math.max(...delivered.map((e) => e.t)) - tStart : NaN;
  const waitSwap: Record<string, number> = {};
  for (const h of H) waitSwap[h.name] = reqs.filter((r) => r.h === h && r.done !== undefined && !(h.failed && r.done === failure?.failedAt && r.delivered === undefined)).reduce((a, r) => a + (r.done! - r.t), 0);
  const waitSwapTotal = Object.values(waitSwap).reduce((a, b) => a + b, 0);
  const role = (a: Agent) => (a.role === 'harvester' ? 'harvesters' : a.role === 'logistics' ? 'logistics' : 'drone');
  const distance = Object.fromEntries(agents.map((a) => [a.name, r4(a.dist / 1000)]));
  const energy = Object.fromEntries(agents.map((a) => [a.name, r4(P.ePerM * a.dist / 1000 + (a.role === 'harvester' ? P.ePerKg * a.kg : 0))]));
  const byRole = (m: Record<string, number>) => { const o: Record<string, number> = {}; for (const a of agents) o[role(a)] = r4((o[role(a)] ?? 0) + m[a.name]); return o; };
  if (failure) {
    const f = failure as NonNullable<S1Metrics['reaction']>;
    f.reactionS = f.reassignedAt !== null ? r4(f.reassignedAt - f.failedAt) : null;
    const recovered = [...failedTasks].filter((id) => H.some((h) => h.rowsDone.includes(id))).length;
    f.recoveredFraction = failedTasks.size ? recovered / failedTasks.size : 1;
  }
  // expected boxes: the fill rule of §4.2 over the actual yields of every task
  const boxesOf = (ys: number[]) => { let n = 0, fill = 0; ys.forEach((y, i) => { fill += y; const next = ys[i + 1]; if (next !== undefined ? fill + next > P.boxKg : fill > 0) { n++; fill = 0; } }); return n; };
  const boxesExpected = level === 'C' ? NaN : actual.reduce((a, ys) => a + boxesOf(ys), 0);

  // timeline & kinematics
  const timeline: S1Timeline = {
    end: Math.max(mission.lt, ...agents.map(lastEnd)),
    agents: agents.map((a) => a.name),
    poseAt: (name, t) => { const a = agents.find((x) => x.name === name); if (!a) throw new Error(`S1 timeline: no robot ${name}`); return poseAt(a, t); },
    apply: (t) => { for (const a of agents) { const p = poseAt(a, t); if (a.role === 'drone') { a.robot.setPose(mulM(translM(p.x, p.y, p.z ?? a.robot.pose()[14]), rotzM(p.th * DEG))); if (a.robot.kin.drive === 'aerial') a.robot.state.z = p.z ?? a.robot.state.z; } else a.robot.setPose2D(p.x, p.y, p.th); } },
  };
  const maxSpeed: Record<string, number> = {}, minR: Record<string, number> = {}, maxStep: Record<string, number> = {};
  let inPlace = 0;
  for (const a of agents) {
    maxSpeed[a.name] = Math.max(0, ...a.legs.filter((l) => l.length > 0).map((l) => l.speed));
    if (a.role !== 'drone') { const k = Math.max(0, ...a.legs.filter((l) => l.kind === 'transit').map((l) => l.maxCurvature())); minR[a.name] = k > 0 ? r4(1 / k) : Infinity; }
    for (const l of a.legs) for (let i = 1; i < l.points.length; i++) { const p = l.points[i], q = l.points[i - 1]; if (Math.hypot(p[0] - q[0], p[1] - q[1]) < 1 && Math.abs(wrap180(p[2] - q[2])) > 0.5 && a.role !== 'drone') inPlace++; }
    let prev = poseAt(a, 0); maxStep[a.name] = 0;
    for (let t = 0.1; t <= timeline.end; t += 0.1) { const p = poseAt(a, t); maxStep[a.name] = Math.max(maxStep[a.name], Math.hypot(p.x - prev.x, p.y - prev.y, (p.z ?? 0) - (prev.z ?? 0))); prev = p; }
  }

  // contacts: replay the whole run on the station items
  const contacts: S1Contact[] = [];
  let samples = 0, minCrown: Record<string, number> = {};
  if (opts.contacts !== false) {
    const all = [...agents.map((a) => a.robot), ...parkedNames.map((n) => robots.get(n)).filter((r): r is MobileRobot => !!r)];
    const saved = all.map((r) => ({ r, x: r.state.x, y: r.state.y, th: r.state.theta, z: r.state.z ?? r.pose()[14] }));
    const crate = station.children.find((c) => /^crate_work$/i.test(c.name));
    const g01 = robots.get('UTP-G-01');
    let crateRestore: (() => void) | null = null;
    if (crate && g01 && hNames.includes('UTP-G-01')) {
      // the work crate rides under UTP-G-01 (portal platform) like in Mission B
      const parent = crate.parent!, pose = crate.pose();
      const rel = multiply(invert(g01.poseAbs()), crate.poseAbs());
      g01.addChild(crate); crate.setPose(rel);
      crateRestore = () => { parent.addChild(crate); crate.setPose(pose); };
    }
    const chk = new S1ContactChecker(world, all, opts.assets);
    const step = opts.contactStep ?? 0.5;
    try {
      for (let t = 0; t <= timeline.end + 1e-9; t += step) {
        timeline.apply(t);
        const moving = new Set(agents.filter((a) => movingAt(a, t)).map((a) => a.name));
        if (moving.size) { chk.check(t, moving); samples++; }
      }
    } finally {
      for (const s of saved) { if (s.z !== undefined && s.r.name === dName) s.r.setPose(mulM(translM(s.x, s.y, s.z), rotzM(s.th * DEG))); if (s.r.kin.drive === 'aerial') s.r.state.z = s.z; s.r.setPose2D(s.x, s.y, s.th); }
      crateRestore?.();
    }
    contacts.push(...chk.contacts);
    minCrown = Object.fromEntries(Object.entries(chk.minCrownClearance).map(([k, v]) => [k, r4(v)]));
  }
  const count = (k: S1Contact['kind']) => contacts.filter((c) => c.kind === k).length;

  const metrics: S1Metrics = {
    outcome, missionStart: 0, surveyDone: flags.surveyDone, makespan, missionEnd: mission.lt,
    kgTotal, kgDelivered, kgLost: kgTotal - kgDelivered, completeness: kgTotal ? kgDelivered / kgTotal : 1,
    boxesDelivered: delivered.length, boxesExpected,
    waitSwap, waitSwapTotal, waitSwapPercent: makespan ? (100 * waitSwapTotal) / makespan : 0,
    pickTime: Object.fromEntries(H.map((h) => [h.name, h.pickS])),
    distance, distanceByRole: byRole(distance), energy, energyByRole: byRole(energy),
    battery: Object.fromEntries(agents.map((a) => [a.name, r4(P.battery - P.ePerM * a.dist / 1000 - (a.role === 'harvester' ? P.ePerKg * a.kg : 0))])),
    trafficWait: Object.fromEntries(ground.map((a) => [a.name, r4(a.trafficWait)])),
    rowsByHarvester: Object.fromEntries(H.map((h) => [h.name, [...h.rowsDone]])),
    reaction: failure, comm, reconfigurations: reconfigs,
    contacts: { trunk: count('trunk'), crown: count('crown'), static: count('static'), robot: count('robot'), flat: count('flat'), samples, minCrownClearance: minCrown },
    kinematics: { maxSpeed, minTurnRadius: minR, inPlaceRotations: inPlace, maxStep: Object.fromEntries(Object.entries(maxStep).map(([k, v]) => [k, r4(v)])) },
    planner: { calls: world.planCalls, ms: world.planMs, failures: [...world.planFailures] },
  };
  const states: Record<string, Array<{ t: number; state: string }>> = { mission: mission.states };
  for (const a of agents) states[a.name] = a.states;
  const fsmTraces: Record<string, Array<{ t: number; path: string; outcome: string; to: string }>> = { mission: mExec.trace };
  for (const a of agents) fsmTraces[a.name] = a.exec?.trace ?? [];
  return {
    level, controller, seed,
    rows: world.rows.map((r, i) => ({ id: r.id, sector: r.sector, laneY: r.laneY, heading: r.heading, trees: r.trees, yields: actual[i].map(r4), estimate: r4(estimate[i]) })),
    events, states, fsmTraces, fsmDocs, metrics, contacts, timeline,
    eventsJsonl: () => events.map((e) => JSON.stringify(e)).join('\n') + '\n',
  };
}

function polysTouch(a: number[][], b: number[][]): boolean {
  for (const P of [a, b]) for (let i = 0; i < P.length; i++) {
    const p = P[i], q = P[(i + 1) % P.length];
    const nx = q[1] - p[1], ny = p[0] - q[0];
    let amin = Infinity, amax = -Infinity, bmin = Infinity, bmax = -Infinity;
    for (const v of a) { const s = v[0] * nx + v[1] * ny; amin = Math.min(amin, s); amax = Math.max(amax, s); }
    for (const v of b) { const s = v[0] * nx + v[1] * ny; bmin = Math.min(bmin, s); bmax = Math.max(bmax, s); }
    if (amax < bmin || bmax < amin) return false;
  }
  return true;
}

export type { Item };
