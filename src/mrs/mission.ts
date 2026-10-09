/**
 * Group missions over configured robots under three control architectures (course chapter 3: centralised,
 * decentralised, hybrid). A mission is a sequence of phases (formation, formation move, task allocation,
 * gathering, coverage, return home, hold) executed by the same robots with the same radio model and faults:
 *
 *  - centralised: a coordinator (fleet manager) collects the states of the robots it can reach, plans with global
 *    information (Hungarian slot / task assignment, global formation error, global phase decisions) and sends each
 *    robot a reference; a robot without a fresh command holds its position; the coordinator can fail;
 *  - decentralised: no coordinator; each robot uses only neighbours within radio range — formation consensus with
 *    slots by index, CBBA over the radio graph, gossip of the phase counter and of finished tasks, lost-winner
 *    timeouts; partitions converge separately;
 *  - hybrid: the coordinator plans (assignments, phase advance) while it is reachable and the robots keep the
 *    formation with local consensus between updates; a robot that has not heard the coordinator for `lost`
 *    seconds falls back to the decentralised laws and returns to the plan when the coordinator is back.
 *
 * Collision avoidance is a local reflex in every architecture (barrier-function filter on sensed neighbours and
 * obstacles — it needs sensors, not radio). The class is closed-loop: `step(dt, positions?)` takes the measured
 * positions (the station robots in the runtime, or its own integration headless) and returns velocity references.
 */
import { Rng, Vec2, dist, clampNorm, mean2, sub } from './rng';
import { formationOffsets } from './consensus';
import { hungarian } from '../ctl/mrta';
import { CBBAAgent, cbbaBuildBundle, cbbaConsensus } from './allocation';
import { Matrix, diskGraph } from './graph';
import { gridPoints, voronoiLabels, massCentroids } from './coverage';
import { pairConstraint, speedPolygon, safeVelocity, HalfPlane } from './safety';

export type Architecture = 'centralized' | 'decentralized' | 'hybrid';
export const ARCHITECTURES: Architecture[] = ['centralized', 'decentralized', 'hybrid'];
export type PhaseKind = 'form' | 'goto' | 'allocate' | 'gather' | 'cover' | 'home' | 'hold';
export interface MissionPhase { kind: PhaseKind; name: string; shape: string; r: number; gain: number; at?: Vec2; zone?: string; targets: Vec2[]; targetNames: string[]; area?: [number, number, number, number]; seconds: number; speed: number; tol: number }
export interface MissionRobot { name: string; at: Vec2; home?: Vec2; speed?: number }
export interface MissionSpec {
  name: string; robots: MissionRobot[]; architecture: Architecture | 'compare'; phases: MissionPhase[];
  comm: { radius: number; drop: number; period: number; lost: number; settle: number };
  coordinator: { at: Vec2; range: number; fail?: number; recover?: number };
  safety: { dSafe: number; gamma: number; sense: number }; obstacles: Array<{ at: Vec2; r: number; name?: string }>;
  failures: Array<{ robot: number; at: number }>; duration: number; dt: number; seed: number; vmax: number; drive: 'unicycle' | 'pose';
  supervisor?: string; modes?: string;
}
export interface PhaseRecord { name: string; kind: PhaseKind; startedAt: number; doneAt: number | null }
export interface MissionMetrics { architecture: Architecture; time: number; completed: boolean; lagging: string[]; phases: PhaseRecord[]; uplink: number; downlink: number; peer: number; stalledSeconds: number; fallbackSeconds: number; minDistance: number; travelled: number; finalError: number; coordinatorDown: number; robotsAlive: number; tasksDone: number; tasksTotal: number }

interface RobotState { slotTable: Vec2[] | null; holdTurnUntil: number; holdTurns: number; escaping: boolean; phase: number; done: boolean; doneSince: number | null; cmd: Vec2 | null; cmdAt: number; cmdPhase: number; heardCoordAt: number; fallback: boolean; lastHeard: Record<number, number>; heardPhase: Record<number, number>; heardDone: Record<number, boolean>; tasksDone: Set<number>; agent: CBBAAgent | null; agentTasks: string; travelled: number; holdUntil: number | null; prev: Vec2 }

const TOL_DEFAULT = 0.08;

export class MissionSim {
  t = 0; p: Vec2[]; v: Vec2[]; alive: boolean[]; coordAlive = true; done = false; rng: Rng;
  robots: RobotState[]; coordPhase = 0; coordDoneSet = new Set<number>(); coordAssign: Record<number, Vec2 | null> = {}; coordSlots: Vec2[] | null = null; coordCentre: Vec2 | null = null; coordPhaseStart = 0; coordSeen: Record<number, { p: Vec2; t: number }> = {}; coordHoldUntil: number | null = null;
  uplink = 0; downlink = 0; peer = 0; stalled = 0; fallback = 0; minDistance = Infinity; coordinatorDown = 0; log: string[] = []; phaseRecords: PhaseRecord[]; errHistory: Array<[number, number]> = []; private lastComm = -Infinity; private slotsByIndex: Vec2[] | null = null;
  constructor(readonly spec: MissionSpec, readonly arch: Architecture) {
    this.rng = new Rng(spec.seed + (arch === 'centralized' ? 1 : arch === 'decentralized' ? 2 : 3)); this.p = spec.robots.map((r) => [r.at[0], r.at[1]] as Vec2); this.v = this.p.map(() => [0, 0] as Vec2); this.alive = this.p.map(() => true);
    this.robots = this.p.map((q) => ({ slotTable: null, holdTurnUntil: -1, holdTurns: 0, escaping: false, phase: 0, done: false, doneSince: null, cmd: null, cmdAt: -Infinity, cmdPhase: -1, heardCoordAt: -Infinity, fallback: false, lastHeard: {}, heardPhase: {}, heardDone: {}, tasksDone: new Set(), agent: null, agentTasks: '', travelled: 0, holdUntil: null, prev: [q[0], q[1]] as Vec2 }));
    this.phaseRecords = spec.phases.map((ph) => ({ name: ph.name, kind: ph.kind, startedAt: 0, doneAt: null }));
    this.say(`${arch}: ${spec.robots.length} robots, ${spec.phases.length} phases (${spec.phases.map((x) => x.name).join(' → ')})`);
  }
  private say(m: string): void { this.log.push(`${this.t.toFixed(1)} s: ${m}`); if (this.log.length > 1000) this.log.shift(); }
  private phase(k: number): MissionPhase | null { return this.spec.phases[k] ?? null; }
  private n(): number { return this.p.length; }
  private slots(ph: MissionPhase): Vec2[] { if (!this.slotsByIndex || this.slotsByIndex.length !== this.n()) this.slotsByIndex = formationOffsets(ph.shape, this.n(), ph.r); return this.slotsByIndex; }
  private homeOf(i: number): Vec2 { return this.spec.robots[i].home ?? this.spec.robots[i].at; }
  private vmaxOf(i: number): number { return this.spec.robots[i].speed ?? this.spec.vmax; }
  private inCoordRange(i: number): boolean { return this.coordAlive && dist(this.p[i], this.spec.coordinator.at) <= this.spec.coordinator.range; }
  private goal(ph: MissionPhase): Vec2 { return ph.at ?? this.coordCentre ?? mean2(this.p.filter((_, i) => this.alive[i])); }

  // --- coordinator (centralised / hybrid) --------------------------------------------------------------------
  private coordinatorPlan(linked: number[]): void {
    const known = linked.filter((i) => this.alive[i]);
    const maxPh = Math.max(this.coordPhase, ...known.map((i) => this.robots[i].phase)); if (maxPh > this.coordPhase) { this.say(`coordinator: the robots advanced to "${this.phase(maxPh)?.name ?? 'complete'}" while it was away — adopting their phase`); for (let k = this.coordPhase; k < maxPh; k++) { const rec = this.phaseRecords[k]; if (rec && rec.doneAt === null) rec.doneAt = this.t; } this.coordPhase = maxPh; this.coordSlots = null; this.coordCentre = null; this.coordDoneSet.clear(); this.coordAssign = {}; this.coordHoldUntil = null; const nx = this.phaseRecords[this.coordPhase]; if (nx && !nx.startedAt) nx.startedAt = this.t; }
    const ph = this.phase(this.coordPhase); if (!ph) return;
    for (const i of known) { this.coordSeen[i] = { p: this.p[i], t: this.t }; if (this.robots[i].phase === this.coordPhase) for (const x of this.robots[i].tasksDone) this.coordDoneSet.add(x); }
    // robots not heard for `lost` seconds are treated as lost: their tasks are reassigned
    const fresh = Object.entries(this.coordSeen).filter(([, s]) => this.t - s.t <= this.spec.comm.lost).map(([i]) => Number(i));
    if (ph.kind === 'form' || ph.kind === 'goto') {
      const slots = this.slots(ph);
      if (!this.coordSlots || !this.coordCentre) { // slots for every robot the coordinator knows (last report, or the mission's initial position): the Hungarian assignment minimises the total travel
        const all = [...Array(this.n()).keys()]; const posOf = (i: number) => this.coordSeen[i]?.p ?? this.spec.robots[i].at; const basis = fresh.length ? fresh : all; this.coordCentre = ph.kind === 'goto' ? mean2(basis.map(posOf)) : ph.at ?? mean2(basis.map(posOf)); const C = all.map((i) => slots.map((s) => dist(posOf(i), [this.coordCentre![0] + s[0], this.coordCentre![1] + s[1]]))); const h = hungarian(C); this.coordSlots = all.map((i) => slots[h.assignment[i]] ?? slots[i % slots.length]); this.say(`coordinator: slots assigned (Hungarian) for ${ph.name}`); }
      if (ph.kind === 'goto') { const g = ph.at!; const d = dist(this.coordCentre, g); const stepLen = ph.speed * this.spec.comm.period; if (d > 1e-9) this.coordCentre = d <= stepLen ? g : [this.coordCentre[0] + ((g[0] - this.coordCentre[0]) / d) * stepLen, this.coordCentre[1] + ((g[1] - this.coordCentre[1]) / d) * stepLen]; }
      for (const i of fresh) this.coordAssign[i] = [this.coordCentre[0] + this.coordSlots[i][0], this.coordCentre[1] + this.coordSlots[i][1]];
      const err = Math.sqrt(fresh.reduce((s, i) => s + dist(this.coordSeen[i].p, this.coordAssign[i]!) ** 2, 0) / Math.max(1, fresh.length)); const atGoal = ph.kind !== 'goto' || dist(this.coordCentre, ph.at!) < 1e-6;
      if (fresh.length && err < ph.tol && atGoal) this.advanceCoordinator();
    } else if (ph.kind === 'allocate') {
      const remaining = ph.targets.map((_, j) => j).filter((j) => !this.coordDoneSet.has(j)); const busy = new Set<number>(); const free: number[] = [];
      for (const i of fresh) { const a = this.coordAssign[i]; const j = a ? ph.targets.findIndex((tg) => tg === a) : -1; if (j >= 0 && !this.coordDoneSet.has(j)) { if (dist(this.coordSeen[i].p, a!) < ph.tol) { this.coordDoneSet.add(j); this.coordAssign[i] = null; free.push(i); this.say(`coordinator: ${this.spec.robots[i].name} finished ${ph.targetNames[j]}`); } else busy.add(j); } else { this.coordAssign[i] = null; free.push(i); } }
      const open = remaining.filter((j) => !this.coordDoneSet.has(j) && !busy.has(j));
      if (free.length && open.length) { const C = free.map((i) => open.map((j) => dist(this.coordSeen[i].p, ph.targets[j]))); const N = Math.max(free.length, open.length); const sq = Array.from({ length: N }, (_, a) => Array.from({ length: N }, (_, b) => (a < free.length && b < open.length ? C[a][b] : 0))); const h = hungarian(sq); free.forEach((i, a) => { const b = h.assignment[a]; if (b < open.length) this.coordAssign[i] = ph.targets[open[b]]; }); }
      for (const i of Object.keys(this.coordAssign).map(Number)) if (!fresh.includes(i) && this.coordAssign[i]) { this.say(`coordinator: ${this.spec.robots[i].name} silent — its task is reassigned`); this.coordAssign[i] = null; }
      if (this.coordDoneSet.size >= ph.targets.length) this.advanceCoordinator();
    } else if (ph.kind === 'gather') {
      const c = mean2(fresh.map((i) => this.coordSeen[i].p)); for (const i of fresh) this.coordAssign[i] = c; if (fresh.length && fresh.every((i) => dist(this.coordSeen[i].p, c) < ph.tol + this.spec.safety.dSafe * 0.6 * Math.sqrt(fresh.length))) this.advanceCoordinator();
    } else if (ph.kind === 'cover') {
      const area = ph.area!; const { q, dA } = gridPoints(area[0], area[1], area[2], area[3], Math.max(0.1, (area[1] - area[0]) / 30)); const P = fresh.map((i) => this.coordSeen[i].p); const { C } = massCentroids(P, q, q.map(() => 1), dA, voronoiLabels(P, q)); let moved = 0; fresh.forEach((i, a) => { this.coordAssign[i] = C[a]; moved = Math.max(moved, dist(P[a], C[a])); }); if (fresh.length && moved < ph.tol) this.advanceCoordinator();
    } else if (ph.kind === 'home') { for (const i of fresh) this.coordAssign[i] = this.homeOf(i); if (fresh.length && fresh.every((i) => dist(this.coordSeen[i].p, this.homeOf(i)) < ph.tol)) this.advanceCoordinator(); }
    else if (ph.kind === 'hold') { for (const i of fresh) this.coordAssign[i] = this.coordSeen[i].p; if (this.coordHoldUntil === null) this.coordHoldUntil = this.t + ph.seconds; if (this.t >= this.coordHoldUntil) this.advanceCoordinator(); }
  }
  private advanceCoordinator(): void { const rec = this.phaseRecords[this.coordPhase]; if (rec && rec.doneAt === null) rec.doneAt = this.t; this.say(`coordinator: phase "${this.phase(this.coordPhase)?.name}" complete → ${this.phase(this.coordPhase + 1)?.name ?? 'mission complete'}`); this.coordPhase++; this.coordSlots = null; this.coordCentre = null; this.coordDoneSet.clear(); this.coordAssign = {}; this.coordHoldUntil = null; const nx = this.phaseRecords[this.coordPhase]; if (nx) nx.startedAt = this.t; }

  // --- decentralised laws ------------------------------------------------------------------------------------------
  private localLaw(i: number, nb: number[], A: Matrix): { u: Vec2; done: boolean } {
    const r = this.robots[i]; const ph = this.phase(r.phase); if (!ph) return { u: [0, 0], done: true }; const pi = this.p[i]; const vmax = this.vmaxOf(i);
    if (ph.kind === 'form' || ph.kind === 'goto') {
      const slots = r.slotTable && r.slotTable.length === this.n() ? r.slotTable : this.slots(ph); const xi = sub(pi, slots[i]); let ux = 0, uy = 0, err = 0; for (const j of nb) { const xj = sub(this.p[j], slots[j]); ux -= ph.gain * (xi[0] - xj[0]); uy -= ph.gain * (xi[1] - xj[1]); err = Math.max(err, dist(xi, xj)); }
      let atGoal = true; if (ph.kind === 'goto') { const g = ph.at!; const d = dist(xi, g); atGoal = d < ph.tol; if (!atGoal) { const sp = Math.min(ph.speed, d / this.spec.comm.period); ux += ((g[0] - xi[0]) / d) * sp; uy += ((g[1] - xi[1]) / d) * sp; } } else if (ph.at) { const d = dist(xi, ph.at); if (d > ph.tol) { ux += ((ph.at[0] - xi[0]) / d) * Math.min(vmax, d); uy += ((ph.at[1] - xi[1]) / d) * Math.min(vmax, d); atGoal = false; } }
      return { u: [ux, uy], done: err < ph.tol && atGoal && nb.length > 0 };
    }
    if (ph.kind === 'allocate') {
      const remaining = ph.targets.map((_, j) => j).filter((j) => !r.tasksDone.has(j)); const key = remaining.join(','); if (!r.agent || r.agentTasks !== key) { const old = r.agent; r.agent = new CBBAAgent(i, pi, ph.targets.length, 1); if (old) { for (const j of remaining) { r.agent.y[j] = old.y[j]; r.agent.z[j] = old.z[j]; } } r.agentTasks = key; }
      const ag = r.agent; ag.start = pi; for (const j of ph.targets.keys()) if (!remaining.includes(j)) { ag.y[j] = Infinity; ag.z[j] = -2; if (ag.path.includes(j)) { ag.path = ag.path.filter((x) => x !== j); ag.bundle = ag.bundle.filter((x) => x !== j); } } // finished tasks can never be bid on
      for (const j of remaining) { const w = ag.z[j]; if (w >= 0 && w !== i && this.t - (r.lastHeard[w] ?? this.t) > this.spec.comm.lost) { ag.y[j] = 0; ag.z[j] = -1; if (ag.path.includes(j)) { ag.path = ag.path.filter((x) => x !== j); ag.bundle = ag.bundle.filter((x) => x !== j); } } }
      if (!ag.path.length) cbbaBuildBundle(ag, ph.targets, ph.targets.map(() => 10), 1, 0.9);
      const j = ag.path[0]; if (j === undefined) return { u: [0, 0], done: remaining.length === 0 };
      const g = ph.targets[j]; const d = dist(pi, g); if (d < ph.tol) { r.tasksDone.add(j); ag.path = []; ag.bundle = []; this.say(`${this.spec.robots[i].name} finished ${ph.targetNames[j]} (CBBA)`); return { u: [0, 0], done: r.tasksDone.size >= ph.targets.length }; }
      return { u: [((g[0] - pi[0]) / d) * Math.min(vmax, d), ((g[1] - pi[1]) / d) * Math.min(vmax, d)], done: false };
    }
    if (ph.kind === 'gather') { if (!nb.length) return { u: [0, 0], done: false }; const c = mean2([pi, ...nb.map((j) => this.p[j])]); const d = dist(pi, c); return { u: [ph.gain * (c[0] - pi[0]), ph.gain * (c[1] - pi[1])], done: d < ph.tol + this.spec.safety.dSafe * 0.6 * Math.sqrt(nb.length + 1) }; }
    if (ph.kind === 'cover') { const area = ph.area!; const { q, dA } = gridPoints(area[0], area[1], area[2], area[3], Math.max(0.1, (area[1] - area[0]) / 30)); const P = [pi, ...nb.map((j) => this.p[j])]; const { C } = massCentroids(P, q, q.map(() => 1), dA, voronoiLabels(P, q)); const c = C[0]; const d = dist(pi, c); return { u: [ph.gain * (c[0] - pi[0]), ph.gain * (c[1] - pi[1])], done: d < ph.tol }; }
    if (ph.kind === 'home') { const g = this.homeOf(i); const d = dist(pi, g); return { u: d > 1e-9 ? [((g[0] - pi[0]) / d) * Math.min(vmax, d), ((g[1] - pi[1]) / d) * Math.min(vmax, d)] : [0, 0], done: d < ph.tol }; }
    if (r.holdUntil === null) r.holdUntil = this.t + ph.seconds; return { u: [0, 0], done: this.t >= r.holdUntil };
    void A;
  }
  private advanceRobot(i: number): void { const r = this.robots[i]; r.phase++; r.done = false; r.doneSince = null; r.agent = null; r.agentTasks = ''; r.tasksDone.clear(); r.holdUntil = null; r.slotTable = null; }
  /** Phase k of the group is complete when every live robot is past it (decentralised / hybrid bookkeeping). */
  private sweepRecords(): void { const alive = [...Array(this.n()).keys()].filter((i) => this.alive[i]); if (!alive.length) return; const minPh = Math.min(...alive.map((i) => this.robots[i].phase)); for (let k = 0; k < Math.min(minPh, this.phaseRecords.length); k++) { const rec = this.phaseRecords[k]; if (rec.doneAt === null) { rec.doneAt = this.t; const nx = this.phaseRecords[k + 1]; if (nx && !nx.startedAt) nx.startedAt = this.t; this.say(`all robots past phase "${rec.name}"`); } } }

  /** One control step. `positions` are the measured positions (closed loop with real robots); without them the model integrates itself. Returns the velocity references. */
  step(dt: number, positions?: Vec2[]): Vec2[] {
    if (positions) this.p = positions.map((q) => [q[0], q[1]] as Vec2);
    const n = this.n(); const S = this.spec;
    for (const f of S.failures) if (f.at <= this.t && f.at > this.t - dt && this.alive[f.robot]) { this.alive[f.robot] = false; this.say(`FAULT: ${S.robots[f.robot].name} stopped (silent)`); }
    if (S.coordinator.fail !== undefined && this.coordAlive && this.t >= S.coordinator.fail && (S.coordinator.recover === undefined || this.t < S.coordinator.recover)) { this.coordAlive = false; this.say('FAULT: coordinator down'); }
    if (S.coordinator.recover !== undefined && !this.coordAlive && this.t >= S.coordinator.recover) { this.coordAlive = true; this.say('coordinator back'); }
    if (!this.coordAlive) this.coordinatorDown += dt;
    const commTick = this.t - this.lastComm >= S.comm.period - 1e-9; if (commTick) this.lastComm = this.t;
    const A = diskGraph(this.p, S.comm.radius, this.alive); if (S.comm.drop > 0 && commTick) for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) if (A[i][j] && this.rng.random() < S.comm.drop) { A[i][j] = A[j][i] = 0; }
    const nbOf = (i: number) => { const out: number[] = []; for (let j = 0; j < n; j++) if (j !== i && A[i][j] > 0) out.push(j); return out; };
    // communication round
    if (commTick) {
      if (this.arch !== 'decentralized') {
        const linked = [...Array(n).keys()].filter((i) => this.alive[i] && this.inCoordRange(i) && !(S.comm.drop > 0 && this.rng.random() < S.comm.drop)); this.uplink += linked.length;
        if (this.coordAlive) { this.coordinatorPlan(linked); for (const i of linked) { const r = this.robots[i]; r.cmd = this.coordAssign[i] ?? null; r.cmdAt = this.t; r.cmdPhase = this.coordPhase; r.heardCoordAt = this.t; r.slotTable = this.coordSlots ? this.coordSlots.map((q) => [q[0], q[1]] as Vec2) : null; if (r.phase < this.coordPhase) { r.phase = this.coordPhase; r.done = false; r.agent = null; r.agentTasks = ''; r.tasksDone.clear(); r.holdUntil = null; } if (r.phase === this.coordPhase) for (const x of this.coordDoneSet) r.tasksDone.add(x); this.downlink++; } }
      }
      if (this.arch !== 'centralized') {
        for (let i = 0; i < n; i++) { if (!this.alive[i]) continue; const r = this.robots[i]; const nb = nbOf(i); this.peer += nb.length; for (const j of nb) { const o = this.robots[j]; r.lastHeard[j] = this.t; r.heardPhase[j] = o.phase; r.heardDone[j] = o.done; for (const x of o.tasksDone) r.tasksDone.add(x); if (!r.slotTable && o.slotTable && o.phase === r.phase) r.slotTable = o.slotTable.map((q) => [q[0], q[1]] as Vec2); /* the coordinator's slot table spreads peer to peer */ if (o.phase > r.phase) { r.phase = o.phase; r.done = false; r.agent = null; r.agentTasks = ''; r.tasksDone = new Set(o.tasksDone); r.holdUntil = null; r.slotTable = o.slotTable ? o.slotTable.map((q) => [q[0], q[1]] as Vec2) : null; } } }
        // CBBA consensus among neighbours in an allocate phase
        for (let i = 0; i < n; i++) { const r = this.robots[i]; const ph = this.phase(r.phase); if (!this.alive[i] || !ph || ph.kind !== 'allocate' || !r.agent) continue; const useLocal = this.arch === 'decentralized' || r.fallback; if (!useLocal) continue; const agents = [r.agent, ...nbOf(i).map((j) => this.robots[j].agent).filter((a): a is CBBAAgent => !!a && this.robots[nbOf(i).find((jj) => this.robots[jj].agent === a)!]?.phase === r.phase)]; if (agents.length > 1) { const ids = agents.map((a) => a.idx); const sub2 = ids.map((a) => ids.map((b) => (a === b ? 0 : A[a][b]))); const remap = agents.map((a, k) => { const c = new CBBAAgent(k, a.start, a.y.length, 1); c.y = a.y; c.z = a.z.map((z) => (z < 0 ? z : ids.indexOf(z) >= 0 ? ids.indexOf(z) : -3 - z)); c.bundle = a.bundle; c.path = a.path; return c; }); cbbaConsensus(remap, sub2); remap.forEach((c, k) => { agents[k].y = c.y; agents[k].z = c.z.map((z) => (z >= 0 ? ids[z] : z <= -3 ? -3 - z : z)); agents[k].bundle = c.bundle; agents[k].path = c.path; }); } }
      }
    }
    // control
    const u: Vec2[] = this.p.map(() => [0, 0]); const poly = speedPolygon(S.vmax); let maxErr = 0; let anyStalled = false, anyFallback = false;
    for (let i = 0; i < n; i++) {
      if (!this.alive[i]) continue; const r = this.robots[i]; const nb = nbOf(i).filter((j) => this.alive[j]);
      const cmdFresh = r.cmd !== null && this.t - r.cmdAt <= S.comm.lost && r.cmdPhase === r.phase;
      let ui: Vec2 = [0, 0]; let done = false;
      const coordFresh = this.t - r.heardCoordAt <= S.comm.lost;
      if (this.arch === 'centralized') { if (cmdFresh) { const d = dist(this.p[i], r.cmd!); ui = d > 1e-9 ? [((r.cmd![0] - this.p[i][0]) / d) * Math.min(this.vmaxOf(i), d * 2), ((r.cmd![1] - this.p[i][1]) / d) * Math.min(this.vmaxOf(i), d * 2)] : [0, 0]; } else if (!coordFresh && r.phase < S.phases.length) { anyStalled = true; } r.fallback = false; }
      else if (this.arch === 'decentralized') { const res = this.localLaw(i, nb, A); ui = res.u; done = res.done; }
      else { // hybrid
        r.fallback = !coordFresh;
        if (!r.fallback && cmdFresh) { const ph = this.phase(r.phase)!; const d = dist(this.p[i], r.cmd!); ui = d > 1e-9 ? [((r.cmd![0] - this.p[i][0]) / d) * Math.min(this.vmaxOf(i), d * 2), ((r.cmd![1] - this.p[i][1]) / d) * Math.min(this.vmaxOf(i), d * 2)] : [0, 0]; if (ph.kind === 'form' || ph.kind === 'goto') { const loc = this.localLaw(i, nb, A); ui = [ui[0] + 0.5 * loc.u[0], ui[1] + 0.5 * loc.u[1]]; } }
        else { const res = this.localLaw(i, nb, A); ui = res.u; done = res.done; if (r.phase < S.phases.length) anyFallback = true; }
      }
      // local phase agreement (decentralised, or hybrid robots in fallback)
      if (this.arch === 'decentralized' || (this.arch === 'hybrid' && r.fallback)) {
        const ph = this.phase(r.phase);
        if (ph) { if (done) { r.done = true; r.doneSince ??= this.t; } else { r.done = false; r.doneSince = null; } const agree = nb.every((j) => (r.heardPhase[j] ?? r.phase) > r.phase || ((r.heardPhase[j] ?? -1) === r.phase && r.heardDone[j])); const soloOk = ph.kind === 'home' || ph.kind === 'hold' || (nb.length === 0 && ph.kind !== 'form' && ph.kind !== 'goto'); if (r.done && r.doneSince !== null && this.t - r.doneSince >= S.comm.settle && (agree || soloOk)) this.advanceRobot(i); }
      }
      // deadlock rule of ПР6 step 4 (keep right with hysteresis): a robot that wants to move but makes no progress rotates its nominal velocity by −45° for a while — symmetric jams at obstacles and crossings break consistently
      // a hold retriggered right after the previous one escalates the turn (−45° → −90° → −135° → −180°): a robot in a dead pocket between a body and a keep-out backs out instead of pushing into the wall
      { const want = Math.hypot(ui[0], ui[1]); const vm: Vec2 = dt > 0 ? [(this.p[i][0] - r.prev[0]) / dt, (this.p[i][1] - r.prev[1]) / dt] : [0, 0]; const prog = want > 1e-9 ? (vm[0] * ui[0] + vm[1] * ui[1]) / want : 1; if (this.t > 2 && want > 0.05 && prog < 0.02 && this.t >= r.holdTurnUntil) { r.holdTurns = this.t - r.holdTurnUntil <= dt + 1e-9 ? (r.holdTurns % 4) + 1 : 1; r.holdTurnUntil = this.t + 3; } else if (prog >= 0.02 && this.t >= r.holdTurnUntil) r.holdTurns = 0; if (this.t < r.holdTurnUntil && want > 1e-9) { const ang = (-Math.PI / 4) * Math.max(1, r.holdTurns); const c = Math.cos(ang), sn = Math.sin(ang); ui = [c * ui[0] - sn * ui[1], sn * ui[0] + c * ui[1]]; } }
      // safety reflex: sensed neighbours (alive or not — bodies) and obstacles
      const cons: HalfPlane[] = [...poly]; for (let j = 0; j < n; j++) if (j !== i && dist(this.p[i], this.p[j]) < S.safety.sense) cons.push(pairConstraint(this.p[i], this.p[j], S.safety.dSafe, S.safety.gamma)); for (const o of S.obstacles) if (dist(this.p[i], o.at) < S.safety.sense + o.r) cons.push(pairConstraint(this.p[i], o.at, o.r + S.safety.dSafe / 2, S.safety.gamma, 1));
      const barriers = cons.slice(poly.length); const uNom = clampNorm(ui, this.vmaxOf(i)); u[i] = safeVelocity(uNom, cons);
      // inside a keep-out already (a robot placed next to a no-go zone, a body that stopped too close): the barrier asks for an escape faster than the speed limit and the QP is infeasible — leave at the speed limit instead of freezing
      if (barriers.length && Math.hypot(u[i][0], u[i][1]) < 1e-9 && !barriers.every((c) => c.b <= 1e-9)) { const esc = clampNorm(safeVelocity(uNom, barriers), this.vmaxOf(i)); if (Math.hypot(esc[0], esc[1]) > 1e-9) { u[i] = esc; if (!r.escaping) { r.escaping = true; this.say(`${S.robots[i].name} is inside a keep-out radius — leaving at the speed limit`); } } } else r.escaping = false;
      const ph = this.phase(r.phase); if (ph && (ph.kind === 'form' || ph.kind === 'goto') && this.coordSlots) maxErr = Math.max(maxErr, 0);
    }
    if (anyStalled) this.stalled += dt; if (anyFallback) this.fallback += dt; if (this.arch !== 'centralized') this.sweepRecords();
    // formation error for the charts (against the by-index or coordinator slots of the current phase)
    const cur = this.arch === 'centralized' || this.arch === 'hybrid' ? this.phase(this.coordPhase) : this.phase(Math.min(...this.robots.filter((_, i) => this.alive[i]).map((r) => r.phase)));
    if (cur && (cur.kind === 'form' || cur.kind === 'goto')) { const slots = (this.arch !== 'decentralized' && this.coordSlots) || this.slots(cur); const alive = [...Array(n).keys()].filter((i) => this.alive[i]); const xi = alive.map((i) => sub(this.p[i], slots[i])); const c = mean2(xi); const e = Math.sqrt(xi.reduce((s, x) => s + dist(x, c) ** 2, 0) / Math.max(1, xi.length)); this.errHistory.push([this.t, e]); } else this.errHistory.push([this.t, 0]);
    for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) this.minDistance = Math.min(this.minDistance, dist(this.p[i], this.p[j]));
    for (let i = 0; i < n; i++) { this.robots[i].travelled += dist(this.p[i], this.robots[i].prev); this.robots[i].prev = [this.p[i][0], this.p[i][1]]; }
    this.v = u; this.t += dt;
    if (!positions) this.p = this.p.map((q, i) => (this.alive[i] ? [q[0] + dt * u[i][0], q[1] + dt * u[i][1]] : q));
    const complete = this.isComplete();
    if (complete && !this.done) { this.done = true; this.say('mission complete'); }
    if (this.t >= S.duration) this.done = true;
    return u.map((w, i) => (this.alive[i] ? w : [0, 0]));
  }
  /** Complete when every live robot has passed the last phase (a robot the coordinator never reached is not complete). */
  isComplete(): boolean { const alive = [...Array(this.n()).keys()].filter((i) => this.alive[i]); return alive.length > 0 && alive.every((i) => this.robots[i].phase >= this.spec.phases.length); }
  /** Live robots that have not reached the end, with their phase. */
  lagging(): string[] { return [...Array(this.n()).keys()].filter((i) => this.alive[i] && this.robots[i].phase < this.spec.phases.length).map((i) => `${this.spec.robots[i].name} (${this.phase(this.robots[i].phase)?.name})`); }
  run(): MissionMetrics { while (!this.done) this.step(this.spec.dt); return this.metrics(); }
  metrics(): MissionMetrics {
    const S = this.spec; const complete = this.isComplete();
    const tasksTotal = S.phases.filter((p) => p.kind === 'allocate').reduce((s, p) => s + p.targets.length, 0); const tasksDone = this.arch === 'centralized' ? S.phases.reduce((s, p, k) => s + (p.kind === 'allocate' ? (k < this.coordPhase ? p.targets.length : this.coordDoneSet.size) : 0), 0) : Math.max(0, ...this.robots.map((r) => S.phases.reduce((s, p, k) => s + (p.kind === 'allocate' ? (k < r.phase ? p.targets.length : k === r.phase ? r.tasksDone.size : 0) : 0), 0)));
    const last = this.errHistory.length ? this.errHistory[this.errHistory.length - 1][1] : 0;
    return { architecture: this.arch, time: this.t, completed: complete, lagging: this.lagging(), phases: this.phaseRecords.map((r) => ({ ...r })), uplink: this.uplink, downlink: this.downlink, peer: this.peer, stalledSeconds: this.stalled, fallbackSeconds: this.fallback, minDistance: this.minDistance, travelled: this.robots.reduce((s, r) => s + r.travelled, 0), finalError: last, coordinatorDown: this.coordinatorDown, robotsAlive: this.alive.filter(Boolean).length, tasksDone, tasksTotal };
  }
  status(): string { const S = this.spec; const ph = this.arch === 'centralized' ? this.phase(this.coordPhase) : this.phase(Math.min(...this.robots.filter((_, i) => this.alive[i]).map((r) => r.phase))); return `${this.arch} · phase ${ph ? `${ph.name} (${ph.kind})` : 'complete'} · coordinator ${this.arch === 'decentralized' ? 'none' : this.coordAlive ? 'up' : 'DOWN'} · msgs ↑${this.uplink} ↓${this.downlink} ⇄${this.peer}${this.fallback ? ` · fallback ${this.fallback.toFixed(0)} s` : ''}${this.stalled ? ` · stalled ${this.stalled.toFixed(0)} s` : ''} · min dist ${this.minDistance.toFixed(2)} m · robots ${this.robots.map((r, i) => `${S.robots[i].name}${this.alive[i] ? '' : '†'}:${r.phase}${r.fallback ? '*' : ''}`).join(' ')}`; }
}

/** Run the mission under each architecture headlessly (the comparison of chapter 3). */
export function compareArchitectures(spec: MissionSpec, archs: Architecture[] = ARCHITECTURES): Array<{ metrics: MissionMetrics; errHistory: Array<[number, number]>; log: string[]; sim: MissionSim }> {
  return archs.map((a) => { const sim = new MissionSim(spec, a); const metrics = sim.run(); return { metrics, errHistory: sim.errHistory, log: sim.log, sim }; });
}
