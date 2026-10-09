/**
 * Fleet runtime: runs a group-control model on the mobile robots of the station inside the world loop. The model
 * is stepped in metres (station mm / 1000), the robots' poses are written back every tick, so the same
 * consensus / formation / swarm / coverage / safety / MAPF / warehouse logic that the analysis evaluates is
 * visible in the 3D view. Independent of the UI: the app wires `tick` into `app.worldHooks` (see ui/control_ui.ts).
 */
import { MobileRobot, MapItem, ZoneItem } from '../mobile/items';
import { Station, ItemType } from '../core/items/item';
import { integrate } from '../mobile/controller';
import { DEG, RAD } from '../core/math/pose';
import { unicycleFromVelocity } from './sim';
import { MissionSim, MissionSpec } from './mission';
import { parseMission, parseDes, parseHybrid } from './dsl_bridge';
import { DES, parallel, supcon, supervisorTable, SupervisorRuntime, SupervisorViolation } from '../ctl/des';
import { ModeMachine } from '../ctl/hybrid';
import { controlModels } from '../ctl/model';
import { Rng, Vec2, dist, clampNorm, mean2 } from './rng';
import { MrsKind } from './model';
import { parseConsensus, parseSwarm, parseCoverage, parseSafety, parseGridMapf, parseWarehouse, ConsensusDoc, SwarmDoc, CoverageDoc, SafetyDoc, GridMapfDoc, WarehouseDoc } from './dsl';
import { buildGraph, coverageSetup, safetyInstance, gridInstance } from './an_practicum';
import { diskGraph, edgeCount, Matrix } from './graph';
import { formationVelocity, formationError, formationOffsets, rendezvousVelocity, consensusStep, disagreement } from './consensus';
import { boidsAcceleration, polarization, minPairDistance, psoVelocity, repulsion, updateBest, gaussianSource, ringNeighborhood } from './swarm';
import { voronoiLabels, massCentroids, coverageCost, limitedCost } from './coverage';
import { cbfController, unstuckNominal, goToGoal, speedPolygon, safeVelocity, pairConstraint, minPairDistance as minDist } from './safety';
import { cbs, prioritizedPlanning, actionDependencies, Path, Action } from './mapf';
import { FleetSim, FleetConfig, Warehouse } from './warehouse';

export const RUNNABLE_KINDS: MrsKind[] = ['mission', 'consensus', 'swarm', 'coverage', 'safety', 'gridmapf', 'warehouse'];
export interface FleetRuntimeOptions { kind: MrsKind; source: string; robots: MobileRobot[]; station: Station; log?: (m: string) => void; /** model metres per station mm (default 1/1000) */ scale?: number; /** speed limit, m/s (default: from the document or 0.5) */ vmax?: number; maxSeconds?: number; /** create missing robots (warehouse names) */ createRobots?: (name: string, x: number, y: number) => MobileRobot; /** how the reference of the group law reaches the robot: `pose` writes the pose, `unicycle` sends (v, ω) through the robot's own kinematic controller (limits of the configured robot apply) */ drive?: 'pose' | 'unicycle'; /** distance ahead of the axle of the tracked point for the unicycle transform, m */ lookahead?: number; /** environment for the mode machine / supervisor (human position, e-stop…) */ env?: () => Record<string, number | boolean> }

interface Driver { name: string; tick(dt: number): void; status(): string; done(): boolean; positions(): Vec2[]; robots: MobileRobot[]; /** the underlying model (missions expose their `MissionSim`) */ model?: unknown }

export class FleetRuntime {
  time = 0; ticks = 0; done = false; log: string[] = []; readonly driver: Driver; readonly kind: MrsKind; private say: (m: string) => void; private maxSeconds: number; private beforeTick: (dt: number) => void = () => {};
  constructor(o: FleetRuntimeOptions) {
    this.kind = o.kind; this.maxSeconds = o.maxSeconds ?? 3600; this.say = (m) => { this.log.push(m); if (this.log.length > 500) this.log.shift(); o.log?.(m); };
    const scale = o.scale ?? 1 / 1000; const toM = (r: MobileRobot): Vec2 => [r.state.x * scale, r.state.y * scale];
    const drive = o.drive ?? (o.kind === 'mission' ? parseMission(o.source).drive : 'pose'); const look = o.lookahead ?? 0.15; let lastDt = 0.05;
    /** Track the model point p[i] (feed-forward u[i]) with the configured robot's own controller: desired point velocity → (v, ω) through a point `look` m ahead of the axle → `integrate` with the kinematic limits of the robot. */
    const driveTo = (r: MobileRobot, target: Vec2, uff: Vec2, dt: number) => { const th = r.state.theta * DEG; const px = r.state.x * scale + look * Math.cos(th), py = r.state.y * scale + look * Math.sin(th); const k = 1.5; const ux = uff[0] + k * (target[0] - px), uy = uff[1] + k * (target[1] - py); const { v, w } = unicycleFromVelocity(th, [ux, uy], look); integrate(r, { v: v / scale, omega: w * RAD }, dt); r.syncPoseFromState(); r.state.status = Math.abs(r.state.v) > 1 ? 'moving' : 'idle'; };
    const place = (robots: MobileRobot[], p: Vec2[], u?: Vec2[]) => { robots.forEach((r, i) => { if (!p[i]) return; if (drive === 'unicycle') { driveTo(r, p[i], u?.[i] ?? [0, 0], lastDt); return; } const x = p[i][0] / scale, y = p[i][1] / scale; const v = u?.[i]; const th = v && Math.hypot(v[0], v[1]) > 1e-6 ? (Math.atan2(v[1], v[0]) * 180) / Math.PI : r.state.theta; r.setPose2D(x, y, th); r.state.v = v ? Math.hypot(v[0], v[1]) / scale : 0; r.state.status = v && Math.hypot(v[0], v[1]) > 1e-4 ? 'moving' : 'idle'; }); };
    this.beforeTick = (dt) => { lastDt = dt; };
    const kinVmax = (rs: MobileRobot[]) => Math.min(...rs.map((r) => r.kin.maxSpeed * scale));
    switch (o.kind) {
      case 'mission': this.driver = missionDriver(parseMission(o.source), o.robots, o.station, drive === 'unicycle' ? (r) => { const th = r.state.theta * DEG; return [r.state.x * scale + look * Math.cos(th), r.state.y * scale + look * Math.sin(th)] as Vec2; } : toM, o.log ?? (() => {}), this.say, o.env, (r, u, dt) => { if (drive === 'unicycle') { const th = r.state.theta * DEG; const { v, w } = unicycleFromVelocity(th, u, look); integrate(r, { v: v / scale, omega: w * RAD }, dt); r.syncPoseFromState(); r.state.status = Math.abs(r.state.v) > 1 ? 'moving' : 'idle'; } else { r.setPose2D(r.state.x + (u[0] * dt) / scale, r.state.y + (u[1] * dt) / scale, Math.hypot(u[0], u[1]) > 1e-6 ? (Math.atan2(u[1], u[0]) * 180) / Math.PI : r.state.theta); r.state.v = Math.hypot(u[0], u[1]) / scale; r.state.status = Math.hypot(u[0], u[1]) > 1e-4 ? 'moving' : 'idle'; } }); break;
      case 'consensus': this.driver = consensusDriver(parseConsensus(o.source), o.robots, toM, place, o.vmax ?? Math.min(0.5, kinVmax(o.robots)), this.say); break;
      case 'swarm': this.driver = swarmDriver(parseSwarm(o.source), o.robots, toM, place, this.say); break;
      case 'coverage': this.driver = coverageDriver(parseCoverage(o.source), o.robots, toM, place, o.vmax ?? Math.min(0.5, kinVmax(o.robots)), this.say); break;
      case 'safety': this.driver = safetyDriver(parseSafety(o.source), o.robots, toM, place, this.say); break;
      case 'gridmapf': this.driver = gridDriver(parseGridMapf(o.source), o.robots, place, o.vmax ?? Math.min(0.5, kinVmax(o.robots)), this.say); break;
      case 'warehouse': this.driver = warehouseDriver(parseWarehouse(o.source), o.robots, o.station, place, this.say, o.createRobots); break;
      default: throw new Error(`the ${o.kind} model is analysed, not executed on the fleet (runnable: ${RUNNABLE_KINDS.join(', ')})`);
    }
    this.say(`fleet run "${this.driver.name}" on ${this.driver.robots.map((r) => r.name).join(', ')}`);
  }
  get robots(): MobileRobot[] { return this.driver.robots; }
  tick(dt: number): void { if (this.done) return; this.beforeTick(dt); this.driver.tick(dt); this.time += dt; this.ticks++; if (this.driver.done() || this.time >= this.maxSeconds) { this.done = true; this.say(`finished after ${this.time.toFixed(1)} s: ${this.driver.status()}`); } }
  status(): string { return `${this.done ? '⏹' : '▶'} ${this.driver.name}: t=${this.time.toFixed(1)} s · ${this.driver.status()}`; }
  stop(): void { this.done = true; for (const r of this.driver.robots) { r.state.v = 0; r.state.status = 'idle'; } }
}

const useRobots = (all: MobileRobot[], names: string[], n: number, say: (m: string) => void): MobileRobot[] => {
  const byName = names.map((nm) => all.find((r) => r.name === nm)).filter(Boolean) as MobileRobot[]; let rs = byName.length ? byName : all.slice(0, n);
  if (!rs.length) throw new Error('the station has no mobile robots (Mobile & Fleet › Add mobile robot…, or load the scenario)');
  if (rs.length < n) say(`only ${rs.length} of ${n} robots are on the station — running with ${rs.length}`); if (rs.length > n) rs = rs.slice(0, n); return rs;
};

function consensusDriver(d: ConsensusDoc, all: MobileRobot[], toM: (r: MobileRobot) => Vec2, place: (rs: MobileRobot[], p: Vec2[], u?: Vec2[]) => void, vmax: number, say: (m: string) => void): Driver {
  const robots = useRobots(all, d.robots.map((r) => r.name), d.graph.n, say); const n = robots.length; let p = robots.map(toM); const spec = { ...d.graph, n: Math.min(d.graph.n, n) }; if (spec.type === 'edges') spec.edges = spec.edges.filter(([a, b]) => a < n && b < n);
  const staticA: Matrix | null = spec.type === 'disk' ? null : buildGraph(spec); const offsets = d.formation ? formationOffsets(d.formation.shape, n, d.formation.r) : null; const A0 = spec.type === 'disk' ? diskGraph(p, spec.radius) : null; const R = spec.type === 'disk' ? spec.radius : d.rendezvous?.radius ?? Infinity;
  const rng = new Rng(d.seed); const commRadius = d.comm?.radius ?? (spec.type === 'disk' ? spec.radius : Infinity); const dropP = d.comm?.drop ?? 0; let A: Matrix = staticA ?? diskGraph(p, R); let err = Infinity, links = edgeCount(A); const mode = d.formation ? 'formation' : d.rendezvous ? 'rendezvous (connectivity-preserving)' : 'rendezvous (consensus on positions)'; const gain = d.formation?.gain ?? d.rendezvous?.gain ?? 1; let elapsed = 0; const failed = new Set<number>();
  return {
    name: `${d.name} — ${mode}`, robots, positions: () => p,
    tick(dt) {
      elapsed += dt; for (const fl of d.failures) if (fl.at <= elapsed && fl.robot < n && !failed.has(fl.robot)) { failed.add(fl.robot); say(`robot ${robots[fl.robot].name} failed at ${elapsed.toFixed(1)} s`); }
      const alive = robots.map((_, i) => !failed.has(i)); A = staticA && commRadius === Infinity ? staticA.map((row, i) => row.map((v, j) => (alive[i] && alive[j] ? v : 0))) : diskGraph(p, commRadius, alive); if (dropP > 0) for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) if (A[i][j] && rng.random() < dropP) { A[i][j] = A[j][i] = 0; }
      let u: Vec2[]; if (offsets) u = formationVelocity(p, A, offsets, gain); else if (d.rendezvous && A0) u = rendezvousVelocity(p, A0, R, gain); else u = consensusStep(p, A, 1).map((q, i) => [(q[0] - p[i][0]) * gain, (q[1] - p[i][1]) * gain] as Vec2);
      u = u.map((v, i) => (alive[i] ? clampNorm(v, vmax) : [0, 0])); p = p.map((q, i) => [q[0] + dt * u[i][0], q[1] + dt * u[i][1]] as Vec2); place(robots, p, u);
      const live = p.filter((_, i) => alive[i]); err = offsets ? formationError(live, offsets.filter((_, i) => alive[i])) : disagreement(live); links = edgeCount(A);
    },
    status: () => `${links} links · ${offsets ? 'formation error' : 'disagreement'} ${err.toFixed(4)} m${failed.size ? ` · failed: ${[...failed].map((i) => robots[i].name).join(', ')}` : ''}`,
    done: () => err < 1e-3 && elapsed > 1,
  };
}

function swarmDriver(d: SwarmDoc, all: MobileRobot[], toM: (r: MobileRobot) => Vec2, place: (rs: MobileRobot[], p: Vec2[], u?: Vec2[]) => void, say: (m: string) => void): Driver {
  if (!['boids', 'vicsek', 'pso'].includes(d.model)) throw new Error(`swarm model "${d.model}" is analysed, not executed on the fleet (boids, vicsek and pso are)`);
  const robots = useRobots(all, d.robotNames, d.n, say); const n = robots.length; const rng = new Rng(d.seed); let p = robots.map(toM); const origin = mean2(p);
  if (d.model === 'boids') { let v: Vec2[] = p.map(() => [rng.normal(0, 0.1), rng.normal(0, 0.1)]); let acc = 0; let phi = polarization(v), dm = minPairDistance(p); return { name: `${d.name} — Reynolds flock`, robots, positions: () => p, tick(dt) { acc += dt; while (acc >= d.dt) { acc -= d.dt; const a = p.map((_, i) => boidsAcceleration(p, v, i, { rSep: d.rSep, rView: d.rView, wSep: d.wSep, wAli: d.wAli, wCoh: d.wCoh })); v = v.map((vi, i) => { const w: Vec2 = [vi[0] + d.dt * a[i][0], vi[1] + d.dt * a[i][1]]; const s = Math.hypot(w[0], w[1]) || 1e-12; const c = Math.min(Math.max(s, d.vmin), d.vmax); return [(w[0] * c) / s, (w[1] * c) / s] as Vec2; }); p = p.map((q, i) => [q[0] + d.dt * v[i][0], q[1] + d.dt * v[i][1]] as Vec2); } phi = polarization(v); dm = minPairDistance(p); place(robots, p, v); }, status: () => `polarization ${phi.toFixed(3)} · min distance ${dm.toFixed(3)} m`, done: () => false }; }
  if (d.model === 'vicsek') { let th = p.map(() => rng.uniform(-Math.PI, Math.PI)); const box = d.box; let acc = 0; let phi = 0; const torus = (a: number) => { let x = a % box; if (x > box / 2) x -= box; if (x < -box / 2) x += box; return x; }; const rel = () => p.map((q) => [q[0] - origin[0] + box / 2, q[1] - origin[1] + box / 2] as Vec2); return { name: `${d.name} — Vicsek (η = ${d.etas[0]})`, robots, positions: () => p, tick(dt) { acc += dt; while (acc >= 1) { acc -= 1; const q = rel(); const nth = th.map((_, i) => { let c = 0, s = 0; for (let j = 0; j < n; j++) { const dx = torus(q[i][0] - q[j][0]), dy = torus(q[i][1] - q[j][1]); if (dx * dx + dy * dy <= d.radius * d.radius) { c += Math.cos(th[j]); s += Math.sin(th[j]); } } return Math.atan2(s, c) + rng.uniform(-d.etas[0] / 2, d.etas[0] / 2); }); th = nth; p = q.map((z, i) => { const x = (((z[0] + d.speed * Math.cos(th[i])) % box) + box) % box, y = (((z[1] + d.speed * Math.sin(th[i])) % box) + box) % box; return [x + origin[0] - box / 2, y + origin[1] - box / 2] as Vec2; }); phi = polarization(th.map((a) => [Math.cos(a), Math.sin(a)] as Vec2)); } place(robots, p, th.map((a) => [d.speed * Math.cos(a), d.speed * Math.sin(a)] as Vec2)); }, status: () => `order φ = ${phi.toFixed(3)} on a ${box} m torus`, done: () => false }; }
  const field = gaussianSource(d.source, d.sigma, 1, d.distractors); const measure = (z: Vec2) => field(z) + (d.noise > 0 ? rng.normal(0, d.noise) : 0); let v: Vec2[] = p.map(() => [0, 0]); let pbest = p.map((q) => [q[0], q[1]] as Vec2); let pbestVal = p.map((q) => measure(q)); const neigh = ringNeighborhood(n, d.k); let acc = 0; let target = p.map((q) => [q[0], q[1]] as Vec2); let best = 0;
  return { name: `${d.name} — swarm source seeking`, robots, positions: () => p, tick(dt) { acc += dt; if (acc >= 1) { acc -= 1; if (d.forget > 0) pbestVal = pbestVal.map((pv) => pv - d.forget * Math.abs(pv)); v = psoVelocity(target, v, pbest, pbestVal, neigh, rng); const rep = repulsion(target, d.dMin); const step = v.map((vi, i) => clampNorm([vi[0] + rep[i][0], vi[1] + rep[i][1]] as Vec2, d.vmax)); target = target.map((q, i) => [q[0] + step[i][0], q[1] + step[i][1]] as Vec2); v = step; ({ pbest, pbestVal } = updateBest(target, target.map((q) => measure(q)), pbest, pbestVal)); let b = 0; for (let i = 1; i < n; i++) if (pbestVal[i] > pbestVal[b]) b = i; best = field(pbest[b]); } const u = p.map((q, i) => { const dx = target[i][0] - q[0], dy = target[i][1] - q[1]; const dd = Math.hypot(dx, dy); const sp = Math.min(dd / Math.max(1 - acc, 0.05), d.vmax); return dd > 1e-6 ? ([(dx / dd) * sp, (dy / dd) * sp] as Vec2) : ([0, 0] as Vec2); }); p = p.map((q, i) => [q[0] + dt * u[i][0], q[1] + dt * u[i][1]] as Vec2); place(robots, p, u); }, status: () => `best field value ${best.toFixed(3)} · centre distance to the source ${dist(mean2(p), d.source).toFixed(3)} m`, done: () => dist(mean2(p), d.source) < 0.1 && acc < 0.2 };
}

function coverageDriver(d: CoverageDoc, all: MobileRobot[], toM: (r: MobileRobot) => Vec2, place: (rs: MobileRobot[], p: Vec2[], u?: Vec2[]) => void, vmax: number, say: (m: string) => void): Driver {
  const robots = useRobots(all, d.robots.map((r) => r.name), d.n, say); const n = robots.length; const setup = coverageSetup({ ...d, n }); const { q, dA, phi } = setup; let p = robots.map(toM); let C = p; let acc = 1; let H = d.range ? limitedCost(p, q, phi, dA, d.range) : coverageCost(p, q, phi, dA); let moving = true;
  return { name: `${d.name} — Lloyd coverage`, robots, positions: () => p, tick(dt) { acc += dt; if (acc >= 1) { acc = 0; const lab = voronoiLabels(p, q).map((i, k) => (d.range && (q[k][0] - p[i][0]) ** 2 + (q[k][1] - p[i][1]) ** 2 > d.range * d.range ? -1 : i)); C = massCentroids(p, q, phi, dA, lab).C; H = d.range ? limitedCost(p, q, phi, dA, d.range) : coverageCost(p, q, phi, dA); } const u = p.map((z, i) => clampNorm([d.gain * (C[i][0] - z[0]), d.gain * (C[i][1] - z[1])], vmax)); moving = u.some((v) => Math.hypot(v[0], v[1]) > 1e-3); p = p.map((z, i) => [z[0] + dt * u[i][0], z[1] + dt * u[i][1]] as Vec2); place(robots, p, u); }, status: () => `H = ${H.toFixed(4)} · ${moving ? 'moving to the centroids' : 'at the centroids'}`, done: () => !moving && acc > 0.5 };
}

function safetyDriver(d: SafetyDoc, all: MobileRobot[], toM: (r: MobileRobot) => Vec2, place: (rs: MobileRobot[], p: Vec2[], u?: Vec2[]) => void, say: (m: string) => void): Driver {
  const inst = safetyInstance(d); const robots = useRobots(all, d.scenario === 'custom' ? d.robots.map((r) => r.name) : [], inst.p.length, say); const n = robots.length; let p = robots.map(toM); let goals: Vec2[];
  if (d.scenario === 'custom') goals = inst.goals.slice(0, n); else { const c = mean2(p); goals = p.map((z) => [2 * c[0] - z[0], 2 * c[1] - z[1]] as Vec2); if (d.scenario === 'crossing' && n >= 2) goals = [p[1], p[0], ...p.slice(2)]; }
  let uPrev: Vec2[] = p.map(() => [1, 1]); let hold = p.map(() => 0); let dmin = Infinity; const poly = speedPolygon(d.vmax); const hist: Vec2[][] = []; let arrived = false;
  return { name: `${d.name} — barrier-function filter${d.unstuck ? ' + keep-right rule' : ''}`, robots, positions: () => p, tick(dt) { let uNom: Vec2[]; if (d.unstuck) ({ uNom, hold } = unstuckNominal(p, goals, uPrev, hold, d.gain, d.vmax, { holdSteps: d.unstuck.hold, angle: d.unstuck.angle })); else uNom = goToGoal(p, goals, d.gain, d.vmax); const speeds = d.scenario === 'custom' ? d.robots.map((r) => r.speed ?? d.vmax) : []; const u = p.map((pi, i) => { if (d.uncooperative.includes(i)) return clampNorm(uNom[i], speeds[i] ?? d.vmax); const cons = [...poly]; for (let j = 0; j < n; j++) if (j !== i && dist(pi, p[j]) < d.sense) cons.push(pairConstraint(pi, p[j], d.dSafe, d.gamma)); return safeVelocity(uNom[i], cons); }); p = p.map((z, i) => [z[0] + dt * u[i][0], z[1] + dt * u[i][1]] as Vec2); uPrev = u; hist.push(p); if (hist.length > 100) hist.shift(); dmin = Math.min(dmin, minDist(p)); arrived = p.every((z, i) => dist(z, goals[i]) < 0.05); place(robots, p, u); void cbfController; }, status: () => `min distance ${dmin.toFixed(3)} m (d_safe ${d.dSafe}) · max goal error ${Math.max(...p.map((z, i) => dist(z, goals[i]))).toFixed(3)} m`, done: () => arrived };
}

function gridDriver(d: GridMapfDoc, all: MobileRobot[], place: (rs: MobileRobot[], p: Vec2[], u?: Vec2[]) => void, vmax: number, say: (m: string) => void): Driver {
  const inst = gridInstance(d); const robots = useRobots(all, inst.names, inst.starts.length, say); const n = robots.length; const cell = d.cellSize; const toXY = (c: [number, number]): Vec2 => [(c[1] + 0.5) * cell, -(c[0] + 0.5) * cell];
  const starts = inst.starts.slice(0, n), goals = inst.goals.slice(0, n); const plan = d.methods.includes('cbs') || !d.methods.includes('prioritized') ? cbs(inst.grid, starts, goals).paths : prioritizedPlanning(inst.grid, starts, goals); if (!plan) throw new Error('no conflict-free plan for these agents');
  const { actions, deps } = actionDependencies(plan as Path[]); const nxt = plan.map(() => 0); const doneSet = new Set<string>(); const rng = new Rng(d.execute?.seed ?? 1); const delayP = d.execute?.delay ?? 0; let p = starts.map(toXY); const target: Array<Vec2 | null> = plan.map(() => null); const cur: Array<Action | null> = plan.map(() => null); let doneActions = 0; const total = actions.reduce((s, a) => s + a.length, 0); let delayed = plan.map(() => 0);
  place(robots, p);
  return { name: `${d.name} — ADG execution of the ${d.methods.includes('cbs') ? 'CBS' : 'prioritized'} plan`, robots, positions: () => p, tick(dt) { const u: Vec2[] = p.map(() => [0, 0]); for (let i = 0; i < n; i++) { if (!target[i]) { if (nxt[i] >= actions[i].length) continue; if (delayed[i] > 0) { delayed[i] -= dt; continue; } const dp = deps.get(`${i},${nxt[i]}`)!; let ok = true; for (const x of dp) if (!doneSet.has(x)) { ok = false; break; } if (!ok) continue; if (delayP > 0 && rng.random() < delayP) { delayed[i] = 0.5 + rng.random(); continue; } cur[i] = actions[i][nxt[i]]; target[i] = toXY(cur[i]!.to); } const t = target[i]!; const dd = dist(p[i], t); const step = vmax * dt; if (dd <= step) { p[i] = t; doneSet.add(`${i},${nxt[i]}`); nxt[i]++; doneActions++; target[i] = null; u[i] = [0, 0]; } else { u[i] = [((t[0] - p[i][0]) / dd) * vmax, ((t[1] - p[i][1]) / dd) * vmax]; p[i] = [p[i][0] + dt * u[i][0], p[i][1] + dt * u[i][1]]; } } place(robots, p, u); }, status: () => `${doneActions}/${total} moves done · ${plan.filter((_, i) => nxt[i] >= actions[i].length).length}/${n} at their goals`, done: () => nxt.every((k, i) => k >= actions[i].length) };
}

function warehouseDriver(d: WarehouseDoc, all: MobileRobot[], station: Station, place: (rs: MobileRobot[], p: Vec2[], u?: Vec2[]) => void, say: (m: string) => void, create?: (name: string, x: number, y: number) => MobileRobot): Driver {
  const sim = new FleetSim(d.cfg); const robots = sim.robots.map((fr) => { let r = all.find((m) => m.name === fr.name); if (!r) { if (!create) throw new Error(`robot ${fr.name} is not on the station (Group › Build warehouse scene)`); r = create(fr.name, fr.x * 1000, fr.y * 1000); say(`added robot ${fr.name} at its home ${fr.home}`); } return r; }); void station;
  let prev = sim.robots.map((r) => [r.x, r.y] as Vec2); place(robots, prev); let logged = 0; let acc = 0;
  return { name: `${d.name} — warehouse fleet`, robots, positions: () => sim.robots.map((r) => [r.x, r.y] as Vec2), tick(dt) { acc += dt; while (acc >= d.cfg.dt - 1e-9) { acc -= d.cfg.dt; sim.step(d.cfg.dt); } const p = sim.robots.map((r) => [r.x, r.y] as Vec2); const u = p.map((q, i) => [(q[0] - prev[i][0]) / Math.max(dt, 1e-6), (q[1] - prev[i][1]) / Math.max(dt, 1e-6)] as Vec2); prev = p; place(robots, p, u); for (; logged < sim.log.length; logged++) say(sim.log[logged]); }, status: () => { const m = sim.metrics(); return `${m.delivered} delivered / ${m.created} created · open ${m.open} · latency ${m.meanLatency.toFixed(0)} s · double commits ${m.doubleCommits} · path conflicts ${m.pathConflicts} · ${sim.robots.map((r) => `${r.name} ${r.exec.state}${r.exec.order ? ' ' + r.exec.order : ''}`).join(', ')}`; }, done: () => sim.time >= d.cfg.duration };
}

/**
 * Group mission over the configured robots of the station: closed loop (the measured robot positions feed the
 * mission model every tick, the velocity references go to the robots through their own controller), station
 * zones resolve `goto zone=` / `allocate zones=` and `nogo` zones become obstacles, an optional `des` supervisor of
 * the station gates the phase transitions (controllable events `<phase>_start`, observed `<phase>_done`) and an
 * optional `hybrid` mode automaton limits the speed (its `vMax`).
 */
function missionDriver(d: MissionSpec, all: MobileRobot[], station: Station, toM: (r: MobileRobot) => Vec2, appLog: (m: string) => void, say: (m: string) => void, env: (() => Record<string, number | boolean>) | undefined, apply: (r: MobileRobot, u: Vec2, dt: number) => void): Driver {
  void appLog;
  const robots = useRobots(all, d.robots.map((r) => r.name), d.robots.length, say); const n = robots.length;
  const zones = station.itemsOfType<ZoneItem>(ItemType.ZONE); const zoneByName = (name: string) => zones.find((z) => z.name.toLowerCase() === name.toLowerCase()); const centroidM = (z: ZoneItem): Vec2 => { const c = z.centroid(); return [c[0] / 1000, c[1] / 1000]; };
  const spec: MissionSpec = JSON.parse(JSON.stringify({ ...d, robots: robots.map((r, i) => ({ name: r.name, at: toM(r), home: d.robots[i]?.home ?? (r.home ? [r.home.x / 1000, r.home.y / 1000] : toM(r)), speed: d.robots[i]?.speed ?? Math.min(d.vmax, r.kin.maxSpeed / 1000) })), failures: d.failures.filter((f) => f.robot < n) }));
  for (const ph of spec.phases) { if (ph.zone) { const z = zoneByName(ph.zone); if (!z) throw new Error(`zone ${ph.zone} not found on the station`); ph.at = centroidM(z); } if (ph.kind === 'allocate' && ph.targetNames.some((nm) => zoneByName(nm))) { ph.targets = ph.targetNames.map((nm) => { const z = zoneByName(nm); if (!z) throw new Error(`zone ${nm} not found on the station`); return centroidM(z); }); } }
  for (const z of zones) if (z.kind === 'nogo' && z.polygon.length) { const c = centroidM(z); const r = Math.max(...z.polygon.map((q) => Math.hypot(q[0] / 1000 - c[0], q[1] / 1000 - c[1]))); spec.obstacles.push({ at: c, r, name: z.name }); }
  if (spec.obstacles.length > d.obstacles.length) say(`no-go zones as obstacles: ${spec.obstacles.slice(d.obstacles.length).map((o) => o.name).join(', ')}`);
  const arch = spec.architecture === 'compare' ? 'hybrid' : spec.architecture; if (spec.architecture === 'compare') say('architecture "compare" runs hybrid on the fleet (set architecture centralized | decentralized | hybrid to choose)');
  const sim = new MissionSim(spec, arch); spec.duration = Math.max(spec.duration, 3600);
  // supervisor of the station (des model) gating the phases; mode automaton (hybrid model) limiting the speed
  let sup: SupervisorRuntime | null = null, supName = ''; const supSigma = new Set<string>(); let modes: ModeMachine | null = null; let denied = 0, mismatches = 0;
  const models = controlModels(station);
  if (d.supervisor) { const m = models.find((x) => x.kind === 'des' && x.name === d.supervisor); if (!m) throw new Error(`supervisor model "${d.supervisor}" not found`); const doc = parseDes(m.source); const g = parallel(...doc.plant.map((a) => DES.fromSpec(a))); const r = supcon(g, doc.specs.map((a) => DES.fromSpec(a)), doc.uncontrollable); if (!r.realizable) throw new Error(`supervisor "${d.supervisor}" is unrealisable`); sup = new SupervisorRuntime(supervisorTable(r.supervisor, doc.uncontrollable, doc.unobservable, doc.plant.length)); supName = m.name; for (const e of g.sigma) supSigma.add(e); for (const a of doc.specs) for (const e of a.events) supSigma.add(e); say(`supervisor "${supName}" gates the phases (events <phase>_start / <phase>_done)`); }
  if (d.modes) { const m = models.find((x) => x.kind === 'hybrid' && x.name === d.modes); if (!m) throw new Error(`mode model "${d.modes}" not found`); modes = new ModeMachine(parseHybrid(m.source)); say(`mode automaton "${m.name}" limits the speed`); }
  let gatedPhase = -1; let gateDenied = false; let elapsed = 0; let logged = 0; let vmaxMode: number | null = null; const reportedDone = new Set<number>();
  const currentPhase = () => (arch === 'centralized' ? sim.coordPhase : Math.min(...sim.robots.filter((_, i) => sim.alive[i]).map((r) => r.phase)));
  const observe = (ev: string) => { if (!sup) return; try { sup.observe(ev); } catch (e) { if (e instanceof SupervisorViolation) { mismatches++; say(`model mismatch: ${e.message}`); } else throw e; } };
  return {
    name: `${d.name} — ${arch} mission`, robots, positions: () => sim.p, model: sim,
    tick(dt) {
      elapsed += dt; const ph = currentPhase();
      if (sup && ph !== gatedPhase && ph < spec.phases.length) {
        const known = (ev: string) => supSigma.has(ev); // events outside the alphabet of the supervisor are not gated; an event in the alphabet that is never enabled is a permanent hold
        if (ph > 0 && !reportedDone.has(ph - 1)) { reportedDone.add(ph - 1); const prev = `${spec.phases[ph - 1].name.replace(/\W+/g, '_')}_done`; if (known(prev)) observe(prev); } // the previous phase ended: report it before asking for the next one
        const ev = `${spec.phases[ph].name.replace(/\W+/g, '_')}_start`;
        if (known(ev)) { if (!sup.allowed(ev)) { if (!gateDenied) { denied++; say(`supervisor denied ${ev} in plant state ${sup.plantState().join(',')} — the group holds`); } gateDenied = true; } else { observe(ev); gateDenied = false; gatedPhase = ph; } } else gatedPhase = ph;
      }
      if (modes) { const e = env?.() ?? {}; const m = modes.step(elapsed, { ...(modes.spec.vars ?? {}), ...e, phase: ph, time: elapsed }); vmaxMode = modes.current().vMax ?? null; void m; }
      const p = robots.map(toM); let u: Vec2[];
      if (gateDenied) u = p.map(() => [0, 0]); else u = sim.step(dt, p);
      if (vmaxMode !== null) u = u.map((w) => clampNorm(w, vmaxMode!));
      robots.forEach((r, i) => { if (sim.alive[i]) apply(r, u[i], dt); else { r.state.v = 0; r.state.status = 'idle'; } });
      { const after = currentPhase(); if (sup && after >= spec.phases.length && !reportedDone.has(after - 1)) { reportedDone.add(after - 1); const last = `${spec.phases[after - 1].name.replace(/\W+/g, '_')}_done`; if (supSigma.has(last)) observe(last); } } // the last phase ended in this very tick: report it before the run stops
      for (; logged < sim.log.length; logged++) say(sim.log[logged]);
    },
    status: () => `${sim.status()}${sup ? ` · supervisor ${supName}: plant ${sup.plantState().join(',')}, denied ${denied}, mismatches ${mismatches}` : ''}${modes ? ` · mode ${modes.mode}${vmaxMode !== null ? ` (vmax ${vmaxMode})` : ''}` : ''}`,
    done: () => sim.done && !gateDenied,
  };
}

/** Build the station scene of a warehouse model: a map with the shelves, one zone per station and the robots at their homes. */
export function buildWarehouseScene(station: Station, cfg: FleetConfig, kinds: Record<string, string>): { map: MapItem; zones: ZoneItem[]; robots: MobileRobot[] } {
  const wh = new Warehouse(cfg.warehouse); const cellMm = wh.cell * 1000; const res = Math.min(100, cellMm / 2);
  const map = new MapItem('Warehouse map'); const w = wh.cols * cellMm, h = wh.rows * cellMm; map.resize(Math.ceil(w / res) + 2, Math.ceil(h / res) + 2, res, -cellMm / 2 - res, -h + cellMm / 2 - res); station.addChild(map);
  wh.grid.forEach((row, r) => row.forEach((b, c) => { if (b) map.fillRect((c - 0.5) * cellMm, (-r - 0.5) * cellMm, (c + 0.5) * cellMm, (-r + 0.5) * cellMm); }));
  const zones: ZoneItem[] = []; for (const [name, c] of Object.entries(wh.stations)) { const z = station.addChild(new ZoneItem(name)); const [x, y] = wh.cellXY(c); const half = cellMm * 0.45; z.polygon = [[x * 1000 - half, y * 1000 - half], [x * 1000 + half, y * 1000 - half], [x * 1000 + half, y * 1000 + half], [x * 1000 - half, y * 1000 + half]]; z.kind = kinds[name] === 'home' ? 'parking' : kinds[name] === 'pickup' ? 'loading' : kinds[name] === 'drop' ? 'unloading' : 'work'; zones.push(z); }
  const robots = cfg.robots.names.map((name, i) => { const r = station.addChild(new MobileRobot(name)); const [x, y] = wh.stationXY(cfg.robots.homes[i % cfg.robots.homes.length]); r.setPose2D(x * 1000, y * 1000, 0); r.kin.maxSpeed = cfg.robots.speed * 1000; r.kin.footprint = [cellMm * 0.7, cellMm * 0.6, 500]; r.home = { x: x * 1000, y: y * 1000, theta: 0 }; return r; });
  return { map, zones, robots };
}
