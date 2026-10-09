/**
 * Group safety with control barrier functions (course §16.4.3, chapter 13, practicum ПР6): pair constraints
 * h = ‖pᵢ − pⱼ‖² − d², an exact planar QP filter (active-set enumeration), the decentralised go-to-goal + filter
 * controller, the "keep right" deadlock rule with hysteresis and constraints on stale neighbour data.
 */
import { Vec2, sub, norm, dist, clampNorm } from './rng';

export type HalfPlane = { a: Vec2; b: number }; // aᵀu ≥ b

/** (a, b) of aᵀuᵢ ≥ b for robot i w.r.t. j: a = 2(pᵢ − pⱼ), b = −share·γ·h (Wang, Li, Egerstedt 2017). */
export function pairConstraint(pi: Vec2, pj: Vec2, dSafe: number, gamma: number, share = 0.5): HalfPlane { const d = sub(pi, pj); const h = d[0] * d[0] + d[1] * d[1] - dSafe * dSafe; return { a: [2 * d[0], 2 * d[1]], b: -share * gamma * h }; }
/** |u| ≤ vmax as an inscribed polygon: aₖᵀu ≥ −vmax cos(π/sides). */
export function speedPolygon(vmax: number, sides = 16): HalfPlane[] { const r = vmax * Math.cos(Math.PI / sides); return Array.from({ length: sides }, (_, k) => { const t = (2 * Math.PI * k) / sides; return { a: [-Math.cos(t), -Math.sin(t)] as Vec2, b: -r }; }); }
/** min ‖u − u_nom‖² s.t. aₖᵀu ≥ bₖ, solved exactly: candidates are u_nom, its projections on the lines and pairwise intersections; zero when nothing is feasible. */
export function safeVelocity(uNom: Vec2, cons: HalfPlane[], tol = 1e-9): Vec2 {
  const feasible = (u: Vec2) => cons.every((c) => c.a[0] * u[0] + c.a[1] * u[1] >= c.b - tol);
  if (feasible(uNom)) return uNom;
  let best: Vec2 | null = null, bd = Infinity; const consider = (u: Vec2) => { if (!feasible(u)) return; const d = (u[0] - uNom[0]) ** 2 + (u[1] - uNom[1]) ** 2; if (d < bd) { bd = d; best = u; } };
  for (const c of cons) { const n2 = c.a[0] * c.a[0] + c.a[1] * c.a[1]; if (n2 < 1e-18) continue; const k = (c.b - (c.a[0] * uNom[0] + c.a[1] * uNom[1])) / n2; consider([uNom[0] + k * c.a[0], uNom[1] + k * c.a[1]]); }
  for (let i = 0; i < cons.length; i++) for (let j = i + 1; j < cons.length; j++) { const A = cons[i].a, B = cons[j].a; const det = A[0] * B[1] - A[1] * B[0]; if (Math.abs(det) < 1e-12) continue; consider([(cons[i].b * B[1] - cons[j].b * A[1]) / det, (A[0] * cons[j].b - B[0] * cons[i].b) / det]); }
  return best ?? [0, 0];
}
/** Nominal controller u = k(goal − p) limited to vmax. */
export function goToGoal(p: Vec2[], goals: Vec2[], gain: number, vmax: number): Vec2[] { return p.map((pi, i) => clampNorm([gain * (goals[i][0] - pi[0]), gain * (goals[i][1] - pi[1])], vmax)); }
/** Decentralised safe controller: nominal velocity filtered by the constraints of neighbours within senseRadius (share ½) and the speed polygon. */
export function cbfController(p: Vec2[], goals: Vec2[], dSafe: number, senseRadius: number, o: { gamma?: number; gain?: number; vmax?: number; uNom?: Vec2[] } = {}): { uSafe: Vec2[]; uNom: Vec2[] } {
  const gamma = o.gamma ?? 1, gain = o.gain ?? 1, vmax = o.vmax ?? 0.2; const uNom = o.uNom ?? goToGoal(p, goals, gain, vmax); const poly = speedPolygon(vmax);
  const uSafe = p.map((pi, i) => { const cons = [...poly]; for (let j = 0; j < p.length; j++) if (j !== i && dist(pi, p[j]) < senseRadius) cons.push(pairConstraint(pi, p[j], dSafe, gamma)); return safeVelocity(uNom[i], cons); });
  return { uSafe, uNom };
}
export const rotate = (v: Vec2, angle: number): Vec2 => { const c = Math.cos(angle), s = Math.sin(angle); return [c * v[0] - s * v[1], s * v[0] + c * v[1]]; };
/** "Keep right" rule with hysteresis: a robot far from its goal whose progress (projection of u_prev on the goal direction) is below stuckSpeed rotates its nominal velocity by `angle` for holdSteps steps. */
export function unstuckNominal(p: Vec2[], goals: Vec2[], uPrev: Vec2[], hold: number[], gain: number, vmax: number, o: { stuckSpeed?: number; goalTol?: number; angle?: number; holdSteps?: number } = {}): { uNom: Vec2[]; hold: number[] } {
  const stuckSpeed = o.stuckSpeed ?? 0.02, goalTol = o.goalTol ?? 0.05, angle = o.angle ?? -Math.PI / 4, holdSteps = o.holdSteps ?? 50;
  const uNom = goToGoal(p, goals, gain, vmax); const h = hold.slice();
  for (let i = 0; i < p.length; i++) {
    const g = sub(goals[i], p[i]); const d = norm(g);
    if (d > goalTol) { const prog = (uPrev[i][0] * g[0] + uPrev[i][1] * g[1]) / d; if (prog < stuckSpeed) h[i] = holdSteps; }
    if (h[i] > 0) { uNom[i] = rotate(uNom[i], angle); h[i]--; }
  }
  return { uNom, hold: h };
}
/** Constraint w.r.t. a neighbour known only `age` s ago: d' = d + vmax_j·age, full responsibility (share 1) and the worst-case approach term 2‖pᵢ − pⱼ‖vmax_j. */
export function staleConstraint(pi: Vec2, pjLast: Vec2, age: number, vmaxJ: number, dSafe: number, gamma: number): HalfPlane { const d = sub(pi, pjLast); const dd = dSafe + vmaxJ * age; const h = d[0] * d[0] + d[1] * d[1] - dd * dd; return { a: [2 * d[0], 2 * d[1]], b: -gamma * h + 2 * norm(d) * vmaxJ }; }
/** Robots on a circle with diametrically opposite goals. */
export function antipodal(n: number, radius: number, center: Vec2 = [0, 0]): { p: Vec2[]; goals: Vec2[] } { const p: Vec2[] = [], goals: Vec2[] = []; for (let i = 0; i < n; i++) { const t = (2 * Math.PI * i) / n; p.push([center[0] + radius * Math.cos(t), center[1] + radius * Math.sin(t)]); goals.push([center[0] - radius * Math.cos(t), center[1] - radius * Math.sin(t)]); } return { p, goals }; }
export function minPairDistance(p: Vec2[]): number { let m = Infinity; for (let i = 0; i < p.length; i++) for (let j = i + 1; j < p.length; j++) m = Math.min(m, dist(p[i], p[j])); return m; }

export interface GroupRunOptions { dSafe?: number; unstuck?: boolean; gamma?: number; vmax?: number; dt?: number; senseRadius?: number; gain?: number; holdSteps?: number; /** neighbours whose state arrives with this delay (steps) use stale constraints */ staleLag?: number; /** indices of robots that ignore the filter (uncooperative) */ uncooperative?: number[]; /** per-robot speed limits (default vmax) */ speeds?: number[] }
/** Run the group to its goals; returns final positions, the minimum pair distance, the goal error and the trajectory (subsampled). */
export function runGroup(p0: Vec2[], goals: Vec2[], steps: number, o: GroupRunOptions = {}): { p: Vec2[]; dmin: number; maxGoalError: number; traj: Vec2[][]; deadlock: boolean; arrivedAt: number | null } {
  const dSafe = o.dSafe ?? 0.2, gamma = o.gamma ?? 2, vmax = o.vmax ?? 0.3, dt = o.dt ?? 0.02, sense = o.senseRadius ?? 1, gain = o.gain ?? 1;
  let p = p0.map((q) => [q[0], q[1]] as Vec2); let uPrev: Vec2[] = p.map(() => [1, 1]); let hold = p.map(() => 0); let dmin = Infinity; const traj: Vec2[][] = [p.map((q) => [q[0], q[1]] as Vec2)]; const every = Math.max(1, Math.floor(steps / 200)); const poly = speedPolygon(vmax); let arrivedAt: number | null = null;
  const hist: Vec2[][] = [p.map((q) => [q[0], q[1]] as Vec2)];
  for (let k = 0; k < steps; k++) {
    let uNom: Vec2[]; if (o.unstuck) ({ uNom, hold } = unstuckNominal(p, goals, uPrev, hold, gain, vmax, { holdSteps: o.holdSteps })); else uNom = goToGoal(p, goals, gain, vmax);
    const u = p.map((pi, i) => {
      if (o.uncooperative?.includes(i)) return clampNorm(uNom[i], o.speeds?.[i] ?? vmax);
      const cons = [...poly];
      for (let j = 0; j < p.length; j++) { if (j === i) continue; if (o.staleLag && o.staleLag > 0) { const lag = Math.min(o.staleLag, hist.length - 1); const known = hist[hist.length - 1 - lag][j]; if (dist(pi, known) < sense + vmax * lag * dt) cons.push(staleConstraint(pi, known, lag * dt, vmax, dSafe, gamma)); } else if (dist(pi, p[j]) < sense) cons.push(pairConstraint(pi, p[j], dSafe, gamma)); }
      return safeVelocity(uNom[i], cons);
    });
    p = p.map((q, i) => [q[0] + dt * u[i][0], q[1] + dt * u[i][1]] as Vec2); uPrev = u; hist.push(p.map((q) => [q[0], q[1]] as Vec2)); if (hist.length > 200) hist.shift();
    dmin = Math.min(dmin, minPairDistance(p)); if (k % every === 0) traj.push(p.map((q) => [q[0], q[1]] as Vec2));
    if (arrivedAt === null && p.every((q, i) => dist(q, goals[i]) < 0.05)) arrivedAt = (k + 1) * dt;
  }
  traj.push(p); const maxGoalError = Math.max(...p.map((q, i) => dist(q, goals[i])));
  return { p, dmin, maxGoalError, traj, deadlock: maxGoalError > 0.5, arrivedAt };
}
