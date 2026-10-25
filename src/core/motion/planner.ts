/**
 * RoboDK-like motion planning used by the station runtime, the program simulator and the trajectory helpers.
 *
 * - MotionGroup: an arm plus its synchronised external axes (a rail / gantry carrying the arm, positioners carrying the
 *   work). Joint vectors are [arm (dof), ext 1, ext 2 ...] like RoboDK targets with 6+N joints.
 * - Move geometry: MoveJ (joint interpolation; a Cartesian target is solved by IK on its pose, the stored joints only
 *   choose the configuration), MoveL and MoveC (Cartesian path relative to the reference frame, external axes
 *   interpolated linearly). IK is solved at adaptive knots only: a knot interval is accepted when the joint-interpolated
 *   midpoint is within 0.1 mm / 0.1 deg of the commanded path, so long dense programs stay cheap.
 * - Blending: a rounding radius r on a move replaces the corner by a quadratic Bezier zone (Cartesian between two
 *   Cartesian moves, joint space otherwise) that starts min(r, L/2) before the target and ends min(r, L/2) after it.
 * - Timing: one time-optimal velocity profile per blended chain (forward/backward acceleration passes); the path speed
 *   is limited by the programmed linear speed, the orientation speed (joint speed in deg/s), per-joint maximum
 *   velocities and, for joint moves, the programmed joint speed. Fine points stop the robot.
 */
import { Mat4, multiply, invert, getPos, slerpPose, rotationAngle, RAD, fromArray, Vec3, sub, add, scale, norm, cross, dot, normalize, identity } from '../math/pose';
import { Robot } from '../items/robot';
import { Item, Target } from '../items/item';
import type { MoveInstruction } from '../items/program';
import { carrierOf, solveIKWithCarrier } from '../kinematics/combined';
import { actuatedIndices } from '../kinematics/chain';
import { wristSingularity } from '../kinematics/ik_analytic';
import type { Trajectory, TrajectorySample } from './trajectory';

export interface MotionSettings { speedLinear: number; speedJoints: number; accelLinear: number; accelJoints: number; rounding: number }

// ---- 1D time-optimal profile ------------------------------------------------------------------------------------------

/** Time-optimal profile over distance d from speed v0 to v1 (max speed vmax, acceleration a). */
export interface Profile { d: number; v0: number; v1: number; vp: number; a: number; t1: number; t2: number; T: number; lin?: boolean }

export function makeProfile(d: number, v0: number, v1: number, vmax: number, a: number): Profile {
  v0 = Math.max(0, Math.min(vmax, v0)); v1 = Math.max(0, Math.min(vmax, v1));
  if (!(d > 1e-12)) return { d: 0, v0, v1, vp: 0, a, t1: 0, t2: 0, T: 0 };
  if (!(a > 0) || !Number.isFinite(a)) return { d, v0: vmax, v1: vmax, vp: vmax, a: Infinity, t1: 0, t2: d / vmax, T: d / vmax };
  const vp = Math.max(v0, v1, Math.min(vmax, Math.sqrt(Math.max(0, a * d + (v0 * v0 + v1 * v1) / 2))));
  const d1 = (vp * vp - v0 * v0) / (2 * a), d3 = (vp * vp - v1 * v1) / (2 * a);
  if (d1 + d3 > d * (1 + 1e-9) + 1e-12) {
    // infeasible after rounding: constant acceleration between the end speeds
    const T = (2 * d) / Math.max(1e-12, v0 + v1);
    return { d, v0, v1, vp, a: (v1 - v0) / T, t1: T, t2: T, T, lin: true };
  }
  const t1 = (vp - v0) / a, tc = Math.max(0, d - d1 - d3) / vp, t3 = (vp - v1) / a;
  return { d, v0, v1, vp, a, t1, t2: t1 + tc, T: t1 + tc + t3 };
}

/** Distance travelled at time t (0..T). */
export function profilePos(p: Profile, t: number): number {
  if (p.T <= 0) return p.d;
  if (t <= 0) return 0;
  if (t >= p.T) return p.d;
  if (p.lin) return p.v0 * t + 0.5 * p.a * t * t;
  if (!Number.isFinite(p.a)) return (p.d * t) / p.T;
  if (t < p.t1) return p.v0 * t + 0.5 * p.a * t * t;
  const d1 = p.v0 * p.t1 + 0.5 * p.a * p.t1 * p.t1;
  if (t < p.t2) return d1 + p.vp * (t - p.t1);
  const tr = p.T - t;
  return p.d - (p.v1 * tr + 0.5 * p.a * tr * tr);
}

// ---- trajectories ---------------------------------------------------------------------------------------------------

/** Trajectory with knots and an exact evaluator (velocity profile per knot interval). */
export class PlannedTrajectory implements Trajectory {
  samples: TrajectorySample[];
  duration: number;
  length: number;
  ok: boolean;
  error?: string;
  constructor(samples: TrajectorySample[], private profiles: Profile[], length: number, ok = true, error?: string) {
    this.samples = samples;
    this.duration = samples.length ? samples[samples.length - 1].t : 0;
    this.length = length;
    this.ok = ok;
    if (error) this.error = error;
  }
  /** Exact sample at time t (s from the trajectory start). */
  eval(t: number): TrajectorySample {
    const s = this.samples;
    if (t <= 0 || s.length === 1) return s[0];
    if (t >= this.duration) return s[s.length - 1];
    let lo = 0, hi = s.length - 1;
    while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (s[mid].t <= t) lo = mid; else hi = mid; }
    const a = s[lo], b = s[hi], p = this.profiles[lo];
    const f = p && p.d > 0 ? profilePos(p, t - a.t) / p.d : b.t > a.t ? (t - a.t) / (b.t - a.t) : 1;
    return { t, joints: a.joints.map((v, i) => v + ((b.joints[i] ?? v) - v) * f), pose: slerpPose(a.pose, b.pose, f) };
  }
}

/** Dense copy of a trajectory sampled every dt seconds (for callers that iterate samples). */
export function densify(tr: PlannedTrajectory, dt: number): PlannedTrajectory {
  if (!(dt > 0) || tr.duration <= 0) return tr;
  const n = Math.max(1, Math.ceil(tr.duration / dt));
  const samples: TrajectorySample[] = [];
  for (let k = 0; k <= n; k++) samples.push(tr.eval((k / n) * tr.duration));
  const out = new PlannedTrajectory(samples, [], tr.length, tr.ok, tr.error);
  out.duration = tr.duration;
  return out;
}

// ---- motion group (arm + synchronised external axes) ----------------------------------------------------------------

export class MotionGroup {
  readonly ext: Robot[];
  readonly dof: number;
  readonly n: number;
  readonly offsets: number[] = [];
  readonly lower: number[];
  readonly upper: number[];
  readonly vmax: number[];
  readonly amax: number[];
  readonly prismatic: boolean[];
  readonly baseNow: Mat4;
  readonly baseNowInv: Mat4;
  private carrierIdx = -1;
  private mount: Mat4 = identity();
  private extAbs: Mat4[];
  private refs = new Map<Item, { k: number; rel: Mat4 } | null>();
  private absCache = new Map<Item, Mat4>();

  constructor(readonly robot: Robot, ext: Robot[] = []) {
    this.ext = ext;
    this.dof = robot.dof;
    let n = this.dof;
    for (const m of ext) { this.offsets.push(n); n += m.dof; }
    this.n = n;
    const lims = [robot, ...ext].map((r) => r.jointLimits());
    this.lower = lims.flatMap((l) => l.lower);
    this.upper = lims.flatMap((l) => l.upper);
    const joints = [robot, ...ext].flatMap((r) => actuatedIndices(r.chain).map((i) => r.chain.joints[i]));
    this.vmax = joints.map((j) => (j.maxVelocity && j.maxVelocity > 0 ? j.maxVelocity : Infinity));
    this.amax = joints.map((j) => (j.maxAcceleration && j.maxAcceleration > 0 ? j.maxAcceleration : Infinity));
    this.prismatic = joints.map((j) => j.type === 'prismatic');
    this.baseNow = robot.poseAbs();
    this.baseNowInv = invert(this.baseNow);
    this.extAbs = ext.map((m) => m.poseAbs());
    const c = carrierOf(robot);
    const k = c ? ext.indexOf(c) : -1;
    if (k >= 0) { this.carrierIdx = k; this.mount = multiply(invert(multiply(this.extAbs[k], c!.solveFKFlange())), this.baseNow); }
  }

  get hasCarrier(): boolean { return this.carrierIdx >= 0; }
  get carrier(): Robot | null { return this.carrierIdx >= 0 ? this.ext[this.carrierIdx] : null; }

  current(): number[] { return [...this.robot.joints(), ...this.ext.flatMap((m) => m.joints())]; }
  apply(q: number[]): void {
    this.robot.setJoints(q.slice(0, this.dof));
    this.ext.forEach((m, k) => { if (q.length >= this.offsets[k] + m.dof) m.setJoints(q.slice(this.offsets[k], this.offsets[k] + m.dof)); });
  }
  /** Pad a joint vector (arm joints, maybe with external axes) to the full group vector using `like` for missing values. */
  full(q: number[], like: number[]): number[] {
    const out = like.slice(0, this.n);
    for (let i = 0; i < Math.min(q.length, this.n); i++) out[i] = q[i];
    return out;
  }
  private extJoints(e: number[], k: number): number[] { return e.slice(this.offsets[k] - this.dof, this.offsets[k] - this.dof + this.ext[k].dof); }
  /** Absolute pose of the arm base for external-axis values e. */
  baseAbs(e: number[]): Mat4 {
    if (this.carrierIdx < 0) return this.baseNow;
    const c = this.ext[this.carrierIdx];
    return multiply(multiply(this.extAbs[this.carrierIdx], c.solveFKFlange(this.extJoints(e, this.carrierIdx))), this.mount);
  }
  /** Absolute pose of an item (target / frame) for external-axis values e (items on a positioner move with it). */
  refAbs(item: Item | null, e: number[]): Mat4 {
    if (!item) return this.baseAbs(e);
    let info = this.refs.get(item);
    if (info === undefined) {
      info = null;
      for (let p = item.parent; p && !info; p = p.parent) {
        const k = this.ext.indexOf(p as Robot);
        if (k >= 0 && k !== this.carrierIdx) info = { k, rel: multiply(invert(multiply(this.extAbs[k], this.ext[k].solveFKFlange())), item.poseAbs()) };
      }
      this.refs.set(item, info);
      // a robot used as reference frame means its flange (RoboDK "Set Ref.: <robot>", a part held by another robot)
      if (!info) this.absCache.set(item, item instanceof Robot && item !== this.robot ? multiply(item.poseAbs(), item.solveFKFlange()) : item.poseAbs());
    }
    if (!info) return this.absCache.get(item)!;
    return multiply(multiply(this.extAbs[info.k], this.ext[info.k].solveFKFlange(this.extJoints(e, info.k))), info.rel);
  }
  /** Joint speed limit used for joint interpolation (deg/s or mm/s). */
  jointSpeed(i: number, s: MotionSettings): number {
    return this.prismatic[i] ? Math.min(this.vmax[i], s.speedLinear) : Math.min(this.vmax[i], s.speedJoints);
  }
}

/** Arm + external axes: RoboDK externalAxes order, the carrier (rail/gantry holding the arm), positioners inferred from targets with extra joints. */
export function motionGroupFor(robot: Robot, refs: Iterable<Item | null> = []): MotionGroup {
  const st = robot.station;
  const ext: Robot[] = [];
  for (const id of robot.externalAxes ?? []) { const m = st?.findById(id); if (m instanceof Robot && m !== robot && !ext.includes(m)) ext.push(m); }
  // a rail / gantry / turntable carrying an arm moves with it; a tool mechanism (gun, gripper) carried by an arm does not
  // make the arm its external axis
  const c = carrierOf(robot);
  if (c && !ext.includes(c) && robot.dof >= 5 && c.dof <= 3) ext.unshift(c);
  let n = robot.dof + ext.reduce((a, m) => a + m.dof, 0);
  for (const r of refs) {
    const q = r instanceof Target ? r.joints : null;
    if (!r || !q || q.length <= n) continue;
    for (let p = r.parent; p; p = p.parent) {
      if (p instanceof Robot && p !== robot && !p.isAncestorOf(robot)) { if (!ext.includes(p)) { ext.push(p); n += p.dof; } break; }
    }
  }
  return new MotionGroup(robot, ext);
}

// ---- move geometry --------------------------------------------------------------------------------------------------

interface Knot { s: number; q: number[]; pose: Mat4; x: Vec3 }
interface CartPoint { P: Mat4; e: number[]; x: Vec3; disp: Mat4 }
interface Geom {
  cart: boolean;
  knots: Knot[];
  ok: boolean;
  error?: string;
  warnings: string[];
  at?: (s: number) => CartPoint;
  /** Joint-space evaluation (joint moves). */
  qAt?: (s: number) => number[];
  tool: Mat4;
  settings: MotionSettings;
  /** Commanded TCP path length (mm, speed frame). */
  pathLen: number;
}

export interface ChainMoveSpec {
  data: MoveInstruction;
  target: Target | null;
  via: Target | null;
  /** TCP relative to the flange. */
  tool: Mat4;
  /** Active reference frame (inline poses are relative to it; null = robot base). */
  frame: Item | null;
  /**
   * Reference frame set explicitly (Set Ref. instruction, program frame or the robot's active frame): Cartesian targets
   * are taken relative to it like RoboDK (target pose in its own frame, applied to the active frame), so one program
   * serves several parts by changing the reference. Null: targets at their absolute pose.
   */
  targetFrame?: Item | null;
  settings: MotionSettings;
}

const TOL_P = 0.1, TOL_R = 0.1 / RAD, MAX_STEP = 15;
const maxDelta = (a: number[], b: number[], n = a.length) => { let m = 0; for (let i = 0; i < n; i++) m = Math.max(m, Math.abs(a[i] - b[i])); return m; };
const lerpQ = (a: number[], b: number[], f: number) => a.map((v, i) => v + ((b[i] ?? v) - v) * f);
const posOf = (m: Mat4): Vec3 => [m[12], m[13], m[14]];

class GeomBuilder {
  constructor(readonly g: MotionGroup) {}
  get robot() { return this.g.robot; }
  private analytic?: boolean;
  /** IK seeded by the previous knot. Closed-form robots: `precise` = false keeps the numeric residual estimate of a failure short (bisection probes). */
  ik(P: Mat4, seed: number[], tool: Mat4, precise = false): { ok: boolean; q: number[]; posError: number; rotError: number } {
    const dof = this.g.dof;
    this.analytic ??= this.robot.hasAnalyticIK();
    let r = this.robot.solveIK(P, { seed: seed.slice(0, dof), keepFirstSolution: true, restarts: 0, maxIterations: this.analytic && !precise ? 1 : 60 }, tool);
    // numeric chains, and closed-form ones on the final (precise) attempt, get restarts: next to a singularity the
    // closed form can reject a reachable point by rounding and the damped solver still lands on it
    if (!r.ok && (!this.analytic || precise)) r = this.robot.solveIK(P, { seed: seed.slice(0, dof), keepFirstSolution: true, restarts: 2, maxIterations: 150 }, tool);
    // near a wrist singularity the numeric solver can stall a few microns short of its 1e-3 mm tolerance:
    // such a point is on the path within half the path tolerance, accept it
    const ok = r.ok || (r.posError <= TOL_P / 2 && r.rotError <= TOL_R / 2);
    return { ok, q: r.joints, posError: r.posError, rotError: r.rotError };
  }
  /** Message for an IK failure at s: a near-miss right at the reachable boundary is a singularity (RoboDK wording). */
  failMsg(label: string, s: number, posError: number, rotError: number): string {
    const pct = (s * 100).toFixed(1);
    return posError < 1 && rotError < 0.01
      ? `${label} crosses a singularity at ${pct}% (residual ${posError.toFixed(2)} mm) — start from a non-singular configuration or use MoveJ`
      : `${label} unreachable at ${pct}% (pos err ${posError.toFixed(2)} mm)`;
  }
  fkDisp(q: number[], tool: Mat4): Mat4 {
    const g = this.g;
    const tcp = this.robot.solveFK(q.slice(0, g.dof), tool);
    return g.hasCarrier ? multiply(multiply(g.baseNowInv, g.baseAbs(q.slice(g.dof))), tcp) : tcp;
  }

  /** Joint-space path (linear or any qAt) with FK knots for the TCP polyline. */
  jointGeom(qAt: (s: number) => number[], tool: Mat4, settings: MotionSettings, mids: number[] = []): Geom {
    const mk = (s: number): Knot => { const q = qAt(s); const pose = this.fkDisp(q, tool); return { s, q, pose, x: posOf(pose) }; };
    const knots: Knot[] = [];
    const sub = (a: Knot, b: Knot, depth: number) => {
      const m = mk((a.s + b.s) / 2);
      const chord = add(a.x, scale(sub3(b.x, a.x), 0.5));
      if (depth < 6 && (norm(sub3(m.x, chord)) > 1 || maxDelta(a.q, b.q) > 30)) { sub(a, m, depth + 1); sub(m, b, depth + 1); } else knots.push(b);
    };
    const fixed = [0, ...mids, 1].map(mk);
    knots.push(fixed[0]);
    for (let i = 1; i < fixed.length; i++) sub(fixed[i - 1], fixed[i], 0);
    let len = 0;
    for (let i = 1; i < knots.length; i++) len += norm(sub3(knots[i].x, knots[i - 1].x));
    return { cart: false, knots, ok: true, warnings: [], qAt, tool, settings, pathLen: len };
  }

  /** Cartesian path: march with continuous IK, then refine adaptively. */
  cartGeom(at: (s: number) => CartPoint, q0: number[], tool: Mat4, settings: MotionSettings, label: string, approxLen: number, evenN = false, qEnd?: number[]): Geom {
    const g = this.g, dof = g.dof;
    const p0 = at(0), p1 = at(1);
    const ang = rotationAngle(p0.P, p1.P) * RAD;
    const de = p0.e.reduce((m, v, i) => Math.max(m, Math.abs(v - p1.e[i]) / (g.prismatic[dof + i] ? 40 : 8)), 0);
    let N = Math.max(1, Math.ceil(approxLen / 40), Math.ceil(ang / 8), Math.ceil(de));
    if (evenN) N = Math.max(2, N + (N % 2));
    const knots: Knot[] = [{ s: 0, q: g.full(q0, q0), pose: p0.disp, x: p0.x }];
    const geom: Geom = { cart: true, knots, ok: true, warnings: [], at, tool, settings, pathLen: 0 };
    // RoboDK rejects a linear / circular move that starts exactly at a wrist singularity (axes 4 and 6 aligned, e.g. the
    // KUKA home posture with J5 = 0): the wrist split of J4/J6 is undefined there (01.d InspectA / RetractInspectA)
    if (label !== 'Blend' && (approxLen > 1e-3 || ang > 1e-4)) {
      const ws = wristSingularity(this.robot.chain, q0.slice(0, dof));
      if (ws !== null && ws < 1e-5) { geom.ok = false; geom.error = `${label} starts at a wrist singularity (axes 4 and 6 aligned): use a joint move first`; return geom; }
    }
    // adaptive march: the IK of each knot is seeded by the previous one and the step shrinks until the joints move
    // less than MAX_STEP, so fast wrist rotations near a singularity stay on the same branch (RoboDK keeps the config)
    let prev = knots[0];
    const base = 1 / N;
    let step = base;
    while (prev.s < 1 - 1e-12) {
      let sn = Math.min(1, prev.s + step);
      let p = sn === 1 ? p1 : at(sn);
      // a known end configuration (blend zones end on a solved knot) saves the IK of the last knot
      let r = sn === 1 && qEnd ? { ok: true, q: qEnd.slice(0, dof), posError: 0, rotError: 0 } : this.ik(p.P, prev.q, tool);
      while ((!r.ok || maxDelta(r.q, prev.q, dof) > MAX_STEP) && sn - prev.s > 1e-5) {
        sn = (prev.s + sn) / 2;
        p = at(sn);
        r = this.ik(p.P, prev.q, tool);
      }
      if (!r.ok) r = this.ik(p.P, prev.q, tool, true);
      if (!r.ok) {
        geom.ok = false;
        // the path leaves the workspace towards an unreachable end pose: "unreachable" (a start in a singular posture
        // keeps the singularity wording)
        const endR = sn > 0.01 && r.posError < 1 ? this.ik(p1.P, prev.q, tool, true) : null;
        geom.error = endR && !endR.ok && endR.posError >= 1 ? `${label} unreachable at ${(sn * 100).toFixed(1)}% (target out of reach by ${endR.posError.toFixed(1)} mm)` : this.failMsg(label, sn, r.posError, r.rotError);
        break;
      }
      const dq = maxDelta(r.q, prev.q, dof);
      if (dq > MAX_STEP) {
        geom.ok = false;
        // the branch being followed ran into a joint limit (the solver switched configuration), or a singularity
        const lim = prev.q.slice(0, dof).findIndex((v, i) => v - g.lower[i] < 3 || g.upper[i] - v < 3);
        geom.error = lim >= 0 ? `${label} reaches the J${lim + 1} joint limit at ${(sn * 100).toFixed(0)}%` : `Joint jump of ${dq.toFixed(0)} deg (singularity) at ${(sn * 100).toFixed(0)}%`;
        break;
      }
      const kn: Knot = { s: sn, q: [...r.q, ...p.e], pose: p.disp, x: p.x };
      const err = this.refine(at, prev, kn, tool, label, knots);
      if (err) { geom.ok = false; geom.error = err; break; }
      step = Math.min(base, (sn - prev.s) * 2);
      prev = kn;
    }
    for (let i = 1; i < knots.length; i++) geom.pathLen += norm(sub3(knots[i].x, knots[i - 1].x));
    return geom;
  }

  /** Append the refined knots of interval (a, b] to `out`; returns an error message when the path cannot be followed. */
  private refine(at: (s: number) => CartPoint, a: Knot, b: Knot, tool: Mat4, label: string, out: Knot[]): string | null {
    const g = this.g, dof = g.dof;
    const rec = (a: Knot, b: Knot, depth: number): string | null => {
      const dq = maxDelta(a.q, b.q, dof);
      const sm = (a.s + b.s) / 2, m = at(sm);
      const ql = lerpQ(a.q, b.q, 0.5);
      const F = this.robot.solveFK(ql.slice(0, dof), tool);
      const ep = norm(sub3(posOf(F), posOf(m.P))), er = rotationAngle(F, m.P);
      if (ep <= TOL_P && er <= TOL_R && dq <= MAX_STEP) { out.push(b); return null; }
      const move = norm(sub3(b.x, a.x)) + rotationAngle(a.pose, b.pose) * 100;
      if ((dq > 5 && move < 0.05) || depth > 30) return `Joint jump of ${dq.toFixed(0)} deg (singularity) at ${(sm * 100).toFixed(0)}%`;
      let r = this.ik(m.P, ql, tool);
      let jump = r.ok ? maxDelta(r.q, ql, dof) : Infinity;
      if (jump > Math.max(20, 2 * dq)) {
        // near a wrist singularity DLS may fall onto the other wrist branch: retry from both ends, keep the continuous one
        for (const seed of [a.q, b.q]) {
          const rr = this.ik(m.P, seed, tool);
          const jj = rr.ok ? maxDelta(rr.q, ql, dof) : Infinity;
          if (jj < jump) { r = rr; jump = jj; }
        }
      }
      if (!r.ok) return this.failMsg(label, sm, r.posError, r.rotError);
      const km: Knot = { s: sm, q: [...r.q, ...m.e], pose: m.disp, x: m.x };
      if (jump > Math.max(20, 2 * dq)) return `Joint jump of ${jump.toFixed(0)} deg (singularity) at ${(sm * 100).toFixed(0)}%`;
      return rec(a, km, depth + 1) ?? rec(km, b, depth + 1);
    };
    return rec(a, b, 0);
  }
}

function sub3(a: Vec3, b: Vec3): Vec3 { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }

/** Circle through three points: returns a parametrisation p(f), f in [0,1] from A to C through B, or null if collinear. */
function circleThrough(A: Vec3, B: Vec3, C: Vec3): { at: (f: number) => Vec3; len: number } | null {
  const a = sub(A, C), b = sub(B, C);
  const axb = cross(a, b);
  const denom = 2 * dot(axb, axb);
  if (denom < 1e-6 * Math.max(1, dot(a, a) * dot(b, b))) return null;
  const term = scale(cross(sub(scale(b, dot(a, a)), scale(a, dot(b, b))), axb), 1 / denom);
  const center = add(C, term), radius = norm(term), nrm = normalize(axb);
  const u = normalize(sub(A, center)), v = cross(nrm, u);
  const angleOf = (P: Vec3) => { const r = sub(P, center); let th = Math.atan2(dot(r, v), dot(r, u)); if (th < 0) th += 2 * Math.PI; return th; };
  const thB = angleOf(B);
  let thC = angleOf(C);
  if (thB > thC) thC -= 2 * Math.PI;
  return { at: (f) => add(center, add(scale(u, radius * Math.cos(thC * f)), scale(v, radius * Math.sin(thC * f)))), len: Math.abs(thC) * radius };
}

function withPos(m: Mat4, p: Vec3): Mat4 { const o = new Float64Array(m); o[12] = p[0]; o[13] = p[1]; o[14] = p[2]; return o; }

/** Geometry of one move from joints q0. */
function moveGeometry(b: GeomBuilder, q0: number[], spec: ChainMoveSpec, extEnd?: number[]): Geom {
  const g = b.g, robot = g.robot, dof = g.dof, d = spec.data, target = spec.target, tool = spec.tool;
  const e0 = q0.slice(dof, g.n);
  const failed = (msg: string): Geom => ({ cart: false, knots: [{ s: 0, q: q0, pose: b.fkDisp(q0, tool), x: posOf(b.fkDisp(q0, tool)) }], ok: false, error: msg, warnings: [], tool, settings: spec.settings, pathLen: 0 });
  const extOf = (t: Target | null): number[] | null => extEnd ?? (t?.joints && t.joints.length >= g.n && g.n > dof ? t.joints.slice(dof, g.n) : null);
  // RoboDK: a Cartesian target is its pose relative to its parent frame, applied to the active reference frame
  const tf = spec.targetFrame ?? null;
  const relFrame = (t: Target | null): boolean => !!(tf && t && t.parent !== tf);
  const targetAbs = (t: Target, e: number[]): Mat4 => (relFrame(t) ? multiply(g.refAbs(tf, e), t.pose()) : g.refAbs(t, e));
  const inLimits = (q: number[], tol = 0.5) => q.every((v, i) => Number.isFinite(v) && v >= g.lower[i] - tol && v <= g.upper[i] + tol);
  const clampQ = (q: number[]) => q.map((v, i) => Math.min(g.upper[i], Math.max(g.lower[i], v)));
  if (d.moveType === 'MoveJ') {
    let q1: number[] | null = null;
    if (target) {
      const j = target.joints;
      // joint targets, and any target of a mechanism (gripper / gun / positioner: < 5 axes, its pose does not define
      // its joints), go to their stored joints
      if ((target.isJointTarget || dof < 5) && j && j.length >= dof) q1 = g.full(j, q0);
      else {
        const e1 = extOf(target) ?? e0;
        const P = multiply(invert(g.baseAbs(e1)), targetAbs(target, e1));
        const stored = j && j.length >= dof && j.slice(0, dof).every(Number.isFinite) ? j.slice(0, dof) : null;
        const r = robot.solveIK(P, { seed: stored ?? q0.slice(0, dof) }, tool);
        if (r.ok) q1 = [...r.joints, ...e1];
        else if (g.hasCarrier && !extOf(target) && g.ext.length === 1) {
          const c = g.carrier!;
          const sol = solveIKWithCarrier(c, robot, multiply(invert(c.poseAbs()), targetAbs(target, e0)), { seed: [...c.joints(), ...(stored ?? q0.slice(0, dof))] });
          if (sol.ok) q1 = [...sol.robotJoints, ...sol.carrierJoints];
        }
      }
    } else if (d.joints && d.joints.length >= dof) q1 = g.full(d.joints, q0);
    else if (d.pose) {
      const P = multiply(invert(g.baseAbs(e0)), multiply(g.refAbs(spec.frame, e0), fromArray(d.pose)));
      const r = robot.solveIK(P, { seed: q0.slice(0, dof) }, tool);
      if (r.ok) q1 = [...r.joints, ...e0];
    }
    if (!q1) return failed(`${target?.name ?? 'pose'} unreachable`);
    if (!inLimits(q1)) return failed(`${target?.name ?? 'Target'} joints outside limits`);
    q1 = clampQ(q1);
    const qa = q0.slice(0, g.n), qb = q1;
    return b.jointGeom((s) => lerpQ(qa, qb, s), tool, spec.settings);
  }
  // ---- Cartesian moves ----
  let ref: Item | null = null, relEnd: Mat4;
  let e1 = e0, inlineFromJoints = false;
  if (target) {
    if (target.isJointTarget && target.joints && target.joints.length >= dof) { relEnd = robot.solveFK(target.joints.slice(0, dof), tool); e1 = extOf(target) ?? e0; }
    else if (relFrame(target)) { ref = tf; relEnd = target.pose(); e1 = extOf(target) ?? e0; }
    else { ref = target; relEnd = identity(); e1 = extOf(target) ?? e0; }
  } else if (d.pose) {
    // inline target (machining / curve-follow moves): its stored 6+N joints carry the synchronised external axes
    ref = spec.frame; relEnd = fromArray(d.pose);
    e1 = extEnd ?? (d.joints && d.joints.length >= g.n && g.n > dof ? d.joints.slice(dof, g.n) : e0);
    // a stored pose that does not match its stored joints (seen in imported stations, e.g. RoboDK 03.i where the
    // joints are what RoboDK plays): the joints win
    if (d.joints && d.joints.length >= dof && d.joints.slice(0, dof).every(Number.isFinite)) {
      const viaJ = multiply(invert(g.refAbs(ref, e1)), multiply(g.baseAbs(e1), robot.solveFK(d.joints.slice(0, dof), tool)));
      if (norm(sub3(posOf(viaJ), posOf(relEnd))) > 1 || rotationAngle(viaJ, relEnd) > 0.01) { relEnd = viaJ; inlineFromJoints = true; }
    }
  }
  else if (d.joints && d.joints.length >= dof) relEnd = robot.solveFK(d.joints.slice(0, dof), tool);
  else return failed('pose could not be resolved');
  const label = d.moveType;
  const refAbs0 = g.refAbs(ref, e0);
  const rel0 = multiply(invert(refAbs0), multiply(g.baseAbs(e0), robot.solveFK(q0.slice(0, dof), tool)));
  let relAt: (s: number) => Mat4 = (s) => slerpPose(rel0, relEnd, s);
  let approxLen = norm(sub3(posOf(relEnd), posOf(rel0)));
  if (d.moveType === 'MoveC') {
    let viaRel: Mat4 | null = null;
    if (spec.via) viaRel = multiply(invert(refAbs0), targetAbs(spec.via, e0));
    else if (d.viaPose) viaRel = multiply(invert(refAbs0), multiply(g.refAbs(spec.frame, e0), fromArray(d.viaPose)));
    if (!viaRel) return failed('MoveC via point missing');
    const circ = circleThrough(posOf(rel0), posOf(viaRel), posOf(relEnd));
    if (circ) { relAt = (s) => withPos(slerpPose(rel0, relEnd, s), circ.at(s)); approxLen = circ.len; }
  }
  const fixedFrame = g.n === dof;
  const A = fixedFrame ? multiply(g.baseNowInv, refAbs0) : null;
  const at = (s: number): CartPoint => {
    const rel = relAt(s);
    if (A) { const P = multiply(A, rel); return { P, e: e0, x: posOf(rel), disp: P }; }
    const e = lerpQ(e0, e1, s);
    const abs = multiply(g.refAbs(ref, e), rel);
    return { P: multiply(invert(g.baseAbs(e)), abs), e, x: posOf(rel), disp: multiply(g.baseNowInv, abs) };
  };
  // carrier: when the end is out of reach and the target has no external-axis values, choose them by combined IK
  if (g.hasCarrier && g.ext.length === 1 && target && !extOf(target) && !b.ik(at(1).P, q0, tool).ok) {
    const c = g.carrier!;
    const sol = solveIKWithCarrier(c, robot, multiply(invert(c.poseAbs()), multiply(refAbs0, relEnd)), { seed: [...c.joints(), ...q0.slice(0, dof)] });
    if (sol.ok) return moveGeometry(b, q0, spec, sol.carrierJoints);
  }
  const geo = b.cartGeom(at, q0, tool, spec.settings, label, approxLen);
  if (inlineFromJoints) geo.warnings.push('Inline target pose differs from its stored joints: the joints were used');
  if (geo.ok && target?.isJointTarget && target.joints && maxDelta(geo.knots[geo.knots.length - 1].q, target.joints, dof) > 1) geo.warnings.push(`${label} ends in a different configuration than joint target ${target.name}`);
  return geo;
}

// ---- chain planning ---------------------------------------------------------------------------------------------------

export interface PlannedMove { trajectory: PlannedTrajectory; t0: number; t1: number; qEnd: number[]; warnings: string[] }
export interface ChainResult { moves: PlannedMove[]; duration: number; failed?: { index: number; message: string } }

interface TPiece { move: number; knots: Knot[]; cart: boolean; settings: MotionSettings; stopBefore: boolean }

/** Knots of a geometry restricted to s in [s0, s1] (interpolated boundary knots). */
function cutKnots(geo: Geom, s0: number, s1: number, b: GeomBuilder): Knot[] {
  const ks = geo.knots;
  const at = (s: number): Knot => {
    if (s <= ks[0].s) return ks[0];
    if (s >= ks[ks.length - 1].s) return ks[ks.length - 1];
    let i = 1; while (i < ks.length - 1 && ks[i].s < s) i++;
    const a = ks[i - 1], c = ks[i];
    if (Math.abs(c.s - s) < 1e-12) return c;
    const f = (s - a.s) / (c.s - a.s);
    let q = lerpQ(a.q, c.q, f);
    let pose: Mat4, x: Vec3;
    if (geo.cart && geo.at) {
      // knot intervals are refined until the joint interpolation stays within the path tolerance: interpolate (no IK)
      const p = geo.at(s);
      q = [...q.slice(0, b.g.dof), ...p.e];
      pose = p.disp; x = p.x;
    } else { q = geo.qAt ? geo.qAt(s) : q; pose = b.fkDisp(q, geo.tool); x = posOf(pose); }
    return { s, q, pose, x };
  };
  const out: Knot[] = [at(s0)];
  for (const k of ks) if (k.s > s0 + 1e-12 && k.s < s1 - 1e-12) out.push(k);
  if (s1 > s0 + 1e-12) out.push(at(s1));
  return out;
}

/** s at which the remaining (fromEnd) or travelled path length equals r. */
function sAtLength(geo: Geom, r: number, fromEnd: boolean): number {
  const ks = geo.knots;
  let acc = 0;
  if (fromEnd) {
    for (let i = ks.length - 1; i > 0; i--) { const l = norm(sub3(ks[i].x, ks[i - 1].x)); if (acc + l >= r) return ks[i].s - ((r - acc) / Math.max(l, 1e-12)) * (ks[i].s - ks[i - 1].s); acc += l; }
    return 0;
  }
  for (let i = 1; i < ks.length; i++) { const l = norm(sub3(ks[i].x, ks[i - 1].x)); if (acc + l >= r) return ks[i - 1].s + ((r - acc) / Math.max(l, 1e-12)) * (ks[i].s - ks[i - 1].s); acc += l; }
  return 1;
}

/** Plan a chain of consecutive moves from joints q0 (blending where the rounding of a move is > 0). */
export function planChain(g: MotionGroup, q0: number[], specs: ChainMoveSpec[]): ChainResult {
  const b = new GeomBuilder(g);
  const geoms: Geom[] = [];
  let q = g.full(q0, g.current());
  let failed: ChainResult['failed'];
  for (let k = 0; k < specs.length; k++) {
    const geo = moveGeometry(b, q, specs[k]);
    geoms.push(geo);
    if (!geo.ok) { failed = { index: k, message: geo.error ?? 'motion failed' }; break; }
    q = geo.knots[geo.knots.length - 1].q;
  }
  const nOk = failed ? failed.index : geoms.length;
  // blend zones between successful moves
  const sIn = new Array(geoms.length).fill(0), sOut = new Array(geoms.length).fill(1);
  const blends: Array<Knot[] | null> = new Array(geoms.length).fill(null);
  const blendCart: boolean[] = new Array(geoms.length).fill(false);
  for (let k = 0; k + 1 < nOk; k++) {
    const r = specs[k].settings.rounding;
    const A = geoms[k], B = geoms[k + 1];
    if (!(r > 0) || A.pathLen < 1e-6 || B.pathLen < 1e-6) continue;
    const rA = Math.min(r, A.pathLen / 2), rB = Math.min(r, B.pathLen / 2);
    const sa = Math.max(sIn[k], sAtLength(A, rA, true)), sb = Math.min(sOut[k + 1], sAtLength(B, rB, false));
    if (!(sa < 1 - 1e-9) || !(sb > 1e-9)) continue;
    const ka = cutKnots(A, sa, sa, b)[0], kb = cutKnots(B, sb, sb, b)[0];
    const kc = A.knots[A.knots.length - 1];
    let knots: Knot[] | null = null;
    if (A.cart && B.cart && g.n === g.dof) {
      const pa = ka.x, pc = kc.x, pb = kb.x;
      // Cartesian quadratic Bezier with orientation slerp, in the display (= robot base) frame
      const Pa = ka.pose, Pb = kb.pose;
      const at = (u: number): CartPoint => {
        const w0 = (1 - u) * (1 - u), w1 = 2 * u * (1 - u), w2 = u * u;
        const pos: Vec3 = [w0 * Pa[12] + w1 * kc.pose[12] + w2 * Pb[12], w0 * Pa[13] + w1 * kc.pose[13] + w2 * Pb[13], w0 * Pa[14] + w1 * kc.pose[14] + w2 * Pb[14]];
        const P = withPos(slerpPose(Pa, Pb, u), pos);
        return { P, e: [], x: pos, disp: P };
      };
      void pa; void pc; void pb;
      const geo = b.cartGeom(at, ka.q, A.tool, A.settings, 'Blend', norm(sub3(ka.x, kc.x)) + norm(sub3(kb.x, kc.x)), true, kb.q);
      const ks = geo.knots;
      if (geo.ok && maxDelta(ks[ks.length - 1].q, kb.q, g.dof) < 1) { knots = ks; blendCart[k] = true; }
    }
    if (!knots) {
      const qa = ka.q, qc = kc.q, qb = kb.q;
      const geo = b.jointGeom((u) => qa.map((v, i) => (1 - u) * (1 - u) * v + 2 * u * (1 - u) * qc[i] + u * u * qb[i]), A.tool, A.settings, [0.5]);
      knots = geo.knots;
    }
    sOut[k] = sa; sIn[k + 1] = sb; blends[k] = knots;
  }
  // pieces in execution order
  const pieces: TPiece[] = [];
  for (let k = 0; k < geoms.length; k++) {
    const geo = geoms[k];
    const main = k < nOk ? cutKnots(geo, sIn[k], sOut[k], b) : geo.knots;
    const blendedIn = k > 0 && blends[k - 1] !== null;
    if (blendedIn) {
      const bk = blends[k - 1]!, mid = bk.findIndex((x) => Math.abs(x.s - 0.5) < 1e-9);
      pieces.push({ move: k, knots: bk.slice(mid), cart: blendCart[k - 1], settings: specs[k].settings, stopBefore: false });
    }
    pieces.push({ move: k, knots: main, cart: geo.cart, settings: specs[k].settings, stopBefore: !blendedIn });
    if (blends[k]) {
      const bk = blends[k]!, mid = bk.findIndex((x) => Math.abs(x.s - 0.5) < 1e-9);
      pieces.push({ move: k, knots: bk.slice(0, mid + 1), cart: blendCart[k], settings: specs[k].settings, stopBefore: false });
    }
  }
  // flatten into intervals
  interface Iv { a: Knot; b: Knot; move: number; dtau: number; acc: number; vfull: number; cart: boolean }
  const ivs: Iv[] = [];
  const stops = new Set<number>(); // node j = start of interval j
  for (const p of pieces) {
    const s = p.settings;
    if (p.stopBefore) stops.add(ivs.length);
    for (let i = 1; i < p.knots.length; i++) {
      const a = p.knots[i - 1], c = p.knots[i];
      // dtau: interval time at full speed (programmed linear / rotation / joint speeds and per-joint limits)
      let dt = 0, dx = 0, dth = 0;
      const dq: number[] = [];
      for (let j = 0; j < g.n; j++) {
        const dqj = Math.abs((c.q[j] ?? 0) - (a.q[j] ?? 0));
        dq.push(dqj);
        if (dqj < 1e-12) continue;
        const w = p.cart ? g.vmax[j] : g.jointSpeed(j, s);
        if (Number.isFinite(w) && w > 0) dt = Math.max(dt, dqj / w);
      }
      if (p.cart) {
        dx = norm(sub3(c.x, a.x));
        dth = rotationAngle(a.pose, c.pose) * RAD;
        dt = Math.max(dt, dx / Math.max(1e-9, s.speedLinear), dth / Math.max(1e-9, s.speedJoints));
      }
      // acceleration of the speed fraction (1/s^2): linear acceleration on the path, joint accelerations (programmed
      // joint acceleration for joint moves, robot maximum per joint) on every moving joint. No separate rotational
      // acceleration limit: RoboDK cycle times of reorientation-heavy paths (02.g Tour Eiffel) match without it
      let acc = Infinity;
      if (dt > 0) {
        for (let j = 0; j < g.n; j++) {
          if (dq[j] < 1e-12) continue;
          const aj = p.cart ? g.amax[j] : Math.min(g.amax[j], g.prismatic[j] ? s.accelLinear : s.accelJoints);
          if (aj > 0 && Number.isFinite(aj)) acc = Math.min(acc, (aj * dt) / dq[j]);
        }
        if (p.cart && dx > 1e-9 && s.accelLinear > 0) acc = Math.min(acc, (s.accelLinear * dt) / dx);
      }
      ivs.push({ a, b: c, move: p.move, dtau: dt, acc, vfull: dt > 0 ? dx / dt : 0, cart: p.cart });
    }
  }
  // velocity passes (phi = fraction of the full speed of each interval)
  const N = ivs.length;
  const kap = new Array(N + 1).fill(1);
  for (let j = 1; j < N; j++) {
    const A = ivs[j - 1], B = ivs[j];
    if (A.cart && B.cart && A.vfull > 1e-9 && B.vfull > 1e-9) kap[j] = A.vfull / B.vfull;
  }
  const phiL = new Array(N + 1).fill(0), phiR = new Array(N + 1).fill(0);
  const isStop = (j: number) => j === 0 || j === N || stops.has(j);
  for (let j = 0; j < N; j++) {
    const v = Math.sqrt(phiR[j] * phiR[j] + (ivs[j].dtau > 0 ? 2 * ivs[j].acc * ivs[j].dtau : 0));
    const nx = j + 1;
    if (isStop(nx)) { phiL[nx] = 0; phiR[nx] = 0; continue; }
    phiL[nx] = Math.min(1, 1 / kap[nx], v);
    phiR[nx] = Math.min(1, kap[nx] * phiL[nx]);
  }
  for (let j = N - 1; j >= 0; j--) {
    const v = Math.sqrt(phiL[j + 1] * phiL[j + 1] + (ivs[j].dtau > 0 ? 2 * ivs[j].acc * ivs[j].dtau : 0));
    if (isStop(j)) { phiL[j] = 0; phiR[j] = 0; continue; }
    phiR[j] = Math.min(phiR[j], v);
    phiL[j] = Math.min(phiL[j], phiR[j] / kap[j]);
    phiR[j] = Math.min(phiR[j], kap[j] * phiL[j]);
  }
  // per-move trajectories
  const moves: PlannedMove[] = [];
  let t = 0;
  let j = 0;
  for (let k = 0; k < geoms.length; k++) {
    const geo = geoms[k];
    const samples: TrajectorySample[] = [];
    const profiles: Profile[] = [];
    const t0 = t;
    let tl = 0, len = 0;
    while (j < N && ivs[j].move === k) {
      const iv = ivs[j];
      if (!samples.length) samples.push({ t: 0, joints: iv.a.q, pose: iv.a.pose });
      const prof = makeProfile(iv.dtau, phiR[j], phiL[j + 1], 1, iv.acc);
      profiles.push(prof);
      tl += prof.T;
      len += norm(sub3(posOf(iv.b.pose), posOf(iv.a.pose)));
      samples.push({ t: tl, joints: iv.b.q, pose: iv.b.pose });
      j++;
    }
    if (!samples.length) { const kn = geo.knots[geo.knots.length - 1]; samples.push({ t: 0, joints: kn.q, pose: kn.pose }); }
    t += tl;
    const ok = geo.ok;
    const tr = new PlannedTrajectory(samples, profiles, len, ok, geo.error);
    moves.push({ trajectory: tr, t0, t1: t, qEnd: samples[samples.length - 1].joints, warnings: geo.warnings });
  }
  return { moves, duration: t, failed };
}

/** Plan a single move (helpers for API / UI callers). */
export function planSingle(robot: Robot, q0: number[], spec: Omit<ChainMoveSpec, 'frame' | 'via' | 'target'> & Partial<ChainMoveSpec>): PlannedMove {
  const g = new MotionGroup(robot, []);
  const res = planChain(g, q0, [{ frame: null, via: null, target: null, ...spec }]);
  return res.moves[0];
}

export { getPos };
