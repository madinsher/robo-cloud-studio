/**
 * MUROM arm kinematics — a line-by-line port of murom_manipulation/arm_kinematics.py (MURKA robot-retailer v2).
 *
 * Chain (base_link frame, metres / radians, left arm; the right arm is mirrored in y):
 *   shoulder_pitch (axis Y) @ (-0.092, ±0.185, 1.447) -> shoulder_roll (X) @ +(0, ±0.083, -0.0023)
 *   -> shoulder_yaw (Z) @ +(0, ±0.087, -0.0047) -> elbow (Y') @ +(0, ∓0.003, -0.281) -> tip @ +(0, 0, -0.248)
 * v2 ("capsule") adds a wrist (roll about the forearm, pitch 0.03 further about x) and a palm; the grasp point is the
 * centre of the palm surface. IK is the original damped least squares on the tip position with joint limits, the palm
 * IK iterates the tip target, and `ikPalmSelect` picks the solution like the original (error, palm tilt, penetration
 * of arm / hand / load into the torso and given boxes, clean joint-linear path from the reference, closest to it).
 * The 48 random selection seeds are numpy's `default_rng(0).uniform(...)` values, so seeds equal the original.
 */

export type V3 = [number, number, number];
/** Row-major 3×3. */
export type M3 = number[];
export type Box = [V3, V3];
/** Load in the palm frame: centre, half sizes, optional rotation (row-major 3×3). */
export type Load = [V3, V3, M3?];

export const JOINT_LIMITS: Record<'shoulder_pitch' | 'shoulder_roll' | 'shoulder_yaw' | 'elbow', [number, number]> = {
  shoulder_pitch: [-2.6, 2.6], shoulder_roll: [-2.1, 2.1], shoulder_yaw: [-1.6, 1.6], elbow: [-2.4, 2.4],
};
export const TOOL_OFFSET: V3 = [0, 0, -0.248];
export const WRIST_PITCH_DZ = -0.03;
export const PALM_SURFACE: [number, number] = [0.016, -0.054];
export const WRIST_LIMITS = { wrist_roll: [-2.6, 2.6] as [number, number], wrist_pitch: [-1.55, 1.55] as [number, number] };
export const TORSO_BOX: Box = [[-0.18, -0.20, 0.612], [0.07, 0.20, 1.502]];
export const BASKET_BOX: Box = [[-0.496, -0.251, 0.85], [-0.179, 0.251, 0.978]];
export const R_UPPER_ARM = 0.046, R_FOREARM = 0.040;
export const PEN_TOL = 0.002;
export const SELECT_DLS_ITERS = 40;

const grid = (xs: number[], ys: number[], zs: number[]): V3[] => { const o: V3[] = []; for (const x of xs) for (const y of ys) for (const z of zs) o.push([x, y, z]); return o; };
const WRIST_PTS = grid([-0.033, 0, 0.033], [-0.03, 0, 0.03], [-0.002, -0.046]);
const PALM_PTS = grid([-0.044, 0, 0.04, 0.076], [-0.024, 0.018], [-0.01, -0.05, -0.09, -0.13, -0.16, -0.195]);
const LOAD_PTS = grid([-1, 0, 1], [-1, 0, 1], [-1, 0, 1]).filter(([x, y, z]) => x || y || z);

/** numpy.random.default_rng(0).uniform([-2.6, -2.1, -1.6, -2.4], [2.6, 2.1, 1.6, 2.4], (48, 4)).round(3) */
const RNG_SEEDS: number[][] = [[0.712, -0.967, -1.469, -2.321], [1.629, 1.734, 0.341, 1.102], [0.227, 1.827, 1.011, -2.387], [1.859, -1.959, 0.735, -1.557], [1.889, 0.174, -0.641, -0.371], [-2.453, -1.578, 0.546, 0.707], [0.6, -0.489, 1.591, 2.308], [0.965, 0.632, 0.603, -0.533], [-1.897, 0.93, 0.081, -0.911], [-0.074, 1.636, 1.389, -0.683], [0.372, -0.748, 0.302, -0.778], [-0.564, 1.639, -0.873, 0.591], [-2.163, 1.397, 0.919, -1.251], [1.958, -1.854, -0.524, -1.679], [-0.258, 1.245, -0.862, -2.15], [-0.496, -1.266, -1.31, 0.386], [-1.047, 0.722, -0.962, 2.122], [-0.701, -1.657, 0.413, 2.05], [-0.31, 1.909, -0.0, -0.359], [0.625, 2.079, 1.437, -0.192], [1.34, -0.011, 0.094, 1.372], [-0.444, 0.985, 0.676, 2.074], [-2.002, 0.962, 1.368, 2.246], [-2.524, 1.527, 1.54, 2.195], [-1.826, 1.985, 1.248, 1.547], [-0.104, -1.124, 0.966, 2.033], [-1.216, 0.164, -0.183, 2.069], [-2.389, 0.974, 0.366, -2.264], [1.14, -2.033, 0.825, 0.061], [2.231, -1.822, 1.092, -2.08], [-0.81, -0.293, 1.491, 0.299], [-1.254, -1.085, 1.242, -1.316], [-1.952, -0.889, 0.276, 0.26], [1.61, 0.254, -0.677, -0.418], [1.654, 0.531, 1.469, -0.627], [0.274, 0.394, 1.115, -1.702], [-0.486, 1.722, -1.462, 1.549], [-0.44, 1.385, -1.568, -0.648], [-2.191, 0.641, -0.724, 0.973], [2.308, -1.567, 1.167, -2.115], [-0.62, -0.295, -0.036, 2.287], [1.434, -0.803, -0.737, 1.743], [1.983, 0.045, -0.498, 2.376], [-0.957, -1.333, 1.216, 1.499], [0.873, 1.925, 1.362, 1.192], [1.876, -1.062, -1.148, 0.816], [1.116, -1.398, -0.334, 1.969], [0.319, 0.329, -0.979, 0.125]];
export const SELECT_SEEDS: number[][] = [
  [0.0, 0.8, 1.5, -0.9], [0.0, 0.8, 1.5, 0.9], [0.0, 0.4, 1.5, -1.4],
  [-0.6, 0.5, 0.0, -0.8], [0.6, 0.5, 0.0, 0.8], [0.0, 0.1, 0.0, 0.0],
  [0.5, 0.1, 1.6, 0.9], [0.5, 0.1, 1.6, -0.9], [0.5, 0.1, -1.6, 0.9], [0.5, 0.1, -1.6, -0.9],
  [0.15, -0.15, 0.7, 0.9], [0.0, -0.24, 1.57, -1.74], [0.0, -0.24, -1.57, 1.74],
  [0.0, 0.0, 0.0, -1.8], [0.0, 0.35, 0.0, -1.2],
  ...RNG_SEEDS,
];

// ------------------------------------------------------------------ small linear algebra
export const rot = (axis: 'x' | 'y' | 'z', q: number): M3 => {
  const c = Math.cos(q), s = Math.sin(q);
  if (axis === 'x') return [1, 0, 0, 0, c, -s, 0, s, c];
  if (axis === 'y') return [c, 0, s, 0, 1, 0, -s, 0, c];
  return [c, -s, 0, s, c, 0, 0, 0, 1];
};
export const mm3 = (a: M3, b: M3): M3 => {
  const o = new Array(9).fill(0);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) o[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
  return o;
};
export const mv3 = (a: M3, v: ArrayLike<number>): V3 => [a[0] * v[0] + a[1] * v[1] + a[2] * v[2], a[3] * v[0] + a[4] * v[1] + a[5] * v[2], a[6] * v[0] + a[7] * v[1] + a[8] * v[2]];
export const tr3 = (a: M3): M3 => [a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]];
const add = (a: ArrayLike<number>, b: ArrayLike<number>): V3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a: ArrayLike<number>, b: ArrayLike<number>): V3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const nrm = (a: ArrayLike<number>) => Math.hypot(a[0], a[1], a[2]);
const clamp = (v: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, v));

/** Signed distance of a point to an axis-aligned box [lo, hi] (< 0 inside). */
export function sdBox(p: ArrayLike<number>, lo: ArrayLike<number>, hi: ArrayLike<number>): number {
  let out = 0, inMax = -Infinity;
  for (let a = 0; a < 3; a++) { const q = Math.abs(p[a] - (lo[a] + hi[a]) / 2) - (hi[a] - lo[a]) / 2; out += Math.max(q, 0) ** 2; inMax = Math.max(inMax, q); }
  return Math.sqrt(out) + Math.min(inMax, 0);
}

/** 3×3 linear solve (Gaussian elimination with partial pivoting). */
function solve3(A: number[], b: number[]): number[] {
  const M = [[A[0], A[1], A[2], b[0]], [A[3], A[4], A[5], b[1]], [A[6], A[7], A[8], b[2]]];
  for (let c = 0; c < 3; c++) {
    let p = c; for (let r = c + 1; r < 3; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = c + 1; r < 3; r++) { const f = M[r][c] / M[c][c]; for (let k = c; k < 4; k++) M[r][k] -= f * M[c][k]; }
  }
  const x = [0, 0, 0];
  for (let r = 2; r >= 0; r--) { let s = M[r][3]; for (let k = r + 1; k < 3; k++) s -= M[r][k] * x[k]; x[r] = s / M[r][r]; }
  return x;
}

export interface PalmSelection { q: number[]; wrist: [number, number]; err: number; pathPen: number }

/** FK / IK of one MUROM arm (side 'left' = +y, 'right' = -y). */
export class ArmKinematics {
  readonly mirror: number;
  readonly pShoulder: V3; readonly oRoll: V3; readonly oYaw: V3; readonly oElbow: V3;
  readonly jointNames: string[]; readonly wristNames: string[];
  readonly limits: Array<[number, number]>;
  constructor(readonly side: 'left' | 'right') {
    const m = side === 'left' ? 1 : -1;
    this.mirror = m;
    this.pShoulder = [-0.092, m * 0.185, 1.447];
    this.oRoll = [0, m * 0.083, -0.0023];
    this.oYaw = [0, m * 0.087, -0.0047];
    this.oElbow = [0, -m * 0.003, -0.281];
    const s = side[0];
    this.jointNames = [`${s}_shoulder_pitch_joint`, `${s}_shoulder_roll_joint`, `${s}_shoulder_yaw_joint`, `${s}_elbow_joint`];
    this.wristNames = [`${s}_wrist_roll_joint`, `${s}_wrist_pitch_joint`];
    this.limits = [JOINT_LIMITS.shoulder_pitch, JOINT_LIMITS.shoulder_roll, JOINT_LIMITS.shoulder_yaw, JOINT_LIMITS.elbow];
  }

  /** Tip position (base_link) for q = [pitch, roll, yaw, elbow]. */
  fk(q: ArrayLike<number>): V3 {
    const [qp, qr, qy, qe] = [q[0], q[1], q[2], q[3]];
    let r = mm3(rot('y', qp), rot('x', qr));
    let p = add(this.pShoulder, mv3(rot('y', qp), this.oRoll));
    p = add(p, mv3(r, this.oYaw));
    r = mm3(r, rot('z', qy));
    p = add(p, mv3(r, this.oElbow));
    r = mm3(r, rot('y', qe));
    return add(p, mv3(r, TOOL_OFFSET));
  }
  /** (yaw joint, elbow, tip, forearm rotation) in base_link. */
  chain(q: ArrayLike<number>): [V3, V3, V3, M3] {
    let r = mm3(rot('y', q[0]), rot('x', q[1]));
    const pYaw = add(add(this.pShoulder, mv3(rot('y', q[0]), this.oRoll)), mv3(r, this.oYaw));
    r = mm3(r, rot('z', q[2]));
    const pElb = add(pYaw, mv3(r, this.oElbow));
    r = mm3(r, rot('y', q[3]));
    return [pYaw, pElb, add(pElb, mv3(r, TOOL_OFFSET)), r];
  }
  forearmRot(q: ArrayLike<number>): M3 { return mm3(mm3(mm3(rot('y', q[0]), rot('x', q[1])), rot('z', q[2])), rot('y', q[3])); }
  /** (roll, pitch) that point the palm normal ((0, -m, 0) in the palm frame) along `normal` (base_link). */
  wristForNormal(q: ArrayLike<number>, normal: ArrayLike<number>): [number, number] {
    const m = this.mirror; const n = nrm(normal);
    const v = mv3(tr3(this.forearmRot(q)), [normal[0] / n, normal[1] / n, normal[2] / n]);
    const pitch = Math.asin(clamp(-m * v[2], -1, 1));
    const roll = Math.atan2(m * v[0], -m * v[1]);
    return [clamp(roll, ...WRIST_LIMITS.wrist_roll), clamp(pitch, ...WRIST_LIMITS.wrist_pitch)];
  }
  /** Rotation and origin of the palm frame (L_PALM / R_PALM node) in base_link. */
  palmFrame(q: ArrayLike<number>, wrist: ArrayLike<number>): [M3, V3] {
    const rf = this.forearmRot(q);
    return [mm3(mm3(rf, rot('z', wrist[0])), rot('x', wrist[1])), add(this.fk(q), mv3(rf, [0, 0, WRIST_PITCH_DZ]))];
  }
  /** Centre of the palm surface (v2 magnetic grasp point) or another palm-frame point, in base_link. */
  palmPoint(q: ArrayLike<number>, wrist: ArrayLike<number>, point?: ArrayLike<number> | null): V3 {
    const [rp, origin] = this.palmFrame(q, wrist);
    const pt = point ?? [0, -this.mirror * PALM_SURFACE[0], PALM_SURFACE[1]];
    return add(origin, mv3(rp, pt));
  }
  /** Palm IK: tip target iterated so the palm (normal `normal`) reaches `target`. Returns [q, wrist, err]. */
  ikPalm(target: V3, normal: V3, seed: number[] | null = null, o: { tol?: number; iters?: number; fallback?: boolean; dlsIters?: number; point?: ArrayLike<number> | null } = {}): [number[], [number, number], number] {
    const tol = o.tol ?? 0.01, iters = o.iters ?? 6, fallback = o.fallback ?? true, dlsIters = o.dlsIters ?? 150;
    let tip: V3 = [...target];
    if (seed) tip = sub(target, sub(this.palmPoint(seed, this.wristForNormal(seed, normal), o.point), this.fk(seed)));
    let q: number[] | null = seed, w: [number, number] = [0, 0], err = Infinity;
    for (let i = 0; i < iters; i++) {
      q = this.ik(tip, q, tol * 0.3, dlsIters, fallback)[0];
      w = this.wristForNormal(q, normal);
      const d = sub(target, this.palmPoint(q, w, o.point));
      err = nrm(d);
      if (err < 0.002) break;
      tip = add(tip, d);
    }
    return [q!, w, err];
  }
  /** Angle (deg) between the palm normal and `normal`, after the wrist limits. */
  normalErr(q: ArrayLike<number>, wrist: ArrayLike<number>, normal: ArrayLike<number>): number {
    const rp = mm3(mm3(this.forearmRot(q), rot('z', wrist[0])), rot('x', wrist[1]));
    const n = nrm(normal);
    const pn = mv3(rp, [0, -this.mirror, 0]);
    const c = (pn[0] * normal[0] + pn[1] * normal[1] + pn[2] * normal[2]) / n;
    return Math.acos(clamp(c, -1, 1)) * 180 / Math.PI;
  }
  /** Sample points of the hand (wrist box + palm with fingers) and of a load, in base_link. */
  handPoints(q: ArrayLike<number>, wrist: ArrayLike<number>, load?: Load | null): V3[] {
    const [, , tip, rf] = this.chain(q);
    const rw = mm3(rf, rot('z', wrist[0]));
    const rp = mm3(rw, rot('x', wrist[1]));
    const org = add(tip, mv3(rf, [0, 0, WRIST_PITCH_DZ]));
    const out: V3[] = [];
    for (const p of WRIST_PTS) out.push(add(mv3(rw, p), tip));
    for (const p of PALM_PTS) out.push(add(mv3(rp, [p[0], this.mirror * p[1], p[2]]), org));
    if (load) {
      const rl = load[2] ?? [1, 0, 0, 0, 1, 0, 0, 0, 1];
      for (const p of LOAD_PTS) out.push(add(mv3(rp, add(load[0], mv3(rl, [p[0] * load[1][0], p[1] * load[1][1], p[2] * load[1][2]]))), org));
    }
    return out;
  }
  /** Penetration depth (m) of upper arm / forearm (capsules), hand and load (points) into the boxes; 0 = clear. */
  bodyPen(q: ArrayLike<number>, wrist: ArrayLike<number>, boxes: Box[], load?: Load | null): number {
    const [pYaw, pElb, tip] = this.chain(q);
    const hand = this.handPoints(q, wrist, load);
    let depth = 0;
    for (const [lo, hi] of boxes) {
      let m = Infinity; for (const p of hand) m = Math.min(m, sdBox(p, lo, hi));
      depth = Math.max(depth, -m);
      for (const [a, b, r] of [[pYaw, pElb, R_UPPER_ARM], [pElb, tip, R_FOREARM]] as Array<[V3, V3, number]>) {
        let mm = Infinity;
        for (let i = 0; i < 15; i++) { const t = i / 14; mm = Math.min(mm, sdBox([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])], lo, hi)); }
        depth = Math.max(depth, r - mm);
      }
    }
    return depth;
  }
  /** Penetration along the joint-linear path (q0, w0) -> (q1, w1) (ends excluded), m. */
  pathPen(q0: number[], w0: ArrayLike<number>, q1: number[], w1: ArrayLike<number>, boxes: Box[], load?: Load | null, step = 0.1): number {
    let dq = 0; for (let i = 0; i < 4; i++) dq = Math.max(dq, Math.abs(q1[i] - q0[i])); for (let i = 0; i < 2; i++) dq = Math.max(dq, Math.abs(w1[i] - w0[i]));
    const n = Math.max(2, Math.ceil(dq / step) + 1);
    let out = 0;
    for (let k = 1; k < n - 1; k++) {
      const t = k / (n - 1);
      out = Math.max(out, this.bodyPen(q0.map((v, i) => v + t * (q1[i] - v)), [w0[0] + t * (w1[0] - w0[0]), w0[1] + t * (w1[1] - w0[1])], boxes, load));
    }
    return out;
  }
  /**
   * Deterministic palm IK selection (ik_palm_select): candidates from ref + SELECT_SEEDS + an unseeded solve; drop
   * those over `tol`, with the palm tilted more than maxNormalDeg or penetrating the torso / obstacles more than penTol;
   * prefer error ≤ 5 mm, then the closest to ref by max|Δq|; the first with a clean joint-linear path from every pose
   * in `paths` wins, else the one with the least path penetration. null when nothing qualifies.
   */
  ikPalmSelect(target: V3, normal: V3, o: { ref?: [number[], ArrayLike<number>] | null; paths?: Array<[number[], ArrayLike<number>, (Load | null)?]> | null; tol?: number; maxNormalDeg?: number; obstacles?: Box[]; load?: Load | null; penTol?: number; point?: ArrayLike<number> | null } = {}): PalmSelection | null {
    const tol = o.tol ?? 0.02, maxN = o.maxNormalDeg ?? 3, penTol = o.penTol ?? PEN_TOL, load = o.load ?? null;
    const boxes: Box[] = [TORSO_BOX, ...(o.obstacles ?? [])];
    const m = this.mirror;
    const seeds: Array<number[] | null> = [...SELECT_SEEDS.map(([p, r, y, e]) => [p, m * r, m * y, e]), null];
    let refQ: number[] | null = null, refW: number[] | null = null;
    if (o.ref) { refQ = [...o.ref[0]]; refW = [o.ref[1][0], o.ref[1][1]]; seeds.unshift(refQ); }
    const paths = (o.paths !== undefined && o.paths !== null ? o.paths : o.ref ? [o.ref] : []).map((p) => [p[0], p[1], p.length > 2 ? p[2] ?? null : load] as [number[], ArrayLike<number>, Load | null]);
    const loads: Array<Load | null> = [load, ...paths.map((p) => p[2]).filter((l): l is Load => !!l && l !== load)];
    const cands: Array<{ key: [boolean, number, number]; q: number[]; w: [number, number]; err: number }> = [];
    const seen: number[][] = [];
    for (const s of seeds) {
      const [q, w, err] = this.ikPalm(target, normal, s, { fallback: s === null, dlsIters: s === null ? 150 : SELECT_DLS_ITERS, point: o.point });
      if (seen.some((q0) => Math.max(...q.map((v, i) => Math.abs(v - q0[i]))) < 1e-4)) continue;
      seen.push(q);
      if (err > tol || this.normalErr(q, w, normal) > maxN || Math.max(...loads.map((ld) => this.bodyPen(q, w, boxes, ld))) > penTol) continue;
      let dq = 0;
      if (refQ && refW) dq = Math.max(Math.max(...q.map((v, i) => Math.abs(v - refQ![i]))), Math.abs(w[0] - refW[0]), Math.abs(w[1] - refW[1]));
      cands.push({ key: [err > 0.005, Math.round(dq * 1e6) / 1e6, Math.round(err * 1e6) / 1e6], q, w, err });
    }
    cands.sort((a, b) => (Number(a.key[0]) - Number(b.key[0])) || (a.key[1] - b.key[1]) || (a.key[2] - b.key[2]));
    let best: PalmSelection | null = null, bestKey = Infinity;
    for (const c of cands) {
      const path = Math.max(0, ...paths.map(([q0, w0, ld]) => this.pathPen(q0, w0, c.q, c.w, boxes, ld)));
      if (path <= penTol) return { q: c.q, wrist: c.w, err: c.err, pathPen: path };
      if (Math.round(path * 1e3) / 1e3 < bestKey) { best = { q: c.q, wrist: c.w, err: c.err, pathPen: path }; bestKey = Math.round(path * 1e3) / 1e3; }
    }
    return best;
  }
  /** Tip IK (damped least squares, finite-difference Jacobian, limits). Returns [q, err]. */
  ik(target: V3, seed: number[] | null = null, tol = 0.01, iters = 150, fallback = true): [number[], number] {
    const m = this.mirror;
    const seeds: number[][] = [];
    if (seed) seeds.push([...seed]);
    if (fallback || !seeds.length) seeds.push([0, m * 0.8, m * 1.5, -0.9], [0, m * 0.8, m * 1.5, 0.9], [0, m * 0.4, m * 1.5, -1.4], [-0.6, m * 0.5, 0, -0.8], [0.6, m * 0.5, 0, 0.8], [0, m * 0.1, 0, 0]);
    let bestQ: number[] | null = null, bestErr = Infinity;
    for (const s of seeds) {
      const [q, err] = this.dls(target, s, tol, iters);
      if (err < bestErr) { bestQ = q; bestErr = err; }
      if (bestErr < tol) break;
    }
    return [bestQ!, bestErr];
  }
  private dls(target: V3, seed: number[], tol: number, iters: number): [number[], number] {
    const q = [...seed]; const lam2 = 0.05 ** 2;
    for (let it = 0; it < iters; it++) {
      const p = this.fk(q); const e = sub(target, p);
      if (nrm(e) < tol) break;
      const J = this.jacobian(q, p); // 3×4 row-major
      const JJt = [0, 0, 0, 0, 0, 0, 0, 0, 0];
      for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) { let s = 0; for (let k = 0; k < 4; k++) s += J[i * 4 + k] * J[j * 4 + k]; JJt[i * 3 + j] = s + (i === j ? lam2 : 0); }
      const y = solve3(JJt, e);
      const dq = [0, 1, 2, 3].map((k) => J[k] * y[0] + J[4 + k] * y[1] + J[8 + k] * y[2]);
      const step = Math.hypot(...dq);
      const sc = step > 0.5 ? 0.5 / step : 1;
      for (let k = 0; k < 4; k++) q[k] = clamp(q[k] + dq[k] * sc, this.limits[k][0], this.limits[k][1]);
    }
    return [q, nrm(sub(target, this.fk(q)))];
  }
  private jacobian(q: number[], p0: V3, eps = 1e-5): number[] {
    const J = new Array(12).fill(0);
    for (let i = 0; i < 4; i++) { const d = [...q]; d[i] += eps; const p = this.fk(d); for (let r = 0; r < 3; r++) J[r * 4 + i] = (p[r] - p0[r]) / eps; }
    return J;
  }
}
