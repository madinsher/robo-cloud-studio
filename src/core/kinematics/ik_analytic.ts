/**
 * Closed-form inverse kinematics (product of exponentials + Paden–Kahan subproblems), derived from the chain geometry
 * itself, so joint senses, DH/DHM offsets, base and flange transforms and J2/J3 couplings are all honoured:
 *  - 'spherical6': 6R arm whose axes 4,5,6 meet in one point and axes 2,3 are parallel (KUKA, ABB, Fanuc, Yaskawa,
 *    Kawasaki, Stäubli, Comau, Meca500 ...). Up to 8 solutions: shoulder (front/rear) × elbow × wrist flip.
 *  - 'ur6': 6R arm with axes 2,3,4 parallel, axes 4/5 and 5/6 intersecting (UR, JAKA, Omron TM, Doosan ...). Up to 8.
 *  - 'delta': 3/4-axis delta (ABB IRB 360): one closed-form solution per arm (elbow out).
 * Everything else uses the numeric solver in ik.ts.
 */
import { Mat4, Vec3, multiply, invert, DEG, RAD } from '../math/pose';
import { ChainDef, chainInfo, forwardKinematics, flangePoseInto, onInvalidateChain, deltaPlatformPosition, deltaMotorFrame } from './chain';

export type AnalyticKind = 'spherical6' | 'ur6' | 'delta';

interface Screw { w: Vec3; p: Vec3 }

export interface ChainAnalysis {
  kind: AnalyticKind | null;
  /** Joint index (in chain.joints) of each actuated joint. */
  idx: number[];
  screws: Screw[];
  /** Flange at q = 0 (relative to the robot base frame). */
  g0: Mat4;
  g0inv: Mat4;
  /** spherical6: wrist centre; ur6: axis 4 ∩ axis 5 at q = 0. */
  wA: Vec3;
  /** ur6: axis 5 ∩ axis 6 at q = 0. */
  wB: Vec3;
  /** Coupling: motion value v_k = q_k + m_k · q_src(k) (indices into q). */
  coupleSrc: number[];
  coupleMul: number[];
  /** Characteristic length (mm) for tolerances. */
  scale: number;
  /** Forward direction ⊥ axis 1 at q1 = 0 (X of link 1), for the REAR flag. */
  front: Vec3;
  /** Elbow motion value (deg) at which upper arm and forearm align, in (-180, 180]. */
  elbowStretch: number;
  /** Why the chain is not analytic (diagnostics). */
  reason?: string;
}

// -- small vector helpers (allocation-light) ---------------------------------------------------------------
const dot = (a: Vec3, b: Vec3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const crs = (a: Vec3, b: Vec3): Vec3 => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const sub3 = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add3 = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scl3 = (a: Vec3, k: number): Vec3 => [a[0] * k, a[1] * k, a[2] * k];
const nrm3 = (a: Vec3) => Math.hypot(a[0], a[1], a[2]);
const unit = (a: Vec3): Vec3 => { const l = nrm3(a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };

/** Rodrigues rotation of vector v about unit axis w by angle t. */
function rotV(w: Vec3, t: number, v: Vec3): Vec3 {
  const c = Math.cos(t), s = Math.sin(t), d = dot(w, v) * (1 - c), x = crs(w, v);
  return [v[0] * c + x[0] * s + w[0] * d, v[1] * c + x[1] * s + w[1] * d, v[2] * c + x[2] * s + w[2] * d];
}
/** Rotation of point x about the line (w, p). */
function rotP(s: Screw, t: number, x: Vec3): Vec3 { return add3(s.p, rotV(s.w, t, sub3(x, s.p))); }

/** 4x4 rigid motion e^{ξθ} (rotation about a line). */
function screwMat(s: Screw, t: number, m: Mat4 = new Float64Array(16)): Mat4 {
  const [x, y, z] = s.w, c = Math.cos(t), sn = Math.sin(t), k = 1 - c;
  m[3] = m[7] = m[11] = 0;
  m[0] = k * x * x + c; m[4] = k * x * y - sn * z; m[8] = k * x * z + sn * y;
  m[1] = k * x * y + sn * z; m[5] = k * y * y + c; m[9] = k * y * z - sn * x;
  m[2] = k * x * z - sn * y; m[6] = k * y * z + sn * x; m[10] = k * z * z + c;
  // translation (I - R) p
  const p = s.p;
  m[12] = p[0] - (m[0] * p[0] + m[4] * p[1] + m[8] * p[2]);
  m[13] = p[1] - (m[1] * p[0] + m[5] * p[1] + m[9] * p[2]);
  m[14] = p[2] - (m[2] * p[0] + m[6] * p[1] + m[10] * p[2]);
  m[15] = 1;
  return m;
}
const xfP = (m: Mat4, p: Vec3): Vec3 => [m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12], m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13], m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14]];
const xfD = (m: Mat4, p: Vec3): Vec3 => [m[0] * p[0] + m[4] * p[1] + m[8] * p[2], m[1] * p[0] + m[5] * p[1] + m[9] * p[2], m[2] * p[0] + m[6] * p[1] + m[10] * p[2]];

/** PK1: angle about s taking point x to point y (projections onto the plane ⊥ w). */
function pk1(s: Screw, x: Vec3, y: Vec3): number {
  const u = sub3(x, s.p), v = sub3(y, s.p);
  const up = sub3(u, scl3(s.w, dot(s.w, u))), vp = sub3(v, scl3(s.w, dot(s.w, v)));
  return Math.atan2(dot(s.w, crs(up, vp)), dot(up, vp));
}
/** PK1 for free vectors (directions): angle about w taking a to b. Returns NaN when both are parallel to w. */
function pk1v(w: Vec3, a: Vec3, b: Vec3): number {
  const ap = sub3(a, scl3(w, dot(w, a))), bp = sub3(b, scl3(w, dot(w, b)));
  if (nrm3(ap) < 1e-9 || nrm3(bp) < 1e-9) return NaN;
  return Math.atan2(dot(w, crs(ap, bp)), dot(ap, bp));
}
/** PK3: angles about s such that |rot(x) - q| = delta (0, 1 or 2 solutions; a near-miss within tol is clamped). */
function pk3(s: Screw, x: Vec3, q: Vec3, delta: number, tol: number): number[] {
  const u = sub3(x, s.p), v = sub3(q, s.p);
  const up = sub3(u, scl3(s.w, dot(s.w, u))), vp = sub3(v, scl3(s.w, dot(s.w, v)));
  const axial = dot(s.w, sub3(u, v));
  const dp2 = delta * delta - axial * axial;
  const a = nrm3(up), b = nrm3(vp);
  if (a < 1e-12 || b < 1e-12) return [];
  const th0 = Math.atan2(dot(s.w, crs(up, vp)), dot(up, vp));
  let c = (a * a + b * b - dp2) / (2 * a * b);
  if (c > 1 || c < -1) {
    // tolerate rounding at the reach boundary (stretched / folded arm)
    const excess = Math.abs(Math.sqrt(Math.max(0, dp2)) - Math.abs(c > 0 ? a - b : a + b));
    if (excess > tol) return [];
    c = Math.max(-1, Math.min(1, c));
  }
  const ph = Math.acos(c);
  return ph < 1e-12 ? [th0, th0] : [th0 + ph, th0 - ph];
}
/** Solve A cos t + B sin t = C (0 or 2 solutions; tangent case gives two equal ones). */
function trig(A: number, B: number, C: number, tol: number): number[] {
  const r = Math.hypot(A, B);
  if (r < 1e-12) return [];
  let c = C / r;
  if (c > 1 || c < -1) { if (Math.abs(Math.abs(C) - r) > tol) return []; c = Math.max(-1, Math.min(1, c)); }
  const base = Math.atan2(B, A), ph = Math.acos(c);
  return [base + ph, base - ph];
}

function linesClosest(a: Screw, b: Screw): { d: number; pa: Vec3; pb: Vec3 } {
  const r = sub3(b.p, a.p), n = crs(a.w, b.w), nn = dot(n, n);
  if (nn < 1e-18) {
    const perp = sub3(r, scl3(a.w, dot(r, a.w)));
    return { d: nrm3(perp), pa: a.p, pb: sub3(b.p, perp) };
  }
  const ta = dot(crs(r, b.w), n) / nn, tb = dot(crs(r, a.w), n) / nn;
  const pa = add3(a.p, scl3(a.w, ta)), pb = add3(b.p, scl3(b.w, tb));
  return { d: nrm3(sub3(pa, pb)), pa, pb };
}
const parallel = (a: Vec3, b: Vec3) => nrm3(crs(a, b)) < 1e-9;

// -- analysis (cached per chain) -----------------------------------------------------------------------------
const cache = new WeakMap<ChainDef, { sig: unknown[]; a: ChainAnalysis }>();
onInvalidateChain((c) => cache.delete(c));

function signature(chain: ChainDef): unknown[] {
  const s: unknown[] = [chain.joints, chain.joints.length, chain.base, chain.flange, chain.parallel, chain.flangeLink];
  for (const j of chain.joints) s.push(j, j.origin, j.post, j.axis, j.axis[0], j.axis[1], j.axis[2], j.type, j.couple, j.mimic);
  return s;
}
const sameSig = (a: unknown[], b: unknown[]) => a.length === b.length && a.every((v, i) => v === b[i]);

export function analyzeChain(chain: ChainDef): ChainAnalysis {
  const sig = signature(chain);
  const c = cache.get(chain);
  if (c && sameSig(c.sig, sig)) return c.a;
  const a = analyze(chain);
  cache.set(chain, { sig, a });
  return a;
}

function analyze(chain: ChainDef): ChainAnalysis {
  const info = chainInfo(chain);
  const idx = info.idx.slice();
  const fk0 = forwardKinematics(chain, idx.map(() => 0));
  const g0 = fk0.flange;
  const none = (reason: string): ChainAnalysis => ({ kind: null, idx, screws: [], g0, g0inv: invert(g0), wA: [0, 0, 0], wB: [0, 0, 0], coupleSrc: [], coupleMul: [], scale: 1, front: [1, 0, 0], elbowStretch: 0, reason });
  if (chain.parallel?.kind === 'delta') return { ...none(''), kind: 'delta', reason: undefined };
  if (chain.flangeLink !== undefined && chain.flangeLink !== chain.joints.length) return none('flange not on the last link');
  if (idx.length !== 6) return none(`${idx.length} actuated joints`);
  if (chain.joints.some((j) => j.mimic)) return none('mimic joints');
  if (idx.some((i) => chain.joints[i].type === 'prismatic')) return none('prismatic joint');
  const coupleSrc: number[] = [], coupleMul: number[] = [];
  for (let k = 0; k < 6; k++) {
    const cs = info.coupleSrc[idx[k]];
    const qs = cs >= 0 ? info.qIndex[cs] : -1;
    if (cs >= 0 && (qs < 0 || qs >= k)) return none('coupling to a later joint');
    coupleSrc.push(qs);
    coupleMul.push(qs >= 0 ? chain.joints[idx[k]].couple!.multiplier : 0);
  }
  const screws: Screw[] = idx.map((ji) => {
    const f = fk0.jointFrames[ji];
    return { w: unit(xfD(f, chain.joints[ji].axis)), p: [f[12], f[13], f[14]] as Vec3 };
  });
  const scale = 1 + Math.max(...screws.map((s) => nrm3(sub3(s.p, screws[0].p))), nrm3(sub3([g0[12], g0[13], g0[14]], screws[0].p)));
  const tol = 1e-7 * scale;
  const [s1, s2, s3, s4, s5, s6] = screws;
  // forward direction: X axis of link 1 at q = 0, projected ⊥ axis 1
  const l1 = fk0.linkPoses[idx[0] + 1];
  let front: Vec3 = [l1[0], l1[1], l1[2]];
  front = sub3(front, scl3(s1.w, dot(s1.w, front)));
  if (nrm3(front) < 1e-9) front = sub3(s2.p, scl3(s1.w, dot(s1.w, s2.p)));
  front = unit(front);
  const base = { idx, screws, g0, g0inv: invert(g0), coupleSrc, coupleMul, scale, front, elbowStretch: 0 };
  const stretch = (w: Vec3) => {
    // angle about axis 3 from (P3 - P2) to (w - P3) at q = 0; stretched when v3 = -angle
    const p2 = linesClosest(s2, s3).pa, p3 = s3.p;
    const t = -pk1v(s3.w, sub3(p3, p2), sub3(w, p3)) * RAD;
    return Number.isFinite(t) ? wrapDeg(t) : 0;
  };
  if (parallel(s1.w, s2.w)) return none('axes 1 and 2 parallel');
  // spherical wrist: axes 4,5,6 through one point, axes 2 ∥ 3
  if (parallel(s2.w, s3.w) && !parallel(s4.w, s5.w) && !parallel(s5.w, s6.w)) {
    const c45 = linesClosest(s4, s5), c56 = linesClosest(s5, s6);
    if (c45.d < tol && c56.d < tol && nrm3(sub3(c45.pa, c56.pb)) < tol * 10) {
      const w = c45.pa;
      // the wrist centre must not lie on axis 3 (degenerate elbow)
      return { ...base, kind: 'spherical6', wA: w, wB: w, elbowStretch: stretch(w) };
    }
  }
  // UR-like: axes 2,3,4 parallel, 4∩5 and 5∩6
  if (parallel(s2.w, s3.w) && parallel(s3.w, s4.w) && !parallel(s4.w, s5.w) && !parallel(s5.w, s6.w)) {
    const c45 = linesClosest(s4, s5), c56 = linesClosest(s5, s6);
    if (c45.d < tol && c56.d < tol) return { ...base, kind: 'ur6', wA: c45.pa, wB: c56.pa, elbowStretch: stretch(c45.pa) };
  }
  return none('no closed-form structure');
}

// -- solvers ----------------------------------------------------------------------------------------------
export interface AnalyticSolution {
  /** Joint values (deg), each wrapped to (-180, 180]. */
  q: number[];
  /** True when the wrist (or UR q5) was singular and q4/q6 were split using the seed. */
  singular: boolean;
}

/** Geometric motion values (rad) -> joint values (deg), undoing couplings. */
function toJoints(a: ChainAnalysis, th: number[]): number[] {
  const q = new Array(th.length);
  for (let k = 0; k < th.length; k++) {
    let v = th[k] * RAD;
    if (a.coupleSrc[k] >= 0) v -= a.coupleMul[k] * q[a.coupleSrc[k]];
    q[k] = wrapDeg(v);
  }
  return q;
}
/** Joint values (deg) -> geometric motion values (rad). */
function toMotion(a: ChainAnalysis, q: number[]): number[] {
  return q.map((v, k) => (v + (a.coupleSrc[k] >= 0 ? a.coupleMul[k] * q[a.coupleSrc[k]] : 0)) * DEG);
}
export function wrapDeg(v: number): number {
  let r = ((v + 180) % 360 + 360) % 360 - 180;
  if (r <= -180 + 1e-12) r += 360;
  return r;
}

/**
 * All closed-form solutions for a flange pose (relative to the robot base frame), or null when the chain has no
 * closed form. The result may be empty (unreachable). `seed` (deg) resolves free joints at singularities.
 */
export function solveAnalytic(chain: ChainDef, flangeIn: Mat4, seed?: number[]): AnalyticSolution[] | null {
  const a = analyzeChain(chain);
  if (!a.kind) return null;
  // poses read from files carry rounded rotations (6 decimals): solve for the nearest proper rotation
  const flange = orthonormalized(flangeIn);
  if (a.kind === 'delta') return solveDelta(chain, flange, seed);
  const seedTh = seed && seed.length >= 6 ? toMotion(a, seed) : [0, 0, 0, 0, 0, 0];
  const raw = a.kind === 'spherical6' ? solveSpherical(a, flange, seedTh) : solveUR(a, flange, seedTh);
  const out: AnalyticSolution[] = [];
  const chk = new Float64Array(16);
  for (const r of raw) {
    const q = toJoints(a, r.th);
    // verify against the forward model; drops spurious branches of near-degenerate subproblems
    flangePoseInto(chain, q, chk);
    let pe = 0, re = 0;
    for (let k = 12; k < 15; k++) pe = Math.max(pe, Math.abs(chk[k] - flange[k]));
    for (const k of ROT_IDX) re = Math.max(re, Math.abs(chk[k] - flange[k]));
    if (pe > 1e-6 * a.scale + 1e-6 || re > 1e-7) continue;
    if (out.some((o) => o.q.every((v, i) => Math.abs(wrapDeg(v - q[i])) < 1e-6))) continue;
    out.push({ q, singular: r.singular });
  }
  return out;
}

/**
 * The pose with its rotation replaced by the nearest orthonormal one (Z kept, X re-orthogonalised), or the same object
 * when it already is orthonormal to 1e-12.
 */
export function orthonormalized(m: Mat4): Mat4 {
  const x: Vec3 = [m[0], m[1], m[2]], y: Vec3 = [m[4], m[5], m[6]], z: Vec3 = [m[8], m[9], m[10]];
  const dev = Math.max(Math.abs(dot(x, x) - 1), Math.abs(dot(y, y) - 1), Math.abs(dot(z, z) - 1), Math.abs(dot(x, y)), Math.abs(dot(y, z)), Math.abs(dot(x, z)));
  if (dev < 1e-12) return m;
  const zn = unit(z);
  const xn = unit(sub3(x, scl3(zn, dot(x, zn))));
  const yn = crs(zn, xn);
  const out = new Float64Array(m);
  out[0] = xn[0]; out[1] = xn[1]; out[2] = xn[2];
  out[4] = yn[0]; out[5] = yn[1]; out[6] = yn[2];
  out[8] = zn[0]; out[9] = zn[1]; out[10] = zn[2];
  return out;
}

interface RawSol { th: number[]; singular: boolean }
const ROT_IDX = [0, 1, 2, 4, 5, 6, 8, 9, 10];

function solveSpherical(a: ChainAnalysis, gt: Mat4, seedTh: number[]): RawSol[] {
  const [s1, s2, s3, s4, s5, s6] = a.screws;
  const tol = 1e-6 * a.scale;
  const G = multiply(gt, a.g0inv); // = e^{ξ1θ1}...e^{ξ6θ6}
  const W = xfP(G, a.wA);
  const out: RawSol[] = [];
  // θ1: the component of the wrist centre along axis 2 (rotated by θ1) is invariant under θ2, θ3
  const u = s2.w;
  const d = sub3(W, s1.p);
  const upar = scl3(s1.w, dot(u, s1.w)), uperp = sub3(u, upar), uxw = crs(s1.w, u);
  const t1s = trig(dot(uperp, d), dot(uxw, d), dot(u, sub3(a.wA, s1.p)) - dot(upar, d), tol);
  if (!t1s.length && Math.hypot(dot(uperp, d), dot(uxw, d)) < tol) t1s.push(seedTh[0]); // wrist on axis 1
  const pk = linesClosest(s2, s3).pa; // a point on axis 2
  const m1 = new Float64Array(16), m2 = new Float64Array(16), m3 = new Float64Array(16);
  for (const t1 of uniqAngles(t1s)) {
    const V = rotP(s1, -t1, W);
    const delta = nrm3(sub3(V, pk));
    const G1 = multiply(screwMat(s1, -t1), G); // e^{-ξ1θ1} G
    for (const t3 of uniqAngles(pk3(s3, a.wA, pk, delta, tol))) {
      const x = rotP(s3, t3, a.wA);
      const t2 = pk1(s2, x, V);
      // e^{ξ4θ4}e^{ξ5θ5}e^{ξ6θ6} = e^{-ξ3θ3} e^{-ξ2θ2} e^{-ξ1θ1} G, a rotation about wA
      const R = multiply(screwMat(s3, -t3, m1), multiply(screwMat(s2, -t2, m2), G1, m3), new Float64Array(16));
      for (const w of wristSolutions(s4, s5, s6, R, a.wA, seedTh)) out.push({ th: [t1, t2, t3, w[0], w[1], w[2]], singular: w[3] === 1 });
    }
  }
  return out;
}

/** θ4, θ5, θ6 of an intersecting wrist for the rotation R (about the common point c). Last entry 1 = singular. */
function wristSolutions(s4: Screw, s5: Screw, s6: Screw, R: Mat4, c: Vec3, seedTh: number[]): number[][] {
  const u = s6.w; // a direction along axis 6 (point c + u is on axis 6)
  const v = xfD(R, u);
  const w1 = s4.w, w2 = s5.w;
  const c12 = dot(w1, w2), den = c12 * c12 - 1;
  const alpha = (c12 * dot(w2, u) - dot(w1, v)) / den;
  const beta = (c12 * dot(w1, v) - dot(w2, u)) / den;
  const n = crs(w1, w2), nn = dot(n, n);
  let g2 = (dot(u, u) - alpha * alpha - beta * beta - 2 * alpha * beta * c12) / nn;
  if (g2 < 0) { if (g2 < -1e-6) return []; g2 = 0; }
  const out: number[][] = [];
  const gammas = g2 < 1e-14 ? [0] : [Math.sqrt(g2), -Math.sqrt(g2)];
  // a direction ⊥ axis 6 to recover θ6
  const perp = unit(Math.abs(u[0]) < 0.9 ? crs(u, [1, 0, 0]) : crs(u, [0, 1, 0]));
  for (const g of gammas) {
    const z = add3(add3(scl3(w1, alpha), scl3(w2, beta)), scl3(n, g));
    let t5 = pk1v(w2, u, z);
    let t4 = pk1v(w1, z, v);
    let singular = 0;
    if (Number.isNaN(t5)) t5 = 0;
    if (Number.isNaN(t4)) { t4 = seedTh[3]; singular = 1; }
    // θ6 from the remaining rotation applied to a vector ⊥ axis 6
    const lhs = rotV(w2, -t5, rotV(w1, -t4, xfD(R, perp)));
    let t6 = pk1v(u, perp, lhs);
    if (Number.isNaN(t6)) t6 = seedTh[5];
    out.push([t4, t5, t6, singular]);
  }
  return out;
}

function solveUR(a: ChainAnalysis, gt: Mat4, seedTh: number[]): RawSol[] {
  const [s1, s2, s3, s4, s5, s6] = a.screws;
  const tol = 1e-6 * a.scale;
  const G = multiply(gt, a.g0inv);
  const W56 = xfP(G, a.wB);
  const u = s2.w;
  const out: RawSol[] = [];
  const d = sub3(W56, s1.p);
  const upar = scl3(s1.w, dot(u, s1.w)), uperp = sub3(u, upar), uxw = crs(s1.w, u);
  const t1s = trig(dot(uperp, d), dot(uxw, d), dot(u, sub3(a.wB, s1.p)) - dot(upar, d), tol);
  if (!t1s.length && Math.hypot(dot(uperp, d), dot(uxw, d)) < tol) t1s.push(seedTh[0]);
  const w6t = xfD(G, s6.w); // axis 6 direction at the target
  const Ginv = invert(G);
  const sgn = (s: Screw) => (dot(s.w, u) >= 0 ? 1 : -1);
  for (const t1 of uniqAngles(t1s)) {
    const u1 = rotV(s1.w, t1, u);
    // θ5: u·R(w5,θ5) w6 = u(θ1)·w6t
    const w6par = scl3(s5.w, dot(s6.w, s5.w)), w6perp = sub3(s6.w, w6par), w5x6 = crs(s5.w, s6.w);
    const t5s = trig(dot(u, w6perp), dot(u, w5x6), dot(u1, w6t) - dot(u, w6par), 1e-9);
    for (const t5 of uniqAngles(t5s)) {
      // θ6: R6 (Gᵀ u1) = R5ᵀ u
      const sv = xfD(Ginv, u1);
      const target = rotV(s5.w, -t5, u);
      let t6 = pk1v(s6.w, sv, target);
      let singular = false;
      if (Number.isNaN(t6)) { t6 = seedTh[5]; singular = true; }
      // θ2+θ3+θ4 (signed by axis directions): R234 = e^{-ξ1θ1} G e^{-ξ6θ6} e^{-ξ5θ5}
      const R234 = multiply(multiply(multiply(screwMat(s1, -t1), G), screwMat(s6, -t6)), screwMat(s5, -t5));
      const ref = unit(Math.abs(u[0]) < 0.9 ? crs(u, [1, 0, 0]) : crs(u, [0, 1, 0]));
      const t234 = pk1v(u, ref, xfD(R234, ref));
      // position of axis4∩axis5: W45 = G e^{-ξ6θ6} wA
      const W45 = xfP(multiply(G, screwMat(s6, -t6)), a.wA);
      const V = rotP(s1, -t1, W45);
      const pk = s2.p;
      const delta = nrm3(sub3(V, pk));
      for (const t3 of uniqAngles(pk3(s3, a.wA, pk, delta, tol))) {
        const x = rotP(s3, t3, a.wA);
        const t2 = pk1(s2, x, V);
        const t4 = sgn(s4) * (t234 - sgn(s2) * t2 - sgn(s3) * t3);
        out.push({ th: [t1, t2, t3, t4, t5, t6], singular });
      }
    }
  }
  return out;
}

function uniqAngles(ts: number[]): number[] {
  const out: number[] = [];
  for (const t of ts) if (Number.isFinite(t) && !out.some((o) => Math.abs(Math.atan2(Math.sin(t - o), Math.cos(t - o))) < 1e-10)) out.push(t);
  return out;
}

// -- delta ---------------------------------------------------------------------------------------------------
function solveDelta(chain: ChainDef, flange: Mat4, seed?: number[]): AnalyticSolution[] {
  const d = chain.parallel!;
  // platform frame (after the 4th axis) = flange · F⁻¹; motor-plane coordinates = B⁻¹ · that
  let plat = multiply(flange, invert(chain.flange));
  if (chain.base) plat = multiply(invert(chain.base), plat);
  const P: Vec3 = [plat[12], plat[13], plat[14]];
  const roots: number[][] = [];
  for (let i = 0; i < 3; i++) {
    // work in the arm's motor frame: x radial, y along the motor axis, z up
    const inv = invert(deltaMotorFrame(d, i));
    // lower-arm end on the platform, expressed in the motor frame (the platform keeps its orientation)
    const ph = d.armAngles[i] * DEG;
    const E: Vec3 = [P[0] + d.platformRadius * Math.cos(ph), P[1] + d.platformRadius * Math.sin(ph), P[2]];
    const e = xfP(inv, E);
    // elbow on the circle x = L1 cos a, z = -L1 sin a (y = 0): |elbow - e| = L2
    const L1 = d.upperArm, L2 = d.lowerArm;
    const r2 = L2 * L2 - e[1] * e[1];
    if (r2 < 0) return [];
    // (L1 cos a - ex)² + (L1 sin a + ez)² = r2  ->  -2 L1 ex cos a + 2 L1 ez sin a = r2 - L1² - ex² - ez²
    const sols = uniqAngles(trig(-2 * L1 * e[0], 2 * L1 * e[2], r2 - L1 * L1 - e[0] * e[0] - e[2] * e[2], 1e-6));
    if (!sols.length) return [];
    // elbow out (elbow further from the centre) first
    sols.sort((x, y) => Math.cos(y) - Math.cos(x));
    roots.push(sols.map((t) => wrapDeg(t * RAD)));
  }
  let q4: number | undefined;
  if (chain.joints[3] && chain.joints[3].type !== 'fixed') {
    // rotation about the platform Z: R_plat = platform · Rz(±q4)
    let R = plat;
    if (d.platform) R = multiply(invert(d.platform), R);
    const ax = chain.joints[3].axis[2] < 0 ? -1 : 1;
    q4 = wrapDeg(Math.atan2(R[1], R[0]) * RAD * ax);
  }
  const out: AnalyticSolution[] = [];
  for (const a of roots[0]) for (const b of roots[1]) for (const c of roots[2]) {
    const q = [a, b, c];
    const pos = deltaPlatformPosition(d, q);
    if (!pos || Math.hypot(pos[0] - P[0], pos[1] - P[1], pos[2] - P[2]) > 1e-6) continue;
    if (q4 !== undefined) q.push(q4);
    out.push({ q, singular: false });
  }
  void seed;
  return out;
}

// -- configuration flags -----------------------------------------------------------------------------------
/**
 * RoboDK configuration flags [REAR, LOWERARM, FLIP] of a 6-axis arm (RoboDK JointsConfig), or null when the chain has
 * no closed-form structure. Verified against RoboDK 5.9.4 for the Library robots (tests/kin_ik.test.ts):
 *  - REAR: the wrist centre is behind axis 1, "front" being the X axis of link 1 at q1 = 0 (the robot's forward direction).
 *  - LOWERARM: the raw elbow value is below the stretched-arm value in the DH sense: s3·(v3 - v3s) < 0, v3 = q3 (+ coupling), v3s the
 *    value at which upper arm and forearm align (RoboDK compares raw joint values: q3 = 274.6 is not "lower" on a UR).
 *  - FLIP: joint 5, measured in the DH sense (joint sense removed), is negative.
 */
export function configFlags(chain: ChainDef, q: number[]): [number, number, number] | null {
  const a = analyzeChain(chain);
  if (a.kind !== 'spherical6' && a.kind !== 'ur6') return null;
  const th = toMotion(a, q);
  const [s1, s2, s3] = a.screws;
  const W = rotP(s1, th[0], rotP(s2, th[1], rotP(s3, th[2], a.wA)));
  const horiz = (v: Vec3): Vec3 => sub3(v, scl3(s1.w, dot(s1.w, v)));
  const f = rotV(s1.w, th[0], a.front);
  const dh = horiz(sub3(W, s1.p));
  const rear = nrm3(dh) > 1e-6 * a.scale && dot(dh, f) < 0 ? 1 : 0;
  const v3 = q[2] + (a.coupleSrc[2] >= 0 ? a.coupleMul[2] * q[a.coupleSrc[2]] : 0);
  const dhSense = (k: number) => { const ax = chain.joints[a.idx[k]].axis; return Math.abs(ax[2]) > 1e-9 && Math.abs(ax[0]) < 1e-9 && Math.abs(ax[1]) < 1e-9 ? Math.sign(ax[2]) : 1; };
  // UR-type arms compare the raw value (Omron TM12 with J3 sense -1 is "upper" at q3 = +90)
  const lower = (a.kind === 'ur6' ? 1 : dhSense(2)) * (v3 - a.elbowStretch) < -1e-9 ? 1 : 0;
  const s5 = dhSense(4);
  const flip = wrapDeg(s5 * q[4]) < 0 ? 1 : 0;
  return [rear, lower, flip];
}

/**
 * Sine of the angle between wrist axes 4 and 6 at q (0 = wrist singularity) for closed-form 6R chains; null otherwise.
 * For UR-type arms: the sine between axis 6 and the parallel axes 2-4 (0 = wrist singularity of that family).
 */
export function wristSingularity(chain: ChainDef, q: number[]): number | null {
  const a = analyzeChain(chain);
  if (a.kind !== 'spherical6' && a.kind !== 'ur6') return null;
  const th = toMotion(a, q);
  const [, s2, , s4, s5, s6] = a.screws;
  const w6 = rotV(s5.w, th[4], s6.w);
  return nrm3(crs(a.kind === 'ur6' ? s2.w : s4.w, w6));
}
