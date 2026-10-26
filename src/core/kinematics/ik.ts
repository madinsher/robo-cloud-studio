import { Mat4, poseToQuat, RAD, DEG, multiply, invert } from '../math/pose';
import { ChainDef, flangePoseInto, jacobianInto, jointLimits, actuatedIndices, homeJoints, chainInfo } from './chain';
import { solveAnalytic, analyzeChain, configFlags, wristSingularity } from './ik_analytic';
import { wrap180 } from '../units';

export interface IKOptions {
  /** Initial guess / preferred solution (deg/mm). Defaults to home. */
  seed?: number[];
  maxIterations?: number;
  /** Position tolerance (mm) and orientation tolerance (rad). */
  posTol?: number;
  rotTol?: number;
  /** Initial damping factor for DLS (relative to the Jacobian scale). */
  damping?: number;
  /** Ignore orientation (3-DOF positioning, e.g. for SCARA-like pickers). */
  positionOnly?: boolean;
  /** Ignore rotation about tool Z (5-DOF tasks: spraying, drilling, picking with symmetric gripper). */
  freeToolZ?: boolean;
  /** Try random restarts within limits if the first attempt fails. */
  restarts?: number;
  /** Enforce joint limits. */
  respectLimits?: boolean;
  /** Skip configuration-aware candidate selection (Robot.solveIK). */
  keepFirstSolution?: boolean;
  /**
   * RoboDK configuration [REAR, LOWERARM, FLIP] the solution must have (entries may be undefined = any).
   * Only honoured by closed-form chains (6R spherical wrist / UR-type).
   */
  config?: Array<number | undefined>;
  /** Use the closed-form solver when the chain allows it (default true). */
  analytic?: boolean;
}

export interface IKResult {
  ok: boolean;
  joints: number[];
  iterations: number;
  posError: number;
  rotError: number;
  /** 'analytic' (closed form) or 'numeric' (damped least squares). */
  method?: 'analytic' | 'numeric';
  /** The solution sits at / next to a singularity (wrist aligned, arm stretched, wrist on axis 1). */
  nearSingular?: boolean;
  /** Non-fatal remark (e.g. a sub-tolerance residual accepted near a singularity). */
  warning?: string;
  /**
   * The closed form proved that no solution exists (pose outside the workspace or outside the joint limits), so a
   * failure is 'unreachable', never a singularity crossing — even when the numeric residual reported is tiny.
   */
  unreachable?: boolean;
}

/** Orientation error as rotation vector (rad), from current to target, in base frame. */
function rotationError(current: Mat4, target: Mat4, out: number[]): number[] {
  // rel = target · currentᵀ (rotation parts only)
  const r = (i: number, j: number) => target[i] * current[j] + target[i + 4] * current[j + 4] + target[i + 8] * current[j + 8];
  const m00 = r(0, 0), m01 = r(0, 1), m02 = r(0, 2), m10 = r(1, 0), m11 = r(1, 1), m12 = r(1, 2), m20 = r(2, 0), m21 = r(2, 1), m22 = r(2, 2);
  const rel = new Float64Array(16);
  rel[0] = m00; rel[4] = m01; rel[8] = m02; rel[1] = m10; rel[5] = m11; rel[9] = m12; rel[2] = m20; rel[6] = m21; rel[10] = m22; rel[15] = 1;
  const [w, x, y, z] = poseToQuat(rel);
  const s = Math.hypot(x, y, z);
  if (s < 1e-14) { out[0] = out[1] = out[2] = 0; return out; }
  const angle = 2 * Math.atan2(s, w);
  const k = (angle > Math.PI ? angle - 2 * Math.PI : angle) / s;
  out[0] = x * k; out[1] = y * k; out[2] = z * k;
  return out;
}

/** Solve (JᵀJ + λ² diag(JᵀJ)+λ²·eps) dq = Jᵀ e (Levenberg–Marquardt scaling) via Gaussian elimination. */
function solveLM(J: number[][], e: number[], lambda2: number, A: number[][]): { dq: number[]; minPivot: number } {
  const rows = J.length, n = J[0].length;
  let trace = 0;
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      let s = 0;
      for (let r = 0; r < rows; r++) s += J[r][i] * J[r][j];
      A[i][j] = s;
    }
    trace += A[i][i];
    let s = 0;
    for (let r = 0; r < rows; r++) s += J[r][i] * e[r];
    A[i][n] = s;
  }
  // damping proportional to the Jacobian scale: λ² (diag + 1e-6 · mean diag) — unit-consistent (mm² per rad²)
  const floor = 1e-6 * (trace / n || 1);
  for (let i = 0; i < n; i++) A[i][i] += lambda2 * (A[i][i] + floor);
  let minPivot = Infinity;
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(A[r][c]) > Math.abs(A[p][c])) p = r;
    if (p !== c) { const t = A[c]; A[c] = A[p]; A[p] = t; }
    const piv = A[c][c];
    minPivot = Math.min(minPivot, Math.abs(piv) / (trace / n || 1));
    if (Math.abs(piv) < 1e-300) continue;
    for (let r = c + 1; r < n; r++) {
      const f = A[r][c] / piv;
      if (f === 0) continue;
      for (let k = c; k <= n; k++) A[r][k] -= f * A[c][k];
    }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let s = A[r][n];
    for (let k = r + 1; k < n; k++) s -= A[r][k] * x[k];
    x[r] = Math.abs(A[r][r]) < 1e-300 ? 0 : s / A[r][r];
  }
  return { dq: x, minPivot };
}

/** Characteristic length of a chain (mm), used to weigh rotation against translation. */
function chainScale(chain: ChainDef): number {
  let r = 0;
  for (const j of chain.joints) {
    r += Math.hypot(j.origin[12], j.origin[13], j.origin[14]);
    if (j.post) r += Math.hypot(j.post[12], j.post[13], j.post[14]);
  }
  return Math.max(100, r);
}

/**
 * Numerical inverse kinematics: Levenberg–Marquardt damped least squares with scale-consistent damping, joint-limit
 * clamping and restarts. Residuals that stay below 50× the tolerance at a (near-)singular Jacobian are accepted with
 * `nearSingular` and a warning instead of failing.
 */
export function inverseKinematics(chain: ChainDef, target: Mat4, opts: IKOptions = {}): IKResult {
  const idx = actuatedIndices(chain);
  const n = idx.length;
  const { lower, upper } = jointLimits(chain);
  const maxIt = opts.maxIterations ?? 200;
  const posTol = opts.posTol ?? 1e-3;
  const rotTol = opts.rotTol ?? 1e-5;
  const lambda0 = opts.damping ?? 1e-3;
  const respect = opts.respectLimits ?? true;
  const restarts = opts.restarts ?? 6;
  const types = idx.map((i) => chain.joints[i].type);
  const isRev = types.map((t) => t === 'revolute' || t === 'continuous');
  const wRot = chainScale(chain); // 1 rad of orientation error ~ one arm length
  const nE = opts.positionOnly ? 3 : 6;
  const A: number[][] = Array.from({ length: n }, () => new Array(n + 1).fill(0));
  const Jfull: number[][] = Array.from({ length: 6 }, () => new Array(n).fill(0));
  const J3 = Jfull.slice(0, 3); // shares the rows of Jfull
  const fk = new Float64Array(16);
  const dr = [0, 0, 0];
  const e = new Array(6).fill(0);

  const evalErr = (q: number[]): { pos: number; rot: number; cost: number } => {
    flangePoseInto(chain, q, fk);
    const dx = target[12] - fk[12], dy = target[13] - fk[13], dz = target[14] - fk[14];
    let rx = 0, ry = 0, rz = 0;
    if (!opts.positionOnly) {
      rotationError(fk, target, dr);
      rx = dr[0]; ry = dr[1]; rz = dr[2];
      if (opts.freeToolZ) {
        // remove the component of the rotation error about the target tool Z axis
        const zx = target[8], zy = target[9], zz = target[10];
        const d = rx * zx + ry * zy + rz * zz;
        rx -= d * zx; ry -= d * zy; rz -= d * zz;
      }
    }
    e[0] = dx; e[1] = dy; e[2] = dz; e[3] = rx * wRot; e[4] = ry * wRot; e[5] = rz * wRot;
    const pos = Math.hypot(dx, dy, dz), rot = Math.hypot(rx, ry, rz);
    return { pos, rot, cost: pos * pos + (rot * wRot) ** 2 };
  };

  let best: IKResult = { ok: false, joints: opts.seed ? [...opts.seed] : homeJoints(chain), iterations: 0, posError: Infinity, rotError: Infinity, method: 'numeric' };

  const attempt = (seed: number[]): IKResult => {
    const q = seed.slice(0, n);
    while (q.length < n) q.push(0);
    let err = evalErr(q);
    let lambda2 = lambda0;
    let it = 0, stall = 0, minPivot = 1;
    for (; it < maxIt; it++) {
      if (err.pos < posTol && err.rot < rotTol) return { ok: true, joints: q, iterations: it, posError: err.pos, rotError: err.rot, method: 'numeric' };
      jacobianInto(chain, q, Jfull);
      if (!opts.positionOnly) for (let r = 3; r < 6; r++) for (let k = 0; k < n; k++) Jfull[r][k] *= wRot;
      const J = opts.positionOnly ? J3 : Jfull;
      if (opts.freeToolZ && !opts.positionOnly) {
        // project the angular rows: no motion is required about the tool Z axis
        const z = [target[8], target[9], target[10]];
        for (let k = 0; k < n; k++) {
          const d = J[3][k] * z[0] + J[4][k] * z[1] + J[5][k] * z[2];
          J[3][k] -= d * z[0]; J[4][k] -= d * z[1]; J[5][k] -= d * z[2];
        }
      }
      if (respect) {
        // active set: joints sitting on a limit whose descent direction points outward do not move this iteration
        for (let k = 0; k < n; k++) {
          const span = upper[k] - lower[k];
          let g = 0;
          for (let r = 0; r < nE; r++) g += J[r][k] * e[r];
          const eps = isRev[k] ? 1e-7 : 1e-6;
          if (span < eps || (q[k] <= lower[k] + eps && g < 0) || (q[k] >= upper[k] - eps && g > 0)) for (let r = 0; r < J.length; r++) J[r][k] = 0;
        }
      }
      // keep the linearised step bounded far from the target
      const en = Math.sqrt(err.cost);
      const lim = 0.25 * wRot;
      const sc = en > lim ? lim / en : 1;
      const es = e.slice(0, nE).map((v) => v * sc);
      const sol = solveLM(J, es, lambda2, A);
      minPivot = sol.minPivot;
      // per-iteration step limit: 20° for revolute joints, 10 % of the range (>= 50 mm) for prismatic
      let sf = 1;
      for (let k = 0; k < n; k++) {
        const d = Math.abs(isRev[k] ? sol.dq[k] * RAD : sol.dq[k]);
        const lm = isRev[k] ? 20 : Math.max(50, (upper[k] - lower[k]) * 0.1);
        if (d > lm) sf = Math.min(sf, lm / d);
      }
      const qn = q.map((v, k) => {
        let x = v + (isRev[k] ? sol.dq[k] * RAD : sol.dq[k]) * sf;
        if (types[k] === 'continuous') x = wrap180(x);
        if (respect) x = Math.max(lower[k], Math.min(upper[k], x));
        return x;
      });
      const errN = evalErr(qn);
      if (errN.cost < err.cost) {
        for (let k = 0; k < n; k++) q[k] = qn[k];
        const gain = (err.cost - errN.cost) / (err.cost || 1);
        err = errN;
        lambda2 = Math.max(1e-9, lambda2 * 0.3);
        stall = gain < 1e-6 ? stall + 1 : 0;
      } else {
        lambda2 = Math.min(1e6, lambda2 * 10);
        stall++;
        evalErr(q); // restore e[] for the current q
      }
      if (stall > 25) break;
    }
    const nearSing = minPivot < 1e-6;
    if (nearSing && err.pos < posTol * 50 && err.rot < rotTol * 50)
      return { ok: true, joints: q, iterations: it, posError: err.pos, rotError: err.rot, method: 'numeric', nearSingular: true, warning: `near a singularity: residual ${err.pos.toFixed(4)} mm` };
    return { ok: false, joints: q, iterations: it, posError: err.pos, rotError: err.rot, method: 'numeric', nearSingular: nearSing };
  };

  const seed0 = opts.seed ? [...opts.seed] : homeJoints(chain);
  const seeds: number[][] = [seed0];
  // Small perturbations of the seed escape exact singular configurations (e.g. a fully stretched arm)
  // while staying in the same joint-space neighbourhood.
  for (let p = 1; p <= 2; p++) seeds.push(seed0.map((v, k) => Math.max(lower[k], Math.min(upper[k], v + (isRev[k] ? 2.5 : 5) * p * ((k % 2 ? 1 : -1) * (p % 2 ? 1 : -1))))));
  for (let r = 0; r < restarts; r++) {
    seeds.push(idx.map((_, k) => {
      const lo = Math.max(lower[k], isRev[k] ? -360 : lower[k]);
      const hi = Math.min(upper[k], isRev[k] ? 360 : upper[k]);
      // deterministic pseudo-random spread for reproducibility
      const t = ((r + 1) * 0.618033988749895 * (k + 1)) % 1;
      return lo + (hi - lo) * t;
    }));
  }
  for (const s of seeds) {
    const r = attempt(s);
    if (r.ok) return r;
    if (r.posError + r.rotError * wRot < best.posError + best.rotError * wRot) best = r;
  }
  return best;
}

/** Normalise joint solution to be closest to a reference configuration (unwrap revolute joints within limits). */
export function closestConfiguration(chain: ChainDef, q: number[], ref: number[]): number[] {
  const idx = chainInfo(chain).idx;
  return q.map((v, k) => {
    const j = chain.joints[idx[k]];
    if (j.type !== 'revolute' && j.type !== 'continuous') return v;
    const r = ref[k] ?? 0;
    // nearest representative v + 360 n to the reference, then the closest one inside the limits
    const n0 = Math.round((r - v) / 360);
    let best = v, bd = Infinity;
    for (let dn = -2; dn <= 2; dn++) {
      const c = v + 360 * (n0 + dn);
      if (c < j.lower - 1e-9 || c > j.upper + 1e-9) continue;
      const d = Math.abs(c - r);
      if (d < bd) { bd = d; best = c; }
    }
    return best;
  });
}

// -- closed-form front end ------------------------------------------------------------------------------------

/** True when the chain has a closed-form solver (6R spherical wrist, UR-type, delta). */
export function hasAnalyticIK(chain: ChainDef): boolean {
  return analyzeChain(chain).kind !== null;
}

const RDK_TYPE_COMAU = 11;

/** Joint-limit test used for solution sets (RoboDK: Comau J3 limits also bound the parallelogram angle q2 + q3). */
function inLimits(chain: ChainDef, q: number[], lower: number[], upper: number[], strictMask?: boolean[]): boolean {
  const e = 1e-6;
  for (let i = 0; i < q.length; i++) {
    const strict = strictMask?.[i];
    const ok = (x: number) => (strict ? x > lower[i] + e && x < upper[i] - e : x >= lower[i] - e && x <= upper[i] + e);
    if (!ok(q[i])) return false;
    if (i === 2 && Math.round(chain.jointsType ?? -1) === RDK_TYPE_COMAU && q.length > 2 && !ok(q[2] + q[1])) return false;
  }
  return true;
}

/**
 * All inverse-kinematics solutions of a flange pose (relative to the robot base frame), inside the joint limits,
 * in a stable order: base solutions sorted by configuration [REAR, LOWERARM, FLIP], then turn variants ascending.
 * `turns: 'rdk'` (default) also lists the ±360° variants of every joint like RoboDK `SolveIK_All` (one turn each way,
 * strictly inside the limits); 'none' keeps one representative per configuration (the one closest to the seed).
 * Chains without a closed form return the distinct numeric solutions found from spread seeds.
 */
export function ikAll(chain: ChainDef, flange: Mat4, opts: { seed?: number[]; turns?: 'rdk' | 'none' } = {}): number[][] {
  const { lower, upper } = jointLimits(chain);
  const sols = solveAnalytic(chain, flange, opts.seed);
  if (sols === null) return numericAll(chain, flange, opts.seed);
  const n = lower.length;
  const keyed = sols.map((s) => ({ q: s.q, cfg: configFlags(chain, s.q) ?? [0, 0, 0] }));
  keyed.sort((a, b) => (a.cfg[0] - b.cfg[0]) || (a.cfg[1] - b.cfg[1]) || (a.cfg[2] - b.cfg[2]) || a.q.reduce((acc, v, i) => acc || Math.sign(v - b.q[i]), 0));
  const out: number[][] = [];
  const revolute = actuatedIndices(chain).map((i) => chain.joints[i].type !== 'prismatic');
  for (const { q } of keyed) {
    if (opts.turns === 'none') {
      const c = opts.seed ? closestConfiguration(chain, q, opts.seed) : q;
      if (inLimits(chain, c, lower, upper)) out.push(c);
      continue;
    }
    const rec = (i: number, cur: number[], strict: boolean[]) => {
      if (i === n) { if (inLimits(chain, cur, lower, upper, strict)) out.push(cur.slice()); return; }
      const ks = revolute[i] ? [-1, 0, 1] : [0];
      for (const k of ks) { cur.push(q[i] + 360 * k); strict.push(k !== 0); rec(i + 1, cur, strict); cur.pop(); strict.pop(); }
    };
    rec(0, [], []);
  }
  return out;
}

function numericAll(chain: ChainDef, flange: Mat4, seed?: number[], attempts = 16): number[][] {
  const { lower, upper } = jointLimits(chain);
  const sols: number[][] = [];
  for (let a = 0; a < attempts; a++) {
    const s = a === 0 && seed ? seed : lower.map((lo, k) => { const l = Math.max(lo, -360), h = Math.min(upper[k], 360); return l + (h - l) * ((((a * 7919) % 104729) / 104729 + k * 0.37) % 1); });
    const r = inverseKinematics(chain, flange, { seed: s, restarts: 0, ...(lower.length <= 3 ? { positionOnly: true } : lower.length <= 5 ? { freeToolZ: true } : {}) });
    if (!r.ok) continue;
    if (!sols.some((x) => x.every((v, i) => Math.abs(v - r.joints[i]) < 1e-3))) sols.push(r.joints);
  }
  return sols;
}

/** Weighted joint distance used to pick the solution closest to a seed. */
export function jointDistance(a: number[], b: number[]): number {
  let s = 0;
  for (let i = 0; i < a.length; i++) { const d = a[i] - (b[i] ?? 0); s += d * d; }
  return s;
}

/**
 * IK for a flange pose: the closed-form solution closest to `seed` (honouring `config` flags) when the chain has one,
 * otherwise the numeric solver. With a seed, consecutive calls track one continuous branch (MoveL): the representative
 * of every joint nearest to the seed is used, inside the limits.
 */
export function solveIKBest(chain: ChainDef, flange: Mat4, opts: IKOptions = {}): IKResult {
  const seed = opts.seed ?? homeJoints(chain);
  if (opts.analytic !== false && !opts.positionOnly && !opts.freeToolZ) {
    const sols = solveAnalytic(chain, flange, seed);
    if (sols !== null) {
      const { lower, upper } = jointLimits(chain);
      let best: number[] | null = null, bd = Infinity, sing = false;
      for (const s of sols) {
        const q = pickRepresentative(chain, s.q, seed, lower, upper, opts.config, opts.respectLimits !== false);
        if (!q) continue;
        const d = jointDistance(q, seed);
        if (d < bd) { bd = d; best = q; sing = s.singular; }
      }
      if (best) {
        const chk = flangePoseInto(chain, best, new Float64Array(16));
        const pe = Math.hypot(chk[12] - flange[12], chk[13] - flange[13], chk[14] - flange[14]);
        const tr = (chk[0] * flange[0] + chk[1] * flange[1] + chk[2] * flange[2] + chk[4] * flange[4] + chk[5] * flange[5] + chk[6] * flange[6] + chk[8] * flange[8] + chk[9] * flange[9] + chk[10] * flange[10]);
        const re = Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2)));
        return { ok: true, joints: best, iterations: 0, posError: pe, rotError: re, method: 'analytic', nearSingular: sing || nearWristSingular(chain, best) };
      }
      // closed form says unreachable (or not with these flags / limits): report the residual of the numeric attempt
      if (!opts.config) {
        const r = inverseKinematics(chain, flange, { ...opts, seed, restarts: 0, maxIterations: Math.min(opts.maxIterations ?? 100, 100) });
        if (r.ok) return r;
        return { ...r, ok: false, unreachable: true, warning: sols.length ? 'outside the joint limits' : 'outside the workspace' };
      }
      return { ok: false, joints: [...seed], iterations: 0, posError: Infinity, rotError: Infinity, method: 'analytic', unreachable: true, warning: 'no solution with the requested configuration' };
    }
  }
  return inverseKinematics(chain, flange, { ...opts, seed });
}

/** Representatives v + 360k (k = -2..2) of every revolute joint inside the limits. */
function representatives(chain: ChainDef, q: number[], lower: number[], upper: number[]): number[][] {
  const idx = chainInfo(chain).idx;
  return q.map((v, i) => {
    if (chain.joints[idx[i]].type === 'prismatic') return v >= lower[i] - 1e-6 && v <= upper[i] + 1e-6 ? [v] : [];
    const out: number[] = [];
    for (let k = -2; k <= 2; k++) { const x = v + 360 * k; if (x >= lower[i] - 1e-6 && x <= upper[i] + 1e-6) out.push(x); }
    return out;
  });
}

/**
 * The joint vector equivalent to q (±360° turns) that is closest to the seed, inside the limits and — when requested —
 * with the given RoboDK configuration. LOWERARM depends on the raw values of joints 2/3, so their turn combinations are
 * searched when the closest choice does not satisfy the flags or the J2/J3 interaction limit.
 */
function pickRepresentative(chain: ChainDef, q: number[], seed: number[], lower: number[], upper: number[], config: IKOptions['config'], limits: boolean): number[] | null {
  const lists = limits ? representatives(chain, q, lower, upper) : q.map((v) => [v]);
  if (lists.some((l) => !l.length)) return null;
  const nearest = (l: number[], r: number) => l.reduce((b, x) => (Math.abs(x - r) < Math.abs(b - r) ? x : b), l[0]);
  const base = lists.map((l, i) => nearest(l, seed[i] ?? 0));
  const okCfg = (c: number[]) => {
    if (!config) return true;
    const f = configFlags(chain, c);
    return !f || !config.some((v, i) => v !== undefined && v !== null && i < 3 && f[i] !== Number(v));
  };
  const valid = (c: number[]) => (!limits || inLimits(chain, c, lower, upper)) && okCfg(c);
  if (valid(base)) return base;
  if (q.length < 3) return null;
  let best: number[] | null = null, bd = Infinity;
  for (const a of lists[1]) for (const b of lists[2]) {
    const c = base.slice(); c[1] = a; c[2] = b;
    if (!valid(c)) continue;
    const d = jointDistance(c, seed);
    if (d < bd) { bd = d; best = c; }
  }
  return best;
}

/** |sin θ5| small: the wrist axes 4 and 6 are (almost) aligned. */
function nearWristSingular(chain: ChainDef, q: number[]): boolean {
  const s = wristSingularity(chain, q);
  return s !== null && s < Math.sin(1 * DEG);
}

export { DEG, invert, multiply };
