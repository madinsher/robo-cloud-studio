import { Mat4, getPos, sub, poseToQuat, RAD, DEG, multiply, invert } from '../math/pose';
import { ChainDef, forwardKinematics, jacobian, jointLimits, actuatedIndices, homeJoints } from './chain';
import { wrap180 } from '../units';

export interface IKOptions {
  /** Initial guess (deg/mm). Defaults to home. */
  seed?: number[];
  maxIterations?: number;
  /** Position tolerance (mm) and orientation tolerance (rad). */
  posTol?: number;
  rotTol?: number;
  /** Damping factor for DLS. */
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
}

export interface IKResult {
  ok: boolean;
  joints: number[];
  iterations: number;
  posError: number;
  rotError: number;
}

/** Orientation error as rotation vector (rad), from current to target, in base frame. */
function rotationError(current: Mat4, target: Mat4): [number, number, number] {
  const rel = multiply(target, invert(current)); // target * inv(current)
  const [w, x, y, z] = poseToQuat(rel);
  const s = Math.hypot(x, y, z);
  if (s < 1e-12) return [0, 0, 0];
  const angle = 2 * Math.atan2(s, w);
  const k = angle / s;
  return [x * k, y * k, z * k];
}

/** Solve linear system (JᵀJ + λ²I) dq = Jᵀ e via Gaussian elimination. */
function solveDLS(J: number[][], e: number[], lambda: number): number[] {
  const rows = J.length, n = J[0].length;
  const A: number[][] = Array.from({ length: n }, () => new Array(n + 1).fill(0));
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      let s = 0;
      for (let r = 0; r < rows; r++) s += J[r][i] * J[r][j];
      A[i][j] = s + (i === j ? lambda * lambda : 0);
    }
    let s = 0;
    for (let r = 0; r < rows; r++) s += J[r][i] * e[r];
    A[i][n] = s;
  }
  // Gaussian elimination with partial pivoting
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(A[r][c]) > Math.abs(A[p][c])) p = r;
    [A[c], A[p]] = [A[p], A[c]];
    const piv = A[c][c];
    if (Math.abs(piv) < 1e-14) continue;
    for (let r = c + 1; r < n; r++) {
      const f = A[r][c] / piv;
      for (let k = c; k <= n; k++) A[r][k] -= f * A[c][k];
    }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let s = A[r][n];
    for (let k = r + 1; k < n; k++) s -= A[r][k] * x[k];
    x[r] = Math.abs(A[r][r]) < 1e-14 ? 0 : s / A[r][r];
  }
  return x;
}

/** Numerical inverse kinematics (damped least squares with limit clamping and restarts). */
export function inverseKinematics(chain: ChainDef, target: Mat4, opts: IKOptions = {}): IKResult {
  const idx = actuatedIndices(chain);
  const n = idx.length;
  const { lower, upper } = jointLimits(chain);
  const maxIt = opts.maxIterations ?? 200;
  const posTol = opts.posTol ?? 1e-3;
  const rotTol = opts.rotTol ?? 1e-5;
  const lambda0 = opts.damping ?? 0.5;
  const respect = opts.respectLimits ?? true;
  const restarts = opts.restarts ?? 6;
  const types = idx.map((i) => chain.joints[i].type);
  const isRev = types.map((t) => t === 'revolute' || t === 'continuous');

  let best: IKResult = { ok: false, joints: opts.seed ? [...opts.seed] : homeJoints(chain), iterations: 0, posError: Infinity, rotError: Infinity };

  const attempt = (seed: number[]): IKResult => {
    let q = seed.slice(0, n);
    while (q.length < n) q.push(0);
    let it = 0;
    let posErr = Infinity, rotErr = Infinity;
    let bestErr = Infinity, stall = 0;
    for (; it < maxIt; it++) {
      const fk = forwardKinematics(chain, q);
      const dp = sub(getPos(target), getPos(fk.flange));
      let dr = opts.positionOnly ? [0, 0, 0] : rotationError(fk.flange, target);
      if (opts.freeToolZ && !opts.positionOnly) {
        // remove component of rotation error about the target tool Z axis
        const zx = target[8], zy = target[9], zz = target[10];
        const d = dr[0] * zx + dr[1] * zy + dr[2] * zz;
        dr = [dr[0] - d * zx, dr[1] - d * zy, dr[2] - d * zz];
      }
      posErr = Math.hypot(dp[0], dp[1], dp[2]);
      rotErr = Math.hypot(dr[0], dr[1], dr[2]);
      if (posErr < posTol && rotErr < rotTol) return { ok: true, joints: q, iterations: it, posError: posErr, rotError: rotErr };
      // stagnation detection: give up early when the error stops improving (unreachable target)
      const errNow = posErr + rotErr * 500;
      if (errNow < bestErr - 1e-4) { bestErr = errNow; stall = 0; } else if (++stall > 12) break;

      // Scale: position error in mm, rotation in rad. Weight rotation so 1 rad ~ 500 mm.
      const wRot = 500;
      const e = opts.positionOnly ? [dp[0], dp[1], dp[2]] : [dp[0], dp[1], dp[2], dr[0] * wRot, dr[1] * wRot, dr[2] * wRot];
      let J = jacobian(chain, q);
      if (opts.positionOnly) J = J.slice(0, 3);
      else J = J.map((row, r) => (r >= 3 ? row.map((v) => v * wRot) : row));
      // Step limiting: clamp error to avoid huge jumps far from target
      const maxStep = 250; // mm-equivalent
      const en = Math.hypot(...e);
      const eScaled = en > maxStep ? e.map((v) => (v * maxStep) / en) : e;
      const lambda = lambda0 * (1 + Math.min(en, 1000) / 200);
      const dq = solveDLS(J, eScaled, lambda); // rad or mm
      // per-iteration step limit: 20 deg for revolute joints, 10% of the range (>= 50 mm) for prismatic
      let sf = 1;
      for (let k = 0; k < n; k++) {
        const d = Math.abs(isRev[k] ? dq[k] * RAD : dq[k]);
        const lim = isRev[k] ? 20 : Math.max(50, (upper[k] - lower[k]) * 0.1);
        if (d > lim) sf = Math.min(sf, lim / d);
      }
      for (let k = 0; k < n; k++) {
        let v = q[k] + (isRev[k] ? dq[k] * RAD : dq[k]) * sf;
        if (isRev[k] && types[k] === 'continuous') v = wrap180(v);
        if (respect) v = Math.max(lower[k], Math.min(upper[k], v));
        q[k] = v;
      }
    }
    return { ok: false, joints: q, iterations: it, posError: posErr, rotError: rotErr };
  };

  const seed0 = opts.seed ? [...opts.seed] : homeJoints(chain);
  const seeds: number[][] = [seed0];
  // Small perturbations of the seed escape exact singular configurations (e.g. a fully stretched arm)
  // while staying in the same joint-space neighbourhood.
  for (let p = 1; p <= 2; p++) seeds.push(seed0.map((v, k) => Math.max(lower[k], Math.min(upper[k], v + (isRev[k] ? 2.5 : 5) * p * ((k % 2 ? 1 : -1) * (p % 2 ? 1 : -1))))));
  for (let r = 0; r < restarts; r++) {
    seeds.push(idx.map((i, k) => {
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
    if (r.posError + r.rotError * 500 < best.posError + best.rotError * 500) best = r;
  }
  return best;
}

/** Normalise joint solution to be closest to a reference configuration (unwrap continuous joints). */
export function closestConfiguration(chain: ChainDef, q: number[], ref: number[]): number[] {
  const idx = actuatedIndices(chain);
  return q.map((v, k) => {
    const j = chain.joints[idx[k]];
    if (j.type !== 'revolute' && j.type !== 'continuous') return v;
    let best = v;
    for (const cand of [v - 360, v, v + 360]) {
      if (cand < j.lower || cand > j.upper) continue;
      if (Math.abs(cand - ref[k]) < Math.abs(best - ref[k])) best = cand;
    }
    return best;
  });
}

export { DEG };
