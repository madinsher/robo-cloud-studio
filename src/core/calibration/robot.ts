/**
 * Robot calibration — RoboDK `Calibrate_Robot` (simplified, numeric).
 * Identifies kinematic parameter corrections (joint offsets, link lengths in DH tables, base/tool offsets)
 * from measured TCP positions (e.g. laser tracker) at known joint configurations, using damped Gauss-Newton
 * with finite-difference Jacobians on the chain's DH parameters. Also provides the accuracy statistics RoboDK
 * reports (mean / max error before and after).
 */
import { ChainDef, chainFromDH, DHParams, flangePose } from '../kinematics/chain';
import { Mat4, getPos, multiply, transl } from '../math/pose';
import { Robot } from '../items/robot';

export interface Measurement {
  joints: number[];
  /** Measured TCP position in the robot base frame (mm). */
  xyz: [number, number, number];
}

export interface RobotCalibrationResult {
  dh: DHParams[];
  /** Position error statistics before and after (mm). */
  before: { mean: number; max: number };
  after: { mean: number; max: number };
  iterations: number;
  /** Which parameters were identified. */
  parameters: string[];
}

export interface CalibrationOptions {
  /** Identify joint offsets (theta) — always recommended. */
  jointOffsets?: boolean;
  /** Identify link lengths a and d. */
  linkLengths?: boolean;
  /** Identify alpha twists. */
  twists?: boolean;
  /** Tool offset in flange frame (mm) used for the measurement (tracker target). */
  tool?: [number, number, number];
  iterations?: number;
}

function errors(chain: ChainDef, ms: Measurement[], tool: Mat4): number[] {
  return ms.map((m) => { const p = getPos(multiply(flangePose(chain, m.joints), tool)); return Math.hypot(p[0] - m.xyz[0], p[1] - m.xyz[1], p[2] - m.xyz[2]); });
}
function stats(e: number[]) { return { mean: e.reduce((a, v) => a + v, 0) / Math.max(1, e.length), max: Math.max(0, ...e) }; }

export function calibrateRobot(robot: Robot, measurements: Measurement[], opts: CalibrationOptions = {}): RobotCalibrationResult {
  if (!robot.chain.dh) throw new Error('Robot calibration requires a DH-defined robot (library or DH import)');
  if (measurements.length < 10) throw new Error('At least 10 measurements are recommended');
  const dh0 = robot.chain.dh.map((d) => ({ ...d }));
  const tool = transl(...(opts.tool ?? [0, 0, 0]));
  const keys: Array<{ j: number; k: keyof DHParams; name: string }> = [];
  dh0.forEach((_, j) => {
    if (opts.jointOffsets !== false) keys.push({ j, k: 'theta', name: `theta${j + 1}` });
    if (opts.linkLengths) { keys.push({ j, k: 'a', name: `a${j + 1}` }); keys.push({ j, k: 'd', name: `d${j + 1}` }); }
    if (opts.twists) keys.push({ j, k: 'alpha', name: `alpha${j + 1}` });
  });
  const build = (dh: DHParams[]) => chainFromDH(robot.chain.name, dh, robot.chain.flange);
  const residual = (dh: DHParams[]) => {
    const c = build(dh);
    const r: number[] = [];
    for (const m of measurements) { const p = getPos(multiply(flangePose(c, m.joints), tool)); r.push(p[0] - m.xyz[0], p[1] - m.xyz[1], p[2] - m.xyz[2]); }
    return r;
  };
  const before = stats(errors(build(dh0), measurements, tool));
  let dh = dh0.map((d) => ({ ...d }));
  const nP = keys.length;
  let lambda = 1e-3;
  let it = 0;
  const maxIt = opts.iterations ?? 30;
  let r = residual(dh);
  let cost = r.reduce((a, v) => a + v * v, 0);
  for (; it < maxIt; it++) {
    // Jacobian by finite differences
    const J: number[][] = r.map(() => new Array(nP).fill(0));
    keys.forEach((key, p) => {
      const h = key.k === 'theta' || key.k === 'alpha' ? 1e-3 : 1e-2;
      const dh2 = dh.map((d) => ({ ...d }));
      (dh2[key.j] as any)[key.k] += h;
      const r2 = residual(dh2);
      for (let i = 0; i < r.length; i++) J[i][p] = (r2[i] - r[i]) / h;
    });
    // (JᵀJ + λ diag) δ = -Jᵀ r
    const A: number[][] = Array.from({ length: nP }, () => new Array(nP + 1).fill(0));
    for (let i = 0; i < nP; i++) {
      for (let j = 0; j < nP; j++) { let s = 0; for (let k = 0; k < r.length; k++) s += J[k][i] * J[k][j]; A[i][j] = s; }
      let s = 0; for (let k = 0; k < r.length; k++) s += J[k][i] * r[k]; A[i][nP] = -s;
      A[i][i] *= 1 + lambda;
      A[i][i] += 1e-9;
    }
    for (let c = 0; c < nP; c++) { let p = c; for (let q = c + 1; q < nP; q++) if (Math.abs(A[q][c]) > Math.abs(A[p][c])) p = q; [A[c], A[p]] = [A[p], A[c]]; if (Math.abs(A[c][c]) < 1e-14) continue; for (let q = 0; q < nP; q++) if (q !== c) { const f = A[q][c] / A[c][c]; for (let k = c; k <= nP; k++) A[q][k] -= f * A[c][k]; } }
    const delta = A.map((row, i) => (Math.abs(row[i]) < 1e-14 ? 0 : row[nP] / row[i]));
    const dhNew = dh.map((d) => ({ ...d }));
    keys.forEach((key, p) => { (dhNew[key.j] as any)[key.k] += delta[p]; });
    const rNew = residual(dhNew);
    const costNew = rNew.reduce((a, v) => a + v * v, 0);
    if (costNew < cost) { dh = dhNew; r = rNew; if (cost - costNew < 1e-9 * cost) { cost = costNew; it++; break; } cost = costNew; lambda = Math.max(1e-6, lambda / 3); }
    else { lambda *= 5; if (lambda > 1e6) break; }
  }
  const after = stats(errors(build(dh), measurements, tool));
  return { dh, before, after, iterations: it, parameters: keys.map((k) => k.name) };
}

/** Apply a calibration result to a robot (replaces its chain). */
export function applyCalibration(robot: Robot, res: RobotCalibrationResult): void {
  const q = robot.joints();
  robot.chain = chainFromDH(robot.chain.name, res.dh, robot.chain.flange);
  robot.setJoints(q);
  robot.params.calibrated = true;
  robot.params.calibrationAccuracy = res.after;
  robot.notify('chain');
}

/** Simulate measurements of a "real" robot (chain with perturbed parameters) for demos/tests. */
export function simulateMeasurements(nominal: Robot, realDh: DHParams[], joints: number[][], noiseMm = 0.02, tool: [number, number, number] = [0, 0, 0]): Measurement[] {
  const real = chainFromDH(nominal.chain.name, realDh, nominal.chain.flange);
  let seed = 7;
  const rnd = () => { seed = (seed * 1103515245 + 12345) & 0x7fffffff; return seed / 0x7fffffff - 0.5; };
  return joints.map((q) => { const p = getPos(multiply(flangePose(real, q), transl(...tool))); return { joints: q, xyz: [p[0] + rnd() * noiseMm, p[1] + rnd() * noiseMm, p[2] + rnd() * noiseMm] }; });
}
