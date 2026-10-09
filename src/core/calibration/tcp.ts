/**
 * Tool (TCP) calibration — RoboDK `CalibrateTool`.
 * Given N flange poses that touch the same fixed point with different orientations, find the TCP offset p
 * (flange frame) minimising the spread of Ti·p. Linear least squares:  (Ri - Rj) p = tj - ti  for all pairs.
 * Optionally estimates the tool Z axis from poses that share the same point along a line (algorithm 2, RoboDK
 * "CALIBRATE_TCP_BY_POINT" vs "CALIBRATE_TCP_BY_LINE" approximated).
 */
import { Mat4, transformPoint, getPos } from '../math/pose';

export interface TcpCalibrationResult {
  /** TCP offset in the flange frame (mm). */
  tcp: [number, number, number];
  /** Mean / max error of the touched point (mm). */
  meanError: number;
  maxError: number;
  /** Estimated fixed point in the robot base frame. */
  point: [number, number, number];
  /** Per-pose residuals. */
  errors: number[];
}

function solveLSQ(A: number[][], b: number[]): number[] {
  // normal equations 3x3
  const n = 3;
  const AtA: number[][] = Array.from({ length: n }, () => new Array(n).fill(0));
  const Atb = new Array(n).fill(0);
  for (let r = 0; r < A.length; r++) {
    for (let i = 0; i < n; i++) {
      Atb[i] += A[r][i] * b[r];
      for (let j = 0; j < n; j++) AtA[i][j] += A[r][i] * A[r][j];
    }
  }
  // Gaussian elimination
  const M = AtA.map((row, i) => [...row, Atb[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    if (Math.abs(M[c][c]) < 1e-12) continue;
    for (let r = 0; r < n; r++) if (r !== c) { const f = M[r][c] / M[c][c]; for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k]; }
  }
  return M.map((row, i) => (Math.abs(row[i]) < 1e-12 ? 0 : row[n] / row[i]));
}

/** Calibrate a TCP from flange poses (robot base frame) touching one point. Needs >= 3 poses with distinct orientations. */
export function calibrateTcpByPoint(flangePoses: Mat4[]): TcpCalibrationResult {
  if (flangePoses.length < 3) throw new Error('At least 3 poses are required');
  const A: number[][] = [];
  const b: number[] = [];
  for (let i = 0; i < flangePoses.length; i++) {
    for (let j = i + 1; j < flangePoses.length; j++) {
      const Ti = flangePoses[i], Tj = flangePoses[j];
      for (let r = 0; r < 3; r++) {
        // (Ri - Rj) p = tj - ti
        A.push([Ti[r] - Tj[r], Ti[4 + r] - Tj[4 + r], Ti[8 + r] - Tj[8 + r]]);
        b.push(Tj[12 + r] - Ti[12 + r]);
      }
    }
  }
  const p = solveLSQ(A, b) as [number, number, number];
  const pts = flangePoses.map((T) => transformPoint(T, p));
  const c: [number, number, number] = [0, 0, 0];
  for (const q of pts) { c[0] += q[0] / pts.length; c[1] += q[1] / pts.length; c[2] += q[2] / pts.length; }
  const errors = pts.map((q) => Math.hypot(q[0] - c[0], q[1] - c[1], q[2] - c[2]));
  return { tcp: p, meanError: errors.reduce((a, v) => a + v, 0) / errors.length, maxError: Math.max(...errors), point: c, errors };
}

/**
 * Calibrate TCP position and tool Z direction: the first N poses touch a point (position), the remaining poses
 * touch points along the tool axis direction (a straight line through the TCP, e.g. a pin held at two heights).
 */
export function calibrateTcpByLine(pointPoses: Mat4[], linePoses: Mat4[]): TcpCalibrationResult & { zAxis: [number, number, number] } {
  const base = calibrateTcpByPoint(pointPoses);
  // points on the tool axis expressed in the flange frame (using the first line pose as reference)
  const dirs: [number, number, number][] = [];
  for (const T of linePoses) {
    // the touched line point in base = the reference point c; express in this flange frame
    const inv = invert3(T);
    const local = transformPoint(inv, base.point);
    const d: [number, number, number] = [local[0] - base.tcp[0], local[1] - base.tcp[1], local[2] - base.tcp[2]];
    const n = Math.hypot(d[0], d[1], d[2]);
    if (n > 1e-6) dirs.push([d[0] / n, d[1] / n, d[2] / n]);
  }
  const z: [number, number, number] = dirs.length ? [dirs.reduce((a, d) => a + d[0], 0), dirs.reduce((a, d) => a + d[1], 0), dirs.reduce((a, d) => a + d[2], 0)] : [0, 0, 1];
  const n = Math.hypot(z[0], z[1], z[2]) || 1;
  return { ...base, zAxis: [z[0] / n, z[1] / n, z[2] / n] };
}

function invert3(m: Mat4): Mat4 {
  const out = new Float64Array(16);
  out[0] = m[0]; out[1] = m[4]; out[2] = m[8]; out[4] = m[1]; out[5] = m[5]; out[6] = m[9]; out[8] = m[2]; out[9] = m[6]; out[10] = m[10];
  const t = getPos(m);
  out[12] = -(out[0] * t[0] + out[4] * t[1] + out[8] * t[2]);
  out[13] = -(out[1] * t[0] + out[5] * t[1] + out[9] * t[2]);
  out[14] = -(out[2] * t[0] + out[6] * t[1] + out[10] * t[2]);
  out[15] = 1;
  return out;
}
