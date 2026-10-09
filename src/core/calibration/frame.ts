/**
 * Reference frame calibration — RoboDK `Calibrate_Reference`.
 * Methods: 3 points (P1 origin, P2 on +X, P3 on XY plane), 3 points (P1 on X axis variant), 6 points (3 on the
 * XY plane, 2 on the XZ... simplified to plane + line fit), turntable (circle fit -> rotation axis frame).
 */
import { Mat4, Vec3, identity, normalize, sub, cross, dot, scale, add, norm, poseFromZ } from '../math/pose';

export const CALIBRATE_FRAME_3P_P1_ON_X = 0;
export const CALIBRATE_FRAME_3P_P1_ORIGIN = 1;
export const CALIBRATE_FRAME_6P = 2;
export const CALIBRATE_TURNTABLE = 3;
export const CALIBRATE_TURNTABLE_2X = 4;

export interface FrameCalibrationResult {
  pose: Mat4;
  /** Residual errors (mm) where applicable. */
  errors: number[];
  /** For turntables: axis direction and circle radius. */
  axis?: Vec3;
  radius?: number;
}

function frameFromAxes(origin: Vec3, x: Vec3, y: Vec3, z: Vec3): Mat4 {
  const m = identity();
  m[0] = x[0]; m[1] = x[1]; m[2] = x[2];
  m[4] = y[0]; m[5] = y[1]; m[6] = y[2];
  m[8] = z[0]; m[9] = z[1]; m[10] = z[2];
  m[12] = origin[0]; m[13] = origin[1]; m[14] = origin[2];
  return m;
}

/** Points are in the robot base frame (mm). */
export function calibrateFrame(points: Vec3[], method = CALIBRATE_FRAME_3P_P1_ORIGIN): FrameCalibrationResult {
  if (method === CALIBRATE_FRAME_3P_P1_ORIGIN || method === CALIBRATE_FRAME_3P_P1_ON_X) {
    if (points.length < 3) throw new Error('3 points required');
    const [p1, p2, p3] = points;
    const x = normalize(sub(p2, p1));
    const v = sub(p3, p1);
    const z = normalize(cross(x, v));
    const y = cross(z, x);
    if (method === CALIBRATE_FRAME_3P_P1_ORIGIN) return { pose: frameFromAxes(p1, x, y, z), errors: [0, 0, 0] };
    // P1 on X axis, P2 on X axis, P3 on Y axis: origin = projection of P3 onto the P1-P2 line
    const t = dot(sub(p3, p1), x);
    const origin = add(p1, scale(x, t));
    const y2 = normalize(sub(p3, origin));
    const z2 = cross(x, y2);
    return { pose: frameFromAxes(origin, x, y2, z2), errors: [0, 0, 0] };
  }
  if (method === CALIBRATE_FRAME_6P) {
    if (points.length < 6) throw new Error('6 points required: 3 on XY plane, 2 on the X axis (XZ plane), 1 on the Y axis');
    const plane = fitPlane(points.slice(0, 3));
    const z = plane.normal;
    const a = project(points[3], plane), b = project(points[4], plane);
    const x = normalize(sub(b, a));
    const y = cross(z, x);
    const py = project(points[5], plane);
    // origin: intersection of X line (through a) with the perpendicular through py
    const t = dot(sub(py, a), x);
    const origin = add(a, scale(x, t));
    const errors = points.slice(0, 3).map((p) => Math.abs(dot(sub(p, plane.point), z)));
    return { pose: frameFromAxes(origin, x, y, z), errors };
  }
  // Turntable: points measured on the turntable at different angles -> circle fit
  if (points.length < 3) throw new Error('At least 3 points on the turntable are required');
  const plane = fitPlane(points);
  const z = plane.normal;
  // 2D circle fit in the plane basis
  const u = normalize(sub(points[0], plane.point));
  const w = cross(z, u);
  const uv = points.map((p) => { const d = sub(p, plane.point); return [dot(d, u), dot(d, w)]; });
  // algebraic fit: x² + y² + D x + E y + F = 0
  let Sxx = 0, Syy = 0, Sxy = 0, Sx = 0, Sy = 0, Sxr = 0, Syr = 0, Sr = 0, n = uv.length;
  for (const [x, y] of uv) { const r = x * x + y * y; Sxx += x * x; Syy += y * y; Sxy += x * y; Sx += x; Sy += y; Sxr += x * r; Syr += y * r; Sr += r; }
  const M = [[Sxx, Sxy, Sx, -Sxr], [Sxy, Syy, Sy, -Syr], [Sx, Sy, n, -Sr]];
  for (let c = 0; c < 3; c++) { let p = c; for (let r = c + 1; r < 3; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r; [M[c], M[p]] = [M[p], M[c]]; for (let r = 0; r < 3; r++) if (r !== c && Math.abs(M[c][c]) > 1e-12) { const f = M[r][c] / M[c][c]; for (let k = c; k < 4; k++) M[r][k] -= f * M[c][k]; } }
  const D = M[0][3] / M[0][0], E = M[1][3] / M[1][1], F = M[2][3] / M[2][2];
  const cx = -D / 2, cy = -E / 2;
  const radius = Math.sqrt(Math.max(0, cx * cx + cy * cy - F));
  const center = add(plane.point, add(scale(u, cx), scale(w, cy)));
  const errors = uv.map(([x, y]) => Math.abs(Math.hypot(x - cx, y - cy) - radius));
  const pose = poseFromZ(center, z, sub(points[0], center));
  return { pose, errors, axis: z, radius };
}

function fitPlane(points: Vec3[]): { point: Vec3; normal: Vec3 } {
  const c: Vec3 = [0, 0, 0];
  for (const p of points) { c[0] += p[0] / points.length; c[1] += p[1] / points.length; c[2] += p[2] / points.length; }
  if (points.length === 3) return { point: c, normal: normalize(cross(sub(points[1], points[0]), sub(points[2], points[0]))) };
  // covariance -> smallest eigenvector via power iteration on the inverse (use cross products of pairs as approximation)
  let nrm: Vec3 = [0, 0, 0];
  for (let i = 0; i < points.length; i++) for (let j = i + 1; j < points.length; j++) for (let k = j + 1; k < points.length; k++) {
    const cr = cross(sub(points[j], points[i]), sub(points[k], points[i]));
    if (dot(cr, nrm) < 0) nrm = add(nrm, scale(cr, -1)); else nrm = add(nrm, cr);
  }
  return { point: c, normal: norm(nrm) > 1e-9 ? normalize(nrm) : [0, 0, 1] };
}
function project(p: Vec3, plane: { point: Vec3; normal: Vec3 }): Vec3 {
  const d = dot(sub(p, plane.point), plane.normal);
  return sub(p, scale(plane.normal, d));
}
