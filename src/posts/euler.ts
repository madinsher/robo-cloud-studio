/**
 * Pose <-> vendor Euler conventions, ported 1:1 from RoboDK's Apache-2.0 robodk.robomath (python/robodk/robomath.py)
 * so post output matches RoboDK's posts digit for digit. Mat4 is column-major (H[r,c] = m[c*4+r]); angles in degrees.
 */
import { Mat4, mul, transl, rotz, roty, rotx, DEG, RAD } from '../core/math/pose';
export { poseToKuka, poseToXyzrpw, getPos, poseToQuat, poseToFanuc, poseToUr } from '../core/math/pose';

type V6 = [number, number, number, number, number, number];
const H = (m: Mat4, r: number, c: number) => m[c * 4 + r];

/** robomath.pose_2_xyzrpw: H = transl*rotz(w)*roty(p)*rotx(r) -> [x,y,z,r,p,w] (also Pose_2_Fanuc / Pose_2_Motoman / Pose_2_Techman). */
export function pose2xyzrpw(m: Mat4): V6 {
  let r: number, p: number, w: number;
  if (H(m, 2, 0) > 1 - 1e-10) { p = -Math.PI / 2; r = 0; w = Math.atan2(-H(m, 1, 2), H(m, 1, 1)); }
  else if (H(m, 2, 0) < -1 + 1e-10) { p = Math.PI / 2; r = 0; w = Math.atan2(H(m, 1, 2), H(m, 1, 1)); }
  else { p = Math.atan2(-H(m, 2, 0), Math.sqrt(H(m, 0, 0) ** 2 + H(m, 1, 0) ** 2)); w = Math.atan2(H(m, 1, 0), H(m, 0, 0)); r = Math.atan2(H(m, 2, 1), H(m, 2, 2)); }
  return [m[12], m[13], m[14], r * RAD, p * RAD, w * RAD];
}
export const pose2Fanuc = pose2xyzrpw;
export const pose2Motoman = pose2xyzrpw;
export const pose2Techman = pose2xyzrpw;
/** robomath.Pose_2_KUKA -> [x,y,z,A,B,C] (A about Z, B about Y, C about X). */
export function pose2Kuka(m: Mat4): V6 { const [x, y, z, r, p, w] = pose2xyzrpw(m); return [x, y, z, w, p, r]; }
/** robomath.Pose_2_Nachi -> [x,y,z,w,p,r]. */
export const pose2Nachi = pose2Kuka;
/** robomath.Pose_2_TxyzRxyz (radians): H = transl*rotx(rx)*roty(ry)*rotz(rz). */
export function pose2TxyzRxyz(m: Mat4): V6 {
  const a = H(m, 0, 0), b = H(m, 0, 1), c = H(m, 0, 2), d = H(m, 1, 2), e = H(m, 2, 2);
  let rx: number, ry: number, rz: number;
  if (c > 1 - 1e-10) { ry = Math.PI / 2; rx = 0; rz = Math.atan2(H(m, 1, 0), H(m, 1, 1)); }
  else if (c < -1 + 1e-10) { ry = -Math.PI / 2; rx = 0; rz = Math.atan2(H(m, 1, 0), H(m, 1, 1)); }
  else { const cy = Math.sqrt(1 - c * c); rx = Math.atan2(-d / cy, e / cy); ry = Math.atan2(c, cy); rz = Math.atan2(-b / cy, a / cy); }
  return [m[12], m[13], m[14], rx, ry, rz];
}
/** robomath.Pose_2_Staubli (degrees, XYZ intrinsic). */
export function pose2Staubli(m: Mat4): V6 { const v = pose2TxyzRxyz(m); return [v[0], v[1], v[2], v[3] * RAD, v[4] * RAD, v[5] * RAD]; }
/** robomath.Pose_2_Adept / Pose_2_Comau: H = transl*rotz(r)*roty(p)*rotz(w) (ZYZ). */
export function pose2Adept(m: Mat4): V6 {
  let r: number, p: number, w: number;
  if (H(m, 2, 2) > 1 - 1e-10) { r = 0; p = 0; w = Math.atan2(H(m, 1, 0), H(m, 0, 0)); }
  else if (H(m, 2, 2) < -1 + 1e-10) { r = 0; p = Math.PI; w = Math.atan2(H(m, 1, 0), H(m, 1, 1)); }
  else { const cb = H(m, 2, 2), sb = Math.sqrt(1 - cb * cb); r = Math.atan2(H(m, 1, 2) / sb, H(m, 0, 2) / sb); p = Math.atan2(sb, cb); w = Math.atan2(H(m, 2, 1) / sb, -H(m, 2, 0) / sb); }
  return [m[12], m[13], m[14], r * RAD, p * RAD, w * RAD];
}
export const pose2Comau = pose2Adept;
/** robomath.pose_2_quaternion -> [q1(w), q2(x), q3(y), q4(z)] (ABB convention). */
export function pose2Quaternion(m: Mat4): [number, number, number, number] {
  const cosangle = Math.min(Math.max((H(m, 0, 0) + H(m, 1, 1) + H(m, 2, 2) - 1) * 0.5, -1), 1);
  if (cosangle > 1 - 1e-9) return [1, 0, 0, 0];
  if (cosangle < -1 + 1e-7) {
    const diag = [H(m, 0, 0), H(m, 1, 1), H(m, 2, 2)];
    const k = diag.indexOf(Math.max(...diag));
    const col = [H(m, 0, k), H(m, 1, k), H(m, 2, k)];
    col[k] += 1;
    const s = Math.sqrt(Math.abs(2 * (1 + diag[k])));
    return [0, col[0] / s, col[1] / s, col[2] / s];
  }
  const a = H(m, 0, 0), b = H(m, 1, 1), c = H(m, 2, 2);
  const s2 = H(m, 2, 1) - H(m, 1, 2) < 0 ? -1 : 1, s3 = H(m, 0, 2) - H(m, 2, 0) < 0 ? -1 : 1, s4 = H(m, 1, 0) - H(m, 0, 1) < 0 ? -1 : 1;
  return [Math.sqrt(Math.max(a + b + c + 1, 0)) / 2, s2 * Math.sqrt(Math.max(a - b - c + 1, 0)) / 2, s3 * Math.sqrt(Math.max(-a + b - c + 1, 0)) / 2, s4 * Math.sqrt(Math.max(-a - b + c + 1, 0)) / 2];
}
/** robomath.Pose_2_UR -> [x,y,z,rx,ry,rz] rotation vector (radians), position in mm. */
export function pose2UR(m: Mat4): V6 {
  const angle = Math.acos(Math.min(Math.max((H(m, 0, 0) + H(m, 1, 1) + H(m, 2, 2) - 1) * 0.5, -1), 1));
  let r = [H(m, 2, 1) - H(m, 1, 2), H(m, 0, 2) - H(m, 2, 0), H(m, 1, 0) - H(m, 0, 1)];
  const nrm = (v: number[]) => Math.hypot(v[0], v[1], v[2]);
  if (angle < 1e-8) r = [0, 0, 0];
  else if (Math.abs(Math.sin(angle)) < 1e-8 || nrm(r) < 1e-8) {
    const d3 = [H(m, 0, 0), H(m, 1, 1), H(m, 2, 2)];
    const mx = Math.max(...d3), id = d3.indexOf(mx);
    r = id === 0 ? [H(m, 0, 0) + 1, H(m, 1, 0), H(m, 2, 0)] : id === 1 ? [H(m, 0, 1), H(m, 1, 1) + 1, H(m, 2, 1)] : [H(m, 0, 2), H(m, 1, 2), H(m, 2, 2) + 1];
    const s = angle / Math.sqrt(Math.max(0, 2 * (1 + mx)));
    r = r.map((v) => v * s);
  } else { const n = nrm(r); r = r.map((v) => (v / n) * angle); }
  return [m[12], m[13], m[14], r[0], r[1], r[2]];
}

/** Euler ZYZ decomposition (Doosan / Kawasaki O,A,T), degrees. Equals Pose_2_Adept away from singularities. */
export function poseToZyz(m: Mat4): V6 { return pose2Adept(m); }
export function zyzToPose(x: number, y: number, z: number, a: number, b: number, c: number): Mat4 {
  return mul(transl(x, y, z), rotz(a * DEG), roty(b * DEG), rotz(c * DEG));
}
/** Inverse of pose2xyzrpw. */
export function xyzrpw2pose(x: number, y: number, z: number, r: number, p: number, w: number): Mat4 {
  return mul(transl(x, y, z), rotz(w * DEG), roty(p * DEG), rotx(r * DEG));
}
/** Inverse of pose2Staubli (TxyzRxyz in degrees). */
export function staubli2pose(x: number, y: number, z: number, rx: number, ry: number, rz: number): Mat4 {
  return mul(transl(x, y, z), rotx(rx * DEG), roty(ry * DEG), rotz(rz * DEG));
}
