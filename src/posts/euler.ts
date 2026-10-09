import { Mat4, mul, transl, rotz, roty, DEG, RAD } from '../core/math/pose';
export { poseToKuka, poseToXyzrpw, getPos, poseToQuat, poseToFanuc, poseToUr } from '../core/math/pose';

/** Euler ZYZ decomposition (Doosan / Comau style), degrees. */
export function poseToZyz(m: Mat4): [number, number, number, number, number, number] {
  const r22 = m[10];
  let a: number, b: number, c: number;
  if (Math.abs(r22) < 1 - 1e-9) {
    b = Math.acos(r22);
    a = Math.atan2(m[9], m[8]);   // r12, r02
    c = Math.atan2(m[6], -m[2]);  // r21, -r20
  } else {
    b = r22 > 0 ? 0 : Math.PI;
    a = Math.atan2(m[1], m[0]);
    c = 0;
  }
  return [m[12], m[13], m[14], a * RAD, b * RAD, c * RAD];
}
export function zyzToPose(x: number, y: number, z: number, a: number, b: number, c: number): Mat4 {
  return mul(transl(x, y, z), rotz(a * DEG), roty(b * DEG), rotz(c * DEG));
}
