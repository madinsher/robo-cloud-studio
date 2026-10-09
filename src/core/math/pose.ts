/**
 * Pose math for VerticalBot Studio.
 *
 * Conventions follow RoboDK: a pose is a 4x4 homogeneous matrix, positions in mm,
 * angles in degrees at the API boundary (radians internally where noted).
 * Matrices are stored column-major in a Float64Array(16) (same layout as three.js Matrix4.elements).
 */
export type Mat4 = Float64Array; // length 16, column-major
export type Vec3 = [number, number, number];
export type Quat = [number, number, number, number]; // w, x, y, z  (ABB / RoboDK order)

export const DEG = Math.PI / 180;
export const RAD = 180 / Math.PI;

export function identity(): Mat4 {
  const m = new Float64Array(16);
  m[0] = m[5] = m[10] = m[15] = 1;
  return m;
}

export function clone(m: Mat4): Mat4 {
  return new Float64Array(m);
}

export function fromArray(a: ArrayLike<number>): Mat4 {
  const m = new Float64Array(16);
  for (let i = 0; i < 16; i++) m[i] = a[i];
  return m;
}

/** Build from row-major 4x4 nested array (as RoboDK Mat prints it). */
export function fromRows(rows: number[][]): Mat4 {
  const m = new Float64Array(16);
  for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++) m[c * 4 + r] = rows[r][c];
  return m;
}

export function toRows(m: Mat4): number[][] {
  const rows: number[][] = [];
  for (let r = 0; r < 4; r++) {
    rows.push([m[r], m[4 + r], m[8 + r], m[12 + r]]);
  }
  return rows;
}

export function multiply(a: Mat4, b: Mat4, out: Mat4 = new Float64Array(16)): Mat4 {
  const a00 = a[0], a01 = a[4], a02 = a[8], a03 = a[12];
  const a10 = a[1], a11 = a[5], a12 = a[9], a13 = a[13];
  const a20 = a[2], a21 = a[6], a22 = a[10], a23 = a[14];
  const a30 = a[3], a31 = a[7], a32 = a[11], a33 = a[15];
  for (let c = 0; c < 4; c++) {
    const b0 = b[c * 4], b1 = b[c * 4 + 1], b2 = b[c * 4 + 2], b3 = b[c * 4 + 3];
    out[c * 4] = a00 * b0 + a01 * b1 + a02 * b2 + a03 * b3;
    out[c * 4 + 1] = a10 * b0 + a11 * b1 + a12 * b2 + a13 * b3;
    out[c * 4 + 2] = a20 * b0 + a21 * b1 + a22 * b2 + a23 * b3;
    out[c * 4 + 3] = a30 * b0 + a31 * b1 + a32 * b2 + a33 * b3;
  }
  return out;
}

export function mul(...ms: Mat4[]): Mat4 {
  let r = ms[0];
  for (let i = 1; i < ms.length; i++) r = multiply(r, ms[i]);
  return clone(r);
}

/** Inverse of a rigid transform (rotation + translation). */
export function invert(m: Mat4): Mat4 {
  const out = new Float64Array(16);
  // transpose rotation
  out[0] = m[0]; out[1] = m[4]; out[2] = m[8];
  out[4] = m[1]; out[5] = m[5]; out[6] = m[9];
  out[8] = m[2]; out[9] = m[6]; out[10] = m[10];
  const tx = m[12], ty = m[13], tz = m[14];
  out[12] = -(out[0] * tx + out[4] * ty + out[8] * tz);
  out[13] = -(out[1] * tx + out[5] * ty + out[9] * tz);
  out[14] = -(out[2] * tx + out[6] * ty + out[10] * tz);
  out[15] = 1;
  return out;
}

/** General 4x4 inverse (for non-rigid transforms, e.g. scaled objects). */
export function invertGeneral(m: Mat4): Mat4 {
  const n11 = m[0], n21 = m[1], n31 = m[2], n41 = m[3];
  const n12 = m[4], n22 = m[5], n32 = m[6], n42 = m[7];
  const n13 = m[8], n23 = m[9], n33 = m[10], n43 = m[11];
  const n14 = m[12], n24 = m[13], n34 = m[14], n44 = m[15];
  const t11 = n23 * n34 * n42 - n24 * n33 * n42 + n24 * n32 * n43 - n22 * n34 * n43 - n23 * n32 * n44 + n22 * n33 * n44;
  const t12 = n14 * n33 * n42 - n13 * n34 * n42 - n14 * n32 * n43 + n12 * n34 * n43 + n13 * n32 * n44 - n12 * n33 * n44;
  const t13 = n13 * n24 * n42 - n14 * n23 * n42 + n14 * n22 * n43 - n12 * n24 * n43 - n13 * n22 * n44 + n12 * n23 * n44;
  const t14 = n14 * n23 * n32 - n13 * n24 * n32 - n14 * n22 * n33 + n12 * n24 * n33 + n13 * n22 * n34 - n12 * n23 * n34;
  const det = n11 * t11 + n21 * t12 + n31 * t13 + n41 * t14;
  if (det === 0) return identity();
  const d = 1 / det;
  const o = new Float64Array(16);
  o[0] = t11 * d;
  o[1] = (n24 * n33 * n41 - n23 * n34 * n41 - n24 * n31 * n43 + n21 * n34 * n43 + n23 * n31 * n44 - n21 * n33 * n44) * d;
  o[2] = (n22 * n34 * n41 - n24 * n32 * n41 + n24 * n31 * n42 - n21 * n34 * n42 - n22 * n31 * n44 + n21 * n32 * n44) * d;
  o[3] = (n23 * n32 * n41 - n22 * n33 * n41 - n23 * n31 * n42 + n21 * n33 * n42 + n22 * n31 * n43 - n21 * n32 * n43) * d;
  o[4] = t12 * d;
  o[5] = (n13 * n34 * n41 - n14 * n33 * n41 + n14 * n31 * n43 - n11 * n34 * n43 - n13 * n31 * n44 + n11 * n33 * n44) * d;
  o[6] = (n14 * n32 * n41 - n12 * n34 * n41 - n14 * n31 * n42 + n11 * n34 * n42 + n12 * n31 * n44 - n11 * n32 * n44) * d;
  o[7] = (n12 * n33 * n41 - n13 * n32 * n41 + n13 * n31 * n42 - n11 * n33 * n42 - n12 * n31 * n43 + n11 * n32 * n43) * d;
  o[8] = t13 * d;
  o[9] = (n14 * n23 * n41 - n13 * n24 * n41 - n14 * n21 * n43 + n11 * n24 * n43 + n13 * n21 * n44 - n11 * n23 * n44) * d;
  o[10] = (n12 * n24 * n41 - n14 * n22 * n41 + n14 * n21 * n42 - n11 * n24 * n42 - n12 * n21 * n44 + n11 * n22 * n44) * d;
  o[11] = (n13 * n22 * n41 - n12 * n23 * n41 - n13 * n21 * n42 + n11 * n23 * n42 + n12 * n21 * n43 - n11 * n22 * n43) * d;
  o[12] = t14 * d;
  o[13] = (n13 * n24 * n31 - n14 * n23 * n31 + n14 * n21 * n33 - n11 * n24 * n33 - n13 * n21 * n34 + n11 * n23 * n34) * d;
  o[14] = (n14 * n22 * n31 - n12 * n24 * n31 - n14 * n21 * n32 + n11 * n24 * n32 + n12 * n21 * n34 - n11 * n22 * n34) * d;
  o[15] = (n12 * n23 * n31 - n13 * n22 * n31 + n13 * n21 * n32 - n11 * n23 * n32 - n12 * n21 * n33 + n11 * n22 * n33) * d;
  return o;
}

export function transl(x: number, y: number, z: number): Mat4 {
  const m = identity();
  m[12] = x; m[13] = y; m[14] = z;
  return m;
}

export function rotx(rad: number): Mat4 {
  const c = Math.cos(rad), s = Math.sin(rad);
  const m = identity();
  m[5] = c; m[6] = s; m[9] = -s; m[10] = c;
  return m;
}

export function roty(rad: number): Mat4 {
  const c = Math.cos(rad), s = Math.sin(rad);
  const m = identity();
  m[0] = c; m[2] = -s; m[8] = s; m[10] = c;
  return m;
}

export function rotz(rad: number): Mat4 {
  const c = Math.cos(rad), s = Math.sin(rad);
  const m = identity();
  m[0] = c; m[1] = s; m[4] = -s; m[5] = c;
  return m;
}

/** Rotation about an arbitrary unit axis (Rodrigues). */
export function rotAxis(axis: Vec3, rad: number): Mat4 {
  const [x, y, z] = normalize(axis);
  const c = Math.cos(rad), s = Math.sin(rad), t = 1 - c;
  const m = identity();
  m[0] = t * x * x + c; m[4] = t * x * y - s * z; m[8] = t * x * z + s * y;
  m[1] = t * x * y + s * z; m[5] = t * y * y + c; m[9] = t * y * z - s * x;
  m[2] = t * x * z - s * y; m[6] = t * y * z + s * x; m[10] = t * z * z + c;
  return m;
}

export function getPos(m: Mat4): Vec3 {
  return [m[12], m[13], m[14]];
}

export function setPos(m: Mat4, p: Vec3): Mat4 {
  m[12] = p[0]; m[13] = p[1]; m[14] = p[2];
  return m;
}

export function normalize(v: Vec3): Vec3 {
  const l = Math.hypot(v[0], v[1], v[2]) || 1;
  return [v[0] / l, v[1] / l, v[2] / l];
}
export function cross(a: Vec3, b: Vec3): Vec3 {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}
export function dot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
export function sub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}
export function add(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}
export function scale(a: Vec3, s: number): Vec3 {
  return [a[0] * s, a[1] * s, a[2] * s];
}
export function norm(a: Vec3): number {
  return Math.hypot(a[0], a[1], a[2]);
}
export function distance(a: Vec3, b: Vec3): number {
  return norm(sub(a, b));
}

/** Transform a point by a pose. */
export function transformPoint(m: Mat4, p: Vec3): Vec3 {
  return [
    m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12],
    m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13],
    m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14],
  ];
}
/** Rotate a direction vector by a pose (no translation). */
export function transformDir(m: Mat4, p: Vec3): Vec3 {
  return [
    m[0] * p[0] + m[4] * p[1] + m[8] * p[2],
    m[1] * p[0] + m[5] * p[1] + m[9] * p[2],
    m[2] * p[0] + m[6] * p[1] + m[10] * p[2],
  ];
}

// ---------------------------------------------------------------------------
// Euler conventions (RoboDK naming). All angles in DEGREES, positions in mm.
// ---------------------------------------------------------------------------

/** RoboDK default "XYZRPW" (Pose_2_TxyzRxyz): pose = transl * rotx(rx) * roty(ry) * rotz(rz) — static XYZ. */
export function xyzrpwToPose(x: number, y: number, z: number, rx: number, ry: number, rz: number): Mat4 {
  return mul(transl(x, y, z), rotx(rx * DEG), roty(ry * DEG), rotz(rz * DEG));
}

export function poseToXyzrpw(m: Mat4): [number, number, number, number, number, number] {
  // m = Rx(a) * Ry(b) * Rz(c)
  const r02 = m[8]; // row0 col2
  let a: number, b: number, c: number;
  if (Math.abs(r02) < 1 - 1e-9) {
    b = Math.asin(r02);
    a = Math.atan2(-m[9], m[10]); // -r12, r22
    c = Math.atan2(-m[4], m[0]);  // -r01, r00
  } else {
    // gimbal lock
    b = r02 > 0 ? Math.PI / 2 : -Math.PI / 2;
    a = Math.atan2(m[1], m[5]); // r10, r11
    c = 0;
  }
  return [m[12], m[13], m[14], a * RAD, b * RAD, c * RAD];
}

/** KUKA / Nachi "XYZABC": pose = transl * rotz(A) * roty(B) * rotx(C). */
export function kukaToPose(x: number, y: number, z: number, a: number, b: number, c: number): Mat4 {
  return mul(transl(x, y, z), rotz(a * DEG), roty(b * DEG), rotx(c * DEG));
}

export function poseToKuka(m: Mat4): [number, number, number, number, number, number] {
  // m = Rz(a) Ry(b) Rx(c)
  const r20 = m[2];
  let a: number, b: number, c: number;
  if (Math.abs(r20) < 1 - 1e-9) {
    b = Math.asin(-r20);
    a = Math.atan2(m[1], m[0]); // r10, r00
    c = Math.atan2(m[6], m[10]); // r21, r22
  } else {
    b = r20 < 0 ? Math.PI / 2 : -Math.PI / 2;
    a = 0;
    c = Math.atan2(-m[9], m[5]); // -r12, r11
  }
  return [m[12], m[13], m[14], a * RAD, b * RAD, c * RAD];
}

/** Fanuc / Motoman "XYZWPR": pose = transl * rotz(R) * roty(P) * rotx(W). Same as KUKA with reordered args. */
export function fanucToPose(x: number, y: number, z: number, w: number, p: number, r: number): Mat4 {
  return kukaToPose(x, y, z, r, p, w);
}
export function poseToFanuc(m: Mat4): [number, number, number, number, number, number] {
  const [x, y, z, a, b, c] = poseToKuka(m);
  return [x, y, z, c, b, a];
}

/** Stäubli / Comau / Mecademic "XYZ Euler ZYZ" — not identical among brands, RoboDK uses Rz(a) Ry(b) Rz(c). */
export function zyzToPose(x: number, y: number, z: number, a: number, b: number, c: number): Mat4 {
  return mul(transl(x, y, z), rotz(a * DEG), roty(b * DEG), rotz(c * DEG));
}

/** ABB quaternion [w,x,y,z] pose. */
export function quatToPose(x: number, y: number, z: number, q: Quat): Mat4 {
  const [w, qx, qy, qz] = q;
  const n = Math.hypot(w, qx, qy, qz) || 1;
  const a = w / n, b = qx / n, c = qy / n, d = qz / n;
  const m = identity();
  m[0] = a * a + b * b - c * c - d * d;
  m[4] = 2 * (b * c - a * d);
  m[8] = 2 * (b * d + a * c);
  m[1] = 2 * (b * c + a * d);
  m[5] = a * a - b * b + c * c - d * d;
  m[9] = 2 * (c * d - a * b);
  m[2] = 2 * (b * d - a * c);
  m[6] = 2 * (c * d + a * b);
  m[10] = a * a - b * b - c * c + d * d;
  m[12] = x; m[13] = y; m[14] = z;
  return m;
}

export function poseToQuat(m: Mat4): Quat {
  const r00 = m[0], r01 = m[4], r02 = m[8];
  const r10 = m[1], r11 = m[5], r12 = m[9];
  const r20 = m[2], r21 = m[6], r22 = m[10];
  const tr = r00 + r11 + r22;
  let w: number, x: number, y: number, z: number;
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2;
    w = 0.25 * s; x = (r21 - r12) / s; y = (r02 - r20) / s; z = (r10 - r01) / s;
  } else if (r00 > r11 && r00 > r22) {
    const s = Math.sqrt(1 + r00 - r11 - r22) * 2;
    w = (r21 - r12) / s; x = 0.25 * s; y = (r01 + r10) / s; z = (r02 + r20) / s;
  } else if (r11 > r22) {
    const s = Math.sqrt(1 + r11 - r00 - r22) * 2;
    w = (r02 - r20) / s; x = (r01 + r10) / s; y = 0.25 * s; z = (r12 + r21) / s;
  } else {
    const s = Math.sqrt(1 + r22 - r00 - r11) * 2;
    w = (r10 - r01) / s; x = (r02 + r20) / s; y = (r12 + r21) / s; z = 0.25 * s;
  }
  return [w, x, y, z];
}

/** Universal Robots pose: x,y,z (mm) + rotation vector (axis * angle, radians). */
export function urToPose(x: number, y: number, z: number, rx: number, ry: number, rz: number): Mat4 {
  const angle = Math.hypot(rx, ry, rz);
  if (angle < 1e-12) return transl(x, y, z);
  const m = rotAxis([rx / angle, ry / angle, rz / angle], angle);
  m[12] = x; m[13] = y; m[14] = z;
  return m;
}

export function poseToUr(m: Mat4): [number, number, number, number, number, number] {
  const [w, x, y, z] = poseToQuat(m);
  const s = Math.hypot(x, y, z);
  const angle = 2 * Math.atan2(s, w);
  if (s < 1e-12) return [m[12], m[13], m[14], 0, 0, 0];
  const k = angle / s;
  return [m[12], m[13], m[14], x * k, y * k, z * k];
}

/** URDF rpy: pose = transl * rotz(yaw) * roty(pitch) * rotx(roll) (radians). */
export function urdfToPose(xyz: Vec3, rpy: Vec3): Mat4 {
  return mul(transl(xyz[0], xyz[1], xyz[2]), rotz(rpy[2]), roty(rpy[1]), rotx(rpy[0]));
}

/** Angular distance between two rotations (radians). */
export function rotationAngle(a: Mat4, b: Mat4): number {
  const r = multiply(invert(a), b);
  const tr = r[0] + r[5] + r[10];
  return Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2)));
}

export function slerpPose(a: Mat4, b: Mat4, t: number): Mat4 {
  const qa = poseToQuat(a), qb = poseToQuat(b);
  let d = qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3];
  const qb2: Quat = d < 0 ? [-qb[0], -qb[1], -qb[2], -qb[3]] : qb;
  d = Math.abs(d);
  let q: Quat;
  if (d > 0.9995) {
    q = [qa[0] + t * (qb2[0] - qa[0]), qa[1] + t * (qb2[1] - qa[1]), qa[2] + t * (qb2[2] - qa[2]), qa[3] + t * (qb2[3] - qa[3])];
  } else {
    const th = Math.acos(d), s = Math.sin(th);
    const wa = Math.sin((1 - t) * th) / s, wb = Math.sin(t * th) / s;
    q = [wa * qa[0] + wb * qb2[0], wa * qa[1] + wb * qb2[1], wa * qa[2] + wb * qb2[2], wa * qa[3] + wb * qb2[3]];
  }
  const pa = getPos(a), pb = getPos(b);
  return quatToPose(pa[0] + t * (pb[0] - pa[0]), pa[1] + t * (pb[1] - pa[1]), pa[2] + t * (pb[2] - pa[2]), q);
}

export function poseEquals(a: Mat4, b: Mat4, tol = 1e-6): boolean {
  for (let i = 0; i < 16; i++) if (Math.abs(a[i] - b[i]) > tol) return false;
  return true;
}

/** Build a pose from a position and Z axis direction (+ optional X hint). Useful for fruit-picking approach frames. */
export function poseFromZ(p: Vec3, zdir: Vec3, xhint: Vec3 = [1, 0, 0]): Mat4 {
  const z = normalize(zdir);
  let x = sub(xhint, scale(z, dot(xhint, z)));
  if (norm(x) < 1e-6) x = sub([0, 1, 0], scale(z, dot([0, 1, 0], z)));
  x = normalize(x);
  const y = cross(z, x);
  const m = identity();
  m[0] = x[0]; m[1] = x[1]; m[2] = x[2];
  m[4] = y[0]; m[5] = y[1]; m[6] = y[2];
  m[8] = z[0]; m[9] = z[1]; m[10] = z[2];
  m[12] = p[0]; m[13] = p[1]; m[14] = p[2];
  return m;
}

export function formatPose(m: Mat4, digits = 3): string {
  return toRows(m).map((r) => r.map((v) => v.toFixed(digits).padStart(10)).join(' ')).join('\n');
}
