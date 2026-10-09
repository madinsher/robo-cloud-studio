/**
 * Camera geometry for the vision stack: pinhole intrinsics, projection / back-projection, depth error models
 * per modality, detection → 3D position → robot target (hand-eye), and a rigid 3D–3D fit (Horn's quaternion
 * method) for camera-to-robot calibration from measured correspondences.
 *
 * Conventions follow the studio camera item: the camera looks along its +Z axis, X right, Y down in the image.
 * Units: mm for the station, pixels for images; the catalogue error models are in metres and converted here.
 */
import type { Camera as CameraItem } from '../core/items/item';
import { Mat4, Vec3, multiply, invert, transl, fromArray, transformPoint, normalize, cross, dot, sub, norm, identity } from '../core/math/pose';
import { VISION_SENSORS, VisionSensor, VisionModality } from './stack';

export interface Intrinsics { fx: number; fy: number; cx: number; cy: number; width: number; height: number }

export function intrinsicsFromCamera(cam: { fov: number; width: number; height: number }): Intrinsics {
  const fx = (cam.width / 2) / Math.tan(((cam.fov * Math.PI) / 180) / 2);
  return { fx, fy: fx, cx: cam.width / 2, cy: cam.height / 2, width: cam.width, height: cam.height };
}
export function intrinsicsFromSensor(s: VisionSensor): Intrinsics {
  const [w, h] = s.resolution[0] ? s.resolution : [1024, 768];
  const fx = (w / 2) / Math.tan((s.hfov * Math.PI) / 360);
  const fy = (h / 2) / Math.tan((s.vfov * Math.PI) / 360);
  return { fx, fy, cx: w / 2, cy: h / 2, width: w, height: h };
}

/** Camera-frame point (mm, +Z forward) → pixel; null when behind the camera. */
export function project(K: Intrinsics, p: Vec3): [number, number] | null {
  if (p[2] <= 1e-6) return null;
  return [K.cx + (K.fx * p[0]) / p[2], K.cy + (K.fy * p[1]) / p[2]];
}
/** Pixel + depth (mm along Z) → camera-frame point (mm). */
export function backproject(K: Intrinsics, u: number, v: number, z: number): Vec3 {
  return [((u - K.cx) / K.fx) * z, ((v - K.cy) / K.fy) * z, z];
}
/** Unit ray in the camera frame through a pixel. */
export function pixelRay(K: Intrinsics, u: number, v: number): Vec3 {
  return normalize([(u - K.cx) / K.fx, (v - K.cy) / K.fy, 1]);
}
export function inImage(K: Intrinsics, uv: [number, number]): boolean {
  return uv[0] >= 0 && uv[1] >= 0 && uv[0] < K.width && uv[1] < K.height;
}

// ---------------------------------------------------------------------------------------------
// Depth error models
// ---------------------------------------------------------------------------------------------

/** 1σ depth error (mm) at range z (mm) for a sensor from the catalogue, or a generic model per modality. */
export function depthSigmaMm(sensorId: string | undefined, modality: VisionModality, zMm: number, opts: { baselineMm?: number; fxPx?: number; disparityNoisePx?: number } = {}): number {
  const s = sensorId ? VISION_SENSORS.find((x) => x.id === sensorId) : undefined;
  const z = zMm / 1000;
  if (s?.depthError) return (s.depthError.a + s.depthError.b * z * z) * 1000;
  switch (modality) {
    case 'stereo': case 'rgbd': {
      // σz = z² · σd / (f · B)
      const f = opts.fxPx ?? 700, B = (opts.baselineMm ?? 100) / 1000, sd = opts.disparityNoisePx ?? 0.25;
      return ((z * z * sd) / (f * B)) * 1000;
    }
    case 'tof': return (0.006 + 0.0005 * z * z) * 1000;
    case 'lidar3d': return 25;
    case 'mono': return 0.15 * zMm; // size-prior range estimate
    default: return 0.1 * zMm;
  }
}

/** Mono range from a known object size: z = f·D/w (mm). */
export function rangeFromSize(K: Intrinsics, boxWidthPx: number, objectSizeMm: number): number {
  return boxWidthPx > 0 ? (K.fx * objectSizeMm) / boxWidthPx : Infinity;
}

/** Pixel footprint at range z (mm per pixel). */
export function mmPerPixel(K: Intrinsics, zMm: number): number { return zMm / K.fx; }

// ---------------------------------------------------------------------------------------------
// Detection → 3D → robot target
// ---------------------------------------------------------------------------------------------

/** Camera pose in the world for a camera that is a child of an item (eye-in-hand: flange; eye-to-hand: base). */
export function cameraWorldPose(cam: CameraItem, handEye?: number[] | null, parentPose?: Mat4): Mat4 {
  if (handEye && handEye.length === 16 && parentPose) return multiply(parentPose, fromArray(handEye));
  return cam.poseAbs();
}

/**
 * Build a target pose (world, mm) for a 3D point seen by the camera: origin at the point, tool Z along the
 * approach direction (camera ray by default, or a provided surface normal pointing *towards* the camera, flipped),
 * X chosen perpendicular to world Z where possible so the wrist stays level.
 */
export function targetPoseFromPoint(pWorld: Vec3, camWorld: Mat4, opts: { normalWorld?: Vec3 | null; approachMm?: number } = {}): { grasp: Mat4; approach: Mat4 } {
  const camPos: Vec3 = [camWorld[12], camWorld[13], camWorld[14]];
  let zAxis: Vec3;
  if (opts.normalWorld) { const n = normalize(opts.normalWorld); zAxis = dot(n, sub(pWorld, camPos)) > 0 ? [-n[0], -n[1], -n[2]] : n; zAxis = [-zAxis[0], -zAxis[1], -zAxis[2]]; }
  else zAxis = normalize(sub(pWorld, camPos));
  let xAxis = cross([0, 0, 1], zAxis);
  if (norm(xAxis) < 1e-6) xAxis = [1, 0, 0]; else xAxis = normalize(xAxis);
  const yAxis = cross(zAxis, xAxis);
  const m = identity();
  m[0] = xAxis[0]; m[1] = xAxis[1]; m[2] = xAxis[2];
  m[4] = yAxis[0]; m[5] = yAxis[1]; m[6] = yAxis[2];
  m[8] = zAxis[0]; m[9] = zAxis[1]; m[10] = zAxis[2];
  m[12] = pWorld[0]; m[13] = pWorld[1]; m[14] = pWorld[2];
  const approach = multiply(m, transl(0, 0, -(opts.approachMm ?? 100)));
  return { grasp: m, approach };
}

/** World point for a pixel with a measured depth (mm along the camera Z). */
export function pixelDepthToWorld(K: Intrinsics, camWorld: Mat4, u: number, v: number, zMm: number): Vec3 {
  return transformPoint(camWorld, backproject(K, u, v, zMm));
}

/** Median of the valid depths inside a box (or under a mask) — robust 3D centroid distance. */
export function medianDepthInBox(depth: Float32Array, width: number, box: { x: number; y: number; w: number; h: number }, mask?: Uint8Array | null): number | null {
  const vals: number[] = [];
  const x0 = Math.max(0, Math.floor(box.x)), y0 = Math.max(0, Math.floor(box.y));
  const x1 = Math.min(width, Math.ceil(box.x + box.w)), y1 = Math.min(depth.length / width, Math.ceil(box.y + box.h));
  for (let y = y0; y < y1; y++) for (let x = x0; x < x1; x++) { const i = y * width + x; if (mask && !mask[i]) continue; const d = depth[i]; if (d > 0 && Number.isFinite(d)) vals.push(d); }
  if (!vals.length) return null;
  vals.sort((a, b) => a - b);
  return vals[vals.length >> 1];
}

// ---------------------------------------------------------------------------------------------
// Rigid 3D–3D registration (Horn 1987) — camera ↔ robot calibration from point correspondences
// ---------------------------------------------------------------------------------------------

/**
 * Find T such that dst ≈ T · src for N ≥ 3 non-collinear correspondences (mm). Returns the pose and RMS
 * residual. Used for hand-eye style calibration: touch N points with the TCP (dst, robot base frame) and
 * measure them with the camera (src, camera frame) → T = camera pose in the robot base.
 */
export function rigidTransform(src: Vec3[], dst: Vec3[]): { pose: Mat4; rms: number } {
  const n = Math.min(src.length, dst.length);
  if (n < 3) throw new Error('rigidTransform needs at least 3 correspondences');
  const cs: Vec3 = [0, 0, 0], cd: Vec3 = [0, 0, 0];
  for (let i = 0; i < n; i++) for (let k = 0; k < 3; k++) { cs[k] += src[i][k] / n; cd[k] += dst[i][k] / n; }
  // cross-covariance
  const S = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < n; i++) for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) S[a][b] += (src[i][a] - cs[a]) * (dst[i][b] - cd[b]);
  const [[Sxx, Sxy, Sxz], [Syx, Syy, Syz], [Szx, Szy, Szz]] = S;
  const N = [
    [Sxx + Syy + Szz, Syz - Szy, Szx - Sxz, Sxy - Syx],
    [Syz - Szy, Sxx - Syy - Szz, Sxy + Syx, Szx + Sxz],
    [Szx - Sxz, Sxy + Syx, -Sxx + Syy - Szz, Syz + Szy],
    [Sxy - Syx, Szx + Sxz, Syz + Szy, -Sxx - Syy + Szz],
  ];
  // largest eigenvector by shifted power iteration
  let shift = 0;
  for (const row of N) shift = Math.max(shift, row.reduce((s, v) => s + Math.abs(v), 0));
  let q = [1, 0.1, 0.1, 0.1];
  for (let it = 0; it < 200; it++) {
    const r = [0, 0, 0, 0];
    for (let a = 0; a < 4; a++) { r[a] = shift * q[a]; for (let b = 0; b < 4; b++) r[a] += N[a][b] * q[b]; }
    const l = Math.hypot(...r) || 1;
    q = r.map((v) => v / l);
  }
  const [w, x, y, z] = q;
  const R = [
    [w * w + x * x - y * y - z * z, 2 * (x * y - w * z), 2 * (x * z + w * y)],
    [2 * (x * y + w * z), w * w - x * x + y * y - z * z, 2 * (y * z - w * x)],
    [2 * (x * z - w * y), 2 * (y * z + w * x), w * w - x * x - y * y + z * z],
  ];
  const pose = identity();
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) pose[c * 4 + r] = R[r][c];
  const rc: Vec3 = [R[0][0] * cs[0] + R[0][1] * cs[1] + R[0][2] * cs[2], R[1][0] * cs[0] + R[1][1] * cs[1] + R[1][2] * cs[2], R[2][0] * cs[0] + R[2][1] * cs[1] + R[2][2] * cs[2]];
  pose[12] = cd[0] - rc[0]; pose[13] = cd[1] - rc[1]; pose[14] = cd[2] - rc[2];
  let se = 0;
  for (let i = 0; i < n; i++) { const p = transformPoint(pose, src[i]); se += (p[0] - dst[i][0]) ** 2 + (p[1] - dst[i][1]) ** 2 + (p[2] - dst[i][2]) ** 2; }
  return { pose, rms: Math.sqrt(se / n) };
}

/** Camera pose in the flange frame from a camera item mounted under a tool/robot (eye-in-hand hand-eye value). */
export function handEyeFromScene(cam: CameraItem, flangeWorld: Mat4): number[] {
  return Array.from(multiply(invert(flangeWorld), cam.poseAbs()));
}
