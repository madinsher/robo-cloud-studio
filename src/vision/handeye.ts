/**
 * Hand-eye calibration from point correspondences (the studio workflow; RoboDK's CameraHandEye macros do the same with
 * a board):
 *
 *  eye-to-hand  the robot carries a marker at its TCP to N poses; the camera measures the marker (camera frame).
 *               Camera pose in the robot base = rigidTransform(camera points → base points) (Horn).
 *  eye-in-hand  a fixed marker is measured from N flange poses. Unknowns X = camera pose in the flange and the marker
 *               position M (base): F_i·X·p_i = M. Linear least squares on (R_X, t_X, M) projected onto SO(3), then
 *               Gauss–Newton on (rotation vector, t_X, M).
 *
 * In simulation the "measurement" is the marker position seen through the camera: inside the image, in range, not
 * occluded (ray cast against the rendered triangles), with the sensor depth noise.
 */
import { Station, Camera as CameraItem } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { Mat4, Vec3, multiply, invert, transformPoint, identity, rotx, roty, rotz, transl, fromArray } from '../core/math/pose';
import { cameraIntrinsics, depthSigmaMm, rigidTransform } from './camera_model';
import { VisionStackConfig, setVisionStack } from './stack';
import { sceneMeshes, firstHit } from './sensing';

export interface HandEyeSample { flange: Mat4; tcpBase: Vec3; pCam: Vec3 }
export interface HandEyeResult { mode: 'eye_to_hand' | 'eye_in_hand'; pose: Mat4; rms: number; marker?: Vec3; samples: number }

function rng(seed: number) { let s = seed >>> 0 || 1; return () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; }; }
function gauss(r: () => number) { return Math.sqrt(-2 * Math.log(r() + 1e-12)) * Math.cos(2 * Math.PI * r()); }

/** Simulated camera measurement of a world point (camera frame, mm) or null when the camera cannot see it. */
export function measurePoint(station: Station, cam: CameraItem, pWorld: Vec3, opts: { noise?: boolean; seed?: number; sensor?: string; ignoreItemIds?: string[] } = {}): Vec3 | null {
  const K = cameraIntrinsics(cam);
  const camW = cam.poseAbs();
  const l = transformPoint(invert(camW), pWorld);
  if (l[2] < cam.near || l[2] > cam.far) return null;
  const u = K.cx + (K.fx * l[0]) / l[2], v = K.cy + (K.fy * l[1]) / l[2];
  if (u < 0 || v < 0 || u >= K.width || v >= K.height) return null;
  const o: Vec3 = [camW[12], camW[13], camW[14]], d: Vec3 = [pWorld[0] - o[0], pWorld[1] - o[1], pWorld[2] - o[2]];
  const len = Math.hypot(...d);
  const skip = new Set(opts.ignoreItemIds ?? []);
  const hit = firstHit(sceneMeshes(station, { fields: false }), o, d, 1 - 5 / len, { tMin: cam.near / len, skip: (m) => skip.has(m.itemId) });
  if (hit) return null;
  if (!opts.noise) return l;
  const r = rng(opts.seed ?? 1);
  const s = depthSigmaMm(opts.sensor, 'rgbd', l[2]) * 0.2; // a marker is localised from many pixels: ~5× below the pixel σ
  const lat = (l[2] / K.fx) * 0.3;
  return [l[0] + gauss(r) * lat, l[1] + gauss(r) * lat, l[2] + gauss(r) * s];
}

/** Eye-to-hand: camera (optical) pose in the robot base frame. */
export function solveEyeToHand(samples: HandEyeSample[]): HandEyeResult {
  const { pose, rms } = rigidTransform(samples.map((s) => s.pCam), samples.map((s) => s.tcpBase));
  return { mode: 'eye_to_hand', pose, rms, samples: samples.length };
}

function solveLinear(A: number[][], b: number[]): number[] {
  const n = b.length, M = A.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c; for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    const d = M[c][c]; if (Math.abs(d) < 1e-14) continue;
    for (let r = 0; r < n; r++) if (r !== c) { const f = M[r][c] / d; if (f) for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k]; }
  }
  return M.map((r, i) => (Math.abs(r[i]) < 1e-14 ? 0 : r[n] / r[i]));
}
/** Nearest rotation (polar decomposition by Higham iteration). */
function orthonormalize(R: number[][]): number[][] {
  let X = R.map((r) => r.slice());
  for (let it = 0; it < 30; it++) {
    const det = X[0][0] * (X[1][1] * X[2][2] - X[1][2] * X[2][1]) - X[0][1] * (X[1][0] * X[2][2] - X[1][2] * X[2][0]) + X[0][2] * (X[1][0] * X[2][1] - X[1][1] * X[2][0]);
    if (Math.abs(det) < 1e-12) break;
    const invT = [
      [(X[1][1] * X[2][2] - X[1][2] * X[2][1]) / det, (X[1][2] * X[2][0] - X[1][0] * X[2][2]) / det, (X[1][0] * X[2][1] - X[1][1] * X[2][0]) / det],
      [(X[0][2] * X[2][1] - X[0][1] * X[2][2]) / det, (X[0][0] * X[2][2] - X[0][2] * X[2][0]) / det, (X[0][1] * X[2][0] - X[0][0] * X[2][1]) / det],
      [(X[0][1] * X[1][2] - X[0][2] * X[1][1]) / det, (X[0][2] * X[1][0] - X[0][0] * X[1][2]) / det, (X[0][0] * X[1][1] - X[0][1] * X[1][0]) / det],
    ];
    X = X.map((r, i) => r.map((v, j) => 0.5 * (v + invT[i][j])));
  }
  return X;
}
const rotVec = (w: Vec3): Mat4 => { const a = Math.hypot(...w); if (a < 1e-12) return identity(); const k = w.map((x) => x / a) as Vec3; const c = Math.cos(a), s = Math.sin(a), t = 1 - c; const m = identity(); m[0] = t * k[0] * k[0] + c; m[4] = t * k[0] * k[1] - s * k[2]; m[8] = t * k[0] * k[2] + s * k[1]; m[1] = t * k[0] * k[1] + s * k[2]; m[5] = t * k[1] * k[1] + c; m[9] = t * k[1] * k[2] - s * k[0]; m[2] = t * k[0] * k[2] - s * k[1]; m[6] = t * k[1] * k[2] + s * k[0]; m[10] = t * k[2] * k[2] + c; return m; };

/** Eye-in-hand: camera (optical) pose in the flange frame and the fixed marker position (base). Needs ≥ 4 varied flange orientations. */
export function solveEyeInHand(samples: HandEyeSample[]): HandEyeResult {
  const n = samples.length;
  if (n < 4) throw new Error('eye-in-hand calibration needs at least 4 samples');
  // linear: R_F·(R_X·p) + R_F·t + t_F − M = 0, unknowns [vec(R_X) row-major (9), t (3), M (3)]
  const AtA = Array.from({ length: 15 }, () => new Array(15).fill(0)), Atb = new Array(15).fill(0);
  for (const s of samples) {
    const F = s.flange, p = s.pCam;
    for (let r = 0; r < 3; r++) {
      const row = new Array(15).fill(0);
      for (let k = 0; k < 3; k++) { const RF = F[k * 4 + r]; for (let j = 0; j < 3; j++) row[k * 3 + j] += RF * p[j]; row[9 + k] += RF; }
      row[12 + r] = -1;
      const rhs = -F[12 + r];
      for (let a = 0; a < 15; a++) { Atb[a] += row[a] * rhs; for (let b = 0; b < 15; b++) AtA[a][b] += row[a] * row[b]; }
    }
  }
  const x = solveLinear(AtA, Atb);
  const R0 = orthonormalize([[x[0], x[1], x[2]], [x[3], x[4], x[5]], [x[6], x[7], x[8]]]);
  let X = identity();
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) X[c * 4 + r] = R0[r][c];
  X[12] = x[9]; X[13] = x[10]; X[14] = x[11];
  let M: Vec3 = [x[12], x[13], x[14]];
  // Gauss–Newton refinement on (δrot, t, M)
  const resid = (Xp: Mat4, Mp: Vec3) => samples.flatMap((s) => { const q = transformPoint(multiply(s.flange, Xp), s.pCam); return [q[0] - Mp[0], q[1] - Mp[1], q[2] - Mp[2]]; });
  for (let it = 0; it < 20; it++) {
    const r0 = resid(X, M);
    const J: number[][] = r0.map(() => new Array(9).fill(0));
    const h = 1e-6;
    for (let k = 0; k < 9; k++) {
      let Xp = X, Mp: Vec3 = [...M];
      if (k < 3) { const w: Vec3 = [0, 0, 0]; w[k] = h; Xp = multiply(X, rotVec(w)); }
      else if (k < 6) { Xp = Float64Array.from(X) as unknown as Mat4; Xp[12 + k - 3] += h; }
      else Mp[k - 6] += h;
      const r1 = resid(Xp, Mp);
      for (let i = 0; i < r0.length; i++) J[i][k] = (r1[i] - r0[i]) / h;
    }
    const JtJ = Array.from({ length: 9 }, (_, a) => Array.from({ length: 9 }, (_, b) => J.reduce((s, row) => s + row[a] * row[b], 0) + (a === b ? 1e-9 : 0)));
    const Jtr = Array.from({ length: 9 }, (_, a) => -J.reduce((s, row, i) => s + row[a] * r0[i], 0));
    const dx = solveLinear(JtJ, Jtr);
    X = multiply(X, rotVec([dx[0], dx[1], dx[2]])); X[12] += dx[3]; X[13] += dx[4]; X[14] += dx[5];
    M = [M[0] + dx[6], M[1] + dx[7], M[2] + dx[8]];
    if (Math.hypot(...dx) < 1e-9) break;
  }
  const r = resid(X, M);
  let se = 0; for (let i = 0; i < r.length; i += 3) se += r[i] ** 2 + r[i + 1] ** 2 + r[i + 2] ** 2;
  return { mode: 'eye_in_hand', pose: X, rms: Math.sqrt(se / n), marker: M, samples: n };
}

/**
 * Simulated data collection: random flange poses around the current one (±`radiusMm`, ±`tiltDeg`), reachable by IK,
 * with the marker seen by the camera. Eye-to-hand: marker = TCP; eye-in-hand: marker = `markerWorld` (fixed).
 * The robot is restored afterwards.
 */
export function collectHandEyeSamples(station: Station, robot: Robot, cam: CameraItem, mode: 'eye_to_hand' | 'eye_in_hand', opts: { n?: number; radiusMm?: number; tiltDeg?: number; markerWorld?: Vec3; seed?: number; noise?: boolean; sensor?: string } = {}): HandEyeSample[] {
  const r = rng(opts.seed ?? 11), n = opts.n ?? 12, R = opts.radiusMm ?? 150, T = ((opts.tiltDeg ?? 20) * Math.PI) / 180;
  const q0 = robot.joints(), base = robot.poseAbs(), baseInv = invert(base), tcp0 = robot.poseTCPAbs();
  const out: HandEyeSample[] = [];
  const own = [...robot.walk()].map((x) => x.id);
  try {
    for (let tries = 0; out.length < n && tries < n * 30; tries++) {
      const d = multiply(transl((r() * 2 - 1) * R, (r() * 2 - 1) * R, (r() * 2 - 1) * R), multiply(rotx((r() * 2 - 1) * T), multiply(roty((r() * 2 - 1) * T), rotz((r() * 2 - 1) * T * 2))));
      const tcpW = multiply(tcp0, d);
      const ik = robot.solveIK(multiply(baseInv, tcpW), { seed: q0 });
      if (!ik.ok) continue;
      robot.setJoints(ik.joints);
      const flangeBase = multiply(baseInv, multiply(robot.poseAbs(), robot.fk().flange));
      const tcpWorld = robot.poseTCPAbs();
      const marker: Vec3 = mode === 'eye_to_hand' ? [tcpWorld[12], tcpWorld[13], tcpWorld[14]] : (opts.markerWorld as Vec3);
      if (!marker) throw new Error('eye-in-hand collection needs markerWorld');
      const p = measurePoint(station, cam, marker, { noise: opts.noise, seed: (opts.seed ?? 11) * 7 + tries, sensor: opts.sensor, ignoreItemIds: mode === 'eye_to_hand' ? own : [] });
      if (!p) continue;
      const tb = transformPoint(baseInv, marker);
      out.push({ flange: flangeBase, tcpBase: tb, pCam: p });
    }
  } finally { robot.setJoints(q0); }
  return out;
}

/** Store the calibration in the camera's vision stack (cfg.handEye, the optical pose in the flange / base frame). */
export function applyHandEye(cam: CameraItem, cfg: VisionStackConfig, res: HandEyeResult): VisionStackConfig {
  const next: VisionStackConfig = { ...cfg, handEye: Array.from(res.pose), handEyeMode: res.mode };
  setVisionStack(cam, next);
  return next;
}

/** Camera world pose from a calibration (eye-in-hand: flange·X; eye-to-hand: base·X). */
export function cameraPoseFromHandEye(robot: Robot, cfg: VisionStackConfig): Mat4 | null {
  if (!cfg.handEye || cfg.handEye.length !== 16) return null;
  const X = fromArray(cfg.handEye);
  return cfg.handEyeMode === 'eye_to_hand' ? multiply(robot.poseAbs(), X) : multiply(multiply(robot.poseAbs(), robot.fk().flange), X);
}
