/**
 * Ray-cast sensors of the ROS simulator backend (Node and browser): a planar LiDAR with the Webots Lidar semantics
 * (ranges ordered left to right like webots_ros2 Ros2Lidar, +inf when nothing is hit or the hit is outside
 * [minRange, maxRange], Gaussian noise with std = noise x maxRange) cast against the station's triangle meshes
 * (src/vision/sensing.ts sceneMeshes: the geometry the 3D view and the camera renders draw, BVH-accelerated).
 */
import type { Mat4 } from '../core/math/pose';
import { firstHit, type SceneMesh } from '../vision/sensing';

/** Deterministic PRNG (mulberry32) in [0, 1). */
export function mulberry(seed: number): () => number {
  let a = seed >>> 0;
  return () => { a = (a + 0x6d2b79f5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
}
/** Standard normal sample (Box-Muller). */
export function gauss(rng: () => number): number { const u = Math.max(1e-12, rng()), v = rng(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }

export interface Lidar2DSpec { fov: number; res: number; minRange: number; maxRange: number; noiseStd?: number; rng?: () => number }

/**
 * Ranges (m, float32) of a planar LiDAR whose ROS frame (+X forward, +Y left, +Z up; metres) is `laserWorld`.
 * Beam i points at angle fov/2 − i·fov/(res−1) (left to right, as webots_ros2 publishes with angle_min = +fov/2).
 */
export function lidarScan2D(meshes: SceneMesh[], laserWorld: Mat4, spec: Lidar2DSpec): Float32Array {
  const out = new Float32Array(spec.res);
  const o: [number, number, number] = [laserWorld[12] * 1000, laserWorld[13] * 1000, laserWorld[14] * 1000];
  const maxMm = spec.maxRange * 1000 * 1.05 + 1;
  const step = spec.res > 1 ? spec.fov / (spec.res - 1) : 0;
  const std = spec.noiseStd ?? 0, rng = spec.rng ?? Math.random;
  for (let i = 0; i < spec.res; i++) {
    const a = spec.fov / 2 - i * step, c = Math.cos(a), s = Math.sin(a);
    const d: [number, number, number] = [laserWorld[0] * c + laserWorld[4] * s, laserWorld[1] * c + laserWorld[5] * s, laserWorld[2] * c + laserWorld[6] * s];
    const hit = firstHit(meshes, o, d, maxMm, { tMin: 1 });
    if (!hit) { out[i] = Infinity; continue; }
    let r = hit.t / 1000;
    if (std > 0) r += std * gauss(rng);
    out[i] = r < spec.minRange || r > spec.maxRange ? Infinity : r;
  }
  return out;
}
