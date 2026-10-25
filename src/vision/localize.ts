/**
 * 3D localisation and grasp surfaces.
 *
 * From a metric depth image (the sensor path) — localizeFromDepth:
 *   1. the detection's pixels: its mask, or (box only) every pixel of the box with a valid depth;
 *   2. up to three dominant planes (RANSAC + least squares); with a mask the most upward-facing well-supported plane
 *      (the face a top grasp uses), without a mask the plane nearest to the camera (the object, not the table);
 *   3. the face outline: pixel rays of the plane inliers intersected with the fitted plane (per-pixel depth noise
 *      cancels laterally), minimum-area rectangle (rotating calipers) → centre, size, in-plane yaw;
 *   4. with an object model (dims), sides cut by the image border or by a nearer surface are completed from the known
 *      face size, and the object centre is the face centre pushed back by half the thickness along the normal;
 *   5. the in-plane X axis is folded by the face symmetry (90° square / 180° rectangle) towards a reference axis, the
 *      "lazy" wrist choice of RoboDK Example 06.c.
 * Non-planar objects (fruit) fall back to the robust surface centroid with the normal towards the camera.
 *
 * From ground truth (simulation, model-based 6D estimators) — surfaceFromTruth: the upward face of the oriented box.
 */
import { Mat4, Vec3, transformPoint, normalize, cross, dot, sub, add, scale, norm, identity, transl, multiply } from '../core/math/pose';
import type { Intrinsics } from './camera_model';
import { maskAt, Box2D, Mask } from './models';

export interface SurfaceEstimate {
  /** Face centre (world, mm), outward unit normal, in-plane X axis (folded by symmetry). */
  point: Vec3; normal: Vec3; xAxis: Vec3;
  /** Face size along xAxis / yAxis (mm). */
  size: [number, number];
  symmetryDeg: number;
  /** Object centre (world) when an object model gave the thickness; otherwise undefined. */
  centroid?: Vec3;
  points: number;
  planar: boolean;
}

export interface DepthFrame { depth: Float32Array; width: number; height: number; K: Intrinsics; camWorld: Mat4 }

function rng(seed: number) { let s = seed >>> 0 || 1; return () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; }; }

interface Plane { n: Vec3; d: number; inl: number[] } // n·p + d = 0 (camera frame), n towards the camera (n·p < 0 side)

function fitPlaneLS(P: Float64Array, idx: number[]): { n: Vec3; d: number } | null {
  if (idx.length < 3) return null;
  let cx = 0, cy = 0, cz = 0;
  for (const i of idx) { cx += P[i * 3]; cy += P[i * 3 + 1]; cz += P[i * 3 + 2]; }
  cx /= idx.length; cy /= idx.length; cz /= idx.length;
  let xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
  for (const i of idx) { const x = P[i * 3] - cx, y = P[i * 3 + 1] - cy, z = P[i * 3 + 2] - cz; xx += x * x; xy += x * y; xz += x * z; yy += y * y; yz += y * z; zz += z * z; }
  // smallest eigenvector of the covariance: inverse power iteration on (C + eps I)
  const C = [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]];
  const tr = xx + yy + zz || 1;
  const A = C.map((r, i) => r.map((v, j) => v / tr + (i === j ? 1e-9 : 0)));
  const det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
  if (Math.abs(det) < 1e-30) {
    // degenerate (perfect plane): normal from the cross product of the two largest spreads
    const a: Vec3 = [P[idx[0] * 3] - cx, P[idx[0] * 3 + 1] - cy, P[idx[0] * 3 + 2] - cz];
    let best: Vec3 = [0, 0, 1], bn = 0;
    for (let k = 1; k < Math.min(idx.length, 200); k++) { const b: Vec3 = [P[idx[k] * 3] - cx, P[idx[k] * 3 + 1] - cy, P[idx[k] * 3 + 2] - cz]; const c = cross(a, b); const l = norm(c); if (l > bn) { bn = l; best = c; } }
    const n = normalize(best); return { n, d: -(n[0] * cx + n[1] * cy + n[2] * cz) };
  }
  const inv = [
    [(A[1][1] * A[2][2] - A[1][2] * A[2][1]) / det, (A[0][2] * A[2][1] - A[0][1] * A[2][2]) / det, (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det],
    [(A[1][2] * A[2][0] - A[1][0] * A[2][2]) / det, (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det, (A[0][2] * A[1][0] - A[0][0] * A[1][2]) / det],
    [(A[1][0] * A[2][1] - A[1][1] * A[2][0]) / det, (A[0][1] * A[2][0] - A[0][0] * A[2][1]) / det, (A[0][0] * A[1][1] - A[0][1] * A[1][0]) / det],
  ];
  let v: Vec3 = [0.3, 0.5, 0.8];
  for (let it = 0; it < 60; it++) { const w: Vec3 = [inv[0][0] * v[0] + inv[0][1] * v[1] + inv[0][2] * v[2], inv[1][0] * v[0] + inv[1][1] * v[1] + inv[1][2] * v[2], inv[2][0] * v[0] + inv[2][1] * v[1] + inv[2][2] * v[2]]; v = normalize(w); }
  return { n: v, d: -(v[0] * cx + v[1] * cy + v[2] * cz) };
}

/** Planes seen at grazing incidence (|n·ray| < minCos) are rejected: depth noise runs along the rays, so such planes collect noise. */
function ransacPlanes(P: Float64Array, pool: number[], thr: number, seed: number, maxPlanes = 3, minFrac = 0.12, minCos = 0.3): Plane[] {
  const out: Plane[] = [];
  let rest = pool.slice();
  const total = pool.length;
  const r = rng(seed);
  for (let k = 0; k < maxPlanes && rest.length >= Math.max(12, total * minFrac); k++) {
    let best: Plane | null = null;
    for (let it = 0; it < 120; it++) {
      const a = rest[Math.floor(r() * rest.length)], b = rest[Math.floor(r() * rest.length)], c = rest[Math.floor(r() * rest.length)];
      const pa: Vec3 = [P[a * 3], P[a * 3 + 1], P[a * 3 + 2]], pb: Vec3 = [P[b * 3], P[b * 3 + 1], P[b * 3 + 2]], pc: Vec3 = [P[c * 3], P[c * 3 + 1], P[c * 3 + 2]];
      const cr = cross(sub(pb, pa), sub(pc, pa)); const l = norm(cr); if (l < 1e-6) continue;
      const n = scale(cr, 1 / l), d = -dot(n, pa);
      const pm = add(add(pa, pb), pc); if (Math.abs(dot(n, pm)) / (norm(pm) || 1) < minCos) continue;
      let cnt = 0; for (const i of rest) if (Math.abs(n[0] * P[i * 3] + n[1] * P[i * 3 + 1] + n[2] * P[i * 3 + 2] + d) < thr) cnt++;
      if (!best || cnt > best.inl.length) { const inl: number[] = []; for (const i of rest) if (Math.abs(n[0] * P[i * 3] + n[1] * P[i * 3 + 1] + n[2] * P[i * 3 + 2] + d) < thr) inl.push(i); best = { n, d, inl }; }
    }
    if (!best || best.inl.length < Math.max(12, total * minFrac)) break;
    // least-squares refinement, two rounds
    for (let round = 0; round < 2; round++) {
      const f = fitPlaneLS(P, best.inl); if (!f) break;
      const inl: number[] = []; for (const i of rest) if (Math.abs(f.n[0] * P[i * 3] + f.n[1] * P[i * 3 + 1] + f.n[2] * P[i * 3 + 2] + f.d) < thr) inl.push(i);
      best = { n: f.n, d: f.d, inl };
    }
    // orient the normal towards the camera (origin): n·p + d = 0 with the origin on the positive side
    if (best.d < 0) { best.n = scale(best.n, -1); best.d = -best.d; }
    { let cx = 0, cy = 0, cz = 0; for (const i of best.inl) { cx += P[i * 3]; cy += P[i * 3 + 1]; cz += P[i * 3 + 2]; } const l = Math.hypot(cx, cy, cz) || 1; if (Math.abs(best.n[0] * cx + best.n[1] * cy + best.n[2] * cz) / l < minCos) { const s0 = new Set(best.inl); rest = rest.filter((i) => !s0.has(i)); continue; } }
    out.push(best);
    const s = new Set(best.inl); rest = rest.filter((i) => !s.has(i));
  }
  return out;
}

/** Convex hull (monotone chain) of 2D points. */
function hull(pts: Array<[number, number]>): Array<[number, number]> {
  const p = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  if (p.length < 3) return p;
  const crossZ = (o: number[], a: number[], b: number[]) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo: Array<[number, number]> = [], up: Array<[number, number]> = [];
  for (const q of p) { while (lo.length >= 2 && crossZ(lo[lo.length - 2], lo[lo.length - 1], q) <= 0) lo.pop(); lo.push(q); }
  for (let i = p.length - 1; i >= 0; i--) { const q = p[i]; while (up.length >= 2 && crossZ(up[up.length - 2], up[up.length - 1], q) <= 0) up.pop(); up.push(q); }
  return lo.slice(0, -1).concat(up.slice(0, -1));
}
/** Minimum-area enclosing rectangle: centre, size along (cos θ, sin θ) and its normal, θ. */
export function minAreaRect(pts: Array<[number, number]>): { c: [number, number]; size: [number, number]; theta: number } | null {
  const h = hull(pts);
  if (h.length < 3) return null;
  let best: { area: number; c: [number, number]; size: [number, number]; theta: number } | null = null;
  for (let i = 0; i < h.length; i++) {
    const a = h[i], b = h[(i + 1) % h.length];
    const th = Math.atan2(b[1] - a[1], b[0] - a[0]); const c = Math.cos(th), s = Math.sin(th);
    let u0 = Infinity, u1 = -Infinity, v0 = Infinity, v1 = -Infinity;
    for (const q of h) { const u = q[0] * c + q[1] * s, v = -q[0] * s + q[1] * c; if (u < u0) u0 = u; if (u > u1) u1 = u; if (v < v0) v0 = v; if (v > v1) v1 = v; }
    const area = (u1 - u0) * (v1 - v0);
    if (!best || area < best.area) { const uc = (u0 + u1) / 2, vc = (v0 + v1) / 2; best = { area, c: [uc * c - vc * s, uc * s + vc * c], size: [u1 - u0, v1 - v0], theta: th }; }
  }
  return best && { c: best.c, size: best.size, theta: best.theta };
}

/** Fold an in-plane axis by the face symmetry so that it is closest to `ref` (projected onto the plane). */
export function foldAxis(axis: Vec3, normal: Vec3, symmetryDeg: number, ref: Vec3): Vec3 {
  const refP = sub(ref, scale(normal, dot(ref, normal)));
  if (norm(refP) < 1e-6 || symmetryDeg >= 360) return axis;
  const r = normalize(refP), y = cross(normal, axis);
  let best = axis, bd = -Infinity;
  const steps = Math.round(360 / symmetryDeg);
  for (let k = 0; k < steps; k++) {
    const a = (k * symmetryDeg * Math.PI) / 180;
    const cand = normalize(add(scale(axis, Math.cos(a)), scale(y, Math.sin(a))));
    const d = dot(cand, r); if (d > bd) { bd = d; best = cand; }
  }
  return best;
}

export interface LocalizeOptions {
  mask?: Mask | null;
  /** Object model size (mm, any order): completes truncated faces and gives the centroid. */
  dims?: Vec3 | null;
  /** 1σ depth noise (mm) at range z. */
  sigma?: (zMm: number) => number;
  /** Reference axis (world) for the symmetric yaw choice. Default world X. */
  refAxis?: Vec3;
  seed?: number;
  /** Max pixel samples (default 6000). */
  maxSamples?: number;
}

/** Surface / centre of one detection from a metric depth image; null when it has no valid depth. */
export function localizeFromDepth(frame: DepthFrame, b: Box2D, opts: LocalizeOptions = {}): SurfaceEstimate | null {
  const { depth, width: W, height: H, K, camWorld } = frame;
  const x0 = Math.max(0, Math.floor(b.x)), y0 = Math.max(0, Math.floor(b.y)), x1 = Math.min(W, Math.ceil(b.x + b.w)), y1 = Math.min(H, Math.ceil(b.y + b.h));
  if (x1 <= x0 || y1 <= y0) return null;
  const area = (x1 - x0) * (y1 - y0);
  const step = Math.max(1, Math.floor(Math.sqrt(area / (opts.maxSamples ?? 4000))));
  const mb = opts.mask ? { x: b.x, y: b.y, mask: opts.mask } : null;
  // noisy sensors: 5×5 edge-preserving mean (neighbours within 3σ of the pixel, inside the mask) — the spatial filter
  // depth SDKs apply; it cuts i.i.d. pixel noise ~4× so planes are found with a tight threshold
  const smooth = !!opts.sigma, R2 = 2;
  const U: number[] = [], V: number[] = [], Z: number[] = [];
  for (let y = y0; y < y1; y += step) for (let x = x0; x < x1; x += step) {
    if (mb && !maskAt(mb, x, y)) continue;
    let d = depth[y * W + x];
    if (!(d > 0) || !Number.isFinite(d)) continue;
    if (smooth) {
      const gate = 3 * opts.sigma!(d);
      let sum = 0, cnt = 0;
      for (let yy = Math.max(0, y - R2); yy <= Math.min(H - 1, y + R2); yy++) {
        const row = yy * W;
        for (let xx = Math.max(0, x - R2); xx <= Math.min(W - 1, x + R2); xx++) {
          const v = depth[row + xx];
          if (v > 0 && Math.abs(v - d) < gate && (!mb || maskAt(mb, xx, yy))) { sum += v; cnt++; }
        }
      }
      d = sum / cnt;
    }
    U.push(x + 0.5); V.push(y + 0.5); Z.push(d);
  }
  const n = Z.length;
  if (n < 6) return null;
  const P = new Float64Array(n * 3);
  for (let i = 0; i < n; i++) { P[i * 3] = ((U[i] - K.cx) / K.fx) * Z[i]; P[i * 3 + 1] = ((V[i] - K.cy) / K.fy) * Z[i]; P[i * 3 + 2] = Z[i]; }
  const zs = Z.slice().sort((a, c) => a - c), zMed = zs[n >> 1];
  const sig = Math.max(0.5, (opts.sigma?.(zMed) ?? 0) * (smooth ? 0.3 : 1));
  const thr = Math.max(3, 2.5 * sig);
  const R = (v: Vec3): Vec3 => [camWorld[0] * v[0] + camWorld[4] * v[1] + camWorld[8] * v[2], camWorld[1] * v[0] + camWorld[5] * v[1] + camWorld[9] * v[2], camWorld[2] * v[0] + camWorld[6] * v[1] + camWorld[10] * v[2]];
  const up: Vec3 = [0, 0, 1];
  // box only: keep the nearest depth layer (the object), not the background behind it
  let pool = Array.from({ length: n }, (_, i) => i);
  if (!mb) {
    const zNear = zs[Math.floor(0.05 * n)];
    const ext = opts.dims ? Math.max(...opts.dims) : (Math.max(x1 - x0, y1 - y0) * zNear) / K.fx;
    const zCut = zNear + ext + 3 * sig;
    pool = pool.filter((i) => Z[i] <= zCut);
  }
  const planes = ransacPlanes(P, pool, thr, opts.seed ?? 7);
  let pl: Plane | null = null;
  if (planes.length) {
    const ok = planes.filter((p) => p.inl.length >= Math.max(12, 0.2 * n));
    const cand = ok.length ? ok : planes.slice(0, 1);
    if (mb) pl = cand.reduce((a, c) => (dot(R(c.n), up) > dot(R(a.n), up) + 0.05 ? c : a));
    else pl = cand.reduce((a, c) => { const za = a.inl.reduce((s, i) => s + P[i * 3 + 2], 0) / a.inl.length, zc = c.inl.reduce((s, i) => s + P[i * 3 + 2], 0) / c.inl.length; return zc < za - thr ? c : a; });
  }
  const refW = opts.refAxis ?? [1, 0, 0];
  // non-planar (the best plane explains less than half of a masked object): robust centroid, normal towards the camera
  if (!pl || (mb && pl.inl.length < 0.5 * n)) {
    const pts = mb ? pool : (pl ? pl.inl : pool);
    const med = (k: number) => { const v = pts.map((i) => P[i * 3 + k]).sort((a, c) => a - c); return v[v.length >> 1]; };
    const pc: Vec3 = [med(0), med(1), med(2)];
    const pw = transformPoint(camWorld, pc);
    const camPos: Vec3 = [camWorld[12], camWorld[13], camWorld[14]];
    const nrm = normalize(sub(camPos, pw));
    let xa = cross(up, nrm); xa = norm(xa) < 1e-6 ? [1, 0, 0] : normalize(xa);
    const r = opts.dims ? Math.max(...opts.dims) / 2 : 0;
    return { point: pw, normal: nrm, xAxis: xa, size: [0, 0], symmetryDeg: 360, centroid: opts.dims ? sub(pw, scale(nrm, r)) : undefined, points: pts.length, planar: false };
  }
  // face outline: rays of the inlier pixels intersected with the plane
  const nC = pl.n, dC = pl.d;
  let e1: Vec3 = [1, 0, 0];
  { // in-plane basis aligned with the world reference projected on the plane (camera frame)
    const camInvR = (v: Vec3): Vec3 => [camWorld[0] * v[0] + camWorld[1] * v[1] + camWorld[2] * v[2], camWorld[4] * v[0] + camWorld[5] * v[1] + camWorld[6] * v[2], camWorld[8] * v[0] + camWorld[9] * v[1] + camWorld[10] * v[2]];
    const refC = camInvR(refW);
    const pr = sub(refC, scale(nC, dot(refC, nC)));
    e1 = norm(pr) > 1e-6 ? normalize(pr) : normalize(cross(nC, [0, 1, 0]));
  }
  const e2 = cross(nC, e1);
  const o = scale(nC, -dC); // plane point closest to the camera origin
  const pts2: Array<[number, number]> = [];
  for (const i of pl.inl) {
    const ray: Vec3 = [(U[i] - K.cx) / K.fx, (V[i] - K.cy) / K.fy, 1];
    const den = dot(nC, ray); if (Math.abs(den) < 1e-9) continue;
    const t = -dC / den; const q = scale(ray, t); const r0 = sub(q, o);
    pts2.push([dot(r0, e1), dot(r0, e2)]);
  }
  const rect = minAreaRect(pts2);
  if (!rect) return null;
  const ca = Math.cos(rect.theta), sa = Math.sin(rect.theta);
  let ax: Vec3 = normalize(add(scale(e1, ca), scale(e2, sa)));
  let ay: Vec3 = cross(nC, ax);
  let size: [number, number] = [rect.size[0], rect.size[1]];
  let cen: Vec3 = add(o, add(scale(e1, rect.c[0]), scale(e2, rect.c[1])));
  // truncation-aware completion with a known face size
  let thickness = 0;
  if (opts.dims) {
    const d = [...opts.dims].sort((a, c) => c - a) as Vec3;
    const combos: Array<[number, number, number]> = [[d[0], d[1], d[2]], [d[0], d[2], d[1]], [d[1], d[2], d[0]]];
    // match (A, B) to the measured sides allowing truncation (measured ≤ model): smallest model excess
    let bestC = combos[0], bestE = Infinity, swap = false;
    for (const c of combos) for (const sw of [false, true]) {
      const A = sw ? c[1] : c[0], B = sw ? c[0] : c[1];
      const tolA = 4 + 0.06 * A, tolB = 4 + 0.06 * B;
      const e = Math.max(0, size[0] - A - tolA) * 10 + Math.max(0, size[1] - B - tolB) * 10 + Math.abs(A - size[0]) + Math.abs(B - size[1]);
      if (e < bestE) { bestE = e; bestC = c; swap = sw; }
    }
    const A = swap ? bestC[1] : bestC[0], B = swap ? bestC[0] : bestC[1];
    thickness = bestC[2];
    const side = (axis: Vec3, sgn: number, half: number, other: Vec3, otherHalf: number): boolean => {
      // a side is cut when just beyond it lies the image border or a nearer surface
      for (const f of [-0.5, 0, 0.5]) {
        const pC = add(cen, add(scale(axis, sgn * (half + 6)), scale(other, f * otherHalf)));
        if (pC[2] <= 1) return true;
        const u = K.cx + (K.fx * pC[0]) / pC[2], v = K.cy + (K.fy * pC[1]) / pC[2];
        if (u < 0 || v < 0 || u >= W || v >= H) return true;
        const dd = depth[Math.floor(v) * W + Math.floor(u)];
        if (dd > 0 && dd < pC[2] - Math.max(15, 2 * sig)) return true;
      }
      return false;
    };
    const fix = (axis: Vec3, measured: number, model: number, other: Vec3, otherMeasured: number): number => {
      if (measured >= model - Math.max(4, 0.06 * model)) return 0;
      const plus = side(axis, 1, measured / 2, other, otherMeasured / 2), minus = side(axis, -1, measured / 2, other, otherMeasured / 2);
      if (plus && !minus) return (model - measured) / 2; // grow towards the cut side
      if (minus && !plus) return -(model - measured) / 2;
      return 0;
    };
    const sx = fix(ax, size[0], A, ay, size[1]), sy = fix(ay, size[1], B, ax, size[0]);
    cen = add(cen, add(scale(ax, sx), scale(ay, sy)));
    size = [A, B];
  }
  const symmetryDeg = Math.abs(size[0] - size[1]) <= 0.08 * Math.max(size[0], size[1]) ? 90 : 180;
  const normalW = normalize(R(nC));
  let xW = normalize(R(ax));
  xW = foldAxis(xW, normalW, symmetryDeg, refW);
  // keep size ordered along the folded axis
  if (Math.abs(dot(xW, normalize(R(ay)))) > 0.7) size = [size[1], size[0]];
  void ay;
  const pointW = transformPoint(camWorld, cen);
  return { point: pointW, normal: normalW, xAxis: xW, size, symmetryDeg, centroid: opts.dims ? sub(pointW, scale(normalW, thickness / 2)) : undefined, points: pl.inl.length, planar: true };
}

/** The upward-facing face of an oriented box (truth pose + dims): face centre, outward normal, in-plane axis folded by symmetry. */
export function surfaceFromTruth(pose: ArrayLike<number>, dims: Vec3, opts: { refAxis?: Vec3; preferUp?: Vec3 } = {}): SurfaceEstimate {
  const up = opts.preferUp ?? [0, 0, 1];
  const axes: Vec3[] = [[pose[0], pose[1], pose[2]], [pose[4], pose[5], pose[6]], [pose[8], pose[9], pose[10]]];
  let k = 0, bd = -Infinity, sg = 1;
  for (let i = 0; i < 3; i++) { const d = dot(axes[i], up); if (Math.abs(d) > bd) { bd = Math.abs(d); k = i; sg = d >= 0 ? 1 : -1; } }
  const normal = scale(axes[k], sg);
  const c: Vec3 = [pose[12], pose[13], pose[14]];
  const point = add(c, scale(normal, dims[k] / 2));
  const i1 = (k + 1) % 3, i2 = (k + 2) % 3;
  const size: [number, number] = [dims[i1], dims[i2]];
  const symmetryDeg = Math.abs(size[0] - size[1]) <= 0.08 * Math.max(size[0], size[1]) ? 90 : 180;
  let xAxis = foldAxis(axes[i1], normal, symmetryDeg, opts.refAxis ?? [1, 0, 0]);
  const swap = Math.abs(dot(xAxis, axes[i2])) > 0.7;
  return { point, normal, xAxis, size: swap ? [size[1], size[0]] : size, symmetryDeg, centroid: c, points: 0, planar: true };
}

/** Grasp + approach poses on a surface: TCP at the face centre (+ standoff along the normal), tool Z into the part, X along the folded face axis. */
export function graspFromSurface(s: { point: Vec3; normal: Vec3; xAxis: Vec3 }, opts: { approachMm?: number; standoffMm?: number } = {}): { grasp: Mat4; approach: Mat4 } {
  const z = scale(normalize(s.normal), -1);
  let x = sub(s.xAxis, scale(z, dot(s.xAxis, z)));
  x = norm(x) < 1e-6 ? normalize(cross([0, 1, 0], z)) : normalize(x);
  const y = cross(z, x);
  const p = add(s.point, scale(normalize(s.normal), opts.standoffMm ?? 2));
  const m = identity();
  m[0] = x[0]; m[1] = x[1]; m[2] = x[2]; m[4] = y[0]; m[5] = y[1]; m[6] = y[2]; m[8] = z[0]; m[9] = z[1]; m[10] = z[2];
  m[12] = p[0]; m[13] = p[1]; m[14] = p[2];
  return { grasp: m, approach: multiply(m, transl(0, 0, -(opts.approachMm ?? 100))) };
}
