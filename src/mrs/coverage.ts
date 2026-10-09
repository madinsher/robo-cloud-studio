/**
 * Area coverage and distributed estimation (course lecture 4, §4.3, §11.3, §11.4.5, practicum ПР5): Voronoi
 * partition on a grid, masses and centroids, the coverage functional and Lloyd's step (Cortés et al. 2004), the
 * limited-range decentralised variant, information-form consensus of measurements and covariance intersection.
 */
import { Vec2 } from './rng';
import { Matrix } from './graph';

/** Cell centres of a square grid and the area of one cell. */
export function gridPoints(xmin: number, xmax: number, ymin: number, ymax: number, res: number): { q: Vec2[]; dA: number } {
  const q: Vec2[] = []; for (let y = ymin + res / 2; y < ymax; y += res) for (let x = xmin + res / 2; x < xmax; x += res) q.push([x, y]); return { q, dA: res * res };
}
/** Importance density: background plus a Gaussian hot spot. */
export function gaussianDensity(q: Vec2[], center: Vec2, sigma: number, base = 0.05): number[] { return q.map((z) => base + Math.exp(-((z[0] - center[0]) ** 2 + (z[1] - center[1]) ** 2) / (2 * sigma * sigma))); }
export function voronoiLabels(p: Vec2[], q: Vec2[]): number[] { return q.map((z) => { let b = 0, bd = Infinity; for (let i = 0; i < p.length; i++) { const d = (z[0] - p[i][0]) ** 2 + (z[1] - p[i][1]) ** 2; if (d < bd) { bd = d; b = i; } } return b; }); }
/** Masses Mᵢ = Σφ dA and centroids Cᵢ of the cells (a zero-mass cell keeps the robot position). */
export function massCentroids(p: Vec2[], q: Vec2[], phi: number[], dA: number, labels: number[]): { M: number[]; C: Vec2[] } {
  const M = p.map(() => 0); const sx = p.map(() => 0), sy = p.map(() => 0);
  for (let k = 0; k < q.length; k++) { const i = labels[k]; if (i < 0) continue; const w = phi[k] * dA; M[i] += w; sx[i] += q[k][0] * w; sy[i] += q[k][1] * w; }
  return { M, C: p.map((pi, i) => (M[i] > 0 ? [sx[i] / M[i], sy[i] / M[i]] : [pi[0], pi[1]]) as Vec2) };
}
/** H(p) = Σ φ(q) minᵢ‖q − pᵢ‖² dA. */
export function coverageCost(p: Vec2[], q: Vec2[], phi: number[], dA: number): number { let s = 0; for (let k = 0; k < q.length; k++) { let m = Infinity; for (const pi of p) m = Math.min(m, (q[k][0] - pi[0]) ** 2 + (q[k][1] - pi[1]) ** 2); s += phi[k] * m; } return s * dA; }
/** pᵢ⁺ = pᵢ + k(Cᵢ − pᵢ); k = 1 is the classical Lloyd iteration (H never increases). */
export function lloydStep(p: Vec2[], q: Vec2[], phi: number[], dA: number, gain = 1): Vec2[] { const { C } = massCentroids(p, q, phi, dA, voronoiLabels(p, q)); return p.map((pi, i) => [pi[0] + gain * (C[i][0] - pi[0]), pi[1] + gain * (C[i][1] - pi[1])] as Vec2); }
/** Lloyd with a sensing radius: the cell of robot i is its Voronoi cell cut at rSense (needs only neighbours closer than 2 rSense). */
export function limitedLloydStep(p: Vec2[], q: Vec2[], phi: number[], dA: number, rSense: number, gain = 1): Vec2[] {
  const lab = voronoiLabels(p, q).map((i, k) => ((q[k][0] - p[i][0]) ** 2 + (q[k][1] - p[i][1]) ** 2 <= rSense * rSense ? i : -1));
  const { C } = massCentroids(p, q, phi, dA, lab); return p.map((pi, i) => [pi[0] + gain * (C[i][0] - pi[0]), pi[1] + gain * (C[i][1] - pi[1])] as Vec2);
}
export function limitedCost(p: Vec2[], q: Vec2[], phi: number[], dA: number, rSense: number): number { let s = 0; for (let k = 0; k < q.length; k++) { let m = Infinity; for (const pi of p) m = Math.min(m, (q[k][0] - pi[0]) ** 2 + (q[k][1] - pi[1]) ** 2); s += phi[k] * Math.min(m, rSense * rSense); } return s * dA; }

// --- 2×2 matrix helpers for the estimation part ---------------------------------------------------------------
export type M2 = [[number, number], [number, number]];
export const inv2 = (m: M2): M2 => { const d = m[0][0] * m[1][1] - m[0][1] * m[1][0]; return [[m[1][1] / d, -m[0][1] / d], [-m[1][0] / d, m[0][0] / d]]; };
export const mul2 = (a: M2, b: M2): M2 => [[a[0][0] * b[0][0] + a[0][1] * b[1][0], a[0][0] * b[0][1] + a[0][1] * b[1][1]], [a[1][0] * b[0][0] + a[1][1] * b[1][0], a[1][0] * b[0][1] + a[1][1] * b[1][1]]];
export const mulv2 = (a: M2, v: Vec2): Vec2 => [a[0][0] * v[0] + a[0][1] * v[1], a[1][0] * v[0] + a[1][1] * v[1]];
export const add2 = (a: M2, b: M2, ka = 1, kb = 1): M2 => [[ka * a[0][0] + kb * b[0][0], ka * a[0][1] + kb * b[0][1]], [ka * a[1][0] + kb * b[1][0], ka * a[1][1] + kb * b[1][1]]];
export const trace2 = (m: M2) => m[0][0] + m[1][1];
export const eig2 = (m: M2): [number, number] => { const tr = trace2(m), det = m[0][0] * m[1][1] - m[0][1] * m[1][0]; const d = Math.sqrt(Math.max(0, (tr * tr) / 4 - det)); return [tr / 2 - d, tr / 2 + d]; };

/** Centralised estimate x̂ = (ΣRᵢ⁻¹)⁻¹ ΣRᵢ⁻¹zᵢ, P = (ΣRᵢ⁻¹)⁻¹. */
export function centralizedEstimate(z: Vec2[], R: M2[]): { x: Vec2; P: M2 } {
  let S: M2 = [[0, 0], [0, 0]]; let y: Vec2 = [0, 0];
  for (let i = 0; i < z.length; i++) { const Ri = inv2(R[i]); S = add2(S, Ri); const yi = mulv2(Ri, z[i]); y = [y[0] + yi[0], y[1] + yi[1]]; }
  const P = inv2(S); return { x: mulv2(P, y), P };
}
/** Information consensus: yᵢ = Rᵢ⁻¹zᵢ, Sᵢ = Rᵢ⁻¹ averaged with a doubly stochastic W; estimates x̂ᵢ = Sᵢ⁻¹yᵢ per iteration. */
export function informationConsensus(z: Vec2[], R: M2[], W: Matrix, iters: number): Vec2[][] {
  let Y = z.map((zi, i) => mulv2(inv2(R[i]), zi)); let S = R.map((Ri) => inv2(Ri)); const n = z.length;
  const est = () => Y.map((yi, i) => mulv2(inv2(S[i]), yi)); const out = [est()];
  for (let k = 0; k < iters; k++) {
    const Y2: Vec2[] = []; const S2: M2[] = [];
    for (let i = 0; i < n; i++) { let y: Vec2 = [0, 0]; let s: M2 = [[0, 0], [0, 0]]; for (let j = 0; j < n; j++) { const w = W[i][j]; if (!w) continue; y = [y[0] + w * Y[j][0], y[1] + w * Y[j][1]]; s = add2(s, S[j], 1, w); } Y2.push(y); S2.push(s); }
    Y = Y2; S = S2; out.push(est());
  }
  return out;
}
/** Plain averaging of the measurements (ignores their accuracy) for comparison. */
export function naiveAverageConsensus(z: Vec2[], W: Matrix, iters: number): Vec2[][] { let X = z.map((v) => [v[0], v[1]] as Vec2); const out = [X]; for (let k = 0; k < iters; k++) { X = X.map((_, i) => { let x = 0, y = 0; for (let j = 0; j < X.length; j++) { x += W[i][j] * X[j][0]; y += W[i][j] * X[j][1]; } return [x, y] as Vec2; }); out.push(X); } return out; }
/** Covariance intersection (Julier & Uhlmann 1997): P⁻¹ = ωA⁻¹ + (1−ω)B⁻¹ with ω minimising trace P over a grid. */
export function covarianceIntersection(a: Vec2, A: M2, b: Vec2, B: M2, nGrid = 101): { x: Vec2; P: M2; omega: number } {
  const Ai = inv2(A), Bi = inv2(B); let best = { x: a, P: A, omega: 0, tr: Infinity };
  for (let k = 0; k < nGrid; k++) { const w = k / (nGrid - 1); const Pi = add2(Ai, Bi, w, 1 - w); const P = inv2(Pi); const tr = trace2(P); if (tr < best.tr - 1e-15) { const ya = mulv2(Ai, a), yb = mulv2(Bi, b); best = { x: mulv2(P, [w * ya[0] + (1 - w) * yb[0], w * ya[1] + (1 - w) * yb[1]]), P, omega: w, tr }; } }
  return { x: best.x, P: best.P, omega: best.omega };
}
