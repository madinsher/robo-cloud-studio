/**
 * Communication graph of a robot group (URRTS course, chapter 4 §4.1): adjacency matrices, classical topologies,
 * the Laplacian and its spectrum (algebraic connectivity λ₂), the admissible consensus step 1/Δmax, Metropolis
 * weights and the (r, s)-robustness needed by W-MSR (chapter 16 §16.8.3).
 */
import { Rng, Vec2, dist } from './rng';

export type Matrix = number[][];

export const zeros = (n: number, m = n): Matrix => Array.from({ length: n }, () => Array(m).fill(0));
export const eye = (n: number): Matrix => zeros(n).map((r, i) => { r[i] = 1; return r; });

/** Adjacency of the disk graph: an edge when the distance is at most `radius`; failed robots (alive[i] = false) have no edges. */
export function diskGraph(p: Vec2[], radius: number, alive?: boolean[]): Matrix {
  const n = p.length; const A = zeros(n);
  for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) if (dist(p[i], p[j]) <= radius && (!alive || (alive[i] && alive[j]))) A[i][j] = A[j][i] = 1;
  return A;
}
export function ringGraph(n: number): Matrix { const A = zeros(n); for (let i = 0; i < n; i++) { A[i][(i + 1) % n] = 1; A[(i + 1) % n][i] = 1; } return A; }
export function pathGraph(n: number): Matrix { const A = zeros(n); for (let i = 0; i + 1 < n; i++) { A[i][i + 1] = A[i + 1][i] = 1; } return A; }
export function starGraph(n: number): Matrix { const A = zeros(n); for (let i = 1; i < n; i++) { A[0][i] = A[i][0] = 1; } return A; }
export function completeGraph(n: number): Matrix { const A = zeros(n); for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) if (i !== j) A[i][j] = 1; return A; }
/** Adjacency from an edge list (1-based or 0-based names are accepted as numbers). */
export function graphFromEdges(n: number, edges: Array<[number, number, number?]>): Matrix { const A = zeros(n); for (const [a, b, w] of edges) { A[a][b] = A[b][a] = w ?? 1; } return A; }
/** Random geometric graph on a size×size square; retries until connected when `connected` is set. */
export function randomGeometricGraph(n: number, radius: number, rng: Rng, size = 1, connected = true, maxTries = 1000): { p: Vec2[]; A: Matrix } {
  for (let t = 0; t < maxTries; t++) { const p: Vec2[] = Array.from({ length: n }, () => [rng.uniform(0, size), rng.uniform(0, size)]); const A = diskGraph(p, radius); if (!connected || isConnected(A)) return { p, A }; }
  throw new Error('could not build a connected geometric graph: increase the radius');
}

export function neighbors(A: Matrix, i: number): number[] { const out: number[] = []; for (let j = 0; j < A.length; j++) if (A[i][j] > 0) out.push(j); return out; }
export function degrees(A: Matrix): number[] { return A.map((r) => r.reduce((s, v) => s + v, 0)); }
export function edgeCount(A: Matrix): number { let e = 0; for (let i = 0; i < A.length; i++) for (let j = i + 1; j < A.length; j++) if (A[i][j] > 0) e++; return e; }
export function edges(A: Matrix): Array<[number, number]> { const out: Array<[number, number]> = []; for (let i = 0; i < A.length; i++) for (let j = i + 1; j < A.length; j++) if (A[i][j] > 0) out.push([i, j]); return out; }

export function components(A: Matrix): number[][] {
  const n = A.length; const seen = Array(n).fill(false); const comps: number[][] = [];
  for (let s = 0; s < n; s++) { if (seen[s]) continue; const stack = [s]; seen[s] = true; const comp: number[] = []; while (stack.length) { const v = stack.pop()!; comp.push(v); for (let w = 0; w < n; w++) if (A[v][w] > 0 && !seen[w]) { seen[w] = true; stack.push(w); } } comps.push(comp.sort((a, b) => a - b)); }
  return comps;
}
export function isConnected(A: Matrix, subset?: number[]): boolean {
  if (subset) { const S = subset.map((i) => subset.map((j) => A[i][j])); return isConnected(S); }
  return A.length <= 1 || components(A).length === 1;
}
export function diameter(A: Matrix): number {
  const n = A.length; let dmax = 0;
  for (let s = 0; s < n; s++) { const d = Array(n).fill(Infinity); d[s] = 0; const q = [s]; while (q.length) { const v = q.shift()!; for (let w = 0; w < n; w++) if (A[v][w] > 0 && d[w] === Infinity) { d[w] = d[v] + 1; q.push(w); } } for (const x of d) { if (x === Infinity) return Infinity; dmax = Math.max(dmax, x); } }
  return dmax;
}

/** Laplacian L = D − A of a weighted undirected graph. */
export function laplacian(A: Matrix): Matrix { const d = degrees(A); return A.map((r, i) => r.map((v, j) => (i === j ? d[i] - v : v ? -v : 0))); }

/** Eigenvalues of a symmetric matrix (cyclic Jacobi rotations), ascending. */
export function symmetricEigenvalues(M: Matrix, tol = 1e-12): number[] {
  const n = M.length; const a = M.map((r) => r.slice());
  for (let sweep = 0; sweep < 100; sweep++) {
    let off = 0; for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) off += a[i][j] * a[i][j];
    if (off < tol * tol) break;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) {
      if (Math.abs(a[p][q]) < 1e-300) continue;
      const theta = (a[q][q] - a[p][p]) / (2 * a[p][q]); const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1)); const c = 1 / Math.sqrt(t * t + 1), s = t * c;
      for (let k = 0; k < n; k++) { const akp = a[k][p], akq = a[k][q]; a[k][p] = c * akp - s * akq; a[k][q] = s * akp + c * akq; }
      for (let k = 0; k < n; k++) { const apk = a[p][k], aqk = a[q][k]; a[p][k] = c * apk - s * aqk; a[q][k] = s * apk + c * aqk; }
    }
  }
  return a.map((r, i) => r[i]).sort((x, y) => x - y);
}
/** Spectrum of the Laplacian (0 = λ₁ ≤ λ₂ ≤ … ≤ λₙ). */
export function laplacianSpectrum(A: Matrix): number[] { return symmetricEigenvalues(laplacian(A)).map((v) => (Math.abs(v) < 1e-9 ? 0 : v)); }
/** Algebraic connectivity λ₂ (Fiedler): positive iff the graph is connected; 0 for a single vertex. */
export function algebraicConnectivity(A: Matrix): number { if (A.length <= 1) return 0; return laplacianSpectrum(A)[1]; }
/** Upper bound of the consensus step: ε < 1/Δmax keeps P = I − εL stochastic with a positive diagonal (∞ without edges). */
export function maxConsensusStep(A: Matrix): number { const d = Math.max(...degrees(A)); return d > 0 ? 1 / d : Infinity; }
/** Metropolis–Hastings weights: a doubly stochastic W built from local degrees only (§4.3.2). */
export function metropolisWeights(A: Matrix): Matrix {
  const n = A.length; const B = A.map((r) => r.map((v) => (v > 0 ? 1 : 0))); const d = degrees(B); const W = zeros(n);
  for (let i = 0; i < n; i++) { let s = 0; for (let j = 0; j < n; j++) if (B[i][j]) { W[i][j] = 1 / (1 + Math.max(d[i], d[j])); s += W[i][j]; } W[i][i] = 1 - s; }
  return W;
}
/** Vertex connectivity by brute force (n ≤ 12): the smallest number of removed vertices that disconnects the graph. */
export function vertexConnectivity(A: Matrix): number {
  const n = A.length; if (!isConnected(A)) return 0; if (edgeCount(A) === (n * (n - 1)) / 2) return n - 1;
  for (let k = 1; k < n - 1; k++) { const combos = combinations([...Array(n).keys()], k); for (const rm of combos) { const keep = [...Array(n).keys()].filter((i) => !rm.includes(i)); if (!isConnected(A, keep)) return k; } }
  return n - 1;
}
export function combinations<T>(xs: T[], k: number): T[][] { if (k === 0) return [[]]; if (xs.length < k) return []; const [h, ...t] = xs; return [...combinations(t, k - 1).map((c) => [h, ...c]), ...combinations(t, k)]; }

/**
 * r-robustness (LeBlanc et al. 2013) by brute force over pairs of disjoint non-empty subsets — feasible for n ≤ 9.
 * The graph is r-robust when for every pair at least one of the subsets has a vertex with ≥ r neighbours outside it.
 * Returns the largest r; W-MSR with F malicious neighbours needs (2F+1)-robustness.
 */
export function robustness(A: Matrix): number {
  const n = A.length; if (n > 10) return NaN; let best = n;
  const outside = (S: number[], v: number) => { let c = 0; for (let w = 0; w < n; w++) if (A[v][w] > 0 && !S.includes(w)) c++; return c; };
  const subsets: number[][] = []; for (let m = 1; m < 1 << n; m++) subsets.push([...Array(n).keys()].filter((i) => m & (1 << i)));
  for (const S1 of subsets) { const m1 = S1.reduce((m, i) => m | (1 << i), 0); for (const S2 of subsets) { const m2 = S2.reduce((m, i) => m | (1 << i), 0); if (m1 & m2) continue; if (S1[0] > S2[0]) continue; const r1 = Math.max(...S1.map((v) => outside(S1, v))), r2 = Math.max(...S2.map((v) => outside(S2, v))); best = Math.min(best, Math.max(r1, r2)); if (best === 0) return 0; } }
  return best;
}

export const TOPOLOGIES: Record<string, (n: number) => Matrix> = { path: pathGraph, ring: ringGraph, star: starGraph, complete: completeGraph };
