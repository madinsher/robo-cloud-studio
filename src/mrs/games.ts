/**
 * Game-theoretic coordination (course chapter 9): two-player normal-form games (dominance, pure and mixed Nash
 * equilibria, Pareto efficiency), best-response dynamics and fictitious play, exact potential games, cooperative
 * games (core, Shapley value) and evolutionary replicator dynamics.
 */
export interface Bimatrix { rows: string[]; cols: string[]; /** payoffs [row player, column player] per cell */ u: Array<Array<[number, number]>> }

export function dominatedStrategies(g: Bimatrix): { row: string[]; col: string[] } {
  const R = g.rows.length, C = g.cols.length; const row: string[] = [], col: string[] = [];
  for (let a = 0; a < R; a++) for (let b = 0; b < R; b++) if (a !== b && g.u[b].every((_, j) => g.u[b][j][0] > g.u[a][j][0])) { row.push(g.rows[a]); break; }
  for (let a = 0; a < C; a++) for (let b = 0; b < C; b++) if (a !== b && g.u.every((r) => r[b][1] > r[a][1])) { col.push(g.cols[a]); break; }
  return { row, col };
}
export function pureNash(g: Bimatrix): Array<[number, number]> {
  const out: Array<[number, number]> = [];
  for (let i = 0; i < g.rows.length; i++) for (let j = 0; j < g.cols.length; j++) { const rowBest = Math.max(...g.u.map((r) => r[j][0])); const colBest = Math.max(...g.u[i].map((c) => c[1])); if (g.u[i][j][0] >= rowBest - 1e-12 && g.u[i][j][1] >= colBest - 1e-12) out.push([i, j]); }
  return out;
}
/** Mixed equilibrium of a 2×2 game (p = probability of row 1, q = probability of column 1) when it is fully mixed. */
export function mixedNash2x2(g: Bimatrix): { p: number; q: number; payoff: [number, number] } | null {
  if (g.rows.length !== 2 || g.cols.length !== 2) return null; const u = g.u;
  const dq = u[0][0][0] - u[0][1][0] - u[1][0][0] + u[1][1][0]; const dp = u[0][0][1] - u[1][0][1] - u[0][1][1] + u[1][1][1]; if (Math.abs(dq) < 1e-12 || Math.abs(dp) < 1e-12) return null;
  const q = (u[1][1][0] - u[0][1][0]) / dq; const p = (u[1][1][1] - u[1][0][1]) / dp; if (p < -1e-9 || p > 1 + 1e-9 || q < -1e-9 || q > 1 + 1e-9) return null;
  const pr = [p, 1 - p], qc = [q, 1 - q]; let e0 = 0, e1 = 0; for (let i = 0; i < 2; i++) for (let j = 0; j < 2; j++) { e0 += pr[i] * qc[j] * u[i][j][0]; e1 += pr[i] * qc[j] * u[i][j][1]; }
  return { p, q, payoff: [e0, e1] };
}
export function paretoOptimal(g: Bimatrix): Array<[number, number]> {
  const cells: Array<[number, number]> = []; for (let i = 0; i < g.rows.length; i++) for (let j = 0; j < g.cols.length; j++) cells.push([i, j]);
  return cells.filter(([i, j]) => !cells.some(([a, b]) => (a !== i || b !== j) && g.u[a][b][0] >= g.u[i][j][0] && g.u[a][b][1] >= g.u[i][j][1] && (g.u[a][b][0] > g.u[i][j][0] || g.u[a][b][1] > g.u[i][j][1])));
}
/** Sequential best-response dynamics from a profile; reports convergence (or a cycle). */
export function bestResponseDynamics(g: Bimatrix, start: [number, number] = [0, 0], maxSteps = 100): { profile: [number, number]; converged: boolean; steps: number; trace: Array<[number, number]> } {
  let [i, j] = start; const trace: Array<[number, number]> = [[i, j]];
  for (let s = 0; s < maxSteps; s++) {
    let bi = i; for (let a = 0; a < g.rows.length; a++) if (g.u[a][j][0] > g.u[bi][j][0] + 1e-12) bi = a;
    let bj = j; for (let b = 0; b < g.cols.length; b++) if (g.u[bi][b][1] > g.u[bi][bj][1] + 1e-12) bj = b;
    if (bi === i && bj === j) return { profile: [i, j], converged: true, steps: s, trace };
    i = bi; j = bj; trace.push([i, j]);
  }
  return { profile: [i, j], converged: false, steps: maxSteps, trace };
}
/** Fictitious play (Brown 1951): each player best-responds to the empirical frequencies of the other. */
export function fictitiousPlay(g: Bimatrix, rounds = 500): { rowFreq: number[]; colFreq: number[]; last: [number, number] } {
  const cr = Array(g.rows.length).fill(1), cc = Array(g.cols.length).fill(1); let last: [number, number] = [0, 0];
  for (let t = 0; t < rounds; t++) {
    const sc = cc.reduce((s, v) => s + v, 0), sr = cr.reduce((s, v) => s + v, 0);
    let bi = 0, bv = -Infinity; for (let a = 0; a < g.rows.length; a++) { let e = 0; for (let b = 0; b < g.cols.length; b++) e += (cc[b] / sc) * g.u[a][b][0]; if (e > bv + 1e-12) { bv = e; bi = a; } }
    let bj = 0, bw = -Infinity; for (let b = 0; b < g.cols.length; b++) { let e = 0; for (let a = 0; a < g.rows.length; a++) e += (cr[a] / sr) * g.u[a][b][1]; if (e > bw + 1e-12) { bw = e; bj = b; } }
    cr[bi]++; cc[bj]++; last = [bi, bj];
  }
  const nr = cr.reduce((s, v) => s + v, 0) - g.rows.length, nc = cc.reduce((s, v) => s + v, 0) - g.cols.length;
  return { rowFreq: cr.map((v) => (v - 1) / nr), colFreq: cc.map((v) => (v - 1) / nc), last };
}
/** Exact potential test for a two-player game: the four-cycle condition on every 2×2 sub-square; returns a potential when it exists. */
export function exactPotential(g: Bimatrix): { isPotential: boolean; phi?: number[][] } {
  const R = g.rows.length, C = g.cols.length; const phi = Array.from({ length: R }, () => Array(C).fill(0));
  for (let i = 1; i < R; i++) phi[i][0] = phi[i - 1][0] + g.u[i][0][0] - g.u[i - 1][0][0];
  for (let i = 0; i < R; i++) for (let j = 1; j < C; j++) phi[i][j] = phi[i][j - 1] + g.u[i][j][1] - g.u[i][j - 1][1];
  for (let i = 1; i < R; i++) for (let j = 0; j < C; j++) if (Math.abs(phi[i][j] - phi[i - 1][j] - (g.u[i][j][0] - g.u[i - 1][j][0])) > 1e-9) return { isPotential: false };
  return { isPotential: true, phi };
}
/** Replicator dynamics of a symmetric game (row payoffs A): ẋᵢ = xᵢ((Ax)ᵢ − xᵀAx). */
export function replicatorDynamics(A: number[][], x0: number[], dt = 0.01, steps = 2000): number[][] { let x = x0.slice(); const out = [x.slice()]; for (let s = 0; s < steps; s++) { const Ax = A.map((r) => r.reduce((acc, v, j) => acc + v * x[j], 0)); const avg = x.reduce((acc, v, i) => acc + v * Ax[i], 0); x = x.map((v, i) => Math.max(0, v + dt * v * (Ax[i] - avg))); const sum = x.reduce((s2, v) => s2 + v, 0) || 1; x = x.map((v) => v / sum); if (s % Math.max(1, Math.floor(steps / 100)) === 0) out.push(x.slice()); } out.push(x); return out; }

// --- cooperative games --------------------------------------------------------------------------------------------
/** Characteristic function over coalitions given as sorted index keys ("0,1,2"); missing coalitions are 0. */
export type CharFn = Record<string, number>;
const keyOf = (S: number[]) => S.slice().sort((a, b) => a - b).join(',');
export function coalitionValue(v: CharFn, S: number[]): number { return v[keyOf(S)] ?? 0; }
export function shapleyValue(v: CharFn, n: number): number[] {
  const fact = (k: number): number => (k <= 1 ? 1 : k * fact(k - 1)); const phi = Array(n).fill(0); const players = [...Array(n).keys()];
  for (let i = 0; i < n; i++) { const others = players.filter((p) => p !== i); for (let m = 0; m < 1 << others.length; m++) { const S = others.filter((_, k) => m & (1 << k)); const w = (fact(S.length) * fact(n - S.length - 1)) / fact(n); phi[i] += w * (coalitionValue(v, [...S, i]) - coalitionValue(v, S)); } }
  return phi;
}
/** Is the allocation x in the core (efficient and no coalition can do better on its own)? Returns the blocking coalitions. */
export function inCore(v: CharFn, n: number, x: number[]): { inCore: boolean; blocking: string[]; efficient: boolean } {
  const players = [...Array(n).keys()]; const total = x.reduce((s, a) => s + a, 0); const efficient = Math.abs(total - coalitionValue(v, players)) < 1e-9; const blocking: string[] = [];
  for (let m = 1; m < (1 << n) - 1; m++) { const S = players.filter((i) => m & (1 << i)); const xs = S.reduce((s, i) => s + x[i], 0); if (coalitionValue(v, S) > xs + 1e-9) blocking.push(keyOf(S)); }
  return { inCore: efficient && !blocking.length, blocking, efficient };
}
/** Superadditivity check of a characteristic function. */
export function isSuperadditive(v: CharFn, n: number): boolean { const players = [...Array(n).keys()]; for (let a = 1; a < 1 << n; a++) for (let b = 1; b < 1 << n; b++) { if (a & b) continue; const S = players.filter((i) => a & (1 << i)), T = players.filter((i) => b & (1 << i)); if (coalitionValue(v, [...S, ...T]) < coalitionValue(v, S) + coalitionValue(v, T) - 1e-9) return false; } return true; }
