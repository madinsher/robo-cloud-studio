/**
 * Performance evaluation of a complex — course chapter 5.
 *
 *  - bottleneck bound (Proposition 5.1): Θ ≤ min_r C(r)/τ(r), and saturation curves over resource capacities;
 *  - Little's law: WIP = Θ · T;
 *  - continuous-time Markov chains: steady state of πQ = 0;
 *  - max-plus algebra: Karp's maximum cycle mean and the cycle time of a timed event graph (max over circuits of
 *    holding time / tokens, by parametric search);
 *  - discrete-event Monte Carlo of a serial line with random durations and confidence intervals.
 */

export interface ResourceLoad { resource: string; /** busy time per cycle (s) */ timePerCycle: number; capacity: number }

/** Θ ≤ 1 / max_r τ(r)/C(r). Returns the bound, the bottleneck and the per-resource cycle-time bounds. */
export function bottleneckBound(loads: ResourceLoad[]): { cycleTime: number; throughputPerHour: number; bottleneck: string; perResource: Array<{ resource: string; cycleBound: number }> } {
  const per = loads.map((l) => ({ resource: l.resource, cycleBound: l.timePerCycle / Math.max(1e-9, l.capacity) }));
  const worst = per.reduce((a, b) => (b.cycleBound > a.cycleBound ? b : a), per[0] ?? { resource: '—', cycleBound: 0 });
  return { cycleTime: worst.cycleBound, throughputPerHour: worst.cycleBound > 0 ? 3600 / worst.cycleBound : Infinity, bottleneck: worst.resource, perResource: per };
}

/** Saturation curve: cycle time / throughput when the capacity of one resource grows 1…maxCapacity. */
export function saturationCurve(loads: ResourceLoad[], resource: string, maxCapacity = 5): Array<{ capacity: number; cycleTime: number; throughputPerHour: number; bottleneck: string }> {
  const out = [];
  for (let c = 1; c <= maxCapacity; c++) { const b = bottleneckBound(loads.map((l) => (l.resource === resource ? { ...l, capacity: c } : l))); out.push({ capacity: c, cycleTime: b.cycleTime, throughputPerHour: b.throughputPerHour, bottleneck: b.bottleneck }); }
  return out;
}

/** Little's law: any two of {wip, throughput, leadTime} give the third. */
export function littlesLaw(v: { wip?: number; throughput?: number; leadTime?: number }): { wip: number; throughput: number; leadTime: number } {
  if (v.wip === undefined) return { wip: v.throughput! * v.leadTime!, throughput: v.throughput!, leadTime: v.leadTime! };
  if (v.throughput === undefined) return { wip: v.wip, throughput: v.wip / v.leadTime!, leadTime: v.leadTime! };
  return { wip: v.wip, throughput: v.throughput, leadTime: v.wip / v.throughput };
}

/** Steady-state distribution of a CTMC generator Q (rows sum to zero): solve πQ = 0, Σπ = 1. */
export function steadyState(Q: number[][]): number[] {
  const n = Q.length; if (!n) return [];
  // build A = Qᵀ with last row replaced by ones; b = e_n
  const A = Q.map((_, i) => Q.map((row) => row[i]));
  for (let j = 0; j < n; j++) A[n - 1][j] = 1;
  const b = new Array(n).fill(0); b[n - 1] = 1;
  return solve(A, b) ?? new Array(n).fill(1 / n);
}

/** Gaussian elimination with partial pivoting. */
export function solve(A: number[][], b: number[]): number[] | null {
  const n = A.length; const M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let piv = c; for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
    if (Math.abs(M[piv][c]) < 1e-14) return null;
    [M[c], M[piv]] = [M[piv], M[c]];
    for (let r = 0; r < n; r++) if (r !== c) { const f = M[r][c] / M[c][c]; if (f) for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k]; }
  }
  return M.map((row, i) => row[n] / row[i]);
}

/** Transient distribution after time t from p0 by uniformisation (for availability / reliability chains). */
export function transient(Q: number[][], p0: number[], t: number, tol = 1e-9): number[] {
  const n = Q.length; const lam = Math.max(...Q.map((row, i) => -row[i])) * 1.05 + 1e-9;
  const P = Q.map((row, i) => row.map((v, j) => (i === j ? 1 : 0) + v / lam));
  let term = [...p0]; const out = [...p0].map((v) => v * Math.exp(-lam * t));
  let poisson = Math.exp(-lam * t);
  for (let k = 1; k < 2000; k++) {
    term = P[0].map((_, j) => term.reduce((s, v, i) => s + v * P[i][j], 0));
    poisson *= (lam * t) / k;
    for (let j = 0; j < n; j++) out[j] += poisson * term[j];
    if (poisson < tol && k > lam * t) break;
  }
  return out;
}

/** Karp: maximum cycle mean of a weighted digraph; `adj[i]` lists {to, w}. Returns -Infinity when acyclic. */
export function maxCycleMean(n: number, adj: Array<Array<{ to: number; w: number }>>): { mean: number; cycle: number[] } {
  // add a virtual source (index n) with 0-weight arcs to every node so all SCCs are reached
  const N = n + 1; const NEG = -Infinity;
  const D: number[][] = Array.from({ length: N + 1 }, () => new Array(N).fill(NEG));
  const parent: number[][] = Array.from({ length: N + 1 }, () => new Array(N).fill(-1));
  D[0][n] = 0;
  const arcs = (i: number) => (i === n ? Array.from({ length: n }, (_, j) => ({ to: j, w: 0 })) : adj[i]);
  for (let k = 1; k <= N; k++) for (let i = 0; i < N; i++) { if (D[k - 1][i] === NEG) continue; for (const a of arcs(i)) { const v = D[k - 1][i] + a.w; if (v > D[k][a.to]) { D[k][a.to] = v; parent[k][a.to] = i; } } }
  let best = NEG, bestNode = -1;
  for (let i = 0; i < n; i++) {
    if (D[N][i] === NEG) continue;
    let worst = Infinity;
    for (let k = 0; k < N; k++) if (D[k][i] !== NEG) worst = Math.min(worst, (D[N][i] - D[k][i]) / (N - k));
    if (worst > best) { best = worst; bestNode = i; }
  }
  // recover a cycle on the critical path by walking parents from bestNode at level N until a repeat
  const cycle: number[] = [];
  if (bestNode >= 0) { const seen = new Map<number, number>(); let node = bestNode, k = N; while (k > 0 && node >= 0 && node !== n) { if (seen.has(node)) { const start = seen.get(node)!; cycle.push(...[...seen.entries()].filter(([, idx]) => idx >= start).sort((a, b) => a[1] - b[1]).map(([v]) => v)); break; } seen.set(node, seen.size); node = parent[k][node]; k--; } }
  return { mean: best, cycle: cycle.reverse() };
}

export interface EventGraphArc { from: string; to: string; delay: number; tokens: number }

/**
 * Cycle time of a timed event graph (max-plus eigenvalue): λ = max over circuits Σ delay / Σ tokens.
 * Parametric search on λ: a positive-weight cycle exists in (delay − λ·tokens) iff λ is below the true value.
 */
export function eventGraphCycleTime(nodes: string[], arcs: EventGraphArc[]): { cycleTime: number; criticalCycle: string[] } {
  const idx = new Map(nodes.map((v, i) => [v, i])); const n = nodes.length;
  const A = arcs.map((a) => ({ u: idx.get(a.from)!, v: idx.get(a.to)!, d: a.delay, k: a.tokens }));
  const hasPositiveCycle = (lam: number): boolean => {
    const dist = new Array(n).fill(0);
    for (let it = 0; it < n; it++) { let changed = false; for (const a of A) { const w = a.d - lam * a.k; if (dist[a.u] + w > dist[a.v] + 1e-12) { dist[a.v] = dist[a.u] + w; changed = true; } } if (!changed) return false; }
    return true;
  };
  if (!A.some((a) => a.k > 0)) return { cycleTime: 0, criticalCycle: [] };
  let lo = 0, hi = A.reduce((s, a) => s + Math.max(0, a.d), 0) + 1;
  for (let i = 0; i < 60; i++) { const mid = (lo + hi) / 2; if (hasPositiveCycle(mid)) lo = mid; else hi = mid; }
  const lam = (lo + hi) / 2;
  // critical cycle: max cycle mean of (d − λ k) with mean ≈ 0
  const adj: Array<Array<{ to: number; w: number }>> = nodes.map(() => []);
  for (const a of A) adj[a.u].push({ to: a.v, w: a.d - lam * a.k });
  const { cycle } = maxCycleMean(n, adj);
  return { cycleTime: lam, criticalCycle: cycle.map((i) => nodes[i]) };
}

/**
 * Operations holding resources (course §5.6.6): every resource r forms a circuit through the operations that hold it,
 * carrying C(r) tokens, so its mean is τ(r)/C(r) with τ(r) the total holding time (load + process + unload for a
 * machine loaded in place). The cycle time is the maximum over resource circuits; the job precedence chain is acyclic.
 */
export interface CycleOperation { id: string; duration: number; resources: string[]; /** predecessor operations in the same job */ after?: string[] }
export function cycleTimeFromOperations(ops: CycleOperation[], capacity: Record<string, number> = {}): { cycleTime: number; throughputPerHour: number; criticalCycle: string[]; resourceCycles: Array<{ resource: string; time: number; tokens: number; mean: number }> } {
  const resources = [...new Set(ops.flatMap((o) => o.resources))];
  const resourceCycles = resources.map((r) => { const users = ops.filter((o) => o.resources.includes(r)); const time = users.reduce((s, o) => s + o.duration, 0); const tokens = capacity[r] ?? 1; return { resource: r, time, tokens, mean: time / tokens, users: users.map((u) => u.id) }; });
  const worst = resourceCycles.reduce((a, b) => (b.mean > a.mean ? b : a), resourceCycles[0]);
  const cycleTime = worst?.mean ?? 0;
  return { cycleTime, throughputPerHour: cycleTime > 0 ? 3600 / cycleTime : Infinity, criticalCycle: worst?.users ?? [], resourceCycles: resourceCycles.map(({ users: _u, ...rest }) => rest) };
}

// ---------------------------------------------------------------------------------------------
// Discrete-event Monte Carlo
// ---------------------------------------------------------------------------------------------

export interface Rng { (): number }
export function mulberry32(seed: number): Rng { let a = seed >>> 0; return () => { a = (a + 0x6d2b79f5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }
export function randn(rng: Rng): number { let s = 0; for (let i = 0; i < 12; i++) s += rng(); return s - 6; }

/** Mean, standard deviation and 95 % confidence half-width; `warmup` samples are discarded. */
export function stats(xs: number[], warmup = 0): { n: number; mean: number; sd: number; ci95: number; min: number; max: number; p95: number } {
  const v = xs.slice(warmup); const n = v.length; const mean = v.reduce((s, x) => s + x, 0) / Math.max(1, n);
  const sd = Math.sqrt(v.reduce((s, x) => s + (x - mean) ** 2, 0) / Math.max(1, n - 1));
  const sorted = [...v].sort((a, b) => a - b);
  return { n, mean, sd, ci95: n > 1 ? 1.96 * sd / Math.sqrt(n) : 0, min: sorted[0] ?? 0, max: sorted[n - 1] ?? 0, p95: sorted[Math.min(n - 1, Math.floor(0.95 * n))] ?? 0 };
}

/**
 * Serial line with stations (mean, cv) and finite buffers: Monte Carlo of `parts` parts, returns cycle-time statistics,
 * throughput and utilisation. Blocking-after-service semantics.
 */
export function simulateSerialLine(stations: Array<{ id: string; mean: number; cv?: number; capacity?: number }>, buffers: number[], parts: number, seed = 1): { throughputPerHour: number; cycleTime: ReturnType<typeof stats>; utilisation: Record<string, number>; makespan: number } {
  const rng = mulberry32(seed); const n = stations.length;
  const dur = (i: number) => { const s = stations[i]; const cv = s.cv ?? 0; return Math.max(0.01, s.mean * (1 + cv * randn(rng))); };
  // each part flows through stations sequentially; start_i(k) = max(end_{i-1}(k), end_i(k-1), release by buffer: end_{i+1}(k - b_i - 1))
  const end: number[][] = Array.from({ length: n }, () => new Array(parts).fill(0));
  const start: number[][] = Array.from({ length: n }, () => new Array(parts).fill(0));
  const busy = new Array(n).fill(0);
  for (let k = 0; k < parts; k++) for (let i = 0; i < n; i++) {
    let s = i > 0 ? end[i - 1][k] : 0;
    if (k > 0) s = Math.max(s, end[i][k - 1]);
    const b = buffers[i] ?? Infinity; // buffer after station i
    if (i < n - 1 && k - b - 1 >= 0) s = Math.max(s, start[i + 1][k - b - 1]);
    const d = dur(i); start[i][k] = s; end[i][k] = s + d; busy[i] += d;
  }
  const makespan = end[n - 1][parts - 1];
  const cycles = Array.from({ length: parts - 1 }, (_, k) => end[n - 1][k + 1] - end[n - 1][k]);
  return { throughputPerHour: 3600 * parts / makespan, cycleTime: stats(cycles, Math.floor(parts * 0.1)), utilisation: Object.fromEntries(stations.map((s, i) => [s.id, busy[i] / makespan])), makespan };
}
