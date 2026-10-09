/**
 * Verification, validation and acceptance (course chapter 17): robust semantics of signal temporal logic (STL),
 * falsification by optimisation (random restarts + adaptive local search, CMA-lite), pairwise covering arrays,
 * the rule of three and Clopper–Pearson bounds with sample-size planning, sim-to-real gap statistics (Welch's
 * t-test), traceability matrix bookkeeping and an acceptance-programme skeleton.
 */
import { expr, Env } from './expr';
import { mulberry32, randn, stats } from './perf';

// ---------------------------------------------------------------------------------------------
// STL
// ---------------------------------------------------------------------------------------------

export type STL =
  | { k: 'pred'; expr: string } | { k: 'not'; a: STL } | { k: 'and'; a: STL; b: STL } | { k: 'or'; a: STL; b: STL } | { k: 'imp'; a: STL; b: STL }
  | { k: 'G'; a: STL; lo: number; hi: number } | { k: 'F'; a: STL; lo: number; hi: number } | { k: 'U'; a: STL; b: STL; lo: number; hi: number };

export interface Signal { t: number[]; values: Env[] }

/** Parse STL: `G[0,60] (human -> dist >= 0.8)`, `F[0,5] done`, `a U[0,3] b`, predicates are expressions (robustness = margin). */
export function parseSTL(src: string): STL {
  let i = 0; const s = src;
  const ws = () => { while (i < s.length && /\s/.test(s[i])) i++; };
  const interval = (): [number, number] => { ws(); if (s[i] !== '[') return [0, Infinity]; const m = /^\[\s*([\d.]+)\s*,\s*([\d.]+|inf)\s*\]/.exec(s.slice(i)); if (!m) throw new Error('bad interval'); i += m[0].length; return [Number(m[1]), m[2] === 'inf' ? Infinity : Number(m[2])]; };
  const primary = (): STL => {
    ws();
    if (s[i] === '(') { i++; const f = impl(); ws(); if (s[i] !== ')') throw new Error("expected ')'"); i++; return f; }
    if (s[i] === '!' || s.startsWith('not ', i)) { i += s[i] === '!' ? 1 : 4; return { k: 'not', a: primary() }; }
    const m = /^(G|F|globally|eventually)\b/.exec(s.slice(i));
    if (m) { i += m[0].length; const [lo, hi] = interval(); return { k: m[1].startsWith('G') || m[1] === 'globally' ? 'G' : 'F', a: primary(), lo, hi } as STL; }
    // predicate: read until an operator at depth 0
    let depth = 0, j = i;
    for (; j < s.length; j++) { const c = s[j]; if (c === '(') depth++; else if (c === ')') { if (depth === 0) break; depth--; } else if (depth === 0 && (s.startsWith('->', j) || s.startsWith('&&', j) || s.startsWith('||', j) || /\sU\s|\sU\[/.test(s.slice(j, j + 3)) || (c === '&' && s[j + 1] !== '&') || (c === '|' && s[j + 1] !== '|'))) break; }
    const text = s.slice(i, j).trim(); if (!text) throw new Error(`unexpected '${s.slice(i, i + 10)}'`); i = j;
    return { k: 'pred', expr: text };
  };
  const until = (): STL => { let a = primary(); ws(); while (/^U(\[|\s)/.test(s.slice(i))) { i++; const [lo, hi] = interval(); const b = primary(); a = { k: 'U', a, b, lo, hi }; ws(); } return a; };
  const conj = (): STL => { let a = until(); ws(); while (s.startsWith('&&', i) || (s[i] === '&' && s[i + 1] !== '&')) { i += s.startsWith('&&', i) ? 2 : 1; const b = until(); a = { k: 'and', a, b }; ws(); } return a; };
  const disj = (): STL => { let a = conj(); ws(); while (s.startsWith('||', i) || (s[i] === '|' && s[i + 1] !== '|')) { i += s.startsWith('||', i) ? 2 : 1; const b = conj(); a = { k: 'or', a, b }; ws(); } return a; };
  const impl = (): STL => { const a = disj(); ws(); if (s.startsWith('->', i)) { i += 2; const b = impl(); return { k: 'imp', a, b }; } return a; };
  const f = impl(); ws(); if (i < s.length) throw new Error(`unexpected '${s.slice(i)}'`);
  return f;
}

/** Robustness ρ(φ, x, t) (Definition 17.1) on a sampled signal; index-based evaluation with time intervals. */
export function robustness(f: STL, sig: Signal, k = 0): number {
  const n = sig.t.length; const t0 = sig.t[k];
  const idxRange = (lo: number, hi: number): number[] => { const out: number[] = []; for (let j = k; j < n; j++) { const d = sig.t[j] - t0; if (d >= lo - 1e-12 && d <= hi + 1e-12) out.push(j); if (d > hi) break; } return out; };
  switch (f.k) {
    case 'pred': return expr(f.expr).robustness(sig.values[k]);
    case 'not': return -robustness(f.a, sig, k);
    case 'and': return Math.min(robustness(f.a, sig, k), robustness(f.b, sig, k));
    case 'or': return Math.max(robustness(f.a, sig, k), robustness(f.b, sig, k));
    case 'imp': return Math.max(-robustness(f.a, sig, k), robustness(f.b, sig, k));
    case 'G': { const r = idxRange(f.lo, f.hi); return r.length ? Math.min(...r.map((j) => robustness(f.a, sig, j))) : Infinity; }
    case 'F': { const r = idxRange(f.lo, f.hi); return r.length ? Math.max(...r.map((j) => robustness(f.a, sig, j))) : -Infinity; }
    case 'U': { const r = idxRange(f.lo, f.hi); let best = -Infinity; for (const j of r) { let m = robustness(f.b, sig, j); for (let q = k; q < j; q++) m = Math.min(m, robustness(f.a, sig, q)); best = Math.max(best, m); } return best; }
  }
}
export function formatSTL(f: STL): string {
  const iv = (lo: number, hi: number) => (lo === 0 && hi === Infinity ? '' : `[${lo},${hi === Infinity ? 'inf' : hi}]`);
  switch (f.k) { case 'pred': return f.expr; case 'not': return `!(${formatSTL(f.a)})`; case 'and': return `(${formatSTL(f.a)} && ${formatSTL(f.b)})`; case 'or': return `(${formatSTL(f.a)} || ${formatSTL(f.b)})`; case 'imp': return `(${formatSTL(f.a)} -> ${formatSTL(f.b)})`; case 'G': return `G${iv(f.lo, f.hi)} (${formatSTL(f.a)})`; case 'F': return `F${iv(f.lo, f.hi)} (${formatSTL(f.a)})`; case 'U': return `(${formatSTL(f.a)} U${iv(f.lo, f.hi)} ${formatSTL(f.b)})`; }
}

// ---------------------------------------------------------------------------------------------
// Falsification
// ---------------------------------------------------------------------------------------------

export interface ParamSpace { [name: string]: [number, number] }
export interface FalsificationResult { minRobustness: number; worst: Record<string, number>; evaluations: number; falsified: boolean; history: Array<{ params: Record<string, number>; rho: number }>; randomBaseline: number }

/**
 * Minimise robustness over scenario parameters: random exploration, then adaptive Gaussian local search (a CMA-ES-like
 * step-size adaptation) from the best points. `simulate(params)` returns the signal; the same budget is also spent on
 * pure random search to report the baseline (§17.5.3).
 */
export function falsify(f: STL | string, space: ParamSpace, simulate: (p: Record<string, number>) => Signal, opts: { budget?: number; seed?: number; restarts?: number } = {}): FalsificationResult {
  const phi = typeof f === 'string' ? parseSTL(f) : f; const rng = mulberry32(opts.seed ?? 1); const budget = opts.budget ?? 200; const names = Object.keys(space);
  const sample = () => Object.fromEntries(names.map((n) => [n, space[n][0] + rng() * (space[n][1] - space[n][0])]));
  const clamp = (p: Record<string, number>) => Object.fromEntries(names.map((n) => [n, Math.min(space[n][1], Math.max(space[n][0], p[n]))]));
  const history: FalsificationResult['history'] = []; let evals = 0;
  const evalP = (p: Record<string, number>) => { evals++; const rho = robustness(phi, simulate(p)); history.push({ params: p, rho }); return rho; };
  // phase 1: random exploration (40 % of the budget)
  const explore = Math.max(4, Math.floor(budget * 0.4)); const pts: Array<{ p: Record<string, number>; rho: number }> = [];
  for (let i = 0; i < explore; i++) { const p = sample(); pts.push({ p, rho: evalP(p) }); }
  pts.sort((a, b) => a.rho - b.rho);
  const randomBaseline = pts[0].rho;
  // phase 2: local search from the best `restarts` points
  const restarts = Math.min(opts.restarts ?? 3, pts.length);
  let best = pts[0];
  const perStart = Math.floor((budget - explore) / restarts);
  for (let r = 0; r < restarts; r++) {
    let cur = pts[r]; let sigma = 0.25;
    for (let i = 0; i < perStart; i++) {
      const cand = clamp(Object.fromEntries(names.map((n) => [n, cur.p[n] + sigma * (space[n][1] - space[n][0]) * randn(rng)])));
      const rho = evalP(cand);
      if (rho < cur.rho) { cur = { p: cand, rho }; sigma = Math.min(0.5, sigma * 1.3); } else sigma = Math.max(0.01, sigma * 0.85);
      if (cur.rho < best.rho) best = cur;
      if (best.rho < 0 && i > 5) break;
    }
  }
  return { minRobustness: best.rho, worst: best.p, evaluations: evals, falsified: best.rho < 0, history, randomBaseline };
}

// ---------------------------------------------------------------------------------------------
// Combinatorial testing
// ---------------------------------------------------------------------------------------------

/** Pairwise (2-wise) covering array by the greedy IPOG-like horizontal growth: covers every pair of values of every pair of factors. */
export function pairwise(factors: Record<string, string[]>): Array<Record<string, string>> {
  const names = Object.keys(factors); if (names.length < 2) return (factors[names[0]] ?? []).map((v) => ({ [names[0]]: v }));
  const uncovered = new Set<string>();
  for (let i = 0; i < names.length; i++) for (let j = i + 1; j < names.length; j++) for (const a of factors[names[i]]) for (const b of factors[names[j]]) uncovered.add(`${names[i]}=${a}|${names[j]}=${b}`);
  const covers = (row: Record<string, string>): string[] => { const out: string[] = []; const ks = Object.keys(row); for (let i = 0; i < ks.length; i++) for (let j = i + 1; j < ks.length; j++) { const [x, y] = names.indexOf(ks[i]) < names.indexOf(ks[j]) ? [ks[i], ks[j]] : [ks[j], ks[i]]; const k = `${x}=${row[x]}|${y}=${row[y]}`; if (uncovered.has(k)) out.push(k); } return out; };
  const rows: Array<Record<string, string>> = [];
  let guard = 0;
  while (uncovered.size && guard++ < 10000) {
    // greedy: build one row picking for each factor the value covering most new pairs
    const row: Record<string, string> = {};
    const first = [...uncovered][0].split('|').map((s) => s.split('=')); row[first[0][0]] = first[0][1]; row[first[1][0]] = first[1][1];
    for (const n of names) { if (row[n] !== undefined) continue; let bestV = factors[n][0], bestC = -1; for (const v of factors[n]) { const c = covers({ ...row, [n]: v }).length; if (c > bestC) { bestC = c; bestV = v; } } row[n] = bestV; }
    for (const k of covers(row)) uncovered.delete(k);
    rows.push(row);
  }
  return rows;
}

// ---------------------------------------------------------------------------------------------
// Statistics for acceptance
// ---------------------------------------------------------------------------------------------

/** Rule of three: upper 95 % bound on the failure probability after n failure-free trials (Proposition 17.1). */
export const ruleOfThree = (n: number): number => 3 / n;
/** Number of failure-free trials to demonstrate p_fail ≤ pMax at 95 % confidence. */
export const trialsForRuleOfThree = (pMax: number): number => Math.ceil(3 / pMax);

function betaInc(x: number, a: number, b: number): number { // regularised incomplete beta by continued fraction
  if (x <= 0) return 0; if (x >= 1) return 1;
  const lbeta = lgamma(a) + lgamma(b) - lgamma(a + b); const front = Math.exp(Math.log(x) * a + Math.log(1 - x) * b - lbeta) / a;
  const cf = (x: number, a: number, b: number) => { const eps = 1e-14; let f = 1, c = 1, d = 0; for (let i = 0; i <= 400; i++) { const m = i >> 1; let num: number; if (i === 0) num = 1; else if (i % 2 === 0) num = (m * (b - m) * x) / ((a + 2 * m - 1) * (a + 2 * m)); else num = -((a + m) * (a + b + m) * x) / ((a + 2 * m) * (a + 2 * m + 1)); d = 1 + num * d; if (Math.abs(d) < 1e-30) d = 1e-30; d = 1 / d; c = 1 + num / c; if (Math.abs(c) < 1e-30) c = 1e-30; const cd = c * d; f *= cd; if (Math.abs(1 - cd) < eps) break; } return f; };
  return x < (a + 1) / (a + b + 2) ? front * cf(x, a, b) : 1 - (Math.exp(Math.log(1 - x) * b + Math.log(x) * a - lbeta) / b) * cf(1 - x, b, a);
}
function lgamma(z: number): number { const g = 7, c = [0.99999999999980993, 676.5203681218851, -1259.1392167224028, 771.32342877765313, -176.61502916214059, 12.507343278686905, -0.13857109526572012, 9.9843695780195716e-6, 1.5056327351493116e-7]; if (z < 0.5) return Math.log(Math.PI / Math.sin(Math.PI * z)) - lgamma(1 - z); z -= 1; let x = c[0]; for (let i = 1; i < g + 2; i++) x += c[i] / (z + i); const t = z + g + 0.5; return 0.5 * Math.log(2 * Math.PI) + (z + 0.5) * Math.log(t) - t + Math.log(x); }

/** Clopper–Pearson one-sided lower bound on the success probability for k successes in n trials at confidence `conf`. */
export function clopperPearsonLower(k: number, n: number, conf = 0.95): number {
  if (k === 0) return 0; if (k === n) return Math.pow(1 - conf, 1 / n);
  let lo = 0, hi = 1; for (let i = 0; i < 60; i++) { const mid = (lo + hi) / 2; const pAtLeastK = betaInc(mid, k, n - k + 1); /* P(X ≥ k | p=mid) = I_p(k, n−k+1) */ if (pAtLeastK < 1 - conf) lo = mid; else hi = mid; }
  return (lo + hi) / 2;
}
/** Trials needed so that with at most `failures` failures the lower bound reaches `target`. */
/** Trials needed at a constant observed failure rate (e.g. 3 %) so that the lower bound reaches `target` (course: ≈250 for 0.95). */
export function trialsForRate(target: number, failRate: number, conf = 0.95): number { for (let n = 10; n < 100000; n++) { const k = Math.round(n * (1 - failRate)); if (clopperPearsonLower(k, n, conf) >= target) return n; } return Infinity; }
export function trialsForTarget(target: number, failures: number, conf = 0.95): number { for (let n = failures + 1; n < 100000; n++) if (clopperPearsonLower(n - failures, n, conf) >= target) return n; return Infinity; }

/** Welch's t-test for the sim-to-real gap: bias, ratio of spreads and whether the difference is significant (α = 0.05). */
export function simRealGap(sim: number[], real: number[]): { bias: number; biasPercent: number; spreadRatio: number; t: number; significant: boolean; simStats: ReturnType<typeof stats>; realStats: ReturnType<typeof stats> } {
  const a = stats(sim), b = stats(real); const se = Math.sqrt(a.sd ** 2 / a.n + b.sd ** 2 / b.n); const t = se ? (b.mean - a.mean) / se : 0;
  return { bias: b.mean - a.mean, biasPercent: a.mean ? (100 * (b.mean - a.mean)) / a.mean : 0, spreadRatio: a.sd ? b.sd / a.sd : Infinity, t, significant: Math.abs(t) > 2.0, simStats: a, realStats: b };
}

// ---------------------------------------------------------------------------------------------
// Traceability and acceptance
// ---------------------------------------------------------------------------------------------

export interface Requirement { id: string; text: string; cls: 'safety' | 'integrity' | 'liveness' | 'performance' | 'other'; formal?: string; verification: string; artifact?: string; result?: 'pass' | 'fail' | 'open' | 'degenerate'; residualRisk?: string; components?: string[] }

/** Requirements affected by a change of a component (re-verification scope, §17.9.1). */
export function impactOfChange(reqs: Requirement[], component: string): Requirement[] { return reqs.filter((r) => (r.components ?? []).includes(component)); }
export function traceabilitySummary(reqs: Requirement[]): { total: number; pass: number; fail: number; open: number; degenerate: number; unformalised: string[] } {
  return { total: reqs.length, pass: reqs.filter((r) => r.result === 'pass').length, fail: reqs.filter((r) => r.result === 'fail').length, open: reqs.filter((r) => !r.result || r.result === 'open').length, degenerate: reqs.filter((r) => r.result === 'degenerate').length, unformalised: reqs.filter((r) => !r.formal).map((r) => r.id) };
}

/** Acceptance metric check with statistical justification (mean and upper 95 % bound vs requirement). */
const T975 = [12.71, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228, 2.201, 2.179, 2.16, 2.145, 2.131, 2.12, 2.11, 2.101, 2.093, 2.086, 2.08, 2.074, 2.069, 2.064, 2.06, 2.056, 2.052, 2.048, 2.045, 2.042];
export const tQuantile975 = (df: number): number => (df <= 0 ? Infinity : df <= 30 ? T975[df - 1] : 1.96 + 2.5 / df);
export function acceptanceCheck(samples: number[], limit: number, kind: 'max' | 'min'): { mean: number; bound95: number; passMean: boolean; passWithConfidence: boolean; note: string } {
  const s = stats(samples); const tq = tQuantile975(s.n - 1); const bound = kind === 'max' ? s.mean + tq * s.sd : s.mean - tq * s.sd;
  const passMean = kind === 'max' ? s.mean <= limit : s.mean >= limit; const passC = kind === 'max' ? bound <= limit : bound >= limit;
  return { mean: s.mean, bound95: bound, passMean, passWithConfidence: passC, note: passC ? 'requirement met with 95 % one-sided confidence' : passMean ? 'mean is fine but the 95 % bound violates the requirement: more runs or a probabilistic wording are needed' : 'requirement not met' };
}

export const ACCEPTANCE_TEMPLATE = [
  '1. Object and configuration (equipment list, software versions, parameters)',
  '2. Conditions (lighting, temperature, part mix, operator qualification)',
  '3. Nominal scenarios (all part types, full shift) — ≥ 20 runs each',
  '4. Failure scenarios — every row of the degradation matrix injected, ≥ 10 runs',
  '5. Boundary scenarios (max load, min tolerances, extreme positions)',
  '6. Protective functions — each one separately incl. channel fault injection, ≥ 50 triggers',
  '7. Endurance run (8–72 h) with failure and degradation logging',
  '8. Acceptance criteria — numeric, with measurement method and statistical justification (rule of three / Clopper–Pearson)',
];
