/**
 * Resilience of a robot group (course chapter 16 and §13.5): Byzantine agreement with oral messages (the
 * four-agent example of §16.8.2 and the general OM(m) recursion, n ≥ 3f + 1), trust / reputation-weighted
 * consensus (§16.8.4), graceful degradation of a formation under robot failures (§16.5.3) and stability of switched
 * linear systems (common Lyapunov test, dwell time, §13.5.1).
 */
import { Matrix, symmetricEigenvalues } from './graph';

export type Value = string | number;
export interface ByzantineAgent { name: string; value: Value; /** liar: value sent to each recipient (by name) or a function of the recipient */ lies?: Record<string, Value> }
/** Two-round oral-messages protocol (OM(1)): round 1 everyone broadcasts, round 2 everyone relays what it heard; majority per source, then majority of sources. */
export function byzantineAgreement(agents: ByzantineAgent[]): { decisions: Record<string, Value>; evidence: Record<string, Record<string, Value[]>>; agreementAmongHonest: boolean; honestDecision: Value | null; /** per-source majorities of honest agents that were ties (undecidable evidence) */ ambiguous: number } {
  const names = agents.map((a) => a.name); const byName = Object.fromEntries(agents.map((a) => [a.name, a]));
  const said = (from: string, to: string): Value => { const a = byName[from]; return a.lies && a.lies[to] !== undefined ? a.lies[to] : a.value; };
  const heard: Record<string, Record<string, Value>> = {}; for (const r of names) { heard[r] = {}; for (const s of names) if (s !== r) heard[r][s] = said(s, r); }
  // round 2: r tells t what s said to r; a liar relays a lie (its lies map, applied to the relayed value) — modelled as: liar relays its own lie value for that recipient
  const evidence: Record<string, Record<string, Value[]>> = {}; const decisions: Record<string, Value> = {}; let ambiguous = 0;
  const isTie = (vals: Value[]) => { const c = new Map<Value, number>(); for (const v of vals) c.set(v, (c.get(v) ?? 0) + 1); const ks = [...c.values()].sort((a, b) => b - a); return ks.length > 1 && ks[0] === ks[1]; };
  for (const t of names) {
    evidence[t] = {};
    for (const s of names) { if (s === t) continue; const vals: Value[] = [heard[t][s]]; for (const r of names) { if (r === t || r === s) continue; const relay = byName[r].lies ? (byName[r].lies![t] ?? heard[r][s]) : heard[r][s]; vals.push(relay); } evidence[t][s] = vals; }
    const perSource = Object.values(evidence[t]).map(majority); if (!byName[t].lies) ambiguous += Object.values(evidence[t]).filter(isTie).length; decisions[t] = majority([byName[t].value, ...perSource]);
  }
  const honest = agents.filter((a) => !a.lies).map((a) => decisions[a.name]); const agreementAmongHonest = honest.every((v) => v === honest[0]);
  return { decisions, evidence, agreementAmongHonest, honestDecision: agreementAmongHonest ? honest[0] : null, ambiguous };
}
export function majority(vals: Value[]): Value { const c = new Map<Value, number>(); for (const v of vals) c.set(v, (c.get(v) ?? 0) + 1); let best: Value = vals[0], bc = -1; for (const [v, k] of [...c].sort((x, y) => String(x[0]).localeCompare(String(y[0])))) if (k > bc) { best = v; bc = k; } return best; }
/** n ≥ 3f + 1 condition and the number of messages of OM(f) (n−1)(n−2)…(n−f−1). */
export function byzantineBound(n: number, f: number): { tolerable: boolean; maxFaulty: number; messagesOM: number } { let m = n - 1; for (let k = 1; k <= f; k++) m *= n - 1 - k; return { tolerable: n >= 3 * f + 1, maxFaulty: Math.floor((n - 1) / 3), messagesOM: m }; }

/** Reputation-weighted consensus: trust of a neighbour decays when its value is an outlier w.r.t. the local median; weights ∝ trust. */
export function trustConsensus(x0: number[], A: Matrix, eps: number, steps: number, liars: Record<number, (k: number) => number>, o: { threshold?: number; decay?: number; recover?: number } = {}): { xs: number[][]; trust: number[][]; finalTrustOfLiars: number } {
  const n = x0.length; let x = x0.slice(); const trust: number[][] = A.map((r) => r.map((v) => (v > 0 ? 1 : 0))); const xs = [x.slice()]; const thr = o.threshold ?? 1, decay = o.decay ?? 0.5, recover = o.recover ?? 0.05;
  for (let k = 0; k < steps; k++) {
    for (const [i, f] of Object.entries(liars)) x[Number(i)] = f(k);
    const nx = x.slice();
    for (let i = 0; i < n; i++) {
      if (liars[i]) continue; const nb: number[] = []; for (let j = 0; j < n; j++) if (A[i][j] > 0) nb.push(j); if (!nb.length) continue;
      const vals = nb.map((j) => x[j]).sort((a, b) => a - b); const med = vals[Math.floor(vals.length / 2)]; const spread = Math.max(1e-9, vals.reduce((s, v) => s + Math.abs(v - med), 0) / vals.length);
      let s = 0; for (const j of nb) { const outlier = Math.abs(x[j] - med) > thr * Math.max(spread, 1e-3) * 3 + 1e-9; trust[i][j] = outlier ? trust[i][j] * (1 - decay) : Math.min(1, trust[i][j] + recover); if (!outlier) s += trust[i][j] * (x[j] - x[i]); } // a value flagged as an outlier is not used this step, and the neighbour's reputation drops
      nx[i] = x[i] + eps * s;
    }
    x = nx; xs.push(x.slice());
  }
  const liarIdx = Object.keys(liars).map(Number); let tl = 0, c = 0; for (let i = 0; i < n; i++) if (!liars[i]) for (const j of liarIdx) if (A[i][j] > 0) { tl += trust[i][j]; c++; }
  return { xs, trust, finalTrustOfLiars: c ? tl / c : 0 };
}

/** Graceful degradation (§16.5.3): remove robots one by one from a formation graph and track connectivity and λ₂. */
export function degradation(A: Matrix, order: number[]): Array<{ removed: number[]; alive: number; connected: boolean; lambda2: number }> {
  const out: Array<{ removed: number[]; alive: number; connected: boolean; lambda2: number }> = []; const removed: number[] = [];
  for (let k = 0; k <= order.length; k++) {
    const keep = [...Array(A.length).keys()].filter((i) => !removed.includes(i)); const S = keep.map((i) => keep.map((j) => A[i][j])); const d = S.map((r) => r.reduce((s, v) => s + v, 0)); const L = S.map((r, i) => r.map((v, j) => (i === j ? d[i] - v : v ? -v : 0)));
    const lam = keep.length > 1 ? symmetricEigenvalues(L)[1] : 0; out.push({ removed: removed.slice(), alive: keep.length, connected: keep.length <= 1 || lam > 1e-9, lambda2: Math.max(0, lam) });
    if (k < order.length) removed.push(order[k]);
  }
  return out;
}

// --- switched linear systems (§13.5.1) ------------------------------------------------------------------------------
export type M2 = [[number, number], [number, number]];
const mulM = (a: M2, b: M2): M2 => [[a[0][0] * b[0][0] + a[0][1] * b[1][0], a[0][0] * b[0][1] + a[0][1] * b[1][1]], [a[1][0] * b[0][0] + a[1][1] * b[1][0], a[1][0] * b[0][1] + a[1][1] * b[1][1]]];
/** Matrix exponential of a 2×2 matrix by scaling and squaring with a Taylor series. */
export function expm2(A: M2, t = 1): M2 { const B: M2 = [[A[0][0] * t, A[0][1] * t], [A[1][0] * t, A[1][1] * t]]; const nrm = Math.max(Math.abs(B[0][0]) + Math.abs(B[0][1]), Math.abs(B[1][0]) + Math.abs(B[1][1])); const s = Math.max(0, Math.ceil(Math.log2(nrm + 1e-12)) + 1); const sc = 2 ** s; const C: M2 = [[B[0][0] / sc, B[0][1] / sc], [B[1][0] / sc, B[1][1] / sc]]; let term: M2 = [[1, 0], [0, 1]]; let sum: M2 = [[1, 0], [0, 1]]; for (let k = 1; k <= 12; k++) { term = mulM(term, C); term = [[term[0][0] / k, term[0][1] / k], [term[1][0] / k, term[1][1] / k]]; sum = [[sum[0][0] + term[0][0], sum[0][1] + term[0][1]], [sum[1][0] + term[1][0], sum[1][1] + term[1][1]]]; } for (let k = 0; k < s; k++) sum = mulM(sum, sum); return sum; }
export const eigReal2 = (A: M2): [number, number] => { const tr = A[0][0] + A[1][1], det = A[0][0] * A[1][1] - A[0][1] * A[1][0]; const d = (tr * tr) / 4 - det; return d >= 0 ? [tr / 2 - Math.sqrt(d), tr / 2 + Math.sqrt(d)] : [tr / 2, tr / 2]; };
/** Does P = I (or a grid search over P > 0) give a common quadratic Lyapunov function AᵀP + PA < 0 for all modes? */
export function commonLyapunov(modes: M2[]): { exists: boolean; P?: M2 } {
  const negDef = (M: M2) => { const S: M2 = [[M[0][0], (M[0][1] + M[1][0]) / 2], [(M[0][1] + M[1][0]) / 2, M[1][1]]]; return S[0][0] < -1e-9 && S[0][0] * S[1][1] - S[0][1] * S[1][0] > 1e-12; };
  for (let a = 0.2; a <= 5; a += 0.2) for (let b = -3; b <= 3; b += 0.2) for (let c = 0.2; c <= 5; c += 0.2) { if (a * c - b * b <= 0) continue; const P: M2 = [[a, b], [b, c]]; if (modes.every((A) => negDef([[A[0][0] * P[0][0] + A[1][0] * P[1][0] + P[0][0] * A[0][0] + P[0][1] * A[1][0], A[0][0] * P[0][1] + A[1][0] * P[1][1] + P[0][0] * A[0][1] + P[0][1] * A[1][1]], [A[0][1] * P[0][0] + A[1][1] * P[1][0] + P[1][0] * A[0][0] + P[1][1] * A[1][0], A[0][1] * P[0][1] + A[1][1] * P[1][1] + P[1][0] * A[0][1] + P[1][1] * A[1][1]]]))) return { exists: true, P }; }
  return { exists: false };
}
/** Simulate a switched system with a periodic switching signal of dwell τ; returns the growth of ‖x‖ per period and the worst dwell found in a sweep. */
export function switchedGrowth(modes: M2[], tau: number, periods = 20, x0: [number, number] = [1, 0]): { growthPerCycle: number; stable: boolean; norms: number[] } {
  let x = x0.slice() as [number, number]; const norms = [Math.hypot(...x)]; const Phis = modes.map((A) => expm2(A, tau));
  for (let p = 0; p < periods; p++) { for (const Phi of Phis) x = [Phi[0][0] * x[0] + Phi[0][1] * x[1], Phi[1][0] * x[0] + Phi[1][1] * x[1]]; norms.push(Math.hypot(...x)); }
  const growth = (norms[norms.length - 1] / norms[0]) ** (1 / periods); return { growthPerCycle: growth, stable: growth < 1, norms };
}
/** Minimum dwell time found by sweeping τ ∈ [tauMin, tauMax]: the smallest τ from which every larger τ in the sweep is stable. */
export function dwellTimeSweep(modes: M2[], tauMin = 0.05, tauMax = 5, n = 100): { table: Array<{ tau: number; growth: number }>; dwellTime: number | null; unstableFor: number[] } {
  const table: Array<{ tau: number; growth: number }> = []; for (let k = 0; k <= n; k++) { const tau = tauMin + ((tauMax - tauMin) * k) / n; table.push({ tau, growth: switchedGrowth(modes, tau).growthPerCycle }); }
  let dwell: number | null = null; for (let k = table.length - 1; k >= 0; k--) { if (table[k].growth < 1) dwell = table[k].tau; else break; }
  return { table, dwellTime: dwell, unstableFor: table.filter((r) => r.growth >= 1).map((r) => r.tau) };
}
