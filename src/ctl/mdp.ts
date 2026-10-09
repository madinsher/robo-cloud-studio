/**
 * Decisions under uncertainty (course chapter 12): MDPs (value iteration with the ε-optimality stopping rule,
 * policy iteration), sensitivity scans, POMDPs (belief update, expected reward, QMDP, exact finite-horizon
 * α-vector backups with pointwise pruning, decision thresholds) and the shield for learned / heuristic policies.
 */
import { solve } from './perf';

export interface MDPSpec {
  states: string[];
  actions: string[];
  /** transitions[s][a] = [{ to, p }]; missing action = not allowed in s */
  transitions: Record<string, Record<string, Array<{ to: string; p: number }>>>;
  /** reward (or cost when `minimize`) for taking a in s; optional per-outcome rewards via `to` */
  reward: Record<string, Record<string, number>>;
  gamma: number;
  terminal?: string[];
  minimize?: boolean;
}

export interface MDPResult { values: Record<string, number>; policy: Record<string, string>; iterations: number; qValues: Record<string, Record<string, number>> }

const better = (m: MDPSpec, a: number, b: number) => (m.minimize ? a < b : a > b);

function qValue(m: MDPSpec, V: Record<string, number>, s: string, a: string): number {
  const tr = m.transitions[s]?.[a]; if (!tr) return m.minimize ? Infinity : -Infinity;
  return (m.reward[s]?.[a] ?? 0) + m.gamma * tr.reduce((sum, t) => sum + t.p * (V[t.to] ?? 0), 0);
}

/** Value iteration; stops when ‖V' − V‖ < ε(1−γ)/(2γ) (Proposition 12.1) or after maxIter. */
export function valueIteration(m: MDPSpec, opts: { epsilon?: number; maxIter?: number } = {}): MDPResult {
  const eps = opts.epsilon ?? 1e-6; const maxIter = opts.maxIter ?? 100000;
  const term = new Set(m.terminal ?? []);
  let V: Record<string, number> = Object.fromEntries(m.states.map((s) => [s, term.has(s) ? m.reward[s]?.['*'] ?? 0 : 0]));
  let it = 0;
  for (; it < maxIter; it++) {
    const Vn: Record<string, number> = { ...V }; let delta = 0;
    for (const s of m.states) {
      if (term.has(s)) { Vn[s] = m.reward[s]?.['*'] ?? 0; continue; }
      const acts = Object.keys(m.transitions[s] ?? {}); if (!acts.length) continue;
      let best = m.minimize ? Infinity : -Infinity;
      for (const a of acts) { const q = qValue(m, V, s, a); if (better(m, q, best)) best = q; }
      Vn[s] = best; delta = Math.max(delta, Math.abs(best - V[s]));
    }
    V = Vn;
    const thr = m.gamma < 1 ? (eps * (1 - m.gamma)) / (2 * m.gamma) : eps;
    if (delta < thr) { it++; break; }
  }
  return extract(m, V, it);
}

function extract(m: MDPSpec, V: Record<string, number>, iterations: number): MDPResult {
  const policy: Record<string, string> = {}; const qValues: Record<string, Record<string, number>> = {};
  const term = new Set(m.terminal ?? []);
  for (const s of m.states) {
    if (term.has(s)) continue;
    const acts = Object.keys(m.transitions[s] ?? {}); qValues[s] = {};
    let bestA = '', best = m.minimize ? Infinity : -Infinity;
    for (const a of acts) { const q = qValue(m, V, s, a); qValues[s][a] = q; if (better(m, q, best)) { best = q; bestA = a; } }
    if (bestA) policy[s] = bestA;
  }
  return { values: V, policy, iterations, qValues };
}

/** Policy iteration: exact policy evaluation (linear system) + greedy improvement; converges in few iterations (Theorem 12.3). */
export function policyIteration(m: MDPSpec, opts: { maxIter?: number } = {}): MDPResult {
  const term = new Set(m.terminal ?? []);
  const nonTerm = m.states.filter((s) => !term.has(s) && Object.keys(m.transitions[s] ?? {}).length);
  let policy: Record<string, string> = Object.fromEntries(nonTerm.map((s) => [s, Object.keys(m.transitions[s])[0]]));
  let V: Record<string, number> = Object.fromEntries(m.states.map((s) => [s, term.has(s) ? m.reward[s]?.['*'] ?? 0 : 0]));
  let it = 0;
  for (; it < (opts.maxIter ?? 100); it++) {
    // evaluate: V = R_π + γ T_π V  →  (I − γ T_π) V = R_π
    const n = nonTerm.length; const idx = new Map(nonTerm.map((s, i) => [s, i]));
    const A = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)));
    const b = new Array(n).fill(0);
    nonTerm.forEach((s, i) => { const a = policy[s]; b[i] = m.reward[s]?.[a] ?? 0; for (const t of m.transitions[s][a]) { const j = idx.get(t.to); if (j !== undefined) A[i][j] -= m.gamma * t.p; else b[i] += m.gamma * t.p * (V[t.to] ?? 0); } });
    const x = solve(A, b) ?? new Array(n).fill(0);
    nonTerm.forEach((s, i) => { V[s] = x[i]; });
    const improved = extract(m, V, it).policy;
    if (nonTerm.every((s) => improved[s] === policy[s])) { it++; break; }
    policy = improved;
  }
  return extract(m, V, it);
}

/** Scan a parameter and report where the optimal action in `state` changes (sensitivity analysis, §12.2.5). */
export function sensitivityScan(build: (p: number) => MDPSpec, state: string, range: [number, number], steps = 50): Array<{ p: number; action: string; value: number }> {
  const out: Array<{ p: number; action: string; value: number }> = [];
  for (let i = 0; i <= steps; i++) { const p = range[0] + ((range[1] - range[0]) * i) / steps; const r = valueIteration(build(p)); out.push({ p, action: r.policy[state], value: r.values[state] }); }
  return out;
}
export function switchPoints(scan: Array<{ p: number; action: string }>): Array<{ p: number; from: string; to: string }> { const out: Array<{ p: number; from: string; to: string }> = []; for (let i = 1; i < scan.length; i++) if (scan[i].action !== scan[i - 1].action) out.push({ p: scan[i].p, from: scan[i - 1].action, to: scan[i].action }); return out; }

/** Course §12.2.5 grasp-strategy MDP (minimise expected time; abort penalty 60 s). */
export function graspStrategyMDP(pDirect = 0.55, refineTime = 3, abortPenalty = 60): MDPSpec {
  return {
    states: ['s0', 's1', 's2', 's3', 's4'], actions: ['refine', 'grasp', 'abort'], gamma: 1, minimize: true, terminal: ['s2', 's4'],
    transitions: {
      s0: { refine: [{ to: 's1', p: 1 }], grasp: [{ to: 's2', p: pDirect }, { to: 's3', p: 1 - pDirect }], abort: [{ to: 's4', p: 1 }] },
      s1: { grasp: [{ to: 's2', p: 0.88 }, { to: 's3', p: 0.12 }], abort: [{ to: 's4', p: 1 }] },
      s3: { refine: [{ to: 's1', p: 0.8 }, { to: 's3', p: 0.2 }], abort: [{ to: 's4', p: 1 }] },
    },
    reward: { s0: { refine: refineTime, grasp: 8, abort: 0 }, s1: { grasp: 8, abort: 0 }, s3: { refine: 5, abort: 0 }, s2: { '*': 0 }, s4: { '*': abortPenalty } },
  };
}

// ---------------------------------------------------------------------------------------------
// POMDP
// ---------------------------------------------------------------------------------------------

export interface POMDPSpec {
  states: string[]; actions: string[]; observations: string[];
  /** T[s][a] = [{to, p}] (missing → stays) */
  transitions: Record<string, Record<string, Array<{ to: string; p: number }>>>;
  /** O[a][s'][o] = P(o | s', a) (missing → uninformative) */
  observation: Record<string, Record<string, Record<string, number>>>;
  /** R[s][a] */
  reward: Record<string, Record<string, number>>;
  gamma: number;
  /** actions that end the episode */
  terminalActions?: string[];
}
export type Belief = Record<string, number>;

export function beliefUpdate(m: POMDPSpec, b: Belief, a: string, o: string): { belief: Belief; pObs: number } {
  const pred: Belief = {};
  for (const s of m.states) { const tr = m.transitions[s]?.[a] ?? [{ to: s, p: 1 }]; for (const t of tr) pred[t.to] = (pred[t.to] ?? 0) + t.p * (b[s] ?? 0); }
  let norm = 0; const out: Belief = {};
  for (const s of m.states) { const po = m.observation[a]?.[s]?.[o] ?? 1 / m.observations.length; out[s] = po * (pred[s] ?? 0); norm += out[s]; }
  for (const s of m.states) out[s] = norm ? out[s] / norm : 1 / m.states.length;
  return { belief: out, pObs: norm };
}
export function expectedReward(m: POMDPSpec, b: Belief, a: string): number { return m.states.reduce((s, x) => s + (b[x] ?? 0) * (m.reward[x]?.[a] ?? 0), 0); }
export function pObservation(m: POMDPSpec, b: Belief, a: string, o: string): number { return beliefUpdate(m, b, a, o).pObs; }

/** One-step lookahead value of an information action followed by the best immediate action (course §12.4.2 calculation). */
export function lookaheadValue(m: POMDPSpec, b: Belief, a: string, decide: (b2: Belief) => { action: string; value: number }): { value: number; branches: Array<{ o: string; p: number; belief: Belief; next: string; value: number }> } {
  let v = expectedReward(m, b, a); const branches = [];
  for (const o of m.observations) { const { belief, pObs } = beliefUpdate(m, b, a, o); if (pObs <= 0) continue; const d = decide(belief); v += m.gamma * pObs * d.value; branches.push({ o, p: pObs, belief, next: d.action, value: d.value }); }
  return { value: v, branches };
}
/** Best terminal (episode-ending) action for a belief. */
export function bestTerminalAction(m: POMDPSpec, b: Belief): { action: string; value: number } {
  let best = { action: '', value: -Infinity };
  for (const a of m.terminalActions ?? m.actions) { const v = expectedReward(m, b, a); if (v > best.value) best = { action: a, value: v }; }
  return best;
}

/** QMDP: solve the underlying MDP, then Q(b,a) = Σ b(s) Q*(s,a). Never chooses information-gathering actions. */
export function qmdp(m: POMDPSpec, b: Belief): { action: string; q: Record<string, number> } {
  const mdp: MDPSpec = { states: m.states, actions: m.actions, transitions: {}, reward: m.reward, gamma: m.gamma, terminal: [] };
  for (const s of m.states) { mdp.transitions[s] = {}; for (const a of m.actions) mdp.transitions[s][a] = (m.terminalActions ?? []).includes(a) ? [] : m.transitions[s]?.[a] ?? [{ to: s, p: 1 }]; }
  const r = valueIteration(mdp, { maxIter: 500 });
  const q: Record<string, number> = {};
  for (const a of m.actions) q[a] = m.states.reduce((sum, s) => sum + (b[s] ?? 0) * (r.qValues[s]?.[a] ?? m.reward[s]?.[a] ?? 0), 0);
  const action = m.actions.reduce((x, y) => (q[y] > q[x] ? y : x));
  return { action, q };
}

/** Exact finite-horizon value iteration with α-vectors (Theorem 12.5) and pointwise-dominance pruning; returns V(b) and the best first action. */
export function alphaVectorBackup(m: POMDPSpec, horizon: number, maxVectors = 400): { vectors: Array<{ action: string; alpha: number[] }>; value: (b: Belief) => { value: number; action: string } } {
  const S = m.states; const idx = new Map(S.map((s, i) => [s, i]));
  let Gamma: Array<{ action: string; alpha: number[] }> = [{ action: '', alpha: S.map(() => 0) }];
  const term = new Set(m.terminalActions ?? []);
  for (let h = 0; h < horizon; h++) {
    const next: Array<{ action: string; alpha: number[] }> = [];
    for (const a of m.actions) {
      const Ra = S.map((s) => m.reward[s]?.[a] ?? 0);
      if (term.has(a)) { next.push({ action: a, alpha: Ra }); continue; }
      // g_{a,o,α}(s) = Σ_{s'} α(s') O(s',a,o) T(s,a,s')
      const gs = m.observations.map((o) => Gamma.map((g) => S.map((s) => { const tr = m.transitions[s]?.[a] ?? [{ to: s, p: 1 }]; return tr.reduce((sum, t) => sum + t.p * (m.observation[a]?.[t.to]?.[o] ?? 1 / m.observations.length) * g.alpha[idx.get(t.to)!], 0); })));
      // enumerate combinations (choice of α per observation) — prune by cross-sum with dominance
      let combos: number[][] = [Ra];
      for (let oi = 0; oi < m.observations.length; oi++) {
        const cand: number[][] = [];
        for (const c of combos) for (const g of gs[oi]) cand.push(c.map((v, i) => v + m.gamma * g[i]));
        combos = prune(cand, maxVectors);
      }
      for (const c of combos) next.push({ action: a, alpha: c });
    }
    const pruned = prune(next.map((v) => v.alpha), maxVectors);
    Gamma = pruned.map((alpha) => next.find((v) => v.alpha === alpha)!);
  }
  return { vectors: Gamma, value: (b) => { let best = { value: -Infinity, action: '' }; for (const g of Gamma) { const v = S.reduce((sum, s, i) => sum + (b[s] ?? 0) * g.alpha[i], 0); if (v > best.value) best = { value: v, action: g.action }; } return best; } };
}
function prune(vs: number[][], max: number): number[][] {
  const out: number[][] = [];
  for (const v of vs) { if (out.some((w) => w.every((x, i) => x >= v[i] - 1e-12))) continue; for (let i = out.length - 1; i >= 0; i--) if (v.every((x, k) => x >= out[i][k] - 1e-12)) out.splice(i, 1); out.push(v); }
  if (out.length > max) { out.sort((a, b) => b.reduce((s, x) => s + x, 0) - a.reduce((s, x) => s + x, 0)); return out.slice(0, max); }
  return out;
}

/** Confidence threshold: minimal belief β in the target class at which acting now beats "observe, then act" (§12.4.3 formula: 20β − 50(1−β) ≥ −c + 20). */
export function decisionThreshold(rewardCorrect: number, penaltyWrong: number, observeCost: number, postObserveValue = rewardCorrect): number { return (postObserveValue - observeCost + penaltyWrong) / (rewardCorrect + penaltyWrong); }

/** Course §12.4 classification POMDP. */
export function classificationPOMDP(penaltyWrong = 50): POMDPSpec {
  const far = { bolt: { obs_bolt: 0.7, obs_nut: 0.2, obs_other: 0.1 }, nut: { obs_bolt: 0.2, obs_nut: 0.7, obs_other: 0.1 }, other: { obs_bolt: 0.15, obs_nut: 0.15, obs_other: 0.7 } };
  const near = { bolt: { obs_bolt: 0.94, obs_nut: 0.04, obs_other: 0.02 }, nut: { obs_bolt: 0.04, obs_nut: 0.94, obs_other: 0.02 }, other: { obs_bolt: 0.03, obs_nut: 0.03, obs_other: 0.94 } };
  return {
    states: ['bolt', 'nut', 'other'], actions: ['look', 'look_closer', 'place_bolt', 'place_nut', 'skip'], observations: ['obs_bolt', 'obs_nut', 'obs_other'],
    transitions: {}, observation: { look: far, look_closer: near }, gamma: 1, terminalActions: ['place_bolt', 'place_nut', 'skip'],
    reward: { bolt: { look: -0.5, look_closer: -4, place_bolt: 20, place_nut: -penaltyWrong, skip: -10 }, nut: { look: -0.5, look_closer: -4, place_bolt: -penaltyWrong, place_nut: 20, skip: -10 }, other: { look: -0.5, look_closer: -4, place_bolt: -penaltyWrong, place_nut: -penaltyWrong, skip: 0 } },
  };
}

// ---------------------------------------------------------------------------------------------
// Shield
// ---------------------------------------------------------------------------------------------

/** a = π(s) if a ∈ Safe(s) else π_safe(s); counts overrides (§12.6.4). */
export class Shield<S, A> {
  overrides = 0; decisions = 0;
  readonly log: Array<{ state: S; proposed: A; applied: A }> = [];
  constructor(readonly safe: (s: S) => Set<A> | A[], readonly fallback: (s: S) => A) {}
  apply(s: S, proposed: A): A {
    this.decisions++;
    const allowed = this.safe(s); const ok = allowed instanceof Set ? allowed.has(proposed) : allowed.includes(proposed);
    const applied = ok ? proposed : this.fallback(s);
    if (!ok) { this.overrides++; if (this.log.length < 1000) this.log.push({ state: s, proposed, applied }); }
    return applied;
  }
}
