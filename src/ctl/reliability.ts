/**
 * Reliability and functional safety (course chapter 16): reliability functions (exponential, Weibull), series /
 * parallel systems and availability, contribution (Pareto) analysis, Markov availability models with repair, FMEA
 * with the AIAG/VDA action priority instead of RPN, fault trees with minimal cut sets (MOCUS) and probabilities,
 * FDIR thresholds, degradation matrices, ISO 12100 risk graph → ISO 13849 PLr, achieved PL from category / MTTFd /
 * DC, ISO/TS 15066 speed-and-separation monitoring distance and admissible speed, IEC 60204-1 stop categories.
 */
import { steadyState } from './perf';

// ---------------------------------------------------------------------------------------------
// Reliability functions
// ---------------------------------------------------------------------------------------------

export const reliabilityExp = (lambda: number, t: number): number => Math.exp(-lambda * t);
export const reliabilityWeibull = (eta: number, beta: number, t: number): number => Math.exp(-Math.pow(t / eta, beta));
export const hazardWeibull = (eta: number, beta: number, t: number): number => (beta / eta) * Math.pow(t / eta, beta - 1);
export function mttfWeibull(eta: number, beta: number): number { return eta * gamma(1 + 1 / beta); }
function gamma(z: number): number { // Lanczos
  const g = 7, c = [0.99999999999980993, 676.5203681218851, -1259.1392167224028, 771.32342877765313, -176.61502916214059, 12.507343278686905, -0.13857109526572012, 9.9843695780195716e-6, 1.5056327351493116e-7];
  if (z < 0.5) return Math.PI / (Math.sin(Math.PI * z) * gamma(1 - z));
  z -= 1; let x = c[0]; for (let i = 1; i < g + 2; i++) x += c[i] / (z + i); const t = z + g + 0.5;
  return Math.sqrt(2 * Math.PI) * Math.pow(t, z + 0.5) * Math.exp(-t) * x;
}

/** Optimal preventive replacement interval for a Weibull component (age replacement): minimise cost rate (cp + cf·F(T)) / ∫0^T R(t)dt. */
export function optimalReplacement(eta: number, beta: number, costPlanned: number, costFailure: number): { interval: number; costRate: number; runToFailureRate: number } {
  let best = { interval: Infinity, costRate: Infinity }; const mttf = mttfWeibull(eta, beta);
  for (let T = eta * 0.05; T <= eta * 3; T += eta * 0.01) {
    let integral = 0; const n = 200; for (let i = 0; i < n; i++) integral += reliabilityWeibull(eta, beta, (T * (i + 0.5)) / n) * (T / n);
    const rate = (costPlanned + costFailure * (1 - reliabilityWeibull(eta, beta, T))) / integral;
    if (rate < best.costRate) best = { interval: T, costRate: rate };
  }
  return { ...best, runToFailureRate: costFailure / mttf };
}

export interface Component { id: string; lambda: number; /** parallel redundancy count (hot) */ redundancy?: number }

/** Series system of (optionally redundant) components: λ_sys, MTTF, availability and each component's contribution. */
export function systemReliability(components: Component[], mttrHours = 4): { lambda: number; mttf: number; availability: number; contributions: Array<{ id: string; lambda: number; share: number }>; downtimePerWeekMin: number } {
  const eff = components.map((c) => { const k = c.redundancy ?? 1; const l = k === 1 ? c.lambda : c.lambda / (k === 2 ? 1.5 : k === 3 ? 1.8333 : k); return { id: c.id, lambda: l }; });
  const lambda = eff.reduce((s, c) => s + c.lambda, 0); const mttf = 1 / lambda; const availability = mttf / (mttf + mttrHours);
  const contributions = eff.map((c) => ({ id: c.id, lambda: c.lambda, share: c.lambda / lambda })).sort((a, b) => b.share - a.share);
  return { lambda, mttf, availability, contributions, downtimePerWeekMin: (1 - availability) * 80 * 60 };
}

/** Availability of a 3-state Markov model (ok → degraded → failed, repairs back to ok) — steady state and comparison with run-to-failure. */
export function markovAvailability(rates: { degrade: number; fail: number; failDirect?: number; repairFromDegraded?: number; repairFromFailed: number }): { pi: { ok: number; degraded: number; failed: number }; availability: number } {
  const Q = [
    [-(rates.degrade + (rates.failDirect ?? 0)), rates.degrade, rates.failDirect ?? 0],
    [rates.repairFromDegraded ?? 0, -((rates.repairFromDegraded ?? 0) + rates.fail), rates.fail],
    [rates.repairFromFailed, 0, -rates.repairFromFailed],
  ];
  const pi = steadyState(Q);
  return { pi: { ok: pi[0], degraded: pi[1], failed: pi[2] }, availability: pi[0] + pi[1] };
}

// ---------------------------------------------------------------------------------------------
// FMEA
// ---------------------------------------------------------------------------------------------

export interface FmeaRow { element: string; failureMode: string; cause?: string; localEffect?: string; systemEffect: string; detection: string; S: number; O: number; D: number; measure: string; residualRisk?: string }

/** AIAG/VDA (2019) action priority: severity first; H = act, M = should act, L = may act. */
export function actionPriority(S: number, O: number, D: number): 'H' | 'M' | 'L' {
  if (S >= 9) return O >= 2 ? 'H' : D >= 5 ? 'M' : 'L';
  if (S >= 7) { if (O >= 6) return 'H'; if (O >= 4) return D >= 5 ? 'H' : 'M'; if (O >= 2) return D >= 7 ? 'H' : 'M'; return 'L'; }
  if (S >= 4) { if (O >= 8) return D >= 5 ? 'H' : 'M'; if (O >= 6) return D >= 7 ? 'H' : 'M'; if (O >= 4) return D >= 7 ? 'M' : 'L'; return 'L'; }
  if (S >= 2) { if (O >= 8) return D >= 7 ? 'M' : 'L'; return 'L'; }
  return 'L';
}
export function fmeaTable(rows: FmeaRow[]): Array<FmeaRow & { rpn: number; ap: 'H' | 'M' | 'L'; mandatory: boolean }> {
  return rows.map((r) => ({ ...r, rpn: r.S * r.O * r.D, ap: actionPriority(r.S, r.O, r.D), mandatory: r.S >= 9 })).sort((a, b) => (a.ap === b.ap ? b.S - a.S || b.rpn - a.rpn : a.ap < b.ap ? -1 : 1));
}

// ---------------------------------------------------------------------------------------------
// Fault trees
// ---------------------------------------------------------------------------------------------

export type FTNode = { kind: 'event'; id: string; p: number; name?: string } | { kind: 'and' | 'or'; id: string; name?: string; children: FTNode[] };

/** Minimal cut sets by MOCUS (top-down expansion, absorption). */
export function minimalCutSets(top: FTNode): string[][] {
  const expand = (n: FTNode): string[][] => {
    if (n.kind === 'event') return [[n.id]];
    if (n.kind === 'or') return n.children.flatMap(expand);
    // and: cross product
    let sets: string[][] = [[]];
    for (const c of n.children) { const cs = expand(c); const next: string[][] = []; for (const a of sets) for (const b of cs) next.push([...new Set([...a, ...b])]); sets = next; }
    return sets;
  };
  const all = expand(top).map((s) => [...s].sort());
  const uniq = all.filter((s, i) => all.findIndex((x) => x.join() === s.join()) === i);
  return uniq.filter((s) => !uniq.some((o) => o !== s && o.length < s.length && o.every((e) => s.includes(e)))).sort((a, b) => a.length - b.length);
}

/** Top-event probability: rare-event bound Σ Π p, exact by inclusion–exclusion for ≤ 12 cut sets. */
export function topEventProbability(top: FTNode): { rareEvent: number; exact: number | null; cutSets: Array<{ events: string[]; p: number }>; minOrder: number; dominant: string[] } {
  const probs = new Map<string, number>(); const collect = (n: FTNode) => { if (n.kind === 'event') probs.set(n.id, n.p); else n.children.forEach(collect); }; collect(top);
  const cs = minimalCutSets(top).map((events) => ({ events, p: events.reduce((s, e) => s * (probs.get(e) ?? 0), 1) }));
  const rare = cs.reduce((s, c) => s + c.p, 0);
  let exact: number | null = null;
  if (cs.length <= 12) { exact = 0; for (let mask = 1; mask < 1 << cs.length; mask++) { const union = new Set<string>(); let bits = 0; for (let i = 0; i < cs.length; i++) if (mask & (1 << i)) { bits++; cs[i].events.forEach((e) => union.add(e)); } const p = [...union].reduce((s, e) => s * (probs.get(e) ?? 0), 1); exact += (bits % 2 ? 1 : -1) * p; } }
  const dominant = cs.length ? cs.reduce((a, b) => (b.p > a.p ? b : a)).events : [];
  return { rareEvent: rare, exact, cutSets: cs, minOrder: cs.length ? cs[0].events.length : 0, dominant };
}

/** Sensitivity: relative change of the top probability when each basic event improves by a factor of 10. */
export function ftaSensitivity(top: FTNode): Array<{ event: string; gain: number }> {
  const base = topEventProbability(top).rareEvent; const ids: string[] = []; const walk = (n: FTNode) => { if (n.kind === 'event') ids.push(n.id); else n.children.forEach(walk); }; walk(top);
  const patch = (n: FTNode, id: string): FTNode => (n.kind === 'event' ? (n.id === id ? { ...n, p: n.p / 10 } : n) : { ...n, children: n.children.map((c) => patch(c, id)) });
  const probs = new Map<string, number>(); const collect = (n: FTNode) => { if (n.kind === 'event') probs.set(n.id, n.p); else n.children.forEach(collect); }; collect(top);
  return [...new Set(ids)].filter((id) => (probs.get(id) ?? 1) < 1).map((id) => ({ event: id, gain: base ? 1 - topEventProbability(patch(top, id)).rareEvent / base : 0 })).sort((a, b) => b.gain - a.gain);
}

// ---------------------------------------------------------------------------------------------
// FDIR
// ---------------------------------------------------------------------------------------------

/** Detection threshold design (§16.6.2): false-alarm probability per check for a k-sigma threshold and the k-of-k confirmation alternative. */
export function detectionThreshold(v: { checkRateHz: number; maxFalseAlarmsPerDay: number; sigma?: number }): { pPerCheck: number; kSigma: number; confirmSteps3Sigma: number; delayMs: number } {
  const pPerCheck = v.maxFalseAlarmsPerDay / (v.checkRateHz * 86400);
  const kSigma = inverseTail(pPerCheck);
  const p3 = 0.0027; const confirmSteps3Sigma = Math.ceil(Math.log(pPerCheck) / Math.log(p3));
  return { pPerCheck, kSigma, confirmSteps3Sigma, delayMs: (confirmSteps3Sigma * 1000) / v.checkRateHz };
}
/** two-sided Gaussian tail: find k with P(|z| > k) = p */
function inverseTail(p: number): number { let lo = 0, hi = 10; for (let i = 0; i < 60; i++) { const mid = (lo + hi) / 2; if (2 * (1 - phi(mid)) > p) lo = mid; else hi = mid; } return (lo + hi) / 2; }
function phi(x: number): number { const t = 1 / (1 + 0.2316419 * Math.abs(x)); const d = 0.3989423 * Math.exp(-x * x / 2); const p = d * t * (0.3193815 + t * (-0.3565638 + t * (1.781478 + t * (-1.821256 + t * 1.330274)))); return x > 0 ? 1 - p : p; }

export interface DegradationRow { failure: string; mode: 'degraded' | 'limited' | 'safe_stop' | 'line_stop' | 'full_stop' | 'autonomous'; functions: string; returnCondition: string; test?: string }

// ---------------------------------------------------------------------------------------------
// Functional safety: ISO 13849 / ISO 12100 / ISO-TS 15066 / IEC 60204-1
// ---------------------------------------------------------------------------------------------

export type PL = 'a' | 'b' | 'c' | 'd' | 'e';
/** Risk graph ISO 13849-1 Annex A. */
export function requiredPL(S: 1 | 2, F: 1 | 2, P: 1 | 2): PL {
  if (S === 1) return F === 1 ? (P === 1 ? 'a' : 'b') : P === 1 ? 'b' : 'c';
  return F === 1 ? (P === 1 ? 'c' : 'd') : P === 1 ? 'd' : 'e';
}
export const PFH_RANGE: Record<PL, [number, number]> = { a: [1e-5, 1e-4], b: [3e-6, 1e-5], c: [1e-6, 3e-6], d: [1e-7, 1e-6], e: [1e-8, 1e-7] };

export type Category = 'B' | '1' | '2' | '3' | '4';
export type MTTFdClass = 'low' | 'medium' | 'high';
export type DCClass = 'none' | 'low' | 'medium' | 'high';
export function mttfdClass(years: number): MTTFdClass | null { if (years < 3) return null; return years < 10 ? 'low' : years < 30 ? 'medium' : 'high'; }
export function dcClass(dc: number): DCClass { return dc < 0.6 ? 'none' : dc < 0.9 ? 'low' : dc < 0.99 ? 'medium' : 'high'; }

/** Simplified ISO 13849-1 Table 7 (achievable PL from category, MTTFd class, DC class); requires CCF ≥ 65 points for cat. 2–4. */
export function achievedPL(cat: Category, mttfd: MTTFdClass, dc: DCClass, ccfPoints = 65): { pl: PL | null; note: string } {
  const key = `${cat}/${dc}`;
  const table: Record<string, Record<MTTFdClass, PL | null>> = {
    'B/none': { low: 'a', medium: 'b', high: 'b' }, '1/none': { low: null, medium: null, high: 'c' },
    '2/low': { low: 'a', medium: 'b', high: 'c' }, '2/medium': { low: 'b', medium: 'c', high: 'd' },
    '3/low': { low: 'b', medium: 'c', high: 'd' }, '3/medium': { low: 'c', medium: 'd', high: 'd' }, '4/high': { low: null, medium: null, high: 'e' },
  };
  if (cat !== 'B' && cat !== '1' && ccfPoints < 65) return { pl: null, note: 'CCF measures below 65 points: category 2–4 not valid' };
  const row = table[key];
  if (!row) return { pl: null, note: `combination category ${cat} with DC ${dc} is not a valid designated architecture` };
  const pl = row[mttfd];
  return { pl, note: pl ? `category ${cat}, MTTFd ${mttfd}, DC ${dc} → PL ${pl}` : 'combination not allowed by ISO 13849-1' };
}
export function plMeets(achieved: PL | null, required: PL): boolean { return !!achieved && achieved >= required; }

/** Which components count towards PL (§16.7.3). */
export function countsTowardsPL(component: 'safety_plc' | 'safety_scanner' | 'plc' | 'ml_perception' | 'cbf_filter' | 'monitor' | 'behavior_tree' | 'hardware_chain'): boolean { return ['safety_plc', 'safety_scanner', 'hardware_chain'].includes(component); }

/** ISO/TS 15066 SSM protective separation distance S_p = S_h + S_r + S_s + C + Z_d + Z_r (§16.8.2). */
export function separationDistance(v: { vRobot: number; tReaction: number; tStop: number; vHuman?: number; C?: number; Zd?: number; Zr?: number }): { Sp: number; Sh: number; Sr: number; Ss: number } {
  const vh = v.vHuman ?? 1.6; const Sh = vh * (v.tReaction + v.tStop), Sr = v.vRobot * v.tReaction, Ss = (v.vRobot * v.tStop) / 2;
  return { Sp: Sh + Sr + Ss + (v.C ?? 0.2) + (v.Zd ?? 0) + (v.Zr ?? 0), Sh, Sr, Ss };
}
/** Admissible robot speed for an available distance with constant deceleration a: solves the quadratic of §16.8.2. */
export function admissibleSpeed(v: { distance: number; tReaction: number; decel: number; vHuman?: number; C?: number; Zd?: number; Zr?: number }): number {
  const vh = v.vHuman ?? 1.6; const fixed = vh * v.tReaction + (v.C ?? 0.2) + (v.Zd ?? 0) + (v.Zr ?? 0);
  // distance = fixed + vh·v/a + v·tR + v²/(2a)
  const a2 = 1 / (2 * v.decel), a1 = vh / v.decel + v.tReaction, a0 = fixed - v.distance;
  if (a0 >= 0) return 0;
  return (-a1 + Math.sqrt(a1 * a1 - 4 * a2 * a0)) / (2 * a2);
}
export function stopCategory(v: { carriesLoad: boolean; collaborative: boolean }): { category: 0 | 1 | 2; rationale: string } {
  if (v.collaborative) return { category: 2, rationale: 'safety-rated monitored stop keeps power for a fast restart' };
  if (v.carriesLoad) return { category: 1, rationale: 'controlled stop first — removing power drops the load and loses the braking trajectory' };
  return { category: 0, rationale: 'immediate power removal is acceptable without a load' };
}

/** Course §16.3.3 component budget for the production cell. */
export const CELL_COMPONENTS: Component[] = [
  { id: 'M1', lambda: 2e-4 }, { id: 'M2', lambda: 2e-4 }, { id: 'S', lambda: 5e-4 }, { id: 'conveyor 1', lambda: 1e-4 }, { id: 'conveyor 2', lambda: 1e-4 }, { id: 'controller', lambda: 5e-5 }, { id: 'network', lambda: 3e-5 }, { id: 'AMR', lambda: 4e-4 },
];
/** Course §16.5.3 fault tree "robot contacts a human with unacceptable force". */
export const CONTACT_FTA: FTNode = {
  kind: 'and', id: 'top', name: 'unacceptable contact', children: [
    { kind: 'or', id: 'undetected', name: 'human in zone and not detected in time', children: [{ kind: 'event', id: 'A', p: 1e-5, name: 'scanner failure' }, { kind: 'event', id: 'B', p: 1e-2, name: 'human outside the field of view' }, { kind: 'event', id: 'C', p: 1e-3, name: 'processing delay' }] },
    { kind: 'and', id: 'moving', name: 'robot moves at a dangerous speed', children: [{ kind: 'event', id: 'D', p: 1, name: 'motion command' }, { kind: 'and', id: 'protect', name: 'protective layer failed', children: [{ kind: 'event', id: 'E', p: 1e-3, name: 'CBF filter failure' }, { kind: 'event', id: 'F', p: 1e-7, name: 'hardware chain failure' }] }] },
  ],
};
