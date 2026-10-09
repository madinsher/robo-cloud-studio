/**
 * Petri nets — structural and behavioural analysis, deadlock policies, timed / stochastic evaluation.
 *
 * Course chapters 4–5: incidence matrix and state equation, P/T-invariants (Farkas), reachability graph
 * (boundedness, liveness, reversibility, deadlock markings with traces), siphons and traps (minimal siphons by
 * branch-and-bound extension, Commoner's condition), S³PR resource nets with bad-siphon detection, GMEC monitor
 * places (Giua–DiCesare–Silva: C_V = −lᵀC, M0(V) = β − lᵀM0), iterative deadlock prevention, banker's algorithm
 * (Algorithm 4.2), resource ordering check, deterministic T-timed simulation (three-phase firing), GSPN → CTMC with
 * vanishing-state elimination and steady-state indices.
 */
import { stronglyConnected } from './graph_util';
import { steadyState } from './perf';

export type PlaceKind = 'idle' | 'activity' | 'resource' | 'monitor' | 'buffer' | 'other';
export interface PlaceSpec { id: string; tokens?: number; kind?: PlaceKind; capacity?: number; label?: string }
export interface TransitionSpec {
  id: string;
  label?: string;
  /** Deterministic delay (s) for timed simulation. */
  delay?: number;
  /** Exponential rate (1/s) for GSPN analysis; `immediate` transitions fire in zero time with `weight`. */
  rate?: number;
  immediate?: boolean;
  weight?: number;
  priority?: number;
}
export interface ArcSpec { from: string; to: string; weight?: number; inhibitor?: boolean }
export interface PetriNetSpec { name: string; places: PlaceSpec[]; transitions: TransitionSpec[]; arcs: ArcSpec[] }

export type Marking = number[];
const markingKey = (m: Marking) => m.join(',');

export class PetriNet {
  readonly places: PlaceSpec[];
  readonly transitions: TransitionSpec[];
  /** pre[t][p], post[t][p] (arc weights); inhibitor[t][p] threshold or 0. */
  readonly pre: number[][]; readonly post: number[][]; readonly inhibitor: number[][];
  readonly m0: Marking;
  readonly pIndex = new Map<string, number>(); readonly tIndex = new Map<string, number>();

  constructor(readonly spec: PetriNetSpec) {
    this.places = spec.places; this.transitions = spec.transitions;
    spec.places.forEach((p, i) => this.pIndex.set(p.id, i)); spec.transitions.forEach((t, i) => this.tIndex.set(t.id, i));
    const m = spec.places.length, n = spec.transitions.length;
    this.pre = Array.from({ length: n }, () => new Array(m).fill(0));
    this.post = Array.from({ length: n }, () => new Array(m).fill(0));
    this.inhibitor = Array.from({ length: n }, () => new Array(m).fill(0));
    for (const a of spec.arcs) {
      const w = a.weight ?? 1;
      if (this.pIndex.has(a.from) && this.tIndex.has(a.to)) { if (a.inhibitor) this.inhibitor[this.tIndex.get(a.to)!][this.pIndex.get(a.from)!] = w; else this.pre[this.tIndex.get(a.to)!][this.pIndex.get(a.from)!] += w; }
      else if (this.tIndex.has(a.from) && this.pIndex.has(a.to)) this.post[this.tIndex.get(a.from)!][this.pIndex.get(a.to)!] += w;
      else throw new Error(`arc ${a.from} -> ${a.to}: unknown place / transition or wrong direction`);
    }
    this.m0 = spec.places.map((p) => p.tokens ?? 0);
  }

  get placeCount(): number { return this.places.length; }
  get transitionCount(): number { return this.transitions.length; }
  placeId(i: number): string { return this.places[i].id; }
  transitionId(j: number): string { return this.transitions[j].id; }
  p(id: string): number { const i = this.pIndex.get(id); if (i === undefined) throw new Error(`unknown place ${id}`); return i; }
  t(id: string): number { const i = this.tIndex.get(id); if (i === undefined) throw new Error(`unknown transition ${id}`); return i; }

  /** Incidence matrix C = post − pre as places × transitions. */
  incidence(): number[][] {
    return this.places.map((_, i) => this.transitions.map((_, j) => this.post[j][i] - this.pre[j][i]));
  }
  /** Input places of a transition / output places. */
  preset(t: number): number[] { return this.pre[t].map((w, i) => (w > 0 ? i : -1)).filter((i) => i >= 0); }
  postset(t: number): number[] { return this.post[t].map((w, i) => (w > 0 ? i : -1)).filter((i) => i >= 0); }
  /** Transitions consuming from / producing into a place. */
  placeOut(p: number): number[] { return this.transitions.map((_, j) => (this.pre[j][p] > 0 ? j : -1)).filter((j) => j >= 0); }
  placeIn(p: number): number[] { return this.transitions.map((_, j) => (this.post[j][p] > 0 ? j : -1)).filter((j) => j >= 0); }

  isEnabled(m: Marking, t: number): boolean {
    for (let i = 0; i < m.length; i++) { if (m[i] < this.pre[t][i]) return false; if (this.inhibitor[t][i] && m[i] >= this.inhibitor[t][i]) return false; }
    return true;
  }
  /** Enabled transitions; immediate ones (highest priority) pre-empt timed ones when `gspn`. */
  enabled(m: Marking, gspn = false): number[] {
    const en = this.transitions.map((_, j) => j).filter((j) => this.isEnabled(m, j));
    if (!gspn) return en;
    const imm = en.filter((j) => this.transitions[j].immediate);
    if (!imm.length) return en;
    const top = Math.max(...imm.map((j) => this.transitions[j].priority ?? 1));
    return imm.filter((j) => (this.transitions[j].priority ?? 1) === top);
  }
  fire(m: Marking, t: number): Marking { return m.map((v, i) => v - this.pre[t][i] + this.post[t][i]); }
  markingString(m: Marking): string { return this.places.map((p, i) => (m[i] ? `${p.id}${m[i] > 1 ? `×${m[i]}` : ''}` : '')).filter(Boolean).join(' ') || '∅'; }
  /** Sum of tokens over a set of places. */
  tokensIn(m: Marking, set: number[]): number { return set.reduce((s, p) => s + m[p], 0); }

  // -------------------------------------------------------------------------------------------
  // Reachability
  // -------------------------------------------------------------------------------------------

  reachability(opts: { limit?: number; gspn?: boolean } = {}): ReachabilityGraph {
    const limit = opts.limit ?? 20000;
    const markings: Marking[] = [this.m0]; const index = new Map<string, number>([[markingKey(this.m0), 0]]);
    const edges: Array<{ from: number; t: number; to: number }> = [];
    const succ: number[][] = [[]];
    let truncated = false;
    for (let i = 0; i < markings.length; i++) {
      const m = markings[i];
      for (const t of this.enabled(m, opts.gspn)) {
        const y = this.fire(m, t); const k = markingKey(y);
        let j = index.get(k);
        if (j === undefined) { if (markings.length >= limit) { truncated = true; continue; } j = markings.length; markings.push(y); index.set(k, j); succ.push([]); }
        edges.push({ from: i, t, to: j }); succ[i].push(j);
      }
    }
    return new ReachabilityGraph(this, markings, edges, succ, truncated);
  }

  // -------------------------------------------------------------------------------------------
  // Invariants (Farkas, Algorithm 4.1)
  // -------------------------------------------------------------------------------------------

  pInvariants(): number[][] { return farkas(this.incidence()); }
  tInvariants(): number[][] { const C = this.incidence(); const Ct = this.transitions.map((_, j) => C.map((row) => row[j])); return farkas(Ct); }
  /** Invariant as a readable equation Σ l(p)·M(p) = const. */
  invariantEquation(y: number[]): string {
    const terms = y.map((c, i) => (c ? `${c > 1 ? c + '·' : ''}M(${this.places[i].id})` : '')).filter(Boolean);
    const c0 = y.reduce((s, c, i) => s + c * this.m0[i], 0);
    return `${terms.join(' + ')} = ${c0}`;
  }
  /** Places covered by some P-invariant are structurally bounded. */
  coveredByPInvariant(inv = this.pInvariants()): { covered: boolean; uncovered: string[] } {
    const cov = new Set<number>(); for (const y of inv) y.forEach((c, i) => { if (c > 0) cov.add(i); });
    const uncovered = this.places.map((p, i) => (cov.has(i) ? '' : p.id)).filter(Boolean);
    return { covered: uncovered.length === 0, uncovered };
  }

  // -------------------------------------------------------------------------------------------
  // Siphons and traps
  // -------------------------------------------------------------------------------------------

  /** Minimal siphons (•S ⊆ S•) by branch-and-bound extension from each seed place. */
  minimalSiphons(maxCount = 500): number[][] { return minimalSets(this, false, maxCount); }
  /** Minimal traps (S• ⊆ •S). */
  minimalTraps(maxCount = 500): number[][] { return minimalSets(this, true, maxCount); }
  isSiphon(S: number[]): boolean { const set = new Set(S); for (const p of S) for (const t of this.placeIn(p)) if (!this.preset(t).some((q) => set.has(q))) return false; return true; }
  isTrap(S: number[]): boolean { const set = new Set(S); for (const p of S) for (const t of this.placeOut(p)) if (!this.postset(t).some((q) => set.has(q))) return false; return true; }
  /** Commoner: siphon S is controlled when it contains a trap marked at M0 (never empties). */
  containsMarkedTrap(S: number[]): number[] | null {
    const set = new Set(S);
    for (const tr of this.minimalTraps()) if (tr.every((p) => set.has(p)) && this.tokensIn(this.m0, tr) > 0) return tr;
    // maximal trap inside S
    let T = [...S];
    for (;;) { const bad = T.filter((p) => this.placeOut(p).some((t) => !this.postset(t).some((q) => T.includes(q)))); if (!bad.length) break; T = T.filter((p) => !bad.includes(p)); }
    return T.length && this.tokensIn(this.m0, T) > 0 ? T : null;
  }

  // -------------------------------------------------------------------------------------------
  // Monitors (GMEC) and deadlock prevention
  // -------------------------------------------------------------------------------------------

  /** Add a monitor place enforcing lᵀM ≤ β (Theorem 4.9). Returns a new net; throws when β < lᵀM0. */
  withMonitor(l: number[], beta: number, id = `V${this.places.filter((p) => p.kind === 'monitor').length + 1}`): PetriNet {
    const C = this.incidence();
    const lM0 = l.reduce((s, c, i) => s + c * this.m0[i], 0);
    if (beta < lM0) throw new Error(`constraint ${beta} violated by M0 (lᵀM0 = ${lM0})`);
    const row = this.transitions.map((_, j) => -l.reduce((s, c, i) => s + c * C[i][j], 0)); // C_V
    const arcs: ArcSpec[] = [...this.spec.arcs];
    row.forEach((c, j) => { if (c < 0) arcs.push({ from: id, to: this.transitions[j].id, weight: -c }); else if (c > 0) arcs.push({ from: this.transitions[j].id, to: id, weight: c }); });
    return new PetriNet({ name: this.spec.name, places: [...this.places, { id, tokens: beta - lM0, kind: 'monitor', label: `monitor: ${l.map((c, i) => (c ? `${c > 1 ? c : ''}${this.places[i].id}` : '')).filter(Boolean).join('+')} ≤ ${beta}` }], transitions: this.transitions, arcs });
  }
  /**
   * Monitor keeping siphon S marked (Ezpeleta et al.): the "thief" operation places [S] — places that hold a resource
   * of S while lying outside S (found through the resource P-invariants) — may not all be marked at once:
   * Σ_{p ∈ [S]} M(p) ≤ M0(S) − 1. Falls back to the operation places of S when the net is not an S³PR.
   */
  monitorForSiphon(S: number[]): { l: number[]; beta: number; net: PetriNet } {
    const inS = new Set(S);
    const resources = S.filter((p) => this.m0[p] > 0);
    const inv = this.pInvariants();
    const thieves = new Set<number>();
    for (const r of resources) {
      const ys = inv.filter((y) => y[r] > 0).sort((a, b) => a.filter(Boolean).length - b.filter(Boolean).length);
      for (const y of ys.slice(0, 1)) y.forEach((c, p) => { if (c > 0 && p !== r && !inS.has(p) && this.m0[p] === 0 && this.places[p].kind !== 'resource') thieves.add(p); });
    }
    const target = thieves.size ? [...thieves] : S.filter((p) => this.m0[p] === 0 && this.places[p].kind !== 'resource');
    const l = this.places.map((_, i) => (target.includes(i) ? 1 : 0));
    const beta = this.tokensIn(this.m0, S) - 1;
    return { l, beta, net: this.withMonitor(l, beta) };
  }

  toSpec(): PetriNetSpec { return JSON.parse(JSON.stringify(this.spec)); }
}

/** Minimal siphons (trap=false) or traps (trap=true) by extension: every input transition of S must have an input place in S. */
function minimalSets(net: PetriNet, trap: boolean, maxCount: number): number[][] {
  const found: number[][] = [];
  const inT = (p: number) => (trap ? net.placeOut(p) : net.placeIn(p));
  const need = (t: number) => (trap ? net.postset(t) : net.preset(t));
  const isMinimalNew = (S: Set<number>) => !found.some((f) => f.every((p) => S.has(p)));
  const extend = (S: Set<number>, depth: number): void => {
    if (found.length >= maxCount || depth > net.placeCount) return;
    for (const p of S) for (const t of inT(p)) {
      const req = need(t);
      if (req.some((q) => S.has(q))) continue;
      if (!req.length) return; // a source transition feeds S: no siphon contains p
      for (const q of req) { const S2 = new Set(S); S2.add(q); if (isMinimalNew(S2)) extend(S2, depth + 1); }
      return;
    }
    // closed
    const arr = [...S].sort((a, b) => a - b);
    if (isMinimalNew(S)) { for (let i = found.length - 1; i >= 0; i--) if (found[i].every((p) => S.has(p)) === false && arr.every((p) => found[i].includes(p))) found.splice(i, 1); found.push(arr); }
  };
  for (let p = 0; p < net.placeCount; p++) extend(new Set([p]), 0);
  // keep only minimal
  return found.filter((s) => !found.some((o) => o !== s && o.length < s.length && o.every((p) => s.includes(p)))).map((s) => s);
}

/** Farkas: minimal non-negative solutions of yᵀC = 0 (rows of C are places). */
export function farkas(C: number[][]): number[][] {
  const m = C.length, n = C[0]?.length ?? 0;
  const gcd = (a: number, b: number): number => (b ? gcd(b, a % b) : Math.abs(a));
  let A = C.map((row, i) => [...row, ...Array.from({ length: m }, (_, k) => (k === i ? 1 : 0))]);
  for (let j = 0; j < n; j++) {
    const plus = A.filter((r) => r[j] > 0), minus = A.filter((r) => r[j] < 0), zero = A.filter((r) => r[j] === 0);
    const next = [...zero];
    for (const rp of plus) for (const rm of minus) {
      const s = rp.map((v, k) => Math.abs(rm[j]) * v + Math.abs(rp[j]) * rm[k]);
      const g = s.reduce((acc, v) => gcd(acc, v), 0) || 1;
      const norm = s.map((v) => v / g);
      const support = norm.slice(n).map((v, k) => (v ? k : -1)).filter((k) => k >= 0);
      if (!next.some((r) => { const sup = r.slice(n).map((v, k) => (v ? k : -1)).filter((k) => k >= 0); return sup.every((k) => support.includes(k)); })) next.push(norm);
    }
    A = next;
  }
  const inv = A.map((r) => r.slice(n)).filter((y) => y.some((v) => v > 0));
  return inv.filter((y, i) => !inv.some((o, k) => k !== i && o.every((v, q) => (v ? y[q] > 0 : true)) && o.some((v, q) => v !== y[q]) && o.filter(Boolean).length < y.filter(Boolean).length));
}

// ---------------------------------------------------------------------------------------------
// Reachability graph analysis
// ---------------------------------------------------------------------------------------------

export class ReachabilityGraph {
  constructor(readonly net: PetriNet, readonly markings: Marking[], readonly edges: Array<{ from: number; t: number; to: number }>, readonly succ: number[][], readonly truncated: boolean) {}
  get size(): number { return this.markings.length; }
  bounds(): number[] { return this.net.places.map((_, i) => Math.max(...this.markings.map((m) => m[i]))); }
  /** Markings enabling no transition. */
  deadlocks(): number[] { return this.markings.map((_, i) => i).filter((i) => this.succ[i].length === 0); }
  /** Shortest firing sequence from M0 to a marking index. */
  traceTo(target: (i: number) => boolean): string[] | null {
    const prev = new Map<number, [number, number] | null>([[0, null]]); const q = [0]; let qi = 0;
    while (qi < q.length) {
      const i = q[qi++];
      if (target(i)) { const path: string[] = []; let c = i; while (prev.get(c)) { const [p, t] = prev.get(c)!; path.push(this.net.transitionId(t)); c = p; } return path.reverse(); }
      for (const e of this.edges) if (e.from === i && !prev.has(e.to)) { prev.set(e.to, [i, e.t]); q.push(e.to); }
    }
    return null;
  }
  /** Reverse reachability: markings from which some marking in `targets` is reachable. */
  private backward(targets: Set<number>): Set<number> {
    const back = new Map<number, number[]>(); for (const e of this.edges) { let a = back.get(e.to); if (!a) { a = []; back.set(e.to, a); } a.push(e.from); }
    const seen = new Set(targets); const q = [...targets];
    while (q.length) { const y = q.pop()!; for (const x of back.get(y) ?? []) if (!seen.has(x)) { seen.add(x); q.push(x); } }
    return seen;
  }
  /** Transition t is live iff from every reachable marking a marking enabling t is reachable. */
  liveness(): { live: boolean; dead: string[]; nonLive: string[] } {
    const dead: string[] = [], nonLive: string[] = [];
    for (let t = 0; t < this.net.transitionCount; t++) {
      const enabling = new Set(this.markings.map((_, i) => i).filter((i) => this.edges.some((e) => e.from === i && e.t === t)));
      if (!enabling.size) { dead.push(this.net.transitionId(t)); continue; }
      const can = this.backward(enabling);
      if (can.size < this.markings.length) nonLive.push(this.net.transitionId(t));
    }
    return { live: dead.length === 0 && nonLive.length === 0, dead, nonLive };
  }
  reversible(): boolean { return this.backward(new Set([0])).size === this.markings.length; }
  /** Markings where a siphon is empty (bad siphon witness). */
  emptyingMarkings(S: number[]): number[] { return this.markings.map((m, i) => (this.net.tokensIn(m, S) === 0 ? i : -1)).filter((i) => i >= 0); }
  /** Livelock: a terminal SCC (no way out) that contains no marking of the "home"/goal set — here: cycles not reaching M0 when the net is meant to be reversible. */
  terminalComponentsWithoutM0(): number[][] {
    const sccs = tarjan(this.markings.length, this.succ);
    const out: number[][] = [];
    for (const c of sccs) {
      const set = new Set(c);
      const terminal = c.every((i) => this.succ[i].every((j) => set.has(j)));
      const cyclic = c.length > 1 || this.succ[c[0]].includes(c[0]);
      if (terminal && cyclic && !set.has(0)) out.push(c);
    }
    return out;
  }
}

function tarjan(n: number, succ: number[][]): number[][] { return stronglyConnected(Array.from({ length: n }, (_, i) => i), (v) => succ[v]); }

// ---------------------------------------------------------------------------------------------
// Structural analysis report and deadlock prevention
// ---------------------------------------------------------------------------------------------

export interface SiphonReport { places: string[]; initialTokens: number; controlled: boolean; emptyAt: string | null; trace: string[] | null; trap: string[] | null }
export interface PetriAnalysis {
  reachable: number; truncated: boolean; bounded: boolean; bounds: Record<string, number>; safe: boolean;
  pInvariants: string[]; tInvariants: string[]; coveredByPInvariants: boolean; uncovered: string[];
  live: boolean; deadTransitions: string[]; nonLiveTransitions: string[]; reversible: boolean;
  deadlocks: Array<{ marking: string; trace: string[] }>;
  livelocks: string[][];
  siphons: SiphonReport[]; traps: string[][];
  badSiphons: SiphonReport[];
}

export function analysePetriNet(net: PetriNet, opts: { limit?: number } = {}): PetriAnalysis {
  const rg = net.reachability({ limit: opts.limit });
  const bounds = rg.bounds();
  const pinv = net.pInvariants(), tinv = net.tInvariants();
  const cov = net.coveredByPInvariant(pinv);
  const live = rg.liveness();
  const dl = rg.deadlocks();
  const siphons = net.minimalSiphons().map((S): SiphonReport => {
    const trap = net.containsMarkedTrap(S);
    const empt = rg.emptyingMarkings(S);
    return { places: S.map((p) => net.placeId(p)), initialTokens: net.tokensIn(net.m0, S), controlled: !!trap || empt.length === 0, emptyAt: empt.length ? net.markingString(rg.markings[empt[0]]) : null, trace: empt.length ? rg.traceTo((i) => empt.includes(i)) : null, trap: trap ? trap.map((p) => net.placeId(p)) : null };
  });
  return {
    reachable: rg.size, truncated: rg.truncated, bounded: !rg.truncated, bounds: Object.fromEntries(net.places.map((p, i) => [p.id, bounds[i]])), safe: bounds.every((b) => b <= 1),
    pInvariants: pinv.map((y) => net.invariantEquation(y)), tInvariants: tinv.map((x) => x.map((c, j) => (c ? `${c > 1 ? c + '·' : ''}${net.transitionId(j)}` : '')).filter(Boolean).join(' + ')),
    coveredByPInvariants: cov.covered, uncovered: cov.uncovered,
    live: live.live, deadTransitions: live.dead, nonLiveTransitions: live.nonLive, reversible: rg.reversible(),
    deadlocks: dl.slice(0, 20).map((i) => ({ marking: net.markingString(rg.markings[i]), trace: rg.traceTo((k) => k === i) ?? [] })),
    livelocks: rg.terminalComponentsWithoutM0().slice(0, 5).map((c) => c.slice(0, 6).map((i) => net.markingString(rg.markings[i]))),
    siphons, traps: net.minimalTraps().map((T) => T.map((p) => net.placeId(p))),
    badSiphons: siphons.filter((s) => s.emptyAt !== null),
  };
}

/** Iteratively add GMEC monitors for every siphon that can empty until the net is live (or the iteration budget ends). */
export function preventDeadlocks(net: PetriNet, maxIterations = 10): { net: PetriNet; monitors: Array<{ id: string; constraint: string; siphon: string[] }>; live: boolean; iterations: number } {
  let cur = net; const monitors: Array<{ id: string; constraint: string; siphon: string[] }> = [];
  for (let it = 1; it <= maxIterations; it++) {
    const rg = cur.reachability();
    const bad = cur.minimalSiphons().filter((S) => rg.emptyingMarkings(S).length > 0 && !cur.containsMarkedTrap(S));
    if (!bad.length) return { net: cur, monitors, live: rg.liveness().live, iterations: it - 1 };
    const S = bad.sort((a, b) => a.length - b.length)[0];
    const { net: next, l, beta } = cur.monitorForSiphon(S);
    const id = next.places[next.places.length - 1].id;
    monitors.push({ id, constraint: `${l.map((c, i) => (c ? cur.placeId(i) : '')).filter(Boolean).join(' + ')} ≤ ${beta}`, siphon: S.map((p) => cur.placeId(p)) });
    cur = next;
  }
  return { net: cur, monitors, live: cur.reachability().liveness().live, iterations: maxIterations };
}

// ---------------------------------------------------------------------------------------------
// Banker's algorithm and resource ordering
// ---------------------------------------------------------------------------------------------

/** Algorithm 4.2 — is the state (Available, Alloc, Need) safe? Returns the completion order when it is. */
export function bankerSafe(available: number[], alloc: number[][], need: number[][]): { safe: boolean; order: number[] } {
  const work = [...available]; const finish = alloc.map(() => false); const order: number[] = [];
  for (;;) {
    const i = finish.findIndex((f, k) => !f && need[k].every((v, r) => v <= work[r]));
    if (i < 0) break;
    alloc[i].forEach((v, r) => { work[r] += v; }); finish[i] = true; order.push(i);
  }
  return { safe: finish.every(Boolean), order };
}
/** Grant a request if the resulting state stays safe. */
export function bankerRequest(available: number[], alloc: number[][], need: number[][], proc: number, request: number[]): { granted: boolean; reason: string } {
  if (request.some((v, r) => v > need[proc][r])) return { granted: false, reason: 'request exceeds the declared need' };
  if (request.some((v, r) => v > available[r])) return { granted: false, reason: 'resources not available — wait' };
  const av = available.map((v, r) => v - request[r]); const al = alloc.map((row, i) => row.map((v, r) => (i === proc ? v + request[r] : v))); const nd = need.map((row, i) => row.map((v, r) => (i === proc ? v - request[r] : v)));
  const s = bankerSafe(av, al, nd);
  return s.safe ? { granted: true, reason: `safe: completion order ${s.order.join(' → ')}` } : { granted: false, reason: 'unsafe state — request deferred' };
}

/** Resource ordering (Proposition 4.2): find a total order consistent with every process' acquisition sequence, or report the cycle. */
export function resourceOrder(sequences: string[][]): { ok: boolean; order: string[]; cycle: string[] | null } {
  const nodes = new Set<string>(); const edges = new Map<string, Set<string>>();
  for (const seq of sequences) for (let i = 0; i < seq.length; i++) { nodes.add(seq[i]); for (let j = i + 1; j < seq.length; j++) { let s = edges.get(seq[i]); if (!s) { s = new Set(); edges.set(seq[i], s); } s.add(seq[j]); } }
  const indeg = new Map([...nodes].map((n) => [n, 0])); for (const s of edges.values()) for (const v of s) indeg.set(v, indeg.get(v)! + 1);
  const q = [...nodes].filter((n) => indeg.get(n) === 0); const order: string[] = [];
  while (q.length) { const n = q.shift()!; order.push(n); for (const v of edges.get(n) ?? []) { indeg.set(v, indeg.get(v)! - 1); if (indeg.get(v) === 0) q.push(v); } }
  if (order.length === nodes.size) return { ok: true, order, cycle: null };
  const rem = [...nodes].filter((n) => !order.includes(n));
  // find a cycle among the remaining
  const start = rem[0]; const path = [start]; const seen = new Set([start]); let cur = start;
  for (;;) { const nxt = [...(edges.get(cur) ?? [])].find((v) => rem.includes(v)); if (!nxt) break; if (seen.has(nxt)) { path.push(nxt); break; } path.push(nxt); seen.add(nxt); cur = nxt; }
  return { ok: false, order, cycle: path };
}

// ---------------------------------------------------------------------------------------------
// Timed simulation (deterministic delays, three-phase firing) and GSPN → CTMC
// ---------------------------------------------------------------------------------------------

export interface TimedSimResult {
  horizon: number; firings: Record<string, number>; throughput: Record<string, number>;
  /** Time-average tokens per place. */
  meanTokens: Record<string, number>;
  /** Fraction of time each place was non-empty (utilisation for resource "busy" places). */
  busy: Record<string, number>;
  log: Array<{ t: number; transition: string; start: number; end: number }>;
  deadlockAt: number | null;
  /** Zero-time firings without progress (a transition without inputs and delay): the simulation stopped early. */
  zeno?: boolean;
}

/** Deterministic T-timed net: tokens are removed at start and produced after `delay`; conflicts resolved by priority then a seeded random choice. */
export function simulateTimed(net: PetriNet, horizon: number, opts: { seed?: number; policy?: 'priority' | 'random' | 'first' } = {}): TimedSimResult {
  let seed = opts.seed ?? 1; const rnd = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
  let m = [...net.m0]; let time = 0;
  const pending: Array<{ end: number; t: number; start: number }> = [];
  const firings: Record<string, number> = {}; const log: TimedSimResult['log'] = [];
  const tokenTime = new Array(net.placeCount).fill(0), busyTime = new Array(net.placeCount).fill(0);
  let deadlockAt: number | null = null; let zeno = false; let zeroIters = 0;
  const account = (dt: number) => { for (let i = 0; i < m.length; i++) { tokenTime[i] += m[i] * dt; if (m[i] > 0) busyTime[i] += dt; } };
  for (let guard = 0; guard < 1e6 && time < horizon; guard++) {
    // start all enabled transitions (with conflicts)
    let en = net.enabled(m);
    let fired = 0; // a transition without input places (source) with zero delay would fire forever
    while (en.length && fired++ < 50) {
      let t: number;
      if (opts.policy === 'first') t = en[0];
      else { const top = Math.max(...en.map((j) => net.transitions[j].priority ?? 0)); const cand = en.filter((j) => (net.transitions[j].priority ?? 0) === top); t = opts.policy === 'priority' ? cand[0] : cand[Math.floor(rnd() * cand.length)]; }
      m = m.map((v, i) => v - net.pre[t][i]);
      const d = net.transitions[t].delay ?? 0;
      pending.push({ end: time + d, t, start: time });
      en = net.enabled(m);
    }
    if (!pending.length) { deadlockAt = time; account(horizon - time); time = horizon; break; }
    pending.sort((a, b) => a.end - b.end);
    const nxt = pending.shift()!;
    const dt = Math.min(nxt.end, horizon) - time; account(dt); time = Math.min(nxt.end, horizon);
    if (dt <= 0) { if (++zeroIters > 500) { zeno = true; break; } } else zeroIters = 0; // zero-time firing forever (a source transition without delay)
    if (nxt.end > horizon) break;
    m = m.map((v, i) => v + net.post[nxt.t][i]);
    const id = net.transitionId(nxt.t); firings[id] = (firings[id] ?? 0) + 1;
    if (log.length < 5000) log.push({ t: nxt.end, transition: id, start: nxt.start, end: nxt.end });
    // complete simultaneous events
    while (pending.length && pending[0].end <= time) { const p = pending.shift()!; m = m.map((v, i) => v + net.post[p.t][i]); const pid = net.transitionId(p.t); firings[pid] = (firings[pid] ?? 0) + 1; if (log.length < 5000) log.push({ t: p.end, transition: pid, start: p.start, end: p.end }); }
  }
  const H = Math.max(time, 1e-9);
  return {
    horizon: H, firings, throughput: Object.fromEntries(Object.entries(firings).map(([k, v]) => [k, v / H])),
    meanTokens: Object.fromEntries(net.places.map((p, i) => [p.id, tokenTime[i] / H])), busy: Object.fromEntries(net.places.map((p, i) => [p.id, busyTime[i] / H])), log, deadlockAt, zeno,
  };
}

export interface GspnResult {
  tangible: number; vanishing: number;
  /** Steady-state probability of each tangible marking. */
  pi: Array<{ marking: string; p: number }>;
  throughput: Record<string, number>;
  meanTokens: Record<string, number>;
  /** P(place non-empty). */
  utilisation: Record<string, number>;
  generator: number[][];
}

/** GSPN analysis: embedded chain, elimination of vanishing markings U' = U_tt + U_tv (I − U_vv)⁻¹ U_vt, CTMC steady state. */
export function analyseGspn(net: PetriNet, opts: { limit?: number } = {}): GspnResult {
  const rg = net.reachability({ limit: opts.limit, gspn: true });
  const n = rg.size;
  if (rg.truncated || n > 4000) throw new Error(`too many markings for the CTMC (${n}${rg.truncated ? '+, truncated' : ''}); bound the net or reduce the tokens`);
  const vanishing = rg.markings.map((m) => net.enabled(m, true).some((t) => net.transitions[t].immediate));
  // rates from marking i via transition t
  const rateOf = (i: number, t: number) => { const tr = net.transitions[t]; if (vanishing[i]) return tr.immediate ? (tr.weight ?? 1) : 0; return tr.rate ?? (tr.delay ? 1 / tr.delay : 1); };
  const out = rg.markings.map((_, i) => rg.edges.filter((e) => e.from === i).map((e) => ({ to: e.to, t: e.t, r: rateOf(i, e.t) })).filter((e) => e.r > 0));
  const exitRate = out.map((es) => es.reduce((s, e) => s + e.r, 0));
  // embedded DTMC U
  const U = Array.from({ length: n }, () => new Array(n).fill(0));
  out.forEach((es, i) => { for (const e of es) U[i][e.to] += exitRate[i] ? e.r / exitRate[i] : 0; });
  const tIdx = rg.markings.map((_, i) => i).filter((i) => !vanishing[i]), vIdx = rg.markings.map((_, i) => i).filter((i) => vanishing[i]);
  const nt = tIdx.length, nv = vIdx.length;
  let Up: number[][];
  if (nv) {
    const Uvv = vIdx.map((i) => vIdx.map((j) => U[i][j])), Uvt = vIdx.map((i) => tIdx.map((j) => U[i][j])), Utv = tIdx.map((i) => vIdx.map((j) => U[i][j])), Utt = tIdx.map((i) => tIdx.map((j) => U[i][j]));
    const IminusUvv = Uvv.map((row, i) => row.map((v, j) => (i === j ? 1 : 0) - v));
    const inv = invert(IminusUvv);
    if (!inv) throw new Error('cycle of immediate transitions (I − U_vv singular)');
    const X = matmul(matmul(Utv, inv), Uvt);
    Up = Utt.map((row, i) => row.map((v, j) => v + X[i][j]));
  } else Up = tIdx.map((i) => tIdx.map((j) => U[i][j]));
  // CTMC generator on tangible states: Q_ij = exitRate_i · U'_ij, Q_ii = −exitRate_i
  const Q = Up.map((row, a) => row.map((v, b) => (a === b ? -exitRate[tIdx[a]] * (1 - v) : exitRate[tIdx[a]] * v)));
  const pi = steadyState(Q);
  const throughput: Record<string, number> = {};
  tIdx.forEach((i, a) => { for (const e of out[i]) { const id = net.transitionId(e.t); throughput[id] = (throughput[id] ?? 0) + pi[a] * e.r; } });
  // immediate transitions fire with the same rate as their vanishing entries: attribute through U' is complex; approximate by flow conservation from tangible predecessors
  const meanTokens: Record<string, number> = {}, utilisation: Record<string, number> = {};
  net.places.forEach((p, k) => { meanTokens[p.id] = tIdx.reduce((s, i, a) => s + pi[a] * rg.markings[i][k], 0); utilisation[p.id] = tIdx.reduce((s, i, a) => s + (rg.markings[i][k] > 0 ? pi[a] : 0), 0); });
  return { tangible: nt, vanishing: nv, pi: tIdx.map((i, a) => ({ marking: net.markingString(rg.markings[i]), p: pi[a] })), throughput, meanTokens, utilisation, generator: Q };
}

function matmul(A: number[][], B: number[][]): number[][] { return A.map((row) => B[0].map((_, j) => row.reduce((s, v, k) => s + v * B[k][j], 0))); }
export function invert(A: number[][]): number[][] | null {
  const n = A.length; const M = A.map((row, i) => [...row, ...Array.from({ length: n }, (_, k) => (k === i ? 1 : 0))]);
  for (let c = 0; c < n; c++) {
    let piv = c; for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
    if (Math.abs(M[piv][c]) < 1e-12) return null;
    [M[c], M[piv]] = [M[piv], M[c]];
    const d = M[c][c]; for (let k = 0; k < 2 * n; k++) M[c][k] /= d;
    for (let r = 0; r < n; r++) if (r !== c) { const f = M[r][c]; if (f) for (let k = 0; k < 2 * n; k++) M[r][k] -= f * M[c][k]; }
  }
  return M.map((row) => row.slice(n));
}

// ---------------------------------------------------------------------------------------------
// S³PR builder: sequential processes with resources → Petri net
// ---------------------------------------------------------------------------------------------

export interface S3PRProcess { id: string; jobs: number; steps: Array<{ id: string; resources: string[]; duration?: number }> }
export interface S3PRSpec { name: string; resources: Record<string, number>; processes: S3PRProcess[] }

/** Build the S³PR net: idle place per process, one activity place per step (holding its resources), resource places with capacities. */
export function buildS3PR(spec: S3PRSpec): PetriNet {
  const places: PlaceSpec[] = [], transitions: TransitionSpec[] = [], arcs: ArcSpec[] = [];
  for (const [r, c] of Object.entries(spec.resources)) places.push({ id: r, tokens: c, kind: 'resource' });
  for (const pr of spec.processes) {
    const idle = `${pr.id}_idle`; places.push({ id: idle, tokens: pr.jobs, kind: 'idle' });
    let prev = idle; let prevRes: string[] = [];
    pr.steps.forEach((st, k) => {
      const act = `${pr.id}.${st.id}`; places.push({ id: act, tokens: 0, kind: 'activity' });
      const tr = `${pr.id}.start_${st.id}`; transitions.push({ id: tr, delay: st.duration ?? 1, rate: st.duration ? 1 / st.duration : 1 });
      arcs.push({ from: prev, to: tr }, { from: tr, to: act });
      for (const r of st.resources) if (!prevRes.includes(r)) arcs.push({ from: r, to: tr });
      for (const r of prevRes) if (!st.resources.includes(r)) arcs.push({ from: tr, to: r });
      prev = act; prevRes = st.resources;
      if (k === pr.steps.length - 1) { const done = `${pr.id}.done`; transitions.push({ id: done, delay: 0, rate: 1e6, immediate: false }); arcs.push({ from: act, to: done }, { from: done, to: idle }); for (const r of prevRes) arcs.push({ from: done, to: r }); }
    });
  }
  return new PetriNet({ name: spec.name, places, transitions, arcs });
}
