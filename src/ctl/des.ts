import { stronglyConnected } from './graph_util';
/**
 * Discrete-event systems (DES) and supervisory control — Ramadge–Wonham framework.
 *
 * Course chapters 2–3: generators G = (X, Σ, f, x0, Xm), parallel composition (synchronous product on shared
 * events), accessible / coaccessible parts, non-blocking (deadlock / livelock) checks with shortest
 * counter-traces, controllability, supremal controllable non-blocking sublanguage (Algorithm 3.1), modular
 * supervisors and the non-conflict check, observers (unobservable-reach closure), observability and
 * diagnosability (twin plant), supervisor tables for runtime execution.
 *
 * States are strings; composed states are joined with `|` and keep their component parts in `parts`.
 */

export interface Transition { from: string; event: string; to: string }

/** Serialisable automaton (what model documents store). */
export interface AutomatonSpec {
  name: string;
  events: string[];
  initial: string;
  marked: string[];
  transitions: Transition[];
  /** Optional explicit state list (states with no transitions). */
  states?: string[];
  /** Uncontrollable events (default: none). */
  uncontrollable?: string[];
  /** Unobservable events (default: none). */
  unobservable?: string[];
  /** Fault events for diagnosability. */
  faults?: string[];
}

export class DES {
  readonly sigma: Set<string>;
  /** state -> event -> next state */
  readonly f = new Map<string, Map<string, string>>();
  readonly X = new Set<string>();
  readonly xm: Set<string>;
  /** Component states for composed automata (state -> [part0, part1, …]). */
  parts = new Map<string, string[]>();
  /** Component names for composed automata. */
  components: string[] = [];

  constructor(readonly name: string, sigma: Iterable<string>, transitions: Transition[], readonly x0: string, marked: Iterable<string>, states: Iterable<string> = []) {
    this.sigma = new Set(sigma);
    this.xm = new Set(marked);
    this.X.add(x0);
    for (const s of states) this.X.add(s);
    for (const t of transitions) this.addTransition(t.from, t.event, t.to);
    for (const m of this.xm) this.X.add(m);
  }

  static fromSpec(s: AutomatonSpec): DES {
    const d = new DES(s.name, s.events, s.transitions, s.initial, s.marked, s.states ?? []);
    return d;
  }

  toSpec(extra: Partial<AutomatonSpec> = {}): AutomatonSpec {
    return { name: this.name, events: [...this.sigma], initial: this.x0, marked: [...this.xm], transitions: this.transitions(), states: [...this.X], ...extra };
  }

  addTransition(from: string, event: string, to: string): void {
    this.sigma.add(event);
    this.X.add(from); this.X.add(to);
    let m = this.f.get(from);
    if (!m) { m = new Map(); this.f.set(from, m); }
    m.set(event, to);
  }

  next(x: string, e: string): string | undefined { return this.f.get(x)?.get(e); }
  active(x: string): string[] { return [...(this.f.get(x)?.keys() ?? [])]; }
  transitions(): Transition[] {
    const out: Transition[] = [];
    for (const [from, m] of this.f) for (const [event, to] of m) out.push({ from, event, to });
    return out;
  }
  get transitionCount(): number { let n = 0; for (const m of this.f.values()) n += m.size; return n; }

  /** Run a trace from x0; returns the visited states or null when an event is not defined. */
  run(trace: string[]): string[] | null {
    let x = this.x0; const out = [x];
    for (const e of trace) { const y = this.next(x, e); if (y === undefined) return null; x = y; out.push(x); }
    return out;
  }

  /** Copy with a subset of states (and only transitions inside it). */
  restrict(states: Set<string>): DES {
    const tr = this.transitions().filter((t) => states.has(t.from) && states.has(t.to));
    const d = new DES(this.name, this.sigma, tr, this.x0, [...this.xm].filter((m) => states.has(m)), [...states]);
    d.components = this.components;
    for (const s of states) { const p = this.parts.get(s); if (p) d.parts.set(s, p); }
    return d;
  }

  /** Copy keeping only transitions on the given events. */
  restrictAlphabet(events: string[]): DES {
    const set = new Set(events);
    const d = new DES(this.name, events, this.transitions().filter((t) => set.has(t.event)), this.x0, this.xm, this.X);
    d.components = this.components; d.parts = this.parts;
    return d;
  }

  /** Component value of a composed state (by component index or name). */
  part(x: string, comp: number | string): string | undefined {
    const p = this.parts.get(x);
    if (!p) return undefined;
    const i = typeof comp === 'number' ? comp : this.components.indexOf(comp);
    return p[i];
  }

  /** Predecessor map. */
  private back(states: Set<string>): Map<string, Set<string>> {
    const back = new Map<string, Set<string>>();
    for (const [x, m] of this.f) { if (!states.has(x)) continue; for (const y of m.values()) { if (!states.has(y)) continue; let s = back.get(y); if (!s) { s = new Set(); back.set(y, s); } s.add(x); } }
    return back;
  }

  /** States reachable from x0 inside `states`. */
  accessible(states: Set<string> = this.X): Set<string> {
    if (!states.has(this.x0)) return new Set();
    const ac = new Set([this.x0]); const q = [this.x0];
    while (q.length) { const x = q.pop()!; const m = this.f.get(x); if (!m) continue; for (const y of m.values()) if (states.has(y) && !ac.has(y)) { ac.add(y); q.push(y); } }
    return ac;
  }

  /** States from which a marked state is reachable inside `states`. */
  coaccessible(states: Set<string> = this.X): Set<string> {
    const back = this.back(states);
    const co = new Set([...this.xm].filter((m) => states.has(m))); const q = [...co];
    while (q.length) { const y = q.pop()!; for (const x of back.get(y) ?? []) if (!co.has(x)) { co.add(x); q.push(x); } }
    return co;
  }

  trim(): DES { const ac = this.accessible(); const co = this.coaccessible(ac); return this.restrict(this.accessible(co)); }

  /** Shortest trace from x0 to any state of `targets` (BFS). */
  traceTo(targets: Set<string> | ((x: string) => boolean), states: Set<string> = this.X): string[] | null {
    const isTarget = typeof targets === 'function' ? targets : (x: string) => targets.has(x);
    if (!states.has(this.x0)) return null;
    const prev = new Map<string, [string, string] | null>([[this.x0, null]]);
    const q = [this.x0]; let qi = 0;
    while (qi < q.length) {
      const x = q[qi++];
      if (isTarget(x)) { const path: string[] = []; let c = x; while (prev.get(c)) { const [p, e] = prev.get(c)!; path.push(e); c = p; } return path.reverse(); }
      const m = this.f.get(x); if (!m) continue;
      for (const e of [...m.keys()].sort()) { const y = m.get(e)!; if (states.has(y) && !prev.has(y)) { prev.set(y, [x, e]); q.push(y); } }
    }
    return null;
  }

  /** Strongly connected components (Tarjan) restricted to `states`. */
  sccs(states: Set<string> = this.X): string[][] {
    return stronglyConnected(states, (v) => [...(this.f.get(v)?.values() ?? [])].filter((w) => states.has(w)));
  }

}

// ---------------------------------------------------------------------------------------------
// Composition
// ---------------------------------------------------------------------------------------------

export const joinState = (parts: string[]): string => parts.join('|');

/** Parallel (synchronous) composition — reachable part only. Shared events synchronise, private ones interleave. */
export function parallel(...gs: DES[]): DES {
  const sigma = new Set<string>(); for (const g of gs) for (const e of g.sigma) sigma.add(e);
  const events = [...sigma].sort();
  const x0parts = gs.map((g) => g.x0);
  const x0 = joinState(x0parts);
  const out = new DES(gs.map((g) => g.name).join('||'), sigma, [], x0, []);
  out.components = gs.flatMap((g) => g.components.length ? g.components : [g.name]);
  const flat = (parts: string[]): string[] => parts.flatMap((p, i) => gs[i].parts.get(p) ?? [p]);
  out.parts.set(x0, flat(x0parts));
  const seen = new Map<string, string[]>([[x0, x0parts]]);
  const q = [x0]; let qi = 0;
  while (qi < q.length) {
    const x = q[qi++]; const xp = seen.get(x)!;
    for (const e of events) {
      const nxt: string[] = []; let ok = true;
      for (let i = 0; i < gs.length; i++) {
        const g = gs[i];
        if (g.sigma.has(e)) { const y = g.next(xp[i], e); if (y === undefined) { ok = false; break; } nxt.push(y); }
        else nxt.push(xp[i]);
      }
      if (!ok) continue;
      const y = joinState(nxt);
      out.addTransition(x, e, y);
      if (!seen.has(y)) { seen.set(y, nxt); out.parts.set(y, flat(nxt)); q.push(y); }
    }
  }
  for (const [s, p] of seen) if (p.every((pi, i) => gs[i].xm.has(pi))) out.xm.add(s);
  return out;
}

/** Product (fully synchronous: every event must be shared) — used for specifications over the same alphabet. */
export function product(a: DES, b: DES): DES {
  const common = [...a.sigma].filter((e) => b.sigma.has(e));
  const ra = a.restrictAlphabet(common), rb = b.restrictAlphabet(common);
  return parallel(ra, rb);
}

// ---------------------------------------------------------------------------------------------
// Blocking analysis
// ---------------------------------------------------------------------------------------------

export interface BlockingReport {
  nonblocking: boolean;
  reachable: number;
  transitions: number;
  /** Reachable states from which no marked state is reachable. */
  blocking: string[];
  /** Blocking states with no outgoing transitions (deadlocks). */
  deadlocks: string[];
  /** Blocking states that keep moving forever without reaching a marked state (livelocks), grouped by SCC. */
  livelocks: string[][];
  /** Shortest trace into a deadlock / livelock. */
  deadlockTrace: string[] | null;
  livelockTrace: string[] | null;
  /** Shortest trace to any blocking state. */
  trace: string[] | null;
}

export function analyseBlocking(g: DES): BlockingReport {
  const ac = g.accessible();
  const co = g.coaccessible(ac);
  const blocking = [...ac].filter((x) => !co.has(x)).sort();
  const bset = new Set(blocking);
  const deadlocks = blocking.filter((x) => (g.f.get(x)?.size ?? 0) === 0 || [...g.f.get(x)!.values()].every((y) => !ac.has(y)));
  const dset = new Set(deadlocks);
  const live = g.sccs(bset).filter((c) => c.length > 1 || (c.length === 1 && [...(g.f.get(c[0])?.values() ?? [])].includes(c[0]))).filter((c) => !c.every((x) => dset.has(x)));
  let n = 0; for (const x of ac) n += g.f.get(x)?.size ?? 0;
  return {
    nonblocking: blocking.length === 0, reachable: ac.size, transitions: n, blocking, deadlocks, livelocks: live,
    deadlockTrace: deadlocks.length ? g.traceTo(dset, ac) : null,
    livelockTrace: live.length ? g.traceTo(new Set(live.flat()), ac) : null,
    trace: blocking.length ? g.traceTo(bset, ac) : null,
  };
}

// ---------------------------------------------------------------------------------------------
// Supervisory control
// ---------------------------------------------------------------------------------------------

export interface SupconLog { iteration: number; removedControllability: number; removedBlocking: number; removed: string[] }
export interface SupconResult {
  /** Supervisor automaton (restricted product) — empty when the specification is unrealisable. */
  supervisor: DES;
  /** Full product H = G || E1 || … before pruning. */
  product: DES;
  states: Set<string>;
  log: SupconLog[];
  realizable: boolean;
  /** Number of states where a controllable event is physically possible in the plant but disabled by the supervisor. */
  disabled: Record<string, number>;
  productStates: number;
}

/**
 * Supremal controllable and non-blocking sublanguage (Algorithm 3.1). H = G || E1 || … ; the plant state is the
 * first component of every product state. Iterate: remove states where the plant enables an uncontrollable event
 * that H does not (or that leads outside), then trim; until a fixed point.
 */
export function supcon(plant: DES, specs: DES[], uncontrollable: Iterable<string>): SupconResult {
  const uc = new Set(uncontrollable);
  const h = parallel(plant, ...specs);
  const plantPartCount = plant.components.length || 1;
  const plantState = (x: string) => { const p = h.parts.get(x)!; return plantPartCount === 1 ? p[0] : joinState(p.slice(0, plantPartCount)); };
  let states = new Set(h.X);
  const log: SupconLog[] = [];
  let it = 0;
  for (;;) {
    it++;
    const bad = new Set<string>();
    for (const x of states) {
      const gx = plantState(x);
      for (const e of uc) {
        if (plant.next(gx, e) === undefined) continue;
        const y = h.next(x, e);
        if (y === undefined || !states.has(y)) { bad.add(x); break; }
      }
    }
    for (const b of bad) states.delete(b);
    const co = h.coaccessible(states);
    const ac = h.accessible(co);
    const removedBlocking = [...states].filter((x) => !ac.has(x));
    log.push({ iteration: it, removedControllability: bad.size, removedBlocking: removedBlocking.length, removed: [...bad, ...removedBlocking] });
    const fixed = removedBlocking.length === 0 && bad.size === 0;
    states = ac;
    if (fixed || states.size === 0) break;
  }
  const supervisor = h.restrict(states);
  const disabled: Record<string, number> = {};
  for (const x of states) {
    const gx = plantState(x);
    for (const e of plant.active(gx)) {
      if (uc.has(e)) continue;
      const y = h.next(x, e);
      if (y === undefined || !states.has(y)) disabled[e] = (disabled[e] ?? 0) + 1;
    }
  }
  return { supervisor, product: h, states, log, realizable: states.size > 0, disabled, productStates: h.X.size };
}

export interface ControllabilityIssue { state: string; plantState: string; event: string; trace: string[] }

/** Controllability of K = L(G || E) w.r.t. L(G): every uncontrollable event enabled in the plant must be enabled in the product. */
export function checkControllability(plant: DES, spec: DES, uncontrollable: Iterable<string>): { controllable: boolean; issues: ControllabilityIssue[] } {
  const uc = new Set(uncontrollable);
  const h = parallel(plant, spec);
  const pc = plant.components.length || 1;
  const issues: ControllabilityIssue[] = [];
  for (const x of h.X) {
    const p = h.parts.get(x)!; const gx = pc === 1 ? p[0] : joinState(p.slice(0, pc));
    for (const e of uc) if (plant.next(gx, e) !== undefined && h.next(x, e) === undefined) issues.push({ state: x, plantState: gx, event: e, trace: h.traceTo(new Set([x])) ?? [] });
  }
  issues.sort((a, b) => a.trace.length - b.trace.length);
  return { controllable: issues.length === 0, issues };
}

/** Modular supervisors S1…Sk run jointly with the plant; conflict = the joint behaviour blocks although each Si is non-blocking. */
export function checkNonconflict(plant: DES, supervisors: DES[]): { nonconflicting: boolean; jointStates: number; blocking: number; trace: string[] | null } {
  const x0 = joinState([plant.x0, ...supervisors.map((s) => s.x0)]);
  const joint = new DES('joint', plant.sigma, [], x0, []);
  const seen = new Map<string, string[]>([[x0, [plant.x0, ...supervisors.map((s) => s.x0)]]]);
  const q = [x0]; let qi = 0;
  while (qi < q.length) {
    const x = q[qi++]; const parts = seen.get(x)!;
    for (const e of plant.active(parts[0]).sort()) {
      const nxt = [plant.next(parts[0], e)!]; let ok = true;
      for (let i = 0; i < supervisors.length; i++) {
        const s = supervisors[i];
        if (!s.sigma.has(e)) { nxt.push(parts[i + 1]); continue; }
        const y = s.next(parts[i + 1], e); if (y === undefined) { ok = false; break; } nxt.push(y);
      }
      if (!ok) continue;
      const y = joinState(nxt); joint.addTransition(x, e, y);
      if (!seen.has(y)) { seen.set(y, nxt); q.push(y); }
    }
  }
  for (const [s, parts] of seen) if (plant.xm.has(parts[0]) && supervisors.every((sv, i) => sv.xm.has(parts[i + 1]))) joint.xm.add(s);
  const co = joint.coaccessible();
  const conflict = [...joint.X].filter((x) => !co.has(x));
  return { nonconflicting: conflict.length === 0, jointStates: joint.X.size, blocking: conflict.length, trace: conflict.length ? joint.traceTo(new Set(conflict)) : null };
}

// ---------------------------------------------------------------------------------------------
// Partial observation
// ---------------------------------------------------------------------------------------------

export interface Observer { initial: string; states: Map<string, string[]>; delta: Map<string, Map<string, string>>; automaton: DES }

/** Observer (state estimator) under unobservable events: subset construction with unobservable reach. */
export function observer(g: DES, unobservable: Iterable<string>): Observer {
  const uo = new Set(unobservable);
  const ur = (S: Iterable<string>): string[] => {
    const set = new Set(S); const q = [...set];
    while (q.length) { const x = q.pop()!; for (const e of uo) { const y = g.next(x, e); if (y !== undefined && !set.has(y)) { set.add(y); q.push(y); } } }
    return [...set].sort();
  };
  const key = (s: string[]) => `{${s.join(',')}}`;
  const q0 = ur([g.x0]);
  const states = new Map<string, string[]>([[key(q0), q0]]);
  const delta = new Map<string, Map<string, string>>();
  const so = [...g.sigma].filter((e) => !uo.has(e)).sort();
  const q = [q0]; let qi = 0;
  while (qi < q.length) {
    const cur = q[qi++]; const k = key(cur);
    for (const e of so) {
      const nxt = new Set<string>(); for (const x of cur) { const y = g.next(x, e); if (y !== undefined) nxt.add(y); }
      if (!nxt.size) continue;
      const r = ur(nxt); const rk = key(r);
      let m = delta.get(k); if (!m) { m = new Map(); delta.set(k, m); } m.set(e, rk);
      if (!states.has(rk)) { states.set(rk, r); q.push(r); }
    }
  }
  const tr: Transition[] = []; for (const [from, m] of delta) for (const [event, to] of m) tr.push({ from, event, to });
  const marked = [...states.entries()].filter(([, s]) => s.some((x) => g.xm.has(x))).map(([k]) => k);
  return { initial: key(q0), states, delta, automaton: new DES(`Obs(${g.name})`, so, tr, key(q0), marked, states.keys()) };
}

export interface ObservabilityIssue { cell: string; event: string; enabledIn: string[]; disabledIn: string[] }

/**
 * Observability of the supervisor decision: within one observer cell (states indistinguishable after the same
 * observable string) a controllable event must be either enabled everywhere or disabled everywhere.
 * `enabled(x, e)` says whether the supervisor allows e in plant state x (from the supervisor states set).
 */
export function checkObservability(plant: DES, supervisorStates: Set<string>, supervisor: DES, unobservable: Iterable<string>, controllable: Iterable<string>): { observable: boolean; issues: ObservabilityIssue[] } {
  const obs = observer(supervisor.restrict(supervisorStates), unobservable);
  const issues: ObservabilityIssue[] = [];
  const ctrl = new Set(controllable);
  for (const [k, cell] of obs.states) {
    for (const e of ctrl) {
      const en: string[] = [], dis: string[] = [];
      for (const x of cell) {
        const gx = plant.components.length ? joinState(supervisor.parts.get(x)!.slice(0, plant.components.length)) : supervisor.parts.get(x)![0];
        if (plant.next(gx, e) === undefined) continue;
        (supervisor.next(x, e) !== undefined && supervisorStates.has(supervisor.next(x, e)!) ? en : dis).push(x);
      }
      if (en.length && dis.length) issues.push({ cell: k, event: e, enabledIn: en, disabledIn: dis });
    }
  }
  return { observable: issues.length === 0, issues };
}

/**
 * Diagnosability of fault events (twin-plant / verifier): two copies of G synchronised on observable events;
 * a cycle reachable where one copy has executed a fault and the other has not means the fault can stay hidden forever.
 */
export function checkDiagnosability(g: DES, faults: Iterable<string>, unobservable: Iterable<string>): { diagnosable: boolean; witness: string[] | null; ambiguousCycle: string[] | null } {
  const F = new Set(faults), uo = new Set([...unobservable, ...F]);
  // verifier states: (x1, f1, x2, f2)
  const key = (a: string, fa: boolean, b: string, fb: boolean) => `${a}${fa ? '!' : ''}#${b}${fb ? '!' : ''}`;
  const parse = (k: string) => { const [l, r] = k.split('#'); return { a: l.replace('!', ''), fa: l.endsWith('!'), b: r.replace('!', ''), fb: r.endsWith('!') }; };
  const start = key(g.x0, false, g.x0, false);
  const ver = new DES('verifier', g.sigma, [], start, []);
  const seen = new Set([start]); const q = [start]; let qi = 0;
  while (qi < q.length) {
    const k = q[qi++]; const { a, fa, b, fb } = parse(k);
    const push = (nk: string, e: string) => { ver.addTransition(k, e, nk); if (!seen.has(nk)) { seen.add(nk); q.push(nk); } };
    for (const e of g.active(a)) if (uo.has(e)) push(key(g.next(a, e)!, fa || F.has(e), b, fb), `${e}·1`);
    for (const e of g.active(b)) if (uo.has(e)) push(key(a, fa, g.next(b, e)!, fb || F.has(e)), `${e}·2`);
    for (const e of g.active(a)) if (!uo.has(e) && g.next(b, e) !== undefined) push(key(g.next(a, e)!, fa, g.next(b, e)!, fb), e);
  }
  // ambiguous cycle: SCC with a cycle whose states have fa != fb
  const amb = new Set([...ver.X].filter((k) => { const p = parse(k); return p.fa !== p.fb; }));
  for (const c of ver.sccs(amb)) {
    const hasCycle = c.length > 1 || [...(ver.f.get(c[0])?.values() ?? [])].includes(c[0]);
    if (hasCycle) return { diagnosable: false, witness: ver.traceTo(new Set(c)), ambiguousCycle: c };
  }
  return { diagnosable: true, witness: null, ambiguousCycle: null };
}

// ---------------------------------------------------------------------------------------------
// Supervisor table (course JSON format) and runtime
// ---------------------------------------------------------------------------------------------

export interface SupervisorTable {
  initial: number;
  uncontrollable: string[];
  unobservable: string[];
  states: Array<{ id: number; plant: string[]; marked: boolean; next: Record<string, number>; enabled_controllable: string[] }>;
}

export function supervisorTable(sup: DES, uncontrollable: Iterable<string>, unobservable: Iterable<string> = [], plantParts?: number): SupervisorTable {
  const uc = new Set(uncontrollable);
  const states = [...sup.X].sort();
  const idx = new Map(states.map((s, i) => [s, i]));
  return {
    initial: idx.get(sup.x0)!, uncontrollable: [...uc].sort(), unobservable: [...unobservable].sort(),
    states: states.map((x) => {
      const next: Record<string, number> = {};
      for (const [e, y] of [...(sup.f.get(x)?.entries() ?? [])].sort()) next[e] = idx.get(y)!;
      const parts = sup.parts.get(x) ?? [x];
      return { id: idx.get(x)!, plant: plantParts ? parts.slice(0, plantParts) : parts, marked: sup.xm.has(x), next, enabled_controllable: Object.keys(next).filter((e) => !uc.has(e)).sort() };
    }),
  };
}

export class SupervisorViolation extends Error {}

/** Runtime executor: allowed(e) for controllable commands, observe(e) to advance on any observed event. */
export class SupervisorRuntime {
  state: number;
  readonly log: Array<{ kind: 'denied' | 'event'; state: number; event: string; next?: number }> = [];
  private byId: Map<number, SupervisorTable['states'][number]>;
  constructor(readonly table: SupervisorTable) { this.state = table.initial; this.byId = new Map(table.states.map((s) => [s.id, s])); }
  allowed(event: string): boolean {
    const ok = this.byId.get(this.state)!.enabled_controllable.includes(event) || (this.table.uncontrollable.includes(event) && this.byId.get(this.state)!.next[event] !== undefined);
    if (!ok) this.log.push({ kind: 'denied', state: this.state, event });
    return ok;
  }
  observe(event: string): void {
    if (this.table.unobservable.includes(event)) throw new Error(`event ${event} is unobservable`);
    const nxt = this.byId.get(this.state)!.next[event];
    if (nxt === undefined) throw new SupervisorViolation(`event ${event} impossible in state ${this.state} ${this.plantState().join(',')}`);
    this.log.push({ kind: 'event', state: this.state, event, next: nxt });
    this.state = nxt;
  }
  plantState(): string[] { return this.byId.get(this.state)!.plant; }
  marked(): boolean { return this.byId.get(this.state)!.marked; }
  enabled(): string[] { return this.byId.get(this.state)!.enabled_controllable; }
  reset(): void { this.state = this.table.initial; this.log.length = 0; }
}

/** Python runtime module (same as the course `supervisor_runtime.py`) with the table embedded. */
export function supervisorPython(table: SupervisorTable, name = 'supervisor'): string {
  return `# -*- coding: utf-8 -*-
"""Supervisor runtime generated by VerticalBot Studio (Ramadge–Wonham supervisor table).
allowed(e) -> may the controller issue the controllable event e now; observe(e) -> advance on an observed event."""
import json

TABLE = json.loads(r'''${JSON.stringify(table)}''')


class SupervisorViolation(RuntimeError):
    pass


class Supervisor:
    def __init__(self, table=TABLE):
        self.states = {s['id']: s for s in table['states']}
        self.uc = set(table['uncontrollable'])
        self.uo = set(table['unobservable'])
        self.state = table['initial']
        self.log = []

    def allowed(self, event):
        s = self.states[self.state]
        ok = event in s['enabled_controllable'] or (event in self.uc and event in s['next'])
        if not ok:
            self.log.append(('denied', self.state, event))
        return ok

    def observe(self, event):
        if event in self.uo:
            raise ValueError('event %s is unobservable' % event)
        nxt = self.states[self.state]['next'].get(event)
        if nxt is None:
            raise SupervisorViolation('event %s impossible in state %s %s' % (event, self.state, self.states[self.state]['plant']))
        self.log.append(('event', self.state, event, nxt))
        self.state = nxt

    def plant_state(self):
        return self.states[self.state]['plant']

    def marked(self):
        return self.states[self.state]['marked']


${name} = Supervisor()
`;
}
