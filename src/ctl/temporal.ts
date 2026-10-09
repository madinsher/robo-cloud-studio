/**
 * Temporal logic — specification, model checking and runtime monitoring (course chapters 7 and 9).
 *
 *  - LTL (X F G U R W, propositional connectives) and CTL (AX EX AG EG AF EF A[U] E[U]) parsers;
 *  - Kripke structures with adapters for automata (DES), Petri-net reachability graphs and behavior trees;
 *  - CTL model checking by the marking algorithm (Algorithm 7.1) with counterexample paths;
 *  - LTL model checking by the automata approach: LTL → generalised Büchi (Gerth–Peled–Vardi–Wolper tableau),
 *    degeneralisation, product with the model, nested-DFS emptiness, lasso counterexamples, fairness constraints;
 *  - LTL₃ three-valued runtime monitors (Algorithm 9.1: pairs of Büchi automata for φ and ¬φ with non-empty
 *    language filtering, determinised on the fly) and exported monitor tables.
 */
import type { DES } from './des';
import type { PetriNet, ReachabilityGraph } from './petri';
import { expr } from './expr';

// ---------------------------------------------------------------------------------------------
// Formulas
// ---------------------------------------------------------------------------------------------

export type LTL =
  | { k: 'true' } | { k: 'false' } | { k: 'ap'; name: string }
  | { k: 'not'; a: LTL } | { k: 'and'; a: LTL; b: LTL } | { k: 'or'; a: LTL; b: LTL }
  | { k: 'X'; a: LTL } | { k: 'U'; a: LTL; b: LTL } | { k: 'R'; a: LTL; b: LTL }
  // CTL-only nodes (path quantifier + temporal operator)
  | { k: 'EX'; a: LTL } | { k: 'AX'; a: LTL } | { k: 'EG'; a: LTL } | { k: 'AG'; a: LTL } | { k: 'EF'; a: LTL } | { k: 'AF'; a: LTL } | { k: 'EU'; a: LTL; b: LTL } | { k: 'AU'; a: LTL; b: LTL };

const T: LTL = { k: 'true' }, Fls: LTL = { k: 'false' };
export const ap = (name: string): LTL => ({ k: 'ap', name });
export const not = (a: LTL): LTL => ({ k: 'not', a });
export const and = (a: LTL, b: LTL): LTL => ({ k: 'and', a, b });
export const or = (a: LTL, b: LTL): LTL => ({ k: 'or', a, b });
export const X = (a: LTL): LTL => ({ k: 'X', a });
export const U = (a: LTL, b: LTL): LTL => ({ k: 'U', a, b });
export const R = (a: LTL, b: LTL): LTL => ({ k: 'R', a, b });
export const F = (a: LTL): LTL => U(T, a);
export const G = (a: LTL): LTL => R(Fls, a);
export const implies = (a: LTL, b: LTL): LTL => or(not(a), b);

const ATOM_RE = /^"[^"]+"|^[A-Za-z_][\w.:]*(?:\s*(?:[+*/]|-(?!>))\s*(?:[A-Za-z_][\w.:]*|\d+(?:\.\d+)?))*(?:\s*(?:==|=|!=|<=|>=|<|>)\s*(?:'[^']*'|"[^"]*"|-?[\w.]+))?/;
const WORDS = new Set(['G', 'F', 'X', 'U', 'R', 'W', 'A', 'E', 'AG', 'AF', 'AX', 'EG', 'EF', 'EX', 'true', 'false', 'TRUE', 'FALSE', 'and', 'or', 'not', 'implies']);

class TLParser {
  private i = 0;
  constructor(private readonly s: string, private readonly ctl: boolean) {}
  private ws() { while (this.i < this.s.length && /\s/.test(this.s[this.i])) this.i++; }
  private peekWord(): string | null { this.ws(); const m = /^[A-Za-z_][\w.:]*/.exec(this.s.slice(this.i)); return m ? m[0] : null; }
  private take(tok: string): boolean { this.ws(); if (this.s.startsWith(tok, this.i)) { const after = this.s[this.i + tok.length]; if (/\w/.test(tok[tok.length - 1]) && after && /[\w.:]/.test(after)) return false; this.i += tok.length; return true; } return false; }
  parse(): LTL { const f = this.parseIff(); this.ws(); if (this.i < this.s.length) throw new Error(`unexpected '${this.s.slice(this.i)}' in "${this.s}"`); return f; }
  private parseIff(): LTL { let a = this.parseImpl(); while (this.take('<->')) { const b = this.parseImpl(); a = and(implies(a, b), implies(b, a)); } return a; }
  private parseImpl(): LTL { const a = this.parseOr(); if (this.take('->') || this.take('=>') || this.take('implies')) { const b = this.parseImpl(); return implies(a, b); } return a; }
  private parseOr(): LTL { let a = this.parseAnd(); while (this.take('||') || this.take('|') || this.take('or')) { const b = this.parseAnd(); a = or(a, b); } return a; }
  private parseAnd(): LTL { let a = this.parseBinaryTemporal(); while (this.take('&&') || this.take('&') || this.take('and')) { const b = this.parseBinaryTemporal(); a = and(a, b); } return a; }
  private parseBinaryTemporal(): LTL {
    let a = this.parseUnary();
    if (this.ctl) return a;
    for (;;) {
      if (this.take('U')) { const b = this.parseUnary(); a = U(a, b); }
      else if (this.take('R') || this.take('V')) { const b = this.parseUnary(); a = R(a, b); }
      else if (this.take('W')) { const b = this.parseUnary(); a = R(b, or(a, b)); }
      else return a;
    }
  }
  private parseUnary(): LTL {
    this.ws();
    if (this.take('!') || this.take('not') || this.take('¬')) return not(this.parseUnary());
    if (this.ctl) {
      for (const [tok, k] of [['AG', 'AG'], ['AF', 'AF'], ['AX', 'AX'], ['EG', 'EG'], ['EF', 'EF'], ['EX', 'EX']] as const) if (this.take(tok)) return { k, a: this.parseUnary() } as LTL;
      for (const q of ['A', 'E'] as const) {
        const save = this.i;
        if (this.take(q)) {
          if (this.take('[') || this.take('(')) { const a = this.parseIff(); if (!this.take('U')) throw new Error(`expected U in ${q}[φ U ψ]`); const b = this.parseIff(); if (!(this.take(']') || this.take(')'))) throw new Error('expected ] or )'); return { k: q === 'A' ? 'AU' : 'EU', a, b }; }
          const w = this.peekWord();
          if (w === 'G' || w === 'F' || w === 'X') { this.take(w); return { k: `${q}${w}` as 'AG', a: this.parseUnary() }; }
          this.i = save;
        }
      }
    } else {
      if (this.take('G') || this.take('[]') || this.take('□')) return G(this.parseUnary());
      if (this.take('F') || this.take('<>') || this.take('◇')) return F(this.parseUnary());
      if (this.take('X') || this.take('○')) return X(this.parseUnary());
    }
    if (this.take('(')) { const f = this.parseIff(); if (!this.take(')')) throw new Error("expected ')'"); return f; }
    if (this.take('true') || this.take('TRUE') || this.take('1')) return T;
    if (this.take('false') || this.take('FALSE') || this.take('0')) return Fls;
    this.ws();
    const m = ATOM_RE.exec(this.s.slice(this.i));
    if (!m) throw new Error(`unexpected '${this.s.slice(this.i, this.i + 12)}' in "${this.s}"`);
    const word = /^[A-Za-z_][\w.:]*/.exec(m[0])?.[0];
    if (word && WORDS.has(word) && m[0] === word) throw new Error(`unexpected keyword '${word}' in "${this.s}"`);
    this.i += m[0].length;
    const text = m[0].startsWith('"') ? m[0].slice(1, -1) : m[0].replace(/\s+/g, '');
    return ap(text);
  }
}

export function parseLTL(src: string): LTL { return new TLParser(src, false).parse(); }
export function parseCTL(src: string): LTL { return new TLParser(src, true).parse(); }

export function fmt(f: LTL): string {
  switch (f.k) {
    case 'true': return 'true'; case 'false': return 'false'; case 'ap': return f.name;
    case 'not': return `!${fmt(f.a)}`; case 'and': return `(${fmt(f.a)} & ${fmt(f.b)})`; case 'or': return `(${fmt(f.a)} | ${fmt(f.b)})`;
    case 'X': return `X ${fmt(f.a)}`;
    case 'U': return f.a.k === 'true' ? `F ${fmt(f.b)}` : `(${fmt(f.a)} U ${fmt(f.b)})`;
    case 'R': return f.a.k === 'false' ? `G ${fmt(f.b)}` : `(${fmt(f.a)} R ${fmt(f.b)})`;
    case 'EU': return `E[${fmt(f.a)} U ${fmt(f.b)}]`; case 'AU': return `A[${fmt(f.a)} U ${fmt(f.b)}]`;
    default: return `${f.k} ${fmt((f as { a: LTL }).a)}`;
  }
}

/** Negation normal form (negations only on atoms). */
export function nnf(f: LTL, neg = false): LTL {
  switch (f.k) {
    case 'true': return neg ? Fls : T; case 'false': return neg ? T : Fls;
    case 'ap': return neg ? not(f) : f;
    case 'not': return nnf(f.a, !neg);
    case 'and': return neg ? or(nnf(f.a, true), nnf(f.b, true)) : and(nnf(f.a), nnf(f.b));
    case 'or': return neg ? and(nnf(f.a, true), nnf(f.b, true)) : or(nnf(f.a), nnf(f.b));
    case 'X': return X(nnf(f.a, neg));
    case 'U': return neg ? R(nnf(f.a, true), nnf(f.b, true)) : U(nnf(f.a), nnf(f.b));
    case 'R': return neg ? U(nnf(f.a, true), nnf(f.b, true)) : R(nnf(f.a), nnf(f.b));
    default: throw new Error(`CTL operator ${f.k} in an LTL formula`);
  }
}
export function atoms(f: LTL, out = new Set<string>()): Set<string> { if (f.k === 'ap') out.add(f.name); else for (const c of ['a', 'b'] as const) { const s = (f as Record<string, unknown>)[c]; if (s) atoms(s as LTL, out); } return out; }
const key = (f: LTL): string => fmt(f);

// ---------------------------------------------------------------------------------------------
// Kripke structures
// ---------------------------------------------------------------------------------------------

export interface Kripke {
  states: string[];
  initial: string[];
  next(s: string): string[];
  holds(s: string, atom: string): boolean;
  describe?(s: string): string;
}

/** Atom over a value environment: `name`, `name=value`, `name!=value`, `name>=3`… evaluated with the expression language. */
export function atomHolds(atom: string, env: Record<string, string | number | boolean | undefined>): boolean {
  const m = /^([A-Za-z_][\w.:]*)\s*(==|=|!=|<=|>=|<|>)\s*(.+)$/.exec(atom);
  if (m) { const v = env[m[1]]; let rhs: string | number = m[3].replace(/^['"]|['"]$/g, ''); if (/^-?\d+(\.\d+)?$/.test(rhs)) rhs = Number(rhs); const l = v === undefined ? 0 : v; switch (m[2]) { case '=': case '==': return String(l) === String(rhs); case '!=': return String(l) !== String(rhs); case '<': return Number(l) < Number(rhs); case '<=': return Number(l) <= Number(rhs); case '>': return Number(l) > Number(rhs); case '>=': return Number(l) >= Number(rhs); } }
  const v = env[atom];
  if (v !== undefined) return v === true || (typeof v === 'number' && v !== 0) || (typeof v === 'string' && v !== '' && v !== 'false');
  try { return expr(atom).bool(env); } catch { return false; }
}

/** Kripke structure of an automaton: atoms `Comp=state` for components of composed states, plain state names, `marked`, `deadlock`. */
export function kripkeFromDES(g: DES): Kripke {
  const env = (s: string): Record<string, string | number | boolean> => {
    const e: Record<string, string | number | boolean> = { state: s, marked: g.xm.has(s), deadlock: (g.f.get(s)?.size ?? 0) === 0 };
    const parts = g.parts.get(s); if (parts) parts.forEach((p, i) => { e[g.components[i] ?? `c${i}`] = p; e[p] = true; });
    e[s] = true;
    return e;
  };
  return { states: [...g.X], initial: [g.x0], next: (s) => [...new Set(g.f.get(s)?.values() ?? [])], holds: (s, atom) => atomHolds(atom, env(s)), describe: (s) => s };
}

/** Kripke structure of a reachability graph: atoms are place names (marked), comparisons on token counts, `deadlock`. */
export function kripkeFromPetri(rg: ReachabilityGraph): Kripke {
  const net: PetriNet = rg.net;
  const env = (i: number) => { const e: Record<string, number | boolean> = { deadlock: rg.succ[i].length === 0 }; net.places.forEach((p, k) => { e[p.id] = rg.markings[i][k]; }); return e; };
  return { states: rg.markings.map((_, i) => String(i)), initial: ['0'], next: (s) => rg.succ[Number(s)].map(String), holds: (s, atom) => atomHolds(atom, env(Number(s))), describe: (s) => net.markingString(rg.markings[Number(s)]) };
}

function successors(m: Kripke, s: string): string[] { const n = m.next(s); return n.length ? n : [s]; } // deadlock states stutter

// ---------------------------------------------------------------------------------------------
// CTL model checking (Algorithm 7.1)
// ---------------------------------------------------------------------------------------------

export interface CTLResult { holds: boolean; formula: string; satisfying: number; total: number; failingInitial: string[]; counterexample: string[] | null; explanation: string }

export function checkCTL(m: Kripke, formula: string | LTL): CTLResult {
  const f = typeof formula === 'string' ? parseCTL(formula) : formula;
  const all = m.states; const idx = new Map(all.map((s, i) => [s, i]));
  const pre = new Map<string, string[]>(); for (const s of all) for (const t of successors(m, s)) { let a = pre.get(t); if (!a) { a = []; pre.set(t, a); } a.push(s); }
  const preE = (Tset: Set<string>): Set<string> => { const out = new Set<string>(); for (const t of Tset) for (const s of pre.get(t) ?? []) out.add(s); return out; };
  const memo = new Map<string, Set<string>>();
  const sat = (g: LTL): Set<string> => {
    const k = key(g); const c = memo.get(k); if (c) return c;
    let r: Set<string>;
    switch (g.k) {
      case 'true': r = new Set(all); break; case 'false': r = new Set(); break;
      case 'ap': r = new Set(all.filter((s) => m.holds(s, g.name))); break;
      case 'not': { const a = sat(g.a); r = new Set(all.filter((s) => !a.has(s))); break; }
      case 'and': { const a = sat(g.a), b = sat(g.b); r = new Set([...a].filter((s) => b.has(s))); break; }
      case 'or': { const a = sat(g.a), b = sat(g.b); r = new Set([...a, ...b]); break; }
      case 'EX': r = preE(sat(g.a)); break;
      case 'AX': { const na = sat(not(g.a)); const ex = preE(na); r = new Set(all.filter((s) => !ex.has(s))); break; }
      case 'EU': { const a = sat(g.a); r = new Set(sat(g.b)); for (;;) { const add = [...preE(r)].filter((s) => a.has(s) && !r.has(s)); if (!add.length) break; for (const s of add) r.add(s); } break; }
      case 'EG': { r = new Set(sat(g.a)); for (;;) { const p = preE(r); const keep = [...r].filter((s) => p.has(s)); if (keep.length === r.size) break; r = new Set(keep); } break; }
      case 'EF': r = sat({ k: 'EU', a: T, b: g.a }); break;
      case 'AF': { const eg = sat({ k: 'EG', a: not(g.a) }); r = new Set(all.filter((s) => !eg.has(s))); break; }
      case 'AG': { const ef = sat({ k: 'EU', a: T, b: not(g.a) }); r = new Set(all.filter((s) => !ef.has(s))); break; }
      case 'AU': { const eu = sat({ k: 'EU', a: not(g.b), b: and(not(g.a), not(g.b)) }); const eg = sat({ k: 'EG', a: not(g.b) }); r = new Set(all.filter((s) => !eu.has(s) && !eg.has(s))); break; }
      default: throw new Error(`LTL operator ${g.k} in a CTL formula`);
    }
    memo.set(k, r); return r;
  };
  const S = sat(f);
  const failing = m.initial.filter((s) => !S.has(s));
  let counterexample: string[] | null = null; let explanation = '';
  if (failing.length) {
    if (f.k === 'AG') { const bad = new Set(all.filter((s) => !sat(f.a).has(s))); counterexample = bfsPath(m, failing[0], bad); explanation = `path to a state violating ${fmt(f.a)}`; }
    else if (f.k === 'AF') { const eg = sat({ k: 'EG', a: not(f.a) }); counterexample = lassoIn(m, failing[0], eg); explanation = `cycle on which ${fmt(f.a)} never holds`; }
    else if (f.k === 'AX') { const bad = new Set(all.filter((s) => !sat(f.a).has(s))); const nxt = successors(m, failing[0]).find((s) => bad.has(s)); counterexample = nxt ? [failing[0], nxt] : null; explanation = 'successor violating the formula'; }
    else if (f.k === 'EF') explanation = `no reachable state satisfies ${fmt(f.a)}`;
    else if (f.k === 'AU') { const bad = new Set(all.filter((s) => !S.has(s))); counterexample = lassoIn(m, failing[0], bad) ?? bfsPath(m, failing[0], new Set(all.filter((s) => !sat(f.b).has(s) && !sat(f.a).has(s)))); explanation = 'path where the until obligation fails'; }
    else explanation = `initial state ${failing[0]} does not satisfy the formula`;
  }
  void idx;
  return { holds: failing.length === 0, formula: fmt(f), satisfying: S.size, total: all.length, failingInitial: failing, counterexample, explanation };
}

function bfsPath(m: Kripke, from: string, targets: Set<string>): string[] | null {
  const prev = new Map<string, string | null>([[from, null]]); const q = [from]; let qi = 0;
  while (qi < q.length) { const s = q[qi++]; if (targets.has(s)) { const path: string[] = []; let c: string | null = s; while (c !== null) { path.push(c); c = prev.get(c) ?? null; } return path.reverse(); } for (const t of successors(m, s)) if (!prev.has(t)) { prev.set(t, s); q.push(t); } }
  return null;
}
/** Path from `from` into `region` followed by a cycle inside `region` (states of the region are closed under some successor). */
function lassoIn(m: Kripke, from: string, region: Set<string>): string[] | null {
  const path = bfsPath(m, from, region); if (!path) return null;
  const start = path[path.length - 1];
  // walk inside the region until a repeat
  const seen = new Map<string, number>(); const cyc: string[] = []; let cur = start;
  for (let i = 0; i < region.size + 2; i++) { if (seen.has(cur)) { const s0 = seen.get(cur)!; return [...path.slice(0, -1), ...cyc.slice(0, s0), ...cyc.slice(s0), cur]; } seen.set(cur, cyc.length); cyc.push(cur); const nxt = successors(m, cur).find((t) => region.has(t)); if (!nxt) break; cur = nxt; }
  return path;
}

// ---------------------------------------------------------------------------------------------
// LTL → Büchi (GPVW tableau)
// ---------------------------------------------------------------------------------------------

interface TNode { id: number; incoming: Set<number>; neu: LTL[]; old: LTL[]; next: LTL[] }

export interface Buchi {
  /** Node labels: positive and negative atoms that must hold when the node is visited. */
  nodes: Array<{ id: number; pos: string[]; neg: string[] }>;
  initial: number[];
  succ: Map<number, number[]>;
  /** Accepting states (after degeneralisation). */
  accepting: Set<number>;
  /** Degeneralised state → tableau node. */
  formula: string;
}

export function ltlToBuchi(f: LTL): Buchi {
  const phi = nnf(f);
  let seq = 0;
  const has = (list: LTL[], g: LTL) => list.some((x) => key(x) === key(g));
  const add = (list: LTL[], g: LTL): LTL[] => (has(list, g) ? list : [...list, g]);
  const nodes: TNode[] = [];
  const INIT = -1;
  const expand = (n: TNode): void => {
    if (n.neu.length === 0) {
      const same = nodes.find((r) => setEq(r.old, n.old) && setEq(r.next, n.next));
      if (same) { for (const i of n.incoming) same.incoming.add(i); return; }
      nodes.push(n);
      expand({ id: seq++, incoming: new Set([n.id]), neu: [...n.next], old: [], next: [] });
      return;
    }
    const eta = n.neu[0]; const rest = n.neu.slice(1);
    switch (eta.k) {
      case 'false': return;
      case 'true': expand({ ...n, neu: rest, old: add(n.old, eta) }); return;
      case 'ap': case 'not': {
        const negated: LTL = eta.k === 'ap' ? not(eta) : eta.a;
        if (has(n.old, negated)) return;
        expand({ ...n, neu: rest, old: add(n.old, eta) }); return;
      }
      case 'and': expand({ ...n, neu: add(add(rest, eta.a), eta.b).filter((g) => !has(n.old, g)), old: add(n.old, eta) }); return;
      case 'X': expand({ ...n, neu: rest, old: add(n.old, eta), next: add(n.next, eta.a) }); return;
      case 'or': case 'U': case 'R': {
        const new1 = eta.k === 'or' ? [eta.a] : eta.k === 'U' ? [eta.a] : [eta.b];
        const new2 = eta.k === 'or' ? [eta.b] : eta.k === 'U' ? [eta.b] : [eta.a, eta.b];
        const next1 = eta.k === 'or' ? [] : [eta];
        const n1: TNode = { id: seq++, incoming: new Set(n.incoming), neu: new1.reduce((l, g) => (has(n.old, g) ? l : add(l, g)), rest), old: add(n.old, eta), next: next1.reduce((l, g) => add(l, g), n.next) };
        const n2: TNode = { id: seq++, incoming: new Set(n.incoming), neu: new2.reduce((l, g) => (has(n.old, g) ? l : add(l, g)), rest), old: add(n.old, eta), next: [...n.next] };
        expand(n1); expand(n2); return;
      }
      default: throw new Error(`operator ${eta.k} not allowed in LTL`);
    }
  };
  expand({ id: seq++, incoming: new Set([INIT]), neu: [phi], old: [], next: [] });
  // generalised acceptance: one set per U-subformula
  const untils: LTL[] = []; const collect = (g: LTL) => { if (g.k === 'U' && !has(untils, g)) untils.push(g); for (const c of ['a', 'b'] as const) { const s = (g as Record<string, unknown>)[c]; if (s) collect(s as LTL); } }; collect(phi);
  const accSets = untils.map((u) => new Set(nodes.filter((n) => !has(n.old, u) || has(n.old, (u as { b: LTL }).b)).map((n) => n.id)));
  const k = Math.max(1, accSets.length);
  const inSet = (i: number, id: number) => (accSets.length ? accSets[i].has(id) : true);
  // degeneralise: state (node, i)
  const sid = (id: number, i: number) => id * k + i;
  const out: Buchi = { nodes: [], initial: [], succ: new Map(), accepting: new Set(), formula: fmt(f) };
  const byId = new Map(nodes.map((n) => [n.id, n]));
  const succOf = (id: number) => nodes.filter((p) => p.incoming.has(id)).map((p) => p.id);
  for (const n of nodes) for (let i = 0; i < k; i++) {
    const s = sid(n.id, i);
    out.nodes.push({ id: s, pos: n.old.filter((g) => g.k === 'ap').map((g) => (g as { name: string }).name), neg: n.old.filter((g) => g.k === 'not').map((g) => ((g as { a: LTL }).a as { name: string }).name) });
    const ni = inSet(i, n.id) ? (i + 1) % k : i;
    out.succ.set(s, succOf(n.id).map((p) => sid(p, ni)));
    if (i === 0 && inSet(0, n.id)) out.accepting.add(s);
  }
  out.initial = nodes.filter((n) => n.incoming.has(INIT)).map((n) => sid(n.id, 0));
  void byId;
  // keep only states reachable from initial
  const reach = new Set<number>(); const q = [...out.initial]; while (q.length) { const s = q.pop()!; if (reach.has(s)) continue; reach.add(s); for (const t of out.succ.get(s) ?? []) q.push(t); }
  out.nodes = out.nodes.filter((n) => reach.has(n.id)); for (const s of [...out.succ.keys()]) if (!reach.has(s)) out.succ.delete(s);
  out.accepting = new Set([...out.accepting].filter((s) => reach.has(s)));
  return out;
}
function setEq(a: LTL[], b: LTL[]): boolean { if (a.length !== b.length) return false; const kb = new Set(b.map(key)); return a.every((x) => kb.has(key(x))); }

/** Büchi states from which an accepting cycle is reachable (non-empty language). */
export function nonEmptyStates(b: Buchi): Set<number> {
  const ids = b.nodes.map((n) => n.id); const idx = new Map(ids.map((id, i) => [id, i]));
  // Tarjan SCC
  let index = 0; const low = new Map<number, number>(), num = new Map<number, number>(); const st: number[] = []; const on = new Set<number>(); const good = new Set<number>();
  const strong = (v: number) => {
    num.set(v, index); low.set(v, index); index++; st.push(v); on.add(v);
    for (const w of b.succ.get(v) ?? []) { if (!num.has(w)) { strong(w); low.set(v, Math.min(low.get(v)!, low.get(w)!)); } else if (on.has(w)) low.set(v, Math.min(low.get(v)!, num.get(w)!)); }
    if (low.get(v) === num.get(v)) { const comp: number[] = []; let w: number; do { w = st.pop()!; on.delete(w); comp.push(w); } while (w !== v); const cyclic = comp.length > 1 || (b.succ.get(v) ?? []).includes(v); if (cyclic && comp.some((s) => b.accepting.has(s))) for (const s of comp) good.add(s); }
  };
  for (const id of ids) if (!num.has(id)) strong(id);
  // backward closure
  const pred = new Map<number, number[]>(); for (const [s, ts] of b.succ) for (const t of ts) { let a = pred.get(t); if (!a) { a = []; pred.set(t, a); } a.push(s); }
  const q = [...good]; while (q.length) { const t = q.pop()!; for (const s of pred.get(t) ?? []) if (!good.has(s)) { good.add(s); q.push(s); } }
  void idx;
  return good;
}

// ---------------------------------------------------------------------------------------------
// LTL model checking (product + nested DFS)
// ---------------------------------------------------------------------------------------------

export interface LTLResult { holds: boolean; formula: string; /** counterexample: prefix then a cycle (states of the model) */ prefix: string[]; cycle: string[]; buchiStates: number; productStates: number; explanation: string }

export function checkLTL(m: Kripke, formula: string | LTL, opts: { fairness?: string[]; maxProduct?: number } = {}): LTLResult {
  let f = typeof formula === 'string' ? parseLTL(formula) : formula;
  for (const fair of opts.fairness ?? []) f = implies(G(F(parseLTL(fair))), f);
  const b = ltlToBuchi(not(f));
  const label = new Map(b.nodes.map((n) => [n.id, n]));
  const compat = (s: string, q: number): boolean => { const n = label.get(q)!; return n.pos.every((a) => m.holds(s, a)) && n.neg.every((a) => !m.holds(s, a)); };
  const maxProduct = opts.maxProduct ?? 2_000_000;
  const pk = (s: string, q: number) => `${s}${q}`;
  const productSucc = (s: string, q: number): Array<[string, number]> => { const out: Array<[string, number]> = []; for (const t of successors(m, s)) for (const r of b.succ.get(q) ?? []) if (compat(t, r)) out.push([t, r]); return out; };
  // nested DFS (Courcoubetis et al.)
  const visited = new Set<string>(), flagged = new Set<string>();
  let found: { prefix: Array<[string, number]>; cycle: Array<[string, number]> } | null = null;
  const dfs2 = (seed: [string, number], path: Array<[string, number]>): boolean => {
    const stack: Array<{ st: [string, number]; it: Array<[string, number]>; i: number }> = [{ st: seed, it: productSucc(seed[0], seed[1]), i: 0 }];
    const trail: Array<[string, number]> = [seed];
    flagged.add(pk(seed[0], seed[1]));
    while (stack.length) {
      const top = stack[stack.length - 1];
      if (top.i >= top.it.length) { stack.pop(); trail.pop(); continue; }
      const nx = top.it[top.i++];
      if (nx[0] === seed[0] && nx[1] === seed[1]) { found = { prefix: path, cycle: [...trail.slice(1), nx] }; return true; }
      const k = pk(nx[0], nx[1]); if (flagged.has(k)) continue; flagged.add(k);
      stack.push({ st: nx, it: productSucc(nx[0], nx[1]), i: 0 }); trail.push(nx);
    }
    return false;
  };
  const dfs1 = (start: [string, number]): boolean => {
    const stack: Array<{ st: [string, number]; it: Array<[string, number]>; i: number }> = [{ st: start, it: productSucc(start[0], start[1]), i: 0 }];
    const path: Array<[string, number]> = [start];
    visited.add(pk(start[0], start[1]));
    while (stack.length) {
      if (visited.size > maxProduct) throw new Error(`product exceeds ${maxProduct} states`);
      const top = stack[stack.length - 1];
      if (top.i >= top.it.length) { // post-order: nested search from accepting states
        stack.pop(); path.pop();
        if (b.accepting.has(top.st[1]) && dfs2(top.st, [...path, top.st])) return true;
        continue;
      }
      const nx = top.it[top.i++];
      const k = pk(nx[0], nx[1]); if (visited.has(k)) continue; visited.add(k);
      stack.push({ st: nx, it: productSucc(nx[0], nx[1]), i: 0 }); path.push(nx);
    }
    return false;
  };
  let init: [string, number] | null = null;
  for (const s0 of m.initial) for (const q0 of b.initial) { if (!compat(s0, q0)) continue; if (dfs1([s0, q0])) { init = [s0, q0]; break; } if (found) break; }
  let fnd = found as { prefix: Array<[string, number]>; cycle: Array<[string, number]> } | null;
  if (fnd && init) fnd = shortenLasso(init, fnd, productSucc, pk);
  const desc = (s: string) => m.describe?.(s) ?? s;
  return {
    holds: !fnd, formula: fmt(f), prefix: fnd ? fnd.prefix.map((p) => desc(p[0])) : [], cycle: fnd ? fnd.cycle.map((p) => desc(p[0])) : [],
    buchiStates: b.nodes.length, productStates: visited.size,
    explanation: fnd ? `violating run: ${fnd.prefix.map((p) => desc(p[0])).join(' → ')} then repeat ${fnd.cycle.map((p) => desc(p[0])).join(' → ')}` : 'no violating run',
  };
}


/** Replace the DFS lasso by the shortest prefix to the accepting seed and the shortest cycle through it (BFS in the product). */
function shortenLasso(init: [string, number], lasso: { prefix: Array<[string, number]>; cycle: Array<[string, number]> }, succ: (s: string, q: number) => Array<[string, number]>, key: (s: string, q: number) => string, limit = 200_000): { prefix: Array<[string, number]>; cycle: Array<[string, number]> } {
  const seed = lasso.cycle[lasso.cycle.length - 1];
  const bfs = (from: Array<[string, number]>, isTarget: (p: [string, number]) => boolean): Array<[string, number]> | null => {
    const prev = new Map<string, [string, number] | null>(); const queue: Array<[string, number]> = [];
    for (const f of from) { const k = key(f[0], f[1]); if (!prev.has(k)) { prev.set(k, null); queue.push(f); } }
    let head = 0;
    while (head < queue.length && prev.size < limit) {
      const cur = queue[head++];
      if (isTarget(cur)) { const path: Array<[string, number]> = []; let c: [string, number] | null = cur; while (c) { path.push(c); c = prev.get(key(c[0], c[1])) ?? null; } return path.reverse(); }
      for (const n of succ(cur[0], cur[1])) { const k = key(n[0], n[1]); if (!prev.has(k)) { prev.set(k, cur); queue.push(n); } }
    }
    return null;
  };
  const isSeed = (p: [string, number]) => p[0] === seed[0] && p[1] === seed[1];
  const prefix = bfs([init], isSeed);
  const cycle = bfs(succ(seed[0], seed[1]), isSeed);
  if (!prefix || !cycle) return lasso;
  return { prefix, cycle };
}

// ---------------------------------------------------------------------------------------------
// LTL₃ runtime monitors
// ---------------------------------------------------------------------------------------------

export type Verdict = '⊤' | '⊥' | '?';

/** Three-valued monitor: feed letters (set of true atoms / value environment); verdict ⊥ = violated for good, ⊤ = satisfied for good. */
export class LTL3Monitor {
  readonly formula: string;
  private readonly bPos: Buchi; private readonly bNeg: Buchi;
  private readonly goodPos: Set<number>; private readonly goodNeg: Set<number>;
  private labelPos: Map<number, { pos: string[]; neg: string[] }>; private labelNeg: Map<number, { pos: string[]; neg: string[] }>;
  sPos: Set<number>; sNeg: Set<number>;
  fresh = true;
  verdict: Verdict = '?';
  steps = 0;
  readonly atoms: string[];
  constructor(formula: string | LTL) {
    const f = typeof formula === 'string' ? parseLTL(formula) : formula;
    this.formula = fmt(f);
    this.bPos = ltlToBuchi(f); this.bNeg = ltlToBuchi(not(f));
    this.goodPos = nonEmptyStates(this.bPos); this.goodNeg = nonEmptyStates(this.bNeg);
    this.labelPos = new Map(this.bPos.nodes.map((n) => [n.id, n])); this.labelNeg = new Map(this.bNeg.nodes.map((n) => [n.id, n]));
    this.sPos = new Set(this.bPos.initial.filter((s) => this.goodPos.has(s))); this.sNeg = new Set(this.bNeg.initial.filter((s) => this.goodNeg.has(s)));
    this.atoms = [...atoms(f)];
    this.updateVerdict();
  }
  private updateVerdict(): void { if (this.verdict !== '?') return; if (!this.sPos.size) this.verdict = '⊥'; else if (!this.sNeg.size) this.verdict = '⊤'; }
  /** Advance on one observation: atoms true in this step (array / Set) or a value environment. */
  step(obs: Iterable<string> | Record<string, string | number | boolean | undefined>): Verdict {
    if (this.verdict !== '?') return this.verdict;
    const holds = (a: string): boolean => (typeof (obs as Iterable<string>)[Symbol.iterator] === 'function' && typeof obs !== 'string' ? new Set(obs as Iterable<string>).has(a) : atomHolds(a, obs as Record<string, string | number | boolean | undefined>));
    const adv = (cur: Set<number>, b: Buchi, labels: Map<number, { pos: string[]; neg: string[] }>, good: Set<number>): Set<number> => {
      const cand = new Set<number>();
      if (this.fresh) for (const s of cur) cand.add(s); else for (const s of cur) for (const t of b.succ.get(s) ?? []) cand.add(t);
      return new Set([...cand].filter((q) => good.has(q) && labels.get(q)!.pos.every(holds) && labels.get(q)!.neg.every((a) => !holds(a))));
    };
    this.sPos = adv(this.sPos, this.bPos, this.labelPos, this.goodPos);
    this.sNeg = adv(this.sNeg, this.bNeg, this.labelNeg, this.goodNeg);
    this.fresh = false; this.steps++;
    this.updateVerdict();
    return this.verdict;
  }
  reset(): void { this.sPos = new Set(this.bPos.initial.filter((s) => this.goodPos.has(s))); this.sNeg = new Set(this.bNeg.initial.filter((s) => this.goodNeg.has(s))); this.fresh = true; this.verdict = '?'; this.steps = 0; this.updateVerdict(); }
  /** Deterministic monitor table over all valuations of the formula's atoms (for export to a PLC / ROS node). */
  table(): { atoms: string[]; states: Array<{ id: number; verdict: Verdict; next: number[] }>; initial: number } {
    const atomsL = this.atoms; const letters = 1 << atomsL.length;
    const keyOf = (m: LTL3Monitor) => `${[...m.sPos].sort().join(',')}|${[...m.sNeg].sort().join(',')}|${m.fresh}`;
    const states: Array<{ id: number; verdict: Verdict; next: number[] }> = []; const ids = new Map<string, number>();
    const clone = (m: LTL3Monitor): LTL3Monitor => { const c = Object.create(LTL3Monitor.prototype) as LTL3Monitor; Object.assign(c, m); c.sPos = new Set(m.sPos); c.sNeg = new Set(m.sNeg); return c; };
    const start = clone(this); start.fresh = true; start.verdict = '?'; start.sPos = new Set(this.bPos.initial.filter((s) => this.goodPos.has(s))); start.sNeg = new Set(this.bNeg.initial.filter((s) => this.goodNeg.has(s)));
    const queue: LTL3Monitor[] = [start]; ids.set(keyOf(start), 0); states.push({ id: 0, verdict: start.sPos.size ? (start.sNeg.size ? '?' : '⊤') : '⊥', next: [] });
    for (let qi = 0; qi < queue.length; qi++) {
      const cur = queue[qi]; const rec = states[qi];
      for (let l = 0; l < letters; l++) {
        const nxt = clone(cur); nxt.verdict = '?';
        const obs = atomsL.filter((_, i) => l & (1 << i));
        nxt.step(obs);
        const k = keyOf(nxt); let id = ids.get(k);
        if (id === undefined) { id = states.length; ids.set(k, id); states.push({ id, verdict: nxt.verdict, next: [] }); queue.push(nxt); }
        rec.next[l] = id;
      }
    }
    return { atoms: atomsL, states, initial: 0 };
  }
}
