/**
 * Task planning (course chapter 10): STRIPS / PDDL domains and problems (typing, negative preconditions, equality,
 * forall / exists / imply in preconditions, action costs via `increase (total-cost)`, durative actions), grounding,
 * delete-relaxation heuristics h_max / h_add / h_FF with helpful actions, A* and greedy best-first search, plan
 * validation, hierarchical task networks (total-order methods with backtracking), simple temporal networks (STN,
 * Floyd–Warshall) and STNU dynamic controllability (Morris' labelled-edge reductions), a Gantt schedule for durative
 * plans and the plan–execute–replan loop.
 */

// ---------------------------------------------------------------------------------------------
// S-expressions
// ---------------------------------------------------------------------------------------------

export type SExpr = string | SExpr[];

export function parseSExpr(src: string): SExpr[] {
  const tokens = src.replace(/;.*$/gm, '').replace(/\(/g, ' ( ').replace(/\)/g, ' ) ').split(/\s+/).filter(Boolean);
  let i = 0;
  const read = (): SExpr => { const t = tokens[i++]; if (t === '(') { const list: SExpr[] = []; while (tokens[i] !== ')') { if (i >= tokens.length) throw new Error('unbalanced parentheses'); list.push(read()); } i++; return list; } if (t === ')') throw new Error('unexpected )'); return t.toLowerCase(); };
  const out: SExpr[] = []; while (i < tokens.length) out.push(read());
  return out;
}
const isList = (e: SExpr): e is SExpr[] => Array.isArray(e);
const head = (e: SExpr): string => (isList(e) ? String(e[0]) : String(e));

/** Parse a typed list `a b - t c - t2 d` into [{name, type}]. */
function typedList(items: SExpr[]): Array<{ name: string; type: string }> {
  const out: Array<{ name: string; type: string }> = []; let pending: string[] = [];
  for (let i = 0; i < items.length; i++) {
    const it = String(items[i]);
    if (it === '-') { const type = String(items[++i]); for (const p of pending) out.push({ name: p, type }); pending = []; }
    else pending.push(it);
  }
  for (const p of pending) out.push({ name: p, type: 'object' });
  return out;
}

// ---------------------------------------------------------------------------------------------
// Domain / problem model
// ---------------------------------------------------------------------------------------------

export interface Predicate { name: string; params: Array<{ name: string; type: string }> }
export type Cond =
  | { k: 'atom'; name: string; args: string[] } | { k: 'not'; a: Cond } | { k: 'and'; items: Cond[] } | { k: 'or'; items: Cond[] }
  | { k: 'eq'; a: string; b: string } | { k: 'forall'; vars: Array<{ name: string; type: string }>; body: Cond } | { k: 'exists'; vars: Array<{ name: string; type: string }>; body: Cond } | { k: 'imply'; a: Cond; b: Cond }
  | { k: 'true' };
export interface Effect { add: Array<{ name: string; args: string[] }>; del: Array<{ name: string; args: string[] }>; cost: Array<{ fn: string; args: string[]; constant?: number }>; /** conditional effects: (when c e) */ when?: Array<{ cond: Cond; effect: Effect }>; /** universal effects: (forall (vars) e) */ forall?: Array<{ vars: Array<{ name: string; type: string }>; effect: Effect }> }
export interface ActionSchema { name: string; params: Array<{ name: string; type: string }>; pre: Cond; eff: Effect; duration?: number | { fn: string; args: string[] }; /** durative: which conditions / effects are at start / at end */ temporal?: { atStartEff: Effect; atEndEff: Effect; overAll: Cond; atStart: Cond; atEnd: Cond } }
export interface Domain { name: string; types: Map<string, string>; predicates: Predicate[]; functions: string[]; actions: ActionSchema[]; requirements: string[] }
export interface Problem { name: string; domain: string; objects: Array<{ name: string; type: string }>; init: Array<{ name: string; args: string[] }>; fluents: Map<string, number>; goal: Cond; metric?: 'minimize' | 'maximize' }

function parseCond(e: SExpr): Cond {
  if (!isList(e)) return { k: 'atom', name: String(e), args: [] };
  if (!e.length) return { k: 'true' };
  const h = head(e);
  switch (h) {
    case 'and': return { k: 'and', items: e.slice(1).map(parseCond) };
    case 'or': return { k: 'or', items: e.slice(1).map(parseCond) };
    case 'not': return { k: 'not', a: parseCond(e[1]) };
    case '=': return { k: 'eq', a: String(e[1]), b: String(e[2]) };
    case 'imply': return { k: 'imply', a: parseCond(e[1]), b: parseCond(e[2]) };
    case 'forall': case 'exists': return { k: h, vars: typedList(e[1] as SExpr[]), body: parseCond(e[2]) };
    case 'at': case 'over': return parseCond(e[2]); // durative wrappers (at start φ) handled by the caller
    default: return { k: 'atom', name: h, args: e.slice(1).map(String) };
  }
}
function parseEffect(e: SExpr, eff: Effect = { add: [], del: [], cost: [] }): Effect {
  if (!isList(e) || !e.length) return eff;
  const h = head(e);
  if (h === 'and') { for (const x of e.slice(1)) parseEffect(x, eff); return eff; }
  if (h === 'not') { const a = e[1] as SExpr[]; eff.del.push({ name: head(a), args: a.slice(1).map(String) }); return eff; }
  if (h === 'increase') { const f = e[1] as SExpr[]; const v = e[2]; if (isList(v)) eff.cost.push({ fn: head(v), args: v.slice(1).map(String) }); else eff.cost.push({ fn: head(f), args: [], constant: Number(v) }); return eff; }
  if (h === 'when') { eff.when = eff.when ?? []; eff.when.push({ cond: parseCond(e[1]), effect: parseEffect(e[2]) }); return eff; }
  if (h === 'forall') { eff.forall = eff.forall ?? []; eff.forall.push({ vars: typedList(e[1] as SExpr[]), effect: parseEffect(e[2]) }); return eff; }
  if (h === 'at' || h === 'over') return parseEffect(e[2], eff);
  eff.add.push({ name: h, args: e.slice(1).map(String) });
  return eff;
}

export function parseDomain(src: string): Domain {
  const top = parseSExpr(src).find((e) => isList(e) && head(e) === 'define') as SExpr[];
  if (!top) throw new Error('no (define (domain …))');
  const d: Domain = { name: String((top[1] as SExpr[])[1]), types: new Map(), predicates: [], functions: [], actions: [], requirements: [] };
  for (const sec of top.slice(2) as SExpr[][]) {
    switch (head(sec)) {
      case ':requirements': d.requirements = sec.slice(1).map(String); break;
      case ':types': for (const t of typedList(sec.slice(1))) d.types.set(t.name, t.type); break;
      case ':constants': break;
      case ':predicates': for (const p of sec.slice(1) as SExpr[][]) d.predicates.push({ name: head(p), params: typedList(p.slice(1)) }); break;
      case ':functions': for (const f of sec.slice(1)) if (isList(f)) d.functions.push(head(f)); break;
      case ':action': case ':durative-action': {
        const a: ActionSchema = { name: String(sec[1]), params: [], pre: { k: 'true' }, eff: { add: [], del: [], cost: [] } };
        for (let i = 2; i < sec.length; i += 2) {
          const key = String(sec[i]), val = sec[i + 1];
          if (key === ':parameters') a.params = typedList(val as SExpr[]);
          else if (key === ':precondition') a.pre = parseCond(val);
          else if (key === ':effect') a.eff = parseEffect(val);
          else if (key === ':duration') { const v = (val as SExpr[])[2]; a.duration = isList(v) ? { fn: head(v), args: v.slice(1).map(String) } : Number(v); }
          else if (key === ':condition') {
            const parts = (isList(val) && head(val) === 'and' ? val.slice(1) : [val]) as SExpr[][];
            const pick = (w: string, t: string) => ({ k: 'and', items: parts.filter((p) => head(p) === w && String(p[1]) === t).map((p) => parseCond(p[2])) } as Cond);
            a.temporal = { ...(a.temporal ?? { atStartEff: { add: [], del: [], cost: [] }, atEndEff: { add: [], del: [], cost: [] } }), atStart: pick('at', 'start'), overAll: pick('over', 'all'), atEnd: pick('at', 'end') } as ActionSchema['temporal'];
            a.pre = { k: 'and', items: [a.temporal!.atStart, a.temporal!.overAll, a.temporal!.atEnd] };
          }
        }
        if (head(sec) === ':durative-action') {
          // split effects at start / at end for scheduling; sequential semantics for planning
          const effSrc = sec[sec.indexOf(':effect') + 1] as SExpr[];
          const parts = (isList(effSrc) && head(effSrc) === 'and' ? effSrc.slice(1) : [effSrc]) as SExpr[][];
          const atStartEff = { add: [], del: [], cost: [] } as Effect, atEndEff = { add: [], del: [], cost: [] } as Effect;
          for (const p of parts) { if (head(p) === 'at' && String(p[1]) === 'start') parseEffect(p[2], atStartEff); else parseEffect(isList(p) && head(p) === 'at' ? p[2] : p, atEndEff); }
          a.temporal = { atStart: { k: 'true' }, overAll: { k: 'true' }, atEnd: { k: 'true' }, ...(a.temporal ?? {}), atStartEff, atEndEff } as ActionSchema['temporal'];
          // a fact deleted at start and re-added at end (a resource held during the action) is not a net effect
          const same = (x: { name: string; args: string[] }, y: { name: string; args: string[] }) => x.name === y.name && x.args.join() === y.args.join();
          const netAdd = [...atEndEff.add, ...atStartEff.add.filter((x) => !atEndEff.del.some((y) => same(x, y)))];
          const netDel = [...atEndEff.del, ...atStartEff.del.filter((x) => !atEndEff.add.some((y) => same(x, y)))];
          a.eff = { add: netAdd, del: netDel, cost: [...atStartEff.cost, ...atEndEff.cost] };
        }
        d.actions.push(a); break;
      }
    }
  }
  return d;
}

export function parseProblem(src: string): Problem {
  const top = parseSExpr(src).find((e) => isList(e) && head(e) === 'define') as SExpr[];
  if (!top) throw new Error('no (define (problem …))');
  const p: Problem = { name: String((top[1] as SExpr[])[1]), domain: '', objects: [], init: [], fluents: new Map(), goal: { k: 'true' } };
  for (const sec of top.slice(2) as SExpr[][]) {
    switch (head(sec)) {
      case ':domain': p.domain = String(sec[1]); break;
      case ':objects': p.objects = typedList(sec.slice(1)); break;
      case ':init': for (const f of sec.slice(1) as SExpr[][]) { if (head(f) === '=') { const fn = f[1] as SExpr[]; p.fluents.set([head(fn), ...fn.slice(1).map(String)].join(' '), Number(f[2])); } else p.init.push({ name: head(f), args: f.slice(1).map(String) }); } break;
      case ':goal': p.goal = parseCond(sec[1]); break;
      case ':metric': p.metric = String(sec[1]) as 'minimize'; break;
    }
  }
  return p;
}

// ---------------------------------------------------------------------------------------------
// Grounding
// ---------------------------------------------------------------------------------------------

export interface GroundAction { name: string; schema: ActionSchema; args: string[]; pre: number[]; preNeg: number[]; add: number[]; del: number[]; cost: number; duration: number }
export interface GroundTask { atoms: string[]; atomIndex: Map<string, number>; actions: GroundAction[]; init: Set<number>; goalPos: number[]; goalNeg: number[]; objects: Array<{ name: string; type: string }>; domain: Domain; problem: Problem }

function subtypeOf(d: Domain, t: string, target: string): boolean { if (target === 'object' || t === target) return true; let cur: string | undefined = t; for (let i = 0; i < 50 && cur; i++) { if (cur === target) return true; cur = d.types.get(cur); } return false; }
function objectsOfType(d: Domain, objs: Array<{ name: string; type: string }>, t: string): string[] { return objs.filter((o) => subtypeOf(d, o.type, t)).map((o) => o.name); }
const atomKey = (name: string, args: string[]) => `${name}(${args.join(',')})`;

/** Instantiate a condition into positive / negative literal sets (disjunctions are expanded by the caller into alternatives). */
function instantiate(c: Cond, sub: Map<string, string>, d: Domain, objs: Array<{ name: string; type: string }>): Array<{ pos: string[]; neg: string[] }> | null {
  const s = (x: string) => sub.get(x) ?? x;
  switch (c.k) {
    case 'true': return [{ pos: [], neg: [] }];
    case 'atom': return [{ pos: [atomKey(c.name, c.args.map(s))], neg: [] }];
    case 'not': { if (c.a.k === 'atom') return [{ pos: [], neg: [atomKey(c.a.name, c.a.args.map(s))] }]; if (c.a.k === 'eq') return s(c.a.a) !== s(c.a.b) ? [{ pos: [], neg: [] }] : null; const inner = instantiate(c.a, sub, d, objs); if (!inner) return [{ pos: [], neg: [] }]; if (inner.length === 1 && inner[0].pos.length + inner[0].neg.length === 0) return null; return inner.flatMap((alt): Array<{ pos: string[]; neg: string[] }> => [...alt.pos.map((p) => ({ pos: [] as string[], neg: [p] })), ...alt.neg.map((n) => ({ pos: [n], neg: [] as string[] }))]); }
    case 'eq': return s(c.a) === s(c.b) ? [{ pos: [], neg: [] }] : null;
    case 'and': { let alts: Array<{ pos: string[]; neg: string[] }> = [{ pos: [], neg: [] }]; for (const it of c.items) { const r = instantiate(it, sub, d, objs); if (!r) return null; const next: typeof alts = []; for (const a of alts) for (const b of r) next.push({ pos: [...a.pos, ...b.pos], neg: [...a.neg, ...b.neg] }); alts = next; if (alts.length > 512) alts = alts.slice(0, 512); } return alts; }
    case 'or': { const out: Array<{ pos: string[]; neg: string[] }> = []; for (const it of c.items) { const r = instantiate(it, sub, d, objs); if (r) out.push(...r); } return out.length ? out : null; }
    case 'imply': return instantiate({ k: 'or', items: [{ k: 'not', a: c.a }, c.b] }, sub, d, objs);
    case 'forall': case 'exists': {
      let subs: Map<string, string>[] = [sub];
      for (const v of c.vars) { const next: Map<string, string>[] = []; for (const sb of subs) for (const o of objectsOfType(d, objs, v.type)) { const m = new Map(sb); m.set(v.name, o); next.push(m); } subs = next; }
      if (c.k === 'forall') return instantiate({ k: 'and', items: subs.map((sb) => bind(c.body, sb)) }, sub, d, objs);
      return instantiate({ k: 'or', items: subs.map((sb) => bind(c.body, sb)) }, sub, d, objs);
    }
  }
}
/** Substitute bound variables in a condition. */
function bind(c: Cond, sub: Map<string, string>): Cond {
  const s = (x: string) => sub.get(x) ?? x;
  switch (c.k) {
    case 'atom': return { k: 'atom', name: c.name, args: c.args.map(s) };
    case 'not': return { k: 'not', a: bind(c.a, sub) };
    case 'and': case 'or': return { k: c.k, items: c.items.map((i) => bind(i, sub)) };
    case 'eq': return { k: 'eq', a: s(c.a), b: s(c.b) };
    case 'imply': return { k: 'imply', a: bind(c.a, sub), b: bind(c.b, sub) };
    case 'forall': case 'exists': return { k: c.k, vars: c.vars, body: bind(c.body, sub) };
    default: return c;
  }
}

/** Substitute and expand universal effects into flat add / delete lists. */
function expandEffect(eff: Effect, sub: Map<string, string>, d: Domain, objs: Array<{ name: string; type: string }>): { add: Array<{ name: string; args: string[] }>; del: Array<{ name: string; args: string[] }> } {
  const s = (x: string) => sub.get(x) ?? x;
  const add = eff.add.map((e) => ({ name: e.name, args: e.args.map(s) })), del = eff.del.map((e) => ({ name: e.name, args: e.args.map(s) }));
  for (const fa of eff.forall ?? []) {
    let subs = [sub];
    for (const v of fa.vars) { const next: Map<string, string>[] = []; for (const sb of subs) for (const o of objectsOfType(d, objs, v.type)) { const m = new Map(sb); m.set(v.name, o); next.push(m); } subs = next; }
    for (const sb of subs) { const r = expandEffect(fa.effect, sb, d, objs); add.push(...r.add); del.push(...r.del); }
  }
  return { add, del };
}

export function ground(d: Domain, p: Problem): GroundTask {
  const atomIndex = new Map<string, number>(); const atoms: string[] = [];
  const idx = (k: string) => { let i = atomIndex.get(k); if (i === undefined) { i = atoms.length; atoms.push(k); atomIndex.set(k, i); } return i; };
  for (const f of p.init) idx(atomKey(f.name, f.args));
  const actions: GroundAction[] = [];
  const costOf = (eff: Effect, sub: Map<string, string>) => eff.cost.reduce((s, c) => s + (c.constant !== undefined ? c.constant : p.fluents.get([c.fn, ...c.args.map((a) => sub.get(a) ?? a)].join(' ')) ?? 0), 0);
  for (const a of d.actions) {
    let subs: Map<string, string>[] = [new Map()];
    for (const v of a.params) { const next: Map<string, string>[] = []; for (const sb of subs) for (const o of objectsOfType(d, p.objects, v.type)) { const m = new Map(sb); m.set(v.name, o); next.push(m); } subs = next; }
    for (const sub of subs) {
      const alts = instantiate(a.pre, sub, d, p.objects); if (!alts) continue;
      const s = (x: string) => sub.get(x) ?? x;
      const flat = expandEffect(a.eff, sub, d, p.objects);
      const add = flat.add.map((e) => idx(atomKey(e.name, e.args))), del = flat.del.map((e) => idx(atomKey(e.name, e.args)));
      const cost = a.eff.cost.length ? costOf(a.eff, sub) : 1;
      const duration = typeof a.duration === 'number' ? a.duration : a.duration ? p.fluents.get([a.duration.fn, ...a.duration.args.map(s)].join(' ')) ?? 1 : cost;
      const args = a.params.map((v) => sub.get(v.name)!);
      for (const alt of alts) actions.push({ name: `${a.name}${args.length ? ' ' + args.join(' ') : ''}`, schema: a, args, pre: [...new Set(alt.pos.map(idx))], preNeg: [...new Set(alt.neg.map(idx))], add, del, cost, duration });
    }
  }
  const galts = instantiate(p.goal, new Map(), d, p.objects) ?? [{ pos: [], neg: [] }];
  const g = galts[0];
  return { atoms, atomIndex, actions, init: new Set(p.init.map((f) => idx(atomKey(f.name, f.args)))), goalPos: g.pos.map(idx), goalNeg: g.neg.map(idx), objects: p.objects, domain: d, problem: p };
}

// ---------------------------------------------------------------------------------------------
// States, heuristics and search
// ---------------------------------------------------------------------------------------------

const applicable = (a: GroundAction, s: Set<number>) => a.pre.every((x) => s.has(x)) && a.preNeg.every((x) => !s.has(x));
export const applyAction = (a: GroundAction, s: Set<number>): Set<number> => { const n = new Set(s); for (const x of a.del) n.delete(x); for (const x of a.add) n.add(x); return n; };
const isGoal = (t: GroundTask, s: Set<number>) => t.goalPos.every((x) => s.has(x)) && t.goalNeg.every((x) => !s.has(x));
const stateKey = (s: Set<number>) => [...s].sort((a, b) => a - b).join(',');

export type Heuristic = 'hmax' | 'hadd' | 'hff' | 'blind' | 'goalcount';

/** Delete-relaxation heuristics: returns h value and, for hff, the helpful actions (first layer of the relaxed plan). */
export function relaxedHeuristic(t: GroundTask, s: Set<number>, kind: Heuristic): { h: number; helpful: Set<GroundAction> } {
  const helpful = new Set<GroundAction>();
  if (kind === 'blind') return { h: isGoal(t, s) ? 0 : 1, helpful };
  if (kind === 'goalcount') return { h: t.goalPos.filter((x) => !s.has(x)).length + t.goalNeg.filter((x) => s.has(x)).length, helpful };
  // atom costs by generalized Dijkstra over the relaxed task (negative preconditions ignored)
  const cost = new Map<number, number>(); for (const x of s) cost.set(x, 0);
  const achiever = new Map<number, GroundAction>();
  const combine = (vals: number[]) => (kind === 'hmax' ? Math.max(0, ...vals) : vals.reduce((a, b) => a + b, 0));
  let changed = true; let iter = 0;
  while (changed && iter++ < 10000) {
    changed = false;
    for (const a of t.actions) {
      if (!a.pre.every((x) => cost.has(x))) continue;
      const c = combine(a.pre.map((x) => cost.get(x)!)) + a.cost;
      for (const x of a.add) if (!cost.has(x) || cost.get(x)! > c) { cost.set(x, c); achiever.set(x, a); changed = true; }
    }
  }
  if (!t.goalPos.every((x) => cost.has(x))) return { h: Infinity, helpful };
  if (kind !== 'hff') return { h: combine(t.goalPos.map((x) => cost.get(x)!)), helpful };
  // relaxed plan extraction (backchaining on best achievers)
  const plan = new Set<GroundAction>(); const open = [...t.goalPos.filter((x) => !s.has(x))]; const seen = new Set(open);
  while (open.length) {
    const x = open.pop()!; const a = achiever.get(x); if (!a) continue;
    plan.add(a);
    for (const y of a.pre) if (!s.has(y) && !seen.has(y)) { seen.add(y); open.push(y); }
  }
  for (const a of plan) if (a.pre.every((x) => s.has(x))) helpful.add(a);
  return { h: [...plan].reduce((sum, a) => sum + a.cost, 0), helpful };
}

export interface PlanResult { found: boolean; plan: GroundAction[]; cost: number; expanded: number; generated: number; timeMs: number; heuristic: Heuristic; search: 'astar' | 'gbfs'; message?: string }

/** Forward state-space search: A* (optimal with admissible h_max) or greedy best-first (h_FF with helpful actions). */
export function plan(t: GroundTask, opts: { search?: 'astar' | 'gbfs'; heuristic?: Heuristic; maxExpansions?: number; helpfulOnly?: boolean; timeLimitMs?: number } = {}): PlanResult {
  const search = opts.search ?? 'gbfs', heuristic = opts.heuristic ?? (search === 'astar' ? 'hmax' : 'hff');
  const t0 = Date.now(); const maxExp = opts.maxExpansions ?? 200000;
  type Node = { s: Set<number>; g: number; h: number; parent: Node | null; action: GroundAction | null; helpful: Set<GroundAction> };
  const h0 = relaxedHeuristic(t, t.init, heuristic);
  if (h0.h === Infinity) return { found: false, plan: [], cost: 0, expanded: 0, generated: 0, timeMs: 0, heuristic, search, message: 'goal unreachable in the relaxed task' };
  const open: Node[] = [{ s: t.init, g: 0, h: h0.h, parent: null, action: null, helpful: h0.helpful }];
  const best = new Map<string, number>([[stateKey(t.init), 0]]);
  let expanded = 0, generated = 1;
  const f = (n: Node) => (search === 'astar' ? n.g + n.h : n.h);
  while (open.length) {
    if (expanded > maxExp) return { found: false, plan: [], cost: 0, expanded, generated, timeMs: Date.now() - t0, heuristic, search, message: 'expansion limit reached' };
    if (opts.timeLimitMs && Date.now() - t0 > opts.timeLimitMs) return { found: false, plan: [], cost: 0, expanded, generated, timeMs: Date.now() - t0, heuristic, search, message: 'time limit reached' };
    // pop best (tie-break on lower h, then FIFO)
    let bi = 0; for (let i = 1; i < open.length; i++) if (f(open[i]) < f(open[bi]) || (f(open[i]) === f(open[bi]) && open[i].h < open[bi].h)) bi = i;
    const n = open.splice(bi, 1)[0];
    if (isGoal(t, n.s)) { const acts: GroundAction[] = []; let c: Node | null = n; while (c && c.action) { acts.push(c.action); c = c.parent; } return { found: true, plan: acts.reverse(), cost: n.g, expanded, generated, timeMs: Date.now() - t0, heuristic, search }; }
    expanded++;
    let cands = t.actions.filter((a) => applicable(a, n.s));
    if (search === 'gbfs' && opts.helpfulOnly !== false && heuristic === 'hff' && n.helpful.size) { const hp = cands.filter((a) => n.helpful.has(a)); if (hp.length) cands = hp; }
    for (const a of cands) {
      const s2 = applyAction(a, n.s); const k = stateKey(s2); const g2 = n.g + a.cost;
      if (best.has(k) && best.get(k)! <= g2) continue;
      best.set(k, g2);
      const hh = relaxedHeuristic(t, s2, heuristic); if (hh.h === Infinity) continue;
      open.push({ s: s2, g: g2, h: hh.h, parent: n, action: a, helpful: hh.helpful }); generated++;
    }
  }
  return { found: false, plan: [], cost: 0, expanded, generated, timeMs: Date.now() - t0, heuristic, search, message: 'search space exhausted' };
}

/** Validate a plan against the task: every action applicable, goal reached. */
export function validatePlan(t: GroundTask, actions: GroundAction[]): { valid: boolean; failedAt: number; reason: string; states: Set<number>[] } {
  let s = t.init; const states = [s];
  for (let i = 0; i < actions.length; i++) { const a = actions[i]; if (!applicable(a, s)) return { valid: false, failedAt: i, reason: `precondition of ${a.name} not satisfied`, states }; s = applyAction(a, s); states.push(s); }
  return isGoal(t, s) ? { valid: true, failedAt: -1, reason: 'ok', states } : { valid: false, failedAt: actions.length, reason: 'goal not reached', states };
}

export function findAction(t: GroundTask, name: string): GroundAction | undefined { return t.actions.find((a) => a.name === name); }

// ---------------------------------------------------------------------------------------------
// Durative plans → Gantt (as-soon-as-possible with causal and resource (mutex) precedence)
// ---------------------------------------------------------------------------------------------

export interface GanttRow { action: string; start: number; end: number; duration: number }

/** Schedule a sequential plan of durative actions: an action starts after every earlier action it depends on (shares an atom in add/del/pre) has ended. */
export function scheduleGantt(actions: GroundAction[]): { rows: GanttRow[]; makespan: number } {
  const rows: GanttRow[] = [];
  for (let i = 0; i < actions.length; i++) {
    const a = actions[i]; let start = 0;
    for (let j = 0; j < i; j++) {
      const b = actions[j];
      const touch = (x: number[], y: number[]) => x.some((v) => y.includes(v));
      const dep = touch(b.add, a.pre) || touch(b.del, a.preNeg) || touch(b.add, a.del) || touch(b.del, a.add) || touch(a.del, b.pre) || touch(a.add, b.preNeg) || touch(a.del, b.del);
      if (dep) start = Math.max(start, rows[j].end);
    }
    rows.push({ action: a.name, start, end: start + a.duration, duration: a.duration });
  }
  return { rows, makespan: rows.reduce((m, r) => Math.max(m, r.end), 0) };
}

// ---------------------------------------------------------------------------------------------
// HTN (total-order decomposition with backtracking)
// ---------------------------------------------------------------------------------------------

export interface HTNMethod { task: string; name?: string; params: string[]; /** precondition over the current state (grounded atoms after substitution) */ pre: Cond; subtasks: Array<{ name: string; args: string[] }> }
export interface HTNDomain { methods: HTNMethod[] }

/** Decompose the initial task network; primitive tasks are ground actions of the STRIPS task. */
export function htnPlan(t: GroundTask, htn: HTNDomain, tasks: Array<{ name: string; args: string[] }>, opts: { maxDepth?: number; maxNodes?: number } = {}): { found: boolean; plan: GroundAction[]; nodes: number; trace: string[] } {
  const maxDepth = opts.maxDepth ?? 30, maxNodes = opts.maxNodes ?? 100000;
  let nodes = 0; const trace: string[] = [];
  const primitive = new Map<string, GroundAction[]>(); for (const a of t.actions) { const k = a.schema.name; let l = primitive.get(k); if (!l) { l = []; primitive.set(k, l); } l.push(a); }
  const solve = (s: Set<number>, agenda: Array<{ name: string; args: string[] }>, depth: number): GroundAction[] | null => {
    if (++nodes > maxNodes || depth > maxDepth) return null;
    if (!agenda.length) return [];
    const [task, ...rest] = agenda;
    const prims = primitive.get(task.name);
    if (prims) {
      const a = prims.find((x) => x.args.join() === task.args.join());
      if (!a || !applicable(a, s)) { trace.push(`✗ ${task.name} ${task.args.join(' ')} not applicable`); return null; }
      const tail = solve(applyAction(a, s), rest, depth); return tail ? [a, ...tail] : null;
    }
    for (const m of htn.methods) {
      if (m.task !== task.name || m.params.length !== task.args.length) continue;
      const sub = new Map(m.params.map((p, i) => [p, task.args[i]]));
      // unbound method variables (appearing in subtasks / pre but not params) range over objects
      const free = [...new Set([...m.subtasks.flatMap((st) => st.args), ...condVars(m.pre)].filter((v) => v.startsWith('?') && !sub.has(v)))];
      let subs = [sub];
      for (const v of free) { const next: Map<string, string>[] = []; for (const sb of subs) for (const o of t.objects) { const mm = new Map(sb); mm.set(v, o.name); next.push(mm); } subs = next; }
      for (const sb of subs) {
        const alts = instantiate(m.pre, sb, t.domain, t.objects); if (!alts) continue;
        const ok = alts.some((alt) => alt.pos.every((k) => t.atomIndex.has(k) && s.has(t.atomIndex.get(k)!)) && alt.neg.every((k) => !t.atomIndex.has(k) || !s.has(t.atomIndex.get(k)!)));
        if (!ok) continue;
        trace.push(`${'  '.repeat(depth)}${task.name}(${task.args.join(',')}) ← ${m.name ?? m.task}`);
        const expanded = m.subtasks.map((st) => ({ name: st.name, args: st.args.map((a) => sb.get(a) ?? a) }));
        const r = solve(s, [...expanded, ...rest], depth + 1);
        if (r) return r;
      }
    }
    return null;
  };
  const p = solve(t.init, tasks, 0);
  return { found: !!p, plan: p ?? [], nodes, trace };
}
function condVars(c: Cond): string[] { switch (c.k) { case 'atom': return c.args; case 'not': return condVars(c.a); case 'and': case 'or': return c.items.flatMap(condVars); case 'eq': return [c.a, c.b]; case 'imply': return [...condVars(c.a), ...condVars(c.b)]; case 'forall': case 'exists': return condVars(c.body).filter((v) => !c.vars.some((x) => x.name === v)); default: return []; } }

// ---------------------------------------------------------------------------------------------
// Simple temporal networks and STNU dynamic controllability
// ---------------------------------------------------------------------------------------------

export interface STNConstraint { from: string; to: string; min: number; max: number; /** contingent (uncontrollable duration) link */ contingent?: boolean }
export interface STNResult { consistent: boolean; nodes: string[]; /** distance matrix d[i][j] = max (t_j − t_i) */ dist: number[][]; earliest: Record<string, number>; latest: Record<string, number>; negativeCycle: string[] | null; makespan: [number, number] }

/** STN consistency and bounds by Floyd–Warshall on the distance graph (Theorem 10.5); node `z` (or the first node) is the reference. */
export function solveSTN(constraints: STNConstraint[], reference?: string): STNResult {
  const nodes = [...new Set(constraints.flatMap((c) => [c.from, c.to]))];
  const ref = reference ?? (nodes.includes('z') ? 'z' : nodes[0]);
  if (!nodes.includes(ref)) nodes.unshift(ref);
  const n = nodes.length; const idx = new Map(nodes.map((v, i) => [v, i]));
  const d = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 0 : Infinity)));
  for (const c of constraints) { const i = idx.get(c.from)!, j = idx.get(c.to)!; d[i][j] = Math.min(d[i][j], c.max); d[j][i] = Math.min(d[j][i], -c.min); }
  for (let k = 0; k < n; k++) for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) if (d[i][k] + d[k][j] < d[i][j]) d[i][j] = d[i][k] + d[k][j];
  const neg = nodes.findIndex((_, i) => d[i][i] < 0);
  const r = idx.get(ref)!;
  const earliest: Record<string, number> = {}, latest: Record<string, number> = {};
  nodes.forEach((v, i) => { earliest[v] = -d[i][r]; latest[v] = d[r][i]; });
  const finite = nodes.filter((v) => Number.isFinite(earliest[v]));
  const makespan: [number, number] = [Math.max(0, ...finite.map((v) => earliest[v])), Math.max(0, ...finite.map((v) => latest[v]))];
  return { consistent: neg < 0, nodes, dist: d, earliest, latest, negativeCycle: neg >= 0 ? [nodes[neg]] : null, makespan };
}

interface LEdge { u: number; v: number; w: number; /** '' ordinary, 'U:X' upper-case, 'L:X' lower-case */ label: string }

/**
 * STNU dynamic controllability (Morris 2006 reductions: no-case, upper-case, lower-case, cross-case, label removal)
 * iterated to a fixed point; the network is DC iff no negative cycle of ordinary / upper-case edges appears.
 */
export function checkSTNU(constraints: STNConstraint[], opts: { maxRounds?: number } = {}): { controllable: boolean; consistent: boolean; rounds: number; witness: string | null; edges: number } {
  const nodes = [...new Set(constraints.flatMap((c) => [c.from, c.to]))]; const idx = new Map(nodes.map((v, i) => [v, i])); const n = nodes.length;
  const stn = solveSTN(constraints);
  if (!stn.consistent) return { controllable: false, consistent: false, rounds: 0, witness: 'inconsistent as an STN', edges: 0 };
  const contingent = new Map<number, { a: number; l: number; u: number }>();
  const edges = new Map<string, LEdge>();
  const put = (e: LEdge): boolean => { const k = `${e.u}>${e.v}|${e.label}`; const cur = edges.get(k); if (cur && cur.w <= e.w) return false; edges.set(k, e); return true; };
  for (const c of constraints) {
    const a = idx.get(c.from)!, b = idx.get(c.to)!;
    if (c.contingent) { contingent.set(b, { a, l: c.min, u: c.max }); put({ u: a, v: b, w: c.max, label: '' }); put({ u: b, v: a, w: -c.min, label: '' }); put({ u: a, v: b, w: c.min, label: `L:${b}` }); put({ u: b, v: a, w: -c.max, label: `U:${b}` }); }
    else { put({ u: a, v: b, w: c.max, label: '' }); put({ u: b, v: a, w: -c.min, label: '' }); }
  }
  const maxRounds = opts.maxRounds ?? 4 * n + 4; let rounds = 0;
  for (; rounds < maxRounds; rounds++) {
    let changed = false;
    const list = [...edges.values()];
    for (const e1 of list) for (const e2 of list) {
      if (e1.v !== e2.u) continue;
      const l1 = e1.label, l2 = e2.label;
      if (l1 === '' && l2 === '') changed = put({ u: e1.u, v: e2.v, w: e1.w + e2.w, label: '' }) || changed; // no case
      else if (l1 === '' && l2.startsWith('U:')) changed = put({ u: e1.u, v: e2.v, w: e1.w + e2.w, label: l2 }) || changed; // upper case
      else if (l1.startsWith('L:') && l2 === '' && e2.w < 0) changed = put({ u: e1.u, v: e2.v, w: e1.w + e2.w, label: '' }) || changed; // lower case
      else if (l1.startsWith('L:') && l2.startsWith('U:') && e2.w < 0 && l1.slice(2) !== l2.slice(2)) changed = put({ u: e1.u, v: e2.v, w: e1.w + e2.w, label: l2 }) || changed; // cross case
    }
    // label removal: B --C:x--> A with x ≥ −l_C becomes ordinary
    for (const e of [...edges.values()]) if (e.label.startsWith('U:')) { const c = contingent.get(Number(e.label.slice(2)))!; if (e.w >= -c.l) changed = put({ u: e.u, v: e.v, w: e.w, label: '' }) || changed; }
    // negative cycle check on ordinary + upper-case edges (Bellman–Ford from a virtual source)
    const dist = new Array(n).fill(0);
    const rel = [...edges.values()].filter((e) => !e.label.startsWith('L:'));
    for (let it = 0; it < n; it++) { let upd = false; for (const e of rel) if (dist[e.u] + e.w < dist[e.v] - 1e-9) { dist[e.v] = dist[e.u] + e.w; upd = true; } if (!upd) break; if (it === n - 1) return { controllable: false, consistent: true, rounds: rounds + 1, witness: `negative cycle through ${nodes[e_negWitness(rel, dist)]}`, edges: edges.size }; }
    if (edges.size > 20000) break;
    if (!changed) break;
  }
  return { controllable: true, consistent: true, rounds, witness: null, edges: edges.size };
}
function e_negWitness(rel: LEdge[], dist: number[]): number { for (const e of rel) if (dist[e.u] + e.w < dist[e.v] - 1e-9) return e.v; return 0; }

// ---------------------------------------------------------------------------------------------
// Plan – execute – replan
// ---------------------------------------------------------------------------------------------

export interface ExecutorHooks { /** execute a ground action in the world; return whether its post-condition (add effects) was reached */ execute(a: GroundAction, state: Set<number>): boolean | Promise<boolean>; /** observe the world: optionally correct the symbolic state (atoms added/removed) */ observe?(state: Set<number>): Set<number>; onReplan?(reason: string): void }

export async function executeWithReplanning(t: GroundTask, hooks: ExecutorHooks, opts: { maxReplans?: number; planOpts?: Parameters<typeof plan>[1] } = {}): Promise<{ success: boolean; executed: GroundAction[]; replans: number; log: string[] }> {
  let task: GroundTask = { ...t, init: new Set(t.init) }; const executed: GroundAction[] = []; const log: string[] = []; let replans = 0;
  let current = plan(task, opts.planOpts);
  if (!current.found) return { success: false, executed, replans, log: [`no plan: ${current.message}`] };
  let state = task.init; let i = 0;
  while (true) {
    if (hooks.observe) state = hooks.observe(state);
    if (isGoal(t, state)) return { success: true, executed, replans, log };
    if (i >= current.plan.length) { if (replans >= (opts.maxReplans ?? 10)) return { success: false, executed, replans, log: [...log, 'plan exhausted'] }; replans++; hooks.onReplan?.('plan exhausted without reaching the goal'); task = { ...task, init: state }; current = plan(task, opts.planOpts); i = 0; if (!current.found) return { success: false, executed, replans, log: [...log, 'replanning failed'] }; continue; }
    const a = current.plan[i];
    if (!applicable(a, state)) { if (replans >= (opts.maxReplans ?? 10)) return { success: false, executed, replans, log: [...log, `precondition of ${a.name} violated; replan limit`] }; replans++; log.push(`precondition of ${a.name} violated → replan`); hooks.onReplan?.(`precondition of ${a.name} violated`); task = { ...task, init: state }; current = plan(task, opts.planOpts); i = 0; if (!current.found) return { success: false, executed, replans, log: [...log, 'replanning failed'] }; continue; }
    const ok = await hooks.execute(a, state);
    executed.push(a); log.push(`${ok ? '✓' : '✗'} ${a.name}`);
    if (ok) { state = applyAction(a, state); i++; }
    else { if (replans >= (opts.maxReplans ?? 10)) return { success: false, executed, replans, log: [...log, 'failure; replan limit'] }; replans++; hooks.onReplan?.(`${a.name} failed`); if (hooks.observe) state = hooks.observe(state); task = { ...task, init: state }; current = plan(task, opts.planOpts); i = 0; if (!current.found) return { success: false, executed, replans, log: [...log, 'replanning failed after action failure'] }; }
  }
}

/** Plain-text plan listing. */
export function planText(r: PlanResult): string { return r.found ? r.plan.map((a, i) => `${i + 1}. ${a.name}${a.cost !== 1 ? ` (cost ${a.cost})` : ''}`).join('\n') + `\n; cost ${r.cost}, ${r.expanded} expanded, ${r.timeMs} ms` : `; no plan: ${r.message}`; }
