/**
 * Synchronous finite-state models in the NuSMV style (course §7.9): variables with finite domains, `init` and
 * `next` given as ordered case lists (a case may yield a set of values → non-determinism), DEFINE macros, fairness
 * constraints and LTL / CTL specifications. The model is unfolded into a Kripke structure for the checkers.
 *
 *   var r1.state : {idle, waiting, in_zone, leaving};
 *   define zone_free := owner = none;
 *   init r1.state := idle;
 *   next r1.state := case r1.state = idle : {idle, waiting}; r1.state = waiting & g1 : in_zone; ... esac;
 */
import { Expr, expr, Env, Value } from './expr';
import { Kripke, atomHolds } from './temporal';

export interface SmvCase { cond: string; value: string }
export interface SmvVar { name: string; domain: Array<string | number | boolean> }
export interface SmvSpec {
  name: string;
  vars: SmvVar[];
  defines?: Record<string, string>;
  /** initial value(s) per variable: constant, `{a, b}` set, or expression; missing = any domain value */
  init?: Record<string, string>;
  /** next-state cases per variable (first matching case wins); missing = keeps its value */
  next?: Record<string, SmvCase[]>;
  fairness?: string[];
  specs?: Array<{ kind: 'LTL' | 'CTL'; formula: string; name?: string }>;
}

function parseValueSet(src: string, env: Env, vars: SmvVar[], v: SmvVar): Value[] {
  const s = src.trim();
  if (s.startsWith('{')) return s.slice(1, -1).split(',').map((x) => coerce(x.trim(), v));
  if (v.domain.some((d) => String(d) === s)) return [coerce(s, v)];
  if (vars.some((x) => x.name === s) || /[()+\-*/<>=!&|?]/.test(s)) { const val = expr(s).eval(env); return [coerce(String(val), v)]; }
  return [coerce(s, v)];
}
function coerce(s: string, v: SmvVar): Value { const d = v.domain.find((x) => String(x) === s); if (d !== undefined) return d; if (s === 'true' || s === 'TRUE') return true; if (s === 'false' || s === 'FALSE') return false; if (/^-?\d+(\.\d+)?$/.test(s)) return Number(s); return s; }

export class SmvModel {
  private compiled: Record<string, Array<{ cond: Expr; value: string }>> = {};
  private defines: Array<[string, Expr]> = [];
  constructor(readonly spec: SmvSpec) {
    for (const [v, cases] of Object.entries(spec.next ?? {})) this.compiled[v] = cases.map((c) => ({ cond: expr(c.cond), value: c.value }));
    this.defines = Object.entries(spec.defines ?? {}).map(([k, e]) => [k, expr(e)]);
  }
  /** Environment of a state: variables, DEFINE macros, and every domain literal bound to itself (so `owner = none` works). */
  env(state: Env): Env {
    const e: Env = {};
    for (const v of this.spec.vars) for (const d of v.domain) if (typeof d === 'string') e[d] = d;
    Object.assign(e, state);
    for (const [k, ex] of this.defines) e[k] = ex.eval(e);
    return e;
  }
  key(state: Env): string { return this.spec.vars.map((v) => String(state[v.name])).join('|'); }
  initialStates(): Env[] {
    let out: Env[] = [{}];
    for (const v of this.spec.vars) {
      const next: Env[] = [];
      for (const a of out) { const vals = this.spec.init?.[v.name] ? parseValueSet(this.spec.init[v.name], this.env(a), this.spec.vars, v) : v.domain; for (const d of vals) next.push({ ...a, [v.name]: d }); }
      out = next;
    }
    return out;
  }
  successors(state: Env): Env[] {
    const e = this.env(state);
    let out: Env[] = [{}];
    for (const v of this.spec.vars) {
      const cases = this.compiled[v.name];
      let vals: Value[] = [state[v.name] as Value];
      if (cases) { const c = cases.find((x) => x.cond.bool(e)); if (c) vals = parseValueSet(c.value, e, this.spec.vars, v); }
      const next: Env[] = []; for (const a of out) for (const d of vals) next.push({ ...a, [v.name]: d }); out = next;
    }
    return out;
  }
  kripke(maxStates = 200000): Kripke {
    const states: Env[] = []; const ids = new Map<string, string>(); const succ = new Map<string, string[]>();
    const idOf = (s: Env): string => { const k = this.key(s); let id = ids.get(k); if (id === undefined) { id = String(states.length); ids.set(k, id); states.push(s); } return id; };
    const init = this.initialStates().map(idOf);
    for (let i = 0; i < states.length && states.length < maxStates; i++) succ.set(String(i), this.successors(states[i]).map(idOf));
    const envs = new Map<string, Env>();
    const envOf = (id: string) => { let e = envs.get(id); if (!e) { e = this.env(states[Number(id)]); envs.set(id, e); } return e; };
    return { states: states.map((_, i) => String(i)), initial: init, next: (s) => succ.get(s) ?? [], holds: (s, atom) => atomHolds(atom, envOf(s) as Record<string, string | number | boolean | undefined>), describe: (s) => this.spec.vars.map((v) => `${v.name}=${states[Number(s)][v.name]}`).join(' ') };
  }
}

/** Course §7.9 cell: two robots and a shared zone with static priority for r1 (r2 starves) or with alternation (`fair`). */
export function cellZoneModel(fair = false): SmvSpec {
  const robot = (id: string, grant: string): Record<string, SmvCase[]> => ({ [`${id}.state`]: [
    { cond: `${id}.state = idle`, value: '{idle, waiting}' },
    { cond: `${id}.state = waiting & ${grant}`, value: 'in_zone' },
    { cond: `${id}.state = waiting & !${grant}`, value: 'waiting' },
    { cond: `${id}.state = in_zone`, value: '{in_zone, leaving}' },
    { cond: `${id}.state = leaving`, value: '{idle, waiting}' },
  ] });
  const spec: SmvSpec = {
    name: fair ? 'Cell zone (alternation)' : 'Cell zone (r1 priority)',
    vars: [{ name: 'r1.state', domain: ['idle', 'waiting', 'in_zone', 'leaving'] }, { name: 'r2.state', domain: ['idle', 'waiting', 'in_zone', 'leaving'] }, { name: 'owner', domain: ['none', 'one', 'two'] }, ...(fair ? [{ name: 'turn', domain: ['one', 'two'] }] : [])],
    defines: fair
      ? { zone_free: 'owner = none', g1: '(owner = none) & (turn = one | r2.state != waiting)', g2: '(owner = none) & (turn = two | r1.state != waiting)' }
      : { zone_free: 'owner = none', g1: '(owner = none) | (owner = one)', g2: '(owner = none) & !(r1.state = waiting)' },
    init: { 'r1.state': 'idle', 'r2.state': 'idle', owner: 'none', ...(fair ? { turn: 'one' } : {}) },
    next: {
      ...robot('r1', 'g1'), ...robot('r2', 'g2'),
      owner: [{ cond: 'r1.state = waiting & g1', value: 'one' }, { cond: 'r2.state = waiting & g2', value: 'two' }, { cond: 'r1.state = leaving & owner = one', value: 'none' }, { cond: 'r2.state = leaving & owner = two', value: 'none' }, { cond: 'TRUE', value: 'owner' }],
      ...(fair ? { turn: [{ cond: 'r1.state = leaving', value: 'two' }, { cond: 'r2.state = leaving', value: 'one' }, { cond: 'TRUE', value: 'turn' }] } : {}),
    },
    fairness: ['!(r1.state = in_zone)', '!(r2.state = in_zone)'],
    specs: [
      { kind: 'LTL', name: 'mutual exclusion', formula: 'G !(r1.state = in_zone & r2.state = in_zone)' },
      { kind: 'LTL', name: 'no starvation r1', formula: 'G (r1.state = waiting -> F r1.state = in_zone)' },
      { kind: 'LTL', name: 'no starvation r2', formula: 'G (r2.state = waiting -> F r2.state = in_zone)' },
      { kind: 'CTL', name: 'recoverability', formula: 'AG EF (owner = none)' },
      { kind: 'CTL', name: 'premise reachable: r2 waits', formula: 'EF (r2.state = waiting)' },
      { kind: 'CTL', name: 'premise reachable: r2 in zone', formula: 'EF (r2.state = in_zone)' },
    ],
  };
  return spec;
}
