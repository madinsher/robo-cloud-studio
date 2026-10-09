/**
 * Reactive synthesis for the GR(1) fragment (course chapter 8).
 *
 *   (θ_e ∧ G ρ_e ∧ ⋀ GF J_e^i) → (θ_s ∧ G ρ_s ∧ ⋀ GF J_s^j)
 *
 * Explicit-state two-player game over finite-domain variables: the environment moves first (its next values must
 * satisfy ρ_e), then the system replies (ρ_s). Winning region by the three nested fixpoints
 *   W = νZ. ⋀_j μY. ⋁_i νX. ((J_s^j ∧ CPre(Z)) ∨ CPre(Y) ∨ (¬J_e^i ∧ CPre(X)))
 * Strategy extraction (Piterman–Pnueli–Sa'ar) gives a Mealy controller; unrealisability is diagnosed with the
 * environment counter-strategy for the losing initial states.
 *
 * Formulas use the expression language: `loc = 'table1'`, `x' = 1` or `X(x) = 1` for next values, `&&`, `||`, `!`, `->`.
 */
import { expr, Env, Expr } from './expr';

export interface GR1Variable { name: string; owner: 'env' | 'sys'; domain: Array<string | number | boolean> }
export interface GR1Spec {
  name?: string;
  vars: GR1Variable[];
  envInit?: string[]; envTrans?: string[]; envLive?: string[];
  sysInit?: string[]; sysTrans?: string[]; sysLive?: string[];
}

export interface GR1Controller {
  /** Controller states: (game state index, goal memory j). */
  states: Array<{ id: number; values: Env; goal: number }>;
  initial: number[];
  /** transitions[id][envMoveKey] = next controller state id */
  transitions: Map<number, Map<string, number>>;
  size: number;
}

export interface GR1Result {
  realizable: boolean;
  gameStates: number;
  winning: number;
  /** Initial states (satisfying θ_e ∧ θ_s) that are losing. */
  losingInitial: Env[];
  controller: GR1Controller | null;
  /** Environment counter-strategy sketch: a few steps the environment can force from a losing initial state. */
  counterStrategy: Array<{ state: Env; envMove: Env; note: string }>;
  diagnosis: string[];
}

const normX = (s: string) => s.replace(/X\s*\(\s*([A-Za-z_][\w.]*)\s*\)/g, "$1'");

export function synthesizeGR1(spec: GR1Spec, opts: { maxStates?: number } = {}): GR1Result {
  const vars = spec.vars; const envVars = vars.filter((v) => v.owner === 'env'), sysVars = vars.filter((v) => v.owner === 'sys');
  const comp = (list: string[] | undefined) => (list ?? []).map((s) => expr(normX(s)));
  const eInit = comp(spec.envInit), eTrans = comp(spec.envTrans), eLive = comp(spec.envLive), sInit = comp(spec.sysInit), sTrans = comp(spec.sysTrans), sLive = comp(spec.sysLive);
  const all = (f: Expr[], env: Env, next?: Env) => f.every((e) => e.bool(env, next));
  // enumerate assignments
  const envAssign = enumerate(envVars), sysAssign = enumerate(sysVars);
  const states: Env[] = [];
  for (const e of envAssign) for (const s of sysAssign) states.push({ ...e, ...s });
  const n = states.length;
  if (n > (opts.maxStates ?? 200000)) throw new Error(`game too large: ${n} states`);
  const idx = new Map(states.map((s, i) => [stateKey(s, vars), i]));
  // moves: for state i, env choices e' (satisfying ρ_e) → list of sys replies s' (satisfying ρ_s) as state indices
  const moves: Array<Array<{ env: Env; sys: number[] }>> = states.map((st) => {
    const out: Array<{ env: Env; sys: number[] }> = [];
    for (const e of envAssign) {
      // ρ_e is evaluated with next env values only (system next values unknown yet) — use current sys values as placeholder
      if (!all(eTrans, st, { ...st, ...e })) continue;
      const sys: number[] = [];
      for (const s of sysAssign) { const nx = { ...e, ...s }; if (all(sTrans, st, nx)) sys.push(idx.get(stateKey(nx, vars))!); }
      out.push({ env: e, sys });
    }
    return out;
  });
  // CPre(S): for every env move there is a sys reply into S (env moves that violate ρ_e are excluded already)
  const cpre = (S: Uint8Array): Uint8Array => {
    const out = new Uint8Array(n);
    for (let i = 0; i < n; i++) out[i] = moves[i].every((mv) => mv.sys.some((j) => S[j])) ? 1 : 0;
    return out;
  };
  const sat = (f: Expr) => { const out = new Uint8Array(n); for (let i = 0; i < n; i++) out[i] = f.bool(states[i]) ? 1 : 0; return out; };
  const Js = sLive.length ? sLive.map(sat) : [new Uint8Array(n).fill(1)];
  const Je = eLive.length ? eLive.map(sat) : [new Uint8Array(n).fill(1)];
  const AND = (a: Uint8Array, b: Uint8Array) => a.map((v, i) => v & b[i]);
  const OR = (a: Uint8Array, b: Uint8Array) => a.map((v, i) => v | b[i]);
  const NOT = (a: Uint8Array) => a.map((v) => (v ? 0 : 1));
  const eq = (a: Uint8Array, b: Uint8Array) => a.every((v, i) => v === b[i]);
  // ν Z
  let Z = new Uint8Array(n).fill(1);
  // memory for strategy: for each goal j, the Y-layers (ranks)
  let layers: Uint8Array[][] = [];
  for (let guard = 0; guard < 10000; guard++) {
    const Znew = new Uint8Array(n).fill(1);
    const layersJ: Uint8Array[][] = [];
    for (let j = 0; j < Js.length; j++) {
      const cpreZ = cpre(Z);
      const jn = (j + 1) % Js.length; void jn;
      const target = AND(Js[j], cpreZ);
      let Y = new Uint8Array(n); const Ylayers: Uint8Array[] = [];
      for (let g2 = 0; g2 < 10000; g2++) {
        const cpreY = cpre(Y);
        let Ynew = new Uint8Array(n);
        for (let i = 0; i < Je.length; i++) {
          // νX. target ∨ cpreY ∨ (¬Je_i ∧ CPre(X))
          let Xs = new Uint8Array(n).fill(1);
          for (let g3 = 0; g3 < 10000; g3++) { const Xn = OR(OR(target, cpreY), AND(NOT(Je[i]), cpre(Xs))); if (eq(Xn, Xs)) break; Xs = Xn; }
          Ynew = OR(Ynew, Xs);
        }
        if (eq(Ynew, Y)) break;
        Ylayers.push(Ynew); Y = Ynew;
      }
      layersJ.push(Ylayers);
      for (let i = 0; i < n; i++) Znew[i] &= Y[i];
    }
    layers = layersJ;
    if (eq(Znew, Z)) break;
    Z = Znew;
  }
  const W = Z;
  const initial = states.map((s, i) => i).filter((i) => all(eInit, states[i]) && all(sInit, states[i]));
  const losing = initial.filter((i) => !W[i]);
  const diagnosis: string[] = [];
  if (!initial.length) diagnosis.push('no state satisfies the initial conditions θ_e ∧ θ_s');
  if (losing.length) diagnosis.push(`${losing.length} of ${initial.length} initial states are losing: the environment can force a violation of a guarantee (or block a goal forever) while respecting its assumptions`);
  // quick hints: which liveness guarantee is unreachable from the initial states under the safety constraints
  if (losing.length) {
    for (let j = 0; j < sLive.length; j++) if (!states.some((s, i) => Js[j][i] && reachable(moves, initial, i))) diagnosis.push(`goal ${j + 1} "${spec.sysLive![j]}" is unreachable under the transition constraints`);
    if (eLive.length === 0) diagnosis.push('no environment liveness assumptions: the environment may hold any input forever (e.g. keep a human present) — consider adding GF assumptions');
  }
  const counter: GR1Result['counterStrategy'] = [];
  if (losing.length) {
    let cur = losing[0];
    for (let step = 0; step < 6; step++) {
      const mv = moves[cur].find((m) => m.sys.every((j) => !W[j]) || m.sys.length === 0);
      if (!mv) break;
      counter.push({ state: states[cur], envMove: mv.env, note: mv.sys.length ? 'every system reply stays losing' : 'no system reply satisfies its transition constraints' });
      if (!mv.sys.length) break;
      cur = mv.sys[0];
    }
  }
  let controller: GR1Controller | null = null;
  if (initial.length && !losing.length) controller = extractStrategy(states, vars, moves, W, Js, Je, layers, initial, cpre, sysVars.length ? sysVars : vars);
  return { realizable: initial.length > 0 && losing.length === 0, gameStates: n, winning: W.reduce((s, v) => s + v, 0), losingInitial: losing.map((i) => states[i]), controller, counterStrategy: counter, diagnosis };
}

function reachable(moves: Array<Array<{ env: Env; sys: number[] }>>, from: number[], target: number): boolean {
  const seen = new Set(from); const q = [...from];
  while (q.length) { const i = q.pop()!; if (i === target) return true; for (const m of moves[i]) for (const j of m.sys) if (!seen.has(j)) { seen.add(j); q.push(j); } }
  return false;
}

function extractStrategy(states: Env[], vars: GR1Variable[], moves: Array<Array<{ env: Env; sys: number[] }>>, W: Uint8Array, Js: Uint8Array[], Je: Uint8Array[], layers: Uint8Array[][], initial: number[], cpre: (S: Uint8Array) => Uint8Array, _sysVars: GR1Variable[]): GR1Controller {
  const rank = (j: number, i: number): number => { const L = layers[j]; for (let r = 0; r < L.length; r++) if (L[r][i]) return r; return Infinity; };
  const cpreW = cpre(W);
  const ctrl: GR1Controller = { states: [], initial: [], transitions: new Map(), size: 0 };
  const ids = new Map<string, number>();
  const idOf = (i: number, j: number) => { const k = `${i}|${j}`; let id = ids.get(k); if (id === undefined) { id = ctrl.states.length; ids.set(k, id); ctrl.states.push({ id, values: states[i], goal: j }); ctrl.transitions.set(id, new Map()); } return id; };
  const q: Array<[number, number]> = [];
  for (const i of initial) { q.push([i, 0]); ctrl.initial.push(idOf(i, 0)); }
  const envKey = (e: Env) => vars.filter((v) => v.owner === 'env').map((v) => `${v.name}=${e[v.name]}`).join(',');
  const seen = new Set<string>();
  while (q.length) {
    const [i, j] = q.shift()!; const k = `${i}|${j}`; if (seen.has(k)) continue; seen.add(k);
    const id = idOf(i, j);
    const goalMet = Js[j][i] === 1;
    const jNext = goalMet ? (j + 1) % Js.length : j;
    for (const mv of moves[i]) {
      const cands = mv.sys.filter((t) => W[t]);
      if (!cands.length) continue; // env move outside the assumptions or losing (cannot happen in W)
      let best: number;
      if (goalMet) best = cands.reduce((a, b) => (rank(jNext, b) <= rank(jNext, a) ? b : a));
      else {
        const r = rank(j, i);
        // prefer a reply with strictly smaller rank; otherwise stay in the νX region of a violated assumption
        const better = cands.filter((t) => rank(j, t) < r);
        best = better.length ? better.reduce((a, b) => (rank(j, b) <= rank(j, a) ? b : a)) : cands.find((t) => Je.some((je, ii) => !je[t] && rank(j, t) <= r)) ?? cands.reduce((a, b) => (rank(j, b) <= rank(j, a) ? b : a));
      }
      const nid = idOf(best, jNext);
      ctrl.transitions.get(id)!.set(envKey(mv.env), nid);
      q.push([best, jNext]);
    }
  }
  void cpreW;
  ctrl.size = ctrl.states.length;
  return ctrl;
}

function enumerate(vs: GR1Variable[]): Env[] {
  let out: Env[] = [{}];
  for (const v of vs) { const next: Env[] = []; for (const a of out) for (const d of v.domain) next.push({ ...a, [v.name]: d }); out = next; }
  return out;
}
function stateKey(s: Env, vars: GR1Variable[]): string { return vars.map((v) => String(s[v.name])).join('|'); }

/** Run the controller: given the current controller state and the environment's next input, return the next state (or null if the input violates the assumptions). */
export function stepController(c: GR1Controller, id: number, envInput: Env, vars: GR1Variable[]): number | null {
  const k = vars.filter((v) => v.owner === 'env').map((v) => `${v.name}=${envInput[v.name]}`).join(',');
  return c.transitions.get(id)?.get(k) ?? null;
}
