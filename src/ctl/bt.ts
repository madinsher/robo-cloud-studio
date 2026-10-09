/**
 * Executive logic — behavior trees and statecharts (course chapter 6).
 *
 *  - Behavior trees with the standard tick semantics (Sequence / Fallback with or without memory, Parallel(M),
 *    decorators Inverter / Retry / Repeat / Timeout / ForceSuccess, leaves Action / Condition), reactive pre-emption
 *    (halting a Running action when a higher-priority branch takes over), skill contracts (pre / post / timeout),
 *    a blackboard, structural checks, Monte-Carlo estimation of finite-time success, and an abstraction of the
 *    tree into a Kripke structure (over non-deterministic leaf outcomes) for LTL / CTL model checking.
 *  - Statecharts (Harel): hierarchy, orthogonal regions, shallow history; flattening to a plain automaton for the
 *    DES / model-checking tools and a runtime dispatcher.
 */
import { AutomatonSpec, Transition } from './des';
import type { Kripke } from './temporal';
import { expr } from './expr';

export type BTStatus = 'success' | 'failure' | 'running';
export type BTNodeType = 'sequence' | 'fallback' | 'parallel' | 'inverter' | 'retry' | 'repeat' | 'timeout' | 'force_success' | 'force_failure' | 'action' | 'condition';

export interface BTNodeSpec {
  type: BTNodeType;
  name?: string;
  /** Sequence / Fallback: remember the running child between ticks (`memory=True` in py_trees). */
  memory?: boolean;
  /** Parallel: number of successes required. */
  threshold?: number;
  /** Retry / Repeat count; Timeout seconds. */
  count?: number;
  seconds?: number;
  /** Leaf binding name and arguments. */
  fn?: string;
  args?: Record<string, unknown>;
  /** Skill contract for actions: pre / post conditions as expressions over the blackboard, timeout in seconds. */
  pre?: string; post?: string; timeout?: number;
  children?: BTNodeSpec[];
}

export interface TickContext { time: number; dt: number; bb: Record<string, unknown>; log?: (msg: string) => void }
export type ActionHandler = (ctx: TickContext, args: Record<string, unknown>, node: BTNode) => BTStatus | void;
export interface BTBindings { actions?: Record<string, ActionHandler>; conditions?: Record<string, (ctx: TickContext, args: Record<string, unknown>) => boolean>; halt?: Record<string, (ctx: TickContext, args: Record<string, unknown>) => void> }

let nodeSeq = 0;

export class BTNode {
  readonly id = `n${nodeSeq++}`;
  readonly children: BTNode[];
  status: BTStatus | 'idle' = 'idle';
  /** memory cursor (sequence / fallback with memory), retry / repeat counters, timeout start */
  cursor = 0; attempts = 0; startedAt: number | null = null;
  ticks = 0;
  constructor(readonly spec: BTNodeSpec, readonly parent: BTNode | null = null) {
    this.children = (spec.children ?? []).map((c) => new BTNode(c, this));
  }
  get name(): string { return this.spec.name ?? (this.spec.fn ? this.spec.fn : this.spec.type); }
  get type(): BTNodeType { return this.spec.type; }
  isLeaf(): boolean { return this.type === 'action' || this.type === 'condition'; }
  *walk(): Generator<BTNode> { yield this; for (const c of this.children) yield* c.walk(); }
  reset(): void { this.status = 'idle'; this.cursor = 0; this.attempts = 0; this.startedAt = null; for (const c of this.children) c.reset(); }
}

export class BehaviorTree {
  readonly root: BTNode;
  readonly bb: Record<string, unknown>;
  readonly bindings: BTBindings;
  time = 0;
  /** Names of actions currently Running (after the last tick). */
  running = new Set<BTNode>();
  readonly trace: Array<{ time: number; node: string; status: BTStatus }> = [];
  private runningPrev = new Set<BTNode>();
  private ticked = new Set<BTNode>();
  constructor(spec: BTNodeSpec, bindings: BTBindings = {}, bb: Record<string, unknown> = {}) { this.root = new BTNode(spec); this.bindings = bindings; this.bb = bb; }

  tick(dt = 0.1): BTStatus {
    const ctx: TickContext = { time: this.time, dt, bb: this.bb };
    this.runningPrev = this.running; this.running = new Set(); this.ticked = new Set();
    const st = this.tickNode(this.root, ctx);
    // pre-emption: actions that were running and were not ticked this cycle are halted
    for (const n of this.runningPrev) if (!this.ticked.has(n)) { this.halt(n, ctx); }
    this.time += dt;
    return st;
  }

  private halt(n: BTNode, ctx: TickContext): void {
    const fn = n.spec.fn ? this.bindings.halt?.[n.spec.fn] : undefined;
    fn?.(ctx, n.spec.args ?? {});
    n.status = 'idle'; n.startedAt = null;
  }

  private done(n: BTNode, st: BTStatus): BTStatus { n.status = st; n.ticks++; if (this.trace.length < 100000) this.trace.push({ time: this.time, node: n.name, status: st }); return st; }

  private tickNode(n: BTNode, ctx: TickContext): BTStatus {
    const s = n.spec;
    switch (s.type) {
      case 'condition': {
        const fn = s.fn ? this.bindings.conditions?.[s.fn] : undefined;
        let ok: boolean;
        if (fn) ok = !!fn(ctx, s.args ?? {});
        else if (s.fn) { try { ok = !!evalBB(s.fn, ctx.bb); } catch { ok = false; } } else ok = false;
        return this.done(n, ok ? 'success' : 'failure');
      }
      case 'action': {
        this.ticked.add(n);
        if (s.pre && n.status !== 'running' && !evalBB(s.pre, ctx.bb)) return this.done(n, 'failure');
        if (n.status !== 'running') n.startedAt = ctx.time;
        const fn = s.fn ? this.bindings.actions?.[s.fn] : undefined;
        let st: BTStatus = fn ? (fn(ctx, s.args ?? {}, n) ?? 'success') : 'success';
        if (st === 'running' && s.timeout !== undefined && n.startedAt !== null && ctx.time - n.startedAt >= s.timeout) { this.halt(n, ctx); st = 'failure'; }
        if (st === 'success' && s.post && !evalBB(s.post, ctx.bb)) st = 'failure';
        if (st === 'running') this.running.add(n); else n.startedAt = null;
        return this.done(n, st);
      }
      case 'sequence': case 'fallback': {
        const stopOn: BTStatus = s.type === 'sequence' ? 'failure' : 'success';
        const start = s.memory ? n.cursor : 0;
        for (let i = start; i < n.children.length; i++) {
          const st = this.tickNode(n.children[i], ctx);
          if (st === 'running') { if (s.memory) n.cursor = i; return this.done(n, 'running'); }
          if (st === stopOn) { n.cursor = 0; return this.done(n, stopOn); }
        }
        n.cursor = 0;
        return this.done(n, s.type === 'sequence' ? 'success' : 'failure');
      }
      case 'parallel': {
        const M = s.threshold ?? n.children.length; let succ = 0, fail = 0;
        for (const c of n.children) { const st = this.tickNode(c, ctx); if (st === 'success') succ++; else if (st === 'failure') fail++; }
        if (succ >= M) return this.done(n, 'success');
        if (fail > n.children.length - M) return this.done(n, 'failure');
        return this.done(n, 'running');
      }
      case 'inverter': { const st = this.tickNode(n.children[0], ctx); return this.done(n, st === 'success' ? 'failure' : st === 'failure' ? 'success' : 'running'); }
      case 'force_success': { const st = this.tickNode(n.children[0], ctx); return this.done(n, st === 'running' ? 'running' : 'success'); }
      case 'force_failure': { const st = this.tickNode(n.children[0], ctx); return this.done(n, st === 'running' ? 'running' : 'failure'); }
      case 'retry': {
        const st = this.tickNode(n.children[0], ctx);
        if (st === 'failure' && n.attempts < (s.count ?? 1)) { n.attempts++; n.children[0].reset(); return this.done(n, 'running'); }
        if (st !== 'running') n.attempts = 0;
        return this.done(n, st);
      }
      case 'repeat': {
        const st = this.tickNode(n.children[0], ctx);
        if (st === 'success' && n.attempts < (s.count ?? 1) - 1) { n.attempts++; n.children[0].reset(); return this.done(n, 'running'); }
        if (st !== 'running') n.attempts = 0;
        return this.done(n, st);
      }
      case 'timeout': {
        if (n.status !== 'running') n.startedAt = ctx.time;
        const st = this.tickNode(n.children[0], ctx);
        if (st === 'running' && ctx.time - (n.startedAt ?? ctx.time) >= (s.seconds ?? 1)) { for (const c of n.children[0].walk()) if (this.running.has(c)) { this.running.delete(c); this.halt(c, ctx); } n.children[0].reset(); return this.done(n, 'failure'); }
        return this.done(n, st);
      }
    }
  }

  reset(): void { this.root.reset(); this.running = new Set(); this.time = 0; this.trace.length = 0; }
}

/** Evaluate a blackboard expression (uses the shared expression language). */
export function evalBB(src: string, bb: Record<string, unknown>): boolean {
  const env: Record<string, string | number | boolean | undefined> = {};
  for (const [k, v] of Object.entries(bb)) env[k] = typeof v === 'number' || typeof v === 'boolean' || typeof v === 'string' ? v : v == null ? undefined : String(v);
  return expr(src).bool(env);
}

// ---------------------------------------------------------------------------------------------
// Skill contracts and builders
// ---------------------------------------------------------------------------------------------

export const seq = (name: string, children: BTNodeSpec[], memory = false): BTNodeSpec => ({ type: 'sequence', name, memory, children });
export const fallback = (name: string, children: BTNodeSpec[], memory = false): BTNodeSpec => ({ type: 'fallback', name, memory, children });
export const parallel = (name: string, children: BTNodeSpec[], threshold?: number): BTNodeSpec => ({ type: 'parallel', name, threshold, children });
export const action = (fn: string, args: Record<string, unknown> = {}, extra: Partial<BTNodeSpec> = {}): BTNodeSpec => ({ type: 'action', fn, args, ...extra });
export const condition = (fn: string, args: Record<string, unknown> = {}, name?: string): BTNodeSpec => ({ type: 'condition', fn, args, name });
export const retry = (count: number, child: BTNodeSpec, name = `Retry×${count}`): BTNodeSpec => ({ type: 'retry', count, name, children: [child] });
export const timeout = (seconds: number, child: BTNodeSpec, name = `Timeout ${seconds}s`): BTNodeSpec => ({ type: 'timeout', seconds, name, children: [child] });
export const inverter = (child: BTNodeSpec): BTNodeSpec => ({ type: 'inverter', children: [child] });

/** Skill = contract (pre, timeout, action, post) as a memory sequence — one description serves the planner and the executor. */
export interface SkillContract { name: string; pre?: string; action: string; args?: Record<string, unknown>; post?: string; timeout?: number; retries?: number }
export function skill(c: SkillContract): BTNodeSpec {
  const body: BTNodeSpec[] = [];
  if (c.pre) body.push(condition(c.pre, {}, `pre: ${c.pre}`));
  const act = action(c.action, c.args ?? {}, { name: c.name, timeout: c.timeout });
  body.push(c.timeout !== undefined ? timeout(c.timeout, act) : act);
  if (c.post) body.push(condition(c.post, {}, `post: ${c.post}`));
  const s = seq(c.name, body, true);
  return c.retries ? retry(c.retries, s, `${c.name} (retry ×${c.retries})`) : s;
}

// ---------------------------------------------------------------------------------------------
// Structural checks
// ---------------------------------------------------------------------------------------------

export interface BTIssue { level: 'error' | 'warning' | 'info'; node: string; message: string }

export function checkBehaviorTree(spec: BTNodeSpec): BTIssue[] {
  const out: BTIssue[] = [];
  const visit = (n: BTNodeSpec, path: string, depth: number, parentType?: BTNodeType) => {
    const name = n.name ?? n.fn ?? n.type; const p = `${path}/${name}`;
    const kids = n.children ?? [];
    if ((n.type === 'sequence' || n.type === 'fallback' || n.type === 'parallel') && kids.length === 0) out.push({ level: 'error', node: p, message: `${n.type} without children` });
    if (['inverter', 'retry', 'repeat', 'timeout', 'force_success', 'force_failure'].includes(n.type) && kids.length !== 1) out.push({ level: 'error', node: p, message: 'decorator must have exactly one child' });
    if ((n.type === 'action' || n.type === 'condition') && kids.length) out.push({ level: 'error', node: p, message: 'leaf with children' });
    if (n.type === 'action' && !n.fn) out.push({ level: 'error', node: p, message: 'action without a binding (fn)' });
    if (n.type === 'action' && n.timeout === undefined && parentType !== 'timeout') out.push({ level: 'warning', node: p, message: 'action without timeout — a liveness obligation that cannot be monitored (chapter 9)' });
    if (n.type === 'parallel' && n.threshold !== undefined && (n.threshold < 1 || n.threshold > kids.length)) out.push({ level: 'error', node: p, message: 'parallel threshold out of range' });
    if (n.type === 'sequence' && n.memory && depth <= 1) out.push({ level: 'warning', node: p, message: 'memory sequence near the root: the tree is not reactive at this level (document why)' });
    if (n.type === 'fallback' && kids.length && depth === 0) {
      const first = kids[0];
      const firstIsGuard = first.type === 'sequence' && first.children?.[0]?.type === 'condition';
      if (!firstIsGuard) out.push({ level: 'warning', node: p, message: 'root fallback: the leftmost child should be the safety branch (condition → reaction), Theorem 6.1' });
    }
    if (n.type === 'sequence') { const idx = kids.findIndex((k) => k.type === 'force_failure'); if (idx >= 0 && idx < kids.length - 1) out.push({ level: 'warning', node: p, message: 'children after ForceFailure in a sequence are unreachable' }); }
    if (n.type === 'fallback') { const idx = kids.findIndex((k) => k.type === 'force_success'); if (idx >= 0 && idx < kids.length - 1) out.push({ level: 'warning', node: p, message: 'children after ForceSuccess in a fallback are unreachable' }); }
    kids.forEach((k) => visit(k, p, depth + 1, n.type));
  };
  visit(spec, '', 0);
  return out;
}

// ---------------------------------------------------------------------------------------------
// Monte-Carlo FTS estimate over a stochastic leaf model
// ---------------------------------------------------------------------------------------------

export interface LeafModel { /** P(success) per completion for actions; P(true) for conditions */ p?: number; /** mean ticks an action runs before completing */ ticks?: number }

/** Simulate `runs` episodes with independent random leaf outcomes; returns success probability and mean ticks to success (finite-time success, Definition 6.4). */
export function estimateFTS(spec: BTNodeSpec, leafModels: Record<string, LeafModel>, runs = 200, maxTicks = 500, seed = 1): { pSuccess: number; meanTicks: number; pFailure: number; pTimeout: number } {
  let s = seed >>> 0; const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
  let succ = 0, fail = 0, tout = 0, ticksSum = 0;
  for (let r = 0; r < runs; r++) {
    const remaining = new Map<BTNode, number>();
    const bt = new BehaviorTree(spec, {
      actions: new Proxy({}, { get: (_t, fn: string) => (_ctx: TickContext, _a: Record<string, unknown>, node: BTNode) => { const m = leafModels[fn] ?? { p: 1, ticks: 1 }; let left = remaining.get(node); if (left === undefined) left = Math.max(1, Math.round((m.ticks ?? 1) * (0.5 + rnd()))); left--; if (left > 0) { remaining.set(node, left); return 'running' as BTStatus; } remaining.delete(node); return rnd() < (m.p ?? 1) ? 'success' as BTStatus : 'failure' as BTStatus; } }),
      conditions: new Proxy({}, { get: (_t, fn: string) => () => rnd() < (leafModels[fn]?.p ?? 1) }),
    });
    let st: BTStatus = 'running'; let t = 0;
    while (t < maxTicks) { st = bt.tick(1); t++; if (st !== 'running') break; }
    if (st === 'success') { succ++; ticksSum += t; } else if (st === 'failure') fail++; else tout++;
  }
  return { pSuccess: succ / runs, meanTicks: succ ? ticksSum / succ : Infinity, pFailure: fail / runs, pTimeout: tout / runs };
}

// ---------------------------------------------------------------------------------------------
// Abstraction to a Kripke structure (non-deterministic leaf outcomes) for model checking
// ---------------------------------------------------------------------------------------------

/**
 * Control state = memory cursors + retry counters; each tick, conditions may be true/false and actions may return
 * success/failure/running (restricted by `outcomes`). Atomic propositions: `tick:<leaf>` (leaf ticked this cycle),
 * `run:<leaf>` (action running), `ok:<leaf>` / `fail:<leaf>` (leaf outcome), `tree:<status>`.
 */
export function btKripke(spec: BTNodeSpec, outcomes: Record<string, BTStatus[]> = {}, maxStates = 5000): Kripke {
  const root = new BTNode(spec);
  const memNodes = [...root.walk()].filter((n) => ((n.type === 'sequence' || n.type === 'fallback') && n.spec.memory) || n.type === 'retry' || n.type === 'repeat');
  const snapshot = () => memNodes.map((n) => (n.type === 'retry' || n.type === 'repeat' ? n.attempts : n.cursor)).join(',');
  const restore = (key: string) => { const vals = key === '' ? [] : key.split(',').map(Number); memNodes.forEach((n, i) => { if (n.type === 'retry' || n.type === 'repeat') n.attempts = vals[i] ?? 0; else n.cursor = vals[i] ?? 0; }); };
  const allowedOf = (n: BTNode): BTStatus[] => outcomes[n.name] ?? (n.type === 'condition' ? ['success', 'failure'] : ['success', 'failure', 'running']);
  type Step = { label: Set<string>; next: string };
  /** One tick with a fixed choice vector; records the allowed alternatives of every choice point. */
  const runTick = (mem: string, choices: BTStatus[], allowedLog: BTStatus[][]): Step => {
    restore(mem);
    const label = new Set<string>(); let ci = 0;
    const tickN = (n: BTNode): BTStatus => {
      const s = n.spec;
      switch (s.type) {
        case 'condition': case 'action': {
          const allowed = allowedOf(n); label.add(`tick:${n.name}`);
          if (ci >= choices.length) choices.push(allowed[0]);
          allowedLog[ci] = allowed;
          const st = choices[ci++];
          label.add(st === 'running' ? `run:${n.name}` : `${st === 'success' ? 'ok' : 'fail'}:${n.name}`);
          return st;
        }
        case 'sequence': case 'fallback': {
          const stopOn: BTStatus = s.type === 'sequence' ? 'failure' : 'success';
          for (let i = s.memory ? n.cursor : 0; i < n.children.length; i++) { const st = tickN(n.children[i]); if (st === 'running') { if (s.memory) n.cursor = i; return 'running'; } if (st === stopOn) { n.cursor = 0; return stopOn; } }
          n.cursor = 0; return s.type === 'sequence' ? 'success' : 'failure';
        }
        case 'parallel': { const M = s.threshold ?? n.children.length; let su = 0, fa = 0; for (const c of n.children) { const st = tickN(c); if (st === 'success') su++; else if (st === 'failure') fa++; } return su >= M ? 'success' : fa > n.children.length - M ? 'failure' : 'running'; }
        case 'inverter': { const st = tickN(n.children[0]); return st === 'success' ? 'failure' : st === 'failure' ? 'success' : 'running'; }
        case 'force_success': { const st = tickN(n.children[0]); return st === 'running' ? 'running' : 'success'; }
        case 'force_failure': { const st = tickN(n.children[0]); return st === 'running' ? 'running' : 'failure'; }
        case 'timeout': return tickN(n.children[0]);
        case 'retry': { const st = tickN(n.children[0]); if (st === 'failure' && n.attempts < (s.count ?? 1)) { n.attempts++; return 'running'; } if (st !== 'running') n.attempts = 0; return st; }
        case 'repeat': { const st = tickN(n.children[0]); if (st === 'success' && n.attempts < (s.count ?? 1) - 1) { n.attempts++; return 'running'; } if (st !== 'running') n.attempts = 0; return st; }
      }
    };
    const treeStatus = tickN(root);
    label.add(`tree:${treeStatus}`);
    return { label, next: snapshot() };
  };
  /** All successor steps of a memory state: DFS over the choice vectors. */
  const explore = (mem: string): Step[] => {
    const out: Step[] = []; let choices: BTStatus[] = [];
    for (let guard = 0; guard < 100000; guard++) {
      const allowedLog: BTStatus[][] = [];
      out.push(runTick(mem, choices, allowedLog));
      // backtrack to the last choice with an untried alternative
      let i = choices.length - 1;
      for (; i >= 0; i--) { const alts = allowedLog[i]; const pos = alts.indexOf(choices[i]); if (pos + 1 < alts.length) { choices = [...choices.slice(0, i), alts[pos + 1]]; break; } }
      if (i < 0) break;
    }
    return out;
  };
  const init = snapshot();
  const states: string[] = ['s0']; const next = new Map<string, string[]>(); const labels = new Map<string, Set<string>>([['s0', new Set(['tree:init'])]]);
  const idOf = new Map<string, string>([[`${init}|`, 's0']]); const stateMem = new Map<string, string>([['s0', init]]);
  const queue = ['s0']; let qi = 0;
  while (qi < queue.length && states.length < maxStates) {
    const sid = queue[qi++]; const mem = stateMem.get(sid)!;
    const succ: string[] = [];
    for (const step of explore(mem)) {
      const k = `${step.next}|${[...step.label].sort().join(' ')}`;
      let tid = idOf.get(k);
      if (!tid) { tid = `s${states.length}`; idOf.set(k, tid); stateMem.set(tid, step.next); states.push(tid); labels.set(tid, step.label); queue.push(tid); }
      if (!succ.includes(tid)) succ.push(tid);
    }
    next.set(sid, succ);
  }
  return { states, initial: ['s0'], next: (s) => next.get(s) ?? [], holds: (s, atom) => labels.get(s)?.has(atom) ?? false, describe: (s) => `${s} [${[...(labels.get(s) ?? [])].join(' ')}]` };
}

// ---------------------------------------------------------------------------------------------
// Statecharts
// ---------------------------------------------------------------------------------------------

export interface SCState { id: string; parent?: string; /** initial substate of a composite parent */ initial?: boolean; /** orthogonal region grouping inside the parent */ region?: string; history?: boolean; entry?: string; exit?: string }
export interface SCTransition { from: string; to: string; event: string; guard?: string; action?: string }
export interface StatechartSpec { name: string; states: SCState[]; transitions: SCTransition[] }

export class Statechart {
  private byId = new Map<string, SCState>();
  private kids = new Map<string, SCState[]>();
  constructor(readonly spec: StatechartSpec) {
    for (const s of spec.states) { this.byId.set(s.id, s); const p = s.parent ?? ''; let k = this.kids.get(p); if (!k) { k = []; this.kids.set(p, k); } k.push(s); }
  }
  children(id: string): SCState[] { return this.kids.get(id) ?? []; }
  isComposite(id: string): boolean { return this.children(id).length > 0; }
  regions(id: string): string[] { return [...new Set(this.children(id).map((c) => c.region ?? ''))]; }
  ancestors(id: string): string[] { const out: string[] = []; let cur = this.byId.get(id)?.parent; while (cur) { out.push(cur); cur = this.byId.get(cur)?.parent; } return out; }
  /** Leaf configuration when entering `id` (default substates in every region). */
  enter(id: string, history: Map<string, string> = new Map()): string[] {
    if (!this.isComposite(id)) return [id];
    const out: string[] = [];
    for (const r of this.regions(id)) {
      const cands = this.children(id).filter((c) => (c.region ?? '') === r);
      const st = this.byId.get(id)!;
      const h = st.history ? history.get(`${id}/${r}`) : undefined;
      const sub = h ? cands.find((c) => c.id === h || this.ancestors(h).includes(c.id)) ?? cands.find((c) => c.initial) ?? cands[0] : cands.find((c) => c.initial) ?? cands[0];
      out.push(...(h && sub && this.ancestors(h).includes(sub.id) ? this.enter(h, history) : this.enter(sub.id, history)));
    }
    return out;
  }
  initialConfiguration(): string[] { const roots = this.children(''); const init = roots.find((r) => r.initial) ?? roots[0]; return init ? this.enter(init.id) : []; }
  /** Transitions enabled in a configuration for an event: from a leaf or any of its ancestors (innermost wins). */
  enabled(config: string[], event: string, guardEnv: Record<string, string | number | boolean> = {}): SCTransition[] {
    const out: SCTransition[] = [];
    for (const leaf of config) {
      const chain = [leaf, ...this.ancestors(leaf)];
      for (const src of chain) {
        const ts = this.spec.transitions.filter((t) => t.from === src && t.event === event && (!t.guard || expr(t.guard).bool(guardEnv)));
        if (ts.length) { out.push(ts[0]); break; }
      }
    }
    return out;
  }
  /** Apply an event: returns the new configuration and the fired transitions. */
  step(config: string[], event: string, history: Map<string, string> = new Map(), guardEnv: Record<string, string | number | boolean> = {}): { config: string[]; fired: SCTransition[] } {
    const fired = this.enabled(config, event, guardEnv);
    if (!fired.length) return { config, fired };
    let next = [...config];
    for (const t of fired) {
      // leaves inside the source scope are exited
      const inScope = (leaf: string) => leaf === t.from || this.ancestors(leaf).includes(t.from);
      for (const leaf of next.filter(inScope)) for (const a of [leaf, ...this.ancestors(leaf)]) { const st = this.byId.get(a); if (st && st.parent) history.set(`${st.parent}/${st.region ?? ''}`, leaf); }
      next = next.filter((l) => !inScope(l));
      next.push(...this.enter(t.to, history));
    }
    return { config: [...new Set(next)].sort(), fired };
  }
  /** Flatten to a plain automaton over event names (guards become part of the event label as `event[guard]`). */
  flatten(maxStates = 20000): AutomatonSpec {
    const key = (c: string[]) => c.join('+');
    const init = this.initialConfiguration();
    const seen = new Map<string, string[]>([[key(init), init]]); const q = [init]; let qi = 0;
    const transitions: Transition[] = [];
    const events = [...new Set(this.spec.transitions.map((t) => t.event))];
    while (qi < q.length && seen.size < maxStates) {
      const c = q[qi++];
      for (const e of events) {
        const cands = this.spec.transitions.filter((t) => t.event === e);
        const guards = [...new Set(cands.map((t) => t.guard ?? ''))];
        for (const g of guards) {
          // evaluate with guards: treat the guard string as satisfied for this branch
          const fired: SCTransition[] = [];
          for (const leaf of c) { const chain = [leaf, ...this.ancestors(leaf)]; for (const src of chain) { const ts = cands.filter((t) => t.from === src && (t.guard ?? '') === g); if (ts.length) { fired.push(ts[0]); break; } } }
          if (!fired.length) continue;
          let next = [...c];
          for (const t of fired) { const inScope = (leaf: string) => leaf === t.from || this.ancestors(leaf).includes(t.from); next = next.filter((l) => !inScope(l)); next.push(...this.enter(t.to)); }
          next = [...new Set(next)].sort();
          const k = key(next);
          transitions.push({ from: key(c), event: g ? `${e}[${g}]` : e, to: k });
          if (!seen.has(k)) { seen.set(k, next); q.push(next); }
        }
      }
    }
    return { name: this.spec.name, events: [...new Set(transitions.map((t) => t.event))], initial: key(init), marked: [...seen.keys()], transitions, states: [...seen.keys()] };
  }
}
