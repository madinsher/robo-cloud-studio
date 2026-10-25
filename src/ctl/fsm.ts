/**
 * Executable hierarchical finite-state machines with named outcomes (yasmin / SMACH style), the kind `fsm`.
 *
 *   fsm Depalletize mission
 *   blackboard remaining=3 errors=0
 *   outcomes done aborted                 # outcomes of the top machine
 *   state Scan initial
 *     entry set=phase value=scan          # instantaneous entry / exit actions (set, signal, event, log)
 *     do program="Scan shelf" timeout=30  # the skill: any action binding (program= target= goto= signal= wait= await= skill= set= event=)
 *     ok -> Depal                         # outcome → sibling state, or → an outcome of the enclosing machine
 *     fail -> aborted
 *   machine Depal                         # a nested machine is a state; its outcomes are the state's outcomes
 *     outcomes done failed
 *     state Plan decision                 # decision state: first `when` whose expression holds gives the outcome
 *       when "remaining > 0" -> found
 *       otherwise -> none_left
 *       found -> Pick
 *       none_left -> done
 *     state Pick
 *       do program=PickBox done=picked fail=missed   # done= / fail= rename the skill's outcomes (default ok / fail)
 *       exit set=remaining value="=remaining - 1"     # `=expr` evaluates over the blackboard
 *       picked -> Plan
 *       missed -> failed
 *     done -> Finish
 *     failed -> aborted
 *   state Finish
 *     ok -> done
 *
 * Skill outcomes: `ok` (or done=), `fail` (or fail=), `timeout` (state `timeout S` or binding timeout=; falls back to the
 * fail mapping when unmapped), `canceled` (cancel(); propagates up to the first machine that maps it). A state with no
 * skill and no decision yields `ok`. An outcome without a transition that names an outcome of the enclosing machine
 * ends that machine with it (as in yasmin). The blackboard holds the declared variables, the station signals and the world's
 * sense values; `fsm_state` publishes the active path (telemetry, like /murom/fsm/state).
 */
import type { TickContext, BTStatus } from './bt';
import { ActionBinding, parseAction } from './dsl';
import { tokenize, lines, DslError } from './dsl_core';
import { ExecutorWorld, Runner, Running, Verdict, makeEval, Sensors, ExecSummary, worldSense } from './exec';

export type FsmStateKind = 'skill' | 'decision' | 'machine' | 'pass';
export interface FsmState { name: string; kind: FsmStateKind; initial?: boolean; action?: ActionBinding; entry: ActionBinding[]; exit: ActionBinding[]; timeout?: number; when: Array<{ expr: string; outcome: string }>; otherwise?: string; transitions: Record<string, string>; machine?: FsmMachine }
export interface FsmMachine { name: string; outcomes: string[]; states: FsmState[] }
export interface FsmDoc { name: string; root: FsmMachine; blackboard: Record<string, string | number | boolean>; layout: Record<string, Record<string, [number, number]>>; sensors: Array<{ event: string; expr: string }> }

const TR_RE = /^(?:on\s+)?([A-Za-z_][\w.-]*)\s*-+>\s*([A-Za-z_][\w.-]*)$/;
const WHEN_RE = /^when\s+(?:"([^"]*)"|'([^']*)'|(.+?))\s*-+>\s*([A-Za-z_][\w.-]*)$/;

/** Outcomes a state can produce. */
export function stateOutcomes(s: FsmState): string[] {
  switch (s.kind) {
    case 'skill': return [s.action?.done ?? 'ok', s.action?.fail ?? 'fail', ...(s.timeout !== undefined || s.action?.timeout !== undefined ? ['timeout'] : [])];
    case 'decision': return [...new Set([...s.when.map((w) => w.outcome), ...(s.otherwise ? [s.otherwise] : [])])];
    case 'machine': return s.machine?.outcomes ?? [];
    case 'pass': return ['ok'];
  }
}
export const initialState = (m: FsmMachine): FsmState | undefined => m.states.find((s) => s.initial) ?? m.states[0];

export function parseFsm(src: string): FsmDoc {
  const doc: FsmDoc = { name: 'FSM', root: { name: 'FSM', outcomes: [], states: [] }, blackboard: {}, layout: {}, sensors: [] };
  // stack of open blocks: machines and states with their indent
  type Block = { indent: number; machine?: FsmMachine; state?: FsmState };
  const stack: Block[] = [{ indent: -1, machine: doc.root }];
  const machineOf = () => { for (let i = stack.length - 1; i >= 0; i--) if (stack[i].machine) return stack[i].machine!; return doc.root; };
  for (const l of lines(src)) {
    const t = l.text.trim(); const { words, opts, quoted } = tokenize(t); const kw = words[0]?.toLowerCase();
    if (kw === 'fsm') { doc.name = words.slice(1).join(' ') || doc.name; doc.root.name = doc.name; continue; }
    if (kw === 'blackboard' || kw === 'bb') { Object.assign(doc.blackboard, opts); continue; }
    if (kw === 'layout') { const m = words[1]; if (!m) throw new DslError('layout <machine path> state=x,y …', l.n); const L: Record<string, [number, number]> = (doc.layout[m] ??= {}); for (const [k, v] of Object.entries(opts)) { const mm = /^(-?[\d.]+),(-?[\d.]+)$/.exec(String(v)); if (mm) L[k] = [Number(mm[1]), Number(mm[2])]; } continue; }
    if (kw === 'sensor') { const ev = words[1]; if (!ev) throw new DslError('sensor <event> "<expression>"', l.n); doc.sensors.push({ event: ev, expr: quoted[0] ?? t.slice(t.indexOf(ev) + ev.length).trim() }); continue; }
    while (stack.length > 1 && stack[stack.length - 1].indent >= l.indent) stack.pop();
    const top = stack[stack.length - 1];
    if (kw === 'state' || kw === 'machine') {
      const name = words[1]; if (!name) throw new DslError(`${kw} <name>`, l.n);
      const parent = machineOf(); if (parent.states.some((s) => s.name === name)) throw new DslError(`duplicate state ${name} in ${parent.name}`, l.n);
      const st: FsmState = { name, kind: kw === 'machine' ? 'machine' : words.includes('decision') ? 'decision' : 'pass', initial: words.includes('initial') || opts.initial === true, entry: [], exit: [], when: [], transitions: {} };
      if (opts.timeout !== undefined) st.timeout = Number(opts.timeout);
      if (kw === 'machine') st.machine = { name, outcomes: [], states: [] };
      parent.states.push(st); stack.push({ indent: l.indent, state: st, machine: st.machine });
      continue;
    }
    const st = top.state; const mach = top.machine;
    if (kw === 'outcomes') { if (!mach) throw new DslError('outcomes belong to a machine (fsm / machine block)', l.n); mach.outcomes.push(...words.slice(1)); continue; }
    if (!st) {
      if (top.machine === doc.root) { const tm = TR_RE.exec(t); if (tm) throw new DslError(`transition '${t}' outside a state`, l.n); }
      throw new DslError(`cannot parse '${t}' (expected state / machine / outcomes)`, l.n);
    }
    if (kw === 'do') { st.action = parseAction(words, opts, l.n, 0); st.action.target = st.name; if (st.kind === 'pass') st.kind = 'skill'; continue; }
    if (kw === 'entry' || kw === 'exit') { const b = parseAction(words, opts, l.n, 0); b.target = `${st.name}.${kw}`; (kw === 'entry' ? st.entry : st.exit).push(b); continue; }
    if (kw === 'timeout' && !t.includes('->')) { st.timeout = Number(words[1]); if (!Number.isFinite(st.timeout)) throw new DslError('timeout <seconds>', l.n); continue; }
    const wm = WHEN_RE.exec(t); if (wm) { st.when.push({ expr: (wm[1] ?? wm[2] ?? wm[3]).trim(), outcome: wm[4] }); if (st.kind === 'pass') st.kind = 'decision'; continue; }
    const om = /^otherwise\s*-+>\s*([A-Za-z_][\w.-]*)$/.exec(t); if (om) { st.otherwise = om[1]; if (st.kind === 'pass') st.kind = 'decision'; continue; }
    const tm = TR_RE.exec(t); if (tm) { st.transitions[tm[1]] = tm[2]; continue; }
    throw new DslError(`cannot parse '${t}' in state ${st.name}`, l.n);
  }
  if (!doc.root.states.length) throw new DslError('an fsm needs at least one state');
  return doc;
}

const fmtNum = (v: number) => Number(v.toFixed(1)).toString();
const qv = (v: string | number | boolean) => (typeof v === 'string' && (/\s/.test(v) || v === '') ? `"${v}"` : String(v));
const bind = (b: ActionBinding) => `${b.kind}=${qv(b.value)}${b.robot ? ` robot=${qv(b.robot)}` : ''}${b.component ? ` component=${qv(b.component)}` : ''}${b.done ? ` done=${b.done}` : ''}${b.fail ? ` fail=${b.fail}` : ''}${b.timeout !== undefined ? ` timeout=${b.timeout}` : ''}${Object.entries(b.args).map(([k, v]) => ` ${k}=${qv(v)}`).join('')}`;
/** Write the document back to the DSL (diagram editor round trip). */
export function fsmToDsl(doc: FsmDoc): string {
  const out = [`fsm ${doc.name}`];
  if (Object.keys(doc.blackboard).length) out.push(`blackboard ${Object.entries(doc.blackboard).map(([k, v]) => `${k}=${qv(v)}`).join(' ')}`);
  if (doc.root.outcomes.length) out.push(`outcomes ${doc.root.outcomes.join(' ')}`);
  const states = (m: FsmMachine, ind: string) => {
    for (const s of m.states) {
      out.push(`${ind}${s.kind === 'machine' ? 'machine' : 'state'} ${s.name}${s.initial ? ' initial' : ''}${s.kind === 'decision' && !s.when.length ? ' decision' : ''}`);
      const i2 = ind + '  ';
      if (s.machine?.outcomes.length) out.push(`${i2}outcomes ${s.machine.outcomes.join(' ')}`);
      for (const b of s.entry) out.push(`${i2}entry ${bind(b)}`);
      if (s.action) out.push(`${i2}do ${bind(s.action)}`);
      for (const b of s.exit) out.push(`${i2}exit ${bind(b)}`);
      if (s.timeout !== undefined) out.push(`${i2}timeout ${s.timeout}`);
      for (const w of s.when) out.push(`${i2}when "${w.expr.replace(/"/g, "'")}" -> ${w.outcome}`);
      if (s.otherwise) out.push(`${i2}otherwise -> ${s.otherwise}`);
      if (s.machine) states(s.machine, i2);
      for (const [o, to] of Object.entries(s.transitions)) out.push(`${i2}${o} -> ${to}`);
    }
  };
  states(doc.root, '');
  for (const s of doc.sensors) out.push(`sensor ${s.event} "${s.expr.replace(/"/g, "'")}"`);
  for (const [m, L] of Object.entries(doc.layout)) { const e = Object.entries(L); if (e.length) out.push(`layout ${m} ${e.map(([s, p]) => `${s}=${fmtNum(p[0])},${fmtNum(p[1])}`).join(' ')}`); }
  return out.join('\n');
}

/** Machines by path (`Root/Depal`). */
export function fsmMachines(doc: FsmDoc): Array<{ path: string; machine: FsmMachine; parent: FsmMachine | null }> {
  const out: Array<{ path: string; machine: FsmMachine; parent: FsmMachine | null }> = [];
  const walk = (m: FsmMachine, path: string, parent: FsmMachine | null) => { out.push({ path, machine: m, parent }); for (const s of m.states) if (s.machine) walk(s.machine, `${path}/${s.name}`, m); };
  walk(doc.root, doc.root.name.replace(/\s+/g, '_') || 'FSM', null);
  return out;
}

export interface FsmIssue { level: 'error' | 'warning'; where: string; message: string }
/** Structural checks: every outcome mapped, targets exist, initial states, reachability, machine outcomes reachable. */
export function checkFsm(doc: FsmDoc): FsmIssue[] {
  const out: FsmIssue[] = [];
  for (const { path, machine: m } of fsmMachines(doc)) {
    if (!m.states.length) { out.push({ level: 'error', where: path, message: 'machine without states' }); continue; }
    if (m.states.filter((s) => s.initial).length > 1) out.push({ level: 'error', where: path, message: 'more than one initial state' });
    const names = new Set(m.states.map((s) => s.name)); const outs = new Set(m.outcomes);
    if (m !== doc.root && !m.outcomes.length) out.push({ level: 'error', where: path, message: 'nested machine without outcomes' });
    for (const s of m.states) {
      const where = `${path}/${s.name}`;
      for (const o of stateOutcomes(s)) if (!(o in s.transitions) && !outs.has(o) && !(o === 'timeout' && (s.action?.fail ?? 'fail') in s.transitions)) out.push({ level: 'error', where, message: `outcome '${o}' is not mapped` });
      for (const [o, to] of Object.entries(s.transitions)) {
        if (!names.has(to) && !outs.has(to)) out.push({ level: 'error', where, message: `${o} -> ${to}: no state or outcome ${to} in ${m.name}` });
        if (!stateOutcomes(s).includes(o) && o !== 'canceled') out.push({ level: 'warning', where, message: `transition on '${o}', which the state never returns (outcomes: ${stateOutcomes(s).join(', ')})` });
      }
      if (s.kind === 'decision' && !s.otherwise) out.push({ level: 'warning', where, message: 'decision without otherwise: the state waits until one condition holds' });
      if (s.kind === 'skill' && s.action && ['program', 'target', 'goto'].includes(s.action.kind) && s.timeout === undefined && s.action.timeout === undefined) out.push({ level: 'warning', where, message: 'skill without a timeout' });
      for (const b of [...s.entry, ...s.exit]) if (!['set', 'signal', 'event', 'log'].includes(b.kind) && !(b.kind === 'skill' && b.value === 'log')) out.push({ level: 'warning', where, message: `${b.target}: ${b.kind} is not instantaneous — entry / exit actions should be set / signal / event` });
    }
    // reachability inside the machine
    const init = initialState(m); const seen = new Set<string>(); const q = init ? [init.name] : [];
    const reached = new Set<string>();
    while (q.length) { const n = q.pop()!; if (seen.has(n)) continue; seen.add(n); const s = m.states.find((x) => x.name === n); if (!s) { reached.add(n); continue; } for (const to of Object.values(s.transitions)) q.push(to); }
    for (const s of m.states) if (!seen.has(s.name)) out.push({ level: 'warning', where: `${path}/${s.name}`, message: 'unreachable state' });
    for (const o of m.outcomes) if (!seen.has(o)) out.push({ level: 'warning', where: path, message: `outcome '${o}' is never reached` });
  }
  return out;
}

// ---------------------------------------------------------------------------------------------
// Executor
// ---------------------------------------------------------------------------------------------

interface Frame { machine: FsmMachine; path: string; state: FsmState; entered: number; run: Running | null }
export interface FsmExecOptions { doc: FsmDoc; world: ExecutorWorld; log?: (m: string) => void; /** max state changes per tick (instantaneous chains) */ maxPerTick?: number }

/**
 * Runs an FSM document: one active state per machine level (the innermost is a skill / decision / pass state), outcomes
 * resolved through the transition maps, nested machines finish with their outcomes, state timeouts and cancel. The
 * trace records every outcome with the time and the state path.
 */
export class FsmExecutor {
  readonly frames: Frame[] = [];
  readonly log: string[] = []; readonly trace: Array<{ t: number; path: string; outcome: string; to: string }> = [];
  private runner: Runner; private evalFn: (src: string, bb: Record<string, unknown>) => boolean; private sensors: Sensors; private cancelReq = false;
  time = 0; ticks = 0; transitions = 0; failed = 0; status: ExecSummary['status'] = 'running'; verdict: Verdict = 'running'; outcome: string | null = null;
  constructor(readonly opts: FsmExecOptions) {
    this.runner = new Runner(opts.world, (m) => this.say(m));
    this.evalFn = makeEval((m) => this.say(m)); this.sensors = new Sensors(opts.doc.sensors, this.evalFn);
    Object.assign(this.runner.bb, worldSense(opts.world), opts.doc.blackboard);
    const root = opts.doc.root; const init = initialState(root);
    if (!init) throw new Error('fsm without states');
    this.enter(root, fsmMachines(opts.doc)[0].path, init);
  }
  get bb(): Record<string, unknown> { return this.runner.bb; }
  private say(m: string): void { this.log.push(`[${this.time.toFixed(1)}] ${m}`); this.opts.log?.(m); }
  private ctx(dt: number): TickContext { return { time: this.time, dt, bb: this.runner.bb, log: (m) => this.say(m) }; }
  /** Active state path, e.g. `Mission/Depal/Pick`. */
  where(): string { const f = this.frames.at(-1); return f ? `${f.path}/${f.state.name}` : this.outcome ? `⊙ ${this.outcome}` : ''; }
  private instant(b: ActionBinding): void {
    const run = this.runner.start(b, this.time); if (!run) { this.failed++; return; }
    const st = this.runner.tick(run, this.ctx(0)); if (st === 'running') { this.say(`${b.target}: ${b.kind} ${b.value} is not instantaneous — halted`); this.runner.stop(run, this.ctx(0)); } else if (st === 'failure') this.failed++;
  }
  private enter(m: FsmMachine, path: string, s: FsmState): void {
    const f: Frame = { machine: m, path, state: s, entered: this.time, run: null };
    this.frames.push(f); this.runner.bb.fsm_state = this.where();
    for (const b of s.entry) this.instant(b);
    if (s.kind === 'machine' && s.machine) { const init = initialState(s.machine); if (!init) { this.say(`${path}/${s.name}: empty machine`); return; } this.enter(s.machine, `${path}/${s.name}`, init); return; }
    if (s.kind === 'skill' && s.action) { f.run = this.runner.start(s.action, this.time); if (!f.run) this.failed++; this.say(`${path}/${s.name}: ${s.action.kind} ${s.action.value}${s.action.robot ? ` on ${s.action.robot}` : ''}`); }
  }
  /** The state of frame `i` produced outcome `o`: exit, follow the transition, pop finished machines. */
  private resolve(i: number, o: string): void {
    for (;;) {
      const f = this.frames[i]; const s = f.state;
      // leave deeper frames (machine timeout / cancel) and this state
      for (let k = this.frames.length - 1; k > i; k--) { const g = this.frames[k]; if (g.run) this.runner.stop(g.run, this.ctx(0)); for (const b of g.state.exit) this.instant(b); }
      this.frames.length = i + 1;
      if (f.run) { this.runner.stop(f.run, this.ctx(0)); f.run = null; }
      for (const b of s.exit) this.instant(b);
      let to = s.transitions[o];
      if (to === undefined && o === 'timeout') to = s.transitions[s.action?.fail ?? 'fail'];
      if (to === undefined && f.machine.outcomes.includes(o)) to = o; // an outcome of the enclosing machine ends it (yasmin)
      this.frames.pop(); this.transitions++;
      if (to === undefined) {
        if (o === 'canceled' || o === 'timeout') to = o; // propagate as the machine's outcome
        else { this.trace.push({ t: this.time, path: `${f.path}/${s.name}`, outcome: o, to: '∅' }); this.say(`${f.path}/${s.name}: outcome '${o}' is not mapped — the machine stops`); this.failed++; this.end(o, 'failed'); return; }
      }
      this.trace.push({ t: this.time, path: `${f.path}/${s.name}`, outcome: o, to });
      this.say(`${f.path}/${s.name} → ${o} → ${to}`);
      const next = f.machine.states.find((x) => x.name === to);
      if (next) { this.enter(f.machine, f.path, next); return; }
      // `to` is an outcome of the machine: the machine state one level up produced it
      if (i === 0) { this.end(to, to === 'canceled' ? 'stopped' : 'done'); return; }
      i -= 1; o = to;
    }
  }
  private end(outcome: string, verdict: Verdict): void { this.outcome = outcome; this.status = 'idle'; this.verdict = verdict; this.frames.length = 0; this.runner.bb.fsm_state = `⊙ ${outcome}`; this.say(`finished with outcome '${outcome}'`); }
  /** Request cancellation: the active skill is halted and the innermost state returns `canceled`. */
  cancel(): void { this.cancelReq = true; }
  tick(dt = 0.1): ExecSummary['status'] {
    if (this.status !== 'running') return this.status;
    const w = this.opts.world; const ctx = this.ctx(dt);
    Object.assign(this.runner.bb, worldSense(w));
    const evs = [...(w.events?.() ?? []), ...this.sensors.poll(this.runner.bb)];
    this.runner.bb.event = evs.at(-1) ?? ''; for (const e of evs) this.runner.bb[`ev_${e}`] = true;
    if (this.cancelReq) { this.cancelReq = false; if (this.frames.length) this.resolve(this.frames.length - 1, 'canceled'); }
    const max = this.opts.maxPerTick ?? 50;
    for (let n = 0; n < max && this.status === 'running' && this.frames.length; n++) {
      // state timeouts, outermost first (a machine timeout pre-empts its children)
      const ti = this.frames.findIndex((f) => f.state.timeout !== undefined && this.time - f.entered >= f.state.timeout - 1e-9);
      if (ti >= 0) { this.say(`${this.frames[ti].path}/${this.frames[ti].state.name}: state timeout ${this.frames[ti].state.timeout} s`); this.resolve(ti, 'timeout'); continue; }
      const i = this.frames.length - 1; const f = this.frames[i]; const s = f.state;
      let o: string | null = null;
      if (s.kind === 'pass') o = 'ok';
      else if (s.kind === 'decision') { const hit = s.when.find((wh) => this.evalFn(wh.expr, this.runner.bb)); o = hit?.outcome ?? s.otherwise ?? null; }
      else if (s.kind === 'skill') {
        if (!f.run) o = s.action?.fail ?? 'fail';
        else { const st: BTStatus = this.runner.tick(f.run, ctx); if (st === 'success') o = s.action?.done ?? 'ok'; else if (st === 'failure') { o = f.run.timedOut ? 'timeout' : (s.action?.fail ?? 'fail'); if (!f.run.timedOut) this.failed++; } }
      }
      if (o === null) break; // waiting (running skill or decision without a true condition)
      this.resolve(i, o);
    }
    for (const e of evs) delete this.runner.bb[`ev_${e}`];
    w.step?.(dt);
    this.time += dt; this.ticks++;
    return this.status;
  }
  run(seconds: number, dt = 0.1, stopOn?: (x: FsmExecutor) => boolean): ExecSummary['status'] { const n = Math.ceil(seconds / dt - 1e-9); for (let i = 0; i < n && this.status === 'running'; i++) { this.tick(dt); if (stopOn?.(this)) break; } return this.status; }
  stop(verdict: Verdict = 'stopped'): void { for (const f of this.frames) if (f.run) this.runner.stop(f.run, this.ctx(0)); this.frames.length = 0; this.status = 'stopped'; this.verdict = verdict; }
  summary(): ExecSummary { return { time: this.time, ticks: this.ticks, status: this.status, verdict: this.verdict, fired: this.transitions, mismatches: 0, denied: 0, failed: this.failed, running: this.frames.filter((f) => f.run).map((f) => `${f.state.name}: ${f.state.action?.kind} ${f.state.action?.value}`), where: this.where() }; }
}
