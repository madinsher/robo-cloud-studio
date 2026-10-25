/**
 * Executors that run a control model on the station: an automaton document as a set of parallel state machines
 * with entry actions, and a Petri net as a cell controller whose transitions are robot operations (the hierarchical
 * FSM executor is in fsm.ts). Actions are the studio's programming artefacts — robot programs, targets, mobile zones,
 * component signals — bound to states / transitions with `action` lines (edited in the diagram inspector). The same
 * world bindings as the behavior-tree runtime are used (ctl/runtime.ts `hostBindings`: programs of different robots run
 * concurrently through the program backend, one action per robot at a time).
 *
 * Run-time inputs: every tick the blackboard receives the world's `sense()` values (station signals, robot IO, program
 * state, mobile robot state); `guard` expressions read it, `sensor` lines raise events on its rising edges, and the
 * world's events (arrivals, actions' `event emit=`) drive uncontrollable transitions.
 *
 * End of a run: `verdict` is `done` when the model reaches a final configuration (all automata in marked states / a
 * declared `final` marking or tokens only in sink / resource places), `deadlock` when nothing can happen anymore
 * elsewhere, `failed` when an action failed without a failure outcome.
 */
import type { ActionHandler, BTNode, BTStatus, TickContext } from './bt';
import type { DesDoc, ActionBinding, SensorSpec } from './dsl';
import type { PetriDoc } from './graphdoc';
import { DES, SupervisorRuntime, SupervisorTable } from './des';
import { PetriNet } from './petri';
import type { WorldBindings } from './runtime';
import { setValue } from './runtime';
import { expr, Env } from './expr';

/** The world seen by an executor: shared host actions (program, move, signal, wait, set, event) and per-robot bindings. */
export interface ExecutorWorld {
  host: WorldBindings;
  /** Bindings of a named mobile robot (goto, dock, grasp…); null when unknown. */
  forRobot(name?: string): WorldBindings | null;
  step?(dt: number): void;
  /** Uncontrollable events reported by the world since the last call (arrivals, sensors, operator). */
  events?(): string[];
  /** Blackboard values of the whole world (host signals + mobile robots); default: host.sense(). */
  sense?(): Env;
}
/** World sense values (falls back to the host's). */
export const worldSense = (w: ExecutorWorld): Env => w.sense?.() ?? w.host.sense?.() ?? {};

export interface Running { binding: ActionBinding; handler: ActionHandler; args: Record<string, unknown>; node: BTNode; started: number; halt?: (ctx: TickContext, args: Record<string, unknown>, node?: BTNode) => void; /** set when the action ended by its timeout */ timedOut?: boolean; /** outcome of an action that finished on its first tick (not ticked again) */ result?: BTStatus }
let seq = 0;
const fakeNode = (): BTNode => ({ id: `x${seq++}`, status: 'idle' } as unknown as BTNode);

/** Map a binding to a world action (name + args). */
export function resolveAction(world: ExecutorWorld, b: ActionBinding): { handler: ActionHandler; args: Record<string, unknown>; halt?: Running['halt'] } | null {
  const robotLib = world.forRobot(b.robot);
  const pick = (name: string, args: Record<string, unknown>, withRobot = true) => { const lib = robotLib?.actions[name] ? robotLib : world.host.actions[name] ? world.host : null; return lib ? { handler: lib.actions[name], args: { ...(withRobot && b.robot ? { robot: b.robot } : {}), ...b.args, ...args }, halt: lib.halt?.[name] } : null; };
  switch (b.kind) {
    case 'program': return pick('program', { program: b.value });
    case 'target': return pick('move', { target: b.value });
    case 'goto': return pick('goto', { zone: b.value });
    case 'signal': return pick('signal', { component: b.component, signal: b.value, value: b.args.value ?? true }, false);
    case 'wait': return pick('wait', { seconds: Number(b.value) }, false);
    case 'event': return pick('event', { emit: b.value }, false);
    case 'await': return pick('await', { expr: b.value }, false);
    case 'skill': return pick(b.value, {});
    case 'set': { const { value: _v, ...rest } = b.args; void _v; return pick('set', { ...rest, [b.value]: b.args.value ?? true }, false); }
  }
}

/** Starts, ticks and halts bound actions; enforces the binding's `timeout=`. Shared by the automaton, Petri and FSM executors. */
export class Runner {
  readonly bb: Record<string, unknown> = {};
  constructor(readonly world: ExecutorWorld, readonly log: (m: string) => void) {}
  start(b: ActionBinding, time: number): Running | null {
    const r = resolveAction(this.world, b);
    if (!r) { this.log(`no world action for '${b.kind}${b.kind === 'skill' ? ' ' + b.value : ''}' (${b.target}${b.robot ? `, robot ${b.robot}` : ''})`); return null; }
    return { binding: b, handler: r.handler, args: r.args, node: fakeNode(), halt: r.halt, started: time };
  }
  tick(run: Running, ctx: TickContext): BTStatus {
    if (run.binding.timeout !== undefined && ctx.time - run.started >= run.binding.timeout - 1e-9) { this.stop(run, ctx); run.timedOut = true; this.log(`${run.binding.kind} ${run.binding.value}: timeout after ${run.binding.timeout} s`); return 'failure'; }
    const st = run.handler(ctx, run.args, run.node) ?? 'success';
    run.node.status = st;
    return st;
  }
  stop(run: Running, ctx: TickContext): void { if (run.node.status === 'running') run.halt?.(ctx, run.args, run.node); run.node.status = 'idle'; }
}

export type Verdict = 'running' | 'done' | 'deadlock' | 'failed' | 'stopped' | 'timeout';
export interface ExecSummary { time: number; ticks: number; status: 'running' | 'idle' | 'stopped'; verdict: Verdict; fired: number; mismatches: number; denied: number; failed: number; running: string[]; where: string }

const envOf = (bb: Record<string, unknown>): Env => { const e: Env = {}; for (const [k, v] of Object.entries(bb)) if (typeof v === 'number' || typeof v === 'boolean' || typeof v === 'string') e[k] = v; return e; };
/** Guard / sensor evaluation that never throws (a malformed expression is false and logged once). */
export function makeEval(log: (m: string) => void): (src: string, bb: Record<string, unknown>) => boolean {
  const bad = new Set<string>();
  return (src, bb) => { try { return expr(src).bool(envOf(bb)); } catch (e) { if (!bad.has(src)) { bad.add(src); log(`expression '${src}': ${(e as Error).message}`); } return false; } };
}
/** Rising-edge detector for `sensor` lines. */
export class Sensors {
  private prev = new Map<string, boolean>();
  constructor(readonly specs: SensorSpec[], readonly evalFn: (src: string, bb: Record<string, unknown>) => boolean) {}
  /** Events of the expressions that became true since the last poll (the first poll only records the baseline). */
  poll(bb: Record<string, unknown>): string[] {
    const out: string[] = [];
    for (const s of this.specs) { const k = `${s.expr}|${s.event}`; const v = this.evalFn(s.expr, bb); const p = this.prev.get(k); if (v && p === false) out.push(s.event); this.prev.set(k, v); }
    return out;
  }
}

// ---------------------------------------------------------------------------------------------
// Automata with entry actions
// ---------------------------------------------------------------------------------------------

export interface AutomatonExecOptions { doc: DesDoc; world: ExecutorWorld; supervisor?: SupervisorTable; log?: (m: string) => void; /** fire enabled controllable events automatically (default true) */ auto?: boolean }

/**
 * Every `automaton` block runs as a state machine; components synchronise on shared events. Entering a state starts
 * its bound action; when the action succeeds the `done=` event is fired, when it fails the `fail=` event (without
 * `fail=` the component stops there: verdict `failed`). Controllable events are fired automatically when enabled in
 * every component that shares them, allowed by the supervisor and by their `guard` (data-driven decisions); a state
 * with a bound action waits for the action. Uncontrollable events come from the world (`event` actions, sensors,
 * mobile arrivals). Unobservable events move the plant but not the supervisor (its table is built over the observer).
 */
export class AutomatonExecutor {
  readonly comps: DES[];
  readonly state = new Map<string, string>();
  readonly running = new Map<string, Running>();
  /** components whose entry action already completed in the current state (or that have none) */
  private settled = new Set<string>();
  /** components stopped by a failed action without a `fail=` outcome */
  readonly blocked = new Set<string>();
  readonly supervisor: SupervisorRuntime | null;
  readonly log: string[] = []; readonly trace: Array<{ t: number; event: string; states: string }> = [];
  private runner: Runner; private uc: Set<string>; private evalFn: (src: string, bb: Record<string, unknown>) => boolean; private sensors: Sensors;
  /** last firing tick per controllable event — auto-firing is fair (least recently fired first), so no component starves */
  private lastFired = new Map<string, number>();
  time = 0; ticks = 0; fired = 0; mismatches = 0; denied = 0; failed = 0; status: ExecSummary['status'] = 'running'; verdict: Verdict = 'running';
  constructor(readonly opts: AutomatonExecOptions) {
    this.comps = opts.doc.plant.map((a) => DES.fromSpec(a));
    for (const c of this.comps) this.state.set(c.name, c.x0);
    this.uc = new Set(opts.doc.uncontrollable);
    this.supervisor = opts.supervisor ? new SupervisorRuntime(opts.supervisor) : null;
    this.runner = new Runner(opts.world, (m) => this.say(m));
    this.evalFn = makeEval((m) => this.say(m)); this.sensors = new Sensors(opts.doc.sensors ?? [], this.evalFn);
    Object.assign(this.runner.bb, worldSense(opts.world), opts.doc.blackboard ?? {});
    for (const c of this.comps) this.enter(c);
  }
  get bb(): Record<string, unknown> { return this.runner.bb; }
  private say(m: string): void { this.log.push(`[${this.time.toFixed(1)}] ${m}`); this.opts.log?.(m); }
  private actionFor(c: DES, s: string): ActionBinding | undefined { return this.opts.doc.actions.find((a) => a.target === `${c.name}.${s}`) ?? this.opts.doc.actions.find((a) => !a.target.includes('.') && a.target === s && c.X.has(s)); }
  private enter(c: DES): void {
    const s = this.state.get(c.name)!; const b = this.actionFor(c, s);
    this.settled.delete(c.name); this.blocked.delete(c.name);
    if (!b) { this.settled.add(c.name); return; }
    const run = this.runner.start(b, this.time);
    if (run) { this.running.set(c.name, run); this.say(`${c.name} enters ${s}: ${b.kind} ${b.value}${b.robot ? ` on ${b.robot}` : ''}`); } else { this.failed++; this.blocked.add(c.name); }
  }
  /** Fire an event in every component whose alphabet contains it. */
  fire(ev: string, source: 'auto' | 'done' | 'fail' | 'world' = 'auto'): boolean {
    const parts = this.comps.filter((c) => c.sigma.has(ev));
    if (!parts.length) { this.say(`event ${ev} belongs to no component`); this.mismatches++; return false; }
    for (const c of parts) if (c.next(this.state.get(c.name)!, ev) === undefined) { this.mismatches++; this.say(`model mismatch: ${ev} (${source}) not enabled in ${c.name}=${this.state.get(c.name)}`); return false; }
    if (this.supervisor) { try { this.supervisor.observe(ev); } catch (e) { this.mismatches++; this.say(`supervisor: ${(e as Error).message}`); } }
    for (const c of parts) {
      const ctx = this.ctx(0); const run = this.running.get(c.name); if (run) { this.runner.stop(run, ctx); this.running.delete(c.name); }
      this.state.set(c.name, c.next(this.state.get(c.name)!, ev)!);
    }
    this.fired++; this.lastFired.set(ev, this.ticks); this.trace.push({ t: this.time, event: ev, states: this.where() });
    for (const c of parts) this.enter(c);
    return true;
  }
  private ctx(dt: number): TickContext { return { time: this.time, dt, bb: this.runner.bb, log: (m) => this.say(m) }; }
  where(): string { return this.comps.map((c) => `${c.name}=${this.state.get(c.name)}`).join(' '); }
  /** Controllable events enabled in every component sharing them (all settled), whose guard holds; `countDenied` counts supervisor denials (once per tick). */
  enabledAuto(countDenied = false): string[] {
    const out: string[] = [];
    for (const c of this.comps) {
      if (!this.settled.has(c.name)) continue;
      for (const ev of c.active(this.state.get(c.name)!)) {
        if (this.uc.has(ev) || out.includes(ev)) continue;
        if (!this.comps.every((d) => !d.sigma.has(ev) || (this.settled.has(d.name) && d.next(this.state.get(d.name)!, ev) !== undefined))) continue;
        const g = this.opts.doc.guards?.[ev]; if (g && !this.evalFn(g, this.runner.bb)) continue;
        if (this.supervisor && !this.supervisor.allowed(ev)) { if (countDenied) this.denied++; continue; }
        out.push(ev);
      }
    }
    return out;
  }
  /** Some component can still receive an uncontrollable event (from the world, a sensor or a running action). */
  private waitingForWorld(): boolean { return this.comps.some((c) => !this.blocked.has(c.name) && [...c.active(this.state.get(c.name)!)].some((e) => this.uc.has(e))) && (!!this.opts.world.events || !!this.opts.doc.sensors?.length); }
  tick(dt = 0.1): ExecSummary['status'] {
    if (this.status !== 'running') return this.status;
    const w = this.opts.world; const ctx = this.ctx(dt);
    Object.assign(this.runner.bb, worldSense(w));
    for (const ev of [...(w.events?.() ?? []), ...this.sensors.poll(this.runner.bb)]) this.fire(ev, 'world');
    for (const [name, run] of [...this.running]) {
      if (this.running.get(name) !== run) continue;
      const st = this.runner.tick(run, ctx);
      if (st === 'running') continue;
      this.running.delete(name);
      if (st === 'failure') {
        this.failed++; this.say(`${name}: action ${run.binding.kind} ${run.binding.value} ${run.timedOut ? 'timed out' : 'failed'}${run.binding.fail ? ` → ${run.binding.fail}` : ' (no fail= outcome: the component stops)'}`);
        if (run.binding.fail) { this.settled.add(name); this.fire(run.binding.fail, 'fail'); } else this.blocked.add(name);
        continue;
      }
      this.settled.add(name);
      if (run.binding.done) this.fire(run.binding.done, 'done');
    }
    if (this.opts.auto !== false) { const evs = this.enabledAuto(true); if (evs.length) { const pick = evs.slice().sort((a, b) => (this.lastFired.get(a) ?? -1) - (this.lastFired.get(b) ?? -1))[0]; this.fire(pick, 'auto'); } }
    w.step?.(dt);
    this.time += dt; this.ticks++;
    // ends when nothing can happen any more; with `terminate marked` also when the task is complete: every automaton (and
    // the supervisor) marked after at least one event, no action running, no controllable event enabled
    if (!this.running.size && !this.enabledAuto().length && (!this.waitingForWorld() || (this.opts.doc.terminate === 'marked' && this.fired > 0 && this.allMarked()))) this.finish();
    return this.status;
  }
  private allMarked(): boolean { return !this.blocked.size && this.comps.every((c) => c.xm.has(this.state.get(c.name)!)) && (!this.supervisor || this.supervisor.marked()); }
  private finish(): void {
    this.status = 'idle';
    const marked = this.comps.every((c) => c.xm.has(this.state.get(c.name)!)) && (!this.supervisor || this.supervisor.marked());
    this.verdict = this.blocked.size ? 'failed' : marked ? 'done' : 'deadlock';
    this.say(`${this.verdict === 'done' ? 'done' : this.verdict === 'failed' ? 'stopped by a failed action' : 'DEADLOCK'} at ${this.where()}${this.blocked.size ? ` (blocked: ${[...this.blocked].join(', ')})` : ''}`);
  }
  run(seconds: number, dt = 0.1, stopOn?: (x: AutomatonExecutor) => boolean): ExecSummary['status'] { const n = Math.ceil(seconds / dt - 1e-9); for (let i = 0; i < n && this.status === 'running'; i++) { this.tick(dt); if (stopOn?.(this)) break; } return this.status; }
  stop(verdict: Verdict = 'stopped'): void { const ctx = this.ctx(0); for (const run of this.running.values()) this.runner.stop(run, ctx); this.running.clear(); this.status = 'stopped'; this.verdict = verdict; }
  summary(): ExecSummary { return { time: this.time, ticks: this.ticks, status: this.status, verdict: this.verdict, fired: this.fired, mismatches: this.mismatches, denied: this.denied, failed: this.failed, running: [...this.running.entries()].map(([c, r]) => `${c}: ${r.binding.kind} ${r.binding.value}`), where: this.where() }; }
}

// ---------------------------------------------------------------------------------------------
// Petri nets as cell controllers
// ---------------------------------------------------------------------------------------------

export interface PetriExecOptions { doc: PetriDoc; world: ExecutorWorld; log?: (m: string) => void; /** max firings per tick (zero-delay loops) */ maxPerTick?: number; /** seed of the exponential delays of `rate=` transitions */ seed?: number; /** end the run on a partial deadlock (some processes blocked for ever while others still cycle), default true */ stopOnPartialDeadlock?: boolean }

/**
 * Interpreted Petri net: a transition starts when it is enabled by the marking (capacities respected), its `guard`
 * holds, its input event (`on=`) occurred in this tick and fewer than `servers` (default 1) firings of it are in
 * progress. Immediate transitions fire first (zero time, highest priority); others in priority, then document order.
 * Starting removes the input tokens and runs the bound action (or the `delay=` / sampled `rate=` time); the output
 * tokens appear when it finishes. A failed action returns the input tokens and disables the transition for the run.
 */
export class PetriExecutor {
  readonly net: PetriNet; marking: number[];
  readonly active: Array<{ t: number; run: Running | null; end: number; start: number }> = [];
  readonly log: string[] = []; readonly firings: Record<string, number> = {};
  readonly trace: Array<{ t: number; transition: string; phase: 'start' | 'end' | 'fail' }> = [];
  private runner: Runner; private evalFn: (src: string, bb: Record<string, unknown>) => boolean; private sensors: Sensors; private faulted = new Set<number>(); private rng: () => number; private lastStart = new Map<number, number>();
  time = 0; ticks = 0; fired = 0; failed = 0; mismatches = 0; status: ExecSummary['status'] = 'running'; verdict: Verdict = 'running';
  constructor(readonly opts: PetriExecOptions) {
    this.net = new PetriNet(opts.doc.spec); this.marking = [...this.net.m0]; this.runner = new Runner(opts.world, (m) => this.say(m));
    this.evalFn = makeEval((m) => this.say(m)); this.sensors = new Sensors(opts.doc.sensors ?? [], this.evalFn);
    Object.assign(this.runner.bb, worldSense(opts.world), opts.doc.blackboard ?? {});
    let s = (opts.seed ?? 1) >>> 0; this.rng = () => { s = (s + 0x6d2b79f5) >>> 0; let t = s; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  }
  get bb(): Record<string, unknown> { return this.runner.bb; }
  private say(m: string): void { this.log.push(`[${this.time.toFixed(1)}] ${m}`); this.opts.log?.(m); }
  private ctx(dt: number): TickContext { return { time: this.time, dt, bb: this.runner.bb, log: (m) => this.say(m) }; }
  where(): string { return this.net.places.map((p, i) => (this.marking[i] ? `${p.id}${this.marking[i] > 1 ? `×${this.marking[i]}` : ''}` : '')).filter(Boolean).join(' '); }
  private inProgress(t: number): number { return this.active.filter((a) => a.t === t).length; }
  /** Transitions that may start now (marking, servers, guard, input events still pending in `evs`). */
  candidates(evs: string[] = []): number[] {
    const out: number[] = [];
    for (let j = 0; j < this.net.transitions.length; j++) {
      const tr = this.net.transitions[j];
      if (this.faulted.has(j) || !this.net.isEnabled(this.marking, j)) continue;
      if (this.inProgress(j) >= (tr.servers ?? 1)) continue;
      if (tr.on && !evs.includes(tr.on)) continue;
      if (tr.guard && !this.evalFn(tr.guard, this.runner.bb)) continue;
      out.push(j);
    }
    const imm = out.filter((j) => this.net.transitions[j].immediate);
    const pool = imm.length ? imm : out;
    const top = Math.max(...pool.map((j) => this.net.transitions[j].priority ?? 0));
    // fair among equal priorities: the transition started least recently first (document order breaks ties), so no process starves
    return pool.filter((j) => (this.net.transitions[j].priority ?? 0) === top).sort((a, b) => (this.lastStart.get(a) ?? -1) - (this.lastStart.get(b) ?? -1) || a - b);
  }
  private duration(t: number): number { const tr = this.net.transitions[t]; if (tr.immediate) return 0; if (tr.delay !== undefined) return tr.delay; if (tr.rate !== undefined && tr.rate > 0) return -Math.log(1 - this.rng()) / tr.rate; return 0; }
  tick(dt = 0.1): ExecSummary['status'] {
    if (this.status !== 'running') return this.status;
    const w = this.opts.world; const ctx = this.ctx(dt);
    Object.assign(this.runner.bb, worldSense(w));
    const evs = [...(w.events?.() ?? []), ...this.sensors.poll(this.runner.bb)];
    // complete (document order of start)
    for (const a of [...this.active]) {
      let done = false;
      if (a.run) {
        const st = a.run.result ?? this.runner.tick(a.run, ctx);
        if (st === 'failure') {
          const id = this.net.transitionId(a.t); this.failed++; this.faulted.add(a.t);
          this.marking = this.marking.map((v, k) => v + this.net.pre[a.t][k]); this.active.splice(this.active.indexOf(a), 1);
          this.trace.push({ t: this.time, transition: id, phase: 'fail' }); this.say(`${id}: action ${a.run.binding.kind} ${a.run.binding.value} ${a.run.timedOut ? 'timed out' : 'failed'} — tokens returned, ${id} disabled`); continue;
        }
        done = st === 'success' && this.time >= a.end - 1e-9;
      } else done = this.time >= a.end - 1e-9;
      if (done) { this.marking = this.marking.map((v, k) => v + this.net.post[a.t][k]); const id = this.net.transitionId(a.t); this.firings[id] = (this.firings[id] ?? 0) + 1; this.active.splice(this.active.indexOf(a), 1); this.trace.push({ t: this.time, transition: id, phase: 'end' }); this.say(`${id} completed → ${this.where()}`); }
    }
    // fire (not after a partial deadlock: the firings in progress complete, nothing new starts)
    let n = 0; const max = this.opts.maxPerTick ?? 20; const pendingEv = [...evs];
    for (;;) {
      if (this.draining) break;
      const en = this.candidates(pendingEv); if (!en.length || n++ >= max) break;
      const t = en[0]; const tr = this.net.transitions[t];
      const id = this.net.transitionId(t); const b = this.opts.doc.actions.find((x) => x.target === id);
      let run: Running | null = null;
      if (b) { run = this.runner.start(b, this.time); if (!run) { this.failed++; this.mismatches++; this.faulted.add(t); continue; } }
      if (tr.on) pendingEv.splice(pendingEv.indexOf(tr.on), 1);
      this.marking = this.marking.map((v, k) => v - this.net.pre[t][k]);
      const d = this.duration(t);
      this.active.push({ t, run, start: this.time, end: this.time + d });
      this.fired++; this.lastStart.set(t, this.ticks * 1e6 + n); this.trace.push({ t: this.time, transition: id, phase: 'start' }); this.say(`${id} started${b ? `: ${b.kind} ${b.value}${b.robot ? ` on ${b.robot}` : ''}` : d ? ` (${d.toFixed(2)} s)` : ''} → ${this.where()}`);
      if (!run && d === 0) { this.marking = this.marking.map((v, k) => v + this.net.post[t][k]); this.firings[id] = (this.firings[id] ?? 0) + 1; this.active.pop(); this.trace.push({ t: this.time, transition: id, phase: 'end' }); }
      else if (run) { const st = this.runner.tick(run, ctx); if (st !== 'running') run.result = st; /* completion is reported on the next tick */ }
    }
    w.step?.(dt);
    this.time += dt; this.ticks++;
    if (this.draining) {
      if (!this.active.length) { this.status = 'idle'; this.verdict = 'deadlock'; this.say(`stopped after the partial deadlock at marking {${this.where()}}: the firings in progress have completed`); }
      return this.status;
    }
    const waitsForEvent = this.net.transitions.some((tr, j) => !this.faulted.has(j) && (tr.on || tr.guard) && this.net.isEnabled(this.marking, j) && this.inProgress(j) < (tr.servers ?? 1));
    if (!this.active.length && !this.candidates().length && !waitsForEvent) this.finish();
    else if (this.opts.stopOnPartialDeadlock !== false) this.checkPartialDeadlock();
    return this.status;
  }
  private lastChecked = ''; private noDeadlockCheck = false; private draining = false;
  /**
   * Partial deadlock: from the marking the running firings lead to, some token-holding activity places can never be left
   * (all their output transitions are dead in the reachability set from there — structure only, so guards / events /
   * timing cannot revive them) while other transitions still cycle. Exact for bounded nets; skipped for large ones.
   * Not a deadlock while a declared `final` marking is still reachable (the places left marked are part of the
   * declared end state, e.g. the loser of a race). When it is one, no new firing starts and the run ends once the
   * firings in progress have completed (robots are not stopped in mid-motion).
   */
  private checkPartialDeadlock(): void {
    if (this.noDeadlockCheck) return;
    const m = this.marking.map((v, i) => v + this.active.reduce((s, a) => s + this.net.post[a.t][i], 0));
    const key = `${m.join(',')}|${this.marking.join(',')}`; if (key === this.lastChecked) return; this.lastChecked = key;
    const spec = this.opts.doc.spec; const sub = new PetriNet({ ...spec, places: spec.places.map((p, i) => ({ ...p, tokens: m[i] })) });
    const rg = sub.reachability({ limit: 3000 }); if (rg.truncated) { this.noDeadlockCheck = true; return; }
    const live = new Set(rg.edges.map((e) => e.t));
    const dead = (j: number) => !live.has(j) && !this.faulted.has(j);
    const blocked = this.net.places.map((p, i) => i).filter((i) => m[i] > 0 && !['resource', 'idle', 'monitor'].includes(this.net.places[i].kind ?? '') && this.net.placeOut(i).length > 0 && this.net.placeOut(i).every(dead));
    // reported when it has happened: every blocked place holds its tokens now (no running firing still produces into it)
    if (!blocked.length || blocked.some((i) => this.marking[i] < m[i])) return;
    if (this.opts.doc.finals?.length && rg.markings.some((x) => this.isFinalMarking(x))) return;
    const deadT = this.net.transitions.map((_, j) => j).filter(dead).map((j) => this.net.transitionId(j));
    this.draining = true; this.verdict = 'deadlock';
    if (!this.active.length) this.status = 'idle';
    this.say(`DEADLOCK (partial) at marking {${this.where()}}: ${blocked.map((i) => this.net.places[i].id).join(', ')} can never be left; dead transitions ${deadT.join(', ')}${this.active.length ? `; no new firing starts, waiting for ${this.active.map((a) => this.net.transitionId(a.t)).join(', ')} to complete` : ''}`);
  }
  /** Final marking reached: a declared `final` marking (= for the places listed), else tokens only in sink places or resource / idle / monitor places. */
  isFinal(): boolean { return this.isFinalMarking(this.marking); }
  private isFinalMarking(mk: number[]): boolean {
    const finals = this.opts.doc.finals;
    if (finals?.length) return finals.some((f) => this.net.places.every((p, i) => (f[p.id] === undefined ? true : mk[i] === f[p.id])) && Object.keys(f).every((k) => this.net.pIndex.has(k)));
    return this.net.places.every((p, i) => !mk[i] || this.net.placeOut(i).length === 0 || ['resource', 'idle', 'monitor'].includes(p.kind ?? ''));
  }
  private finish(): void {
    this.status = 'idle';
    this.verdict = this.isFinal() ? (this.faulted.size ? 'failed' : 'done') : this.faulted.size ? 'failed' : 'deadlock';
    this.say(`no enabled transition at marking {${this.where()}} — ${this.verdict === 'done' ? 'final marking reached' : this.verdict === 'failed' ? `failed actions (${[...this.faulted].map((j) => this.net.transitionId(j)).join(', ')})` : this.fired ? 'DEADLOCK' : 'DEADLOCK: nothing can fire'}`);
  }
  run(seconds: number, dt = 0.1, stopOn?: (x: PetriExecutor) => boolean): ExecSummary['status'] { const n = Math.ceil(seconds / dt - 1e-9); for (let i = 0; i < n && this.status === 'running'; i++) { this.tick(dt); if (stopOn?.(this)) break; } return this.status; }
  stop(verdict: Verdict = 'stopped'): void { const ctx = this.ctx(0); for (const a of this.active) if (a.run) this.runner.stop(a.run, ctx); this.active.length = 0; this.status = 'stopped'; this.verdict = verdict; }
  summary(): ExecSummary { return { time: this.time, ticks: this.ticks, status: this.status, verdict: this.verdict, fired: this.fired, mismatches: this.mismatches, denied: 0, failed: this.failed, running: this.active.map((a) => `${this.net.transitionId(a.t)}${a.run ? `: ${a.run.binding.kind} ${a.run.binding.value}` : ''}`), where: this.where() }; }
}
export { setValue };
