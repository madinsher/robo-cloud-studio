/**
 * Executors that run a control model on the station: an automaton document as a set of parallel state machines
 * with entry actions, and a Petri net as a cell controller whose transitions are robot operations. Actions are the
 * studio's programming artefacts — robot programs, targets, mobile zones, component signals — bound to states /
 * transitions with `action` lines (edited in the diagram inspector). The same world bindings as the behavior-tree
 * runtime are used, so a program written in the Program tab, a target taught in the 3D view or a zone drawn on the
 * map become the body of a state or a transition.
 */
import type { ActionHandler, BTNode, BTStatus, TickContext } from './bt';
import type { DesDoc, ActionBinding } from './dsl';
import type { PetriDoc } from './graphdoc';
import { DES, SupervisorRuntime, SupervisorTable, SupervisorViolation } from './des';
import { PetriNet } from './petri';
import type { WorldBindings } from './runtime';

/** The world seen by an executor: shared host actions (program, move, signal, wait, set, event) and per-robot bindings. */
export interface ExecutorWorld {
  host: WorldBindings;
  /** Bindings of a named mobile robot (goto, dock, grasp…); null when unknown. */
  forRobot(name?: string): WorldBindings | null;
  step?(dt: number): void;
  /** Uncontrollable events reported by the world since the last call (arrivals, sensors, operator). */
  events?(): string[];
}

interface Running { binding: ActionBinding; handler: ActionHandler; args: Record<string, unknown>; node: BTNode; halt?: (ctx: TickContext, args: Record<string, unknown>) => void }
let seq = 0;
const fakeNode = (): BTNode => ({ id: `x${seq++}`, status: 'idle' } as unknown as BTNode);

/** Map a binding to a world action (name + args). */
export function resolveAction(world: ExecutorWorld, b: ActionBinding): { handler: ActionHandler; args: Record<string, unknown>; halt?: Running['halt'] } | null {
  const robotLib = world.forRobot(b.robot);
  const pick = (name: string, args: Record<string, unknown>, withRobot = true) => { const lib = robotLib?.actions[name] ? robotLib : world.host.actions[name] ? world.host : null; return lib ? { handler: lib.actions[name], args: { ...(withRobot && b.robot ? { robot: b.robot } : {}), ...b.args, ...args }, halt: lib.halt?.[name] } : null; };
  switch (b.kind) {
    case 'program': return pick('program', { name: b.value });
    case 'target': return pick('move', { target: b.value });
    case 'goto': return pick('goto', { zone: b.value });
    case 'signal': return pick('signal', { component: b.component, name: b.value, value: b.args.value ?? true }, false);
    case 'wait': return pick('wait', { seconds: Number(b.value) }, false);
    case 'event': return pick('event', { emit: b.value }, false);
    case 'set': { const { value: _v, ...rest } = b.args; void _v; return pick('set', { ...rest, [b.value]: b.args.value ?? true }, false); }
  }
}

class Runner {
  readonly bb: Record<string, unknown> = {};
  constructor(readonly world: ExecutorWorld, readonly log: (m: string) => void) {}
  start(b: ActionBinding, time: number): Running | null {
    const r = resolveAction(this.world, b);
    if (!r) { this.log(`no world action for '${b.kind}' (${b.target}${b.robot ? `, robot ${b.robot}` : ''})`); return null; }
    const run: Running = { binding: b, handler: r.handler, args: r.args, node: fakeNode(), halt: r.halt };
    void time; return run;
  }
  tick(run: Running, ctx: TickContext): BTStatus {
    const st = run.handler(ctx, run.args, run.node) ?? 'success';
    run.node.status = st;
    return st;
  }
  stop(run: Running, ctx: TickContext): void { if (run.node.status === 'running') run.halt?.(ctx, run.args); run.node.status = 'idle'; }
}

export interface ExecSummary { time: number; ticks: number; status: 'running' | 'idle' | 'stopped'; fired: number; mismatches: number; denied: number; failed: number; running: string[]; where: string }

// ---------------------------------------------------------------------------------------------
// Automata with entry actions
// ---------------------------------------------------------------------------------------------

export interface AutomatonExecOptions { doc: DesDoc; world: ExecutorWorld; supervisor?: SupervisorTable; log?: (m: string) => void; /** fire enabled controllable events automatically (default true) */ auto?: boolean }

/**
 * Every `automaton` block runs as a state machine; components synchronise on shared events. Entering a state starts
 * its bound action; when the action succeeds the `done=` event is fired (it must be enabled there). Controllable
 * events are fired automatically when enabled in every component that shares them and allowed by the supervisor;
 * a state with a bound action waits for the action before firing. Uncontrollable events come from the world.
 */
export class AutomatonExecutor {
  readonly comps: DES[];
  readonly state = new Map<string, string>();
  readonly running = new Map<string, Running>();
  /** components whose entry action already completed in the current state (or that have none) */
  private settled = new Set<string>();
  readonly supervisor: SupervisorRuntime | null;
  readonly log: string[] = []; readonly trace: Array<{ t: number; event: string; states: string }> = [];
  private runner: Runner; private uc: Set<string>;
  /** last firing tick per controllable event — auto-firing is fair (least recently fired first), so no component starves */
  private lastFired = new Map<string, number>();
  time = 0; ticks = 0; fired = 0; mismatches = 0; denied = 0; failed = 0; status: ExecSummary['status'] = 'running';
  constructor(readonly opts: AutomatonExecOptions) {
    this.comps = opts.doc.plant.map((a) => DES.fromSpec(a));
    for (const c of this.comps) this.state.set(c.name, c.x0);
    this.uc = new Set(opts.doc.uncontrollable);
    this.supervisor = opts.supervisor ? new SupervisorRuntime(opts.supervisor) : null;
    this.runner = new Runner(opts.world, (m) => this.say(m));
    for (const c of this.comps) this.enter(c);
  }
  private say(m: string): void { this.log.push(`[${this.time.toFixed(1)}] ${m}`); this.opts.log?.(m); }
  private actionFor(c: DES, s: string): ActionBinding | undefined { return this.opts.doc.actions.find((a) => a.target === `${c.name}.${s}`) ?? this.opts.doc.actions.find((a) => !a.target.includes('.') && a.target === s && c.X.has(s)); }
  private enter(c: DES): void {
    const s = this.state.get(c.name)!; const b = this.actionFor(c, s);
    this.settled.delete(c.name);
    if (!b) { this.settled.add(c.name); return; }
    const run = this.runner.start(b, this.time);
    if (run) { this.running.set(c.name, run); this.say(`${c.name} enters ${s}: ${b.kind} ${b.value}${b.robot ? ` on ${b.robot}` : ''}`); } else { this.settled.add(c.name); this.failed++; }
  }
  /** Fire an event in every component whose alphabet contains it. */
  fire(ev: string, source: 'auto' | 'done' | 'world' = 'auto'): boolean {
    const parts = this.comps.filter((c) => c.sigma.has(ev));
    if (!parts.length) { this.say(`event ${ev} belongs to no component`); this.mismatches++; return false; }
    for (const c of parts) if (c.next(this.state.get(c.name)!, ev) === undefined) { this.mismatches++; this.say(`model mismatch: ${ev} (${source}) not enabled in ${c.name}=${this.state.get(c.name)}`); return false; }
    if (this.supervisor) { try { this.supervisor.observe(ev); } catch (e) { if (e instanceof SupervisorViolation) { this.mismatches++; this.say(`supervisor: ${e.message}`); } else throw e; } }
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
  /** Controllable events enabled in every component sharing them, allowed by the supervisor, from settled components. */
  enabledAuto(): string[] {
    const out: string[] = [];
    for (const c of this.comps) {
      if (!this.settled.has(c.name)) continue;
      for (const ev of c.active(this.state.get(c.name)!)) {
        if (this.uc.has(ev) || out.includes(ev)) continue;
        if (!this.comps.every((d) => !d.sigma.has(ev) || (this.settled.has(d.name) && d.next(this.state.get(d.name)!, ev) !== undefined))) continue;
        if (this.supervisor && !this.supervisor.allowed(ev)) { this.denied++; continue; }
        out.push(ev);
      }
    }
    return out;
  }
  tick(dt = 0.1): ExecSummary['status'] {
    if (this.status !== 'running') return this.status;
    const w = this.opts.world; const ctx = this.ctx(dt);
    Object.assign(this.runner.bb, w.host.sense?.() ?? {});
    for (const ev of w.events?.() ?? []) this.fire(ev, 'world');
    for (const [name, run] of [...this.running]) {
      const st = this.runner.tick(run, ctx);
      if (st === 'running') continue;
      this.running.delete(name); this.settled.add(name);
      if (st === 'failure') { this.failed++; this.say(`${name}: action ${run.binding.kind} ${run.binding.value} failed`); continue; }
      if (run.binding.done) this.fire(run.binding.done, 'done');
    }
    if (this.opts.auto !== false) { const evs = this.enabledAuto(); if (evs.length) { const pick = evs.slice().sort((a, b) => (this.lastFired.get(a) ?? -1) - (this.lastFired.get(b) ?? -1))[0]; this.fire(pick, 'auto'); } }
    w.step?.(dt);
    this.time += dt; this.ticks++;
    if (!this.running.size && !this.enabledAuto().length && !(w.events && this.comps.some((c) => [...c.active(this.state.get(c.name)!)].some((e) => this.uc.has(e))))) { this.status = 'idle'; this.say(`quiescent at ${this.where()}`); }
    return this.status;
  }
  run(seconds: number, dt = 0.1, stopOn?: (x: AutomatonExecutor) => boolean): ExecSummary['status'] { const n = Math.ceil(seconds / dt); for (let i = 0; i < n && this.status === 'running'; i++) { this.tick(dt); if (stopOn?.(this)) break; } return this.status; }
  stop(): void { const ctx = this.ctx(0); for (const run of this.running.values()) this.runner.stop(run, ctx); this.running.clear(); this.status = 'stopped'; }
  summary(): ExecSummary { return { time: this.time, ticks: this.ticks, status: this.status, fired: this.fired, mismatches: this.mismatches, denied: this.denied, failed: this.failed, running: [...this.running.entries()].map(([c, r]) => `${c}: ${r.binding.kind} ${r.binding.value}`), where: this.where() }; }
}

// ---------------------------------------------------------------------------------------------
// Petri nets as cell controllers
// ---------------------------------------------------------------------------------------------

export interface PetriExecOptions { doc: PetriDoc; world: ExecutorWorld; log?: (m: string) => void; /** max firings per tick (zero-delay loops) */ maxPerTick?: number }

/**
 * Transitions fire when enabled (priority, then document order): the input tokens are removed, the bound action
 * runs on the station (or the `delay=` elapses), the output tokens are produced when it finishes. Conflicts are
 * resolved by priority; a transition whose robot is busy simply waits — the marking shows where the cell is.
 */
export class PetriExecutor {
  readonly net: PetriNet; marking: number[];
  readonly active: Array<{ t: number; run: Running | null; end: number; start: number }> = [];
  readonly log: string[] = []; readonly firings: Record<string, number> = {};
  private runner: Runner;
  time = 0; ticks = 0; fired = 0; failed = 0; mismatches = 0; status: ExecSummary['status'] = 'running';
  constructor(readonly opts: PetriExecOptions) { this.net = new PetriNet(opts.doc.spec); this.marking = [...this.net.m0]; this.runner = new Runner(opts.world, (m) => this.say(m)); }
  private say(m: string): void { this.log.push(`[${this.time.toFixed(1)}] ${m}`); this.opts.log?.(m); }
  private ctx(dt: number): TickContext { return { time: this.time, dt, bb: this.runner.bb, log: (m) => this.say(m) }; }
  where(): string { return this.net.places.map((p, i) => (this.marking[i] ? `${p.id}${this.marking[i] > 1 ? `×${this.marking[i]}` : ''}` : '')).filter(Boolean).join(' '); }
  tick(dt = 0.1): ExecSummary['status'] {
    if (this.status !== 'running') return this.status;
    const w = this.opts.world; const ctx = this.ctx(dt);
    Object.assign(this.runner.bb, w.host.sense?.() ?? {});
    // complete
    for (let i = this.active.length - 1; i >= 0; i--) {
      const a = this.active[i];
      let done = false;
      if (a.run) { const st = this.runner.tick(a.run, ctx); if (st === 'failure') { this.failed++; this.say(`${this.net.transitionId(a.t)}: action failed — tokens returned`); this.marking = this.marking.map((v, k) => v + this.net.pre[a.t][k]); this.active.splice(i, 1); continue; } done = st === 'success'; }
      else done = this.time >= a.end;
      if (done) { this.marking = this.marking.map((v, k) => v + this.net.post[a.t][k]); const id = this.net.transitionId(a.t); this.firings[id] = (this.firings[id] ?? 0) + 1; this.active.splice(i, 1); this.say(`${id} completed → ${this.where()}`); }
    }
    // fire
    let n = 0; const max = this.opts.maxPerTick ?? 20;
    for (;;) {
      const en = this.net.enabled(this.marking); if (!en.length || n++ >= max) break;
      const top = Math.max(...en.map((j) => this.net.transitions[j].priority ?? 0)); const t = en.find((j) => (this.net.transitions[j].priority ?? 0) === top)!;
      const id = this.net.transitionId(t); const b = this.opts.doc.actions.find((x) => x.target === id);
      let run: Running | null = null;
      if (b) { run = this.runner.start(b, this.time); if (!run) { this.failed++; this.mismatches++; break; } }
      this.marking = this.marking.map((v, k) => v - this.net.pre[t][k]);
      this.active.push({ t, run, start: this.time, end: this.time + (this.net.transitions[t].delay ?? 0) });
      this.fired++; this.say(`${id} started${b ? `: ${b.kind} ${b.value}${b.robot ? ` on ${b.robot}` : ''}` : ''} → ${this.where()}`);
      if (run) { const st = this.runner.tick(run, ctx); if (st !== 'running') { /* instantaneous action: complete on the next tick */ } }
    }
    w.step?.(dt);
    this.time += dt; this.ticks++;
    if (!this.active.length && !this.net.enabled(this.marking).length) { this.status = 'idle'; this.say(`no enabled transition at marking {${this.where()}} — ${this.fired ? 'deadlock or end of the run' : 'nothing to do'}`); }
    return this.status;
  }
  run(seconds: number, dt = 0.1, stopOn?: (x: PetriExecutor) => boolean): ExecSummary['status'] { const n = Math.ceil(seconds / dt); for (let i = 0; i < n && this.status === 'running'; i++) { this.tick(dt); if (stopOn?.(this)) break; } return this.status; }
  stop(): void { const ctx = this.ctx(0); for (const a of this.active) if (a.run) this.runner.stop(a.run, ctx); this.active.length = 0; this.status = 'stopped'; }
  summary(): ExecSummary { return { time: this.time, ticks: this.ticks, status: this.status, fired: this.fired, mismatches: this.mismatches, denied: 0, failed: this.failed, running: this.active.map((a) => `${this.net.transitionId(a.t)}${a.run ? `: ${a.run.binding.kind} ${a.run.binding.value}` : ''}`), where: this.where() }; }
}
