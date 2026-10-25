/**
 * Execution layer of the Robolink API: programs started through the API (RunProgram/RunCode/RunInstruction), robot
 * moves (MoveJ/MoveL/MoveC/SearchL, blocking or not), Busy/WaitMove/WaitFinished and station Python programs.
 *
 * It drives a station runtime through the StationRuntimeLike interface (contract I1: the shared StationRuntime of
 * src/core/motion/runtime.ts when the app/server attaches one); without one it uses FallbackRuntime, a small
 * multi-program player built on ProgramSimulator (programs on different robots run concurrently, the same robot
 * queues, code calls to station Python programs wait for the script).
 *
 * Clock modes:
 *  - 'virtual'  (default, tests, in-process Pyodide without JSPI): waits advance the simulation synchronously.
 *  - 'realtime' (server, browser): a ticker (or the app's animation loop when the runtime is external) advances the
 *    simulation; blocking calls return promises that resolve when the motion/program is done.
 */
import { Station, Item, ItemType, Target } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { Program, Instruction, MoveInstruction } from '../core/items/program';
import { ProgramSimulator } from '../core/motion/simulator';
import { Mat4 } from '../core/math/pose';

export type RunStatus = 'queued' | 'running' | 'done' | 'error' | 'stopped';
export interface RunHandleLike {
  id: string | number;
  program: Item;
  robot?: Item | null;
  status: RunStatus;
  error?: { message: string; instructionIndex: number };
  done: Promise<RunHandleLike>;
}
export interface SignalTableLike { get(name: string): any; set(name: string, v: any): void; on?(name: string, cb: (v: any) => void): any; snapshot?(): Record<string, any> }
export interface ScriptContext { station: Station; runtime: StationRuntimeLike; name: string; args: string[]; robot: Item | null; program: Item | null }
export type ScriptRunner = (ctx: ScriptContext) => Promise<void>;
export interface StationRuntimeLike {
  start(program: any, opts?: { robot?: Item | null; startJoints?: number[]; args?: string[] }): RunHandleLike;
  tick(dt: number): void;
  runFor?(seconds: number): void;
  stop(id?: string | number): void;
  signals?: SignalTableLike;
  registerScriptRunner?(fn: ScriptRunner): void | (() => void);
  captureHome?(): void;
  reset?(): void;
  station?: Station;
  time?: number;
}

/** "SprayOn(1)" -> { name: 'SprayOn', args: ['1'] }; "Prog" -> { name: 'Prog', args: [] }. */
export function parseCall(code: string): { name: string; args: string[] } {
  const m = String(code).trim().match(/^([^(\s]+)\s*(?:\((.*)\))?\s*;?$/s);
  if (!m) return { name: String(code).trim(), args: [] };
  const raw = (m[2] ?? '').trim();
  const args: string[] = [];
  if (raw) {
    let depth = 0, cur = '', q = '';
    for (const ch of raw) {
      if (q) { if (ch === q) q = ''; else cur += ch; continue; }
      if (ch === '"' || ch === "'") { q = ch; continue; }
      if (ch === '(' || ch === '[') depth++;
      if (ch === ')' || ch === ']') depth--;
      if (ch === ',' && depth === 0) { args.push(cur.trim()); cur = ''; continue; }
      cur += ch;
    }
    args.push(cur.trim());
  }
  return { name: m[1], args };
}

/** Station Python program called by a code/call instruction, if any. */
export function scriptCallOf(station: Station, ins: Instruction): { item: Item; args: string[] } | null {
  const d: any = ins.data;
  let name = '', args: string[] = [];
  if (d.kind === 'code') ({ name, args } = parseCall(d.code));
  else if (d.kind === 'call' || d.kind === 'thread') { const p = d.programId ? station.findById(d.programId) : null; if (p?.type === ItemType.PROGRAM_PYTHON) return { item: p, args: [] }; name = d.programName ?? ''; }
  if (!name) return null;
  const it = station.find(name, ItemType.PROGRAM_PYTHON);
  return it ? { item: it, args } : null;
}

/** Source text of the station Python program a script context names (params.pythonCode from the RoboDK import), or null. */
export function pythonSource(ctx: { station: Station; name: string; program: Item | null }): string | null {
  const it = ctx.program?.type === ItemType.PROGRAM_PYTHON ? ctx.program : ctx.station.find(parseCall(ctx.name).name, ItemType.PROGRAM_PYTHON);
  const p: any = it?.params ?? {};
  const code = p.pythonCode ?? p.source ?? p.code;
  return typeof code === 'string' && code.length ? code : null;
}

class SimpleSignals implements SignalTableLike {
  private v = new Map<string, any>();
  private cbs = new Map<string, Array<(v: any) => void>>();
  get(name: string) { return this.v.get(name); }
  set(name: string, val: any) { this.v.set(name, val); for (const cb of this.cbs.get(name) ?? []) cb(val); }
  on(name: string, cb: (v: any) => void) { const l = this.cbs.get(name) ?? []; l.push(cb); this.cbs.set(name, l); return () => this.cbs.set(name, (this.cbs.get(name) ?? []).filter((x) => x !== cb)); }
  snapshot() { return Object.fromEntries(this.v); }
}

interface FRun {
  handle: RunHandleLike;
  resolve: (h: RunHandleLike) => void;
  sim: ProgramSimulator | null;
  t: number;
  duration: number;
  robotId: string | null;
  nextStep: number;
  waiting: Promise<void> | null;
  startJoints?: number[];
  args: string[];
}

/** Minimal StationRuntime used when no shared runtime is attached (contract I1 subset). */
export class FallbackRuntime implements StationRuntimeLike {
  time = 0;
  readonly signals = new SimpleSignals();
  private runs: FRun[] = [];
  private seq = 0;
  private scriptRunner: ScriptRunner | null = null;
  constructor(readonly station: Station) {}
  registerScriptRunner(fn: ScriptRunner): void { this.scriptRunner = fn; }
  active(): RunHandleLike[] { return this.runs.map((r) => r.handle); }

  start(program: any, opts: { robot?: Item | null; startJoints?: number[]; args?: string[] } = {}): RunHandleLike {
    const robot = opts.robot !== undefined ? opts.robot : program instanceof Program ? program.robot() : null;
    let resolve!: (h: RunHandleLike) => void;
    const handle: RunHandleLike = { id: `run${++this.seq}`, program, robot, status: 'queued', done: new Promise((r) => (resolve = r)) };
    const run: FRun = { handle, resolve, sim: null, t: 0, duration: 0, robotId: robot?.id ?? null, nextStep: 0, waiting: null, startJoints: opts.startJoints, args: opts.args ?? [] };
    this.runs.push(run);
    if (!this.runs.some((r) => r !== run && r.robotId && r.robotId === run.robotId && r.handle.status === 'running')) this.begin(run);
    return handle;
  }

  private begin(run: FRun): void {
    const program = run.handle.program;
    if (!(program instanceof Program)) {
      // a station Python program started directly
      run.handle.status = 'running';
      if (!this.scriptRunner) return this.finish(run, 'error', { message: `No Python runner registered to run ${program.name}`, instructionIndex: -1 });
      run.waiting = this.scriptRunner({ station: this.station, runtime: this, name: program.name, args: run.args, robot: null, program })
        .then(() => this.finish(run, 'done'), (e) => this.finish(run, 'error', { message: String(e?.message ?? e), instructionIndex: -1 }));
      return;
    }
    const sim = new ProgramSimulator(this.station);
    const robot = run.handle.robot;
    const start = robot && run.startJoints ? new Map([[robot.id, run.startJoints]]) : undefined;
    const res = sim.compile(program, start);
    run.sim = sim;
    run.duration = sim.duration;
    run.handle.status = 'running';
    const err = res.problems.find((p) => p.severity === 'error');
    if (err) {
      const ins = program.instructions();
      const idx = ins.findIndex((i) => i.id === err.instructionId);
      run.handle.error = { message: err.message, instructionIndex: idx };
      const step = sim.steps.find((s) => s.instruction.id === err.instructionId);
      if (step) run.duration = Math.min(run.duration, step.t0);
    }
    sim.seek(0);
    if (run.duration <= 1e-9 && !sim.steps.some((s) => scriptCallOf(this.station, s.instruction))) this.finish(run, run.handle.error ? 'error' : 'done');
  }

  private finish(run: FRun, status: RunStatus, error?: { message: string; instructionIndex: number }): void {
    if (!this.runs.includes(run)) return;
    if (error) run.handle.error = error;
    run.handle.status = status;
    this.runs = this.runs.filter((r) => r !== run);
    const robot = run.handle.robot;
    if (robot instanceof Robot) robot.state.moving = false;
    run.resolve(run.handle);
    const next = this.runs.find((r) => r.handle.status === 'queued' && (!r.robotId || r.robotId === run.robotId));
    if (next) this.begin(next);
  }

  tick(dt: number): void {
    this.time += dt;
    for (const run of [...this.runs]) {
      if (run.handle.status !== 'running' || run.waiting || !run.sim) continue;
      let nt = Math.min(run.t + dt, run.duration);
      const steps = run.sim.steps;
      while (run.nextStep < steps.length && steps[run.nextStep].t0 <= nt + 1e-12) {
        const s = steps[run.nextStep++];
        const call = scriptCallOf(this.station, s.instruction);
        if (call && this.scriptRunner) {
          run.sim.seek(s.t0); run.t = s.t0; nt = s.t0;
          const p = this.scriptRunner({ station: this.station, runtime: this, name: call.item.name, args: call.args, robot: run.handle.robot ?? null, program: run.handle.program });
          run.waiting = p.catch((e) => { console.warn(`[api] ${call.item.name}: ${e?.message ?? e}`); }).then(() => { run.waiting = null; });
          break;
        }
      }
      if (run.waiting) continue;
      run.t = nt;
      run.sim.seek(run.t);
      if (run.t >= run.duration - 1e-9 && run.nextStep >= steps.filter((s) => s.t0 <= run.duration + 1e-9).length) this.finish(run, run.handle.error ? 'error' : 'done');
    }
  }

  runFor(seconds: number): void {
    let t = 0;
    while (t < seconds - 1e-12) { const d = Math.min(0.02, seconds - t); this.tick(d); t += d; }
  }

  stop(id?: string | number): void {
    for (const run of [...this.runs]) if (id === undefined || run.handle.id === id) this.finish(run, 'stopped');
  }
}

/** API execution state attached to a Robolink (rdk.exec). */
export class ApiExec {
  runtime: StationRuntimeLike;
  /** true when the runtime is owned and ticked by the app (browser animation loop). */
  external = false;
  mode: 'virtual' | 'realtime' = 'virtual';
  speed = 1;
  private handles: RunHandleLike[] = [];
  private timer: any = null;
  private last = 0;
  private ticking = 0;
  private scriptRunner: ScriptRunner | null = null;
  /** >0 while a synchronous (in-process) transport call runs: waits must complete synchronously. */
  syncDepth = 0;
  /** true once a shared runtime was attached (attach()). */
  attachedRuntime = false;
  /** Called after every simulation step this layer performs (spray deposition, ...). */
  onTick: ((dt: number) => void) | null = null;

  constructor(private readonly owner: { station: Station }) { this.runtime = new FallbackRuntime(owner.station); }

  /** Attach the shared station runtime (I1). external=true: the app ticks it; mode realtime makes waits asynchronous. */
  attach(runtime: StationRuntimeLike, opts: { external?: boolean; mode?: 'virtual' | 'realtime' } = {}): void {
    this.unregister?.();
    this.unregister = null;
    this.runtime = runtime;
    this.attachedRuntime = true;
    this.external = !!opts.external;
    if (opts.mode) this.mode = opts.mode;
    this.registerOn(runtime);
  }
  /** Replace the station Python runner (one registration on the runtime at a time). */
  setScriptRunner(fn: ScriptRunner | null): void {
    this.unregister?.();
    this.unregister = null;
    this.scriptRunner = fn;
    this.registerOn(this.rt());
  }
  /** The current station Python runner. */
  getScriptRunner(): ScriptRunner | null { return this.scriptRunner; }
  private unregister: (() => void) | null = null;
  private registerOn(rt: StationRuntimeLike): void {
    if (!this.scriptRunner) return;
    const off = rt.registerScriptRunner?.(this.scriptRunner);
    this.unregister = typeof off === 'function' ? off : null;
  }
  hasScriptRunner(): boolean { return !!this.scriptRunner; }

  /** The runtime for the current station (a station swap recreates the fallback runtime). */
  rt(): StationRuntimeLike {
    if (this.runtime instanceof FallbackRuntime && this.runtime.station !== this.owner.station) {
      this.runtime.stop();
      this.runtime = new FallbackRuntime(this.owner.station);
      this.registerOn(this.runtime);
      this.handles = [];
    }
    return this.runtime;
  }
  private get virtual(): boolean { return this.mode === 'virtual' || this.syncDepth > 0; }

  start(program: Item, opts: { robot?: Item | null; startJoints?: number[]; args?: string[] } = {}, rt: StationRuntimeLike = this.rt()): RunHandleLike {
    const h = rt.start(program, opts);
    this.handles.push(h);
    h.done.then(() => { this.handles = this.handles.filter((x) => x !== h); });
    this.ensureTicker();
    return h;
  }

  /** Busy = a run that is queued/running on this robot or for this program (or a program of this robot). */
  busy(item: Item | null): boolean {
    if (item instanceof Robot) return this.robotRuns(item).length > 0;
    const rt: any = this.runtime;
    // runs started elsewhere on the shared runtime (the app's Play, threads started by a program) count too
    if (this.attachedRuntime && typeof rt.busy === 'function' && rt.busy(item ?? undefined)) return true;
    if (!item) return this.handles.some((h) => h.status === 'running' || h.status === 'queued');
    return this.handles.some((h) => (h.status === 'running' || h.status === 'queued') && (h.program === item || h.robot === item || (h.program instanceof Program && h.program.robot() === item)));
  }
  /** Something is still running on the runtime (own handles, threads, script runs): the ticker keeps going. */
  private anyActive(): boolean {
    const rt: any = this.runtime;
    if (this.handles.some((h) => h.status === 'running' || h.status === 'queued')) return true;
    return this.attachedRuntime && typeof rt.active === 'function' ? rt.active().length > 0 : false;
  }
  handlesOf(item: Item | null): RunHandleLike[] { return this.handles.filter((h) => !item || h.program === item || h.robot === item); }

  /** Wait until `item` is idle (or the given handles are done). Synchronous in virtual mode, a promise in realtime mode. */
  wait(item: Item | null, timeoutS = 3600, handles?: RunHandleLike[]): void | Promise<void> {
    const pending = () => (handles ?? (item instanceof Robot ? this.robotRuns(item) : this.handles.filter((h) => !item || h.program === item || h.robot === item || (h.program instanceof Program && h.program.robot() === item)))).filter((h) => h.status === 'running' || h.status === 'queued');
    if (!pending().length) return;
    if (this.ticking) return;
    if (this.virtual) {
      let t = 0;
      while (pending().length && t < timeoutS) {
        const before = pending().map((h) => h.status).join();
        this.step(0.02); t += 0.02;
        // a run waiting for an asynchronous script cannot progress synchronously
        if (pending().length && t > 5 && pending().every((h) => h.status === 'running') && before === pending().map((h) => h.status).join() && this.stalled()) break;
      }
      return;
    }
    const all = Promise.all(pending().map((h) => h.done)).then(() => undefined);
    if (!(timeoutS > 0) || timeoutS >= 3600 * 24) return all;
    return Promise.race([all, new Promise<void>((_, rej) => setTimeout(() => rej(new Error(`Timeout waiting for ${item?.name ?? 'programs'}`)), timeoutS * 1000))]);
  }
  private stalled(): boolean { return this.runtime instanceof FallbackRuntime && (this.runtime as any).runs.some((r: FRun) => r.waiting); }

  /** Advance the simulation by `seconds` (virtual mode helper; RoboDK scripts call pause()/time.sleep). */
  advance(seconds: number): void {
    let t = 0;
    while (t < seconds - 1e-12) { const d = Math.min(0.02, seconds - t); this.step(d); t += d; }
  }
  /** One simulation step of dt seconds (runtime + per-step hooks). */
  step(dt: number): void {
    this.ticking++;
    try { this.rt().tick(dt); if (this.side) this.side.tick(dt); this.onTick?.(dt); } finally { this.ticking--; }
  }

  stop(item?: Item | null): void {
    const rt = this.rt();
    for (const h of [...this.handles]) if (!item || h.program === item || h.robot === item) rt.stop(h.id);
  }

  /**
   * Move a robot through the runtime: a one-instruction temporary program (not shown in the tree) so speed, tool,
   * frame, path checks and timing match program playback. Throws TargetReachError when the target is unreachable.
   */
  moveRobot(robot: Robot, build: (p: Program) => void, blocking: boolean): void | Promise<void> {
    const prog = new Program(`__api_move_${robot.name}`);
    prog.parent = this.owner.station;          // resolves station lookups without adding the program to the tree
    prog.setRobot(robot);
    // the robot's current API state (setPoseFrame/setPoseTool/setSpeed/setRounding) applies to this move: the shared
    // runtime keeps per-robot state from earlier programs, so state it explicitly
    const st = this.owner.station;
    prog.setFrame(robot.activeFrameId ? st.findById(robot.activeFrameId) : null);
    prog.setTool(robot.activeToolId ? st.findById(robot.activeToolId) : null);
    prog.setSpeed(robot.motion.speedLinear, robot.motion.speedJoints, robot.motion.accelLinear, robot.motion.accelJoints);
    prog.setRounding(robot.motion.rounding);
    build(prog);
    // validate first (RoboDK raises before moving)
    const check = new ProgramSimulator(this.owner.station).compile(prog);
    const err = check.problems.find((p) => p.severity === 'error');
    if (err) throw new Error(`TargetReachError: ${err.message}`);
    if (this.ticking) {
      // called from inside a tick (a script run by the runtime): apply instantly instead of nesting the clock
      const sim = new ProgramSimulator(this.owner.station); sim.compile(prog); sim.runToEnd(); robot.state.moving = false;
      return;
    }
    // a robot finishes its previous motion first (RoboDK _moveX calls WaitMove)
    const prev = this.handles.filter((h) => h.robot === robot && h.program.name.startsWith('__api_move_') && (h.status === 'running' || h.status === 'queued'));
    // a program of this robot waiting for a station script (the script is the caller: PartsToPallet moving robot B
    // while MainSimulation on robot B waits in 'Call PartsToPallet') does not hold the robot, like in RoboDK
    const h = this.blockedByScript(robot) ? this.start(prog, { robot }, this.sideRuntime()) : this.start(prog, { robot });
    if (!blocking) return;
    return this.wait(robot, 3600 * 24 * 7, [...prev, h]);
  }

  /**
   * Runs that hold a robot (RoboDK robot Busy/WaitMove): API moves of the robot and programs moving it, on this layer
   * and on the shared runtime; a program blocked in a station-script call does not hold its robot.
   */
  robotRuns(robot: Robot): RunHandleLike[] {
    const act = (h: RunHandleLike) => h.status === 'running' || h.status === 'queued';
    const own = this.handles.filter((h) => act(h) && (h.robot === robot || (h.program instanceof Program && h.program.robot() === robot)));
    const rt: any = this.runtime;
    const shared: RunHandleLike[] = this.attachedRuntime && typeof rt.active === 'function' ? rt.active().filter((h: RunHandleLike) => act(h) && h.robot === robot) : [];
    return [...new Set([...own, ...shared])].filter((h) => !this.inScript(h));
  }
  private inScript(h: RunHandleLike): boolean {
    const ctxs: any[] | undefined = (this.runtime as any).ctxs;
    return Array.isArray(ctxs) && ctxs.some((c) => c.handle === h && c.block?.kind === 'script');
  }
  /** Second runtime for API moves of a robot whose program is blocked in a script call (ticked with the main one). */
  private side: StationRuntimeLike | null = null;
  private sideRuntime(): StationRuntimeLike {
    if (!this.side || this.side.station !== this.owner.station) this.side = new FallbackRuntime(this.owner.station);
    return this.side;
  }
  /** The robot's runs on the shared runtime are all waiting for a station script (StationRuntime internals, feature-detected). */
  private blockedByScript(robot: Robot): boolean {
    const ctxs: any[] | undefined = (this.runtime as any).ctxs;
    if (!this.attachedRuntime || !Array.isArray(ctxs)) return false;
    const mine = ctxs.filter((c) => c.robot === robot && c.handle?.status === 'running');
    return mine.length > 0 && mine.every((c) => c.block?.kind === 'script');
  }

  /** Run a station Python program (PROGRAM_PYTHON item) with arguments through the registered runner. */
  runScript(item: Item, args: string[] = [], robot: Item | null = null): Promise<void> {
    if (!this.scriptRunner) return Promise.reject(new Error(`No Python runner available to run ${item.name} (start the studio server, or allow Pyodide in the browser)`));
    return this.scriptRunner({ station: this.owner.station, runtime: this.rt(), name: item.name, args, robot, program: item });
  }

  private ensureTicker(): void {
    if (this.mode !== 'realtime' || this.external || this.timer) return;
    this.last = Date.now();
    this.timer = setInterval(() => {
      const now = Date.now();
      const dt = Math.min(0.2, (now - this.last) / 1000) * this.speed;
      this.last = now;
      this.step(dt);
      if (!this.anyActive()) { clearInterval(this.timer); this.timer = null; }
    }, 16);
    (this.timer as any)?.unref?.();
  }
  dispose(): void { if (this.timer) clearInterval(this.timer); this.timer = null; this.rt().stop(); }
}

/** Move instruction data for a robot move to a target item, joints or a pose (in the robot's active frame). */
export function moveTo(kind: MoveInstruction['moveType'], target: Target | number[] | Mat4, via?: Target | number[] | Mat4): MoveInstruction {
  const put = (t: Target | number[] | Mat4, d: MoveInstruction, via: boolean) => {
    if (t instanceof Target) { if (via) d.viaTargetId = t.id; else d.targetId = t.id; }
    else if (t instanceof Float64Array) { if (via) d.viaPose = Array.from(t); else d.pose = Array.from(t); }
    else if (!via) d.joints = [...t];
    else d.viaPose = undefined;
  };
  const d: MoveInstruction = { kind: 'move', moveType: kind, targetId: null };
  put(target, d, false);
  if (via) put(via, d, true);
  return d;
}
