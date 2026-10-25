/**
 * Station runtime (contract I1): one shared simulation clock per station, RoboDK-like program execution.
 *
 * - Programs on different robots run concurrently (RoboDK threads / "Run" instructions); a program for a busy robot is
 *   queued until the robot is free.
 * - Instructions are interpreted at run time: waitDI / wait signal / IF read the SignalTable when they execute, threads
 *   start new runs, program calls inherit the robot state (speed, rounding, tool, frame persist per robot like RoboDK),
 *   code calls and calls naming a station Python program go through the registered script runners (I5).
 * - Consecutive moves are planned as one blended chain (planner.ts); a run plays up to its first error and stops there
 *   (status 'error' with the instruction index); other runs continue.
 * - captureHome() / reset() snapshot and restore object poses, parents (attachments), visibility, robot joints,
 *   tools/frames and signals.
 *
 * The program simulator (simulator.ts) uses the same interpreter in 'record' mode (event driven, continue after
 * errors, no script execution) to compile programs for validation, previews and cycle times.
 */
import { Station, Item, ItemType, Target, Tool, SceneObject } from '../items/item';
import { Robot } from '../items/robot';
import { Program, Instruction, InstructionData, MoveInstruction } from '../items/program';
import { EventBus } from '../events';
import { Mat4, identity, multiply, invert, fromArray } from '../math/pose';
import { motionGroupFor, planChain, ChainMoveSpec, MotionSettings, MotionGroup, PlannedTrajectory, updateModelOf, updateMoveTime } from './planner';
import type { Trajectory, TrajectorySample } from './trajectory';
import { getMobileMotionPlanner, Pose2D } from './mobile_hook';

// ---- signals ---------------------------------------------------------------------------------------------------------

export type SignalValue = number | boolean | string;

const toNum = (v: unknown): number => {
  if (typeof v === 'boolean') return v ? 1 : 0;
  if (typeof v === 'number') return v;
  if (typeof v === 'string') { const s = v.trim().toLowerCase(); if (s === 'true' || s === 'on') return 1; if (s === 'false' || s === 'off') return 0; const n = Number(s); return s !== '' && Number.isFinite(n) ? n : NaN; }
  return NaN;
};
/** Loose signal equality: true == 1 == '1', numbers by value, other strings exactly. */
export function signalEquals(a: unknown, b: unknown): boolean {
  if (a === undefined || a === null) return toNum(b) === 0 || b === false;
  const x = toNum(a), y = toNum(b);
  if (Number.isFinite(x) && Number.isFinite(y)) return Math.abs(x - y) < 1e-9;
  return String(a) === String(b);
}

/** The single signal table: robot DI/DO, program signals, process-component signals and control-model events. */
export class SignalTable {
  private values = new Map<string, SignalValue>();
  private times = new Map<string, number>();
  private listeners = new Map<string, Set<(v: SignalValue, name: string) => void>>();
  /** Time stamp source for set() (the runtime points it at the executing program's clock). */
  clock: () => number = () => 0;
  get(name: string | number): SignalValue | undefined { return this.values.get(String(name)); }
  has(name: string | number): boolean { return this.values.has(String(name)); }
  /** Time (s) at which the signal was last set. */
  timeOf(name: string | number): number | undefined { return this.times.get(String(name)); }
  set(name: string | number, v: SignalValue): void {
    const key = String(name);
    this.values.set(key, v);
    this.times.set(key, this.clock());
    for (const k of [key, '*']) for (const cb of [...(this.listeners.get(k) ?? [])]) { try { cb(v, key); } catch (e) { console.error('[signals]', e); } }
  }
  /** Subscribe to one signal ('*' = every signal). Returns an unsubscribe function. */
  on(name: string, cb: (v: SignalValue, name: string) => void): () => void {
    let s = this.listeners.get(name);
    if (!s) this.listeners.set(name, (s = new Set()));
    s.add(cb);
    return () => s!.delete(cb);
  }
  snapshot(): Record<string, SignalValue> { return Object.fromEntries(this.values); }
  /** Replace all values (listeners are told about the ones that change). */
  restore(snap: Record<string, SignalValue>): void {
    const old = this.values;
    this.values = new Map(Object.entries(snap));
    this.times = new Map([...this.values.keys()].map((k) => [k, this.clock()]));
    for (const k of new Set([...old.keys(), ...this.values.keys()])) {
      if (old.get(k) === this.values.get(k)) continue;
      const v = this.values.get(k) ?? 0;
      for (const kk of [k, '*']) for (const cb of [...(this.listeners.get(kk) ?? [])]) { try { cb(v, k); } catch (e) { console.error('[signals]', e); } }
    }
  }
}

/**
 * Evaluate a condition such as `DI[3] == 1 && !Busy`, `count > 2 or Part`, `S == "go"` against the signal table.
 * Unknown signals read as 0.
 */
export function evalCondition(expr: string, get: (name: string) => unknown): boolean {
  const toks = String(expr).match(/\s*(\d+\.?\d*(?:e[+-]?\d+)?|"[^"]*"|'[^']*'|==|!=|<=|>=|&&|\|\||[A-Za-z_][\w.:]*|[!<>=()[\]+\-*/])/gi)?.map((t) => t.trim()) ?? [];
  let i = 0;
  const peek = () => toks[i], next = () => toks[i++];
  const lookup = (name: string): unknown => { const v = get(name); return v === undefined ? 0 : v; };
  const primary = (): unknown => {
    const t = next();
    if (t === undefined) return 0;
    if (t === '(') { const v = or(); if (peek() === ')') i++; return v; }
    if (t === '!' || /^not$/i.test(t)) return !truthy(primary());
    if (t === '-') return -toNum(primary());
    if (/^["']/.test(t)) return t.slice(1, -1);
    if (/^\d/.test(t)) return Number(t);
    if (/^true$/i.test(t)) return true;
    if (/^false$/i.test(t)) return false;
    if (peek() === '[') { i++; const idx = String(sum()); if (peek() === ']') i++; const k1 = `${t}[${idx}]`; const v = get(k1); return v !== undefined ? v : lookup(`${t}${idx}`); }
    return lookup(t);
  };
  const truthy = (v: unknown) => (typeof v === 'string' ? (Number.isFinite(toNum(v)) ? toNum(v) !== 0 : v.length > 0) : !!toNum(v));
  const term = (): unknown => { let v = primary(); while (peek() === '*' || peek() === '/') { const op = next(); const r = toNum(primary()); v = op === '*' ? toNum(v) * r : toNum(v) / r; } return v; };
  const sum = (): unknown => { let v = term(); while (peek() === '+' || peek() === '-') { const op = next(); const r = toNum(term()); v = op === '+' ? toNum(v) + r : toNum(v) - r; } return v; };
  const cmp = (): unknown => {
    const a = sum();
    const op = peek();
    if (op === '==' || op === '=' || op === '!=' || op === '<' || op === '>' || op === '<=' || op === '>=') {
      i++;
      const b = sum();
      if (op === '==' || op === '=') return signalEquals(a, b);
      if (op === '!=') return !signalEquals(a, b);
      const x = toNum(a), y = toNum(b);
      return op === '<' ? x < y : op === '>' ? x > y : op === '<=' ? x <= y : x >= y;
    }
    return a;
  };
  // without a logical operator the raw value passes through (so "(n - 1) * 2" keeps its number)
  const and = (): unknown => { let v = cmp(); while (peek() === '&&' || /^and$/i.test(peek() ?? '')) { i++; const r = truthy(cmp()); v = truthy(v) && r; } return v; };
  const or = (): unknown => { let v = and(); while (peek() === '||' || /^or$/i.test(peek() ?? '')) { i++; const r = truthy(and()); v = truthy(v) || r; } return v; };
  return truthy(or());
}

/** "SprayOn(1)" -> { name: 'SprayOn', args: ['1'] }; "Prog" -> { name: 'Prog', args: [] }. */
export function parseCallText(code: string): { name: string; args: string[] } {
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

// ---- station snapshot ----------------------------------------------------------------------------------------------

/** Restorable simulation state of a station: object poses/parents/visibility, robot joints/tool/frame, tool states. */
export class StationSnapshot {
  private objects = new Map<Item, { parent: Item | null; pose: Mat4 }>();
  private visible = new Map<Item, boolean>();
  private tools = new Map<Tool, { closed: boolean; attached: string[] }>();
  private robots = new Map<Robot, { joints: number[]; frame: string | null; tool: string | null; io: Record<string, number | boolean> }>();
  private mobiles = new Map<Item, Mat4>();
  constructor(readonly station: Station, startJoints?: Map<string, number[]>) {
    for (const it of station.walk()) {
      if (it === station) continue;
      this.visible.set(it, it.visible);
      if (it instanceof Tool) this.tools.set(it, { closed: it.closed, attached: [...it.attached] });
      else if (it instanceof SceneObject) this.objects.set(it, { parent: it.parent, pose: it.pose() });
      if (it instanceof Robot) this.robots.set(it, { joints: startJoints?.get(it.id) ?? it.joints(), frame: it.activeFrameId, tool: it.activeToolId, io: { ...(it.state.io ?? {}) } });
      if (it.type === ItemType.MOBILE_ROBOT) this.mobiles.set(it, it.pose());
    }
  }
  /** Restore objects only (RoboDK "Replace objects"). */
  restoreObjects(): void {
    for (const [obj, s] of this.objects) {
      if (!s.parent) continue;
      if (obj.parent !== s.parent) { if (obj.parent instanceof Tool) obj.parent.attached = obj.parent.attached.filter((x) => x !== obj.id); s.parent.addChild(obj); }
      obj.setPose(s.pose);
    }
    for (const [obj] of this.objects) { const v = this.visible.get(obj); if (v !== undefined && obj.visible !== v) obj.setVisible(v); }
  }
  restore(): void {
    this.restoreObjects();
    for (const [tool, s] of this.tools) { tool.closed = s.closed; tool.attached = [...s.attached]; }
    for (const [r, s] of this.robots) { r.setJoints(s.joints); r.activeFrameId = s.frame; r.activeToolId = s.tool; r.state.moving = false; if (r.state.io) r.state.io = { ...s.io }; }
    for (const [m, p] of this.mobiles) m.setPose(p);
    for (const [it, v] of this.visible) if (it.visible !== v) it.setVisible(v);
  }
  jointsOf(r: Robot): number[] | undefined { return this.robots.get(r)?.joints; }
}

// ---- runtime types ---------------------------------------------------------------------------------------------------

export type RunStatus = 'queued' | 'running' | 'done' | 'error' | 'stopped';
export interface RunError { message: string; instructionIndex: number; instructionId?: string; program?: string }
export interface RunHandle {
  readonly id: string;
  readonly program: Program | Item;
  readonly robot: Robot | Item | null;
  status: RunStatus;
  error?: RunError;
  done: Promise<RunHandle>;
  /** Simulation time at which the run started / ended. */
  startTime?: number;
  endTime?: number;
  /** Index (in program.instructions()) of the top-level instruction being executed. */
  instructionIndex?: number;
  /** Run that started this one with a thread instruction. */
  parentId?: string;
  /** RoboDK Item.Update() cycle-time estimate of this run (moves rest to rest at the robot motion state, pauses; see planner UpdateModel). */
  estimate?: number;
}
export interface StartOptions {
  robot?: Robot | Item | null;
  startJoints?: number[];
  args?: string[];
  /** Start at this top-level instruction ("Run from here"); the speed / rounding / tool / frame set before it apply. */
  fromIndex?: number;
}
export interface ScriptContext {
  station: Station; runtime: StationRuntime; name: string; args: string[]; robot: Robot | Item | null; program: Program | Item | null;
  /** Set when the run that launched the script stops (Stop / Reset / error) while the script still runs: runners end it. */
  abort?: { aborted: boolean };
  /**
   * Simulated seconds the script took, when the runner knows it (an in-page script that never yielded: 0). The calling
   * run resumes at launch time + elapsed instead of at the clock time when the runner's promise settled (that clock
   * already moved on by the frame in which the asynchronous answer arrived).
   */
  elapsed?: number;
}
/** A script runner resolves `false` when it does not handle the call: the next registered runner is tried. */
export type ScriptRunner = ((ctx: ScriptContext) => Promise<void | boolean>) & {
  /** true while the runner cannot run scripts yet (an interpreter downloading): the live clock holds meanwhile. */
  loading?: () => boolean;
  /** The runner executes calls of this name in live playback (validation does not report them as ignored). */
  handles?: (name: string) => boolean;
};
export interface Problem { instructionId: string; message: string; severity: 'error' | 'warning'; runId?: string; /** Simulation time at which it happened. */ t?: number }

export interface RuntimeEvents extends Record<string, unknown> {
  run: { handle: RunHandle };
  instruction: { handle: RunHandle; program: Program; instruction: Instruction; index: number };
  message: { text: string; level: 'info' | 'warn' | 'error'; handle?: RunHandle };
  io: { name: string; value: SignalValue };
  tick: { t: number };
}

/** A recorded step (record mode): what the program simulator replays. */
export interface RecordedStep {
  instruction: Instruction;
  robot: Robot | null;
  trajectory?: Trajectory;
  group?: MotionGroup;
  mobile?: { item: Item; sample: (t: number) => Pose2D; duration: number };
  t0: number;
  t1: number;
  apply?: () => void;
  runId: string;
  /** Item.Update() time of this move (RoboDK cycle-time model, see planner UpdateModel). */
  estimate?: number;
  /** Joints at the move's target (blended moves pass beside it). */
  qTarget?: number[];
}

/** Hooks the program simulator uses to mirror state changes it replays. */
export interface RuntimeSink {
  io?(name: string, value: SignalValue): void;
  signal?(name: string, value: SignalValue): void;
  message?(text: string, level: 'info' | 'warn' | 'error'): void;
  attach?(objectId: string, tool: Tool): void;
  detach?(objectId: string): void;
}

interface RobotRunState {
  settings: MotionSettings; toolId: string | null; frameId: string | null;
  /** The frame was set by the program (Set Ref. instruction or program frame): Cartesian targets are taken relative to it. */
  frameSet?: boolean;
}
/** Placement restored by a RoboDK "Replace objects" event: object id, parent frame id and pose relative to it (16 numbers, column-major). */
export interface ObjectPlacement { id: string; parentId?: string | null; pose?: number[]; visible?: boolean }
interface Frame {
  program: Program; list: Instruction[]; pc: number; robot: Robot | null; mover: Item | null; repeat: number;
  /** Called program of another robot than its caller's: its first move starts, for Item.Update(), from that robot's joints at the start (RoboDK does not carry other robots' simulated state from call to call). */
  foreign?: boolean;
  /** The program failed: its remaining instructions do not count for Item.Update(). */
  noEstimate?: boolean;
}
interface MotionSeg { t0: number; t1: number; traj?: Trajectory; group?: MotionGroup; mobile?: { item: Item; sample: (t: number) => Pose2D }; ins?: Instruction; index?: number }
type Block =
  | { kind: 'motion'; t1: number; segs: MotionSeg[]; events: Array<{ t: number; fn: () => void }>; ei: number; cur?: number; fail?: { ins: Instruction; message: string } }
  | { kind: 'until'; t: number; step?: RecordedStep }
  | { kind: 'cond'; test: () => boolean; names: string[]; deadline: number; label: string; ins: Instruction; step?: RecordedStep }
  | { kind: 'script'; state: { done: boolean; error?: unknown; at?: number }; ins: Instruction | null; step?: RecordedStep }
  | { kind: 'user'; step?: RecordedStep };
interface Ctx {
  handle: RunHandle; frames: Frame[]; cursor: number; block: Block | null; robot: Robot | null; resolve: (h: RunHandle) => void; parent?: Ctx; children: Ctx[];
  /** Robots this run drives (its own moves and those of the programs it calls; threads are runs of their own). */
  holds: Set<Robot>;
  /** Item.Update() estimate accumulated by this run (calls included, threads are runs of their own). */
  estimate: number;
}

const PASSTHROUGH = new Set(['speed', 'rounding', 'print', 'io', 'signal']);
const CHAIN_MAX = 400;
const globalRunners: ScriptRunner[] = [];

/** Register a script runner for every runtime (Pyodide in the browser, CPython on the server). Returns an unregister function. */
export function registerScriptRunner(fn: ScriptRunner): () => void {
  globalRunners.unshift(fn);
  return () => { const i = globalRunners.indexOf(fn); if (i >= 0) globalRunners.splice(i, 1); };
}
/** A registered runner is still loading its interpreter (Pyodide download / start-up). */
export function scriptRunnersLoading(extra: ScriptRunner[] = []): boolean {
  return [...extra, ...globalRunners].some((r) => { try { return !!r.loading?.(); } catch { return false; } });
}

const runtimes = new WeakMap<Station, StationRuntime>();
let runCounter = 0;

/** Main robot of a program: a carrier (rail/gantry) bound program drives the arm it carries. */
export function programRobot(p: Program | Item, override?: Robot | Item | null): Robot | Item | null {
  const r = override !== undefined && override !== null ? override : p instanceof Program ? p.robot() : null;
  if (r instanceof Robot && r.dof < 5) {
    for (const it of r.walk()) if (it !== r && it instanceof Robot && it.dof >= 5 && it.dof > r.dof) return it;
  }
  return r ?? null;
}
const mobileOf = (it: Item | null): Item | null => { for (let p = it; p; p = p.parent) if (p.type === ItemType.MOBILE_ROBOT) return p; return null; };

export class StationRuntime {
  readonly signals = new SignalTable();
  readonly events = new EventBus<RuntimeEvents>();
  /** Simulation time (s). */
  time = 0;
  /** Every run started on this runtime (most recent last). */
  readonly runs: RunHandle[] = [];
  /** Problems reported by runs (errors stop their run, warnings do not). */
  readonly problems: Problem[] = [];
  /** 'live' (ticked, scripts run, waits block) or 'record' (event driven compile for the program simulator). */
  readonly mode: 'live' | 'record';
  /** Keep executing after an error (validation). Live runs stop at their first error. */
  continueOnError: boolean;
  /** Optional asset store (mesh bounds for attach distance). */
  assets: { get(id: string): { mesh?: { min: number[]; max: number[] } } | undefined } | null = null;
  sink: RuntimeSink = {};
  /**
   * Do not apply Set speed instructions (robot default speeds only). RoboDK's Item.Update() cycle-time estimate behaves
   * like this (31 of 32 Library programs with speed changes match it within 1 %), while its playback and
   * InstructionListJoints honour the programmed speeds (the default here).
   */
  ignoreSpeedInstructions = false;
  /** Record mode: every executed instruction. */
  readonly recorded: RecordedStep[] = [];
  /** Record mode: TCP distance and executed instruction count. */
  distance = 0;
  executed = 0;
  private ctxs: Ctx[] = [];
  private queue: Ctx[] = [];
  private runners: ScriptRunner[] = [];
  private rstates = new Map<Robot, RobotRunState>();
  private home: { snap: StationSnapshot; signals: Record<string, SignalValue>; rstates: Map<Robot, RobotRunState>; time: number } | null = null;
  private homeParents = new Map<string, Item>();
  private warned = new Set<string>();
  private nowCursor = 0;

  constructor(readonly station: Station, opts: { mode?: 'live' | 'record'; continueOnError?: boolean } = {}) {
    this.mode = opts.mode ?? 'live';
    this.continueOnError = opts.continueOnError ?? this.mode === 'record';
    this.signals.clock = () => this.nowCursor;
    if (this.mode === 'live' && !runtimes.has(station)) runtimes.set(station, this);
  }

  /** The shared runtime of a station (created on first use). */
  static of(station: Station): StationRuntime {
    return runtimes.get(station) ?? new StationRuntime(station);
  }

  /** Register a script runner for this runtime (takes precedence over the global ones). */
  registerScriptRunner(fn: ScriptRunner): () => void {
    this.runners.unshift(fn);
    return () => { const i = this.runners.indexOf(fn); if (i >= 0) this.runners.splice(i, 1); };
  }

  // -- state -------------------------------------------------------------------------------------------------------

  /** Snapshot the current station state as the home state restored by reset(). */
  captureHome(startJoints?: Map<string, number[]>): void {
    this.home = { snap: new StationSnapshot(this.station, startJoints), signals: this.signals.snapshot(), rstates: new Map([...this.rstates].map(([r, s]) => [r, { ...s, settings: { ...s.settings } }])), time: this.time };
  }
  /** Stop everything and restore the home state (object poses, attachments, joints, tools/frames, signals, robot state). */
  reset(): void {
    this.stop();
    if (!this.home) return;
    this.home.snap.restore();
    this.signals.restore(this.home.signals);
    this.rstates = new Map([...this.home.rstates].map(([r, s]) => [r, { ...s, settings: { ...s.settings } }]));
    this.homeParents.clear();
  }
  get hasHome(): boolean { return !!this.home; }
  /** Forget where attached objects came from (after an external state restore). */
  resetAttachState(): void { this.homeParents.clear(); }

  /** Robot state that persists across program calls and runs (RoboDK: speed, rounding, tool and frame belong to the robot). */
  robotState(robot: Robot): RobotRunState {
    let s = this.rstates.get(robot);
    // a robot imported from RoboDK keeps RoboDK's rule from the start: Cartesian targets are relative to the active
    // reference frame even before a Set Ref. (01.c DrillingPart run alone is invalid in RoboDK: robot-base frame)
    if (!s) this.rstates.set(robot, (s = { settings: { ...robot.motion }, toolId: robot.activeToolId, frameId: robot.activeFrameId, frameSet: robot.params.rdkClass !== undefined && !!robot.activeFrameId }));
    return s;
  }

  // -- runs --------------------------------------------------------------------------------------------------------

  /** Start a program (or a station Python program). Programs of a busy robot are queued. */
  start(program: Program | Item, opts: StartOptions = {}): RunHandle {
    if (!this.home && this.mode === 'live') this.captureHome();
    this.nameIndex = null;
    const robot = programRobot(program, opts.robot);
    let resolve!: (h: RunHandle) => void;
    const done = new Promise<RunHandle>((r) => (resolve = r));
    const handle: RunHandle = { id: `run${++runCounter}`, program, robot, status: 'queued', done };
    const arm = robot instanceof Robot ? robot : null;
    // a run holds the robots it moves, not the robot its program is linked to: RoboDK 06.e StartSimulation (UR10 A)
    // only calls TakeConveyor (UR10 B) while its thread FeedConveyor drives UR10 A
    const c: Ctx = { handle, frames: [], cursor: this.time, block: null, robot: arm, resolve, children: [], holds: program instanceof Program ? this.movedRobots(program, arm) : new Set(), estimate: 0 };
    if (program instanceof Program) {
      const fr = this.frameFor(program, arm);
      if (opts.fromIndex && opts.fromIndex > 0) this.skipTo(fr, Math.min(opts.fromIndex, fr.list.length));
      c.frames.push(fr);
    }
    else if (program.type === ItemType.PROGRAM_PYTHON) {
      const st = { done: false as boolean, error: undefined as unknown, at: undefined as number | undefined };
      c.block = { kind: 'script', state: st, ins: null };
      (c as any).pendingScript = { name: program.name, args: opts.args ?? [], state: st };
    }
    if (opts.startJoints && arm) (c as any).startJoints = opts.startJoints;
    this.runs.push(handle);
    if (this.holdsBusy(c)) this.queue.push(c);
    else this.begin(c);
    return handle;
  }

  /** Robots moved by `program` run on `robot` (own moves, calls, loops, branches; threads excluded). */
  private movedRobots(program: Program, robot: Robot | null, out = new Set<Robot>(), seen = new Set<Program>()): Set<Robot> {
    if (seen.has(program) || seen.size > 256) return out;
    seen.add(program);
    const r = programRobot(program) ?? robot;
    const arm = r instanceof Robot ? r : robot;
    for (const ins of program.instructions()) {
      const d = ins.data;
      if ((d.kind === 'move' || d.kind === 'jointPath') && arm) out.add(arm);
      let sub: Item | null = null;
      if (d.kind === 'call') sub = (d.programId ? this.station.findById(d.programId) : null) ?? (d.programName ? this.findProgram(parseCallText(d.programName).name) : null);
      else if (d.kind === 'code' && d.asFunctionCall) sub = this.findProgram(parseCallText(d.code).name);
      else if (d.kind === 'loop') sub = d.bodyProgramId ? this.station.findById(d.bodyProgramId) : null;
      else if (d.kind === 'if') { for (const id of [d.thenProgramId, d.elseProgramId]) { const b = id ? this.station.findById(id) : null; if (b instanceof Program) this.movedRobots(b, arm, out, seen); } }
      if (sub instanceof Program) this.movedRobots(sub, arm, out, seen);
    }
    return out;
  }
  private holdsBusy(c: Ctx): boolean {
    for (const r of c.holds) if (this.robotBusy(r)) return true;
    return false;
  }

  /** "Run from here": robot state instructions before `index` take effect (no motion, no IO). */
  private skipTo(fr: Frame, index: number): void {
    const robot = fr.robot;
    for (let j = 0; j < index; j++) {
      const x = fr.list[j];
      if (!x.enabled || !robot) continue;
      const d = x.data, st = this.robotState(robot);
      if (d.kind === 'speed' && !this.ignoreSpeedInstructions) this.applySpeed(st.settings, d);
      else if (d.kind === 'rounding') st.settings.rounding = d.radius;
      else if (d.kind === 'tool') { st.toolId = d.toolId; const t = d.toolId ? this.station.findById(d.toolId) : null; robot.setTool(t instanceof Tool ? t : null); }
      else if (d.kind === 'frame') { st.frameId = d.frameId; st.frameSet = true; robot.setFrame(d.frameId ? this.station.findById(d.frameId) : null); }
    }
    fr.pc = index;
  }

  private frameFor(program: Program, robot: Robot | null): Frame {
    const r = programRobot(program) ?? robot;
    const arm = r instanceof Robot ? r : robot;
    if (arm && program instanceof Program) {
      const st = this.robotState(arm);
      if (program.toolId) st.toolId = program.toolId;
      if (program.frameId) { st.frameId = program.frameId; st.frameSet = true; }
    }
    return { program, list: program.instructions(), pc: 0, robot: arm, mover: mobileOf(r instanceof Item ? r : null), repeat: 0 };
  }

  private robotBusy(r: Robot): boolean {
    return this.ctxs.some((c) => c.holds.has(r) && c.handle.status === 'running');
  }

  private begin(c: Ctx): void {
    c.handle.status = 'running';
    c.cursor = Math.max(c.cursor, this.time);
    c.handle.startTime = c.cursor;
    const sj = (c as any).startJoints as number[] | undefined;
    if (sj && c.robot) c.robot.setJoints(sj);
    this.ctxs.push(c);
    const ps = (c as any).pendingScript;
    if (ps) {
      // record mode (compile) never executes scripts
      if (this.mode === 'record') { ps.state.done = true; ps.state.at = c.cursor; }
      else this.launchScript(c, ps.name, ps.args, ps.state, c.handle.program);
    }
    this.events.emit('run', { handle: c.handle });
  }

  private finish(c: Ctx, status: RunStatus, error?: RunError): void {
    if (c.handle.status !== 'running' && c.handle.status !== 'queued') return;
    c.handle.status = status;
    if (error) c.handle.error = error;
    // a script this run is still waiting for ends with it (RoboDK stops the Python programs a stopped program started)
    for (const a of ((c as any).scriptAborts as Array<{ aborted: boolean }> | undefined) ?? []) a.aborted = true;
    c.handle.endTime = c.cursor;
    c.handle.estimate = c.estimate;
    c.block = null;
    if (c.robot) c.robot.state.moving = false;
    this.ctxs = this.ctxs.filter((x) => x !== c);
    this.queue = this.queue.filter((x) => x !== c);
    this.events.emit('run', { handle: c.handle });
    c.resolve(c.handle);
    // start queued runs whose robot became free
    for (const q of [...this.queue]) {
      if (this.holdsBusy(q)) continue;
      this.queue = this.queue.filter((x) => x !== q);
      q.cursor = Math.max(q.cursor, c.cursor);
      this.begin(q);
    }
  }

  /** Stop one run (and the threads it started) or all runs. Robots stay where they are. */
  stop(id?: string): void {
    const all = [...this.queue, ...this.ctxs];
    const targets = id === undefined ? all : all.filter((c) => c.handle.id === id);
    const tree: Ctx[] = [];
    const collect = (c: Ctx) => { if (!tree.includes(c)) tree.push(c); for (const ch of c.children) collect(ch); };
    for (const c of targets) collect(c);
    // drop queued ones first so stopping a running run does not start them
    this.queue = this.queue.filter((q) => !tree.includes(q));
    for (const c of tree) { if (c.handle.status === 'queued') { c.handle.status = 'stopped'; this.events.emit('run', { handle: c.handle }); c.resolve(c.handle); } }
    for (const c of tree) if (c.handle.status === 'running') this.finish(c, 'stopped');
  }
  /** Continue runs paused by a "Pause (wait for user)" instruction. */
  resume(id?: string): void {
    for (const c of this.ctxs) if ((id === undefined || c.handle.id === id) && c.block?.kind === 'user') { c.block = null; c.cursor = this.time; }
  }
  /** Queued or running runs. */
  active(): RunHandle[] { return [...this.ctxs, ...this.queue].map((c) => c.handle); }
  /**
   * The live clock should stand still: a run waits for a station script whose runner is still loading its interpreter
   * (the download time is not simulation time; RoboDK's scripts start at once).
   */
  holding(): boolean {
    return this.mode === 'live' && this.ctxs.some((c) => c.block?.kind === 'script' && !c.block.state.done) && scriptRunnersLoading(this.runners);
  }
  /** Any run active (optionally: on a robot / for a program). */
  busy(what?: Robot | Item | null): boolean {
    if (!what) return this.ctxs.length + this.queue.length > 0;
    return [...this.ctxs, ...this.queue].some((c) => c.robot === what || c.handle.program === what || c.handle.robot === what);
  }

  // -- clock -------------------------------------------------------------------------------------------------------

  /** Advance the shared clock by dt seconds. Returns true while something is running. */
  tick(dt: number): boolean {
    this.advance(this.time + Math.max(0, dt));
    return this.ctxs.length + this.queue.length > 0;
  }
  /** Advance by `seconds` in steps of `step` (robots are sampled at every step). */
  runFor(seconds: number, step = 0.02): void {
    const end = this.time + Math.max(0, seconds);
    while (this.time < end - 1e-12) this.advance(Math.min(end, this.time + step));
  }
  /** Run until nothing is active or maxTime passes; returns the time reached. Scripts are awaited between steps. */
  async runUntilDone(maxTime = 3600, step = 0.05): Promise<number> {
    const end = this.time + maxTime;
    while (this.ctxs.length + this.queue.length > 0 && this.time < end) {
      this.advance(Math.min(end, this.time + step));
      await new Promise((r) => setTimeout(r, 0));
    }
    return this.time;
  }

  private advance(T: number): void {
    for (let round = 0; round < 10000; round++) {
      let progressed = false;
      for (const c of [...this.ctxs]) if (this.stepCtx(c, T)) progressed = true;
      if (!progressed) break;
    }
    if (this.mode === 'live') for (const c of this.ctxs) if (c.block?.kind === 'motion') this.sampleMotion(c, c.block, T);
    this.time = Math.max(this.time, T);
    this.nowCursor = this.time;
    this.events.emit('tick', { t: this.time });
  }

  /** Record mode: run event by event until every run is finished (waits nobody satisfies are released with a warning). */
  runToCompletion(maxSteps = 5_000_000): void {
    for (let n = 0; n < maxSteps && this.ctxs.length + this.queue.length > 0; n++) {
      const tn = this.nextEventTime();
      if (!Number.isFinite(tn)) { if (!this.releaseDeadlock()) break; continue; }
      this.advance(Math.max(tn, this.time));
    }
  }

  private nextEventTime(): number {
    let tn = Infinity;
    for (const c of this.ctxs) {
      const b = c.block;
      if (!b) tn = Math.min(tn, c.cursor);
      else if (b.kind === 'motion') tn = Math.min(tn, b.ei < b.events.length ? b.events[b.ei].t : b.t1, b.t1);
      else if (b.kind === 'until') tn = Math.min(tn, b.t);
      else if (b.kind === 'cond') tn = Math.min(tn, b.test() ? c.cursor : b.deadline);
      else if (b.kind === 'script') { if (b.state.done) tn = Math.min(tn, c.cursor); }
    }
    return tn;
  }

  private releaseDeadlock(): boolean {
    let best: Ctx | null = null;
    for (const c of this.ctxs) if (c.block && (c.block.kind === 'cond' || c.block.kind === 'user' || c.block.kind === 'script') && (!best || c.cursor < best.cursor)) best = c;
    if (!best) return false;
    const b = best.block!;
    if (b.kind === 'cond') this.problem(best, b.ins, `${b.label}: never set during the simulation, continued`, 'warning');
    if (b.kind === 'script') b.state.done = true;
    if ('step' in b && b.step) b.step.t1 = best.cursor;
    best.block = null;
    return true;
  }

  // -- interpreter -------------------------------------------------------------------------------------------------

  private stepCtx(c: Ctx, T: number): boolean {
    let progressed = false;
    for (let guard = 0; guard < 1_000_000; guard++) {
      if (c.handle.status !== 'running') return progressed;
      if (c.block) {
        if (!this.resolveBlock(c, T)) return progressed;
        progressed = true;
        continue;
      }
      if (c.cursor > T + 1e-9) return progressed;
      const fr = c.frames[c.frames.length - 1];
      if (!fr) { this.finish(c, 'done'); return true; }
      if (fr.pc >= fr.list.length) {
        if (fr.repeat !== 0) { if (fr.repeat > 0) fr.repeat--; fr.pc = 0; continue; }
        c.frames.pop();
        continue;
      }
      this.nowCursor = c.cursor;
      this.execNext(c, fr);
      progressed = true;
    }
    return progressed;
  }

  private resolveBlock(c: Ctx, T: number): boolean {
    const b = c.block!;
    switch (b.kind) {
      case 'motion': {
        while (b.ei < b.events.length && b.events[b.ei].t <= T + 1e-12) { const e = b.events[b.ei++]; this.nowCursor = e.t; e.fn(); }
        if (T + 1e-12 < b.t1) return false;
        this.sampleMotion(c, b, b.t1);
        c.cursor = b.t1;
        c.block = null;
        if (b.fail) this.fail(c, b.fail.ins, b.fail.message);
        return true;
      }
      case 'until':
        if (T + 1e-12 < b.t) return false;
        c.cursor = b.t; c.block = null;
        if (b.step) b.step.t1 = b.t;
        return true;
      case 'cond': {
        if (b.test()) {
          const ts = Math.max(...b.names.map((n) => this.signals.timeOf(n) ?? -Infinity));
          c.cursor = Math.max(c.cursor, Number.isFinite(ts) ? Math.min(ts, T) : c.cursor);
          c.block = null;
          if (b.step) b.step.t1 = c.cursor;
          return true;
        }
        if (T + 1e-12 >= b.deadline) {
          c.cursor = b.deadline; c.block = null;
          if (b.step) b.step.t1 = c.cursor;
          this.problem(c, b.ins, `${b.label}: timeout`, 'warning');
          return true;
        }
        return false;
      }
      case 'script':
        if (!b.state.done) return false;
        c.cursor = Math.max(c.cursor, Math.min(T, b.state.at ?? this.time));
        c.block = null;
        if (b.step) b.step.t1 = c.cursor;
        if (b.state.error !== undefined) {
          const msg = `Script ${b.ins ? b.ins.name : c.handle.program.name} failed: ${(b.state.error as any)?.message ?? String(b.state.error)}`;
          if (b.ins) this.fail(c, b.ins, msg);
          else this.finish(c, 'error', { message: msg, instructionIndex: -1 });
        } else if (!b.ins && !c.frames.length) this.finish(c, 'done');
        return true;
      case 'user':
        return false;
    }
  }

  private sampleMotion(c: Ctx, b: Extract<Block, { kind: 'motion' }>, T: number): void {
    if (!b.segs.length) return;
    let k = 0;
    for (let i = 0; i < b.segs.length; i++) if (b.segs[i].t0 <= T + 1e-12) k = i; else break;
    const s = b.segs[k];
    if (k !== b.cur) {
      b.cur = k;
      if (k > 0 && s.ins && this.mode === 'live') {
        if (c.frames.length === 1 && s.index !== undefined) c.handle.instructionIndex = s.index;
        this.events.emit('instruction', { handle: c.handle, program: s.ins.parent as Program, instruction: s.ins, index: s.index ?? -1 });
      }
    }
    const local = Math.max(0, Math.min(s.t1 - s.t0, T - s.t0));
    if (s.traj && s.group) {
      s.group.apply(sampleTraj(s.traj, local).joints);
      const r = s.group.robot;
      r.state.moving = T < b.t1 - 1e-9;
      r.state.progress = s.t1 > s.t0 ? Math.min(1, local / (s.t1 - s.t0)) : 1;
    } else if (s.mobile) {
      const p = s.mobile.sample(local);
      const m = s.mobile.item as any;
      if (typeof m.setPose2D === 'function') m.setPose2D(p.x, p.y, p.heading);
    }
  }

  private problem(c: Ctx | null, ins: Instruction | null, message: string, severity: 'error' | 'warning'): void {
    this.problems.push({ instructionId: ins?.id ?? '', message, severity, runId: c?.handle.id, t: c ? c.cursor : this.time });
    this.events.emit('message', { text: ins ? `${(ins.parent as Program | null)?.name ?? ''}: ${message}` : message, level: severity === 'error' ? 'error' : 'warn', handle: c?.handle });
  }
  private warnOnce(c: Ctx, ins: Instruction, key: string, message: string): void {
    if (this.warned.has(key)) return;
    this.warned.add(key);
    this.problem(c, ins, message, 'warning');
  }

  private fail(c: Ctx, ins: Instruction, message: string): void {
    this.problem(c, ins, message, 'error');
    // RoboDK Item.Update() stops counting a program at its failing instruction; a caller goes on after the call
    const top = c.frames[c.frames.length - 1];
    if (top) top.noEstimate = true;
    if (this.continueOnError) return;
    const prog = ins.parent instanceof Program ? ins.parent : null;
    this.finish(c, 'error', { message, instructionIndex: prog ? prog.instructions().indexOf(ins) : -1, instructionId: ins.id, program: prog?.name });
  }

  private record(c: Ctx, ins: Instruction, robot: Robot | null, t0: number, t1: number, extra: Partial<RecordedStep> = {}): RecordedStep | undefined {
    if (this.mode !== 'record') return undefined;
    const s: RecordedStep = { instruction: ins, robot, t0, t1, runId: c.handle.id, ...extra };
    this.recorded.push(s);
    return s;
  }

  /** Execute an instant action now and record it for replay. */
  private instant(c: Ctx, ins: Instruction, robot: Robot | null, fn?: () => void): void {
    fn?.();
    this.record(c, ins, robot, this.nowCursor, this.nowCursor, fn ? { apply: fn } : {});
  }

  private execNext(c: Ctx, fr: Frame): void {
    const ins = fr.list[fr.pc];
    if (!ins.enabled) { fr.pc++; return; }
    if (c.frames.length === 1) c.handle.instructionIndex = fr.pc;
    this.executed++;
    this.events.emit('instruction', { handle: c.handle, program: fr.program, instruction: ins, index: fr.pc });
    const d = ins.data, robot = fr.robot;
    switch (d.kind) {
      case 'move':
        this.execMoves(c, fr);
        return;
      case 'jointPath': {
        fr.pc++;
        if (!robot) return this.fail(c, ins, 'Program has no robot');
        const rows = d.joints.filter((r) => r.length >= robot.dof).map((r) => r.slice(0, robot.dof));
        if (rows.length < 1) return this.fail(c, ins, 'Joint path is empty');
        const st = this.robotState(robot);
        const tool = this.toolPose(robot, st.toolId);
        const q0 = robot.joints();
        const samples: TrajectorySample[] = [{ t: 0, joints: q0, pose: robot.solveFK(q0, tool) }];
        let tt = 0, length = 0, prev = q0, prevPose = samples[0].pose;
        for (const r of rows) {
          const dq = Math.max(...r.map((v, i) => Math.abs(v - prev[i])), 0);
          const pose = robot.solveFK(r, tool);
          const dist = Math.hypot(pose[12] - prevPose[12], pose[13] - prevPose[13], pose[14] - prevPose[14]);
          tt += d.dt ?? Math.max(dq / Math.max(1, st.settings.speedJoints), dist / Math.max(1, st.settings.speedLinear), 0.001);
          length += dist;
          samples.push({ t: tt, joints: r, pose });
          prev = r; prevPose = pose;
        }
        const traj = new PlannedTrajectory(samples, [], length);
        const group = new MotionGroup(robot, []);
        this.distance += length;
        this.addEstimate(c, tt);
        this.record(c, ins, robot, c.cursor, c.cursor + tt, { trajectory: traj, group });
        c.block = { kind: 'motion', t1: c.cursor + tt, segs: [{ t0: c.cursor, t1: c.cursor + tt, traj, group }], events: [], ei: 0 };
        return;
      }
      case 'speed': {
        fr.pc++;
        if (robot && !this.ignoreSpeedInstructions) this.applySpeed(this.robotState(robot).settings, d);
        this.record(c, ins, robot, c.cursor, c.cursor);
        return;
      }
      case 'rounding':
        fr.pc++;
        if (robot) this.robotState(robot).settings.rounding = d.radius;
        this.record(c, ins, robot, c.cursor, c.cursor);
        return;
      case 'frame': {
        fr.pc++;
        if (robot) { const rs = this.robotState(robot); rs.frameId = d.frameId; rs.frameSet = true; }
        const st = this.station, id = d.frameId;
        this.instant(c, ins, robot, () => robot?.setFrame(id ? st.findById(id) : null));
        return;
      }
      case 'tool': {
        fr.pc++;
        if (robot) this.robotState(robot).toolId = d.toolId;
        const st = this.station, id = d.toolId;
        this.instant(c, ins, robot, () => { const t = id ? st.findById(id) : null; robot?.setTool(t instanceof Tool ? t : null); });
        return;
      }
      case 'pause': {
        fr.pc++;
        if (d.timeMs < 0) {
          if (this.mode === 'record') { this.record(c, ins, robot, c.cursor, c.cursor); return; }
          this.events.emit('message', { text: `${fr.program.name}: paused, waiting for the user`, level: 'info', handle: c.handle });
          c.block = { kind: 'user' };
          return;
        }
        const t1 = c.cursor + d.timeMs / 1000;
        this.addEstimate(c, d.timeMs / 1000);
        c.block = { kind: 'until', t: t1, step: this.record(c, ins, robot, c.cursor, t1) };
        return;
      }
      case 'io':
      case 'signal': {
        fr.pc++;
        const name = d.kind === 'io' ? d.io : d.signal, value = d.value;
        if (d.wait) {
          const timeout = d.kind === 'io' && d.timeoutMs !== undefined && d.timeoutMs > 0 ? d.timeoutMs / 1000 : Infinity;
          this.waitFor(c, ins, robot, [name], () => signalEquals(this.signals.get(name), value), timeout, `Wait ${name} = ${value}`);
          return;
        }
        this.instant(c, ins, robot, this.setterFor(d.kind, name, value, robot));
        return;
      }
      case 'wait': {
        fr.pc++;
        if (d.what === 'time') { const t1 = c.cursor + Math.max(0, d.timeMs ?? 0) / 1000; this.addEstimate(c, t1 - c.cursor); c.block = { kind: 'until', t: t1, step: this.record(c, ins, robot, c.cursor, t1) }; return; }
        if (d.what === 'signal' && d.signal) {
          const name = d.signal, value = d.value ?? true;
          const timeout = d.timeMs !== undefined && d.timeMs > 0 ? d.timeMs / 1000 : Infinity;
          this.waitFor(c, ins, robot, [name], () => signalEquals(this.signals.get(name), value), timeout, `Wait ${name} = ${value}`);
          return;
        }
        this.record(c, ins, robot, c.cursor, c.cursor);
        return;
      }
      case 'event': {
        fr.pc++;
        const toolId = robot ? this.robotState(robot).toolId : null;
        const fn = this.eventAction(robot, toolId, d.action, d.objectId ?? null, d.name ?? null, (d as { restore?: ObjectPlacement[] }).restore, (d as { frameId?: string | null }).frameId ?? null);
        this.instant(c, ins, robot, fn);
        return;
      }
      case 'print': {
        fr.pc++;
        const text = d.message, comment = d.isComment;
        this.instant(c, ins, robot, comment ? undefined : () => { this.events.emit('message', { text, level: 'info', handle: c.handle }); this.sink.message?.(text, 'info'); });
        return;
      }
      case 'mission_task':
        fr.pc++;
        this.record(c, ins, robot, c.cursor, c.cursor);
        return;
      case 'code': {
        fr.pc++;
        if (!d.asFunctionCall) { this.record(c, ins, robot, c.cursor, c.cursor); return; }
        const { name, args } = parseCallText(d.code);
        this.callByName(c, fr, ins, name, args, d.code);
        return;
      }
      case 'call': {
        fr.pc++;
        const sub = d.programId ? this.station.findById(d.programId) : null;
        if (sub instanceof Program) { this.pushCall(c, fr, ins, sub); return; }
        if (sub && sub.type === ItemType.PROGRAM_PYTHON) { this.runScript(c, ins, robot, sub.name, [], sub); return; }
        if (!sub && d.programName && !d.programId) { const { name, args } = parseCallText(d.programName); const found = this.findProgram(name); if (found) { this.callByName(c, fr, ins, name, args, d.programName); return; } }
        return this.fail(c, ins, `Program ${d.programName ?? d.programId} not found`);
      }
      case 'loop': {
        fr.pc++;
        const body = d.bodyProgramId ? this.station.findById(d.bodyProgramId) : null;
        if (!(body instanceof Program) || d.count === 0) return;
        if (c.frames.some((f) => f.program === body) || c.frames.length > 32) return this.fail(c, ins, `Recursive call to ${body.name}`);
        let repeat = d.count > 0 ? d.count - 1 : -1;
        if (repeat < 0 && this.mode === 'record') { repeat = 0; this.warnOnce(c, ins, `loop:${ins.id}`, `Endless loop over ${body.name}: simulated once`); }
        const f = this.frameFor(body, robot);
        f.repeat = repeat;
        c.frames.push(f);
        return;
      }
      case 'if': {
        fr.pc++;
        const v = evalCondition(d.condition, (n) => this.signals.get(n));
        const id = v ? d.thenProgramId : d.elseProgramId ?? null;
        const body = id ? this.station.findById(id) : null;
        if (body instanceof Program) this.pushCall(c, fr, ins, body);
        return;
      }
      case 'thread': {
        fr.pc++;
        const sub = (d.programId ? this.station.findById(d.programId) : null) ?? this.findProgram(parseCallText(d.programName ?? '').name);
        if (sub && sub.type === ItemType.PROGRAM_PYTHON) {
          // a station Python program on its own thread (RoboDK "Run on a separate thread"): script runner in live
          // playback, not simulated when compiling
          this.record(c, ins, robot, c.cursor, c.cursor);
          if (this.mode === 'record') { this.warnOnce(c, ins, `pythread:${sub.id}`, `Thread ${sub.name} is a Python program: it runs through the script runner in live playback only`); return; }
          const saved = this.time;
          this.time = Math.max(this.time, c.cursor);
          const h = this.start(sub, { args: parseCallText(d.programName ?? '').args });
          this.time = saved;
          h.parentId = c.handle.id;
          const child = [...this.ctxs, ...this.queue].find((x) => x.handle === h);
          if (child) { child.parent = c; c.children.push(child); }
          return;
        }
        if (!(sub instanceof Program)) return this.fail(c, ins, `Thread program ${d.programName ?? d.programId} not found`);
        const subRobot = programRobot(sub);
        // the thread waits for the robot only when this run moves it too (06.e StartSimulation only calls UR10 B)
        if (subRobot instanceof Robot && c.holds.has(subRobot)) this.problem(c, ins, 'Thread uses the same robot as the main program', 'warning');
        this.record(c, ins, robot, c.cursor, c.cursor);
        this.nowCursor = c.cursor;
        const saved = this.time;
        this.time = Math.max(this.time, c.cursor);
        const h = this.start(sub);
        this.time = saved;
        (h as RunHandle).parentId = c.handle.id;
        const child = [...this.ctxs, ...this.queue].find((x) => x.handle === h);
        if (child) { child.parent = c; c.children.push(child); child.cursor = Math.max(child.cursor, c.cursor); if (child.handle.startTime !== undefined) child.handle.startTime = child.cursor; }
        return;
      }
      case 'mobile_move':
      case 'mobile_follow': {
        fr.pc++;
        const mob = fr.mover ?? mobileOf(robot);
        if (!mob) return this.fail(c, ins, 'Program has no mobile robot');
        const s = (mob as any).state ?? {};
        const start: Pose2D = { x: s.x ?? mob.pose()[12], y: s.y ?? mob.pose()[13], heading: s.theta ?? (Math.atan2(mob.pose()[1], mob.pose()[0]) * 180) / Math.PI };
        let plan;
        try { plan = getMobileMotionPlanner()(this.station, mob, d, start); } catch (e) { return this.fail(c, ins, `Mobile planner failed: ${(e as Error).message}`); }
        // a drive that did not end at the goal still drives (the base stops where the drive model stopped, like an arm
        // motion plays up to its failing point) and fails afterwards; a plan without a trajectory fails at once
        if (!plan.ok && !(plan.duration > 0)) return this.fail(c, ins, plan.error ?? 'Mobile motion failed');
        const fail = plan.ok ? undefined : { ins, message: plan.error ?? 'Mobile motion failed' };
        const dur = Math.max(0, plan.duration);
        const mobile = { item: mob, sample: plan.sample, duration: dur };
        this.record(c, ins, robot, c.cursor, c.cursor + dur, { mobile });
        this.distance += Math.hypot(plan.sample(dur).x - start.x, plan.sample(dur).y - start.y);
        this.addEstimate(c, dur);
        c.block = { kind: 'motion', t1: c.cursor + dur, segs: [{ t0: c.cursor, t1: c.cursor + dur, mobile }], events: [], ei: 0, ...(fail ? { fail } : {}) };
        return;
      }
    }
  }

  /** Items by name (built once per start(): code calls such as Extruder(N) and named events are looked up per instruction). */
  private nameIndex: Map<string, Item[]> | null = null;
  /** First item named `name` (optionally of a type) in tree order, like Station.find but O(1). */
  findNamed(name: string, type?: ItemType): Item | null {
    if (!name) return null;
    if (!this.nameIndex) {
      this.nameIndex = new Map();
      for (const it of this.station.walk()) { if (it === this.station) continue; const l = this.nameIndex.get(it.name); if (l) l.push(it); else this.nameIndex.set(it.name, [it]); }
    }
    for (const it of this.nameIndex.get(name) ?? []) {
      if (it.station !== this.station) { this.nameIndex = null; return this.findNamed(name, type); }
      if (type === undefined || it.type === type) return it;
    }
    return null;
  }
  private findProgram(name: string): Item | null {
    return this.findNamed(name, ItemType.PROGRAM) ?? this.findNamed(name, ItemType.PROGRAM_PYTHON);
  }

  private callByName(c: Ctx, fr: Frame, ins: Instruction, name: string, args: string[], text: string): void {
    const item = this.findProgram(name);
    if (item instanceof Program) { this.pushCall(c, fr, ins, item); return; }
    if (item && item.type === ItemType.PROGRAM_PYTHON) { this.runScript(c, ins, fr.robot, name, args, item); return; }
    if (this.runnersFor().length && this.mode === 'live') { this.runScript(c, ins, fr.robot, name, args, null); return; }
    // record mode (validation / preview): a call a registered runner declares it handles (VisionDetect, ...) runs in live
    // playback, so it is not reported as ignored
    if (!this.runnersFor().some((r) => { try { return !!r.handles?.(name); } catch { return false; } })) this.warnOnce(c, ins, `call:${name}`, `Call ${text}: no program or script named ${name} in the station (ignored in simulation)`);
    this.record(c, ins, fr.robot, c.cursor, c.cursor);
  }

  private pushCall(c: Ctx, fr: Frame, ins: Instruction, sub: Program): void {
    if (c.frames.some((f) => f.program === sub)) return this.fail(c, ins, `Recursive call to ${sub.name}`);
    if (c.frames.length > 32) return this.fail(c, ins, 'Recursion too deep');
    const f = this.frameFor(sub, fr.robot);
    if (f.robot && f.robot !== fr.robot) f.foreign = true;
    c.frames.push(f);
  }

  private runnersFor(): ScriptRunner[] { return [...this.runners, ...globalRunners]; }

  private runScript(c: Ctx, ins: Instruction, robot: Robot | null, name: string, args: string[], item: Item | null): void {
    const step = this.record(c, ins, robot, c.cursor, c.cursor);
    if (this.mode === 'record') return;
    const state = { done: false as boolean, error: undefined as unknown, at: undefined as number | undefined };
    c.block = { kind: 'script', state, ins, step };
    this.launchScript(c, name, args, state, item);
  }

  private launchScript(c: Ctx, name: string, args: string[], state: { done: boolean; error?: unknown; at?: number }, item: Item | null): void {
    const runners = this.runnersFor();
    if (!runners.length) {
      if (item) this.events.emit('message', { text: `No script runner registered: ${name} not executed`, level: 'warn', handle: c.handle });
      state.done = true; state.at = c.cursor;
      return;
    }
    const robot = c.robot;
    const abort = { aborted: false };
    const aborts: Array<{ aborted: boolean }> = ((c as any).scriptAborts ??= []);
    aborts.push(abort);
    const launchedAt = c.cursor;
    const ctx: ScriptContext = { station: this.station, runtime: this, name, args, robot, program: item ?? c.handle.program, abort };
    // the run resumes when the script returned: at the clock time then, or at launch + the script's own duration when
    // the runner reports it (a script answered in the same page task took no simulated time)
    const doneAt = () => (typeof ctx.elapsed === 'number' && ctx.elapsed >= 0 ? Math.min(this.time, launchedAt + ctx.elapsed) : this.time);
    const settle = () => { const k = aborts.indexOf(abort); if (k >= 0) aborts.splice(k, 1); };
    // runners in order (per-runtime first, then global); a runner resolving `false` does not handle the call
    const tryAt = (i: number): void => {
      if (i >= runners.length) {
        this.events.emit('message', { text: item ? `No script runner handled ${name}: not executed` : `Call ${name}: no program or script named ${name} in the station (ignored in simulation)`, level: 'warn', handle: c.handle });
        state.done = true; state.at = this.time; settle();
        return;
      }
      let p: Promise<void | boolean>;
      try { p = Promise.resolve(runners[i](ctx)); } catch (e) { p = Promise.reject(e); }
      p.then((r) => { if (r === false) tryAt(i + 1); else { state.done = true; state.at = doneAt(); settle(); } }, (e) => { state.done = true; state.error = e ?? 'error'; state.at = doneAt(); settle(); });
    };
    tryAt(0);
  }

  private waitFor(c: Ctx, ins: Instruction, robot: Robot | null, names: string[], test: () => boolean, timeout: number, label: string): void {
    const step = this.record(c, ins, robot, c.cursor, c.cursor);
    if (test()) return;
    c.block = { kind: 'cond', test, names, deadline: c.cursor + timeout, label, ins, step };
  }

  private setterFor(kind: 'io' | 'signal', name: string, value: SignalValue, robot: Robot | null): () => void {
    return () => {
      this.signals.set(name, value);
      if (kind === 'io') { if (robot) robot.state.io[name] = value as number | boolean; this.events.emit('io', { name, value }); this.sink.io?.(name, value); }
      else this.sink.signal?.(name, value);
    };
  }

  private applySpeed(s: MotionSettings, d: Extract<InstructionData, { kind: 'speed' }>): void {
    if (d.speedLinear !== undefined && d.speedLinear > 0) s.speedLinear = d.speedLinear;
    if (d.speedJoints !== undefined && d.speedJoints > 0) s.speedJoints = d.speedJoints;
    if (d.accelLinear !== undefined && d.accelLinear > 0) s.accelLinear = d.accelLinear;
    if (d.accelJoints !== undefined && d.accelJoints > 0) s.accelJoints = d.accelJoints;
  }

  /**
   * TCP relative to the robot flange. A tool mounted on a mechanism the robot carries (weld gun / gripper robot, RoboDK
   * 03.i, 06.f) is resolved through that mechanism at its current joints, not as if it sat on the flange.
   */
  toolPose(robot: Robot, toolId: string | null): Mat4 {
    const t = toolId ? this.station.findById(toolId) : robot.activeTool();
    if (!(t instanceof Tool)) return identity();
    if (t.parent && t.parent !== robot && robot.isAncestorOf(t)) return multiply(invert(multiply(robot.poseAbs(), robot.solveFKFlange())), t.poseAbs());
    return t.poseTool();
  }
  private frameItem(robot: Robot, frameId: string | null): Item | null {
    const f = frameId ? this.station.findById(frameId) : robot.activeFrame();
    return f ?? null;
  }

  /** Plan and start a blended chain of consecutive moves starting at fr.pc. */
  private execMoves(c: Ctx, fr: Frame): void {
    const robot = fr.robot;
    const first = fr.list[fr.pc];
    if (!robot) { fr.pc++; return this.fail(c, first, 'Program has no robot'); }
    const st = this.robotState(robot);
    const settings = { ...st.settings }, base = { ...st.settings }, pc0 = fr.pc;
    const specs: ChainMoveSpec[] = [], chainIns: Instruction[] = [], chainIdx: number[] = [];
    const between: Array<{ after: number; ins: Instruction }> = [];
    let pending: Instruction[] = [];
    let settingsAfterLast = { ...settings };
    const tool = this.toolPose(robot, st.toolId), frame = this.frameItem(robot, st.frameId);
    // RoboDK reference semantics apply once the program set its reference frame (RoboDK programs always do; a studio
    // program that never sets one keeps its targets at their absolute poses whatever frame is active in the UI)
    const targetFrame = st.frameSet && st.frameId ? this.station.findById(st.frameId) : null;
    for (let j = fr.pc; j < fr.list.length && specs.length < CHAIN_MAX; j++) {
      const x = fr.list[j];
      if (!x.enabled) continue;
      const xd = x.data;
      if (xd.kind === 'move') {
        const target = xd.targetId ? this.station.findById(xd.targetId) : null;
        if (xd.targetId && !(target instanceof Target)) {
          if (!specs.length) { fr.pc = j + 1; return this.fail(c, x, 'Target not found'); }
          break;
        }
        const via = xd.viaTargetId ? this.station.findById(xd.viaTargetId) : null;
        specs.push({ data: xd as MoveInstruction, target: target as Target | null, via: via instanceof Target ? via : null, tool, frame, targetFrame, settings: { ...settings, speedLinear: xd.speed && xd.speed > 0 ? xd.speed : settings.speedLinear, rounding: xd.rounding ?? settings.rounding } });
        chainIns.push(x); chainIdx.push(j);
        for (const p of pending) between.push({ after: specs.length - 2, ins: p });
        pending = [];
        settingsAfterLast = { ...settings };
        continue;
      }
      if (PASSTHROUGH.has(xd.kind) && !((xd.kind === 'io' || xd.kind === 'signal') && xd.wait)) {
        if (xd.kind === 'speed' && !this.ignoreSpeedInstructions) this.applySpeed(settings, xd);
        if (xd.kind === 'rounding') settings.rounding = xd.radius;
        pending.push(x);
        continue;
      }
      break;
    }
    const group = motionGroupFor(robot, specs.flatMap((s) => [s.target, s.via]));
    const res = planChain(group, group.current(), specs, { estimate: updateModelOf(robot) });
    const t0 = c.cursor;
    // a failing move is not executed: the robot stops at the end of the previous move (RoboDK stops the program at the
    // instruction it cannot execute)
    const nRun = res.failed ? res.failed.index : res.moves.length;
    const segs: MotionSeg[] = [];
    for (let k = 0; k < nRun; k++) {
      const m = res.moves[k];
      segs.push({ t0: t0 + m.t0, t1: t0 + m.t1, traj: m.trajectory, group, ins: chainIns[k], index: chainIdx[k] });
      this.distance += m.trajectory.length;
      let est = m.estimate ?? 0;
      if (k === 0 && fr.foreign) {
        // RoboDK Item.Update(): a call into a program of another robot starts from that robot's joints at the start of
        // the update, not from where an earlier call left it (06.d Stamp1: ToolLoad 0 s, UnloadMachine full)
        fr.foreign = false;
        const q0 = this.home?.snap.jointsOf(robot);
        if (q0 && res.moves.length) est = this.estimateFrom(robot, q0, specs[0], m, tool);
      }
      this.addEstimate(c, est);
      for (const w of m.warnings) this.warnOnce(c, chainIns[k], `${fr.program.id}:${w}`, w);
      this.record(c, chainIns[k], robot, t0 + m.t0, t0 + m.t1, { trajectory: m.trajectory, group, estimate: m.estimate, qTarget: m.qTarget });
      if (k > 0) this.executed++;
    }
    const events: Array<{ t: number; fn: () => void }> = [];
    for (const b of between) {
      if (b.after >= nRun) continue;
      const tEv = b.after >= 0 ? t0 + res.moves[b.after].t1 : t0;
      const bi = b.ins, bd = bi.data;
      events.push({ t: tEv, fn: () => {
        this.executed++;
        if (bd.kind === 'speed' || bd.kind === 'rounding') { this.record(c, bi, robot, this.nowCursor, this.nowCursor); return; }
        if (bd.kind === 'print') { const text = bd.message; this.instant(c, bi, robot, bd.isComment ? undefined : () => { this.events.emit('message', { text, level: 'info', handle: c.handle }); this.sink.message?.(text, 'info'); }); return; }
        if (bd.kind === 'io') { this.instant(c, bi, robot, this.setterFor('io', bd.io, bd.value, robot)); return; }
        if (bd.kind === 'signal') { this.instant(c, bi, robot, this.setterFor('signal', bd.signal, bd.value, robot)); }
      } });
    }
    events.sort((a, b) => a.t - b.t);
    if (res.failed) {
      fr.pc = chainIdx[res.failed.index] + 1;
      const k = res.failed.index;
      st.settings = this.settingsUpTo(fr, pc0, chainIdx[k], base);
      const tf = segs.length ? segs[segs.length - 1].t1 : t0;
      this.record(c, chainIns[k], robot, tf, tf);
      if (k > 0) this.executed++;
      c.block = { kind: 'motion', t1: tf, segs, events, ei: 0, fail: { ins: chainIns[k], message: res.failed.message } };
    } else {
      fr.pc = chainIdx[chainIdx.length - 1] + 1;
      st.settings = settingsAfterLast;
      c.block = { kind: 'motion', t1: t0 + res.duration, segs, events, ei: 0 };
    }
  }

  /** Add to the Item.Update() estimate of a run unless the executing program (or a caller) already failed. */
  private addEstimate(c: Ctx, dt: number): void {
    if (!c.frames.some((f) => f.noEstimate)) c.estimate += dt;
  }

  /** Item.Update() time of a planned move re-measured from other start joints (rest to rest). */
  private estimateFrom(robot: Robot, q0: number[], spec: ChainMoveSpec, m: { trajectory: Trajectory; qEnd: number[] }, tool: Mat4): number {
    const qb = m.qEnd.slice(0, robot.dof), qa = q0.slice(0, robot.dof);
    const travel = qa.reduce((a, v, i) => Math.max(a, Math.abs((qb[i] ?? v) - v)), 0);
    const model = updateModelOf(robot);
    if (spec.data.moveType === 'MoveJ') return updateMoveTime(model, false, 0, travel);
    const pa = robot.solveFK(qa, tool), pb = robot.solveFK(qb, tool);
    return updateMoveTime(model, true, Math.hypot(pb[12] - pa[12], pb[13] - pa[13], pb[14] - pa[14]), travel);
  }

  /** Robot settings after executing the speed/rounding instructions between list indices [from, to). */
  private settingsUpTo(fr: Frame, from: number, to: number, base: MotionSettings): MotionSettings {
    const s = { ...base };
    for (let j = from; j < to; j++) {
      const d = fr.list[j].data;
      if (!fr.list[j].enabled) continue;
      if (d.kind === 'speed' && !this.ignoreSpeedInstructions) this.applySpeed(s, d);
      if (d.kind === 'rounding') s.rounding = d.radius;
    }
    return s;
  }

  // -- events (attach / detach / show / hide) -----------------------------------------------------------------------

  private eventAction(robot: Robot | null, toolId: string | null, action: string, objectId: string | null, label: string | null, restore?: ObjectPlacement[], frameId: string | null = null): (() => void) | undefined {
    const st = this.station;
    const toolOf = (): Tool | null => {
      let tool = (toolId ? st.findById(toolId) : robot?.activeTool()) as Tool | null;
      if (label) { const named = this.findNamed(label, ItemType.TOOL); if (named instanceof Tool) tool = named; }
      return tool instanceof Tool ? tool : null;
    };
    switch (action) {
      case 'attach':
      case 'gripper_close': {
        const tool = toolOf();
        if (!tool) return undefined;
        // resolve the object now (closest to the TCP within the tool's reach), replay attaches the same object
        const obj = objectId ? st.findById(objectId) : this.closestObject(tool, robot);
        const id = obj?.id ?? null;
        return () => {
          if (action === 'gripper_close') tool.closed = true;
          const o = id ? st.findById(id) : null;
          if (o instanceof SceneObject) this.attachObject(o, tool);
        };
      }
      case 'detach':
      case 'gripper_open': {
        const tool = toolOf();
        return () => {
          if (tool && action === 'gripper_open') tool.closed = false;
          const ids = objectId ? [objectId] : tool ? [...tool.attached] : [...st.walk()].filter((t): t is Tool => t instanceof Tool).flatMap((t) => t.attached);
          // RoboDK "Detach from <tool>" places the objects in the instruction's reference frame (when it names one)
          const dest = frameId ? st.findById(frameId) : null;
          for (const oid of ids) this.detachObject(oid, dest);
        };
      }
      case 'show':
      case 'hide': {
        if (label === 'objects') return () => setPartVisibility(st, action === 'show' ? 'show-raw' : 'hide-done');
        const target = objectId ? st.findById(objectId) : label ? this.findNamed(label) : null;
        return target ? () => target.setVisible(action === 'show') : undefined;
      }
      case 'custom': {
        if (label && /^replace objects$/i.test(label)) {
          // RoboDK "Replace objects" stores the placements to restore (imported as `restore`): parent frame + pose
          if (restore?.length) return () => { for (const r of restore) this.placeObject(r); };
          const home = this.home?.snap ?? null;
          return () => { home?.restoreObjects(); setPartVisibility(st, 'reset'); };
        }
        return undefined;
      }
    }
    return undefined;
  }

  /** Put an object back under a frame at a pose relative to it (releasing it from a tool). */
  placeObject(r: ObjectPlacement): void {
    const o = this.station.findById(r.id);
    if (!(o instanceof SceneObject)) return;
    if (o.parent instanceof Tool) { o.parent.attached = o.parent.attached.filter((x) => x !== o.id); this.sink.detach?.(o.id); }
    for (const t of this.station.walk()) if (t instanceof Tool && t.attached.includes(o.id)) t.attached = t.attached.filter((x) => x !== o.id);
    this.homeParents.delete(o.id);
    const par = r.parentId ? this.station.findById(r.parentId) : null;
    if (par && o.parent !== par && !o.isAncestorOf(par)) par.addChild(o);
    if (Array.isArray(r.pose) && r.pose.length === 16) o.setPose(fromArray(r.pose));
    if (r.visible !== undefined && o.visible !== r.visible) o.setVisible(r.visible);
  }

  /** Attach: the object becomes a child of the tool (RoboDK semantics), keeping its absolute pose. */
  attachObject(obj: SceneObject, tool: Tool): void {
    if (obj === tool || obj.isAncestorOf(tool)) return;
    const prev = obj.parent;
    if (prev instanceof Tool) prev.attached = prev.attached.filter((x) => x !== obj.id);
    else if (prev && !this.homeParents.has(obj.id)) this.homeParents.set(obj.id, prev);
    if (obj.parent !== tool) obj.setParentStatic(tool);
    if (!tool.attached.includes(obj.id)) tool.attached.push(obj.id);
    this.sink.attach?.(obj.id, tool);
  }
  /**
   * Detach: the object goes to `to` (the Detach instruction's reference frame), else back to the frame it was attached
   * from (station root if unknown), keeping its absolute pose.
   */
  detachObject(id: string, to: Item | null = null): void {
    const obj = this.station.findById(id);
    if (!obj) return;
    const tool = obj.parent;
    if (tool instanceof Tool) {
      tool.attached = tool.attached.filter((x) => x !== id);
      const dest = to && to.station === this.station && !obj.isAncestorOf(to) && to !== obj ? to : this.homeParents.get(id);
      obj.setParentStatic(dest && dest.station === this.station ? dest : this.station);
    } else for (const t of this.station.walk()) if (t instanceof Tool && t.attached.includes(id)) t.attached = t.attached.filter((x) => x !== id);
    this.homeParents.delete(id);
    this.sink.detach?.(id);
  }

  /**
   * Object closest to the TCP whose bounding box is within the tool's reach (tool param attachTolerance, mm). A hidden
   * object counts only as the twin of a part this tool already holds (same pose): RoboDK stations carry a hidden raw part
   * and its visible finished twin with two Attach events (06.d), while a single Attach must take the visible part (06.f).
   */
  closestObject(tool: Tool, robot: Robot | null): SceneObject | null {
    const tcp = tool.poseAbs();
    const p: [number, number, number] = [tcp[12], tcp[13], tcp[14]];
    const tolParam = Number(tool.getParam('attachTolerance'));
    let best: SceneObject | null = null, bestD = Infinity, bestO = Infinity, bestV = false;
    const held = tool.attached.map((id) => this.station.findById(id)).filter((o): o is Item => !!o).map((o) => o.poseAbs());
    const twinOfHeld = (o: Item) => { const q = o.poseAbs(); return held.some((h) => Math.hypot(h[12] - q[12], h[13] - q[13], h[14] - q[14]) < 1); };
    for (const it of this.station.walk()) {
      if (!(it instanceof SceneObject) || it instanceof Tool) continue;
      if (!it.visible && !twinOfHeld(it)) continue;
      if (it.parent instanceof Tool || it.isAncestorOf(tool) || (robot && (it.isAncestorOf(robot) || robot.isAncestorOf(it)))) continue;
      const box = this.localBox(it);
      const abs = it.poseAbs();
      const inv = invert(abs);
      const lx = inv[0] * p[0] + inv[4] * p[1] + inv[8] * p[2] + inv[12], ly = inv[1] * p[0] + inv[5] * p[1] + inv[9] * p[2] + inv[13], lz = inv[2] * p[0] + inv[6] * p[1] + inv[10] * p[2] + inv[14];
      const originD = Math.hypot(lx, ly, lz);
      let d = originD;
      if (box) {
        const dx = Math.max(box.min[0] - lx, 0, lx - box.max[0]), dy = Math.max(box.min[1] - ly, 0, ly - box.max[1]), dz = Math.max(box.min[2] - lz, 0, lz - box.max[2]);
        d = Math.hypot(dx, dy, dz);
      }
      const tol = Number.isFinite(tolParam) && tolParam > 0 ? tolParam : box ? 50 : 300;
      if (d > tol) continue;
      const better = d < bestD - 1e-6 || (Math.abs(d - bestD) <= 1e-6 && ((it.visible && !bestV) || (it.visible === bestV && originD < bestO)));
      if (better) { best = it; bestD = d; bestO = originD; bestV = it.visible; }
    }
    return best;
  }

  /** Local bounding box of an object from its bbox, primitives or mesh bounds (null when unknown). */
  private localBox(o: SceneObject): { min: number[]; max: number[] } | null {
    if (o.bbox) return o.bbox;
    const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
    const geo = o.colliders ?? o.geometry;
    for (const g of geo) {
      let lo: number[] | null = null, hi: number[] | null = null;
      const pr = g.primitive;
      if (pr?.kind === 'box') { lo = pr.size.map((v) => -v / 2); hi = pr.size.map((v) => v / 2); }
      else if (pr?.kind === 'cylinder' || pr?.kind === 'cone') { lo = [-pr.radius, -pr.radius, -pr.length / 2]; hi = [pr.radius, pr.radius, pr.length / 2]; }
      else if (pr?.kind === 'sphere') { lo = [-pr.radius, -pr.radius, -pr.radius]; hi = [pr.radius, pr.radius, pr.radius]; }
      else if (pr?.kind === 'plane') { lo = [-pr.size[0] / 2, -pr.size[1] / 2, -1]; hi = [pr.size[0] / 2, pr.size[1] / 2, 1]; }
      else if (g.mesh && this.assets) { const m = this.assets.get(g.mesh)?.mesh; if (m) { const s = g.scale ?? [1, 1, 1]; lo = m.min.map((v, i) => v * s[i]); hi = m.max.map((v, i) => v * s[i]); } }
      if (!lo || !hi) continue;
      const org = g.origin ?? [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
      for (let k = 0; k < 8; k++) {
        const c = [k & 1 ? hi[0] : lo[0], k & 2 ? hi[1] : lo[1], k & 4 ? hi[2] : lo[2]];
        for (let a = 0; a < 3; a++) {
          const v = org[a] * c[0] + org[4 + a] * c[1] + org[8 + a] * c[2] + org[12 + a];
          min[a] = Math.min(min[a], v); max[a] = Math.max(max[a], v);
        }
      }
    }
    return Number.isFinite(min[0]) ? { min, max } : null;
  }
}

/** Raw parts and their finished twins share a pose (RoboDK machining examples): show/hide sets and reset. */
function setPartVisibility(station: Station, mode: 'show-raw' | 'hide-done' | 'reset') {
  for (const it of station.walk()) {
    if (!(it instanceof SceneObject) || it instanceof Tool) continue;
    const done = /^Part Done\b/i.test(it.name);
    const raw = /^Part \d+$/i.test(it.name);
    if (mode === 'show-raw' && raw) it.setVisible(true);
    else if (mode === 'hide-done' && done) it.setVisible(false);
    else if (mode === 'reset') { if (done) it.setVisible(false); else if (raw) it.setVisible(true); }
  }
}

function sampleTraj(tr: Trajectory, t: number): TrajectorySample {
  const ev = (tr as PlannedTrajectory).eval;
  if (typeof ev === 'function') return ev.call(tr, t);
  const s = tr.samples;
  if (t <= 0 || s.length === 1) return s[0];
  if (t >= tr.duration) return s[s.length - 1];
  let lo = 0, hi = s.length - 1;
  while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (s[mid].t <= t) lo = mid; else hi = mid; }
  const a = s[lo], b = s[hi], f = b.t > a.t ? (t - a.t) / (b.t - a.t) : 1;
  return { t, joints: a.joints.map((v, i) => v + (b.joints[i] - v) * f), pose: b.pose };
}

/** Record a program run (event driven, continue after errors) and restore the station afterwards. */
export function recordProgram(station: Station, program: Program, opts: { startJoints?: Map<string, number[]>; sink?: RuntimeSink; assets?: StationRuntime['assets']; ignoreSpeedInstructions?: boolean } = {}) {
  const rt = new StationRuntime(station, { mode: 'record', continueOnError: true });
  rt.ignoreSpeedInstructions = !!opts.ignoreSpeedInstructions;
  if (opts.sink) rt.sink = opts.sink;
  if (opts.assets) rt.assets = opts.assets;
  const snap = new StationSnapshot(station, opts.startJoints);
  let main: RunHandle | null = null;
  station.batch(() => {
    if (opts.startJoints) for (const it of station.walk()) if (it instanceof Robot && opts.startJoints.has(it.id)) it.setJoints(opts.startJoints.get(it.id)!);
    rt.captureHome(opts.startJoints);
    main = rt.start(program);
    rt.runToCompletion();
    snap.restore();
  });
  const endJoints = new Map<string, number[]>();
  for (const s of rt.recorded) if (s.trajectory?.samples.length && s.robot) endJoints.set(s.robot.id, s.trajectory.samples[s.trajectory.samples.length - 1].joints);
  const mainEnd = (main as RunHandle | null)?.endTime ?? 0;
  const total = Math.max(mainEnd, ...rt.runs.map((h) => h.endTime ?? 0), ...rt.recorded.map((s) => s.t1));
  const updateTime = (main as RunHandle | null)?.estimate ?? 0;
  return { runtime: rt, steps: rt.recorded, problems: rt.problems, duration: mainEnd, totalDuration: total, distance: rt.distance, executed: rt.executed, endJoints, snapshot: snap, updateTime };
}

export { MotionGroup };
