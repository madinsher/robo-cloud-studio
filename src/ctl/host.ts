/**
 * Host layer between the control executors (automata, Petri nets, behavior trees, FSMs) and the station: who runs robot
 * programs, where the signals live, how an arm moves to a target.
 *
 *  - `ProgramBackend` starts programs and reports a real outcome (done / error with the compile message / stopped).
 *    Programs of different robots run concurrently; a second program for a busy robot is queued (FIFO per robot).
 *    Two implementations: `stationRuntimeBackend` adapts the station runtime of interface I1 (src/core/motion/runtime.ts,
 *    one shared clock, RoboDK-like threads) and `perRobotBackend` keeps one ProgramSimulator per robot (used while no
 *    station runtime is available). `programBackendFor` is the single switch between them.
 *  - `SignalTableLike` is the one signal table (I1 SignalTable): robot DI/DO written by programs, process-component
 *    signals, control-model events. Executors merge its snapshot into their blackboard every tick, so guards and sensor
 *    events read it.
 *  - `planArmMove` plans a MoveJ / MoveL to a target with the trajectory module (the executor plays the samples, so a
 *    linear move really is straight).
 */
import { ItemType, Station, Target, Item, Tool } from '../core/items/item';
import type { Program } from '../core/items/program';
import { Robot } from '../core/items/robot';
import { ProgramSimulator } from '../core/motion/simulator';
import { planMoveJ, planMoveL, sampleAt, Trajectory } from '../core/motion/trajectory';
import { multiply, invert } from '../core/math/pose';

export type RunStatus = 'queued' | 'running' | 'done' | 'error' | 'stopped';
/** A program run (subset of the I1 RunHandle). */
export interface ProgramRunHandle { readonly id: string; readonly program: Program; readonly robot: Robot | null; readonly status: RunStatus; readonly error?: { message: string; instructionIndex?: number }; startedAt?: number; endedAt?: number }
/** I1 SignalTable (subset): one table for robot IO, process signals and control-model events. */
export interface SignalTableLike { get(name: string): unknown; set(name: string, v: unknown): void; on?(name: string, cb: (v: unknown) => void): (() => void) | void; snapshot(): Record<string, unknown> }
export interface ProgramBackend {
  readonly kind: 'station-runtime' | 'per-robot-sim' | 'simulated';
  start(p: Program, opts?: { robot?: Robot | null }): ProgramRunHandle;
  stop(h: ProgramRunHandle): void;
  /** Advance the backend clock by dt (no-op when the app steps the backend itself). */
  tick(dt: number): void;
  readonly signals: SignalTableLike;
  /** Runs not finished yet (queued or running). */
  active(): ProgramRunHandle[];
  /** Stop everything this backend started. */
  stopAll(): void;
}

/** The I1 StationRuntime as seen from here (structural, so this file compiles before the runtime lands). */
export interface StationRuntimeLike {
  start(program: Program, opts?: { robot?: Robot; startJoints?: number[]; args?: string[] }): { id: string; program: Program; robot: Robot | null; status: RunStatus; error?: { message: string; instructionIndex: number } };
  tick(dt: number): void;
  stop(id?: string): void;
  signals: SignalTableLike;
  /** shared simulation clock (s), when exposed */
  time?: number;
}

let rid = 0;

/** Signal table over a Map (process-simulator signals), with listeners. `write` lets the owner keep its own side effects (events). */
export function mapSignals(map: Map<string, unknown>, write?: (name: string, v: unknown) => void): SignalTableLike {
  const ls = new Map<string, Set<(v: unknown) => void>>();
  return {
    get: (n) => map.get(n),
    set: (n, v) => { if (write) write(n, v); else map.set(n, v); for (const cb of ls.get(n) ?? []) cb(v); },
    on: (n, cb) => { let s = ls.get(n); if (!s) { s = new Set(); ls.set(n, s); } s.add(cb); return () => { s!.delete(cb); }; },
    snapshot: () => Object.fromEntries(map),
  };
}

/** Adapter for the I1 station runtime. `drive`: tick the runtime from the executor (false when the app already steps it). */
export function stationRuntimeBackend(rt: StationRuntimeLike, opts: { drive?: boolean } = {}): ProgramBackend {
  const runs = new Set<ProgramRunHandle>();
  let lastTime = rt.time;
  return {
    kind: 'station-runtime', signals: rt.signals,
    start: (p, o = {}) => { const h = rt.start(p, o.robot ? { robot: o.robot } : {}); runs.add(h); return h; },
    stop: (h) => { if (h.status === 'queued' || h.status === 'running') rt.stop(h.id); },
    tick: (dt) => {
      // drive the runtime unless something else advanced its clock since our last tick
      const external = rt.time !== undefined && lastTime !== undefined && rt.time !== lastTime;
      if (opts.drive !== false && !external) rt.tick(dt);
      lastTime = rt.time;
      for (const h of runs) if (h.status !== 'queued' && h.status !== 'running') runs.delete(h);
    },
    active: () => [...runs].filter((h) => h.status === 'queued' || h.status === 'running'),
    stopAll: () => { for (const h of runs) if (h.status === 'queued' || h.status === 'running') rt.stop(h.id); },
  };
}

interface SimRun extends ProgramRunHandle { status: RunStatus; error?: { message: string; instructionIndex?: number }; sim?: ProgramSimulator; lane: string }

/**
 * One ProgramSimulator per robot: programs on different robots play concurrently on the executor clock; a program for a
 * busy robot waits in a FIFO. A program that does not compile ends with status `error` and the compiler's message.
 * Stopping pauses the program where it is (it never rewinds the station, other robots keep their state). Robot DO set
 * by a program are written into the signal table.
 */
export function perRobotBackend(station: Station, opts: { signals?: SignalTableLike; collisions?: boolean; log?: (m: string, level?: 'info' | 'warn' | 'error') => void; speed?: () => number } = {}): ProgramBackend {
  const signals = opts.signals ?? mapSignals(new Map());
  const busy = new Map<string, SimRun>(); const queue = new Map<string, SimRun[]>(); const all = new Set<SimRun>();
  let clock = 0;
  const launch = (h: SimRun) => {
    const sim = new ProgramSimulator(station); sim.collisionOptions = { enabled: !!opts.collisions, sampleStep: 0.05 };
    const res = sim.compile(h.program);
    const err = res.problems.find((p) => p.severity === 'error');
    if (!res.ok) {
      const idx = err ? h.program.instructions().findIndex((i) => i.id === err.instructionId) : -1;
      h.status = 'error'; h.error = { message: err?.message ?? 'program does not compile', instructionIndex: idx >= 0 ? idx : undefined }; h.endedAt = clock;
      opts.log?.(`${h.program.name}: ${h.error.message}`, 'error'); next(h.lane); return;
    }
    sim.events.on('io', (e) => signals.set(e.name, e.value));
    // objects held by this robot's tools (attached by an earlier program) keep following the tool
    if (h.robot) for (const tl of station.itemsOfType<Tool>(ItemType.TOOL)) {
      if (tl.parent !== h.robot || !tl.attached.length) continue;
      const toolAbs = multiply(h.robot.poseAbs(), h.robot.solveFK(h.robot.joints(), tl.poseTool()));
      for (const id of tl.attached) { const obj = station.findById(id); if (obj) sim.attachments.set(id, { tool: tl, local: multiply(invert(toolAbs), obj.poseAbs()) }); }
    }
    h.sim = sim; h.status = 'running'; h.startedAt = clock; busy.set(h.lane, h);
    sim.speedFactor = 1; sim.play();
    if (!sim.playing) { h.status = 'done'; h.endedAt = clock; busy.delete(h.lane); next(h.lane); }
  };
  const next = (lane: string) => { const q = queue.get(lane); const h = q?.shift(); if (h) launch(h); };
  return {
    kind: 'per-robot-sim', signals,
    start: (p, o = {}) => {
      const robot = (o.robot ?? (p.robot() instanceof Robot ? (p.robot() as Robot) : null));
      const h: SimRun = { id: `run${++rid}`, program: p, robot, status: 'queued', lane: robot?.id ?? `prog:${p.id}` };
      all.add(h);
      if (busy.has(h.lane)) { let q = queue.get(h.lane); if (!q) { q = []; queue.set(h.lane, q); } q.push(h); } else launch(h);
      return h;
    },
    stop: (h0) => {
      const h = h0 as SimRun;
      if (h.status === 'queued') { const q = queue.get(h.lane); if (q) q.splice(q.indexOf(h), 1); h.status = 'stopped'; h.endedAt = clock; return; }
      if (h.status === 'running') { h.sim?.pause(); h.status = 'stopped'; h.endedAt = clock; busy.delete(h.lane); next(h.lane); }
    },
    tick: (dt) => {
      clock += dt; const k = opts.speed?.() ?? 1;
      for (const h of [...busy.values()]) {
        if (!h.sim) continue;
        h.sim.speedFactor = k;
        if (!h.sim.tick(dt)) { h.status = 'done'; h.endedAt = clock; busy.delete(h.lane); next(h.lane); }
      }
      for (const h of all) if (h.status !== 'queued' && h.status !== 'running') all.delete(h);
    },
    active: () => [...all].filter((h) => h.status === 'queued' || h.status === 'running'),
    stopAll: () => { for (const q of queue.values()) for (const h of q.splice(0)) h.status = 'stopped'; for (const h of [...busy.values()]) { h.sim?.pause(); h.status = 'stopped'; busy.delete(h.lane); } all.clear(); },
  };
}

/** Stand-in backend for tests and headless scenarios: program durations by name, same per-robot lanes and outcomes. */
export function simulatedBackend(durations: Record<string, number>, opts: { robotOf?: (p: Program) => string; fail?: string[] } = {}): ProgramBackend & { log: Array<{ t: number; program: string; event: 'start' | 'end' | 'error' | 'stop' }>; time: number } {
  const busy = new Map<string, { h: SimRun; end: number }>(); const queue = new Map<string, SimRun[]>(); const all = new Set<SimRun>();
  const out = { log: [] as Array<{ t: number; program: string; event: 'start' | 'end' | 'error' | 'stop' }>, time: 0 };
  const sig = mapSignals(new Map());
  const launch = (h: SimRun) => {
    const d = durations[h.program.name];
    if (d === undefined || opts.fail?.includes(h.program.name)) { h.status = 'error'; h.error = { message: d === undefined ? `no duration for ${h.program.name}` : `${h.program.name} failed` }; out.log.push({ t: out.time, program: h.program.name, event: 'error' }); const q = queue.get(h.lane)?.shift(); if (q) launch(q); return; }
    h.status = 'running'; h.startedAt = out.time; busy.set(h.lane, { h, end: out.time + d }); out.log.push({ t: out.time, program: h.program.name, event: 'start' });
  };
  return Object.assign(out, {
    kind: 'simulated' as const, signals: sig,
    start: (p: Program, o: { robot?: Robot | null } = {}) => {
      const lane = o.robot?.id ?? opts.robotOf?.(p) ?? (p.robot()?.id ?? `prog:${p.id}`);
      const h: SimRun = { id: `run${++rid}`, program: p, robot: o.robot ?? null, status: 'queued', lane }; all.add(h);
      if (busy.has(lane)) { let q = queue.get(lane); if (!q) { q = []; queue.set(lane, q); } q.push(h); } else launch(h);
      return h as ProgramRunHandle;
    },
    stop: (h0: ProgramRunHandle) => { const h = h0 as SimRun; if (h.status === 'queued') { const q = queue.get(h.lane); if (q) q.splice(q.indexOf(h), 1); h.status = 'stopped'; } else if (h.status === 'running') { busy.delete(h.lane); h.status = 'stopped'; out.log.push({ t: out.time, program: h.program.name, event: 'stop' }); const q = queue.get(h.lane)?.shift(); if (q) launch(q); } },
    tick: (dt: number) => {
      out.time += dt;
      for (const [lane, b] of [...busy]) if (out.time >= b.end - 1e-9) { b.h.status = 'done'; b.h.endedAt = out.time; busy.delete(lane); out.log.push({ t: out.time, program: b.h.program.name, event: 'end' }); const q = queue.get(lane)?.shift(); if (q) launch(q); }
    },
    active: () => [...all].filter((h) => h.status === 'queued' || h.status === 'running') as ProgramRunHandle[],
    stopAll: () => { busy.clear(); queue.clear(); },
  });
}

// ---------------------------------------------------------------------------------------------
// The switch: station runtime (I1) when present, per-robot simulators otherwise
// ---------------------------------------------------------------------------------------------

type BackendFactory = (station: Station) => ProgramBackend | null;
let factory: BackendFactory | null = null;
/** Let the station runtime (I1) or the app provide the program backend for the control executors. */
export function registerProgramBackendFactory(f: BackendFactory | null): void { factory = f; }
/**
 * The program backend for a station: a registered factory, else a station runtime found on `owner` (`runtime` /
 * `stationRuntime` property implementing I1), else one ProgramSimulator per robot.
 */
export function programBackendFor(station: Station, owner?: unknown, fallback: Parameters<typeof perRobotBackend>[1] = {}): ProgramBackend {
  const viaFactory = factory?.(station); if (viaFactory) return viaFactory;
  const o = owner as Record<string, unknown> | undefined;
  const rt = (o?.runtime ?? o?.stationRuntime) as StationRuntimeLike | undefined;
  if (rt && typeof rt.start === 'function' && typeof rt.tick === 'function' && rt.signals) return stationRuntimeBackend(rt);
  return perRobotBackend(station, fallback);
}

// ---------------------------------------------------------------------------------------------
// Arm moves to targets
// ---------------------------------------------------------------------------------------------

/** The robot a target belongs to: the explicit name, the target's own robot link, a robot parent, else the only robot of the station (null when ambiguous). */
export function robotForTarget(station: Station, tgt: Target, name?: unknown): { robot: Robot | null; why: string } {
  const robots = station.itemsOfType<Robot>(ItemType.ROBOT);
  if (typeof name === 'string' && name) { const r = robots.find((x) => x.name === name) ?? robots.find((x) => x.name.toLowerCase() === name.toLowerCase()); return { robot: r ?? null, why: r ? 'robot=' : `robot ${name} not found` }; }
  if (tgt.robotId) { const r = station.findById(tgt.robotId); if (r instanceof Robot) return { robot: r, why: 'target link' }; }
  for (let p: Item | null = tgt.parent as Item | null; p; p = p.parent as Item | null) if (p instanceof Robot) return { robot: p, why: 'target parent' };
  // a target under a frame used by exactly one robot
  const users = robots.filter((r) => r.activeFrameId && (tgt.parent as Item | null)?.id === r.activeFrameId);
  if (users.length === 1) return { robot: users[0], why: 'frame of the robot' };
  if (robots.length === 1) return { robot: robots[0], why: 'only robot' };
  return { robot: null, why: `target ${tgt.name} is not linked to a robot and the station has ${robots.length} robots — set robot=` };
}

/** Plan a MoveJ / MoveL from the robot's current joints to a target with the robot's motion parameters. */
export function planArmMove(robot: Robot, tgt: Target, linear: boolean): { traj: Trajectory | null; error?: string } {
  const q0 = robot.joints();
  const q1 = robot.jointsForTarget(tgt);
  if (!q1) return { traj: null, error: `${tgt.name} is unreachable for ${robot.name}` };
  let tr: Trajectory;
  if (linear) {
    // TCP pose of the target relative to the robot base, same tool as the robot's active tool
    const pose = tgt.isJointTarget ? robot.solveFK(q1) : multiply(invert(robot.poseAbs()), tgt.poseAbs());
    tr = planMoveL(robot, q0, pose, robot.motion.speedLinear, robot.motion.accelLinear);
  } else tr = planMoveJ(robot, q0, q1, robot.motion.speedJoints, robot.motion.accelJoints);
  if (!tr.ok) return { traj: null, error: tr.error ?? `cannot plan the move to ${tgt.name}` };
  return { traj: tr };
}
export { sampleAt };

/**
 * Objects held by a tool (Tool.attached) follow it between programs, e.g. while the mobile base of a mobile manipulator
 * drives: the relative pose seen the first time an object is held is kept. Call once per world step.
 */
export function carryAttachedObjects(station: Station, memo: Map<string, { tool: string; local: ReturnType<typeof invert> }>): void {
  const held = new Set<string>();
  for (const tl of station.itemsOfType<Tool>(ItemType.TOOL)) {
    const robot = tl.parent instanceof Robot ? tl.parent : null; if (!robot || !tl.attached.length) continue;
    const toolAbs = multiply(robot.poseAbs(), robot.solveFK(robot.joints(), tl.poseTool()));
    for (const id of tl.attached) {
      const obj = station.findById(id); if (!obj) continue; held.add(id);
      const m = memo.get(id);
      if (!m || m.tool !== tl.id) memo.set(id, { tool: tl.id, local: multiply(invert(toolAbs), obj.poseAbs()) });
      else obj.setPoseAbs(multiply(toolAbs, m.local));
    }
  }
  for (const id of [...memo.keys()]) if (!held.has(id)) memo.delete(id);
}
