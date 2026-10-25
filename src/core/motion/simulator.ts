import './mobile_hook';
import { Station, Item, Tool, SceneObject } from '../items/item';
import { Robot } from '../items/robot';
import { Program, Instruction, ProgramRunResult } from '../items/program';
import { Trajectory, sampleAt } from './trajectory';
import { EventBus } from '../events';
import { CollisionPair, CollisionOptions, Collider, robotCollisionChecker } from '../collision/collision';
import { recordProgram, RecordedStep, StationSnapshot, StationRuntime } from './runtime';
import type { MotionGroup } from './planner';
import type { Pose2D } from './mobile_hook';

export { setMobileMotionPlanner, getMobileMotionPlanner, straightLinePlanner } from './mobile_hook';
export type { MobileMotionPlanner, MobileMotionPlan, Pose2D } from './mobile_hook';

export interface SimEvents extends Record<string, unknown> {
  instruction: { program: Program; instruction: Instruction; index: number };
  message: { text: string; level: 'info' | 'warn' | 'error' };
  finished: { program: Program; result: ProgramRunResult };
  io: { name: string; value: number | boolean };
  tick: { t: number };
}

export interface Step {
  instruction: Instruction;
  robot: Robot | null;
  trajectory?: Trajectory;
  /** Arm + synchronised external axes moved by the trajectory (joint vectors are [arm, ext...]). */
  group?: MotionGroup;
  /** Mobile-base motion (Navigate / Follow path). */
  mobile?: { item: Item; sample: (t: number) => Pose2D; duration: number };
  /** Absolute time at which this step starts / ends (s). */
  t0: number;
  t1: number;
  apply?: () => void;
}

/**
 * Program simulator: compiles a Program into timed steps with the station runtime's interpreter (record mode: RoboDK
 * semantics, blending, external axes, threads and signals, continue after errors to list every problem) and replays
 * them on a clock. Used for validation, previews, cycle times and timeline scrubbing; live playback of several
 * programs runs on StationRuntime.
 */
export class ProgramSimulator {
  readonly events = new EventBus<SimEvents>();
  steps: Step[] = [];
  result: ProgramRunResult | null = null;
  duration = 0;
  time = 0;
  playing = false;
  speedFactor = 1;
  /** Time of the first error: playback stops there (RoboDK plays up to the failing instruction). */
  errorTime: number | null = null;
  io: Record<string, number | boolean> = {};
  signals: Record<string, number | boolean | string> = {};
  /** Object -> tool attachment (object id -> tool and pose relative to the tool) during simulation. */
  attachments = new Map<string, { tool: Tool; local: Float64Array }>();
  /** Collision checking during compile (sampled along trajectories). */
  collisionOptions: (CollisionOptions & { enabled: boolean; sampleStep?: number; /** Joint motion (deg / mm) between samples (default 2). */ jointStep?: number; /** Sample budget of one check (default 20000; the joint step grows for longer programs). */ maxSamples?: number; /** Colliding instructions listed before the check stops (default 20). */ maxCollisions?: number; /** Wall-clock budget of the check (ms, default 60000). */ timeBudgetMs?: number }) = { enabled: false, sampleStep: 0.02 };
  collisions: Array<{ instructionId: string; t: number; pairs: CollisionPair[] }> = [];
  /**
   * 'ignore' reproduces RoboDK's Item.Update() cycle-time estimate (Set speed instructions are not applied there;
   * playback honours them). Default 'apply'.
   */
  speedInstructions: 'apply' | 'ignore' = 'apply';
  /** Asset store for mesh bounds (attach distance); collision uses collisionOptions.assets. */
  assets: StationRuntime['assets'] = null;
  private currentStep = -1;
  private initialIO: Record<string, number | boolean> = {};
  private initialSignals: Record<string, number | boolean | string> = {};
  private snapshot: StationSnapshot | null = null;
  private recRuntime: StationRuntime | null = null;
  private robotEndJoints = new Map<string, number[]>();
  private replaying = false;
  private lastMotion = new Map<string, Step>();
  private version = 0;

  constructor(readonly station: Station) {}

  /** Compile a program into steps. Returns the validation result. */
  compile(program: Program, startJoints?: Map<string, number[]>): ProgramRunResult {
    this.playing = false;
    this.steps = [];
    this.attachments.clear();
    this.collisions = [];
    this.errorTime = null;
    this.initialIO = { ...this.io };
    this.initialSignals = { ...this.signals };
    const sink = {
      io: (name: string, value: number | boolean | string) => { if (!this.replaying) return; this.io[name] = value as number | boolean; this.events.emit('io', { name, value: value as number | boolean }); },
      signal: (name: string, value: number | boolean | string) => { if (this.replaying) this.signals[name] = value; },
      message: (text: string, level: 'info' | 'warn' | 'error') => { if (this.replaying) this.events.emit('message', { text, level }); },
      attach: (id: string, tool: Tool) => { if (!this.replaying) return; const o = this.station.findById(id); if (o) this.attachments.set(id, { tool, local: o.pose() }); this.version++; },
      detach: (id: string) => { if (this.replaying) { this.attachments.delete(id); this.version++; } },
    };
    const rec = recordProgram(this.station, program, { startJoints, sink, assets: this.assets ?? (this.collisionOptions.assets as any) ?? null, ignoreSpeedInstructions: this.speedInstructions === 'ignore' });
    this.recRuntime = rec.runtime;
    this.snapshot = rec.snapshot;
    this.steps = rec.steps.map((s: RecordedStep): Step => {
      const st: Step = { instruction: s.instruction, robot: s.robot, t0: s.t0, t1: s.t1 };
      if (s.trajectory) st.trajectory = s.trajectory;
      if (s.group) st.group = s.group;
      if (s.mobile) st.mobile = s.mobile;
      if (s.apply) st.apply = s.apply;
      return st;
    });
    // stable sort by start time (threads interleave)
    this.steps = this.steps.map((s, i) => [s, i] as const).sort((a, b) => a[0].t0 - b[0].t0 || a[1] - b[1]).map(([s]) => s);
    const problems: ProgramRunResult['problems'] = rec.problems.map((p) => ({ instructionId: p.instructionId, message: p.message, severity: p.severity }));
    const firstRecErr = rec.problems.find((p) => p.severity === 'error');
    this.duration = rec.totalDuration;
    this.robotEndJoints = rec.endJoints;
    this.time = 0;
    this.currentStep = -1;
    this.lastMotion.clear();
    if (this.collisionOptions.enabled) this.checkTrajectoryCollisions(problems);
    const firstErr = problems.find((p) => p.severity === 'error');
    if (firstErr) {
      const tCol = this.collisions.length ? Math.min(...this.collisions.map((x) => x.t)) : Infinity;
      this.errorTime = Math.min(tCol, firstRecErr?.t ?? Infinity);
      if (!Number.isFinite(this.errorTime)) this.errorTime = 0;
    }
    this.result = { ok: !firstErr, duration: rec.duration, distance: rec.distance, problems, executed: rec.executed };
    program.lastResult = this.result;
    return this.result;
  }

  private restoreInitial(): void {
    this.snapshot?.restore();
    this.recRuntime?.resetAttachState();
    this.attachments.clear();
    this.io = { ...this.initialIO };
    this.signals = { ...this.initialSignals };
    this.currentStep = -1;
    this.lastMotion.clear();
    this.version++;
  }

  private applyMotion(s: Step, at: number): void {
    const local = Math.max(0, at - s.t0);
    if (s.trajectory && s.trajectory.samples.length) {
      const tr = s.trajectory;
      const q = sampleAt(tr, Math.min(local, tr.duration)).joints;
      if (s.group) s.group.apply(q);
      else s.robot?.setJoints(q);
      const r = s.group?.robot ?? s.robot;
      if (r) { r.state.moving = local < tr.duration; r.state.progress = tr.duration > 0 ? Math.min(1, local / tr.duration) : 1; }
    } else if (s.mobile) {
      const p = s.mobile.sample(Math.min(local, s.mobile.duration));
      const m = s.mobile.item as any;
      if (typeof m.setPose2D === 'function') m.setPose2D(p.x, p.y, p.heading);
    }
  }
  private motionKey(s: Step): string | null {
    if (s.trajectory && s.trajectory.samples.length) return (s.group?.robot ?? s.robot)?.id ?? null;
    if (s.mobile) return s.mobile.item.id;
    return null;
  }

  /** Seek the simulation to absolute time t (applies robot joints and events up to t). */
  seek(t: number): void {
    t = Math.max(0, Math.min(this.duration, t));
    let stepIdx = this.stepIndexAt(t);
    const rewinding = t < this.time || stepIdx < this.currentStep;
    this.time = t;
    this.replaying = true;
    try {
      if (rewinding) this.restoreInitial();
      for (let i = this.currentStep + 1; i <= stepIdx; i++) {
        const s = this.steps[i];
        for (const m of this.lastMotion.values()) this.applyMotion(m, s.t0);
        const key = this.motionKey(s);
        if (key) this.lastMotion.set(key, s);
        if (s.apply) { s.apply(); this.version++; }
        this.events.emit('instruction', { program: s.instruction.parent as Program, instruction: s.instruction, index: i });
      }
      this.currentStep = stepIdx;
      for (const m of this.lastMotion.values()) this.applyMotion(m, t);
    } finally {
      this.replaying = false;
    }
    void stepIdx;
    this.events.emit('tick', { t });
  }

  /** Advance the simulation by dt seconds (real time * speedFactor). */
  tick(dt: number): boolean {
    if (!this.playing) return false;
    const end = this.playEnd();
    const nt = Math.min(end, this.time + dt * this.speedFactor);
    this.seek(nt);
    if (nt >= end) {
      this.playing = false;
      for (const s of this.steps) { const r = s.group?.robot ?? s.robot; if (r) r.state.moving = false; }
      return false;
    }
    return true;
  }
  private playEnd(): number { return this.errorTime ?? this.duration; }

  /** Play from the current time; a program with errors plays up to its first error. */
  play(): void {
    const end = this.playEnd();
    if (this.time >= end) this.seek(0);
    this.playing = end > 0 && this.time < end;
  }
  pause(): void {
    this.playing = false;
  }
  stop(): void {
    this.playing = false;
    this.seek(0);
  }
  /** Jump to the end (apply final state). */
  runToEnd(): void {
    this.seek(this.duration);
  }
  endJoints(robotId: string): number[] | undefined {
    return this.robotEndJoints.get(robotId);
  }
  /**
   * Joint list along the whole program (RoboDK InstructionListJoints): rows of
   * [time, instruction index, j1..jn] sampled every `dt` seconds for the robot (external axes appended).
   */
  jointsList(robotId: string, dt = 0.05): number[][] {
    const rows: number[][] = [];
    let last: number[] | null = null;
    this.steps.forEach((s, idx) => {
      const r = s.group?.robot ?? s.robot;
      if (r?.id !== robotId || !s.trajectory || !s.trajectory.samples.length) return;
      const tr = s.trajectory;
      const n = Math.max(1, Math.ceil(tr.duration / dt));
      for (let k = 0; k <= n; k++) {
        const t = (k / n) * tr.duration;
        const q = sampleAt(tr, t).joints;
        if (last && k === 0 && last.every((v, i) => Math.abs(v - q[i]) < 1e-9)) continue;
        rows.push([s.t0 + t, idx, ...q]);
        last = q;
      }
    });
    return rows;
  }

  stepIndexAt(t: number): number {
    let lo = 0, hi = this.steps.length - 1, idx = -1;
    while (lo <= hi) { const mid = (lo + hi) >> 1; if (this.steps[mid].t0 <= t) { idx = mid; lo = mid + 1; } else hi = mid - 1; }
    return idx;
  }

  /** Sample the timeline and report colliding instructions (moving robot vs station, honouring the collision map). */
  private checkTrajectoryCollisions(problems: ProgramRunResult['problems']): void {
    const step = Math.max(1e-3, this.collisionOptions.sampleStep ?? 0.1);
    const opts = this.collisionOptions;
    const motionSteps = this.steps.filter((s) => s.robot && s.trajectory && s.trajectory.samples.length);
    if (!motionSteps.length) return;
    const robots = [...new Set(motionSteps.map((s) => s.group?.robot ?? s.robot!))];
    const checkers = new Map(robots.map((r) => [r, robotCollisionChecker(this.station, r, opts)]));
    const keyOf = (p: CollisionPair) => `${p.a.item.id}/${p.a.part}|${p.b.item.id}/${p.b.part}`;
    const initial = new Map<string, number>();
    const reported = new Set<string>();
    this.seek(0);
    for (const r of robots) {
      for (const p of checkers.get(r)!.check(this.version)) {
        if (p.a.item === p.b.item || (p.a.item instanceof Tool && p.a.item.parent === p.b.item) || (p.b.item instanceof Tool && p.b.item.parent === p.a.item)) continue;
        const key = keyOf(p);
        initial.set(key, Math.max(initial.get(key) ?? 0, p.depth));
        if (reported.has(key)) continue;
        reported.add(key);
        problems.push({ instructionId: '', message: `Initial contact ignored: ${p.a.item.name} × ${p.b.item.name} (${p.depth.toFixed(0)} mm)`, severity: 'warning' });
      }
    }
    // sample times adapted to the motion: a sample whenever a joint moved `jointStep` deg (mm for prismatic axes) since
    // the last sample of that robot, at most every `sampleStep` s, plus every move end. Dense machining programs with
    // thousands of sub-millimetre moves get one sample per few moves instead of several per move.
    const dmax = (a: number[], b: number[]) => { let m = 0; for (let i = 0; i < Math.min(a.length, b.length); i++) m = Math.max(m, Math.abs(a[i] - b[i])); return m; };
    // very long programs (15k-move 3D printing paths): coarser joint step so the check stays within maxSamples
    let travel = 0;
    for (const s of motionSteps) { const ks = s.trajectory!.samples; for (let k = 1; k < ks.length; k++) travel += dmax(ks[k - 1].joints, ks[k].joints); }
    const jStep = Math.max(0.1, this.collisionOptions.jointStep ?? 2, travel / Math.max(1000, this.collisionOptions.maxSamples ?? 20000));
    const samples: Array<{ t: number; s: Step }> = [];
    const lastQ = new Map<Robot, number[]>();
    for (const s of motionSteps) {
      const tr = s.trajectory!, r = s.group?.robot ?? s.robot!;
      const ks = tr.samples;
      if (!lastQ.has(r)) samples.push({ t: s.t0, s }); // the start posture of every robot is checked too
      let prev = lastQ.get(r) ?? ks[0].joints;
      for (let k = 1; k < ks.length; k++) {
        const a = ks[k - 1], b = ks[k];
        const n = Math.min(400, Math.max(dmax(prev, b.joints) >= jStep ? 1 : 0, Math.ceil(dmax(a.joints, b.joints) / jStep), Math.ceil((b.t - a.t) / Math.max(step, 0.2))));
        for (let i = 1; i <= n; i++) samples.push({ t: s.t0 + a.t + ((b.t - a.t) * i) / n, s });
        if (n) prev = b.joints;
      }
      if (dmax(prev, ks[ks.length - 1].joints) > 1e-6) { samples.push({ t: s.t0 + tr.duration, s }); prev = ks[ks.length - 1].joints; }
      lastQ.set(r, prev);
    }
    samples.sort((a, b) => a.t - b.t);
    // mesh pairs touching at the start have a boolean depth: they can never be reported later, so do not re-test them
    const initialMesh = new Set<string>();
    for (const r of robots) for (const p of checkers.get(r)!.check(this.version)) if (initial.has(keyOf(p)) && (p.a.shape.kind === 'mesh' || p.b.shape.kind === 'mesh')) initialMesh.add(keyOf(p));
    const skipInitial = initialMesh.size ? (a: Collider, b: Collider) => initialMesh.has(`${a.item.id}/${a.part}|${b.item.id}/${b.part}`) || initialMesh.has(`${b.item.id}/${b.part}|${a.item.id}/${a.part}`) : undefined;
    const hit = new Set<string>();
    // RoboDK reports the first collision (the program is valid up to it): list a few, then stop; and bound the time
    const maxHits = Math.max(1, opts.maxCollisions ?? 20), deadline = performance.now() + Math.max(1000, opts.timeBudgetMs ?? 60000);
    for (const { t, s } of samples) {
      if (hit.has(s.instruction.id)) continue;
      if (hit.size >= maxHits) { problems.push({ instructionId: '', message: `Collision check stopped after ${maxHits} colliding instructions`, severity: 'warning' }); break; }
      if (performance.now() > deadline) { problems.push({ instructionId: s.instruction.id, message: `Collision check stopped at ${t.toFixed(1)} s (time budget ${(opts.timeBudgetMs ?? 60000) / 1000} s): the rest of the program was not checked`, severity: 'warning' }); break; }
      this.seek(t);
      const r = s.group?.robot ?? s.robot!;
      const pairs = checkers.get(r)!.check(this.version, skipInitial).filter((p) => p.depth > (initial.get(keyOf(p)) ?? -Infinity) + 0.5);
      if (!pairs.length) continue;
      hit.add(s.instruction.id);
      this.collisions.push({ instructionId: s.instruction.id, t, pairs });
      const p = pairs[0];
      problems.push({ instructionId: s.instruction.id, message: `Collision: ${p.a.item.name}${p.a.part !== p.a.item.name ? '/' + p.a.part : ''} × ${p.b.item.name}${p.b.part !== p.b.item.name ? '/' + p.b.part : ''} at ${t.toFixed(2)} s`, severity: 'error' });
    }
    this.seek(0);
    this.restoreInitial();
    this.time = 0;
  }
}

/**
 * Convenience: estimate cycle time & validity without touching robot state. `roboDKUpdate` reproduces RoboDK's
 * Item.Update() numbers (programmed Set speed instructions ignored, see ProgramSimulator.speedInstructions).
 */
export function estimateProgram(station: Station, program: Program, opts: { roboDKUpdate?: boolean } = {}): ProgramRunResult {
  const sim = new ProgramSimulator(station);
  if (opts.roboDKUpdate) sim.speedInstructions = 'ignore';
  return sim.compile(program);
}

export { Item, SceneObject };
