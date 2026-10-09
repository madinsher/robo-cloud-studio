import { Station, Item, Target, Tool, SceneObject, ItemType } from '../items/item';
import { Robot } from '../items/robot';
import { Program, Instruction, ProgramRunResult, MoveInstruction } from '../items/program';
import { Mat4, fromArray, multiply, invert, identity } from '../math/pose';
import { planMoveJ, planMoveL, planMoveC, Trajectory, TrajectorySample, sampleAt } from './trajectory';
import { EventBus } from '../events';
import { checkRobotCollisions, CollisionPair, CollisionOptions } from '../collision/collision';

export interface SimEvents extends Record<string, unknown> {
  instruction: { program: Program; instruction: Instruction; index: number };
  message: { text: string; level: 'info' | 'warn' | 'error' };
  finished: { program: Program; result: ProgramRunResult };
  io: { name: string; value: number | boolean };
  tick: { t: number };
}

interface Step {
  instruction: Instruction;
  robot: Robot | null;
  trajectory?: Trajectory;
  /** Absolute time at which this step starts / ends (s). */
  t0: number;
  t1: number;
  apply?: () => void;
}

/**
 * Program simulator: compiles a Program into timed steps (trajectories + events) and plays them
 * on a simulation clock. Also used headless for cycle-time estimation and validation.
 */
export class ProgramSimulator {
  readonly events = new EventBus<SimEvents>();
  steps: Step[] = [];
  result: ProgramRunResult | null = null;
  duration = 0;
  time = 0;
  playing = false;
  speedFactor = 1;
  private currentStep = -1;
  io: Record<string, number | boolean> = {};
  signals: Record<string, number | boolean | string> = {};
  /** Object -> tool attachment (object id -> tool id) during simulation. */
  attachments = new Map<string, { tool: Tool; local: Mat4 }>();
  /** Joint state at program end for each robot. */
  private robotEndJoints = new Map<string, number[]>();
  /** Collision checking during compile (sampled along trajectories). */
  collisionOptions: (CollisionOptions & { enabled: boolean; sampleStep?: number }) = { enabled: false, sampleStep: 0.1 };
  collisions: Array<{ instructionId: string; t: number; pairs: CollisionPair[] }> = [];
  /** End times of parallel threads (the total duration covers them). */
  private threadEnds: number[] = [];

  constructor(readonly station: Station) {}

  /** Compile a program into steps. Returns validation result. */
  compile(program: Program, startJoints?: Map<string, number[]>): ProgramRunResult {
    this.steps = [];
    this.attachments.clear();
    this.threadEnds = [];
    const problems: ProgramRunResult['problems'] = [];
    const jointsOf = new Map<string, number[]>(startJoints ?? []);
    let t = 0;
    let distance = 0;
    let executed = 0;
    const visited = new Set<string>();

    const compileProgram = (prog: Program, depth: number) => {
      if (depth > 32) {
        problems.push({ instructionId: prog.id, message: 'Recursion too deep', severity: 'error' });
        return;
      }
      if (visited.has(prog.id) && depth > 0) {
        problems.push({ instructionId: prog.id, message: `Recursive call to ${prog.name}`, severity: 'error' });
        return;
      }
      visited.add(prog.id);
      const robotItem = prog.robot();
      const robot = robotItem instanceof Robot ? robotItem : null;
      let speed = robot ? { ...robot.motion } : { speedLinear: 500, speedJoints: 90, accelLinear: 2000, accelJoints: 360, rounding: -1 };
      let frameId = prog.frameId ?? robot?.activeFrameId ?? null;
      let toolId = prog.toolId ?? robot?.activeToolId ?? null;
      const framePoseInBase = (): Mat4 => {
        if (!robot) return identity();
        const f = frameId ? this.station.findById(frameId) : robot.activeFrame();
        if (!f) return identity();
        return multiply(invert(robot.poseAbs()), f.poseAbs());
      };
      const toolPose = (): Mat4 => {
        if (!robot) return identity();
        const tl = toolId ? this.station.findById(toolId) : robot.activeTool();
        return tl instanceof Tool ? tl.poseTool() : identity();
      };

      for (const ins of prog.instructions()) {
        if (!ins.enabled) continue;
        executed++;
        const d = ins.data;
        const t0 = t;
        switch (d.kind) {
          case 'move': {
            if (!robot) {
              problems.push({ instructionId: ins.id, message: 'Program has no robot', severity: 'error' });
              break;
            }
            const q0 = jointsOf.get(robot.id) ?? robot.joints();
            const target = d.targetId ? (this.station.findById(d.targetId) as Target | null) : null;
            if (d.targetId && !target) {
              problems.push({ instructionId: ins.id, message: 'Target not found', severity: 'error' });
              break;
            }
            const tool = toolPose();
            const frame = framePoseInBase();
            const speedL = d.speed ?? speed.speedLinear;
            let traj: Trajectory | null = null;
            if (d.moveType === 'MoveJ') {
              let q1: number[] | null = null;
              if (target) {
                if (target.isJointTarget && target.joints) q1 = target.joints;
                else {
                  const pInBase = multiply(invert(robot.poseAbs()), target.poseAbs());
                  const r = robot.solveIK(pInBase, { seed: target.joints ?? q0 }, tool);
                  q1 = r.ok ? r.joints : null;
                }
              } else if (d.joints) q1 = d.joints;
              else if (d.pose) {
                const r = robot.solveIK(multiply(frame, fromArray(d.pose)), { seed: q0 }, tool);
                q1 = r.ok ? r.joints : null;
              }
              if (!q1) {
                problems.push({ instructionId: ins.id, message: `${target?.name ?? 'pose'} unreachable`, severity: 'error' });
                break;
              }
              traj = planMoveJ(robot, q0, q1, speed.speedJoints, speed.accelJoints);
            } else {
              const p1 = this.resolvePose(robot, target, d, frame, tool, q0);
              if (!p1) {
                problems.push({ instructionId: ins.id, message: `${target?.name ?? 'pose'} could not be resolved`, severity: 'error' });
                break;
              }
              if (d.moveType === 'MoveC') {
                const via = d.viaTargetId ? (this.station.findById(d.viaTargetId) as Target | null) : null;
                const pVia = via ? this.resolvePose(robot, via, { ...d, pose: undefined }, frame, tool, q0) : d.viaPose ? multiply(frame, fromArray(d.viaPose)) : null;
                if (!pVia) {
                  problems.push({ instructionId: ins.id, message: 'MoveC via point missing', severity: 'error' });
                  break;
                }
                traj = planMoveC(robot, q0, pVia, p1, speedL, speed.accelLinear);
              } else {
                traj = planMoveL(robot, q0, p1, speedL, speed.accelLinear, 0.02, speed.speedJoints);
              }
            }
            if (!traj.ok) problems.push({ instructionId: ins.id, message: traj.error ?? 'motion failed', severity: 'error' });
            if (traj.samples.length) {
              jointsOf.set(robot.id, traj.samples[traj.samples.length - 1].joints);
              t += traj.duration;
              distance += traj.length;
            }
            this.steps.push({ instruction: ins, robot, trajectory: traj, t0, t1: t });
            break;
          }
          case 'jointPath': {
            if (!robot) { problems.push({ instructionId: ins.id, message: 'Program has no robot', severity: 'error' }); break; }
            const rows = d.joints.filter((r) => r.length >= robot.dof).map((r) => r.slice(0, robot.dof));
            if (rows.length < 1) { problems.push({ instructionId: ins.id, message: 'Joint path is empty', severity: 'error' }); break; }
            const q0 = jointsOf.get(robot.id) ?? robot.joints();
            const tool = toolPose();
            const samples: TrajectorySample[] = [];
            let tt = 0, length = 0;
            let prev = q0, prevPose = robot.solveFK(q0, tool);
            samples.push({ t: 0, joints: q0, pose: prevPose });
            for (const r of rows) {
              const dq = Math.max(...r.map((v, i) => Math.abs(v - prev[i])), 0);
              const pose = robot.solveFK(r, tool);
              const dist = Math.hypot(pose[12] - prevPose[12], pose[13] - prevPose[13], pose[14] - prevPose[14]);
              // time per row: explicit dt, else the slower of joint-speed and linear-speed limits
              const dtRow = d.dt ?? Math.max(dq / Math.max(1, speed.speedJoints), dist / Math.max(1, speed.speedLinear), 0.001);
              tt += dtRow;
              length += dist;
              samples.push({ t: tt, joints: r, pose });
              prev = r; prevPose = pose;
            }
            const traj: Trajectory = { samples, duration: tt, length, ok: true };
            jointsOf.set(robot.id, rows[rows.length - 1]);
            t += tt;
            distance += length;
            this.steps.push({ instruction: ins, robot, trajectory: traj, t0, t1: t });
            break;
          }
          case 'speed':
            speed = { ...speed, ...(d.speedLinear !== undefined && { speedLinear: d.speedLinear }), ...(d.speedJoints !== undefined && { speedJoints: d.speedJoints }), ...(d.accelLinear !== undefined && { accelLinear: d.accelLinear }), ...(d.accelJoints !== undefined && { accelJoints: d.accelJoints }) };
            this.steps.push({ instruction: ins, robot, t0, t1: t });
            break;
          case 'rounding':
            speed.rounding = d.radius;
            this.steps.push({ instruction: ins, robot, t0, t1: t });
            break;
          case 'frame':
            frameId = d.frameId;
            this.steps.push({ instruction: ins, robot, t0, t1: t, apply: () => robot?.setFrame(d.frameId ? this.station.findById(d.frameId) : null) });
            break;
          case 'tool':
            toolId = d.toolId;
            this.steps.push({ instruction: ins, robot, t0, t1: t, apply: () => robot?.setTool(d.toolId ? (this.station.findById(d.toolId) as Tool) : null) });
            break;
          case 'pause':
            t += Math.max(0, d.timeMs) / 1000;
            this.steps.push({ instruction: ins, robot, t0, t1: t });
            break;
          case 'io':
            if (d.wait) t += 0.1; // nominal wait
            this.steps.push({ instruction: ins, robot, t0, t1: t, apply: () => { if (!d.wait) { this.io[d.io] = d.value; this.events.emit('io', { name: d.io, value: d.value }); } } });
            break;
          case 'signal':
            this.steps.push({ instruction: ins, robot, t0, t1: t, apply: () => { if (!d.wait) this.signals[d.signal] = d.value; } });
            break;
          case 'event': {
            const capturedTool = toolId;
            this.steps.push({ instruction: ins, robot, t0, t1: t, apply: () => this.applyEvent(robot, capturedTool, d.action, d.objectId ?? null) });
            break;
          }
          case 'code':
          case 'print':
          case 'mission_task':
            this.steps.push({ instruction: ins, robot, t0, t1: t, apply: () => { if (d.kind === 'print' && !d.isComment) this.events.emit('message', { text: d.message, level: 'info' }); } });
            break;
          case 'call': {
            const sub = d.programId ? this.station.findById(d.programId) : this.station.find(d.programName ?? '', ItemType.PROGRAM);
            if (sub instanceof Program) compileProgram(sub, depth + 1);
            else problems.push({ instructionId: ins.id, message: `Program ${d.programName ?? d.programId} not found`, severity: 'error' });
            break;
          }
          case 'loop': {
            const body = d.bodyProgramId ? this.station.findById(d.bodyProgramId) : null;
            if (body instanceof Program) for (let i = 0; i < d.count; i++) compileProgram(body, depth + 1);
            break;
          }
          case 'if': {
            // Static evaluation is impossible: simulate the "then" branch and warn.
            const body = d.thenProgramId ? this.station.findById(d.thenProgramId) : null;
            if (body instanceof Program) compileProgram(body, depth + 1);
            problems.push({ instructionId: ins.id, message: 'Conditional: simulated "then" branch', severity: 'warning' });
            break;
          }
          case 'mobile_move':
          case 'mobile_follow':
            // handled by the mobile simulator; estimate time with nominal speed
            this.steps.push({ instruction: ins, robot, t0, t1: t });
            break;
          case 'thread': {
            const sub = d.programId ? this.station.findById(d.programId) : this.station.find(d.programName ?? '', ItemType.PROGRAM);
            if (!(sub instanceof Program)) { problems.push({ instructionId: ins.id, message: `Thread program ${d.programName ?? d.programId} not found`, severity: 'error' }); break; }
            // parallel timeline: compile the sub program starting at the current time, then restore the main clock
            const tMain = t;
            const subRobot = sub.robot();
            if (subRobot && robot && subRobot.id === robot.id) problems.push({ instructionId: ins.id, message: 'Thread uses the same robot as the main program', severity: 'warning' });
            compileProgram(sub, depth + 1);
            const tEnd = t;
            t = tMain;
            this.threadEnds.push(tEnd);
            this.steps.push({ instruction: ins, robot, t0, t1: t });
            break;
          }
          case 'wait':
            if (d.what === 'time') t += Math.max(0, d.timeMs ?? 0) / 1000;
            else if (d.what === 'signal') t += (d.timeMs ?? 100) / 1000; // nominal wait for a signal
            else if (d.what === 'move_done') { /* moves are already blocking in simulation */ }
            this.steps.push({ instruction: ins, robot, t0, t1: t });
            break;
        }
      }
      visited.delete(prog.id);
    };

    compileProgram(program, 0);
    this.collisions = [];
    if (this.collisionOptions.enabled) this.checkTrajectoryCollisions(problems);
    this.duration = Math.max(t, ...this.threadEnds);
    // steps must be sorted by start time for seeking (threads insert out of order)
    this.steps.sort((a, b) => a.t0 - b.t0);
    this.time = 0;
    this.currentStep = -1;
    this.robotEndJoints = jointsOf;
    this.result = { ok: !problems.some((p) => p.severity === 'error'), duration: t, distance, problems, executed };
    program.lastResult = this.result;
    return this.result;
  }

  /** Sample every trajectory and report colliding instructions (robot vs station). */
  private checkTrajectoryCollisions(problems: ProgramRunResult['problems']): void {
    const step = this.collisionOptions.sampleStep ?? 0.1;
    const saved = new Map<string, number[]>();
    for (const s of this.steps) if (s.robot && !saved.has(s.robot.id)) saved.set(s.robot.id, s.robot.joints());
    // Resting contacts present before any motion (robot on its pedestal, part in a fixture) are not collisions:
    // record them once as warnings and ignore those pairs along the trajectory.
    const ignore: Array<[string, string]> = [...(this.collisionOptions.ignore ?? [])];
    const reported = new Set<string>();
    for (const [id] of saved) {
      const robot = this.station.findById(id) as Robot | null;
      if (!robot) continue;
      for (const p of checkRobotCollisions(this.station, robot, this.collisionOptions)) {
        const key = `${p.a.item.id}|${p.b.item.id}`;
        if (reported.has(key)) continue;
        reported.add(key);
        ignore.push([p.a.item.id, p.b.item.id]);
        problems.push({ instructionId: '', message: `Initial contact ignored: ${p.a.item.name} × ${p.b.item.name} (${p.depth.toFixed(0)} mm)`, severity: 'warning' });
      }
    }
    const collOpts = { ...this.collisionOptions, ignore };
    for (const s of this.steps) {
      if (!s.robot || !s.trajectory || !s.trajectory.samples.length) continue;
      let lastT = -Infinity;
      for (const sample of s.trajectory.samples) {
        if (sample.t - lastT < step && sample !== s.trajectory.samples[s.trajectory.samples.length - 1]) continue;
        lastT = sample.t;
        s.robot.setJoints(sample.joints);
        const pairs = checkRobotCollisions(this.station, s.robot, collOpts);
        if (pairs.length) {
          this.collisions.push({ instructionId: s.instruction.id, t: s.t0 + sample.t, pairs });
          const p = pairs[0];
          problems.push({ instructionId: s.instruction.id, message: `Collision: ${p.a.item.name}${p.a.part !== p.a.item.name ? '/' + p.a.part : ''} × ${p.b.item.name}${p.b.part !== p.b.item.name ? '/' + p.b.part : ''} at ${(s.t0 + sample.t).toFixed(2)} s`, severity: 'error' });
          break;
        }
      }
    }
    for (const [id, q] of saved) (this.station.findById(id) as Robot | null)?.setJoints(q);
  }

  private resolvePose(robot: Robot, target: Target | null, d: MoveInstruction, frame: Mat4, tool: Mat4, q0: number[]): Mat4 | null {
    if (target) {
      if (target.isJointTarget && target.joints) return robot.solveFK(target.joints, tool);
      return multiply(invert(robot.poseAbs()), target.poseAbs());
    }
    if (d.pose) return multiply(frame, fromArray(d.pose));
    if (d.joints) return robot.solveFK(d.joints, tool);
    void q0;
    return null;
  }

  private applyEvent(robot: Robot | null, toolId: string | null, action: string, objectId: string | null) {
    const tool = (toolId ? this.station.findById(toolId) : robot?.activeTool()) as Tool | null;
    switch (action) {
      case 'attach':
      case 'gripper_close': {
        if (!tool || !robot) return;
        if (action === 'gripper_close') tool.closed = true;
        const obj = objectId ? (this.station.findById(objectId) as SceneObject | null) : this.nearestObject(robot, tool);
        if (!obj) return;
        const toolAbs = multiply(robot.poseAbs(), robot.solveFK(robot.joints(), tool.poseTool()));
        const local = multiply(invert(toolAbs), obj.poseAbs());
        this.attachments.set(obj.id, { tool, local });
        if (!tool.attached.includes(obj.id)) tool.attached.push(obj.id);
        break;
      }
      case 'detach':
      case 'gripper_open': {
        if (tool && action === 'gripper_open') tool.closed = false;
        const ids = objectId ? [objectId] : [...this.attachments.keys()].filter((id) => !tool || this.attachments.get(id)!.tool === tool);
        for (const id of ids) {
          this.attachments.delete(id);
          if (tool) tool.attached = tool.attached.filter((x) => x !== id);
        }
        break;
      }
      case 'show':
      case 'hide': {
        const obj = objectId ? this.station.findById(objectId) : null;
        obj?.setVisible(action === 'show');
        break;
      }
    }
  }

  private nearestObject(robot: Robot, tool: Tool): SceneObject | null {
    const tcp = multiply(robot.poseAbs(), robot.solveFK(robot.joints(), tool.poseTool()));
    let best: SceneObject | null = null, bestD = 300; // mm
    for (const it of this.station.walk()) {
      if (!(it instanceof SceneObject) || it instanceof Tool || it.isAncestorOf(robot) || robot.isAncestorOf(it)) continue;
      const p = it.poseAbs();
      const dd = Math.hypot(p[12] - tcp[12], p[13] - tcp[13], p[14] - tcp[14]);
      if (dd < bestD) { bestD = dd; best = it; }
    }
    return best;
  }

  /** Update attached objects to follow their tool. */
  private updateAttachments() {
    for (const [id, { tool, local }] of this.attachments) {
      const obj = this.station.findById(id);
      const robot = tool.parent instanceof Robot ? tool.parent : null;
      if (!obj || !robot) continue;
      const toolAbs = multiply(robot.poseAbs(), robot.solveFK(robot.joints(), tool.poseTool()));
      obj.setPoseAbs(multiply(toolAbs, local));
    }
  }

  /** Seek the simulation to absolute time t (applies robot joints and events up to t). */
  seek(t: number): void {
    t = Math.max(0, Math.min(this.duration, t));
    this.time = t;
    // apply discrete events of steps that start before t (re-run from scratch if going backwards)
    let stepIdx = -1;
    for (let i = 0; i < this.steps.length; i++) {
      const s = this.steps[i];
      if (s.t0 <= t) stepIdx = i;
      else break;
    }
    if (stepIdx < this.currentStep) {
      this.attachments.clear();
      this.currentStep = -1;
    }
    for (let i = this.currentStep + 1; i <= stepIdx; i++) {
      const s = this.steps[i];
      s.apply?.();
      this.events.emit('instruction', { program: s.instruction.parent as Program, instruction: s.instruction, index: i });
    }
    this.currentStep = stepIdx;
    // find the active trajectory for each robot
    const lastByRobot = new Map<string, Step>();
    for (let i = 0; i <= stepIdx; i++) {
      const s = this.steps[i];
      if (s.robot && s.trajectory) lastByRobot.set(s.robot.id, s);
    }
    for (const s of lastByRobot.values()) {
      const tr = s.trajectory!;
      if (!tr.samples.length) continue;
      const local = t - s.t0;
      const sample = sampleAt(tr, Math.min(local, tr.duration));
      s.robot!.setJoints(sample.joints);
      s.robot!.state.moving = local < tr.duration;
      s.robot!.state.progress = tr.duration > 0 ? Math.min(1, local / tr.duration) : 1;
    }
    this.updateAttachments();
    this.events.emit('tick', { t });
  }

  /** Advance the simulation by dt seconds (real time * speedFactor). */
  tick(dt: number): boolean {
    if (!this.playing) return false;
    const nt = this.time + dt * this.speedFactor;
    this.seek(nt);
    if (nt >= this.duration) {
      this.playing = false;
      for (const s of this.steps) if (s.robot) s.robot.state.moving = false;
      return false;
    }
    return true;
  }

  play(): void {
    if (this.time >= this.duration) this.seek(0);
    this.playing = true;
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
   * [time, instruction index, j1..jn] sampled every `dt` seconds for the program's robot.
   */
  jointsList(robotId: string, dt = 0.05): number[][] {
    const rows: number[][] = [];
    const steps = this.steps.filter((s) => s.robot?.id === robotId && s.trajectory && s.trajectory.samples.length);
    let last: number[] | null = null;
    for (const s of steps) {
      const tr = s.trajectory!;
      const idx = this.steps.indexOf(s);
      const n = Math.max(1, Math.ceil(tr.duration / dt));
      for (let k = 0; k <= n; k++) {
        const t = (k / n) * tr.duration;
        const q = sampleAt(tr, t).joints;
        if (last && k === 0 && last.every((v, i) => Math.abs(v - q[i]) < 1e-9)) continue;
        rows.push([s.t0 + t, idx, ...q]);
        last = q;
      }
    }
    return rows;
  }

  stepIndexAt(t: number): number {
    let idx = -1;
    for (let i = 0; i < this.steps.length; i++) if (this.steps[i].t0 <= t) idx = i;
    return idx;
  }
}

/** Convenience: estimate cycle time & validity without touching robot state. */
export function estimateProgram(station: Station, program: Program): ProgramRunResult {
  const sim = new ProgramSimulator(station);
  return sim.compile(program);
}

export { Item };
