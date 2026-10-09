import { Station, Target, Tool, Item, ItemType } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { Program, Instruction } from '../core/items/program';
import { Mat4, multiply, invert, fromArray, identity, poseToXyzrpw } from '../core/math/pose';

/** Flat, resolved representation of a program that post processors consume. */
export type PostEvent =
  | { kind: 'moveJ'; joints: number[]; pose: Mat4 | null; name: string; speed?: number; rounding?: number }
  | { kind: 'moveL'; joints: number[] | null; pose: Mat4; name: string; speed?: number; rounding?: number }
  | { kind: 'moveC'; joints: number[] | null; via: Mat4; pose: Mat4; name: string; viaName: string; speed?: number; rounding?: number }
  | { kind: 'setFrame'; pose: Mat4; name: string }
  | { kind: 'setTool'; pose: Mat4; name: string; toolId: string | null }
  | { kind: 'setSpeed'; speedLinear?: number; speedJoints?: number; accelLinear?: number; accelJoints?: number }
  | { kind: 'setRounding'; radius: number }
  | { kind: 'pause'; timeMs: number }
  | { kind: 'setDO'; io: string; value: number | boolean }
  | { kind: 'waitDI'; io: string; value: number | boolean; timeoutMs: number }
  | { kind: 'runCode'; code: string; isCall: boolean }
  | { kind: 'comment'; text: string }
  | { kind: 'message'; text: string }
  | { kind: 'callProgram'; name: string }
  | { kind: 'gripper'; close: boolean }
  | { kind: 'navigate'; x: number; y: number; heading?: number; speed?: number; label?: string }
  | { kind: 'task'; task: string; params: Record<string, any> };

export interface PostProgram {
  name: string;
  robot: Robot | null;
  robotName: string;
  dof: number;
  jointNames: string[];
  events: PostEvent[];
  /** Sub programs referenced by callProgram (name -> PostProgram). */
  subprograms: PostProgram[];
  /** Initial frame & tool. */
  frame: { pose: Mat4; name: string };
  tool: { pose: Mat4; name: string };
  problems: string[];
}

export interface PostFile {
  name: string;
  content: string;
  mime?: string;
}

export interface PostProcessor {
  id: string;
  name: string;
  brand: string;
  extension: string;
  description?: string;
  generate(program: PostProgram, options?: Record<string, any>): PostFile[];
}

const registry = new Map<string, PostProcessor>();
export function registerPost(p: PostProcessor): void {
  registry.set(p.id, p);
}
export function getPost(id: string): PostProcessor | undefined {
  return registry.get(id);
}
export function listPosts(): PostProcessor[] {
  return [...registry.values()];
}

/**
 * Resolve a Program into PostEvents: poses are expressed w.r.t. the active reference frame
 * (as robot controllers expect), joints solved with IK where needed.
 */
export function compileForPost(station: Station, program: Program): PostProgram {
  const robotItem = program.robot();
  const robot = robotItem instanceof Robot ? robotItem : null;
  const problems: string[] = [];
  const events: PostEvent[] = [];
  const subprograms: PostProgram[] = [];
  let frameItem: Item | null = program.frameId ? station.findById(program.frameId) : robot?.activeFrame() ?? null;
  let toolItem: Item | null = program.toolId ? station.findById(program.toolId) : robot?.activeTool() ?? null;
  const framePoseInBase = (): Mat4 => (robot && frameItem ? multiply(invert(robot.poseAbs()), frameItem.poseAbs()) : identity());
  const toolPose = (): Mat4 => (toolItem instanceof Tool ? toolItem.poseTool() : identity());
  const initialFrame = { pose: framePoseInBase(), name: frameItem?.name ?? 'World' };
  const initialTool = { pose: toolPose(), name: toolItem?.name ?? 'Flange' };
  let q = robot?.joints() ?? [];

  const poseInFrame = (target: Target | null, ins: Instruction): { pose: Mat4 | null; joints: number[] | null } => {
    const d = ins.data;
    if (d.kind !== 'move') return { pose: null, joints: null };
    const frame = framePoseInBase();
    const tool = toolPose();
    if (target) {
      if (target.isJointTarget && target.joints) {
        const pose = robot ? multiply(invert(frame), robot.solveFK(target.joints, tool)) : null;
        return { pose, joints: target.joints };
      }
      if (!robot) return { pose: multiply(invert(frameItem?.poseAbs() ?? identity()), target.poseAbs()), joints: target.joints };
      const inBase = multiply(invert(robot.poseAbs()), target.poseAbs());
      const pose = multiply(invert(frame), inBase);
      let joints = target.joints;
      const r = robot.solveIK(inBase, { seed: joints ?? q }, tool);
      if (r.ok) joints = r.joints;
      else problems.push(`${ins.name} -> ${target.name}: unreachable (${r.posError.toFixed(1)} mm)`);
      return { pose, joints };
    }
    if (d.joints) return { pose: robot ? multiply(invert(frame), robot.solveFK(d.joints, tool)) : null, joints: d.joints };
    if (d.pose) {
      const pose = fromArray(d.pose);
      let joints: number[] | null = null;
      if (robot) {
        const r = robot.solveIK(multiply(frame, pose), { seed: q }, tool);
        if (r.ok) joints = r.joints;
      }
      return { pose, joints };
    }
    return { pose: null, joints: null };
  };

  for (const ins of program.instructions()) {
    if (!ins.enabled) continue;
    const d = ins.data;
    switch (d.kind) {
      case 'move': {
        const target = d.targetId ? (station.findById(d.targetId) as Target | null) : null;
        const { pose, joints } = poseInFrame(target, ins);
        const name = target?.name ?? ins.name;
        if (d.moveType === 'MoveJ') {
          if (!joints) { problems.push(`${name}: no joint solution for MoveJ`); break; }
          q = joints;
          events.push({ kind: 'moveJ', joints, pose, name, speed: d.speed, rounding: d.rounding });
        } else if (d.moveType === 'MoveL') {
          if (!pose) { problems.push(`${name}: no pose for MoveL`); break; }
          if (joints) q = joints;
          events.push({ kind: 'moveL', joints, pose, name, speed: d.speed, rounding: d.rounding });
        } else {
          const via = d.viaTargetId ? (station.findById(d.viaTargetId) as Target | null) : null;
          const viaRes = via ? poseInFrame(via, ins) : { pose: d.viaPose ? fromArray(d.viaPose) : null, joints: null };
          if (!pose || !viaRes.pose) { problems.push(`${name}: MoveC needs via and target poses`); break; }
          if (joints) q = joints;
          events.push({ kind: 'moveC', joints, via: viaRes.pose, pose, name, viaName: via?.name ?? 'via', speed: d.speed, rounding: d.rounding });
        }
        break;
      }
      case 'jointPath': {
        // recorded joint path: emit joint moves (subsampled to keep controller programs manageable)
        const maxRows = 2000;
        const stride = Math.max(1, Math.ceil(d.joints.length / maxRows));
        d.joints.forEach((row, i) => {
          if (i % stride && i !== d.joints.length - 1) return;
          const joints = robot ? row.slice(0, robot.dof) : row;
          q = joints;
          const pose = robot ? multiply(invert(framePoseInBase()), robot.solveFK(joints, robot.poseTool())) : identity();
          events.push({ kind: 'moveJ', joints, pose, name: `${ins.name} ${i + 1}` });
        });
        break;
      }
      case 'frame':
        frameItem = d.frameId ? station.findById(d.frameId) : null;
        events.push({ kind: 'setFrame', pose: framePoseInBase(), name: frameItem?.name ?? 'World' });
        break;
      case 'tool':
        toolItem = d.toolId ? station.findById(d.toolId) : null;
        events.push({ kind: 'setTool', pose: toolPose(), name: toolItem?.name ?? 'Flange', toolId: d.toolId });
        break;
      case 'speed':
        events.push({ kind: 'setSpeed', speedLinear: d.speedLinear, speedJoints: d.speedJoints, accelLinear: d.accelLinear, accelJoints: d.accelJoints });
        break;
      case 'rounding':
        events.push({ kind: 'setRounding', radius: d.radius });
        break;
      case 'pause':
        events.push({ kind: 'pause', timeMs: d.timeMs });
        break;
      case 'io':
        events.push(d.wait ? { kind: 'waitDI', io: d.io, value: d.value, timeoutMs: d.timeoutMs ?? -1 } : { kind: 'setDO', io: d.io, value: d.value });
        break;
      case 'code':
        events.push({ kind: 'runCode', code: d.code, isCall: d.asFunctionCall });
        break;
      case 'print':
        events.push(d.isComment ? { kind: 'comment', text: d.message } : { kind: 'message', text: d.message });
        break;
      case 'event':
        if (d.action === 'gripper_close' || d.action === 'attach') events.push({ kind: 'gripper', close: true });
        else if (d.action === 'gripper_open' || d.action === 'detach') events.push({ kind: 'gripper', close: false });
        else events.push({ kind: 'comment', text: `event ${d.action}` });
        break;
      case 'call': {
        const sub = d.programId ? station.findById(d.programId) : station.find(d.programName ?? '', ItemType.PROGRAM);
        if (sub instanceof Program) {
          if (!subprograms.some((s) => s.name === sub.name)) subprograms.push(compileForPost(station, sub));
          events.push({ kind: 'callProgram', name: sub.name });
        } else problems.push(`Program ${d.programName ?? d.programId} not found`);
        break;
      }
      case 'signal':
        events.push(d.wait ? { kind: 'waitDI', io: d.signal, value: d.value as any, timeoutMs: -1 } : { kind: 'setDO', io: d.signal, value: d.value as any });
        break;
      case 'mobile_move':
        events.push({ kind: 'navigate', x: d.x, y: d.y, heading: d.heading, speed: d.speed, label: d.label });
        break;
      case 'mobile_follow':
        events.push({ kind: 'comment', text: `follow path ${d.pathId}` });
        break;
      case 'mission_task':
        events.push({ kind: 'task', task: d.task, params: d.params });
        break;
      case 'loop':
      case 'if':
        events.push({ kind: 'comment', text: `${d.kind} (not supported by post)` });
        break;
      case 'thread':
        events.push({ kind: 'comment', text: `start thread ${d.programName ?? d.programId ?? ''}` });
        break;
      case 'wait':
        if (d.what === 'time') events.push({ kind: 'pause', timeMs: d.timeMs ?? 0 });
        else if (d.what === 'signal') events.push({ kind: 'waitDI', io: d.signal ?? '', value: (d.value as any) ?? true, timeoutMs: d.timeMs ?? -1 });
        else events.push({ kind: 'comment', text: 'wait move done' });
        break;
    }
  }
  return {
    name: program.name,
    robot,
    robotName: robot?.name ?? '',
    dof: robot?.dof ?? 6,
    jointNames: robot?.jointNames() ?? [],
    events,
    subprograms,
    frame: initialFrame,
    tool: initialTool,
    problems,
  };
}

// ---- helpers shared by posts -------------------------------------------------

export const f = (v: number, d = 3) => {
  const s = v.toFixed(d);
  return s === '-0.000' || s === '-0.00' || s === '-0' ? s.slice(1) : s;
};
export const safeName = (s: string) => s.replace(/[^A-Za-z0-9_]/g, '_').replace(/^(\d)/, '_$1');
export { poseToXyzrpw };
