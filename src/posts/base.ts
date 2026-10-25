import { Station, Target, Tool, Item, ItemType } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { Program, Instruction } from '../core/items/program';
import { Mat4, multiply, invert, fromArray, identity, poseToXyzrpw } from '../core/math/pose';
import { actuatedIndices } from '../core/kinematics/chain';

/** Flat, resolved representation of a program that post processors consume. */
export type PostEvent =
  | { kind: 'moveJ'; joints: number[]; pose: Mat4 | null; name: string; speed?: number; rounding?: number; conf?: number[] }
  | { kind: 'moveL'; joints: number[] | null; pose: Mat4; name: string; speed?: number; rounding?: number; conf?: number[] }
  | { kind: 'moveC'; joints: number[] | null; via: Mat4; pose: Mat4; name: string; viaName: string; viaJoints?: number[] | null; viaConf?: number[]; conf?: number[]; speed?: number; rounding?: number }
  | { kind: 'setFrame'; pose: Mat4; name: string; frameId?: number }
  | { kind: 'setTool'; pose: Mat4; name: string; toolId: string | null; toolNum?: number }
  | { kind: 'setSpeed'; speedLinear?: number; speedJoints?: number; accelLinear?: number; accelJoints?: number }
  | { kind: 'setRounding'; radius: number }
  | { kind: 'pause'; timeMs: number }
  | { kind: 'setDO'; io: string; value: number | boolean }
  | { kind: 'setAO'; io: string; value: number | string }
  | { kind: 'waitDI'; io: string; value: number | boolean; timeoutMs: number }
  | { kind: 'runCode'; code: string; isCall: boolean }
  | { kind: 'comment'; text: string }
  | { kind: 'message'; text: string }
  | { kind: 'callProgram'; name: string }
  /** Attach/detach or gripper simulation event: posts write a comment only (RoboDK writes no code for them). */
  | { kind: 'gripper'; close: boolean; text?: string }
  | { kind: 'navigate'; x: number; y: number; heading?: number; speed?: number; label?: string }
  | { kind: 'task'; task: string; params: Record<string, any> }
  /** Control flow (I6): loop calling a sub-program `count` times (<= 0 forever), IF on a signal, start a thread. */
  | { kind: 'loop'; count: number; body: string }
  | { kind: 'if'; condition: string; then: string | null; else: string | null }
  | { kind: 'thread'; name: string };

export interface PostProgram {
  name: string;
  robot: Robot | null;
  robotName: string;
  dof: number;
  /** RoboDK axes_type per joint: 'R' rotary, 'T' linear, 'J' / 'L' synchronised external axes. */
  axesType?: string[];
  jointNames: string[];
  events: PostEvent[];
  /** Sub programs referenced by callProgram / loop / if / thread (name -> PostProgram). */
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
  /** 'base64' for binary files (zip archives written by some posts). */
  encoding?: 'utf-8' | 'base64';
}

export interface PostProcessor {
  id: string;
  name: string;
  brand: string;
  extension: string;
  description?: string;
  /** RoboDK post-processor names this post stands for (robot files reference posts by these names). */
  aliases?: string[];
  generate(program: PostProgram, options?: Record<string, any>): PostFile[];
  /** Asynchronous generation (Python / native RoboDK posts). Built-in posts resolve to generate(). */
  generateAsync?(program: PostProgram, options?: Record<string, any>): Promise<PostFile[]>;
}

const registry = new Map<string, PostProcessor>();
const aliases = new Map<string, string>();
export function registerPost(p: PostProcessor): void {
  registry.set(p.id, p);
  for (const a of p.aliases ?? []) aliases.set(a.toLowerCase(), p.id);
}
/** Look a post up by studio id or by RoboDK post name (e.g. 'Fanuc_R30i', 'Omron', 'Universal_Robots_URP'). */
export function getPost(id: string): PostProcessor | undefined {
  if (!id) return undefined;
  const base = id.replace(/\.(py|pyc)$/i, '').replace(/^.*[\\/]/, '');
  return registry.get(id) ?? registry.get(base) ?? registry.get(aliases.get(base.toLowerCase()) ?? '') ?? [...registry.values()].find((p) => p.id.toLowerCase() === base.toLowerCase());
}
export function resolvePostId(id: string): string | undefined { return getPost(id)?.id; }
export function listPosts(): PostProcessor[] {
  return [...registry.values()];
}
/** Generate with the async path when the post has one (user Python posts), else synchronously. */
export async function generateProgram(post: PostProcessor, program: PostProgram, options?: Record<string, any>): Promise<PostFile[]> {
  return post.generateAsync ? post.generateAsync(program, options) : post.generate(program, options);
}

// ---- RoboDK configuration flags ---------------------------------------------------------------------------------------
type V3 = [number, number, number];
const sub3 = (a: V3, b: V3): V3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot3 = (a: V3, b: V3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross3 = (a: V3, b: V3): V3 => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const scl3 = (a: V3, s: number): V3 => [a[0] * s, a[1] * s, a[2] * s];
const nrm3 = (a: V3) => Math.hypot(a[0], a[1], a[2]);
/** Closest point on line (p1,d1) to line (p2,d2). */
function closestOnLine(p1: V3, d1: V3, p2: V3, d2: V3): V3 {
  const r = sub3(p1, p2), a = dot3(d1, d1), b = dot3(d1, d2), c = dot3(d2, d2), d = dot3(d1, r), e = dot3(d2, r);
  const den = a * c - b * b;
  const t = Math.abs(den) < 1e-12 ? 0 : (b * e - c * d) / den;
  return [p1[0] + d1[0] * t, p1[1] + d1[1] * t, p1[2] + d1[2] * t];
}
interface ArmGeo { a1o: V3; a1d: V3; shoulder: V3; elbow: V3; wrist: V3; a2d: V3 }
function armGeometry(robot: Robot, q: number[]): ArmGeo | null {
  const fk = robot.fk(q);
  const jf = fk.jointFrames;
  const act = actuatedIndices(robot.chain).map((i) => ({ j: robot.chain.joints[i], i }));
  if (act.length < 6) return null;
  const axis = (k: number): { o: V3; d: V3 } => {
    const { j, i } = act[k], m = jf[i];
    const a = j.axis;
    return { o: [m[12], m[13], m[14]], d: [m[0] * a[0] + m[4] * a[1] + m[8] * a[2], m[1] * a[0] + m[5] * a[1] + m[9] * a[2], m[2] * a[0] + m[6] * a[1] + m[10] * a[2]] };
  };
  const j1 = axis(0), j2 = axis(1), j3 = axis(2), j4 = axis(3), j5 = axis(4);
  return { a1o: j1.o, a1d: j1.d, shoulder: closestOnLine(j2.o, j2.d, j1.o, j1.d), elbow: closestOnLine(j3.o, j3.d, j2.o, j2.d), wrist: closestOnLine(j5.o, j5.d, j4.o, j4.d), a2d: j2.d };
}
/**
 * RoboDK conf_RLF = [REAR, LOWERARM, FLIP] for a 6-axis arm: REAR when the wrist centre is behind the J1 axis w.r.t.
 * the arm's front direction (where the wrist points with J2..J6 at zero), LOWERARM when the elbow lies below the
 * shoulder-wrist chord as seen with the arm reaching forward (rotation invariant about J2), FLIP when J5 < 0.
 * Robots with fewer than 6 axes return [0,0,0].
 */
export function confRLF(robot: Robot | null, q: number[] | null): number[] {
  if (!robot || !q || robot.dof < 6) return [0, 0, 0];
  try {
    const g = armGeometry(robot, q);
    if (!g) return [0, 0, 0];
    const perp = (p: V3): V3 => { const r = sub3(p, g.a1o); const d = scl3(g.a1d, dot3(r, g.a1d) / dot3(g.a1d, g.a1d)); return sub3(r, d); };
    const q0 = q.map((v, i) => (i === 0 ? v : 0));
    const g0 = armGeometry(robot, q0);
    let front = g0 ? perp(g0.wrist) : perp(g.shoulder);
    if (nrm3(front) < 1) front = perp(g.shoulder);
    if (nrm3(front) < 1) front = cross3([0, 0, 1], g.a2d);
    const rear = dot3(perp(g.wrist), front) < -1e-6 ? 1 : 0;
    const side = dot3(g.a2d, cross3(sub3(g.wrist, g.shoulder), sub3(g.elbow, g.shoulder)));
    const upSign = Math.sign(dot3(g.a2d, cross3(front, [0, 0, 1]))) || 1;
    const lower = side * upSign < 0 ? 1 : 0;
    const flip = (q[4] ?? 0) < 0 ? 1 : 0;
    return [rear, lower, flip];
  } catch {
    return [0, 0, 0];
  }
}

/** RoboDK frame/tool index from the item name ("Frame 3" -> 3, "Spindle" -> -1). */
export function nameIndex(name: string): number {
  const m = name.match(/(\d+)\s*$/);
  return m ? Number(m[1]) : -1;
}

export interface CompileOptions {
  /** Emit the program's frame and tool before the first move when the program does not set them (default true for the exported program; sub-programs inherit the caller's). */
  initialFrameTool?: boolean;
  /** Subsample recorded joint paths to at most this many rows (default: keep all). */
  maxJointPathRows?: number;
}
interface CompileState { frame: Item | null; tool: Item | null; q: number[] }

const poseClose = (a: Mat4, b: Mat4, mm: number, rot: number) => {
  if (Math.abs(a[12] - b[12]) > mm || Math.abs(a[13] - b[13]) > mm || Math.abs(a[14] - b[14]) > mm) return false;
  for (const i of [0, 1, 2, 4, 5, 6, 8, 9, 10]) if (Math.abs(a[i] - b[i]) > rot) return false;
  return true;
};

/**
 * Resolve a Program into PostEvents the way RoboDK feeds its posts: Cartesian targets are written relative to their
 * reference frame (target.Pose(), so a program can be reused in another frame), joints come from the target's stored
 * joints when they match (else IK seeded with them), RoboDK conf_RLF flags are attached, MoveC carries the via joints,
 * per-move speed / rounding overrides become explicit setSpeed / setRounding events, no move is ever dropped
 * (unreachable ones are kept and reported in `problems`).
 */
export function compileForPost(station: Station, program: Program, options: CompileOptions = {}, inherited?: CompileState, compiled: Map<string, PostProgram> = new Map()): PostProgram {
  const robotItem = program.robot();
  const robot = robotItem instanceof Robot ? robotItem : null;
  const isSub = !!inherited;
  const problems: string[] = [];
  const events: PostEvent[] = [];
  const subprograms: PostProgram[] = [];
  const dof = robot?.dof ?? 6;
  const ins = program.instructions().filter((i) => i.enabled);
  const firstCartTarget = (): Target | null => {
    for (const i of ins) if (i.data.kind === 'move' && i.data.targetId) { const t = station.findById(i.data.targetId); if (t instanceof Target && !t.isJointTarget) return t; }
    return null;
  };
  let frameItem: Item | null = inherited ? inherited.frame : program.frameId ? station.findById(program.frameId) : null;
  if (!inherited && !frameItem) { const t = firstCartTarget(); frameItem = t?.parent && t.parent.type === ItemType.FRAME ? t.parent : robot?.activeFrame() ?? null; }
  let toolItem: Item | null = inherited ? inherited.tool : program.toolId ? station.findById(program.toolId) : robot?.activeTool() ?? null;
  const framePoseInBase = (): Mat4 => (robot && frameItem ? multiply(invert(robot.poseAbs()), frameItem.poseAbs()) : identity());
  const toolPose = (): Mat4 => (toolItem instanceof Tool ? toolItem.poseTool() : identity());
  const initialFrame = { pose: framePoseInBase(), name: frameItem?.name ?? 'World' };
  const initialTool = { pose: toolPose(), name: toolItem?.name ?? 'Flange' };
  let q = inherited?.q ?? robot?.joints() ?? [];
  const conf = (j: number[] | null) => (robot && j ? confRLF(robot, j) : [0, 0, 0]);

  /** Pose of a target in the frame the robot program uses (RoboDK: target pose w.r.t. its reference). */
  const targetPose = (t: Target): Mat4 => {
    if (t.parent && t.parent.type === ItemType.FRAME) return t.pose();
    const ref = frameItem ? frameItem.poseAbs() : robot ? robot.poseAbs() : identity();
    return multiply(invert(ref), t.poseAbs());
  };
  const solve = (poseInFrame: Mat4, seed: number[] | null, label: string): number[] | null => {
    if (!robot) return seed;
    const tool = toolPose();
    const inBase = multiply(framePoseInBase(), poseInFrame);
    if (seed && seed.length >= dof) {
      const s = seed.slice(0, dof);
      if (poseClose(robot.solveFK(s, tool), inBase, 1, 1e-3)) return seed.length > dof ? seed.slice() : s;
    }
    const r = robot.solveIK(inBase, { seed: seed && seed.length >= dof ? seed.slice(0, dof) : q }, tool);
    if (r.ok) return r.joints;
    problems.push(`${label}: unreachable (${r.posError.toFixed(1)} mm), joints kept from the target`);
    return seed && seed.length >= dof ? seed.slice(0, Math.max(dof, seed.length)) : r.joints?.length ? r.joints : q;
  };
  const resolveTarget = (target: Target | null, d: Extract<Instruction['data'], { kind: 'move' }>, viaPart: boolean): { pose: Mat4 | null; joints: number[] | null } => {
    const tool = toolPose();
    if (target) {
      if (target.isJointTarget && target.joints) {
        const pose = robot ? multiply(invert(framePoseInBase()), robot.solveFK(target.joints.slice(0, dof), tool)) : null;
        return { pose, joints: target.joints.slice() };
      }
      const pose = targetPose(target);
      return { pose, joints: solve(pose, target.joints, `${target.name}`) };
    }
    const jj = viaPart ? undefined : d.joints;
    const pp = viaPart ? d.viaPose : d.pose;
    if (jj) return { pose: robot ? multiply(invert(framePoseInBase()), robot.solveFK(jj.slice(0, dof), tool)) : null, joints: jj.slice() };
    if (pp) { const pose = fromArray(pp); return { pose, joints: solve(pose, null, 'inline pose') }; }
    return { pose: null, joints: null };
  };

  // speed / rounding state: per-move overrides become explicit events (RoboDK posts have no per-move parameters)
  const motion = robot?.motion ?? { speedLinear: 500, speedJoints: 90, accelLinear: 2000, accelJoints: 360, rounding: -1 };
  let progSpeed = motion.speedLinear, effSpeed: number | undefined;
  let progRound = motion.rounding, effRound: number | undefined;
  const applyOverrides = (d: { speed?: number; rounding?: number }, linear: boolean) => {
    if (linear) {
      const want = d.speed ?? progSpeed;
      if ((d.speed !== undefined || effSpeed !== undefined) && want !== effSpeed) { events.push({ kind: 'setSpeed', speedLinear: want }); effSpeed = want; }
    }
    const wantR = d.rounding ?? progRound;
    if ((d.rounding !== undefined || effRound !== undefined) && wantR !== effRound) { events.push({ kind: 'setRounding', radius: wantR }); effRound = wantR; }
  };
  const compileSub = (p: Program): string => {
    if (!compiled.has(p.name)) {
      const placeholder = { name: p.name } as PostProgram;
      compiled.set(p.name, placeholder);
      const sp = compileForPost(station, p, options, { frame: frameItem, tool: toolItem, q }, compiled);
      Object.assign(placeholder, sp);
      subprograms.push(placeholder);
      for (const pr of sp.problems) problems.push(`${p.name}: ${pr}`);
    } else if (!subprograms.includes(compiled.get(p.name)!) && compiled.get(p.name)!.events) subprograms.push(compiled.get(p.name)!);
    return p.name;
  };
  const findProgram = (id: string | null | undefined, name?: string): Program | null => {
    const it = id ? station.findById(id) : name ? station.find(name, ItemType.PROGRAM) : null;
    return it instanceof Program ? it : null;
  };
  compiled.set(program.name, compiled.get(program.name) ?? ({ name: program.name } as PostProgram));

  for (const insn of ins) {
    const d = insn.data;
    switch (d.kind) {
      case 'move': {
        const target = d.targetId ? (station.findById(d.targetId) as Target | null) : null;
        const { pose, joints } = resolveTarget(target, d, false);
        const name = target?.name ?? insn.name;
        if (d.moveType === 'MoveJ') {
          let j = joints;
          if (!j) { problems.push(`${name}: no joint values for MoveJ, previous joints kept`); j = q.slice(); }
          applyOverrides(d, false);
          q = j;
          events.push({ kind: 'moveJ', joints: j, pose, name, speed: d.speed, rounding: d.rounding, conf: conf(j) });
        } else if (d.moveType === 'MoveL') {
          if (!pose) { problems.push(`${name}: MoveL without a pose skipped`); break; }
          applyOverrides(d, true);
          if (joints) q = joints;
          events.push({ kind: 'moveL', joints, pose, name, speed: d.speed, rounding: d.rounding, conf: conf(joints) });
        } else {
          const via = d.viaTargetId ? (station.findById(d.viaTargetId) as Target | null) : null;
          const v = resolveTarget(via, d, true);
          if (!pose || !v.pose) { problems.push(`${name}: MoveC needs via and target poses`); break; }
          applyOverrides(d, true);
          if (joints) q = joints;
          events.push({ kind: 'moveC', joints, via: v.pose, pose, name, viaName: via?.name ?? 'via', viaJoints: v.joints, viaConf: conf(v.joints), conf: conf(joints), speed: d.speed, rounding: d.rounding });
        }
        break;
      }
      case 'jointPath': {
        const maxRows = options.maxJointPathRows ?? Infinity;
        const stride = Math.max(1, Math.ceil(d.joints.length / maxRows));
        d.joints.forEach((row, i) => {
          if (i % stride && i !== d.joints.length - 1) return;
          const joints = robot ? row.slice(0, robot.dof) : row;
          q = joints;
          const pose = robot ? multiply(invert(framePoseInBase()), robot.solveFK(joints, toolPose())) : identity();
          events.push({ kind: 'moveJ', joints, pose, name: `${insn.name} ${i + 1}`, conf: conf(joints) });
        });
        break;
      }
      case 'frame':
        frameItem = d.frameId ? station.findById(d.frameId) : null;
        events.push({ kind: 'setFrame', pose: framePoseInBase(), name: frameItem?.name ?? 'World', frameId: nameIndex(frameItem?.name ?? '') });
        break;
      case 'tool':
        toolItem = d.toolId ? station.findById(d.toolId) : null;
        events.push({ kind: 'setTool', pose: toolPose(), name: toolItem?.name ?? 'Flange', toolId: d.toolId, toolNum: nameIndex(toolItem?.name ?? '') });
        break;
      case 'speed':
        events.push({ kind: 'setSpeed', speedLinear: d.speedLinear, speedJoints: d.speedJoints, accelLinear: d.accelLinear, accelJoints: d.accelJoints });
        if (d.speedLinear !== undefined) { progSpeed = d.speedLinear; effSpeed = d.speedLinear; }
        break;
      case 'rounding':
        events.push({ kind: 'setRounding', radius: d.radius });
        progRound = d.radius; effRound = d.radius;
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
      case 'event': {
        const obj = d.objectId ? station.findById(d.objectId) : null;
        const who = toolItem?.name ?? 'Tool';
        if (d.action === 'attach' || d.action === 'gripper_close') events.push({ kind: 'gripper', close: true, text: d.action === 'attach' ? `Attach to ${who}` : 'gripper close' });
        else if (d.action === 'detach' || d.action === 'gripper_open') events.push({ kind: 'gripper', close: false, text: d.action === 'detach' ? `Detach from ${who}` : 'gripper open' });
        else events.push({ kind: 'comment', text: `${d.action === 'custom' ? d.name ?? 'event' : d.action} ${obj?.name ?? d.name ?? ''}`.trim() });
        break;
      }
      case 'call': {
        const sp = findProgram(d.programId, d.programName);
        if (sp) events.push({ kind: 'callProgram', name: compileSub(sp) });
        else events.push({ kind: 'runCode', code: d.programName ?? String(d.programId ?? ''), isCall: true });
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
      case 'loop': {
        const body = findProgram(d.bodyProgramId);
        if (body) events.push({ kind: 'loop', count: d.count, body: compileSub(body) });
        else problems.push(`Loop body program ${d.bodyProgramId} not found`);
        break;
      }
      case 'if': {
        const th = findProgram(d.thenProgramId), el = findProgram(d.elseProgramId ?? null);
        events.push({ kind: 'if', condition: d.condition, then: th ? compileSub(th) : null, else: el ? compileSub(el) : null });
        break;
      }
      case 'thread': {
        const tp = findProgram(d.programId, d.programName);
        if (tp) events.push({ kind: 'thread', name: compileSub(tp) });
        else problems.push(`Thread program ${d.programName ?? d.programId} not found`);
        break;
      }
      case 'wait':
        if (d.what === 'time') events.push({ kind: 'pause', timeMs: d.timeMs ?? 0 });
        else if (d.what === 'signal') events.push({ kind: 'waitDI', io: d.signal ?? '', value: (d.value as any) ?? true, timeoutMs: d.timeMs ?? -1 });
        else events.push({ kind: 'comment', text: 'wait move done' });
        break;
    }
  }
  // the exported program starts in its own frame / with its own tool (sub-programs inherit the caller's)
  if (!isSub && options.initialFrameTool !== false) {
    const firstMove = events.findIndex((e) => e.kind === 'moveJ' || e.kind === 'moveL' || e.kind === 'moveC' || e.kind === 'callProgram' || e.kind === 'loop' || e.kind === 'if');
    const before = firstMove < 0 ? events : events.slice(0, firstMove);
    const pre: PostEvent[] = [];
    if (firstMove >= 0 && !before.some((e) => e.kind === 'setFrame')) pre.push({ kind: 'setFrame', pose: initialFrame.pose, name: initialFrame.name, frameId: nameIndex(initialFrame.name) });
    if (firstMove >= 0 && !before.some((e) => e.kind === 'setTool')) pre.push({ kind: 'setTool', pose: initialTool.pose, name: initialTool.name, toolId: toolItem?.id ?? null, toolNum: nameIndex(initialTool.name) });
    events.unshift(...pre);
  }
  const out: PostProgram = {
    name: program.name,
    robot,
    robotName: robot?.name ?? '',
    dof,
    axesType: robot ? actuatedIndices(robot.chain).map((i) => (robot.chain.joints[i].type === 'prismatic' ? 'T' : 'R')) : undefined,
    jointNames: robot?.jointNames() ?? [],
    events,
    subprograms,
    frame: initialFrame,
    tool: initialTool,
    problems,
  };
  if (!isSub) {
    // flatten nested sub-programs into the exported program's list (each generated once)
    const all: PostProgram[] = [];
    const walk = (p: PostProgram) => { for (const s of p.subprograms) if (!all.includes(s) && s !== out) { all.push(s); walk(s); } };
    walk(out);
    out.subprograms = all;
  }
  return out;
}

// ---- helpers shared by posts -------------------------------------------------
export const f = (v: number, d = 3) => {
  const s = v.toFixed(d);
  return /^-0\.?0*$/.test(s) ? s.slice(1) : s;
};
export const safeName = (s: string) => s.replace(/[^A-Za-z0-9_]/g, '_').replace(/^(\d)/, '_$1');
export { poseToXyzrpw };
