import { Item, ItemType, Target, SerializedItem, DeserializeContext, registerItemType } from './item';
import { Robot } from './robot';
import { Mat4, fromArray } from '../math/pose';

/** Instruction types (RoboDK INS_TYPE_* compatible where applicable). */
export enum InstructionType {
  INVALID = -1,
  MOVE = 0,        // MoveJ / MoveL / MoveC
  MOVEC = 1,
  CHANGESPEED = 2,
  CHANGEFRAME = 3,
  CHANGETOOL = 4,
  CHANGEROBOT = 5,
  PAUSE = 6,
  EVENT = 7,       // attach/detach/show/hide
  CODE = 8,        // raw program code / call
  PRINT = 9,       // message / comment
  ROUNDING = 10,
  IO = 11,         // set/wait digital IO
  // Extensions
  CALL = 20,
  MOBILE_MOVE = 30,   // navigate mobile base to a waypoint / pose
  MOBILE_FOLLOW = 31, // follow a path item
  MISSION_TASK = 32,  // agri task (harvest/spray/...) marker
  WAIT_SIGNAL = 33,
  SET_SIGNAL = 34,
  LOOP = 35,
  IF = 36,
  THREAD = 37,
  WAIT = 38,
}

export type MoveType = 'MoveJ' | 'MoveL' | 'MoveC';

export interface MoveInstruction {
  kind: 'move';
  moveType: MoveType;
  /** Target id in the station. */
  targetId: string | null;
  /** Inline joints or pose if no target item. */
  joints?: number[];
  pose?: number[]; // relative to active frame
  /** For MoveC: intermediate via target. */
  viaTargetId?: string | null;
  viaPose?: number[];
  /** Optional per-move override of speed / rounding. */
  speed?: number;
  rounding?: number;
}
export interface SpeedInstruction { kind: 'speed'; speedLinear?: number; speedJoints?: number; accelLinear?: number; accelJoints?: number }
export interface FrameInstruction { kind: 'frame'; frameId: string | null }
export interface ToolInstruction { kind: 'tool'; toolId: string | null }
export interface PauseInstruction { kind: 'pause'; timeMs: number } // -1 => wait for user
export interface EventInstruction { kind: 'event'; action: 'attach' | 'detach' | 'show' | 'hide' | 'gripper_close' | 'gripper_open' | 'custom'; objectId?: string | null; name?: string }
export interface CodeInstruction { kind: 'code'; code: string; asFunctionCall: boolean }
export interface PrintInstruction { kind: 'print'; message: string; isComment: boolean }
export interface RoundingInstruction { kind: 'rounding'; radius: number }
export interface IOInstruction { kind: 'io'; io: string; value: number | boolean; wait: boolean; timeoutMs?: number }
export interface CallInstruction { kind: 'call'; programId: string | null; programName?: string }
export interface MobileMoveInstruction { kind: 'mobile_move'; x: number; y: number; heading?: number; speed?: number; tolerance?: number; label?: string }
export interface MobileFollowInstruction { kind: 'mobile_follow'; pathId: string | null; speed?: number; reverse?: boolean }
export interface MissionTaskInstruction { kind: 'mission_task'; task: string; params: Record<string, any> }
export interface SignalInstruction { kind: 'signal'; signal: string; value: number | boolean | string; wait: boolean }
/** Replay a recorded joint path (e.g. RoboDK InstructionListJoints, ROS bag, CSV log). Rows are joint values (deg/mm). */
export interface JointPathInstruction { kind: 'jointPath'; joints: number[][]; /** seconds per row when known */ dt?: number; /** source label */ source?: string }
export interface LoopInstruction { kind: 'loop'; count: number; bodyProgramId: string | null }
export interface IfInstruction { kind: 'if'; condition: string; thenProgramId: string | null; elseProgramId?: string | null }
/** Start another program in parallel (RoboDK INSTRUCTION_START_THREAD). */
export interface ThreadInstruction { kind: 'thread'; programId: string | null; programName?: string }
/** Wait for a condition: a signal/IO value, or a time (RoboDK "Wait" instructions). */
export interface WaitInstruction { kind: 'wait'; what: 'time' | 'signal' | 'move_done'; signal?: string; value?: number | boolean | string; timeMs?: number }

export type InstructionData =
  | MoveInstruction | SpeedInstruction | FrameInstruction | ToolInstruction | PauseInstruction | EventInstruction | CodeInstruction
  | PrintInstruction | RoundingInstruction | IOInstruction | CallInstruction | MobileMoveInstruction | MobileFollowInstruction
  | MissionTaskInstruction | SignalInstruction | LoopInstruction | IfInstruction | ThreadInstruction | WaitInstruction | JointPathInstruction;

export class Instruction extends Item {
  data: InstructionData;
  enabled = true;

  constructor(data: InstructionData, name?: string, id?: string) {
    super(ItemType.INSTRUCTION, name ?? describeInstruction(data), id);
    this.data = data;
  }

  get insType(): InstructionType {
    switch (this.data.kind) {
      case 'move': return this.data.moveType === 'MoveC' ? InstructionType.MOVEC : InstructionType.MOVE;
      case 'jointPath': return InstructionType.MOVE;
      case 'speed': return InstructionType.CHANGESPEED;
      case 'frame': return InstructionType.CHANGEFRAME;
      case 'tool': return InstructionType.CHANGETOOL;
      case 'pause': return InstructionType.PAUSE;
      case 'event': return InstructionType.EVENT;
      case 'code': return InstructionType.CODE;
      case 'print': return InstructionType.PRINT;
      case 'rounding': return InstructionType.ROUNDING;
      case 'io': return InstructionType.IO;
      case 'call': return InstructionType.CALL;
      case 'mobile_move': return InstructionType.MOBILE_MOVE;
      case 'mobile_follow': return InstructionType.MOBILE_FOLLOW;
      case 'mission_task': return InstructionType.MISSION_TASK;
      case 'signal': return this.data.wait ? InstructionType.WAIT_SIGNAL : InstructionType.SET_SIGNAL;
      case 'loop': return InstructionType.LOOP;
      case 'if': return InstructionType.IF;
      case 'thread': return InstructionType.THREAD;
      case 'wait': return InstructionType.WAIT;
    }
  }

  protected override serializeExtra() {
    return { data: this.data, enabled: this.enabled };
  }
  override deserializeExtra(d: SerializedItem) {
    this.data = d.data as InstructionData;
    this.enabled = d.enabled !== false;
  }
}

export function describeInstruction(d: InstructionData): string {
  switch (d.kind) {
    case 'move': return `${d.moveType}`;
    case 'jointPath': return `Joint path (${d.joints.length} points${d.source ? `, ${d.source}` : ''})`;
    case 'speed': return `Set Speed (${d.speedLinear ?? '-'} mm/s, ${d.speedJoints ?? '-'} deg/s)`;
    case 'frame': return 'Set Reference Frame';
    case 'tool': return 'Set Tool Frame';
    case 'pause': return d.timeMs < 0 ? 'Pause (wait user)' : `Pause ${d.timeMs} ms`;
    case 'event': return `Event: ${d.action}`;
    case 'code': return d.asFunctionCall ? `Call ${d.code}` : `Code: ${d.code.slice(0, 30)}`;
    case 'print': return d.isComment ? `// ${d.message}` : `Show "${d.message}"`;
    case 'rounding': return `Rounding ${d.radius} mm`;
    case 'io': return `${d.wait ? 'Wait' : 'Set'} ${d.io} = ${d.value}`;
    case 'call': return `Call program ${d.programName ?? d.programId ?? ''}`;
    case 'mobile_move': return `Navigate to (${d.x.toFixed(0)}, ${d.y.toFixed(0)})${d.label ? ' ' + d.label : ''}`;
    case 'mobile_follow': return `Follow path`;
    case 'mission_task': return `Task: ${d.task}`;
    case 'signal': return `${d.wait ? 'Wait' : 'Set'} signal ${d.signal} = ${d.value}`;
    case 'loop': return `Loop x${d.count}`;
    case 'if': return `If ${d.condition}`;
    case 'thread': return `Start thread ${d.programName ?? d.programId ?? ''}`;
    case 'wait': return d.what === 'time' ? `Wait ${d.timeMs ?? 0} ms` : d.what === 'signal' ? `Wait ${d.signal} = ${d.value}` : 'Wait move done';
  }
}

/**
 * Program: an ordered list of Instruction children, bound to a robot (or a mobile robot).
 * Mirrors RoboDK ITEM_TYPE_PROGRAM.
 */
export class Program extends Item {
  robotId: string | null = null;
  /** Default frame/tool at program start. */
  frameId: string | null = null;
  toolId: string | null = null;
  /** Result of the last simulation/validation run. */
  lastResult: ProgramRunResult | null = null;
  /** Run mode: simulate | generate | run_on_robot (RoboDK RUNMODE_*). */
  runMode: 'simulate' | 'generate' | 'run_on_robot' = 'simulate';

  constructor(name = 'Program', id?: string) {
    super(ItemType.PROGRAM, name, id);
  }

  instructions(): Instruction[] {
    return this.children.filter((c) => c instanceof Instruction) as Instruction[];
  }

  robot(): Robot | Item | null {
    return this.robotId ? this.station?.findById(this.robotId) ?? null : null;
  }

  setRobot(r: Item | null): void {
    this.robotId = r?.id ?? null;
    this.notify('robot');
  }

  addInstruction(data: InstructionData, index?: number): Instruction {
    return this.addChild(new Instruction(data), index);
  }

  addMoveJ(target: Target | number[], opts: Partial<MoveInstruction> = {}): Instruction {
    return this.addInstruction(moveData('MoveJ', target, opts));
  }
  addMoveL(target: Target | number[] | Mat4, opts: Partial<MoveInstruction> = {}): Instruction {
    return this.addInstruction(moveData('MoveL', target, opts));
  }
  addMoveC(via: Target, target: Target, opts: Partial<MoveInstruction> = {}): Instruction {
    return this.addInstruction({ ...moveData('MoveC', target, opts), viaTargetId: via.id });
  }
  setSpeed(speedLinear?: number, speedJoints?: number, accelLinear?: number, accelJoints?: number): Instruction {
    return this.addInstruction({ kind: 'speed', speedLinear, speedJoints, accelLinear, accelJoints });
  }
  setRounding(radius: number): Instruction {
    return this.addInstruction({ kind: 'rounding', radius });
  }
  setFrame(frame: Item | null): Instruction {
    return this.addInstruction({ kind: 'frame', frameId: frame?.id ?? null });
  }
  setTool(tool: Item | null): Instruction {
    return this.addInstruction({ kind: 'tool', toolId: tool?.id ?? null });
  }
  pause(timeMs: number): Instruction {
    return this.addInstruction({ kind: 'pause', timeMs });
  }
  setDO(io: string, value: number | boolean): Instruction {
    return this.addInstruction({ kind: 'io', io, value, wait: false });
  }
  waitDI(io: string, value: number | boolean, timeoutMs = -1): Instruction {
    return this.addInstruction({ kind: 'io', io, value, wait: true, timeoutMs });
  }
  runInstruction(code: string, asFunctionCall = true): Instruction {
    return this.addInstruction({ kind: 'code', code, asFunctionCall });
  }
  showMessage(message: string, isComment = false): Instruction {
    return this.addInstruction({ kind: 'print', message, isComment });
  }
  comment(message: string): Instruction {
    return this.showMessage(message, true);
  }
  callProgram(p: Program): Instruction {
    return this.addInstruction({ kind: 'call', programId: p.id, programName: p.name });
  }
  startThread(p: Program): Instruction {
    return this.addInstruction({ kind: 'thread', programId: p.id, programName: p.name });
  }
  waitSignal(signal: string, value: number | boolean | string, timeoutMs?: number): Instruction {
    return this.addInstruction({ kind: 'wait', what: 'signal', signal, value, timeMs: timeoutMs });
  }
  event(action: EventInstruction['action'], objectId?: string | null, name?: string): Instruction {
    return this.addInstruction({ kind: 'event', action, objectId, name });
  }
  navigateTo(x: number, y: number, heading?: number, opts: Partial<MobileMoveInstruction> = {}): Instruction {
    return this.addInstruction({ kind: 'mobile_move', x, y, heading, ...opts });
  }

  protected override serializeExtra() {
    return { robotId: this.robotId, frameId: this.frameId, toolId: this.toolId, runMode: this.runMode };
  }
  override deserializeExtra(d: SerializedItem, _ctx: DeserializeContext) {
    this.robotId = (d.robotId as string | null) ?? null;
    this.frameId = (d.frameId as string | null) ?? null;
    this.toolId = (d.toolId as string | null) ?? null;
    this.runMode = (d.runMode as any) ?? 'simulate';
  }
}

export function moveData(moveType: MoveType, target: Target | number[] | Mat4, opts: Partial<MoveInstruction> = {}): MoveInstruction {
  if (target instanceof Target) return { kind: 'move', moveType, targetId: target.id, ...opts };
  if (target instanceof Float64Array) return { kind: 'move', moveType, targetId: null, pose: Array.from(target), ...opts };
  return { kind: 'move', moveType, targetId: null, joints: [...target], ...opts };
}

export interface ProgramRunResult {
  ok: boolean;
  /** Total simulated time (s). */
  duration: number;
  /** Travelled TCP distance (mm). */
  distance: number;
  problems: Array<{ instructionId: string; message: string; severity: 'error' | 'warning' }>;
  /** Number of instructions executed. */
  executed: number;
}

/** Path / trajectory item (2D-3D polyline used by mobile robots, conveyors, crop rows). */
export class PathItem extends Item {
  /** Points relative to this item (mm). */
  points: number[][] = [];
  closed = false;
  /** Nominal travel speed (mm/s). */
  speed = 1000;
  width = 0;
  constructor(name = 'Path', id?: string) {
    super(ItemType.PATH, name, id);
  }
  length(): number {
    let l = 0;
    for (let i = 1; i < this.points.length; i++) {
      const a = this.points[i - 1], b = this.points[i];
      l += Math.hypot(b[0] - a[0], b[1] - a[1], (b[2] ?? 0) - (a[2] ?? 0));
    }
    if (this.closed && this.points.length > 2) {
      const a = this.points[this.points.length - 1], b = this.points[0];
      l += Math.hypot(b[0] - a[0], b[1] - a[1], (b[2] ?? 0) - (a[2] ?? 0));
    }
    return l;
  }
  protected override serializeExtra() {
    return { points: this.points, closed: this.closed, speed: this.speed, width: this.width };
  }
  override deserializeExtra(d: SerializedItem) {
    this.points = (d.points as number[][]) ?? [];
    this.closed = !!d.closed;
    this.speed = (d.speed as number) ?? 1000;
    this.width = (d.width as number) ?? 0;
  }
}

registerItemType(ItemType.PROGRAM, (n, id) => new Program(n, id));
registerItemType(ItemType.INSTRUCTION, (n, id) => new Instruction({ kind: 'print', message: '', isComment: true }, n, id));
registerItemType(ItemType.PATH, (n, id) => new PathItem(n, id));

export { fromArray as poseFromArray };
