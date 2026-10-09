/**
 * RoboDK-compatible scripting API (JavaScript). Mirrors robodk.robolink.Robolink / Item so that
 * RoboDK scripts port with minimal changes. Used by the in-app console, the WebSocket server and
 * the Python drop-in client (python/robodk/robolink.py).
 */
import { Station, Item as CoreItem, ItemType, Frame, Target, Tool, SceneObject, Folder } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { Program, Instruction, InstructionType } from '../core/items/program';
import { createRobotFromLibrary, ROBOT_LIBRARY } from '../core/items/library';
import { Mat4, identity, multiply, invert, transl, rotx, roty, rotz, xyzrpwToPose, poseToXyzrpw, kukaToPose, poseToKuka, fanucToPose, poseToFanuc, urToPose, poseToUr, quatToPose, poseToQuat, toRows, fromRows, DEG } from '../core/math/pose';
import { ProgramSimulator } from '../core/motion/simulator';
import { compileForPost, getPost } from '../posts/index';
import { MobileRobot } from '../mobile/items';
import { Camera as CameraItem } from '../core/items/item';
import { checkCollisionsMapped, itemsCollide, collisionLine, setCollisionPair, getCollisionMap, collidersOf, CollisionPair } from '../core/collision/collision';
import { calibrateTcpByPoint, calibrateTcpByLine } from '../core/calibration/tcp';
import { calibrateFrame } from '../core/calibration/frame';
import { calibrateRobot, applyCalibration, Measurement } from '../core/calibration/robot';
import { createIsoCubeProgram, createBallbarProgram, simulateTrackerMeasure } from '../core/calibration/iso9283';
import { buildMechanism, chainFromDHM, dhToDHM, MechanismSpec } from '../core/kinematics/mechanism';
import { SpraySimulator } from '../core/motion/spray';
import { clipboard, cloneItem } from '../core/items/clone';
import { EventQueue } from '../core/events-queue';
import { generateCurveFollow, FollowOptions } from '../core/motion/pathfollow';
import { generateMachining } from '../core/motion/machining';
import { planMoveJ, planMoveL } from '../core/motion/trajectory';
import { getPos as _getPos, poseFromZ, transformDir } from '../core/math/pose';

export const ITEM_TYPE_ANY = -1, ITEM_TYPE_STATION = 1, ITEM_TYPE_ROBOT = 2, ITEM_TYPE_FRAME = 3, ITEM_TYPE_TOOL = 4, ITEM_TYPE_OBJECT = 5, ITEM_TYPE_TARGET = 6, ITEM_TYPE_PROGRAM = 8, ITEM_TYPE_INSTRUCTION = 9, ITEM_TYPE_PROGRAM_PYTHON = 10, ITEM_TYPE_FOLDER = 17, ITEM_TYPE_CAMERA = 19;
export const INSTRUCTION_CALL_PROGRAM = 0, INSTRUCTION_INSERT_CODE = 1, INSTRUCTION_START_THREAD = 2, INSTRUCTION_COMMENT = 3, INSTRUCTION_SHOW_MESSAGE = 4;
export const RUNMODE_SIMULATE = 1, RUNMODE_QUICKVALIDATE = 2, RUNMODE_MAKE_ROBOTPROG = 3, RUNMODE_RUN_ROBOT = 6;
export const ROBOTCOM_READY = 2, ROBOTCOM_DISCONNECTED = 0, ROBOTCOM_CONNECTING = 1, ROBOTCOM_UNKNOWN = -1, ROBOTCOM_PROBLEMS = -2;
export const ITEM_TYPE_MACHINING = 11, ITEM_TYPE_NOTES = 22;
export const COLLISION_OFF = 0, COLLISION_ON = 1;
export const PROGRAM_RUN_ON_SIMULATOR = 1, PROGRAM_RUN_ON_ROBOT = 2;
export const CALIBRATE_TCP_BY_POINT = 0, CALIBRATE_TCP_BY_PLANE = 1, CALIBRATE_TCP_BY_LINE = 2;
export const CALIBRATE_FRAME_3P_P1_ON_X = 0, CALIBRATE_FRAME_3P_P1_ORIGIN = 1, CALIBRATE_FRAME_6P = 2, CALIBRATE_TURNTABLE = 3, CALIBRATE_TURNTABLE_2X = 4;
export const MAKE_ROBOT_1R = 1, MAKE_ROBOT_1T = 2, MAKE_ROBOT_2R = 3, MAKE_ROBOT_2T = 4, MAKE_ROBOT_3R = 5, MAKE_ROBOT_3T = 6, MAKE_ROBOT_4R = 7, MAKE_ROBOT_4T = 8, MAKE_ROBOT_6DOF = 9, MAKE_ROBOT_7DOF = 10, MAKE_ROBOT_SCARA = 11, MAKE_ROBOT_1R1T = 12;
export const WINDOWSTATE_HIDDEN = -1, WINDOWSTATE_SHOW = 0, WINDOWSTATE_MINIMIZED = 1, WINDOWSTATE_NORMAL = 2, WINDOWSTATE_MAXIMIZED = 3, WINDOWSTATE_FULLSCREEN = 4, WINDOWSTATE_CINEMA = 5, WINDOWSTATE_FULLSCREEN_CINEMA = 6;
export const FLAG_ROBODK_TREE_ACTIVE = 1, FLAG_ROBODK_3DVIEW_ACTIVE = 2, FLAG_ROBODK_LEFT_CLICK = 4, FLAG_ROBODK_RIGHT_CLICK = 8, FLAG_ROBODK_DOUBLE_CLICK = 16, FLAG_ROBODK_MENU_ACTIVE = 32, FLAG_ROBODK_MENUFILE_ACTIVE = 64, FLAG_ROBODK_MENUEDIT_ACTIVE = 128, FLAG_ROBODK_MENUPROGRAM_ACTIVE = 256, FLAG_ROBODK_MENUTOOLS_ACTIVE = 512, FLAG_ROBODK_MENUUTILITIES_ACTIVE = 1024, FLAG_ROBODK_MENUCONNECT_ACTIVE = 2048, FLAG_ROBODK_WINDOWKEYS_ACTIVE = 4096, FLAG_ROBODK_TREE_VISIBLE = 8192, FLAG_ROBODK_REFERENCES_VISIBLE = 16384, FLAG_ROBODK_STATUSBAR_VISIBLE = 32768, FLAG_ROBODK_NONE = 0, FLAG_ROBODK_ALL = 0xffff;
export const FLAG_ITEM_SELECTABLE = 1, FLAG_ITEM_EDITABLE = 2, FLAG_ITEM_DRAGALLOWED = 4, FLAG_ITEM_DROPALLOWED = 8, FLAG_ITEM_ENABLED = 32, FLAG_ITEM_AUTOTRISTATE = 64, FLAG_ITEM_NOCHILDREN = 128, FLAG_ITEM_USERTRISTATE = 256, FLAG_ITEM_NONE = 0, FLAG_ITEM_ALL = 64 + 32 + 8 + 4 + 2 + 1;
export const SELECT_RESET = -1, SELECT_NONE = 0, SELECT_RECTANGLE = 1, SELECT_ROTATE = 2, SELECT_ZOOM = 3, SELECT_PAN = 4, SELECT_MOVE = 5, SELECT_MOVE_SHIFT = 6, SELECT_MOVE_CLEAR = 7;
export const EVENT_SELECTION_TREE_CHANGED = 1, EVENT_ITEM_MOVED = 2, EVENT_REFERENCE_PICKED = 3, EVENT_REFERENCE_RELEASED = 4, EVENT_TOOL_MODIFIED = 5, EVENT_CREATED_ISOCUBE = 6, EVENT_SELECTION_3D_CHANGED = 7, EVENT_3DVIEW_MOVED = 8, EVENT_ROBOT_MOVED = 9, EVENT_KEY = 10, EVENT_ITEM_MOVED_POSE = 11, EVENT_COLLISIONMAP_RESET = 12, EVENT_COLLISIONMAP_TOO_LARGE = 13, EVENT_CALIB_MEASUREMENT = 14, EVENT_SELECTION3D_CLICK = 15, EVENT_ITEM_CHANGED = 16, EVENT_ITEM_RENAMED = 17, EVENT_ITEM_VISIBILITY = 18, EVENT_STATION_CHANGED = 19, EVENT_PROGSLIDER_CHANGED = 20, EVENT_PROGSLIDER_SET = 21;
export const INS_TYPE_INVALID = -1, INS_TYPE_MOVE = 0, INS_TYPE_MOVEC = 1, INS_TYPE_CHANGESPEED = 2, INS_TYPE_CHANGEFRAME = 3, INS_TYPE_CHANGETOOL = 4, INS_TYPE_CHANGEROBOT = 5, INS_TYPE_PAUSE = 6, INS_TYPE_EVENT = 7, INS_TYPE_CODE = 8, INS_TYPE_PRINT = 9;
export const MOVE_TYPE_INVALID = -1, MOVE_TYPE_JOINT = 1, MOVE_TYPE_LINEAR = 2, MOVE_TYPE_CIRCULAR = 3;
export const PROJECTION_NONE = 0, PROJECTION_CLOSEST = 1, PROJECTION_ALONG_NORMAL = 2, PROJECTION_ALONG_NORMAL_RECALC = 3, PROJECTION_CLOSEST_RECALC = 4, PROJECTION_RECALC = 5;
export const JOINT_FORMAT = -1, TRACKER_TYPE_LASER = 0;
export const RUNMODE_MAKE_ROBOTPROG_AND_UPLOAD = 4, RUNMODE_MAKE_ROBOTPROG_AND_START = 5, RUNMODE_TEACH = 7;

/** robodk.robomath-like Mat (row-major nested arrays API over our column-major Mat4). */
export class Mat {
  constructor(public m: Mat4 = identity()) {}
  static fromRows(rows: number[][]): Mat { return new Mat(fromRows(rows)); }
  rows(): number[][] { return toRows(this.m); }
  toString(): string { return this.rows().map((r) => r.map((v) => v.toFixed(3)).join(', ')).join('\n'); }
  mul(o: Mat): Mat { return new Mat(multiply(this.m, o.m)); }
  inv(): Mat { return new Mat(invert(this.m)); }
  Pos(): number[] { return [this.m[12], this.m[13], this.m[14]]; }
  setPos(p: number[]): Mat { this.m[12] = p[0]; this.m[13] = p[1]; this.m[14] = p[2]; return this; }
  Pose_2_TxyzRxyz(): number[] { return poseToXyzrpw(this.m); }
  Pose_2_KUKA(): number[] { return poseToKuka(this.m); }
  Pose_2_Fanuc(): number[] { return poseToFanuc(this.m); }
  Pose_2_UR(): number[] { return poseToUr(this.m); }
  Pose_2_ABB(): number[] { const [w, x, y, z] = poseToQuat(this.m); return [this.m[12], this.m[13], this.m[14], w, x, y, z]; }
  toArray(): number[] { return Array.from(this.m); }
}
export const transl_ = (x: number, y: number, z: number) => new Mat(transl(x, y, z));
export const robomath = {
  Mat,
  eye: () => new Mat(identity()),
  transl: (x: number, y: number, z: number) => new Mat(transl(x, y, z)),
  rotx: (r: number) => new Mat(rotx(r)),
  roty: (r: number) => new Mat(roty(r)),
  rotz: (r: number) => new Mat(rotz(r)),
  TxyzRxyz_2_Pose: (v: number[]) => new Mat(xyzrpwToPose(v[0], v[1], v[2], v[3], v[4], v[5])),
  KUKA_2_Pose: (v: number[]) => new Mat(kukaToPose(v[0], v[1], v[2], v[3], v[4], v[5])),
  Fanuc_2_Pose: (v: number[]) => new Mat(fanucToPose(v[0], v[1], v[2], v[3], v[4], v[5])),
  UR_2_Pose: (v: number[]) => new Mat(urToPose(v[0], v[1], v[2], v[3], v[4], v[5])),
  ABB_2_Pose: (v: number[]) => new Mat(quatToPose(v[0], v[1], v[2], [v[3], v[4], v[5], v[6]])),
  Pose_2_TxyzRxyz: (m: Mat) => m.Pose_2_TxyzRxyz(),
  Pose_2_KUKA: (m: Mat) => m.Pose_2_KUKA(),
  Pose_2_UR: (m: Mat) => m.Pose_2_UR(),
  pi: Math.PI,
  DEG,
};

const toMat4 = (p: Mat | Mat4 | number[] | number[][]): Mat4 => {
  if (p instanceof Mat) return p.m;
  if (p instanceof Float64Array) return p;
  if (Array.isArray(p) && Array.isArray(p[0])) return fromRows(p as number[][]);
  return Float64Array.from(p as number[]);
};

function dist3(a: Mat4, b: Mat4): number { return Math.hypot(a[12] - b[12], a[13] - b[13], a[14] - b[14]); }
function toCss(color: string | number[]): string {
  if (typeof color === 'string') return color;
  return `#${color.slice(0, 3).map((c) => Math.round(Math.max(0, Math.min(1, c)) * 255).toString(16).padStart(2, '0')).join('')}`;
}

export class RobolinkItem {
  constructor(readonly rdk: Robolink, public item: CoreItem | null) {}
  Valid(): boolean { return !!this.item && (this.item.type === ItemType.STATION || !!this.item.parent); }
  private req<T extends CoreItem>(): T { if (!this.item) throw new Error('Invalid item'); return this.item as T; }
  Name(): string { return this.item?.name ?? ''; }
  setName(n: string): this { this.req().setName(n); return this; }
  Type(): number { return this.item?.type ?? -1; }
  Parent(): RobolinkItem { return new RobolinkItem(this.rdk, this.item?.parent ?? null); }
  Childs(): RobolinkItem[] { return (this.item?.children ?? []).map((c) => new RobolinkItem(this.rdk, c)); }
  Delete(): void { this.item?.delete(); this.item = null; }
  Visible(): boolean { return !!this.item?.visible; }
  setVisible(v: boolean): this { this.req().setVisible(v); return this; }
  setParent(p: RobolinkItem): this { this.req().setParent(p.req()); return this; }
  setParentStatic(p: RobolinkItem): this { this.req().setParentStatic(p.req()); return this; }
  Pose(): Mat {
    const it = this.req();
    if (it instanceof Robot) return new Mat(it.poseTCP());
    return new Mat(it.pose());
  }
  setPose(p: Mat | Mat4 | number[] | number[][]): this {
    const it = this.req();
    const m = toMat4(p);
    if (it instanceof Robot) { const r = it.solveIKFrame(m); if (r.ok) it.setJoints(r.joints); else throw new Error('Target not reachable'); }
    else it.setPose(m);
    return this;
  }
  PoseAbs(): Mat { const it = this.req(); return new Mat(it instanceof Robot ? it.poseTCPAbs() : it.poseAbs()); }
  setPoseAbs(p: Mat | Mat4 | number[]): this { this.req().setPoseAbs(toMat4(p)); return this; }
  PoseTool(): Mat { const it = this.req(); return new Mat(it instanceof Robot ? it.poseTool() : it instanceof Tool ? it.poseTool() : identity()); }
  setPoseTool(p: Mat | Mat4 | number[] | RobolinkItem): this {
    const it = this.req();
    if (p instanceof RobolinkItem) { if (it instanceof Robot && p.item instanceof Tool) it.setTool(p.item); if (it instanceof Program) it.toolId = p.item?.id ?? null; }
    else if (it instanceof Robot) it.setPoseTool(toMat4(p));
    else if (it instanceof Tool) it.setPoseTool(toMat4(p));
    return this;
  }
  PoseFrame(): Mat { const it = this.req(); return new Mat(it instanceof Robot ? it.poseFrame() : identity()); }
  setPoseFrame(p: Mat | Mat4 | number[] | RobolinkItem): this {
    const it = this.req();
    if (p instanceof RobolinkItem) { if (it instanceof Robot) it.setFrame(p.item); if (it instanceof Program) it.frameId = p.item?.id ?? null; }
    else if (it instanceof Robot) { const f = new Frame('Frame'); (it.parent ?? this.rdk.station).addChild(f); f.setPose(multiply(it.pose(), toMat4(p))); it.setFrame(f); }
    return this;
  }
  Joints(): number[] { const it = this.req(); if (it instanceof Robot) return it.joints(); if (it instanceof Target) return it.joints ?? []; return []; }
  setJoints(q: number[]): this { const it = this.req(); if (it instanceof Robot) it.setJoints(q); else if (it instanceof Target) it.setJoints(q); return this; }
  JointsHome(): number[] { return this.req<Robot>().jointsHome(); }
  JointLimits(): [number[], number[]] { const l = this.req<Robot>().jointLimits(); return [l.lower, l.upper]; }
  setJointLimits(lo: number[], hi: number[]): this { this.req<Robot>().setJointLimits(lo, hi); return this; }
  SolveFK(q: number[]): Mat { return new Mat(this.req<Robot>().solveFK(q)); }
  SolveIK(p: Mat | Mat4 | number[], jointsApprox?: number[]): number[] { const r = this.req<Robot>().solveIK(toMat4(p), { seed: jointsApprox }); return r.ok ? r.joints : []; }
  SolveIK_All(p: Mat | Mat4 | number[]): number[][] { return this.req<Robot>().solveIKAll(toMat4(p)); }
  Connect(_ip?: string): boolean { return true; }
  ConnectedState(): number { return ROBOTCOM_READY; }
  setSpeed(speedLinear: number, speedJoints = -1, accelLinear = -1, accelJoints = -1): this {
    const it = this.req();
    if (it instanceof Robot) { if (speedLinear > 0) it.motion.speedLinear = speedLinear; if (speedJoints > 0) it.motion.speedJoints = speedJoints; if (accelLinear > 0) it.motion.accelLinear = accelLinear; if (accelJoints > 0) it.motion.accelJoints = accelJoints; }
    else if (it instanceof Program) it.setSpeed(speedLinear > 0 ? speedLinear : undefined, speedJoints > 0 ? speedJoints : undefined, accelLinear > 0 ? accelLinear : undefined, accelJoints > 0 ? accelJoints : undefined);
    return this;
  }
  setRounding(r: number): this { const it = this.req(); if (it instanceof Robot) it.motion.rounding = r; else if (it instanceof Program) it.setRounding(r); return this; }
  setZoneData(r: number): this { return this.setRounding(r); }
  /** MoveJ: on a robot -> move instantly (simulate); on a program -> add an instruction. */
  MoveJ(target: RobolinkItem | number[] | Mat, _blocking = true): this {
    const it = this.req();
    const tgt = target instanceof RobolinkItem ? (target.item as Target) : target instanceof Mat ? target.m : target;
    if (it instanceof Robot) { if (!it.moveJInstant(tgt as any)) throw new Error('MoveJ: target not reachable'); }
    else if (it instanceof Program) it.addMoveJ(tgt as any);
    this.rdk.render();
    return this;
  }
  MoveL(target: RobolinkItem | number[] | Mat, _blocking = true): this {
    const it = this.req();
    const tgt = target instanceof RobolinkItem ? (target.item as Target) : target instanceof Mat ? target.m : target;
    if (it instanceof Robot) { if (!it.moveJInstant(tgt as any)) throw new Error('MoveL: target not reachable'); }
    else if (it instanceof Program) it.addMoveL(tgt as any);
    this.rdk.render();
    return this;
  }
  MoveC(via: RobolinkItem, target: RobolinkItem): this {
    const it = this.req();
    if (it instanceof Program) it.addMoveC(via.item as Target, target.item as Target);
    else if (it instanceof Robot) it.moveJInstant(target.item as Target);
    return this;
  }
  Pause(ms: number): this { const it = this.req(); if (it instanceof Program) it.pause(ms); return this; }
  setDO(io: string, value: string | number | boolean): this { const it = this.req(); if (it instanceof Program) it.setDO(io, typeof value === 'string' ? value !== '0' : value); else if (it instanceof Robot) it.state.io[io] = typeof value === 'string' ? value !== '0' : value; return this; }
  waitDI(io: string, value: string | number | boolean, timeout = -1): this { const it = this.req(); if (it instanceof Program) it.waitDI(io, typeof value === 'string' ? value !== '0' : value, timeout); return this; }
  RunInstruction(code: string, type = INSTRUCTION_CALL_PROGRAM): this {
    const it = this.req();
    if (it instanceof Program) {
      if (type === INSTRUCTION_COMMENT) it.comment(code);
      else if (type === INSTRUCTION_SHOW_MESSAGE) it.showMessage(code);
      else if (type === INSTRUCTION_INSERT_CODE) it.runInstruction(code, false);
      else if (/^(attach|detach|gripper_open|gripper_close)$/i.test(code)) it.event(code.toLowerCase() as any);
      else it.runInstruction(code, true);
    }
    return this;
  }
  RunCodeCustom(code: string, type = INSTRUCTION_CALL_PROGRAM): this { return this.RunInstruction(code, type); }
  ShowInstructions(_v = true): this { return this; }
  ShowTargets(_v = true): this { return this; }
  InstructionCount(): number { return this.req<Program>().instructions().length; }
  Instruction(id: number): { name: string; type: number; moveType: number; isJointTarget: boolean; pose: Mat | null; joints: number[] | null } {
    const ins = this.req<Program>().instructions()[id];
    const d = ins.data;
    if (d.kind === 'move') {
      const t = d.targetId ? (this.rdk.station.findById(d.targetId) as Target | null) : null;
      return { name: ins.name, type: ins.insType, moveType: d.moveType === 'MoveJ' ? 1 : d.moveType === 'MoveL' ? 2 : 3, isJointTarget: !!t?.isJointTarget, pose: t ? new Mat(t.pose()) : d.pose ? new Mat(Float64Array.from(d.pose)) : null, joints: t?.joints ?? d.joints ?? null };
    }
    return { name: ins.name, type: ins.insType, moveType: 0, isJointTarget: false, pose: null, joints: null };
  }
  InstructionList(): Array<ReturnType<RobolinkItem['Instruction']>> { return this.req<Program>().instructions().map((_, i) => this.Instruction(i)); }
  InstructionDelete(id: number): this { this.req<Program>().instructions()[id]?.delete(); return this; }
  setRobot(r: RobolinkItem): this { this.req<Program>().setRobot(r.item); return this; }
  getLink(type: number): RobolinkItem {
    const it = this.req();
    if (it instanceof Program) { const r = it.robot(); return new RobolinkItem(this.rdk, type === ITEM_TYPE_ROBOT ? r : type === ITEM_TYPE_FRAME ? (it.frameId ? this.rdk.station.findById(it.frameId) : (r instanceof Robot ? r.activeFrame() : null)) : type === ITEM_TYPE_TOOL ? (it.toolId ? this.rdk.station.findById(it.toolId) : (r instanceof Robot ? r.activeTool() : null)) : null); }
    if (it instanceof Robot) return new RobolinkItem(this.rdk, type === ITEM_TYPE_TOOL ? it.activeTool() : type === ITEM_TYPE_FRAME ? it.activeFrame() : it);
    if (it instanceof Tool) return new RobolinkItem(this.rdk, it.parent);
    return new RobolinkItem(this.rdk, null);
  }
  AddTool(pose: Mat | Mat4 | number[], name = 'Tool'): RobolinkItem {
    const r = this.req<Robot>();
    const t = new Tool(name);
    t.setPoseTool(toMat4(pose));
    r.addChild(t);
    r.setTool(t);
    return new RobolinkItem(this.rdk, t);
  }
  AddFrame(name: string): RobolinkItem { return this.rdk.AddFrame(name, this); }
  AddTarget(name: string): RobolinkItem { return this.rdk.AddTarget(name, this); }
  setAsCartesianTarget(): this { this.req<Target>().setAsCartesianTarget(); return this; }
  setAsJointTarget(): this { this.req<Target>().setAsJointTarget(); return this; }
  isJointTarget(): boolean { return !!this.req<Target>().isJointTarget; }
  /** Run/validate the program: returns [valid_instructions, program_time, program_distance, valid_ratio, readable_msg]. */
  Update(): [number, number, number, number, string] {
    const p = this.req<Program>();
    const sim = new ProgramSimulator(this.rdk.station);
    const r = sim.compile(p);
    const errs = r.problems.filter((x) => x.severity === 'error');
    return [r.executed - errs.length, r.duration, r.distance, r.executed ? (r.executed - errs.length) / r.executed : 1, errs.map((e) => e.message).join('; ') || 'OK'];
  }
  RunProgram(): number { const p = this.req<Program>(); this.rdk.onRunProgram?.(p); return 0; }
  RunCode(): number { return this.RunProgram(); }
  MakeProgram(path = '', postId?: string): [boolean, string, string] {
    const p = this.req<Program>();
    const robot = p.robot();
    const post = getPost(postId ?? (robot instanceof Robot ? robot.postProcessor : 'Generic')) ?? getPost('Generic')!;
    const files = post.generate(compileForPost(this.rdk.station, p));
    this.rdk.generated.set(p.id, files);
    return [true, files[0]?.content ?? '', `${path}${files[0]?.name ?? ''}`];
  }
  setParam(key: string, value: any): this { this.req().setParam(key, value); return this; }
  getParam(key: string): any { return this.req().getParam(key); }
  setColor(color: string | number[]): this { const it = this.req(); it.color = Array.isArray(color) ? `#${color.slice(0, 3).map((c) => Math.round(c * 255).toString(16).padStart(2, '0')).join('')}` : color; it.notify('color'); return this; }
  Recolor(color: string | number[]): this { return this.setColor(color); }
  setGeometryPose(p: Mat | Mat4 | number[]): this { const it = this.req(); if (it instanceof SceneObject) { for (const g of it.geometry) g.origin = Array.from(toMat4(p)); it.notify('geometry'); } return this; }
  Busy(): boolean { const it = this.req(); return it instanceof Robot ? it.state.moving : false; }
  Stop(): void { this.rdk.onStop?.(); }
  WaitMove(): void { /* simulation is instantaneous in API mode */ }
  ProgramStart(name: string): number { this.rdk.AddProgram(name, this); return 1; }
  DOF(): number { return this.req<Robot>().dof; }

  // ---- attach / detach (RoboDK AttachClosest / DetachClosest / DetachAll) ----
  /** Attach the closest object to this tool (within `maxDistance` mm of the TCP). Returns the attached item. */
  AttachClosest(maxDistance = 500): RobolinkItem {
    const tool = this.req();
    const tcp = tool instanceof Tool ? tool.poseAbs() : tool instanceof Robot ? tool.poseTCPAbs() : tool.poseAbs();
    const owner = tool instanceof Robot ? tool.activeTool() ?? tool : tool;
    let best: CoreItem | null = null, bd = maxDistance;
    for (const it of this.rdk.station.walk()) {
      if (!(it instanceof SceneObject) || it instanceof Tool || it.isAncestorOf(owner) || owner.isAncestorOf(it)) continue;
      const p = it.poseAbs();
      const d = Math.hypot(p[12] - tcp[12], p[13] - tcp[13], p[14] - tcp[14]);
      if (d < bd) { bd = d; best = it; }
    }
    if (!best) return new RobolinkItem(this.rdk, null);
    best.setParentStatic(owner);
    if (owner instanceof Tool) { owner.closed = true; if (!owner.attached.includes(best.id)) owner.attached.push(best.id); }
    this.rdk.events.push({ type: 'attach', itemId: best.id });
    return new RobolinkItem(this.rdk, best);
  }
  /** Detach the closest attached object and place it under `parent` (station by default). */
  DetachClosest(parent?: RobolinkItem): RobolinkItem {
    const owner = this.owner();
    const kids = owner.children.filter((c) => c instanceof SceneObject && !(c instanceof Tool)) as SceneObject[];
    if (!kids.length) return new RobolinkItem(this.rdk, null);
    const tcp = owner instanceof Tool ? owner.poseAbs() : owner.poseAbs();
    kids.sort((a, b) => dist3(a.poseAbs(), tcp) - dist3(b.poseAbs(), tcp));
    const it = kids[0];
    it.setParentStatic(parent?.item ?? this.rdk.station);
    if (owner instanceof Tool) owner.attached = owner.attached.filter((x) => x !== it.id);
    return new RobolinkItem(this.rdk, it);
  }
  DetachAll(parent?: RobolinkItem): this {
    const owner = this.owner();
    for (const c of [...owner.children]) if (c instanceof SceneObject && !(c instanceof Tool)) c.setParentStatic(parent?.item ?? this.rdk.station);
    if (owner instanceof Tool) { owner.attached = []; owner.closed = false; }
    return this;
  }
  private owner(): CoreItem {
    const it = this.req();
    return it instanceof Robot ? it.activeTool() ?? it : it;
  }
  /** 1 if this item collides with item2 (RoboDK Item.Collision). */
  Collision(item2: RobolinkItem): number {
    if (!item2.item) return 0;
    return itemsCollide(this.req(), item2.item, { assets: this.rdk.assets, meshAccurate: true }).length ? 1 : 0;
  }
  Copy(): this { clipboard.copy(this.req()); return this; }
  Paste(): RobolinkItem { return new RobolinkItem(this.rdk, clipboard.paste(this.req(), this.rdk.station)); }
  GeometryPose(): Mat {
    const it = this.req();
    const g = (it as SceneObject).geometry?.[0];
    return new Mat(g?.origin ? Float64Array.from(g.origin) : identity());
  }
  /** Program joints along the path: [message, rows[time, ins_index, j1..jn], status]. */
  InstructionListJoints(_mm_step = 1, _deg_step = 1, _save_to_file = '', _collision_check = 0, _flags = 0, time_step = 0.05): [string, number[][], number] {
    const p = this.req<Program>();
    const sim = new ProgramSimulator(this.rdk.station);
    const r = sim.compile(p);
    const robot = p.robot();
    const rows = robot ? sim.jointsList(robot.id, time_step) : [];
    const errs = r.problems.filter((x) => x.severity === 'error');
    return [errs.length ? errs.map((e) => e.message).join('; ') : 'Success', rows, errs.length ? -1 : 0];
  }
  InstructionSelect(ins_id = -1): this {
    const p = this.req<Program>();
    const ins = p.instructions()[ins_id < 0 ? p.instructions().length - 1 : ins_id];
    if (ins) this.rdk.station.select(ins);
    return this;
  }
  setInstruction(ins_id: number, name: string, instype: number, movetype: number, isjointtarget: boolean, target: Mat | Mat4 | number[] | null, joints: number[] | null): this {
    const p = this.req<Program>();
    const ins = p.instructions()[ins_id];
    if (!ins) return this;
    ins.name = name;
    if (ins.data.kind === 'move' && (instype === INS_TYPE_MOVE || instype === INS_TYPE_MOVEC)) {
      ins.data.moveType = movetype === MOVE_TYPE_LINEAR ? 'MoveL' : movetype === MOVE_TYPE_CIRCULAR ? 'MoveC' : 'MoveJ';
      const t = ins.data.targetId ? (this.rdk.station.findById(ins.data.targetId) as Target | null) : null;
      if (t) { if (target) t.setPose(toMat4(target)); if (joints) t.setJoints(joints); if (isjointtarget) t.setAsJointTarget(); else t.setAsCartesianTarget(); }
      else { if (target) ins.data.pose = Array.from(toMat4(target)); if (joints) ins.data.joints = joints; }
    }
    ins.notify('data');
    return this;
  }
  /** Configuration flags [rear, lowerarm(elbow down), flip, turns...] like RoboDK JointsConfig. */
  JointsConfig(joints: number[]): number[] {
    const r = this.req<Robot>();
    const f = r.configFlags(joints);
    return [f.front ? 0 : 1, f.elbowUp ? 0 : 1, f.wristFlip ? 1 : 0, ...joints.map((q) => Math.trunc(q / 360))];
  }
  setJointsHome(joints: number[]): this {
    const r = this.req<Robot>();
    const idx = r.chain.joints.map((j, i) => (j.type !== 'fixed' && !j.mimic ? i : -1)).filter((i) => i >= 0);
    idx.forEach((ji, k) => { r.chain.joints[ji].home = joints[k] ?? 0; });
    r.notify('home');
    return this;
  }
  setAccuracyActive(accurate = true): this { this.req().setParam('accuracyActive', accurate); return this; }
  AccuracyActive(): boolean { return !!this.req().getParam('accuracyActive'); }
  setAcceleration(a: number): this { const it = this.req(); if (it instanceof Robot) it.motion.accelLinear = a; else if (it instanceof Program) it.setSpeed(undefined, undefined, a); return this; }
  setAccelerationJoints(a: number): this { const it = this.req(); if (it instanceof Robot) it.motion.accelJoints = a; else if (it instanceof Program) it.setSpeed(undefined, undefined, undefined, a); return this; }
  setSpeedJoints(v: number): this { const it = this.req(); if (it instanceof Robot) it.motion.speedJoints = v; else if (it instanceof Program) it.setSpeed(undefined, v); return this; }
  setLink(item: RobolinkItem): this { const it = this.req(); if (it instanceof Program) it.setRobot(item.item); else if (it instanceof Target) it.robotId = item.item?.id ?? null; else if (it instanceof Robot && item.item instanceof Tool) it.setTool(item.item); return this; }
  ObjectLink(link_id = 0): RobolinkItem {
    const it = this.req();
    const kids = it.children.filter((c) => c instanceof SceneObject);
    return new RobolinkItem(this.rdk, kids[link_id] ?? null);
  }
  /** Serialize this item (or the station) as JSON (RoboDK Item.Save). Returns the JSON text. */
  Save(_filename = ''): string {
    const it = this.req();
    return JSON.stringify(it instanceof Station ? it.serialize() : it.serialize());
  }
  setMachiningParameters(ncfile = '', part?: RobolinkItem, params = ''): [Mat, number, number] {
    const it = this.req();
    it.setParam('ncfile', ncfile);
    it.setParam('machiningParams', params);
    if (part?.item) it.setParam('partId', part.item.id);
    const res = this.rdk.updateMachining(it);
    return [new Mat(identity()), res?.points ?? 0, res?.unreachable ?? 0];
  }
  MachiningParameters(): any { const it = this.req(); return { ncfile: it.getParam('ncfile'), part: it.getParam('partId'), params: it.getParam('machiningParams'), programId: it.getParam('programId') }; }
  FilterTarget(pose: Mat | Mat4 | number[], joints_approx?: number[]): [Mat, number[]] {
    const r = this.req<Robot>();
    const m = toMat4(pose);
    const res = r.solveIK(m, { seed: joints_approx });
    return [new Mat(m), res.ok ? res.joints : joints_approx ?? r.joints()];
  }
  FilterProgram(_filestr = ''): [number, string] { return [0, 'Accuracy filtering is not required: simulation kinematics are nominal']; }
  setRunType(type: number): this { const p = this.req<Program>(); p.runMode = type === PROGRAM_RUN_ON_ROBOT ? 'run_on_robot' : 'simulate'; return this; }
  RunType(): number { return this.req<Program>().runMode === 'run_on_robot' ? PROGRAM_RUN_ON_ROBOT : PROGRAM_RUN_ON_SIMULATOR; }
  WaitFinished(_timeout = 3600): this { return this; }
  /** Test a joint move for collisions/reachability: 0 = OK, -1 = collision, -2 = out of limits. */
  MoveJ_Test(j1: number[], j2: number[], minstep_deg = 1): number {
    const r = this.req<Robot>();
    if (!r.jointsValid(j2)) return -2;
    const tr = planMoveJ(r, j1, j2, r.motion.speedJoints, r.motion.accelJoints);
    return this.rdk.trajectoryCollides(r, tr.samples.map((s) => s.joints), minstep_deg) ? -1 : 0;
  }
  MoveL_Test(j1: number[], pose: Mat | Mat4 | number[], minstep_mm = 1): number {
    const r = this.req<Robot>();
    const target = multiply(r.poseFrame(), toMat4(pose));
    const tr = planMoveL(r, j1, target, r.motion.speedLinear, r.motion.accelLinear);
    if (!tr.ok) return -2;
    return this.rdk.trajectoryCollides(r, tr.samples.map((s) => s.joints), minstep_mm) ? -1 : 0;
  }
  /** Linear search move: move towards the target until a collision (contact) is detected; returns joints at contact. */
  SearchL(target: RobolinkItem | Mat | number[], _blocking = true): number[] {
    const r = this.req<Robot>();
    const tgt = target instanceof RobolinkItem ? multiply(invert(r.poseAbs()), target.item!.poseAbs()) : multiply(r.poseFrame(), target instanceof Mat ? target.m : Float64Array.from(target as number[]));
    const tr = planMoveL(r, r.joints(), tgt, r.motion.speedLinear, r.motion.accelLinear, 0.01);
    for (const s of tr.samples) {
      r.setJoints(s.joints);
      if (checkCollisionsMapped(this.rdk.station, { assets: this.rdk.assets }).some((p) => p.a.item === r || p.b.item === r || r.isAncestorOf(p.a.item) || r.isAncestorOf(p.b.item))) return s.joints;
    }
    return r.joints();
  }
  Scale(scale: number | number[]): this {
    const it = this.req<SceneObject>();
    const sv: [number, number, number] = Array.isArray(scale) ? [scale[0], scale[1] ?? scale[0], scale[2] ?? scale[0]] : [scale, scale, scale];
    for (const g of it.geometry) {
      if (g.primitive) {
        const pr: any = g.primitive;
        if (pr.kind === 'box') pr.size = [pr.size[0] * sv[0], pr.size[1] * sv[1], pr.size[2] * sv[2]];
        else if (pr.kind === 'sphere') pr.radius *= sv[0];
        else if (pr.kind === 'cylinder' || pr.kind === 'cone') { pr.radius *= sv[0]; pr.length *= sv[2]; }
      } else g.scale = [(g.scale?.[0] ?? 1) * sv[0], (g.scale?.[1] ?? 1) * sv[1], (g.scale?.[2] ?? 1) * sv[2]];
    }
    for (const c of it.curves) c.points = c.points.map((p) => [p[0] * sv[0], p[1] * sv[1], (p[2] ?? 0) * sv[2]]);
    it.notify('geometry');
    return this;
  }
  setColorShape(color: string | number[], shape_id = 0): this { const it = this.req<SceneObject>(); const g = it.geometry[shape_id]; if (g) { g.color = toCss(color); it.notify('geometry'); } return this; }
  setColorCurve(color: string | number[], _curve_id = -1): this { this.req().setParam('curveColor', toCss(color)); return this; }
  Color(): number[] { const c = this.req().color ?? '#9aa3ad'; return [parseInt(c.slice(1, 3), 16) / 255, parseInt(c.slice(3, 5), 16) / 255, parseInt(c.slice(5, 7), 16) / 255, 1]; }
  setValue(varname: string, value: any = ''): this { this.req().setParam(varname, value); return this; }
  Value(varname = ''): any { return varname ? this.req().getParam(varname) : this.req().params; }
  setAO(io: string, value: number): this { const it = this.req(); if (it instanceof Program) it.addInstruction({ kind: 'io', io, value, wait: false }); else if (it instanceof Robot) it.state.io[io] = value; return this; }
  getDI(io: string): number | boolean { const r = this.req(); return (r instanceof Robot ? r.state.io[io] : undefined) ?? 0; }
  getAI(io: string): number { const v = this.getDI(io); return typeof v === 'number' ? v : v ? 1 : 0; }
  customInstruction(name: string, path_run: string, _path_icon = '', _blocking = true, cmd_run_on_robot = ''): this {
    const p = this.req<Program>();
    const ins = p.runInstruction(cmd_run_on_robot || path_run, !!cmd_run_on_robot);
    ins.name = name;
    ins.setParam('customPath', path_run);
    return this;
  }
  addMoveJ(target: RobolinkItem | number[] | Mat): this { return this.MoveJ(target); }
  addMoveL(target: RobolinkItem | number[] | Mat): this { return this.MoveL(target); }
  ConnectSafe(robot_ip = '', _max_attempts = 5, _wait = 4): number { this.req<Robot>().connection.ip = robot_ip || this.req<Robot>().connection.ip; return ROBOTCOM_READY; }
  ConnectionParams(): [string, number, string, string, string] { const c = this.req<Robot>().connection; return [c.ip ?? '', c.port ?? 0, '', '', '']; }
  setConnectionParams(ip: string, port = 0, _path = '', _user = '', _pass = ''): this { const r = this.req<Robot>(); r.connection = { ...r.connection, ip, port }; return this; }
  Disconnect(): this { return this; }
  /** Absolute pose of each link for the given joints (RoboDK JointPoses). */
  JointPoses(joints?: number[]): Mat[] {
    const r = this.req<Robot>();
    const fk = r.fk(joints ?? r.joints());
    return fk.linkPoses.map((m) => new Mat(m));
  }
  setJointsConfig(): this { return this; }
  setRobotParams(dhm: number[][], poseBase?: Mat, poseTool?: Mat): this {
    const r = this.req<Robot>();
    const q = r.joints();
    r.chain = chainFromDHM(r.name, dhm);
    if (poseTool) r.chain.flange = poseTool.m;
    if (poseBase) r.setPose(poseBase.m);
    r.setJoints(q.slice(0, r.dof));
    r.notify('chain');
    return this;
  }
  RobotParams(): number[][] { const r = this.req<Robot>(); return r.chain.dh ? dhToDHM(r.chain.dh) : []; }
  Type_(): number { return this.Type(); }
  toString(): string { return `Item(${this.Name()})`; }
}

export class Robolink {
  generated = new Map<string, any>();
  onRender: (() => void) | null = null;
  onRunProgram: ((p: Program) => void) | null = null;
  onStop: (() => void) | null = null;
  onMessage: ((msg: string, popup: boolean) => void) | null = null;
  runMode = RUNMODE_SIMULATE;
  constructor(public station: Station) { this.spray = new SpraySimulator(station); }
  render(): void { this.onRender?.(); }
  Render(_v = true): void { this.render(); }
  /** Item by name (exact, else case-insensitive partial match). Empty name returns the first item of the type. */
  Item(name: string, type: number = ITEM_TYPE_ANY): RobolinkItem {
    if (type === ITEM_TYPE_STATION || (name === '' && type === ITEM_TYPE_ANY)) return new RobolinkItem(this, this.station);
    if (name === '') return new RobolinkItem(this, this.station.itemsOfType(type as ItemType)[0] ?? null);
    const exact = this.station.find(name, type as ItemType);
    if (exact) return new RobolinkItem(this, exact);
    const lower = name.toLowerCase();
    const items = type === ITEM_TYPE_ANY ? [...this.station.walk()].slice(1) : this.station.itemsOfType(type as ItemType);
    return new RobolinkItem(this, items.find((i) => i.name.toLowerCase().includes(lower)) ?? null);
  }
  ItemList(type: number = ITEM_TYPE_ANY, listNames = false): any[] {
    const items = type === ITEM_TYPE_ANY ? [...this.station.walk()].slice(1) : this.station.itemsOfType(type as ItemType);
    return listNames ? items.map((i) => i.name) : items.map((i) => new RobolinkItem(this, i));
  }
  ItemUserPick(_msg = '', type: number = ITEM_TYPE_ANY): RobolinkItem {
    const sel = this.station.selection.find((s) => type === ITEM_TYPE_ANY || s.type === type);
    return new RobolinkItem(this, sel ?? (type === ITEM_TYPE_ANY ? null : this.station.itemsOfType(type as ItemType)[0] ?? null));
  }
  ActiveStation(): RobolinkItem { return new RobolinkItem(this, this.station); }
  AddStation(name: string): RobolinkItem { this.station.setName(name); return new RobolinkItem(this, this.station); }
  AddFrame(name: string, parent?: RobolinkItem): RobolinkItem {
    const f = new Frame(name);
    (parent?.item ?? this.station).addChild(f);
    return new RobolinkItem(this, f);
  }
  AddFolder(name: string, parent?: RobolinkItem): RobolinkItem { const f = new Folder(name); (parent?.item ?? this.station).addChild(f); return new RobolinkItem(this, f); }
  AddTarget(name: string, parent?: RobolinkItem, robot?: RobolinkItem): RobolinkItem {
    const t = new Target(name);
    const r = (robot?.item as Robot | null) ?? this.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    const p = parent?.item ?? r?.activeFrame() ?? this.station;
    p.addChild(t);
    if (r) { t.setPoseAbs(r.poseTCPAbs()); t.setJoints(r.joints()); t.robotId = r.id; }
    return new RobolinkItem(this, t);
  }
  AddProgram(name: string, robot?: RobolinkItem): RobolinkItem {
    const p = new Program(name);
    this.station.addChild(p);
    const r = robot?.item ?? this.station.itemsOfType(ItemType.ROBOT)[0] ?? null;
    if (r) p.setRobot(r);
    return new RobolinkItem(this, p);
  }
  AddRobot(libraryIdOrName: string, parent?: RobolinkItem): RobolinkItem {
    const e = ROBOT_LIBRARY.find((x) => x.id === libraryIdOrName || x.name === libraryIdOrName || x.name.toLowerCase().includes(libraryIdOrName.toLowerCase()));
    if (!e) throw new Error(`Robot ${libraryIdOrName} not in library`);
    const r = createRobotFromLibrary(e.id);
    (parent?.item ?? this.station).addChild(r);
    return new RobolinkItem(this, r);
  }
  AddMobileRobot(name: string): RobolinkItem { const m = new MobileRobot(name); this.station.addChild(m); return new RobolinkItem(this, m); }

  // ---- hooks the application provides (browser only) ----
  assets?: import('../../src/scene/assets').AssetStore;
  onSnapshot: ((cam: CoreItem, w: number, h: number) => string) | null = null;
  onViewPose: { get: () => Mat4; set: (m: Mat4) => void } | null = null;
  onStations: { list: () => Station[]; setActive: (s: Station) => void; add: (name: string) => Station; close: () => void } | null = null;
  onWindow: ((cmd: string, value?: any) => void) | null = null;
  onShowSequence: ((robot: Robot, rows: number[][]) => void) | null = null;
  onInteractiveMode: ((mode: number) => void) | null = null;
  simTime: { get: () => number; set: (t: number) => void } | null = null;
  readonly events = new EventQueue();
  private listening = false;
  readonly spray: SpraySimulator;
  private plugins = new Map<string, { command?: (name: string, value: any) => any; unload?: () => void }>();
  flagsRoboDK = FLAG_ROBODK_ALL;
  private itemFlags = new Map<string, number>();
  private cams = new Map<string, CameraItem>();

  // ---- targets / mechanisms ----
  AddTargetJ(name: string, joints: number[], parent?: RobolinkItem, robot?: RobolinkItem): RobolinkItem {
    const t = this.AddTarget(name, parent, robot);
    const r = (robot?.item as Robot | null) ?? this.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    if (t.item instanceof Target && r) { t.item.setJoints(joints); t.item.setPoseAbs(r.poseTCPAbs(joints)); t.item.setAsJointTarget(); }
    return t;
  }
  BuildMechanism(type: number, list_obj: RobolinkItem[] = [], parameters: number[] = [], joints_build: number[] = [], joints_home: number[] = [], joints_senses: number[] = [], joints_lim_low: number[] = [], joints_lim_high: number[] = [], base?: Mat, tool?: Mat, name = 'New robot', robot?: RobolinkItem): RobolinkItem {
    const spec: MechanismSpec = { type, parameters, jointsBuild: joints_build, jointsHome: joints_home, jointsSenses: joints_senses, lower: joints_lim_low.length ? joints_lim_low : undefined, upper: joints_lim_high.length ? joints_lim_high : undefined, base: base?.m, tool: tool?.m, name };
    const r = buildMechanism(spec);
    // attach link geometry: list_obj[0] = base, list_obj[i] = link i
    list_obj.forEach((o, i) => {
      if (!(o.item instanceof SceneObject)) return;
      const link = r.chain.links[i];
      if (!link) return;
      for (const g of o.item.geometry) link.visuals.push({ mesh: g.mesh, primitive: g.primitive as any, origin: g.origin ? Float64Array.from(g.origin) : identity(), color: g.color, scale: g.scale });
      o.item.delete();
    });
    if (robot?.item instanceof Robot) { const old = robot.item; r.setPose(old.pose()); old.parent?.addChild(r, old.parent.children.indexOf(old)); old.delete(); }
    else this.station.addChild(r);
    return new RobolinkItem(this, r);
  }
  setRobotParams(robot: RobolinkItem, dhm: number[][], poseBase?: Mat, poseTool?: Mat): boolean { robot.setRobotParams(dhm, poseBase, poseTool); return true; }
  AddMachiningProject(name = 'Curve follow settings', robot?: RobolinkItem): RobolinkItem {
    const it = new CoreItem(ItemType.MACHINING, name);
    this.station.addChild(it);
    const r = robot?.item ?? this.station.itemsOfType(ItemType.ROBOT)[0];
    if (r) it.setParam('robotId', r.id);
    return new RobolinkItem(this, it);
  }
  AddMillingProject(name = 'Milling settings', robot?: RobolinkItem): RobolinkItem { return this.AddMachiningProject(name, robot); }
  /** (Re)generate the program of a machining project item from its part curves. */
  updateMachining(it: CoreItem): { points: number; unreachable: number } | null {
    const robot = this.station.findById(String(it.getParam('robotId') ?? '')) as Robot | null;
    const part = this.station.findById(String(it.getParam('partId') ?? '')) as SceneObject | null;
    if (!(robot instanceof Robot) || !(part instanceof SceneObject) || !part.curves.length) return null;
    const params = String(it.getParam('machiningParams') ?? '');
    const old = this.station.findById(String(it.getParam('programId') ?? ''));
    old?.delete();
    const str = (k: string): string | undefined => params.match(new RegExp(`${k}\\s*[=:]\\s*([^\\s,;]+)`, 'i'))?.[1];
    if (part.curves.some((c) => (c as any).kind)) {
      // NC program (milling / cutting / 3D printing): feeds, spindle and extruder come from the G-code
      const res = generateMachining(this.station, robot, part, { name: it.name, approach: num(params, 'Approach', 50), step: num(params, 'Step', 0), rapidSpeed: num(params, 'RapidSpeed', 250), cutSpeed: params.match(/Speed\s*[=:]/i) ? num(params, 'Speed', 20) : undefined, spindleIO: str('SpindleIO') ?? str('Spindle'), extruderIO: str('ExtruderIO') ?? str('Extruder'), freeToolZ: !/RotZ\s*[=:]\s*0|FreeZ\s*[=:]\s*0/i.test(params), zMode: /Normal/i.test(params) ? 'normal' : 'down', rounding: num(params, 'Rounding', 1) });
      it.setParam('programId', res.program.id);
      it.setParam('machiningStats', { cutLength: res.cutLength, rapidLength: res.rapidLength, estimatedTime: res.estimatedTime });
      return { points: res.points, unreachable: res.unreachable };
    }
    const opts: FollowOptions = { name: it.name, step: num(params, 'Step', 0), approach: num(params, 'Approach', 50), speed: num(params, 'Speed', 50), freeToolZ: /RotZ|FreeZ/i.test(params), zMode: /Normal/i.test(params) ? 'normal' : 'down' };
    const res = generateCurveFollow(this.station, robot, part, { points: part.curves.flatMap((c) => c.points) }, opts);
    it.setParam('programId', res.program.id);
    return { points: res.points, unreachable: res.unreachable };
  }

  // ---- cameras ----
  Cam2D_Add(item: RobolinkItem, params = ''): RobolinkItem {
    const cam = new CameraItem('Camera');
    (item.item ?? this.station).addChild(cam);
    this.applyCamParams(cam, params);
    this.cams.set(cam.id, cam);
    return new RobolinkItem(this, cam);
  }
  private applyCamParams(cam: CameraItem, params: string) {
    const get = (k: string) => { const m = params.match(new RegExp(`${k}=([^\\s]+)`, 'i')); return m ? m[1] : null; };
    const fov = get('FOV'); if (fov) cam.fov = +fov;
    const size = get('SIZE'); if (size) { const [w, h] = size.split('x').map(Number); if (w) cam.width = w; if (h) cam.height = h; }
    const far = get('FAR_LENGTH') ?? get('FAR'); if (far) cam.far = +far;
    const near = get('NEAR_LENGTH') ?? get('NEAR'); if (near) cam.near = +near;
    if (/DEPTH/i.test(params)) cam.kind = 'depth';
    cam.setParam('cam2dParams', params);
  }
  Cam2D_SetParams(params: string, cam?: RobolinkItem): boolean {
    const c = cam?.item instanceof CameraItem ? cam.item : [...this.cams.values()][0];
    if (!c) return false;
    this.applyCamParams(c, params);
    return true;
  }
  Cam2D_Close(cam?: RobolinkItem): boolean {
    if (cam?.item) { this.cams.delete(cam.item.id); cam.item.delete(); return true; }
    for (const c of this.cams.values()) c.delete();
    this.cams.clear();
    return true;
  }
  /** Returns a PNG data URL (browser) of the camera view; empty string headless. */
  Cam2D_Snapshot(_file_save_img = '', cam?: RobolinkItem): string {
    const c = cam?.item ?? [...this.cams.values()][0] ?? this.station.itemsOfType<CameraItem>(ItemType.CAMERA)[0];
    if (!c || !this.onSnapshot) return '';
    return this.onSnapshot(c, c instanceof CameraItem ? c.width : 640, c instanceof CameraItem ? c.height : 480);
  }

  // ---- calibration & measurement ----
  /** Points are joints (use_joints=true, FK with the robot's tool) or XYZ in the robot base frame. */
  Calibrate_Reference(joints_points: number[][], method = CALIBRATE_FRAME_3P_P1_ORIGIN, use_joints = false, robot?: RobolinkItem): Mat {
    const r = (robot?.item as Robot | null) ?? this.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    const pts = joints_points.map((p) => (use_joints && r ? (_getPos(r.solveFK(p)) as [number, number, number]) : ([p[0], p[1], p[2]] as [number, number, number])));
    const res = calibrateFrame(pts, method);
    this.lastCalibration = res;
    return new Mat(res.pose);
  }
  lastCalibration: any = null;
  /** poses: joints (input_format = JOINT_FORMAT) or flange poses [x,y,z,w,p,r] (KUKA style). Returns [tcp_xyz, stats[mean,max,...]] */
  CalibrateTool(poses_xyzwpr: number[][], input_format = JOINT_FORMAT, algorithm = CALIBRATE_TCP_BY_POINT, robot?: RobolinkItem, tool?: RobolinkItem): [number[], number[]] {
    const r = (robot?.item as Robot | null) ?? this.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    const flanges = poses_xyzwpr.map((p) => (input_format === JOINT_FORMAT && r ? r.solveFKFlange(p) : kukaToPose(p[0], p[1], p[2], p[3], p[4], p[5])));
    const res = algorithm === CALIBRATE_TCP_BY_LINE ? calibrateTcpByLine(flanges.slice(0, Math.max(3, flanges.length - 2)), flanges.slice(Math.max(3, flanges.length - 2))) : calibrateTcpByPoint(flanges);
    if (tool?.item instanceof Tool) { const m = tool.item.poseTool(); m[12] = res.tcp[0]; m[13] = res.tcp[1]; m[14] = res.tcp[2]; tool.item.setPoseTool(m); }
    this.lastCalibration = res;
    return [[...res.tcp], [res.meanError, res.maxError, ...res.errors]];
  }
  /** measurements: rows [j1..jn, x, y, z] (tracker XYZ in robot base). Returns statistics and applies the calibration. */
  Calibrate_Robot(measurements: number[][], robot?: RobolinkItem, options: { jointOffsets?: boolean; linkLengths?: boolean; tool?: [number, number, number] } = {}): { before: any; after: any; parameters: string[] } {
    const r = (robot?.item as Robot | null) ?? this.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    if (!r) throw new Error('No robot');
    const ms: Measurement[] = measurements.map((row) => ({ joints: row.slice(0, r.dof), xyz: [row[r.dof], row[r.dof + 1], row[r.dof + 2]] }));
    const res = calibrateRobot(r, ms, { jointOffsets: options.jointOffsets ?? true, linkLengths: options.linkLengths ?? false, tool: options.tool });
    applyCalibration(r, res);
    return { before: res.before, after: res.after, parameters: res.parameters };
  }
  /** Simulated laser tracker: measures the TCP of the first robot (or the estimate) with noise. Returns [x,y,z]. */
  LaserTracker_Measure(estimate: number[] = [0, 0, 0], search = false): number[] {
    const r = this.station.itemsOfType<Robot>(ITEM_TYPE_ROBOT as ItemType)[0];
    if (!r) return estimate;
    const noise = Number(this.station.getParam('trackerNoiseMm', 0.03));
    void search;
    return simulateTrackerMeasure(r, noise, this.seed);
  }
  private seed = { s: 42 };
  MeasurePose(_target = -1, _time_avg = 0, _tip_xyz = [0, 0, 0]): [Mat, number] {
    const r = this.station.itemsOfType<Robot>(ITEM_TYPE_ROBOT as ItemType)[0];
    if (!r) return [new Mat(identity()), -1];
    const m = r.poseTCPAbs();
    const p = simulateTrackerMeasure(r, Number(this.station.getParam('trackerNoiseMm', 0.03)), this.seed);
    m[12] = p[0]; m[13] = p[1]; m[14] = p[2];
    return [new Mat(m), 1];
  }
  StereoCamera_Measure(): [Mat, number] { return this.MeasurePose(); }
  Popup_ISO9283_CubeProgram(robot?: RobolinkItem, center: number[] = [1000, 0, 800], side = 400, _blocking = true): RobolinkItem {
    const r = (robot?.item as Robot | null) ?? this.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    if (!r) return new RobolinkItem(this, null);
    const { program } = createIsoCubeProgram(this.station, r, [center[0], center[1], center[2]], side);
    return new RobolinkItem(this, program);
  }
  BallbarProgram(robot?: RobolinkItem, center: number[] = [1000, 0, 800], radius = 150): RobolinkItem {
    const r = (robot?.item as Robot | null) ?? this.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    if (!r) return new RobolinkItem(this, null);
    return new RobolinkItem(this, createBallbarProgram(this.station, r, [center[0], center[1], center[2]], radius).program);
  }

  // ---- collisions ----
  Collision_Line(p1: number[], p2: number[], ref?: Mat): [boolean, RobolinkItem, number[]] {
    const a = ref ? transformPointM(ref.m, p1) : p1, b = ref ? transformPointM(ref.m, p2) : p2;
    const hit = collisionLine(this.station, [a[0], a[1], a[2]], [b[0], b[1], b[2]], { assets: this.assets });
    return [!!hit, new RobolinkItem(this, hit?.item ?? null), hit ? [...hit.point] : [0, 0, 0]];
  }
  Collision_SetPair(item1: RobolinkItem, item2: RobolinkItem, id1 = -1, id2 = -1, collision_check = true): boolean {
    if (!item1.item || !item2.item) return false;
    setCollisionPair(this.station, item1.item, item2.item, id1, id2, collision_check);
    return true;
  }
  Collision_SetPairList(list1: RobolinkItem[], list2: RobolinkItem[], id1: number[] = [], id2: number[] = [], check_state: boolean | boolean[] = true): number {
    let n = 0;
    list1.forEach((a, i) => { const b = list2[i]; if (a?.item && b?.item) { setCollisionPair(this.station, a.item, b.item, id1[i] ?? -1, id2[i] ?? -1, Array.isArray(check_state) ? check_state[i] : check_state); n++; } });
    return n;
  }
  setCollisionActive(check_state = COLLISION_ON): number { getCollisionMap(this.station).active = check_state !== COLLISION_OFF; this.station.notify('collisionMap'); return check_state; }
  setCollisionActivePair(check_state: number, item1: RobolinkItem, item2: RobolinkItem, id1 = -1, id2 = -1): boolean { return this.Collision_SetPair(item1, item2, id1, id2, check_state !== COLLISION_OFF); }
  private lastCollisions: CollisionPair[] = [];
  Collisions(): number { this.lastCollisions = checkCollisionsMapped(this.station, { assets: this.assets, meshAccurate: true }); return this.lastCollisions.length; }
  CollisionItems(): RobolinkItem[] { if (!this.lastCollisions.length) this.Collisions(); const set = new Set<CoreItem>(); for (const p of this.lastCollisions) { set.add(p.a.item); set.add(p.b.item); } return [...set].map((i) => new RobolinkItem(this, i)); }
  CollisionPairs(): Array<[RobolinkItem, RobolinkItem, number, number]> { if (!this.lastCollisions.length) this.Collisions(); return this.lastCollisions.map((p) => [new RobolinkItem(this, p.a.item), new RobolinkItem(this, p.b.item), p.a.linkIndex ?? -1, p.b.linkIndex ?? -1]); }
  trajectoryCollides(robot: Robot, samples: number[][], step: number): boolean {
    const q0 = robot.joints();
    const stride = Math.max(1, Math.round(step));
    const mine = (p: CollisionPair) => p.a.item === robot || p.b.item === robot || robot.isAncestorOf(p.a.item) || robot.isAncestorOf(p.b.item);
    const key = (p: CollisionPair) => `${p.a.item.id}|${p.b.item.id}`;
    try {
      // resting contacts at the start (robot on its pedestal) are not collisions
      const baseline = new Set<string>();
      if (samples.length) { robot.setJoints(samples[0]); for (const p of checkCollisionsMapped(this.station, { assets: this.assets }).filter(mine)) baseline.add(key(p)); }
      for (let i = 0; i < samples.length; i += stride) {
        robot.setJoints(samples[i]);
        if (checkCollisionsMapped(this.station, { assets: this.assets }).some((p) => mine(p) && !baseline.has(key(p)))) return true;
      }
      return false;
    } finally { robot.setJoints(q0); }
  }

  // ---- clipboard / stations / flags / window ----
  Copy(item: RobolinkItem, _copy_children = true): void { if (item.item) clipboard.copy(item.item); }
  Paste(paste_to?: RobolinkItem, paste_times = 1): RobolinkItem | RobolinkItem[] {
    const out: RobolinkItem[] = [];
    for (let i = 0; i < paste_times; i++) out.push(new RobolinkItem(this, clipboard.paste(paste_to?.item ?? this.station, this.station)));
    return paste_times === 1 ? out[0] : out;
  }
  Duplicate(item: RobolinkItem): RobolinkItem { return new RobolinkItem(this, item.item ? cloneItem(item.item) : null); }
  getOpenStations(): RobolinkItem[] { return (this.onStations?.list() ?? [this.station]).map((s) => new RobolinkItem(this, s)); }
  setActiveStation(stn: RobolinkItem): void { if (stn.item instanceof Station) { this.onStations?.setActive(stn.item); this.station = stn.item; } }
  CloseStation(): void { this.onStations ? this.onStations.close() : (this.station = new Station('New station')); }
  CloseRoboDK(): void { this.onWindow?.('close'); }
  getFlagsItem(item: RobolinkItem): number { return this.itemFlags.get(item.item?.id ?? '') ?? FLAG_ITEM_ALL; }
  setFlagsItem(item: RobolinkItem, flags = FLAG_ITEM_ALL): number { if (item.item) { this.itemFlags.set(item.item.id, flags); item.item.setParam('flags', flags); } return flags; }
  getFlagsRoboDK(): number { return this.flagsRoboDK; }
  setFlagsRoboDK(flags = FLAG_ROBODK_ALL): number { this.flagsRoboDK = flags; this.onWindow?.('flags', flags); return flags; }
  HideRoboDK(): void { this.onWindow?.('hide'); }
  ShowRoboDK(): void { this.onWindow?.('show'); }
  setWindowState(state = WINDOWSTATE_NORMAL): void { this.onWindow?.('state', state); }
  setInteractiveMode(mode = SELECT_MOVE, _default_ref_flags = 0, _custom_items?: any, _custom_ref_flags?: any): void { this.onInteractiveMode?.(mode); }
  setViewPose(pose: Mat | Mat4 | number[]): void { this.onViewPose?.set(toMat4(pose)); }
  ViewPose(_preset = -1): Mat { return new Mat(this.onViewPose?.get() ?? identity()); }
  Joints(robots?: RobolinkItem[]): number[][] { return (robots ?? this.ItemList(ITEM_TYPE_ROBOT)).map((r: RobolinkItem) => r.Joints()); }
  setJoints(robots: RobolinkItem[], joints: number[][]): void { robots.forEach((r, i) => r.setJoints(joints[i])); }
  setPoses(items: RobolinkItem[], poses: Array<Mat | number[]>): void { items.forEach((it, i) => it.setPose(poses[i])); }
  MergeItems(list: RobolinkItem[]): RobolinkItem {
    const objs = list.map((i) => i.item).filter((i): i is SceneObject => i instanceof SceneObject);
    if (!objs.length) return new RobolinkItem(this, null);
    const merged = new SceneObject(objs.map((o) => o.name).join('+'));
    (objs[0].parent ?? this.station).addChild(merged);
    merged.setPoseAbs(objs[0].poseAbs());
    const inv = invert(merged.poseAbs());
    for (const o of objs) {
      const rel = multiply(inv, o.poseAbs());
      for (const g of o.geometry) merged.geometry.push({ ...g, origin: Array.from(multiply(rel, g.origin ? Float64Array.from(g.origin) : identity())) });
      for (const c of o.curves) merged.curves.push({ name: `${o.name}:${c.name}`, points: c.points.map((p) => { const q = transformPointM(rel, [p[0], p[1], p[2] ?? 0]); return [q[0], q[1], q[2]]; }) });
      o.delete();
    }
    return new RobolinkItem(this, merged);
  }

  // ---- program execution helpers ----
  RunCode(code: string, code_is_fcn_call = false): number { const p = this.station.itemsOfType<Program>(ItemType.PROGRAM).find((x) => x.name === code); if (p && code_is_fcn_call) { this.onRunProgram?.(p); return 0; } this.onMessage?.(code, false); return 0; }
  RunMessage(message: string, message_is_comment = false): void { if (!message_is_comment) this.onMessage?.(message, false); }
  RunProgram(fcn_param: string, _wait_for_finished = false): number { const p = this.station.find(fcn_param.split('(')[0], ItemType.PROGRAM); if (p instanceof Program) { this.onRunProgram?.(p); return 0; } return -1; }
  ShowSequence(matrix: number[][], _display_type = 0, _timeout = -1): void {
    const r = this.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    if (!r) return;
    if (this.onShowSequence) this.onShowSequence(r, matrix);
    else if (matrix.length) r.setJoints(matrix[matrix.length - 1].slice(0, r.dof));
  }
  FilterTarget(pose: Mat | number[], joints_approx?: number[], robot?: RobolinkItem): [Mat, number[]] { const r = robot ?? new RobolinkItem(this, this.station.itemsOfType(ItemType.ROBOT)[0]); return r.FilterTarget(pose, joints_approx); }
  getParams(): Array<[string, any]> { return Object.entries({ ...this.station.settings, ...this.station.params }); }
  SimulationTime(): number { return this.simTime?.get() ?? this.station.simTime; }
  setSimulationTime(t: number): void { if (this.simTime) this.simTime.set(t); else this.station.simTime = t; }

  // ---- spray ----
  Spray_Add(item_tool?: RobolinkItem, item_object?: RobolinkItem, params = '', _points?: number[][], _geometry?: number[][]): number {
    const tool = (item_tool?.item as Tool | null) ?? (this.station.itemsOfType<Robot>(ItemType.ROBOT)[0]?.activeTool() ?? null);
    const obj = (item_object?.item as SceneObject | null) ?? this.station.itemsOfType<SceneObject>(ItemType.OBJECT)[0];
    if (!tool || !obj) return -1;
    const g = this.spray.add(tool, obj, { angleDeg: num(params, 'ANGLE', 15), range: num(params, 'RANGE', 400), rate: num(params, 'RATE', 1), sampleSpacing: num(params, 'STEP', 20) });
    return g.id;
  }
  Spray_SetState(state = 1, id_spray: number | 'all' = 'all'): number { this.spray.setState(id_spray === -1 ? 'all' : id_spray, !!state); return 1; }
  Spray_GetStats(id_spray: number | 'all' = 'all'): [string, number[]] { const s = this.spray.stats(id_spray === -1 ? 'all' : id_spray); return ['coverage_pct,mean,min,max,overspray_s,samples', [s.coverage, s.mean, s.min, s.max, s.oversprayTime, s.samples]]; }
  Spray_Clear(id_spray: number | 'all' = 'all'): number { this.spray.clear(id_spray === -1 ? 'all' : id_spray); this.spray.remove(id_spray === -1 ? 'all' : id_spray); return 1; }

  // ---- events / plugins ----
  EventsListen(): boolean { this.listening = true; return true; }
  WaitForEvent(timeout = 3600): Promise<[number, RobolinkItem] | null> {
    return this.events.wait(timeout * 1000).then((e) => (e ? [eventCode(e.type), new RobolinkItem(this, e.itemId ? this.station.findById(e.itemId) : null)] : null));
  }
  EventsLoop(): boolean { return this.listening; }
  PluginLoad(plugin_name = '', load = 1): boolean {
    if (load === 0) { const p = this.plugins.get(plugin_name); p?.unload?.(); this.plugins.delete(plugin_name); return true; }
    return this.plugins.has(plugin_name) || !!this.onPluginLoad?.(plugin_name);
  }
  onPluginLoad: ((name: string) => boolean) | null = null;
  registerPlugin(name: string, plugin: { command?: (name: string, value: any) => any; unload?: () => void }): void { this.plugins.set(name, plugin); }
  PluginCommand(plugin_name: string, plugin_command = '', value: any = ''): any { const p = this.plugins.get(plugin_name); return p?.command ? p.command(plugin_command, value) : `Plugin ${plugin_name} not loaded`; }
  Plugins(): string[] { return [...this.plugins.keys()]; }
  AddFile(_path: string, _parent?: RobolinkItem): RobolinkItem { this.onMessage?.('AddFile: drop the file onto the viewport or use File > Import', false); return new RobolinkItem(this, null); }
  AddShape(triangles: number[][], parent?: RobolinkItem, name = 'Shape'): RobolinkItem {
    const o = new SceneObject(name);
    const pos = new Float32Array(triangles.flat());
    const id = `shape_${o.id}`;
    this.assetsRegister?.(id, pos);
    o.geometry = [{ mesh: id, origin: Array.from(identity()), color: '#8ca0b3' }];
    (parent?.item ?? this.station).addChild(o);
    return new RobolinkItem(this, o);
  }
  assetsRegister: ((id: string, positions: Float32Array) => void) | null = null;
  AddCurve(points: number[][], parent?: RobolinkItem, _addToRef = false): RobolinkItem {
    const o = (parent?.item instanceof SceneObject ? parent.item : null) ?? new SceneObject('Curve');
    if (!o.parent) (parent?.item ?? this.station).addChild(o);
    o.curves.push({ name: `curve ${o.curves.length + 1}`, points });
    o.notify('geometry');
    return new RobolinkItem(this, o);
  }
  AddPoints(points: number[][], parent?: RobolinkItem): RobolinkItem {
    const o = (parent?.item instanceof SceneObject ? parent.item : null) ?? new SceneObject('Points');
    if (!o.parent) (parent?.item ?? this.station).addChild(o);
    points.forEach((p, i) => o.points.push({ name: `p${i + 1}`, point: p }));
    return new RobolinkItem(this, o);
  }
  ShowMessage(msg: string, popup = true): void { this.onMessage?.(msg, popup); }
  setRunMode(m: number): void { this.runMode = m; }
  RunMode(): number { return this.runMode; }
  setSimulationSpeed(_s: number): void { /* handled by the app */ }
  SimulationSpeed(): number { return 1; }
  Save(_file: string, item?: RobolinkItem): string { return JSON.stringify((item?.item ?? this.station).serialize()); }
  getParam(key: string): any { return this.station.settings[key] ?? this.station.getParam(key); }
  setParam(key: string, value: any): void { this.station.setParam(key, value); }
  Command(cmd: string, value: any = ''): string { this.station.setParam(`cmd:${cmd}`, value); return 'OK'; }
  Version(): string { return 'VerticalBot Studio 0.1 (RoboDK API compatible)'; }
  License(): [string, string] { return ['VerticalBot Studio', 'MIT']; }
  Selection(): RobolinkItem[] { return this.station.selection.map((s) => new RobolinkItem(this, s)); }
  setSelection(items: RobolinkItem[]): void { this.station.setSelection(items.map((i) => i.item!).filter(Boolean)); }
  Update(): void { this.render(); }
  Finish(): void {}
  Disconnect(): void {}
  Connect(): boolean { return true; }
  ProjectPoints(points: number[][], obj?: RobolinkItem, _projection_type = PROJECTION_ALONG_NORMAL): number[][] {
    // project along -Z of each point onto the object's colliders (ray cast)
    return points.map((p) => { const hit = collisionLine(this.station, [p[0], p[1], p[2] + 1e4], [p[0], p[1], p[2] - 1e4], { assets: this.assets }); return hit && (!obj?.item || hit.item === obj.item) ? [...hit.point, ...(p.slice(3))] : p; });
  }
  IsInside(a: RobolinkItem, b: RobolinkItem): number { if (!a.item || !b.item) return 0; const ca = collidersOf(a.item)[0]; if (!ca) return 0; const cb = collidersOf(b.item); const c = ca.shape.kind === 'sphere' ? ca.shape.c : ca.shape.kind === 'capsule' ? ca.shape.a : ca.shape.kind === 'obb' ? ca.shape.center : ca.shape.obb.center; return cb.some((s) => s.shape.kind === 'obb' && insideObb(s.shape, c)) ? 1 : 0; }
  Delete(items: RobolinkItem[]): void { for (const i of items) i.Delete(); }
}

export { ItemType, InstructionType, Robot, Program, Target, Frame, Tool, SceneObject, Instruction };

function num(params: string, key: string, def: number): number { const m = params.match(new RegExp(`${key}\\s*[=:]\\s*([-+]?\\d*\\.?\\d+)`, 'i')); return m ? parseFloat(m[1]) : def; }
function transformPointM(m: Mat4, p: number[]): [number, number, number] { return [m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12], m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13], m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14]]; }
function insideObb(o: { center: number[]; axes: number[][]; half: number[] }, p: number[]): boolean { const d = [p[0] - o.center[0], p[1] - o.center[1], p[2] - o.center[2]]; return [0, 1, 2].every((i) => Math.abs(d[0] * o.axes[i][0] + d[1] * o.axes[i][1] + d[2] * o.axes[i][2]) <= o.half[i]); }
function eventCode(type: string): number {
  switch (type) { case 'selection': return EVENT_SELECTION_TREE_CHANGED; case 'moved': return EVENT_ITEM_MOVED; case 'robotMoved': return EVENT_ROBOT_MOVED; case 'renamed': return EVENT_ITEM_RENAMED; case 'visibility': return EVENT_ITEM_VISIBILITY; case 'station': return EVENT_STATION_CHANGED; case 'attach': return EVENT_ITEM_CHANGED; default: return EVENT_ITEM_CHANGED; }
}
export { poseFromZ, transformDir };
