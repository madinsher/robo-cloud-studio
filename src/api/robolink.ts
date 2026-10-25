/**
 * RoboDK-compatible scripting API (JavaScript). Mirrors robodk.robolink.Robolink / Item so that RoboDK scripts port
 * with minimal changes. Used by the in-app console, the studio server (WebSocket / TCP / RoboDK binary protocol),
 * the Python drop-in (python/robodk/robolink.py) and the station Python runners (src/api/runners.ts).
 *
 * Constant values are RoboDK's (generated list in python/robodk/_official.py). Semantics follow RoboDK:
 * SolveFK/SolveIK work on the flange (tool/reference optional), robot PoseAbs is the robot base, program
 * setPoseFrame/setPoseTool insert instructions, robot moves are timed motions played by the station runtime
 * (src/api/exec.ts), RUNMODE_MAKE_ROBOTPROG records robot moves into the program opened by ProgramStart.
 */
import { Station, Item as CoreItem, ItemType, Frame, Target, Tool, SceneObject, Folder } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { Program, Instruction, InstructionType, InstructionData, MoveInstruction } from '../core/items/program';
import { createRobotFromLibrary, ROBOT_LIBRARY } from '../core/items/library';
import { Mat4, identity, multiply, invert, transl, rotx, roty, rotz, xyzrpwToPose, poseToXyzrpw, kukaToPose, poseToKuka, fanucToPose, poseToFanuc, urToPose, poseToUr, quatToPose, poseToQuat, toRows, fromRows, DEG } from '../core/math/pose';
import { ProgramSimulator } from '../core/motion/simulator';
import { compileForPost, getPost } from '../posts/index';
import { MobileRobot } from '../mobile/items';
import { Camera as CameraItem } from '../core/items/item';
import { renderCamera } from '../vision/sensing';
import { encodePNG, encodeGrey32 } from '../vision/png';
import { setFocalLengthPx, cameraFovs } from '../vision/camera_model';
import { checkCollisionsMapped, itemsCollide, collisionLine, setCollisionPair, getCollisionMap, collidersOf, CollisionPair } from '../core/collision/collision';
import { calibrateTcpByPoint, calibrateTcpByLine } from '../core/calibration/tcp';
import { calibrateFrame } from '../core/calibration/frame';
import { calibrateRobot, applyCalibration, Measurement } from '../core/calibration/robot';
import { createIsoCubeProgram, createBallbarProgram, simulateTrackerMeasure } from '../core/calibration/iso9283';
import { buildMechanism, chainFromDHM, dhToDHM, MechanismSpec } from '../core/kinematics/mechanism';
import * as Mech from '../core/kinematics/mechanism';
import { SpraySimulator } from '../core/motion/spray';
import { clipboard, cloneItem } from '../core/items/clone';
import { EventQueue } from '../core/events-queue';
import { generateCurveFollow, FollowOptions } from '../core/motion/pathfollow';
import { generateMachining } from '../core/motion/machining';
import { planMoveJ, planMoveL, sampleAt, Trajectory } from '../core/motion/trajectory';
import { getPos as _getPos, poseFromZ, transformDir } from '../core/math/pose';
import { ApiExec, moveTo, parseCall, RunHandleLike } from './exec';

// ---- constants (RoboDK values; python/robodk/_official.py is generated from RoboDK's robolink.py) ----
export const ITEM_TYPE_ANY = -1, ITEM_TYPE_STATION = 1, ITEM_TYPE_ROBOT = 2, ITEM_TYPE_FRAME = 3, ITEM_TYPE_TOOL = 4, ITEM_TYPE_OBJECT = 5, ITEM_TYPE_TARGET = 6, ITEM_TYPE_CURVE = 7, ITEM_TYPE_PROGRAM = 8, ITEM_TYPE_INSTRUCTION = 9, ITEM_TYPE_PROGRAM_PYTHON = 10, ITEM_TYPE_MACHINING = 11, ITEM_TYPE_BALLBARVALIDATION = 12, ITEM_TYPE_CALIBPROJECT = 13, ITEM_TYPE_VALID_ISO9283 = 14, ITEM_TYPE_FOLDER = 17, ITEM_TYPE_ROBOT_ARM = 18, ITEM_TYPE_CAMERA = 19, ITEM_TYPE_GENERIC = 20, ITEM_TYPE_ROBOT_AXES = 21, ITEM_TYPE_NOTES = 22;
export const INSTRUCTION_CALL_PROGRAM = 0, INSTRUCTION_INSERT_CODE = 1, INSTRUCTION_START_THREAD = 2, INSTRUCTION_COMMENT = 3, INSTRUCTION_SHOW_MESSAGE = 4;
export const RUNMODE_SIMULATE = 1, RUNMODE_QUICKVALIDATE = 2, RUNMODE_MAKE_ROBOTPROG = 3, RUNMODE_MAKE_ROBOTPROG_AND_UPLOAD = 4, RUNMODE_MAKE_ROBOTPROG_AND_START = 5, RUNMODE_RUN_ROBOT = 6, RUNMODE_TEACH = 7;
export const ROBOTCOM_PROBLEMS = -3, ROBOTCOM_DISCONNECTED = -2, ROBOTCOM_NOT_CONNECTED = -1, ROBOTCOM_READY = 0, ROBOTCOM_WORKING = 1, ROBOTCOM_WAITING = 2, ROBOTCOM_UNKNOWN = -1000;
export const COLLISION_OFF = 0, COLLISION_ON = 1;
export const SPRAY_OFF = 0, SPRAY_ON = 1;
export const PROGRAM_RUN_ON_SIMULATOR = 1, PROGRAM_RUN_ON_ROBOT = 2;
export const CALIBRATE_TCP_BY_POINT = 0, CALIBRATE_TCP_BY_PLANE = 1, CALIBRATE_TCP_BY_PLANE_SCARA = 4, CALIBRATE_TCP_BY_LINE = 2;
export const CALIBRATE_FRAME_3P_P1_ON_X = 0, CALIBRATE_FRAME_3P_P1_ORIGIN = 1, CALIBRATE_FRAME_6P = 2, CALIBRATE_TURNTABLE = 3, CALIBRATE_TURNTABLE_2X = 4;
export const MAKE_ROBOT_1R = 1, MAKE_ROBOT_2R = 2, MAKE_ROBOT_3R = 3, MAKE_ROBOT_1T = 4, MAKE_ROBOT_2T = 5, MAKE_ROBOT_3T = 6, MAKE_ROBOT_6DOF = 7, MAKE_ROBOT_7DOF = 8, MAKE_ROBOT_SCARA = 9, MAKE_ROBOT_GRIPPER = 10, MAKE_ROBOT_6COBOT = 11, MAKE_ROBOT_1T1R = 12, MAKE_ROBOT_5XCNC = 13, MAKE_ROBOT_3T1R = 15, MAKE_ROBOT_GENERIC = 16;
/** Studio extensions (no RoboDK equivalent): 4-axis serial mechanisms. */
export const MAKE_ROBOT_4R = 107, MAKE_ROBOT_4T = 108;
export const WINDOWSTATE_HIDDEN = -1, WINDOWSTATE_SHOW = 0, WINDOWSTATE_MINIMIZED = 1, WINDOWSTATE_NORMAL = 2, WINDOWSTATE_MAXIMIZED = 3, WINDOWSTATE_FULLSCREEN = 4, WINDOWSTATE_CINEMA = 5, WINDOWSTATE_FULLSCREEN_CINEMA = 6, WINDOWSTATE_VIDEO = 7;
export const FLAG_ROBODK_TREE_ACTIVE = 1, FLAG_ROBODK_3DVIEW_ACTIVE = 2, FLAG_ROBODK_LEFT_CLICK = 4, FLAG_ROBODK_RIGHT_CLICK = 8, FLAG_ROBODK_DOUBLE_CLICK = 16, FLAG_ROBODK_MENU_ACTIVE = 32, FLAG_ROBODK_MENUFILE_ACTIVE = 64, FLAG_ROBODK_MENUEDIT_ACTIVE = 128, FLAG_ROBODK_MENUPROGRAM_ACTIVE = 256, FLAG_ROBODK_MENUTOOLS_ACTIVE = 512, FLAG_ROBODK_MENUUTILITIES_ACTIVE = 1024, FLAG_ROBODK_MENUCONNECT_ACTIVE = 2048, FLAG_ROBODK_WINDOWKEYS_ACTIVE = 4096, FLAG_ROBODK_TREE_VISIBLE = 8192, FLAG_ROBODK_REFERENCES_VISIBLE = 16384, FLAG_ROBODK_STATUSBAR_VISIBLE = 32768, FLAG_ROBODK_NONE = 0, FLAG_ROBODK_ALL = 0xffff;
export const FLAG_ITEM_SELECTABLE = 1, FLAG_ITEM_EDITABLE = 2, FLAG_ITEM_DRAGALLOWED = 4, FLAG_ITEM_DROPALLOWED = 8, FLAG_ITEM_ENABLED = 32, FLAG_ITEM_AUTOTRISTATE = 64, FLAG_ITEM_NOCHILDREN = 128, FLAG_ITEM_USERTRISTATE = 256, FLAG_ITEM_NONE = 0, FLAG_ITEM_ALL = 64 + 32 + 8 + 4 + 2 + 1;
export const SELECT_RESET = -1, SELECT_NONE = 0, SELECT_RECTANGLE = 1, SELECT_ROTATE = 2, SELECT_ZOOM = 3, SELECT_PAN = 4, SELECT_MOVE = 5, SELECT_MOVE_SHIFT = 6, SELECT_MOVE_CLEAR = 7;
export const EVENT_SELECTION_TREE_CHANGED = 1, EVENT_ITEM_MOVED = 2, EVENT_REFERENCE_PICKED = 3, EVENT_REFERENCE_RELEASED = 4, EVENT_TOOL_MODIFIED = 5, EVENT_CREATED_ISOCUBE = 6, EVENT_SELECTION_3D_CHANGED = 7, EVENT_3DVIEW_MOVED = 8, EVENT_ROBOT_MOVED = 9, EVENT_KEY = 10, EVENT_ITEM_MOVED_POSE = 11, EVENT_COLLISIONMAP_RESET = 12, EVENT_COLLISIONMAP_TOO_LARGE = 13, EVENT_CALIB_MEASUREMENT = 14, EVENT_SELECTION3D_CLICK = 15, EVENT_ITEM_CHANGED = 16, EVENT_ITEM_RENAMED = 17, EVENT_ITEM_VISIBILITY = 18, EVENT_STATION_CHANGED = 19, EVENT_PROGSLIDER_CHANGED = 20, EVENT_PROGSLIDER_SET = 21;
export const INS_TYPE_INVALID = -1, INS_TYPE_MOVE = 0, INS_TYPE_MOVEC = 1, INS_TYPE_CHANGESPEED = 2, INS_TYPE_CHANGEFRAME = 3, INS_TYPE_CHANGETOOL = 4, INS_TYPE_CHANGEROBOT = 5, INS_TYPE_PAUSE = 6, INS_TYPE_EVENT = 7, INS_TYPE_CODE = 8, INS_TYPE_PRINT = 9, INS_TYPE_ROUNDING = 10, INS_TYPE_IO = 11, INS_TYPE_CUSTOM = 12;
export const MOVE_TYPE_INVALID = -1, MOVE_TYPE_JOINT = 1, MOVE_TYPE_LINEAR = 2, MOVE_TYPE_CIRCULAR = 3, MOVE_TYPE_LINEARSEARCH = 5;
export const PROJECTION_NONE = 0, PROJECTION_CLOSEST = 1, PROJECTION_ALONG_NORMAL = 2, PROJECTION_ALONG_NORMAL_RECALC = 3, PROJECTION_CLOSEST_RECALC = 4, PROJECTION_RECALC = 5;
export const JOINT_FORMAT = -1, TRACKER_TYPE_LASER = 0;
export const EULER_RX_RYp_RZpp = 0, EULER_RZ_RYp_RXpp = 1, EULER_RZ_RYp_RZpp = 2, EULER_RZ_RXp_RZpp = 3, EULER_RX_RY_RZ = 4, EULER_RZ_RY_RX = 5, EULER_QUEATERNION = 6;
export const FEATURE_NONE = 0, FEATURE_SURFACE = 1, FEATURE_CURVE = 2, FEATURE_POINT = 3, FEATURE_OBJECT_MESH = 7, FEATURE_SURFACE_PREVIEW = 8, FEATURE_MESH = 9, FEATURE_HOVER_OBJECT_MESH = 10, FEATURE_HOVER_OBJECT = 11;
export const SEQUENCE_DISPLAY_DEFAULT = -1, SEQUENCE_DISPLAY_TOOL_POSES = 0, SEQUENCE_DISPLAY_ROBOT_POSES = 256, SEQUENCE_DISPLAY_ROBOT_JOINTS = 2048;
export const ERROR_KINEMATIC = 0b001, ERROR_PATH_LIMIT = 0b010, ERROR_PATH_SINGULARITY = 0b100, ERROR_PATH_NEARSINGULARITY = 0b1000, ERROR_COLLISION = 0b100000;

/** RoboDK MAKE_ROBOT_* -> studio mechanism builder type. */
const MECH_TYPE: Record<number, number> = { [MAKE_ROBOT_1R]: Mech.MAKE_ROBOT_1R, [MAKE_ROBOT_2R]: Mech.MAKE_ROBOT_2R, [MAKE_ROBOT_3R]: Mech.MAKE_ROBOT_3R, [MAKE_ROBOT_1T]: Mech.MAKE_ROBOT_1T, [MAKE_ROBOT_2T]: Mech.MAKE_ROBOT_2T, [MAKE_ROBOT_3T]: Mech.MAKE_ROBOT_3T, [MAKE_ROBOT_6DOF]: Mech.MAKE_ROBOT_6DOF, [MAKE_ROBOT_7DOF]: Mech.MAKE_ROBOT_7DOF, [MAKE_ROBOT_SCARA]: Mech.MAKE_ROBOT_SCARA, [MAKE_ROBOT_6COBOT]: Mech.MAKE_ROBOT_6DOF, [MAKE_ROBOT_1T1R]: Mech.MAKE_ROBOT_1R1T, [MAKE_ROBOT_GENERIC]: Mech.MAKE_ROBOT_GENERIC, [MAKE_ROBOT_4R]: Mech.MAKE_ROBOT_4R, [MAKE_ROBOT_4T]: Mech.MAKE_ROBOT_4T };

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

type PoseLike = Mat | Mat4 | number[] | number[][];
const toMat4 = (p: PoseLike): Mat4 => {
  if (p instanceof Mat) return p.m;
  if (p instanceof Float64Array) return p;
  if (Array.isArray(p) && Array.isArray(p[0])) return fromRows(p as number[][]);
  return Float64Array.from(p as number[]);
};
const isPose = (p: any): boolean => p instanceof Mat || (p instanceof Float64Array && p.length === 16) || (Array.isArray(p) && p.length === 4 && Array.isArray(p[0]) && (p[0] as any[]).length === 4);
const flat = (q: any): number[] => (q == null ? [] : Array.isArray(q) ? (q as any[]).flat(2).map(Number) : ArrayBuffer.isView(q) ? Array.from(q as any as ArrayLike<number>) : [Number(q)]);
const ioValue = (v: any): number | boolean => (typeof v === 'boolean' || typeof v === 'number' ? v : /^(true|on)$/i.test(String(v)) ? true : /^(false|off)$/i.test(String(v)) ? false : Number.isFinite(Number(v)) ? Number(v) : String(v) !== '0' && String(v) !== '');
const ioEq = (a: any, b: any): boolean => (a === undefined ? Number(b) === 0 || b === false : Number(a) === Number(b) || String(a) === String(b));

function dist3(a: Mat4, b: Mat4): number { return Math.hypot(a[12] - b[12], a[13] - b[13], a[14] - b[14]); }
function toCss(color: string | number[]): string {
  if (typeof color === 'string') return color;
  return `#${color.slice(0, 3).map((c) => Math.round(Math.max(0, Math.min(1, c)) * 255).toString(16).padStart(2, '0')).join('')}`;
}
/** RoboDK INS_TYPE of an instruction (studio extensions map to CODE / IO / PAUSE). */
function rdkInsType(ins: Instruction): number {
  const t = ins.insType;
  if (t <= InstructionType.IO) return t;
  if (t === InstructionType.WAIT_SIGNAL || t === InstructionType.SET_SIGNAL) return INS_TYPE_IO;
  if (t === InstructionType.WAIT) return INS_TYPE_PAUSE;
  return INS_TYPE_CODE;
}
const moveTypeOf = (d: InstructionData) => (d.kind === 'jointPath' ? MOVE_TYPE_JOINT : d.kind !== 'move' ? MOVE_TYPE_INVALID : d.moveType === 'MoveJ' ? MOVE_TYPE_JOINT : d.moveType === 'MoveL' ? MOVE_TYPE_LINEAR : MOVE_TYPE_CIRCULAR);

/** File access for AddFile / Save / MakeProgram / Cam2D_Snapshot (Node fs on the server; browser: downloads). */
export interface ApiFileSystem {
  readFile(path: string): Uint8Array;
  writeFile(path: string, data: string | Uint8Array): void;
  exists(path: string): boolean;
  mkdirp?(dir: string): void;
}
/** Offline programming (RUNMODE_MAKE_ROBOTPROG) recorder opened by ProgramStart. */
interface OfflineRecorder { program: Program; folder: string; post: string; robot: Robot | null }

export class RobolinkItem {
  constructor(readonly rdk: Robolink, public item: CoreItem | null) {}
  Valid(): boolean { return !!this.item && (this.item.type === ItemType.STATION || !!this.item.parent); }
  private req<T extends CoreItem>(): T { if (!this.item) throw new Error('InputError: Invalid item'); return this.item as T; }
  Name(): string { return this.item?.name ?? ''; }
  setName(n: string): this { this.req().setName(n); return this; }
  Type(): number { return this.item?.type ?? -1; }
  equals(o: RobolinkItem): boolean { return !!o && o.item === this.item; }
  Parent(): RobolinkItem { return new RobolinkItem(this.rdk, this.item?.parent ?? null); }
  Childs(): RobolinkItem[] { return (this.item?.children ?? []).filter((c) => !(c instanceof Instruction)).map((c) => new RobolinkItem(this.rdk, c)); }
  Delete(): void { const it = this.req(); this.rdk.exec.stop(it); it.delete(); this.item = null; }
  Visible(): boolean { return !!this.item?.visible; }
  setVisible(v: boolean | number, _visible_frame?: any): this { this.req().setVisible(!!v); return this; }
  setParent(p: RobolinkItem): this { this.req().setParent(p.req()); return this; }
  setParentStatic(p: RobolinkItem): this { this.req().setParentStatic(p.req()); return this; }
  /** Robot: TCP w.r.t. the active reference frame; other items: pose w.r.t. the parent. */
  Pose(): Mat {
    const it = this.req();
    if (it instanceof Robot) return new Mat(it.poseTCP());
    return new Mat(it.pose());
  }
  setPose(p: PoseLike): this {
    const it = this.req();
    const m = toMat4(p);
    if (it instanceof Robot) { const r = it.solveIKFrame(m); if (r.ok) it.setJoints(r.joints); else throw new Error('TargetReachError: target not reachable'); }
    else it.setPose(m);
    return this;
  }
  /** Absolute pose; for robots the robot base (RoboDK). */
  PoseAbs(): Mat { return new Mat(this.req().poseAbs()); }
  setPoseAbs(p: PoseLike): this { this.req().setPoseAbs(toMat4(p)); return this; }
  PoseWrt(other: RobolinkItem): Mat {
    const abs = (r: RobolinkItem) => (r.item instanceof Station || !r.item ? identity() : r.item.poseAbs());
    return new Mat(multiply(invert(abs(other)), abs(this)));
  }
  PoseTool(): Mat { const it = this.req(); return new Mat(it instanceof Robot ? it.poseTool() : it instanceof Tool ? it.poseTool() : it instanceof Program && it.robot() instanceof Robot ? (it.robot() as Robot).poseTool() : identity()); }
  Htool(): Mat { return this.PoseTool(); }
  /** Item: link the tool; Mat: set the TCP (creates the robot's tool if it has none). Programs get a "Set Tool" instruction. */
  setPoseTool(p: PoseLike | RobolinkItem): this {
    const it = this.req();
    if (it instanceof Program) {
      const tool = p instanceof RobolinkItem ? p.item : this.toolFor(it.robot() as Robot | null, toMat4(p as PoseLike));
      if (this.rdk.recording(it)) return this;
      it.setTool(tool);
      return this;
    }
    if (p instanceof RobolinkItem) { if (it instanceof Robot && p.item instanceof Tool) { it.setTool(p.item); this.rdk.record(it, { kind: 'tool', toolId: p.item.id }); } return this; }
    if (it instanceof Robot) { const t = this.toolFor(it, toMat4(p)); this.rdk.record(it, { kind: 'tool', toolId: t?.id ?? null }); }
    else if (it instanceof Tool) it.setPoseTool(toMat4(p));
    return this;
  }
  setHtool(p: PoseLike | RobolinkItem): this { return this.setPoseTool(p); }
  setTool(p: PoseLike | RobolinkItem): this { return this.setPoseTool(p); }
  Tool(): Mat { return this.PoseTool(); }
  private toolFor(robot: Robot | null, m: Mat4): Tool | null {
    if (!robot) return null;
    let t = robot.activeTool();
    if (!t) { t = new Tool('Tool'); robot.addChild(t); robot.setTool(t); }
    t.setPoseTool(m);
    return t;
  }
  PoseFrame(): Mat { const it = this.req(); return new Mat(it instanceof Robot ? it.poseFrame() : identity()); }
  Frame(): Mat { return this.PoseFrame(); }
  /** Item: use that reference frame; Mat: pose of the robot's user frame w.r.t. its base (one frame item reused). */
  setPoseFrame(p: PoseLike | RobolinkItem): this {
    const it = this.req();
    if (it instanceof Program) {
      const frame = p instanceof RobolinkItem ? p.item : this.frameFor(it.robot() as Robot | null, toMat4(p as PoseLike));
      it.setFrame(frame);
      return this;
    }
    if (!(it instanceof Robot)) return this;
    if (p instanceof RobolinkItem) { it.setFrame(p.item); this.rdk.record(it, { kind: 'frame', frameId: p.item?.id ?? null }); return this; }
    const f = this.frameFor(it, toMat4(p));
    this.rdk.record(it, { kind: 'frame', frameId: f?.id ?? null });
    return this;
  }
  setFrame(p: PoseLike | RobolinkItem): this { return this.setPoseFrame(p); }
  private frameFor(robot: Robot | null, m: Mat4): CoreItem | null {
    if (!robot) return null;
    const id = String(robot.getParam('apiFrameId', '') ?? '');
    let f = id ? this.rdk.station.findById(id) : null;
    if (!f) { f = new Frame(`${robot.name} Frame`); (robot.parent ?? this.rdk.station).addChild(f); robot.setParam('apiFrameId', f.id); }
    f.setPoseAbs(multiply(robot.poseAbs(), m));
    robot.setFrame(f);
    return f;
  }
  Joints(): number[] {
    const it = this.req();
    if (it instanceof Robot) return it.joints();
    if (it instanceof Target) return it.joints ?? [];
    if (it instanceof Program) { const r = it.robot(); return r instanceof Robot ? r.joints() : []; }
    return [];
  }
  SimulatorJoints(): number[] { return this.Joints(); }
  setJoints(q: number[]): this {
    const it = this.req();
    q = flat(q);
    if (it instanceof Robot) { if (q.length < it.dof) throw new Error(`InputError: ${it.name} needs ${it.dof} joint values`); it.setJoints(q.slice(0, it.dof)); }
    else if (it instanceof Target) it.setJoints(q);
    return this;
  }
  JointsHome(): number[] { return this.req<Robot>().jointsHome(); }
  /** [lower, upper, joints type] (RoboDK JointLimits). */
  JointLimits(): [number[], number[], number] { const l = this.req<Robot>().jointLimits(); return [l.lower, l.upper, 1]; }
  setJointLimits(lo: number[], hi: number[]): this { this.req<Robot>().setJointLimits(flat(lo), flat(hi)); return this; }
  /** Flange pose w.r.t. the robot base for joints q (RoboDK: independent of the active tool); optional tool/reference. */
  SolveFK(q: number[], tool?: PoseLike | null, reference?: PoseLike | null): Mat {
    const r = this.req<Robot>();
    let m = r.solveFK(flat(q), identity());
    if (tool) m = multiply(m, toMat4(tool));
    if (reference) m = multiply(invert(toMat4(reference)), m);
    return new Mat(m);
  }
  /** Joints for a flange pose w.r.t. the robot base (optional tool/reference like RoboDK). [] when unreachable. */
  SolveIK(p: PoseLike, jointsApprox?: number[] | null, tool?: PoseLike | null, reference?: PoseLike | null): number[] {
    const r = this.req<Robot>();
    let m = toMat4(p);
    if (tool) m = multiply(m, invert(toMat4(tool)));
    if (reference) m = multiply(toMat4(reference), m);
    const seed = jointsApprox && flat(jointsApprox).length >= r.dof ? flat(jointsApprox).slice(0, r.dof) : undefined;
    const res = r.solveIK(m, { seed }, identity());
    return res.ok && r.jointsValid(res.joints) ? res.joints : [];
  }
  SolveIK_All(p: PoseLike, tool?: PoseLike | null, reference?: PoseLike | null): number[][] {
    const r = this.req<Robot>();
    let m = toMat4(p);
    if (tool) m = multiply(m, invert(toMat4(tool)));
    if (reference) m = multiply(toMat4(reference), m);
    return r.solveIKAll(m, identity()).filter((q) => r.jointsValid(q));
  }
  Connect(_ip?: string): number { return ROBOTCOM_READY; }
  ConnectedState(): number { return ROBOTCOM_READY; }
  setSpeed(speedLinear: number, speedJoints = -1, accelLinear = -1, accelJoints = -1): this {
    const it = this.req();
    if (it instanceof Robot) {
      if (speedLinear > 0) it.motion.speedLinear = speedLinear; if (speedJoints > 0) it.motion.speedJoints = speedJoints; if (accelLinear > 0) it.motion.accelLinear = accelLinear; if (accelJoints > 0) it.motion.accelJoints = accelJoints;
      this.rdk.record(it, { kind: 'speed', speedLinear: speedLinear > 0 ? speedLinear : undefined, speedJoints: speedJoints > 0 ? speedJoints : undefined, accelLinear: accelLinear > 0 ? accelLinear : undefined, accelJoints: accelJoints > 0 ? accelJoints : undefined });
    } else if (it instanceof Program) it.setSpeed(speedLinear > 0 ? speedLinear : undefined, speedJoints > 0 ? speedJoints : undefined, accelLinear > 0 ? accelLinear : undefined, accelJoints > 0 ? accelJoints : undefined);
    return this;
  }
  setRounding(r: number): this { const it = this.req(); if (it instanceof Robot) { it.motion.rounding = r; this.rdk.record(it, { kind: 'rounding', radius: r }); } else if (it instanceof Program) it.setRounding(r); return this; }
  setZoneData(r: number): this { return this.setRounding(r); }

  // ---- motion ----
  private resolveTarget(t: any): Target | number[] | Mat4 {
    if (t instanceof RobolinkItem) { if (!(t.item instanceof Target)) throw new Error('InputError: invalid target item'); return t.item; }
    if (isPose(t)) return toMat4(t);
    return flat(t);
  }
  private robotMove(kind: MoveInstruction['moveType'], target: any, via: any, blocking: boolean): void | Promise<void> {
    const r = this.req<Robot>();
    const t = this.resolveTarget(target);
    let v = via !== undefined ? this.resolveTarget(via) : undefined;
    const asPose = (x: Target | number[] | Mat4 | undefined) => (Array.isArray(x) && kind === 'MoveC' ? r.poseTCP(x) : x);
    const tt = asPose(t)!, vv = asPose(v);
    if (Array.isArray(t)) {
      if (t.length < r.dof) throw new Error(`InputError: ${r.name} needs ${r.dof} joint values`);
      if (!r.jointsValid(t.slice(0, r.dof))) throw new Error(`TargetReachError: joints [${t.map((x) => x.toFixed(2)).join(', ')}] are outside the joint limits of ${r.name}`);
    }
    const mode = this.rdk.runMode;
    if (mode === RUNMODE_MAKE_ROBOTPROG || mode === RUNMODE_MAKE_ROBOTPROG_AND_START || mode === RUNMODE_MAKE_ROBOTPROG_AND_UPLOAD || mode === RUNMODE_QUICKVALIDATE) {
      const q = Array.isArray(tt) ? tt.slice(0, r.dof) : r.jointsForTarget(tt as any);
      if (!q) throw new Error(`TargetReachError: target not reachable by ${r.name}`);
      const rec = this.rdk.recorderFor(r);
      if (rec && mode !== RUNMODE_QUICKVALIDATE) rec.program.addInstruction(moveTo(kind, tt instanceof Target ? (tt.isJointTarget && tt.joints ? [...tt.joints] : r.poseTCP(q)) : tt, vv instanceof Target ? r.poseTCP(r.jointsForTarget(vv) ?? q) : vv));
      r.setJoints(q);
      return;
    }
    return this.rdk.exec.moveRobot(r, (p) => p.addInstruction(moveTo(kind, tt, vv)), blocking);
  }
  /** Robot: timed joint move (blocking returns when done); program: adds a MoveJ instruction. */
  MoveJ(target: RobolinkItem | number[] | Mat, blocking: boolean | number = true): any {
    const it = this.req();
    if (it instanceof Program) { if (target instanceof RobolinkItem && target.item instanceof Target) it.addMoveJ(target.item); else it.addInstruction(moveTo('MoveJ', this.resolveTarget(target))); return this; }
    return this.robotMove('MoveJ', target, undefined, !!blocking);
  }
  MoveL(target: RobolinkItem | number[] | Mat, blocking: boolean | number = true): any {
    const it = this.req();
    if (it instanceof Program) { if (target instanceof RobolinkItem && target.item instanceof Target) it.addMoveL(target.item); else it.addInstruction(moveTo('MoveL', this.resolveTarget(target))); return this; }
    return this.robotMove('MoveL', target, undefined, !!blocking);
  }
  MoveC(via: RobolinkItem | number[] | Mat, target: RobolinkItem | number[] | Mat, blocking: boolean | number = true): any {
    const it = this.req();
    if (it instanceof Program) {
      if (via instanceof RobolinkItem && target instanceof RobolinkItem && via.item instanceof Target && target.item instanceof Target) { it.addMoveC(via.item, target.item); return this; }
      const r = it.robot() instanceof Robot ? (it.robot() as Robot) : null;
      const conv = (x: any) => { const v = this.resolveTarget(x); return Array.isArray(v) ? (r ? r.poseTCP(v) : (() => { throw new Error('InputError: MoveC with joints needs a program robot'); })()) : v; };
      it.addInstruction(moveTo('MoveC', conv(target), conv(via)));
      return this;
    }
    return this.robotMove('MoveC', target, via, !!blocking);
  }
  addMoveJ(target: RobolinkItem | number[] | Mat): this { return this.MoveJ(target); }
  addMoveL(target: RobolinkItem | number[] | Mat): this { return this.MoveL(target); }
  addMoveC(t1: RobolinkItem, t2: RobolinkItem): this { return this.MoveC(t1, t2); }
  Pause(ms = -1): any {
    const it = this.req();
    if (it instanceof Program) { it.pause(ms); return this; }
    if (it instanceof Robot && this.rdk.record(it, { kind: 'pause', timeMs: ms })) return this;
    if (ms > 0) return this.rdk.Pause(ms / 1000);
    return this;
  }
  setDO(io: string, value: string | number | boolean): this {
    const it = this.req();
    const v = ioValue(value);
    if (it instanceof Program) it.setDO(String(io), v);
    else if (it instanceof Robot) { if (!this.rdk.record(it, { kind: 'io', io: String(io), value: v, wait: false })) { it.state.io[io] = v; this.rdk.exec.rt().signals?.set(String(io), v); } }
    else this.rdk.setSignal(String(io), v);
    return this;
  }
  setAO(io: string, value: number | string): this { const it = this.req(); const v = Number(value); if (it instanceof Program) it.addInstruction({ kind: 'io', io: String(io), value: v, wait: false }); else if (it instanceof Robot) { if (!this.rdk.record(it, { kind: 'io', io: String(io), value: v, wait: false })) { it.state.io[io] = v; this.rdk.exec.rt().signals?.set(String(io), v); } } return this; }
  getDI(io: string): number | boolean | string { const r = this.req(); const v = r instanceof Robot ? r.state.io[io] ?? this.rdk.exec.rt().signals?.get(String(io)) : this.rdk.getSignal(String(io)); return v === undefined ? 0 : typeof v === 'boolean' ? (v ? 1 : 0) : v; }
  getAI(io: string): number { const v = this.getDI(io); return Number(v) || 0; }
  /** Program: adds a wait instruction; robot: waits (simulated time) until the input has the value. */
  waitDI(io: string, value: string | number | boolean, timeout = -1): any {
    const it = this.req();
    const v = ioValue(value);
    if (it instanceof Program) { it.waitDI(String(io), v, timeout); return this; }
    if (it instanceof Robot && this.rdk.record(it, { kind: 'io', io: String(io), value: v, wait: true, timeoutMs: timeout })) return this;
    return this.rdk.waitUntil(() => ioEq(this.getDI(io), v), timeout > 0 ? timeout / 1000 : 3600 * 24);
  }
  RunInstruction(code: string, type = INSTRUCTION_CALL_PROGRAM): any {
    const it = this.req();
    const call = parseCall(code);
    const prog = this.rdk.station.find(call.name, ItemType.PROGRAM);
    const py = this.rdk.station.find(call.name, ItemType.PROGRAM_PYTHON);
    if (it instanceof Program) {
      if (type === INSTRUCTION_COMMENT) it.comment(code);
      else if (type === INSTRUCTION_SHOW_MESSAGE) it.showMessage(code);
      else if (type === INSTRUCTION_INSERT_CODE) it.runInstruction(code, false);
      else if (type === INSTRUCTION_START_THREAD && prog instanceof Program) it.startThread(prog);
      else if (type === INSTRUCTION_CALL_PROGRAM && prog instanceof Program && !call.args.length) it.callProgram(prog);
      else if (/^(attach|detach|gripper_open|gripper_close)$/i.test(code)) it.event(code.toLowerCase() as any);
      else it.runInstruction(code, true);
      return this;
    }
    if (it instanceof Robot && this.rdk.recorderFor(it) && (this.rdk.runMode === RUNMODE_MAKE_ROBOTPROG || this.rdk.runMode === RUNMODE_MAKE_ROBOTPROG_AND_START || this.rdk.runMode === RUNMODE_MAKE_ROBOTPROG_AND_UPLOAD)) {
      const rec = this.rdk.recorderFor(it)!;
      if (type === INSTRUCTION_COMMENT) rec.program.comment(code); else if (type === INSTRUCTION_SHOW_MESSAGE) rec.program.showMessage(code); else rec.program.runInstruction(code, type !== INSTRUCTION_INSERT_CODE);
      return 0;
    }
    if (type === INSTRUCTION_COMMENT) return 0;
    if (type === INSTRUCTION_SHOW_MESSAGE || type === INSTRUCTION_INSERT_CODE) { this.rdk.onMessage?.(code, false); return 0; }
    if (py) { this.rdk.startScript(py, call.args, it instanceof Robot ? it : null); return 0; }
    if (prog instanceof Program) { const h = this.rdk.exec.start(prog); return type === INSTRUCTION_CALL_PROGRAM ? this.rdk.exec.wait(prog, 3600 * 24, [h]) ?? 0 : 0; }
    this.rdk.onMessage?.(code, false);
    return 0;
  }
  RunCodeCustom(code: string, type = INSTRUCTION_CALL_PROGRAM): any { return this.RunInstruction(code, type); }
  ShowInstructions(_v = true): this { return this; }
  ShowTargets(_v = true): this { return this; }
  InstructionCount(): number { return this.req<Program>().instructions().length; }
  private ins(id: number): Instruction {
    const list = this.req<Program>().instructions();
    const ins = list[id < 0 ? list.length + id : id];
    if (!ins) throw new Error(`InputError: instruction ${id} does not exist (program ${this.Name()} has ${list.length})`);
    return ins;
  }
  /** RoboDK Instruction(): name, INS_TYPE, MOVE_TYPE, joint target flag, pose (in the active frame), joints. -1 = last. */
  Instruction(id = -1): { name: string; type: number; moveType: number | null; isJointTarget: number | null; pose: Mat | null; joints: number[] | null } {
    const ins = this.ins(id);
    const d = ins.data;
    const type = rdkInsType(ins);
    if (d.kind === 'move') {
      const t = d.targetId ? (this.rdk.station.findById(d.targetId) as Target | null) : null;
      const prog = this.req<Program>();
      const robot = prog.robot() instanceof Robot ? (prog.robot() as Robot) : null;
      let pose: Mat4 | null = t ? t.pose() : d.pose ? Float64Array.from(d.pose) : null;
      const joints = t?.joints ?? d.joints ?? (robot && pose ? robot.jointsForTarget(pose) : null) ?? null;
      if (!pose && robot && joints) pose = robot.poseTCP(joints);
      return { name: ins.name, type: INS_TYPE_MOVE, moveType: moveTypeOf(d), isJointTarget: t ? (t.isJointTarget ? 1 : 0) : d.joints ? 1 : 0, pose: pose ? new Mat(pose) : null, joints };
    }
    return { name: ins.name, type, moveType: null, isJointTarget: null, pose: null, joints: null };
  }
  /** RoboKiSim sequence: one row per move instruction [j1..jn, MOVE_TYPE] (Python returns the columns) and the error count. */
  InstructionList(): [number[][], number] {
    const p = this.req<Program>();
    const robot = p.robot() instanceof Robot ? (p.robot() as Robot) : null;
    const rows: number[][] = [];
    let errors = 0;
    p.instructions().forEach((ins, i) => {
      if (ins.data.kind !== 'move') return;
      const d = this.Instruction(i);
      const q = d.joints ?? (robot && d.pose ? robot.jointsForTarget(d.pose.m) : null);
      if (!q) { errors++; return; }
      rows.push([...q, d.moveType ?? MOVE_TYPE_JOINT]);
    });
    return [rows, errors];
  }
  InstructionDelete(id = 0): this { const ins = this.req<Program>().instructions()[id < 0 ? this.InstructionCount() + id : id]; ins?.delete(); return this; }
  setRobot(r: RobolinkItem | null): this { this.req<Program>().setRobot(r?.item ?? null); return this; }
  getLink(type = ITEM_TYPE_ROBOT): RobolinkItem {
    const it = this.req();
    const st = this.rdk.station;
    if (it instanceof Program) { const r = it.robot(); return new RobolinkItem(this.rdk, type === ITEM_TYPE_ROBOT ? r : type === ITEM_TYPE_FRAME ? (it.frameId ? st.findById(it.frameId) : (r instanceof Robot ? r.activeFrame() : null)) : type === ITEM_TYPE_TOOL ? (it.toolId ? st.findById(it.toolId) : (r instanceof Robot ? r.activeTool() : null)) : null); }
    if (it instanceof Robot) return new RobolinkItem(this.rdk, type === ITEM_TYPE_TOOL ? it.activeTool() : type === ITEM_TYPE_FRAME ? it.activeFrame() : type === ITEM_TYPE_ROBOT ? it : null);
    if (it instanceof Tool) return new RobolinkItem(this.rdk, type === ITEM_TYPE_ROBOT || type === ITEM_TYPE_ANY ? it.parent : null);
    if (it instanceof Target) { const r = it.robotId ? st.findById(it.robotId) : null; return new RobolinkItem(this.rdk, type === ITEM_TYPE_ROBOT ? r ?? st.itemsOfType(ItemType.ROBOT)[0] ?? null : type === ITEM_TYPE_FRAME ? it.parent : null); }
    if (it.type === ItemType.MACHINING) {
      const id = type === ITEM_TYPE_PROGRAM ? it.getParam('programId') : type === ITEM_TYPE_ROBOT ? it.getParam('robotId') : type === ITEM_TYPE_OBJECT ? it.getParam('partId') : type === ITEM_TYPE_FRAME ? it.getParam('frameId') : type === ITEM_TYPE_TOOL ? it.getParam('toolId') : null;
      let found = id ? st.findById(String(id)) : null;
      if (!found && type === ITEM_TYPE_TOOL) { const r = st.findById(String(it.getParam('robotId') ?? '')); found = r instanceof Robot ? r.activeTool() : null; }
      return new RobolinkItem(this.rdk, found);
    }
    return new RobolinkItem(this.rdk, null);
  }
  getLinks(type = ITEM_TYPE_ROBOT): RobolinkItem[] { return this.rdk.ItemList(type).filter((c: RobolinkItem) => c.item !== this.item && c.getLink(this.Type()).item === this.item); }
  AddTool(pose: PoseLike, name = 'New TCP'): RobolinkItem {
    const r = this.req<Robot>();
    const t = new Tool(name);
    t.setPoseTool(toMat4(pose));
    r.addChild(t);
    r.setTool(t);
    return new RobolinkItem(this.rdk, t);
  }
  AddFrame(name: string): RobolinkItem { return this.rdk.AddFrame(name, this); }
  /** Robot: target in the robot's active frame linked to the robot (children of a robot follow its flange); else a child target. */
  AddTarget(name: string): RobolinkItem { return this.item instanceof Robot ? this.rdk.AddTarget(name, undefined, this) : this.rdk.AddTarget(name, this); }
  AddFile(filename: string): any { return this.rdk.AddFile(filename, this); }
  AddShape(points: number[][]): RobolinkItem { return this.rdk.AddShape(points, this); }
  AddCurve(points: number[][], add_to_ref = false, projection_type = PROJECTION_ALONG_NORMAL_RECALC): RobolinkItem { return this.rdk.AddCurve(points, this, add_to_ref, projection_type); }
  AddPoints(points: number[][], add_to_ref = false, projection_type = PROJECTION_ALONG_NORMAL_RECALC): RobolinkItem { return this.rdk.AddPoints(points, this, add_to_ref, projection_type); }
  ProjectPoints(points: number[][], projection_type = PROJECTION_ALONG_NORMAL_RECALC): number[][] { return this.rdk.ProjectPoints(points, this, projection_type); }
  setAsCartesianTarget(): this { this.req<Target>().setAsCartesianTarget(); return this; }
  setAsJointTarget(): this { this.req<Target>().setAsJointTarget(); return this; }
  isJointTarget(): boolean { return !!this.req<Target>().isJointTarget; }
  /** Validate the program: [valid_instructions, program_time, program_distance, valid_ratio, readable_msg]. */
  Update(check_collisions = COLLISION_OFF, _timeout = 3600, _mm_step = -1, _deg_step = -1): [number, number, number, number, string] | any {
    const it = this.req();
    if (it.type === ItemType.MACHINING) { const res = this.rdk.updateMachining(it); const p = this.getLink(ITEM_TYPE_PROGRAM); return p.item instanceof Program ? p.Update(check_collisions) : [0, 0, 0, 0, res ? 'OK' : 'Machining project has no part/curves']; }
    const p = this.req<Program>();
    const sim = new ProgramSimulator(this.rdk.station);
    if (check_collisions) sim.collisionOptions = { ...sim.collisionOptions, enabled: true, assets: this.rdk.assets } as any;
    const r = sim.compile(p);
    const errs = r.problems.filter((x) => x.severity === 'error');
    const total = p.instructions().filter((i) => i.enabled).length;
    const failed = new Set(errs.map((e) => e.instructionId));
    const ins = p.instructions();
    const firstBad = ins.findIndex((i) => failed.has(i.id));
    const valid = firstBad < 0 ? total : firstBad;
    return [valid, r.duration, r.distance, total ? valid / total : 1, errs.map((e) => e.message).join('; ')];
  }
  RunProgram(params?: any): number { return this.RunCode(params); }
  RunCode(params?: any): number {
    const it = this.req();
    const args = Array.isArray(params) ? params.map(String) : params == null || params === '' ? [] : [String(params)];
    if (it.type === ItemType.PROGRAM_PYTHON) { this.rdk.startScript(it, args, null); return 0; }
    if (it.type === ItemType.MACHINING) { const p = this.getLink(ITEM_TYPE_PROGRAM); return p.item ? p.RunCode(params) : -1; }
    const p = this.req<Program>();
    if (this.rdk.onRunProgram && !this.rdk.exec.attachedRuntime && this.rdk.exec.mode !== 'realtime') { this.rdk.onRunProgram(p); return 0; }
    this.rdk.exec.start(p, { args });
    return 0;
  }
  /**
   * Generate the robot program with the robot's post processor (or `post`) and write it to `folder` (RoboDK
   * MakeProgram). Returns [success, log message, transfer_ok]. A string 2nd argument is taken as the post (studio JS).
   */
  MakeProgram(folder = '', runMode: number | string = RUNMODE_MAKE_ROBOTPROG, post?: string | null): [boolean, string, boolean] | Promise<[boolean, string, boolean]> {
    if (typeof runMode === 'string') { post = runMode; runMode = RUNMODE_MAKE_ROBOTPROG; }
    const it = this.req();
    if (it.type === ItemType.MACHINING) { const p = this.getLink(ITEM_TYPE_PROGRAM); if (!p.item) throw new Error('InputError: machining project has no program (call Update first)'); return p.MakeProgram(folder, runMode, post); }
    return this.rdk.generateProgram(this.req<Program>(), folder, post ?? null, runMode);
  }
  /** Item.setParam: RoboDK item command channel (ICMD subset); other keys are stored as item parameters. */
  setParam(key: string | number, value: any = '', _skip = false): any {
    const it = this.req();
    if (typeof key === 'number' || (/^-?\d+$/.test(String(key)) && it instanceof Program)) return this.instructionParam(Number(key), value);
    const k = String(key);
    if (it instanceof Program && k === 'Add' && value) { const d = typeof value === 'string' ? JSON.parse(value) : value; return this.addFromDict(d); }
    if (k === 'PostProcessor' && it instanceof Robot) { const p = getPost(String(value)); if (!p) throw new Error(`InputError: unknown post processor ${value}`); it.postProcessor = p.id; it.setParam('PostProcessor', String(value)); return 'OK'; }
    if (/^(Tree|Display|Recalculate|UpdatePath|Clear|DeleteCurves|Open|Settings)$/.test(k)) {
      if (k === 'Clear' || k === 'DeleteCurves') { if (it instanceof SceneObject) { if (k === 'DeleteCurves' || /Curve/i.test(String(value))) it.curves = []; if (/Point/i.test(String(value))) it.points = []; it.notify('geometry'); } }
      if (k === 'UpdatePath' && it.type === ItemType.MACHINING) this.rdk.updateMachining(it);
      if (k === 'Open') { it.setParam('open', value === '' ? it.getParam('open', 1) : value); return String(it.getParam('open', 1)); }
      return 'OK';
    }
    if (value === '' || value === undefined) { const v = it.getParam(k); return v === undefined ? '' : typeof v === 'object' ? JSON.stringify(v) : String(v); }
    it.setParam(k, value);
    return 'OK';
  }
  getParam(key: string): any { const v = this.req().getParam(key); return v === undefined ? null : v; }
  private instructionParam(id: number, value: any): any {
    const ins = this.ins(id);
    if (value !== '' && value != null) {
      const upd = typeof value === 'string' ? JSON.parse(value) : value;
      if (upd.Type === -1) { ins.delete(); return 'OK'; }
      const d: any = ins.data;
      if (upd.Name) ins.name = String(upd.Name);
      if (d.kind === 'speed') { if (upd.Speed != null) d.speedLinear = Number(upd.Speed); if (upd.SpeedJoints != null) d.speedJoints = Number(upd.SpeedJoints); if (upd.Accel != null) d.accelLinear = Number(upd.Accel); if (upd.AccelJoints != null) d.accelJoints = Number(upd.AccelJoints); if (!upd.Name) ins.name = `Set speed (${d.speedLinear ?? '-'} mm/s)`; }
      if (d.kind === 'code' && upd.Code != null) d.code = String(upd.Code);
      if (d.kind === 'pause' && upd.Time != null) d.timeMs = Number(upd.Time);
      if (d.kind === 'rounding' && upd.Rounding != null) d.radius = Number(upd.Rounding);
      if (d.kind === 'print' && upd.Message != null) d.message = String(upd.Message);
      if (d.kind === 'io') { if (upd.IO_Name != null) d.io = String(upd.IO_Name); if (upd.IO_Value != null) d.value = ioValue(upd.IO_Value); }
      if ((d.kind === 'frame' || d.kind === 'tool') && upd.Item != null) { const ref = this.rdk.station.findById(String(upd.Item)) ?? this.rdk.station.find(String(upd.Item)); if (ref) { if (d.kind === 'frame') d.frameId = ref.id; else d.toolId = ref.id; } }
      if (d.kind === 'move') { if (upd.MoveType != null) d.moveType = Number(upd.MoveType) === MOVE_TYPE_LINEAR ? 'MoveL' : Number(upd.MoveType) === MOVE_TYPE_CIRCULAR ? 'MoveC' : 'MoveJ'; const tgt = upd.Target != null ? this.rdk.station.findById(String(upd.Target)) ?? this.rdk.station.find(String(upd.Target), ItemType.TARGET) : null; if (tgt) d.targetId = tgt.id; }
      ins.notify('data');
      return 'OK';
    }
    return JSON.stringify(this.instructionDict(ins));
  }
  private instructionDict(ins: Instruction): Record<string, any> {
    const d: any = ins.data;
    const o: Record<string, any> = { Type: rdkInsType(ins), Name: ins.name };
    const st = this.rdk.station;
    switch (d.kind) {
      case 'move': { o.MoveType = moveTypeOf(d); const t = d.targetId ? st.findById(d.targetId) : null; if (t) { o.Target = t.id; o.TargetName = t.name; } if (d.pose) o.Pose = toRows(Float64Array.from(d.pose)); if (d.joints) o.Joints = d.joints; if (d.speed != null) o.Speed = d.speed; break; }
      case 'speed': o.Speed = d.speedLinear ?? -1; o.SpeedJoints = d.speedJoints ?? -1; o.Accel = d.accelLinear ?? -1; o.AccelJoints = d.accelJoints ?? -1; break;
      case 'frame': case 'tool': { const ref = st.findById(d.frameId ?? d.toolId ?? ''); o.Item = ref?.id ?? ''; o.ItemName = ref?.name ?? ''; break; }
      case 'pause': o.Time = d.timeMs; break;
      case 'code': o.Code = d.code; o.Behavior = d.asFunctionCall ? 0 : 1; break;
      case 'call': case 'thread': { const p = d.programId ? st.findById(d.programId) : null; o.Code = p?.name ?? d.programName ?? ''; o.Behavior = d.kind === 'thread' ? 2 : 0; break; }
      case 'print': o.Message = d.message; o.Behavior = d.isComment ? 3 : 4; break;
      case 'rounding': o.Rounding = d.radius; break;
      case 'io': o.IO_Name = d.io; o.IO_Value = typeof d.value === 'boolean' ? (d.value ? 1 : 0) : d.value; o.IO_Wait = d.wait ? 1 : 0; if (d.timeoutMs != null) o.Timeout = d.timeoutMs; break;
      case 'event': o.Code = d.action; break;
      case 'signal': o.IO_Name = d.signal; o.IO_Value = d.value; o.IO_Wait = d.wait ? 1 : 0; break;
    }
    return o;
  }
  private addFromDict(o: any): string {
    const p = this.req<Program>();
    const st = this.rdk.station;
    switch (Number(o.Type)) {
      case INS_TYPE_MOVE: case INS_TYPE_MOVEC: {
        const t = o.Target ? st.findById(String(o.Target)) : null;
        const mt = Number(o.MoveType) === MOVE_TYPE_LINEAR ? 'MoveL' : Number(o.MoveType) === MOVE_TYPE_CIRCULAR ? 'MoveC' : 'MoveJ';
        if (t instanceof Target) p.addInstruction({ kind: 'move', moveType: mt, targetId: t.id }); else if (o.Joints) p.addInstruction(moveTo(mt, flat(o.Joints))); else if (o.Pose) p.addInstruction(moveTo(mt, fromRows(o.Pose)));
        break;
      }
      case INS_TYPE_CHANGESPEED: p.setSpeed(o.Speed > 0 ? o.Speed : undefined, o.SpeedJoints > 0 ? o.SpeedJoints : undefined, o.Accel > 0 ? o.Accel : undefined, o.AccelJoints > 0 ? o.AccelJoints : undefined); break;
      case INS_TYPE_CHANGEFRAME: p.setFrame(st.findById(String(o.Item ?? '')) ?? null); break;
      case INS_TYPE_CHANGETOOL: p.setTool(st.findById(String(o.Item ?? '')) ?? null); break;
      case INS_TYPE_PAUSE: p.pause(Number(o.Time ?? -1)); break;
      case INS_TYPE_CODE: this.RunInstruction(String(o.Code ?? ''), o.Behavior === 1 ? INSTRUCTION_INSERT_CODE : o.Behavior === 2 ? INSTRUCTION_START_THREAD : INSTRUCTION_CALL_PROGRAM); break;
      case INS_TYPE_PRINT: if (o.Behavior === 3) p.comment(String(o.Message ?? '')); else p.showMessage(String(o.Message ?? '')); break;
      case INS_TYPE_ROUNDING: p.setRounding(Number(o.Rounding ?? -1)); break;
      case INS_TYPE_IO: if (o.IO_Wait) p.waitDI(String(o.IO_Name), ioValue(o.IO_Value), Number(o.Timeout ?? -1)); else p.setDO(String(o.IO_Name), ioValue(o.IO_Value)); break;
      default: throw new Error(`InputError: unsupported instruction type ${o.Type}`);
    }
    const last = p.instructions()[p.instructions().length - 1];
    if (last && o.Name) last.name = String(o.Name);
    return 'OK';
  }
  setColor(color: string | number[]): this { const it = this.req(); it.color = Array.isArray(color) ? toCss(color) : color; it.notify('color'); return this; }
  Recolor(color: string | number[], _from?: any, _tol?: any): this { return this.setColor(color); }
  setGeometryPose(p: PoseLike, _apply = false): this { const it = this.req(); if (it instanceof SceneObject) { for (const g of it.geometry) g.origin = Array.from(toMat4(p)); it.notify('geometry'); } return this; }
  /** Robot: moving (API move or program of this robot); program: running. */
  Busy(): boolean {
    const it = this.req();
    if (this.rdk.scriptRuns.has(it.id)) return true;
    if (this.rdk.exec.busy(it)) return true;
    if (it instanceof Robot) return !!it.state.moving && !!this.rdk.onRunProgram && !this.rdk.exec.attachedRuntime;
    return it instanceof Program ? !!this.rdk.isPlaying?.(it) : false;
  }
  Stop(): void { const it = this.req(); this.rdk.exec.stop(it instanceof Robot || it instanceof Program ? it : null); this.rdk.onStop?.(); }
  /** Wait until the robot finished its motion (RoboDK WaitMove). */
  WaitMove(timeout = 360000): any { const it = this.req(); if (!(it instanceof Robot)) return; return this.rdk.exec.wait(it, timeout / 1000); }
  /** Wait until the program (or the robot's program) finished. */
  WaitFinished(timeout = 3600 * 24 * 7): any {
    const it = this.req();
    const s = this.rdk.scriptRuns.get(it.id);
    if (s) return s.then(() => undefined);
    return this.rdk.exec.wait(it, timeout);
  }
  ProgramStart(name: string, folder = '', post = ''): number { return this.rdk.ProgramStart(name, folder, post, this); }
  DOF(): number { return this.req<Robot>().dof; }

  // ---- attach / detach (RoboDK AttachClosest / DetachClosest / DetachAll) ----
  /** Attach the closest object (within tolerance mm of the TCP, optional name keyword / candidate list) to this tool. */
  AttachClosest(keyword: string | number = '', tolerance_mm = -1, list_objects: RobolinkItem[] = []): RobolinkItem {
    if (typeof keyword === 'number') { tolerance_mm = keyword; keyword = ''; }
    const maxDistance = tolerance_mm > 0 ? tolerance_mm : 500;
    const tool = this.req();
    const owner = tool instanceof Robot ? tool.activeTool() ?? tool : tool;
    const tcp = owner instanceof Tool ? multiply(owner.poseAbs(), owner.poseTool()) : tool instanceof Robot ? tool.poseTCPAbs() : tool.poseAbs();
    const candidates = list_objects?.length ? list_objects.map((o) => o.item).filter((x): x is CoreItem => !!x) : [...this.rdk.station.walk()];
    let best: CoreItem | null = null, bd = maxDistance;
    for (const it of candidates) {
      if (!(it instanceof SceneObject) || it instanceof Tool || it.isAncestorOf(owner) || owner.isAncestorOf(it) || !it.visible) continue;
      if (keyword && !it.name.toLowerCase().includes(String(keyword).toLowerCase())) continue;
      const d = this.rdk.distanceToObject(it, tcp);
      if (d < bd) { bd = d; best = it; }
    }
    if (!best) return new RobolinkItem(this.rdk, null);
    best.setParentStatic(owner);
    if (owner instanceof Tool) { owner.closed = true; if (!owner.attached.includes(best.id)) owner.attached.push(best.id); }
    this.rdk.events.push({ type: 'attach', itemId: best.id });
    return new RobolinkItem(this.rdk, best);
  }
  /** Detach the closest attached object and place it under `parent` (station by default). */
  DetachClosest(parent?: RobolinkItem | number): RobolinkItem {
    const owner = this.owner();
    const kids = owner.children.filter((c) => c instanceof SceneObject && !(c instanceof Tool)) as SceneObject[];
    if (!kids.length) return new RobolinkItem(this.rdk, null);
    const tcp = owner.poseAbs();
    kids.sort((a, b) => dist3(a.poseAbs(), tcp) - dist3(b.poseAbs(), tcp));
    const it = kids[0];
    it.setParentStatic((parent instanceof RobolinkItem ? parent.item : null) ?? this.rdk.station);
    if (owner instanceof Tool) owner.attached = owner.attached.filter((x) => x !== it.id);
    return new RobolinkItem(this.rdk, it);
  }
  DetachAll(parent?: RobolinkItem | number): this {
    const owner = this.owner();
    for (const c of [...owner.children]) if (c instanceof SceneObject && !(c instanceof Tool)) c.setParentStatic((parent instanceof RobolinkItem ? parent.item : null) ?? this.rdk.station);
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
  IsInside(object: RobolinkItem): number { return this.rdk.IsInside(this, object); }
  Copy(_copy_children = true): this { clipboard.copy(this.req()); return this; }
  Paste(): RobolinkItem { return new RobolinkItem(this.rdk, clipboard.paste(this.req(), this.rdk.station)); }
  GeometryPose(): Mat {
    const it = this.req();
    const g = (it as SceneObject).geometry?.[0];
    return new Mat(g?.origin ? Float64Array.from(g.origin) : identity());
  }
  /**
   * Joints along the program path like RoboDK: rows [J1..Jn, ERROR, MM_STEP, DEG_STEP, MOVE_ID, TIME, X, Y, Z]
   * (flags >= 1 add TIME and the TCP XYZ in the active frame; Python returns the transposed Mat, one column per row).
   * Linear moves are split every mm_step, joint moves every deg_step. Status = executable instruction count, or
   * negative when the program has issues. save_to_file writes the CSV through the file system hook.
   */
  InstructionListJoints(mm_step = 10, deg_step = 5, save_to_file = '', collision_check = COLLISION_OFF, flags = 0, time_step = 0.1): [string, number[][], number] {
    const p = this.req<Program>();
    const robot = p.robot() instanceof Robot ? (p.robot() as Robot) : null;
    const sim = new ProgramSimulator(this.rdk.station);
    if (collision_check) sim.collisionOptions = { ...sim.collisionOptions, enabled: true, assets: this.rdk.assets } as any;
    const r = sim.compile(p);
    const errs = r.problems.filter((x) => x.severity === 'error');
    const rows: number[][] = [];
    if (robot) {
      const st = this.rdk.station;
      let frameId = p.frameId ?? robot.activeFrameId, toolId = p.toolId ?? robot.activeToolId;
      let moveId = 0, prevQ: number[] | null = null, prevT = 0;
      const xyz = (q: number[]) => {
        const f = frameId ? st.findById(frameId) : robot.activeFrame();
        const tl = toolId ? st.findById(toolId) : robot.activeTool();
        const frameInBase = f ? multiply(invert(robot.poseAbs()), f.poseAbs()) : identity();
        const m = multiply(invert(frameInBase), robot.solveFK(q, tl instanceof Tool ? tl.poseTool() : identity()));
        return [m[12], m[13], m[14]];
      };
      const push = (q: number[], err: number, mm: number, deg: number, t: number) => {
        const row = [...q, err, mm, deg, moveId];
        if (flags >= 1) row.push(Math.max(0, t - prevT), ...xyz(q));
        rows.push(row); prevQ = q; prevT = t;
      };
      for (const s of sim.steps) {
        const d: any = s.instruction.data;
        if (d.kind === 'frame') frameId = d.frameId ?? frameId;
        if (d.kind === 'tool') toolId = d.toolId ?? toolId;
        if (s.robot !== robot || !s.trajectory?.samples.length || (d.kind !== 'move' && d.kind !== 'jointPath')) continue;
        moveId++;
        const err = errs.some((e) => e.instructionId === s.instruction.id) ? 1 : 0;
        const tr = s.trajectory;
        if (!prevQ) push(sampleAt(tr, 0).joints, 0, 0, 0, s.t0);
        if (flags >= 4) {
          const n = Math.max(1, Math.ceil(tr.duration / Math.max(1e-3, time_step)));
          for (let k = 1; k <= n; k++) { const t = (k / n) * tr.duration; const q = sampleAt(tr, t).joints; push(q, err, dist3(robot.solveFK(prevQ!), robot.solveFK(q)), Math.max(...q.map((v, i) => Math.abs(v - prevQ![i]))), s.t0 + t); }
          continue;
        }
        const pts = densePath(tr, robot);
        const linear = d.kind === 'move' && d.moveType !== 'MoveJ';
        const total = linear ? pts[pts.length - 1].mm : pts[pts.length - 1].deg;
        const n = Math.max(1, Math.ceil(total / Math.max(1e-6, linear ? mm_step : deg_step)), linear ? Math.ceil(pts[pts.length - 1].deg / Math.max(1e-6, deg_step * 4)) : 0);
        const mmStep = pts[pts.length - 1].mm / n;
        for (let k = 1; k <= n; k++) {
          const target = (k / n) * total;
          const pt = interpDense(pts, target, linear ? 'mm' : 'deg');
          push(pt.q, err, mmStep, Math.max(...pt.q.map((v, i) => Math.abs(v - prevQ![i]))), s.t0 + pt.t);
        }
      }
    }
    const executable = this.Update()[0];
    const status = errs.length ? -Math.max(1, errs.length) : executable;
    const msg = errs.length ? errs.map((e) => e.message).join('; ') : 'Success';
    if (save_to_file) {
      const header = [...(robot ? Array.from({ length: robot.dof }, (_, i) => `J${i + 1}`) : []), 'ERROR', 'MM_STEP', 'DEG_STEP', 'MOVE_ID', ...(flags >= 1 ? ['TIME', 'X', 'Y', 'Z'] : [])].join(',');
      this.rdk.writeFile(save_to_file, `${header}\n${rows.map((r) => r.map((v) => +v.toFixed(6)).join(',')).join('\n')}\n`);
    }
    return [msg, rows, status];
  }
  InstructionSelect(ins_id = -1): this {
    const p = this.req<Program>();
    const ins = p.instructions()[ins_id < 0 ? p.instructions().length - 1 : ins_id];
    if (ins) this.rdk.station.select(ins);
    return this;
  }
  setInstruction(ins_id: number, name: string, instype: number, movetype: number, isjointtarget: boolean | number, target: PoseLike | null, joints: number[] | null): this {
    const p = this.req<Program>();
    const ins = p.instructions()[ins_id < 0 ? p.instructions().length + ins_id : ins_id];
    if (!ins) return this;
    ins.name = name;
    if (ins.data.kind === 'move' && (instype === INS_TYPE_MOVE || instype === INS_TYPE_MOVEC)) {
      ins.data.moveType = movetype === MOVE_TYPE_LINEAR ? 'MoveL' : movetype === MOVE_TYPE_CIRCULAR ? 'MoveC' : 'MoveJ';
      const t = ins.data.targetId ? (this.rdk.station.findById(ins.data.targetId) as Target | null) : null;
      const q = joints ? flat(joints) : null;
      if (t) { if (target) t.setPose(toMat4(target)); if (q?.length) t.setJoints(q); if (isjointtarget) t.setAsJointTarget(); else t.setAsCartesianTarget(); }
      else { if (target) ins.data.pose = Array.from(toMat4(target)); if (q?.length) ins.data.joints = q; }
    }
    ins.notify('data');
    return this;
  }
  /** Configuration flags [REAR, LOWERARM, FLIP, turns...] like RoboDK JointsConfig. */
  JointsConfig(joints: number[]): number[] {
    const r = this.req<Robot>();
    const q = flat(joints);
    const f = r.configFlags(q);
    return [f.front ? 0 : 1, f.elbowUp ? 0 : 1, f.wristFlip ? 1 : 0, ...q.map((v) => Math.trunc(v / 360))];
  }
  setJointsHome(joints: number[]): this {
    const r = this.req<Robot>();
    const q = flat(joints);
    const idx = r.chain.joints.map((j, i) => (j.type !== 'fixed' && !j.mimic ? i : -1)).filter((i) => i >= 0);
    idx.forEach((ji, k) => { r.chain.joints[ji].home = q[k] ?? 0; });
    r.notify('home');
    return this;
  }
  setAccuracyActive(accurate: boolean | number = true): this { this.req().setParam('accuracyActive', !!accurate); return this; }
  AccuracyActive(): number { return this.req().getParam('accuracyActive') ? 1 : 0; }
  setAcceleration(a: number): this { const it = this.req(); if (it instanceof Robot) it.motion.accelLinear = a; else if (it instanceof Program) it.setSpeed(undefined, undefined, a); return this; }
  setAccelerationJoints(a: number): this { const it = this.req(); if (it instanceof Robot) it.motion.accelJoints = a; else if (it instanceof Program) it.setSpeed(undefined, undefined, undefined, a); return this; }
  setSpeedJoints(v: number): this { const it = this.req(); if (it instanceof Robot) it.motion.speedJoints = v; else if (it instanceof Program) it.setSpeed(undefined, v); return this; }
  setLink(item: RobolinkItem | null): this { const it = this.req(); const o = item?.item ?? null; if (it instanceof Program) it.setRobot(o); else if (it instanceof Target) it.robotId = o?.id ?? null; else if (it instanceof Robot && o instanceof Tool) it.setTool(o); else if (it.type === ItemType.MACHINING) { if (o instanceof Robot) it.setParam('robotId', o.id); else if (o instanceof Tool) it.setParam('toolId', o.id); else if (o instanceof SceneObject) it.setParam('partId', o.id); else if (o) it.setParam('frameId', o.id); } return this; }
  ObjectLink(link_id = 0): RobolinkItem {
    const it = this.req();
    const kids = it.children.filter((c) => c instanceof SceneObject);
    return new RobolinkItem(this.rdk, kids[link_id] ?? null);
  }
  /** Save this item (RoboDK Item.Save): writes the file through the file system hook; returns the JSON when no file name. */
  Save(filename = ''): string | void { return this.rdk.Save(filename, this); }
  /** RoboDK setMachiningParameters: (program, status) — status = ratio of reachable path points (1 = all), -1 without a part. */
  setMachiningParameters(ncfile = '', part?: RobolinkItem | number | null, params = ''): [RobolinkItem, number] {
    const it = this.req();
    if (ncfile) it.setParam('ncfile', ncfile);
    if (params) it.setParam('machiningParams', params);
    if (part instanceof RobolinkItem && part.item) it.setParam('partId', part.item.id);
    const res = this.rdk.updateMachining(it);
    const prog = this.rdk.station.findById(String(it.getParam('programId') ?? ''));
    return [new RobolinkItem(this.rdk, prog), res ? (res.points ? 1 - res.unreachable / res.points : 0) : -1];
  }
  setMillingParameters(ncfile = '', part?: RobolinkItem | number | null, params = ''): [RobolinkItem, number] { return this.setMachiningParameters(ncfile, part, params); }
  MachiningParameters(): any { const it = this.req(); return { ncfile: it.getParam('ncfile'), part: it.getParam('partId'), params: it.getParam('machiningParams'), programId: it.getParam('programId') }; }
  FilterTarget(pose: PoseLike, joints_approx?: number[]): [Mat, number[]] {
    const r = this.req<Robot>();
    const m = toMat4(pose);
    const res = r.solveIK(m, { seed: joints_approx ? flat(joints_approx) : undefined }, identity());
    return [new Mat(m), res.ok ? res.joints : joints_approx ? flat(joints_approx) : r.joints()];
  }
  FilterProgram(_filestr = ''): [number, string] { return [0, 'Accuracy filtering is not required: simulation kinematics are nominal']; }
  setRunType(type: number): this { const p = this.req<Program>(); p.runMode = type === PROGRAM_RUN_ON_ROBOT ? 'run_on_robot' : 'simulate'; return this; }
  RunType(): number { return this.req<Program>().runMode === 'run_on_robot' ? PROGRAM_RUN_ON_ROBOT : PROGRAM_RUN_ON_SIMULATOR; }
  /** Test a joint move for collisions/reachability: 0 = OK, -1 = collision, -2 = out of limits. */
  MoveJ_Test(j1: number[], j2: number[], minstep_deg = -1): number {
    const r = this.req<Robot>();
    const a = flat(j1), b = flat(j2);
    if (!r.jointsValid(b) || !r.jointsValid(a)) return -2;
    const tr = planMoveJ(r, a, b, r.motion.speedJoints, r.motion.accelJoints);
    return this.rdk.trajectoryCollides(r, tr.samples.map((s) => s.joints), minstep_deg > 0 ? minstep_deg : 1) ? -1 : 0;
  }
  MoveJ_Test_Blend(j1: number[], j2: number[], j3: number[], _blend_deg = 5, minstep_deg = -1): number { const a = this.MoveJ_Test(j1, j2, minstep_deg); return a !== 0 ? a : this.MoveJ_Test(j2, j3, minstep_deg); }
  MoveL_Test(j1: number[], pose: PoseLike, minstep_mm = -1): number {
    const r = this.req<Robot>();
    const target = multiply(r.poseFrame(), toMat4(pose));
    const tr = planMoveL(r, flat(j1), multiply(target, invert(r.poseTool())), r.motion.speedLinear, r.motion.accelLinear);
    if (!tr.ok) return -2;
    return this.rdk.trajectoryCollides(r, tr.samples.map((s) => s.joints), minstep_mm > 0 ? minstep_mm : 1) ? -1 : 0;
  }
  /** Linear search move: move towards the target until a collision (contact) is detected; returns joints at contact. */
  SearchL(target: RobolinkItem | Mat | number[], _blocking = true): number[] | this {
    const it = this.req();
    if (it instanceof Program) { if (target instanceof RobolinkItem && target.item instanceof Target) it.addMoveL(target.item); else it.addInstruction(moveTo('MoveL', this.resolveTarget(target))); return this; }
    const r = it as Robot;
    const tgt = target instanceof RobolinkItem ? multiply(invert(r.poseAbs()), target.item!.poseAbs()) : multiply(r.poseFrame(), toMat4(target as PoseLike));
    const tr = planMoveL(r, r.joints(), multiply(tgt, invert(r.poseTool())), r.motion.speedLinear, r.motion.accelLinear, 0.01);
    for (const s of tr.samples) {
      r.setJoints(s.joints);
      if (checkCollisionsMapped(this.rdk.station, { assets: this.rdk.assets }).some((p) => p.a.item === r || p.b.item === r || r.isAncestorOf(p.a.item) || r.isAncestorOf(p.b.item))) return s.joints;
    }
    return r.joints();
  }
  Scale(scale: number | number[], _pre?: any, _post?: any): this {
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
    for (const c of it.curves) c.points = c.points.map((p) => [p[0] * sv[0], p[1] * sv[1], (p[2] ?? 0) * sv[2], ...p.slice(3)]);
    it.notify('geometry');
    return this;
  }
  setColorShape(color: string | number[], shape_id = 0): this { const it = this.req<SceneObject>(); const g = it.geometry[shape_id]; if (g) { g.color = toCss(color); it.notify('geometry'); } return this; }
  setColorCurve(color: string | number[], _curve_id = -1): this { this.req().setParam('curveColor', toCss(color)); return this; }
  Color(): number[] { const c = this.req().color ?? '#9aa3ad'; return [parseInt(c.slice(1, 3), 16) / 255, parseInt(c.slice(3, 5), 16) / 255, parseInt(c.slice(5, 7), 16) / 255, 1]; }
  setValue(varname: string, value: any = ''): any { this.req().setParam(varname, value); return value; }
  Value(varname = ''): any { return varname ? this.req().getParam(varname) : this.req().params; }
  customInstruction(name: string, path_run: string, _path_icon = '', _blocking = true, cmd_run_on_robot = ''): this {
    const p = this.req<Program>();
    const ins = p.runInstruction(cmd_run_on_robot || path_run, !!cmd_run_on_robot);
    ins.name = name;
    ins.setParam('customPath', path_run);
    return this;
  }
  addMoveSearch(target: RobolinkItem): this { this.SearchL(target); return this; }
  ConnectSafe(robot_ip = '', _max_attempts = 5, _wait = 4): number { this.req<Robot>().connection.ip = robot_ip || this.req<Robot>().connection.ip; return ROBOTCOM_READY; }
  ConnectionParams(): [string, number, string, string, string] { const c = this.req<Robot>().connection; return [c.ip ?? '', c.port ?? 0, '', '', '']; }
  setConnectionParams(ip: string, port = 0, _path = '', _user = '', _pass = ''): this { const r = this.req<Robot>(); r.connection = { ...r.connection, ip, port }; return this; }
  Disconnect(): this { return this; }
  setParamRobotTool(tool_mass = 5, tool_cog?: number[] | null): this { const it = this.req(); it.setParam('toolMass', tool_mass); if (tool_cog) it.setParam('toolCOG', flat(tool_cog)); return this; }
  SelectedFeature(): [number, number, number] { return [this.rdk.station.selection.includes(this.req()) ? 1 : 0, FEATURE_NONE, 0]; }
  GetPoints(feature_type = FEATURE_SURFACE, feature_id = 0): [number[][], string] {
    const it = this.req<SceneObject>();
    if (feature_type === FEATURE_CURVE) { const c = it.curves[feature_id]; return [c ? c.points.map((p) => [...p]) : [], c?.name ?? '']; }
    if (feature_type === FEATURE_POINT) { const p = it.points[feature_id]; return [p ? [[...p.point]] : [], p?.name ?? '']; }
    return [it.points.map((p) => [...p.point]), 'points'];
  }
  ShowSequence(matrix: any, display_type = SEQUENCE_DISPLAY_DEFAULT, timeout = -1): void { this.rdk.ShowSequence(matrix, display_type, timeout, this.item instanceof Robot ? this.item : undefined); }
  /** Absolute pose of each link for the given joints (RoboDK JointPoses). */
  JointPoses(joints?: number[] | null): Mat[] {
    const r = this.req<Robot>();
    const fk = r.fk(joints && flat(joints).length ? flat(joints) : r.joints());
    return fk.linkPoses.map((m) => new Mat(m));
  }
  AddGeometry(fromitem: RobolinkItem, pose: PoseLike): this {
    const it = this.req<SceneObject>();
    const src = fromitem.item as SceneObject;
    if (!(src instanceof SceneObject)) throw new Error('InputError: AddGeometry needs an object');
    const m = toMat4(pose);
    for (const g of src.geometry) it.geometry.push({ ...g, origin: Array.from(multiply(m, g.origin ? Float64Array.from(g.origin) : identity())) });
    it.notify('geometry');
    return this;
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

/** Dense samples of a trajectory with cumulative TCP distance (mm) and joint-space path length (max-norm deg). */
function densePath(tr: Trajectory, robot: Robot): Array<{ t: number; q: number[]; mm: number; deg: number }> {
  const n = Math.max(8, Math.min(2000, Math.ceil(tr.duration / 0.002)));
  const out: Array<{ t: number; q: number[]; mm: number; deg: number }> = [];
  let prev: { q: number[]; p: Mat4 } | null = null, mm = 0, deg = 0;
  for (let k = 0; k <= n; k++) {
    const t = (k / n) * tr.duration;
    const q = sampleAt(tr, t).joints;
    const p = robot.solveFK(q);
    if (prev) { mm += dist3(prev.p, p); deg += Math.max(...q.map((v, i) => Math.abs(v - prev!.q[i]))); }
    out.push({ t, q, mm, deg });
    prev = { q, p };
  }
  return out;
}
function interpDense(pts: Array<{ t: number; q: number[]; mm: number; deg: number }>, target: number, key: 'mm' | 'deg'): { t: number; q: number[] } {
  let lo = 0, hi = pts.length - 1;
  if (target >= pts[hi][key]) return pts[hi];
  while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (pts[mid][key] < target) lo = mid; else hi = mid; }
  const a = pts[lo], b = pts[hi];
  const f = b[key] - a[key] > 1e-12 ? (target - a[key]) / (b[key] - a[key]) : 1;
  return { t: a.t + (b.t - a.t) * f, q: a.q.map((v, i) => v + (b.q[i] - v) * f) };
}

export class Robolink {
  generated = new Map<string, any>();
  onRender: (() => void) | null = null;
  onRunProgram: ((p: Program) => void) | null = null;
  onStop: (() => void) | null = null;
  onMessage: ((msg: string, popup: boolean) => void) | null = null;
  /** Browser: whether the app is playing this program (legacy playback path without a shared runtime). */
  isPlaying: ((p: Program) => boolean) | null = null;
  /** Generated program files when no file system is available (browser downloads them). */
  onGeneratedFiles: ((files: Array<{ name: string; content: string | Uint8Array }>, folder: string) => void) | null = null;
  /** Called when AddFile opens a new station (.rdk/.vbstation) or setActiveStation switches it. */
  onStationReplaced: ((s: Station) => void) | null = null;
  fileSystem: ApiFileSystem | null = null;
  /** RoboDK path parameters (PATH_OPENSTATION, FILE_OPENSTATION, PATH_DESKTOP, PATH_LIBRARY, PATH_ROBODK). */
  paths: Record<string, string> = {};
  runMode = RUNMODE_SIMULATE;
  simSpeed = 1;
  readonly exec: ApiExec & { attachedRuntime?: boolean };
  /** Running station Python scripts by item id (Busy / WaitFinished). */
  readonly scriptRuns = new Map<string, Promise<void>>();
  /** Nesting depth of RPC calls being executed (rpc.ts). */
  callDepth = 0;
  private offline = new Map<string, OfflineRecorder>();
  private stationSignals = new Map<string, any>();
  constructor(public station: Station) {
    this.spray = new SpraySimulator(station);
    this.exec = new ApiExec(this);
    this.exec.onTick = (dt: number) => this.spray.step(dt);
  }
  render(): void { this.onRender?.(); }
  Render(_v = true): void { this.render(); }
  /** Replace the station (AddFile .rdk, setActiveStation): runtime, spray and hooks follow. */
  setStation(s: Station): void {
    this.station = s;
    (this as any).spray = new SpraySimulator(s, this.assets);
    this.onStationReplaced?.(s);
  }
  /** Item by name (exact, else case-insensitive partial match). Empty name returns the first item of the type. */
  Item(name: string, type: number = ITEM_TYPE_ANY): RobolinkItem {
    if (type === ITEM_TYPE_STATION || (name === '' && type === ITEM_TYPE_ANY)) return new RobolinkItem(this, this.station);
    const t = type === ITEM_TYPE_ROBOT_ARM ? ITEM_TYPE_ROBOT : type;
    if (name === '') return new RobolinkItem(this, this.station.itemsOfType(t as ItemType)[0] ?? null);
    const exact = this.station.find(name, t as ItemType);
    if (exact) return new RobolinkItem(this, exact);
    const lower = name.toLowerCase();
    const items = (t === ITEM_TYPE_ANY ? [...this.station.walk()].slice(1) : this.station.itemsOfType(t as ItemType)).filter((i) => !(i instanceof Instruction));
    return new RobolinkItem(this, items.find((i) => i.name.toLowerCase() === lower) ?? items.find((i) => i.name.toLowerCase().includes(lower)) ?? null);
  }
  ItemList(type: number = ITEM_TYPE_ANY, listNames = false): any[] {
    const t = type === ITEM_TYPE_ROBOT_ARM ? ITEM_TYPE_ROBOT : type;
    const items = (t === ITEM_TYPE_ANY ? [...this.station.walk()].slice(1) : this.station.itemsOfType(t as ItemType)).filter((i) => !(i instanceof Instruction));
    return listNames ? items.map((i) => i.name) : items.map((i) => new RobolinkItem(this, i));
  }
  ItemUserPick(_msg = '', type: number | RobolinkItem[] = ITEM_TYPE_ANY): RobolinkItem {
    if (Array.isArray(type)) return type.find((i) => this.station.selection.includes(i.item!)) ?? type[0] ?? new RobolinkItem(this, null);
    const sel = this.station.selection.find((s) => type === ITEM_TYPE_ANY || s.type === type);
    return new RobolinkItem(this, sel ?? (type === ITEM_TYPE_ANY ? null : this.station.itemsOfType(type as ItemType)[0] ?? null));
  }
  ActiveStation(): RobolinkItem { return new RobolinkItem(this, this.station); }
  AddStation(name = 'New Station'): RobolinkItem {
    if (this.onStations) { const s = this.onStations.add(name); this.setStation(s); return new RobolinkItem(this, s); }
    this.station.setName(name);
    return new RobolinkItem(this, this.station);
  }
  AddFrame(name: string, parent?: RobolinkItem | number): RobolinkItem {
    const f = new Frame(name);
    ((parent instanceof RobolinkItem ? parent.item : null) ?? this.station).addChild(f);
    return new RobolinkItem(this, f);
  }
  AddFolder(name: string, parent?: RobolinkItem | number): RobolinkItem { const f = new Folder(name); ((parent instanceof RobolinkItem ? parent.item : null) ?? this.station).addChild(f); return new RobolinkItem(this, f); }
  AddTarget(name: string, parent?: RobolinkItem | number, robot?: RobolinkItem | number): RobolinkItem {
    const t = new Target(name);
    const r = ((robot instanceof RobolinkItem ? robot.item : null) as Robot | null) ?? this.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    const p = (parent instanceof RobolinkItem ? parent.item : null) ?? r?.activeFrame() ?? this.station;
    p.addChild(t);
    if (r instanceof Robot) { t.setPoseAbs(r.poseTCPAbs()); t.setJoints(r.joints()); t.robotId = r.id; }
    return new RobolinkItem(this, t);
  }
  AddProgram(name: string, robot?: RobolinkItem | number): RobolinkItem {
    const p = new Program(name);
    this.station.addChild(p);
    const r = (robot instanceof RobolinkItem ? robot.item : null) ?? this.station.itemsOfType(ItemType.ROBOT)[0] ?? null;
    if (r) p.setRobot(r);
    return new RobolinkItem(this, p);
  }
  AddRobot(libraryIdOrName: string, parent?: RobolinkItem | number): RobolinkItem {
    const e = ROBOT_LIBRARY.find((x) => x.id === libraryIdOrName || x.name === libraryIdOrName || x.name.toLowerCase().includes(libraryIdOrName.toLowerCase()));
    if (!e) throw new Error(`Robot ${libraryIdOrName} not in library`);
    const r = createRobotFromLibrary(e.id);
    ((parent instanceof RobolinkItem ? parent.item : null) ?? this.station).addChild(r);
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

  // ---- execution helpers (exec.ts) ----
  /** Advance simulated time by `seconds` (virtual clock) or wait for it (realtime). */
  Pause(seconds: number): void | Promise<void> {
    if (this.exec.mode === 'virtual' || this.exec.syncDepth > 0) { this.exec.advance(seconds); return; }
    return new Promise((r) => setTimeout(r, (seconds * 1000) / Math.max(1e-3, this.exec.speed)));
  }
  /** Wait until cond() holds, advancing the simulation (virtual) or polling (realtime). */
  waitUntil(cond: () => boolean, timeoutS: number): void | Promise<void> {
    if (cond()) return;
    if (this.exec.mode === 'virtual' || this.exec.syncDepth > 0) {
      let t = 0;
      while (!cond() && t < timeoutS) { this.exec.advance(0.05); t += 0.05; if (t > 60 && !this.exec.busy(null)) break; }
      if (!cond()) throw new Error(`StoppedError: timeout waiting for the condition (${timeoutS.toFixed(1)} s simulated)`);
      return;
    }
    const t0 = Date.now();
    return new Promise((resolve, reject) => {
      const iv = setInterval(() => { if (cond()) { clearInterval(iv); resolve(); } else if (Date.now() - t0 > timeoutS * 1000) { clearInterval(iv); reject(new Error('StoppedError: timeout')); } }, 20);
    });
  }
  /** Start a station Python program (non-blocking); tracked for Busy/WaitFinished. */
  startScript(item: CoreItem, args: string[], robot: CoreItem | null): void {
    const p = this.exec.runScript(item, args, robot).catch((e) => { this.onMessage?.(`${item.name}: ${e?.message ?? e}`, false); }).finally(() => { if (this.scriptRuns.get(item.id) === p) this.scriptRuns.delete(item.id); });
    this.scriptRuns.set(item.id, p);
  }
  setSignal(name: string, v: any): void { this.stationSignals.set(name, v); this.exec.rt().signals?.set(name, v); }
  getSignal(name: string): any { return this.exec.rt().signals?.get(name) ?? this.stationSignals.get(name) ?? this.station.getParam(name); }
  /** Distance from a point (TCP pose) to an object: bounding box / colliders when available, else origin distance. */
  distanceToObject(o: SceneObject, tcp: Mat4): number {
    const p = [tcp[12], tcp[13], tcp[14]];
    const bb = o.bbox;
    if (bb) {
      const local = multiply(invert(o.poseAbs()), tcp);
      const q = [local[12], local[13], local[14]];
      const d = q.map((v, i) => Math.max(bb.min[i] - v, 0, v - bb.max[i]));
      return Math.hypot(d[0], d[1], d[2]);
    }
    const a = o.poseAbs();
    return Math.hypot(a[12] - p[0], a[13] - p[1], a[14] - p[2]);
  }

  // ---- offline programming (RUNMODE_MAKE_ROBOTPROG) ----
  /** Open an offline program for `robot` (RoboDK ProgramStart): robot moves are recorded until Finish(). Returns 0. */
  ProgramStart(name: string, folder = '', post = '', robot?: RobolinkItem | null): number {
    const given = robot?.item ?? null;
    const r = (given instanceof Robot ? given : given instanceof Program && given.robot() instanceof Robot ? (given.robot() as Robot) : null) ?? this.station.itemsOfType<Robot>(ItemType.ROBOT)[0] ?? null;
    if (post && !getPost(post)) throw new Error(`InputError: unknown post processor ${post}`);
    const key = r?.id ?? '';
    if (this.offline.has(key)) this.finishOffline(key);
    const p = new Program(name);
    p.parent = this.station;
    if (r) p.setRobot(r);
    if (r instanceof Robot) { if (r.activeFrameId) p.setFrame(this.station.findById(r.activeFrameId)); if (r.activeToolId) p.setTool(this.station.findById(r.activeToolId)); }
    this.offline.set(key, { program: p, folder, post, robot: r instanceof Robot ? r : null });
    if (this.runMode === RUNMODE_SIMULATE) this.runMode = RUNMODE_MAKE_ROBOTPROG;
    return 0;
  }
  recorderFor(r: Robot): OfflineRecorder | undefined { return this.offline.get(r.id) ?? this.offline.get(''); }
  recording(p: Program): boolean { return [...this.offline.values()].some((o) => o.program === p); }
  /** Record an instruction for a robot in offline-programming mode. Returns true when recorded. */
  record(r: Robot, d: InstructionData): boolean {
    if (this.runMode !== RUNMODE_MAKE_ROBOTPROG && this.runMode !== RUNMODE_MAKE_ROBOTPROG_AND_START && this.runMode !== RUNMODE_MAKE_ROBOTPROG_AND_UPLOAD) return false;
    const rec = this.recorderFor(r);
    if (!rec) return false;
    rec.program.addInstruction(d);
    return true;
  }
  private finishOffline(key: string): string[] {
    const rec = this.offline.get(key);
    if (!rec) return [];
    this.offline.delete(key);
    const res = this.generateProgram(rec.program, rec.folder, rec.post || null, RUNMODE_MAKE_ROBOTPROG, true);
    const files = (res as any).files as string[] | undefined;
    return files ?? [];
  }
  /** Generate pending offline programs (RoboDK Finish / disconnect). Returns the written file names. */
  Finish(): string[] {
    const out: string[] = [];
    for (const k of [...this.offline.keys()]) out.push(...this.finishOffline(k));
    if (this.runMode === RUNMODE_MAKE_ROBOTPROG) this.runMode = RUNMODE_SIMULATE;
    return out;
  }
  /** Post-process a program and write the files (MakeProgram / Finish). */
  generateProgram(p: Program, folder: string, post: string | null, runMode: number, sync = false): any {
    const robot = p.robot();
    const postName = post || (robot ? String(robot.getParam('PostProcessor', '') || (robot instanceof Robot ? robot.postProcessor : '') || 'Generic') : 'Generic');
    const pp = getPost(postName);
    if (!pp) throw new Error(`InputError: unknown post processor ${postName}`);
    const finish = (files: Array<{ name: string; content: string; encoding?: string }>): [boolean, string, boolean] => {
      this.generated.set(p.id, files);
      const dir = folder || this.paths.PATH_PROGRAMS || this.paths.PATH_DESKTOP || '';
      const written: string[] = [];
      if (this.fileSystem && dir) {
        const sep = /[\\/]$/.test(dir) ? '' : '/';
        this.fileSystem.mkdirp?.(dir);
        for (const f of files) { const path = `${dir}${sep}${f.name}`; this.fileSystem.writeFile(path, f.encoding === 'base64' ? b64(f.content) : f.content); written.push(path); }
      } else this.onGeneratedFiles?.(files.map((f) => ({ name: f.name, content: f.encoding === 'base64' ? b64(f.content) : f.content })), dir);
      const log = written.length ? `Program ${p.name} generated with ${pp.name}: ${written.join(', ')}` : `Program ${p.name} generated with ${pp.name}: ${files.map((f) => f.name).join(', ')}`;
      const res: any = [true, log, runMode === RUNMODE_MAKE_ROBOTPROG_AND_UPLOAD || runMode === RUNMODE_MAKE_ROBOTPROG_AND_START ? false : false];
      res.files = written;
      return res;
    };
    const prog = compileForPost(this.station, p);
    if (pp.generateAsync && !sync) return pp.generateAsync(prog).then(finish);
    return finish(pp.generate(prog));
  }

  // ---- targets / mechanisms ----
  AddTargetJ(name: string, joints: number[], parent?: RobolinkItem | number, robot?: RobolinkItem | number): RobolinkItem {
    const t = this.AddTarget(name, parent, robot);
    const r = ((robot instanceof RobolinkItem ? robot.item : null) as Robot | null) ?? this.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    if (t.item instanceof Target && r) { t.item.setJoints(flat(joints)); t.item.setPoseAbs(r.poseTCPAbs(flat(joints))); t.item.setAsJointTarget(); }
    return t;
  }
  /** RoboDK BuildMechanism (official MAKE_ROBOT_* numbers). */
  BuildMechanism(type: number, list_obj: RobolinkItem[] = [], parameters: number[] = [], joints_build: number[] = [], joints_home: number[] = [], joints_senses: number[] = [], joints_lim_low: number[] = [], joints_lim_high: number[] = [], base?: Mat | null, tool?: Mat | null, name = 'New robot', robot?: RobolinkItem | null): RobolinkItem {
    const mech = MECH_TYPE[type];
    if (mech === undefined) throw new Error(`InputError: BuildMechanism type ${type} is not supported`);
    const spec: MechanismSpec = { type: mech, parameters: flat(parameters), jointsBuild: flat(joints_build), jointsHome: flat(joints_home), jointsSenses: flat(joints_senses), lower: flat(joints_lim_low).length ? flat(joints_lim_low) : undefined, upper: flat(joints_lim_high).length ? flat(joints_lim_high) : undefined, base: base?.m, tool: tool?.m, name };
    const r = buildMechanism(spec);
    // attach link geometry: list_obj[0] = base, list_obj[i] = link i
    (list_obj ?? []).forEach((o, i) => {
      if (!(o?.item instanceof SceneObject)) return;
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
  AddMachiningProject(name = 'Curve follow settings', robot?: RobolinkItem | number): RobolinkItem {
    const it = new CoreItem(ItemType.MACHINING, name);
    this.station.addChild(it);
    const r = (robot instanceof RobolinkItem ? robot.item : null) ?? this.station.itemsOfType(ItemType.ROBOT)[0];
    if (r) it.setParam('robotId', r.id);
    return new RobolinkItem(this, it);
  }
  AddMillingProject(name = 'Milling settings', robot?: RobolinkItem | number): RobolinkItem { return this.AddMachiningProject(name, robot); }
  /** (Re)generate the program of a machining project item from its part curves (the previous program is replaced in place). */
  updateMachining(it: CoreItem): { points: number; unreachable: number } | null {
    const robot = this.station.findById(String(it.getParam('robotId') ?? '')) as Robot | null;
    const part = this.station.findById(String(it.getParam('partId') ?? '')) as SceneObject | null;
    if (!(robot instanceof Robot) || !(part instanceof SceneObject) || !part.curves.length) return null;
    const params = String(it.getParam('machiningParams') ?? '');
    const old = this.station.findById(String(it.getParam('programId') ?? ''));
    // remove the previous program and the targets it generated (no leaks on regeneration)
    if (old instanceof Program) { const used = new Set(old.instructions().flatMap((i) => (i.data.kind === 'move' ? [i.data.targetId, i.data.viaTargetId] : []))); for (const t of this.station.itemsOfType<Target>(ItemType.TARGET)) if (used.has(t.id)) t.delete(); }
    old?.delete();
    const str = (k: string): string | undefined => params.match(new RegExp(`${k}\\s*[=:]\\s*([^\\s,;]+)`, 'i'))?.[1];
    if (part.curves.some((c) => (c as any).kind)) {
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
  Cam2D_Add(item?: RobolinkItem | number | null, params = '', camera_item?: RobolinkItem | null): RobolinkItem {
    if (camera_item?.item instanceof CameraItem) { this.applyCamParams(camera_item.item, params); this.cams.set(camera_item.item.id, camera_item.item); return camera_item; }
    const cam = new CameraItem('Camera');
    ((item instanceof RobolinkItem ? item.item : null) ?? this.station).addChild(cam);
    this.applyCamParams(cam, params);
    this.cams.set(cam.id, cam);
    return new RobolinkItem(this, cam);
  }
  /** RoboDK Cam2D parameters: FOV (vertical, deg), SIZE=WxH, FAR_LENGTH / NEAR_LENGTH (and RoboDK's FAR_LENGHT spelling), FOCAL_LENGTH (mm) + PIXELSIZE (µm) → focal length in pixels, SNAPSHOT=WxH, DEPTH, BG_COLOR; LIGHT_* / window flags are kept but do not change the render. */
  private applyCamParams(cam: CameraItem, params: string) {
    const get = (k: string) => { const m = params.match(new RegExp(`(?:^|\\s)${k}=([^\\s]+)`, 'i')); return m ? m[1] : null; };
    cam.fovAxis = 'vertical';
    const size = get('SIZE'); if (size) { const [w, h] = size.split('x').map(Number); if (w) cam.width = w; if (h) cam.height = h; }
    const fov = get('FOV'); if (fov && Number.isFinite(+fov)) cam.fov = +fov;
    const focal = get('FOCAL_LENGTH') ?? get('FOCAL_LENGHT'), pix = get('PIXELSIZE');
    if (focal && pix && +pix > 0) setFocalLengthPx(cam, (+focal * 1000) / +pix);
    const far = get('FAR_LENGTH') ?? get('FAR_LENGHT') ?? get('FAR'); if (far && +far > 0) cam.far = +far;
    const near = get('NEAR_LENGTH') ?? get('NEAR_LENGHT') ?? get('NEAR'); if (near && +near > 0) cam.near = +near;
    if (/(?:^|\s)DEPTH(?:\s|$)/i.test(params)) cam.kind = 'depth';
    const snap = get('SNAPSHOT'); if (snap) cam.setParam('cam2dSnapshot', snap);
    const bg = get('BG_COLOR'); if (bg) cam.setParam('cam2dBackground', bg);
    cam.setParam('cam2dParams', params);
  }
  /** Settings string of a camera as RoboDK reports it (cam_item.setParam('Settings')): FOCAL_LENGTH FOV FAR_LENGTH NEAR_LENGTH SIZE [DEPTH]. */
  Cam2D_Settings(cam?: RobolinkItem | number): string {
    const c = cam instanceof RobolinkItem && cam.item instanceof CameraItem ? cam.item : [...this.cams.values()][0] ?? this.station.itemsOfType<CameraItem>(ItemType.CAMERA)[0];
    if (!c) return '';
    const fv = cameraFovs(c).v;
    return `FOV=${+fv.toFixed(4)} FAR_LENGTH=${c.far} NEAR_LENGTH=${c.near} SIZE=${c.width}x${c.height}${c.kind === 'depth' ? ' DEPTH' : ''}`;
  }
  Cam2D_SetParams(params: string, cam?: RobolinkItem | number): boolean {
    const c = cam instanceof RobolinkItem && cam.item instanceof CameraItem ? cam.item : [...this.cams.values()][0];
    if (!c) return false;
    this.applyCamParams(c, params);
    return true;
  }
  Cam2D_Close(cam?: RobolinkItem | number): boolean {
    if (cam instanceof RobolinkItem && cam.item) { this.cams.delete(cam.item.id); cam.item.delete(); return true; }
    for (const c of this.cams.values()) c.delete();
    this.cams.clear();
    return true;
  }
  /**
   * RoboDK Cam2D_Snapshot: with a file name, renders the camera, saves the image (PNG; `.grey32` or params 'DEPTH' save
   * RoboDK's 32-bit depth map) and returns 1 (0 when nothing could be written); with "" returns the encoded bytes
   * (PNG, decoded by cv2.imdecode as BGR; 'DEPTH' → grey32: big-endian u32 w, h, rows bottom-up, depth/FAR_LENGTH·(2³²−1)).
   * Pixels come from the WebGL renderer when the browser registered it, else from the software rasterizer (headless).
   */
  Cam2D_Snapshot(file_save_img = '', cam?: RobolinkItem | number, params = ''): Uint8Array | number {
    const it = (cam instanceof RobolinkItem ? cam.item : null) ?? [...this.cams.values()][0] ?? this.station.itemsOfType<CameraItem>(ItemType.CAMERA)[0];
    if (!(it instanceof CameraItem)) return file_save_img ? 0 : new Uint8Array(0);
    const depth = /(?:^|\s)DEPTH(?:\s|$)/i.test(params) || /\.grey32$/i.test(file_save_img);
    const snap = String(it.params.cam2dSnapshot ?? '').split('x').map(Number);
    const w = snap[0] > 0 ? snap[0] : it.width, h = snap[1] > 0 ? snap[1] : it.height;
    const r = renderCamera(this.station, it, { width: w, height: h, rgb: !depth, ground: false });
    let bytes: Uint8Array;
    if (depth) bytes = encodeGrey32(r.depth, r.width, r.height, it.far);
    else {
      const rgba = r.rgba ?? new Uint8ClampedArray(r.width * r.height * 4);
      const bg = String(it.params.cam2dBackground ?? '').toLowerCase();
      if (bg && r.ids.length === r.width * r.height) { const col = bg === 'black' ? [0, 0, 0] : bg === 'white' ? [255, 255, 255] : /^#?[0-9a-f]{6}$/.test(bg) ? [0, 2, 4].map((k) => parseInt(bg.replace('#', '').slice(k, k + 2), 16)) : null; if (col) for (let i = 0; i < r.ids.length; i++) if (r.ids[i] < 0) { rgba[i * 4] = col[0]; rgba[i * 4 + 1] = col[1]; rgba[i * 4 + 2] = col[2]; } }
      bytes = encodePNG(rgba, r.width, r.height);
    }
    if (!file_save_img) return bytes;
    try {
      if (this.fileSystem) { this.writeFile(file_save_img, bytes); return 1; }
      const fs = (globalThis as any).process?.getBuiltinModule?.('node:fs');
      if (fs) { const dir = file_save_img.replace(/[\\/][^\\/]*$/, ''); if (dir && dir !== file_save_img) fs.mkdirSync(dir, { recursive: true }); fs.writeFileSync(file_save_img, bytes); return 1; }
      if (this.onGeneratedFiles) { this.writeFile(file_save_img, bytes); return 1; }
    } catch { return 0; }
    return 0;
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
  /** RoboDK CalibrateReference: [pose, stats]. */
  CalibrateReference(joints_points: number[][], method = CALIBRATE_FRAME_3P_P1_ON_X, use_joints = false, robot?: RobolinkItem | null): [Mat, number[]] {
    const pose = this.Calibrate_Reference(joints_points, method, use_joints, robot ?? undefined);
    const res = this.lastCalibration;
    return [pose, [res?.meanError ?? 0, res?.maxError ?? 0, ...(res?.errors ?? [])]];
  }
  lastCalibration: any = null;
  /** poses: joints (input_format = JOINT_FORMAT) or flange poses [x,y,z,w,p,r] (KUKA style). Returns [tcp_xyz, stats[mean,max,...], errors] */
  CalibrateTool(poses_xyzwpr: number[][], input_format = JOINT_FORMAT, algorithm = CALIBRATE_TCP_BY_POINT, robot?: RobolinkItem | null, tool?: RobolinkItem | null): [number[], number[], number[]] {
    const r = (robot?.item as Robot | null) ?? this.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    const flanges = poses_xyzwpr.map((p) => (input_format === JOINT_FORMAT && r ? r.solveFKFlange(p) : kukaToPose(p[0], p[1], p[2], p[3], p[4], p[5])));
    const algo = Array.isArray(algorithm) ? algorithm[0] : algorithm;
    const res = algo === CALIBRATE_TCP_BY_LINE ? calibrateTcpByLine(flanges.slice(0, Math.max(3, flanges.length - 2)), flanges.slice(Math.max(3, flanges.length - 2))) : calibrateTcpByPoint(flanges);
    if (tool?.item instanceof Tool) { const m = tool.item.poseTool(); m[12] = res.tcp[0]; m[13] = res.tcp[1]; m[14] = res.tcp[2]; tool.item.setPoseTool(m); }
    this.lastCalibration = res;
    return [[...res.tcp], [res.meanError, res.maxError], [...res.errors]];
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
  MeasurePose(_target = -1, _time_avg = 0, _tip_xyz = [0, 0, 0]): [Mat, number[]] {
    const r = this.station.itemsOfType<Robot>(ITEM_TYPE_ROBOT as ItemType)[0];
    if (!r) return [new Mat(identity()), [-1]];
    const m = r.poseTCPAbs();
    const p = simulateTrackerMeasure(r, Number(this.station.getParam('trackerNoiseMm', 0.03)), this.seed);
    m[12] = p[0]; m[13] = p[1]; m[14] = p[2];
    return [new Mat(m), [1]];
  }
  StereoCamera_Measure(): [Mat, number[]] { return this.MeasurePose(); }
  Popup_ISO9283_CubeProgram(robot?: RobolinkItem | number, center: number[] = [1000, 0, 800], side = 400, _blocking = true): RobolinkItem {
    const r = ((robot instanceof RobolinkItem ? robot.item : null) as Robot | null) ?? this.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
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
  Collision_Line(p1: number[] | number[][], p2: number[] | number[][], ref?: Mat | null): any {
    if (Array.isArray(p1[0])) { const out = (p1 as number[][]).map((a, i) => this.Collision_Line(a, (p2 as number[][])[i], ref)); const hit = out.find((o) => o[0]); return [hit ? hit[1] : new RobolinkItem(this, null), out.map((o) => o[2])]; }
    const a = ref ? transformPointM(ref.m, p1 as number[]) : (p1 as number[]), b = ref ? transformPointM(ref.m, p2 as number[]) : (p2 as number[]);
    const hit = collisionLine(this.station, [a[0], a[1], a[2]], [b[0], b[1], b[2]], { assets: this.assets });
    return [!!hit, new RobolinkItem(this, hit?.item ?? null), hit ? [...hit.point] : [0, 0, 0]];
  }
  Collision_SetPair(item1: RobolinkItem, item2: RobolinkItem, id1 = -1, id2 = -1, collision_check = true): boolean {
    if (!item1?.item || !item2?.item) return false;
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
  setCollisionActivePairList(list_check_state: number[], list_item1: RobolinkItem[], list_item2: RobolinkItem[], list_id1?: number[] | null, list_id2?: number[] | null): number { return this.Collision_SetPairList(list_item1, list_item2, list_id1 ?? [], list_id2 ?? [], list_check_state.map((s) => s !== COLLISION_OFF)); }
  CollisionActivePairList(): Array<[RobolinkItem, RobolinkItem, number, number]> { const m = getCollisionMap(this.station); return m.enabled.map((p) => [new RobolinkItem(this, this.station.findById(p[0])), new RobolinkItem(this, this.station.findById(p[2])), p[1], p[3]] as [RobolinkItem, RobolinkItem, number, number]); }
  private lastCollisions: CollisionPair[] = [];
  Collisions(): number { this.lastCollisions = checkCollisionsMapped(this.station, { assets: this.assets, meshAccurate: true }); return this.lastCollisions.length; }
  Collision(item1: RobolinkItem, item2: RobolinkItem): number { return item1?.item && item2?.item ? itemsCollide(item1.item, item2.item, { assets: this.assets, meshAccurate: true }).length : 0; }
  CollisionItems(): RobolinkItem[] { this.Collisions(); const set = new Set<CoreItem>(); for (const p of this.lastCollisions) { set.add(p.a.item); set.add(p.b.item); } return [...set].map((i) => new RobolinkItem(this, i)); }
  CollisionPairs(): Array<[RobolinkItem, RobolinkItem, number, number]> { this.Collisions(); return this.lastCollisions.map((p) => [new RobolinkItem(this, p.a.item), new RobolinkItem(this, p.b.item), p.a.linkIndex ?? -1, p.b.linkIndex ?? -1]); }
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
  Paste(paste_to?: RobolinkItem | number, paste_times = 1): RobolinkItem | RobolinkItem[] {
    const out: RobolinkItem[] = [];
    for (let i = 0; i < paste_times; i++) out.push(new RobolinkItem(this, clipboard.paste((paste_to instanceof RobolinkItem ? paste_to.item : null) ?? this.station, this.station)));
    return paste_times === 1 ? out[0] : out;
  }
  Duplicate(item: RobolinkItem): RobolinkItem { return new RobolinkItem(this, item.item ? cloneItem(item.item) : null); }
  getOpenStations(): RobolinkItem[] { return (this.onStations?.list() ?? [this.station]).map((s) => new RobolinkItem(this, s)); }
  setActiveStation(stn: RobolinkItem): void { if (stn.item instanceof Station) { this.onStations?.setActive(stn.item); this.setStation(stn.item); } }
  CloseStation(): void { if (this.onStations) this.onStations.close(); else this.setStation(new Station('New station')); }
  CloseRoboDK(): void { this.onWindow?.('close'); }
  getFlagsItem(item: RobolinkItem): number { return this.itemFlags.get(item.item?.id ?? '') ?? FLAG_ITEM_ALL; }
  setFlagsItem(item: RobolinkItem, flags = FLAG_ITEM_ALL): number { if (item.item) { this.itemFlags.set(item.item.id, flags); item.item.setParam('flags', flags); } return flags; }
  getFlagsRoboDK(): number { return this.flagsRoboDK; }
  setFlagsRoboDK(flags = FLAG_ROBODK_ALL): number { this.flagsRoboDK = flags; this.onWindow?.('flags', flags); return flags; }
  HideRoboDK(): void { this.onWindow?.('hide'); }
  ShowRoboDK(): void { this.onWindow?.('show'); }
  setWindowState(state = WINDOWSTATE_NORMAL): void { this.onWindow?.('state', state); }
  setInteractiveMode(mode = SELECT_MOVE, _default_ref_flags = 0, _custom_items?: any, _custom_ref_flags?: any): void { this.onInteractiveMode?.(mode); }
  setViewPose(pose: PoseLike): void { this.onViewPose?.set(toMat4(pose)); }
  ViewPose(_preset = -1): Mat { return new Mat(this.onViewPose?.get() ?? identity()); }
  Joints(robots?: RobolinkItem[] | null): number[][] { return (robots ?? this.ItemList(ITEM_TYPE_ROBOT)).map((r: RobolinkItem) => r.Joints()); }
  setJoints(robots: RobolinkItem[], joints: number[][]): void { robots.forEach((r, i) => r.setJoints(joints[i])); }
  setPoses(items: RobolinkItem[], poses: Array<Mat | number[]>): void { if (items.length !== poses.length) throw new Error('InputError: the number of items must match the number of poses'); items.forEach((it, i) => it.setPose(poses[i])); }
  setPosesAbs(items: RobolinkItem[], poses: Array<Mat | number[]>): void { if (items.length !== poses.length) throw new Error('InputError: the number of items must match the number of poses'); items.forEach((it, i) => it.setPoseAbs(poses[i])); }
  MergeItems(list: RobolinkItem[] = []): RobolinkItem {
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

  // ---- program execution ----
  private findRunnable(name: string): CoreItem | null { const n = parseCall(name).name; return this.station.find(n, ItemType.PROGRAM) ?? this.station.find(n, ItemType.PROGRAM_PYTHON) ?? null; }
  /** RunCode(code, True) runs the program/Python program named in the call (with arguments); otherwise shows the code. */
  RunCode(code: string, code_is_fcn_call = false): number {
    const call = parseCall(code);
    const it = code_is_fcn_call ? this.findRunnable(code) : null;
    if (it) { new RobolinkItem(this, it).RunCode(call.args); return 0; }
    this.onMessage?.(code, false);
    return 0;
  }
  RunMessage(message: string, message_is_comment = false): void { if (!message_is_comment) this.onMessage?.(message, false); }
  /** Run a program by name (optionally waiting for it to finish). Returns 0, or -1 when there is no such program. */
  RunProgram(fcn_param: string, wait_for_finished = false): any {
    const it = this.findRunnable(fcn_param);
    if (!it) { if (wait_for_finished) throw new Error(`InputError: Invalid program ${fcn_param}`); return -1; }
    const item = new RobolinkItem(this, it);
    item.RunCode(parseCall(fcn_param).args);
    if (wait_for_finished) { const w = item.WaitFinished(); if (w && typeof w.then === 'function') return w.then(() => 0); }
    return 0;
  }
  /** Display a joint/pose sequence (browser: ghost robots); headless: a no-op (the robot does not move, like RoboDK). */
  ShowSequence(matrix: any, _display_type = SEQUENCE_DISPLAY_DEFAULT, _timeout = -1, robot?: Robot): void {
    const r = robot ?? this.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    if (!r || !this.onShowSequence) return;
    const rows: number[][] = Array.isArray(matrix) ? matrix.map((m: any) => (m instanceof Mat ? r.jointsForTarget(m.m) ?? [] : flat(m))) : [];
    this.onShowSequence(r, rows.filter((q) => q.length));
  }
  FilterTarget(pose: Mat | number[], joints_approx?: number[], robot?: RobolinkItem): [Mat, number[]] { const r = robot ?? new RobolinkItem(this, this.station.itemsOfType(ItemType.ROBOT)[0]); return r.FilterTarget(pose, joints_approx); }
  getParams(): Array<[string, any]> { return Object.entries({ ...this.station.settings, ...this.station.params }).filter(([, v]) => typeof v !== 'object' || v === null).map(([k, v]) => [k, v]); }
  /** Simulated time (s): the app clock in the browser, else the station runtime clock (plus setSimulationTime offset). */
  SimulationTime(): number { if (this.simTime) return this.simTime.get(); const t = (this.exec.rt().time ?? 0) + this.simTimeBase; this.station.simTime = t; return t; }
  setSimulationTime(t: number): void { if (this.simTime) this.simTime.set(t); else { this.simTimeBase = t - (this.exec.rt().time ?? 0); this.station.simTime = t; } }
  private simTimeBase = 0;

  // ---- spray ----
  Spray_Add(item_tool?: RobolinkItem | number | null, item_object?: RobolinkItem | number | null, params = '', _points?: any, _geometry?: any): number {
    const tool = ((item_tool instanceof RobolinkItem ? item_tool.item : null) as Tool | null) ?? (this.station.itemsOfType<Robot>(ItemType.ROBOT).map((r) => r.activeTool()).find(Boolean) ?? null);
    const obj = ((item_object instanceof RobolinkItem ? item_object.item : null) as SceneObject | null) ?? this.station.itemsOfType<SceneObject>(ItemType.OBJECT).find((o) => !(o instanceof Tool) && o.visible) ?? null;
    if (!tool || !obj) return -1;
    const g = this.spray.add(tool, obj, { angleDeg: num(params, 'ANGLE', 15), range: num(params, 'RANGE', 400), rate: num(params, 'RATE', 1), sampleSpacing: num(params, 'SAMPLE', 20) });
    return g.id;
  }
  Spray_SetState(state = SPRAY_ON, id_spray: number | 'all' = -1): number { this.spray.setState(id_spray === -1 || id_spray === 'all' ? 'all' : id_spray, !!state); return 1; }
  /** [info, rows]: one row per spray gun [id, material used %, material wasted %, particles, coverage %] (Python: one column each). */
  Spray_GetStats(id_spray: number | 'all' = -1): [string, number[][]] {
    const guns = [...this.spray.guns.values()].filter((g) => id_spray === -1 || id_spray === 'all' || g.id === id_spray);
    const rows = guns.map((g) => { const s = this.spray.stats(g.id); const hits = g.samples.filter((x) => x.dep > 0).length; const used = g.totalTime > 0 ? (100 * (g.totalTime - g.overspray)) / g.totalTime : 0; return [g.id, used, g.totalTime > 0 ? 100 - used : 0, hits, s.coverage]; });
    return ['Spray id<br>Material used (%)<br>Material wasted (%)<br>Particles<br>Coverage (%)', rows];
  }
  Spray_Clear(id_spray: number | 'all' = -1): number { const id = id_spray === -1 || id_spray === 'all' ? 'all' : id_spray; this.spray.clear(id); this.spray.remove(id); return 1; }

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
  /** Command to a loaded plug-in; RoboDK plug-ins (CollisionFreePlanner, ...) have no equivalent: returns 'Plugin X not loaded'. */
  PluginCommand(plugin_name: string, plugin_command = '', value: any = ''): any { const p = this.plugins.get(plugin_name); return p?.command ? p.command(plugin_command, value) : `Plugin ${plugin_name} not loaded`; }
  Plugins(): string[] { return [...this.plugins.keys()]; }

  // ---- files ----
  readFile(path: string): Uint8Array { if (!this.fileSystem) throw new Error(`InputError: no file access here (${path}); use the studio server or drop the file onto the viewport`); return this.fileSystem.readFile(path); }
  writeFile(path: string, data: string | Uint8Array): void {
    if (this.fileSystem) { const dir = path.replace(/[\\/][^\\/]*$/, ''); if (dir && dir !== path) this.fileSystem.mkdirp?.(dir); this.fileSystem.writeFile(path, data); }
    else this.onGeneratedFiles?.([{ name: path.replace(/^.*[\\/]/, ''), content: data }], path.replace(/[\\/][^\\/]*$/, ''));
  }
  /** Load a file through the importers (RoboDK AddFile): meshes, CAD, .robot/.tool/.rdk, .vbstation, robot programs, target CSV. */
  async AddFile(path: string, parent?: RobolinkItem | number | null): Promise<RobolinkItem> {
    const name = String(path).replace(/^.*[\\/]/, '');
    const ext = name.split('.').pop()?.toLowerCase() ?? '';
    const bytes = this.readFile(path);
    const ab = bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) as ArrayBuffer;
    const par = (parent instanceof RobolinkItem ? parent.item : null) ?? this.station;
    const base = name.replace(/\.[^.]+$/, '');
    if (!this.assets) { const { AssetStore } = await import('../scene/assets'); this.assets = new AssetStore(); }
    const assets = this.assets!;
    if (['stl', 'obj', 'dae'].includes(ext)) {
      const id = `file:${path}`;
      const a = assets.registerRaw(id, ext as any, bytes, name);
      const o = new SceneObject(base);
      o.geometry = [{ mesh: id, origin: Array.from(identity()), color: '#a5b1c2' }];
      if (a.mesh) o.bbox = { min: a.mesh.min as any, max: a.mesh.max as any };
      par.addChild(o);
      return new RobolinkItem(this, o);
    }
    if (['step', 'stp', 'iges', 'igs', 'brep'].includes(ext)) {
      const { importCad } = await import('../io/mesh/step');
      const res = await importCad(ab, name);
      const o = new SceneObject(base);
      res.meshes.forEach((m: any, i: number) => { const id = `file:${path}#${i}`; assets.registerMesh(id, m.mesh, m.name); o.geometry.push({ mesh: id, origin: Array.from(identity()), color: m.color ?? '#a5b1c2' }); });
      par.addChild(o);
      return new RobolinkItem(this, o);
    }
    if (ext === 'rdk' || ext === 'robot' || ext === 'tool' || ext === 'sld') {
      const { importRdkBestEffort } = await import('../io/robodk/rdk_container');
      const { station } = await importRdkBestEffort(ab, base, assets);
      if (ext === 'rdk') { station.filePath = path; this.paths.PATH_OPENSTATION = path.replace(/[\\/][^\\/]*$/, ''); this.paths.FILE_OPENSTATION = path; if (this.onStations) this.onStations.setActive(station); this.setStation(station); return new RobolinkItem(this, station); }
      const kids = [...station.children];
      for (const c of kids) par.addChild(c);
      if (ext === 'tool' && par instanceof Robot) { const t = kids.find((k) => k instanceof Tool) as Tool | undefined; if (t) par.setTool(t); }
      return new RobolinkItem(this, kids.find((k) => k instanceof Robot || k instanceof Tool) ?? kids[0] ?? null);
    }
    if (ext === 'vbstation' || ext === 'json') {
      const { loadStation } = await import('../io/station-file');
      const st = loadStation(JSON.parse(new TextDecoder().decode(bytes)), assets);
      st.filePath = path;
      this.paths.PATH_OPENSTATION = path.replace(/[\\/][^\\/]*$/, ''); this.paths.FILE_OPENSTATION = path;
      this.setStation(st);
      return new RobolinkItem(this, st);
    }
    if (['src', 'mod', 'prg', 'ls', 'script', 'urp', 'nc', 'gcode', 'apt', 'jbi', 'pdl', 'csv', 'txt'].includes(ext)) {
      const text = new TextDecoder().decode(bytes);
      const robot = (par instanceof Robot ? par : null) ?? this.station.itemsOfType<Robot>(ItemType.ROBOT)[0] ?? null;
      if ((ext === 'csv' || ext === 'txt') && !/^type,name/m.test(text)) {
        const { importTargets } = await import('../io/robodk/targets');
        const ts = importTargets(text, { parent: robot?.activeFrame() ?? par, namePrefix: base } as any);
        return new RobolinkItem(this, ts[0]?.parent ?? null);
      }
      const { importProgram } = await import('../io/programs/import');
      const r = importProgram(text, { station: this.station, robot, filename: name });
      return new RobolinkItem(this, r.program);
    }
    throw new Error(`InputError: AddFile does not support .${ext} files`);
  }
  AddShape(triangles: number[][] | number[][][], parent?: RobolinkItem | number | null, override_shapes = false, name = 'Shape'): RobolinkItem {
    const par = parent instanceof RobolinkItem ? parent.item : null;
    const o = par instanceof SceneObject ? par : new SceneObject(typeof override_shapes === 'string' ? override_shapes : name);
    if (o === par && override_shapes === true) o.geometry = [];
    const shapes: number[][][] = Array.isArray(triangles[0]?.[0]) ? (triangles as number[][][]) : [triangles as number[][]];
    for (const tri of shapes) {
      const pos = new Float32Array(tri.flatMap((p) => [p[0], p[1], p[2]]));
      const id = `shape_${o.id}_${o.geometry.length}`;
      const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
      for (let i = 0; i < pos.length; i++) { min[i % 3] = Math.min(min[i % 3], pos[i]); max[i % 3] = Math.max(max[i % 3], pos[i]); }
      if (this.assetsRegister) this.assetsRegister(id, pos);
      else { const mesh = { positions: pos, normals: new Float32Array(0), min, max, triangles: pos.length / 9 } as any; if (!this.assets) this.pendingMeshes.set(id, mesh); else this.assets.registerMesh(id, mesh, name); }
      o.geometry.push({ mesh: id, origin: Array.from(identity()), color: '#8ca0b3' });
      o.bbox = { min: min as any, max: max as any };
    }
    if (!o.parent) (par ?? this.station).addChild(o);
    o.notify('geometry');
    return new RobolinkItem(this, o);
  }
  /** Meshes added before an asset store exists (headless without assets). */
  readonly pendingMeshes = new Map<string, any>();
  assetsRegister: ((id: string, positions: Float32Array) => void) | null = null;
  AddCurve(points: number[][] | number[][][], parent?: RobolinkItem | number | null, add_to_ref = false, projection_type = PROJECTION_ALONG_NORMAL_RECALC, multi = false): RobolinkItem {
    const par = parent instanceof RobolinkItem ? parent.item : null;
    const o = (add_to_ref && par instanceof SceneObject ? par : null) ?? new SceneObject('Curve');
    if (!o.parent) ((par instanceof SceneObject && !add_to_ref ? par.parent : par) ?? this.station).addChild(o);
    if (!add_to_ref && par instanceof SceneObject) o.setPoseAbs(par.poseAbs());
    const curves: number[][][] = multi || Array.isArray(points[0]?.[0]) ? (points as number[][][]) : [points as number[][]];
    for (const c of curves) {
      const pts = par instanceof SceneObject && projection_type !== PROJECTION_NONE ? this.ProjectPoints(c, new RobolinkItem(this, par), projection_type) : c;
      o.curves.push({ name: `curve ${o.curves.length + 1}`, points: pts.map((p) => [...p]) });
    }
    o.notify('geometry');
    return new RobolinkItem(this, o);
  }
  AddPoints(points: number[][], parent?: RobolinkItem | number | null, add_to_ref = false, projection_type = PROJECTION_ALONG_NORMAL_RECALC): RobolinkItem {
    const par = parent instanceof RobolinkItem ? parent.item : null;
    const o = (add_to_ref && par instanceof SceneObject ? par : null) ?? new SceneObject('Points');
    if (!o.parent) ((par instanceof SceneObject && !add_to_ref ? par.parent : par) ?? this.station).addChild(o);
    if (!add_to_ref && par instanceof SceneObject) o.setPoseAbs(par.poseAbs());
    const pts = par instanceof SceneObject && projection_type !== PROJECTION_NONE ? this.ProjectPoints(points, new RobolinkItem(this, par), projection_type) : points;
    pts.forEach((p, i) => o.points.push({ name: `p${o.points.length + i + 1}`, point: [...p] }));
    o.notify('geometry');
    return new RobolinkItem(this, o);
  }
  ShowMessage(msg: string, popup = true): void { this.onMessage?.(msg, popup); }
  setRunMode(m = RUNMODE_SIMULATE): void { if (m === RUNMODE_SIMULATE && this.offline.size) this.Finish(); this.runMode = m; }
  RunMode(): number { return this.runMode; }
  setSimulationSpeed(s: number): void { this.simSpeed = s > 0 ? s : 1; this.exec.speed = this.simSpeed; this.onWindow?.('speed', this.simSpeed); }
  SimulationSpeed(): number { return this.simSpeed; }
  /** Save the station (or one item) as studio JSON to `file` (RoboDK Save); returns the JSON when no file is given. */
  Save(file = '', item?: RobolinkItem | number | null): string | void {
    const it = (item instanceof RobolinkItem ? item.item : null) ?? this.station;
    const json = it instanceof Station ? JSON.stringify(this.saveStationFile(it)) : JSON.stringify(it.serialize());
    if (!file) return json;
    this.writeFile(file, json);
    if (it instanceof Station) { it.filePath = file; this.paths.FILE_OPENSTATION = file; this.paths.PATH_OPENSTATION = file.replace(/[\\/][^\\/]*$/, ''); }
  }
  saveStationFile: (s: Station) => any = (s) => ({ format: 'vbstation', version: 1, station: s.serialize() });
  /** Station parameter or RoboDK path (PATH_OPENSTATION, FILE_OPENSTATION, PATH_DESKTOP, PATH_LIBRARY, PATH_ROBODK). null = unknown. */
  getParam(key: string, _str_type = true): any {
    if (/^(PATH_|FILE_)/.test(key)) {
      const fp = this.station.filePath ?? '';
      if (key === 'PATH_OPENSTATION') return this.paths.PATH_OPENSTATION ?? (fp ? fp.replace(/[\\/][^\\/]*$/, '') : this.paths.PATH_DESKTOP ?? '');
      if (key === 'FILE_OPENSTATION') return this.paths.FILE_OPENSTATION ?? fp;
      if (key in this.paths) return this.paths[key];
    }
    const v = this.station.settings[key as keyof typeof this.station.settings] ?? this.station.getParam(key) ?? this.stationSignals.get(key);
    return v === undefined ? null : v;
  }
  setParam(key: string, value: any): string { this.station.setParam(key, value); this.stationSignals.set(key, value); this.exec.rt().signals?.set(key, value); return 'OK'; }
  Command(cmd: string, value: any = '', _skip = false): any { if (Array.isArray(value)) return []; this.station.setParam(`cmd:${cmd}`, value); return 'OK'; }
  Version(): string { return '5.9.4 (VerticalBot Studio, RoboDK API compatible)'; }
  License(): [string, string] { return ['VerticalBot Studio', 'MIT']; }
  Selection(): RobolinkItem[] { return this.station.selection.map((s) => new RobolinkItem(this, s)); }
  setSelection(items: RobolinkItem[] = []): void { this.station.setSelection(items.map((i) => i.item!).filter(Boolean)); }
  Update(): void { this.render(); }
  Disconnect(): void {}
  Connect(): number { return 1; }
  NewLink(): number { return 1; }
  isNewInstance(): boolean { return false; }
  CursorXYZ(_x = -1, _y = -1): [number[], RobolinkItem] { throw new Error('CursorXYZ needs the RoboDK 3D view (mouse position): not available through the studio API'); }
  GetPoints(_feature_type = FEATURE_HOVER_OBJECT_MESH): never { throw new Error('GetPoints(FEATURE_HOVER_*) needs the mouse hovering the RoboDK 3D view: not available through the studio API'); }
  EmbedWindow(..._a: any[]): never { throw new Error('EmbedWindow docks a desktop window into RoboDK: not available in VerticalBot Studio'); }
  /** Project points onto an object: along the point normal (6-vectors) or the object's closest surface along -Z (3-vectors). */
  ProjectPoints(points: number[][], obj?: RobolinkItem | number | null, projection_type = PROJECTION_ALONG_NORMAL_RECALC): number[][] {
    const target = obj instanceof RobolinkItem ? obj.item : null;
    if (projection_type === PROJECTION_NONE) return points.map((p) => [...p]);
    return points.map((p) => {
      const n = p.length >= 6 && Math.hypot(p[3], p[4], p[5]) > 1e-9 ? [p[3], p[4], p[5]].map((v) => v / Math.hypot(p[3], p[4], p[5])) : [0, 0, -1];
      const L = 1e4;
      const a: [number, number, number] = [p[0] - n[0] * L, p[1] - n[1] * L, p[2] - n[2] * L], b: [number, number, number] = [p[0] + n[0] * L, p[1] + n[1] * L, p[2] + n[2] * L];
      const hit = collisionLine(this.station, a, b, { assets: this.assets, only: target ? [target] : undefined } as any);
      if (!hit || (target && hit.item !== target && !target.isAncestorOf(hit.item))) return [...p];
      return [...hit.point, ...p.slice(3)];
    });
  }
  IsInside(a: RobolinkItem, b: RobolinkItem): number { if (!a.item || !b.item) return 0; const ca = collidersOf(a.item)[0]; if (!ca) return 0; const cb = collidersOf(b.item); const c = ca.shape.kind === 'sphere' ? ca.shape.c : ca.shape.kind === 'capsule' ? ca.shape.a : ca.shape.kind === 'obb' ? ca.shape.center : ca.shape.obb.center; return cb.some((s) => s.shape.kind === 'obb' && insideObb(s.shape, c)) ? 1 : 0; }
  Delete(items: RobolinkItem[] | RobolinkItem): void { for (const i of Array.isArray(items) ? items : [items]) if (i?.item) i.Delete(); }
}

export { ItemType, InstructionType, Robot, Program, Target, Frame, Tool, SceneObject, Instruction };
export type { RunHandleLike };

function num(params: string, key: string, def: number): number { const m = params.match(new RegExp(`${key}\\s*[=:]\\s*([-+]?\\d*\\.?\\d+)`, 'i')); return m ? parseFloat(m[1]) : def; }
function transformPointM(m: Mat4, p: number[]): [number, number, number] { return [m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12], m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13], m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14]]; }
function insideObb(o: { center: number[]; axes: number[][]; half: number[] }, p: number[]): boolean { const d = [p[0] - o.center[0], p[1] - o.center[1], p[2] - o.center[2]]; return [0, 1, 2].every((i) => Math.abs(d[0] * o.axes[i][0] + d[1] * o.axes[i][1] + d[2] * o.axes[i][2]) <= o.half[i]); }
function b64(s: string): Uint8Array {
  if ((globalThis as any).Buffer) return Uint8Array.from((globalThis as any).Buffer.from(s, 'base64'));
  const bin = atob(s); const out = new Uint8Array(bin.length); for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i); return out;
}
function eventCode(type: string): number {
  switch (type) { case 'selection': return EVENT_SELECTION_TREE_CHANGED; case 'moved': return EVENT_ITEM_MOVED; case 'robotMoved': return EVENT_ROBOT_MOVED; case 'renamed': return EVENT_ITEM_RENAMED; case 'visibility': return EVENT_ITEM_VISIBILITY; case 'station': return EVENT_STATION_CHANGED; case 'attach': return EVENT_ITEM_CHANGED; default: return EVENT_ITEM_CHANGED; }
}
export { poseFromZ, transformDir };
