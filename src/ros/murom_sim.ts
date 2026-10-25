/**
 * The studio as the simulator backend of the MURKA / MUROM ROS 2 stack (robot-retailer-murka-v2 docs/interfaces.md
 * §1-2): the exact interface the Webots backend (murom_webots: MuromDriver + SupervisorPlugin + webots_ros2 sensor
 * plugins) and the MuJoCo backend (murom_mujoco/sim_node.py) expose, on a station holding a decomposed MUROM robot
 * (Webots import: MobileRobot "murom" with the wheel joints + Robot "murom_left_arm" / "murom_right_arm" /
 * "murom_head"; products = SceneObjects with params.sku / params.slot from murom_world/config).
 *
 *   /clock                       rosgraph_msgs/Clock          every control frame (basicTimeStep 32 ms, 31 Hz)
 *   /odom + /tf odom->base_link  nav_msgs/Odometry            every frame (driver period >= 20 ms), from the start pose
 *   /joint_states                sensor_msgs/JointState       20 Hz, wheels (angle + commanded wheel speed) then
 *                                                             l_shoulder_pitch .. r_elbow, head pan / tilt, wrists
 *   /imu                         sensor_msgs/Imu              every frame, imu_link (world orientation, gyro, accel + g)
 *   /scan                        sensor_msgs/LaserScan        10 Hz, laser_frame, 640 beams over 4.189 rad, left to
 *                                                             right like webots_ros2 Ros2Lidar (angle_min = +fov/2,
 *                                                             negative increment), inf outside [0.15, 8] m, Webots
 *                                                             noise (std = noise x maxRange); ray cast on the scene
 *                                                             geometry except the robot itself (hall + products)
 *   /head_camera/image_color     sensor_msgs/Image bgra8      5 Hz (cameraRate), camera_link_optical
 *   /head_camera/camera_info     sensor_msgs/CameraInfo       K of the unified camera model (fx = W / 2 tan(fov/2))
 *   /head_depth/image            sensor_msgs/Image 32FC1      depth along the optical axis (m), inf outside [0.1, 5] m
 *   /head_depth/camera_info
 *   /murom/ground_truth_pose     geometry_msgs/PoseStamped    every frame, map, same stamp as /odom
 *   /murom/sim/tool_tips         geometry_msgs/PoseArray      every frame, map, [left, right] forearm tips
 *   /murom/sim/objects           murom_msgs/ProductArray      2 Hz, sku / name / location_id / pose (map)
 *   /cmd_vel                     geometry_msgs/Twist   (sub)  diff drive, wheel speed <= 18 rad/s, watchdog 0.6 s
 *   /murom/joint_trajectory      JointTrajectory      (sub)  linear interpolation from the current position, a new
 *                                                             message replaces the active trajectory; Webots servo
 *                                                             (v = clip(P (target - q), +-maxVelocity), P = 10)
 *   /murom/sim/attach, /detach   murom_msgs/srv/AttachObject / DetachObject: magnetic grasp with the supervisor's
 *                                                             rules (palm surface centre within 0.18 m of the top face,
 *                                                             two-hand width check, basket items ride along)
 *   /murom/sim/screenshot, /murom/sim/record  std_srvs/srv/SetBool (screenshot through the host hook; record: refused)
 *
 * Robot TF (base_link .. camera_link_optical, laser_frame, imu_link) comes from robot_state_publisher with
 * murom_description/urdf/murom.urdf, exactly as in the Webots / MuJoCo launch files.
 * Images: the camera render hook (`render`) — WebGL buffers in the browser — or the software rasterizer of
 * src/vision/sensing.ts (Node / headless server), both with the same camera model.
 */
import type { Station, Item } from '../core/items/item';
import { ItemType, SceneObject, Camera as CameraItem } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { MobileRobot } from '../mobile/items';
import { DEG, RAD, Mat4, Vec3, multiply, invert, transl, rotz, transformPoint, poseToQuat } from '../core/math/pose';
import { sceneMeshes, rasterize, type CameraRender, type SceneMesh } from '../vision/sensing';
import { cameraIntrinsics } from '../vision/camera_model';
import type { AssetStore } from '../scene/assets';
import type { RosTransport } from './rosbridge_client';
import { lidarScan2D, mulberry } from './sim_sensors';

export const MUROM_WHEEL_JOINTS = ['left_wheel_joint', 'right_wheel_joint'];
/** Order of the driver's /joint_states after the wheels (murom_driver.py ARM_HEAD_JOINTS; absent joints skipped). */
export const MUROM_ARM_HEAD_JOINTS = [
  'l_shoulder_pitch_joint', 'l_shoulder_roll_joint', 'l_shoulder_yaw_joint', 'l_elbow_joint',
  'r_shoulder_pitch_joint', 'r_shoulder_roll_joint', 'r_shoulder_yaw_joint', 'r_elbow_joint',
  'head_pan_joint', 'head_tilt_joint',
  'l_wrist_roll_joint', 'l_wrist_pitch_joint', 'r_wrist_roll_joint', 'r_wrist_pitch_joint',
];
/** v2 start pose (driver START_POSE: arms in the travel pose of the grasp server). */
export const MUROM_START_POSE_V2: Record<string, number> = { l_shoulder_pitch_joint: 0.3, r_shoulder_pitch_joint: 0.3, l_elbow_joint: -2.2, r_elbow_joint: -2.2 };
export const MUROM = {
  wheelRadius: 0.177, wheelTrack: 0.441, maxWheelSpeed: 18, cmdVelTimeout: 0.6, servoP: 10, controlPeriod: 0.032,
  toolOffset: [0, 0, -0.248] as Vec3, attachMaxDist: 0.18, bothMaxDist: 0.15, bothWidthTol: 0.03, palmSurface: [0.016, -0.054] as [number, number], gripClearance: 0.012,
  lidar: { fov: 4.189, res: 640, minRange: 0.15, maxRange: 8, noise: 0.003, rate: 10 },
  depth: { minRange: 0.1, maxRange: 5 },
  periods: { odom: 0.02, js: 0.05, obj: 0.5, imu: 0.032 },
};

export interface MuromSimOptions {
  /** Camera / depth publishing rate in sim time (Hz), default 5 (murom_mujoco camera_rate); 0 disables images. */
  cameraRate?: number;
  /** Lidar noise: true = the Webots `noise` field of the device (default), false / 0 = exact ranges, number = std (m). */
  lidarNoise?: boolean | number;
  /** Seed of the sensor noise. */
  seed?: number;
  /** Magnet reach (m), default MUROM_ATTACH_MAX_DIST 0.18. */
  attachMaxDist?: number;
  /** Put a v2 robot (with wrists) into the driver's START_POSE (default true). */
  startPose?: boolean;
  /** Mesh assets for the software camera render and the lidar (Node). */
  assets?: AssetStore | null;
  /** Camera buffer renderer (browser: WebGL); returns null to fall back to the software rasterizer. */
  render?: (station: Station, cam: CameraItem, req: { rgb: boolean; near: number; far: number }) => CameraRender | null;
  /** /murom/sim/screenshot handler (host-specific: writes a PNG); default answers success=false. */
  onScreenshot?: () => { success: boolean; message: string };
  log?: (msg: string, level?: 'info' | 'warn' | 'error') => void;
}

/** Sim-time schedule with an exact mean period (murom_mujoco _Every). */
class Every {
  next = 0;
  constructor(public period: number) {}
  due(now: number): boolean {
    if (now + 1e-9 < this.next) return false;
    this.next += this.period;
    if (this.next <= now) this.next = now + this.period;
    return true;
  }
}

interface Servo { name: string; robot: Robot; indep: number; lower: number; upper: number; vmax: number; target: number }
interface Product { item: SceneObject; name: string; sku: string; location: string; size: Vec3; palmRel?: Mat4; bothRel?: [number, number, number, number, number]; basketRel?: [number, number, number, number] }
type Arm = 'left' | 'right';

const stampOf = (t: number) => { const sec = Math.floor(t + 1e-12); return { sec, nanosec: Math.min(999999999, Math.max(0, Math.round((t - sec) * 1e9))) }; };
const quatZ = (yaw: number) => ({ x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) });
const quatOf = (m: Mat4) => { const [w, x, y, z] = poseToQuat(m); return { x, y, z, w }; };
const yawOf = (m: Mat4) => Math.atan2(m[1], m[0]);
const dist3 = (a: ArrayLike<number>, b: ArrayLike<number>) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
const fmt = (v: number) => v.toFixed(3);
const ZERO9 = new Array(9).fill(0);
const ZERO36 = new Array(36).fill(0);

/** The decomposed MUROM robot of a station (Webots import), or null. */
export function findMurom(station: Station): { base: MobileRobot; robots: Robot[] } | null {
  for (const m of station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)) {
    const wheels = (m.params.wheelJoints as string[] | undefined) ?? [];
    if (!MUROM_WHEEL_JOINTS.every((w) => wheels.includes(w))) continue;
    const robots = [...m.walk()].filter((r): r is Robot => r instanceof Robot);
    if (robots.some((r) => r.jointNames().includes('l_elbow_joint')) && robots.some((r) => r.jointNames().includes('head_pan_joint'))) return { base: m, robots };
  }
  return null;
}

export class MuromSimInterface {
  time = 0;
  readonly base: MobileRobot;
  readonly servos: Servo[] = [];
  readonly products = new Map<string, Product>();
  readonly attached: Record<Arm, string | null> = { left: null, right: null };
  attachedBoth: string | null = null;
  /** Wheel angles (rad) and commanded wheel speeds (rad/s). */
  readonly wheel = { left: 0, right: 0, wl: 0, wr: 0 };
  readonly topics = new Set<string>();
  readonly warnings: string[] = [];
  private cmd = { v: 0, w: 0, t: -Infinity };
  private traj: { idx: number[]; points: { t: number; q: number[] }[]; t0: number; start: number[] } | null = null;
  private sched = { odom: new Every(MUROM.periods.odom), js: new Every(MUROM.periods.js), obj: new Every(MUROM.periods.obj), imu: new Every(MUROM.periods.imu), scan: new Every(1 / MUROM.lidar.rate), cam: new Every(0.2) };
  private origin = { x: 0, y: 0, yaw: 0 };
  private lastV = 0;
  private rng: () => number;
  private headCam: CameraItem | null = null;
  private headDepth: CameraItem | null = null;
  private lidar: CameraItem | null = null;
  private started = false;
  private links = new Map<string, { robot: Robot; index: number }>();
  readonly attachMaxDist: number;

  constructor(readonly station: Station, readonly io: RosTransport, readonly opts: MuromSimOptions = {}) {
    const m = findMurom(station);
    if (!m) throw new Error('no MUROM robot in the station (MobileRobot with left/right_wheel_joint and arm / head Robots)');
    this.base = m.base;
    this.rng = mulberry(opts.seed ?? 7);
    const envDist = typeof process !== 'undefined' ? Number((process as { env?: Record<string, string | undefined> }).env?.MUROM_ATTACH_MAX_DIST || NaN) : NaN;
    this.attachMaxDist = opts.attachMaxDist ?? (Number.isFinite(envDist) ? envDist : MUROM.attachMaxDist);
    const byName = new Map<string, Servo>();
    for (const r of m.robots) {
      const lim = r.jointLimits(), vmax = r.jointMaxVelocities();
      r.jointNames().forEach((n, i) => byName.set(n, { name: n, robot: r, indep: i, lower: lim.lower[i] * DEG, upper: lim.upper[i] * DEG, vmax: vmax[i] > 0 ? vmax[i] * DEG : Infinity, target: r.joints()[i] * DEG }));
      // Webots solid names (l_palm) and URDF link names (l_palm_link) both resolve
      r.chain.links.forEach((l, i) => { const e = { robot: r, index: i }; this.links.set(l.name, e); this.links.set(l.name.endsWith('_link') ? l.name.slice(0, -5) : `${l.name}_link`, e); });
    }
    for (const n of MUROM_ARM_HEAD_JOINTS) { const s = byName.get(n); if (s) this.servos.push(s); }
    for (const c of this.base.walk()) {
      if (!(c instanceof CameraItem)) continue;
      if (c.kind === 'lidar2d') this.lidar ??= c;
      else if (c.kind === 'depth') this.headDepth ??= c;
      else if (c.kind === 'rgb') this.headCam ??= c;
    }
    for (const o of station.itemsOfType<SceneObject>(ItemType.OBJECT)) {
      if (typeof o.params.sku !== 'string') continue;
      const size = (o.params.size as number[] | undefined) ?? [100, 100, 100];
      this.products.set(o.name, { item: o, name: o.name, sku: o.params.sku, location: String(o.params.slot ?? 'unknown'), size: [size[0] / 1000, size[1] / 1000, size[2] / 1000] });
    }
    const rate = opts.cameraRate ?? 5;
    this.sched.cam = new Every(rate > 0 ? 1 / rate : 1e12);
  }

  private log(msg: string, level: 'info' | 'warn' | 'error' = 'info') { if (level !== 'info') this.warnings.push(msg); this.opts.log?.(msg, level); }
  private pub(topic: string, type: string, msg: any) { this.topics.add(topic); this.io.publish(topic, type, msg); }

  /** v2 (five-finger hands with wrists) or classic Murom. */
  get isV2(): boolean { return this.servos.some((s) => s.name === 'l_wrist_roll_joint'); }

  start(time = 0): void {
    if (this.started) return;
    this.started = true;
    this.time = time;
    const st = this.base.state;
    this.origin = { x: st.x / 1000, y: st.y / 1000, yaw: st.theta * DEG };
    this.cmd.t = time - 1e9;
    if (this.opts.startPose !== false && this.isV2) {
      const byRobot = new Map<Robot, number[]>();
      for (const s of this.servos) {
        const v = MUROM_START_POSE_V2[s.name] ?? 0;
        s.target = v;
        const q = byRobot.get(s.robot) ?? s.robot.joints(); q[s.indep] = v * RAD; byRobot.set(s.robot, q);
      }
      for (const [r, q] of byRobot) r.setJoints(q);
    }
    this.io.subscribe('/cmd_vel', 'geometry_msgs/msg/Twist', (msg) => this.onCmdVel(msg));
    this.io.subscribe('/murom/joint_trajectory', 'trajectory_msgs/msg/JointTrajectory', (msg) => { const w = this.onTrajectory(msg); if (w) this.log(w, 'warn'); });
    if (this.io.advertiseService) {
      this.io.advertiseService('/murom/sim/attach', 'murom_msgs/srv/AttachObject', (req) => this.attach(String(req?.object_name ?? ''), String(req?.arm ?? '')));
      this.io.advertiseService('/murom/sim/detach', 'murom_msgs/srv/DetachObject', (req) => this.detach(String(req?.object_name ?? ''), !!req?.use_place_pose, req?.place_pose?.position, String(req?.location_id ?? '')));
      this.io.advertiseService('/murom/sim/screenshot', 'std_srvs/srv/SetBool', () => this.opts.onScreenshot?.() ?? { success: false, message: 'Кадр 3D-вида: не поддерживается этим хостом студии (используйте окно студии)' });
      this.io.advertiseService('/murom/sim/record', 'std_srvs/srv/SetBool', (req) => (req?.data ? { success: false, message: 'Запись видео: не поддерживается бэкендом студии (запись — в окне студии)' } : { success: false, message: 'Запись не идёт' }));
    }
  }

  // ------------------------------------------------------------------ commands
  onCmdVel(msg: any): void {
    this.cmd = { v: Number(msg?.linear?.x ?? 0), w: Number(msg?.angular?.z ?? 0), t: this.time };
  }

  /** JointTrajectory (MuromDriver.__on_trajectory): returns a warning for unknown joints (message ignored). */
  onTrajectory(msg: any): string | null {
    const names: string[] = (msg?.joint_names ?? []).map(String);
    const idx = names.map((n) => this.servos.findIndex((s) => s.name === n));
    const unknown = names.filter((_, k) => idx[k] < 0);
    if (unknown.length) return `Неизвестные суставы: ${JSON.stringify(unknown)}`;
    const points = ((msg?.points ?? []) as any[]).map((p) => ({ t: Number(p?.time_from_start?.sec ?? 0) + Number(p?.time_from_start?.nanosec ?? 0) * 1e-9, q: (p?.positions ?? []).map(Number) as number[] }));
    if (!points.length) { this.traj = null; return null; }
    points.sort((a, b) => a.t - b.t);
    this.traj = { idx, points, t0: this.time, start: idx.map((i) => this.servoValue(this.servos[i])) };
    return null;
  }

  servoValue(s: Servo): number { return s.robot.joints()[s.indep] * DEG; }

  // ------------------------------------------------------------------ simulation step
  /** Advance by dt (s) to sim time `time` and publish what is due. */
  step(dt: number, time = this.time + dt): void {
    if (!this.started) this.start(time - dt);
    this.time = time;
    this.drive(dt);
    this.followTrajectory();
    this.servo(dt);
    this.followObjects();
    this.publish();
  }

  private drive(dt: number): void {
    let { v, w } = this.cmd;
    if (this.time - this.cmd.t > MUROM.cmdVelTimeout) { v = 0; w = 0; }
    let wl = (v - (w * MUROM.wheelTrack) / 2) / MUROM.wheelRadius, wr = (v + (w * MUROM.wheelTrack) / 2) / MUROM.wheelRadius;
    const scale = Math.max(Math.abs(wl), Math.abs(wr)) / MUROM.maxWheelSpeed;
    if (scale > 1) { wl /= scale; wr /= scale; }
    this.wheel.wl = wl; this.wheel.wr = wr;
    // velocity-mode wheel motors reach the commanded speed within the step (Webots motor, no slip on the hall floor):
    // exact arc integration of the differential drive, independent of the studio navigation limits of the base
    const vb = ((wl + wr) / 2) * MUROM.wheelRadius, wb = ((wr - wl) * MUROM.wheelRadius) / MUROM.wheelTrack;
    const m = this.base, st = m.state;
    if (vb !== 0 || wb !== 0 || st.v !== 0 || st.omega !== 0) {
      const th = st.theta * DEG, dth = wb * dt;
      const ds = vb * dt, k = Math.abs(dth) > 1e-9 ? (2 * Math.sin(dth / 2)) / dth : 1;
      st.x += ds * k * Math.cos(th + dth / 2) * 1000;
      st.y += ds * k * Math.sin(th + dth / 2) * 1000;
      const t2 = th + dth;
      st.theta = Math.atan2(Math.sin(t2), Math.cos(t2)) * RAD;
      st.v = vb * 1000; st.omega = wb * RAD;
      st.odometer = (st.odometer ?? 0) + Math.abs(ds) * 1000;
      m.syncPoseFromState();
    }
    this.wheel.left += wl * dt;
    this.wheel.right += wr * dt;
  }

  private followTrajectory(): void {
    const tr = this.traj;
    if (!tr) return;
    const t = this.time - tr.t0;
    const last = tr.points[tr.points.length - 1];
    if (t >= last.t) { tr.idx.forEach((i, k) => { if (last.q[k] !== undefined) this.servos[i].target = last.q[k]; }); this.traj = null; return; }
    let pt = 0, pq = tr.start;
    for (const p of tr.points) {
      if (t < p.t) {
        const span = p.t - pt, a = span <= 0 ? 0 : (t - pt) / span;
        tr.idx.forEach((i, k) => { const q1 = p.q[k] ?? pq[k]; this.servos[i].target = pq[k] + a * (q1 - pq[k]); });
        return;
      }
      pt = p.t; pq = p.q;
    }
  }

  /** Webots RotationalMotor position control: v = clip(P (target - q), +-maxVelocity), targets clamped to the limits. */
  private servo(dt: number): void {
    const byRobot = new Map<Robot, number[]>();
    for (const s of this.servos) {
      const tgt = Math.max(s.lower, Math.min(s.upper, s.target));
      const q = this.servoValue(s);
      if (Math.abs(tgt - q) < 1e-12) continue;
      const v = Math.max(-s.vmax, Math.min(s.vmax, MUROM.servoP * (tgt - q)));
      let qn = q + v * dt;
      if ((tgt - q) * (tgt - qn) < 0) qn = tgt;
      const arr = byRobot.get(s.robot) ?? s.robot.joints();
      arr[s.indep] = qn * RAD;
      byRobot.set(s.robot, arr);
    }
    for (const [r, q] of byRobot) r.setJoints(q);
  }

  // ------------------------------------------------------------------ poses (metres, world = map)
  /** World pose (m) of a link of the arm / head chains by name (Webots solid name or URDF `<name>_link`). */
  linkPose(name: string): Mat4 | null {
    const l = this.links.get(name);
    if (!l) return null;
    const P = multiply(l.robot.poseAbs(), l.robot.fk().linkPoses[l.index]);
    const out = Float64Array.from(P) as Mat4; out[12] /= 1000; out[13] /= 1000; out[14] /= 1000;
    return out;
  }
  basePose(): Mat4 { const P = Float64Array.from(this.base.poseAbs()) as Mat4; P[12] /= 1000; P[13] /= 1000; P[14] /= 1000; return P; }
  toolTip(arm: Arm): Vec3 {
    const F = this.linkPose(arm === 'left' ? 'l_forearm' : 'r_forearm');
    return F ? transformPoint(F, MUROM.toolOffset) : [NaN, NaN, NaN];
  }
  /** Grip point: v2 palm surface centre, classic forearm tip (SupervisorPlugin.__grip_point). */
  gripPoint(arm: Arm): { p: Vec3; key: 'palm' | 'tip' } {
    const T = this.isV2 ? this.linkPose(arm === 'left' ? 'l_palm' : 'r_palm') : null;
    if (!T) return { p: this.toolTip(arm), key: 'tip' };
    const m = arm === 'left' ? 1 : -1;
    return { p: transformPoint(T, [0, -m * MUROM.palmSurface[0], MUROM.palmSurface[1]]), key: 'palm' };
  }
  productPose(p: Product): Mat4 { const P = Float64Array.from(p.item.poseAbs()) as Mat4; P[12] /= 1000; P[13] /= 1000; P[14] /= 1000; return P; }
  private setProductPose(p: Product, T: Mat4): void { const P = Float64Array.from(T) as Mat4; P[12] *= 1000; P[13] *= 1000; P[14] *= 1000; p.item.setPoseAbs(P); }
  private robot2d(): { x: number; y: number; z: number; yaw: number } { const B = this.basePose(); return { x: B[12], y: B[13], z: B[14], yaw: yawOf(B) }; }
  private bothBasis(l: Vec3, r: Vec3): { lat: [number, number]; fwd: [number, number] } {
    const lx = r[0] - l[0], ly = r[1] - l[1], n = Math.hypot(lx, ly) || 1;
    const lat: [number, number] = [lx / n, ly / n];
    return { lat, fwd: [-lat[1], lat[0]] };
  }

  // ------------------------------------------------------------------ services (SupervisorPlugin semantics)
  attach(name: string, arm: string): { success: boolean; message: string } {
    if (arm !== 'left' && arm !== 'right' && arm !== 'both') return { success: false, message: `Неизвестная рука: '${arm}'` };
    const obj = this.products.get(name);
    if (!obj) return { success: false, message: `Объект '${name}' не найден` };
    if (arm === 'both') {
      const busy = (['left', 'right'] as Arm[]).filter((a) => this.attached[a] !== null);
      if (busy.length || this.attachedBoth !== null) return { success: false, message: `Руки заняты: ${busy.length ? JSON.stringify(busy) : this.attachedBoth}` };
      const L = this.gripPoint('left'), R = this.gripPoint('right');
      const mid: Vec3 = [(L.p[0] + R.p[0]) / 2, (L.p[1] + R.p[1]) / 2, (L.p[2] + R.p[2]) / 2];
      const T = this.productPose(obj);
      const center: Vec3 = [T[12], T[13], T[14]];
      const dist = dist3(mid, center);
      const { lat, fwd } = this.bothBasis(L.p, R.p);
      if (L.key === 'palm') {
        // box width along the palm line: |lat . R| . size / 2 (rows of R: world axes)
        let width = 0;
        for (let k = 0; k < 3; k++) width += Math.abs(lat[0] * T[k * 4] + lat[1] * T[k * 4 + 1]) * (obj.size[k] / 2);
        width *= 2;
        const sep = Math.hypot(R.p[0] - L.p[0], R.p[1] - L.p[1]);
        if (dist > MUROM.bothMaxDist || Math.abs(sep - width) > MUROM.bothWidthTol) return { success: false, message: `Ладони далеко от ${name}: середина-центр ${fmt(dist)} м, между ладонями ${fmt(sep)} при ширине ${fmt(width)}` };
      } else if (dist > 0.35) return { success: false, message: `Кончики далеко от ${name}: середина-центр ${fmt(dist)} м` };
      const rel = [center[0] - mid[0], center[1] - mid[1], center[2] - mid[2]];
      obj.bothRel = [rel[0] * lat[0] + rel[1] * lat[1], rel[0] * fwd[0] + rel[1] * fwd[1], rel[2], Math.atan2(fwd[1], fwd[0]), yawOf(T)];
      this.attachedBoth = name;
      obj.location = 'carried_both';
      obj.basketRel = undefined;
      return this.done(`${name} захвачен ДВУМЯ руками (mid-dist ${fmt(dist)})`);
    }
    if (this.attached[arm] !== null) return { success: false, message: `Рука ${arm} уже держит ${this.attached[arm]}` };
    const g = this.gripPoint(arm);
    const T = this.productPose(obj);
    const top: Vec3 = [T[12], T[13], T[14] + obj.size[2] / 2];
    const dist = dist3(g.p, top);
    if (!(dist <= this.attachMaxDist)) {
      const what = g.key === 'palm' ? 'Ладонь' : 'Кончик';
      return { success: false, message: `${what} слишком далеко от ${name}: ${fmt(dist)} м (${g.key}=(${fmt(g.p[0])},${fmt(g.p[1])},${fmt(g.p[2])}) top=(${fmt(top[0])},${fmt(top[1])},${fmt(top[2])}))` };
    }
    this.attached[arm] = name;
    obj.location = `carried_${arm}`;
    obj.basketRel = undefined;
    const palm = this.isV2 ? this.linkPose(arm === 'left' ? 'l_palm' : 'r_palm') : null;
    if (palm) obj.palmRel = multiply(invert(palm), T);
    else this.setProductPose(obj, multiply(transl(T[12], T[13], T[14]), rotz(0)));
    return this.done(`${name} захвачен рукой ${arm} (dist ${fmt(dist)})`);
  }

  detach(name: string, usePlacePose: boolean, place?: { x?: number; y?: number; z?: number }, locationId = ''): { success: boolean; message: string } {
    const obj = this.products.get(name);
    if (!obj) return { success: false, message: `Объект '${name}' не найден` };
    let arm: Arm | 'both' | null = null;
    for (const a of ['left', 'right'] as Arm[]) if (this.attached[a] === name) arm = a;
    if (arm === null && this.attachedBoth === name) arm = 'both';
    if (arm === null) return { success: false, message: `${name} не захвачен` };
    if (arm === 'both') { this.attachedBoth = null; obj.bothRel = undefined; }
    else { this.attached[arm] = null; obj.palmRel = undefined; }
    const T0 = this.productPose(obj);
    const wyaw = yawOf(T0);
    let jump = '';
    let wp: Vec3 = [T0[12], T0[13], T0[14]];
    if (usePlacePose) {
      wp = [Number(place?.x ?? 0), Number(place?.y ?? 0), Number(place?.z ?? 0)];
      const tilt = Math.acos(Math.max(-1, Math.min(1, T0[10]))) * RAD;
      jump = `, скачок ${fmt(dist3([T0[12], T0[13], T0[14]], wp))} м ${tilt.toFixed(1)}°`;
      this.setProductPose(obj, multiply(transl(wp[0], wp[1], wp[2]), rotz(wyaw)));
    }
    obj.location = locationId || 'unknown';
    if (obj.location.startsWith('basket')) {
      const r = this.robot2d(), dx = wp[0] - r.x, dy = wp[1] - r.y, c = Math.cos(r.yaw), s = Math.sin(r.yaw);
      obj.basketRel = [c * dx + s * dy, -s * dx + c * dy, wp[2] - r.z, wyaw - r.yaw];
    } else obj.basketRel = undefined;
    return this.done(`${name} отпущен (${obj.location}${jump})`);
  }

  private done(message: string) { this.log(message); return { success: true, message }; }

  /** Carried products follow the palms / tips, two-hand loads the palm midpoint, basket products the robot. */
  private followObjects(): void {
    for (const arm of ['left', 'right'] as Arm[]) {
      const name = this.attached[arm];
      if (!name) continue;
      const obj = this.products.get(name)!;
      if (obj.palmRel) {
        const palm = this.linkPose(arm === 'left' ? 'l_palm' : 'r_palm');
        if (palm) { this.setProductPose(obj, multiply(palm, obj.palmRel)); continue; }
      }
      const tip = this.toolTip(arm);
      this.setProductPose(obj, multiply(transl(tip[0], tip[1], tip[2] - obj.size[2] / 2 - MUROM.gripClearance), rotz(yawOf(this.productPose(obj)))));
    }
    if (this.attachedBoth) {
      const obj = this.products.get(this.attachedBoth)!;
      const [a, b, c, yaw0, boxYaw0] = obj.bothRel!;
      const L = this.gripPoint('left').p, R = this.gripPoint('right').p;
      const mid = [(L[0] + R[0]) / 2, (L[1] + R[1]) / 2, (L[2] + R[2]) / 2];
      const { lat, fwd } = this.bothBasis(L, R);
      const dyaw = Math.atan2(fwd[1], fwd[0]) - yaw0;
      this.setProductPose(obj, multiply(transl(mid[0] + a * lat[0] + b * fwd[0], mid[1] + a * lat[1] + b * fwd[1], mid[2] + c), rotz(boxYaw0 + dyaw)));
    }
    const r = this.robot2d(), cs = Math.cos(r.yaw), sn = Math.sin(r.yaw);
    for (const obj of this.products.values()) {
      const rel = obj.basketRel;
      if (!rel) continue;
      this.setProductPose(obj, multiply(transl(r.x + cs * rel[0] - sn * rel[1], r.y + sn * rel[0] + cs * rel[1], r.z + rel[2]), rotz(r.yaw + rel[3])));
    }
  }

  // ------------------------------------------------------------------ publishing
  /** Odometry in the odom frame (= start pose), metres / rad. The studio base has no wheel slip: exact. */
  odomPose(): { x: number; y: number; yaw: number } {
    const st = this.base.state, s = this.origin;
    const dx = st.x / 1000 - s.x, dy = st.y / 1000 - s.y;
    const yaw = st.theta * DEG - s.yaw;
    return { x: Math.cos(s.yaw) * dx + Math.sin(s.yaw) * dy, y: -Math.sin(s.yaw) * dx + Math.cos(s.yaw) * dy, yaw: Math.atan2(Math.sin(yaw), Math.cos(yaw)) };
  }

  jointStateMsg(stamp = stampOf(this.time)) {
    const name = [...MUROM_WHEEL_JOINTS], position = [this.wheel.left, this.wheel.right], velocity = [this.wheel.wl, this.wheel.wr];
    for (const s of this.servos) { name.push(s.name); position.push(this.servoValue(s)); velocity.push(0); }
    return { header: { stamp, frame_id: '' }, name, position, velocity, effort: [] as number[] };
  }

  private publish(): void {
    const now = this.time, stamp = stampOf(now), s = this.sched;
    this.pub('/clock', 'rosgraph_msgs/msg/Clock', { clock: stamp });
    if (s.odom.due(now)) {
      const o = this.odomPose(), st = this.base.state;
      this.pub('/odom', 'nav_msgs/msg/Odometry', { header: { stamp, frame_id: 'odom' }, child_frame_id: 'base_link', pose: { pose: { position: { x: o.x, y: o.y, z: 0 }, orientation: quatZ(o.yaw) }, covariance: ZERO36 }, twist: { twist: { linear: { x: st.v / 1000, y: 0, z: 0 }, angular: { x: 0, y: 0, z: st.omega * DEG } }, covariance: ZERO36 } });
      this.pub('/tf', 'tf2_msgs/msg/TFMessage', { transforms: [{ header: { stamp, frame_id: 'odom' }, child_frame_id: 'base_link', transform: { translation: { x: o.x, y: o.y, z: 0 }, rotation: quatZ(o.yaw) } }] });
      const B = this.basePose();
      this.pub('/murom/ground_truth_pose', 'geometry_msgs/msg/PoseStamped', { header: { stamp, frame_id: 'map' }, pose: { position: { x: B[12], y: B[13], z: B[14] }, orientation: quatOf(B) } });
      this.pub('/murom/sim/tool_tips', 'geometry_msgs/msg/PoseArray', { header: { stamp, frame_id: 'map' }, poses: (['left', 'right'] as Arm[]).map((a) => { const t = this.toolTip(a); return { position: { x: t[0], y: t[1], z: t[2] }, orientation: { x: 0, y: 0, z: 0, w: 1 } }; }) });
    }
    if (s.js.due(now)) this.pub('/joint_states', 'sensor_msgs/msg/JointState', this.jointStateMsg(stamp));
    if (s.obj.due(now)) this.pub('/murom/sim/objects', 'murom_msgs/msg/ProductArray', this.productArrayMsg(stamp));
    if (s.imu.due(now)) this.pub('/imu', 'sensor_msgs/msg/Imu', this.imuMsg(stamp));
    if (s.scan.due(now)) this.pub('/scan', 'sensor_msgs/msg/LaserScan', this.scanMsg(stamp));
    if (s.cam.due(now)) this.publishCamera(stamp);
  }

  productArrayMsg(stamp = stampOf(this.time)) {
    const products = [...this.products.values()].map((p) => { const T = this.productPose(p); return { sku: p.sku, name: p.name, location_id: p.location, pose: { position: { x: T[12], y: T[13], z: T[14] }, orientation: quatOf(T) } }; });
    return { header: { stamp, frame_id: 'map' }, products };
  }

  imuMsg(stamp = stampOf(this.time)) {
    const st = this.base.state, B = this.basePose();
    const v = st.v / 1000, w = st.omega * DEG;
    const ax = (v - this.lastV) / MUROM.periods.imu; this.lastV = v;
    return { header: { stamp, frame_id: 'imu_link' }, orientation: quatOf(B), orientation_covariance: ZERO9, angular_velocity: { x: 0, y: 0, z: w }, angular_velocity_covariance: ZERO9, linear_acceleration: { x: ax, y: v * w, z: 9.81 }, linear_acceleration_covariance: ZERO9 };
  }

  /** Lidar pose in the world (m) as the ROS laser frame (+X forward, +Y left, +Z up). */
  lidarFrame(): Mat4 {
    // studio sensor items are optical frames (+Z forward, +X right, +Y down)
    const P = this.lidar ? multiply(this.lidar.poseAbs(), LINK_FROM_OPTICAL) : multiply(this.base.poseAbs(), transl(300, 0, 660));
    const out = Float64Array.from(P) as Mat4; out[12] /= 1000; out[13] /= 1000; out[14] /= 1000;
    return out;
  }

  private meshCache: { t: number; meshes: SceneMesh[] } | null = null;
  /** Scene meshes the lidar sees: everything but the robot itself (hall + products, like murom_mujoco LIDAR_GROUPS). */
  lidarMeshes(): SceneMesh[] {
    if (this.meshCache && this.meshCache.t === this.time) return this.meshCache.meshes;
    const own = new Set<Item>(this.base.walk());
    const meshes = sceneMeshes(this.station, { assets: this.opts.assets ?? null, fields: true, exclude: (it) => own.has(it) });
    this.meshCache = { t: this.time, meshes };
    return meshes;
  }

  scanMsg(stamp = stampOf(this.time)) {
    const L = MUROM.lidar;
    const fov = this.lidar ? this.lidar.fov * DEG : L.fov;
    const res = this.lidar?.width ?? L.res;
    const minR = this.lidar ? this.lidar.near / 1000 : L.minRange, maxR = this.lidar ? this.lidar.far / 1000 : L.maxRange;
    const noiseParam = Number(this.lidar?.params.noise ?? L.noise);
    const std = this.opts.lidarNoise === false || this.opts.lidarNoise === 0 ? 0 : typeof this.opts.lidarNoise === 'number' ? this.opts.lidarNoise : noiseParam * maxR;
    const ranges = lidarScan2D(this.lidarMeshes(), this.lidarFrame(), { fov, res, minRange: minR, maxRange: maxR, noiseStd: std, rng: this.rng });
    const scanTime = 1 / L.rate;
    return { header: { stamp, frame_id: 'laser_frame' }, angle_min: fov / 2, angle_max: -fov / 2, angle_increment: -fov / (res - 1), time_increment: scanTime / res, scan_time: scanTime, range_min: minR, range_max: maxR, ranges: Array.from(ranges), intensities: [] as number[] };
  }

  /** CameraInfo of a head sensor (frame camera_link_optical; K of the unified camera model). */
  cameraInfoMsg(cam: CameraItem, stamp = stampOf(this.time)) {
    const K = cameraIntrinsics(cam);
    const cx = K.cx - 0.5, cy = K.cy - 0.5; // ROS: pixel centres at integer coordinates (studio / OpenGL: +0.5)
    return { header: { stamp, frame_id: 'camera_link_optical' }, height: cam.height, width: cam.width, distortion_model: 'plumb_bob', d: [0, 0, 0, 0, 0], k: [K.fx, 0, cx, 0, K.fy, cy, 0, 0, 1], r: [1, 0, 0, 0, 1, 0, 0, 0, 1], p: [K.fx, 0, cx, 0, 0, K.fy, cy, 0, 0, 0, 1, 0], binning_x: 0, binning_y: 0, roi: { x_offset: 0, y_offset: 0, height: 0, width: 0, do_rectify: false } };
  }

  /** Render the head camera (bgra8) and the range finder (32FC1, inf outside [minRange, maxRange]). */
  renderHead(): { bgra?: Uint8Array; depth?: Float32Array; width: number; height: number; source: string } | null {
    const cam = this.headCam ?? this.headDepth;
    if (!cam) return null;
    const dcam = this.headDepth;
    const near = Math.min(cam.near > 0 ? cam.near : 10, 10), far = Math.max(cam.far > 0 ? cam.far : 100000, (dcam?.far ?? 0) + 1);
    const samePose = !dcam || dcam === cam || (dist3(dcam.poseAbs().subarray(12, 15), cam.poseAbs().subarray(12, 15)) < 1e-3 && dcam.width === cam.width && dcam.height === cam.height && Math.abs(dcam.fov - cam.fov) < 1e-9);
    const render = (c: CameraItem, rgb: boolean): CameraRender => {
      const hooked = this.opts.render?.(this.station, c, { rgb, near, far });
      if (hooked) return hooked;
      const meshes = sceneMeshes(this.station, { assets: this.opts.assets ?? null, fields: true, ground: true });
      return rasterize(meshes, cameraIntrinsics(c), c.poseAbs(), near, far, { rgb, background: [180, 186, 194] });
    };
    const out: { bgra?: Uint8Array; depth?: Float32Array; width: number; height: number; source: string } = { width: cam.width, height: cam.height, source: '' };
    if (this.headCam) {
      const r = render(this.headCam, true);
      out.source = r.source;
      if (r.rgba) {
        const n = r.width * r.height, bgra = new Uint8Array(n * 4);
        for (let i = 0; i < n; i++) { bgra[i * 4] = r.rgba[i * 4 + 2]; bgra[i * 4 + 1] = r.rgba[i * 4 + 1]; bgra[i * 4 + 2] = r.rgba[i * 4]; bgra[i * 4 + 3] = 255; }
        out.bgra = bgra;
      }
      if (dcam && samePose) out.depth = depthOf(r, dcam);
    }
    if (dcam && !out.depth) { const r = render(dcam, false); out.depth = depthOf(r, dcam); out.source ||= r.source; }
    return out;
  }

  private publishCamera(stamp: { sec: number; nanosec: number }): void {
    const f = this.renderHead();
    if (!f) return;
    if (f.bgra && this.headCam) {
      this.pub('/head_camera/image_color', 'sensor_msgs/msg/Image', { header: { stamp, frame_id: 'camera_link_optical' }, height: f.height, width: f.width, encoding: 'bgra8', is_bigendian: 0, step: f.width * 4, data: base64(f.bgra) });
      this.pub('/head_camera/camera_info', 'sensor_msgs/msg/CameraInfo', this.cameraInfoMsg(this.headCam, stamp));
    }
    if (f.depth && this.headDepth) {
      const d = this.headDepth;
      this.pub('/head_depth/image', 'sensor_msgs/msg/Image', { header: { stamp, frame_id: 'camera_link_optical' }, height: d.height, width: d.width, encoding: '32FC1', is_bigendian: 0, step: d.width * 4, data: base64(float32LE(f.depth)) });
      this.pub('/head_depth/camera_info', 'sensor_msgs/msg/CameraInfo', this.cameraInfoMsg(d, stamp));
    }
  }
}

/** camera_link-like body frame (+X forward, +Y left, +Z up) from a studio optical frame: optical · R_ol. */
const LINK_FROM_OPTICAL: Mat4 = Float64Array.from([0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1]) as Mat4;

/** Range-finder depth (m) from a render: depth along the optical axis, inf outside [near, far] of the device. */
export function depthOf(r: CameraRender, dev: { near: number; far: number }): Float32Array {
  const lo = dev.near / 1000, hi = dev.far / 1000;
  const out = new Float32Array(r.width * r.height);
  for (let i = 0; i < out.length; i++) { const z = r.depth[i] / 1000; out[i] = z > 0 && z >= lo && z <= hi ? z : Infinity; }
  return out;
}

function float32LE(f: Float32Array): Uint8Array {
  const u8 = new Uint8Array(f.buffer.slice(f.byteOffset, f.byteOffset + f.byteLength));
  if (new Uint8Array(new Uint16Array([1]).buffer)[0] !== 1) for (let i = 0; i < u8.length; i += 4) { const a = u8[i], b = u8[i + 1]; u8[i] = u8[i + 3]; u8[i + 1] = u8[i + 2]; u8[i + 2] = b; u8[i + 3] = a; }
  return u8;
}

export function base64(u8: Uint8Array): string {
  const B = (globalThis as { Buffer?: { from(b: ArrayBufferLike, o: number, l: number): { toString(e: string): string } } }).Buffer;
  if (B) return B.from(u8.buffer, u8.byteOffset, u8.byteLength).toString('base64');
  let s = ''; for (let i = 0; i < u8.length; i += 0x8000) s += String.fromCharCode(...u8.subarray(i, i + 0x8000));
  return btoa(s);
}
