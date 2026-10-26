/**
 * ROS 2 digital twin: the studio as the simulator backend of a ROS 2 stack, over rosbridge (or any publish /
 * subscribe transport). The topic set is the one webots_ros2 drivers expose (and the MURKA robots' interface,
 * docs/interfaces.md §1-2), so a stack written against Webots runs against the studio unchanged:
 *
 *   published  /clock (rosgraph_msgs/Clock, sim time)                                every step
 *              /joint_states (sensor_msgs/JointState, every joint by its URDF name,  20 Hz
 *                             wheels with angle + velocity, mimic joints included)
 *              /robot_description (std_msgs/String, latched)                        once
 *              /tf (odom -> base link of every mobile robot)                        31 Hz
 *              /tf_static (map -> odom when `mapFrame` is set, latched)             once
 *              <ns>/odom (nav_msgs/Odometry from the start pose, base-frame twist)  31 Hz
 *              <ns>/scan (sensor_msgs/LaserScan, sensor-relative angles)           10 Hz (when a 2D LiDAR runs)
 *              <ns>/imu (sensor_msgs/Imu)                                           31 Hz
 *              <ns|/robot>/ground_truth_pose (geometry_msgs/PoseStamped, map)      31 Hz
 *              <ns>/<cam>/image_raw, /camera_info, /depth/image_raw + camera TF    5 Hz (option cameras)
 *   subscribed <ns>/cmd_vel (geometry_msgs/Twist, watchdog 0.6 s like the MURKA driver)
 *              /<robot>/joint_trajectory, /joint_trajectory_controller/joint_trajectory
 *              (trajectory_msgs/JointTrajectory: linear interpolation, a new message replaces the trajectory of
 *              the joints it names, targets clamped to the joint limits)
 *
 * The robot description is the kept URDF of an imported robot (names, ros2_control … unchanged) or the station URDF
 * export with floating mobile bases, so robot_state_publisher reproduces every link from /joint_states.
 * Stations with the MURKA MUROM robot use ./murom_sim.ts instead (the exact murom_webots interface).
 */
import type { Station, Item } from '../core/items/item';
import { ItemType } from '../core/items/item';
import { Robot } from '../core/items/robot';
import type { MobileRobot } from '../mobile/items';
import { integrate } from '../mobile/controller';
import { exportStationURDF, exportDescriptionURDF, urdfTreeRoot, sanitizeName } from '../io/urdf/urdf_export';
import { parseXML, childElements } from '../io/urdf/xml';
import { DEG, RAD, poseToQuat, rotz, transl, mul } from '../core/math/pose';
import { laserScanMsg, scanHeading } from './publishers';
import { Camera as CameraItem } from '../core/items/item';
import { renderCamera } from '../vision/sensing';
import { publishCameraFrame, cameraNamespace } from '../vision/vision_ros';

export interface RosIO {
  publish(topic: string, type: string, msg: any, opts?: { latch?: boolean }): void;
  subscribe(topic: string, type: string, cb: (msg: any) => void): void;
}

export interface RosTwinOptions {
  /** Subscribe /cmd_vel and joint trajectories (default true; false = read-only mirror). */
  commands?: boolean;
  /** Publish /clock (default true). */
  clock?: boolean;
  /** Publish <ns>/scan from the simulated 2D LiDAR (default true). */
  scan?: boolean;
  rates?: { jointStates?: number; odom?: number; scan?: number };
  /** cmd_vel watchdog (s), default 0.6. */
  cmdVelTimeout?: number;
  /** Extra JointTrajectory topics. */
  trajectoryTopics?: string[];
  /** Publish a static `<mapFrame> -> odom` at each mobile robot's start pose (default: none, a localization node does). */
  mapFrame?: string;
  /**
   * Camera items (rgb / depth) published as sensor_msgs/Image (rgb8, or 32FC1 depth in m, NaN = no return) +
   * CameraInfo (render K) + TF <world> -> <cam>_link -> <cam>_optical_frame, on <ns>/<cam>/image_raw, /camera_info,
   * /depth/image_raw (default true in 'sim'; rate default 5 Hz; WebGL buffers in the browser, software render in Node).
   */
  cameras?: boolean | { rate?: number; worldFrame?: string };
}

export interface TwinJoint {
  /** Name on /joint_states (the URDF name, prefixed only when it clashes with another robot of the station). */
  name: string;
  /** The joint's name in its own robot description (what a JointTrajectory for that robot names). */
  local: string;
  /** URDF import root or native robot the joint belongs to (per-robot trajectory topics resolve names inside it). */
  owner?: Item;
  kind: 'arm' | 'mimic' | 'wheel' | 'steer';
  prismatic: boolean;
  robot?: Robot;
  /** Index in robot.chain.joints. */
  index: number;
  /** Index among the robot's independent joints (-1 for mimic joints). */
  indep: number;
  mobile?: MobileRobot;
  /** Wheel side: +1 left, -1 right, 0 centre. */
  side: number;
  angle: number;
}

interface MobileCtx { m: MobileRobot; ns: string; frame: string; odom: string; base: string; start: { x: number; y: number; theta: number }; cmd?: { v: number; vy: number; omega: number; t: number }; lastV: number }

const stampOf = (t: number) => { const sec = Math.floor(t); return { sec, nanosec: Math.min(999999999, Math.round((t - sec) * 1e9)) }; };
const quatZ = (yawRad: number) => ({ x: 0, y: 0, z: Math.sin(yawRad / 2), w: Math.cos(yawRad / 2) });
const normNs = (s: string | undefined) => (s ? `/${s.replace(/^\/+|\/+$/g, '')}` : '');
const ZERO36 = new Array(36).fill(0);
const ZERO9 = new Array(9).fill(0);

/** Robot description for the station + URDF joint names per robot + base link per mobile robot / import root (+ the
 *  joint-name prefix of an import root whose joint names clash with another robot's). */
export function twinDescription(station: Station): { xml: string; jointNames: Map<Robot, string[]>; baseLinks: Map<Item, string>; jointPrefix: Map<Item, string> } {
  const all = [...station.walk()];
  const trees = all.filter((it) => it.params.source === 'urdf' && typeof it.params.robotDescription === 'string' && urdfTreeRoot(it.parent) === null);
  const natives = all.filter((it) => (it instanceof Robot || it.type === ItemType.MOBILE_ROBOT) && !urdfTreeRoot(it));
  if (trees.length === 1 && !natives.length) {
    const t = trees[0];
    const res = exportDescriptionURDF(t, undefined, { extras: false });
    const doc = parseXML(res.urdf);
    const children = new Set(childElements(doc, 'joint').map((j) => childElements(j, 'child')[0]?.attrs.link));
    const root = childElements(doc, 'link').find((l) => !children.has(l.attrs.name))?.attrs.name ?? 'base_link';
    const jointNames = new Map<Robot, string[]>();
    for (const r of t.walk()) if (r instanceof Robot) jointNames.set(r, r.chain.joints.map((j) => j.name));
    return { xml: res.urdf, jointNames, baseLinks: new Map([[t, root]]), jointPrefix: new Map() };
  }
  const res = exportStationURDF(station, undefined, { extras: false, mobileBases: 'floating' });
  return { xml: res.urdf, jointNames: res.jointNames ?? new Map(), baseLinks: res.baseLinks ?? new Map(), jointPrefix: res.jointPrefix ?? new Map() };
}

/** Whether a camera item has a colour stream: rgb cameras, and depth cameras except depth-only devices (a Webots
 *  RangeFinder, a Gazebo `depth` / `depth_camera` sensor, or `params.depthOnly`). RGB-D (wizard rgbd / stereo / ToF,
 *  Gazebo rgbd_camera) publish colour + depth like their drivers. */
export function cameraHasColour(cam: CameraItem): boolean {
  if (cam.kind === 'rgb') return true;
  if (cam.kind !== 'depth') return false;
  const p = cam.params as Record<string, unknown>;
  return !(p.depthOnly === true || p.webotsDevice === 'RangeFinder' || p.sensorType === 'depth' || p.sensorType === 'depth_camera');
}

export class RosTwin {
  time = 0;
  joints: TwinJoint[] = [];
  description = '';
  private mobiles: MobileCtx[] = [];
  private acc = { js: 0, odom: 0, scan: 0, cam: 0 };
  private traj = new Map<string, { t: number; v: number }[]>();
  private prevPos = new Map<string, number>();
  private started = false;
  readonly topics = new Set<string>();

  constructor(readonly station: Station, readonly io: RosIO, readonly opts: RosTwinOptions = {}) {}

  private pub(topic: string, type: string, msg: any, latch = false) { this.topics.add(topic); this.io.publish(topic, type, msg, latch ? { latch: true } : undefined); }

  /** Build the joint table, publish the latched description / static TF and subscribe the command topics. */
  start(time = 0): void {
    this.time = time;
    const d = twinDescription(this.station);
    this.description = d.xml;
    this.joints = [];
    for (const r of this.station.itemsOfType<Robot>(ItemType.ROBOT)) {
      const names = d.jointNames.get(r) ?? r.chain.joints.map((j) => j.name);
      const owner = urdfTreeRoot(r) ?? r;
      let k = 0;
      r.chain.joints.forEach((j, i) => { if (j.type !== 'fixed') this.joints.push({ name: names[i] ?? j.name, local: j.name, owner, kind: j.mimic ? 'mimic' : 'arm', prismatic: j.type === 'prismatic', robot: r, index: i, indep: j.mimic ? -1 : k++, side: 0, angle: 0 }); });
    }
    const mobiles = this.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT);
    this.mobiles = mobiles.map((m) => {
      const ns = normNs(m.rosNamespace);
      const frame = mobiles.length > 1 && ns ? `${ns.slice(1)}/` : '';
      const base = d.baseLinks.get(m) ?? `${frame}base_link`;
      const owner = urdfTreeRoot(m) ?? m;
      const jp = d.jointPrefix.get(owner) ?? '';
      for (const [kind, key] of [['wheel', 'wheelJoints'], ['steer', 'steerJoints']] as const) {
        for (const n of (m.params[key] as string[] | undefined) ?? []) {
          const side = /(^|[_/])(l|left|fl|rl)([_/]|$)|left/i.test(n) ? 1 : /(^|[_/])(r|right|fr|rr)([_/]|$)|right/i.test(n) ? -1 : 0;
          this.joints.push({ name: `${jp}${n}`, local: n, owner, kind, prismatic: false, mobile: m, index: -1, indep: -1, side, angle: 0 });
        }
      }
      return { m, ns, frame, odom: `${frame}odom`, base, start: { x: m.state.x, y: m.state.y, theta: m.state.theta }, lastV: m.state.v };
    });
    this.pub('/robot_description', 'std_msgs/msg/String', { data: this.description }, true);
    if (this.opts.mapFrame) {
      this.pub('/tf_static', 'tf2_msgs/msg/TFMessage', { transforms: this.mobiles.map((c) => ({ header: { stamp: stampOf(this.time), frame_id: this.opts.mapFrame }, child_frame_id: c.odom, transform: { translation: { x: c.start.x / 1000, y: c.start.y / 1000, z: 0 }, rotation: quatZ(c.start.theta * DEG) } })) }, true);
    }
    if (this.opts.commands !== false && !this.started) {
      for (const c of this.mobiles) {
        this.io.subscribe(`${c.ns}/cmd_vel`, 'geometry_msgs/msg/Twist', (msg) => { c.cmd = { v: (msg?.linear?.x ?? 0) * 1000, vy: (msg?.linear?.y ?? 0) * 1000, omega: (msg?.angular?.z ?? 0) * RAD, t: this.time }; });
      }
      const topics = new Map<string, Item | undefined>(['/joint_trajectory_controller/joint_trajectory', ...(this.opts.trajectoryTopics ?? [])].map((t) => [t, undefined]));
      for (const it of this.station.walk()) {
        if (it.params.source === 'urdf' && typeof it.params.robotDescription === 'string') topics.set(`/${sanitizeName(String(it.params.urdfName ?? it.name))}/joint_trajectory`, it);
        else if (it instanceof Robot && !urdfTreeRoot(it)) topics.set(`${normNs(it.connection.rosNamespace) || `/${sanitizeName(it.name)}`}/joint_trajectory`, it);
      }
      for (const [t, owner] of topics) this.io.subscribe(t, 'trajectory_msgs/msg/JointTrajectory', (msg) => this.onTrajectory(msg, owner));
    }
    this.started = true;
  }

  /** Current value of a joint in URDF units (rad / m). */
  value(j: TwinJoint): number {
    if (j.kind === 'wheel') return j.angle;
    if (j.kind === 'steer') return (j.mobile!.state.steer ?? 0) * DEG;
    const r = j.robot!;
    const def = r.chain.joints[j.index];
    let v: number;
    if (j.indep >= 0) v = r.joints()[j.indep];
    else {
      const master = r.chain.joints.filter((x) => x.type !== 'fixed' && !x.mimic).findIndex((x) => x.name === def.mimic!.joint);
      v = def.mimic!.multiplier * (master >= 0 ? r.joints()[master] : 0) + def.mimic!.offset;
    }
    return j.prismatic ? v / 1000 : v * DEG;
  }

  /**
   * Joint commanded by a JointTrajectory name: the /joint_states name, else the robot's own URDF name when it is unique
   * (inside `owner` for a per-robot topic, in the whole station for the shared controller topic).
   */
  resolveJoint(n: string, owner?: Item): TwinJoint | undefined {
    const within = (a: Item | undefined): boolean => { for (let p: Item | null | undefined = a; p; p = p.parent) if (p === owner) return true; return false; };
    const movable = this.joints.filter((x) => x.indep >= 0 && (!owner || within(x.owner)));
    const exact = movable.find((x) => x.name === n);
    if (exact) return exact;
    const local = movable.filter((x) => x.local === n);
    return local.length === 1 ? local[0] : undefined;
  }

  private onTrajectory(msg: any, owner?: Item) {
    const names: string[] = msg?.joint_names ?? [];
    const pts: any[] = msg?.points ?? [];
    names.forEach((n0, i) => {
      const j = this.resolveJoint(n0, owner);
      if (!j) return;
      const n = j.name;
      const seq = [{ t: this.time, v: this.value(j) }];
      for (const p of pts) {
        const v = p?.positions?.[i];
        if (typeof v !== 'number') continue;
        const tf = p.time_from_start ?? {};
        seq.push({ t: this.time + (tf.sec ?? 0) + (tf.nanosec ?? 0) * 1e-9, v });
      }
      if (seq.length > 1) this.traj.set(n, seq); else this.traj.delete(n);
    });
  }

  private applyTrajectories() {
    const byRobot = new Map<Robot, number[]>();
    for (const [n, seq] of this.traj) {
      const j = this.joints.find((x) => x.name === n && x.indep >= 0)!;
      let v = seq[seq.length - 1].v;
      for (let k = 1; k < seq.length; k++) {
        if (this.time <= seq[k].t) { const a = seq[k - 1], b = seq[k]; const s = b.t > a.t ? Math.max(0, Math.min(1, (this.time - a.t) / (b.t - a.t))) : 1; v = a.v + (b.v - a.v) * s; break; }
      }
      if (this.time >= seq[seq.length - 1].t) this.traj.delete(n);
      const r = j.robot!;
      const def = r.chain.joints[j.index];
      const q = byRobot.get(r) ?? r.joints();
      q[j.indep] = Math.max(def.lower, Math.min(def.upper, j.prismatic ? v * 1000 : v * RAD));
      byRobot.set(r, q);
    }
    for (const [r, q] of byRobot) r.setJoints(q);
  }

  /** Advance the twin by dt seconds (commands, wheel odometry) and publish the topics that are due. */
  step(dt: number, simTime?: number): void {
    if (!this.started) this.start(simTime ?? 0);
    this.time = simTime ?? this.time + dt;
    const timeout = this.opts.cmdVelTimeout ?? 0.6;
    for (const c of this.mobiles) {
      if (c.cmd) {
        const live = this.time - c.cmd.t <= timeout;
        integrate(c.m, live ? { v: c.cmd.v, omega: c.cmd.omega, vy: c.cmd.vy } : { v: 0, omega: 0 }, dt);
        c.m.syncPoseFromState();
        if (!live && Math.abs(c.m.state.v) < 1e-3 && Math.abs(c.m.state.omega) < 1e-3) c.cmd = undefined;
      }
      const k = c.m.kin, st = c.m.state;
      const half = k.track / 2;
      for (const j of this.joints) if (j.kind === 'wheel' && j.mobile === c.m) {
        const vw = ['differential', 'tracked', 'skid'].includes(k.drive) ? st.v - j.side * (st.omega * DEG) * half : st.v;
        j.angle += (vw / Math.max(1, k.wheelRadius)) * dt;
      }
    }
    if (this.traj.size) this.applyTrajectories();
    const stamp = stampOf(this.time);
    if (this.opts.clock !== false) this.pub('/clock', 'rosgraph_msgs/msg/Clock', { clock: stamp });
    const rates = this.opts.rates ?? {};
    const due = (key: 'js' | 'odom' | 'scan' | 'cam', hz: number) => { this.acc[key] += dt; if (this.acc[key] + 1e-9 >= 1 / hz) { const span = this.acc[key]; this.acc[key] = 0; return span; } return 0; };
    const jsSpan = due('js', rates.jointStates ?? 20);
    if (jsSpan) this.pub('/joint_states', 'sensor_msgs/msg/JointState', this.jointStateMsg(stamp, jsSpan));
    const odomSpan = due('odom', rates.odom ?? 31);
    if (odomSpan && this.mobiles.length) {
      this.pub('/tf', 'tf2_msgs/msg/TFMessage', { transforms: this.mobiles.map((c) => { const o = this.odomPose(c); return { header: { stamp, frame_id: c.odom }, child_frame_id: c.base, transform: { translation: { x: o.x, y: o.y, z: 0 }, rotation: quatZ(o.yaw) } }; }) });
      for (const c of this.mobiles) {
        const o = this.odomPose(c), st = c.m.state;
        this.pub(`${c.ns}/odom`, 'nav_msgs/msg/Odometry', { header: { stamp, frame_id: c.odom }, child_frame_id: c.base, pose: { pose: { position: { x: o.x, y: o.y, z: 0 }, orientation: quatZ(o.yaw) }, covariance: ZERO36 }, twist: { twist: { linear: { x: st.v / 1000, y: (st.vy ?? 0) / 1000, z: 0 }, angular: { x: 0, y: 0, z: st.omega * DEG } }, covariance: ZERO36 } });
        const ax = (st.v - c.lastV) / 1000 / odomSpan; c.lastV = st.v;
        this.pub(`${c.ns}/imu`, 'sensor_msgs/msg/Imu', { header: { stamp, frame_id: `${c.frame}imu_link` }, orientation: quatZ(st.theta * DEG), orientation_covariance: ZERO9, angular_velocity: { x: 0, y: 0, z: st.omega * DEG }, angular_velocity_covariance: ZERO9, linear_acceleration: { x: ax, y: (st.v / 1000) * st.omega * DEG, z: 9.80665 }, linear_acceleration_covariance: ZERO9 });
        const P = mul(transl(st.x, st.y, 0), rotz(st.theta * DEG));
        const [w, x, y, z] = poseToQuat(P);
        this.pub(`${c.ns || `/${sanitizeName(String(c.m.params.urdfName ?? c.m.name))}`}/ground_truth_pose`, 'geometry_msgs/msg/PoseStamped', { header: { stamp, frame_id: 'map' }, pose: { position: { x: st.x / 1000, y: st.y / 1000, z: 0 }, orientation: { x, y, z, w } } });
      }
    }
    const camOpt = this.opts.cameras ?? (this.opts.commands !== false);
    if (camOpt && due('cam', (typeof camOpt === 'object' ? camOpt.rate : undefined) ?? 5)) this.publishCameras(typeof camOpt === 'object' ? camOpt.worldFrame : undefined);
    if (this.opts.scan !== false && due('scan', rates.scan ?? 10)) {
      for (const c of this.mobiles) {
        const rt = (c.m as unknown as { _nav?: { lastScan?: any } })._nav;
        if (rt?.lastScan) this.pub(`${c.ns}/scan`, 'sensor_msgs/msg/LaserScan', laserScanMsg(rt.lastScan, String(c.m.params.laserFrame ?? `${c.frame}laser_frame`), this.time, scanHeading(c.m, rt as any)));
      }
    }
  }

  /** Render and publish every rgb / depth camera of the station (non-MURKA stations; MURKA: ./murom_sim.ts). A depth
   *  camera is an RGB-D / stereo / ToF sensor with a colour stream too, unless it is a depth-only device (Webots
   *  RangeFinder, Gazebo depth / depth_camera sensor): see cameraHasColour. */
  publishCameras(worldFrame = this.opts.mapFrame ?? 'map'): string[] {
    const topics: string[] = [];
    for (const cam of this.station.itemsOfType<CameraItem>(ItemType.CAMERA)) {
      if (cam.kind !== 'rgb' && cam.kind !== 'depth') continue;
      const r = renderCamera(this.station, cam, { rgb: cameraHasColour(cam), ground: true });
      const ns = cameraNamespace(cam);
      const io = { publish: (t: string, type: string, msg: any) => this.pub(t, type, msg) };
      topics.push(...publishCameraFrame(io, cam, { rgba: r.rgba, depth: cam.kind === 'depth' ? r.depth : undefined, width: r.width, height: r.height, timeS: this.time }, { namespace: ns, worldFrame }));
    }
    return topics;
  }

  /** Pose of the base in the odom frame (odom = start pose), metres / rad. */
  odomPose(c: MobileCtx): { x: number; y: number; yaw: number } {
    const st = c.m.state, s = c.start;
    const dx = st.x - s.x, dy = st.y - s.y, t0 = s.theta * DEG;
    return { x: (Math.cos(t0) * dx + Math.sin(t0) * dy) / 1000, y: (-Math.sin(t0) * dx + Math.cos(t0) * dy) / 1000, yaw: Math.atan2(Math.sin((st.theta - s.theta) * DEG), Math.cos((st.theta - s.theta) * DEG)) };
  }

  jointStateMsg(stamp = stampOf(this.time), span = 0): any {
    const name: string[] = [], position: number[] = [], velocity: number[] = [];
    for (const j of this.joints) {
      const v = this.value(j);
      name.push(j.name); position.push(v);
      const p = this.prevPos.get(j.name);
      velocity.push(p !== undefined && span > 0 ? (v - p) / span : 0);
      this.prevPos.set(j.name, v);
    }
    return { header: { stamp, frame_id: '' }, name, position, velocity, effort: [] };
  }
}
