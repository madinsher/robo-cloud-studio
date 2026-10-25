/**
 * Minimal rosbridge (rosbridge_suite websocket) client: publish/subscribe/advertise/call service.
 * Modes:
 *  - 'sim': the studio is the simulator backend of a ROS 2 stack. A station holding a decomposed MUROM robot (MURKA
 *    retail store) gets the exact MURKA Webots interface (./murom_sim.ts: driver + supervisor topics, head camera and
 *    range finder, lidar, /murom/sim/attach|detach services); any other station gets ./twin.ts RosTwin: /clock, /joint_states (every
 *    joint, URDF names), /robot_description, /tf (odom -> base), <ns>/odom, <ns>/scan (sensor-relative), <ns>/imu,
 *    ground truth; subscribes <ns>/cmd_vel and JointTrajectory topics — the webots_ros2 / MURKA topic set.
 *  - 'publish': mirrors the simulation (the standard state topics of 'sim' without /clock and commands, plus the
 *    repository's cmd_joint_state / tcp_pose / robot_pose / nav / vision topics).
 *  - 'follow': applies incoming /joint_states (by joint name) and /odom to the sim; 'both' = follow + legacy publish.
 */
import { App } from '../app';
import { Robot } from '../core/items/robot';
import { MobileRobot } from '../mobile/items';
import { ItemType } from '../core/items/item';
import { DEG, RAD, poseToQuat, getPos } from '../core/math/pose';
import { publishNavRuntime, publishVisionOutput } from './publishers';
import { visionRuntimeOf } from '../vision/pipeline';
import { getVisionStack } from '../vision/stack';
import { Camera as CameraItem } from '../core/items/item';
import { RosTwin, RosTwinOptions } from './twin';
import { MuromSimInterface, findMurom, type MuromSimOptions } from './murom_sim';
import { rosJsonStringify, rosJsonParse, type RosServiceHandler } from './rosbridge_client';
import { stationStep } from './sim_backend';
import { renderCamera, cameraRendererRegistered } from '../vision/sensing';

export interface RosBridgeOptions {
  url: string;
  /** Publish rate (Hz). */
  rate?: number;
  /** Mirror mode: 'publish' pushes sim state to ROS, 'follow' applies incoming joint_states to the sim, 'sim' = digital twin backend. */
  mode?: 'publish' | 'follow' | 'both' | 'sim';
  /** Digital twin options ('sim' / 'publish'). */
  twin?: RosTwinOptions;
  /** MURKA interface options ('sim' on a station with a MUROM robot). */
  murom?: MuromSimOptions;
}

export class RosBridge {
  ws: WebSocket | null = null;
  connected = false;
  private timer: number | null = null;
  private subs = new Map<string, (msg: any) => void>();
  private advertised = new Set<string>();
  private jointNamesByRobot = new Map<string, string[]>();
  private services = new Map<string, { type: string; handler: RosServiceHandler }>();

  constructor(readonly app: App, readonly opts: RosBridgeOptions) {}

  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      const ws = new WebSocket(this.opts.url);
      this.ws = ws;
      ws.onopen = () => {
        this.connected = true;
        this.app.log(`rosbridge connected: ${this.opts.url}`);
        this.setup();
        resolve();
      };
      ws.onerror = () => reject(new Error(`rosbridge connection failed: ${this.opts.url}`));
      ws.onclose = () => { this.connected = false; this.app.log('rosbridge disconnected', 'warn'); if (this.timer) { clearInterval(this.timer); this.timer = null; } };
      ws.onmessage = (ev) => {
        try {
          const m = rosJsonParse(ev.data);
          if (m.op === 'publish') this.subs.get(m.topic)?.(m.msg);
          else if (m.op === 'call_service' && this.services.has(m.service)) void this.serve(m);
        } catch { /* ignore */ }
      };
    });
  }

  disconnect(): void {
    this.ws?.close();
  }

  private send(o: any) {
    // non-finite floats (LaserScan inf) as the Infinity / NaN tokens rosbridge's Python json accepts
    if (this.ws && this.ws.readyState === WebSocket.OPEN) this.ws.send(rosJsonStringify(o));
  }

  /** Serve a ROS service through rosbridge (advertise_service; calls arrive as call_service ops). */
  advertiseService(service: string, type: string, handler: RosServiceHandler): void {
    this.services.set(service, { type, handler });
    this.send({ op: 'advertise_service', service, type });
  }
  private async serve(m: { id?: string; service: string; args?: any }): Promise<void> {
    const s = this.services.get(m.service)!;
    try { this.send({ op: 'service_response', service: m.service, id: m.id, values: (await s.handler(m.args ?? {})) ?? {}, result: true }); }
    catch (e) { this.app.log(`ROS service ${m.service} failed: ${(e as Error).message}`, 'error'); this.send({ op: 'service_response', service: m.service, id: m.id, values: {}, result: false }); }
  }

  advertise(topic: string, type: string, latch = false): void {
    if (this.advertised.has(topic)) return;
    this.advertised.add(topic);
    // latch -> transient-local publisher in rosbridge_suite (/robot_description, /tf_static)
    this.send(latch ? { op: 'advertise', topic, type, latch: true } : { op: 'advertise', topic, type });
  }
  publish(topic: string, type: string, msg: any, opts?: { latch?: boolean }): void {
    this.advertise(topic, type, !!opts?.latch);
    this.send({ op: 'publish', topic, msg });
  }
  subscribe(topic: string, type: string, cb: (msg: any) => void): void {
    this.subs.set(topic, cb);
    this.send({ op: 'subscribe', topic, type });
  }
  callService(service: string, args: any = {}): void {
    this.send({ op: 'call_service', service, args });
  }

  /** The digital twin ('sim' and 'publish' modes). */
  twin: RosTwin | null = null;
  /** The MURKA simulator interface ('sim' mode on a station with a MUROM robot). */
  murom: MuromSimInterface | null = null;
  private lastTick = 0;

  private setup() {
    const mode = this.opts.mode ?? 'both';
    if (mode === 'sim' && findMurom(this.app.station)) {
      // MURKA backend: control frame = the world's basicTimeStep, own sim clock paced by the wall clock (RTF 1);
      // head camera / range finder from the WebGL buffers when the 3D view registered its renderer
      const dt = stationStep(this.app.station);
      const render = (st: typeof this.app.station, cam: CameraItem, req: { rgb: boolean }) => (cameraRendererRegistered() ? renderCamera(st, cam, { rgb: req.rgb }) : null);
      this.murom = new MuromSimInterface(this.app.station, this, { render, log: (m, l) => this.app.log(m, l === 'error' ? 'error' : l === 'warn' ? 'warn' : 'info'), ...this.opts.murom });
      this.murom.start(this.app.station.simTime ?? 0);
      let t = this.murom.time;
      this.timer = window.setInterval(() => { t += dt; this.murom!.step(dt, t); this.app.station.simTime = t; }, dt * 1000);
      return;
    }
    if (mode === 'sim' || mode === 'publish') {
      this.twin = new RosTwin(this.app.station, this, mode === 'sim' ? { ...this.opts.twin } : { commands: false, clock: false, scan: false, ...this.opts.twin });
      this.twin.start(this.app.simTime ?? 0);
      this.lastTick = Date.now();
    }
    if (mode === 'sim') {
      // the twin owns the timing: step at `rate` Hz on the studio's sim time (wall time when the sim clock stands still)
      const rate = this.opts.rate ?? 50;
      this.timer = window.setInterval(() => {
        const now = Date.now(), dt = (now - this.lastTick) / 1000;
        this.lastTick = now;
        const t = this.app.simTime ?? 0;
        this.twin!.step(dt, t > this.twin!.time ? t : this.twin!.time + dt);
      }, 1000 / rate);
      return;
    }
    if (mode !== 'publish') {
      // follow real robots: joint_states -> sim
      this.subscribe('/joint_states', 'sensor_msgs/msg/JointState', (msg) => {
        for (const r of this.app.station.itemsOfType<Robot>(ItemType.ROBOT)) {
          const names = r.jointNames();
          const q = r.joints();
          let changed = false;
          names.forEach((n, i) => {
            const k = msg.name?.indexOf(n) ?? -1;
            if (k >= 0 && msg.position?.[k] !== undefined) {
              const isPrism = r.chain.joints.filter((j) => j.type !== 'fixed' && !j.mimic)[i].type === 'prismatic';
              q[i] = isPrism ? msg.position[k] * 1000 : msg.position[k] * RAD;
              changed = true;
            }
          });
          if (changed) r.setJoints(q);
        }
      });
      this.subscribe('/odom', 'nav_msgs/msg/Odometry', (msg) => {
        const m = this.app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)[0];
        if (!m) return;
        const p = msg.pose?.pose;
        if (!p) return;
        const q = p.orientation;
        const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        m.setPose2D(p.position.x * 1000, p.position.y * 1000, yaw * RAD);
      });
    }
    if (mode !== 'follow') {
      const rate = this.opts.rate ?? 10;
      this.timer = window.setInterval(() => this.publishState(), 1000 / rate);
    }
  }

  private publishState() {
    const now = Date.now();
    const stamp = { sec: Math.floor(now / 1000), nanosec: (now % 1000) * 1e6 };
    if (this.twin) { const dt = (now - this.lastTick) / 1000; this.lastTick = now; this.twin.step(dt, now / 1000); }
    for (const r of this.app.station.itemsOfType<Robot>(ItemType.ROBOT)) {
      const ns = r.connection.rosNamespace?.replace(/\/$/, '') ?? '';
      const names = r.jointNames();
      const types = r.chain.joints.filter((j) => j.type !== 'fixed' && !j.mimic).map((j) => j.type);
      const position = r.joints().map((v, i) => (types[i] === 'prismatic' ? v / 1000 : v * DEG));
      this.publish(`${ns}/cmd_joint_state`, 'sensor_msgs/msg/JointState', { header: { stamp, frame_id: r.name }, name: names, position, velocity: [], effort: [] });
      const tcp = r.solveFK();
      const p = getPos(tcp);
      this.publish(`${ns}/cmd_point`, 'std_msgs/msg/Float32MultiArray', { layout: { dim: [], data_offset: 0 }, data: [p[0] / 1000, p[1] / 1000, p[2] / 1000] });
      this.publish(`${ns}/tcp_pose`, 'geometry_msgs/msg/PoseStamped', { header: { stamp, frame_id: 'base_link' }, pose: poseMsg(tcp) });
    }
    for (const m of this.app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)) {
      const ns = m.rosNamespace.replace(/\/$/, '');
      this.publish(`${ns}/cmd_vel`, 'geometry_msgs/msg/Twist', { linear: { x: m.state.v / 1000, y: 0, z: 0 }, angular: { x: 0, y: 0, z: m.state.omega * DEG } });
      this.publish(`${ns}/robot_pose`, 'geometry_msgs/msg/Pose', poseMsg(m.poseAbs()));
      this.publish(`${ns}/battery_state`, 'sensor_msgs/msg/BatteryState', { percentage: m.batteryLevel(), voltage: 48, present: true });
      // navigation stack simulation: localization estimate, ground truth, LiDAR scan, SLAM map
      const rt = (m as any)._nav;
      if (rt) { this.mapCounter = (this.mapCounter + 1) % 20; publishNavRuntime(this, m, rt, { map: this.mapCounter === 0 }); }
    }
    // machine vision: publish each new pipeline output of cameras whose stack has "publish to ROS 2" enabled
    for (const cam of this.app.station.itemsOfType<CameraItem>(ItemType.CAMERA)) {
      const cfg = getVisionStack(cam); const vrt = visionRuntimeOf(cam);
      if (!cfg?.publishRos || !vrt?.last || this.lastVision.get(cam) === vrt.last) continue;
      this.lastVision.set(cam, vrt.last);
      const parent = cam.parent instanceof MobileRobot ? cam.parent : cam.parent?.parent instanceof MobileRobot ? cam.parent.parent : null;
      publishVisionOutput(this, cam, vrt.last, { namespace: parent?.rosNamespace ?? '' });
    }
  }
  private mapCounter = 0;
  private lastVision = new WeakMap<CameraItem, unknown>();
}

function poseMsg(m: Float64Array) {
  const [w, x, y, z] = poseToQuat(m);
  return { position: { x: m[12] / 1000, y: m[13] / 1000, z: m[14] / 1000 }, orientation: { x, y, z, w } };
}
