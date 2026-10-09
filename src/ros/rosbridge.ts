/**
 * Minimal rosbridge (rosbridge_suite websocket) client: publish/subscribe/advertise/call service.
 * Used to mirror the simulation to ROS 2 (joint_states, cmd_vel, poses) and to drive the studio
 * from real robots (joint_states / odom feedback). Topics default to those used in this repository.
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

export interface RosBridgeOptions {
  url: string;
  /** Publish rate (Hz). */
  rate?: number;
  /** Mirror mode: 'publish' pushes sim state to ROS, 'follow' applies incoming joint_states to the sim. */
  mode?: 'publish' | 'follow' | 'both';
}

export class RosBridge {
  ws: WebSocket | null = null;
  connected = false;
  private timer: number | null = null;
  private subs = new Map<string, (msg: any) => void>();
  private advertised = new Set<string>();
  private jointNamesByRobot = new Map<string, string[]>();

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
          const m = JSON.parse(ev.data);
          if (m.op === 'publish') this.subs.get(m.topic)?.(m.msg);
        } catch { /* ignore */ }
      };
    });
  }

  disconnect(): void {
    this.ws?.close();
  }

  private send(o: any) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(o));
  }

  advertise(topic: string, type: string): void {
    if (this.advertised.has(topic)) return;
    this.advertised.add(topic);
    this.send({ op: 'advertise', topic, type });
  }
  publish(topic: string, type: string, msg: any): void {
    this.advertise(topic, type);
    this.send({ op: 'publish', topic, msg });
  }
  subscribe(topic: string, type: string, cb: (msg: any) => void): void {
    this.subs.set(topic, cb);
    this.send({ op: 'subscribe', topic, type });
  }
  callService(service: string, args: any = {}): void {
    this.send({ op: 'call_service', service, args });
  }

  private setup() {
    const mode = this.opts.mode ?? 'both';
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
