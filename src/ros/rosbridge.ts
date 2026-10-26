/**
 * Minimal rosbridge (rosbridge_suite websocket) client: publish/subscribe/advertise/call service.
 * Modes:
 *  - 'sim': the studio is the simulator backend of a ROS 2 stack. A station holding a decomposed MUROM robot (MURKA
 *    retail store) gets the exact MURKA Webots interface (./murom_sim.ts: driver + supervisor topics, head camera and
 *    range finder, lidar, /murom/sim/attach|detach services). In the browser it runs in a Web Worker on a copy of the
 *    station (./sim_worker.ts: the headless ./sim_backend.ts loop, wall-clock paced with catch-up, software-rendered
 *    camera images), so the 3D view cannot stall it; the page mirrors what it moves (./sim_mirror.ts). Without
 *    workers (Node, ?worker=off) the same loop runs on this thread with WebGL camera buffers when the 3D view registered
 *    its renderer. Any other station gets ./twin.ts RosTwin: /clock, /joint_states (every
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
import { startSimBackend, type SimBackendHandle } from './sim_backend';
import { renderCamera, cameraRendererRegistered } from '../vision/sensing';
import { applySnapshot, stationTransfer, type MirrorSnapshot } from './sim_mirror';
import type { SimWorkerStart } from './sim_worker';
import type { AssetStore } from '../scene/assets';
import type { Station } from '../core/items/item';
import { workerAvailable } from '../scene/worker_client';
import { retailProductCount } from './murka_layout';

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
  /** Run the MURKA backend in a Web Worker (default: whenever the browser can; false = on the page thread). */
  worker?: boolean;
}

/** Progress of the simulator backend ('sim' mode on a MURKA station). */
export interface SimBackendStats { where: 'worker' | 'thread'; kind: 'murom' | 'twin'; time: number; rtf: number; steps: number; sent: number; received?: number; busyMs?: number; maxStepMs?: number; rosLag?: number; camera?: { rendered: number; skipped: number; dropped: number } }

export { retailProductCount } from './murka_layout';

/** The message that starts ./sim_worker.ts on a copy of the station (options that are functions stay behind). */
export function simWorkerStart(station: Station, assets: AssetStore | null, url: string, o: MuromSimOptions = {}): SimWorkerStart {
  return { type: 'start', url, ...stationTransfer(station, assets), rtf: 1, murom: { cameraRate: o.cameraRate, lidarNoise: o.lidarNoise, seed: o.seed, attachMaxDist: o.attachMaxDist, startPose: o.startPose } };
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
    if (this.isMuromBackend()) {
      if (retailProductCount(this.app.station) === 0) this.app.log('MURKA backend: the station has no products (murom_world/config/products.yaml was not in the opened folder), so /murom/sim/objects stays empty — load the retail layout in the Connect dialog or open ros2_ws/src', 'warn');
      if (this.opts.worker !== false && workerAvailable()) {
        return this.connectWorker().catch((e: Error & { workerLoad?: boolean }) => {
          if (!e.workerLoad) throw e;
          this.app.log(`MURKA backend worker unavailable (${e.message}): running on the page thread`, 'warn');
          return this.connectSocket();
        });
      }
    }
    return this.connectSocket();
  }

  private isMuromBackend(): boolean { return (this.opts.mode ?? 'both') === 'sim' && !!findMurom(this.app.station); }

  private connectSocket(): Promise<void> {
    return new Promise((resolve, reject) => {
      const ws = new WebSocket(this.opts.url);
      this.ws = ws;
      ws.onopen = () => {
        this.connected = true;
        this.app.log(`rosbridge connected: ${this.opts.url}`);
        Promise.resolve(this.setup()).then(resolve, reject);
      };
      ws.onerror = () => reject(new Error(`rosbridge connection failed: ${this.opts.url}`));
      ws.onclose = () => { this.connected = false; this.app.log('rosbridge disconnected', 'warn'); if (this.timer) { clearInterval(this.timer); this.timer = null; } this.backend?.stop(); this.backend = null; };
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
    if (this.simWorker) {
      const w = this.simWorker;
      w.postMessage({ type: 'stop' });
      // the worker answers 'closed' after closing its socket; terminate it in any case
      setTimeout(() => w.terminate(), 1000);
      return;
    }
    this.ws?.close();
  }

  /** The Web Worker running the MURKA backend (browser 'sim' mode), or null. */
  simWorker: Worker | null = null;
  /** The backend loop when it runs on this thread. */
  backend: SimBackendHandle | null = null;
  private workerStats: SimBackendStats | null = null;
  private sentCount = 0;
  /** Sim time, real-time factor and message count of the simulator backend ('sim' on a MURKA station). */
  backendStats(): SimBackendStats | null {
    if (this.workerStats) return { ...this.workerStats };
    const b = this.backend;
    return b ? { where: 'thread', kind: b.kind, time: b.time(), rtf: b.rtf(), steps: b.steps(), sent: this.sentCount, busyMs: b.busy().ms, maxStepMs: b.busy().maxMs, camera: b.kind === 'murom' ? { ...(b.sim as MuromSimInterface).cameraStats } : undefined } : null;
  }

  /** Start the MURKA backend in a Web Worker on a copy of the station; resolves when it is connected to rosbridge. */
  private connectWorker(): Promise<void> {
    const station = this.app.station;
    const start = simWorkerStart(station, (this.app as unknown as { assets?: AssetStore }).assets ?? null, this.opts.url, this.opts.murom);
    return new Promise((resolve, reject) => {
      let w: Worker;
      const fail = (message: string, workerLoad: boolean) => { const e = new Error(message) as Error & { workerLoad?: boolean }; e.workerLoad = workerLoad; reject(e); };
      try { w = new Worker(new URL('./sim_worker.ts', import.meta.url), { type: 'module' }); } catch (e) { fail((e as Error).message, true); return; }
      let up = false;
      w.onerror = (ev) => { ev.preventDefault?.(); if (!up) { w.terminate(); fail(ev.message || 'worker script error', true); } else this.app.log(`MURKA backend worker error: ${ev.message}`, 'error'); };
      w.onmessage = (ev: MessageEvent) => {
        const m = ev.data as { type: string; [k: string]: any };
        if (m.type === 'state') {
          if (this.app.station !== station) { this.disconnect(); return; } // another station was opened meanwhile
          applySnapshot(station, m.snap as MirrorSnapshot);
          this.workerStats = { where: 'worker', kind: this.workerStats?.kind ?? 'murom', time: m.time, rtf: m.rtf, steps: m.steps, sent: m.sent, received: m.received, busyMs: m.busyMs, maxStepMs: m.maxStepMs, rosLag: m.rosLag, camera: m.camera };
        } else if (m.type === 'log') this.app.log(m.msg, m.level === 'error' ? 'error' : m.level === 'warn' ? 'warn' : 'info');
        else if (m.type === 'connected') {
          up = true; this.connected = true; this.simWorker = w;
          this.workerStats = { where: 'worker', kind: m.kind, time: station.simTime ?? 0, rtf: 0, steps: 0, sent: 0 };
          this.app.log(`rosbridge connected: ${this.opts.url} (MURKA simulator backend in a Web Worker, ${m.items} items)`);
          resolve();
        } else if (m.type === 'error') { if (!up) { w.terminate(); fail(m.message, false); } else this.app.log(m.message, 'error'); }
        else if (m.type === 'closed') {
          this.connected = false; this.simWorker = null;
          this.app.log('rosbridge disconnected', 'warn');
          w.terminate();
        }
      };
      w.postMessage(start);
    });
  }

  private send(o: any) {
    // non-finite floats (LaserScan inf) as the Infinity / NaN tokens rosbridge's Python json accepts
    if (this.ws && this.ws.readyState === WebSocket.OPEN) { this.ws.send(rosJsonStringify(o)); this.sentCount++; }
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

  private async setup(): Promise<void> {
    const mode = this.opts.mode ?? 'both';
    if (mode === 'sim' && findMurom(this.app.station)) {
      // MURKA backend on this thread: the headless backend loop (control frame = the world's basicTimeStep, sim clock
      // paced by the wall clock with catch-up frames when late); head camera / range finder from the WebGL buffers
      // when the 3D view registered its renderer
      const render = (st: Station, cam: CameraItem, req: { rgb: boolean }) => (cameraRendererRegistered() ? renderCamera(st, cam, { rgb: req.rgb }) : null);
      const log = (m: string, l?: 'info' | 'warn' | 'error') => this.app.log(m, l === 'error' ? 'error' : l === 'warn' ? 'warn' : 'info');
      const assets = (this.app as unknown as { assets?: AssetStore }).assets ?? null;
      this.backend = await startSimBackend({ station: this.app.station, transport: this, rtf: 1, murom: { render, log, ...(assets ? { assets } : {}), ...this.opts.murom }, log });
      this.murom = this.backend.sim as MuromSimInterface;
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
