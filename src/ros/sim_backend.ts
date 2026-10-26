/**
 * Headless ROS 2 simulator backend: runs a station in sim time (fixed control frame, real-time factor `rtf`) and
 * serves it to a ROS 2 graph through rosbridge_server (ws://host:9090). Stations holding a decomposed MUROM robot get
 * the exact MURKA interface (./murom_sim.ts, contract C3); any other station gets the generic webots_ros2-style twin
 * (./twin.ts: /clock, /joint_states, /robot_description, /tf, /odom, /scan, /imu, cameras).
 *
 * Node: `await startSimBackend({ url: 'ws://localhost:9090', station, assets })` (global WebSocket of Node >= 22, or
 * pass `WebSocket` from the `ws` package). The browser uses the same interfaces through RosBridge mode 'sim'.
 */
import type { Station } from '../core/items/item';
import type { AssetStore } from '../scene/assets';
import { RosbridgeClient, type WebSocketCtor, type RosTransport } from './rosbridge_client';
import { MuromSimInterface, findMurom, type MuromSimOptions } from './murom_sim';
import { RosTwin, type RosTwinOptions } from './twin';
import { setSensingAssets } from '../vision/sensing';

export interface SimBackendOptions {
  /** rosbridge websocket URL (default ws://localhost:9090). */
  url?: string;
  station: Station;
  assets?: AssetStore | null;
  /** Real-time factor target (1 = real time, 0 = as fast as possible). */
  rtf?: number;
  /** Control frame (s): the world's basicTimeStep (Webots import) or 0.032. */
  step?: number;
  murom?: MuromSimOptions;
  twin?: RosTwinOptions;
  /** Use an existing transport (tests) instead of connecting to `url`. */
  transport?: RosTransport;
  WebSocket?: WebSocketCtor;
  log?: (msg: string, level?: 'info' | 'warn' | 'error') => void;
  /**
   * Render the camera devices only while their topics have subscribers, like the webots_ros2 devices without
   * <alwaysOn> (default true; needs rosapi_node, started by rosbridge_websocket_launch.xml — without it the cameras
   * publish at their rate all the time).
   */
  cameraOnDemand?: boolean;
  /** Called after every tick (one or more control frames), e.g. to mirror the state to the studio's 3D view. */
  onFrame?: (h: SimBackendHandle) => void;
}

/**
 * Subscriber counts of ROS topics through rosbridge: polls rosapi's /rosapi/subscribers (rosapi_msgs/srv/Subscribers)
 * every `periodMs` for the topics asked about. `has(topics)` is false until the first answer (a device stays off until
 * a subscription is seen, as in webots_ros2) and undefined for good when rosapi is not running (the calls fail three
 * times in a row), so callers fall back to "always on".
 */
export class SubscriberWatch {
  private known = new Map<string, number>();
  private wanted = new Set<string>();
  private failures = 0;
  private timer: ReturnType<typeof setInterval> | null = null;
  available: boolean | null = null;
  constructor(readonly client: { callService(service: string, args?: any, type?: string, timeoutMs?: number): Promise<any> }, readonly periodMs = 1000) {}
  has(topics: string[]): boolean | undefined {
    for (const t of topics) if (!this.wanted.has(t)) { this.wanted.add(t); this.start(); }
    if (this.available === false) return undefined;
    return topics.some((t) => (this.known.get(t) ?? 0) > 0);
  }
  private start(): void {
    if (this.timer || this.available === false) return;
    void this.poll();
    this.timer = setInterval(() => void this.poll(), this.periodMs);
  }
  private inFlight = false;
  async poll(): Promise<void> {
    if (this.inFlight || this.available === false) return;
    this.inFlight = true;
    try {
      for (const t of this.wanted) {
        const r = await this.client.callService('/rosapi/subscribers', { topic: t }, 'rosapi_msgs/srv/Subscribers', Math.max(2000, this.periodMs * 2));
        this.known.set(t, Array.isArray(r?.subscribers) ? r.subscribers.length : 0);
        this.available = true; this.failures = 0;
      }
    } catch {
      // no rosapi (or a busy graph): after a few failures in a row, treat the demand as unknown for good
      if (++this.failures >= 3 && this.available !== true) { this.available = false; this.stop(); }
    } finally { this.inFlight = false; }
  }
  stop(): void { if (this.timer) clearInterval(this.timer); this.timer = null; }
}

/**
 * How far the ROS side lags behind the backend: the backend's own /clock as rosbridge delivers it back (throttled to
 * one message per 100 ms) went through rosbridge's whole queue. The round trip has a baseline (rosbridge, the WSL
 * relay: 0.3-0.4 s measured; a 5 Hz bgra8 + 32FC1 pair adds transient spikes), so congestion is a lag `high` s above
 * the smallest lag of the last `windowS` s of sim time — a queue that keeps growing — and ends below `low` s above it;
 * an unknown lag (no echo yet, or none at all) is not congestion.
 */
export class ClockEcho {
  private echoed = NaN;
  private on = false;
  private samples: Array<[number, number]> = [];
  constructor(client: { subscribe(topic: string, type: string, cb: (m: any) => void, opts?: { throttleRate?: number }): void }, readonly now: () => number, readonly high = 1.0, readonly low = 0.4, readonly windowS = 30) {
    client.subscribe('/clock', 'rosgraph_msgs/msg/Clock', (m) => {
      const c = m?.clock; if (!c) return;
      this.echoed = Number(c.sec) + Number(c.nanosec) * 1e-9;
      const t = this.now();
      this.samples.push([t, t - this.echoed]);
      while (this.samples.length && this.samples[0][0] < t - this.windowS) this.samples.shift();
    }, { throttleRate: 100 });
  }
  lag(): number | undefined { return Number.isFinite(this.echoed) ? this.now() - this.echoed : undefined; }
  /** Smallest round-trip lag of the window (s). */
  baseline(): number | undefined { let m = Infinity; for (const [, l] of this.samples) if (l < m) m = l; return Number.isFinite(m) ? m : undefined; }
  congested(): boolean {
    const l = this.lag(), b = this.baseline();
    if (l === undefined || b === undefined) return (this.on = false);
    if (this.on ? l < b + this.low : l > b + this.high) this.on = !this.on;
    return this.on;
  }
}

export interface SimBackendHandle {
  readonly kind: 'murom' | 'twin';
  readonly sim: MuromSimInterface | RosTwin;
  readonly client: RosbridgeClient | null;
  /** Sim time (s). */
  time(): number;
  /** Measured real-time factor since start. */
  rtf(): number;
  steps(): number;
  /** Wall time spent inside the control frames (ms, cumulative) and the longest frame (ms). */
  busy(): { ms: number; maxMs: number };
  /** Sim seconds the ROS side lags behind (the /clock echo through rosbridge; MURKA backend over rosbridge only). */
  rosLag(): number | undefined;
  stop(): void;
  /** Resolves when stopped. */
  done: Promise<void>;
}

/** Control frame of a station: Webots basicTimeStep when the station came from a .wbt, else 32 ms. */
export function stationStep(station: Station): number {
  const w = (station.settings as Record<string, unknown>).webots as { basicTimeStep?: number } | undefined;
  const ms = Number(w?.basicTimeStep);
  return ms > 0 ? ms / 1000 : 0.032;
}

/** Build the simulator interface for a station on a transport (MURKA when the station holds a MUROM robot). */
export function createSimInterface(station: Station, io: RosTransport, opts: { murom?: MuromSimOptions; twin?: RosTwinOptions; assets?: AssetStore | null } = {}): { kind: 'murom' | 'twin'; sim: MuromSimInterface | RosTwin } {
  if (findMurom(station)) return { kind: 'murom', sim: new MuromSimInterface(station, io, { assets: opts.assets, ...opts.murom }) };
  return { kind: 'twin', sim: new RosTwin(station, io, { ...opts.twin }) };
}

export async function startSimBackend(opts: SimBackendOptions): Promise<SimBackendHandle> {
  const log = opts.log ?? ((m: string, l?: string) => (l === 'error' || l === 'warn' ? console.warn : console.log)(`[sim-backend] ${m}`));
  let client: RosbridgeClient | null = null;
  let io: RosTransport;
  if (opts.transport) io = opts.transport;
  else {
    client = new RosbridgeClient(opts.url ?? 'ws://localhost:9090', { WebSocket: opts.WebSocket, log });
    await client.connect();
    io = client;
  }
  if (opts.assets) setSensingAssets(opts.assets);
  const watch = client && opts.cameraOnDemand !== false && !opts.murom?.sensorDemand ? new SubscriberWatch(client) : null;
  let t = opts.station.simTime ?? 0;
  // the MURKA interface drops camera frames while rosbridge falls behind (its /clock echo lags): the clock stays real time
  const echo = client && findMurom(opts.station) && !opts.murom?.cameraBackpressure ? new ClockEcho(client, () => t) : null;
  const murom: MuromSimOptions = { ...opts.murom, ...(watch ? { sensorDemand: (topics: string[]) => watch.has(topics) } : {}), ...(echo ? { cameraBackpressure: () => echo.congested() } : {}) };
  const { kind, sim } = createSimInterface(opts.station, io, { murom, twin: opts.twin, assets: opts.assets });
  const dt = opts.step ?? stationStep(opts.station);
  const rtf = opts.rtf ?? 1;
  sim.start(t);
  let steps = 0, stopped = false, busyMs = 0, maxMs = 0;
  const wall0 = Date.now(), sim0 = t;
  let wallRef = wall0, simRef = t;
  let resolveDone!: () => void;
  const done = new Promise<void>((r) => { resolveDone = r; });
  log(`${kind === 'murom' ? 'MURKA interface (murom_webots / murom_mujoco topic set)' : 'webots_ros2-style twin'} on ${opts.url ?? 'transport'}, frame ${(dt * 1000).toFixed(0)} ms, rtf ${rtf}`);
  const tick = () => {
    if (stopped) return;
    try {
      // a few frames per tick when behind (bounded), one when on time
      for (let k = 0; k < 4; k++) {
        t += dt;
        const a = performance.now();
        sim.step(dt, t);
        const d = performance.now() - a;
        busyMs += d; if (d > maxMs) maxMs = d;
        opts.station.simTime = t;
        steps++;
        if (rtf <= 0) break;
        const ahead = wallRef + ((t - simRef) / rtf) * 1000 - Date.now();
        if (ahead > 0) break;
      }
    } catch (e) {
      log(`step failed: ${(e as Error).stack ?? e}`, 'error');
    }
    if (opts.onFrame) { try { opts.onFrame(handle); } catch (e) { log(`frame hook failed: ${(e as Error).message}`, 'error'); } }
    if (client && !client.connected) { log('rosbridge connection lost — backend stopped', 'error'); handle.stop(); return; }
    const now = Date.now();
    let delay = 0;
    if (rtf > 0) {
      delay = wallRef + ((t - simRef) / rtf) * 1000 - now;
      if (delay < -500) { wallRef = now; simRef = t; delay = 0; } // behind: continue from here instead of a burst
    }
    setTimeout(tick, Math.max(0, delay));
  };
  setTimeout(tick, 0);
  const handle: SimBackendHandle = {
    kind, sim, client,
    time: () => t,
    rtf: () => { const w = (Date.now() - wall0) / 1000; return w > 0 ? (t - sim0) / w : 0; },
    steps: () => steps,
    busy: () => ({ ms: busyMs, maxMs }),
    rosLag: () => echo?.lag(),
    stop: () => { if (stopped) return; stopped = true; watch?.stop(); client?.close(); resolveDone(); },
    done,
  };
  return handle;
}
