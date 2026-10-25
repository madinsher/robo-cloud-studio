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
  const { kind, sim } = createSimInterface(opts.station, io, { murom: opts.murom, twin: opts.twin, assets: opts.assets });
  const dt = opts.step ?? stationStep(opts.station);
  const rtf = opts.rtf ?? 1;
  let t = opts.station.simTime ?? 0;
  sim.start(t);
  let steps = 0, stopped = false;
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
        sim.step(dt, t);
        opts.station.simTime = t;
        steps++;
        if (rtf <= 0) break;
        const ahead = wallRef + ((t - simRef) / rtf) * 1000 - Date.now();
        if (ahead > 0) break;
      }
    } catch (e) {
      log(`step failed: ${(e as Error).stack ?? e}`, 'error');
    }
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
    stop: () => { if (stopped) return; stopped = true; client?.close(); resolveDone(); },
    done,
  };
  return handle;
}
