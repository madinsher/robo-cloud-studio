/**
 * Node host for the Robolink API (studio server, headless tests, scripted Library runs): a Robolink on a station with
 * Node file access, RoboDK path parameters, a realtime station clock, the CPython station-script runner (I5) and the
 * WebSocket/TCP API server (api_server.ts). server/index.ts is a thin wrapper around createNodeHost().
 */
import { readFileSync, writeFileSync, existsSync, mkdirSync } from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import type http from 'node:http';
import { Station } from '../core/items/item';
import { StationRuntime } from '../core/motion/runtime';
import { AssetStore } from '../scene/assets';
import { Robolink, ApiFileSystem } from './robolink';
import { startApiServer, ApiServer } from './api_server';
import { createCPythonRunner, CPythonRunner } from './runner_cpython';
import type { RpcRequest, RpcResponse } from './rpc';
import type { StationRuntimeLike } from './exec';

export function nodeFileSystem(): ApiFileSystem {
  return {
    readFile: (p) => new Uint8Array(readFileSync(p)),
    writeFile: (p, data) => { const dir = path.dirname(p); if (dir && !existsSync(dir)) mkdirSync(dir, { recursive: true }); writeFileSync(p, data); },
    exists: (p) => existsSync(p),
    mkdirp: (d) => { if (d && !existsSync(d)) mkdirSync(d, { recursive: true }); },
  };
}

/** RoboDK path parameters on this machine (PATH_DESKTOP, PATH_LIBRARY, PATH_ROBODK). */
export function nodePaths(): Record<string, string> {
  const home = os.homedir();
  const desktop = [path.join(home, 'Desktop'), path.join(home, 'OneDrive', 'Desktop'), home].find((p) => existsSync(p)) ?? home;
  const robodk = process.env.ROBODK_PATH ?? (process.platform === 'win32' ? 'C:/RoboDK' : path.join(home, 'RoboDK'));
  const out: Record<string, string> = { PATH_DESKTOP: desktop.replace(/\\/g, '/') };
  if (existsSync(robodk)) out.PATH_ROBODK = robodk.replace(/\\/g, '/');
  const lib = process.env.ROBODK_LIBRARY ?? path.join(robodk, 'Library');
  out.PATH_PROGRAMS = path.join(home, 'Documents', 'RoboDK').replace(/\\/g, '/');   // RoboDK's default programs folder
  out.PATH_LIBRARY = (existsSync(lib) ? lib : path.join(process.cwd(), 'public', 'library')).replace(/\\/g, '/');
  return out;
}

export interface NodeHostOptions {
  station?: Station;
  /** Asset store holding the station's meshes (an imported .rdk); default: a new empty store. */
  assets?: AssetStore;
  /** Attach to an HTTP server (WebSocket upgrade) or listen on `port` (0 = ephemeral). */
  server?: http.Server;
  port?: number;
  /** TCP JSON-lines port (0 = ephemeral, -1 = disabled). */
  tcpPort?: number;
  mode?: 'realtime' | 'virtual';
  /** Simulation speed factor of the realtime clock. */
  speed?: number;
  python?: string;
  scriptTimeoutS?: number;
  /** Station scripts run with the headless tkinter (unattended runs; default STUDIO_HEADLESS_GUI=1). */
  headlessGui?: boolean;
  /** Station runtime factory (default: the station's shared StationRuntime, contract I1); null = the built-in fallback player. */
  runtimeFactory?: ((station: Station) => StationRuntimeLike) | null;
  special?: (req: RpcRequest) => Promise<RpcResponse> | undefined;
  afterHeadless?: (req: RpcRequest) => void;
  log?: (msg: string) => void;
}

export interface NodeHost {
  rdk: Robolink;
  api: ApiServer;
  runner: CPythonRunner;
  url: string;
  port: number;
  tcpPort: number;
  station(): Station;
  setStation(s: Station, assets?: AssetStore): void;
  close(): Promise<void>;
}

export async function createNodeHost(opts: NodeHostOptions = {}): Promise<NodeHost> {
  const log = opts.log ?? ((m: string) => console.log(`[studio-server] ${m}`));
  const rdk = new Robolink(opts.station ?? new Station('Headless station'));
  rdk.assets = opts.assets ?? new AssetStore();
  rdk.setStation(rdk.station);           // spray / collision helpers pick up the asset store
  rdk.fileSystem = nodeFileSystem();
  rdk.paths = nodePaths();
  rdk.onMessage = (m) => log(`message: ${m}`);
  rdk.exec.mode = opts.mode ?? 'realtime';
  const speed = opts.speed ?? Number(process.env.STUDIO_SIM_SPEED ?? 1);
  rdk.setSimulationSpeed(speed > 0 ? speed : 1);
  // the station's shared StationRuntime (I1): API moves, RunProgram, program -> script calls and signals share one clock
  const factory = opts.runtimeFactory === undefined ? (s: Station) => StationRuntime.of(s) as unknown as StationRuntimeLike : opts.runtimeFactory;
  const attachRuntime = (s: Station) => {
    if (!factory) return;
    const rt: any = factory(s);
    if ('assets' in rt && !rt.assets) rt.assets = rdk.assets;   // mesh bounds for AttachClosest distances
    rdk.exec.attach(rt, { mode: rdk.exec.mode });
  };
  attachRuntime(rdk.station);
  rdk.onStationReplaced = (s) => { attachRuntime(s); };
  let port = 0;
  const runner = createCPythonRunner({ url: () => `ws://127.0.0.1:${port}`, python: opts.python, timeoutS: opts.scriptTimeoutS, headlessGui: opts.headlessGui, onOutput: (name, line, stream) => log(`${name}${stream === 'stderr' ? ' !' : ':'} ${line}`) });
  rdk.exec.setScriptRunner(runner);
  const api = startApiServer({ server: opts.server, port: opts.server ? undefined : opts.port ?? 0, tcpPort: opts.tcpPort ?? -1, getRDK: () => rdk, special: opts.special, afterHeadless: opts.afterHeadless, log });
  const r = await api.ready;
  port = r.port;
  return {
    rdk, api, runner, url: `ws://127.0.0.1:${port}`, port, tcpPort: r.tcpPort,
    station: () => rdk.station,
    setStation: (s, assets) => { rdk.exec.stop(); if (assets) rdk.assets = assets; rdk.setStation(s); },
    close: async () => { runner.stopAll(); rdk.exec.dispose(); await api.close(); },
  };
}
