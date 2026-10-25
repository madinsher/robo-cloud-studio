/**
 * Web Worker host of the simulator backend in the browser (Connect › ROS 2 › Simulator backend on a MURKA station).
 * The worker gets a copy of the station (serialised item tree + the meshes it references), connects to rosbridge
 * itself and runs ./sim_backend.ts exactly as the headless CLI does: fixed control frames paced to the wall clock
 * (RTF 1, catch-up when late), lidar ray casts and head camera / range finder images from the software rasterizer of
 * src/vision/sensing.ts. The page's 3D view and UI therefore cannot stall the simulation, and a background tab does
 * not throttle it. What the backend moves (base, joints, products) goes back to the page as ./sim_mirror.ts snapshots.
 *
 * Protocol (main -> worker): {type:'start', url, tree, meshes, rtf?, step?, murom?, snapshotHz?}, {type:'stop'}.
 * Worker -> main: {type:'connected', kind, items}, {type:'error', message}, {type:'state', snap, rtf, steps, time,
 * sent}, {type:'log', msg, level}, {type:'closed'}.
 */
import { AssetStore } from '../scene/assets';
import { loadStation } from '../io/station-file';
import { startSimBackend, type SimBackendHandle } from './sim_backend';
import type { MuromSimOptions } from './murom_sim';
import { StateMirror, type StationTransfer } from './sim_mirror';
import type { MuromSimInterface } from './murom_sim';

export interface SimWorkerStart extends StationTransfer {
  type: 'start';
  url: string;
  rtf?: number;
  step?: number;
  murom?: Pick<MuromSimOptions, 'cameraRate' | 'lidarNoise' | 'seed' | 'attachMaxDist' | 'startPose'>;
  /** State snapshots to the page per wall second (default 30). */
  snapshotHz?: number;
}

const ctx = self as unknown as { postMessage(m: unknown): void; onmessage: ((e: MessageEvent) => void) | null };
let handle: SimBackendHandle | null = null;

ctx.onmessage = async (e: MessageEvent) => {
  const m = e.data as SimWorkerStart | { type: 'stop' };
  if (m.type === 'stop') { handle?.stop(); return; }
  if (m.type !== 'start' || handle) return;
  const log = (msg: string, level: 'info' | 'warn' | 'error' = 'info') => ctx.postMessage({ type: 'log', msg, level });
  try {
    const assets = new AssetStore();
    for (const a of m.meshes) assets.registerMesh(a.id, a.mesh, a.name, a.units);
    const station = loadStation(m.tree);
    const mirror = new StateMirror(station);
    const period = 1000 / (m.snapshotHz ?? 30);
    let last = -Infinity;
    handle = await startSimBackend({
      url: m.url, station, assets, rtf: m.rtf ?? 1, step: m.step,
      murom: { ...m.murom, log },
      log,
      onFrame: (h) => {
        const now = performance.now();
        if (now - last < period) return;
        last = now;
        ctx.postMessage({ type: 'state', snap: mirror.snapshot(h.time()), rtf: h.rtf(), steps: h.steps(), time: h.time(), sent: h.client?.stats.sent ?? 0, received: h.client?.stats.received ?? 0, busyMs: h.busy().ms, maxStepMs: h.busy().maxMs, rosLag: h.rosLag(), camera: h.kind === 'murom' ? { ...(h.sim as MuromSimInterface).cameraStats } : undefined });
      },
    });
    let items = 0; for (const _ of station.walk()) items++;
    ctx.postMessage({ type: 'connected', kind: handle.kind, items });
    void handle.done.then(() => { ctx.postMessage({ type: 'state', snap: mirror.snapshot(handle!.time()), rtf: handle!.rtf(), steps: handle!.steps(), time: handle!.time(), sent: handle!.client?.stats.sent ?? 0 }); ctx.postMessage({ type: 'closed' }); });
  } catch (err) {
    ctx.postMessage({ type: 'error', message: (err as Error).message ?? String(err) });
  }
};
