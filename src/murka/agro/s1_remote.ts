/**
 * Benchmark S1 off the page thread (V3 F3: the synchronous run blocked the app for ~20 s).
 *
 *  - `s1Payload(station, assets)`: what a worker needs — the serialized item tree and the meshes it references (the
 *    crown profile is measured on the tree mesh, contacts use the colliders); structured-clone safe, nothing is encoded.
 *  - `runS1OnPayload(payload, opts)`: rebuilds the station from the payload and runs `runS1Benchmark` on it; returns the
 *    result as plain data (`S1ResultData`: metrics, events, traces, the timeline's legs) and the download JSON text, so
 *    the page does not stringify megabytes either. The rebuilt station is cached while the tree is unchanged (S1 plans
 *    are cached per station).
 *  - `s1ResultFromData(station, data)`: the full `S1Result` on the app's own station (timeline / eventsJsonl rebuilt).
 *  - `runS1InWorker(station, assets, opts, hooks)`: the module worker `s1_worker.ts`; progress (mission / contact replay
 *    time) and the event log lines arrive while it computes. `s1WorkerAvailable()` is false in Node / jsdom and with
 *    `?worker=off`, where the caller runs `runS1Benchmark` directly.
 */
import type { Station, SerializedItem } from '../../core/items/item';
import { AssetStore, Asset } from '../../scene/assets';
import { loadStation, referencedMeshIds } from '../../io/station-file';
import type { MeshData } from '../../io/mesh/stl';
import { runS1Benchmark, s1Timeline, S1Options, S1Result, S1TimelineData } from './s1';

export interface S1Payload {
  tree: SerializedItem;
  meshes: Array<{ id: string; type: Asset['type']; name?: string; units?: 'mm' | 'm'; mesh: MeshData }>;
}

/** Options that cross the worker boundary (no callbacks, no station objects). */
export type S1RemoteOptions = Omit<S1Options, 'assets' | 'world' | 'log' | 'progress'>;

export type S1ResultData = Omit<S1Result, 'timeline' | 'eventsJsonl'> & { timelineData: S1TimelineData };

/** Serialized station + referenced meshes for a worker run. */
export function s1Payload(station: Station, assets?: AssetStore): S1Payload {
  const tree = station.serialize();
  const meshes: S1Payload['meshes'] = [];
  if (assets) for (const ref of referencedMeshIds(tree)) {
    const a = assets.resolve(ref);
    if (a?.mesh) meshes.push({ id: ref, type: a.type, ...(a.name ? { name: a.name } : {}), ...(a.units ? { units: a.units } : {}), mesh: a.mesh });
  }
  return { tree, meshes };
}

let cached: { key: string; station: Station; assets: AssetStore } | null = null;

/** Station + asset store of a payload (cached while the serialized tree and mesh list are unchanged). */
export function stationFromPayload(p: S1Payload): { station: Station; assets: AssetStore } {
  const key = JSON.stringify(p.tree) + '|' + p.meshes.map((m) => `${m.id}:${m.mesh.positions?.length ?? 0}`).join(',');
  if (cached && cached.key === key) return cached;
  const assets = new AssetStore();
  for (const m of p.meshes) assets.registerMesh(m.id, m.mesh, m.name, m.units);
  const station = loadStation(p.tree);
  cached = { key, station, assets };
  return cached;
}

/** The download JSON of a run (what the app saved before: the result without functions; the timeline as end + agents). */
export function s1DownloadJson(r: S1ResultData | S1Result): string {
  const { timelineData, ...rest } = r as S1ResultData & Partial<Pick<S1Result, 'timeline' | 'eventsJsonl'>>;
  delete rest.timeline; delete rest.eventsJsonl;
  return JSON.stringify({ ...rest, timeline: { end: timelineData.end, agents: timelineData.agents.map((a) => a.name) } }, null, 1);
}

/** Run S1 on the station of a payload; plain-data result + its download JSON. */
export function runS1OnPayload(p: S1Payload, opts: S1RemoteOptions, hooks: { log?: (m: string) => void; progress?: S1Options['progress'] } = {}): { data: S1ResultData; json: string } {
  const { station, assets } = stationFromPayload(p);
  const r = runS1Benchmark(station, { ...opts, assets, log: hooks.log, progress: hooks.progress });
  const { timeline: _t, eventsJsonl: _e, ...data } = r;
  void _t; void _e;
  return { data, json: s1DownloadJson(data) };
}

/** The full result on `station` (the app's): timeline and events.jsonl rebuilt from the plain data. */
export function s1ResultFromData(station: Station, data: S1ResultData): S1Result {
  const events = data.events;
  return { ...data, timeline: s1Timeline(station, data.timelineData), eventsJsonl: () => events.map((e) => JSON.stringify(e)).join('\n') + '\n' };
}

/** True in a browser that can run module workers (and unless disabled with ?worker=off). */
export function s1WorkerAvailable(): boolean {
  if (typeof Worker === 'undefined' || typeof window === 'undefined') return false;
  try { if (new URLSearchParams(location.search).get('worker') === 'off') return false; } catch { /* no location */ }
  return !(globalThis as { __VBS_NO_WORKER?: boolean }).__VBS_NO_WORKER;
}

let worker: Worker | null = null;
let seq = 0;

/** Stop the S1 worker (a run in progress is rejected). */
export function cancelS1Worker(reason = 'cancelled'): void {
  worker?.terminate();
  worker = null;
  for (const [, p] of pending) p.reject(new Error(reason));
  pending.clear();
}
const pending = new Map<number, { resolve: (v: { data: S1ResultData; json: string }) => void; reject: (e: Error) => void; onProgress?: (p: { phase: string; t: number; end: number }) => void; onLines?: (l: string[]) => void }>();

/**
 * Run S1 in the module worker on a copy of `station`; the result's timeline drives the robots of `station`.
 * `onProgress` gets the computation's progress, `onLines` the event log lines (batched every ~200 ms).
 */
export async function runS1InWorker(station: Station, assets: AssetStore | undefined, opts: S1RemoteOptions, hooks: { onProgress?: (p: { phase: string; t: number; end: number }) => void; onLines?: (lines: string[]) => void } = {}): Promise<{ result: S1Result; json: string }> {
  const payload = s1Payload(station, assets);
  if (!worker) {
    worker = new Worker(new URL('./s1_worker.ts', import.meta.url), { type: 'module' });
    worker.onmessage = (e: MessageEvent) => {
      const { id, ok, result, error, progress, lines } = e.data as { id: number; ok?: boolean; result?: { data: S1ResultData; json: string }; error?: string; progress?: { phase: string; t: number; end: number }; lines?: string[] };
      const p = pending.get(id);
      if (!p) return;
      if (lines?.length) p.onLines?.(lines);
      if (progress) { p.onProgress?.(progress); return; }
      if (ok === undefined) return;
      pending.delete(id);
      if (ok && result) p.resolve(result); else p.reject(new Error(error ?? 'S1 worker failed'));
    };
    worker.onerror = (e) => cancelS1Worker(e.message || 'S1 worker error');
  }
  const id = ++seq;
  const out = await new Promise<{ data: S1ResultData; json: string }>((resolve, reject) => {
    pending.set(id, { resolve, reject, onProgress: hooks.onProgress, onLines: hooks.onLines });
    worker!.postMessage({ id, payload, opts });
  });
  return { result: s1ResultFromData(station, out.data), json: out.json };
}
