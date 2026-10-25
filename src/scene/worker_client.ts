/** Main-thread side of the studio worker (see worker.ts). Falls back to `workerAvailable() === false` in node/jsdom. */
import type { MeshData } from '../io/mesh/stl';
import type { SerializedItem } from '../core/items/item';
import type { ProgramRunResult } from '../core/items/program';
import type { StationFile } from '../io/station-file';
import type { Asset } from './assets';

export interface ImportProgress { detail: string }
export interface WorkerImport {
  tree: SerializedItem;
  report: any;
  meshes: Array<{ id: string; name?: string; units?: 'mm' | 'm'; type: Asset['type']; mesh: MeshData; source?: Uint8Array }>;
  itemCount: number;
}
export interface WorkerCompile { result: ProgramRunResult; steps: Array<[string, number, number, number]>; points: Float32Array }

type Kind = 'import' | 'compile';
/** One worker per job kind: a long background compile never delays an import, and can be cancelled alone. */
const workers = new Map<Kind, Worker>();
let seq = 0;
const pending = new Map<number, { kind: Kind; resolve: (v: any) => void; reject: (e: Error) => void; onProgress?: (p: ImportProgress) => void }>();

/** True in a browser that can run module workers (and unless disabled with ?worker=off). */
export function workerAvailable(): boolean {
  if (typeof Worker === 'undefined' || typeof window === 'undefined') return false;
  try { if (new URLSearchParams(location.search).get('worker') === 'off') return false; } catch { /* no location */ }
  return !(globalThis as any).__VBS_NO_WORKER;
}

function getWorker(kind: Kind): Worker {
  let w = workers.get(kind);
  if (w) return w;
  w = new Worker(new URL('./worker.ts', import.meta.url), { type: 'module' });
  w.onmessage = (e: MessageEvent) => {
    const { id, ok, result, error, progress } = e.data;
    const p = pending.get(id);
    if (!p) return;
    if (progress) { p.onProgress?.(progress); return; }
    pending.delete(id);
    if (ok) p.resolve(result); else p.reject(new Error(error));
  };
  w.onerror = (e) => { cancelWorkerJobs(kind, e.message || 'worker error'); };
  workers.set(kind, w);
  return w;
}

/** Terminate the worker of a kind and reject its pending jobs (e.g. a preview compile of a station that was closed). */
export function cancelWorkerJobs(kind: Kind, reason = 'cancelled'): void {
  workers.get(kind)?.terminate();
  workers.delete(kind);
  for (const [id, p] of pending) if (p.kind === kind) { pending.delete(id); p.reject(new Error(reason)); }
}

function call<T>(kind: Kind, msg: Record<string, unknown>, transfer: Transferable[], onProgress?: (p: ImportProgress) => void): Promise<T> {
  const id = ++seq;
  return new Promise<T>((resolve, reject) => {
    pending.set(id, { kind, resolve, reject, onProgress });
    getWorker(kind).postMessage({ ...msg, id }, transfer);
  });
}

/** Import a RoboDK station in the worker. The input buffer is copied (the caller keeps its bytes). */
export function workerImportRdk(data: ArrayBuffer, name: string, onProgress?: (p: ImportProgress) => void): Promise<WorkerImport> {
  const bytes = data.slice(0);
  return call<WorkerImport>('import', { kind: 'importRdk', bytes, name }, [bytes], onProgress);
}

/** Compile a program of a serialized station in the worker. */
export function workerCompile(file: StationFile, programId: string, opts: { collisions: boolean; selfOnly: boolean }): Promise<WorkerCompile> {
  return call<WorkerCompile>('compile', { kind: 'compile', file, programId, ...opts }, []);
}
