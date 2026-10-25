/**
 * File system of the Robolink API in the browser: files written by API calls and station scripts (Cam2D_Snapshot,
 * MakeProgram, Save, ProgramStart/Finish code, CSV exports) are kept in memory for the session, so the calls succeed
 * like in RoboDK (Cam2D_Snapshot(file) returns 1, a script can AddFile what it saved) and the user downloads them from
 * File › Files saved by scripts. Reading a path that was not written in this session fails with a clear message.
 */
import type { ApiFileSystem } from './robolink';

export interface MemoryFile { path: string; data: Uint8Array; time: number }

const norm = (p: string) => String(p).replace(/\\/g, '/').replace(/\/{2,}/g, '/');

export class MemoryFileSystem implements ApiFileSystem {
  readonly files = new Map<string, MemoryFile>();
  /** Called after every write (UI notice). */
  onWrite: ((f: MemoryFile) => void) | null = null;
  readFile(path: string): Uint8Array {
    const f = this.files.get(norm(path));
    if (!f) throw new Error(`InputError: file not found: ${path} (the browser keeps the files written in this session only; drop files onto the viewport or use the studio server for disk access)`);
    return f.data;
  }
  writeFile(path: string, data: string | Uint8Array): void {
    const f: MemoryFile = { path: norm(path), data: typeof data === 'string' ? new TextEncoder().encode(data) : data, time: Date.now() };
    this.files.set(f.path, f);
    this.onWrite?.(f);
  }
  exists(path: string): boolean {
    const p = norm(path).replace(/\/$/, '');
    if (this.files.has(p)) return true;
    for (const k of this.files.keys()) if (k.startsWith(p + '/')) return true;   // a folder holding saved files
    return false;
  }
  mkdirp(): void { /* folders are implicit */ }
  list(): MemoryFile[] { return [...this.files.values()].sort((a, b) => a.time - b.time); }
  clear(): void { this.files.clear(); }
}
