/**
 * User-imported RoboDK post processors (Python files from RoboDK's `Posts/` folder or your own).
 * They are kept in localStorage so they survive reloads and appear next to the built-in posts in
 * every export dialog and robot post selector. Execution happens in Pyodide with the official
 * robodk.robomath / robofileio modules and a headless robodialogs (see python_post.ts).
 */
import { registerPythonPost, setSiblingPostSource } from './python_post';
import { getPost, PostProcessor, registerPost } from './base';
import { nativePost, nativePostServer } from './native_client';

const KEY = 'studio.userPosts.v1';

export interface UserPost {
  id: string;
  name: string;
  file: string;
  source: string;
  addedAt: string;
  /** Robot brand guessed from the file name (KUKA_KRC4 -> KUKA). */
  brand: string;
  /** Extension the post declares (PROG_EXT), if found. */
  extension: string;
}

function storage(): Storage | null {
  try { return typeof localStorage !== 'undefined' ? localStorage : null; } catch { return null; }
}

export function listUserPosts(): UserPost[] {
  try { return JSON.parse(storage()?.getItem(KEY) ?? '[]'); } catch { return []; }
}

function save(list: UserPost[]): void {
  try { storage()?.setItem(KEY, JSON.stringify(list)); } catch { /* quota */ }
}

export function guessBrand(file: string): string {
  const f = file.toLowerCase();
  const table: Array<[RegExp, string]> = [[/kuka/, 'KUKA'], [/abb|rapid/, 'ABB'], [/fanuc/, 'Fanuc'], [/universal|ur\b|_ur|urscript/, 'UR'], [/motoman|yaskawa/, 'Yaskawa'], [/staubli|val3/, 'Staubli'], [/doosan/, 'Doosan'], [/kawasaki/, 'Kawasaki'], [/denso/, 'Denso'], [/mecademic/, 'Mecademic'], [/nachi/, 'Nachi'], [/comau/, 'Comau'], [/omron|techman/, 'Omron/Techman'], [/epson/, 'Epson'], [/hanwha/, 'Hanwha'], [/siemens|gcode|g_code/, 'CNC']];
  for (const [re, b] of table) if (re.test(f)) return b;
  return 'Custom';
}

/**
 * Kind of a RoboDK post file: 'source' (defines class RobotPost: runs in Pyodide in the browser), 'compiled' (the
 * 3-line wrappers RoboDK ships, `from libspp.<post> import RobotPost` / `exec("from v"+ver+...)`, which load native
 * .pyd / version-specific .pyc modules: they run on the studio server with RoboDK's Python), or 'none'.
 */
export function classifyRoboDKPost(source: string): 'source' | 'compiled' | 'none' {
  if (/class\s+RobotPost\b/.test(source) && /def\s+(MoveJ|MoveL|ProgStart)\s*\(/.test(source)) return 'source';
  if (/from\s+libspp\.\w+\s+import\s+RobotPost|import\s+RobotPost\s+as\s+MainPost|exec\(\s*["']from\s+v["']|from\s+v\d{2,3}\.\w+\s+import/.test(source)) return 'compiled';
  return 'none';
}
/** Validate that a file is a RoboDK source post (defines class RobotPost with motion callbacks). */
export function looksLikeRoboDKPost(source: string): boolean {
  return classifyRoboDKPost(source) === 'source';
}

export function postIdFor(file: string): string {
  return `PY_${file.replace(/\.py$/i, '').replace(/\W+/g, '_')}`;
}

/** Register (and persist) a post. Returns the registered PostProcessor. */
export function addUserPost(file: string, source: string, persist = true): PostProcessor {
  const id = postIdFor(file);
  const name = file.replace(/\.py$/i, '');
  const ext = source.match(/PROG_EXT\s*=\s*['"]([^'"]+)['"]/)?.[1] ?? 'txt';
  setSiblingPostSource(file, source);
  const post = registerPythonPost(id, name, source);
  (post as any).brand = guessBrand(file);
  (post as any).extension = ext;
  (post as any).name = `${name} (RoboDK post)`;
  if (persist) {
    const list = listUserPosts().filter((p) => p.id !== id);
    list.push({ id, name, file, source, addedAt: new Date().toISOString(), brand: guessBrand(file), extension: ext });
    save(list);
  }
  return post;
}

export function removeUserPost(id: string): void {
  save(listUserPosts().filter((p) => p.id !== id));
}

/** Re-register persisted posts at startup. */
export function restoreUserPosts(): number {
  let n = 0;
  for (const p of listUserPosts()) {
    if (getPost(p.id)) continue;
    try { addUserPost(p.file, p.source, false); n++; } catch { /* ignore broken entries */ }
  }
  return n;
}

/**
 * Import many files (e.g. the whole RoboDK Posts folder). Source posts run in Pyodide; RoboDK's compiled posts are
 * registered as server posts when the studio server is configured (setNativePostServer), otherwise skipped with a
 * note. Every .py file is also made available to the others (sibling imports). Returns accepted / skipped names.
 */
export async function importPostFiles(files: File[]): Promise<{ added: string[]; skipped: string[]; notes?: string[] }> {
  const added: string[] = [], skipped: string[] = [], notes: string[] = [];
  const texts: Array<[File, string | null]> = [];
  for (const f of files) {
    const src = /\.py$/i.test(f.name) ? await f.text() : null;
    if (src !== null) setSiblingPostSource(f.name, src);
    texts.push([f, src]);
  }
  for (const [f, src] of texts) {
    if (src === null) { skipped.push(f.name); continue; }
    const kind = classifyRoboDKPost(src);
    if (kind === 'source') { addUserPost(f.name, src); added.push(f.name); continue; }
    if (kind === 'compiled' && nativePostServer()) { registerPost(nativePost(f.name.replace(/\.py$/i, ''))); added.push(f.name); continue; }
    if (kind === 'compiled') notes.push(`${f.name}: compiled RoboDK post (native module) - runs on the studio server with RoboDK installed`);
    skipped.push(f.name);
  }
  return notes.length ? { added, skipped, notes } : { added, skipped };
}
