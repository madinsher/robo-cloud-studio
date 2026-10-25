/** Shared helpers for the T2 kinematics tests: RoboDK ground truth and robot tables read from C:/RoboDK/Library. */
import { existsSync, readFileSync, readdirSync } from 'node:fs';
import { gunzipSync, inflateSync } from 'node:zlib';
import { chainFromRobodk, rdkParamsFromTable } from '../src/core/kinematics/rdk_model';
import { ChainDef } from '../src/core/kinematics/chain';
import { fromRows, Mat4 } from '../src/core/math/pose';

export const GT_DIR = 'tests/fixtures/robodk_gt';
export const LIB = 'C:/RoboDK/Library';
export const haveLib = existsSync(LIB);

export interface GtRobot { name: string; joints: number[]; lower: number[]; upper: number[]; jointsType: number; fk: Array<{ q: number[]; flange: number[] | null }>; ikAllCount?: number | null; jointsConfig?: number[] | null; fkCurrent?: number[] | null; poseAbs?: number[] }
export interface GtFile { file: string; robots: GtRobot[] }

export function loadGt(f: string): GtFile | null {
  const s = gunzipSync(readFileSync(`${GT_DIR}/${f}`)).toString();
  try { return JSON.parse(s.replace(/\bNaN\b/g, 'null').replace(/-?Infinity/g, 'null')); } catch { return null; }
}

export function m16(a: number[]): Mat4 {
  return fromRows([a.slice(0, 4), a.slice(4, 8), a.slice(8, 12), a.slice(12, 16)]);
}

/** Every robot table (doubles, starting with 1.0, 4000) found in a Library file, in file order. */
export function robotTables(file: string): Array<{ name: string; t: Float64Array }> {
  const raw = readFileSync(file);
  let b: Buffer;
  try { b = inflateSync(raw.subarray(4)); } catch { b = raw; }
  const out: Array<{ name: string; t: Float64Array }> = [];
  for (let o = 0; o + 16 <= b.length; o++) {
    if (b[o + 6] !== 0xf0 || b[o + 7] !== 0x3f || b.readDoubleLE(o) !== 1 || b.readDoubleLE(o + 8) !== 4000) continue;
    const n = Math.min(4000, Math.floor((b.length - o) / 8));
    const t = new Float64Array(n);
    for (let i = 0; i < n; i++) t[i] = b.readDoubleLE(o + i * 8);
    let name = '';
    for (let i = 1800; i < 1864 && i < n; i++) { const c = Math.round(t[i]); if (c < 32 || c > 126 || Math.abs(t[i] - c) > 1e-6) break; name += String.fromCharCode(c); }
    out.push({ name, t });
    o += n * 8 - 1;
  }
  return out;
}

/** All (GT robot, chain) pairs of the Library, deduplicated by robot name and table. */
export function libraryChains(): Array<{ file: string; gt: GtRobot; chain: ChainDef; table: Float64Array }> {
  const out: Array<{ file: string; gt: GtRobot; chain: ChainDef; table: Float64Array }> = [];
  const seen = new Set<string>();
  for (const f of readdirSync(GT_DIR).filter((x) => x.endsWith('.json.gz')).sort()) {
    const gt = loadGt(f);
    if (!gt?.robots?.length || !existsSync(gt.file)) continue;
    const tables = robotTables(gt.file);
    for (const r of gt.robots) {
      const tb = tables.find((t) => t.name === r.name);
      if (!tb) continue;
      const key = `${r.name}|${Array.from(tb.t.subarray(0, 700)).join(',')}`;
      if (seen.has(key)) continue;
      seen.add(key);
      const p = rdkParamsFromTable(tb.t, r.name);
      if (!p) continue;
      out.push({ file: f, gt: r, chain: chainFromRobodk(p), table: tb.t });
    }
  }
  return out;
}


/**
 * Rotation difference (rad) that stays accurate for tiny angles and for the 6-decimal GT matrices
 * (acos of the trace loses precision below ~1e-3 rad): ||Ra - Rb||_F / √2 ≈ angle.
 */
export function rotDiff(a: Mat4, b: Mat4): number {
  let s = 0;
  for (const k of [0, 1, 2, 4, 5, 6, 8, 9, 10]) s += (a[k] - b[k]) ** 2;
  return Math.sqrt(s / 2);
}

export function posDiff(a: Mat4, b: Mat4): number {
  return Math.hypot(a[12] - b[12], a[13] - b[13], a[14] - b[14]);
}
