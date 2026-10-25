/**
 * Low-level reader for inflated RoboDK payloads (.rdk station, .robot / .tool / .sld item tables).
 * Everything is little-endian. Strings are UTF-16LE with a u32 byte-length prefix (any code point).
 * Poses are 16 doubles in column-major order, the same layout as `Mat4`.
 *
 * Station layout (verified on all 59 Library stations, see docs/robodk-file-format.md):
 *   "Station File 01.01" | chunk* | "Station File End" ...
 *   chunk = str "lvl" | u32 kind (RoboDK ITEM_TYPE_*) | u32 depth | u64 next-chunk offset | payload
 * The chunk index in file order (station = 0) is the item reference used by every other chunk.
 */
import type { Mat4 } from '../../core/math/pose';

const utf16 = typeof TextDecoder !== 'undefined' ? new TextDecoder('utf-16le') : null;

export class RdkBytes {
  readonly dv: DataView;
  constructor(readonly b: Uint8Array) {
    this.dv = new DataView(b.buffer, b.byteOffset, b.byteLength);
  }
  get length(): number { return this.b.length; }
  u8(o: number): number { return this.b[o]; }
  u16(o: number): number { return this.dv.getUint16(o, true); }
  u32(o: number): number { return this.dv.getUint32(o, true); }
  i32(o: number): number { return this.dv.getInt32(o, true); }
  u64(o: number): number { return this.dv.getUint32(o, true) + this.dv.getUint32(o + 4, true) * 0x100000000; }
  f32(o: number): number { return this.dv.getFloat32(o, true); }
  f64(o: number): number { return this.dv.getFloat64(o, true); }
  ok(o: number, n: number): boolean { return o >= 0 && o + n <= this.b.length; }
  /** Length-prefixed UTF-16LE string at `o`, or null when the prefix is not a plausible byte length. */
  str(o: number, max = 1 << 24): { text: string; next: number } | null {
    if (!this.ok(o, 4)) return null;
    const n = this.u32(o);
    if (n % 2 !== 0 || n > max || o + 4 + n > this.b.length) return null;
    const bytes = this.b.subarray(o + 4, o + 4 + n);
    let text: string;
    if (utf16) text = utf16.decode(bytes);
    else { text = ''; for (let i = 0; i < n; i += 2) text += String.fromCharCode(bytes[i] | (bytes[i + 1] << 8)); }
    return { text, next: o + 4 + n };
  }
  /** String that must equal `want` (used for tags such as "lvl", "ins", "FrameChkptStart"). */
  tag(o: number, want: string): number {
    if (!this.ok(o, 4) || this.u32(o) !== want.length * 2) return -1;
    for (let i = 0; i < want.length; i++) if (this.b[o + 4 + 2 * i] !== want.charCodeAt(i) || this.b[o + 5 + 2 * i] !== 0) return -1;
    return o + 4 + want.length * 2;
  }
  /** 16 doubles, column-major (RoboDK and `Mat4` share the layout). */
  pose(o: number): Mat4 {
    const m = new Float64Array(16);
    for (let i = 0; i < 16; i++) m[i] = this.f64(o + i * 8);
    return m as unknown as Mat4;
  }
}

export interface RdkChunk {
  /** Item reference used across the file. */
  index: number;
  /** RoboDK ITEM_TYPE_* (1 station, 2 robot, 3 frame, 4 tool, 5 object, 6 target, 8 program, 9 instruction, 10 Python, 11 machining, 17 folder, 19 camera, 22 notes). */
  kind: number;
  depth: number;
  /** First byte after the chunk header. */
  payload: number;
  end: number;
}

export function isStationFile(r: RdkBytes): boolean {
  const s = r.str(0, 256);
  return !!s && s.text.startsWith('Station File');
}

/** Walk the linked list of `lvl` chunks. Linear in the payload size. */
export function rdkChunks(r: RdkBytes): RdkChunk[] {
  let o = -1;
  for (let p = 0; p < Math.min(r.length, 1024); p++) if (r.tag(p, 'lvl') > 0) { o = p; break; }
  const out: RdkChunk[] = [];
  while (o >= 0 && o + 24 < r.length) {
    const h = r.tag(o, 'lvl');
    if (h < 0) break;
    const kind = r.u32(h), depth = r.u32(h + 4), next = r.u64(h + 8);
    const end = next > o && next <= r.length ? next : r.length;
    out.push({ index: out.length, kind, depth, payload: h + 16, end });
    if (next <= o || next >= r.length) break;
    o = next;
  }
  return out;
}

const utf8 = typeof TextDecoder !== 'undefined' ? new TextDecoder('utf-8') : null;

/**
 * Name stored one char per double (robot table [1800], object / tool table [1440], post id [3800]): the UTF-8 bytes of
 * the name as signed chars ("Инструмент" = -48,-104,...; verified with a RoboDK-made station, synthetic fixture).
 */
export function tableChars(t: (i: number) => number, index: number, max = 120): string {
  const bytes: number[] = [];
  for (let i = 0; i < max; i++) {
    const v = t(index + i);
    if (!Number.isFinite(v) || v !== Math.round(v) || v < -128 || v > 255 || (v >= 0 && v < 32)) break;
    bytes.push(v < 0 ? v + 256 : v);
  }
  if (!bytes.some((b) => b >= 128)) return String.fromCharCode(...bytes);
  if (utf8) return utf8.decode(Uint8Array.from(bytes));
  return String.fromCharCode(...bytes);
}

/** xyz (mm) + rx, ry, rz (rad) applied as transl · rotx · roty · rotz (the order RoboDK uses in its tables). */
export function pose6(x: number, y: number, z: number, rx: number, ry: number, rz: number): Mat4 {
  const ca = Math.cos(rx), sa = Math.sin(rx), cb = Math.cos(ry), sb = Math.sin(ry), cc = Math.cos(rz), sc = Math.sin(rz);
  // R = Rx(a) Ry(b) Rz(c), column-major
  const m = new Float64Array(16);
  m[0] = cb * cc; m[4] = -cb * sc; m[8] = sb;
  m[1] = sa * sb * cc + ca * sc; m[5] = -sa * sb * sc + ca * cc; m[9] = -sa * cb;
  m[2] = -ca * sb * cc + sa * sc; m[6] = ca * sb * sc + sa * cc; m[10] = ca * cb;
  m[12] = x; m[13] = y; m[14] = z; m[15] = 1;
  return m as unknown as Mat4;
}

export function isIdentity(m: Mat4, tol = 1e-9): boolean {
  for (let i = 0; i < 16; i++) if (Math.abs(m[i] - (i % 5 === 0 ? 1 : 0)) > tol) return false;
  return true;
}
