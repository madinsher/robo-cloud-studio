/** Minimal synchronous gzip / zip writers (stored, uncompressed blocks) for posts that write archives (UR .urp,
 * Techman / JAKA projects). Valid for every gzip / unzip reader; no dependency, works in the browser and Node. */
const CRC_TABLE = (() => {
  const t = new Uint32Array(256);
  for (let n = 0; n < 256; n++) { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1; t[n] = c >>> 0; }
  return t;
})();
export function crc32(data: Uint8Array): number {
  let c = 0xffffffff;
  for (let i = 0; i < data.length; i++) c = CRC_TABLE[(c ^ data[i]) & 0xff] ^ (c >>> 8);
  return (c ^ 0xffffffff) >>> 0;
}
export const utf8 = (s: string) => new TextEncoder().encode(s);
function concat(parts: Uint8Array[]): Uint8Array {
  const out = new Uint8Array(parts.reduce((a, p) => a + p.length, 0));
  let o = 0;
  for (const p of parts) { out.set(p, o); o += p.length; }
  return out;
}
const u16 = (v: number) => new Uint8Array([v & 0xff, (v >>> 8) & 0xff]);
const u32 = (v: number) => new Uint8Array([v & 0xff, (v >>> 8) & 0xff, (v >>> 16) & 0xff, (v >>> 24) & 0xff]);
/** Deflate "stored" blocks (max 65535 bytes each). */
function stored(data: Uint8Array): Uint8Array {
  const parts: Uint8Array[] = [];
  for (let o = 0; o < data.length || o === 0; o += 65535) {
    const chunk = data.subarray(o, Math.min(data.length, o + 65535));
    const last = o + 65535 >= data.length ? 1 : 0;
    parts.push(new Uint8Array([last]), u16(chunk.length), u16(~chunk.length & 0xffff), chunk);
    if (data.length === 0) break;
  }
  return concat(parts);
}
export function gzipStore(data: Uint8Array, name = ''): Uint8Array {
  const header = new Uint8Array([0x1f, 0x8b, 8, name ? 8 : 0, 0, 0, 0, 0, 0, 0xff]);
  return concat([header, ...(name ? [utf8(name), new Uint8Array([0])] : []), stored(data), u32(crc32(data)), u32(data.length)]);
}
export function zipStore(files: Array<{ name: string; data: Uint8Array }>): Uint8Array {
  const local: Uint8Array[] = [], central: Uint8Array[] = [];
  let offset = 0;
  for (const f of files) {
    const nm = utf8(f.name), crc = crc32(f.data);
    const lh = concat([u32(0x04034b50), u16(20), u16(0x0800), u16(0), u16(0), u16(0x21), u32(crc), u32(f.data.length), u32(f.data.length), u16(nm.length), u16(0), nm, f.data]);
    central.push(concat([u32(0x02014b50), u16(20), u16(20), u16(0x0800), u16(0), u16(0), u16(0x21), u32(crc), u32(f.data.length), u32(f.data.length), u16(nm.length), u16(0), u16(0), u16(0), u16(0), u32(0), u32(offset), nm]));
    local.push(lh);
    offset += lh.length;
  }
  const cd = concat(central);
  return concat([...local, cd, u32(0x06054b50), u16(0), u16(0), u16(files.length), u16(files.length), u32(cd.length), u32(offset), u16(0)]);
}
export function toBase64(data: Uint8Array): string {
  if (typeof Buffer !== 'undefined') return Buffer.from(data).toString('base64');
  let s = '';
  for (let i = 0; i < data.length; i += 0x8000) s += String.fromCharCode(...data.subarray(i, i + 0x8000));
  return btoa(s);
}
