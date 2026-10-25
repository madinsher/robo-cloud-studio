/**
 * Image encoders that run in Node and the browser (no canvas): PNG (RGB/RGBA, zlib via fflate) and RoboDK's
 * `.grey32` depth format (big-endian uint32: width, height, then rows bottom-up; value = depth / far · (2³²−1), 0 = no
 * measurement — what Cam2D_Snapshot(..., 'DEPTH') returns and Library/Macros/Depth_Camera.py decodes).
 */
import { zlibSync, unzlibSync } from 'fflate';

let CRC: Uint32Array | null = null;
function crc32(buf: Uint8Array, start: number, end: number): number {
  if (!CRC) { CRC = new Uint32Array(256); for (let n = 0; n < 256; n++) { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1; CRC[n] = c >>> 0; } }
  let c = 0xffffffff;
  for (let i = start; i < end; i++) c = CRC[(c ^ buf[i]) & 0xff] ^ (c >>> 8);
  return (c ^ 0xffffffff) >>> 0;
}

/** Encode RGBA pixels (row 0 = top) as PNG; `alpha: false` writes RGB (what OpenCV imdecode returns as BGR). */
export function encodePNG(rgba: Uint8Array | Uint8ClampedArray, width: number, height: number, opts: { alpha?: boolean; level?: number } = {}): Uint8Array {
  const ch = opts.alpha ? 4 : 3;
  const raw = new Uint8Array((width * ch + 1) * height);
  for (let y = 0; y < height; y++) {
    const o = y * (width * ch + 1); raw[o] = 0;
    for (let x = 0; x < width; x++) { const s = (y * width + x) * 4, d = o + 1 + x * ch; raw[d] = rgba[s]; raw[d + 1] = rgba[s + 1]; raw[d + 2] = rgba[s + 2]; if (ch === 4) raw[d + 3] = rgba[s + 3]; }
  }
  const idat = zlibSync(raw, { level: (opts.level ?? 6) as 6 });
  const chunks: Array<[string, Uint8Array]> = [];
  const ihdr = new Uint8Array(13); const dv = new DataView(ihdr.buffer);
  dv.setUint32(0, width); dv.setUint32(4, height); ihdr[8] = 8; ihdr[9] = ch === 4 ? 6 : 2; ihdr[10] = 0; ihdr[11] = 0; ihdr[12] = 0;
  chunks.push(['IHDR', ihdr], ['IDAT', idat], ['IEND', new Uint8Array(0)]);
  const total = 8 + chunks.reduce((s, [, d]) => s + 12 + d.length, 0);
  const out = new Uint8Array(total); const ov = new DataView(out.buffer);
  out.set([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a], 0);
  let p = 8;
  for (const [type, data] of chunks) {
    ov.setUint32(p, data.length); for (let i = 0; i < 4; i++) out[p + 4 + i] = type.charCodeAt(i);
    out.set(data, p + 8);
    ov.setUint32(p + 8 + data.length, crc32(out, p + 4, p + 8 + data.length));
    p += 12 + data.length;
  }
  return out;
}

/** Decode a PNG written by encodePNG (8-bit RGB/RGBA, filter 0..4) → RGBA. For tests and round trips. */
export function decodePNG(png: Uint8Array): { width: number; height: number; rgba: Uint8ClampedArray } {
  const dv = new DataView(png.buffer, png.byteOffset, png.byteLength);
  let p = 8, width = 0, height = 0, ct = 2; const idat: Uint8Array[] = [];
  while (p < png.length) {
    const len = dv.getUint32(p), type = String.fromCharCode(png[p + 4], png[p + 5], png[p + 6], png[p + 7]);
    const data = png.subarray(p + 8, p + 8 + len);
    if (type === 'IHDR') { width = dv.getUint32(p + 8); height = dv.getUint32(p + 12); ct = png[p + 17]; }
    else if (type === 'IDAT') idat.push(data);
    p += 12 + len;
  }
  const all = new Uint8Array(idat.reduce((s, d) => s + d.length, 0)); let o = 0; for (const d of idat) { all.set(d, o); o += d.length; }
  const raw = unzlibSync(all);
  const ch = ct === 6 ? 4 : 3, stride = width * ch;
  const rgba = new Uint8ClampedArray(width * height * 4);
  const prev = new Uint8Array(stride), cur = new Uint8Array(stride);
  for (let y = 0; y < height; y++) {
    const f = raw[y * (stride + 1)];
    for (let i = 0; i < stride; i++) {
      const x = raw[y * (stride + 1) + 1 + i], a = i >= ch ? cur[i - ch] : 0, b = prev[i], c = i >= ch ? prev[i - ch] : 0;
      const pa = Math.abs(b - c), pb = Math.abs(a - c), pc = Math.abs(a + b - 2 * c);
      cur[i] = (x + (f === 1 ? a : f === 2 ? b : f === 3 ? (a + b) >> 1 : f === 4 ? (pa <= pb && pa <= pc ? a : pb <= pc ? b : c) : 0)) & 255;
    }
    for (let xx = 0; xx < width; xx++) { const d = (y * width + xx) * 4; rgba[d] = cur[xx * ch]; rgba[d + 1] = cur[xx * ch + 1]; rgba[d + 2] = cur[xx * ch + 2]; rgba[d + 3] = ch === 4 ? cur[xx * ch + 3] : 255; }
    prev.set(cur);
  }
  return { width, height, rgba };
}

/** RoboDK grey32 depth: big-endian uint32 [w, h, rows bottom-up], value = depth/far·(2³²−1); 0 = no measurement. */
export function encodeGrey32(depth: Float32Array, width: number, height: number, farMm: number): Uint8Array {
  const out = new Uint8Array((2 + width * height) * 4); const dv = new DataView(out.buffer);
  dv.setUint32(0, width); dv.setUint32(4, height);
  const MAX = 4294967295;
  for (let y = 0; y < height; y++) for (let x = 0; x < width; x++) {
    const d = depth[(height - 1 - y) * width + x];
    const v = d > 0 && d <= farMm ? Math.max(1, Math.min(MAX, Math.round((d / farMm) * MAX))) : 0;
    dv.setUint32((2 + y * width + x) * 4, v);
  }
  return out;
}
export function decodeGrey32(buf: Uint8Array, farMm: number): { width: number; height: number; depth: Float32Array } {
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  const width = dv.getUint32(0), height = dv.getUint32(4), depth = new Float32Array(width * height);
  for (let y = 0; y < height; y++) for (let x = 0; x < width; x++) { const v = dv.getUint32((2 + y * width + x) * 4); depth[(height - 1 - y) * width + x] = v ? (v / 4294967295) * farMm : 0; }
  return { width, height, depth };
}
