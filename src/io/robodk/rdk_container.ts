/**
 * Best-effort reader for RoboDK .rdk station files.
 *
 * The .rdk container is a proprietary binary format that is not publicly documented. This reader
 * does NOT claim full fidelity. It:
 *   1. detects and inflates zlib/gzip/zip wrapped payloads (via DecompressionStream when available),
 *   2. scans for item names (UTF-8 / UTF-16LE strings) and 4x4 pose matrices (runs of 16 doubles
 *      whose last row is 0 0 0 1),
 *   3. extracts embedded STL / mesh blobs, and
 *   4. returns a sketch station (frames + objects with recovered names/poses) plus a report.
 *
 * For full-fidelity import use `python/rdk_export.py` (runs inside RoboDK and writes our JSON),
 * or the RoboDK API bridge in the server.
 */
import { Station, Frame, SceneObject } from '../../core/items/item';
import { fromRows } from '../../core/math/pose';

export interface RdkScanReport {
  bytes: number;
  compressed: 'none' | 'zlib' | 'gzip' | 'zip' | 'unknown';
  strings: string[];
  poses: number;
  meshes: Array<{ offset: number; triangles: number }>;
  notes: string[];
}

async function inflate(buf: Uint8Array, format: 'deflate' | 'gzip' | 'deflate-raw'): Promise<Uint8Array<ArrayBuffer> | null> {
  try {
    if (typeof DecompressionStream === 'undefined') return null;
    const ds = new DecompressionStream(format);
    const stream = new Blob([new Uint8Array(buf) as unknown as BlobPart]).stream().pipeThrough(ds);
    const out = new Uint8Array(await new Response(stream).arrayBuffer());
    return out;
  } catch {
    return null;
  }
}

export async function scanRdk(data: ArrayBuffer): Promise<{ payload: Uint8Array; report: RdkScanReport }> {
  let bytes: Uint8Array = new Uint8Array(data);
  const report: RdkScanReport = { bytes: bytes.length, compressed: 'none', strings: [], poses: 0, meshes: [], notes: [] };
  if (bytes[0] === 0x1f && bytes[1] === 0x8b) {
    report.compressed = 'gzip';
    bytes = (await inflate(bytes, 'gzip')) ?? bytes;
  } else if (bytes[0] === 0x78 && (bytes[1] === 0x9c || bytes[1] === 0x01 || bytes[1] === 0xda)) {
    report.compressed = 'zlib';
    bytes = (await inflate(bytes, 'deflate')) ?? bytes;
  } else if (bytes[0] === 0x50 && bytes[1] === 0x4b) {
    report.compressed = 'zip';
    report.notes.push('ZIP container detected: only stored (uncompressed) members are scanned inline.');
  } else {
    // Try to find a zlib stream inside (RoboDK often prefixes a small header)
    for (let i = 0; i < Math.min(bytes.length - 2, 4096); i++) {
      if (bytes[i] === 0x78 && (bytes[i + 1] === 0x9c || bytes[i + 1] === 0xda)) {
        const inflated = await inflate(bytes.subarray(i), 'deflate');
        if (inflated && inflated.length > bytes.length / 2) {
          report.compressed = 'zlib';
          report.notes.push(`zlib stream found at offset ${i}`);
          bytes = inflated;
          break;
        }
      }
    }
  }
  // Strings
  const seen = new Set<string>();
  const pushStr = (s: string) => {
    s = s.trim();
    if (s.length >= 3 && s.length <= 64 && /^[\x20-\x7eЀ-ӿ]+$/.test(s) && /[A-Za-zЀ-ӿ]/.test(s) && !seen.has(s)) {
      seen.add(s);
      report.strings.push(s);
    }
  };
  let cur = '';
  for (let i = 0; i < bytes.length; i++) {
    const b = bytes[i];
    if (b >= 0x20 && b < 0x7f) cur += String.fromCharCode(b);
    else { if (cur.length >= 4) pushStr(cur); cur = ''; }
  }
  // UTF-16LE strings
  cur = '';
  for (let i = 0; i + 1 < bytes.length; i += 2) {
    const c = bytes[i] | (bytes[i + 1] << 8);
    if (c >= 0x20 && c < 0x7f || (c >= 0x400 && c <= 0x4ff)) cur += String.fromCharCode(c);
    else { if (cur.length >= 4) pushStr(cur); cur = ''; }
  }
  // Poses: 16 little-endian doubles, row-major with last row 0 0 0 1 or column-major with [3],[7],[11]=0,[15]=1
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const poses: number[][][] = [];
  for (let off = 0; off + 128 <= bytes.length; off += 8) {
    const d15 = dv.getFloat64(off + 120, true);
    if (d15 !== 1) continue;
    const vals: number[] = [];
    let ok = true;
    for (let k = 0; k < 16; k++) {
      const v = dv.getFloat64(off + k * 8, true);
      if (!Number.isFinite(v) || Math.abs(v) > 1e7) { ok = false; break; }
      vals.push(v);
    }
    if (!ok) continue;
    const rowMajor = vals[12] === 0 && vals[13] === 0 && vals[14] === 0;
    const colMajor = vals[3] === 0 && vals[7] === 0 && vals[11] === 0;
    if (!rowMajor && !colMajor) continue;
    const rows = rowMajor ? [vals.slice(0, 4), vals.slice(4, 8), vals.slice(8, 12), vals.slice(12, 16)] : [[vals[0], vals[4], vals[8], vals[12]], [vals[1], vals[5], vals[9], vals[13]], [vals[2], vals[6], vals[10], vals[14]], [0, 0, 0, 1]];
    // rotation part must be orthonormal-ish
    const r = rows;
    const n0 = Math.hypot(r[0][0], r[1][0], r[2][0]), n1 = Math.hypot(r[0][1], r[1][1], r[2][1]);
    if (Math.abs(n0 - 1) > 1e-6 || Math.abs(n1 - 1) > 1e-6) continue;
    poses.push(rows);
    off += 120;
  }
  report.poses = poses.length;
  // Binary STL blobs: header(80) + uint32 count with 84 + 50*count plausible
  for (let off = 0; off + 84 < bytes.length; off++) {
    const n = dv.getUint32(off + 80, true);
    if (n > 10 && n < 5_000_000 && off + 84 + n * 50 <= bytes.length) {
      // check normals are unit-ish for the first few facets
      let plausible = true;
      for (let k = 0; k < Math.min(n, 5) && plausible; k++) {
        const o = off + 84 + k * 50;
        const nx = dv.getFloat32(o, true), ny = dv.getFloat32(o + 4, true), nz = dv.getFloat32(o + 8, true);
        const l = Math.hypot(nx, ny, nz);
        if (!Number.isFinite(l) || (l > 0 && Math.abs(l - 1) > 0.05)) plausible = false;
      }
      if (plausible) {
        report.meshes.push({ offset: off, triangles: n });
        off += 84 + n * 50 - 1;
      }
    }
  }
  (report as any)._poses = poses;
  return { payload: bytes, report };
}

/** Build a sketch station from the scan. Names are paired with poses in order of appearance. */
export async function importRdkBestEffort(data: ArrayBuffer, name = 'RoboDK import'): Promise<{ station: Station; report: RdkScanReport }> {
  const { report } = await scanRdk(data);
  const st = new Station(name);
  const poses: number[][][] = (report as any)._poses ?? [];
  const names = report.strings.filter((s) => !/^(Program|Target|Frame|Robot|Tool|Object)\d*$/i.test(s) && !s.includes('\\') && !s.includes('/'));
  const rootF = st.addChild(new Frame('Recovered items'));
  poses.forEach((rows, i) => {
    const it = new SceneObject(names[i] ?? `Pose ${i + 1}`);
    it.setPose(fromRows(rows));
    rootF.addChild(it);
  });
  report.notes.push(`Recovered ${poses.length} poses and ${report.strings.length} strings; ${report.meshes.length} embedded meshes detected.`);
  report.notes.push('The .rdk format is proprietary: use python/rdk_export.py inside RoboDK for a lossless transfer.');
  delete (report as any)._poses;
  st.params.rdkReport = JSON.parse(JSON.stringify(report));
  return { station: st, report };
}
