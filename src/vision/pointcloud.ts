/**
 * Point-cloud tools for LiDAR / depth-camera perception: PCD & PLY read/write, voxel down-sampling, cropping,
 * transforms, RANSAC ground plane, Euclidean clustering with 3D boxes, PCA line fit (crop rows / trunks),
 * depth image → cloud, and a simulated 3D LiDAR / depth sensor that ray-casts the station's analytic scene
 * (ground, trunks, canopies, object boxes, mobile robots). Everything runs in the browser and in Node (tests).
 *
 * Units are mm in the station frame (the studio convention); PCD/PLY files are read/written in metres.
 */
import type { Station } from '../core/items/item';
import { ItemType, SceneObject } from '../core/items/item';
import { Mat4, Vec3, transformPoint, transformDir, invert } from '../core/math/pose';
import { CropRow, FieldItem } from '../agri/items';
import { plantPosition } from '../agri/orchard';
import type { MobileRobot } from '../mobile/items';

export interface PointCloud {
  /** Interleaved xyz (mm). */
  xyz: Float32Array;
  intensity?: Float32Array;
  /** Interleaved rgb bytes. */
  rgb?: Uint8Array;
  /** Per-point label (0 = unlabelled) set by segmentation. */
  label?: Uint16Array;
  frame: 'sensor' | 'world';
}
export const pointCount = (c: PointCloud): number => c.xyz.length / 3;

export function makeCloud(points: ArrayLike<number>, frame: PointCloud['frame'] = 'world'): PointCloud {
  return { xyz: Float32Array.from(points), frame };
}

// ---------------------------------------------------------------------------------------------
// Files
// ---------------------------------------------------------------------------------------------

/** Parse a .pcd (ascii / binary / binary_compressed is not supported) — metres → mm. */
export function parsePCD(buf: ArrayBuffer): PointCloud {
  const bytes = new Uint8Array(buf);
  let headerEnd = 0, lines: string[] = [];
  {
    let line = '', i = 0;
    while (i < bytes.length) {
      const ch = bytes[i++];
      if (ch === 10) { lines.push(line); if (/^DATA\s/i.test(line)) { headerEnd = i; break; } line = ''; } else line += String.fromCharCode(ch);
    }
  }
  const field = (k: string) => lines.find((l) => l.toUpperCase().startsWith(k + ' '))?.split(/\s+/).slice(1) ?? [];
  const fields = field('FIELDS'), sizes = field('SIZE').map(Number), types = field('TYPE'), counts = field('COUNT').map(Number);
  const n = Number(field('POINTS')[0] ?? field('WIDTH')[0] ?? 0) * (field('POINTS')[0] ? 1 : Number(field('HEIGHT')[0] ?? 1));
  const data = (field('DATA')[0] ?? 'ascii').toLowerCase();
  const ix = fields.indexOf('x'), iy = fields.indexOf('y'), iz = fields.indexOf('z'), ii = fields.indexOf('intensity'), irgb = fields.indexOf('rgb');
  if (ix < 0 || iy < 0 || iz < 0) throw new Error('PCD without x/y/z fields');
  const xyz = new Float32Array(n * 3);
  const intensity = ii >= 0 ? new Float32Array(n) : undefined;
  const rgb = irgb >= 0 ? new Uint8Array(n * 3) : undefined;
  if (data === 'ascii') {
    const text = new TextDecoder().decode(bytes.subarray(headerEnd));
    const rows = text.split('\n').filter((l) => l.trim().length);
    for (let p = 0; p < Math.min(n, rows.length); p++) {
      const v = rows[p].trim().split(/\s+/).map(Number);
      xyz[p * 3] = v[ix] * 1000; xyz[p * 3 + 1] = v[iy] * 1000; xyz[p * 3 + 2] = v[iz] * 1000;
      if (intensity) intensity[p] = v[ii];
      if (rgb) { const f = new Float32Array([v[irgb]]); const u = new Uint32Array(f.buffer)[0]; rgb[p * 3] = (u >> 16) & 255; rgb[p * 3 + 1] = (u >> 8) & 255; rgb[p * 3 + 2] = u & 255; }
    }
  } else if (data === 'binary') {
    const offsets: number[] = []; let stride = 0;
    fields.forEach((_, k) => { offsets.push(stride); stride += (sizes[k] ?? 4) * (counts[k] ?? 1); });
    const dv = new DataView(buf, headerEnd);
    const read = (p: number, k: number) => { const o = p * stride + offsets[k]; const t = types[k] ?? 'F', s = sizes[k] ?? 4; if (t === 'F') return s === 8 ? dv.getFloat64(o, true) : dv.getFloat32(o, true); if (t === 'U') return s === 1 ? dv.getUint8(o) : s === 2 ? dv.getUint16(o, true) : dv.getUint32(o, true); return s === 1 ? dv.getInt8(o) : s === 2 ? dv.getInt16(o, true) : dv.getInt32(o, true); };
    const count = Math.min(n, Math.floor(dv.byteLength / stride));
    for (let p = 0; p < count; p++) {
      xyz[p * 3] = read(p, ix) * 1000; xyz[p * 3 + 1] = read(p, iy) * 1000; xyz[p * 3 + 2] = read(p, iz) * 1000;
      if (intensity) intensity[p] = read(p, ii);
      if (rgb) { const o = p * stride + offsets[irgb]; rgb[p * 3] = dv.getUint8(o + 2); rgb[p * 3 + 1] = dv.getUint8(o + 1); rgb[p * 3 + 2] = dv.getUint8(o); }
    }
  } else throw new Error(`PCD DATA ${data} not supported (use ascii or binary)`);
  return dropInvalid({ xyz, intensity, rgb, frame: 'sensor' });
}

/** Parse a .ply (ascii or binary_little_endian) with float x y z [red green blue] [intensity] — metres → mm. */
export function parsePLY(buf: ArrayBuffer): PointCloud {
  const bytes = new Uint8Array(buf);
  const headerText = new TextDecoder().decode(bytes.subarray(0, Math.min(bytes.length, 8192)));
  const endIdx = headerText.indexOf('end_header');
  if (endIdx < 0) throw new Error('PLY header not found');
  const header = headerText.slice(0, endIdx).split('\n').map((l) => l.trim());
  const headerEnd = endIdx + 'end_header'.length + (headerText[endIdx + 'end_header'.length] === '\r' ? 2 : 1);
  const format = header.find((l) => l.startsWith('format'))?.split(/\s+/)[1] ?? 'ascii';
  let n = 0; const props: Array<{ name: string; type: string }> = [];
  let inVertex = false;
  for (const l of header) {
    if (l.startsWith('element')) { const [, name, count] = l.split(/\s+/); inVertex = name === 'vertex'; if (inVertex) n = Number(count); }
    else if (l.startsWith('property') && inVertex) { const parts = l.split(/\s+/); if (parts[1] === 'list') continue; props.push({ name: parts[2], type: parts[1] }); }
  }
  const idx = (k: string) => props.findIndex((p) => p.name === k);
  const ix = idx('x'), iy = idx('y'), iz = idx('z'), ir = idx('red'), ig = idx('green'), ib = idx('blue'), ii = idx('intensity') >= 0 ? idx('intensity') : idx('scalar_Intensity');
  const xyz = new Float32Array(n * 3);
  const rgb = ir >= 0 ? new Uint8Array(n * 3) : undefined;
  const intensity = ii >= 0 ? new Float32Array(n) : undefined;
  if (format === 'ascii') {
    const rows = new TextDecoder().decode(bytes.subarray(headerEnd)).split('\n').filter((l) => l.trim().length);
    for (let p = 0; p < Math.min(n, rows.length); p++) {
      const v = rows[p].trim().split(/\s+/).map(Number);
      xyz[p * 3] = v[ix] * 1000; xyz[p * 3 + 1] = v[iy] * 1000; xyz[p * 3 + 2] = v[iz] * 1000;
      if (rgb) { rgb[p * 3] = v[ir]; rgb[p * 3 + 1] = v[ig]; rgb[p * 3 + 2] = v[ib]; }
      if (intensity) intensity[p] = v[ii];
    }
  } else {
    const little = format === 'binary_little_endian';
    const size: Record<string, number> = { char: 1, uchar: 1, int8: 1, uint8: 1, short: 2, ushort: 2, int16: 2, uint16: 2, int: 4, uint: 4, int32: 4, uint32: 4, float: 4, float32: 4, double: 8, float64: 8 };
    const offs: number[] = []; let stride = 0;
    for (const p of props) { offs.push(stride); stride += size[p.type] ?? 4; }
    const dv = new DataView(buf, headerEnd);
    const rd = (p: number, k: number) => { const o = p * stride + offs[k]; const t = props[k].type; switch (t) { case 'float': case 'float32': return dv.getFloat32(o, little); case 'double': case 'float64': return dv.getFloat64(o, little); case 'uchar': case 'uint8': return dv.getUint8(o); case 'char': case 'int8': return dv.getInt8(o); case 'ushort': case 'uint16': return dv.getUint16(o, little); case 'short': case 'int16': return dv.getInt16(o, little); case 'uint': case 'uint32': return dv.getUint32(o, little); default: return dv.getInt32(o, little); } };
    const count = Math.min(n, Math.floor(dv.byteLength / stride));
    for (let p = 0; p < count; p++) {
      xyz[p * 3] = rd(p, ix) * 1000; xyz[p * 3 + 1] = rd(p, iy) * 1000; xyz[p * 3 + 2] = rd(p, iz) * 1000;
      if (rgb) { rgb[p * 3] = rd(p, ir); rgb[p * 3 + 1] = rd(p, ig); rgb[p * 3 + 2] = rd(p, ib); }
      if (intensity) intensity[p] = rd(p, ii);
    }
  }
  return dropInvalid({ xyz, rgb, intensity, frame: 'sensor' });
}

export function writePCD(c: PointCloud, binary = true): Uint8Array {
  const n = pointCount(c);
  const fields = ['x', 'y', 'z', ...(c.intensity ? ['intensity'] : []), ...(c.rgb ? ['rgb'] : [])];
  const header = `# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS ${fields.join(' ')}\nSIZE ${fields.map(() => 4).join(' ')}\nTYPE ${fields.map((f) => (f === 'rgb' ? 'U' : 'F')).join(' ')}\nCOUNT ${fields.map(() => 1).join(' ')}\nWIDTH ${n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS ${n}\nDATA ${binary ? 'binary' : 'ascii'}\n`;
  const head = new TextEncoder().encode(header);
  if (!binary) {
    let body = '';
    for (let p = 0; p < n; p++) {
      body += `${(c.xyz[p * 3] / 1000).toFixed(4)} ${(c.xyz[p * 3 + 1] / 1000).toFixed(4)} ${(c.xyz[p * 3 + 2] / 1000).toFixed(4)}`;
      if (c.intensity) body += ` ${c.intensity[p]}`;
      if (c.rgb) body += ` ${(c.rgb[p * 3] << 16) | (c.rgb[p * 3 + 1] << 8) | c.rgb[p * 3 + 2]}`;
      body += '\n';
    }
    const b = new TextEncoder().encode(body);
    const out = new Uint8Array(head.length + b.length); out.set(head); out.set(b, head.length); return out;
  }
  const stride = fields.length * 4;
  const out = new Uint8Array(head.length + n * stride); out.set(head);
  const dv = new DataView(out.buffer, head.length);
  for (let p = 0; p < n; p++) {
    let o = p * stride;
    dv.setFloat32(o, c.xyz[p * 3] / 1000, true); dv.setFloat32(o + 4, c.xyz[p * 3 + 1] / 1000, true); dv.setFloat32(o + 8, c.xyz[p * 3 + 2] / 1000, true); o += 12;
    if (c.intensity) { dv.setFloat32(o, c.intensity[p], true); o += 4; }
    if (c.rgb) dv.setUint32(o, (c.rgb[p * 3] << 16) | (c.rgb[p * 3 + 1] << 8) | c.rgb[p * 3 + 2], true);
  }
  return out;
}

export function writePLY(c: PointCloud): Uint8Array {
  const n = pointCount(c);
  const header = `ply\nformat binary_little_endian 1.0\ncomment VerticalBot Studio\nelement vertex ${n}\nproperty float x\nproperty float y\nproperty float z\n${c.rgb ? 'property uchar red\nproperty uchar green\nproperty uchar blue\n' : ''}${c.intensity ? 'property float intensity\n' : ''}end_header\n`;
  const head = new TextEncoder().encode(header);
  const stride = 12 + (c.rgb ? 3 : 0) + (c.intensity ? 4 : 0);
  const out = new Uint8Array(head.length + n * stride); out.set(head);
  const dv = new DataView(out.buffer, head.length);
  for (let p = 0; p < n; p++) {
    let o = p * stride;
    dv.setFloat32(o, c.xyz[p * 3] / 1000, true); dv.setFloat32(o + 4, c.xyz[p * 3 + 1] / 1000, true); dv.setFloat32(o + 8, c.xyz[p * 3 + 2] / 1000, true); o += 12;
    if (c.rgb) { out[head.length + o] = c.rgb[p * 3]; out[head.length + o + 1] = c.rgb[p * 3 + 1]; out[head.length + o + 2] = c.rgb[p * 3 + 2]; o += 3; }
    if (c.intensity) dv.setFloat32(o, c.intensity[p], true);
  }
  return out;
}

function dropInvalid(c: PointCloud): PointCloud {
  const n = pointCount(c);
  let m = 0;
  for (let p = 0; p < n; p++) if (Number.isFinite(c.xyz[p * 3]) && Number.isFinite(c.xyz[p * 3 + 1]) && Number.isFinite(c.xyz[p * 3 + 2])) m++;
  if (m === n) return c;
  return selectPoints(c, (p) => Number.isFinite(c.xyz[p * 3]) && Number.isFinite(c.xyz[p * 3 + 1]) && Number.isFinite(c.xyz[p * 3 + 2]));
}

export function selectPoints(c: PointCloud, keep: (i: number) => boolean): PointCloud {
  const n = pointCount(c);
  const idx: number[] = [];
  for (let p = 0; p < n; p++) if (keep(p)) idx.push(p);
  const xyz = new Float32Array(idx.length * 3);
  const intensity = c.intensity ? new Float32Array(idx.length) : undefined;
  const rgb = c.rgb ? new Uint8Array(idx.length * 3) : undefined;
  const label = c.label ? new Uint16Array(idx.length) : undefined;
  idx.forEach((p, k) => {
    xyz[k * 3] = c.xyz[p * 3]; xyz[k * 3 + 1] = c.xyz[p * 3 + 1]; xyz[k * 3 + 2] = c.xyz[p * 3 + 2];
    if (intensity) intensity[k] = c.intensity![p];
    if (rgb) { rgb[k * 3] = c.rgb![p * 3]; rgb[k * 3 + 1] = c.rgb![p * 3 + 1]; rgb[k * 3 + 2] = c.rgb![p * 3 + 2]; }
    if (label) label[k] = c.label![p];
  });
  return { xyz, intensity, rgb, label, frame: c.frame };
}

// ---------------------------------------------------------------------------------------------
// Filters
// ---------------------------------------------------------------------------------------------

export function transformCloud(c: PointCloud, m: Mat4, frame: PointCloud['frame'] = 'world'): PointCloud {
  const n = pointCount(c);
  const xyz = new Float32Array(n * 3);
  for (let p = 0; p < n; p++) {
    const x = c.xyz[p * 3], y = c.xyz[p * 3 + 1], z = c.xyz[p * 3 + 2];
    xyz[p * 3] = m[0] * x + m[4] * y + m[8] * z + m[12];
    xyz[p * 3 + 1] = m[1] * x + m[5] * y + m[9] * z + m[13];
    xyz[p * 3 + 2] = m[2] * x + m[6] * y + m[10] * z + m[14];
  }
  return { ...c, xyz, frame };
}

/** Voxel-grid down-sampling (centroid per voxel), voxel size in mm. */
export function voxelDownsample(c: PointCloud, voxel: number): PointCloud {
  const n = pointCount(c);
  const cells = new Map<string, { s: [number, number, number]; i: number; k: number; r: [number, number, number]; l: number }>();
  for (let p = 0; p < n; p++) {
    const x = c.xyz[p * 3], y = c.xyz[p * 3 + 1], z = c.xyz[p * 3 + 2];
    const key = `${Math.floor(x / voxel)},${Math.floor(y / voxel)},${Math.floor(z / voxel)}`;
    let cell = cells.get(key);
    if (!cell) { cell = { s: [0, 0, 0], i: 0, k: 0, r: [0, 0, 0], l: 0 }; cells.set(key, cell); }
    cell.s[0] += x; cell.s[1] += y; cell.s[2] += z; cell.k++;
    if (c.intensity) cell.i += c.intensity[p];
    if (c.rgb) { cell.r[0] += c.rgb[p * 3]; cell.r[1] += c.rgb[p * 3 + 1]; cell.r[2] += c.rgb[p * 3 + 2]; }
    if (c.label) cell.l = Math.max(cell.l, c.label[p]);
  }
  const m = cells.size;
  const xyz = new Float32Array(m * 3), intensity = c.intensity ? new Float32Array(m) : undefined, rgb = c.rgb ? new Uint8Array(m * 3) : undefined, label = c.label ? new Uint16Array(m) : undefined;
  let k = 0;
  for (const cell of cells.values()) {
    xyz[k * 3] = cell.s[0] / cell.k; xyz[k * 3 + 1] = cell.s[1] / cell.k; xyz[k * 3 + 2] = cell.s[2] / cell.k;
    if (intensity) intensity[k] = cell.i / cell.k;
    if (rgb) { rgb[k * 3] = cell.r[0] / cell.k; rgb[k * 3 + 1] = cell.r[1] / cell.k; rgb[k * 3 + 2] = cell.r[2] / cell.k; }
    if (label) label[k] = cell.l;
    k++;
  }
  return { xyz, intensity, rgb, label, frame: c.frame };
}

export function cropBox(c: PointCloud, min: Vec3, max: Vec3): PointCloud {
  return selectPoints(c, (p) => { const x = c.xyz[p * 3], y = c.xyz[p * 3 + 1], z = c.xyz[p * 3 + 2]; return x >= min[0] && x <= max[0] && y >= min[1] && y <= max[1] && z >= min[2] && z <= max[2]; });
}

export function bounds(c: PointCloud): { min: Vec3; max: Vec3 } {
  const min: Vec3 = [Infinity, Infinity, Infinity], max: Vec3 = [-Infinity, -Infinity, -Infinity];
  for (let p = 0; p < pointCount(c); p++) for (let k = 0; k < 3; k++) { const v = c.xyz[p * 3 + k]; if (v < min[k]) min[k] = v; if (v > max[k]) max[k] = v; }
  return { min, max };
}

// ---------------------------------------------------------------------------------------------
// Ground plane (RANSAC) and clustering
// ---------------------------------------------------------------------------------------------

export interface Plane { normal: Vec3; d: number; inliers: number }

function rng(seed: number) { let s = seed >>> 0 || 1; return () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; }; }

/** RANSAC plane fit; `thresh` in mm. Prefers planes whose normal is near +Z when `upBias` is set. */
export function fitGroundPlane(c: PointCloud, opts: { iterations?: number; thresh?: number; seed?: number; upBias?: boolean } = {}): Plane | null {
  const n = pointCount(c);
  if (n < 3) return null;
  const it = opts.iterations ?? 200, th = opts.thresh ?? 80, rnd = rng(opts.seed ?? 7);
  let best: Plane | null = null;
  const P = (i: number): Vec3 => [c.xyz[i * 3], c.xyz[i * 3 + 1], c.xyz[i * 3 + 2]];
  for (let k = 0; k < it; k++) {
    const a = P(Math.floor(rnd() * n)), b = P(Math.floor(rnd() * n)), d = P(Math.floor(rnd() * n));
    const u: Vec3 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], v: Vec3 = [d[0] - a[0], d[1] - a[1], d[2] - a[2]];
    let nrm: Vec3 = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    const l = Math.hypot(...nrm); if (l < 1e-6) continue;
    nrm = [nrm[0] / l, nrm[1] / l, nrm[2] / l];
    if (nrm[2] < 0) nrm = [-nrm[0], -nrm[1], -nrm[2]];
    if (opts.upBias !== false && nrm[2] < 0.7) continue;
    const dd = -(nrm[0] * a[0] + nrm[1] * a[1] + nrm[2] * a[2]);
    let inl = 0;
    for (let p = 0; p < n; p++) if (Math.abs(nrm[0] * c.xyz[p * 3] + nrm[1] * c.xyz[p * 3 + 1] + nrm[2] * c.xyz[p * 3 + 2] + dd) < th) inl++;
    if (!best || inl > best.inliers) best = { normal: nrm, d: dd, inliers: inl };
  }
  return best;
}

export function planeDistance(pl: Plane, x: number, y: number, z: number): number {
  return pl.normal[0] * x + pl.normal[1] * y + pl.normal[2] * z + pl.d;
}

/** Split into ground / non-ground by plane distance (mm). */
export function removeGround(c: PointCloud, pl: Plane, thresh = 100): { ground: PointCloud; objects: PointCloud } {
  const g = selectPoints(c, (p) => Math.abs(planeDistance(pl, c.xyz[p * 3], c.xyz[p * 3 + 1], c.xyz[p * 3 + 2])) < thresh);
  const o = selectPoints(c, (p) => planeDistance(pl, c.xyz[p * 3], c.xyz[p * 3 + 1], c.xyz[p * 3 + 2]) >= thresh);
  return { ground: g, objects: o };
}

export interface Cluster {
  id: number;
  indices: number[];
  centroid: Vec3;
  min: Vec3;
  max: Vec3;
  /** Footprint size (mm) and height. */
  size: Vec3;
  count: number;
  /** Heuristic class from shape: trunk (tall thin), canopy, person-sized, low obstacle, wall/row. */
  shape: 'trunk' | 'canopy' | 'person' | 'low' | 'wall' | 'blob';
}

/** Euclidean clustering on a hashed voxel grid (tolerance in mm). */
export function euclideanCluster(c: PointCloud, tol = 300, minPts = 5, maxPts = 1e6): Cluster[] {
  const n = pointCount(c);
  const grid = new Map<string, number[]>();
  const key = (x: number, y: number, z: number) => `${Math.floor(x / tol)},${Math.floor(y / tol)},${Math.floor(z / tol)}`;
  for (let p = 0; p < n; p++) { const k = key(c.xyz[p * 3], c.xyz[p * 3 + 1], c.xyz[p * 3 + 2]); const arr = grid.get(k); if (arr) arr.push(p); else grid.set(k, [p]); }
  const seen = new Uint8Array(n);
  const out: Cluster[] = [];
  const t2 = tol * tol;
  for (let s = 0; s < n; s++) {
    if (seen[s]) continue;
    seen[s] = 1;
    const stack = [s], members: number[] = [];
    while (stack.length) {
      const p = stack.pop()!; members.push(p);
      if (members.length > maxPts) break;
      const px = c.xyz[p * 3], py = c.xyz[p * 3 + 1], pz = c.xyz[p * 3 + 2];
      const gx = Math.floor(px / tol), gy = Math.floor(py / tol), gz = Math.floor(pz / tol);
      for (let dx = -1; dx <= 1; dx++) for (let dy = -1; dy <= 1; dy++) for (let dz = -1; dz <= 1; dz++) {
        const arr = grid.get(`${gx + dx},${gy + dy},${gz + dz}`); if (!arr) continue;
        for (const q of arr) { if (seen[q]) continue; const ddx = c.xyz[q * 3] - px, ddy = c.xyz[q * 3 + 1] - py, ddz = c.xyz[q * 3 + 2] - pz; if (ddx * ddx + ddy * ddy + ddz * ddz <= t2) { seen[q] = 1; stack.push(q); } }
      }
    }
    if (members.length < minPts) continue;
    const min: Vec3 = [Infinity, Infinity, Infinity], max: Vec3 = [-Infinity, -Infinity, -Infinity], cen: Vec3 = [0, 0, 0];
    for (const p of members) for (let k = 0; k < 3; k++) { const v = c.xyz[p * 3 + k]; if (v < min[k]) min[k] = v; if (v > max[k]) max[k] = v; cen[k] += v / members.length; }
    const size: Vec3 = [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
    out.push({ id: out.length + 1, indices: members, centroid: cen, min, max, size, count: members.length, shape: classifyShape(size, min[2]) });
  }
  out.sort((a, b) => b.count - a.count);
  out.forEach((cl, i) => (cl.id = i + 1));
  return out;
}

function classifyShape(size: Vec3, zMin: number): Cluster['shape'] {
  const foot = Math.max(size[0], size[1]), h = size[2];
  if (foot > 4000 && h < 2500) return 'wall';
  if (h > 1200 && foot < 500) return 'trunk';
  if (h > 1300 && h < 2200 && foot > 300 && foot < 1000) return 'person';
  if (h > 800 && foot > 800) return 'canopy';
  if (h < 600 && zMin < 400) return 'low';
  return 'blob';
}

/** Label points by cluster id (0 = unclustered). */
export function labelClusters(c: PointCloud, clusters: Cluster[]): PointCloud {
  const label = new Uint16Array(pointCount(c));
  for (const cl of clusters) for (const i of cl.indices) label[i] = cl.id;
  return { ...c, label };
}

// ---------------------------------------------------------------------------------------------
// Lines (crop rows, trunk lines), canopy metrics
// ---------------------------------------------------------------------------------------------

export interface Line2D { point: [number, number]; dir: [number, number]; rms: number; length: number; n: number }

/** PCA line fit through 2D points (mm). */
export function fitLine2D(pts: Array<[number, number]>): Line2D | null {
  if (pts.length < 2) return null;
  let mx = 0, my = 0; for (const p of pts) { mx += p[0] / pts.length; my += p[1] / pts.length; }
  let sxx = 0, sxy = 0, syy = 0; for (const p of pts) { const dx = p[0] - mx, dy = p[1] - my; sxx += dx * dx; sxy += dx * dy; syy += dy * dy; }
  const th = 0.5 * Math.atan2(2 * sxy, sxx - syy);
  const dir: [number, number] = [Math.cos(th), Math.sin(th)];
  let rms = 0, smin = Infinity, smax = -Infinity;
  for (const p of pts) { const dx = p[0] - mx, dy = p[1] - my; const s = dx * dir[0] + dy * dir[1]; const d = -dx * dir[1] + dy * dir[0]; rms += d * d; smin = Math.min(smin, s); smax = Math.max(smax, s); }
  return { point: [mx, my], dir, rms: Math.sqrt(rms / pts.length), length: smax - smin, n: pts.length };
}

/** Group trunk-like clusters into row lines: greedy assignment by lateral distance to a growing line. */
export function detectRows(clusters: Cluster[], opts: { maxLateral?: number; minTrunks?: number } = {}): Line2D[] {
  const trunks = clusters.filter((c) => c.shape === 'trunk' || c.shape === 'canopy' || c.shape === 'blob').map((c) => [c.centroid[0], c.centroid[1]] as [number, number]);
  const maxLat = opts.maxLateral ?? 600, minT = opts.minTrunks ?? 3;
  const used = new Uint8Array(trunks.length);
  const lines: Line2D[] = [];
  for (let i = 0; i < trunks.length; i++) {
    if (used[i]) continue;
    // seed with the nearest unused neighbour
    let best = -1, bd = Infinity;
    for (let j = 0; j < trunks.length; j++) if (j !== i && !used[j]) { const d = Math.hypot(trunks[j][0] - trunks[i][0], trunks[j][1] - trunks[i][1]); if (d < bd) { bd = d; best = j; } }
    if (best < 0) break;
    let members = [i, best];
    let line = fitLine2D(members.map((k) => trunks[k]))!;
    let grew = true;
    while (grew) {
      grew = false;
      for (let j = 0; j < trunks.length; j++) {
        if (used[j] || members.includes(j)) continue;
        const dx = trunks[j][0] - line.point[0], dy = trunks[j][1] - line.point[1];
        const lat = Math.abs(-dx * line.dir[1] + dy * line.dir[0]);
        if (lat < maxLat) { members.push(j); line = fitLine2D(members.map((k) => trunks[k]))!; grew = true; }
      }
    }
    if (members.length >= minT) { for (const k of members) used[k] = 1; lines.push(line); }
    else members = [];
  }
  return lines;
}

/** Canopy metrics for a cluster: volume of the bounding box (m³), height (m), max width (m). */
export function canopyMetrics(cl: Cluster): { volumeM3: number; heightM: number; widthM: number } {
  return { volumeM3: (cl.size[0] * cl.size[1] * cl.size[2]) / 1e9, heightM: cl.size[2] / 1000, widthM: Math.max(cl.size[0], cl.size[1]) / 1000 };
}

// ---------------------------------------------------------------------------------------------
// Depth image → cloud
// ---------------------------------------------------------------------------------------------

export function depthToCloud(depth: Float32Array, K: { fx: number; fy: number; cx: number; cy: number; width: number; height: number }, camWorld?: Mat4, step = 1): PointCloud {
  const pts: number[] = [];
  for (let v = 0; v < K.height; v += step) for (let u = 0; u < K.width; u += step) {
    const z = depth[v * K.width + u];
    if (!(z > 0) || !Number.isFinite(z)) continue;
    const x = ((u - K.cx) / K.fx) * z, y = ((v - K.cy) / K.fy) * z;
    if (camWorld) { const p = transformPoint(camWorld, [x, y, z]); pts.push(p[0], p[1], p[2]); } else pts.push(x, y, z);
  }
  return { xyz: Float32Array.from(pts), frame: camWorld ? 'world' : 'sensor' };
}

// ---------------------------------------------------------------------------------------------
// Simulated 3D LiDAR / depth camera against the station's analytic geometry
// ---------------------------------------------------------------------------------------------

export type Primitive =
  | { kind: 'plane'; z: number; cls: string }
  | { kind: 'cylinder'; x: number; y: number; z0: number; z1: number; r: number; cls: string; obj?: unknown }
  | { kind: 'sphere'; c: Vec3; r: number; cls: string; obj?: unknown }
  | { kind: 'box'; pose: Mat4; inv: Mat4; half: Vec3; cls: string; obj?: unknown };

/** Collect ray-castable primitives from the station: ground, trunks, canopies, fruit (optional), object boxes, mobile robots. */
export function scenePrimitives(station: Station, opts: { fruit?: boolean; ground?: boolean } = {}): Primitive[] {
  const prims: Primitive[] = [];
  if (opts.ground !== false) prims.push({ kind: 'plane', z: 0, cls: 'ground' });
  for (const f of station.itemsOfType<FieldItem>(ItemType.FIELD)) {
    const abs = f.poseAbs();
    const c = f.crop;
    const box = c.training === 'trellis_2d' || c.training === 'v_trellis' || c.training === 'greenhouse_gutter';
    for (const row of f.rows()) {
      const [dx, dy] = row.direction();
      for (const p of row.plants) {
        const [lx, ly] = plantPosition(row, p);
        const wx = abs[0] * lx + abs[4] * ly + abs[12], wy = abs[1] * lx + abs[5] * ly + abs[13], wz = abs[14];
        const trunkH = Math.max(1, c.canopyBase);
        prims.push({ kind: 'cylinder', x: wx, y: wy, z0: wz, z1: wz + trunkH, r: 60, cls: 'trunk', obj: p });
        if (box) {
          const pose = new Float64Array([dx, dy, 0, 0, -dy, dx, 0, 0, 0, 0, 1, 0, wx, wy, wz + c.canopyBase + p.height / 2, 1]);
          prims.push({ kind: 'box', pose, inv: invert(pose), half: [c.plantSpacing * 0.475, p.width * 0.3, p.height / 2], cls: 'canopy', obj: p });
        } else prims.push({ kind: 'sphere', c: [wx, wy, wz + c.canopyBase + p.height / 2], r: Math.max(p.width, p.height) / 2, cls: 'canopy', obj: p });
        if (opts.fruit) for (const fr of p.fruit) { if (fr.picked) continue; const fx = lx + dx * fr.p[0] - dy * fr.p[1], fy = ly + dy * fr.p[0] + dx * fr.p[1]; prims.push({ kind: 'sphere', c: [abs[0] * fx + abs[4] * fy + abs[12], abs[1] * fx + abs[5] * fy + abs[13], fr.p[2] + wz], r: fr.d / 2, cls: 'fruit', obj: fr }); }
      }
    }
  }
  for (const o of station.itemsOfType<SceneObject>(ItemType.OBJECT)) {
    const pose = o.poseAbs();
    let half: Vec3 | null = null, centre: Vec3 = [0, 0, 0];
    if (o.bbox) { half = [(o.bbox.max[0] - o.bbox.min[0]) / 2, (o.bbox.max[1] - o.bbox.min[1]) / 2, (o.bbox.max[2] - o.bbox.min[2]) / 2]; centre = [(o.bbox.max[0] + o.bbox.min[0]) / 2, (o.bbox.max[1] + o.bbox.min[1]) / 2, (o.bbox.max[2] + o.bbox.min[2]) / 2]; }
    else for (const g of o.geometry) { const pr = g.primitive; if (!pr) continue; if (pr.kind === 'box') half = [pr.size[0] / 2, pr.size[1] / 2, pr.size[2] / 2]; else if (pr.kind === 'cylinder') half = [pr.radius, pr.radius, pr.length / 2]; else if (pr.kind === 'sphere') half = [pr.radius, pr.radius, pr.radius]; if (g.origin && g.origin.length >= 3) centre = [g.origin[0], g.origin[1], g.origin[2]]; break; }
    if (!half) continue;
    const p2 = Float64Array.from(pose) as Mat4; const cw = transformPoint(pose, centre); p2[12] = cw[0]; p2[13] = cw[1]; p2[14] = cw[2];
    prims.push({ kind: 'box', pose: p2, inv: invert(p2), half, cls: 'object', obj: o });
  }
  for (const r of station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)) {
    const th = (r.state.theta * Math.PI) / 180, [L, W, H] = r.kin.footprint;
    const pose = new Float64Array([Math.cos(th), Math.sin(th), 0, 0, -Math.sin(th), Math.cos(th), 0, 0, 0, 0, 1, 0, r.state.x, r.state.y, H / 2, 1]);
    prims.push({ kind: 'box', pose, inv: invert(pose), half: [L / 2, W / 2, H / 2], cls: 'vehicle', obj: r });
  }
  return prims;
}

/** Nearest hit distance along a ray; returns [t, primitive] or null. */
export function castRay(prims: Primitive[], o: Vec3, d: Vec3, maxT: number): { t: number; prim: Primitive } | null {
  let best: { t: number; prim: Primitive } | null = null;
  for (const p of prims) {
    let t = Infinity;
    if (p.kind === 'plane') { if (Math.abs(d[2]) > 1e-9) { const tt = (p.z - o[2]) / d[2]; if (tt > 0) t = tt; } }
    else if (p.kind === 'sphere') {
      const ox = o[0] - p.c[0], oy = o[1] - p.c[1], oz = o[2] - p.c[2];
      const b = ox * d[0] + oy * d[1] + oz * d[2], cc = ox * ox + oy * oy + oz * oz - p.r * p.r;
      const disc = b * b - cc; if (disc >= 0) { const tt = -b - Math.sqrt(disc); if (tt > 0) t = tt; }
    } else if (p.kind === 'cylinder') {
      const ox = o[0] - p.x, oy = o[1] - p.y;
      const a = d[0] * d[0] + d[1] * d[1];
      if (a > 1e-12) { const b = ox * d[0] + oy * d[1], cc = ox * ox + oy * oy - p.r * p.r; const disc = b * b - a * cc; if (disc >= 0) { const tt = (-b - Math.sqrt(disc)) / a; if (tt > 0) { const z = o[2] + d[2] * tt; if (z >= p.z0 && z <= p.z1) t = tt; } } }
    } else {
      const lo = transformPoint(p.inv, o), ld = transformDir(p.inv, d);
      let tmin = 0, tmax = maxT;
      for (let k = 0; k < 3 && tmin <= tmax; k++) {
        if (Math.abs(ld[k]) < 1e-12) { if (Math.abs(lo[k]) > p.half[k]) { tmin = Infinity; break; } continue; }
        let t1 = (-p.half[k] - lo[k]) / ld[k], t2 = (p.half[k] - lo[k]) / ld[k];
        if (t1 > t2) [t1, t2] = [t2, t1];
        tmin = Math.max(tmin, t1); tmax = Math.min(tmax, t2);
      }
      if (tmin <= tmax && tmin > 0 && Number.isFinite(tmin)) t = tmin;
    }
    if (t < maxT && (!best || t < best.t)) best = { t, prim: p };
  }
  return best;
}

export interface Lidar3DOptions { channels?: number; vfov?: [number, number]; hfov?: number; hres?: number; range?: number; noise?: number; fruit?: boolean; seed?: number; classes?: boolean }

/** Simulated spinning / solid-state 3D LiDAR at `pose` (sensor +Z up, +X forward for spinning LiDARs). Returns a world-frame cloud with class labels. */
export function simulateLidar3D(station: Station, pose: Mat4, opts: Lidar3DOptions = {}): { cloud: PointCloud; classes: string[] } {
  const prims = scenePrimitives(station, { fruit: opts.fruit });
  return castCloud(prims, pose, opts, 'lidar');
}

/** Simulated depth camera at `pose` (camera +Z forward, X right, Y down) → depth image (mm) and cloud. */
export function simulateDepthCamera(station: Station, pose: Mat4, K: { fx: number; fy: number; cx: number; cy: number; width: number; height: number }, opts: { range?: number; noise?: number; step?: number; fruit?: boolean; seed?: number } = {}): { depth: Float32Array; cloud: PointCloud; classes: string[]; labelImage: Uint16Array } {
  const prims = scenePrimitives(station, { fruit: opts.fruit ?? true });
  const depth = new Float32Array(K.width * K.height);
  const labelImage = new Uint16Array(K.width * K.height);
  const classes: string[] = [];
  const clsIndex = (c: string) => { let i = classes.indexOf(c); if (i < 0) { classes.push(c); i = classes.length - 1; } return i + 1; };
  const o: Vec3 = [pose[12], pose[13], pose[14]];
  const step = opts.step ?? 2, range = opts.range ?? 10000, rnd = rng(opts.seed ?? 3), noise = opts.noise ?? 0;
  const pts: number[] = [], labels: number[] = [];
  for (let v = 0; v < K.height; v += step) for (let u = 0; u < K.width; u += step) {
    const dl: Vec3 = [(u - K.cx) / K.fx, (v - K.cy) / K.fy, 1];
    const l = Math.hypot(...dl); const dn: Vec3 = [dl[0] / l, dl[1] / l, dl[2] / l];
    const d = transformDir(pose, dn);
    const hit = castRay(prims, o, d, range);
    if (!hit) continue;
    const z = hit.t / l * (1 + noise * (rnd() - 0.5) * 2); // depth along Z
    const li = clsIndex(hit.prim.cls);
    for (let vv = v; vv < Math.min(K.height, v + step); vv++) for (let uu = u; uu < Math.min(K.width, u + step); uu++) { depth[vv * K.width + uu] = z; labelImage[vv * K.width + uu] = li; }
    const p = transformPoint(pose, [((u - K.cx) / K.fx) * z, ((v - K.cy) / K.fy) * z, z]);
    pts.push(p[0], p[1], p[2]); labels.push(li);
  }
  return { depth, cloud: { xyz: Float32Array.from(pts), label: Uint16Array.from(labels), frame: 'world' }, classes, labelImage };
}

function castCloud(prims: Primitive[], pose: Mat4, opts: Lidar3DOptions, _kind: 'lidar'): { cloud: PointCloud; classes: string[] } {
  const channels = opts.channels ?? 16, [v0, v1] = opts.vfov ?? [-15, 15], hfov = opts.hfov ?? 360, hres = opts.hres ?? 1, range = opts.range ?? 60000, noise = opts.noise ?? 20;
  const rnd = rng(opts.seed ?? 11);
  const o: Vec3 = [pose[12], pose[13], pose[14]];
  const pts: number[] = [], labels: number[] = [], intens: number[] = [];
  const classes: string[] = [];
  const clsIndex = (c: string) => { let i = classes.indexOf(c); if (i < 0) { classes.push(c); i = classes.length - 1; } return i + 1; };
  const nH = Math.max(1, Math.round(hfov / hres));
  for (let ch = 0; ch < channels; ch++) {
    const el = ((v0 + ((v1 - v0) * ch) / Math.max(1, channels - 1)) * Math.PI) / 180;
    for (let k = 0; k < nH; k++) {
      const az = ((-hfov / 2 + k * hres) * Math.PI) / 180;
      const dl: Vec3 = [Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el)];
      const d = transformDir(pose, dl);
      const hit = castRay(prims, o, d, range);
      if (!hit) continue;
      const t = hit.t + noise * (rnd() - 0.5) * 2;
      pts.push(o[0] + d[0] * t, o[1] + d[1] * t, o[2] + d[2] * t);
      labels.push(clsIndex(hit.prim.cls));
      intens.push(hit.prim.cls === 'ground' ? 0.2 : hit.prim.cls === 'trunk' ? 0.6 : hit.prim.cls === 'vehicle' ? 0.9 : 0.4);
    }
  }
  return { cloud: { xyz: Float32Array.from(pts), label: Uint16Array.from(labels), intensity: Float32Array.from(intens), frame: 'world' }, classes };
}

/** Summary used by the UI. */
export function cloudStats(c: PointCloud): { n: number; min: Vec3; max: Vec3; centroid: Vec3 } {
  const n = pointCount(c);
  const { min, max } = bounds(c);
  const centroid: Vec3 = [0, 0, 0];
  for (let p = 0; p < n; p++) for (let k = 0; k < 3; k++) centroid[k] += c.xyz[p * 3 + k] / Math.max(1, n);
  return { n, min, max, centroid };
}

/** Sub-sample to at most `max` points (uniform stride). */
export function decimate(c: PointCloud, max: number): PointCloud {
  const n = pointCount(c);
  if (n <= max) return c;
  const stride = n / max;
  let next = 0;
  return selectPoints(c, (p) => { if (p >= next) { next += stride; return true; } return false; });
}
