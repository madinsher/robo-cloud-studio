/**
 * Wavefront OBJ parser (v / vt / vn / f with polygons triangulated, usemtl groups) -> MeshData. With a material
 * library (`mtllib`, read through `opts.readFile`), triangles are sorted into one group per material and the MTL colours
 * (Kd, d/Tr, Ke) and diffuse textures (map_Kd) are kept in MeshData.materials.
 */
import { MeshData, MeshMaterial, MeshGroup, imageMime } from './stl';

export interface OBJOptions {
  /** Read a file referenced by the OBJ (mtllib) or the MTL (map_Kd), relative to the OBJ file. */
  readFile?: (relPath: string) => Uint8Array | undefined;
}

interface MTLEntry { name: string; Kd?: [number, number, number]; Ke?: [number, number, number]; d?: number; map?: string }

/** Parse an MTL material library. */
export function parseMTL(text: string): Map<string, MTLEntry> {
  const out = new Map<string, MTLEntry>();
  let cur: MTLEntry | null = null;
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line || line.startsWith('#')) continue;
    const sp = line.indexOf(' ');
    const key = sp < 0 ? line : line.slice(0, sp);
    const rest = sp < 0 ? '' : line.slice(sp + 1).trim();
    const nums = () => rest.split(/\s+/).map(Number);
    if (key === 'newmtl') { cur = { name: rest }; out.set(rest, cur); }
    else if (!cur) continue;
    else if (key === 'Kd') { const v = nums(); cur.Kd = [v[0], v[1] ?? v[0], v[2] ?? v[0]]; }
    else if (key === 'Ke') { const v = nums(); cur.Ke = [v[0], v[1] ?? v[0], v[2] ?? v[0]]; }
    else if (key === 'd') cur.d = +rest;
    else if (key === 'Tr') cur.d = 1 - +rest;
    else if (key === 'map_Kd') cur.map = rest.split(/\s+/).pop();
  }
  return out;
}

export function parseOBJ(text: string, scale = 1, opts: OBJOptions = {}): MeshData {
  const v: number[] = [];
  const vt: number[] = [];
  const vn: number[] = [];
  // per material bucket: positions, normals, uvs
  const buckets = new Map<string, { pos: number[]; nor: number[]; uv: number[] }>();
  const order: string[] = [];
  let cur = '';
  let anyUV = false;
  const libs: string[] = [];
  const bucket = (name: string) => {
    let b = buckets.get(name);
    if (!b) { b = { pos: [], nor: [], uv: [] }; buckets.set(name, b); order.push(name); }
    return b;
  };
  const min: [number, number, number] = [Infinity, Infinity, Infinity];
  const max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
  const lines = text.split(/\r?\n/);
  for (let li = 0; li < lines.length; li++) {
    const line = lines[li].trim();
    if (!line || line.charCodeAt(0) === 35) continue;
    const parts = line.split(/\s+/);
    const k = parts[0];
    if (k === 'v') v.push(+parts[1] * scale, +parts[2] * scale, +parts[3] * scale);
    else if (k === 'vn') vn.push(+parts[1], +parts[2], +parts[3]);
    else if (k === 'vt') vt.push(+parts[1], +(parts[2] ?? 0));
    else if (k === 'usemtl') cur = parts.slice(1).join(' ');
    else if (k === 'mtllib') libs.push(parts.slice(1).join(' '));
    else if (k === 'f') {
      const nv = v.length / 3, nt = vt.length / 2, nn = vn.length / 3;
      const idx = parts.slice(1).map((p) => {
        const [a, b, c] = p.split('/');
        const vi = +a > 0 ? +a - 1 : nv + +a;
        const ti = b ? (+b > 0 ? +b - 1 : nt + +b) : -1;
        const ni = c ? (+c > 0 ? +c - 1 : nn + +c) : -1;
        return [vi, ti, ni];
      });
      const bk = bucket(cur);
      for (let i = 1; i + 1 < idx.length; i++) {
        const tri = [idx[0], idx[i], idx[i + 1]];
        const pts = tri.map(([vi]) => (vi >= 0 && vi < nv ? [v[vi * 3], v[vi * 3 + 1], v[vi * 3 + 2]] : [0, 0, 0]));
        const ux = pts[1][0] - pts[0][0], uy = pts[1][1] - pts[0][1], uz = pts[1][2] - pts[0][2];
        const wx = pts[2][0] - pts[0][0], wy = pts[2][1] - pts[0][1], wz = pts[2][2] - pts[0][2];
        let nx = uy * wz - uz * wy, ny = uz * wx - ux * wz, nz = ux * wy - uy * wx;
        const l = Math.hypot(nx, ny, nz) || 1;
        nx /= l; ny /= l; nz /= l;
        for (let c = 0; c < 3; c++) {
          const p = pts[c];
          const [, ti, ni] = tri[c];
          bk.pos.push(p[0], p[1], p[2]);
          if (ni >= 0 && ni < nn) bk.nor.push(vn[ni * 3], vn[ni * 3 + 1], vn[ni * 3 + 2]);
          else bk.nor.push(nx, ny, nz);
          if (ti >= 0 && ti < nt) { bk.uv.push(vt[ti * 2], vt[ti * 2 + 1]); anyUV = true; } else bk.uv.push(0, 0);
          for (let a = 0; a < 3; a++) { if (p[a] < min[a]) min[a] = p[a]; if (p[a] > max[a]) max[a] = p[a]; }
        }
      }
    }
  }
  let total = 0;
  for (const b of buckets.values()) total += b.pos.length;
  const positions = new Float32Array(total), normals = new Float32Array(total);
  const uvs = anyUV ? new Float32Array((total / 3) * 2) : undefined;
  const groups: MeshGroup[] = [];
  let off = 0;
  order.forEach((name, mi) => {
    const b = buckets.get(name)!;
    positions.set(b.pos, off);
    normals.set(b.nor, off);
    if (uvs) uvs.set(b.uv, (off / 3) * 2);
    groups.push({ start: off / 3, count: b.pos.length / 3, material: mi });
    off += b.pos.length;
  });
  const out: MeshData = { positions, normals, min, max, triangles: total / 9 };
  if (uvs) out.uvs = uvs;
  const named = order.some((n) => n !== '');
  if (named || libs.length) {
    const lib = new Map<string, MTLEntry>();
    if (opts.readFile) for (const l of libs) {
      const bytes = opts.readFile(l);
      if (bytes) for (const [n, e] of parseMTL(new TextDecoder().decode(bytes))) lib.set(n, e);
    }
    out.materials = order.map((name): MeshMaterial => {
      const e = lib.get(name);
      const m: MeshMaterial = { name };
      if (e?.Kd) m.color = e.Kd;
      if (e?.d !== undefined && e.d < 1) m.opacity = e.d;
      if (e?.Ke && (e.Ke[0] || e.Ke[1] || e.Ke[2])) m.emissive = e.Ke;
      if (e?.map && opts.readFile) { const img = opts.readFile(e.map); if (img) m.map = { data: img, mime: imageMime(e.map), name: e.map }; }
      return m;
    });
    out.groups = groups;
  }
  return out;
}
