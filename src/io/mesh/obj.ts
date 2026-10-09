/** Wavefront OBJ parser (v / vn / f with polygons triangulated) -> MeshData. */
import { MeshData } from './stl';

export function parseOBJ(text: string, scale = 1): MeshData {
  const v: number[][] = [];
  const vn: number[][] = [];
  const pos: number[] = [];
  const nor: number[] = [];
  const min: [number, number, number] = [Infinity, Infinity, Infinity];
  const max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line || line.startsWith('#')) continue;
    const parts = line.split(/\s+/);
    if (parts[0] === 'v') v.push([+parts[1] * scale, +parts[2] * scale, +parts[3] * scale]);
    else if (parts[0] === 'vn') vn.push([+parts[1], +parts[2], +parts[3]]);
    else if (parts[0] === 'f') {
      const idx = parts.slice(1).map((p) => { const [vi, , ni] = p.split('/'); return [(+vi > 0 ? +vi - 1 : v.length + +vi), ni ? (+ni > 0 ? +ni - 1 : vn.length + +ni) : -1]; });
      for (let i = 1; i + 1 < idx.length; i++) {
        const tri = [idx[0], idx[i], idx[i + 1]];
        const pts = tri.map(([vi]) => v[vi] ?? [0, 0, 0]);
        // face normal fallback
        const ux = pts[1][0] - pts[0][0], uy = pts[1][1] - pts[0][1], uz = pts[1][2] - pts[0][2];
        const wx = pts[2][0] - pts[0][0], wy = pts[2][1] - pts[0][1], wz = pts[2][2] - pts[0][2];
        let nx = uy * wz - uz * wy, ny = uz * wx - ux * wz, nz = ux * wy - uy * wx;
        const l = Math.hypot(nx, ny, nz) || 1;
        nx /= l; ny /= l; nz /= l;
        tri.forEach(([vi, ni], k) => {
          const p = pts[k];
          pos.push(p[0], p[1], p[2]);
          const n = ni >= 0 && vn[ni] ? vn[ni] : [nx, ny, nz];
          nor.push(n[0], n[1], n[2]);
          for (let a = 0; a < 3; a++) { if (p[a] < min[a]) min[a] = p[a]; if (p[a] > max[a]) max[a] = p[a]; }
          void vi;
        });
      }
    }
  }
  return { positions: new Float32Array(pos), normals: new Float32Array(nor), min, max, triangles: pos.length / 9 };
}
