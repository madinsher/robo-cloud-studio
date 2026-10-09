/**
 * Minimal COLLADA (.dae) geometry reader: library_geometries -> mesh -> triangles/polylist/polygons with
 * VERTEX/NORMAL inputs, applied node transforms from the visual scene (matrix / translate / rotate / scale),
 * and the asset <unit> scale. Enough for ROS-Industrial visual meshes. Output in mm when `toMm` is true.
 */
import { parseXML, childElements, childElement, XNode } from '../urdf/xml';
import { MeshData } from './stl';
import { Mat4, identity, multiply, transl, rotAxis, transformPoint, transformDir, DEG } from '../../core/math/pose';

function floats(s: string | undefined): number[] {
  return (s ?? '').trim().split(/\s+/).filter(Boolean).map(Number);
}
function ints(s: string | undefined): number[] {
  return (s ?? '').trim().split(/\s+/).filter(Boolean).map((v) => parseInt(v, 10));
}
function text(n: XNode | undefined): string {
  return n ? n.children.filter((c) => c.type === 'text').map((c) => c.text ?? '').join(' ') : '';
}
function findById(root: XNode, id: string): XNode | undefined {
  const key = id.replace(/^#/, '');
  let found: XNode | undefined;
  const walk = (n: XNode) => { if (found) return; if (n.attrs.id === key) { found = n; return; } for (const c of n.children) if (c.type === 'element') walk(c); };
  walk(root);
  return found;
}

interface GeomTris { positions: number[]; normals: number[] }

function readGeometry(root: XNode, geom: XNode): GeomTris {
  const mesh = childElement(geom, 'mesh');
  const out: GeomTris = { positions: [], normals: [] };
  if (!mesh) return out;
  const sourceArray = (id: string): { data: number[]; stride: number } => {
    const src = findById(root, id);
    if (!src) return { data: [], stride: 3 };
    const arr = childElement(src, 'float_array');
    const acc = childElement(childElement(src, 'technique_common') ?? src, 'accessor');
    return { data: floats(text(arr)), stride: parseInt(acc?.attrs.stride ?? '3', 10) };
  };
  const vertices = childElement(mesh, 'vertices');
  const vertexInputs = vertices ? childElements(vertices, 'input') : [];
  const resolveInput = (inp: XNode): { data: number[]; stride: number } | null => {
    if (inp.attrs.semantic === 'VERTEX') {
      const pos = vertexInputs.find((v) => v.attrs.semantic === 'POSITION');
      return pos ? sourceArray(pos.attrs.source) : null;
    }
    return sourceArray(inp.attrs.source);
  };
  for (const prim of mesh.children.filter((c) => c.type === 'element' && ['triangles', 'polylist', 'polygons', 'tristrips', 'trifans'].includes(c.name))) {
    const inputs = childElements(prim, 'input').map((i) => ({ semantic: i.attrs.semantic, offset: parseInt(i.attrs.offset ?? '0', 10), src: resolveInput(i) }));
    const maxOffset = Math.max(0, ...inputs.map((i) => i.offset));
    const stride = maxOffset + 1;
    const pos = inputs.find((i) => i.semantic === 'VERTEX');
    const nor = inputs.find((i) => i.semantic === 'NORMAL');
    if (!pos?.src) continue;
    const emit = (a: number, b: number, c: number, p: number[]) => {
      for (const vi of [a, b, c]) {
        const pi = p[vi * stride + pos.offset];
        out.positions.push(pos.src!.data[pi * pos.src!.stride], pos.src!.data[pi * pos.src!.stride + 1], pos.src!.data[pi * pos.src!.stride + 2]);
        if (nor?.src) { const ni = p[vi * stride + nor.offset]; out.normals.push(nor.src.data[ni * nor.src.stride], nor.src.data[ni * nor.src.stride + 1], nor.src.data[ni * nor.src.stride + 2]); }
      }
    };
    if (prim.name === 'triangles') {
      const p = ints(text(childElement(prim, 'p')));
      const n = p.length / stride;
      for (let t = 0; t + 2 < n; t += 3) emit(t, t + 1, t + 2, p);
    } else if (prim.name === 'polylist') {
      const vc = ints(text(childElement(prim, 'vcount')));
      const p = ints(text(childElement(prim, 'p')));
      let base = 0;
      for (const c of vc) { for (let k = 1; k + 1 < c; k++) emit(base, base + k, base + k + 1, p); base += c; }
    } else if (prim.name === 'polygons') {
      for (const pe of childElements(prim, 'p')) { const p = ints(text(pe)); const c = p.length / stride; for (let k = 1; k + 1 < c; k++) emit(0, k, k + 1, p); }
    } else if (prim.name === 'tristrips' || prim.name === 'trifans') {
      for (const pe of childElements(prim, 'p')) { const p = ints(text(pe)); const c = p.length / stride; for (let k = 0; k + 2 < c; k++) { if (prim.name === 'trifans') emit(0, k + 1, k + 2, p); else if (k % 2 === 0) emit(k, k + 1, k + 2, p); else emit(k + 1, k, k + 2, p); } }
    }
  }
  return out;
}

function nodeTransform(node: XNode): Mat4 {
  let m = identity();
  for (const c of node.children) {
    if (c.type !== 'element') continue;
    if (c.name === 'matrix') { const v = floats(text(c)); if (v.length === 16) { const t = new Float64Array(16); for (let r = 0; r < 4; r++) for (let col = 0; col < 4; col++) t[col * 4 + r] = v[r * 4 + col]; m = multiply(m, t); } }
    else if (c.name === 'translate') { const v = floats(text(c)); m = multiply(m, transl(v[0], v[1], v[2])); }
    else if (c.name === 'rotate') { const v = floats(text(c)); if (Math.abs(v[3]) > 1e-12) m = multiply(m, rotAxis([v[0], v[1], v[2]], v[3] * DEG)); }
    else if (c.name === 'scale') { const v = floats(text(c)); const s = identity(); s[0] = v[0]; s[5] = v[1]; s[10] = v[2]; m = multiply(m, s); }
  }
  return m;
}

export function parseDAE(textSrc: string, toMm = true): MeshData {
  const root = parseXML(textSrc);
  const asset = childElement(root, 'asset');
  const unit = parseFloat(childElement(asset ?? root, 'unit')?.attrs.meter ?? '1');
  const upAxis = text(childElement(asset ?? root, 'up_axis')).trim().toUpperCase();
  const scale = (toMm ? 1000 : 1) * unit;
  const positions: number[] = [], normals: number[] = [];
  const geomLib = childElement(root, 'library_geometries');
  const geometries = geomLib ? childElements(geomLib, 'geometry') : [];
  const scenes = childElement(root, 'library_visual_scenes');
  const instanced = new Set<string>();
  const addGeom = (g: XNode, T: Mat4) => {
    const tri = readGeometry(root, g);
    for (let i = 0; i < tri.positions.length; i += 3) {
      const p = transformPoint(T, [tri.positions[i], tri.positions[i + 1], tri.positions[i + 2]]);
      positions.push(p[0] * scale, p[1] * scale, p[2] * scale);
      if (tri.normals.length === tri.positions.length) { const n = transformDir(T, [tri.normals[i], tri.normals[i + 1], tri.normals[i + 2]]); normals.push(n[0], n[1], n[2]); }
    }
  };
  const walkNode = (node: XNode, parentT: Mat4) => {
    const T = multiply(parentT, nodeTransform(node));
    for (const c of node.children) {
      if (c.type !== 'element') continue;
      if (c.name === 'instance_geometry') { const g = findById(root, c.attrs.url); if (g) { instanced.add(c.attrs.url.replace(/^#/, '')); addGeom(g, T); } }
      else if (c.name === 'instance_node') { const n = findById(root, c.attrs.url); if (n) walkNode(n, T); }
      else if (c.name === 'node') walkNode(c, T);
    }
  };
  if (scenes) for (const vs of childElements(scenes, 'visual_scene')) for (const n of childElements(vs, 'node')) walkNode(n, identity());
  // geometries not instanced by any scene: add at identity
  for (const g of geometries) if (!instanced.has(g.attrs.id)) addGeom(g, identity());
  let pos = new Float32Array(positions);
  let nor = new Float32Array(normals.length === positions.length ? normals : []);
  if (upAxis === 'Y_UP') {
    // convert Y-up to Z-up: (x, y, z) -> (x, -z, y)
    const conv = (a: Float32Array) => { for (let i = 0; i < a.length; i += 3) { const y = a[i + 1], z = a[i + 2]; a[i + 1] = -z; a[i + 2] = y; } };
    conv(pos); if (nor.length) conv(nor);
  }
  if (!nor.length) {
    nor = new Float32Array(pos.length);
    for (let i = 0; i < pos.length; i += 9) {
      const ux = pos[i + 3] - pos[i], uy = pos[i + 4] - pos[i + 1], uz = pos[i + 5] - pos[i + 2], vx = pos[i + 6] - pos[i], vy = pos[i + 7] - pos[i + 1], vz = pos[i + 8] - pos[i + 2];
      let nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx; const l = Math.hypot(nx, ny, nz) || 1; nx /= l; ny /= l; nz /= l;
      for (let k = 0; k < 3; k++) { nor[i + k * 3] = nx; nor[i + k * 3 + 1] = ny; nor[i + k * 3 + 2] = nz; }
    }
  }
  const min: [number, number, number] = [Infinity, Infinity, Infinity], max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < pos.length; i += 3) for (let a = 0; a < 3; a++) { if (pos[i + a] < min[a]) min[a] = pos[i + a]; if (pos[i + a] > max[a]) max[a] = pos[i + a]; }
  return { positions: pos, normals: nor, min, max, triangles: pos.length / 9 };
}
