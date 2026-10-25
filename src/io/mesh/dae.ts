/**
 * Minimal COLLADA (.dae) geometry reader: library_geometries -> mesh -> triangles/polylist/polygons/tristrips/trifans
 * with VERTEX/NORMAL/TEXCOORD inputs, node transforms of the visual scene (matrix / translate / rotate / scale),
 * instance_node, the asset <unit> scale and Y_UP -> Z_UP conversion. Materials: bind_material/instance_material ->
 * library_materials -> library_effects (phong/lambert/blinn/constant diffuse colour or texture -> sampler -> surface ->
 * library_images), kept as per-material groups in MeshData; texture images are read through `opts.readFile`.
 * Output in mm when `toMm` is true.
 */
import { parseXML, childElements, childElement, XNode } from '../urdf/xml';
import { MeshData, MeshMaterial, MeshGroup, imageMime } from './stl';
import { Mat4, identity, multiply, transl, rotAxis, transformPoint, transformDir, DEG } from '../../core/math/pose';

export interface DAEOptions {
  /** Read an image referenced by the file (relative to the .dae). */
  readFile?: (relPath: string) => Uint8Array | undefined;
}

function floats(s: string | undefined): number[] {
  return (s ?? '').trim().split(/\s+/).filter(Boolean).map(Number);
}
function ints(s: string | undefined): number[] {
  return (s ?? '').trim().split(/\s+/).filter(Boolean).map((v) => parseInt(v, 10));
}
function text(n: XNode | undefined): string {
  return n ? n.children.filter((c) => c.type === 'text').map((c) => c.text ?? '').join(' ') : '';
}

interface GeomPrim { material: string; positions: number[]; normals: number[]; uvs: number[] }

function readGeometry(geom: XNode, byId: (id: string) => XNode | undefined): GeomPrim[] {
  const mesh = childElement(geom, 'mesh');
  const out: GeomPrim[] = [];
  if (!mesh) return out;
  const cache = new Map<string, { data: number[]; stride: number }>();
  const sourceArray = (id: string): { data: number[]; stride: number } => {
    const c = cache.get(id);
    if (c) return c;
    const src = byId(id);
    let r = { data: [] as number[], stride: 3 };
    if (src) {
      const arr = childElement(src, 'float_array');
      const acc = childElement(childElement(src, 'technique_common') ?? src, 'accessor');
      r = { data: floats(text(arr)), stride: parseInt(acc?.attrs.stride ?? '3', 10) };
    }
    cache.set(id, r);
    return r;
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
    const inputs = childElements(prim, 'input').map((i) => ({ semantic: i.attrs.semantic, set: i.attrs.set, offset: parseInt(i.attrs.offset ?? '0', 10), src: resolveInput(i) }));
    const maxOffset = Math.max(0, ...inputs.map((i) => i.offset));
    const stride = maxOffset + 1;
    const pos = inputs.find((i) => i.semantic === 'VERTEX');
    let nor = inputs.find((i) => i.semantic === 'NORMAL');
    if (!nor) {
      // normals declared inside <vertices>
      const vn = vertexInputs.find((v) => v.attrs.semantic === 'NORMAL');
      if (vn && pos) nor = { semantic: 'NORMAL', set: '', offset: pos.offset, src: sourceArray(vn.attrs.source) };
    }
    const tcs = inputs.filter((i) => i.semantic === 'TEXCOORD');
    const tc = tcs.find((i) => !i.set || i.set === '0') ?? tcs[0];
    if (!pos?.src) continue;
    const g: GeomPrim = { material: prim.attrs.material ?? '', positions: [], normals: [], uvs: [] };
    out.push(g);
    const emit = (a: number, b: number, c: number, p: number[]) => {
      for (const vi of [a, b, c]) {
        const pi = p[vi * stride + pos.offset];
        g.positions.push(pos.src!.data[pi * pos.src!.stride], pos.src!.data[pi * pos.src!.stride + 1], pos.src!.data[pi * pos.src!.stride + 2]);
        if (nor?.src) { const ni = p[vi * stride + nor.offset]; g.normals.push(nor.src.data[ni * nor.src.stride], nor.src.data[ni * nor.src.stride + 1], nor.src.data[ni * nor.src.stride + 2]); }
        if (tc?.src) { const ti = p[vi * stride + tc.offset]; g.uvs.push(tc.src.data[ti * tc.src.stride] ?? 0, tc.src.data[ti * tc.src.stride + 1] ?? 0); }
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
    if (g.normals.length !== g.positions.length) g.normals = [];
    if (g.uvs.length / 2 !== g.positions.length / 3) g.uvs = [];
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

/** Material id -> MeshMaterial (diffuse colour / texture, transparency, emission). */
function readMaterial(matId: string, byId: (id: string) => XNode | undefined, opts: DAEOptions): MeshMaterial {
  const mat = byId(matId);
  const out: MeshMaterial = { name: mat?.attrs.name ?? matId };
  const effUrl = mat ? childElement(mat, 'instance_effect')?.attrs.url : undefined;
  const eff = effUrl ? byId(effUrl) : undefined;
  if (!eff) return out;
  const prof = childElement(eff, 'profile_COMMON');
  if (!prof) return out;
  const tech = childElement(prof, 'technique');
  const shader = tech ? tech.children.find((c) => c.type === 'element' && ['phong', 'lambert', 'blinn', 'constant'].includes(c.name)) : undefined;
  if (!shader) return out;
  const colorOf = (el: XNode | undefined): [number, number, number] | undefined => { const c = el ? childElement(el, 'color') : undefined; if (!c) return undefined; const v = floats(text(c)); return v.length >= 3 ? [v[0], v[1], v[2]] : undefined; };
  const diffuse = childElement(shader, 'diffuse') ?? childElement(shader, 'emission');
  const dc = colorOf(diffuse);
  if (dc) out.color = dc;
  const em = colorOf(childElement(shader, 'emission'));
  if (em && (em[0] || em[1] || em[2]) && shader.name !== 'constant') out.emissive = em;
  const tr = childElement(shader, 'transparency');
  const trv = tr ? floats(text(childElement(tr, 'float'))) : [];
  if (trv.length && trv[0] < 1 && trv[0] > 0) out.opacity = trv[0];
  const tex = diffuse ? childElement(diffuse, 'texture') : undefined;
  if (tex && opts.readFile) {
    // texture -> sampler2D newparam -> surface newparam -> init_from image id -> library_images init_from path
    const params = new Map<string, XNode>();
    for (const np of childElements(prof, 'newparam')) params.set(np.attrs.sid, np);
    let imageId = tex.attrs.texture;
    const sampler = params.get(imageId);
    if (sampler) {
      const s2d = childElement(sampler, 'sampler2D');
      const surfSid = text(childElement(s2d ?? sampler, 'source')).trim();
      const inst = s2d ? childElement(s2d, 'instance_image') : undefined;
      if (inst) imageId = inst.attrs.url;
      else {
        const surf = params.get(surfSid);
        const init = surf ? childElement(childElement(surf, 'surface') ?? surf, 'init_from') : undefined;
        if (init) imageId = text(init).trim();
      }
    }
    const img = byId(imageId);
    const initFrom = img ? childElement(img, 'init_from') : undefined;
    let file = initFrom ? (childElement(initFrom, 'ref') ? text(childElement(initFrom, 'ref')) : text(initFrom)).trim() : '';
    if (file) {
      file = decodeURIComponent(file.replace(/^file:\/\/\/?/, ''));
      const data = opts.readFile(file) ?? opts.readFile(file.split(/[\\/]/).pop() ?? file);
      if (data) out.map = { data, mime: imageMime(file), name: file };
    }
  }
  return out;
}

export function parseDAE(textSrc: string, toMm = true, opts: DAEOptions = {}): MeshData {
  const root = parseXML(textSrc);
  const ids = new Map<string, XNode>();
  const index = (n: XNode) => { if (n.attrs.id && !ids.has(n.attrs.id)) ids.set(n.attrs.id, n); for (const c of n.children) if (c.type === 'element') index(c); };
  index(root);
  const byId = (id: string) => ids.get(id.replace(/^#/, ''));
  const asset = childElement(root, 'asset');
  const unit = parseFloat(childElement(asset ?? root, 'unit')?.attrs.meter ?? '1');
  const upAxis = text(childElement(asset ?? root, 'up_axis')).trim().toUpperCase();
  const scale = (toMm ? 1000 : 1) * unit;
  const buckets = new Map<string, { pos: number[]; nor: number[]; uv: number[] }>();
  const order: string[] = [];
  const bucket = (k: string) => { let b = buckets.get(k); if (!b) { b = { pos: [], nor: [], uv: [] }; buckets.set(k, b); order.push(k); } return b; };
  const geomLib = childElement(root, 'library_geometries');
  const geometries = geomLib ? childElements(geomLib, 'geometry') : [];
  const scenes = childElement(root, 'library_visual_scenes');
  const instanced = new Set<string>();
  const parsed = new Map<XNode, GeomPrim[]>();
  let anyUV = false, allNormals = true;
  const addGeom = (g: XNode, T: Mat4, binds: Map<string, string>) => {
    let prims = parsed.get(g);
    if (!prims) { prims = readGeometry(g, byId); parsed.set(g, prims); }
    for (const pr of prims) {
      const matKey = binds.get(pr.material) ?? (pr.material && byId(pr.material) ? pr.material : '');
      const b = bucket(matKey);
      if (pr.normals.length !== pr.positions.length) allNormals = false;
      for (let i = 0; i < pr.positions.length; i += 3) {
        const p = transformPoint(T, [pr.positions[i], pr.positions[i + 1], pr.positions[i + 2]]);
        b.pos.push(p[0] * scale, p[1] * scale, p[2] * scale);
        if (pr.normals.length === pr.positions.length) { const n = transformDir(T, [pr.normals[i], pr.normals[i + 1], pr.normals[i + 2]]); b.nor.push(n[0], n[1], n[2]); }
        else b.nor.push(0, 0, 0);
        if (pr.uvs.length) { b.uv.push(pr.uvs[(i / 3) * 2], pr.uvs[(i / 3) * 2 + 1]); anyUV = true; } else b.uv.push(0, 0);
      }
    }
  };
  const bindsOf = (inst: XNode): Map<string, string> => {
    const m = new Map<string, string>();
    const bm = childElement(inst, 'bind_material');
    const tc = bm ? childElement(bm, 'technique_common') : undefined;
    if (tc) for (const im of childElements(tc, 'instance_material')) m.set(im.attrs.symbol, (im.attrs.target ?? '').replace(/^#/, ''));
    return m;
  };
  const walkNode = (node: XNode, parentT: Mat4, depth: number) => {
    if (depth > 64) return;
    const T = multiply(parentT, nodeTransform(node));
    for (const c of node.children) {
      if (c.type !== 'element') continue;
      if (c.name === 'instance_geometry') { const g = byId(c.attrs.url); if (g) { instanced.add(c.attrs.url.replace(/^#/, '')); addGeom(g, T, bindsOf(c)); } }
      else if (c.name === 'instance_node') { const n = byId(c.attrs.url); if (n) walkNode(n, T, depth + 1); }
      else if (c.name === 'node') walkNode(c, T, depth + 1);
    }
  };
  if (scenes) for (const vs of childElements(scenes, 'visual_scene')) for (const n of childElements(vs, 'node')) walkNode(n, identity(), 0);
  // geometries not instanced by any scene: add at identity
  for (const g of geometries) if (!instanced.has(g.attrs.id)) addGeom(g, identity(), new Map());
  let total = 0;
  for (const b of buckets.values()) total += b.pos.length;
  const pos = new Float32Array(total);
  let nor = new Float32Array(allNormals ? total : 0);
  const uvs = anyUV ? new Float32Array((total / 3) * 2) : undefined;
  const groups: MeshGroup[] = [];
  let off = 0;
  order.forEach((k, mi) => {
    const b = buckets.get(k)!;
    pos.set(b.pos, off);
    if (allNormals) nor.set(b.nor, off);
    if (uvs) uvs.set(b.uv, (off / 3) * 2);
    groups.push({ start: off / 3, count: b.pos.length / 3, material: mi });
    off += b.pos.length;
  });
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
  const out: MeshData = { positions: pos, normals: nor, min, max, triangles: pos.length / 9 };
  if (uvs) out.uvs = uvs;
  if (order.some((k) => k !== '')) {
    out.groups = groups;
    out.materials = order.map((k) => (k ? readMaterial(k, byId, opts) : { name: '' }));
  }
  return out;
}
