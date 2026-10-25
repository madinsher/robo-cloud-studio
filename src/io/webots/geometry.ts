/**
 * Webots geometry -> studio GeometryRef: Pose/Transform/Group flattening (metres -> mm, axis-angle -> Mat4, innermost
 * scale into GeometryRef.scale or primitive sizes, sheared meshes baked), Shape/CadShape with PBRAppearance /
 * Appearance colours and ImageTexture on meshes, primitives (Box, Cylinder (Z axis), Sphere, Cone, Capsule, Plane),
 * Mesh / CadShape files (STL, OBJ+MTL, COLLADA with materials and textures) registered in the AssetStore,
 * IndexedFaceSet / ElevationGrid generated meshes, boundingObject -> collision geometry.
 */
import { GeometryRef } from '../../core/items/item';
import { Mat4, Vec3, identity, multiply, transl, rotAxis, transformPoint, transformDir } from '../../core/math/pose';
import { MeshData, MeshMaterial, parseSTL, imageMime } from '../mesh/stl';
import { parseOBJ } from '../mesh/obj';
import { parseDAE } from '../mesh/dae';
import type { AssetStore } from '../../scene/assets';
import { WNode, fNum, fNum1, fStr, fStrs, fNode, fNodes, fBool } from './vrml';
import { ProtoRegistry, WebotsSource, resolveUrl, isRemote } from './proto';

export const M2MM = 1000;

export interface BuildCtx {
  reg: ProtoRegistry;
  src: WebotsSource;
  assets?: AssetStore;
  report: string[];
  loadMeshes: boolean;
  meshes: Map<string, { id: string; mesh: MeshData | null }>;
  warned: Set<string>;
}

export function warnOnce(ctx: BuildCtx, msg: string): void {
  if (ctx.warned.has(msg)) return;
  ctx.warned.add(msg);
  ctx.report.push(msg);
}

/** Rigid pose of a node (translation m -> mm, rotation axis-angle). */
export function poseOf(n: WNode | null | undefined): Mat4 {
  const t = fNum(n, 'translation', [0, 0, 0]);
  const r = fNum(n, 'rotation', [0, 0, 1, 0]);
  const T = transl(t[0] * M2MM, t[1] * M2MM, (t[2] ?? 0) * M2MM);
  if (r.length >= 4 && Math.abs(r[3]) > 1e-12 && Math.hypot(r[0], r[1], r[2]) > 1e-12) return multiply(T, rotAxis([r[0], r[1], r[2]], r[3]));
  return T;
}

export function scaleMat(s: number[]): Mat4 {
  const m = identity();
  m[0] = s[0]; m[5] = s[1]; m[10] = s[2];
  return m;
}

/** Split an affine matrix into a rigid origin and per-axis scale; `shear` when the axes are not orthogonal. */
export function decompose(M: Mat4): { origin: Mat4; scale: Vec3; shear: boolean } {
  const c = [[M[0], M[1], M[2]], [M[4], M[5], M[6]], [M[8], M[9], M[10]]];
  const s = c.map((v) => Math.hypot(v[0], v[1], v[2])) as Vec3;
  const r = c.map((v, i) => v.map((x) => x / (s[i] || 1)));
  const dot = (a: number[], b: number[]) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const shear = Math.abs(dot(r[0], r[1])) > 1e-4 || Math.abs(dot(r[0], r[2])) > 1e-4 || Math.abs(dot(r[1], r[2])) > 1e-4;
  const det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) - r[1][0] * (r[0][1] * r[2][2] - r[0][2] * r[2][1]) + r[2][0] * (r[0][1] * r[1][2] - r[0][2] * r[1][1]);
  if (det < 0) { s[2] = -s[2]; r[2] = r[2].map((x) => -x); }
  const o = identity();
  for (let i = 0; i < 3; i++) { o[i * 4] = r[i][0]; o[i * 4 + 1] = r[i][1]; o[i * 4 + 2] = r[i][2]; }
  o[12] = M[12]; o[13] = M[13]; o[14] = M[14];
  return { origin: o, scale: s, shear };
}

export function rigidOf(M: Mat4): Mat4 {
  return decompose(M).origin;
}

const hex2 = (x: number) => Math.round(Math.max(0, Math.min(1, x)) * 255).toString(16).padStart(2, '0');
export const rgbHex = (c: number[]) => `#${hex2(c[0])}${hex2(c[1])}${hex2(c[2])}`;

interface Look { color?: string; rgb?: [number, number, number]; opacity?: number; texture?: { url: string; base: string }; hidden?: boolean }

/** Appearance (PBRAppearance, Appearance{Material}, appearance PROTO) -> colour / opacity / texture. */
export function appearanceOf(app: WNode | null, ctx: BuildCtx): Look {
  if (!app) return { color: '#e6e6e6', rgb: [0.9, 0.9, 0.9] };
  const a = ctx.reg.resolve(app);
  if (!a) return { color: '#9aa3ad' };
  const texOf = (t: WNode | null): Look['texture'] => {
    const tn = t ? ctx.reg.resolve(t) : null;
    if (!tn || tn.type !== 'ImageTexture') return undefined;
    const url = fStrs(tn, 'url')[0];
    return url ? { url, base: tn.base } : undefined;
  };
  if (a.type === 'PBRAppearance') {
    const c = fNum(a, 'baseColor', [1, 1, 1]);
    const tr = fNum1(a, 'transparency', 0);
    return { color: rgbHex(c), rgb: [c[0], c[1], c[2]], ...(tr > 0 ? { opacity: 1 - tr } : {}), ...(tr >= 0.999 ? { hidden: true } : {}), texture: texOf(fNode(a, 'baseColorMap')) };
  }
  if (a.type === 'Appearance') {
    const m = fNode(a, 'material');
    const mat = m ? ctx.reg.resolve(m) : null;
    const c = mat ? fNum(mat, 'diffuseColor', [0.8, 0.8, 0.8]) : [1, 1, 1];
    const tr = mat ? fNum1(mat, 'transparency', 0) : 0;
    return { color: rgbHex(c), rgb: [c[0], c[1], c[2]], ...(tr > 0 ? { opacity: 1 - tr } : {}), ...(tr >= 0.999 ? { hidden: true } : {}), texture: texOf(fNode(a, 'texture')) };
  }
  warnOnce(ctx, `appearance ${a.type} not supported, grey used`);
  return { color: '#9aa3ad' };
}

function extOf(p: string): string {
  return (p.split('?')[0].split('.').pop() ?? '').toLowerCase();
}

/** Load (once) and register a mesh file referenced by a node. Returns the asset id (resolved path). */
export function loadMesh(url: string, base: string, ctx: BuildCtx): { id: string; mesh: MeshData | null } | null {
  const wanted = resolveUrl(url, base);
  if (isRemote(wanted)) return loadRemoteMesh(ctx.reg.canonicalUrl(wanted), ctx);
  const key = wanted.toLowerCase();
  const cached = ctx.meshes.get(key);
  if (cached) return cached;
  const found = ctx.reg.findFile(wanted, false) ?? ctx.reg.findFile(wanted, true);
  if (!found) { warnOnce(ctx, `mesh ${url} (from ${base}) not found`); const r = { id: wanted, mesh: null }; ctx.meshes.set(key, r); return r; }
  if (found.toLowerCase() !== key) warnOnce(ctx, `mesh ${url} resolved by name to ${found}`);
  const res = { id: found, mesh: null as MeshData | null };
  ctx.meshes.set(key, res);
  if (!ctx.loadMeshes) return res;
  const existing = ctx.assets?.get(found)?.mesh;
  if (existing) { res.mesh = existing; return res; }
  const bytes = ctx.src.read(found);
  if (!bytes) return res;
  const readRel = (rel: string) => ctx.src.read(resolveUrl(rel, found)) ?? (() => { const f = ctx.reg.findFile(resolveUrl(rel, found)); return f ? ctx.src.read(f) : undefined; })();
  return parseMeshInto(res, found, bytes, readRel, ctx);
}

/** A mesh referenced by a remote PROTO (standard Webots objects / robots): read through the registry's remote fetcher. */
function loadRemoteMesh(url: string, ctx: BuildCtx): { id: string; mesh: MeshData | null } | null {
  const key = url.toLowerCase();
  const cached = ctx.meshes.get(key);
  if (cached) return cached;
  const res = { id: url, mesh: null as MeshData | null };
  ctx.meshes.set(key, res);
  if (!ctx.loadMeshes) return res;
  const existing = ctx.assets?.get(url)?.mesh;
  if (existing) { res.mesh = existing; return res; }
  const bytes = ctx.reg.fetchRemote(url);
  if (!bytes) { warnOnce(ctx, `mesh ${url}: remote file not available offline`); return null; }
  return parseMeshInto(res, url, bytes, (rel) => ctx.reg.fetchRemote(resolveUrl(rel, url)), ctx);
}

function parseMeshInto(res: { id: string; mesh: MeshData | null }, found: string, bytes: Uint8Array, readRel: (rel: string) => Uint8Array | undefined, ctx: BuildCtx): { id: string; mesh: MeshData | null } {
  try {
    const e = extOf(found);
    if (e === 'stl') res.mesh = parseSTL(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) as ArrayBuffer, 1);
    else if (e === 'obj') res.mesh = parseOBJ(new TextDecoder().decode(bytes), 1, { readFile: readRel });
    else if (e === 'dae') res.mesh = parseDAE(new TextDecoder().decode(bytes), false, { readFile: readRel });
    else warnOnce(ctx, `mesh ${found}: .${e} files are not supported`);
  } catch (err) {
    warnOnce(ctx, `mesh ${found}: ${(err as Error).message}`);
  }
  if (res.mesh && ctx.assets) ctx.assets.registerMesh(found, res.mesh, found.split('/').pop(), 'm');
  return res;
}

let genCounter = 0;
function hashNums(a: ArrayLike<number>, seed = 2166136261): string {
  let h = seed >>> 0;
  for (let i = 0; i < a.length; i++) { h ^= Math.round(a[i] * 1e5) | 0; h = Math.imul(h, 16777619) >>> 0; }
  return h.toString(36);
}

function registerGenerated(kind: string, base: string, mesh: MeshData, ctx: BuildCtx): string {
  const id = `${base}#${kind}-${hashNums(mesh.positions)}`;
  if (ctx.assets && !ctx.assets.has(id)) ctx.assets.registerMesh(id, mesh, `${kind}${++genCounter}`, 'm');
  ctx.meshes.set(id.toLowerCase(), { id, mesh });
  return id;
}

function finishMesh(pos: number[], uv?: number[]): MeshData {
  const positions = new Float32Array(pos);
  const normals = new Float32Array(pos.length);
  const min: [number, number, number] = [Infinity, Infinity, Infinity], max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < pos.length; i += 9) {
    const ux = pos[i + 3] - pos[i], uy = pos[i + 4] - pos[i + 1], uz = pos[i + 5] - pos[i + 2], vx = pos[i + 6] - pos[i], vy = pos[i + 7] - pos[i + 1], vz = pos[i + 8] - pos[i + 2];
    let nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const l = Math.hypot(nx, ny, nz) || 1; nx /= l; ny /= l; nz /= l;
    for (let k = 0; k < 3; k++) { normals[i + k * 3] = nx; normals[i + k * 3 + 1] = ny; normals[i + k * 3 + 2] = nz; }
  }
  for (let i = 0; i < pos.length; i += 3) for (let a = 0; a < 3; a++) { if (pos[i + a] < min[a]) min[a] = pos[i + a]; if (pos[i + a] > max[a]) max[a] = pos[i + a]; }
  const m: MeshData = { positions, normals, min, max, triangles: pos.length / 9 };
  if (uv && uv.length === (pos.length / 3) * 2) m.uvs = new Float32Array(uv);
  return m;
}

/** IndexedFaceSet (coord, coordIndex with -1 separators, texCoord, ccw) -> MeshData in metres. */
export function indexedFaceSetMesh(n: WNode, ctx: BuildCtx): MeshData | null {
  const coord = fNode(n, 'coord');
  const c = coord ? ctx.reg.resolve(coord) : null;
  const pts = c ? fNum(c, 'point', []) : [];
  const idx = fNum(n, 'coordIndex', []);
  if (!pts.length || !idx.length) return null;
  const tcN = fNode(n, 'texCoord');
  const tc = tcN ? ctx.reg.resolve(tcN) : null;
  const tpts = tc ? fNum(tc, 'point', []) : [];
  const tidx = fNum(n, 'texCoordIndex', []);
  const ccw = fBool(n, 'ccw', true);
  const pos: number[] = [], uv: number[] = [];
  let face: number[] = [], tface: number[] = [];
  const flush = () => {
    for (let k = 1; k + 1 < face.length; k++) {
      const tri = ccw ? [0, k, k + 1] : [0, k + 1, k];
      for (const t of tri) {
        const i = face[t];
        pos.push(pts[i * 3] ?? 0, pts[i * 3 + 1] ?? 0, pts[i * 3 + 2] ?? 0);
        const ti = tface.length ? tface[t] : i;
        uv.push(tpts[ti * 2] ?? 0, tpts[ti * 2 + 1] ?? 0);
      }
    }
    face = []; tface = [];
  };
  let fi = 0;
  for (let i = 0; i < idx.length; i++) {
    if (idx[i] < 0) { flush(); fi++; continue; }
    face.push(idx[i]);
    if (tidx.length) tface.push(tidx[i] >= 0 ? tidx[i] : 0);
  }
  flush();
  void fi;
  return finishMesh(pos, tpts.length ? uv : undefined);
}

/** ElevationGrid (xDimension x yDimension heights, spacing; XY grid, heights along Z) -> MeshData in metres. */
export function elevationGridMesh(n: WNode): MeshData | null {
  const nx = fNum1(n, 'xDimension', 0), ny = fNum1(n, 'yDimension', 0);
  const sx = fNum1(n, 'xSpacing', 1), sy = fNum1(n, 'ySpacing', 1);
  const h = fNum(n, 'height', []);
  if (nx < 2 || ny < 2) return null;
  const z = (i: number, j: number) => h[j * nx + i] ?? 0;
  const pos: number[] = [];
  for (let j = 0; j + 1 < ny; j++) for (let i = 0; i + 1 < nx; i++) {
    const p = (a: number, b: number) => [a * sx, b * sy, z(a, b)];
    const a = p(i, j), b = p(i + 1, j), c = p(i + 1, j + 1), d = p(i, j + 1);
    pos.push(...a, ...b, ...c, ...a, ...c, ...d);
  }
  return finishMesh(pos);
}

/** Texture coordinates by projection onto the two largest extents of the bounding box (Webots IndexedFaceSet default). */
export function boxProjectUVs(m: MeshData): Float32Array {
  const ext = [0, 1, 2].map((a) => m.max[a] - m.min[a]);
  const [u, v] = [0, 1, 2].sort((a, b) => ext[b] - ext[a]).slice(0, 2).sort((a, b) => a - b);
  const n = m.positions.length / 3;
  const out = new Float32Array(n * 2);
  for (let i = 0; i < n; i++) {
    out[i * 2] = (m.positions[i * 3 + u] - m.min[u]) / (ext[u] || 1);
    out[i * 2 + 1] = (m.positions[i * 3 + v] - m.min[v]) / (ext[v] || 1);
  }
  return out;
}

/** Bake an affine (sheared) transform into a copy of a mesh (positions in metres). */
function bakeMesh(m: MeshData, L: Mat4): MeshData {
  const pos: number[] = [];
  for (let i = 0; i < m.positions.length; i += 3) { const p = transformPoint(L, [m.positions[i], m.positions[i + 1], m.positions[i + 2]]); pos.push(p[0], p[1], p[2]); }
  const out = finishMesh(pos, m.uvs ? Array.from(m.uvs) : undefined);
  if (m.groups) out.groups = m.groups;
  if (m.materials) out.materials = m.materials;
  return out;
}

/** Geometry node at affine transform M (mm) -> GeometryRefs (colour/opacity optional). */
export function geometryRefs(geomNode: WNode | null, M: Mat4, look: Look | null, ctx: BuildCtx, useFileMaterials = false): GeometryRef[] {
  if (!geomNode) return [];
  const g = ctx.reg.resolve(geomNode);
  if (!g) return [];
  const { origin, scale, shear } = decompose(M);
  const out: GeometryRef[] = [];
  const style = (r: GeometryRef): GeometryRef => {
    if (look?.color && !useFileMaterials) r.color = look.color;
    if (look?.opacity !== undefined && look.opacity < 1) r.opacity = look.opacity;
    return r;
  };
  const o = Array.from(origin);
  const a = scale.map(Math.abs) as Vec3;
  if (shear && !['Mesh', 'CadShape', 'IndexedFaceSet', 'ElevationGrid'].includes(g.type)) warnOnce(ctx, `${g.type}: sheared transform approximated`);
  const uniformXY = (r: number) => { if (Math.abs(a[0] - a[1]) > 1e-3 * Math.max(a[0], a[1])) warnOnce(ctx, `${g.type}: non-uniform XY scale approximated`); return r * Math.max(a[0], a[1]); };
  switch (g.type) {
    case 'Box': { const s = fNum(g, 'size', [0.1, 0.1, 0.1]); out.push(style({ primitive: { kind: 'box', size: [s[0] * M2MM * a[0], s[1] * M2MM * a[1], s[2] * M2MM * a[2]] }, origin: o })); break; }
    case 'Cylinder': out.push(style({ primitive: { kind: 'cylinder', radius: uniformXY(fNum1(g, 'radius', 0.05) * M2MM), length: fNum1(g, 'height', 0.1) * M2MM * a[2] }, origin: o })); break;
    case 'Sphere': { const r = fNum1(g, 'radius', 0.1) * M2MM; if (Math.max(...a) - Math.min(...a) > 1e-3 * Math.max(...a)) warnOnce(ctx, 'Sphere: non-uniform scale approximated'); out.push(style({ primitive: { kind: 'sphere', radius: r * Math.max(...a) }, origin: o })); break; }
    case 'Cone': out.push(style({ primitive: { kind: 'cone', radius: uniformXY(fNum1(g, 'bottomRadius', 0.05) * M2MM), length: fNum1(g, 'height', 0.1) * M2MM * a[2] }, origin: o })); break;
    case 'Capsule': {
      const r = uniformXY(fNum1(g, 'radius', 0.05) * M2MM), h = fNum1(g, 'height', 0.1) * M2MM * a[2];
      out.push(style({ primitive: { kind: 'cylinder', radius: r, length: h }, origin: o }));
      for (const z of [h / 2, -h / 2]) out.push(style({ primitive: { kind: 'sphere', radius: r }, origin: Array.from(multiply(origin, transl(0, 0, z))) }));
      break;
    }
    case 'Plane': { const s = fNum(g, 'size', [1, 1]); out.push(style({ primitive: { kind: 'plane', size: [s[0] * M2MM * a[0], s[1] * M2MM * a[1]] }, origin: o })); break; }
    case 'Mesh': case 'CadShape': {
      const url = fStrs(g, 'url')[0];
      if (!url) { warnOnce(ctx, `${g.type} without url`); break; }
      if (g.type === 'Mesh' && (fStr(g, 'name', '') || fNum1(g, 'materialIndex', -1) >= 0)) warnOnce(ctx, `Mesh ${url}: sub-mesh selection (name/materialIndex) ignored, whole file used`);
      const m = loadMesh(url, g.base, ctx);
      if (!m) break;
      let id = m.id;
      let mesh = m.mesh;
      if (shear && mesh) { const L = identity(); for (let i = 0; i < 12; i++) if (i % 4 !== 3) L[i] = M[i]; mesh = bakeMesh(mesh, L); id = registerGenerated('baked', m.id, mesh, ctx); out.push(style({ mesh: id, origin: Array.from(transl(M[12], M[13], M[14])), scale: [M2MM, M2MM, M2MM] })); break; }
      const ref: GeometryRef = { mesh: id, origin: o, scale: [scale[0] * M2MM, scale[1] * M2MM, scale[2] * M2MM] };
      if (look?.texture && !useFileMaterials) {
        const texPath = resolveUrl(look.texture.url, look.texture.base);
        const f = isRemote(texPath) ? ctx.reg.canonicalUrl(texPath) : ctx.reg.findFile(texPath);
        const img = f ? (isRemote(f) ? ctx.reg.fetchRemote(f) : ctx.src.read(f)) : undefined;
        if (img && mesh && !mesh.uvs) {
          mesh = { ...mesh, uvs: boxProjectUVs(mesh) };
          warnOnce(ctx, `texture ${look.texture.url} on ${url}: the mesh has no texture coordinates, generated by bounding-box projection`);
        }
        if (img && mesh?.uvs) {
          const tid = `${id}#tex=${f}`;
          if (ctx.assets && !ctx.assets.has(tid)) {
            const mat: MeshMaterial = { name: f, color: look.rgb ?? [1, 1, 1], map: { data: img, mime: imageMime(f!), name: f } };
            if (look.opacity !== undefined) mat.opacity = look.opacity;
            ctx.assets.registerMesh(tid, { ...mesh, groups: [{ start: 0, count: mesh.positions.length / 3, material: 0 }], materials: [mat] }, `${f!.split('/').pop()}`, 'm');
          }
          ref.mesh = tid;
          if (look.opacity !== undefined && look.opacity < 1) ref.opacity = look.opacity;
          out.push(ref);
          break;
        }
        if (ctx.loadMeshes) warnOnce(ctx, `texture ${look.texture.url} on ${url}: ${img ? 'mesh has no texture coordinates' : 'image not found'}, base colour used`);
      }
      out.push(style(ref));
      break;
    }
    case 'IndexedFaceSet': case 'ElevationGrid': {
      const mesh = g.type === 'IndexedFaceSet' ? indexedFaceSetMesh(g, ctx) : elevationGridMesh(g);
      if (!mesh) { warnOnce(ctx, `${g.type}: empty geometry`); break; }
      const id = registerGenerated(g.type, g.base, shear ? bakeMesh(mesh, (() => { const L = identity(); for (let i = 0; i < 12; i++) if (i % 4 !== 3) L[i] = M[i]; return L; })()) : mesh, ctx);
      out.push(style(shear ? { mesh: id, origin: Array.from(transl(M[12], M[13], M[14])), scale: [M2MM, M2MM, M2MM] } : { mesh: id, origin: o, scale: [scale[0] * M2MM, scale[1] * M2MM, scale[2] * M2MM] }));
      break;
    }
    case 'IndexedLineSet': case 'PointSet': warnOnce(ctx, `${g.type} is not rendered`); break;
    default: warnOnce(ctx, `geometry ${g.type} not supported`);
  }
  return out;
}

export interface WalkHandlers {
  /** Called for nodes that are not grouping/shape nodes (Solids, joints, devices...). Return true when handled. */
  other?(n: WNode, M: Mat4): boolean;
}

/** Collect the visual geometry under `nodes` (relative to the frame M maps from); other nodes go to `h.other`. */
export function walkShapes(nodes: WNode[], M: Mat4, ctx: BuildCtx, out: GeometryRef[], h: WalkHandlers = {}, depth = 0): void {
  if (depth > 64) { warnOnce(ctx, 'node nesting too deep'); return; }
  for (const raw of nodes) {
    const n = ctx.reg.resolve(raw);
    if (!n) continue;
    switch (n.type) {
      case 'Pose': walkShapes(fNodes(n, 'children'), multiply(M, poseOf(n)), ctx, out, h, depth + 1); break;
      case 'Transform': walkShapes(fNodes(n, 'children'), multiply(multiply(M, poseOf(n)), scaleMat(fNum(n, 'scale', [1, 1, 1]))), ctx, out, h, depth + 1); break;
      case 'Group': case 'Billboard': walkShapes(fNodes(n, 'children'), M, ctx, out, h, depth + 1); break;
      case 'Slot': { const ep = fNode(n, 'endPoint'); if (ep) walkShapes([ep], M, ctx, out, h, depth + 1); break; } // pass-through plug
      case 'Shape': {
        const look = appearanceOf(fNode(n, 'appearance'), ctx);
        out.push(...geometryRefs(fNode(n, 'geometry'), M, look, ctx));
        break;
      }
      case 'CadShape': out.push(...geometryRefs(n, M, null, ctx, true)); break;
      default:
        if (!h.other || !h.other(n, M)) warnOnce(ctx, `${n.type} ignored here (${n.base})`);
    }
  }
}

/** boundingObject -> collision GeometryRefs; Plane bounds are reported through `ground`. */
export function boundingRefs(bo: WNode | null, M: Mat4, ctx: BuildCtx, ground?: { plane: boolean }, depth = 0): GeometryRef[] {
  if (!bo || depth > 64) return [];
  const n = ctx.reg.resolve(bo);
  if (!n) return [];
  switch (n.type) {
    case 'Pose': return fNodes(n, 'children').flatMap((c) => boundingRefs(c, multiply(M, poseOf(n)), ctx, ground, depth + 1));
    case 'Transform': return fNodes(n, 'children').flatMap((c) => boundingRefs(c, multiply(multiply(M, poseOf(n)), scaleMat(fNum(n, 'scale', [1, 1, 1]))), ctx, ground, depth + 1));
    case 'Group': return fNodes(n, 'children').flatMap((c) => boundingRefs(c, M, ctx, ground, depth + 1));
    case 'Shape': return boundingRefs(fNode(n, 'geometry'), M, ctx, ground, depth + 1);
    case 'Plane': if (ground) ground.plane = true; return [];
    case 'Solid': case 'Robot': warnOnce(ctx, `${n.type} inside a boundingObject ignored`); return [];
    default: return geometryRefs(n, M, null, ctx);
  }
}

/** Direction helper used by lights / viewpoint. */
export function dirMm(v: number[]): Vec3 {
  return [v[0], v[1], v[2]];
}
export { transformDir };
