/**
 * Pixel-true camera sensing, shared by the browser (WebGL) and Node.
 *
 *  - sceneMeshes      the station as triangle meshes with the SAME tessellation the 3D view draws (three.js geometries:
 *                     primitives, STL/OBJ assets, procedural robot links, vehicles, orchard trunks/canopies/fruit);
 *  - rasterize        software z-buffer at pixel centres: metric depth (mm along the optical +Z), an instance-ID buffer
 *                     and a head-light shaded RGB image — the Node fallback of SceneRenderer.renderCameraBuffers;
 *  - regionsFromRender per-instance truth read from the ID buffer: exact box, mask, visible pixels, visible fraction;
 *  - firstHit         ray / segment occlusion against the same triangles (BVH for big meshes), used by detectFruit.
 *
 * Instance keys: `object:<id>`, `robot:<id>`, `tool:<id>`, `vehicle:<id>`, `component:<id>`, `field:<id>:<part>`, `ground`.
 * The browser registers its GPU implementation with setCameraRenderer; renderCamera picks it when present.
 * Camera frame: +Z forward, +X right, +Y down (studio camera item); K from cameraIntrinsics (one camera model).
 */
import * as THREE from 'three';
import { Station, Item, SceneObject, Tool, Camera as CameraItem, GeometryRef, isShown } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { MobileRobot } from '../mobile/items';
import { Component } from '../vc/component';
import { FieldItem } from '../agri/items';
import { plantPosition } from '../agri/orchard';
import { Mat4, Vec3, multiply, invert, identity, fromArray } from '../core/math/pose';
import { primitiveGeometry } from '../scene/three-geometry';
import type { AssetStore } from '../scene/assets';
import { Intrinsics, cameraIntrinsics } from './camera_model';
import type { Mask } from './models';

export type SceneMeshKind = 'object' | 'robot' | 'tool' | 'vehicle' | 'component' | 'field' | 'ground';
export interface SceneMesh {
  key: string; itemId: string; kind: SceneMeshKind;
  /** local → world (mm). */
  world: Mat4;
  /** Local triangle soup, 9 floats per triangle. */
  tris: Float32Array;
  color: [number, number, number];
  /** World AABB. */
  min: Vec3; max: Vec3;
  /** Closed surface: back faces can be skipped (same result, half the fill work). */
  closed?: boolean;
  /** Foliage (orchard canopies): a surface for depth / ID, except where `seeThrough` meshes (fruit) lie behind it —
   *  fruit stay visible through the leaves (which fruit the leaves hide is the statistical canopy model of detectFruit). */
  foliage?: boolean;
  seeThrough?: boolean;
  /** Lazily computed inverse of `world` and BVH (ray queries). */
  inv?: Mat4; bvh?: TriBVH;
}

// ---------------------------------------------------------------------------------------------
// Station → triangle meshes
// ---------------------------------------------------------------------------------------------

const triCache = new WeakMap<THREE.BufferGeometry, Float32Array>();
function trisOf(g: THREE.BufferGeometry): Float32Array {
  let t = triCache.get(g);
  if (t) return t;
  const pos = g.getAttribute('position'), idx = g.getIndex();
  const n = idx ? idx.count : pos.count;
  t = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) { const k = idx ? idx.getX(i) : i; t[i * 3] = pos.getX(k); t[i * 3 + 1] = pos.getY(k); t[i * 3 + 2] = pos.getZ(k); }
  triCache.set(g, t);
  return t;
}
const m4 = (t: THREE.Matrix4): Mat4 => fromArray(t.elements as unknown as number[]);
function hexColor(c: string | undefined, fb: [number, number, number]): [number, number, number] {
  if (!c) return fb;
  const m = /^#?([0-9a-f]{6})$/i.exec(c.trim());
  if (!m) return fb;
  const v = parseInt(m[1], 16);
  return [(v >> 16) & 255, (v >> 8) & 255, v & 255];
}
const localBounds = new WeakMap<Float32Array, number[]>();
function aabbOf(world: Mat4, tris: Float32Array): { min: Vec3; max: Vec3 } {
  let lb = localBounds.get(tris);
  if (!lb) {
    let lx0 = Infinity, ly0 = Infinity, lz0 = Infinity, hx0 = -Infinity, hy0 = -Infinity, hz0 = -Infinity;
    for (let i = 0; i < tris.length; i += 3) { const x = tris[i], y = tris[i + 1], z = tris[i + 2]; if (x < lx0) lx0 = x; if (y < ly0) ly0 = y; if (z < lz0) lz0 = z; if (x > hx0) hx0 = x; if (y > hy0) hy0 = y; if (z > hz0) hz0 = z; }
    lb = [lx0, ly0, lz0, hx0, hy0, hz0]; localBounds.set(tris, lb);
  }
  const [lx, ly, lz, hx, hy, hz] = lb;
  const min: Vec3 = [Infinity, Infinity, Infinity], max: Vec3 = [-Infinity, -Infinity, -Infinity];
  if (!(lx <= hx)) return { min: [0, 0, 0], max: [0, 0, 0] };
  for (const x of [lx, hx]) for (const y of [ly, hy]) for (const z of [lz, hz]) {
    const p = [world[0] * x + world[4] * y + world[8] * z + world[12], world[1] * x + world[5] * y + world[9] * z + world[13], world[2] * x + world[6] * y + world[10] * z + world[14]];
    for (let k = 0; k < 3; k++) { if (p[k] < min[k]) min[k] = p[k]; if (p[k] > max[k]) max[k] = p[k]; }
  }
  return { min, max };
}
function findAsset(assets: AssetStore | null | undefined, uri: string) {
  if (!assets) return undefined;
  const a = assets.get(uri);
  if (a) return a;
  const base = uri.split(/[\\/]/).pop()?.toLowerCase();
  for (const [id, x] of assets.entries()) if (id.split(/[\\/]/).pop()?.toLowerCase() === base || x.name?.toLowerCase() === base) return x;
  return undefined;
}
/** Triangles + local matrix of a GeometryRef (primitive or loaded mesh asset); null for a missing mesh (drawn as a wire placeholder). */
function geometryRefTris(r: { mesh?: string; primitive?: any; origin?: ArrayLike<number>; scale?: [number, number, number] | number[]; opacity?: number }, assets: AssetStore | null | undefined): { tris: Float32Array; local: Mat4; closed: boolean } | null {
  if ((r.opacity ?? 1) < 0.3) return null;
  let tris: Float32Array | null = null;
  if (r.mesh) { const a = findAsset(assets, r.mesh); if (a?.mesh) tris = a.mesh.positions; }
  else if (r.primitive) tris = trisOf(primitiveGeometry(r.primitive));
  if (!tris || !tris.length) return null;
  let local = r.origin && r.origin.length === 16 ? fromArray(Array.from(r.origin)) : identity();
  if (r.scale) { const s = identity(); s[0] = r.scale[0]; s[5] = r.scale[1]; s[10] = r.scale[2]; local = multiply(local, s); }
  const mirrored = !!r.scale && r.scale[0] * r.scale[1] * r.scale[2] < 0;
  return { tris, local, closed: !!r.primitive && r.primitive.kind !== 'plane' && !mirrored };
}

/** Procedural robot link pieces in the link frame — mirrors SceneRenderer.addProceduralLink (same three.js geometries). */
const procCache = new Map<string, Array<{ tris: Float32Array; local: Mat4; dark: boolean }>>();
/** The procedural shapes the 3D view draws for a robot link without meshes (link frame, mm): also the solids of the
 *  estimated inertials of the URDF export. */
export function proceduralLink(robot: Robot, linkIndex: number): Array<{ tris: Float32Array; local: Mat4; dark: boolean }> {
  const chain = robot.chain;
  const reach = Math.max(300, robot.reach);
  const key = `${robot.id}:${linkIndex}:${chain.joints.length}:${reach.toFixed(1)}:${chain.joints.map((j) => Array.from(j.origin).map((v) => v.toFixed(2)).join(',') + j.type).join('|')}:${Array.from(chain.flange).map((v) => v.toFixed(2)).join(',')}`;
  const hit = procCache.get(key);
  if (hit) return hit;
  const out: Array<{ tris: Float32Array; local: Mat4; dark: boolean }> = [];
  const n = chain.joints.length;
  const r0 = Math.max(18, reach / 22);
  const radius = r0 * (1 - 0.55 * (linkIndex / Math.max(1, n)));
  const add = (mesh: THREE.Mesh, dark: boolean) => { mesh.updateMatrix(); out.push({ tris: trisOf(mesh.geometry), local: m4(mesh.matrix), dark }); };
  const addCyl = (a: THREE.Vector3, b: THREE.Vector3, r: number) => {
    const len = a.distanceTo(b);
    if (len < 1) return;
    const cyl = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 20));
    cyl.position.copy(a).add(b).multiplyScalar(0.5);
    cyl.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), b.clone().sub(a).normalize());
    add(cyl, false);
    const s1 = new THREE.Mesh(new THREE.SphereGeometry(r, 16, 12));
    s1.position.copy(b);
    add(s1, false);
  };
  if (linkIndex === 0) {
    const j0 = chain.joints[0];
    if (j0) {
      const p = new THREE.Vector3(j0.origin[12], j0.origin[13], j0.origin[14]);
      const base = new THREE.Mesh(new THREE.CylinderGeometry(r0 * 1.6, r0 * 2, Math.max(30, r0), 28).rotateX(Math.PI / 2));
      base.position.z = Math.max(15, r0 / 2);
      add(base, true);
      addCyl(new THREE.Vector3(0, 0, 0), p, r0 * 1.1);
      const hub = new THREE.Mesh(new THREE.CylinderGeometry(r0 * 1.2, r0 * 1.2, r0 * 1.6, 24).rotateX(Math.PI / 2));
      hub.position.copy(p);
      hub.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), new THREE.Vector3(j0.axis[0], j0.axis[1], j0.axis[2]).normalize());
      add(hub, true);
    }
  } else {
    const j = chain.joints[linkIndex - 1];
    const pivot = j.post ? invert(j.post) : identity();
    const pv = new THREE.Vector3(pivot[12], pivot[13], pivot[14]);
    addCyl(pv, new THREE.Vector3(0, 0, 0), radius);
    const next = chain.joints[linkIndex];
    if (next) {
      const t = new THREE.Vector3(next.origin[12], next.origin[13], next.origin[14]);
      addCyl(new THREE.Vector3(0, 0, 0), t, radius * 0.95);
      const axisLocal = new THREE.Vector3(next.axis[0], next.axis[1], next.axis[2]).applyMatrix4(new THREE.Matrix4().extractRotation(new THREE.Matrix4().fromArray(Array.from(next.origin)))).normalize();
      const hub = new THREE.Mesh(new THREE.CylinderGeometry(radius * 1.25, radius * 1.25, radius * 2.2, 24).rotateX(Math.PI / 2));
      hub.position.copy(t);
      hub.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), axisLocal);
      add(hub, true);
    } else addCyl(new THREE.Vector3(0, 0, 0), new THREE.Vector3(chain.flange[12], chain.flange[13], chain.flange[14]), radius * 0.8);
    if (j.type === 'prismatic') {
      const len = Math.max(50, j.upper - j.lower);
      const rail = new THREE.Mesh(new THREE.BoxGeometry(radius * 1.2, radius * 1.2, len));
      const ax = new THREE.Vector3(j.axis[0], j.axis[1], j.axis[2]).normalize();
      rail.position.copy(pv).add(ax.clone().multiplyScalar(-len / 2));
      rail.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), ax);
      add(rail, true);
    }
  }
  procCache.set(key, out);
  return out;
}

const UNIT_GEOS = { trunk: new THREE.CylinderGeometry(50, 70, 1, 8).rotateX(Math.PI / 2), sphere: new THREE.SphereGeometry(0.5, 10, 8), box: new THREE.BoxGeometry(1, 1, 1), fruit: new THREE.SphereGeometry(1, 6, 5) };
function compose(x: number, y: number, z: number, yaw: number, sx: number, sy: number, sz: number): Mat4 {
  const c = Math.cos(yaw), s = Math.sin(yaw);
  return Float64Array.from([c * sx, s * sx, 0, 0, -s * sy, c * sy, 0, 0, 0, 0, sz, 0, x, y, z, 1]) as unknown as Mat4;
}

export interface SceneMeshOptions {
  assets?: AssetStore | null;
  /** Orchard trunks / canopies / ripe fruit (as the 3D view draws them). Default true. */
  fields?: boolean;
  /** Infinite-ish floor at z = 0 (key `ground`), for depth sensors. Default false. */
  ground?: boolean;
  /** Skip items (e.g. the item carrying the camera). */
  exclude?: (it: Item) => boolean;
}

function shownInView(it: Item, _station: Station): boolean {
  // the same rule as the 3D view (RoboDK: a hidden reference frame does not hide the robot / objects under it)
  return isShown(it);
}

const fieldCache = new WeakMap<FieldItem, { key: string; meshes: SceneMesh[] }>();
/** The station as triangle meshes in world coordinates (what the 3D view and the camera renders draw). */
export function sceneMeshes(station: Station, opts: SceneMeshOptions = {}): SceneMesh[] {
  const out: SceneMesh[] = [];
  const push = (key: string, itemId: string, kind: SceneMeshKind, world: Mat4, tris: Float32Array, color: [number, number, number], flags: { closed?: boolean; foliage?: boolean; seeThrough?: boolean } = {}) => { const bb = aabbOf(world, tris); out.push({ key, itemId, kind, world, tris, color, min: bb.min, max: bb.max, ...flags }); };
  const C = { closed: true };
  for (const it of station.walk()) {
    if (it === station || !shownInView(it, station) || opts.exclude?.(it)) continue;
    if (it instanceof Robot) {
      const fk = it.fk(); const base = it.poseAbs(); const chain = it.chain;
      const hasMeshes = chain.links.some((l) => l.visuals.length > 0);
      const col = hexColor(it.color, [0xe8, 0xa3, 0x3d]);
      for (let i = 0; i < chain.links.length; i++) {
        const lp = multiply(base, fk.linkPoses[i] ?? fk.linkPoses[fk.linkPoses.length - 1]);
        const link = chain.links[i];
        if (link.visuals.length) for (const v of link.visuals) { const g = geometryRefTris({ mesh: v.mesh, primitive: v.primitive, origin: v.origin, scale: v.scale }, opts.assets); if (g) push(`robot:${it.id}`, it.id, 'robot', multiply(lp, g.local), g.tris, hexColor(v.color, col), { closed: g.closed }); }
        else if (!hasMeshes) for (const p of proceduralLink(it, i)) push(`robot:${it.id}`, it.id, 'robot', multiply(lp, p.local), p.tris, p.dark ? [0x2f, 0x34, 0x3b] : col, C);
      }
    } else if (it instanceof Tool) {
      const fl = it.flangeAbs(); const col = hexColor(it.color, [0x5b, 0x64, 0x70]);
      for (const r of it.geometry) { const g = geometryRefTris(r, opts.assets); if (g) push(`tool:${it.id}`, it.id, 'tool', multiply(fl, g.local), g.tris, hexColor(r.color, col), { closed: g.closed }); }
    } else if (it instanceof SceneObject) {
      const pose = it.poseAbs(); const col = hexColor(it.color, [0x9a, 0xa3, 0xad]);
      for (const r of it.geometry) { const g = geometryRefTris(r, opts.assets); if (g) push(`object:${it.id}`, it.id, 'object', multiply(pose, g.local), g.tris, hexColor(r.color, col), { closed: g.closed }); }
    } else if (it instanceof Component) {
      const pose = it.poseAbs(); const col = hexColor(it.color, [0x6c, 0x75, 0x7d]);
      for (const r of it.geometry) { const g = geometryRefTris(r as GeometryRef, opts.assets); if (g) push(`component:${it.id}`, it.id, 'component', multiply(pose, g.local), g.tris, hexColor(r.color, col), { closed: g.closed }); }
    } else if (it instanceof MobileRobot) {
      const pose = it.poseAbs(); const col = hexColor(it.color, [0x2b, 0x8a, 0x3e]);
      if (it.geometry.length) for (const r of it.geometry) { const g = geometryRefTris(r, opts.assets); if (g) push(`vehicle:${it.id}`, it.id, 'vehicle', multiply(pose, g.local), g.tris, hexColor(r.color, col), { closed: g.closed }); }
      else {
        const [L, W, H] = it.kin.footprint;
        push(`vehicle:${it.id}`, it.id, 'vehicle', multiply(pose, compose(0, 0, H * 0.5, 0, L, W, H * 0.6)), trisOf(UNIT_GEOS.box), col, C);
        const wheelR = it.kin.wheelRadius, wheelW = Math.max(60, W * 0.12);
        const pos = it.kin.drive === 'ackermann' || it.kin.drive === 'tracked' ? [[L * 0.35, W / 2 + wheelW / 2], [L * 0.35, -W / 2 - wheelW / 2], [-L * 0.35, W / 2 + wheelW / 2], [-L * 0.35, -W / 2 - wheelW / 2]] : [[0, W / 2 + wheelW / 2], [0, -W / 2 - wheelW / 2], [L * 0.35, 0], [-L * 0.35, 0]];
        pos.forEach(([x, y], i) => {
          const r = i < 2 || it.kin.drive !== 'differential' ? wheelR : wheelR * 0.5;
          // CylinderGeometry(r, r, wheelW, 20): axis along Y, unit cylinder scaled
          push(`vehicle:${it.id}`, it.id, 'vehicle', multiply(pose, compose(x, y, r, 0, 1, 1, 1)), trisOf(cylY(r, wheelW)), [0x22, 0x26, 0x2b], C);
        });
      }
    } else if (it instanceof FieldItem && opts.fields !== false) {
      const abs = it.poseAbs(); const c = it.crop;
      // orchards hold tens of thousands of fruit: reuse the meshes while the field (pose, crop, plants, fruit state) is unchanged
      const rows = it.rows();
      let sig = 0, nPlants = 0, nFruit = 0;
      for (const r of rows) for (const p of r.plants) { nPlants++; sig += p.height * 1.3 + p.width * 0.7; for (const fr of p.fruit) { nFruit++; if (fr.picked) sig += 17 + nFruit * 1e-4; if (fr.ripe >= 0.5) sig += 0.5; } }
      const key = `${Array.from(abs).map((v) => v.toFixed(3)).join(',')}|${JSON.stringify(c)}|${it.indoor}|${rows.length}|${nPlants}|${nFruit}|${sig}`;
      const hit = fieldCache.get(it);
      if (hit && hit.key === key) { for (const m of hit.meshes) out.push(m); continue; }
      const start = out.length;
      const boxCanopy = c.training === 'trellis_2d' || c.training === 'v_trellis' || c.training === 'greenhouse_gutter';
      const fruitCol = hexColor(c.crop === 'citrus' ? '#ff922b' : c.crop === 'grape' ? '#7048e8' : '#e03131', [224, 49, 49]);
      for (const r of rows) {
        const [dx, dy] = r.direction(); const yaw = Math.atan2(dy, dx);
        for (const p of r.plants) {
          const [x, y] = plantPosition(r, p);
          const trunkH = Math.max(1, c.canopyBase);
          push(`field:${it.id}:trunk`, it.id, 'field', multiply(abs, compose(x, y, trunkH / 2, 0, 1, 1, trunkH)), trisOf(UNIT_GEOS.trunk), [0x6b, 0x4f, 0x2a], C);
          push(`field:${it.id}:canopy`, it.id, 'field', multiply(abs, compose(x, y, c.canopyBase + p.height / 2, yaw, boxCanopy ? c.plantSpacing * 0.95 : p.width, boxCanopy ? p.width * 0.6 : p.width, p.height)), trisOf(boxCanopy ? UNIT_GEOS.box : UNIT_GEOS.sphere), it.indoor ? [0x4c, 0xaf, 0x50] : [0x3f, 0x8f, 0x3a], { closed: true, foliage: true });
          for (const fr of p.fruit) {
            // every unpicked fruit is physically there (unripe ones green); the 3D view currently draws ripe fruit only
            if (fr.picked) continue;
            const lx = x + dx * fr.p[0] - dy * fr.p[1], ly = y + dy * fr.p[0] + dx * fr.p[1];
            push(`field:${it.id}:fruit`, it.id, 'field', multiply(abs, compose(lx, ly, fr.p[2], 0, fr.d / 2, fr.d / 2, fr.d / 2)), trisOf(UNIT_GEOS.fruit), fr.ripe >= 0.5 ? fruitCol : [0x8c, 0xc0, 0x3c], { closed: true, seeThrough: true });
          }
        }
      }
      fieldCache.set(it, { key, meshes: out.slice(start) });
    }
  }
  if (opts.ground) {
    const S = 1e6; // 1 km half-size floor at z = 0
    push('ground', '', 'ground', identity(), Float32Array.from([-S, -S, 0, S, -S, 0, S, S, 0, -S, -S, 0, S, S, 0, -S, S, 0]), [0x50, 0x55, 0x5c]);
  }
  return out;
}
const cylCache = new Map<string, THREE.BufferGeometry>();
function cylY(r: number, h: number): THREE.BufferGeometry {
  const k = `${r}:${h}`; let g = cylCache.get(k);
  if (!g) { g = new THREE.CylinderGeometry(r, r, h, 20); cylCache.set(k, g); }
  return g;
}

// ---------------------------------------------------------------------------------------------
// Software rasterizer (pixel centres, perspective-correct depth, near-plane clipping)
// ---------------------------------------------------------------------------------------------

export interface CameraRender {
  width: number; height: number;
  K: Intrinsics; camWorld: Mat4; near: number; far: number;
  /** Depth (mm along the optical +Z), 0 where nothing is hit within [near, far]. Row 0 = top of the image. */
  depth: Float32Array;
  /** Instance index per pixel into `keys`, -1 = background. */
  ids: Int32Array;
  keys: string[];
  rgba?: Uint8ClampedArray;
  source: 'webgl' | 'software';
  /** The meshes a software render was made from (reused for silhouettes / truth). */
  meshes?: SceneMesh[];
}

/** Rasterize meshes into depth / id (and optionally RGB) buffers for a camera with intrinsics K at camWorld. */
export function rasterize(meshes: SceneMesh[], K: Intrinsics, camWorld: Mat4, near: number, far: number, opts: { rgb?: boolean; background?: [number, number, number] } = {}): CameraRender {
  const w = Math.round(K.width), h = Math.round(K.height);
  const zbuf = new Float32Array(w * h).fill(Infinity);
  const ids = new Int32Array(w * h).fill(-1);
  const rgb = opts.rgb ? new Uint8ClampedArray(w * h * 4) : undefined;
  const keys: string[] = [], keyIdx = new Map<string, number>();
  const inv = invert(camWorld);
  for (const m of meshes) {
    if (m.foliage) continue;
    let ki = keyIdx.get(m.key);
    if (ki === undefined) { ki = keys.length; keys.push(m.key); keyIdx.set(m.key, ki); }
    rasterMesh(m, ki, K, inv, near, far, 0, 0, w, h, zbuf, ids, rgb);
  }
  // foliage alone, then merged: the canopy surface wins where it is nearer, except over see-through pixels (fruit)
  if (meshes.some((m) => m.foliage)) {
    const zf = new Float32Array(w * h).fill(Infinity), idf = new Int32Array(w * h).fill(-1);
    const rgbf = rgb ? new Uint8ClampedArray(w * h * 4) : undefined;
    for (const m of meshes) {
      if (!m.foliage) continue;
      let ki = keyIdx.get(m.key);
      if (ki === undefined) { ki = keys.length; keys.push(m.key); keyIdx.set(m.key, ki); }
      rasterMesh(m, ki, K, inv, near, far, 0, 0, w, h, zf, idf, rgbf);
    }
    const see = new Set(meshes.filter((m) => m.seeThrough).map((m) => keyIdx.get(m.key)!));
    for (let i = 0; i < w * h; i++) {
      if (idf[i] < 0 || zf[i] >= zbuf[i] || (ids[i] >= 0 && see.has(ids[i]))) continue;
      zbuf[i] = zf[i]; ids[i] = idf[i];
      if (rgb && rgbf) { rgb[i * 4] = rgbf[i * 4]; rgb[i * 4 + 1] = rgbf[i * 4 + 1]; rgb[i * 4 + 2] = rgbf[i * 4 + 2]; }
    }
  }
  const depth = new Float32Array(w * h);
  for (let i = 0; i < depth.length; i++) depth[i] = zbuf[i] === Infinity ? 0 : zbuf[i];
  if (rgb) { const bg = opts.background ?? [40, 44, 52]; for (let i = 0; i < w * h; i++) { if (ids[i] < 0) { rgb[i * 4] = bg[0]; rgb[i * 4 + 1] = bg[1]; rgb[i * 4 + 2] = bg[2]; } rgb[i * 4 + 3] = 255; } }
  return { width: w, height: h, K, camWorld, near, far, depth, ids, keys, rgba: rgb, source: 'software' };
}

const SCR = { px: new Float64Array(8), py: new Float64Array(8), pz: new Float64Array(8), su: new Float64Array(8), sv: new Float64Array(8), iz: new Float64Array(8), vx: new Float64Array(3), vy: new Float64Array(3), vz: new Float64Array(3) };
/** Core triangle loop. Buffers cover the pixel window [ox, ox+bw) × [oy, oy+bh) of the image. ki < 0: colour only. Returns fragments written. */
function rasterMesh(m: SceneMesh, ki: number, K: Intrinsics, inv: Mat4, near: number, far: number, ox: number, oy: number, bw: number, bh: number, zbuf: Float32Array, ids: Int32Array | null, rgb?: Uint8ClampedArray): number {
  // frustum cull with the world AABB
  let allNear = true, allFar = true, allL = true, allR = true, allT = true, allB = true;
  for (let c = 0; c < 8; c++) {
    const x = c & 1 ? m.max[0] : m.min[0], y = c & 2 ? m.max[1] : m.min[1], z = c & 4 ? m.max[2] : m.min[2];
    const cx = inv[0] * x + inv[4] * y + inv[8] * z + inv[12], cy = inv[1] * x + inv[5] * y + inv[9] * z + inv[13], cz = inv[2] * x + inv[6] * y + inv[10] * z + inv[14];
    if (cz >= near) allNear = false;
    if (cz <= far) allFar = false;
    if (cz > 1e-9) { const u = K.cx + (K.fx * cx) / cz, v = K.cy + (K.fy * cy) / cz; if (u >= ox) allL = false; if (u <= ox + bw) allR = false; if (v >= oy) allT = false; if (v <= oy + bh) allB = false; }
    else { allL = allR = allT = allB = false; }
  }
  if (allNear || allFar || allL || allR || allT || allB) return 0;
  const M = multiply(inv, m.world);
  const m0 = M[0], m1 = M[1], m2 = M[2], m4 = M[4], m5 = M[5], m6 = M[6], m8 = M[8], m9 = M[9], m10 = M[10], m12 = M[12], m13 = M[13], m14 = M[14];
  const t = m.tris; let written = 0;
  const { px, py, pz, su, sv, iz, vx, vy, vz } = SCR;
  const cr = m.color[0], cg = m.color[1], cb = m.color[2];
  const fx = K.fx, fy = K.fy, ccx = K.cx - ox, ccy = K.cy - oy;
  const cull = !!m.closed;
  for (let i = 0; i + 8 < t.length; i += 9) {
    let behind = 0, beyond = 0;
    for (let k = 0; k < 3; k++) {
      const x = t[i + k * 3], y = t[i + k * 3 + 1], z = t[i + k * 3 + 2];
      vx[k] = m0 * x + m4 * y + m8 * z + m12; vy[k] = m1 * x + m5 * y + m9 * z + m13; vz[k] = m2 * x + m6 * y + m10 * z + m14;
      if (vz[k] < near) behind++; if (vz[k] > far) beyond++;
    }
    if (behind === 3 || beyond === 3) continue;
    const ax = vx[1] - vx[0], ay = vy[1] - vy[0], az = vz[1] - vz[0], bx = vx[2] - vx[0], by = vy[2] - vy[0], bz = vz[2] - vz[0];
    const nx = ay * bz - az * by, ny = az * bx - ax * bz, nz = ax * by - ay * bx;
    const facing = nx * vx[0] + ny * vy[0] + nz * vz[0]; // < 0: the face looks at the camera
    if (cull && facing >= 0) continue;
    let shade = 1;
    if (rgb) { const ln = Math.hypot(nx, ny, nz) || 1, lc = Math.hypot(vx[0], vy[0], vz[0]) || 1; shade = 0.3 + 0.7 * Math.abs(facing / (ln * lc)); }
    // Sutherland–Hodgman against z = near
    let n = 0;
    if (behind === 0) { for (let k = 0; k < 3; k++) { px[k] = vx[k]; py[k] = vy[k]; pz[k] = vz[k]; } n = 3; }
    else for (let k = 0; k < 3; k++) {
      const k2 = k === 2 ? 0 : k + 1;
      const inA = vz[k] >= near, inB = vz[k2] >= near;
      if (inA) { px[n] = vx[k]; py[n] = vy[k]; pz[n] = vz[k]; n++; }
      if (inA !== inB) { const sc = (near - vz[k]) / (vz[k2] - vz[k]); px[n] = vx[k] + sc * (vx[k2] - vx[k]); py[n] = vy[k] + sc * (vy[k2] - vy[k]); pz[n] = near; n++; }
    }
    if (n < 3) continue;
    for (let k = 0; k < n; k++) { const izk = 1 / pz[k]; su[k] = ccx + fx * px[k] * izk; sv[k] = ccy + fy * py[k] * izk; iz[k] = izk; }
    for (let f = 1; f + 1 < n; f++) {
      const x0 = su[0], y0 = sv[0], x1 = su[f], y1 = sv[f], x2 = su[f + 1], y2 = sv[f + 1];
      const area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
      if (Math.abs(area) < 1e-12) continue;
      const minX = Math.max(0, Math.ceil(Math.min(x0, x1, x2) - 0.5)), maxX = Math.min(bw - 1, Math.floor(Math.max(x0, x1, x2) - 0.5));
      const minY = Math.max(0, Math.ceil(Math.min(y0, y1, y2) - 0.5)), maxY = Math.min(bh - 1, Math.floor(Math.max(y0, y1, y2) - 0.5));
      if (minX > maxX || minY > maxY) continue;
      const ia = 1 / area, iz0 = iz[0], iz1 = iz[f], iz2 = iz[f + 1];
      // barycentric weights are affine in (x, y): step them incrementally
      const w0dx = -(y2 - y1) * ia, w1dx = -(y0 - y2) * ia;
      for (let yy = minY; yy <= maxY; yy++) {
        const sy = yy + 0.5, sx0 = minX + 0.5;
        let w0 = ((x1 - sx0) * (y2 - sy) - (x2 - sx0) * (y1 - sy)) * ia;
        let w1 = ((x2 - sx0) * (y0 - sy) - (x0 - sx0) * (y2 - sy)) * ia;
        let pi = yy * bw + minX;
        for (let xx = minX; xx <= maxX; xx++, pi++, w0 += w0dx, w1 += w1dx) {
          const w2 = 1 - w0 - w1;
          // small tolerance: a pixel centre exactly on an edge shared by two triangles is filled (no cracks)
          if (w0 < -1e-9 || w1 < -1e-9 || w2 < -1e-9) continue;
          const z = 1 / (w0 * iz0 + w1 * iz1 + w2 * iz2);
          if (z > far || z >= zbuf[pi]) continue;
          zbuf[pi] = z; written++;
          if (ids && ki >= 0) ids[pi] = ki;
          if (rgb) { rgb[pi * 4] = cr * shade; rgb[pi * 4 + 1] = cg * shade; rgb[pi * 4 + 2] = cb * shade; }
        }
      }
    }
  }
  return written;
}

/** Pixels covered by these meshes ALONE (no occluders) inside a pixel window — the unoccluded silhouette area. */
export function silhouettePixels(meshes: SceneMesh[], K: Intrinsics, camWorld: Mat4, near: number, far: number, win: { x: number; y: number; w: number; h: number }): number {
  const ox = Math.max(0, Math.floor(win.x)), oy = Math.max(0, Math.floor(win.y));
  const bw = Math.min(Math.round(K.width), Math.ceil(win.x + win.w)) - ox, bh = Math.min(Math.round(K.height), Math.ceil(win.y + win.h)) - oy;
  if (bw <= 0 || bh <= 0) return 0;
  const zbuf = new Float32Array(bw * bh).fill(Infinity);
  const inv = invert(camWorld);
  for (const m of meshes) rasterMesh(m, 0, K, inv, near, far, ox, oy, bw, bh, zbuf, null);
  let n = 0; for (let i = 0; i < zbuf.length; i++) if (zbuf[i] !== Infinity) n++;
  return n;
}

/** Image-space bounds of meshes projected without occlusion (clipped to the image), or null when not in view. */
export function projectedBounds(meshes: SceneMesh[], K: Intrinsics, camWorld: Mat4, near: number, far: number): { x: number; y: number; w: number; h: number } | null {
  const inv = invert(camWorld);
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const m of meshes) {
    const M = multiply(inv, m.world), t = m.tris;
    for (let i = 0; i < t.length; i += 3) {
      const z = M[2] * t[i] + M[6] * t[i + 1] + M[10] * t[i + 2] + M[14];
      if (z < near || z > far) continue;
      const x = M[0] * t[i] + M[4] * t[i + 1] + M[8] * t[i + 2] + M[12], y = M[1] * t[i] + M[5] * t[i + 1] + M[9] * t[i + 2] + M[13];
      const u = K.cx + (K.fx * x) / z, v = K.cy + (K.fy * y) / z;
      if (u < x0) x0 = u; if (u > x1) x1 = u; if (v < y0) y0 = v; if (v > y1) y1 = v;
    }
  }
  if (!(x0 <= x1)) return null;
  const bx = Math.max(0, x0), by = Math.max(0, y0), ex = Math.min(K.width, x1), ey = Math.min(K.height, y1);
  return ex > bx && ey > by ? { x: bx, y: by, w: ex - bx, h: ey - by } : null;
}

// ---------------------------------------------------------------------------------------------
// Camera render entry point (GPU when registered, software otherwise)
// ---------------------------------------------------------------------------------------------

export interface RenderRequest {
  width?: number; height?: number;
  rgb?: boolean;
  assets?: AssetStore | null;
  fields?: boolean;
  /** Add the z = 0 floor (depth sensors). */
  ground?: boolean;
}
export type CameraBufferRenderer = (station: Station, cam: CameraItem, req: RenderRequest) => CameraRender | null;
let gpuRenderer: CameraBufferRenderer | null = null;
let defaultAssets: AssetStore | null = null;
/** Browser: SceneRenderer.renderCameraBuffers registers itself here (returns null for another station). */
export function setCameraRenderer(fn: CameraBufferRenderer | null): void { gpuRenderer = fn; }
export function cameraRendererRegistered(): boolean { return !!gpuRenderer; }
/** Mesh assets for the software path (server / headless API). */
export function setSensingAssets(assets: AssetStore | null): void { defaultAssets = assets; }

/** Depth / ID / RGB buffers of what the camera sees, at the camera resolution unless width/height are given. */
export function renderCamera(station: Station, cam: CameraItem, req: RenderRequest = {}): CameraRender {
  if (gpuRenderer) { try { const r = gpuRenderer(station, cam, req); if (r) return r; } catch { /* fall back to software */ } }
  const K = cameraIntrinsics(cam, req.width ?? cam.width, req.height ?? cam.height);
  const meshes = sceneMeshes(station, { assets: req.assets ?? defaultAssets, fields: req.fields, ground: req.ground });
  const r = rasterize(meshes, K, cam.poseAbs(), cam.near, cam.far, { rgb: req.rgb });
  r.meshes = meshes;
  return r;
}

// ---------------------------------------------------------------------------------------------
// Truth from the ID buffer
// ---------------------------------------------------------------------------------------------

export interface RenderRegion { key: string; pixels: number; box: { x: number; y: number; w: number; h: number }; mask: Mask; depthMin: number }

/** Exact per-instance boxes / masks / visible pixel counts from an ID buffer. */
export function regionsFromRender(r: CameraRender): Map<string, RenderRegion> {
  const n = r.keys.length;
  const cnt = new Int32Array(n), x0 = new Int32Array(n).fill(1 << 30), y0 = new Int32Array(n).fill(1 << 30), x1 = new Int32Array(n).fill(-1), y1 = new Int32Array(n).fill(-1);
  const dmin = new Float64Array(n).fill(Infinity);
  for (let y = 0; y < r.height; y++) for (let x = 0; x < r.width; x++) {
    const i = y * r.width + x, k = r.ids[i];
    if (k < 0) continue;
    cnt[k]++; if (x < x0[k]) x0[k] = x; if (x > x1[k]) x1[k] = x; if (y < y0[k]) y0[k] = y; if (y > y1[k]) y1[k] = y;
    const d = r.depth[i]; if (d > 0 && d < dmin[k]) dmin[k] = d;
  }
  const out = new Map<string, RenderRegion>();
  for (let k = 0; k < n; k++) {
    if (!cnt[k]) continue;
    const mw = x1[k] - x0[k] + 1, mh = y1[k] - y0[k] + 1, data = new Uint8Array(mw * mh);
    for (let y = 0; y < mh; y++) for (let x = 0; x < mw; x++) if (r.ids[(y + y0[k]) * r.width + x + x0[k]] === k) data[y * mw + x] = 1;
    out.set(r.keys[k], { key: r.keys[k], pixels: cnt[k], box: { x: x0[k], y: y0[k], w: mw, h: mh }, mask: { data, width: mw, height: mh, x0: x0[k], y0: y0[k] }, depthMin: dmin[k] });
  }
  return out;
}

// ---------------------------------------------------------------------------------------------
// Ray queries (occlusion): Möller–Trumbore with a median-split BVH for big meshes
// ---------------------------------------------------------------------------------------------

interface TriBVH { nodes: Float64Array; /* per node: minx,miny,minz,maxx,maxy,maxz */ left: Int32Array; right: Int32Array; start: Int32Array; count: Int32Array; order: Int32Array }
// BVHs live in mesh-local coordinates: shared by every SceneMesh built on the same triangle soup (sceneMeshes is
// rebuilt per frame by the sensor loops; the soups are cached per geometry)
const bvhCache = new WeakMap<Float32Array, TriBVH>();
function bvhOf(t: Float32Array): TriBVH { let b = bvhCache.get(t); if (!b) { b = buildBVH(t); bvhCache.set(t, b); } return b; }
function buildBVH(t: Float32Array): TriBVH {
  const nt = t.length / 9;
  const order = new Int32Array(nt); for (let i = 0; i < nt; i++) order[i] = i;
  const cen = new Float32Array(nt * 3);
  for (let i = 0; i < nt; i++) for (let a = 0; a < 3; a++) cen[i * 3 + a] = (t[i * 9 + a] + t[i * 9 + 3 + a] + t[i * 9 + 6 + a]) / 3;
  const nodes: number[] = [], left: number[] = [], right: number[] = [], start: number[] = [], count: number[] = [];
  const build = (s: number, e: number): number => {
    const id = left.length;
    left.push(-1); right.push(-1); start.push(s); count.push(e - s);
    const bb = [Infinity, Infinity, Infinity, -Infinity, -Infinity, -Infinity];
    for (let i = s; i < e; i++) { const o = order[i] * 9; for (let v = 0; v < 3; v++) for (let a = 0; a < 3; a++) { const x = t[o + v * 3 + a]; if (x < bb[a]) bb[a] = x; if (x > bb[a + 3]) bb[a + 3] = x; } }
    nodes.push(...bb);
    if (e - s > 8) {
      const ext = [bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]], ax = ext[0] > ext[1] ? (ext[0] > ext[2] ? 0 : 2) : (ext[1] > ext[2] ? 1 : 2);
      const seg = Array.from(order.subarray(s, e)).sort((p, q) => cen[p * 3 + ax] - cen[q * 3 + ax]);
      order.set(seg, s);
      const mid = (s + e) >> 1;
      const l = build(s, mid), r = build(mid, e);
      left[id] = l; right[id] = r; count[id] = 0;
    }
    return id;
  };
  build(0, nt);
  return { nodes: Float64Array.from(nodes), left: Int32Array.from(left), right: Int32Array.from(right), start: Int32Array.from(start), count: Int32Array.from(count), order };
}
function slab(o: ArrayLike<number>, inv: number[], min: ArrayLike<number>, max: ArrayLike<number>, tMax: number, minOff = 0, maxOff = 0): boolean {
  let t0 = 0, t1 = tMax;
  for (let a = 0; a < 3; a++) {
    let ta = (min[minOff + a] - o[a]) * inv[a], tb = (max[maxOff + a] - o[a]) * inv[a];
    if (ta > tb) { const s = ta; ta = tb; tb = s; }
    if (ta > t0) t0 = ta; if (tb < t1) t1 = tb;
    if (t0 > t1) return false;
  }
  return true;
}
function triHit(t: Float32Array, o: number, ro: number[], rd: number[]): number {
  const ax = t[o], ay = t[o + 1], az = t[o + 2];
  const e1x = t[o + 3] - ax, e1y = t[o + 4] - ay, e1z = t[o + 5] - az, e2x = t[o + 6] - ax, e2y = t[o + 7] - ay, e2z = t[o + 8] - az;
  const px = rd[1] * e2z - rd[2] * e2y, py = rd[2] * e2x - rd[0] * e2z, pz = rd[0] * e2y - rd[1] * e2x;
  const det = e1x * px + e1y * py + e1z * pz;
  if (Math.abs(det) < 1e-12) return Infinity;
  const id = 1 / det, sx = ro[0] - ax, sy = ro[1] - ay, sz = ro[2] - az;
  const u = (sx * px + sy * py + sz * pz) * id; if (u < 0 || u > 1) return Infinity;
  const qx = sy * e1z - sz * e1y, qy = sz * e1x - sx * e1z, qz = sx * e1y - sy * e1x;
  const v = (rd[0] * qx + rd[1] * qy + rd[2] * qz) * id; if (v < 0 || u + v > 1) return Infinity;
  const tt = (e2x * qx + e2y * qy + e2z * qz) * id;
  return tt > 0 ? tt : Infinity;
}
/** Nearest hit along o + t·d (t in (tMin, tMax), d need not be unit) against the meshes; null when free. */
export function firstHit(meshes: SceneMesh[], o: Vec3, d: Vec3, tMax: number, opts: { tMin?: number; skip?: (m: SceneMesh) => boolean } = {}): { t: number; mesh: SceneMesh } | null {
  let best = tMax, bestM: SceneMesh | null = null;
  const tMin = opts.tMin ?? 0;
  const invD = [1 / d[0], 1 / d[1], 1 / d[2]];
  for (const m of meshes) {
    if (opts.skip?.(m)) continue;
    if (!slab(o, invD, m.min, m.max, best)) continue;
    if (!m.inv) m.inv = invert(m.world);
    const I = m.inv;
    const ro = [I[0] * o[0] + I[4] * o[1] + I[8] * o[2] + I[12], I[1] * o[0] + I[5] * o[1] + I[9] * o[2] + I[13], I[2] * o[0] + I[6] * o[1] + I[10] * o[2] + I[14]];
    const rd = [I[0] * d[0] + I[4] * d[1] + I[8] * d[2], I[1] * d[0] + I[5] * d[1] + I[9] * d[2], I[2] * d[0] + I[6] * d[1] + I[10] * d[2]];
    const t = m.tris, nt = t.length / 9;
    const consider = (tri: number) => { const h = triHit(t, tri * 9, ro, rd); if (h > tMin && h < best) { best = h; bestM = m; } };
    if (nt <= 64) { for (let k = 0; k < nt; k++) consider(k); continue; }
    if (!m.bvh) m.bvh = bvhOf(t);
    const b = m.bvh, rinv = [1 / rd[0], 1 / rd[1], 1 / rd[2]];
    const stack = [0];
    while (stack.length) {
      const nd = stack.pop()!;
      if (!slab(ro, rinv, b.nodes, b.nodes, best, nd * 6, nd * 6 + 3)) continue;
      if (b.left[nd] < 0) { for (let k = b.start[nd]; k < b.start[nd] + b.count[nd]; k++) consider(b.order[k]); }
      else { stack.push(b.left[nd], b.right[nd]); }
    }
  }
  return bestM ? { t: best, mesh: bestM } : null;
}
