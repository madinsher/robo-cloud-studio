/**
 * Collision detection (RoboDK-style collision map).
 * Shapes: capsules for procedural robot links, oriented boxes for primitives / mesh bounding boxes,
 * spheres for fruit/points. Pairs of adjacent links and parent/child tool relations are ignored.
 * Mesh-accurate checks use a triangle-vs-triangle test on the stored STL data when both items are meshes
 * and their bounding boxes overlap (bounded by a triangle budget to keep the UI responsive).
 */
import { Station, Item, ItemType, SceneObject, Tool } from '../items/item';
import { Robot } from '../items/robot';
import { MobileRobot } from '../../mobile/items';
import { Component } from '../../vc/component';
import { Mat4, multiply, transformPoint, getPos, sub, dot, cross, norm, add, scale, invert, transformDir } from '../math/pose';
import { AssetStore } from '../../scene/assets';

export type Shape =
  | { kind: 'capsule'; a: [number, number, number]; b: [number, number, number]; r: number }
  | { kind: 'obb'; center: [number, number, number]; axes: [[number, number, number], [number, number, number], [number, number, number]]; half: [number, number, number] }
  | { kind: 'sphere'; c: [number, number, number]; r: number }
  | { kind: 'mesh'; positions: Float32Array; pose: Mat4; obb: Extract<Shape, { kind: 'obb' }> };

export interface Collider {
  item: Item;
  /** Sub-part label (link name) for robots. */
  part: string;
  /** Index of the link in the robot chain (for adjacency filtering). */
  linkIndex?: number;
  /** Procedural (approximate) geometry — self-collision rules are more lenient. */
  approx?: boolean;
  shape: Shape;
}

export interface CollisionPair {
  a: Collider;
  b: Collider;
  /** Approximate penetration depth (mm). */
  depth: number;
}

export interface CollisionOptions {
  assets?: AssetStore;
  /** Safety margin added to all shapes (mm). */
  margin?: number;
  /** Check triangle meshes exactly (slower). */
  meshAccurate?: boolean;
  /** Ignore pairs (item ids). */
  ignore?: Array<[string, string]>;
  /** Restrict to items that are (or belong to) robots vs everything else. */
  robotsOnly?: boolean;
}

// ---- shape builders -------------------------------------------------------------

function obbFromBox(pose: Mat4, size: [number, number, number], localCenter: [number, number, number] = [0, 0, 0]): Extract<Shape, { kind: 'obb' }> {
  const center = transformPoint(pose, localCenter);
  return {
    kind: 'obb',
    center,
    axes: [[pose[0], pose[1], pose[2]], [pose[4], pose[5], pose[6]], [pose[8], pose[9], pose[10]]],
    half: [size[0] / 2, size[1] / 2, size[2] / 2],
  };
}

/** Colliders for one item at its current absolute pose. */
export function collidersOf(item: Item, opts: CollisionOptions = {}): Collider[] {
  const out: Collider[] = [];
  if (item instanceof Robot) {
    const fk = item.fk();
    const base = item.poseAbs();
    const chain = item.chain;
    const reach = Math.max(300, item.reach);
    const r0 = Math.max(18, reach / 22);
    const hasMeshes = chain.links.some((l) => l.visuals.length > 0);
    for (let i = 0; i < chain.links.length; i++) {
      const linkPose = multiply(base, fk.linkPoses[i]);
      const link = chain.links[i];
      if (link.visuals.length) {
        for (const v of link.visuals) out.push(...visualColliders(item, link.name, i, multiply(linkPose, v.origin), v, opts));
      } else if (!hasMeshes) {
        // capsule from the joint pivot to the link end (mirrors the procedural renderer)
        const radius = 0.85 * r0 * (1 - 0.55 * (i / Math.max(1, chain.joints.length)));
        if (i === 0) {
          const j0 = chain.joints[0];
          // start the base capsule above the mounting plane so its end cap does not sink into the pedestal/floor
          if (j0) { const rb = r0 * 1.1; const top = transformPoint(base, [j0.origin[12], j0.origin[13], j0.origin[14]]); const bot = transformPoint(base, [0, 0, Math.min(rb, Math.hypot(j0.origin[12], j0.origin[13], j0.origin[14]))]); out.push({ item, part: link.name, linkIndex: 0, approx: true, shape: { kind: 'capsule', a: bot, b: top, r: rb } }); }
          continue;
        }
        const j = chain.joints[i - 1];
        const pivot = j.post ? getPos(invert(j.post)) : [0, 0, 0] as [number, number, number];
        const a = transformPoint(linkPose, pivot);
        const b = getPos(linkPose);
        if (norm(sub(a, b)) > 1) out.push({ item, part: link.name, linkIndex: i, approx: true, shape: { kind: 'capsule', a, b, r: radius } });
        const next = chain.joints[i];
        if (next) {
          const c = transformPoint(linkPose, [next.origin[12], next.origin[13], next.origin[14]]);
          if (norm(sub(b, c)) > 1) out.push({ item, part: link.name, linkIndex: i, approx: true, shape: { kind: 'capsule', a: b, b: c, r: radius * 0.95 } });
        } else {
          const f = transformPoint(linkPose, [chain.flange[12], chain.flange[13], chain.flange[14]]);
          if (norm(sub(b, f)) > 1) out.push({ item, part: link.name, linkIndex: i, approx: true, shape: { kind: 'capsule', a: b, b: f, r: radius * 0.8 } });
        }
      }
    }
    return out;
  }
  if (item instanceof Tool) {
    const flange = item.flangeAbs();
    if (item.geometry.length) for (const g of item.geometry) out.push(...visualColliders(item, item.name, undefined, multiply(flange, Float64Array.from(g.origin ?? identity16())), g, opts));
    else {
      const tcp = getPos(item.poseAbs());
      const fl = getPos(flange);
      const len = norm(sub(tcp, fl));
      const r = 30;
      // end the capsule at the TCP (cap inside), so a tool touching a part at its TCP is not a collision
      if (len > r + 1) out.push({ item, part: item.name, shape: { kind: 'capsule', a: fl, b: add(fl, scale(sub(tcp, fl), (len - r) / len)), r } });
    }
    return out;
  }
  if (item instanceof SceneObject || item instanceof Component || item instanceof MobileRobot) {
    const pose = item.poseAbs();
    const geometry = (item as any).geometry as SceneObject['geometry'];
    if (geometry?.length) for (const g of geometry) out.push(...visualColliders(item, item.name, undefined, multiply(pose, Float64Array.from(g.origin ?? identity16())), g, opts));
    else if (item instanceof MobileRobot) {
      const [L, W, H] = item.kin.footprint;
      out.push({ item, part: item.name, shape: obbFromBox(pose, [L, W, H], [0, 0, H / 2]) });
    } else if (item instanceof SceneObject && item.bbox) {
      const size: [number, number, number] = [item.bbox.max[0] - item.bbox.min[0], item.bbox.max[1] - item.bbox.min[1], item.bbox.max[2] - item.bbox.min[2]];
      const c: [number, number, number] = [(item.bbox.max[0] + item.bbox.min[0]) / 2, (item.bbox.max[1] + item.bbox.min[1]) / 2, (item.bbox.max[2] + item.bbox.min[2]) / 2];
      out.push({ item, part: item.name, shape: obbFromBox(pose, size, c) });
    }
  }
  return out;
}

function identity16(): number[] {
  return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}

function visualColliders(item: Item, part: string, linkIndex: number | undefined, pose: Mat4, g: { mesh?: string; primitive?: any; scale?: [number, number, number] }, opts: CollisionOptions): Collider[] {
  if (g.primitive) {
    const p = g.primitive;
    if (p.kind === 'box') return [{ item, part, linkIndex, shape: obbFromBox(pose, p.size) }];
    if (p.kind === 'cylinder') {
      // long cylinders: capsule whose caps stay inside the cylinder ends; short/fat cylinders: oriented box
      if (p.length > 2 * p.radius) return [{ item, part, linkIndex, shape: { kind: 'capsule', a: transformPoint(pose, [0, 0, -p.length / 2 + p.radius]), b: transformPoint(pose, [0, 0, p.length / 2 - p.radius]), r: p.radius } }];
      return [{ item, part, linkIndex, shape: obbFromBox(pose, [2 * p.radius, 2 * p.radius, p.length]) }];
    }
    if (p.kind === 'sphere') return [{ item, part, linkIndex, shape: { kind: 'sphere', c: getPos(pose), r: p.radius } }];
    if (p.kind === 'plane') return [{ item, part, linkIndex, shape: obbFromBox(pose, [p.size[0], p.size[1], 2]) }];
    if (p.kind === 'cone') return [{ item, part, linkIndex, shape: { kind: 'capsule', a: transformPoint(pose, [0, 0, -p.length / 2]), b: transformPoint(pose, [0, 0, p.length / 2]), r: p.radius * 0.7 } }];
  }
  if (g.mesh && opts.assets) {
    const a = opts.assets.get(g.mesh) ?? [...opts.assets.entries()].find(([id]) => id.split(/[\\/]/).pop()?.toLowerCase() === g.mesh!.split(/[\\/]/).pop()?.toLowerCase())?.[1];
    if (a?.mesh) {
      const s = g.scale ?? [1, 1, 1];
      const min = a.mesh.min.map((v, i) => v * s[i]) as [number, number, number];
      const max = a.mesh.max.map((v, i) => v * s[i]) as [number, number, number];
      const size: [number, number, number] = [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
      const c: [number, number, number] = [(max[0] + min[0]) / 2, (max[1] + min[1]) / 2, (max[2] + min[2]) / 2];
      const obb = obbFromBox(pose, size, c);
      if (opts.meshAccurate && a.mesh.triangles < 400000) {
        // pre-scale positions
        let positions = a.mesh.positions;
        if (s[0] !== 1 || s[1] !== 1 || s[2] !== 1) {
          positions = new Float32Array(a.mesh.positions.length);
          for (let i = 0; i < positions.length; i += 3) { positions[i] = a.mesh.positions[i] * s[0]; positions[i + 1] = a.mesh.positions[i + 1] * s[1]; positions[i + 2] = a.mesh.positions[i + 2] * s[2]; }
        }
        return [{ item, part, linkIndex, shape: { kind: 'mesh', positions, pose, obb } }];
      }
      return [{ item, part, linkIndex, shape: obb }];
    }
  }
  return [];
}

// ---- primitive tests -------------------------------------------------------------

type V = [number, number, number];

function closestPtSegSeg(p1: V, q1: V, p2: V, q2: V): [V, V] {
  const d1 = sub(q1, p1), d2 = sub(q2, p2), r = sub(p1, p2);
  const a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
  let s = 0, t = 0;
  if (a <= 1e-9 && e <= 1e-9) return [p1, p2];
  if (a <= 1e-9) t = Math.max(0, Math.min(1, f / e));
  else {
    const c = dot(d1, r);
    if (e <= 1e-9) s = Math.max(0, Math.min(1, -c / a));
    else {
      const b = dot(d1, d2);
      const denom = a * e - b * b;
      s = denom !== 0 ? Math.max(0, Math.min(1, (b * f - c * e) / denom)) : 0;
      t = (b * s + f) / e;
      if (t < 0) { t = 0; s = Math.max(0, Math.min(1, -c / a)); }
      else if (t > 1) { t = 1; s = Math.max(0, Math.min(1, (b - c) / a)); }
    }
  }
  return [add(p1, scale(d1, s)), add(p2, scale(d2, t))];
}

function closestPtSegPoint(a: V, b: V, p: V): V {
  const ab = sub(b, a);
  const t = Math.max(0, Math.min(1, dot(sub(p, a), ab) / (dot(ab, ab) || 1)));
  return add(a, scale(ab, t));
}

function obbClosestPoint(o: Extract<Shape, { kind: 'obb' }>, p: V): V {
  const d = sub(p, o.center);
  let q = o.center;
  for (let i = 0; i < 3; i++) {
    const dist = Math.max(-o.half[i], Math.min(o.half[i], dot(d, o.axes[i])));
    q = add(q, scale(o.axes[i], dist));
  }
  return q;
}

/** OBB-OBB separating axis test; returns penetration depth (>0) or negative distance estimate. */
function obbObb(a: Extract<Shape, { kind: 'obb' }>, b: Extract<Shape, { kind: 'obb' }>): number {
  const axes: V[] = [...a.axes, ...b.axes];
  for (const ax of a.axes) for (const bx of b.axes) { const c = cross(ax, bx); if (norm(c) > 1e-6) axes.push(scale(c, 1 / norm(c))); }
  const t = sub(b.center, a.center);
  let minOverlap = Infinity;
  for (const L of axes) {
    const ra = a.half[0] * Math.abs(dot(a.axes[0], L)) + a.half[1] * Math.abs(dot(a.axes[1], L)) + a.half[2] * Math.abs(dot(a.axes[2], L));
    const rb = b.half[0] * Math.abs(dot(b.axes[0], L)) + b.half[1] * Math.abs(dot(b.axes[1], L)) + b.half[2] * Math.abs(dot(b.axes[2], L));
    const overlap = ra + rb - Math.abs(dot(t, L));
    if (overlap < 0) return overlap;
    minOverlap = Math.min(minOverlap, overlap);
  }
  return minOverlap;
}

function capsuleObb(c: Extract<Shape, { kind: 'capsule' }>, o: Extract<Shape, { kind: 'obb' }>): number {
  // approximate: sample the segment and use the closest point on the box
  let best = -Infinity;
  const n = 6;
  for (let i = 0; i <= n; i++) {
    const p = add(c.a, scale(sub(c.b, c.a), i / n)) as V;
    const q = obbClosestPoint(o, p);
    const d = norm(sub(p, q));
    // inside the box -> distance 0 -> penetration approx r + min half extent projection
    let pen = c.r - d;
    if (d < 1e-6) {
      const rel = sub(p, o.center);
      const inside = Math.min(...[0, 1, 2].map((k) => o.half[k] - Math.abs(dot(rel, o.axes[k]))));
      pen = c.r + inside;
    }
    best = Math.max(best, pen);
  }
  return best;
}

/** Signed penetration depth between two shapes (>0 => collision). */
export function shapeDistance(s1: Shape, s2: Shape): number {
  const a = s1.kind === 'mesh' ? s1.obb : s1;
  const b = s2.kind === 'mesh' ? s2.obb : s2;
  if (a.kind === 'sphere' && b.kind === 'sphere') return a.r + b.r - norm(sub(a.c, b.c));
  if (a.kind === 'capsule' && b.kind === 'capsule') { const [p, q] = closestPtSegSeg(a.a, a.b, b.a, b.b); return a.r + b.r - norm(sub(p, q)); }
  if (a.kind === 'capsule' && b.kind === 'sphere') return a.r + b.r - norm(sub(closestPtSegPoint(a.a, a.b, b.c), b.c));
  if (a.kind === 'sphere' && b.kind === 'capsule') return shapeDistance(b, a);
  if (a.kind === 'obb' && b.kind === 'obb') return obbObb(a, b);
  if (a.kind === 'capsule' && b.kind === 'obb') return capsuleObb(a, b);
  if (a.kind === 'obb' && b.kind === 'capsule') return capsuleObb(b, a);
  if (a.kind === 'sphere' && b.kind === 'obb') return a.r - norm(sub(obbClosestPoint(b, a.c), a.c));
  if (a.kind === 'obb' && b.kind === 'sphere') return shapeDistance(b, a);
  return -Infinity;
}

/** Triangle-triangle intersection (Möller) for mesh-accurate confirmation. */
function triTri(a: V[], b: V[]): boolean {
  const planeSide = (t: V[], p: V) => { const n = cross(sub(t[1], t[0]), sub(t[2], t[0])); return dot(n, sub(p, t[0])); };
  const s1 = b.map((p) => planeSide(a, p));
  if (s1.every((v) => v > 1e-9) || s1.every((v) => v < -1e-9)) return false;
  const s2 = a.map((p) => planeSide(b, p));
  if (s2.every((v) => v > 1e-9) || s2.every((v) => v < -1e-9)) return false;
  // edges of a vs triangle b and vice versa (segment-triangle tests)
  const segTri = (p: V, q: V, t: V[]) => {
    const d = sub(q, p);
    const e1 = sub(t[1], t[0]), e2 = sub(t[2], t[0]);
    const h = cross(d, e2);
    const det = dot(e1, h);
    if (Math.abs(det) < 1e-12) return false;
    const f = 1 / det;
    const s = sub(p, t[0]);
    const u = f * dot(s, h);
    if (u < 0 || u > 1) return false;
    const qv = cross(s, e1);
    const v = f * dot(d, qv);
    if (v < 0 || u + v > 1) return false;
    const tt = f * dot(e2, qv);
    return tt >= 0 && tt <= 1;
  };
  for (let i = 0; i < 3; i++) {
    if (segTri(a[i], a[(i + 1) % 3], b)) return true;
    if (segTri(b[i], b[(i + 1) % 3], a)) return true;
  }
  return false;
}

function meshMesh(m1: Extract<Shape, { kind: 'mesh' }>, m2: Extract<Shape, { kind: 'mesh' }>, budget = 20_000): boolean {
  const t1 = m1.positions.length / 9, t2 = m2.positions.length / 9;
  if (t1 * t2 > budget) return meshMeshBVH(m1, m2);
  const w1: V[][] = [], w2: V[][] = [];
  for (let i = 0; i < t1; i++) w1.push([0, 1, 2].map((k) => transformPoint(m1.pose, [m1.positions[i * 9 + k * 3], m1.positions[i * 9 + k * 3 + 1], m1.positions[i * 9 + k * 3 + 2]])) as V[]);
  for (let i = 0; i < t2; i++) w2.push([0, 1, 2].map((k) => transformPoint(m2.pose, [m2.positions[i * 9 + k * 3], m2.positions[i * 9 + k * 3 + 1], m2.positions[i * 9 + k * 3 + 2]])) as V[]);
  for (const a of w1) for (const b of w2) if (triTri(a, b)) return true;
  return false;
}

// ---- station-level check ----------------------------------------------------------

function rootOf(item: Item): Item {
  // group colliders by their owning robot/mobile robot/object for adjacency rules
  let p: Item | null = item;
  while (p && !(p instanceof Robot) && !(p instanceof MobileRobot)) p = p.parent;
  return p ?? item;
}

/** Check all collidable items of a station. */
export function checkCollisions(station: Station, opts: CollisionOptions = {}): CollisionPair[] {
  const colliders: Collider[] = [];
  for (const it of station.walk()) {
    if (it === station || !it.visible) continue;
    if (it.type === ItemType.FIELD || it.type === ItemType.CROP_ROW || it.type === ItemType.MAP || it.type === ItemType.ZONE) continue;
    colliders.push(...collidersOf(it, opts));
  }
  return checkColliders(colliders, opts);
}

export function checkColliders(colliders: Collider[], opts: CollisionOptions = {}): CollisionPair[] {
  const margin = opts.margin ?? 0;
  const ignore = new Set((opts.ignore ?? []).map(([a, b]) => `${a}|${b}`));
  const pairs: CollisionPair[] = [];
  for (let i = 0; i < colliders.length; i++) {
    for (let j = i + 1; j < colliders.length; j++) {
      const A = colliders[i], B = colliders[j];
      if (A.item === B.item) {
        // same robot: skip the same link and neighbours (approximate capsules overlap at the shoulder/wrist)
        if (A.linkIndex !== undefined && B.linkIndex !== undefined && Math.abs(A.linkIndex - B.linkIndex) <= (A.approx || B.approx ? 3 : 2)) continue;
        if (A.linkIndex === undefined || B.linkIndex === undefined) continue;
      }
      // tool vs its robot's last links; object attached to tool vs tool
      if (A.item instanceof Tool && A.item.parent === B.item && B.linkIndex !== undefined && B.linkIndex >= (B.item as Robot).chain.links.length - 2) continue;
      if (B.item instanceof Tool && B.item.parent === A.item && A.linkIndex !== undefined && A.linkIndex >= (A.item as Robot).chain.links.length - 2) continue;
      if (A.item instanceof Tool && A.item.attached.includes(B.item.id)) continue;
      if (B.item instanceof Tool && B.item.attached.includes(A.item.id)) continue;
      // robot base link vs the item it is mounted on (mobile platform / pedestal parent)
      if ((A.linkIndex === 0 && A.item.parent === B.item) || (B.linkIndex === 0 && B.item.parent === A.item)) continue;
      if (opts.robotsOnly && !(rootOf(A.item) instanceof Robot) && !(rootOf(B.item) instanceof Robot)) continue;
      if (ignore.has(`${A.item.id}|${B.item.id}`) || ignore.has(`${B.item.id}|${A.item.id}`)) continue;
      let d = shapeDistance(A.shape, B.shape) + margin;
      // self-collision with approximate capsules: shrink both radii by 30% to avoid false positives at the wrist/shoulder
      if (A.item === B.item && A.shape.kind === 'capsule' && B.shape.kind === 'capsule') d -= 0.3 * (A.shape.r + B.shape.r);
      if (d <= 0) continue;
      if (A.shape.kind === 'mesh' && B.shape.kind === 'mesh' && opts.meshAccurate && !meshMesh(A.shape, B.shape)) continue;
      pairs.push({ a: A, b: B, depth: d });
    }
  }
  return pairs;
}

/** Robot-centric check: colliders of one robot (with its tools) against the rest of the station. */
export function checkRobotCollisions(station: Station, robot: Robot, opts: CollisionOptions = {}): CollisionPair[] {
  const mine = new Set<Item>([...robot.walk()]);
  const colliders: Collider[] = [];
  for (const it of station.walk()) {
    if (it === station || !it.visible) continue;
    if (it.type === ItemType.FIELD || it.type === ItemType.CROP_ROW || it.type === ItemType.MAP || it.type === ItemType.ZONE) continue;
    colliders.push(...collidersOf(it, opts));
  }
  return checkColliders(colliders, opts).filter((p) => mine.has(p.a.item) || mine.has(p.b.item));
}

export { transformDir };

// ---- Collision map (RoboDK Collision_SetPair / setCollisionActive) -----------------------------------

export interface CollisionMapData {
  /** Global collision checking switch (COLLISION_ON / COLLISION_OFF). */
  active: boolean;
  /** Pairs explicitly disabled: [itemA, linkA(-1 = any), itemB, linkB(-1 = any)]. */
  disabled: Array<[string, number, string, number]>;
  /** Pairs explicitly enabled (override default rules such as adjacent links). */
  enabled: Array<[string, number, string, number]>;
}

export function getCollisionMap(station: Station): CollisionMapData {
  let m = station.params.collisionMap as unknown as CollisionMapData | undefined;
  if (!m || typeof m !== 'object') { m = { active: true, disabled: [], enabled: [] }; station.params.collisionMap = m as any; }
  return m;
}

export function setCollisionPair(station: Station, a: Item, b: Item, linkA = -1, linkB = -1, check = true): void {
  const m = getCollisionMap(station);
  const same = (p: [string, number, string, number]) => (p[0] === a.id && p[2] === b.id && p[1] === linkA && p[3] === linkB) || (p[0] === b.id && p[2] === a.id && p[1] === linkB && p[3] === linkA);
  m.disabled = m.disabled.filter((p) => !same(p));
  m.enabled = m.enabled.filter((p) => !same(p));
  (check ? m.enabled : m.disabled).push([a.id, linkA, b.id, linkB]);
  station.notify('collisionMap');
}

function pairState(m: CollisionMapData, A: Collider, B: Collider): 'enabled' | 'disabled' | 'default' {
  const match = (p: [string, number, string, number]) => {
    const f = (id: string, link: number, c: Collider) => id === c.item.id && (link < 0 || link === (c.linkIndex ?? -1));
    return (f(p[0], p[1], A) && f(p[2], p[3], B)) || (f(p[0], p[1], B) && f(p[2], p[3], A));
  };
  if (m.disabled.some(match)) return 'disabled';
  if (m.enabled.some(match)) return 'enabled';
  return 'default';
}

/** Station-wide check honouring the collision map. */
export function checkCollisionsMapped(station: Station, opts: CollisionOptions = {}): CollisionPair[] {
  const m = getCollisionMap(station);
  if (!m.active) return [];
  const colliders: Collider[] = [];
  for (const it of station.walk()) {
    if (it === station || !it.visible) continue;
    if (it.type === ItemType.FIELD || it.type === ItemType.CROP_ROW || it.type === ItemType.MAP || it.type === ItemType.ZONE) continue;
    colliders.push(...collidersOf(it, opts));
  }
  const pairs = checkColliders(colliders, opts).filter((p) => pairState(m, p.a, p.b) !== 'disabled');
  // explicitly enabled pairs are checked even if default rules skip them
  for (const e of m.enabled) {
    const as = colliders.filter((c) => c.item.id === e[0] && (e[1] < 0 || c.linkIndex === e[1]));
    const bs = colliders.filter((c) => c.item.id === e[2] && (e[3] < 0 || c.linkIndex === e[3]));
    for (const A of as) for (const B of bs) {
      if (pairs.some((p) => (p.a === A && p.b === B) || (p.a === B && p.b === A))) continue;
      const d = shapeDistance(A.shape, B.shape) + (opts.margin ?? 0);
      if (d > 0) pairs.push({ a: A, b: B, depth: d });
    }
  }
  return pairs;
}

/** Collision between two specific items (RoboDK Item.Collision). */
export function itemsCollide(a: Item, b: Item, opts: CollisionOptions = {}): CollisionPair[] {
  const ca = [...a.walk()].flatMap((i) => collidersOf(i, opts));
  const cb = [...b.walk()].flatMap((i) => collidersOf(i, opts));
  const out: CollisionPair[] = [];
  for (const A of ca) for (const B of cb) {
    if (A.item instanceof Tool && A.item.attached.includes(B.item.id)) continue;
    if (B.item instanceof Tool && B.item.attached.includes(A.item.id)) continue;
    let d = shapeDistance(A.shape, B.shape) + (opts.margin ?? 0);
    if (d <= 0) continue;
    if (A.shape.kind === 'mesh' && B.shape.kind === 'mesh' && opts.meshAccurate && !meshMesh(A.shape, B.shape)) continue;
    out.push({ a: A, b: B, depth: d });
  }
  return out;
}

// ---- Ray casting (RoboDK Collision_Line) ----------------------------------------------------------

function raySphere(o: V, d: V, c: V, r: number): number | null {
  const oc = sub(o, c);
  const b = dot(oc, d), cc = dot(oc, oc) - r * r;
  const disc = b * b - cc;
  if (disc < 0) return null;
  const t = -b - Math.sqrt(disc);
  return t >= 0 ? t : -b + Math.sqrt(disc) >= 0 ? 0 : null;
}
function rayCapsule(o: V, d: V, a: V, b: V, r: number): number | null {
  // sample-based: test spheres along the segment plus the endpoint spheres (adequate for mm-scale checks)
  const n = Math.max(2, Math.ceil(norm(sub(b, a)) / (r * 0.5)));
  let best: number | null = null;
  for (let i = 0; i <= n; i++) {
    const c = add(a, scale(sub(b, a), i / n)) as V;
    const t = raySphere(o, d, c, r);
    if (t !== null && (best === null || t < best)) best = t;
  }
  return best;
}
function rayObb(o: V, d: V, box: Extract<Shape, { kind: 'obb' }>): number | null {
  let tmin = 0, tmax = Infinity;
  const p = sub(box.center, o);
  for (let i = 0; i < 3; i++) {
    const e = dot(box.axes[i], p), f = dot(box.axes[i], d);
    if (Math.abs(f) > 1e-12) {
      let t1 = (e + box.half[i]) / f, t2 = (e - box.half[i]) / f;
      if (t1 > t2) [t1, t2] = [t2, t1];
      tmin = Math.max(tmin, t1); tmax = Math.min(tmax, t2);
      if (tmin > tmax) return null;
    } else if (-e - box.half[i] > 0 || -e + box.half[i] < 0) return null;
  }
  return tmin;
}
function rayTri(o: V, d: V, a: V, b: V, c: V): number | null {
  const e1 = sub(b, a), e2 = sub(c, a);
  const h = cross(d, e2);
  const det = dot(e1, h);
  if (Math.abs(det) < 1e-12) return null;
  const f = 1 / det;
  const s = sub(o, a);
  const u = f * dot(s, h);
  if (u < 0 || u > 1) return null;
  const q = cross(s, e1);
  const v = f * dot(d, q);
  if (v < 0 || u + v > 1) return null;
  const t = f * dot(e2, q);
  return t >= 0 ? t : null;
}

export interface RayHit { item: Item; part: string; point: [number, number, number]; distance: number }

/** First intersection of the segment p1->p2 (station frame, mm) with any collider. */
export function collisionLine(station: Station, p1: V, p2: V, opts: CollisionOptions = {}): RayHit | null {
  const dir = sub(p2, p1);
  const len = norm(dir);
  if (len < 1e-9) return null;
  const d = scale(dir, 1 / len) as V;
  let best: RayHit | null = null;
  for (const it of station.walk()) {
    if (it === station || !it.visible) continue;
    if (it.type === ItemType.FIELD || it.type === ItemType.CROP_ROW || it.type === ItemType.MAP || it.type === ItemType.ZONE) continue;
    for (const c of collidersOf(it, opts)) {
      let t: number | null = null;
      const s = c.shape;
      if (s.kind === 'sphere') t = raySphere(p1, d, s.c, s.r);
      else if (s.kind === 'capsule') t = rayCapsule(p1, d, s.a, s.b, s.r);
      else if (s.kind === 'obb') t = rayObb(p1, d, s);
      else if (s.kind === 'mesh') {
        const tb = rayObb(p1, d, s.obb);
        if (tb !== null && tb <= len) {
          const bvh = meshBVH(s);
          t = bvh.raycast(p1, d, len);
        }
      }
      if (t !== null && t <= len && (!best || t < best.distance)) best = { item: it, part: c.part, point: add(p1, scale(d, t)) as V, distance: t };
    }
  }
  return best;
}

// ---- Mesh BVH (triangle-accurate checks) ---------------------------------------------------------

interface BVHNode { min: V; max: V; left?: BVHNode; right?: BVHNode; tris?: number[] }

export class MeshBVH {
  root: BVHNode;
  /** World-space triangle vertices (flat, 9 per triangle). */
  world: Float64Array;
  constructor(positions: Float32Array, pose: Mat4) {
    const n = positions.length / 9;
    this.world = new Float64Array(n * 9);
    for (let i = 0; i < n * 3; i++) {
      const p = transformPoint(pose, [positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]]);
      this.world[i * 3] = p[0]; this.world[i * 3 + 1] = p[1]; this.world[i * 3 + 2] = p[2];
    }
    const idx = Array.from({ length: n }, (_, i) => i);
    this.root = this.build(idx, 0);
  }
  private bounds(tris: number[]): [V, V] {
    const min: V = [Infinity, Infinity, Infinity], max: V = [-Infinity, -Infinity, -Infinity];
    for (const t of tris) for (let k = 0; k < 3; k++) for (let a = 0; a < 3; a++) { const v = this.world[t * 9 + k * 3 + a]; if (v < min[a]) min[a] = v; if (v > max[a]) max[a] = v; }
    return [min, max];
  }
  private build(tris: number[], depth: number): BVHNode {
    const [min, max] = this.bounds(tris);
    if (tris.length <= 8 || depth > 24) return { min, max, tris };
    const ext = [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
    const axis = ext[0] >= ext[1] && ext[0] >= ext[2] ? 0 : ext[1] >= ext[2] ? 1 : 2;
    const centroid = (t: number) => (this.world[t * 9 + axis] + this.world[t * 9 + 3 + axis] + this.world[t * 9 + 6 + axis]) / 3;
    tris.sort((p, q) => centroid(p) - centroid(q));
    const mid = tris.length >> 1;
    return { min, max, left: this.build(tris.slice(0, mid), depth + 1), right: this.build(tris.slice(mid), depth + 1) };
  }
  tri(i: number): [V, V, V] {
    const w = this.world, k = i * 9;
    return [[w[k], w[k + 1], w[k + 2]], [w[k + 3], w[k + 4], w[k + 5]], [w[k + 6], w[k + 7], w[k + 8]]];
  }
  static overlap(a: BVHNode, b: BVHNode): boolean {
    return a.min[0] <= b.max[0] && a.max[0] >= b.min[0] && a.min[1] <= b.max[1] && a.max[1] >= b.min[1] && a.min[2] <= b.max[2] && a.max[2] >= b.min[2];
  }
  /** Any triangle of this mesh intersecting a triangle of the other. */
  intersects(other: MeshBVH, budget = { n: 0 }, max = 5_000_000): boolean {
    const stack: Array<[BVHNode, BVHNode]> = [[this.root, other.root]];
    while (stack.length) {
      const [a, b] = stack.pop()!;
      if (!MeshBVH.overlap(a, b)) continue;
      if (a.tris && b.tris) {
        for (const i of a.tris) for (const j of b.tris) { if (++budget.n > max) return true; if (triTri(this.tri(i), other.tri(j))) return true; }
      } else if (a.tris) { stack.push([a, b.left!], [a, b.right!]); }
      else if (b.tris) { stack.push([a.left!, b], [a.right!, b]); }
      else { stack.push([a.left!, b.left!], [a.left!, b.right!], [a.right!, b.left!], [a.right!, b.right!]); }
    }
    return false;
  }
  raycast(o: V, d: V, maxT = Infinity): number | null {
    let best: number | null = null;
    const stack: BVHNode[] = [this.root];
    const rayBox = (n: BVHNode) => {
      let tmin = 0, tmax = maxT;
      for (let a = 0; a < 3; a++) {
        if (Math.abs(d[a]) < 1e-12) { if (o[a] < n.min[a] || o[a] > n.max[a]) return false; continue; }
        let t1 = (n.min[a] - o[a]) / d[a], t2 = (n.max[a] - o[a]) / d[a];
        if (t1 > t2) [t1, t2] = [t2, t1];
        tmin = Math.max(tmin, t1); tmax = Math.min(tmax, t2);
        if (tmin > tmax) return false;
      }
      return true;
    };
    while (stack.length) {
      const n = stack.pop()!;
      if (!rayBox(n)) continue;
      if (n.tris) { for (const i of n.tris) { const [a, b, c] = this.tri(i); const t = rayTri(o, d, a, b, c); if (t !== null && t <= maxT && (best === null || t < best)) best = t; } }
      else { stack.push(n.left!, n.right!); }
    }
    return best;
  }
}

const bvhCache = new WeakMap<Float32Array, { pose: string; bvh: MeshBVH }>();
function meshBVH(s: Extract<Shape, { kind: 'mesh' }>): MeshBVH {
  const key = Array.from(s.pose).map((v) => v.toFixed(3)).join(',');
  const c = bvhCache.get(s.positions);
  if (c && c.pose === key) return c.bvh;
  const bvh = new MeshBVH(s.positions, s.pose);
  bvhCache.set(s.positions, { pose: key, bvh });
  return bvh;
}

/** Triangle-accurate mesh/mesh test using BVHs (replaces the brute-force version for large meshes). */
export function meshMeshBVH(m1: Extract<Shape, { kind: 'mesh' }>, m2: Extract<Shape, { kind: 'mesh' }>): boolean {
  return meshBVH(m1).intersects(meshBVH(m2));
}
