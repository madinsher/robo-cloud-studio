/**
 * Collision detection (RoboDK-style collision map).
 * Shapes: capsules for procedural robot links, oriented boxes for primitives / mesh bounding boxes,
 * spheres for fruit/points. Pairs of adjacent links and parent/child tool relations are ignored.
 * Mesh-accurate checks use local-space BVHs built once per mesh: mesh/mesh by triangle intersection, mesh/box,
 * mesh/sphere and mesh/capsule by triangle tests. Items with a separate collider list (SceneObject.colliders, e.g.
 * Webots boundingObject) use it instead of their visual geometry; tools without geometry have no collider.
 * robotCollisionChecker() is the cached robot-vs-station check used along program timelines; it honours the station
 * collision map (RoboDK Collision_SetPair / imported .rdk maps).
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
  /** Joint vector for robot-link queries without mutating the station. */
  joints?: number[];
  /** Safety margin added to all shapes (mm). */
  margin?: number;
  /** Check triangle meshes exactly (slower). */
  meshAccurate?: boolean;
  /** Ignore pairs (item ids). */
  ignore?: Array<[string, string]>;
  /** Restrict to items that are (or belong to) robots vs everything else. */
  robotsOnly?: boolean;
  /** Check only links/tools belonging to the same robot. */
  selfOnly?: boolean;
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
    const fk = item.fk(opts.joints);
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
    // a tool without geometry has no collider (RoboDK): only its meshes / primitives collide
    const flange = item.flangeAbs();
    for (const g of item.colliders ?? item.geometry) out.push(...visualColliders(item, item.name, undefined, multiply(flange, Float64Array.from(g.origin ?? identity16())), g, opts));
    return out;
  }
  if (item instanceof SceneObject || item instanceof Component || item instanceof MobileRobot) {
    const pose = item.poseAbs();
    const geometry = ((item as SceneObject).colliders ?? (item as any).geometry) as SceneObject['geometry'];
    if ((item as SceneObject).colliders && !geometry.length) return out;
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
    // exact id, else the basename index (O(1); the old linear scan per mesh dominated checks of big stations)
    const st = opts.assets as AssetStore & { resolve?: AssetStore['resolve'] };
    const a = st.get(g.mesh) ?? (typeof st.resolve === 'function' ? st.resolve(g.mesh.split('\\').pop()!) : undefined);
    if (a?.mesh) {
      const s = g.scale ?? [1, 1, 1];
      const min = a.mesh.min.map((v, i) => v * s[i]) as [number, number, number];
      const max = a.mesh.max.map((v, i) => v * s[i]) as [number, number, number];
      const size: [number, number, number] = [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
      const c: [number, number, number] = [(max[0] + min[0]) / 2, (max[1] + min[1]) / 2, (max[2] + min[2]) / 2];
      const obb = obbFromBox(pose, size, c);
      if (opts.meshAccurate && a.mesh.triangles < 400000) {
        return [{ item, part, linkIndex, shape: { kind: 'mesh', positions: scaledPositions(a.mesh.positions, s), pose, obb } }];
      }
      // A box around a CAD link is much fatter than the shell, so it is only an approximate self-check.
      return [{ item, part, linkIndex, approx: true, shape: obb }];
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
      if (skipPair(A, B, opts, ignore)) continue;
      const d = pairDepth(A, B, opts, margin);
      if (d > 0) pairs.push({ a: A, b: B, depth: d });
    }
  }
  return pairs;
}

/** SRDF <disable_collisions> pairs stored by the URDF importer on the imported root (params.disabledCollisions). */
const srdfCache = new WeakMap<object, Set<string>>();
function srdfDisabled(it: Item): Set<string> | null {
  for (let p: Item | null = it; p; p = p.parent) {
    const d = p.params?.disabledCollisions as unknown;
    if (!Array.isArray(d) || !d.length) continue;
    let set = srdfCache.get(d);
    if (!set) srdfCache.set(d, (set = new Set(d.flatMap((x: any) => (Array.isArray(x) ? [`${x[0]}|${x[1]}`, `${x[1]}|${x[0]}`] : [])))));
    return set;
  }
  return null;
}

/** Default collision rules (RoboDK-like): adjacent links, tool vs its flange links, attached parts, mounts. */
function skipPair(A: Collider, B: Collider, opts: CollisionOptions, ignore: Set<string>): boolean {
  const sd = srdfDisabled(A.item);
  if (sd && sd.has(`${A.part}|${B.part}`) && srdfDisabled(B.item) === sd) return true;
  if (A.item === B.item) {
    // same robot: skip the same link and neighbours (approximate capsules overlap at the shoulder/wrist)
    if (A.linkIndex !== undefined && B.linkIndex !== undefined) {
      const lo = Math.min(A.linkIndex, B.linkIndex), hi = Math.max(A.linkIndex, B.linkIndex);
      if (hi - lo <= 1) return true;
      // Bounding boxes of imported link shells are much fatter than the metal and overlap
      // links further along the arm. Exact triangle meshes still test those pairs.
      if (hi - lo >= 2 && A.approx && B.approx && A.shape.kind === 'obb' && B.shape.kind === 'obb') return true;
      // Procedural hubs separated only by short wrist spacers are mechanically connected.
      // Long intervening links must still be tested, even when their indices differ by two.
      if ((A.approx || B.approx) && A.item instanceof Robot) {
        let gap = 0;
        for (let k = lo; k < hi - 1; k++) {
          const j = A.item.chain.joints[k], next = A.item.chain.joints[k + 1];
          if (j?.post) gap += Math.hypot(j.post[12], j.post[13], j.post[14]);
          if (next) gap += Math.hypot(next.origin[12], next.origin[13], next.origin[14]);
        }
        const radius = (c: Collider) => c.shape.kind === 'capsule' ? c.shape.r : 0;
        if (gap <= radius(A) + radius(B)) return true;
      }
    }
    if (A.linkIndex === undefined || B.linkIndex === undefined) return true;
  }
  // tool vs its robot's last links; object attached to tool vs tool
  if (A.item instanceof Tool && A.item.parent === B.item && B.linkIndex !== undefined && B.linkIndex >= (B.item as Robot).chain.links.length - 2) return true;
  if (B.item instanceof Tool && B.item.parent === A.item && A.linkIndex !== undefined && A.linkIndex >= (A.item as Robot).chain.links.length - 2) return true;
  if (A.item instanceof Tool && A.item.attached.includes(B.item.id)) return true;
  if (B.item instanceof Tool && B.item.attached.includes(A.item.id)) return true;
  if ((A.item instanceof Tool && B.item.parent === A.item) || (B.item instanceof Tool && A.item.parent === B.item)) return true;
  if (heldBy(A.item, B.item) || heldBy(B.item, A.item)) return true;
  // robot base link vs the item it is mounted on (mobile platform / pedestal parent)
  if ((A.linkIndex === 0 && A.item.parent === B.item) || (B.linkIndex === 0 && B.item.parent === A.item)) return true;
  if (opts.selfOnly && (rootOf(A.item) !== rootOf(B.item) || !(rootOf(A.item) instanceof Robot))) return true;
  if (opts.robotsOnly && !(rootOf(A.item) instanceof Robot) && !(rootOf(B.item) instanceof Robot)) return true;
  // A box around a tool, finger or part mounted on the robot overlaps the arm. The shell usually does not.
  if ((A.approx || B.approx) && A.item !== B.item && rootOf(A.item) === rootOf(B.item) && rootOf(A.item) instanceof Robot) return true;
  if (ignore.has(`${A.item.id}|${B.item.id}`) || ignore.has(`${B.item.id}|${A.item.id}`)) return true;
  // A box around an imported tool is fatter than the shell and crosses the forearm while the triangles do not.
  const ownedTool = (A.item instanceof Tool && A.item.parent === B.item) || (B.item instanceof Tool && B.item.parent === A.item);
  if (ownedTool && (A.approx || B.approx)) return true;
  return false;
}

/**
 * A part held by a tool (attached) vs the gripper mechanism doing the holding (RoboDK gripper robots mounted with the
 * tool on the arm, 06.f RobotiQ fingers): the fingers touch the part. The arm's own links still collide with it.
 */
function heldBy(obj: Item, other: Item): boolean {
  const t = obj.parent;
  if (!(t instanceof Tool) || !t.attached.includes(obj.id) || !(other instanceof Robot) || other.dof >= 5) return false;
  return other.isAncestorOf(t) || other.parent === t.parent || (!!t.parent && t.parent.isAncestorOf(other));
}

/** Pairs never checked, whatever the collision map says: one link with itself, a part with the tool holding it. */
function hardSkip(A: Collider, B: Collider): boolean {
  if (A.item === B.item && (A.linkIndex === B.linkIndex || A.linkIndex === undefined || B.linkIndex === undefined)) return true;
  if (heldBy(A.item, B.item) || heldBy(B.item, A.item)) return true;
  if (A.item instanceof Tool && (A.item.attached.includes(B.item.id) || B.item.parent === A.item)) return true;
  if (B.item instanceof Tool && (B.item.attached.includes(A.item.id) || A.item.parent === B.item)) return true;
  return false;
}

/** Penetration estimate (> 0 = collision) with exact triangle tests for meshes when meshAccurate is set. */
function pairDepth(A: Collider, B: Collider, opts: CollisionOptions, margin = opts.margin ?? 0): number {
  let d = shapeDistance(A.shape, B.shape) + margin;
  // self-collision with approximate capsules: shrink both radii by 30% to avoid false positives at the wrist/shoulder
  if (A.item === B.item && A.shape.kind === 'capsule' && B.shape.kind === 'capsule') d -= 0.3 * (A.shape.r + B.shape.r);
  if (d <= 0) return d;
  if (opts.meshAccurate && (A.shape.kind === 'mesh' || B.shape.kind === 'mesh')) return exactHit(A.shape, B.shape, margin) ? Math.max(1, Math.min(d, 1)) : -1;
  return d;
}

/** Robot-centric check: colliders of one robot (with its tools) against the rest of the station. */
export function checkRobotCollisions(station: Station, robot: Robot, opts: CollisionOptions = {}): CollisionPair[] {
  const mine = new Set<Item>([...robot.walk()]);
  const colliders: Collider[] = [];
  for (const it of opts.selfOnly ? robot.walk() : station.walk()) {
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
  /**
   * RoboDK station collision map (imported from .rdk): the collision entities (robots one per mesh link, objects, tools)
   * and the index pairs RoboDK checks. Between two entities only these pairs are checked; explicit enabled/disabled
   * entries (setCollisionPair) override it, items outside the entity list follow the default rules.
   */
  entities?: Array<{ id: string; name?: string; link?: number }>;
  pairs?: Array<[number, number]>;
}

export function getCollisionMap(station: Station): CollisionMapData {
  let m = station.params.collisionMap as unknown as CollisionMapData | undefined;
  if (!m || typeof m !== 'object') { m = { active: true, disabled: [], enabled: [] }; station.params.collisionMap = m as any; }
  // an imported RoboDK map ({ entities, pairs }) gets the switch and override lists, keeping its pairs
  if (typeof m.active !== 'boolean') m.active = true;
  if (!Array.isArray(m.disabled)) m.disabled = [];
  if (!Array.isArray(m.enabled)) m.enabled = [];
  return m;
}

/** Entity index lookup and checked-pair set of an imported RoboDK map (cached per map object). */
const rdkMapCache = new WeakMap<object, { index: Map<string, number>; pairs: Set<number>; n: number }>();
function rdkMap(m: CollisionMapData): { index: Map<string, number>; pairs: Set<number>; n: number } | null {
  if (!Array.isArray(m.entities) || !Array.isArray(m.pairs)) return null;
  let c = rdkMapCache.get(m);
  if (!c || c.n !== m.pairs.length) {
    const index = new Map<string, number>();
    m.entities.forEach((e, i) => index.set(`${e.id}|${e.link ?? -1}`, i));
    const N = m.entities.length;
    c = { index, pairs: new Set(m.pairs.map(([i, j]) => Math.min(i, j) * N + Math.max(i, j))), n: m.pairs.length };
    rdkMapCache.set(m, c);
  }
  return c;
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
  const r = rdkMap(m);
  if (r) {
    // a robot whose RoboDK mesh links do not map 1:1 onto its chain links (KR 90 R3100: 9 meshes, 7 links, extra
    // counterbalance parts) cannot be matched link by link: its pairs follow the default rules
    const unmapped = (c: Collider) => c.item instanceof Robot && c.item.params.rdkMeshLinks !== undefined && Number(c.item.params.rdkMeshLinks) !== c.item.chain.links.length;
    const ent = (c: Collider) => (unmapped(c) ? undefined : r.index.get(`${c.item.id}|${c.linkIndex ?? -1}`) ?? (c.linkIndex !== undefined ? undefined : r.index.get(`${c.item.id}|-1`)));
    const i = ent(A), j = ent(B);
    if (i !== undefined && j !== undefined) return r.pairs.has(Math.min(i, j) * m.entities!.length + Math.max(i, j)) ? 'enabled' : 'disabled';
  }
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
      const d = pairDepth(A, B, opts);
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
    const d = pairDepth(A, B, opts);
    if (d > 0) out.push({ a: A, b: B, depth: d });
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


// ---- Local-space mesh BVH (built once per mesh, queried with relative transforms) --------------------------------

const scaledCache = new WeakMap<Float32Array, Map<string, Float32Array>>();
function scaledPositions(pos: Float32Array, s: number[]): Float32Array {
  if (s[0] === 1 && s[1] === 1 && s[2] === 1) return pos;
  const key = s.join(',');
  let m = scaledCache.get(pos);
  if (!m) scaledCache.set(pos, (m = new Map()));
  let out = m.get(key);
  if (!out) {
    out = new Float32Array(pos.length);
    for (let i = 0; i < pos.length; i += 3) { out[i] = pos[i] * s[0]; out[i + 1] = pos[i + 1] * s[1]; out[i + 2] = pos[i + 2] * s[2]; }
    m.set(key, out);
  }
  return out;
}

/** BVH over the triangles of a mesh in its own coordinates. */
export class LocalBVH {
  readonly order: Uint32Array;
  readonly min: number[] = [];
  readonly max: number[] = [];
  readonly left: number[] = [];
  readonly right: number[] = [];
  readonly start: number[] = [];
  readonly count: number[] = [];
  constructor(readonly pos: Float32Array) {
    const n = Math.floor(pos.length / 9);
    this.order = new Uint32Array(n);
    for (let i = 0; i < n; i++) this.order[i] = i;
    const cen = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) for (let a = 0; a < 3; a++) cen[i * 3 + a] = (pos[i * 9 + a] + pos[i * 9 + 3 + a] + pos[i * 9 + 6 + a]) / 3;
    if (n) this.build(0, n, cen, 0);
  }
  private build(lo: number, hi: number, cen: Float32Array, depth: number): number {
    const id = this.left.length;
    const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
    for (let k = lo; k < hi; k++) { const t = this.order[k] * 9; for (let v = 0; v < 9; v += 3) for (let a = 0; a < 3; a++) { const x = this.pos[t + v + a]; if (x < mn[a]) mn[a] = x; if (x > mx[a]) mx[a] = x; } }
    this.min.push(mn[0], mn[1], mn[2]); this.max.push(mx[0], mx[1], mx[2]);
    this.left.push(-1); this.right.push(-1); this.start.push(lo); this.count.push(hi - lo);
    if (hi - lo <= 8 || depth > 40) return id;
    const ext = [mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]];
    const axis = ext[0] >= ext[1] && ext[0] >= ext[2] ? 0 : ext[1] >= ext[2] ? 1 : 2;
    const sub = Array.from(this.order.subarray(lo, hi)).sort((p, q) => cen[p * 3 + axis] - cen[q * 3 + axis]);
    this.order.set(sub, lo);
    const mid = (lo + hi) >> 1;
    const l = this.build(lo, mid, cen, depth + 1);
    const r = this.build(mid, hi, cen, depth + 1);
    this.left[id] = l; this.right[id] = r;
    return id;
  }
  tri(i: number, out: V[]): V[] {
    const p = this.pos, k = i * 9;
    out[0] = [p[k], p[k + 1], p[k + 2]]; out[1] = [p[k + 3], p[k + 4], p[k + 5]]; out[2] = [p[k + 6], p[k + 7], p[k + 8]];
    return out;
  }
}
const localBvhCache = new WeakMap<Float32Array, LocalBVH>();
export function localBVH(pos: Float32Array): LocalBVH {
  let b = localBvhCache.get(pos);
  if (!b) localBvhCache.set(pos, (b = new LocalBVH(pos)));
  return b;
}

const xf = (m: Mat4, p: V): V => [m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12], m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13], m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14]];
/** AABB (center, half extents) of node i of a BVH after transform T. */
function xfBox(T: Mat4, mn: number[], mx: number[], i: number): [V, V] {
  const c: V = [(mn[i * 3] + mx[i * 3]) / 2, (mn[i * 3 + 1] + mx[i * 3 + 1]) / 2, (mn[i * 3 + 2] + mx[i * 3 + 2]) / 2];
  const h = [(mx[i * 3] - mn[i * 3]) / 2, (mx[i * 3 + 1] - mn[i * 3 + 1]) / 2, (mx[i * 3 + 2] - mn[i * 3 + 2]) / 2];
  const cc = xf(T, c);
  const hh: V = [0, 0, 0];
  for (let r = 0; r < 3; r++) hh[r] = Math.abs(T[r]) * h[0] + Math.abs(T[4 + r]) * h[1] + Math.abs(T[8 + r]) * h[2];
  return [cc, hh];
}
const boxOverlap = (bvh: LocalBVH, i: number, c: V, h: V, m = 0) =>
  bvh.min[i * 3] <= c[0] + h[0] + m && bvh.max[i * 3] >= c[0] - h[0] - m && bvh.min[i * 3 + 1] <= c[1] + h[1] + m && bvh.max[i * 3 + 1] >= c[1] - h[1] - m && bvh.min[i * 3 + 2] <= c[2] + h[2] + m && bvh.max[i * 3 + 2] >= c[2] - h[2] - m;

/** Triangle vs axis-aligned box centred at the origin with half extents h (separating axis test). */
function triBox(t: V[], h: V): boolean {
  for (let a = 0; a < 3; a++) { const mn = Math.min(t[0][a], t[1][a], t[2][a]), mx = Math.max(t[0][a], t[1][a], t[2][a]); if (mn > h[a] || mx < -h[a]) return false; }
  const e = [sub(t[1], t[0]), sub(t[2], t[1]), sub(t[0], t[2])];
  const n = cross(e[0], e[1]);
  const r0 = h[0] * Math.abs(n[0]) + h[1] * Math.abs(n[1]) + h[2] * Math.abs(n[2]);
  if (Math.abs(dot(n, t[0])) > r0) return false;
  const axesU: V[] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  for (const u of axesU) for (const ed of e) {
    const ax = cross(u, ed);
    if (Math.abs(ax[0]) + Math.abs(ax[1]) + Math.abs(ax[2]) < 1e-12) continue;
    const p0 = dot(t[0], ax), p1 = dot(t[1], ax), p2 = dot(t[2], ax);
    const r = h[0] * Math.abs(ax[0]) + h[1] * Math.abs(ax[1]) + h[2] * Math.abs(ax[2]);
    if (Math.min(p0, p1, p2) > r || Math.max(p0, p1, p2) < -r) return false;
  }
  return true;
}

/** Closest point on triangle abc to p (Ericson). */
function closestOnTri(p: V, a: V, b: V, c: V): V {
  const ab = sub(b, a), ac = sub(c, a), ap = sub(p, a);
  const d1 = dot(ab, ap), d2 = dot(ac, ap);
  if (d1 <= 0 && d2 <= 0) return a;
  const bp = sub(p, b), d3 = dot(ab, bp), d4 = dot(ac, bp);
  if (d3 >= 0 && d4 <= d3) return b;
  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) return add(a, scale(ab, d1 / (d1 - d3)));
  const cp = sub(p, c), d5 = dot(ab, cp), d6 = dot(ac, cp);
  if (d6 >= 0 && d5 <= d6) return c;
  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) return add(a, scale(ac, d2 / (d2 - d6)));
  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) return add(b, scale(sub(c, b), (d4 - d3) / (d4 - d3 + (d5 - d6))));
  const den = 1 / (va + vb + vc);
  return add(a, add(scale(ab, vb * den), scale(ac, vc * den)));
}

type MeshShape = Extract<Shape, { kind: 'mesh' }>;

/** Exact (triangle-level) intersection test of a mesh against another shape. */
export function exactHit(s1: Shape, s2: Shape, margin = 0): boolean {
  if (s1.kind !== 'mesh') { if (s2.kind !== 'mesh') return shapeDistance(s1, s2) + margin > 0; return exactHit(s2, s1, margin); }
  const m = s1 as MeshShape, bvh = localBVH(m.positions);
  if (!bvh.count.length) return false;
  const inv = invert(m.pose);
  const tA: V[] = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], tB: V[] = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  if (s2.kind === 'mesh') {
    const b2 = localBVH(s2.positions);
    if (!b2.count.length) return false;
    const T = multiply(inv, s2.pose); // mesh 2 -> mesh 1 coordinates
    const stack: number[] = [0, 0];
    let budget = 0;
    while (stack.length) {
      const j = stack.pop()!, i = stack.pop()!;
      const [c, h] = xfBox(T, b2.min, b2.max, j);
      if (!boxOverlap(bvh, i, c, h, margin)) continue;
      const leafA = bvh.left[i] < 0, leafB = b2.left[j] < 0;
      if (leafA && leafB) {
        for (let x = bvh.start[i]; x < bvh.start[i] + bvh.count[i]; x++) {
          bvh.tri(bvh.order[x], tA);
          for (let y = b2.start[j]; y < b2.start[j] + b2.count[j]; y++) {
            b2.tri(b2.order[y], tB);
            if (triTri(tA, [xf(T, tB[0]), xf(T, tB[1]), xf(T, tB[2])])) return true;
            // coplanar contact (a robot base standing on its pedestal) can make the traversal explode: past this many
            // triangle tests the meshes are taken as touching
            if (++budget > 250_000) return true;
          }
        }
      } else if (leafA) stack.push(i, b2.left[j], i, b2.right[j]);
      else if (leafB) stack.push(bvh.left[i], j, bvh.right[i], j);
      else stack.push(bvh.left[i], b2.left[j], bvh.left[i], b2.right[j], bvh.right[i], b2.left[j], bvh.right[i], b2.right[j]);
    }
    return false;
  }
  if (s2.kind === 'obb') {
    // box into mesh coordinates
    const c = xf(inv, s2.center);
    const ax = s2.axes.map((a) => transformDir(inv, a)) as V[];
    const half: V = [s2.half[0] + margin, s2.half[1] + margin, s2.half[2] + margin];
    const hh: V = [0, 0, 0];
    for (let r = 0; r < 3; r++) hh[r] = Math.abs(ax[0][r]) * half[0] + Math.abs(ax[1][r]) * half[1] + Math.abs(ax[2][r]) * half[2];
    const toBox = (p: V): V => { const d = sub(p, c); return [dot(d, ax[0]), dot(d, ax[1]), dot(d, ax[2])]; };
    const stack = [0];
    while (stack.length) {
      const i = stack.pop()!;
      if (!boxOverlap(bvh, i, c, hh)) continue;
      if (bvh.left[i] >= 0) { stack.push(bvh.left[i], bvh.right[i]); continue; }
      for (let x = bvh.start[i]; x < bvh.start[i] + bvh.count[i]; x++) {
        bvh.tri(bvh.order[x], tA);
        if (triBox([toBox(tA[0]), toBox(tA[1]), toBox(tA[2])], half)) return true;
      }
    }
    return false;
  }
  // sphere / capsule: points along the axis within r of a triangle
  const r = s2.r + margin;
  const a = xf(inv, s2.kind === 'sphere' ? s2.c : s2.a), b = xf(inv, s2.kind === 'sphere' ? s2.c : s2.b);
  const len = norm(sub(b, a));
  const nS = s2.kind === 'sphere' ? 0 : Math.max(1, Math.ceil(len / Math.max(1e-6, r * 0.5)));
  const pts: V[] = [];
  for (let k = 0; k <= nS; k++) pts.push(nS ? add(a, scale(sub(b, a), k / nS)) : a);
  const c: V = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2];
  const h: V = [Math.abs(b[0] - a[0]) / 2 + r, Math.abs(b[1] - a[1]) / 2 + r, Math.abs(b[2] - a[2]) / 2 + r];
  const stack = [0];
  while (stack.length) {
    const i = stack.pop()!;
    if (!boxOverlap(bvh, i, c, h)) continue;
    if (bvh.left[i] >= 0) { stack.push(bvh.left[i], bvh.right[i]); continue; }
    for (let x = bvh.start[i]; x < bvh.start[i] + bvh.count[i]; x++) {
      bvh.tri(bvh.order[x], tA);
      for (const p of pts) if (norm(sub(closestOnTri(p, tA[0], tA[1], tA[2]), p)) <= r) return true;
    }
  }
  return false;
}

interface AABB { min: V; max: V }
function shapeAABB(s: Shape): AABB {
  const b = s.kind === 'mesh' ? s.obb : s;
  if (b.kind === 'sphere') return { min: [b.c[0] - b.r, b.c[1] - b.r, b.c[2] - b.r], max: [b.c[0] + b.r, b.c[1] + b.r, b.c[2] + b.r] };
  if (b.kind === 'capsule') return { min: [Math.min(b.a[0], b.b[0]) - b.r, Math.min(b.a[1], b.b[1]) - b.r, Math.min(b.a[2], b.b[2]) - b.r], max: [Math.max(b.a[0], b.b[0]) + b.r, Math.max(b.a[1], b.b[1]) + b.r, Math.max(b.a[2], b.b[2]) + b.r] };
  const h: V = [0, 0, 0];
  for (let r = 0; r < 3; r++) h[r] = Math.abs(b.axes[0][r]) * b.half[0] + Math.abs(b.axes[1][r]) * b.half[1] + Math.abs(b.axes[2][r]) * b.half[2];
  return { min: [b.center[0] - h[0], b.center[1] - h[1], b.center[2] - h[2]], max: [b.center[0] + h[0], b.center[1] + h[1], b.center[2] + h[2]] };
}
const aabbOverlap = (a: AABB, b: AABB, m: number) => a.min[0] <= b.max[0] + m && a.max[0] >= b.min[0] - m && a.min[1] <= b.max[1] + m && a.max[1] >= b.min[1] - m && a.min[2] <= b.max[2] + m && a.max[2] >= b.min[2] - m;
const IGNORED_TYPES = new Set([ItemType.FIELD, ItemType.CROP_ROW, ItemType.MAP, ItemType.ZONE]);

/**
 * Cached collision check of one robot (its links, tools and attached parts) against itself and the station, used
 * along program timelines. Static colliders are rebuilt only when `version` changes (attach/detach/show/hide).
 * Link and object meshes are tested triangle-exactly (BVH) when an asset store is given (meshAccurate defaults on);
 * the station collision map (disabled / enabled pairs) is honoured.
 */
const shapeIds = new WeakMap<object, number>();
let shapeSeq = 0;
/** Stable id of a collider's geometry (mesh positions or the collider object) for result caching. */
function shapeId(s: Shape): string {
  const k: object = s.kind === 'mesh' ? s.positions : s;
  let id = shapeIds.get(k);
  if (id === undefined) shapeIds.set(k, (id = ++shapeSeq));
  return s.kind === 'mesh' ? `m${id}` : `p${s.kind}${id}`;
}
const samePose = (a: ArrayLike<number> | undefined, b: ArrayLike<number> | undefined): boolean => {
  if (!a || !b || a.length !== b.length) return false;
  for (let i = 0; i < a.length; i++) if (Math.abs(a[i] - b[i]) > 1e-9) return false;
  return true;
};

export function robotCollisionChecker(station: Station, robot: Robot, opts: CollisionOptions = {}): { check(version?: number, skip?: (a: Collider, b: Collider) => boolean): CollisionPair[] } {
  const o: CollisionOptions = { ...opts, meshAccurate: opts.meshAccurate ?? !!opts.assets };
  const margin = o.margin ?? 0;
  const ignore = new Set((o.ignore ?? []).map(([a, b]) => `${a}|${b}`));
  let version = NaN;
  let mine = new Set<Item>();
  let statics: Array<{ c: Collider; box: AABB }> = [];
  let movers: Item[] = [];
  const meshCache = new Map<string, { pa: number[]; pb: number[]; d: number }>();
  // items that move during a program (other robots and mechanisms, the carrier, mobile robots and everything they
  // carry) are re-evaluated at every check; the rest of the station is cached until `version` changes
  const moving = (it: Item): boolean => { for (let p: Item | null = it; p && p !== station; p = p.parent) if (p instanceof Robot || p instanceof MobileRobot) return true; return false; };
  return {
    /** `skip(a, b)`: pairs the caller does not want (e.g. meshes already in contact at the start). */
    check(v = 0, skip?: (a: Collider, b: Collider) => boolean): CollisionPair[] {
      const map = station.params.collisionMap ? getCollisionMap(station) : null;
      if (map && !map.active) return [];
      if (v !== version) {
        version = v;
        mine = new Set(robot.walk());
        statics = []; movers = [];
        if (!o.selfOnly) for (const it of station.walk()) {
          if (it === station || !it.visible || mine.has(it) || IGNORED_TYPES.has(it.type)) continue;
          if (moving(it)) { movers.push(it); continue; }
          for (const c of collidersOf(it, o)) statics.push({ c, box: shapeAABB(c.shape) });
        }
      }
      const own: Array<{ c: Collider; box: AABB }> = [];
      for (const it of mine) if (it.visible && !IGNORED_TYPES.has(it.type)) for (const c of collidersOf(it, o)) own.push({ c, box: shapeAABB(c.shape) });
      const others: Array<{ c: Collider; box: AABB }> = [...statics];
      for (const it of movers) for (const c of collidersOf(it, o)) others.push({ c, box: shapeAABB(c.shape) });
      const pairs: CollisionPair[] = [];
      const test = (A: { c: Collider; box: AABB }, B: { c: Collider; box: AABB }) => {
        const state = map ? pairState(map, A.c, B.c) : 'default';
        if (state === 'disabled') return;
        if (state === 'enabled' ? hardSkip(A.c, B.c) : skipPair(A.c, B.c, o, ignore)) return;
        if (skip && skip(A.c, B.c)) return;
        if (!aabbOverlap(A.box, B.box, margin)) return;
        // exact mesh tests are expensive: reuse the result while neither mesh moved (base link vs its pedestal, two
        // static parts of a robot standing still)
        const ma = A.c.shape.kind === 'mesh' ? A.c.shape : null, mb = B.c.shape.kind === 'mesh' ? B.c.shape : null;
        let key = '';
        if (ma || mb) {
          key = `${shapeId(A.c.shape)}|${shapeId(B.c.shape)}`;
          const hit = meshCache.get(key);
          const pa = ma ? ma.pose : (A.c.shape as any).center, pb = mb ? mb.pose : (B.c.shape as any).center;
          if (hit && samePose(hit.pa, pa) && samePose(hit.pb, pb)) { if (hit.d > 0) pairs.push({ a: A.c, b: B.c, depth: hit.d }); return; }
          const d = pairDepth(A.c, B.c, o, margin);
          meshCache.set(key, { pa: pa ? Array.from(pa as ArrayLike<number>) : [], pb: pb ? Array.from(pb as ArrayLike<number>) : [], d });
          if (d > 0) pairs.push({ a: A.c, b: B.c, depth: d });
          return;
        }
        const d = pairDepth(A.c, B.c, o, margin);
        if (d > 0) pairs.push({ a: A.c, b: B.c, depth: d });
      };
      for (let i = 0; i < own.length; i++) for (let j = i + 1; j < own.length; j++) test(own[i], own[j]);
      for (const A of own) for (const B of others) test(A, B);
      return pairs;
    },
  };
}
