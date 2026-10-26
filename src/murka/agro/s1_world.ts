/**
 * World model of benchmark S1 on the imported orchard_demo.wbt (agromarca article/BENCHMARK_SCENARIO.md §2), and the
 * motion / contact layer of the S1 runner:
 *
 *  - rows: the 8 three-tree rows of the four sectors (upgradeOrchard fields), each harvested from its OUTER lane (row
 *    line ± rowSpacing/2 = 1.5 m, i.e. the 3 m aisle centre MURKA measured 0.18 m clearance in), entered from the open
 *    corridor between the west and east blocks (heading west for x < 0 blocks, east for the others), one stop per
 *    tree (robot centre abreast of the trunk), left by reversing to the lane entrance;
 *  - planning: Hybrid A* with Reeds–Shepp shots (src/mobile/hybrid_astar.ts, the UTP-G turning radius 2.19 m / tan 0.3
 *    = 7.08 m at the rear axle — body x −1.095 m, the proto origin is the wheelbase midpoint —, reverse allowed) on an occupancy map rasterised from the world's boundingObjects in the platform height
 *    band + crown discs of the trees + footprints of objects without colliders (collection point pad, depot shelf and
 *    modules); in-lane motion is straight along the lane axis;
 *  - legs: constant-speed playback of a pose path by the arc length of its reference point (the rear axle of a UTP-G;
 *    §9: constant speeds on route segments), interpolated along circular arcs between the planner's samples, so the
 *    rear wheels roll without side slip;
 *  - contacts: the studio collision module on the world's 3D colliders (trunks, fences, buildings, rocks, crates, CP2),
 *    the crowns' measured radial profile against the chassis collision boxes, robot–robot 3D colliders and the
 *    footprints of collider-less obstacles.
 */
import { Station, Item, ItemType, SceneObject, GeometryRef } from '../../core/items/item';
import { Mat4, fromArray, identity, multiply, transformPoint } from '../../core/math/pose';
import { MobileRobot } from '../../mobile/items';
import { FieldItem, CropRow } from '../../agri/items';
import { CollisionWorld, CircleObstacle, coveringCircles, rectIntersectsPolygon, shiftPosePath } from '../../mobile/footprint';
import { planHybridAStar } from '../../mobile/hybrid_astar';
import { itemsCollide } from '../../core/collision/collision';
import type { AssetStore } from '../../scene/assets';
import { TreeProfile, crownRadiusIn } from './orchard';

export interface Pose { x: number; y: number; th: number; z?: number }
const wrap180 = (a: number) => ((((a + 180) % 360) + 360) % 360) - 180;
const DEG = Math.PI / 180;

export interface S1Row {
  /** r1..r8 (sector order, then y). */
  id: string;
  index: number;
  sector: string;
  rowItem: CropRow;
  trees: Array<{ x: number; y: number }>;
  /** Lane axis (y, mm), heading into the lane (deg) and the lane entrance pose (outside the block). */
  laneY: number;
  heading: number;
  entrance: Pose;
  /** Robot-centre stops abreast of each tree, in harvest order. */
  stops: Pose[];
}

/** Arc length between two poses of a constant-curvature sample (chord / sinc of half the heading change). */
function arcLength(a: number[], b: number[]): number {
  const c = Math.hypot(b[0] - a[0], b[1] - a[1]);
  const h = (wrap180(b[2] - a[2]) * DEG) / 2;
  return Math.abs(h) > 1e-9 && Math.abs(h) < Math.PI / 4 ? (c * h) / Math.sin(h) : c;
}
/** Pose at fraction f of the circular arc from a to b (through both points, heading a[2] → b[2]; a line when straight). */
function arcPose(a: number[], b: number[], f: number): [number, number, number] {
  const dth = wrap180(b[2] - a[2]) * DEG;
  const dx = b[0] - a[0], dy = b[1] - a[1], c = Math.hypot(dx, dy);
  const th = a[2] + (dth / DEG) * f;
  if (Math.abs(dth) < 1e-9 || c < 1e-9 || Math.abs(dth) >= Math.PI / 2) return [a[0] + dx * f, a[1] + dy * f, th];
  // sub-chord of the arc: length c·sin(f·Δθ/2)/sin(Δθ/2), direction (chord direction − Δθ/2 + f·Δθ/2)
  const l = (c * Math.sin((f * dth) / 2)) / Math.sin(dth / 2);
  const ang = Math.atan2(dy, dx) - dth / 2 + (f * dth) / 2;
  return [a[0] + l * Math.cos(ang), a[1] + l * Math.sin(ang), th];
}

/**
 * A leg: pose path (x, y, th, dir) of the robot origin driven at constant speed starting at t0. The speed, the length
 * and the interpolation refer to the point at body x = `refX` (the rear axle of a car-like platform, 0 otherwise),
 * which moves along circular arcs between the samples: it has no side slip.
 */
export class Leg {
  readonly s: number[] = [0];
  readonly length: number;
  readonly duration: number;
  /** Reference-point poses of `points`. */
  private readonly ref: number[][];
  constructor(readonly robot: string, readonly points: number[][], readonly speed: number, readonly t0: number, readonly kind: string, readonly z0?: number, readonly z1?: number, readonly vz = 1000, readonly refX = 0) {
    this.ref = shiftPosePath(points, refX);
    for (let i = 1; i < points.length; i++) this.s.push(this.s[i - 1] + arcLength(this.ref[i - 1], this.ref[i]));
    this.length = this.s[this.s.length - 1];
    const tz = z0 !== undefined && z1 !== undefined ? Math.abs(z1 - z0) / vz : 0;
    this.duration = this.length / speed + tz;
  }
  get t1(): number { return this.t0 + this.duration; }
  get end(): Pose { const p = this.points[this.points.length - 1]; return { x: p[0], y: p[1], th: wrap180(p[2]), z: this.z1 ?? this.z0 }; }
  poseAt(t: number): Pose {
    const tz = this.z0 !== undefined && this.z1 !== undefined ? Math.abs(this.z1 - this.z0) / this.vz : 0;
    const u = Math.max(0, Math.min(this.duration, t - this.t0));
    // vertical legs: climb / descend first (takeoff), horizontal motion otherwise
    if (tz > 0 && this.length === 0) { const f = tz ? u / tz : 1; const p = this.points[0]; return { x: p[0], y: p[1], th: wrap180(p[2]), z: this.z0! + (this.z1! - this.z0!) * f }; }
    const d = Math.min(this.length, u * this.speed);
    let lo = 0, hi = this.s.length - 1;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (this.s[m] <= d) lo = m; else hi = m; }
    const a = this.ref[lo], b = this.ref[Math.min(hi, this.points.length - 1)];
    const seg = this.s[hi] - this.s[lo];
    const f = seg > 1e-9 ? (d - this.s[lo]) / seg : 0;
    const [rx, ry, th] = arcPose(a, b, f);
    const c = Math.cos(th * DEG), sn = Math.sin(th * DEG);
    return { x: rx - this.refX * c, y: ry - this.refX * sn, th: wrap180(th), z: this.z1 ?? this.z0 };
  }
  /** Max curvature (1/mm) of a ground leg (heading change per arc length over 300 mm windows). */
  maxCurvature(window = 300): number {
    let k = 0;
    for (let i = 0, j = 0; i < this.points.length; i++) {
      while (j < this.points.length - 1 && this.s[j] - this.s[i] < window) j++;
      const ds = this.s[j] - this.s[i];
      if (ds < window * 0.5) continue;
      if (this.points.slice(i, j + 1).some((p, q, arr) => q > 0 && p[3] !== arr[q - 1][3])) continue; // across a cusp
      k = Math.max(k, Math.abs(wrap180(this.points[j][2] - this.points[i][2])) * DEG / ds);
    }
    return k;
  }
}

export interface S1WorldOptions {
  profile: TreeProfile;
  /** Lane offset from the row line (mm, default rowSpacing / 2). */
  laneOffset?: number;
  /** Extra clearance between the robot front and the crown at the lane entrance (mm). */
  entranceGap?: number;
  /** Planning margin (mm). */
  margin?: number;
  assets?: AssetStore;
  /** Robots that never move during the run (obstacles for the planner). */
  parked?: MobileRobot[];
  /** Items excluded from the static obstacles (carried crates). */
  movable?: Item[];
  /** Ground platform footprint used for lanes / entrances (mm): L, W, H. */
  footprint: [number, number, number];
}

const collidersOf = (it: Item): GeometryRef[] => (it as unknown as { colliders?: GeometryRef[] }).colliders ?? [];

/** 2-D bounding polygon (rectangle) of an item's visual geometry in the station frame, with its height range. */
function footprintPoly(it: SceneObject): { poly: number[][]; z0: number; z1: number } | null {
  const P = it.poseAbs();
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity, z0 = Infinity, z1 = -Infinity;
  const visit = (o: Item, T: Mat4) => {
    for (const g of (o as SceneObject).geometry ?? []) {
      const pr = g.primitive; if (!pr) continue;
      const h: [number, number, number] = pr.kind === 'box' ? [pr.size[0] / 2, pr.size[1] / 2, pr.size[2] / 2] : pr.kind === 'sphere' ? [pr.radius, pr.radius, pr.radius] : pr.kind === 'plane' ? [pr.size[0] / 2, pr.size[1] / 2, 0] : [pr.radius, pr.radius, pr.length / 2];
      const M = multiply(T, g.origin ? fromArray(g.origin) : identity());
      for (let c = 0; c < 8; c++) {
        const q = transformPoint(M, [c & 1 ? h[0] : -h[0], c & 2 ? h[1] : -h[1], c & 4 ? h[2] : -h[2]]);
        x0 = Math.min(x0, q[0]); x1 = Math.max(x1, q[0]); y0 = Math.min(y0, q[1]); y1 = Math.max(y1, q[1]); z0 = Math.min(z0, q[2]); z1 = Math.max(z1, q[2]);
      }
    }
    for (const c of o.children) visit(c, multiply(T, c.pose()));
  };
  visit(it, P);
  if (!Number.isFinite(x0)) return null;
  return { poly: [[x0, y0], [x1, y0], [x1, y1], [x0, y1]], z0, z1 };
}

/** 2-D outlines (convex hulls of the projected corners) of an item's colliders whose height range meets [zMin, zMax]. */
export function colliderOutlines(it: Item, zMin: number, zMax: number): number[][][] {
  const out: number[][][] = [];
  const visit = (o: Item, T: Mat4) => {
    for (const g of collidersOf(o)) {
      const pr = g.primitive; if (!pr) continue;
      const M = multiply(T, g.origin ? fromArray(g.origin) : identity());
      const pts: number[][] = []; let z0 = Infinity, z1 = -Infinity;
      const add = (q: number[]) => { pts.push([q[0], q[1]]); z0 = Math.min(z0, q[2]); z1 = Math.max(z1, q[2]); };
      if (pr.kind === 'cylinder' || pr.kind === 'cone') {
        const r = pr.radius / Math.cos(Math.PI / 16);
        for (let k = 0; k < 16; k++) { const a = (k / 16) * 2 * Math.PI; for (const z of [-pr.length / 2, pr.length / 2]) add(transformPoint(M, [r * Math.cos(a), r * Math.sin(a), z])); }
      } else {
        const h = pr.kind === 'box' ? [pr.size[0] / 2, pr.size[1] / 2, pr.size[2] / 2] : pr.kind === 'sphere' ? [pr.radius, pr.radius, pr.radius] : [pr.size[0] / 2, pr.size[1] / 2, 0];
        for (let c = 0; c < 8; c++) add(transformPoint(M, [c & 1 ? h[0] : -h[0], c & 2 ? h[1] : -h[1], c & 4 ? h[2] : -h[2]]));
      }
      if (z1 >= zMin && z0 <= zMax) out.push(hull(pts));
    }
    for (const c of o.children) visit(c, multiply(T, c.pose()));
  };
  visit(it, it.poseAbs());
  return out;
}

/** S1 world on an imported orchard_demo station. */
export class S1World {
  readonly rows: S1Row[] = [];
  readonly trees: SceneObject[];
  readonly planWorld: CollisionWorld;
  /** Collider-less obstacles (≥ 30 mm high): polygons the platforms must not cover (collection point pad, shelf…). */
  readonly flat: Array<{ name: string; poly: number[][] }> = [];
  /** Static items with 3-D colliders (trunks, fences, base, depot, rocks, crates, CP2). */
  readonly staticItems: SceneObject[] = [];
  /** 2-D outlines of the static colliders (trees excepted) in the platform height band. */
  readonly polys: Array<{ name: string; poly: number[][] }> = [];
  readonly cpPad: SceneObject | null;
  readonly unloadPose: Pose;
  readonly laneOffset: number;
  private cache = new Map<string, number[][] | null>();
  planCalls = 0; planMs = 0; readonly planFailures: string[] = []; readonly planLog: Array<{ ms: number; ok: boolean; from: Pose; to: Pose; extra: number }> = [];
  constructor(readonly station: Station, readonly o: S1WorldOptions) {
    const movable = new Set((o.movable ?? []).map((i) => i.id));
    this.trees = station.children.filter((c): c is SceneObject => c instanceof SceneObject && /tree/i.test(c.name));
    for (const c of station.children) {
      if (!(c instanceof SceneObject) || movable.has(c.id) || c.params.ground) continue;
      if (collidersOf(c).length) { this.staticItems.push(c); continue; }
      const fp = footprintPoly(c);
      if (fp && fp.z1 - fp.z0 >= 30 && fp.z1 >= 30) this.flat.push({ name: c.name, poly: fp.poly });
    }
    const zTop = Math.max(1700, o.footprint[2]);
    for (const it of this.staticItems) if (!/tree/i.test(it.name)) for (const poly of colliderOutlines(it, 20, zTop)) this.polys.push({ name: it.name, poly });
    this.cpPad = (station.children.find((c) => c.name === 'collection_point_1') as SceneObject) ?? null;
    const fields = station.itemsOfType<FieldItem>(ItemType.FIELD);
    const rowSpacing = fields[0]?.crop.rowSpacing ?? 3000;
    this.laneOffset = o.laneOffset ?? rowSpacing / 2;
    const allTrees = this.trees.map((t) => ({ x: t.poseAbs()[12], y: t.poseAbs()[13] }));
    const crownR = o.profile.crownRadius;
    // planning world: crown discs of the trees (they cover the trunks); static outlines and collider-less obstacles
    // are exact blocked polygons (the occupancy-map check of the planner is conservative by ~W/2 at the footprint ends)
    const circles: CircleObstacle[] = allTrees.map((t) => ({ x: t.x, y: t.y, r: crownR, zMin: 0, zMax: o.profile.crownTop, kind: 'crown' }));
    this.planWorld = new CollisionWorld({ circles, ignoreCropCells: false });
    const midX = (() => { const xs = [...new Set(allTrees.map((t) => Math.round(t.x)))].sort((a, b) => a - b); let best = 0, gx = 0; for (let i = 1; i < xs.length; i++) if (xs[i] - xs[i - 1] > best) { best = xs[i] - xs[i - 1]; gx = (xs[i] + xs[i - 1]) / 2; } return gx; })();
    const [L, W] = o.footprint;
    const gap = o.entranceGap ?? 350;
    const m = o.margin ?? 60;
    const blocked = this.blocked();
    let idx = 0;
    // rows of 3 trees, sectors in order; lower row → south lane, upper row → north lane (outer aisles); the inner aisle
    // between the two rows of a sector when the outer one is obstructed (the collection point pad south of sector 3)
    for (const f of [...fields].sort((a, b) => a.name.localeCompare(b.name))) {
      const rows = f.rows().filter((r) => r.plants.length === 3).sort((a, b) => a.start[1] - b.start[1]);
      let innerUsed = false;
      rows.forEach((row, k) => {
        const abs = row.parent ? row.parent.poseAbs() : null;
        const tw = row.plants.map((p) => { const [lx, ly] = row.pointAt(p.s); return abs ? { x: abs[0] * lx + abs[4] * ly + abs[12], y: abs[1] * lx + abs[5] * ly + abs[13] } : { x: lx, y: ly }; });
        const y = tw[0].y;
        const west = Math.max(...tw.map((t) => t.x)) < midX;
        const heading = west ? 180 : 0;
        const order = [...tw].sort((a, b) => (west ? b.x - a.x : a.x - b.x));
        const ex = west ? order[0].x + crownR + L / 2 + gap : order[0].x - crownR - L / 2 - gap;
        const outer = k === 0 ? y - this.laneOffset : y + this.laneOffset;
        const inner = k === 0 ? y + this.laneOffset : y - this.laneOffset;
        const corridorFree = (ly: number) => {
          const xEnd = order[order.length - 1].x + (west ? -1 : 1) * (L / 2), xBeg = ex + (west ? 1 : -1) * (L / 2);
          const rect = { x: (xBeg + xEnd) / 2, y: ly, theta: 0, L: Math.abs(xEnd - xBeg) + 2 * m, W: W + 2 * m };
          return !blocked.some((p) => rectIntersectsPolygon(rect, p));
        };
        let laneY = outer;
        if (!corridorFree(outer) && !innerUsed && corridorFree(inner)) { laneY = inner; innerUsed = true; }
        this.rows.push({ id: `r${++idx}`, index: idx - 1, sector: f.name, rowItem: row, trees: order, laneY, heading, entrance: { x: ex, y: laneY, th: heading }, stops: order.map((t) => ({ x: t.x, y: laneY, th: heading })) });
      });
    }
    const cp = this.cpPad ? footprintPoly(this.cpPad) : null;
    // unload pose: beside the pad (east of it), heading north, clear of the crate station south of it
    this.unloadPose = cp ? { x: cp.poly[1][0] + 250 + o.footprint[1] / 2, y: (cp.poly[0][1] + cp.poly[2][1]) / 2 + 700, th: 90 } : { x: -6500, y: -8300, th: 90 };
  }

  /** Polygons the platforms may not cover: static collider outlines + collider-less obstacles. */
  blocked(): number[][][] { return [...this.polys.map((p) => p.poly), ...this.flat.map((f) => f.poly)]; }

  /** Extra circles for robots that stand still (parked platforms, blockers). */
  static robotCircles(p: Pose, fp: [number, number, number]): CircleObstacle[] {
    const c = coveringCircles(fp[0], fp[1]);
    const th = p.th * DEG;
    return c.offsets.map((d) => ({ x: p.x + d * Math.cos(th), y: p.y + d * Math.sin(th), r: c.r, zMin: 0, zMax: fp[2], kind: 'robot' }));
  }

  /** A robot footprint as a polygon (exact; for parked / waiting platforms). */
  static robotPoly(p: Pose, fp: [number, number, number], grow = 0): number[][] {
    const [L, W] = fp; const th = p.th * DEG, c = Math.cos(th), s = Math.sin(th);
    return [[L / 2 + grow, W / 2 + grow], [-L / 2 - grow, W / 2 + grow], [-L / 2 - grow, -W / 2 - grow], [L / 2 + grow, -W / 2 - grow]].map(([u, v]) => [p.x + c * u - s * v, p.y + s * u + c * v]);
  }

  /** Is a footprint pose free (crowns, static outlines, collider-less obstacles, extra polygons)? */
  free(fp: [number, number, number], p: Pose, extraPolys: number[][][] = [], margin = this.o.margin ?? 60): boolean {
    if (this.planWorld.clearance(p.x, p.y, p.th, fp[0], fp[1], fp[2], margin) < margin) return false;
    const rect = { x: p.x, y: p.y, theta: p.th, L: fp[0] + 2 * margin, W: fp[1] + 2 * margin };
    return ![...this.blocked(), ...extraPolys].some((q) => rectIntersectsPolygon(rect, q));
  }

  /**
   * Hybrid A* pose path (x, y, th, dir) of the origin between two poses for a ground platform whose arcs of `radius`
   * are driven by the point at body x = `refX` (the rear axle); cached; null when no path.
   */
  plan(fp: [number, number, number], radius: number, from: Pose, to: Pose, extraPolys: number[][][] = [], refX = 0): number[][] | null {
    const parkedPolys = (this.o.parked ?? []).map((mr) => S1World.robotPoly({ x: mr.state.x, y: mr.state.y, th: mr.state.theta }, mr.kin.footprint as [number, number, number]));
    const key = `${fp.join(',')}|${radius.toFixed(0)}|${refX.toFixed(0)}|${[from.x, from.y, from.th, to.x, to.y, to.th].map((v) => Math.round(v)).join(',')}|${extraPolys.map((q) => q.map((v) => `${Math.round(v[0])},${Math.round(v[1])}`).join(' ')).join(';')}`;
    if (this.cache.has(key)) return this.cache.get(key)!;
    const t0 = Date.now();
    const blocked = [...this.blocked(), ...parkedPolys, ...extraPolys];
    const opt = { footprint: fp, radius, allowReverse: true, blocked, refX };
    // a re-plan around a standing robot gets one bounded attempt (the caller waits when it fails); otherwise forward,
    // then the time-reversed plan (goal → start, driven backwards: a car path reversed is a car path; it is found much
    // faster when the start is a confined pocket such as a parking bay), then forward with half the margin
    const plan1 = (A: Pose, B: Pose, margin: number, maxExpand: number) => planHybridAStar(this.planWorld, { x: A.x, y: A.y, theta: A.th }, { x: B.x, y: B.y, theta: B.th }, { ...opt, margin, maxExpand });
    const m0 = this.o.margin ?? 60;
    let res = plan1(from, to, m0, extraPolys.length ? 20000 : 30000);
    if (!res.ok && !extraPolys.length) {
      const back = plan1(to, from, m0, 30000);
      if (back.ok) res = { ...back, path: back.path.slice().reverse().map((q) => [q[0], q[1], q[2], -(q[3] ?? 1)]) };
    }
    if (!res.ok && !extraPolys.length) res = plan1(from, to, m0 / 2, 200000);
    this.planCalls++; this.planMs += Date.now() - t0;
    this.planLog.push({ ms: Date.now() - t0, ok: res.ok, from, to, extra: extraPolys.length });
    const path = res.ok ? res.path.map((p) => [p[0], p[1], wrap180(p[2]), p[3] ?? 1]) : null;
    if (path) path[0] = [from.x, from.y, from.th, path[0][3]];
    else this.planFailures.push(`(${Math.round(from.x)},${Math.round(from.y)},${Math.round(from.th)}) → (${Math.round(to.x)},${Math.round(to.y)},${Math.round(to.th)}): ${res.error}`);
    this.cache.set(key, path);
    return path;
  }
}

// ------------------------------------------------------------------------------------------------------------------
// Contacts
// ------------------------------------------------------------------------------------------------------------------

interface Box2 { T: Mat4; h: [number, number, number] }
/** Collision boxes of an item subtree in the item's own frame (primitives bounded by boxes). */
function localBoxes(root: Item): Box2[] {
  const out: Box2[] = [];
  const visit = (o: Item, T: Mat4) => {
    for (const g of collidersOf(o)) {
      const pr = g.primitive; if (!pr) continue;
      const h: [number, number, number] = pr.kind === 'box' ? [pr.size[0] / 2, pr.size[1] / 2, pr.size[2] / 2] : pr.kind === 'sphere' ? [pr.radius, pr.radius, pr.radius] : pr.kind === 'plane' ? [pr.size[0] / 2, pr.size[1] / 2, 0] : [pr.radius, pr.radius, pr.length / 2];
      const s = g.scale ?? [1, 1, 1];
      out.push({ T: multiply(T, g.origin ? fromArray(g.origin) : identity()), h: [h[0] * s[0], h[1] * s[1], h[2] * s[2]] });
    }
    for (const c of o.children) if (!(c instanceof MobileRobot)) visit(c, multiply(T, c.pose()));
  };
  visit(root, identity());
  return out;
}

const cross = (o: number[], a: number[], b: number[]) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
function hull(pts: number[][]): number[][] {
  const p = [...pts].sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const lo: number[][] = [], up: number[][] = [];
  for (const q of p) { while (lo.length >= 2 && cross(lo[lo.length - 2], lo[lo.length - 1], q) <= 0) lo.pop(); lo.push(q); }
  for (const q of p.reverse()) { while (up.length >= 2 && cross(up[up.length - 2], up[up.length - 1], q) <= 0) up.pop(); up.push(q); }
  return [...lo.slice(0, -1), ...up.slice(0, -1)];
}
/** Signed distance from a point to a convex polygon (negative inside). */
function pointPolyDist(x: number, y: number, poly: number[][]): number {
  let inside = true, d = Infinity;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i], b = poly[(i + 1) % poly.length];
    if (cross(a, b, [x, y]) < 0) inside = false;
    const ex = b[0] - a[0], ey = b[1] - a[1], L2 = ex * ex + ey * ey;
    const t = L2 > 0 ? Math.max(0, Math.min(1, ((x - a[0]) * ex + (y - a[1]) * ey) / L2)) : 0;
    d = Math.min(d, Math.hypot(x - a[0] - t * ex, y - a[1] - t * ey));
  }
  return inside ? -d : d;
}
function polysOverlap(a: number[][], b: number[][]): boolean {
  for (const P of [a, b]) for (let i = 0; i < P.length; i++) {
    const p = P[i], q = P[(i + 1) % P.length];
    const nx = q[1] - p[1], ny = p[0] - q[0];
    let amin = Infinity, amax = -Infinity, bmin = Infinity, bmax = -Infinity;
    for (const v of a) { const s = v[0] * nx + v[1] * ny; amin = Math.min(amin, s); amax = Math.max(amax, s); }
    for (const v of b) { const s = v[0] * nx + v[1] * ny; bmin = Math.min(bmin, s); bmax = Math.max(bmax, s); }
    if (amax < bmin || bmax < amin) return false;
  }
  return true;
}

export interface S1Contact { t: number; robot: string; other: string; kind: 'trunk' | 'crown' | 'static' | 'robot' | 'flat'; depth: number }

/** Contact checks along the run: poses are applied to the station items, then checked in 3-D. */
export class S1ContactChecker {
  readonly contacts: S1Contact[] = [];
  /** Smallest clearance seen to crowns (mm) per robot (2-D distance of chassis boxes to the crown profile). */
  readonly minCrownClearance: Record<string, number> = {};
  readonly minRobotDistance: Record<string, number> = {};
  checks = 0;
  private chassis = new Map<string, Box2[]>();
  private treePos: Array<{ it: SceneObject; x: number; y: number }>;
  constructor(readonly world: S1World, readonly robots: MobileRobot[], readonly assets?: AssetStore) {
    for (const r of robots) this.chassis.set(r.name, localBoxes(r).filter((b) => b.h[2] > 0));
    this.treePos = world.trees.map((it) => ({ it, x: it.poseAbs()[12], y: it.poseAbs()[13] }));
  }
  /** Check the station at time t (robot items already posed). */
  check(t: number, moving: Set<string>): void {
    this.checks++;
    const R = this.robots;
    const pos = (it: Item) => { const P = it.poseAbs(); return [P[12], P[13]]; };
    for (const r of R) {
      if (!moving.has(r.name)) continue;
      const [rx, ry] = pos(r);
      const rz = r.poseAbs()[14];
      if (r.kin.drive === 'aerial' && rz > 4000) continue; // above every tree and building
      // static 3-D colliders near the robot
      for (const s of this.world.staticItems) {
        const [sx, sy] = pos(s);
        if (Math.abs(sx - rx) > 25000 || Math.abs(sy - ry) > 25000) continue;
        const near = /fence/i.test(s.name) || Math.hypot(sx - rx, sy - ry) < 6000;
        if (!near) continue;
        const pairs = itemsCollide(r, s, { assets: this.assets });
        if (pairs.length) this.contacts.push({ t, robot: r.name, other: s.name, kind: /tree/i.test(s.name) ? 'trunk' : 'static', depth: Math.max(...pairs.map((p) => p.depth)) });
      }
      // crowns: measured radial profile vs chassis boxes
      const P = r.poseAbs();
      const boxes = this.chassis.get(r.name) ?? [];
      for (const tp of this.treePos) {
        if (Math.hypot(tp.x - rx, tp.y - ry) > 4500) continue;
        let clear = Infinity;
        for (const b of boxes) {
          const M = multiply(P, b.T);
          const pts: number[][] = []; let z0 = Infinity, z1 = -Infinity;
          for (let c = 0; c < 8; c++) { const q = transformPoint(M, [c & 1 ? b.h[0] : -b.h[0], c & 2 ? b.h[1] : -b.h[1], c & 4 ? b.h[2] : -b.h[2]]); pts.push([q[0], q[1]]); z0 = Math.min(z0, q[2]); z1 = Math.max(z1, q[2]); }
          const rc = crownRadiusIn(this.world.o.profile, z0, z1);
          if (rc <= 0) continue;
          clear = Math.min(clear, pointPolyDist(tp.x, tp.y, hull(pts)) - rc);
        }
        if (Number.isFinite(clear)) {
          this.minCrownClearance[r.name] = Math.min(this.minCrownClearance[r.name] ?? Infinity, clear);
          if (clear < 0) this.contacts.push({ t, robot: r.name, other: tp.it.name, kind: 'crown', depth: -clear });
        }
      }
      // collider-less obstacles: footprint overlap
      if (r.kin.drive !== 'aerial') {
        const [L, W] = r.kin.footprint; const th = r.state.theta * DEG, c = Math.cos(th), s = Math.sin(th);
        const fp = [[L / 2, W / 2], [-L / 2, W / 2], [-L / 2, -W / 2], [L / 2, -W / 2]].map(([u, v]) => [rx + c * u - s * v, ry + s * u + c * v]);
        for (const f of this.world.flat) if (polysOverlap(fp, f.poly)) this.contacts.push({ t, robot: r.name, other: f.name, kind: 'flat', depth: 0 });
      }
    }
    // robot – robot (3-D colliders)
    for (let i = 0; i < R.length; i++) for (let j = i + 1; j < R.length; j++) {
      const a = R[i], b = R[j];
      if (!moving.has(a.name) && !moving.has(b.name)) continue;
      const [ax, ay] = pos(a), [bx, by] = pos(b);
      const d = Math.hypot(ax - bx, ay - by);
      for (const [n] of [[a.name], [b.name]]) this.minRobotDistance[n] = Math.min(this.minRobotDistance[n] ?? Infinity, d);
      if (d > 6000) continue;
      const pairs = itemsCollide(a, b, { assets: this.assets });
      if (pairs.length) this.contacts.push({ t, robot: a.name, other: b.name, kind: 'robot', depth: Math.max(...pairs.map((p) => p.depth)) });
    }
  }
}
