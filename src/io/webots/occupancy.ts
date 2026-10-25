/**
 * Occupancy grid from scene collision geometry: every collider (SceneObject.colliders when present — Webots
 * boundingObjects — else the visual geometry) intersecting a height band marks the cells whose vertical segment
 * [zMin, zMax] meets it. Robots, mobile robots and their children, ground planes and objects without colliders are
 * skipped, so the map matches what a robot of that height can bump into (e.g. 0..0.62 m for MUROM).
 */
import { Item, Station, SceneObject, GeometryRef } from '../../core/items/item';
import { Robot } from '../../core/items/robot';
import { MobileRobot, MapItem } from '../../mobile/items';
import type { AssetStore } from '../../scene/assets';
import { Mat4, identity, multiply, invert, transformPoint, fromArray } from '../../core/math/pose';

export interface OccupancyOptions {
  /** Height band (mm, station Z). */
  zMin?: number;
  zMax?: number;
  /** Cell size (mm). */
  resolution?: number;
  /** Map bounds (mm); default: the ground objects' extent or the colliders' extent plus a margin. */
  bounds?: { minX: number; minY: number; maxX: number; maxY: number };
  /** Extra exclusion test. */
  exclude?: (item: Item) => boolean;
  assets?: AssetStore;
  name?: string;
  /** Inflation radius stored in the MapItem (mm). */
  inflation?: number;
}

interface Prim { inv: Mat4; T: Mat4; g: GeometryRef; half: [number, number, number]; kind: string; mesh?: { min: number[]; max: number[] } }

const collidersOf = (it: Item): GeometryRef[] | undefined => (it as unknown as { colliders?: GeometryRef[] }).colliders;

function excludedSubtree(it: Item): boolean {
  for (let p: Item | null = it; p && !(p instanceof Station); p = p.parent) if (p instanceof Robot || p instanceof MobileRobot) return true;
  return false;
}

/** Is a point (local frame of the primitive) inside it? */
function inside(p: Prim, q: number[]): boolean {
  const g = p.g, pr = g.primitive;
  if (p.mesh) {
    const s = g.scale ?? [1, 1, 1];
    for (let a = 0; a < 3; a++) { const lo = Math.min(p.mesh.min[a] * s[a], p.mesh.max[a] * s[a]), hi = Math.max(p.mesh.min[a] * s[a], p.mesh.max[a] * s[a]); if (q[a] < lo || q[a] > hi) return false; }
    return true;
  }
  if (!pr) return false;
  switch (pr.kind) {
    case 'box': return Math.abs(q[0]) <= pr.size[0] / 2 && Math.abs(q[1]) <= pr.size[1] / 2 && Math.abs(q[2]) <= pr.size[2] / 2;
    case 'cylinder': return Math.hypot(q[0], q[1]) <= pr.radius && Math.abs(q[2]) <= pr.length / 2;
    case 'cone': return Math.abs(q[2]) <= pr.length / 2 && Math.hypot(q[0], q[1]) <= pr.radius * (0.5 - q[2] / pr.length);
    case 'sphere': return Math.hypot(q[0], q[1], q[2]) <= pr.radius;
    default: return false;
  }
}

/** Rasterise the station's colliders in a height band into a new MapItem (not added to the station). */
export function rasterizeOccupancy(station: Station, opts: OccupancyOptions = {}): MapItem {
  const zMin = opts.zMin ?? 20, zMax = opts.zMax ?? 600, res = opts.resolution ?? 50;
  const prims: Prim[] = [];
  let gb: OccupancyOptions['bounds'] | undefined;
  const ext = { minX: Infinity, minY: Infinity, maxX: -Infinity, maxY: -Infinity };
  for (const it of station.walk()) {
    if (!(it instanceof SceneObject) || excludedSubtree(it) || opts.exclude?.(it)) continue;
    const P = it.poseAbs();
    const extentOf = (refs: GeometryRef[]) => {
      for (const g of refs) {
        const pr = g.primitive;
        if (pr?.kind !== 'plane' && pr?.kind !== 'box') continue;
        const T = multiply(P, g.origin ? fromArray(g.origin) : identity());
        const [sx, sy] = pr.size;
        for (const [x, y] of [[-sx / 2, -sy / 2], [sx / 2, sy / 2], [-sx / 2, sy / 2], [sx / 2, -sy / 2]]) {
          const w = transformPoint(T, [x, y, 0]);
          gb = gb ?? { minX: Infinity, minY: Infinity, maxX: -Infinity, maxY: -Infinity };
          gb.minX = Math.min(gb.minX, w[0]); gb.maxX = Math.max(gb.maxX, w[0]); gb.minY = Math.min(gb.minY, w[1]); gb.maxY = Math.max(gb.maxY, w[1]);
        }
      }
    };
    if (it.params.ground) { extentOf(it.geometry); continue; }
    const refs = collidersOf(it) ?? it.geometry;
    for (const g of refs) {
      if (!g.primitive && !g.mesh) continue;
      if (g.primitive?.kind === 'plane') continue;
      const T = multiply(P, g.origin ? fromArray(g.origin) : identity());
      let half: [number, number, number];
      let mesh: Prim['mesh'];
      if (g.mesh) {
        const m = opts.assets?.resolve ? opts.assets.resolve(g.mesh)?.mesh : opts.assets?.get(g.mesh)?.mesh;
        if (!m || !isFinite(m.min[0])) continue;
        mesh = { min: m.min, max: m.max };
        const s = g.scale ?? [1, 1, 1];
        half = [0, 1, 2].map((a) => Math.max(Math.abs(m.min[a] * s[a]), Math.abs(m.max[a] * s[a]))) as [number, number, number];
      } else {
        const pr = g.primitive!;
        half = pr.kind === 'box' ? [pr.size[0] / 2, pr.size[1] / 2, pr.size[2] / 2] : pr.kind === 'sphere' ? [pr.radius, pr.radius, pr.radius] : pr.kind === 'plane' ? [0, 0, 0] : [pr.radius, pr.radius, pr.length / 2];
      }
      const p: Prim = { T, inv: invert(T), g, half, kind: g.mesh ? 'mesh' : g.primitive!.kind, mesh };
      // world-space bounds of the primitive (box corners)
      let zlo = Infinity, zhi = -Infinity;
      for (let c = 0; c < 8; c++) {
        const w = transformPoint(T, [c & 1 ? half[0] : -half[0], c & 2 ? half[1] : -half[1], c & 4 ? half[2] : -half[2]]);
        zlo = Math.min(zlo, w[2]); zhi = Math.max(zhi, w[2]);
        ext.minX = Math.min(ext.minX, w[0]); ext.maxX = Math.max(ext.maxX, w[0]); ext.minY = Math.min(ext.minY, w[1]); ext.maxY = Math.max(ext.maxY, w[1]);
      }
      if (zhi < zMin || zlo > zMax) continue;
      prims.push(p);
    }
  }
  const b = opts.bounds ?? (gb && isFinite(gb.minX) ? gb : isFinite(ext.minX) ? { minX: ext.minX - 500, minY: ext.minY - 500, maxX: ext.maxX + 500, maxY: ext.maxY + 500 } : { minX: -5000, minY: -5000, maxX: 5000, maxY: 5000 });
  const map = new MapItem(opts.name ?? 'occupancy');
  const w = Math.max(1, Math.ceil((b.maxX - b.minX) / res)), h = Math.max(1, Math.ceil((b.maxY - b.minY) / res));
  map.resize(w, h, res, b.minX, b.minY);
  if (opts.inflation !== undefined) map.inflation = opts.inflation;
  map.params.source = 'colliders';
  map.params.band = [zMin, zMax];
  const steps = Math.max(4, Math.ceil((zMax - zMin) / Math.min(25, res / 2)));
  for (const p of prims) {
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (let c = 0; c < 8; c++) {
      const q = transformPoint(p.T, [c & 1 ? p.half[0] : -p.half[0], c & 2 ? p.half[1] : -p.half[1], c & 4 ? p.half[2] : -p.half[2]]);
      x0 = Math.min(x0, q[0]); x1 = Math.max(x1, q[0]); y0 = Math.min(y0, q[1]); y1 = Math.max(y1, q[1]);
    }
    const [cx0, cy0] = map.worldToCell(x0, y0), [cx1, cy1] = map.worldToCell(x1, y1);
    for (let cy = Math.max(0, cy0); cy <= Math.min(h - 1, cy1); cy++) for (let cx = Math.max(0, cx0); cx <= Math.min(w - 1, cx1); cx++) {
      if (map.cells[cy * w + cx] >= 100) continue;
      // sample the cell (centre and 4 inner points) along the band
      const [wx, wy] = map.cellToWorld(cx, cy);
      let hit = false;
      for (const [ox, oy] of [[0, 0], [-0.3, -0.3], [0.3, -0.3], [-0.3, 0.3], [0.3, 0.3]]) {
        for (let k = 0; k <= steps && !hit; k++) {
          const q = transformPoint(p.inv, [wx + ox * res, wy + oy * res, zMin + ((zMax - zMin) * k) / steps]);
          if (inside(p, q)) hit = true;
        }
        if (hit) break;
      }
      if (hit) map.cells[cy * w + cx] = 100;
    }
  }
  map.notify('map');
  return map;
}
