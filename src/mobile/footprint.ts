/**
 * Footprint geometry for ground robots and the 2-D collision world used by planning, traffic management and contact
 * detection: oriented-rectangle footprints, exact rectangle–circle distances, SAT overlap with a safety gap, and a
 * world made of an occupancy map (Euclidean distance field) plus circular obstacles with height bands (tree trunks,
 * canopies above `zMin`, posts) indexed in a spatial hash.
 */
import { MapItem } from './items';

/** Planar pose: position (mm) and heading (deg). */
export interface Pose2 { x: number; y: number; theta: number }

/** Oriented rectangle: centre, heading (deg), length along the heading and width (mm). */
export interface Rect2 { x: number; y: number; theta: number; L: number; W: number }

export function rectCorners(r: Rect2, margin = 0): number[][] {
  const th = (r.theta * Math.PI) / 180, c = Math.cos(th), s = Math.sin(th);
  const hl = r.L / 2 + margin, hw = r.W / 2 + margin;
  return [[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]].map(([u, v]) => [r.x + c * u - s * v, r.y + s * u + c * v]);
}

/** Distance from a point to an oriented rectangle (0 inside). */
export function pointRectDistance(px: number, py: number, r: Rect2): number {
  const th = (r.theta * Math.PI) / 180;
  const dx = px - r.x, dy = py - r.y;
  const lx = Math.cos(th) * dx + Math.sin(th) * dy;
  const ly = -Math.sin(th) * dx + Math.cos(th) * dy;
  const ex = Math.max(0, Math.abs(lx) - r.L / 2), ey = Math.max(0, Math.abs(ly) - r.W / 2);
  return Math.hypot(ex, ey);
}

/** Separating-axis test of two oriented rectangles grown by `gap` (each by gap/2). True when they intersect. */
export function rectsOverlap(a: Rect2, b: Rect2, gap = 0): boolean {
  if (Math.hypot(a.x - b.x, a.y - b.y) > (Math.hypot(a.L, a.W) + Math.hypot(b.L, b.W)) / 2 + gap) return false;
  const A = rectCorners(a, gap / 2), B = rectCorners(b, gap / 2);
  const axes: number[][] = [];
  for (const r of [a, b]) { const th = (r.theta * Math.PI) / 180; axes.push([Math.cos(th), Math.sin(th)], [-Math.sin(th), Math.cos(th)]); }
  for (const [ax, ay] of axes) {
    let amin = Infinity, amax = -Infinity, bmin = Infinity, bmax = -Infinity;
    for (const p of A) { const v = p[0] * ax + p[1] * ay; if (v < amin) amin = v; if (v > amax) amax = v; }
    for (const p of B) { const v = p[0] * ax + p[1] * ay; if (v < bmin) bmin = v; if (v > bmax) bmax = v; }
    if (amax < bmin || bmax < amin) return false;
  }
  return true;
}

/**
 * Signed separation of two oriented rectangles along the SAT axes (mm): > 0 a lower bound of their distance (the
 * largest gap between projections), < 0 the smallest penetration depth. Used to tell whether a motion separates
 * two footprints that are already too close.
 */
export function rectSeparation(a: Rect2, b: Rect2): number {
  const A = rectCorners(a), B = rectCorners(b);
  let best = -Infinity;
  for (const r of [a, b]) {
    const th = (r.theta * Math.PI) / 180;
    for (const [ax, ay] of [[Math.cos(th), Math.sin(th)], [-Math.sin(th), Math.cos(th)]]) {
      let amin = Infinity, amax = -Infinity, bmin = Infinity, bmax = -Infinity;
      for (const p of A) { const v = p[0] * ax + p[1] * ay; if (v < amin) amin = v; if (v > amax) amax = v; }
      for (const p of B) { const v = p[0] * ax + p[1] * ay; if (v < bmin) bmin = v; if (v > bmax) bmax = v; }
      best = Math.max(best, Math.max(bmin - amax, amin - bmax));
    }
  }
  return best;
}

/** Point in polygon (even-odd). */
export function pointInPolygon(x: number, y: number, p: number[][]): boolean {
  let inside = false;
  for (let i = 0, j = p.length - 1; i < p.length; j = i++) {
    const xi = p[i][0], yi = p[i][1], xj = p[j][0], yj = p[j][1];
    if (yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

/** True when an oriented rectangle intersects a polygon (corner in polygon, polygon vertex in rectangle or edge crossing). */
export function rectIntersectsPolygon(r: Rect2, poly: number[][]): boolean {
  const C = rectCorners(r);
  if (C.some((p) => pointInPolygon(p[0], p[1], poly))) return true;
  if (poly.some((p) => pointRectDistance(p[0], p[1], r) === 0)) return true;
  for (let i = 0; i < 4; i++) for (let j = 0, k = poly.length - 1; j < poly.length; k = j++) if (segmentsCross(C[i], C[(i + 1) % 4], poly[k], poly[j])) return true;
  return false;
}

function segmentsCross(a: number[], b: number[], c: number[], d: number[]): boolean {
  const o = (p: number[], q: number[], r: number[]) => (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
  const d1 = o(c, d, a), d2 = o(c, d, b), d3 = o(a, b, c), d4 = o(a, b, d);
  return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
}

/** Circles covering an L x W rectangle (centres along the length): offsets (mm, body frame) and the common radius. */
export function coveringCircles(L: number, W: number): { offsets: number[]; r: number } {
  const n = Math.max(1, Math.ceil(L / W));
  const seg = L / n;
  const offsets = Array.from({ length: n }, (_, i) => -L / 2 + seg * (i + 0.5));
  return { offsets, r: Math.hypot(seg / 2, W / 2) };
}

/** Circular obstacle with a height band (mm above the ground plane of the world). */
export interface CircleObstacle { x: number; y: number; r: number; zMin: number; zMax: number; kind: string; ref?: { rowId?: string; plant?: number; itemId?: string } }

/** Footprint of a robot: length, width, height (mm). */
export type Footprint = [number, number, number];

/**
 * 2-D collision world: occupancy map (cells >= `threshold` are obstacles; crop cells written by the field rasteriser are
 * value 90 and are skipped when `ignoreCropCells` because the exact crop circles replace them) + circle obstacles.
 */
export class CollisionWorld {
  readonly circles: CircleObstacle[];
  readonly map: MapItem | null;
  private hash = new Map<number, number[]>();
  private readonly cell = 2000;
  private edt: Float32Array | null = null;
  private edtKey = '';
  readonly ignoreCropCells: boolean;
  /** Largest circle radius (hash query padding). */
  private rmax = 0;
  private clearGrids = new Map<string, { res: number; x0: number; y0: number; w: number; h: number; c: Float32Array }>();

  constructor(opts: { map?: MapItem | null; circles?: CircleObstacle[]; ignoreCropCells?: boolean } = {}) {
    this.map = opts.map ?? null;
    this.circles = opts.circles ?? [];
    this.ignoreCropCells = opts.ignoreCropCells ?? this.circles.length > 0;
    this.circles.forEach((c, i) => {
      this.rmax = Math.max(this.rmax, c.r);
      const x0 = Math.floor((c.x - c.r) / this.cell), x1 = Math.floor((c.x + c.r) / this.cell);
      const y0 = Math.floor((c.y - c.r) / this.cell), y1 = Math.floor((c.y + c.r) / this.cell);
      for (let gx = x0; gx <= x1; gx++) for (let gy = y0; gy <= y1; gy++) {
        const k = this.key(gx, gy);
        let a = this.hash.get(k);
        if (!a) { a = []; this.hash.set(k, a); }
        a.push(i);
      }
    });
  }

  private key(gx: number, gy: number): number { return (gx + 32768) * 65536 + (gy + 32768); }

  /** Circles whose disk may lie within `pad` of the axis-aligned box [x0,x1]x[y0,y1]. */
  near(x0: number, y0: number, x1: number, y1: number, height = Infinity): CircleObstacle[] {
    const out: CircleObstacle[] = [];
    if (!this.circles.length) return out;
    const seen = new Set<number>();
    for (let gx = Math.floor(x0 / this.cell); gx <= Math.floor(x1 / this.cell); gx++) for (let gy = Math.floor(y0 / this.cell); gy <= Math.floor(y1 / this.cell); gy++) {
      const a = this.hash.get(this.key(gx, gy));
      if (!a) continue;
      for (const i of a) {
        if (seen.has(i)) continue;
        seen.add(i);
        const c = this.circles[i];
        if (c.zMin >= height) continue;
        out.push(c);
      }
    }
    return out;
  }

  /** Distance field of the map obstacles (mm from each cell centre to the nearest obstacle cell centre). */
  private distanceField(): Float32Array | null {
    const m = this.map;
    if (!m) return null;
    const key = `${(m as any)._version ?? 0}:${m.width}:${m.height}:${m.cells.length}`;
    if (this.edt && this.edtKey === key) return this.edt;
    const W = m.width, H = m.height, INF = 1e20;
    const thr = (v: number) => (this.ignoreCropCells ? v >= 50 && v !== 90 : v >= 50);
    const f = new Float64Array(Math.max(W, H));
    const g = new Float64Array(W * H);
    for (let i = 0; i < W * H; i++) g[i] = thr(m.cells[i]) ? 0 : INF;
    const v = new Int32Array(Math.max(W, H)), z = new Float64Array(Math.max(W, H) + 1), d = new Float64Array(Math.max(W, H));
    const dt1 = (n: number) => {
      let k = 0; v[0] = 0; z[0] = -INF; z[1] = INF;
      for (let q = 1; q < n; q++) {
        let s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
        while (s <= z[k]) { k--; s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]); }
        k++; v[k] = q; z[k] = s; z[k + 1] = INF;
      }
      k = 0;
      for (let q = 0; q < n; q++) { while (z[k + 1] < q) k++; d[q] = (q - v[k]) * (q - v[k]) + f[v[k]]; }
    };
    for (let x = 0; x < W; x++) { for (let y = 0; y < H; y++) f[y] = g[y * W + x]; dt1(H); for (let y = 0; y < H; y++) g[y * W + x] = d[y]; }
    for (let y = 0; y < H; y++) { for (let x = 0; x < W; x++) f[x] = g[y * W + x]; dt1(W); for (let x = 0; x < W; x++) g[y * W + x] = d[x]; }
    const out = new Float32Array(W * H);
    for (let i = 0; i < W * H; i++) out[i] = g[i] >= INF / 2 ? 1e9 : Math.sqrt(g[i]) * m.resolution;
    this.edt = out; this.edtKey = key;
    return out;
  }

  /** Conservative clearance (mm) of a point to the map obstacles (outside the map = obstacle). */
  mapClearance(x: number, y: number): number {
    const m = this.map;
    if (!m) return Infinity;
    const edt = this.distanceField()!;
    // map item pose: world -> map local (maps are placed with a planar pose)
    const [lx, ly] = this.toMap(x, y);
    const [cx, cy] = m.worldToCell(lx, ly);
    if (cx < 0 || cy < 0 || cx >= m.width || cy >= m.height) return -1;
    const [wx, wy] = m.cellToWorld(cx, cy);
    return edt[cy * m.width + cx] - Math.hypot(lx - wx, ly - wy) - 0.71 * m.resolution;
  }

  private mapInv: number[] | null = null;
  private toMap(x: number, y: number): [number, number] {
    const m = this.map!;
    if (!this.mapInv) {
      const a = m.poseAbs();
      // inverse of the planar part
      const c = a[0], s = a[1];
      this.mapInv = [c, s, a[12], a[13]];
    }
    const [c, s, tx, ty] = this.mapInv;
    if (c === 1 && s === 0 && tx === 0 && ty === 0) return [x, y];
    const dx = x - tx, dy = y - ty;
    return [c * dx + s * dy, -s * dx + c * dy];
  }

  /** Clearance (mm) of a point to everything (circles below `height` and the map). */
  pointClearance(x: number, y: number, height = Infinity): number {
    let best = this.mapClearance(x, y);
    const pad = this.rmax + 3000;
    for (const c of this.near(x - pad, y - pad, x + pad, y + pad, height)) best = Math.min(best, Math.hypot(x - c.x, y - c.y) - c.r);
    return best;
  }

  /**
   * Signed clearance (mm) of a footprint rectangle at a pose: exact against circle obstacles below `height`, conservative
   * (covering circles on the distance field) against the map. Negative = penetration depth. Checks stop early once the
   * clearance is known to be below `stopBelow`.
   */
  clearance(x: number, y: number, thetaDeg: number, L: number, W: number, height = Infinity, stopBelow = -Infinity): number {
    let best = Infinity;
    if (this.map) {
      const { offsets, r } = coveringCircles(L, W);
      const th = (thetaDeg * Math.PI) / 180, c = Math.cos(th), s = Math.sin(th);
      for (const o of offsets) {
        const d = this.mapClearance(x + c * o, y + s * o) - r;
        if (d < best) best = d;
        if (best < stopBelow) return best;
      }
    }
    if (this.circles.length) {
      const rr = Math.hypot(L, W) / 2 + this.rmax;
      const rect: Rect2 = { x, y, theta: thetaDeg, L, W };
      for (const c of this.near(x - rr, y - rr, x + rr, y + rr, height)) {
        const d = pointRectDistance(c.x, c.y, rect) - c.r;
        if (d < best) best = d;
        if (best < stopBelow) return best;
      }
    }
    return best;
  }

  /** Circle obstacles currently in contact with (penetrating) a footprint. */
  contacts(rect: Rect2, height = Infinity, tol = 0): CircleObstacle[] {
    const rr = Math.hypot(rect.L, rect.W) / 2 + this.rmax;
    return this.near(rect.x - rr, rect.y - rr, rect.x + rr, rect.y + rr, height).filter((c) => pointRectDistance(c.x, c.y, rect) < c.r - tol);
  }

  /**
   * Clearance grid (mm) over the map extent (or the circles' bounding box) at resolution `res` for a robot height:
   * min distance from each cell centre to obstacles. Used by grid A* (inscribed-radius feasibility + proximity cost).
   */
  clearanceGrid(res: number, height = Infinity, bounds?: { x0: number; y0: number; x1: number; y1: number }): { res: number; x0: number; y0: number; w: number; h: number; c: Float32Array } {
    const b = bounds ?? this.bounds();
    const key = `${res}:${height}:${b.x0}:${b.y0}:${b.x1}:${b.y1}`;
    const cached = this.clearGrids.get(key);
    if (cached) return cached;
    const w = Math.max(1, Math.ceil((b.x1 - b.x0) / res)), h = Math.max(1, Math.ceil((b.y1 - b.y0) / res));
    const c = new Float32Array(w * h);
    for (let j = 0; j < h; j++) for (let i = 0; i < w; i++) c[j * w + i] = this.pointClearance(b.x0 + (i + 0.5) * res, b.y0 + (j + 0.5) * res, height);
    const g = { res, x0: b.x0, y0: b.y0, w, h, c };
    if (this.clearGrids.size > 8) this.clearGrids.clear();
    this.clearGrids.set(key, g);
    return g;
  }

  /** World bounds: the map extent, else the circles' bounding box grown by 20 m. */
  bounds(): { x0: number; y0: number; x1: number; y1: number } {
    if (this.map) {
      const m = this.map;
      const a = m.poseAbs();
      const pts = [[m.originX, m.originY], [m.originX + m.width * m.resolution, m.originY], [m.originX, m.originY + m.height * m.resolution], [m.originX + m.width * m.resolution, m.originY + m.height * m.resolution]]
        .map(([x, y]) => [a[0] * x + a[4] * y + a[12], a[1] * x + a[5] * y + a[13]]);
      return { x0: Math.min(...pts.map((p) => p[0])), y0: Math.min(...pts.map((p) => p[1])), x1: Math.max(...pts.map((p) => p[0])), y1: Math.max(...pts.map((p) => p[1])) };
    }
    if (!this.circles.length) return { x0: -50000, y0: -50000, x1: 50000, y1: 50000 };
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    for (const c of this.circles) { x0 = Math.min(x0, c.x - c.r); y0 = Math.min(y0, c.y - c.r); x1 = Math.max(x1, c.x + c.r); y1 = Math.max(y1, c.y + c.r); }
    return { x0: x0 - 20000, y0: y0 - 20000, x1: x1 + 20000, y1: y1 + 20000 };
  }

  /** True when the point lies inside the world bounds (always true without a map). */
  inBounds(x: number, y: number): boolean {
    if (!this.map) return true;
    const [lx, ly] = this.toMap(x, y);
    const [cx, cy] = this.map.worldToCell(lx, ly);
    return cx >= 0 && cy >= 0 && cx < this.map.width && cy < this.map.height;
  }
}

/**
 * Pose of the body point at x = `a` (mm, body frame) for a body pose (same heading). A car-like base is planned and
 * tracked at its rear axle (`shiftPose(origin, rearAxle)`), its footprint and item pose stay at the body origin
 * (`shiftPose(rear, -rearAxle)`).
 */
export function shiftPose<T extends { x: number; y: number; theta: number }>(p: T, a: number): T {
  if (!a) return p;
  const th = (p.theta * Math.PI) / 180;
  return { ...p, x: p.x + Math.cos(th) * a, y: p.y + Math.sin(th) * a };
}

/** Pose path ([x, y, headingDeg, dir, …] points) of the body point at x = `a` (see shiftPose). */
export function shiftPosePath(path: number[][], a: number): number[][] {
  if (!a) return path;
  return path.map((p) => { const th = (p[2] * Math.PI) / 180; const q = p.slice(); q[0] = p[0] + Math.cos(th) * a; q[1] = p[1] + Math.sin(th) * a; return q; });
}

/** Footprint rectangle of a robot-like state. */
export function footprintRect(st: { x: number; y: number; theta: number }, fp: number[]): Rect2 {
  return { x: st.x, y: st.y, theta: st.theta, L: fp[0], W: fp[1] };
}
