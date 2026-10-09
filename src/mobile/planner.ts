/** Grid path planning (A* 8-connected with line-of-sight smoothing) and coverage planners. */
import { MapItem } from './items';

export interface PlanResult {
  ok: boolean;
  /** World-space path (mm). */
  path: number[][];
  length: number;
  expanded: number;
  error?: string;
}

class MinHeap<T> {
  private a: Array<{ k: number; v: T }> = [];
  get size() { return this.a.length; }
  push(k: number, v: T) {
    this.a.push({ k, v });
    let i = this.a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (this.a[p].k <= this.a[i].k) break;
      [this.a[p], this.a[i]] = [this.a[i], this.a[p]];
      i = p;
    }
  }
  pop(): T | undefined {
    if (!this.a.length) return undefined;
    const top = this.a[0];
    const last = this.a.pop()!;
    if (this.a.length) {
      this.a[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1;
        let m = i;
        if (l < this.a.length && this.a[l].k < this.a[m].k) m = l;
        if (r < this.a.length && this.a[r].k < this.a[m].k) m = r;
        if (m === i) break;
        [this.a[m], this.a[i]] = [this.a[i], this.a[m]];
        i = m;
      }
    }
    return top.v;
  }
}

export function planPath(map: MapItem, start: [number, number], goal: [number, number], opts: { inflation?: number; smooth?: boolean; maxExpand?: number } = {}): PlanResult {
  const grid = map.inflated(opts.inflation ?? map.inflation);
  const W = map.width, H = map.height;
  const [sx, sy] = map.worldToCell(start[0], start[1]);
  const [gx, gy] = map.worldToCell(goal[0], goal[1]);
  const idx = (x: number, y: number) => y * W + x;
  const free = (x: number, y: number) => x >= 0 && y >= 0 && x < W && y < H && grid[idx(x, y)] < 50;
  if (!free(gx, gy)) return { ok: false, path: [], length: 0, expanded: 0, error: 'Goal is in an obstacle' };
  // allow start inside inflation (robot next to a wall)
  const startBlocked = !free(sx, sy);
  const g = new Float64Array(W * H).fill(Infinity);
  const parent = new Int32Array(W * H).fill(-1);
  const closed = new Uint8Array(W * H);
  const h = (x: number, y: number) => { const dx = Math.abs(x - gx), dy = Math.abs(y - gy); return Math.max(dx, dy) + (Math.SQRT2 - 1) * Math.min(dx, dy); };
  const open = new MinHeap<number>();
  g[idx(sx, sy)] = 0;
  open.push(h(sx, sy), idx(sx, sy));
  let expanded = 0;
  const maxExpand = opts.maxExpand ?? W * H;
  const dirs = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2]];
  let found = false;
  while (open.size) {
    const cur = open.pop()!;
    if (closed[cur]) continue;
    closed[cur] = 1;
    expanded++;
    if (expanded > maxExpand) break;
    const cx = cur % W, cy = (cur / W) | 0;
    if (cx === gx && cy === gy) { found = true; break; }
    for (const [dx, dy, c] of dirs) {
      const nx = cx + dx, ny = cy + dy;
      if (!free(nx, ny)) {
        // permit leaving a blocked start cell through neighbouring blocked cells within 3 cells
        if (!(startBlocked && Math.abs(nx - sx) <= 3 && Math.abs(ny - sy) <= 3 && nx >= 0 && ny >= 0 && nx < W && ny < H && map.cells[idx(nx, ny)] < 50)) continue;
      }
      // no corner cutting
      if (dx && dy && (!free(cx + dx, cy) || !free(cx, cy + dy))) continue;
      const ni = idx(nx, ny);
      if (closed[ni]) continue;
      // penalise proximity to obstacles (inflated ring cells have value 99)
      const pen = grid[ni] === 99 ? 2 : 1;
      const ng = g[cur] + c * pen;
      if (ng < g[ni]) {
        g[ni] = ng;
        parent[ni] = cur;
        open.push(ng + h(nx, ny), ni);
      }
    }
  }
  if (!found) return { ok: false, path: [], length: 0, expanded, error: 'No path found' };
  const cells: number[] = [];
  for (let i = idx(gx, gy); i !== -1; i = parent[i]) cells.push(i);
  cells.reverse();
  let pts = cells.map((i) => map.cellToWorld(i % W, (i / W) | 0) as number[]);
  pts[0] = [start[0], start[1]];
  pts[pts.length - 1] = [goal[0], goal[1]];
  if (opts.smooth !== false) pts = smoothPath(pts, (a, b) => lineFree(map, grid, a, b));
  return { ok: true, path: pts, length: pathLength(pts), expanded };
}

function lineFree(map: MapItem, grid: Uint8Array, a: number[], b: number[]): boolean {
  const n = Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / (map.resolution * 0.5)) + 1;
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    const [cx, cy] = map.worldToCell(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t);
    if (cx < 0 || cy < 0 || cx >= map.width || cy >= map.height || grid[cy * map.width + cx] >= 50) return false;
  }
  return true;
}

/** String-pulling: remove intermediate points when a direct segment is collision-free. */
export function smoothPath(pts: number[][], canSee: (a: number[], b: number[]) => boolean): number[][] {
  if (pts.length <= 2) return pts;
  const out = [pts[0]];
  let i = 0;
  while (i < pts.length - 1) {
    let j = pts.length - 1;
    while (j > i + 1 && !canSee(pts[i], pts[j])) j--;
    out.push(pts[j]);
    i = j;
  }
  return out;
}

export function pathLength(pts: number[][]): number {
  let l = 0;
  for (let i = 1; i < pts.length; i++) l += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
  return l;
}

/**
 * Boustrophedon coverage of a convex polygon (mm) with a given swath width and heading (deg).
 * Returns a lawn-mower path with turns at the headland.
 */
export function coveragePath(polygon: number[][], swath: number, headingDeg = 0, headlandOffset = 0): number[][] {
  if (polygon.length < 3) return [];
  const th = (headingDeg * Math.PI) / 180;
  const c = Math.cos(th), s = Math.sin(th);
  // rotate polygon so heading is along +X
  const rot = polygon.map(([x, y]) => [x * c + y * s, -x * s + y * c]);
  const minY = Math.min(...rot.map((p) => p[1])) + headlandOffset + swath / 2;
  const maxY = Math.max(...rot.map((p) => p[1])) - headlandOffset;
  const path: number[][] = [];
  let dir = 1;
  for (let y = minY; y <= maxY + 1e-6; y += swath) {
    // intersect scanline y with polygon edges
    const xs: number[] = [];
    for (let i = 0; i < rot.length; i++) {
      const a = rot[i], b = rot[(i + 1) % rot.length];
      if (a[1] === b[1]) continue;
      if (y >= Math.min(a[1], b[1]) && y < Math.max(a[1], b[1])) xs.push(a[0] + ((y - a[1]) * (b[0] - a[0])) / (b[1] - a[1]));
    }
    if (xs.length < 2) continue;
    xs.sort((p, q) => p - q);
    let x0 = xs[0] + headlandOffset, x1 = xs[xs.length - 1] - headlandOffset;
    if (dir < 0) [x0, x1] = [x1, x0];
    path.push([x0, y], [x1, y]);
    dir = -dir;
  }
  // rotate back
  return path.map(([x, y]) => [x * c - y * s, x * s + y * c]);
}

/**
 * Row-following path for orchards/vineyards: visit rows in a given order, entering at one end
 * and leaving at the other, connecting via headland arcs (approximated by polyline).
 */
export function rowTraversalPath(rows: Array<{ start: number[]; end: number[] }>, order: number[] = rows.map((_, i) => i), headlandLength = 4000, turnRadius = 2500): number[][] {
  const path: number[][] = [];
  let forward = true;
  const ext = (a: number[], b: number[], d: number) => { const l = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1; return [b[0] + ((b[0] - a[0]) / l) * d, b[1] + ((b[1] - a[1]) / l) * d]; };
  for (let k = 0; k < order.length; k++) {
    const r = rows[order[k]];
    const a = forward ? r.start : r.end;
    const b = forward ? r.end : r.start;
    const entry = ext(b, a, headlandLength);
    const exit = ext(a, b, headlandLength);
    if (path.length) {
      // headland turn: arc between previous exit and this entry
      const prev = path[path.length - 1];
      const mid = [(prev[0] + entry[0]) / 2, (prev[1] + entry[1]) / 2];
      const dx = entry[0] - prev[0], dy = entry[1] - prev[1];
      const l = Math.hypot(dx, dy) || 1;
      const nx = -dy / l, ny = dx / l;
      // choose bulge away from the rows (towards the exit direction)
      const outDir = [exit[0] - b[0], exit[1] - b[1]];
      const sign = Math.sign(nx * -outDir[0] + ny * -outDir[1]) || 1;
      const bulge = Math.min(turnRadius, l / 2);
      for (let i = 1; i < 6; i++) {
        const t = i / 6;
        const bx = prev[0] + dx * t + sign * nx * Math.sin(Math.PI * t) * bulge;
        const by = prev[1] + dy * t + sign * ny * Math.sin(Math.PI * t) * bulge;
        path.push([bx, by]);
      }
      void mid;
    }
    path.push(entry, [...a], [...b], exit);
    forward = !forward;
  }
  return path;
}

/** Nearest-neighbour row ordering starting from a point (fast heuristic; rows alternate direction). */
export function orderRowsNearest(rows: Array<{ start: number[]; end: number[] }>, from: number[]): number[] {
  const left = new Set(rows.map((_, i) => i));
  const order: number[] = [];
  let pos = from;
  while (left.size) {
    let best = -1, bd = Infinity;
    for (const i of left) {
      const d = Math.min(Math.hypot(rows[i].start[0] - pos[0], rows[i].start[1] - pos[1]), Math.hypot(rows[i].end[0] - pos[0], rows[i].end[1] - pos[1]));
      if (d < bd) { bd = d; best = i; }
    }
    order.push(best);
    left.delete(best);
    const r = rows[best];
    pos = Math.hypot(r.start[0] - pos[0], r.start[1] - pos[1]) < Math.hypot(r.end[0] - pos[0], r.end[1] - pos[1]) ? r.end : r.start;
  }
  return order;
}
