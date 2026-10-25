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

// ---------------------------------------------------------------------------------------------------------------
// Footprint- and drive-aware planning (pose paths)
// ---------------------------------------------------------------------------------------------------------------
import type { MobileRobot } from './items';
import { CollisionWorld, CircleObstacle, Pose2 } from './footprint';
import { planGridFootprint, planHybridAStar, PosePathResult } from './hybrid_astar';
import { minTurnRadius } from './controller';

export interface RobotPlanOptions {
  world: CollisionWorld;
  /** Clearance kept between the footprint and obstacles (mm, default 120). */
  margin?: number;
  /** Final heading (deg). */
  goalHeading?: number;
  /** Polygons not to enter (traffic segments reserved by other robots, no-go zones). */
  blocked?: number[][][];
  /** Dynamic obstacles (other robots' footprints as circles). */
  extra?: CircleObstacle[];
  allowReverse?: boolean;
  maxExpand?: number;
}

/**
 * Plan a collision-free pose path for a robot: Hybrid A* with Reeds–Shepp shots for Ackermann (curvature ≤ 1/R_min,
 * reverse allowed), footprint-aware grid A* with oriented smoothing for differential / tracked / legged / omni (falling
 * back to Hybrid A* with in-place rotations where the turn-then-drive path does not fit), straight flight for aerial.
 */
export function planForRobot(robot: MobileRobot, start: Pose2, goal: { x: number; y: number }, o: RobotPlanOptions): PosePathResult {
  const k = robot.kin;
  const footprint: [number, number, number] = [k.footprint[0], k.footprint[1], k.footprint[2]];
  const base = { footprint, margin: o.margin ?? 120, goalHeading: o.goalHeading, blocked: o.blocked, extra: o.extra, maxExpand: o.maxExpand };
  if (k.drive === 'aerial') {
    const th = o.goalHeading ?? start.theta;
    const len = Math.hypot(goal.x - start.x, goal.y - start.y);
    const n = Math.max(1, Math.ceil(len / 500));
    const path = [[start.x, start.y, start.theta, 1]];
    for (let i = 1; i <= n; i++) path.push([start.x + ((goal.x - start.x) * i) / n, start.y + ((goal.y - start.y) * i) / n, th, 1]);
    return { ok: true, path, length: len, expanded: 0, method: 'straight' };
  }
  if (k.drive === 'ackermann') {
    return planHybridAStar(o.world, start, { x: goal.x, y: goal.y, theta: o.goalHeading }, { ...base, radius: minTurnRadius(k) * 1.15, allowReverse: o.allowReverse ?? true });
  }
  const g = planGridFootprint(o.world, start, goal, base);
  if (g.ok) return g;
  const h = planHybridAStar(o.world, start, { x: goal.x, y: goal.y, theta: o.goalHeading }, { ...base, radius: Math.max(800, k.footprint[1]), inPlace: true, allowReverse: o.allowReverse ?? true, reverseCost: 1.5 });
  return h.ok ? h : { ...g, error: `${g.error}; hybrid: ${h.error}` };
}

export interface HeadlandTurn { kind: 'U' | 'omega' | 'fishtail'; path: number[][]; length: number; /** Depth needed beyond the row-end line (mm), footprint included. */ depth: number }

/**
 * Headland turns from a row exit pose to the next row's entry pose for a car-like robot with turning radius R:
 * forward-only (U turn when the row spacing ≥ 2R, else an Ω / bulb turn) and with reverse (fish-tail / 3-point turn).
 * `depth` is how far the footprint reaches beyond the exit line — compare it with the headland width.
 */
export function headlandTurns(exit: { x: number; y: number; theta: number }, entry: { x: number; y: number; theta: number }, R: number, footprint: [number, number]): HeadlandTurn[] {
  const out: HeadlandTurn[] = [];
  const u = [Math.cos((exit.theta * Math.PI) / 180), Math.sin((exit.theta * Math.PI) / 180)];
  const spacing = Math.abs(-(entry.x - exit.x) * u[1] + (entry.y - exit.y) * u[0]);
  const depthOf = (path: number[][]) => {
    let d = 0;
    const [L, W] = footprint;
    for (const p of path) {
      const th = (p[2] * Math.PI) / 180, c = Math.cos(th), s = Math.sin(th);
      for (const [a, b] of [[L / 2, W / 2], [L / 2, -W / 2], [-L / 2, W / 2], [-L / 2, -W / 2]]) d = Math.max(d, (p[0] + c * a - s * b - exit.x) * u[0] + (p[1] + s * a + c * b - exit.y) * u[1]);
    }
    return d;
  };
  const fwd = reedsSheppPath(exit, entry, R, { allowReverse: false, step: 100 });
  if (fwd) out.push({ kind: spacing >= 2 * R - 1 ? 'U' : 'omega', path: fwd.poses, length: fwd.length, depth: depthOf(fwd.poses) });
  const rs = reedsSheppPath(exit, entry, R, { allowReverse: true, step: 100, reverseCost: 1.2 });
  if (rs && rs.poses.some((p) => p[3] < 0)) out.push({ kind: 'fishtail', path: rs.poses, length: rs.length, depth: depthOf(rs.poses) });
  return out;
}
import { reedsSheppPath } from './reeds_shepp';
