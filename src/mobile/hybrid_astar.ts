/**
 * Footprint- and curvature-aware planning for ground robots on a CollisionWorld.
 *
 * - `planGridFootprint`: 8-connected A* on a clearance grid (cells whose clearance ≥ inscribed radius + margin), costs
 *   that prefer open corridors, then string pulling that only accepts segments along which the *oriented* footprint
 *   (heading = segment direction) stays free and rotations in place at vertices that sweep free space. Output: a
 *   turn-then-drive pose path for differential / tracked / legged / omni robots.
 * - `planHybridAStar`: Hybrid A* (Dolgov et al.) over (x, y, θ) with arc primitives limited by the turning radius,
 *   optional reverse and rotate-in-place primitives, a 2-D obstacle-aware Dijkstra heuristic and Reeds–Shepp analytic
 *   expansions to hit the goal pose exactly. Output: a pose path with direction per point (cusps for 3-point turns).
 */
import { CollisionWorld, CircleObstacle, Pose2, rectIntersectsPolygon, pointRectDistance } from './footprint';
import { reedsSheppPath } from './reeds_shepp';

export interface FootprintPlanOptions {
  /** Footprint length / width / height (mm). */
  footprint: [number, number, number];
  /** Required clearance between footprint and obstacles (mm). */
  margin?: number;
  /** Final heading (deg); free when undefined. */
  goalHeading?: number;
  /** Polygons the footprint may not touch (traffic segments reserved by others, no-go zones). */
  blocked?: number[][][];
  /** Extra circular obstacles (other robots). */
  extra?: CircleObstacle[];
  maxExpand?: number;
}

export interface PosePathResult { ok: boolean; path: number[][]; length: number; expanded: number; error?: string; method: string }

const wrap180 = (a: number) => ((((a + 180) % 360) + 360) % 360) - 180;
const RAD = 180 / Math.PI, DEG = Math.PI / 180;

/** Collision predicate for a footprint pose on a world (+ blocked polygons and extra circles). */
export function footprintChecker(world: CollisionWorld, o: FootprintPlanOptions): (x: number, y: number, thDeg: number) => boolean {
  const [L, W, H] = o.footprint;
  const m = o.margin ?? 100;
  const blocked = o.blocked ?? [];
  const extra = o.extra ?? [];
  const bb = blocked.map((p) => ({ p, x0: Math.min(...p.map((q) => q[0])), y0: Math.min(...p.map((q) => q[1])), x1: Math.max(...p.map((q) => q[0])), y1: Math.max(...p.map((q) => q[1])) }));
  const R = Math.hypot(L, W) / 2;
  return (x, y, th) => {
    if (!world.inBounds(x, y)) return false;
    if (world.clearance(x, y, th, L, W, H, m) < m) return false;
    for (const c of extra) {
      if (Math.hypot(c.x - x, c.y - y) > R + c.r + m) continue;
      if (pointRectDistance(c.x, c.y, { x, y, theta: th, L, W }) < c.r + m) return false;
    }
    for (const b of bb) {
      if (x + R < b.x0 || x - R > b.x1 || y + R < b.y0 || y - R > b.y1) continue;
      if (rectIntersectsPolygon({ x, y, theta: th, L: L + 2 * m, W: W + 2 * m }, b.p)) return false;
    }
    return true;
  };
}

/** Sweep of an in-place rotation from th0 to th1 (deg) — sampled every 5°. */
function rotationFree(free: (x: number, y: number, th: number) => boolean, x: number, y: number, th0: number, th1: number): boolean {
  const d = wrap180(th1 - th0);
  const n = Math.max(1, Math.ceil(Math.abs(d) / 5));
  for (let i = 1; i <= n; i++) if (!free(x, y, th0 + (d * i) / n)) return false;
  return true;
}

function segmentFree(free: (x: number, y: number, th: number) => boolean, a: number[], b: number[], step: number): boolean {
  const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
  if (len < 1) return true;
  const th = Math.atan2(b[1] - a[1], b[0] - a[0]) * RAD;
  const n = Math.max(1, Math.ceil(len / step));
  for (let i = 1; i <= n; i++) if (!free(a[0] + ((b[0] - a[0]) * i) / n, a[1] + ((b[1] - a[1]) * i) / n, th)) return false;
  return true;
}

class Heap {
  private k: number[] = []; private v: number[] = [];
  get size() { return this.k.length; }
  push(key: number, val: number) {
    const k = this.k, v = this.v; k.push(key); v.push(val);
    let i = k.length - 1;
    while (i > 0) { const p = (i - 1) >> 1; if (k[p] <= k[i]) break; [k[p], k[i]] = [k[i], k[p]]; [v[p], v[i]] = [v[i], v[p]]; i = p; }
  }
  pop(): number {
    const k = this.k, v = this.v; const top = v[0];
    const lk = k.pop()!, lv = v.pop()!;
    if (k.length) {
      k[0] = lk; v[0] = lv; let i = 0;
      for (;;) { const l = 2 * i + 1, r = l + 1; let m = i; if (l < k.length && k[l] < k[m]) m = l; if (r < k.length && k[r] < k[m]) m = r; if (m === i) break; [k[m], k[i]] = [k[i], k[m]]; [v[m], v[i]] = [v[i], v[m]]; i = m; }
    }
    return top;
  }
}

/** Grid resolution for a footprint, bounded so the grid stays below ~600 k cells. */
function gridRes(world: CollisionWorld, W: number): number {
  const b = world.bounds();
  const area = (b.x1 - b.x0) * (b.y1 - b.y0);
  const base = W >= 1200 ? 250 : W >= 700 ? 150 : 100;
  return Math.max(base, Math.ceil(Math.sqrt(area / 600000) / 50) * 50);
}

/** Obstacle-aware 2-D cost-to-goal (Dijkstra on the clearance grid, inscribed-radius feasibility). */
function dijkstra2D(world: CollisionWorld, o: FootprintPlanOptions, goal: [number, number], res: number): { g: Float64Array; w: number; h: number; x0: number; y0: number; res: number; ok: (i: number) => boolean } {
  const [L, W, H] = o.footprint;
  const m = o.margin ?? 100;
  const cg = world.clearanceGrid(res, H);
  const need = W / 2 + m;
  const extra = o.extra ?? [];
  const blocked = o.blocked ?? [];
  const okArr = new Uint8Array(cg.w * cg.h);
  for (let j = 0; j < cg.h; j++) for (let i = 0; i < cg.w; i++) {
    const idx = j * cg.w + i;
    if (cg.c[idx] < need - res * 0.5) continue;
    const x = cg.x0 + (i + 0.5) * res, y = cg.y0 + (j + 0.5) * res;
    if (extra.some((c) => Math.hypot(c.x - x, c.y - y) < c.r + need - res * 0.5)) continue;
    if (blocked.some((p) => rectIntersectsPolygon({ x, y, theta: 0, L: W, W }, p))) continue;
    okArr[idx] = 1;
  }
  const g = new Float64Array(cg.w * cg.h).fill(Infinity);
  const gi = Math.floor((goal[0] - cg.x0) / res), gj = Math.floor((goal[1] - cg.y0) / res);
  const heap = new Heap();
  const start = (i: number, j: number) => { if (i >= 0 && j >= 0 && i < cg.w && j < cg.h) { g[j * cg.w + i] = 0; heap.push(0, j * cg.w + i); } };
  start(gi, gj);
  void L;
  const dirs = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2]];
  while (heap.size) {
    const cur = heap.pop();
    const ci = cur % cg.w, cj = (cur / cg.w) | 0;
    const gc = g[cur];
    for (const [di, dj, c] of dirs) {
      const ni = ci + di, nj = cj + dj;
      if (ni < 0 || nj < 0 || ni >= cg.w || nj >= cg.h) continue;
      const n = nj * cg.w + ni;
      if (!okArr[n]) continue;
      const ng = gc + c * res;
      if (ng < g[n]) { g[n] = ng; heap.push(ng, n); }
    }
  }
  return { g, w: cg.w, h: cg.h, x0: cg.x0, y0: cg.y0, res, ok: (i) => okArr[i] === 1 };
}

/**
 * Footprint-aware grid A* + oriented string pulling (turn-then-drive pose path). Where the robot cannot turn on the
 * spot at the start (leaving a crop alley) it first drives straight out along its heading; with a goal heading it
 * plans to a pre-goal point on the goal axis, turns there and drives straight in (entering an alley aligned).
 */
export function planGridFootprint(world: CollisionWorld, start: Pose2, goal: { x: number; y: number }, o: FootprintPlanOptions): PosePathResult {
  const L = o.footprint[0];
  const free = footprintChecker(world, o);
  const spinFree = (x: number, y: number) => { for (let a = 0; a < 360; a += 10) if (!free(x, y, a)) return false; return true; };
  const head: number[][] = [];
  let s0 = start;
  if (!spinFree(start.x, start.y)) {
    // exit manoeuvre straight ahead (then behind) until the robot can turn freely
    const u = [Math.cos(start.theta * DEG), Math.sin(start.theta * DEG)];
    let found = false;
    for (const sgn of [1, -1]) {
      for (const D of [L * 0.5 + 400, L + 800, 1.5 * L + 1500, 2 * L + 2500, 3 * L + 4000]) {
        const e = [start.x + u[0] * D * sgn, start.y + u[1] * D * sgn];
        if (!segmentFreeDir(free, [start.x, start.y], e, start.theta, 150)) break;
        if (!spinFree(e[0], e[1])) continue;
        const n = Math.max(1, Math.ceil(D / 200));
        for (let i = 1; i <= n; i++) head.push([start.x + ((e[0] - start.x) * i) / n, start.y + ((e[1] - start.y) * i) / n, start.theta, sgn]);
        s0 = { x: e[0], y: e[1], theta: start.theta };
        found = true;
        break;
      }
      if (found) break;
    }
  }
  let tail: number[][] = [];
  let g0 = goal;
  if (o.goalHeading !== undefined && !spinFree(goal.x, goal.y)) {
    const u = [Math.cos(o.goalHeading * DEG), Math.sin(o.goalHeading * DEG)];
    for (const D of [L * 0.5 + 400, L + 800, 1.5 * L + 1500, 2 * L + 2500, 3 * L + 4000]) {
      const pg = [goal.x - u[0] * D, goal.y - u[1] * D];
      if (!segmentFreeDir(free, pg, [goal.x, goal.y], o.goalHeading, 150)) break;
      if (!spinFree(pg[0], pg[1])) continue;
      const n = Math.max(1, Math.ceil(D / 200));
      tail = [[pg[0], pg[1], o.goalHeading, 0]];
      for (let i = 1; i <= n; i++) tail.push([pg[0] + ((goal.x - pg[0]) * i) / n, pg[1] + ((goal.y - pg[1]) * i) / n, o.goalHeading, 1]);
      g0 = { x: pg[0], y: pg[1] };
      break;
    }
  }
  const core = planGridCore(world, s0, g0, tail.length ? { ...o, goalHeading: undefined } : o);
  if (!core.ok) return core;
  const path = [[start.x, start.y, start.theta, 1], ...head, ...core.path.slice(1)];
  if (tail.length) {
    // the rotation onto the goal axis happens at the pre-goal point (skip it when already aligned)
    if (Math.abs(wrap180(path[path.length - 1][2] - (o.goalHeading as number))) <= 0.5) tail.shift();
    path.push(...tail);
  }
  let length = 0;
  for (let k = 1; k < path.length; k++) length += Math.hypot(path[k][0] - path[k - 1][0], path[k][1] - path[k - 1][1]);
  return { ...core, path, length };
}

function segmentFreeDir(free: (x: number, y: number, th: number) => boolean, a: number[], b: number[], th: number, step: number): boolean {
  const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
  const n = Math.max(1, Math.ceil(len / step));
  for (let i = 1; i <= n; i++) if (!free(a[0] + ((b[0] - a[0]) * i) / n, a[1] + ((b[1] - a[1]) * i) / n, th)) return false;
  return true;
}

function planGridCore(world: CollisionWorld, start: Pose2, goal: { x: number; y: number }, o: FootprintPlanOptions): PosePathResult {
  const [L, W, H] = o.footprint;
  const m = o.margin ?? 100;
  const free = footprintChecker(world, o);
  const res = gridRes(world, W);
  const cg = world.clearanceGrid(res, H);
  const need = W / 2 + m;
  const Rc = Math.hypot(L, W) / 2 + m;
  const extra = o.extra ?? [];
  const blocked = o.blocked ?? [];
  const cell = (x: number, y: number): [number, number] => [Math.floor((x - cg.x0) / res), Math.floor((y - cg.y0) / res)];
  const [si, sj] = cell(start.x, start.y), [gi, gj] = cell(goal.x, goal.y);
  const inG = (i: number, j: number) => i >= 0 && j >= 0 && i < cg.w && j < cg.h;
  if (!inG(gi, gj)) return { ok: false, path: [], length: 0, expanded: 0, error: 'Goal outside the map', method: 'grid' };
  const okCache = new Int8Array(cg.w * cg.h); // 0 unknown, 1 ok, -1 blocked
  const ok = (i: number, j: number) => {
    const idx = j * cg.w + i;
    if (okCache[idx]) return okCache[idx] > 0;
    let v = cg.c[idx] >= need;
    const x = cg.x0 + (i + 0.5) * res, y = cg.y0 + (j + 0.5) * res;
    if (v && extra.length) v = !extra.some((c) => Math.hypot(c.x - x, c.y - y) < c.r + need);
    if (v && blocked.length) v = !blocked.some((p) => rectIntersectsPolygon({ x, y, theta: 0, L: W + 2 * m, W: W + 2 * m }, p));
    okCache[idx] = v ? 1 : -1;
    return v;
  };
  if (!ok(gi, gj)) return { ok: false, path: [], length: 0, expanded: 0, error: 'Goal is in an obstacle', method: 'grid' };
  const g = new Float64Array(cg.w * cg.h).fill(Infinity);
  const parent = new Int32Array(cg.w * cg.h).fill(-1);
  const closed = new Uint8Array(cg.w * cg.h);
  const h = (i: number, j: number) => { const di = Math.abs(i - gi), dj = Math.abs(j - gj); return (Math.max(di, dj) + (Math.SQRT2 - 1) * Math.min(di, dj)) * res; };
  const heap = new Heap();
  const s0 = sj * cg.w + si;
  g[s0] = 0; heap.push(h(si, sj), s0);
  // the start may lie in the inflated band (robot next to a tree): allow leaving it within 2 m
  const nearStart = (i: number, j: number) => Math.hypot(i - si, j - sj) * res < Math.max(2000, L);
  const dirs = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2]];
  let expanded = 0, found = false;
  const maxExpand = o.maxExpand ?? cg.w * cg.h;
  while (heap.size) {
    const cur = heap.pop();
    if (closed[cur]) continue;
    closed[cur] = 1;
    if (++expanded > maxExpand) break;
    const ci = cur % cg.w, cj = (cur / cg.w) | 0;
    if (ci === gi && cj === gj) { found = true; break; }
    for (const [di, dj, c] of dirs) {
      const ni = ci + di, nj = cj + dj;
      if (!inG(ni, nj)) continue;
      const n = nj * cg.w + ni;
      if (closed[n]) continue;
      const okN = ok(ni, nj);
      if (!okN && !(nearStart(ni, nj) && cg.c[n] > 0)) continue;
      if (di && dj && !(ok(ci + di, cj) || nearStart(ci + di, cj)) && !(ok(ci, cj + dj) || nearStart(ci, cj + dj))) continue;
      const clr = cg.c[n];
      const pen = 1 + 2.5 * Math.max(0, (Rc - clr) / Rc) + (okN ? 0 : 4);
      const ng = g[cur] + c * res * pen;
      if (ng < g[n]) { g[n] = ng; parent[n] = cur; heap.push(ng + h(ni, nj), n); }
    }
  }
  if (!found) return { ok: false, path: [], length: 0, expanded, error: 'No path found', method: 'grid' };
  const cells: number[] = [];
  for (let i = gj * cg.w + gi; i !== -1; i = parent[i]) cells.push(i);
  cells.reverse();
  const raw = cells.map((i) => [cg.x0 + ((i % cg.w) + 0.5) * res, cg.y0 + (((i / cg.w) | 0) + 0.5) * res]);
  raw[0] = [start.x, start.y];
  raw[raw.length - 1] = [goal.x, goal.y];
  // oriented string pulling: farthest visible point whose straight segment sweeps free space with the rotation at i
  const step = Math.min(200, res);
  const pts: number[][] = [raw[0]];
  let heading = start.theta;
  let i = 0;
  while (i < raw.length - 1) {
    let j = raw.length - 1;
    let okJ = -1;
    for (; j > i; j--) {
      const a = raw[i], b = raw[j];
      const th = Math.atan2(b[1] - a[1], b[0] - a[0]) * RAD;
      if (!segmentFree(free, a, b, step)) continue;
      if (Math.abs(wrap180(th - heading)) > 8 && !rotationFree(free, a[0], a[1], heading, th) && !(i === 0 && rotationNearStartOk(free, a, heading, th))) continue;
      okJ = j;
      heading = th;
      break;
    }
    if (okJ < 0) return { ok: false, path: [], length: 0, expanded, error: `Footprint does not fit near (${raw[i][0].toFixed(0)}, ${raw[i][1].toFixed(0)})`, method: 'grid' };
    pts.push(raw[okJ]);
    i = okJ;
  }
  if (o.goalHeading !== undefined && !rotationFree(free, goal.x, goal.y, heading, o.goalHeading)) return { ok: false, path: [], length: 0, expanded, error: 'Cannot turn to the goal heading', method: 'grid' };
  // pose path
  const out: number[][] = [[start.x, start.y, start.theta, 1]];
  let hd = start.theta;
  for (let k = 1; k < pts.length; k++) {
    const a = pts[k - 1], b = pts[k];
    const th = Math.atan2(b[1] - a[1], b[0] - a[0]) * RAD;
    if (Math.abs(wrap180(th - hd)) > 8) out.push([a[0], a[1], th, 0]);
    const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
    const n = Math.max(1, Math.ceil(len / 200));
    for (let s = 1; s <= n; s++) out.push([a[0] + ((b[0] - a[0]) * s) / n, a[1] + ((b[1] - a[1]) * s) / n, th, 1]);
    hd = th;
  }
  if (o.goalHeading !== undefined && Math.abs(wrap180(o.goalHeading - hd)) > 0.5) out.push([goal.x, goal.y, o.goalHeading, 0]);
  let length = 0;
  for (let k = 1; k < pts.length; k++) length += Math.hypot(pts[k][0] - pts[k - 1][0], pts[k][1] - pts[k - 1][1]);
  return { ok: true, path: out, length, expanded, method: 'grid' };
}

/** The robot may already be in contact at its start pose: accept the start rotation if it does not get worse. */
function rotationNearStartOk(free: (x: number, y: number, th: number) => boolean, a: number[], th0: number, th1: number): boolean {
  return free(a[0], a[1], th1) && !free(a[0], a[1], th0);
}

export interface HybridOptions extends FootprintPlanOptions {
  /** Turning radius used by the arc primitives (mm). */
  radius: number;
  allowReverse?: boolean;
  /** Rotate-in-place primitives (differential / tracked / legged). */
  inPlace?: boolean;
  reverseCost?: number;
  /** Heading tolerance at the goal when it is reached by a primitive (deg). */
  headingTolerance?: number;
}

interface HNode { x: number; y: number; th: number; g: number; parent: number; dir: number; kappa: number; len: number; rot: number }

/** Hybrid A* with Reeds–Shepp (or Dubins) analytic expansion. */
export function planHybridAStar(world: CollisionWorld, start: Pose2, goal: Pose2 | { x: number; y: number; theta?: number }, o: HybridOptions): PosePathResult {
  const [L, W] = o.footprint;
  const free = footprintChecker(world, o);
  const R = Math.max(o.radius, 1);
  const allowRev = o.allowReverse ?? true;
  const rc = o.reverseCost ?? 2;
  const goalFree = goal.theta === undefined;
  const goalThetas = goalFree ? [0, 45, 90, 135, 180, -135, -90, -45].map((d) => d + Math.atan2(goal.y - start.y, goal.x - start.x) * RAD) : [goal.theta as number];
  if (!goalThetas.some((t) => free(goal.x, goal.y, t))) return { ok: false, path: [], length: 0, expanded: 0, error: 'Goal pose is in collision', method: 'hybrid' };
  const xyRes = Math.max(250, Math.min(600, Math.round(W / 3 / 50) * 50 || 300));
  const NB = 72;
  const step = Math.max(xyRes * 1.6, 500);
  const sample = 150;
  const h2 = dijkstra2D(world, o, [goal.x, goal.y], gridRes(world, W));
  const h2At = (x: number, y: number) => {
    const i = Math.floor((x - h2.x0) / h2.res), j = Math.floor((y - h2.y0) / h2.res);
    if (i < 0 || j < 0 || i >= h2.w || j >= h2.h) return Infinity;
    return h2.g[j * h2.w + i];
  };
  const kmax = 1 / R;
  const kappas = o.inPlace ? [-kmax, 0, kmax] : [-kmax, -kmax / 2, 0, kmax / 2, kmax];
  const prims: Array<{ dir: number; kappa: number; rot: number }> = [];
  for (const dir of allowRev ? [1, -1] : [1]) for (const k of kappas) prims.push({ dir, kappa: k, rot: 0 });
  if (o.inPlace) prims.push({ dir: 0, kappa: 0, rot: 15 }, { dir: 0, kappa: 0, rot: -15 });
  const nodes: HNode[] = [];
  const best = new Map<number, number>(); // key -> node index
  const key = (x: number, y: number, th: number) => {
    const ix = Math.floor(x / xyRes) + 100000, iy = Math.floor(y / xyRes) + 100000;
    const it = ((Math.round(((th % 360) + 360) % 360 / (360 / NB)) % NB) + NB) % NB;
    return (ix * 200003 + iy) * NB + it;
  };
  const heap = new Heap();
  const hOf = (x: number, y: number) => Math.max(Math.hypot(goal.x - x, goal.y - y), h2At(x, y));
  nodes.push({ x: start.x, y: start.y, th: start.theta, g: 0, parent: -1, dir: 1, kappa: 0, len: 0, rot: 0 });
  best.set(key(start.x, start.y, start.theta), 0);
  heap.push(hOf(start.x, start.y), 0);
  const maxExpand = o.maxExpand ?? 40000;
  let expanded = 0;
  let shot: { node: number; poses: number[][] } | null = null;
  let bestH = Infinity;
  const tryShot = (ni: number): number[][] | null => {
    const n = nodes[ni];
    let bestPoses: number[][] | null = null, bestCost = Infinity;
    for (const gt of goalThetas) {
      const rs = reedsSheppPath({ x: n.x, y: n.y, theta: n.th }, { x: goal.x, y: goal.y, theta: gt }, R, { allowReverse: allowRev, step: sample, reverseCost: rc });
      if (!rs || rs.cost >= bestCost) continue;
      let okP = true;
      for (const p of rs.poses) if (!free(p[0], p[1], p[2])) { okP = false; break; }
      if (okP) { bestCost = rs.cost; bestPoses = rs.poses; }
    }
    return bestPoses;
  };
  while (heap.size) {
    const ci = heap.pop();
    const c = nodes[ci];
    if (best.get(key(c.x, c.y, c.th)) !== ci) continue;
    if (++expanded > maxExpand) break;
    const hc = h2At(c.x, c.y);
    if (hc < bestH || expanded % 8 === 0 || hc < 4 * R) {
      bestH = Math.min(bestH, hc);
      const p = tryShot(ci);
      if (p) { shot = { node: ci, poses: p }; break; }
    }
    for (const pr of prims) {
      let x = c.x, y = c.y, th = c.th * DEG;
      let okP = true;
      if (pr.dir === 0) {
        const th1 = c.th + pr.rot;
        if (!rotationFree(free, x, y, c.th, th1)) continue;
        th = th1 * DEG;
      } else {
        const n = Math.max(1, Math.ceil(step / sample));
        const ds = (pr.dir * step) / n;
        for (let s = 1; s <= n; s++) {
          if (Math.abs(pr.kappa) < 1e-12) { x += ds * Math.cos(th); y += ds * Math.sin(th); }
          else { const dth = ds * pr.kappa; x += (Math.sin(th + dth) - Math.sin(th)) / pr.kappa; y += (-Math.cos(th + dth) + Math.cos(th)) / pr.kappa; th += dth; }
          if (!free(x, y, th * RAD)) { okP = false; break; }
        }
        if (!okP) continue;
      }
      const thd = wrap180(th * RAD);
      const cost = pr.dir === 0 ? (Math.abs(pr.rot) * DEG) * (W / 2) + 200 : step * (pr.dir < 0 ? rc : 1) * (1 + 0.15 * Math.abs(pr.kappa) * R) + (c.parent >= 0 && pr.dir !== 0 && c.dir !== 0 && pr.dir !== c.dir ? 2 * R : 0);
      const ng = c.g + cost;
      const k = key(x, y, thd);
      const prev = best.get(k);
      if (prev !== undefined && nodes[prev].g <= ng) continue;
      const hn = hOf(x, y);
      if (!Number.isFinite(hn)) continue;
      nodes.push({ x, y, th: thd, g: ng, parent: ci, dir: pr.dir, kappa: pr.kappa, len: step, rot: pr.rot });
      best.set(k, nodes.length - 1);
      heap.push(ng + hn * 1.05, nodes.length - 1);
    }
  }
  if (!shot) return { ok: false, path: [], length: 0, expanded, error: expanded > maxExpand ? 'Hybrid A*: expansion limit' : 'Hybrid A*: no path', method: 'hybrid' };
  // reconstruct
  const chain: number[] = [];
  for (let i = shot.node; i !== -1; i = nodes[i].parent) chain.push(i);
  chain.reverse();
  const out: number[][] = [[start.x, start.y, start.theta, 1]];
  for (let q = 1; q < chain.length; q++) {
    const p = nodes[chain[q - 1]], n = nodes[chain[q]];
    if (n.dir === 0) { out.push([p.x, p.y, n.th, 0]); continue; }
    let x = p.x, y = p.y, th = p.th * DEG;
    const cnt = Math.max(1, Math.ceil(step / sample));
    const ds = (n.dir * step) / cnt;
    for (let s = 1; s <= cnt; s++) {
      if (Math.abs(n.kappa) < 1e-12) { x += ds * Math.cos(th); y += ds * Math.sin(th); }
      else { const dth = ds * n.kappa; x += (Math.sin(th + dth) - Math.sin(th)) / n.kappa; y += (-Math.cos(th + dth) + Math.cos(th)) / n.kappa; th += dth; }
      out.push([x, y, wrap180(th * RAD), n.dir]);
    }
  }
  out.push(...shot.poses.slice(1));
  let length = 0;
  for (let q = 1; q < out.length; q++) length += Math.hypot(out[q][0] - out[q - 1][0], out[q][1] - out[q - 1][1]);
  return { ok: true, path: out, length, expanded, method: 'hybrid' };
}
