/**
 * Multi-agent path finding on a warehouse grid (course lecture 10, §11.5–11.6, practicum ПР4): space-time A* with
 * vertex / edge constraints, prioritized planning with a reservation table, first-conflict detection, conflict-based
 * search (CBS) and execution of the plan through an action dependency graph (ADG) that tolerates delays.
 */
import { Rng } from './rng';

export type Cell = [number, number];
export type Grid = boolean[][]; // true = shelf (blocked)
export type Path = Cell[];
export type Constraint = { kind: 'v'; cell: Cell; t: number } | { kind: 'e'; from: Cell; to: Cell; t: number };
export type Conflict = { kind: 'v'; a: number; b: number; cell: Cell; t: number } | { kind: 'e'; a: number; b: number; u: Cell; w: Cell; t: number };

const MOVES: Cell[] = [[0, 0], [1, 0], [-1, 0], [0, 1], [0, -1]];
export const key = (c: Cell) => `${c[0]},${c[1]}`;
export const same = (a: Cell, b: Cell) => a[0] === b[0] && a[1] === b[1];

/** Map from text: '#' shelf, '.' free. */
export function loadMap(text: string): Grid { return text.replace(/^\n+|\n+$/g, '').split('\n').map((r) => [...r.trim()].map((ch) => ch === '#')); }
export function freeCells(grid: Grid): Cell[] { const out: Cell[] = []; grid.forEach((row, r) => row.forEach((b, c) => { if (!b) out.push([r, c]); })); return out; }
export function successors(grid: Grid, cell: Cell): Cell[] { const out: Cell[] = []; for (const [dr, dc] of MOVES) { const r = cell[0] + dr, c = cell[1] + dc; if (r >= 0 && r < grid.length && c >= 0 && c < grid[0].length && !grid[r][c]) out.push([r, c]); } return out; }
export const manhattan = (a: Cell, b: Cell) => Math.abs(a[0] - b[0]) + Math.abs(a[1] - b[1]);
/** Cell of an agent at time t (parked at its last cell after the end). */
export const at = (path: Path, t: number): Cell => (t < path.length ? path[t] : path[path.length - 1]);

const cKey = (c: Constraint) => (c.kind === 'v' ? `v|${key(c.cell)}|${c.t}` : `e|${key(c.from)}|${key(c.to)}|${c.t}`);

class Heap<T> { a: T[] = []; constructor(private lt: (x: T, y: T) => boolean) {} get size() { return this.a.length; } push(x: T) { this.a.push(x); let i = this.a.length - 1; while (i > 0) { const p = (i - 1) >> 1; if (this.lt(this.a[i], this.a[p])) { [this.a[i], this.a[p]] = [this.a[p], this.a[i]]; i = p; } else break; } } pop(): T | undefined { if (!this.a.length) return undefined; const top = this.a[0]; const last = this.a.pop()!; if (this.a.length) { this.a[0] = last; let i = 0; for (;;) { const l = 2 * i + 1, r = l + 1; let m = i; if (l < this.a.length && this.lt(this.a[l], this.a[m])) m = l; if (r < this.a.length && this.lt(this.a[r], this.a[m])) m = r; if (m === i) break; [this.a[i], this.a[m]] = [this.a[m], this.a[i]]; i = m; } } return top; } }

/** Space-time A*: shortest path with constraints; the goal counts only when no later vertex constraint would evict the agent. */
export function spaceTimeAStar(grid: Grid, start: Cell, goal: Cell, constraints: Constraint[] = [], maxT?: number): Path | null {
  const cs = new Set(constraints.map(cKey)); let lastGoalCon = -1; for (const c of constraints) if (c.kind === 'v' && same(c.cell, goal)) lastGoalCon = Math.max(lastGoalCon, c.t);
  const maxCT = constraints.reduce((m, c) => Math.max(m, c.t), 0); const horizon = maxT ?? grid.length * grid[0].length + maxCT + 1;
  type Node = { f: number; g: number; tie: number; cell: Cell; t: number };
  let tie = 0; const open = new Heap<Node>((x, y) => x.f !== y.f ? x.f < y.f : x.g !== y.g ? x.g > y.g : x.tie < y.tie);
  const parent = new Map<string, string | null>(); const sk = `${key(start)}|0`; parent.set(sk, null); open.push({ f: manhattan(start, goal), g: 0, tie: tie++, cell: start, t: 0 });
  const closed = new Set<string>();
  while (open.size) {
    const n = open.pop()!; const nk = `${key(n.cell)}|${n.t}`; if (closed.has(nk)) continue; closed.add(nk);
    if (same(n.cell, goal) && n.t > lastGoalCon) { const path: Path = []; let k: string | null = nk; while (k) { const [rc] = k.split('|'); const [r, c] = rc.split(',').map(Number); path.push([r, c]); k = parent.get(k) ?? null; } return path.reverse(); }
    if (n.t >= horizon) continue;
    for (const s of successors(grid, n.cell)) { const t = n.t + 1; if (cs.has(`v|${key(s)}|${t}`) || cs.has(`e|${key(n.cell)}|${key(s)}|${t}`)) continue; const k2 = `${key(s)}|${t}`; if (closed.has(k2) || parent.has(k2)) continue; parent.set(k2, nk); open.push({ f: t + manhattan(s, goal), g: t, tie: tie++, cell: s, t }); }
  }
  return null;
}
/** Constraints a higher-priority path imposes: its vertex at every time up to the horizon (parking included) and no swap. */
export function pathConstraints(path: Path, horizon: number): Constraint[] { const cons: Constraint[] = []; for (let t = 0; t <= horizon; t++) cons.push({ kind: 'v', cell: at(path, t), t }); for (let t = 1; t < path.length; t++) cons.push({ kind: 'e', from: path[t], to: path[t - 1], t }); return cons; }
/** Prioritized planning (not complete): agents plan in `order`, each avoiding all previous paths. */
export function prioritizedPlanning(grid: Grid, starts: Cell[], goals: Cell[], order?: number[], horizon?: number): Path[] | null {
  const ord = order ?? [...starts.keys()]; const H = horizon ?? freeCells(grid).length + starts.length; const paths: Path[] = Array(starts.length); let cons: Constraint[] = [];
  for (const i of ord) { const p = spaceTimeAStar(grid, starts[i], goals[i], cons, H); if (!p) return null; paths[i] = p; cons = cons.concat(pathConstraints(p, H)); }
  return paths;
}
export const sumOfCosts = (paths: Path[]) => paths.reduce((s, p) => s + p.length - 1, 0);
export const makespanOf = (paths: Path[]) => Math.max(...paths.map((p) => p.length - 1));
/** First conflict in time: vertex conflicts of pairs (a < b) first, then edge (swap) conflicts. */
export function firstConflict(paths: Path[]): Conflict | null {
  const T = Math.max(...paths.map((p) => p.length)); const n = paths.length;
  for (let t = 0; t < T; t++) {
    for (let a = 0; a < n; a++) for (let b = a + 1; b < n; b++) if (same(at(paths[a], t), at(paths[b], t))) return { kind: 'v', a, b, cell: at(paths[a], t), t };
    if (t > 0) for (let a = 0; a < n; a++) for (let b = a + 1; b < n; b++) { const ua = at(paths[a], t - 1), wa = at(paths[a], t), ub = at(paths[b], t - 1), wb = at(paths[b], t); if (same(ua, wb) && same(wa, ub) && !same(ua, wa)) return { kind: 'e', a, b, u: ua, w: wa, t }; }
  }
  return null;
}
/** Conflict-based search (Sharon et al. 2015): high level over constraint trees, low level space-time A*. */
export function cbs(grid: Grid, starts: Cell[], goals: Cell[], maxNodes = 20000): { paths: Path[] | null; nodes: number } {
  type CTNode = { cost: number; id: number; cons: Constraint[][]; paths: Path[] };
  const root: Path[] = []; for (let i = 0; i < starts.length; i++) { const p = spaceTimeAStar(grid, starts[i], goals[i]); if (!p) return { paths: null, nodes: 0 }; root.push(p); }
  let id = 0; const open = new Heap<CTNode>((x, y) => x.cost !== y.cost ? x.cost < y.cost : x.id < y.id); open.push({ cost: sumOfCosts(root), id: id++, cons: starts.map(() => []), paths: root });
  let expanded = 0;
  while (open.size && expanded < maxNodes) {
    const n = open.pop()!; expanded++; const c = firstConflict(n.paths); if (!c) return { paths: n.paths, nodes: expanded };
    const branches: Array<[number, Constraint]> = c.kind === 'v' ? [[c.a, { kind: 'v', cell: c.cell, t: c.t }], [c.b, { kind: 'v', cell: c.cell, t: c.t }]] : [[c.a, { kind: 'e', from: c.u, to: c.w, t: c.t }], [c.b, { kind: 'e', from: c.w, to: c.u, t: c.t }]];
    for (const [ag, con] of branches) { const cons = n.cons.map((x, i) => (i === ag ? [...x, con] : x)); const p = spaceTimeAStar(grid, starts[ag], goals[ag], cons[ag]); if (!p) continue; const paths = n.paths.map((x, i) => (i === ag ? p : x)); open.push({ cost: sumOfCosts(paths), id: id++, cons, paths }); }
  }
  return { paths: null, nodes: expanded };
}

export type Action = { from: Cell; to: Cell; t: number };
/** Path → moves (waits dropped) with their planned arrival times. */
export function compress(path: Path): Action[] { const out: Action[] = []; for (let t = 1; t < path.length; t++) if (!same(path[t], path[t - 1])) out.push({ from: path[t - 1], to: path[t], t }); return out; }
/** Simplified action dependency graph (Hönig et al. 2019): (i,k) entering cell c at planned time t depends on the exits of every agent that was in c earlier. */
export function actionDependencies(paths: Path[]): { actions: Action[][]; deps: Map<string, Set<string>> } {
  const actions = paths.map(compress); const deps = new Map<string, Set<string>>();
  type Visit = { cell: Cell; enter: number; exit: number | null };
  const visits = actions.map((acts, j) => { const v: Visit[] = [{ cell: paths[j][0], enter: 0, exit: null }]; acts.forEach((a, l) => { v[v.length - 1].exit = l; v.push({ cell: a.to, enter: a.t, exit: null }); }); return v; });
  actions.forEach((acts, i) => acts.forEach((a, k) => { const d = new Set<string>(); visits.forEach((vs, j) => { if (j === i) return; for (const v of vs) if (same(v.cell, a.to) && v.enter < a.t && v.exit !== null) d.add(`${j},${v.exit}`); }); deps.set(`${i},${k}`, d); }));
  return { actions, deps };
}
/** Execute through the ADG: an agent performs its next move when the dependencies are done and it is not delayed (probability delayProb). */
export function executeAdg(paths: Path[], delayProb: number, rng: Rng, maxTicks = 10000): { ticks: number | null; log: Cell[][]; collisions: number } {
  const { actions, deps } = actionDependencies(paths); const nxt = paths.map(() => 0); const pos = paths.map((p) => p[0]); const done = new Set<string>(); const log: Cell[][] = [pos.slice()]; let collisions = 0;
  for (let tick = 1; tick <= maxTicks; tick++) {
    if (nxt.every((k, i) => k === actions[i].length)) return { ticks: tick - 1, log, collisions };
    const ready: number[] = [];
    for (let i = 0; i < paths.length; i++) { if (nxt[i] === actions[i].length || rng.random() < delayProb) continue; const d = deps.get(`${i},${nxt[i]}`)!; let ok = true; for (const x of d) if (!done.has(x)) { ok = false; break; } if (ok) ready.push(i); }
    for (const i of ready) { pos[i] = actions[i][nxt[i]].to; done.add(`${i},${nxt[i]}`); nxt[i]++; }
    collisions += pos.length - new Set(pos.map(key)).size; log.push(pos.slice());
  }
  return { ticks: null, log, collisions };
}
/** Clock-driven execution without coordination (for comparison): counts pairs of robots in the same cell. */
export function executeNaive(paths: Path[], delayProb: number, rng: Rng, maxTicks = 10000): number {
  const actions = paths.map(compress); const nxt = paths.map(() => 0); const pos = paths.map((p) => p[0]); let collisions = 0;
  for (let tick = 1; tick <= maxTicks; tick++) { if (nxt.every((k, i) => k === actions[i].length)) break; for (let i = 0; i < paths.length; i++) if (nxt[i] < actions[i].length && tick >= actions[i][nxt[i]].t && rng.random() >= delayProb) { pos[i] = actions[i][nxt[i]].to; nxt[i]++; } collisions += pos.length - new Set(pos.map(key)).size; }
  return collisions;
}
export const WAREHOUSE_MAP = `..........
.##.##.##.
..........
.##.##.##.
..........
.##.##.##.
..........`;
