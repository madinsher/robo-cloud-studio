/**
 * Multi-robot coordination (course chapter 13): task allocation (Hungarian algorithm for ST-SR-IA, bottleneck /
 * minimax assignment, sequential greedy auction, consensus-based bundle algorithm CBBA) and multi-agent path
 * finding (priority planning with space-time A*, conflict-based search CBS, the temporal plan graph TPG with an
 * acyclicity check, section reservation with a structural deadlock check by resource ordering).
 */
import { resourceOrder } from './petri';

// ---------------------------------------------------------------------------------------------
// Assignment
// ---------------------------------------------------------------------------------------------

/** Hungarian (Kuhn–Munkres) algorithm on a rectangular cost matrix (rows = robots, cols = tasks); Infinity = incompatible. Returns assignment[row] = col or −1. */
export function hungarian(cost: number[][]): { assignment: number[]; total: number } {
  const n = cost.length, m = cost[0]?.length ?? 0;
  if (!n || !m) return { assignment: new Array(n).fill(-1), total: 0 };
  const N = Math.max(n, m); const BIG = 1e9;
  const a = Array.from({ length: N + 1 }, (_, i) => Array.from({ length: N + 1 }, (_, j) => (i >= 1 && j >= 1 && i <= n && j <= m ? (Number.isFinite(cost[i - 1][j - 1]) ? cost[i - 1][j - 1] : BIG) : 0)));
  const u = new Array(N + 1).fill(0), v = new Array(N + 1).fill(0), p = new Array(N + 1).fill(0), way = new Array(N + 1).fill(0);
  for (let i = 1; i <= N; i++) {
    p[0] = i; let j0 = 0; const minv = new Array(N + 1).fill(Infinity); const used = new Array(N + 1).fill(false);
    do {
      used[j0] = true; const i0 = p[j0]; let delta = Infinity, j1 = 0;
      for (let j = 1; j <= N; j++) if (!used[j]) { const cur = a[i0][j] - u[i0] - v[j]; if (cur < minv[j]) { minv[j] = cur; way[j] = j0; } if (minv[j] < delta) { delta = minv[j]; j1 = j; } }
      for (let j = 0; j <= N; j++) if (used[j]) { u[p[j]] += delta; v[j] -= delta; } else minv[j] -= delta;
      j0 = j1;
    } while (p[j0] !== 0);
    do { const j1 = way[j0]; p[j0] = p[j1]; j0 = j1; } while (j0);
  }
  const assignment = new Array(n).fill(-1); let total = 0;
  for (let j = 1; j <= N; j++) { const i = p[j]; if (i >= 1 && i <= n && j <= m && Number.isFinite(cost[i - 1][j - 1])) { assignment[i - 1] = j - 1; total += cost[i - 1][j - 1]; } }
  return { assignment, total };
}

/** Greedy assignment (repeatedly take the globally cheapest free pair) — the baseline the Hungarian algorithm beats. */
export function greedyAssignment(cost: number[][]): { assignment: number[]; total: number } {
  const n = cost.length, m = cost[0]?.length ?? 0; const assignment = new Array(n).fill(-1); const usedCol = new Set<number>(); let total = 0;
  for (let k = 0; k < Math.min(n, m); k++) {
    let best = Infinity, bi = -1, bj = -1;
    for (let i = 0; i < n; i++) if (assignment[i] < 0) for (let j = 0; j < m; j++) if (!usedCol.has(j) && cost[i][j] < best) { best = cost[i][j]; bi = i; bj = j; }
    if (bi < 0) break; assignment[bi] = bj; usedCol.add(bj); total += best;
  }
  return { assignment, total };
}

/** Bottleneck assignment (minimise the maximum cost): binary search on the threshold with a bipartite matching test. */
export function bottleneckAssignment(cost: number[][]): { assignment: number[]; makespan: number } {
  const n = cost.length, m = cost[0]?.length ?? 0;
  const values = [...new Set(cost.flat().filter(Number.isFinite))].sort((a, b) => a - b);
  const matching = (thr: number): number[] | null => {
    const matchCol = new Array(m).fill(-1);
    const tryRow = (i: number, seen: boolean[]): boolean => { for (let j = 0; j < m; j++) if (cost[i][j] <= thr && !seen[j]) { seen[j] = true; if (matchCol[j] < 0 || tryRow(matchCol[j], seen)) { matchCol[j] = i; return true; } } return false; };
    for (let i = 0; i < n; i++) if (!tryRow(i, new Array(m).fill(false))) return null;
    const assignment = new Array(n).fill(-1); matchCol.forEach((i, j) => { if (i >= 0) assignment[i] = j; }); return assignment;
  };
  let lo = 0, hi = values.length - 1, best: number[] | null = null;
  while (lo <= hi) { const mid = (lo + hi) >> 1; const r = matching(values[mid]); if (r) { best = r; hi = mid - 1; } else lo = mid + 1; }
  if (!best) return { assignment: new Array(n).fill(-1), makespan: Infinity };
  return { assignment: best, makespan: Math.max(...best.map((j, i) => cost[i][j])) };
}

// ---------------------------------------------------------------------------------------------
// Auctions
// ---------------------------------------------------------------------------------------------

export interface Point { x: number; y: number }
export interface AuctionTask { id: string; at: Point; duration?: number }
export interface AuctionRobot { id: string; at: Point; speed?: number; /** max tasks in the bundle */ capacity?: number }

/** Route cost for a robot serving tasks in order (travel time + durations). */
export function routeCost(robot: AuctionRobot, tasks: AuctionTask[]): number {
  let t = 0, cur = robot.at; const v = robot.speed ?? 1;
  for (const task of tasks) { t += Math.hypot(task.at.x - cur.x, task.at.y - cur.y) / v + (task.duration ?? 0); cur = task.at; }
  return t;
}

/** Sequential single-item auction: tasks offered one by one, each robot bids its marginal route cost (2-approximation for sum of costs with submodular costs). */
export function sequentialAuction(robots: AuctionRobot[], tasks: AuctionTask[]): { bundles: Record<string, string[]>; totalCost: number; makespan: number; rounds: Array<{ task: string; winner: string; bid: number }> } {
  const bundles: Record<string, AuctionTask[]> = Object.fromEntries(robots.map((r) => [r.id, []]));
  const rounds: Array<{ task: string; winner: string; bid: number }> = [];
  for (const task of tasks) {
    let best = Infinity, winner = robots[0];
    for (const r of robots) { if ((r.capacity ?? Infinity) <= bundles[r.id].length) continue; const bid = routeCost(r, [...bundles[r.id], task]) - routeCost(r, bundles[r.id]); if (bid < best) { best = bid; winner = r; } }
    bundles[winner.id].push(task); rounds.push({ task: task.id, winner: winner.id, bid: best });
  }
  const costs = robots.map((r) => routeCost(r, bundles[r.id]));
  return { bundles: Object.fromEntries(robots.map((r) => [r.id, bundles[r.id].map((t) => t.id)])), totalCost: costs.reduce((a, b) => a + b, 0), makespan: Math.max(...costs), rounds };
}

/**
 * CBBA (consensus-based bundle algorithm): every robot greedily builds a bundle by marginal score, then robots exchange
 * winner / bid vectors and resolve conflicts (higher bid wins); repeated until no changes. Communication graph optional.
 */
export function cbba(robots: AuctionRobot[], tasks: AuctionTask[], opts: { maxIterations?: number; neighbours?: (id: string) => string[] } = {}): { bundles: Record<string, string[]>; iterations: number; converged: boolean; totalCost: number; makespan: number } {
  const n = robots.length; const idx = new Map(robots.map((r, i) => [r.id, i]));
  const winner: number[][] = robots.map(() => tasks.map(() => -1)); // each robot's belief: winner of task j
  const winBid: number[][] = robots.map(() => tasks.map(() => -Infinity));
  const bundles: number[][] = robots.map(() => []);
  const scoreOf = (r: AuctionRobot, bundle: number[], j: number) => { const list = bundle.map((k) => tasks[k]); const base = routeCost(r, list); const c = routeCost(r, [...list, tasks[j]]) - base; return 100 / (1 + c); };
  let it = 0, converged = false;
  for (; it < (opts.maxIterations ?? 50); it++) {
    // phase 1: bundle construction
    for (let i = 0; i < n; i++) {
      const r = robots[i];
      for (;;) {
        if (bundles[i].length >= (r.capacity ?? tasks.length)) break;
        let bj = -1, bs = -Infinity;
        for (let j = 0; j < tasks.length; j++) { if (bundles[i].includes(j)) continue; const s = scoreOf(r, bundles[i], j); if (s > winBid[i][j] + 1e-9 && s > bs) { bs = s; bj = j; } }
        if (bj < 0) break;
        bundles[i].push(bj); winner[i][bj] = i; winBid[i][bj] = bs;
      }
    }
    // phase 2: consensus (synchronous exchange among neighbours)
    let changed = false;
    const snapW = winner.map((w) => [...w]), snapB = winBid.map((b) => [...b]);
    for (let i = 0; i < n; i++) {
      const nb = opts.neighbours ? opts.neighbours(robots[i].id).map((id) => idx.get(id)!).filter((k) => k !== undefined) : robots.map((_, k) => k).filter((k) => k !== i);
      for (const k of nb) for (let j = 0; j < tasks.length; j++) {
        if (snapB[k][j] > winBid[i][j] + 1e-9 || (Math.abs(snapB[k][j] - winBid[i][j]) <= 1e-9 && snapW[k][j] >= 0 && snapW[k][j] < winner[i][j])) {
          if (winner[i][j] !== snapW[k][j] || winBid[i][j] !== snapB[k][j]) { winner[i][j] = snapW[k][j]; winBid[i][j] = snapB[k][j]; changed = true; }
        }
      }
      // release bundle entries lost to others (and everything after them, as their scores depended on the bundle)
      const lost = bundles[i].findIndex((j) => winner[i][j] !== i);
      if (lost >= 0) { for (const j of bundles[i].slice(lost)) if (winner[i][j] === i) { winner[i][j] = -1; winBid[i][j] = -Infinity; } bundles[i] = bundles[i].slice(0, lost); changed = true; }
    }
    if (!changed) { converged = true; it++; break; }
  }
  const out: Record<string, string[]> = {};
  robots.forEach((r, i) => { out[r.id] = bundles[i].map((j) => tasks[j].id); });
  const costs = robots.map((r, i) => routeCost(r, bundles[i].map((j) => tasks[j])));
  return { bundles: out, iterations: it, converged, totalCost: costs.reduce((a, b) => a + b, 0), makespan: Math.max(...costs) };
}

// ---------------------------------------------------------------------------------------------
// MAPF
// ---------------------------------------------------------------------------------------------

export interface Graph { nodes: string[]; edges: Array<[string, string]>; /** undirected by default */ directed?: boolean }
export interface Agent { id: string; start: string; goal: string }
export interface Constraint { agent: string; node: string; t: number; /** edge constraint: moving from `from` to `node` at time t is forbidden */ from?: string }
export type Path = string[]; // path[t] = node at time t

function adjacency(g: Graph): Map<string, string[]> {
  const adj = new Map<string, string[]>(g.nodes.map((n) => [n, []]));
  for (const [a, b] of g.edges) { adj.get(a)!.push(b); if (!g.directed) adj.get(b)!.push(a); }
  return adj;
}
function bfsDist(adj: Map<string, string[]>, goal: string): Map<string, number> {
  // distances to goal on the reversed graph (undirected assumption for the heuristic)
  const d = new Map<string, number>([[goal, 0]]); const q = [goal];
  const rev = new Map<string, string[]>(); for (const [a, bs] of adj) for (const b of bs) { let l = rev.get(b); if (!l) { l = []; rev.set(b, l); } l.push(a); }
  while (q.length) { const x = q.shift()!; for (const y of rev.get(x) ?? []) if (!d.has(y)) { d.set(y, d.get(x)! + 1); q.push(y); } }
  return d;
}

/** Space-time A* for one agent under vertex / edge constraints and a reservation table (occupied[t] = set of nodes; edges as "a>b@t"). */
export function spaceTimeAStar(g: Graph, start: string, goal: string, constraints: Constraint[], reserved: { nodes: Map<number, Set<string>>; edges: Set<string>; horizon: number }, maxT = 200): Path | null {
  const adj = adjacency(g); const h = bfsDist(adj, goal);
  if (!h.has(start)) return null;
  const vc = new Set(constraints.filter((c) => !c.from).map((c) => `${c.node}@${c.t}`)), ec = new Set(constraints.filter((c) => c.from).map((c) => `${c.from}>${c.node}@${c.t}`));
  const lastConstraint = Math.max(0, ...constraints.map((c) => c.t));
  type N = { node: string; t: number; g: number; f: number; parent: N | null };
  const open: N[] = [{ node: start, t: 0, g: 0, f: h.get(start)!, parent: null }]; const closed = new Set<string>();
  const blockedNode = (n: string, t: number) => vc.has(`${n}@${t}`) || reserved.nodes.get(t)?.has(n) || (t >= reserved.horizon && [...(reserved.nodes.get(reserved.horizon) ?? [])].includes(n) && reserved.horizon > 0);
  while (open.length) {
    let bi = 0; for (let i = 1; i < open.length; i++) if (open[i].f < open[bi].f || (open[i].f === open[bi].f && open[i].g > open[bi].g)) bi = i;
    const cur = open.splice(bi, 1)[0];
    const key = `${cur.node}@${cur.t}`; if (closed.has(key)) continue; closed.add(key);
    if (cur.node === goal && cur.t >= lastConstraint && !futureBlocked(cur.node, cur.t)) { const path: string[] = []; let c: N | null = cur; while (c) { path.push(c.node); c = c.parent; } return path.reverse(); }
    if (cur.t >= maxT) continue;
    for (const nx of [cur.node, ...adj.get(cur.node)!]) {
      const t2 = cur.t + 1;
      if (blockedNode(nx, t2) || ec.has(`${cur.node}>${nx}@${t2}`) || reserved.edges.has(`${cur.node}>${nx}@${t2}`)) continue;
      open.push({ node: nx, t: t2, g: cur.g + 1, f: cur.g + 1 + (h.get(nx) ?? 1e6), parent: cur });
    }
  }
  return null;
  function futureBlocked(n: string, t: number): boolean { for (let k = t + 1; k <= reserved.horizon; k++) if (reserved.nodes.get(k)?.has(n)) return true; return false; }
}

function reserve(paths: Record<string, Path>): { nodes: Map<number, Set<string>>; edges: Set<string>; horizon: number } {
  const nodes = new Map<number, Set<string>>(); const edges = new Set<string>(); let horizon = 0;
  for (const p of Object.values(paths)) {
    horizon = Math.max(horizon, p.length - 1);
    p.forEach((n, t) => { let s = nodes.get(t); if (!s) { s = new Set(); nodes.set(t, s); } s.add(n); if (t > 0) edges.add(`${n}>${p[t - 1]}@${t}`); /* swap forbidden */ });
  }
  // agents stay at their goals afterwards
  for (const p of Object.values(paths)) for (let t = p.length; t <= horizon; t++) { let s = nodes.get(t); if (!s) { s = new Set(); nodes.set(t, s); } s.add(p[p.length - 1]); }
  return { nodes, edges, horizon };
}

/** Priority planning: agents planned in order, earlier paths become dynamic obstacles. Incomplete (§13.5.3). */
export function priorityPlanning(g: Graph, agents: Agent[], order?: string[]): { paths: Record<string, Path>; failed: string[]; sumOfCosts: number; makespan: number } {
  const paths: Record<string, Path> = {}; const failed: string[] = [];
  const seq = order ? order.map((id) => agents.find((a) => a.id === id)!) : agents;
  for (const a of seq) { const p = spaceTimeAStar(g, a.start, a.goal, [], reserve(paths)); if (p) paths[a.id] = p; else failed.push(a.id); }
  return { paths, failed, sumOfCosts: Object.values(paths).reduce((s, p) => s + p.length - 1, 0), makespan: Math.max(0, ...Object.values(paths).map((p) => p.length - 1)) };
}

export interface Conflict { a: string; b: string; node: string; t: number; kind: 'vertex' | 'edge'; from?: string; to?: string }
export function firstConflict(paths: Record<string, Path>): Conflict | null {
  const ids = Object.keys(paths); const T = Math.max(...ids.map((i) => paths[i].length));
  const at = (id: string, t: number) => paths[id][Math.min(t, paths[id].length - 1)];
  for (let t = 0; t < T; t++) for (let i = 0; i < ids.length; i++) for (let j = i + 1; j < ids.length; j++) {
    const a = ids[i], b = ids[j];
    if (at(a, t) === at(b, t)) return { a, b, node: at(a, t), t, kind: 'vertex' };
    if (t > 0 && at(a, t) === at(b, t - 1) && at(b, t) === at(a, t - 1)) return { a, b, node: at(a, t), t, kind: 'edge', from: at(a, t - 1), to: at(a, t) };
  }
  return null;
}

/** Conflict-based search (Sharon et al.): complete and optimal for sum of costs (§13.5.4). */
export function cbs(g: Graph, agents: Agent[], opts: { maxNodes?: number } = {}): { paths: Record<string, Path>; found: boolean; sumOfCosts: number; makespan: number; expanded: number } {
  type Node = { constraints: Constraint[]; paths: Record<string, Path>; cost: number };
  const empty = { nodes: new Map<number, Set<string>>(), edges: new Set<string>(), horizon: 0 };
  const plan = (a: Agent, cs: Constraint[]) => spaceTimeAStar(g, a.start, a.goal, cs.filter((c) => c.agent === a.id), empty);
  const root: Node = { constraints: [], paths: {}, cost: 0 };
  for (const a of agents) { const p = plan(a, []); if (!p) return { paths: {}, found: false, sumOfCosts: 0, makespan: 0, expanded: 0 }; root.paths[a.id] = p; root.cost += p.length - 1; }
  const open: Node[] = [root]; let expanded = 0;
  while (open.length) {
    if (expanded > (opts.maxNodes ?? 20000)) break;
    let bi = 0; for (let i = 1; i < open.length; i++) if (open[i].cost < open[bi].cost) bi = i;
    const n = open.splice(bi, 1)[0]; expanded++;
    const c = firstConflict(n.paths);
    if (!c) return { paths: n.paths, found: true, sumOfCosts: n.cost, makespan: Math.max(...Object.values(n.paths).map((p) => p.length - 1)), expanded };
    for (const who of [c.a, c.b]) {
      const cons: Constraint = c.kind === 'vertex' ? { agent: who, node: c.node, t: c.t } : who === c.a ? { agent: who, node: c.to!, from: c.from, t: c.t } : { agent: who, node: c.from!, from: c.to, t: c.t };
      const constraints = [...n.constraints, cons];
      const ag = agents.find((x) => x.id === who)!; const p = plan(ag, constraints); if (!p) continue;
      const paths = { ...n.paths, [who]: p };
      open.push({ constraints, paths, cost: Object.values(paths).reduce((s, x) => s + x.length - 1, 0) });
    }
  }
  return { paths: {}, found: false, sumOfCosts: 0, makespan: 0, expanded };
}

// ---------------------------------------------------------------------------------------------
// Temporal plan graph
// ---------------------------------------------------------------------------------------------

export interface TPG { events: Array<{ id: string; agent: string; node: string; step: number }>; arcs: Array<{ from: string; to: string; kind: 'route' | 'priority' }>; acyclic: boolean; cycle: string[] | null }

/** Build the TPG (Definition 13.1): route arcs along each path, priority arcs "i leaves v → j enters v" following the plan order. Cyclic ⇔ deadlock. */
export function buildTPG(paths: Record<string, Path>): TPG {
  const events: TPG['events'] = []; const arcs: TPG['arcs'] = [];
  const ev = (agent: string, k: number) => `${agent}#${k}`;
  for (const [agent, p] of Object.entries(paths)) {
    // compress consecutive waits
    const nodes: string[] = []; for (const n of p) if (nodes[nodes.length - 1] !== n) nodes.push(n);
    nodes.forEach((n, k) => { events.push({ id: ev(agent, k), agent, node: n, step: k }); if (k > 0) arcs.push({ from: ev(agent, k - 1), to: ev(agent, k), kind: 'route' }); });
  }
  // priority arcs: for each node, order visits by planned time
  const visits = new Map<string, Array<{ agent: string; k: number; t: number }>>();
  for (const [agent, p] of Object.entries(paths)) { let k = -1; let last = ''; p.forEach((n, t) => { if (n !== last) { k++; last = n; let v = visits.get(n); if (!v) { v = []; visits.set(n, v); } v.push({ agent, k, t }); } }); }
  for (const [, v] of visits) { v.sort((a, b) => a.t - b.t); for (let i = 1; i < v.length; i++) { const prev = v[i - 1], cur = v[i]; if (prev.agent === cur.agent) continue; arcs.push({ from: ev(prev.agent, prev.k + 1), to: ev(cur.agent, cur.k), kind: 'priority' }); } }
  const ids = new Set(events.map((e) => e.id)); const arcsOk = arcs.filter((a) => ids.has(a.from) && ids.has(a.to));
  const cycle = findCycle(events.map((e) => e.id), arcsOk);
  return { events, arcs: arcsOk, acyclic: !cycle, cycle };
}
function findCycle(nodes: string[], arcs: Array<{ from: string; to: string }>): string[] | null {
  const adj = new Map<string, string[]>(nodes.map((n) => [n, []])); for (const a of arcs) adj.get(a.from)!.push(a.to);
  const color = new Map<string, number>(); const stack: string[] = [];
  const dfs = (v: string): string[] | null => { color.set(v, 1); stack.push(v); for (const w of adj.get(v) ?? []) { if (color.get(w) === 1) return [...stack.slice(stack.indexOf(w)), w]; if (!color.has(w)) { const r = dfs(w); if (r) return r; } } stack.pop(); color.set(v, 2); return null; };
  for (const n of nodes) if (!color.has(n)) { const r = dfs(n); if (r) return r; }
  return null;
}

/** Execute a TPG with per-agent random delays: every agent advances only when its incoming arcs are satisfied; reports conflicts (none expected) and the makespan. */
export function simulateTPG(tpg: TPG, delays: Record<string, number>, steps = 1000): { finished: boolean; makespan: number; collisions: number } {
  const done = new Set<string>(); const pos = new Map<string, number>(); const agents = [...new Set(tpg.events.map((e) => e.agent))];
  for (const a of agents) { pos.set(a, 0); done.add(`${a}#0`); }
  const pending = new Map(agents.map((a) => [a, delays[a] ?? 0]));
  const incoming = new Map<string, string[]>(); for (const arc of tpg.arcs) { let l = incoming.get(arc.to); if (!l) { l = []; incoming.set(arc.to, l); } l.push(arc.from); }
  const lastStep = new Map(agents.map((a) => [a, Math.max(...tpg.events.filter((e) => e.agent === a).map((e) => e.step))]));
  let t = 0, collisions = 0;
  for (; t < steps; t++) {
    let all = true;
    for (const a of agents) {
      const k = pos.get(a)!; if (k >= lastStep.get(a)!) continue; all = false;
      if (pending.get(a)! > 0) { pending.set(a, pending.get(a)! - 1); continue; }
      const next = `${a}#${k + 1}`;
      if ((incoming.get(next) ?? []).every((f) => done.has(f))) { pos.set(a, k + 1); done.add(next); }
    }
    const occ = new Map<string, string>();
    for (const a of agents) { const node = tpg.events.find((e) => e.id === `${a}#${pos.get(a)}`)!.node; if (occ.has(node)) collisions++; occ.set(node, a); }
    if (all) break;
  }
  return { finished: agents.every((a) => pos.get(a)! >= lastStep.get(a)!), makespan: t, collisions };
}

// ---------------------------------------------------------------------------------------------
// Section reservation
// ---------------------------------------------------------------------------------------------

/** Routes as ordered lists of sections; a common acquisition order over all routes is a structural deadlock-freedom proof (Proposition 4.2). */
export function reservationDeadlockCheck(routes: Record<string, string[]>): { deadlockFree: boolean; order: string[]; cycle: string[] | null; advice: string } {
  const r = resourceOrder(Object.values(routes));
  return { deadlockFree: r.ok, order: r.order, cycle: r.cycle, advice: r.ok ? `robots acquire sections in the order ${r.order.join(' < ')}` : `opposite acquisition order on ${r.cycle?.join(' → ')}: make the corridor one resource, add a direction monitor with alternation, or apply the banker's algorithm` };
}

/** Critical sections: sections whose blocking disconnects some start from its goal (robot failure analysis, §13.6.4). */
export function criticalSections(g: Graph, agents: Agent[]): string[] {
  const adj = adjacency(g); const out: string[] = [];
  const connected = (blocked: string, s: string, t: string) => { if (s === blocked || t === blocked) return false; const seen = new Set([s]); const q = [s]; while (q.length) { const x = q.pop()!; if (x === t) return true; for (const y of adj.get(x) ?? []) if (y !== blocked && !seen.has(y)) { seen.add(y); q.push(y); } } return false; };
  for (const n of g.nodes) if (agents.some((a) => a.start !== n && a.goal !== n && !connected(n, a.start, a.goal))) out.push(n);
  return out;
}
