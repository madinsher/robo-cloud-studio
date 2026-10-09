/**
 * Multi-robot warehouse (course homework): the warehouse model (grid map, stations, shortest paths), CBBA on a
 * stream of orders with commitment and lost-winner detection, the order executor state machine, distributed cell
 * reservation (traffic) with settle / acknowledgement / priority rules and asymmetric detours, and a fleet simulator
 * that runs all of it without ROS: order arrivals, radio range / loss / latency, faults, and the metrics of the
 * homework (throughput, order latency, double commits, path conflicts).
 */
import { Rng } from './rng';

export type Cell = [number, number];
const ck = (c: Cell) => `${c[0]},${c[1]}`;
const sameCell = (a: Cell | null | undefined, b: Cell | null | undefined) => !!a && !!b && a[0] === b[0] && a[1] === b[1];

export interface WarehouseConfig { cell: number; map: string; stations: Record<string, Cell> }
export class Warehouse {
  cell: number; grid: boolean[][]; rows: number; cols: number; stations: Record<string, Cell>; private distCache = new Map<string, number>();
  constructor(cfg: WarehouseConfig) {
    this.cell = cfg.cell; this.grid = cfg.map.replace(/^\n+|\n+$/g, '').split('\n').map((r) => [...r.trim()].map((ch) => ch === '#')); this.rows = this.grid.length; this.cols = this.grid[0].length; this.stations = cfg.stations;
    for (const [name, [r, c]] of Object.entries(this.stations)) if (!this.free([r, c])) throw new Error(`station ${name} is on a shelf or outside the map`);
  }
  cellXY(cell: Cell): [number, number] { return [cell[1] * this.cell, -cell[0] * this.cell]; }
  xyCell(x: number, y: number): Cell { return [Math.min(Math.max(Math.round(-y / this.cell), 0), this.rows - 1), Math.min(Math.max(Math.round(x / this.cell), 0), this.cols - 1)]; }
  stationXY(name: string): [number, number] { return this.cellXY(this.stations[name]); }
  free(cell: Cell): boolean { const [r, c] = cell; return r >= 0 && r < this.rows && c >= 0 && c < this.cols && !this.grid[r][c]; }
  /** Shortest 4-connected path (A*), optionally treating extra cells as blocked. */
  path(start: Cell, goal: Cell, blocked?: Set<string>): Cell[] | null {
    const h = (a: Cell) => Math.abs(a[0] - goal[0]) + Math.abs(a[1] - goal[1]);
    const open: Array<{ f: number; g: number; c: Cell }> = [{ f: h(start), g: 0, c: start }]; const parent = new Map<string, Cell | null>([[ck(start), null]]); const g = new Map<string, number>([[ck(start), 0]]);
    while (open.length) {
      let bi = 0; for (let i = 1; i < open.length; i++) if (open[i].f < open[bi].f || (open[i].f === open[bi].f && open[i].g > open[bi].g)) bi = i; const cur = open.splice(bi, 1)[0];
      if (sameCell(cur.c, goal)) { const out: Cell[] = []; let k: Cell | null = cur.c; while (k) { out.push(k); k = parent.get(ck(k)) ?? null; } return out.reverse(); }
      if (cur.g > (g.get(ck(cur.c)) ?? Infinity)) continue;
      for (const [dr, dc] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) { const nxt: Cell = [cur.c[0] + dr, cur.c[1] + dc]; if (!this.free(nxt) || (blocked?.has(ck(nxt)) && !sameCell(nxt, goal))) continue; const ng = cur.g + 1; if (ng < (g.get(ck(nxt)) ?? Infinity)) { g.set(ck(nxt), ng); parent.set(ck(nxt), cur.c); open.push({ f: ng + h(nxt), g: ng, c: nxt }); } }
    }
    return null;
  }
  distance(a: Cell, b: Cell): number { const k = `${ck(a)}|${ck(b)}`; let d = this.distCache.get(k); if (d === undefined) { const p = this.path(a, b); d = p ? (p.length - 1) * this.cell : Infinity; this.distCache.set(k, d); } return d; }
  stationDistance(a: string, b: string): number { return this.distance(this.stations[a], this.stations[b]); }
}
/** The warehouse of the homework (config/warehouse.yaml). */
export const HOMEWORK_WAREHOUSE: WarehouseConfig = {
  cell: 0.5,
  map: `................
.##.###..###.##.
.##.###..###.##.
................
................
.##.###..###.##.
.##.###..###.##.
................
................
................`,
  stations: { P1: [3, 0], P2: [4, 0], P3: [7, 0], D1: [3, 15], D2: [4, 15], D3: [7, 15], H1: [9, 2], H2: [9, 5], H3: [9, 10], H4: [9, 13] },
};

// --- CBBA on an order stream --------------------------------------------------------------------------------------
export interface Order { id: string; pickup: string; drop: string; reward: number; createdAt?: number }
export interface CbbaState { stamp: number; y: Record<string, number>; z: Record<string, string>; committed: string[] }
const EPS = 1e-9;
export class CbbaAgent {
  orders = new Map<string, Order>(); bundle: string[] = []; path: string[] = []; y: Record<string, number> = {}; z: Record<string, string> = {}; committed = new Set<string>(); done = new Set<string>();
  startStation: string | null = null; startDelay = 0; lastHeard: Record<string, number> = {}; private firstForget: number | null = null;
  constructor(public name: string, public wh: Warehouse, public capacity = 3, public discount = 0.98, public speed = 0.3, public handleTime = 3) {}
  addOrder(o: Order): void { if (this.done.has(o.id) || this.committed.has(o.id)) return; if (!this.orders.has(o.id)) { this.orders.set(o.id, o); this.y[o.id] ??= 0; this.z[o.id] ??= ''; } }
  /** An order was committed (by anyone) or delivered: leave the auction, the bundle and the path. */
  removeOrder(oid: string, done = false): void { if (done) this.done.add(oid); this.committed.add(oid); this.orders.delete(oid); const k = this.bundle.indexOf(oid); if (k >= 0) this.releaseFrom(k); this.path = this.path.filter((o) => this.bundle.includes(o)); }
  /** Re-open an order that a robot gave back. */
  reopenOrder(o: Order): void { this.committed.delete(o.id); this.done.delete(o.id); this.y[o.id] = 0; this.z[o.id] = ''; this.addOrder(o); }
  orderTime(at: string, o: Order): number { return this.wh.stationDistance(at, o.pickup) / this.speed + this.handleTime + this.wh.stationDistance(o.pickup, o.drop) / this.speed + this.handleTime; }
  pathScore(path: string[]): number { let t = this.startDelay, at = this.startStation!, s = 0; for (const oid of path) { const o = this.orders.get(oid)!; t += this.orderTime(at, o); if (!Number.isFinite(t)) return -Infinity; s += o.reward * this.discount ** t; at = o.drop; } return s; }
  buildBundle(): string[] {
    const added: string[] = [];
    while (this.bundle.length < this.capacity) {
      const base = this.pathScore(this.path); let best: { id: string; gain: number; pos: number } | null = null;
      for (const oid of [...this.orders.keys()].sort()) { if (this.path.includes(oid) || this.committed.has(oid) || this.done.has(oid)) continue; let gain = -Infinity, pos = 0; for (let k = 0; k <= this.path.length; k++) { const s = this.pathScore([...this.path.slice(0, k), oid, ...this.path.slice(k)]) - base; if (s > gain + EPS) { gain = s; pos = k; } } if (gain > (this.y[oid] ?? 0) + EPS && (!best || gain > best.gain + EPS)) best = { id: oid, gain, pos }; }
      if (!best) break; this.path.splice(best.pos, 0, best.id); this.bundle.push(best.id); this.y[best.id] = best.gain; this.z[best.id] = this.name; added.push(best.id);
    }
    return added;
  }
  private releaseFrom(k: number, keepFirst = false): void { const start = keepFirst ? k + 1 : k; for (const oid of this.bundle.slice(start)) if (this.z[oid] === this.name) { this.y[oid] = 0; this.z[oid] = ''; } const removed = new Set(this.bundle.slice(k)); this.bundle = this.bundle.slice(0, k); this.path = this.path.filter((o) => !removed.has(o)); }
  /** Merge fresh neighbour states: committed orders leave, max bid wins (ties → lexicographically smaller name), release from the first lost order. */
  consensus(neighbours: Record<string, CbbaState>, now: number, staleAfter = 3): boolean {
    let changed = false; const fresh = Object.entries(neighbours).filter(([n, st]) => n !== this.name && now - st.stamp <= staleAfter);
    for (const [n, st] of fresh) { this.lastHeard[n] = Math.max(this.lastHeard[n] ?? -Infinity, st.stamp); for (const oid of st.committed) if (this.orders.has(oid) || this.bundle.includes(oid)) { this.removeOrder(oid); changed = true; } }
    for (const oid of this.orders.keys()) {
      let by = this.y[oid] ?? 0, bz = this.z[oid] ?? '';
      for (const [n, st] of fresh) if (bz === n && st.z[oid] !== undefined && st.z[oid] !== n) { by = st.y[oid] ?? 0; bz = st.z[oid] ?? ''; }
      for (const [n, st] of fresh) { const y = st.y[oid], z = st.z[oid]; if (y === undefined || !z || z === this.name) continue; void n; if (y > by + EPS || (Math.abs(y - by) <= EPS && (!bz || z < bz)) || !bz) { by = y; bz = z; } }
      if (by !== this.y[oid] || bz !== this.z[oid]) { this.y[oid] = by; this.z[oid] = bz; changed = true; }
    }
    const k0 = this.bundle.findIndex((oid) => this.z[oid] !== this.name); if (k0 >= 0) { this.releaseFrom(k0); changed = true; }
    return changed;
  }
  /** A winner silent for longer than lostAfter is considered failed: its bids are reset and its orders return to the auction. */
  forgetSilentWinners(now: number, lostAfter: number): string[] {
    if (this.firstForget === null) this.firstForget = now; const freed: string[] = [];
    for (const oid of this.orders.keys()) { const w = this.z[oid]; if (!w || w === this.name) continue; const last = this.lastHeard[w] ?? this.firstForget; if (now - last > lostAfter) { this.y[oid] = 0; this.z[oid] = ''; freed.push(oid); } }
    return freed;
  }
  exportState(now: number): CbbaState { const y: Record<string, number> = {}, z: Record<string, string> = {}; for (const oid of this.orders.keys()) { y[oid] = this.y[oid] ?? 0; z[oid] = this.z[oid] ?? ''; } return { stamp: now, y, z, committed: [...this.committed].sort() }; }
  head(): string | null { return this.path[0] ?? null; }
}

// --- executor state machine ---------------------------------------------------------------------------------------
export type ExecState = 'IDLE' | 'TO_PICKUP' | 'LOADING' | 'TO_DROP' | 'UNLOADING' | 'TO_HOME' | 'FAILED';
export type ExecEvent = ['order', string] | ['arrived', null] | ['nav_failed', null] | ['handled', null] | ['handle_failed', null] | ['no_orders', null] | ['stop', null] | ['reset', null];
export type ExecCommand = ['goto', string] | ['handle', 'pick' | 'drop'] | ['release', string] | ['delivered', string] | ['cancel', null];
export class Executor {
  state: ExecState = 'IDLE'; order: string | null = null; pickup: string | null = null; drop: string | null = null; retries = 0;
  constructor(public home: string, public maxRetries = 1) {}
  assign(oid: string, pickup: string, drop: string): ExecCommand[] { if (this.state !== 'IDLE' && this.state !== 'TO_HOME') return []; this.pickup = pickup; this.drop = drop; return this.step(['order', oid]); }
  step(ev: ExecEvent): ExecCommand[] {
    const [name, data] = ev; const s = this.state;
    if (name === 'stop') { if (s === 'FAILED') return []; const out: ExecCommand[] = [['cancel', null]]; if (this.order && (s === 'TO_PICKUP' || s === 'LOADING')) { out.push(['release', this.order]); this.order = null; } this.state = 'FAILED'; return out; }
    if (s === 'FAILED') { if (name === 'reset') { if (this.order) { this.state = 'TO_DROP'; return [['goto', this.drop!]]; } this.state = 'IDLE'; return []; } return []; }
    switch (s) {
      case 'IDLE': if (name === 'order') { this.order = data as string; this.retries = 0; this.state = 'TO_PICKUP'; return [['goto', this.pickup!]]; } if (name === 'no_orders') { return []; } return [];
      case 'TO_HOME': if (name === 'order') { this.order = data as string; this.retries = 0; this.state = 'TO_PICKUP'; return [['cancel', null], ['goto', this.pickup!]]; } if (name === 'arrived' || name === 'nav_failed') { this.state = 'IDLE'; return []; } return [];
      case 'TO_PICKUP': if (name === 'arrived') { this.state = 'LOADING'; return [['handle', 'pick']]; } if (name === 'nav_failed') { if (this.retries < this.maxRetries) { this.retries++; return [['goto', this.pickup!]]; } const o = this.order!; this.order = null; this.state = 'IDLE'; return [['release', o]]; } return [];
      case 'LOADING': if (name === 'handled') { this.state = 'TO_DROP'; this.retries = 0; return [['goto', this.drop!]]; } if (name === 'handle_failed') { const o = this.order!; this.order = null; this.state = 'IDLE'; return [['release', o]]; } return [];
      case 'TO_DROP': if (name === 'arrived') { this.state = 'UNLOADING'; return [['handle', 'drop']]; } if (name === 'nav_failed') return [['goto', this.drop!]]; return [];
      case 'UNLOADING': if (name === 'handled' || name === 'handle_failed') { const o = this.order!; this.order = null; this.state = 'IDLE'; return [['delivered', o]]; } return [];
    }
    return [];
  }
  goHome(): ExecCommand[] { if (this.state !== 'IDLE') return []; this.state = 'TO_HOME'; return [['goto', this.home]]; }
}

// --- distributed cell reservation ---------------------------------------------------------------------------------
export interface TrafficState { stamp: number; holds: Cell[]; want: Cell | null; wantSince: number | null }
export class Traffic {
  cell: Cell | null = null; route: Cell[] = []; reserved: Cell[] = []; goal: Cell | null = null; want: Cell | null = null; wantSince: number | null = null; replans = 0; waits = 0;
  constructor(public name: string, public wh: Warehouse, public settle = 0.3, public staleAfter = 2, public waitLimit = 6, public lookahead = 2) {}
  setGoal(cell: Cell, goal: Cell): boolean { this.cell = cell; this.goal = goal; const start = this.reserved.length ? this.reserved[this.reserved.length - 1] : cell; const p = this.wh.path(start, goal); this.route = p ? p.slice(1) : []; this.want = this.wantSince = null; return p !== null; }
  clear(): void { this.route = []; this.want = this.wantSince = null; }
  holds(): Cell[] { return [...(this.cell ? [this.cell] : []), ...this.reserved]; }
  done(): boolean { return !this.route.length && !this.reserved.length && sameCell(this.cell, this.goal); }
  exportState(now: number): TrafficState { return { stamp: now, holds: this.holds(), want: this.want, wantSince: this.wantSince }; }
  fresh(neighbours: Record<string, TrafficState>, now: number): Record<string, TrafficState> { const out: Record<string, TrafficState> = {}; for (const [k, v] of Object.entries(neighbours)) if (k !== this.name && now - v.stamp <= this.staleAfter) out[k] = v; return out; }
  /** Rules 1–4: nobody holds the cell, highest priority (earlier want, then smaller name), settle elapsed, every fresh neighbour acknowledged (its stamp > wantSince + settle). */
  mayEnter(cell: Cell, neighbours: Record<string, TrafficState>, now: number): boolean {
    if (!sameCell(this.want, cell) || this.wantSince === null) return false; if (now - this.wantSince < this.settle) return false;
    for (const [n, st] of Object.entries(neighbours)) { if (st.holds.some((h) => sameCell(h, cell))) return false; if (st.want && sameCell(st.want, cell) && st.wantSince !== null) { if (st.wantSince < this.wantSince || (st.wantSince === this.wantSince && n < this.name)) return false; } if (st.stamp <= this.wantSince + this.settle) return false; }
    return true;
  }
  step(neighbours: Record<string, TrafficState>, now: number): boolean {
    if (!this.route.length || this.reserved.length >= this.lookahead) return false;
    const fresh = this.fresh(neighbours, now); const nxt = this.route[0];
    if (!sameCell(this.want, nxt)) { this.want = nxt; this.wantSince = now; return false; }
    if (this.mayEnter(nxt, fresh, now)) { this.reserved.push(this.route.shift()!); this.want = this.wantSince = null; return true; }
    this.waits++;
    const blockers = Object.entries(fresh).filter(([, v]) => v.holds.some((h) => sameCell(h, nxt))).map(([r]) => r); const limit = blockers.some((r) => r < this.name) ? this.waitLimit : 2 * this.waitLimit;
    if (now - (this.wantSince ?? now) > limit) this.replanAround(fresh, now);
    return false;
  }
  nextTarget(): Cell | null { return this.reserved[0] ?? null; }
  arrived(): void { if (this.reserved.length) this.cell = this.reserved.shift()!; }
  /** Detour from the last reserved cell treating cells held by fresh neighbours as obstacles; keeps waiting (resetting the claim time) when none exists. */
  replanAround(fresh: Record<string, TrafficState>, now: number): boolean {
    const blocked = new Set<string>(); for (const st of Object.values(fresh)) for (const h of st.holds) blocked.add(ck(h));
    const start = this.reserved.length ? this.reserved[this.reserved.length - 1] : this.cell!; const p = this.wh.path(start, this.goal!, blocked);
    if (!p || p.slice(1).every((c, i) => sameCell(c, this.route[i]))) { this.wantSince = now; return false; }
    this.route = p.slice(1); this.want = this.wantSince = null; this.replans++; return true;
  }
}

// --- fleet simulator ----------------------------------------------------------------------------------------------
export interface FleetConfig {
  warehouse: WarehouseConfig; robots: { names: string[]; homes: string[]; speed: number; handleTime: number; radioRange: number; radioDrop: number; radioLatency: number };
  orders: { rate: number; reward: number; firstBatch: number; pickups: string[]; drops: string[]; /** explicit orders (id, pickup, drop, reward, time) */ fixed?: Array<Order & { at: number }> };
  traffic: { settle: number; staleAfter: number; waitLimit: number; lookahead: number; /** protective layer (stage 4): keep the last known cells of a silent robot as obstacles and stop before a robot body detected by the proximity sensor */ protective?: boolean };
  cbba: { capacity: number; discount: number; period: number; staleAfter: number; commitAfter: number; lostAfter: number };
  faults: Array<{ robot: string; at: number; kind: 'stop' | 'mute' | 'recover' }>;
  duration: number; dt: number; seed: number; idleToHome: number;
}
export const HOMEWORK_FLEET: FleetConfig = {
  warehouse: HOMEWORK_WAREHOUSE,
  robots: { names: ['r1', 'r2', 'r3', 'r4'], homes: ['H1', 'H2', 'H3', 'H4'], speed: 0.3, handleTime: 3, radioRange: 4, radioDrop: 0, radioLatency: 0.05 },
  orders: { rate: 0.05, reward: 10, firstBatch: 6, pickups: ['P1', 'P2', 'P3'], drops: ['D1', 'D2', 'D3'] },
  traffic: { settle: 0.3, staleAfter: 2, waitLimit: 6, lookahead: 2 },
  cbba: { capacity: 3, discount: 0.98, period: 0.5, staleAfter: 3, commitAfter: 2, lostAfter: 6 },
  faults: [], duration: 600, dt: 0.1, seed: 1, idleToHome: 10,
};
export interface FleetRobot { name: string; home: string; x: number; y: number; cell: Cell; agent: CbbaAgent; exec: Executor; traffic: Traffic; handleUntil: number | null; alive: boolean; muted: boolean; idleSince: number; headSince: Record<string, number>; delivered: number; distance: number; lastCbba: number; busyTime: number; waitingCell: boolean; proximityWait?: number; lastFreed?: string }
export interface FleetMetrics { time: number; created: number; delivered: number; open: number; throughputPerHour: number; meanLatency: number; maxLatency: number; doubleCommits: number; pathConflicts: number; replans: number; messages: number; utilisation: number; perRobot: Record<string, number>; nearMisses: number; proximityStops: number }

export class FleetSim {
  wh: Warehouse; robots: FleetRobot[]; rng: Rng; time = 0; orders = new Map<string, Order>(); openOrders = new Map<string, Order>(); deliveredOrders: Array<{ id: string; createdAt: number; deliveredAt: number; robot: string }> = []; nextOrderAt: number; orderSeq = 0;
  private cbbaStates: Record<string, CbbaState> = {}; private trafficStates: Record<string, TrafficState> = {}; private conflictPairs = new Set<string>(); private faultsDone = new Set<number>();
  doubleCommits = 0; pathConflicts = 0; messages = 0; nearMisses = 0; proximityStops = 0; log: string[] = []; private lastKnown: Record<string, Record<string, TrafficState>> = {}; snapshots: Array<{ t: number; pos: Array<[number, number]> }> = [];
  constructor(public cfg: FleetConfig) {
    this.wh = new Warehouse(cfg.warehouse); this.rng = new Rng(cfg.seed);
    this.robots = cfg.robots.names.map((name, i) => { const home = cfg.robots.homes[i % cfg.robots.homes.length]; const cell = this.wh.stations[home]; const [x, y] = this.wh.cellXY(cell); const traffic = new Traffic(name, this.wh, cfg.traffic.settle, cfg.traffic.staleAfter, cfg.traffic.waitLimit, cfg.traffic.lookahead); traffic.cell = cell; traffic.goal = cell; const agent = new CbbaAgent(name, this.wh, cfg.cbba.capacity, cfg.cbba.discount, cfg.robots.speed, cfg.robots.handleTime); agent.startStation = home; return { name, home, x, y, cell, agent, exec: new Executor(home), traffic, handleUntil: null, alive: true, muted: false, idleSince: 0, headSince: {}, delivered: 0, distance: 0, lastCbba: -Infinity, busyTime: 0, waitingCell: false }; });
    for (let k = 0; k < cfg.orders.firstBatch; k++) this.createOrder(0);
    this.nextOrderAt = cfg.orders.rate > 0 ? this.rng.exponential(cfg.orders.rate) : Infinity;
    for (const f of cfg.orders.fixed ?? []) if (f.at <= 0) this.addOrder({ id: f.id, pickup: f.pickup, drop: f.drop, reward: f.reward, createdAt: 0 });
  }
  private createOrder(t: number): Order { const o: Order = { id: `o${++this.orderSeq}`, pickup: this.rng.choice(this.cfg.orders.pickups), drop: this.rng.choice(this.cfg.orders.drops), reward: this.cfg.orders.reward, createdAt: t }; this.addOrder(o); return o; }
  private addOrder(o: Order): void { this.orders.set(o.id, o); this.openOrders.set(o.id, o); for (const r of this.robots) if (r.alive) r.agent.addOrder(o); }
  private say(m: string): void { this.log.push(`${this.time.toFixed(1)} s: ${m}`); if (this.log.length > 2000) this.log.shift(); }
  private inRange(a: FleetRobot, b: FleetRobot): boolean { return Math.hypot(a.x - b.x, a.y - b.y) <= this.cfg.robots.radioRange; }
  private inbox: Record<string, Record<string, unknown>> = {};
  /** Radio: every robot in range delivers its latest state unless the packet is lost; the receiver keeps the LAST RECEIVED state of each neighbour (the staleness rules of CBBA / traffic decide whether it is still usable). */
  private neighbourStates<T>(r: FleetRobot, table: Record<string, T>, channel: string): Record<string, T> {
    const box = (this.inbox[`${channel}:${r.name}`] ??= {}) as Record<string, T>;
    for (const o of this.robots) { if (o === r || !o.alive || o.muted || !this.inRange(r, o) || !table[o.name]) continue; if (this.cfg.robots.radioDrop > 0 && this.rng.random() < this.cfg.robots.radioDrop) continue; box[o.name] = table[o.name]; this.messages++; }
    return { ...box };
  }
  private applyCommands(r: FleetRobot, cmds: ExecCommand[]): void {
    for (const [c, arg] of cmds) {
      if (c === 'goto') { const ok = r.traffic.setGoal(r.cell, this.wh.stations[arg as string]); if (!ok) this.applyCommands(r, r.exec.step(['nav_failed', null])); }
      else if (c === 'handle') { r.handleUntil = this.time + this.cfg.robots.handleTime; }
      else if (c === 'cancel') { r.traffic.clear(); r.handleUntil = null; }
      else if (c === 'release') { const o = this.orders.get(arg as string); if (o) { this.openOrders.set(o.id, o); for (const q of this.robots) q.agent.reopenOrder(o); this.say(`${r.name} released ${o.id}`); } }
      else if (c === 'delivered') { const o = this.orders.get(arg as string)!; this.deliveredOrders.push({ id: o.id, createdAt: o.createdAt ?? 0, deliveredAt: this.time, robot: r.name }); r.delivered++; for (const q of this.robots) q.agent.removeOrder(o.id, true); this.say(`${r.name} delivered ${o.id} (latency ${(this.time - (o.createdAt ?? 0)).toFixed(0)} s)`); }
    }
  }
  /** Advance the simulation by dt seconds. */
  step(dt = this.cfg.dt): void {
    const t = this.time; const cfg = this.cfg;
    // orders
    while (t >= this.nextOrderAt) { this.createOrder(this.nextOrderAt); this.nextOrderAt += this.rng.exponential(cfg.orders.rate); }
    for (const f of cfg.orders.fixed ?? []) if (f.at > 0 && f.at <= t && !this.orders.has(f.id)) this.addOrder({ id: f.id, pickup: f.pickup, drop: f.drop, reward: f.reward, createdAt: t });
    // faults
    cfg.faults.forEach((f, i) => { if (this.faultsDone.has(i) || f.at > t) return; this.faultsDone.add(i); const r = this.robots.find((x) => x.name === f.robot); if (!r) return; if (f.kind === 'stop') { r.alive = false; r.muted = true; this.applyCommands(r, r.exec.step(['stop', null])); this.say(`FAULT: ${r.name} stopped`); } else if (f.kind === 'mute') { r.muted = true; this.say(`FAULT: ${r.name} radio muted`); } else { r.alive = true; r.muted = false; this.applyCommands(r, r.exec.step(['reset', null])); this.say(`${r.name} recovered`); } });
    // CBBA period
    for (const r of this.robots) {
      if (!r.alive || t - r.lastCbba < cfg.cbba.period) continue; r.lastCbba = t;
      const ex = r.exec; r.agent.startStation = ex.order && ex.drop ? ex.drop : ex.pickup && ex.state === 'TO_PICKUP' ? ex.pickup : this.wh.xyCell(r.x, r.y) ? this.nearestStation(r) : r.home; r.agent.startDelay = ex.order ? this.remainingTime(r) : 0;
      r.agent.buildBundle(); const nb = this.neighbourStates(r, this.cbbaStates, 'cbba'); r.agent.consensus(nb, t, cfg.cbba.staleAfter); const freed = r.agent.forgetSilentWinners(t, cfg.cbba.lostAfter); const fk = freed.join(','); if (freed.length && fk !== r.lastFreed) this.say(`${r.name} reclaims ${freed.join(', ')} from a silent winner`); r.lastFreed = fk;
      // commitment: the head order must have been won continuously for commitAfter seconds
      const head = r.agent.head(); for (const k of Object.keys(r.headSince)) if (k !== head) delete r.headSince[k];
      if (head) {
        r.headSince[head] ??= t;
        if (t - r.headSince[head] >= cfg.cbba.commitAfter && (ex.state === 'IDLE' || ex.state === 'TO_HOME')) {
          const o = this.orders.get(head)!;
          if (!this.openOrders.has(o.id)) { this.doubleCommits++; r.agent.removeOrder(o.id); this.say(`DOUBLE COMMIT: ${r.name} tried to claim ${o.id}, already taken by ${this.robots.find((q) => q.exec.order === o.id)?.name ?? 'another robot'}`); }
          else { const cmds = ex.assign(o.id, o.pickup, o.drop); if (cmds.length) { r.agent.removeOrder(o.id); this.openOrders.delete(o.id); this.say(`${r.name} commits to ${o.id} (${o.pickup} → ${o.drop})`); this.applyCommands(r, cmds); } }
        }
      }
    }
    for (const r of this.robots) if (r.alive && !r.muted) this.cbbaStates[r.name] = r.agent.exportState(t);
    // motion, handling, traffic
    for (const r of this.robots) {
      if (!r.alive) continue; const ex = r.exec;
      if (ex.order || ex.state === 'TO_HOME') r.busyTime += dt;
      if (r.handleUntil !== null) { if (t >= r.handleUntil) { r.handleUntil = null; this.applyCommands(r, ex.step(['handled', null])); } continue; }
      if (ex.state === 'IDLE') { if (!sameCell(r.cell, this.wh.stations[r.home]) && t - r.idleSince > cfg.idleToHome && !r.agent.head()) this.applyCommands(r, ex.goHome()); } else r.idleSince = t;
      if (r.traffic.route.length || r.traffic.reserved.length) {
        const nb = this.neighbourStates(r, this.trafficStates, 'traffic');
        if (cfg.traffic.protective !== false) { // stage 4: a silent neighbour keeps its last known cells as obstacles (its stamp is refreshed so that it stays "fresh" for the reservation rules)
          const known = (this.lastKnown[r.name] ??= {}); for (const [k, v] of Object.entries(nb)) known[k] = v; for (const [k, v] of Object.entries(known)) if (!nb[k] && t - v.stamp > cfg.traffic.staleAfter) nb[k] = { stamp: t, holds: v.holds, want: null, wantSince: null };
        }
        r.traffic.step(nb, t);
        const target = r.traffic.nextTarget(); r.waitingCell = !target;
        if (target) {
          const [tx, ty] = this.wh.cellXY(target); const d = Math.hypot(tx - r.x, ty - r.y); const stepLen = cfg.robots.speed * dt;
          // proximity sensor (protective layer): a robot body on the next cell stops the motion even when the radio is silent; after the (asymmetric) wait limit the robot detours around the bodies it sees
          const bodies = cfg.traffic.protective !== false ? this.robots.filter((o) => o !== r && Math.hypot(o.x - tx, o.y - ty) < 0.6 * this.wh.cell && Math.hypot(o.x - r.x, o.y - r.y) < 1.5 * this.wh.cell) : [];
          if (bodies.length) { r.proximityWait = (r.proximityWait ?? 0) + dt; this.proximityStops++; const limit = bodies.some((o) => o.name < r.name) ? cfg.traffic.waitLimit : 2 * cfg.traffic.waitLimit; if (r.proximityWait > limit) { const seen = { ...nb }; for (const o of this.robots) if (o !== r && Math.hypot(o.x - r.x, o.y - r.y) < 3 * this.wh.cell) seen[`sensed:${o.name}`] = { stamp: t, holds: [this.wh.xyCell(o.x, o.y)], want: null, wantSince: null }; if (r.traffic.replanAround(seen, t)) this.say(`${r.name} detours around ${bodies.map((o) => o.name).join(', ')} (proximity sensor)`); r.proximityWait = 0; } }
          else { r.proximityWait = 0; if (d <= stepLen) { r.x = tx; r.y = ty; r.cell = target; r.traffic.arrived(); r.distance += d; } else { r.x += ((tx - r.x) / d) * stepLen; r.y += ((ty - r.y) / d) * stepLen; r.distance += stepLen; } }
        }
      }
      if (r.traffic.done() && (ex.state === 'TO_PICKUP' || ex.state === 'TO_DROP' || ex.state === 'TO_HOME') && r.handleUntil === null) { r.traffic.goal = null; this.applyCommands(r, ex.step(['arrived', null])); }
    }
    for (const r of this.robots) if (r.alive && !r.muted) this.trafficStates[r.name] = r.traffic.exportState(t);
    // path conflicts: two robots within half a cell (counted once per contact)
    for (let i = 0; i < this.robots.length; i++) for (let j = i + 1; j < this.robots.length; j++) { const a = this.robots[i], b = this.robots[j]; const d = Math.hypot(a.x - b.x, a.y - b.y); const k = `${a.name}|${b.name}`; if (d < this.wh.cell * 0.5) { if (!this.conflictPairs.has(k)) { this.conflictPairs.add(k); this.pathConflicts++; this.say(`PATH CONFLICT ${a.name} / ${b.name} at (${a.x.toFixed(1)}, ${a.y.toFixed(1)})`); } } else { if (d < this.wh.cell * 0.9 && !this.conflictPairs.has(k)) this.nearMisses++; this.conflictPairs.delete(k); } }
    this.time = t + dt;
    if (Math.round(this.time / dt) % Math.max(1, Math.round(5 / dt)) === 0) this.snapshots.push({ t: this.time, pos: this.robots.map((r) => [r.x, r.y]) });
  }
  private nearestStation(r: FleetRobot): string { let best = r.home, bd = Infinity; for (const [n, c] of Object.entries(this.wh.stations)) { const [x, y] = this.wh.cellXY(c); const d = Math.hypot(x - r.x, y - r.y); if (d < bd) { bd = d; best = n; } } return best; }
  private remainingTime(r: FleetRobot): number { const ex = r.exec; const c = this.wh.stations; const here = this.wh.xyCell(r.x, r.y); const v = this.cfg.robots.speed, h = this.cfg.robots.handleTime; if (ex.state === 'TO_PICKUP') return this.wh.distance(here, c[ex.pickup!]) / v + h + this.wh.distance(c[ex.pickup!], c[ex.drop!]) / v + h; if (ex.state === 'LOADING') return h + this.wh.distance(c[ex.pickup!], c[ex.drop!]) / v + h; if (ex.state === 'TO_DROP') return this.wh.distance(here, c[ex.drop!]) / v + h; if (ex.state === 'UNLOADING') return h; return 0; }
  run(duration = this.cfg.duration): FleetMetrics { while (this.time < duration - 1e-9) this.step(); return this.metrics(); }
  metrics(): FleetMetrics {
    const lat = this.deliveredOrders.map((d) => d.deliveredAt - d.createdAt); const alive = this.robots.filter((r) => r.alive).length || 1;
    return { time: this.time, created: this.orders.size, delivered: this.deliveredOrders.length, open: this.openOrders.size + this.robots.filter((r) => r.exec.order).length, throughputPerHour: this.time > 0 ? (3600 * this.deliveredOrders.length) / this.time : 0, meanLatency: lat.length ? lat.reduce((s, v) => s + v, 0) / lat.length : 0, maxLatency: lat.length ? Math.max(...lat) : 0, doubleCommits: this.doubleCommits, pathConflicts: this.pathConflicts, replans: this.robots.reduce((s, r) => s + r.traffic.replans, 0), messages: this.messages, utilisation: this.time > 0 ? this.robots.reduce((s, r) => s + r.busyTime, 0) / (this.time * alive) : 0, perRobot: Object.fromEntries(this.robots.map((r) => [r.name, r.delivered])), nearMisses: this.nearMisses, proximityStops: this.proximityStops };
  }
}
