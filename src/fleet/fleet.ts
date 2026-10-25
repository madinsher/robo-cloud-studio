/**
 * Fleet manager: task queue, pluggable allocation (nearest, sequential auction with queued work, Hungarian, CBBA,
 * contract net), traffic management that never lets footprints overlap (transit reservations of shared segments such
 * as orchard alleys, predictive stopping instead of creeping, a wait-for-graph deadlock resolver with yield
 * manoeuvres and replanning around reserved segments), charging policy, robot–obstacle / robot–robot contact detection
 * and KPIs. Task execution is phase based (travel → work → optional service trips → done) with process models plugged
 * in per task kind (the agri module registers harvest / spray / mow / prune / scout / thin / irrigate / transport).
 * Runs on the simulation clock and can drive real robots through the ROS 2 bridge (same task objects).
 */
import { Item, ItemType, SerializedItem, registerItemType, Station } from '../core/items/item';
import { MobileRobot, MapItem, ZoneItem } from '../mobile/items';
import { pathLength, planForRobot } from '../mobile/planner';
import { followPath, stepMobile, estimateTravelTime, integrate, isPosePath, toPosePath } from '../mobile/controller';
import { stepNavRuntime, getNavStack, NavRuntime } from '../mobile/navstack';
import { CollisionWorld, CircleObstacle, Rect2, rectsOverlap, rectIntersectsPolygon, footprintRect, coveringCircles } from '../mobile/footprint';
import { stationWorld } from '../mobile/world';
import { EventBus } from '../core/events';

function integrateStop(r: MobileRobot, dt: number) { integrate(r, { v: 0, omega: 0, vy: 0, vz: 0 }, dt); }

export type TaskType = 'transport' | 'harvest' | 'spray' | 'mow' | 'prune' | 'scout' | 'pollinate' | 'weed' | 'thin' | 'irrigate' | 'charge' | 'goto' | 'service' | 'custom';
export type TaskStatus = 'pending' | 'assigned' | 'travelling' | 'working' | 'done' | 'failed' | 'cancelled';

export interface FleetTask {
  id: string;
  type: TaskType;
  /** Where to go (mm, station frame) — for row tasks the row entry point. */
  location: [number, number];
  /** Heading to arrive with (deg); for work-path tasks the work-path direction. */
  goalHeading?: number;
  /** Optional work path to follow at the location (e.g. the crop row). */
  workPath?: number[][];
  /** Nominal work duration (s) if no work path, or work speed (mm/s) if a work path exists. */
  duration?: number;
  workSpeed?: number;
  priority: number;
  requiredCapabilities: string[];
  /** Deadline (sim seconds) — used by the allocator as a soft constraint. */
  deadline?: number;
  status: TaskStatus;
  robotId: string | null;
  createdAt: number;
  startedAt?: number;
  finishedAt?: number;
  /** Payload / crop mass (kg) */
  payloadKg?: number;
  /** Free data; `meta.process` selects the process model (default: the task type); `meta._rt` is the persisted runtime state. */
  meta: Record<string, any>;
  /** Segment ids this task occupies while working (traffic management). */
  segments?: string[];
}

export interface ContactEvent { t: number; robot: string; other: string; kind: string; depth: number }

export interface FleetKPIs {
  /** Agricultural yield collected by harvest tasks (fruit actually removed from the plants). */
  fruitPicked: number;
  yieldKg: number;
  areaWorkedM2: number;
  tasksDone: number;
  tasksFailed: number;
  tasksPending: number;
  throughputPerHour: number;
  utilization: number; // 0-1 averaged
  distanceTravelled: number; // mm
  energyUsedWh: number;
  meanWaitTime: number;
  /** Measured contacts: events (entering contact) and simulation steps in contact, robot–obstacle and robot–robot. */
  contacts: { robotObstacle: number; robotRobot: number; robotObstacleSteps: number; robotRobotSteps: number; maxDepth: number; log: ContactEvent[] };
  /** Traffic management statistics. */
  traffic: { waitSeconds: number; replans: number; yields: number; deadlocks: number };
  perRobot: Record<string, { busyTime: number; distance: number; tasks: number; battery: number; status: string; waitTime: number }>;
  /** Process results per kind (litres sprayed, coverage, observations, …). */
  process: Record<string, Record<string, number>>;
}

export interface FleetEvents extends Record<string, unknown> {
  taskUpdated: { task: FleetTask };
  robotUpdated: { robot: MobileRobot };
  log: { t: number; text: string; level: 'info' | 'warn' | 'error' };
  contact: ContactEvent;
}

export type AllocationMode = 'nearest' | 'auction' | 'round_robin' | 'hungarian' | 'cbba' | 'contract_net';

/** Fleet item: groups robots and holds the manager configuration. */
export class FleetItem extends Item {
  robotIds: string[] = [];
  mapId: string | null = null;
  allocation: AllocationMode = 'auction';
  chargingZoneIds: string[] = [];
  /** Max robots per shared segment (row). */
  segmentCapacity = 1;
  tasks: FleetTask[] = [];
  /** Max tasks queued per robot by queueing allocators (auction / CBBA / contract net). */
  queueLimit = 2;
  constructor(name = 'Fleet', id?: string) {
    super(ItemType.FLEET, name, id);
  }
  protected override serializeExtra() {
    return { robotIds: this.robotIds, mapId: this.mapId, allocation: this.allocation, chargingZoneIds: this.chargingZoneIds, segmentCapacity: this.segmentCapacity, tasks: this.tasks, queueLimit: this.queueLimit };
  }
  override deserializeExtra(d: SerializedItem) {
    this.robotIds = (d.robotIds as string[]) ?? []; this.mapId = (d.mapId as string | null) ?? null; this.allocation = (d.allocation as any) ?? 'auction';
    this.chargingZoneIds = (d.chargingZoneIds as string[]) ?? []; this.segmentCapacity = (d.segmentCapacity as number) ?? 1; this.tasks = (d.tasks as FleetTask[]) ?? [];
    this.queueLimit = (d.queueLimit as number) ?? 2;
  }
}
registerItemType(ItemType.FLEET, (n, id) => new FleetItem(n, id));

// ---------------------------------------------------------------------------------------------------------------
// Plug-in registries (process models, traffic segments, allocators, manager plugins)
// ---------------------------------------------------------------------------------------------------------------

/** What a process model asks the manager to do while working along the work path. */
export interface WorkDirective {
  /** Stand still for this many seconds (picking at a stop, pruning a plant). */
  hold?: number;
  /** Work speed override (mm/s). */
  speed?: number;
  /** Interrupt the work: drive to `location`, spend `duration` s (refill / unload), come back and resume. */
  service?: { kind: string; location: [number, number]; duration: number };
}

export interface TaskProcess {
  /** A robot was assigned (e.g. orient the work path so the arm faces the row). */
  onAssign?(fm: FleetManager, task: FleetTask, robot: MobileRobot): void;
  /** Work starts at the location (prepare stops etc.). */
  onWorkStart?(fm: FleetManager, task: FleetTask, robot: MobileRobot): void;
  /** Every step while on the work path; `s` = progress along the work path (mm); `holding` while standing still. */
  onWork?(fm: FleetManager, task: FleetTask, robot: MobileRobot, s: number, dt: number, holding: boolean): WorkDirective | void;
  /** Service trip finished (tank refilled, bin emptied). */
  onService?(fm: FleetManager, task: FleetTask, robot: MobileRobot, kind: string): void;
  /** Work path traversed (or timed work over). Returns yield to book (already-booked yield must not be returned). */
  onDone?(fm: FleetManager, task: FleetTask, robot: MobileRobot): { fruit?: number; kg?: number; areaM2?: number } | void;
}
const processes = new Map<string, TaskProcess>();
export function registerTaskProcess(kind: string, p: TaskProcess): void { processes.set(kind, p); }
export function taskProcess(kind: string): TaskProcess | undefined { return processes.get(kind); }

/** Shared traffic segment (alley, corridor, gate): at most `capacity` robots inside. */
export interface TrafficSegment { id: string; polygon: number[][]; capacity?: number }
const segmentProviders = new Map<string, (station: Station) => TrafficSegment[]>();
export function registerSegmentProvider(id: string, fn: (station: Station) => TrafficSegment[]): void { segmentProviders.set(id, fn); }

export interface AllocationContext {
  fm: FleetManager;
  robots: MobileRobot[];
  /** Pending tasks in priority order. */
  tasks: FleetTask[];
  /** Cost (s) for a robot to execute a task starting from `from` at time `t` (Infinity = infeasible). */
  cost(robot: MobileRobot, task: FleetTask, from?: { x: number; y: number; t: number }): number;
  /** Where/when a robot becomes free (end of its current and queued tasks). */
  queueEnd(robot: MobileRobot): { x: number; y: number; t: number; n: number };
  queueLimit: number;
}
export interface Assignment { task: FleetTask; robot: MobileRobot }
export type Allocator = (ctx: AllocationContext) => Assignment[];
const allocators = new Map<string, Allocator>();
export function registerAllocator(name: string, fn: Allocator): void { allocators.set(name, fn); }
export function allocatorNames(): string[] { return [...allocators.keys()]; }

/** Plugins run for every new FleetManager (the agri module installs its hooks here, so the app's manager has them). */
export const fleetPlugins: Array<(fm: FleetManager) => void> = [];

// ---------------------------------------------------------------------------------------------------------------

interface TaskRuntime {
  phase: 'travel' | 'work' | 'service' | 'return' | 'serviceWait';
  hold?: number;
  service?: { kind: string; location: [number, number]; duration: number; left?: number };
  resumeS?: number;
  workT?: number;
  noPath?: number;
  retryAt?: number;
}

interface RobotTraffic { waitSince: number | null; by: string | null; seg: string | null; replanAt: number; yield?: { waitFor: string; resume: { x: number; y: number; theta?: number } | null; arrived: boolean; t0: number } }

const wrap180 = (a: number) => ((((a + 180) % 360) + 360) % 360) - 180;

export class FleetManager {
  readonly events = new EventBus<FleetEvents>();
  time = 0;
  private busy = new Map<string, number>();
  private waitT = new Map<string, number>();
  private distance0 = new Map<string, number>();
  private energy0 = new Map<string, number>();
  private waits: number[] = [];
  /** Segment reservations: segment id -> robot ids (transit and work holds). */
  private reservations = new Map<string, Set<string>>();
  /** Work holds: robot id -> segments held for the whole work phase. */
  private taskHolds = new Map<string, Set<string>>();
  private log: Array<{ t: number; text: string; level: 'info' | 'warn' | 'error' }> = [];
  /** Yield counters (fruit actually removed, kg, area). */
  yield = { fruit: 0, kg: 0, areaM2: 0 };
  /** Process results per kind (litres, coverage, observations…). */
  processStats: Record<string, Record<string, number>> = {};
  /** Inter-robot safety: legacy braking distance (mm) — the predictive check below replaces it. */
  safetyDistance = 2500;
  /** Gap kept between robot footprints (mm). */
  safetyGap = 300;
  /** Clearance kept to obstacles when planning (mm). */
  planMargin = 120;
  /** Measure robot–obstacle and robot–robot contacts every step. */
  checkContacts = true;
  /** Hook to mark fruit as picked when a harvest task without a process model finishes. */
  onHarvestDone: ((task: FleetTask) => { fruit: number; kg: number }) | null = null;
  private pendingPathRequests = new Map<string, { task: FleetTask; phase: 'travel' | 'work' | 'charge' }>();
  private traffic = new Map<string, RobotTraffic>();
  private contactState = new Map<string, number>();
  private contactKpi = { robotObstacle: 0, robotRobot: 0, robotObstacleSteps: 0, robotRobotSteps: 0, maxDepth: 0, log: [] as ContactEvent[] };
  private trafficKpi = { waitSeconds: 0, replans: 0, yields: 0, deadlocks: 0 };
  private segCache: { key: string; segs: TrafficSegment[] } | null = null;
  private segCacheT = -1;
  private queueSeq = 0;

  constructor(readonly station: Station, readonly fleet: FleetItem) {
    for (const r of this.robots()) {
      this.distance0.set(r.id, r.state.odometer);
      this.energy0.set(r.id, r.battery.levelWh);
    }
    for (const p of fleetPlugins) p(this);
    // resume work holds of tasks that were working when the station was saved
    for (const t of this.fleet.tasks) if (t.status === 'working' && t.robotId && t.segments?.length) this.holdSegments(t.robotId, t.segments);
  }

  robots(): MobileRobot[] {
    return this.fleet.robotIds.map((id) => this.station.findById(id)).filter((r): r is MobileRobot => r instanceof MobileRobot);
  }

  addRobot(r: MobileRobot): void {
    if (!this.fleet.robotIds.includes(r.id)) this.fleet.robotIds.push(r.id);
    r.fleetId = this.fleet.id;
    this.distance0.set(r.id, r.state.odometer);
    this.energy0.set(r.id, r.battery.levelWh);
  }

  zones(): ZoneItem[] {
    return this.station.itemsOfType<ZoneItem>(ItemType.ZONE);
  }
  map(): MapItem | null {
    const m = this.fleet.mapId ? this.station.findById(this.fleet.mapId) : this.station.itemsOfType<MapItem>(ItemType.MAP)[0];
    return m instanceof MapItem ? m : null;
  }
  /** Collision world: navigation map + registered obstacles (trees) — cached for the duration of a step. */
  world(): CollisionWorld {
    if (this.stepCache.world && this.stepCache.t === this.time) return this.stepCache.world;
    const w = stationWorld(this.station, this.map());
    this.stepCache.world = w; this.stepCache.t = this.time;
    return w;
  }
  private stepCache: { t: number; world?: CollisionWorld } = { t: -1 };

  /** Traffic segments from the registered providers (cached while the world is unchanged). */
  segments(): TrafficSegment[] {
    if (this.segCache && this.segCacheT === this.time) return this.segCache.segs;
    this.segCacheT = this.time;
    const rows = this.station.itemsOfType<Item & { start: number[] }>(ItemType.CROP_ROW);
    const key = `${segmentProviders.size}:${rows.length}:${rows.reduce((a, r) => a + r.start[0] + 3 * r.start[1], 0).toFixed(0)}`;
    if (this.segCache && this.segCache.key === key) return this.segCache.segs;
    const segs: TrafficSegment[] = [];
    for (const fn of segmentProviders.values()) segs.push(...fn(this.station));
    for (const sg of segs) (sg as any)._bb = { x0: Math.min(...sg.polygon.map((p) => p[0])), y0: Math.min(...sg.polygon.map((p) => p[1])), x1: Math.max(...sg.polygon.map((p) => p[0])), y1: Math.max(...sg.polygon.map((p) => p[1])) };
    this.segCache = { key, segs };
    return segs;
  }

  addTask(partial: Partial<FleetTask> & { type: TaskType; location: [number, number] }): FleetTask {
    const task: FleetTask = {
      id: partial.id ?? this.nextTaskId(),
      priority: 1,
      requiredCapabilities: [],
      status: 'pending',
      robotId: null,
      createdAt: this.time,
      meta: {},
      ...partial,
    };
    this.fleet.tasks.push(task);
    this.logMsg(`Task ${task.id} (${task.type}) added`);
    this.events.emit('taskUpdated', { task });
    return task;
  }

  /** Next free task id: unique within the fleet (and the station's other fleets), also after a reload. */
  private nextTaskId(): string {
    let max = 0;
    for (const f of this.station.itemsOfType<FleetItem>(ItemType.FLEET)) for (const t of f.tasks) { const m = /^task_(\d+)$/.exec(t.id); if (m) max = Math.max(max, +m[1]); }
    return `task_${max + 1}`;
  }

  cancelTask(id: string): void {
    const t = this.fleet.tasks.find((x) => x.id === id);
    if (!t) return;
    if (t.robotId) {
      const r = this.station.findById(t.robotId) as MobileRobot | null;
      if (r && r.state.taskId === t.id) { r.state.path = null; r.state.status = 'idle'; r.state.taskId = null; }
      this.releaseAll(t.robotId);
    }
    t.status = 'cancelled';
    this.events.emit('taskUpdated', { task: t });
  }

  /**
   * Plan a footprint-aware, drive-aware pose path from the robot's pose to `goal` (straight line when the station has
   * neither a map nor obstacles). Returns null when no collision-free path exists.
   */
  planTo(robot: MobileRobot, goal: [number, number], opts: { goalHeading?: number; blocked?: number[][][]; extra?: CircleObstacle[]; from?: { x: number; y: number; theta: number } } = {}): number[][] | null {
    const world = this.world();
    const from = opts.from ?? { x: robot.state.x, y: robot.state.y, theta: robot.state.theta };
    if (!world.map && !world.circles.length && !opts.blocked?.length && !opts.extra?.length) return toPosePath(robot, [[from.x, from.y], goal], { goalHeading: opts.goalHeading, start: from });
    if (Math.hypot(goal[0] - from.x, goal[1] - from.y) < 30 && (opts.goalHeading === undefined || Math.abs(wrap180(opts.goalHeading - from.theta)) < 2)) return [[from.x, from.y, from.theta, 1], [goal[0], goal[1], from.theta, 1]];
    const res = planForRobot(robot, from, { x: goal[0], y: goal[1] }, { world, margin: this.planMargin, goalHeading: opts.goalHeading, blocked: opts.blocked, extra: opts.extra });
    if (!res.ok) this.logMsg(`Plan for ${robot.name} to (${goal[0].toFixed(0)}, ${goal[1].toFixed(0)}) failed: ${res.error}`, 'warn');
    return res.ok ? res.path : null;
  }

  /** Cost (s) for a robot to execute a task from a start point: travel + work, battery feasibility, capabilities. */
  taskCost(robot: MobileRobot, task: FleetTask, from?: { x: number; y: number; t: number }): number {
    if (task.requiredCapabilities.some((c) => !robot.capabilities.includes(c))) return Infinity;
    if (robot.state.status === 'error') return Infinity;
    const f = from ?? { x: robot.state.x, y: robot.state.y, t: this.time };
    const straight = Math.hypot(task.location[0] - f.x, task.location[1] - f.y);
    const travel = estimateTravelTime(robot, [[f.x, f.y], task.location]) + (robot.kin.drive === 'ackermann' ? 10 : 0) + straight * 0.3 / Math.max(1, robot.kin.maxSpeed);
    const work = task.workPath ? pathLength(task.workPath) / (task.workSpeed ?? robot.kin.maxSpeed * 0.3) + (task.meta?.expectedWorkSeconds ?? 0) : task.duration ?? 0;
    const energyNeeded = ((robot.battery.drivePerMps + robot.battery.idleW) * (travel + work)) / 3600;
    if (energyNeeded > robot.battery.levelWh * 0.9) return Infinity;
    // a work segment held by another robot means waiting at its mouth
    const busySeg = task.segments?.some((sg) => this.holders(sg, robot.id).length >= this.fleet.segmentCapacity) ? 1800 : 0;
    return (f.t - this.time) + travel + work + busySeg + (1 - robot.batteryLevel()) * 60;
  }

  /** Where and when a robot is free again (after its current task and its queue). */
  queueEnd(robot: MobileRobot): { x: number; y: number; t: number; n: number } {
    let x = robot.state.x, y = robot.state.y, t = this.time, n = 0;
    const cur = robot.state.taskId ? this.fleet.tasks.find((k) => k.id === robot.state.taskId) : null;
    const add = (task: FleetTask) => {
      const end = task.workPath?.length ? task.workPath[task.workPath.length - 1] : task.location;
      const c = this.taskCost(robot, task, { x, y, t });
      if (Number.isFinite(c)) t = this.time + c;
      x = end[0]; y = end[1]; n++;
    };
    if (cur && cur.type !== 'charge') add(cur);
    for (const q of this.queued(robot)) add(q);
    return { x, y, t, n };
  }

  private queued(robot: MobileRobot): FleetTask[] {
    return this.fleet.tasks.filter((k) => k.status === 'assigned' && k.robotId === robot.id).sort((a, b) => (a.meta._queue ?? a.createdAt) - (b.meta._queue ?? b.createdAt));
  }

  /** Allocate pending tasks with the fleet's allocator; start tasks of idle robots (own queue first). */
  allocate(): void {
    const robots = this.robots();
    const free = (r: MobileRobot) => !r.state.taskId && r.state.status !== 'charging' && r.state.status !== 'error' && !this.traffic.get(r.id)?.yield;
    // idle robots start their queued tasks first
    for (const r of robots) if (free(r)) { const q = this.queued(r)[0]; if (q && !(q.meta?._rt?.retryAt > this.time)) this.startTask(r, q); }
    const pending = this.fleet.tasks.filter((t) => t.status === 'pending' && !(t.meta?._rt?.retryAt > this.time)).sort((a, b) => b.priority - a.priority || a.createdAt - b.createdAt);
    if (!pending.length) return;
    const alloc = allocators.get(this.fleet.allocation) ?? allocators.get('auction')!;
    const ctx: AllocationContext = { fm: this, robots: robots.filter((r) => r.state.status !== 'error'), tasks: pending, cost: (r, t, f) => this.taskCost(r, t, f), queueEnd: (r) => this.queueEnd(r), queueLimit: this.fleet.queueLimit };
    for (const a of alloc(ctx)) {
      if (a.task.status !== 'pending') continue;
      if (free(a.robot) && !this.queued(a.robot).length) this.startTask(a.robot, a.task);
      else {
        a.task.status = 'assigned'; a.task.robotId = a.robot.id; a.task.meta._queue = this.time + (this.queueSeq++) * 1e-6;
        taskProcess(a.task.meta.process ?? a.task.type)?.onAssign?.(this, a.task, a.robot);
        this.logMsg(`Task ${a.task.id} queued for ${a.robot.name}`);
        this.events.emit('taskUpdated', { task: a.task });
      }
    }
  }

  /** Assign a task to an idle robot and start travelling to it. */
  startTask(robot: MobileRobot, task: FleetTask): boolean {
    const proc = taskProcess(task.meta.process ?? task.type);
    if (task.robotId !== robot.id) proc?.onAssign?.(this, task, robot);
    const heading = task.goalHeading ?? (task.workPath && task.workPath.length > 1 ? Math.atan2(task.workPath[1][1] - task.workPath[0][1], task.workPath[1][0] - task.workPath[0][0]) * 180 / Math.PI : undefined);
    const path = this.planTo(robot, task.location, { goalHeading: heading });
    const rt = this.rt(task);
    if (!path) {
      rt.noPath = (rt.noPath ?? 0) + 1;
      rt.retryAt = this.time + 20;
      this.logMsg(`No path for ${robot.name} to task ${task.id}`, 'warn');
      if (rt.noPath >= 5) { task.status = 'failed'; task.robotId = robot.id; this.events.emit('taskUpdated', { task }); }
      else if (task.status === 'assigned') { task.status = 'pending'; task.robotId = null; }
      return false;
    }
    task.robotId = robot.id;
    task.status = 'travelling';
    task.startedAt = this.time;
    rt.phase = 'travel';
    this.waits.push(this.time - task.createdAt);
    robot.state.taskId = task.id;
    followPath(robot, path);
    this.logMsg(`Task ${task.id} -> ${robot.name} (ETA ${estimateTravelTime(robot, path).toFixed(0)} s)`);
    this.events.emit('taskUpdated', { task });
    return true;
  }

  private rt(task: FleetTask): TaskRuntime {
    if (!task.meta) task.meta = {};
    if (!task.meta._rt) task.meta._rt = { phase: task.status === 'working' ? 'work' : 'travel' } as TaskRuntime;
    return task.meta._rt as TaskRuntime;
  }

  // -- segment reservations ---------------------------------------------------------------------------------
  private canReserve(robotId: string, seg: string, cap = this.fleet.segmentCapacity): boolean {
    const set = this.reservations.get(seg);
    return !set || set.has(robotId) || set.size < cap;
  }
  private reserve(robotId: string, seg: string): void {
    if (!this.reservations.has(seg)) this.reservations.set(seg, new Set());
    this.reservations.get(seg)!.add(robotId);
  }
  private holdSegments(robotId: string, segs: string[]): boolean {
    if (!segs.every((s) => this.canReserve(robotId, s))) return false;
    for (const s of segs) this.reserve(robotId, s);
    this.taskHolds.set(robotId, new Set(segs));
    return true;
  }
  private releaseAll(robotId: string): void {
    for (const set of this.reservations.values()) set.delete(robotId);
    this.taskHolds.delete(robotId);
  }
  /** Robots holding a segment (other than `except`). */
  private holders(seg: string, except: string): string[] { return [...(this.reservations.get(seg) ?? [])].filter((x) => x !== except); }

  /** Charging policy: send low-battery idle robots to the nearest charging zone. */
  private manageCharging(): void {
    const zones = this.station.itemsOfType<ZoneItem>(ItemType.ZONE).filter((z) => z.kind === 'charging');
    for (const r of this.robots()) {
      if (r.state.status === 'charging') {
        if (r.batteryLevel() >= 0.95) { r.state.status = 'idle'; this.logMsg(`${r.name} charged`); }
        continue;
      }
      if (r.state.taskId || r.state.status !== 'idle' || this.traffic.get(r.id)?.yield) continue;
      if (r.batteryLevel() > r.battery.lowThreshold) continue;
      const home = r.home ?? (zones.length ? { ...(() => { const [x, y] = zones[0].centroid(); const abs = zones[0].poseAbs(); return { x: abs[12] + x, y: abs[13] + y }; })(), theta: 0 } : null);
      if (!home) continue;
      const path = this.planTo(r, [home.x, home.y], { goalHeading: r.home ? home.theta : undefined });
      if (!path) continue;
      const task = this.addTask({ type: 'charge', location: [home.x, home.y], priority: 100, robotId: r.id, status: 'travelling', startedAt: this.time });
      this.rt(task).phase = 'travel';
      r.state.taskId = task.id;
      followPath(r, path);
      this.logMsg(`${r.name} battery ${(r.batteryLevel() * 100).toFixed(0)}% -> charging`);
    }
  }

  /** Advance the fleet simulation by dt seconds. */
  step(dt: number): void {
    this.time += dt;
    this.manageCharging();
    this.allocate();
    const robots = this.robots();
    for (const r of robots) this.stepRobot(r, dt, robots);
    this.resolveDeadlocks(robots);
    if (this.checkContacts) this.measureContacts(robots);
  }

  private stepRobot(r: MobileRobot, dt: number, robots: MobileRobot[]): void {
    const task = r.state.taskId ? this.fleet.tasks.find((t) => t.id === r.state.taskId) ?? null : null;
    const tr = this.trafficOf(r);
    // yield manoeuvre in progress
    if (tr.yield) { this.stepYield(r, dt, robots); return; }
    if (task && (task.status === 'done' || task.status === 'cancelled' || task.status === 'failed')) { r.state.taskId = null; if (r.state.status !== 'charging') r.state.status = 'idle'; }
    if (!task || task.status === 'done' || task.status === 'cancelled' || task.status === 'failed') { this.drive(r, dt, robots, undefined); return; }
    const rt = this.rt(task);
    const proc = taskProcess(task.meta.process ?? task.type);
    // arrival at the task location
    if (task.status === 'travelling' && !r.state.path) {
      if (task.type === 'charge') {
        task.status = 'done'; task.finishedAt = this.time; r.state.taskId = null; r.state.status = 'charging';
        this.events.emit('taskUpdated', { task });
        return;
      }
      if (task.workPath && task.workPath.length > 1) {
        if (task.segments?.length && !this.holdSegments(r.id, task.segments)) { this.setWaiting(r, dt, null, task.segments[0]); integrateStop(r, dt); return; }
        task.status = 'working';
        rt.phase = 'work';
        followPath(r, task.workPath);
        r.state.status = 'working';
        proc?.onWorkStart?.(this, task, r);
        this.events.emit('taskUpdated', { task });
      } else {
        task.status = 'working';
        rt.phase = 'work';
        rt.workT = task.duration ?? 5;
        r.state.status = 'working';
        proc?.onWorkStart?.(this, task, r);
        this.events.emit('taskUpdated', { task });
      }
    }
    if (task.status === 'working' && !task.workPath) {
      if (rt.phase === 'service' || rt.phase === 'serviceWait') { this.stepService(r, task, rt, proc, dt, robots); return; }
      rt.workT = (rt.workT ?? task.meta._workLeft ?? 0) - dt;
      r.battery.levelWh = Math.max(0, r.battery.levelWh - (r.battery.idleW * 3 * dt) / 3600);
      integrateStop(r, dt);
      const d = proc?.onWork?.(this, task, r, 0, dt, rt.workT > 0) ?? {};
      this.addBusy(r, dt);
      if (rt.workT > 0) return;
      if (d.service) {
        // timed work followed by a delivery / service leg (transport: load here, unload there)
        rt.phase = 'service'; rt.service = { ...d.service };
        const path = this.planTo(r, d.service.location);
        if (path) followPath(r, path); else { rt.phase = 'serviceWait'; rt.service.left = d.service.duration; }
        return;
      }
      this.finishTask(r, task);
      return;
    }
    if (task.status === 'working') {
      // service trip (refill / unload) interrupting the work
      if (rt.phase === 'service' || rt.phase === 'serviceWait' || rt.phase === 'return') { this.stepService(r, task, rt, proc, dt, robots); return; }
      const s = this.progressOnWork(r, task);
      if (rt.hold && rt.hold > 0) {
        rt.hold -= dt;
        integrateStop(r, dt);
        r.state.status = 'working';
        proc?.onWork?.(this, task, r, s, dt, true);
        this.addBusy(r, dt);
        this.events.emit('robotUpdated', { robot: r });
        return;
      }
      const d = proc?.onWork?.(this, task, r, s, dt, false) ?? {};
      if (d.service) {
        rt.phase = 'service'; rt.service = { ...d.service }; rt.resumeS = s;
        this.releaseAll(r.id);
        const path = this.planTo(r, d.service.location);
        if (path) { followPath(r, path); this.logMsg(`${r.name}: ${d.service.kind} trip from ${task.id}`); }
        else { this.logMsg(`${r.name}: no path for ${d.service.kind}`, 'warn'); rt.phase = 'serviceWait'; rt.service.left = d.service.duration; }
        r.state.status = 'working';
        return;
      }
      if (d.hold && d.hold > 0) { rt.hold = d.hold; integrateStop(r, dt); return; }
      const speed = d.speed ?? task.workSpeed ?? r.kin.maxSpeed * 0.3;
      const moving = this.drive(r, dt, robots, speed);
      if (!moving && r.state.path === null) this.finishTask(r, task);
      return;
    }
    // travelling
    this.drive(r, dt, robots, undefined);
  }

  /** Progress (mm) of the robot along the task's work path (projection). */
  progressOnWork(r: MobileRobot, task: FleetTask): number {
    const w = task.workPath!;
    let best = Infinity, sBest = 0, acc = 0;
    for (let i = 0; i + 1 < w.length; i++) {
      const a = w[i], b = w[i + 1];
      const dx = b[0] - a[0], dy = b[1] - a[1], l = Math.hypot(dx, dy) || 1;
      const u = Math.max(0, Math.min(1, ((r.state.x - a[0]) * dx + (r.state.y - a[1]) * dy) / (l * l)));
      const d = Math.hypot(r.state.x - a[0] - u * dx, r.state.y - a[1] - u * dy);
      if (d < best) { best = d; sBest = acc + u * l; }
      acc += l;
    }
    return sBest;
  }

  private stepService(r: MobileRobot, task: FleetTask, rt: TaskRuntime, proc: TaskProcess | undefined, dt: number, robots: MobileRobot[]): void {
    const sv = rt.service!;
    if (rt.phase === 'service') {
      if (r.state.path) { this.drive(r, dt, robots, undefined); return; }
      rt.phase = 'serviceWait'; sv.left = sv.duration;
    }
    if (rt.phase === 'serviceWait') {
      sv.left = (sv.left ?? sv.duration) - dt;
      integrateStop(r, dt);
      this.addBusy(r, dt);
      if (sv.left > 0) return;
      proc?.onService?.(this, task, r, sv.kind);
      if (!task.workPath || task.workPath.length < 2) { this.finishTask(r, task); return; }
      // back to the work path where it was interrupted
      const w = task.workPath!;
      const at = pointAtS(w, rt.resumeS ?? 0);
      const path = this.planTo(r, [at[0], at[1]], { goalHeading: at[2] });
      if (!path) { this.logMsg(`${r.name}: no path back to ${task.id}`, 'warn'); return; }
      followPath(r, path);
      rt.phase = 'return';
      return;
    }
    if (rt.phase === 'return') {
      if (r.state.path) { this.drive(r, dt, robots, undefined); return; }
      if (task.segments?.length && !this.holdSegments(r.id, task.segments)) { this.setWaiting(r, dt, null, task.segments[0]); integrateStop(r, dt); return; }
      followPath(r, remainderOf(task.workPath!, rt.resumeS ?? 0));
      rt.phase = 'work';
      r.state.status = 'working';
    }
  }

  // -- motion with traffic management -------------------------------------------------------------------------

  private trafficOf(r: MobileRobot): RobotTraffic {
    let t = this.traffic.get(r.id);
    if (!t) { t = { waitSince: null, by: null, seg: null, replanAt: 0 }; this.traffic.set(r.id, t); }
    return t;
  }

  private setWaiting(r: MobileRobot, dt: number, by: string | null, seg: string | null): void {
    const t = this.trafficOf(r);
    if (t.waitSince === null) t.waitSince = this.time;
    t.by = by; t.seg = seg;
    if (r.state.status !== 'charging') r.state.status = 'waiting';
    this.waitT.set(r.id, (this.waitT.get(r.id) ?? 0) + dt);
    this.trafficKpi.waitSeconds += dt;
  }

  /** Rectangle of a robot's footprint at its current pose. */
  rectOf(r: MobileRobot): Rect2 { return footprintRect(r.state, r.kin.footprint); }

  /**
   * Distance the robot may still travel along its path before its footprint would come within `safetyGap` of another
   * robot (current footprint and, when moving, its stopping footprint) or enter a segment it cannot reserve.
   * Reserves transit segments on the way.
   */
  private freeDistance(r: MobileRobot, robots: MobileRobot[]): { d: number; by: string | null; seg: string | null } {
    const path = r.state.path;
    if (!path || !isPosePath(path)) return { d: Infinity, by: null, seg: null };
    const k = r.kin;
    const v = Math.abs(r.state.v);
    const brake = (v * v) / (2 * Math.max(100, k.maxAccel * 0.8));
    const horizon = brake + this.safetyGap + 1500;
    const others = robots.filter((o) => o !== r && Math.hypot(o.state.x - r.state.x, o.state.y - r.state.y) < horizon + (Math.hypot(o.kin.footprint[0], o.kin.footprint[1]) + Math.hypot(k.footprint[0], k.footprint[1])) / 2 + 3000);
    const orects: Array<{ id: string; rects: Rect2[] }> = others.map((o) => {
      const rects = [this.rectOf(o)];
      const ov = Math.hypot(o.state.v, o.state.vy ?? 0);
      if (ov > 50) {
        const ob = (ov * ov) / (2 * Math.max(100, o.kin.maxAccel * 0.8)) + 200;
        const th = (o.state.theta * Math.PI) / 180, sgn = Math.sign(o.state.v) || 1;
        rects.push({ ...rects[0], x: o.state.x + Math.cos(th) * ob * sgn, y: o.state.y + Math.sin(th) * ob * sgn });
        rects.push({ ...rects[0], x: o.state.x + Math.cos(th) * ob * 0.5 * sgn, y: o.state.y + Math.sin(th) * ob * 0.5 * sgn });
      }
      return { id: o.id, rects };
    });
    const segs = this.segments();
    const segHorizon = horizon + k.footprint[0];
    const L = k.footprint[0], W = k.footprint[1];
    const mine: Rect2 = this.rectOf(r);
    // samples along the remaining path (current pose first)
    let dist = 0;
    let prev: number[] = [r.state.x, r.state.y];
    const samples: Array<{ x: number; y: number; th: number; d: number; rot: boolean }> = [{ x: r.state.x, y: r.state.y, th: r.state.theta, d: 0, rot: false }];
    for (let i = Math.max(1, r.state.pathIndex); i < path.length && dist < segHorizon; i++) {
      const p = path[i];
      const step = Math.hypot(p[0] - prev[0], p[1] - prev[1]);
      dist += step;
      prev = p;
      if (p[3] !== 0 && dist - samples[samples.length - 1].d < 250 && i + 1 < path.length) continue;
      if (p[3] === 0 && samples[samples.length - 1].rot) continue;
      samples.push({ x: p[0], y: p[1], th: p[2], d: dist, rot: p[3] === 0 });
    }
    let dFree = Infinity, by: string | null = null, seg: string | null = null;
    for (const s of samples) {
      if (s.d > horizon) break;
      const rect: Rect2 = s.rot ? { x: s.x, y: s.y, theta: 0, L: Math.hypot(L, W), W: Math.hypot(L, W) } : { x: s.x, y: s.y, theta: s.th, L, W };
      for (const o of orects) for (const q of o.rects) {
        if (!rectsOverlap(rect, q, this.safetyGap)) continue;
        // already overlapping at the current pose: only block motion that does not increase the separation
        if (s.d === 0 && !s.rot) { if (this.separating(r, robots.find((x) => x.id === o.id)!, samples[1])) continue; }
        if (s.d < dFree) { dFree = s.d; by = o.id; }
      }
      if (dFree <= s.d) break;
    }
    // transit reservations
    const rad = Math.hypot(L, W) / 2;
    const nearSeg = (sg: TrafficSegment, x: number, y: number) => { const b = (sg as any)._bb; return !b || (x + rad >= b.x0 && x - rad <= b.x1 && y + rad >= b.y0 && y - rad <= b.y1); };
    for (const sg of segs) {
      if (this.reservations.get(sg.id)?.has(r.id)) continue;
      for (const s of samples) {
        if (s.d > segHorizon || s.d >= dFree) break;
        if (!nearSeg(sg, s.x, s.y)) continue;
        const rect: Rect2 = { x: s.x, y: s.y, theta: s.th, L, W };
        if (!rectIntersectsPolygon(rect, sg.polygon)) continue;
        if (this.canReserve(r.id, sg.id, sg.capacity ?? this.fleet.segmentCapacity)) this.reserve(r.id, sg.id);
        else if (s.d - 50 < dFree) { dFree = Math.max(0, s.d - 50); seg = sg.id; by = this.holders(sg.id, r.id)[0] ?? null; }
        break;
      }
    }
    // release transit segments the robot left and will not re-enter within the horizon
    const holds = this.taskHolds.get(r.id);
    for (const sg of segs) {
      const set = this.reservations.get(sg.id);
      if (!set?.has(r.id) || holds?.has(sg.id)) continue;
      if (nearSeg(sg, mine.x, mine.y) && rectIntersectsPolygon(mine, sg.polygon)) continue;
      if (samples.some((s) => s.d < segHorizon && nearSeg(sg, s.x, s.y) && rectIntersectsPolygon({ x: s.x, y: s.y, theta: s.th, L, W }, sg.polygon))) continue;
      set.delete(r.id);
    }
    return { d: dFree, by, seg };
  }

  /** True when the robot's next path sample increases its centre distance to `o` (allowed even if already too close). */
  private separating(r: MobileRobot, o: MobileRobot, next?: { x: number; y: number }): boolean {
    if (!next) return false;
    return Math.hypot(next.x - o.state.x, next.y - o.state.y) > Math.hypot(r.state.x - o.state.x, r.state.y - o.state.y) + 1;
  }

  /** Move a robot along its path with the traffic speed cap. Returns true while it moves. */
  private drive(r: MobileRobot, dt: number, robots: MobileRobot[], desired: number | undefined): boolean {
    const tr = this.trafficOf(r);
    if (!r.state.path) { stepMobile(r, dt); tr.waitSince = null; tr.by = null; tr.seg = null; return false; }
    const fd = this.freeDistance(r, robots);
    let cap: number | undefined;
    if (Number.isFinite(fd.d)) {
      const a = Math.max(100, r.kin.maxAccel * 0.8);
      cap = Math.sqrt(2 * a * Math.max(0, fd.d - 100));
      if (cap < 40) {
        this.setWaiting(r, dt, fd.by, fd.seg);
        integrateStop(r, dt);
        this.maybeReplan(r, fd);
        this.events.emit('robotUpdated', { robot: r });
        return true;
      }
    }
    if (tr.waitSince !== null) { tr.waitSince = null; tr.by = null; tr.seg = null; }
    const task = r.state.taskId ? this.fleet.tasks.find((t) => t.id === r.state.taskId) : null;
    if (r.state.status === 'waiting') r.state.status = task?.status === 'working' ? 'working' : 'moving';
    const nav = getNavStack(r);
    const rtn = nav?.simulate ? ((r as any)._nav as NavRuntime | undefined) : undefined;
    if (rtn?.estimator.state.lost) { integrateStop(r, dt); this.setWaiting(r, dt, null, null); stepNavRuntime(r, dt, this.map(), this.zones()); return true; }
    const pose = rtn ? { x: rtn.estimator.state.x, y: rtn.estimator.state.y, theta: rtn.estimator.state.theta } : undefined;
    const speed = cap !== undefined ? Math.min(cap, desired ?? r.kin.maxSpeed) : desired;
    const moving = stepMobile(r, dt, { speed, pose });
    if (nav?.simulate) stepNavRuntime(r, dt, this.map(), this.zones());
    if (moving) this.addBusy(r, dt);
    this.events.emit('robotUpdated', { robot: r });
    return moving;
  }

  private addBusy(r: MobileRobot, dt: number) { this.busy.set(r.id, (this.busy.get(r.id) ?? 0) + dt); }

  /** After waiting a while: replan around segments held by others / the blocking robot. */
  private maybeReplan(r: MobileRobot, fd: { by: string | null; seg: string | null }): void {
    const tr = this.trafficOf(r);
    if (tr.waitSince === null || this.time - tr.waitSince < 6 || this.time < tr.replanAt) return;
    tr.replanAt = this.time + 10;
    const path = r.state.path;
    if (!path || !isPosePath(path)) return;
    const goal = path[path.length - 1];
    const task = r.state.taskId ? this.fleet.tasks.find((t) => t.id === r.state.taskId) : null;
    if (task?.status === 'working' && (this.rt(task).phase === 'work')) return; // stay on the work line
    const blocked = this.segments().filter((s) => this.holders(s.id, r.id).length >= (s.capacity ?? this.fleet.segmentCapacity) && !this.reservations.get(s.id)?.has(r.id)).map((s) => s.polygon);
    const blocker = fd.by ? this.robots().find((o) => o.id === fd.by) : null;
    const extra = blocker && !blocker.state.path ? robotCircles(blocker, this.safetyGap) : [];
    if (!blocked.length && !extra.length) return;
    const np = this.planTo(r, [goal[0], goal[1]], { goalHeading: goal[2], blocked, extra });
    if (!np) return;
    const oldLen = remainingLength(path, r.state.pathIndex);
    if (pathLength(np) > oldLen * 3 + 20000) return;
    followPath(r, np);
    this.trafficKpi.replans++;
    this.logMsg(`${r.name} replanned around ${fd.seg ?? fd.by ?? 'traffic'}`);
  }

  // -- deadlock resolution -----------------------------------------------------------------------------------

  private resolveDeadlocks(robots: MobileRobot[]): void {
    const byId = new Map(robots.map((r) => [r.id, r]));
    const waitFor = (r: MobileRobot): string | null => {
      const t = this.traffic.get(r.id);
      if (!t || t.waitSince === null || t.yield) return null;
      return t.by;
    };
    for (const r of robots) {
      const t = this.traffic.get(r.id);
      if (!t || t.waitSince === null || t.yield || this.time - t.waitSince < 3) continue;
      // follow the wait-for chain
      const chain: MobileRobot[] = [r];
      let cur: MobileRobot | undefined = r;
      let cycle = false;
      for (let i = 0; i < robots.length + 1; i++) {
        const nxt: string | null = cur ? waitFor(cur) : null;
        if (!nxt) break;
        const o = byId.get(nxt);
        if (!o) break;
        if (o === r) { cycle = true; break; }
        if (chain.includes(o)) break;
        chain.push(o);
        cur = o;
      }
      const last = chain[chain.length - 1];
      const lastIdle = !cycle && last !== r && !last.state.path && !this.traffic.get(last.id)?.yield;
      if (!cycle && !lastIdle) continue;
      if (lastIdle && this.time - t.waitSince < 5) continue;
      // choose who yields: not working on its work line, not inside a held segment; prefer the idle robot, then the waiting-for-segment robot
      const cands = (lastIdle ? [last] : chain).filter((x) => this.canYield(x));
      if (!cands.length) continue;
      cands.sort((a, b) => this.yieldScore(a) - this.yieldScore(b));
      const y = cands[0];
      const pass = robots.find((x) => x !== y && this.traffic.get(x.id)?.by === y.id && this.traffic.get(x.id)?.waitSince !== null) ?? byId.get(this.traffic.get(y.id)?.by ?? '') ?? chain.find((x) => x !== y);
      if (!pass) continue;
      if (this.startYield(y, pass, robots)) { this.trafficKpi.deadlocks++; break; }
    }
  }

  private canYield(r: MobileRobot): boolean {
    const task = r.state.taskId ? this.fleet.tasks.find((t) => t.id === r.state.taskId) : null;
    if (task?.status === 'working' && this.rt(task).phase === 'work') return false;
    const mine = this.rectOf(r);
    return !this.segments().some((s) => this.reservations.get(s.id)?.has(r.id) && rectIntersectsPolygon(mine, s.polygon));
  }
  private yieldScore(r: MobileRobot): number {
    const t = this.traffic.get(r.id);
    const task = r.state.taskId ? this.fleet.tasks.find((k) => k.id === r.state.taskId) : null;
    return (r.state.path ? 0 : -100) + (t?.seg ? -10 : 0) + (task?.priority ?? 0);
  }

  /** Move `y` out of the way of `pass`: nearest free pose whose footprint keeps clear of pass's remaining path. */
  private startYield(y: MobileRobot, pass: MobileRobot, robots: MobileRobot[]): boolean {
    const world = this.world();
    const P = pass.state.path && isPosePath(pass.state.path) ? pass.state.path.slice(pass.state.pathIndex) : [[pass.state.x, pass.state.y]];
    const needP = Math.hypot(y.kin.footprint[0], y.kin.footprint[1]) / 2 + pass.kin.footprint[1] / 2 + this.safetyGap + 200;
    const nearPath = (x: number, yy: number) => {
      let acc = 0;
      for (let i = 0; i < P.length; i++) {
        if (i > 0) acc += Math.hypot(P[i][0] - P[i - 1][0], P[i][1] - P[i - 1][1]);
        if (acc > 40000) break;
        if (Math.hypot(P[i][0] - x, P[i][1] - yy) < needP) return true;
      }
      return false;
    };
    const L = y.kin.footprint[0], W = y.kin.footprint[1], H = y.kin.footprint[2];
    const others = robots.filter((o) => o !== y);
    const cands: Array<{ x: number; y: number; th: number; d: number }> = [];
    for (const rad of [1500, 2500, 3500, 5000, 7000, 9000]) for (let a = 0; a < 360; a += 30) for (const dth of [0, 90, -90, 180]) {
      const x = y.state.x + rad * Math.cos((a * Math.PI) / 180), yy = y.state.y + rad * Math.sin((a * Math.PI) / 180);
      const th = wrap180(y.state.theta + dth);
      if (nearPath(x, yy)) continue;
      if (world.clearance(x, yy, th, L, W, H, this.planMargin) < this.planMargin) continue;
      if (others.some((o) => rectsOverlap({ x, y: yy, theta: th, L, W }, this.rectOf(o), this.safetyGap * 2))) continue;
      cands.push({ x, y: yy, th, d: rad + Math.abs(dth) * 5 });
    }
    cands.sort((a, b) => a.d - b.d);
    const extra = robotCircles(pass, this.safetyGap);
    for (const c of cands.slice(0, 12)) {
      const path = this.planTo(y, [c.x, c.y], { goalHeading: c.th, extra });
      if (!path) continue;
      const tr = this.trafficOf(y);
      const resume = y.state.path && isPosePath(y.state.path) ? (() => { const g = y.state.path![y.state.path!.length - 1]; return { x: g[0], y: g[1], theta: g[2] }; })() : null;
      tr.yield = { waitFor: pass.id, resume, arrived: false, t0: this.time };
      tr.waitSince = null; tr.by = null; tr.seg = null;
      // drop transit reservations so the passing robot can enter
      const holds = this.taskHolds.get(y.id);
      for (const [sid, set] of this.reservations) if (!holds?.has(sid)) set.delete(y.id);
      followPath(y, path);
      if (y.state.status !== 'charging') y.state.status = 'moving';
      this.trafficKpi.yields++;
      this.logMsg(`${y.name} yields to ${pass.name}`);
      return true;
    }
    return false;
  }

  private stepYield(r: MobileRobot, dt: number, robots: MobileRobot[]): void {
    const tr = this.trafficOf(r);
    const y = tr.yield!;
    if (!y.arrived) {
      if (r.state.path) { this.drive(r, dt, robots, undefined); if (this.time - y.t0 < 120) return; }
      y.arrived = true; y.t0 = this.time;
    }
    integrateStop(r, dt);
    const pass = robots.find((o) => o.id === y.waitFor);
    const need = Math.hypot(r.kin.footprint[0], r.kin.footprint[1]) / 2 + (pass ? Math.hypot(pass.kin.footprint[0], pass.kin.footprint[1]) / 2 : 0) + this.safetyGap;
    const passed = !pass || !pass.state.path || Math.hypot(pass.state.x - r.state.x, pass.state.y - r.state.y) > need + 3000 && !this.pathNear(pass, r, need);
    if (!passed && this.time - y.t0 < 90) { this.waitT.set(r.id, (this.waitT.get(r.id) ?? 0) + dt); return; }
    tr.yield = undefined;
    if (y.resume) {
      const np = this.planTo(r, [y.resume.x, y.resume.y], { goalHeading: y.resume.theta });
      if (np) followPath(r, np);
      const task = r.state.taskId ? this.fleet.tasks.find((t) => t.id === r.state.taskId) : null;
      r.state.status = task?.status === 'working' ? 'working' : 'moving';
    } else if (r.state.status !== 'charging') r.state.status = 'idle';
  }

  private pathNear(p: MobileRobot, r: MobileRobot, need: number): boolean {
    const path = p.state.path;
    if (!path) return false;
    let acc = 0;
    for (let i = p.state.pathIndex; i < path.length; i++) {
      if (i > p.state.pathIndex) acc += Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]);
      if (acc > 30000) break;
      if (Math.hypot(path[i][0] - r.state.x, path[i][1] - r.state.y) < need) return true;
    }
    return false;
  }

  // -- contacts --------------------------------------------------------------------------------------------------

  private measureContacts(robots: MobileRobot[]): void {
    const world = this.world();
    const seen = new Set<string>();
    for (const r of robots) {
      const rect = this.rectOf(r);
      // aerial robots only collide with obstacles reaching their altitude
      const relevant = r.kin.drive === 'aerial' ? world.contacts(rect).filter((c) => c.zMax > (r.state.z ?? 0) && c.zMin < (r.state.z ?? 0) + r.kin.footprint[2]) : world.contacts(rect, r.kin.footprint[2]);
      if (relevant.length) this.contactKpi.robotObstacleSteps++;
      for (const c of relevant) {
        const key = `${r.id}|${c.kind}|${c.ref?.rowId ?? ''}:${c.ref?.plant ?? `${c.x.toFixed(0)},${c.y.toFixed(0)}`}`;
        seen.add(key);
        const depth = c.r - Math.max(0, pointRectDistanceSafe(c.x, c.y, rect));
        this.contactKpi.maxDepth = Math.max(this.contactKpi.maxDepth, depth);
        if (!this.contactState.has(key)) {
          this.contactKpi.robotObstacle++;
          const ev = { t: this.time, robot: r.name, other: `${c.kind}${c.ref?.plant !== undefined ? ` ${c.ref.rowId}#${c.ref.plant}` : ''}`, kind: c.kind, depth };
          this.pushContact(ev);
        }
        this.contactState.set(key, this.time);
      }
    }
    let rrStep = false;
    for (let i = 0; i < robots.length; i++) for (let j = i + 1; j < robots.length; j++) {
      const a = robots[i], b = robots[j];
      if (a.kin.drive === 'aerial' || b.kin.drive === 'aerial') { if (Math.abs((a.state.z ?? 0) - (b.state.z ?? 0)) > Math.max(a.kin.footprint[2], b.kin.footprint[2])) continue; }
      if (!rectsOverlap(this.rectOf(a), this.rectOf(b), 0)) continue;
      rrStep = true;
      const key = `${a.id}|robot|${b.id}`;
      seen.add(key);
      if (!this.contactState.has(key)) { this.contactKpi.robotRobot++; this.pushContact({ t: this.time, robot: a.name, other: b.name, kind: 'robot', depth: 0 }); }
      this.contactState.set(key, this.time);
    }
    if (rrStep) this.contactKpi.robotRobotSteps++;
    for (const k of [...this.contactState.keys()]) if (!seen.has(k)) this.contactState.delete(k);
  }

  private pushContact(ev: ContactEvent) {
    this.contactKpi.log.push(ev);
    if (this.contactKpi.log.length > 200) this.contactKpi.log.shift();
    this.events.emit('contact', ev);
    if (this.contactKpi.log.length <= 20) this.logMsg(`Contact: ${ev.robot} × ${ev.other}`, 'warn');
  }

  /** Book yield from a process model (called at the moment fruit is removed). */
  bookYield(y: { fruit?: number; kg?: number; areaM2?: number }): void {
    this.yield.fruit += y.fruit ?? 0;
    this.yield.kg += y.kg ?? 0;
    this.yield.areaM2 += y.areaM2 ?? 0;
  }
  /** Add to a process statistic (e.g. bookStat('spray', 'litres', 3.2)). */
  bookStat(kind: string, key: string, v: number): void {
    const s = (this.processStats[kind] ??= {});
    s[key] = (s[key] ?? 0) + v;
  }

  private finishTask(r: MobileRobot, task: FleetTask): void {
    task.status = 'done';
    task.finishedAt = this.time;
    const proc = taskProcess(task.meta.process ?? task.type);
    const y = proc?.onDone?.(this, task, r) ?? undefined;
    if (y) this.bookYield(y);
    else if (!proc) {
      if (task.workPath && task.workPath.length > 1) this.yield.areaM2 += (pathLength(task.workPath) / 1000) * ((task.meta?.swathM as number) ?? 2);
      if (task.type === 'harvest') {
        const h = this.onHarvestDone?.(task) ?? { fruit: task.meta?.fruit ?? 0, kg: ((task.meta?.fruit ?? 0) * (task.meta?.fruitKg ?? 0.18)) };
        this.yield.fruit += h.fruit;
        this.yield.kg += h.kg;
        task.meta.harvestedKg = h.kg;
      }
    }
    r.state.taskId = null;
    r.state.status = 'idle';
    this.releaseAll(r.id);
    this.logMsg(`Task ${task.id} done by ${r.name} in ${(task.finishedAt - (task.startedAt ?? task.createdAt)).toFixed(0)} s`);
    this.events.emit('taskUpdated', { task });
  }

  kpis(): FleetKPIs {
    const robots = this.robots();
    const done = this.fleet.tasks.filter((t) => t.status === 'done' && t.type !== 'charge');
    const perRobot: FleetKPIs['perRobot'] = {};
    let busySum = 0, dist = 0, energy = 0;
    for (const r of robots) {
      const b = this.busy.get(r.id) ?? 0;
      busySum += b;
      const d = r.state.odometer - (this.distance0.get(r.id) ?? 0);
      dist += d;
      energy += Math.max(0, (this.energy0.get(r.id) ?? r.battery.levelWh) - r.battery.levelWh);
      perRobot[r.id] = { busyTime: b, distance: d, tasks: done.filter((t) => t.robotId === r.id).length, battery: r.batteryLevel(), status: r.state.status, waitTime: this.waitT.get(r.id) ?? 0 };
    }
    return {
      fruitPicked: this.yield.fruit,
      yieldKg: this.yield.kg,
      areaWorkedM2: this.yield.areaM2,
      tasksDone: done.length,
      tasksFailed: this.fleet.tasks.filter((t) => t.status === 'failed').length,
      tasksPending: this.fleet.tasks.filter((t) => t.status === 'pending' || t.status === 'assigned').length,
      throughputPerHour: this.time > 0 ? (done.length * 3600) / this.time : 0,
      utilization: robots.length && this.time > 0 ? busySum / (robots.length * this.time) : 0,
      distanceTravelled: dist,
      energyUsedWh: energy,
      meanWaitTime: this.waits.length ? this.waits.reduce((a, b) => a + b, 0) / this.waits.length : 0,
      contacts: { ...this.contactKpi, log: [...this.contactKpi.log] },
      traffic: { ...this.trafficKpi },
      perRobot,
      process: JSON.parse(JSON.stringify(this.processStats)),
    };
  }

  logMsg(text: string, level: 'info' | 'warn' | 'error' = 'info') {
    const e = { t: this.time, text, level };
    this.log.push(e);
    if (this.log.length > 2000) this.log.shift();
    this.events.emit('log', e);
  }
  getLog() {
    return this.log;
  }
  get pending() {
    return this.pendingPathRequests;
  }
}

function pointRectDistanceSafe(px: number, py: number, r: Rect2): number {
  const th = (r.theta * Math.PI) / 180;
  const dx = px - r.x, dy = py - r.y;
  const lx = Math.cos(th) * dx + Math.sin(th) * dy, ly = -Math.sin(th) * dx + Math.cos(th) * dy;
  return Math.hypot(Math.max(0, Math.abs(lx) - r.L / 2), Math.max(0, Math.abs(ly) - r.W / 2));
}

/** Covering circles of a robot footprint (dynamic obstacle for replanning). */
export function robotCircles(r: MobileRobot, grow = 0): CircleObstacle[] {
  const { offsets, r: rad } = coveringCircles(r.kin.footprint[0], r.kin.footprint[1]);
  const th = (r.state.theta * Math.PI) / 180;
  return offsets.map((o) => ({ x: r.state.x + Math.cos(th) * o, y: r.state.y + Math.sin(th) * o, r: rad + grow, zMin: 0, zMax: r.kin.footprint[2], kind: 'robot', ref: { itemId: r.id } }));
}

function remainingLength(path: number[][], from: number): number {
  let l = 0;
  for (let i = Math.max(1, from); i < path.length; i++) l += Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]);
  return l;
}

/** Point (x, y, heading) at arc length s along a polyline. */
export function pointAtS(w: number[][], s: number): [number, number, number] {
  let acc = 0;
  for (let i = 0; i + 1 < w.length; i++) {
    const a = w[i], b = w[i + 1];
    const l = Math.hypot(b[0] - a[0], b[1] - a[1]);
    const th = (Math.atan2(b[1] - a[1], b[0] - a[0]) * 180) / Math.PI;
    if (acc + l >= s || i + 2 === w.length) { const u = l > 0 ? Math.max(0, Math.min(1, (s - acc) / l)) : 0; return [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, th]; }
    acc += l;
  }
  const p = w[w.length - 1];
  return [p[0], p[1], 0];
}

/** Remainder of a polyline from arc length s. */
export function remainderOf(w: number[][], s: number): number[][] {
  const out: number[][] = [];
  let acc = 0;
  const p0 = pointAtS(w, s);
  out.push([p0[0], p0[1]]);
  for (let i = 0; i + 1 < w.length; i++) {
    acc += Math.hypot(w[i + 1][0] - w[i][0], w[i + 1][1] - w[i][1]);
    if (acc > s + 1) out.push([w[i + 1][0], w[i + 1][1]]);
  }
  if (out.length < 2) out.push([w[w.length - 1][0], w[w.length - 1][1]]);
  return out;
}

// ---------------------------------------------------------------------------------------------------------------
// Built-in allocators
// ---------------------------------------------------------------------------------------------------------------

const idle = (ctx: AllocationContext, r: MobileRobot) => !r.state.taskId && r.state.status !== 'charging' && ctx.queueEnd(r).n === 0;

/** Nearest idle robot by straight-line distance (one task per idle robot per round). */
registerAllocator('nearest', (ctx) => {
  const used = new Set<string>();
  const out: Assignment[] = [];
  for (const t of ctx.tasks) {
    let best: MobileRobot | null = null, bd = Infinity;
    for (const r of ctx.robots) {
      if (used.has(r.id) || !idle(ctx, r) || !Number.isFinite(ctx.cost(r, t))) continue;
      const d = Math.hypot(t.location[0] - r.state.x, t.location[1] - r.state.y);
      if (d < bd) { bd = d; best = r; }
    }
    if (best) { used.add(best.id); out.push({ task: t, robot: best }); }
  }
  return out;
});

/** First feasible idle robot. */
registerAllocator('round_robin', (ctx) => {
  const used = new Set<string>();
  const out: Assignment[] = [];
  for (const t of ctx.tasks) {
    const r = ctx.robots.find((x) => !used.has(x.id) && idle(ctx, x) && Number.isFinite(ctx.cost(x, t)));
    if (r) { used.add(r.id); out.push({ task: t, robot: r }); }
  }
  return out;
});

/**
 * Sequential single-item auction: every robot (busy ones too, up to the queue limit) bids its marginal completion time
 * = time until it is free + travel from where it will be + work; the lowest bid wins and the winner's queue end moves.
 */
registerAllocator('auction', (ctx) => {
  const ends = new Map(ctx.robots.map((r) => [r.id, ctx.queueEnd(r)]));
  const out: Assignment[] = [];
  for (const t of ctx.tasks) {
    let best: MobileRobot | null = null, bid = Infinity;
    for (const r of ctx.robots) {
      const e = ends.get(r.id)!;
      if (e.n >= ctx.queueLimit + (r.state.taskId ? 1 : 0)) continue;
      const c = ctx.cost(r, t, e);
      if (c < bid) { bid = c; best = r; }
    }
    if (!best) continue;
    out.push({ task: t, robot: best });
    const e = ends.get(best.id)!;
    const end = t.workPath?.length ? t.workPath[t.workPath.length - 1] : t.location;
    ends.set(best.id, { x: end[0], y: end[1], t: ctx.fm.time + bid, n: e.n + 1 });
  }
  return out;
});

/** Optimal one-to-one assignment of idle robots to pending tasks (Hungarian / Kuhn–Munkres, min total cost). */
registerAllocator('hungarian', (ctx) => {
  const R = ctx.robots.filter((r) => idle(ctx, r));
  const T = ctx.tasks;
  if (!R.length || !T.length) return [];
  const n = Math.max(R.length, T.length);
  const BIG = 1e9;
  const C: number[][] = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => { if (i >= R.length || j >= T.length) return 0; const c = ctx.cost(R[i], T[j]); return Number.isFinite(c) ? c - 1e3 * T[j].priority : BIG; }));
  const assign = hungarian(C);
  const out: Assignment[] = [];
  assign.forEach((j, i) => { if (i < R.length && j < T.length && C[i][j] < BIG / 2) out.push({ task: T[j], robot: R[i] }); });
  return out;
});

/**
 * CBBA-style bundle allocation (centralised consensus): robots greedily build bundles by marginal cost (up to the
 * queue limit), conflicting claims are resolved in favour of the lowest bid, iterated until no robot changes.
 */
registerAllocator('cbba', (ctx) => {
  const winner = new Map<string, { robot: MobileRobot; bid: number }>();
  for (let iter = 0; iter < 20; iter++) {
    let changed = false;
    for (const r of ctx.robots) {
      let e = ctx.queueEnd(r);
      const mine = [...winner.entries()].filter(([, w]) => w.robot === r).map(([id]) => id);
      for (const id of mine) { const t = ctx.tasks.find((x) => x.id === id)!; const end = t.workPath?.length ? t.workPath[t.workPath.length - 1] : t.location; e = { x: end[0], y: end[1], t: ctx.fm.time + ctx.cost(r, t, e), n: e.n + 1 }; }
      while (e.n < ctx.queueLimit + (r.state.taskId ? 1 : 0)) {
        let best: FleetTask | null = null, bc = Infinity;
        for (const t of ctx.tasks) {
          const w = winner.get(t.id);
          if (w?.robot === r) continue;
          const c = ctx.cost(r, t, e);
          if (!Number.isFinite(c) || (w && w.bid <= c)) continue;
          if (c < bc) { bc = c; best = t; }
        }
        if (!best) break;
        winner.set(best.id, { robot: r, bid: bc });
        changed = true;
        const end = best.workPath?.length ? best.workPath[best.workPath.length - 1] : best.location;
        e = { x: end[0], y: end[1], t: ctx.fm.time + bc, n: e.n + 1 };
      }
    }
    if (!changed) break;
  }
  return ctx.tasks.filter((t) => winner.has(t.id)).map((t) => ({ task: t, robot: winner.get(t.id)!.robot }));
});

/**
 * Contract net (announce → bid → award): every task is announced to all robots; each robot answers with one bid for
 * its best task (marginal cost from its queue end, capability-checked); the manager awards each task to the best
 * bidder; rounds repeat until no bids remain.
 */
registerAllocator('contract_net', (ctx) => {
  const open = new Set(ctx.tasks.map((t) => t.id));
  const ends = new Map(ctx.robots.map((r) => [r.id, ctx.queueEnd(r)]));
  const out: Assignment[] = [];
  for (let round = 0; round < ctx.tasks.length + 1 && open.size; round++) {
    const bids: Array<{ r: MobileRobot; t: FleetTask; c: number }> = [];
    for (const r of ctx.robots) {
      const e = ends.get(r.id)!;
      if (e.n >= ctx.queueLimit + (r.state.taskId ? 1 : 0)) continue;
      let best: { t: FleetTask; c: number } | null = null;
      for (const t of ctx.tasks) if (open.has(t.id)) { const c = ctx.cost(r, t, e) - 1e3 * t.priority; if (Number.isFinite(c) && (!best || c < best.c)) best = { t, c }; }
      if (best) bids.push({ r, ...best });
    }
    if (!bids.length) break;
    const awarded = new Set<string>();
    for (const b of bids.sort((a, b2) => a.c - b2.c)) {
      if (!open.has(b.t.id) || awarded.has(b.r.id)) continue;
      open.delete(b.t.id); awarded.add(b.r.id);
      out.push({ task: b.t, robot: b.r });
      const e = ends.get(b.r.id)!;
      const end = b.t.workPath?.length ? b.t.workPath[b.t.workPath.length - 1] : b.t.location;
      ends.set(b.r.id, { x: end[0], y: end[1], t: ctx.fm.time + ctx.cost(b.r, b.t, e), n: e.n + 1 });
    }
  }
  return out;
});

/** Hungarian algorithm (square cost matrix) → column assigned to each row. */
export function hungarian(C: number[][]): number[] {
  const n = C.length;
  const u = new Float64Array(n + 1), v = new Float64Array(n + 1), p = new Int32Array(n + 1), way = new Int32Array(n + 1);
  for (let i = 1; i <= n; i++) {
    p[0] = i;
    let j0 = 0;
    const minv = new Float64Array(n + 1).fill(Infinity);
    const used = new Uint8Array(n + 1);
    do {
      used[j0] = 1;
      const i0 = p[j0];
      let delta = Infinity, j1 = 0;
      for (let j = 1; j <= n; j++) if (!used[j]) {
        const cur = C[i0 - 1][j - 1] - u[i0] - v[j];
        if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
        if (minv[j] < delta) { delta = minv[j]; j1 = j; }
      }
      for (let j = 0; j <= n; j++) if (used[j]) { u[p[j]] += delta; v[j] -= delta; } else minv[j] -= delta;
      j0 = j1;
    } while (p[j0] !== 0);
    do { const j1 = way[j0]; p[j0] = p[j1]; j0 = j1; } while (j0);
  }
  const ans = new Array(n).fill(-1);
  for (let j = 1; j <= n; j++) if (p[j]) ans[p[j] - 1] = j - 1;
  return ans;
}
