/**
 * Fleet manager: task queue, allocation (cost-based auction), traffic management on a shared
 * segment graph (row reservations), charging policy and KPIs. Runs on the simulation clock and
 * can drive real robots through the ROS 2 bridge (same task objects).
 */
import { Item, ItemType, SerializedItem, registerItemType, Station } from '../core/items/item';
import { MobileRobot, MapItem, ZoneItem } from '../mobile/items';
import { planPath, pathLength } from '../mobile/planner';
import { followPath, stepMobile, estimateTravelTime, integrate } from '../mobile/controller';
import { stepNavRuntime, getNavStack, NavRuntime } from '../mobile/navstack';

function integrateStop(r: MobileRobot, dt: number) { integrate(r, { v: 0, omega: 0 }, dt); }
import { EventBus } from '../core/events';

export type TaskType = 'transport' | 'harvest' | 'spray' | 'mow' | 'prune' | 'scout' | 'pollinate' | 'weed' | 'charge' | 'goto' | 'custom';
export type TaskStatus = 'pending' | 'assigned' | 'travelling' | 'working' | 'done' | 'failed' | 'cancelled';

export interface FleetTask {
  id: string;
  type: TaskType;
  /** Where to go (mm, station frame) — for row tasks the row entry point. */
  location: [number, number];
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
  meta: Record<string, any>;
  /** Segment ids this task occupies while working (traffic management). */
  segments?: string[];
}

export interface FleetKPIs {
  /** Agricultural yield collected by harvest tasks. */
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
  perRobot: Record<string, { busyTime: number; distance: number; tasks: number; battery: number; status: string }>;
}

export interface FleetEvents extends Record<string, unknown> {
  taskUpdated: { task: FleetTask };
  robotUpdated: { robot: MobileRobot };
  log: { t: number; text: string; level: 'info' | 'warn' | 'error' };
}

/** Fleet item: groups robots and holds the manager configuration. */
export class FleetItem extends Item {
  robotIds: string[] = [];
  mapId: string | null = null;
  allocation: 'nearest' | 'auction' | 'round_robin' = 'auction';
  chargingZoneIds: string[] = [];
  /** Max robots per shared segment (row). */
  segmentCapacity = 1;
  tasks: FleetTask[] = [];
  constructor(name = 'Fleet', id?: string) {
    super(ItemType.FLEET, name, id);
  }
  protected override serializeExtra() {
    return { robotIds: this.robotIds, mapId: this.mapId, allocation: this.allocation, chargingZoneIds: this.chargingZoneIds, segmentCapacity: this.segmentCapacity, tasks: this.tasks };
  }
  override deserializeExtra(d: SerializedItem) {
    this.robotIds = (d.robotIds as string[]) ?? []; this.mapId = (d.mapId as string | null) ?? null; this.allocation = (d.allocation as any) ?? 'auction';
    this.chargingZoneIds = (d.chargingZoneIds as string[]) ?? []; this.segmentCapacity = (d.segmentCapacity as number) ?? 1; this.tasks = (d.tasks as FleetTask[]) ?? [];
  }
}
registerItemType(ItemType.FLEET, (n, id) => new FleetItem(n, id));

let taskCounter = 0;

export class FleetManager {
  readonly events = new EventBus<FleetEvents>();
  time = 0;
  private busy = new Map<string, number>();
  private distance0 = new Map<string, number>();
  private energy0 = new Map<string, number>();
  private waits: number[] = [];
  /** Segment reservations: segment id -> robot ids. */
  private reservations = new Map<string, Set<string>>();
  private log: Array<{ t: number; text: string; level: 'info' | 'warn' | 'error' }> = [];
  /** Yield counters (updated by finishTask for harvest tasks). */
  yield = { fruit: 0, kg: 0, areaM2: 0 };
  /** Inter-robot safety: robots brake when another robot is within this distance ahead (mm). */
  safetyDistance = 2500;
  /** Hook to mark fruit as picked when a harvest task finishes (installed by the agri module). */
  onHarvestDone: ((task: FleetTask) => { fruit: number; kg: number }) | null = null;
  private pendingPathRequests = new Map<string, { task: FleetTask; phase: 'travel' | 'work' | 'charge' }>();

  constructor(readonly station: Station, readonly fleet: FleetItem) {
    for (const r of this.robots()) {
      this.distance0.set(r.id, r.state.odometer);
      this.energy0.set(r.id, r.battery.levelWh);
    }
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

  addTask(partial: Partial<FleetTask> & { type: TaskType; location: [number, number] }): FleetTask {
    const task: FleetTask = {
      id: partial.id ?? `task_${++taskCounter}`,
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

  cancelTask(id: string): void {
    const t = this.fleet.tasks.find((x) => x.id === id);
    if (!t) return;
    if (t.robotId) {
      const r = this.station.findById(t.robotId) as MobileRobot | null;
      if (r) { r.state.path = null; r.state.status = 'idle'; r.state.taskId = null; }
      this.releaseSegments(t.robotId);
    }
    t.status = 'cancelled';
    this.events.emit('taskUpdated', { task: t });
  }

  /** Plan a path with the map (or straight line if no map). */
  planTo(robot: MobileRobot, goal: [number, number]): number[][] | null {
    const map = this.map();
    if (!map) return [[robot.state.x, robot.state.y], goal];
    const res = planPath(map, [robot.state.x, robot.state.y], goal);
    return res.ok ? res.path : null;
  }

  /** Cost for a robot to execute a task: travel time + queue + battery penalty. */
  private cost(robot: MobileRobot, task: FleetTask): number {
    if (task.requiredCapabilities.some((c) => !robot.capabilities.includes(c))) return Infinity;
    if (robot.state.status === 'charging' || robot.state.status === 'error') return Infinity;
    if (robot.state.taskId) return Infinity; // one task at a time
    const straight = Math.hypot(task.location[0] - robot.state.x, task.location[1] - robot.state.y);
    const travel = estimateTravelTime(robot, [[robot.state.x, robot.state.y], task.location]);
    const work = task.workPath ? pathLength(task.workPath) / (task.workSpeed ?? robot.kin.maxSpeed * 0.3) : task.duration ?? 0;
    const energyNeeded = ((robot.battery.drivePerMps + robot.battery.idleW) * (travel + work)) / 3600;
    if (energyNeeded > robot.battery.levelWh * 0.9) return Infinity;
    const batteryPenalty = (1 - robot.batteryLevel()) * 60;
    return travel + batteryPenalty + straight * 1e-6;
  }

  /** Allocate pending tasks to idle robots. */
  allocate(): void {
    const robots = this.robots();
    const pending = this.fleet.tasks.filter((t) => t.status === 'pending').sort((a, b) => b.priority - a.priority || a.createdAt - b.createdAt);
    for (const task of pending) {
      let best: MobileRobot | null = null, bestCost = Infinity;
      if (this.fleet.allocation === 'round_robin') {
        best = robots.find((r) => this.cost(r, task) < Infinity) ?? null;
      } else {
        for (const r of robots) {
          const c = this.cost(r, task);
          if (c < bestCost) { bestCost = c; best = r; }
        }
      }
      if (!best) continue;
      const path = this.planTo(best, task.location);
      if (!path) { this.logMsg(`No path for ${best.name} to task ${task.id}`, 'warn'); continue; }
      task.robotId = best.id;
      task.status = 'travelling';
      task.startedAt = this.time;
      this.waits.push(this.time - task.createdAt);
      best.state.taskId = task.id;
      followPath(best, path);
      this.logMsg(`Task ${task.id} -> ${best.name} (ETA ${estimateTravelTime(best, path).toFixed(0)} s)`);
      this.events.emit('taskUpdated', { task });
    }
  }

  private reserveSegments(robotId: string, segments: string[]): boolean {
    for (const s of segments) {
      const set = this.reservations.get(s);
      if (set && !set.has(robotId) && set.size >= this.fleet.segmentCapacity) return false;
    }
    for (const s of segments) {
      if (!this.reservations.has(s)) this.reservations.set(s, new Set());
      this.reservations.get(s)!.add(robotId);
    }
    return true;
  }
  private releaseSegments(robotId: string): void {
    for (const set of this.reservations.values()) set.delete(robotId);
  }

  /** Charging policy: send low-battery idle robots to the nearest charging zone. */
  private manageCharging(): void {
    const zones = this.station.itemsOfType<ZoneItem>(ItemType.ZONE).filter((z) => z.kind === 'charging');
    for (const r of this.robots()) {
      if (r.state.status === 'charging') {
        if (r.batteryLevel() >= 0.95) { r.state.status = 'idle'; this.logMsg(`${r.name} charged`); }
        continue;
      }
      if (r.state.taskId || r.state.status !== 'idle') continue;
      if (r.batteryLevel() > r.battery.lowThreshold) continue;
      const home = r.home ?? (zones.length ? { ...(() => { const [x, y] = zones[0].centroid(); const abs = zones[0].poseAbs(); return { x: abs[12] + x, y: abs[13] + y }; })(), theta: 0 } : null);
      if (!home) continue;
      const path = this.planTo(r, [home.x, home.y]);
      if (!path) continue;
      const task = this.addTask({ type: 'charge', location: [home.x, home.y], priority: 100, robotId: r.id, status: 'travelling', startedAt: this.time });
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
    for (const r of this.robots()) {
      const task = r.state.taskId ? this.fleet.tasks.find((t) => t.id === r.state.taskId) : null;
      const wasMoving = r.state.path !== null;
      // Traffic: before entering a work path, reserve its segments
      if (task && task.status === 'travelling' && !wasMoving) {
        // arrived at location
        if (task.type === 'charge') {
          task.status = 'done'; task.finishedAt = this.time; r.state.taskId = null; r.state.status = 'charging';
          this.events.emit('taskUpdated', { task });
          continue;
        }
        if (task.workPath && task.workPath.length > 1) {
          if (task.segments && !this.reserveSegments(r.id, task.segments)) {
            r.state.status = 'waiting';
            continue;
          }
          task.status = 'working';
          followPath(r, task.workPath);
          r.state.status = 'working';
          this.events.emit('taskUpdated', { task });
        } else {
          task.status = 'working';
          task.meta._workLeft = task.duration ?? 5;
          r.state.status = 'working';
          this.events.emit('taskUpdated', { task });
        }
      }
      if (task && task.status === 'working' && !task.workPath) {
        task.meta._workLeft = (task.meta._workLeft ?? 0) - dt;
        r.battery.levelWh = Math.max(0, r.battery.levelWh - (r.battery.idleW * 3 * dt) / 3600);
        if (task.meta._workLeft <= 0) this.finishTask(r, task);
        continue;
      }
      // simple inter-robot safety: slow down / stop when another robot is close ahead.
      // Stationary robots only block when very close; after a few seconds blocked the robot creeps to avoid deadlocks.
      let speedCap: number | undefined;
      if (r.state.path) {
        const th = (r.state.theta * Math.PI) / 180;
        for (const o of this.robots()) {
          if (o === r) continue;
          const dx = o.state.x - r.state.x, dy = o.state.y - r.state.y;
          const ahead = dx * Math.cos(th) + dy * Math.sin(th);
          const lateral = Math.abs(-dx * Math.sin(th) + dy * Math.cos(th));
          const dist = Math.hypot(dx, dy);
          const otherMoving = Math.abs(o.state.v) > 50;
          const range = otherMoving ? this.safetyDistance : Math.min(this.safetyDistance, 1200);
          if (ahead > 0 && lateral < (r.kin.footprint[1] + o.kin.footprint[1]) / 2 && dist < range) {
            const cap = Math.max(0, (dist - range * 0.4) / (range * 0.6)) * r.kin.maxSpeed;
            speedCap = Math.min(speedCap ?? Infinity, cap);
          }
        }
      }
      const desired = task?.status === 'working' ? task.workSpeed ?? r.kin.maxSpeed * 0.3 : undefined;
      const blocked = (r.state as any)._blocked ?? 0;
      if (speedCap !== undefined && speedCap < 1 && blocked < 4) {
        (r.state as any)._blocked = blocked + dt;
        integrateStop(r, dt);
        if (r.state.status !== 'waiting') r.state.status = 'waiting';
        this.busy.set(r.id, (this.busy.get(r.id) ?? 0) + dt);
        this.events.emit('robotUpdated', { robot: r });
        continue;
      }
      if (speedCap !== undefined && speedCap < 1) speedCap = r.kin.maxSpeed * 0.2; // creep
      if (speedCap === undefined) (r.state as any)._blocked = 0;
      if (r.state.status === 'waiting' && r.state.path && (!task || !task.segments || this.reserveSegments(r.id, task.segments))) r.state.status = task?.status === 'working' ? 'working' : 'moving';
      // navigation-stack simulation: the controller steers from the localization estimate, not the ground truth
      const nav = getNavStack(r);
      const rt = nav?.simulate ? ((r as any)._nav as NavRuntime | undefined) : undefined;
      const pose = rt && !rt.estimator.state.lost ? { x: rt.estimator.state.x, y: rt.estimator.state.y, theta: rt.estimator.state.theta } : rt?.estimator.state.lost ? { x: r.state.x + 1e6, y: r.state.y, theta: r.state.theta } : undefined;
      if (rt?.estimator.state.lost) { integrateStop(r, dt); if (r.state.status !== 'waiting') r.state.status = 'waiting'; stepNavRuntime(r, dt, this.map(), this.zones()); continue; }
      const moving = stepMobile(r, dt, { speed: speedCap !== undefined ? Math.min(speedCap, desired ?? r.kin.maxSpeed) : desired, pose });
      if (nav?.simulate) stepNavRuntime(r, dt, this.map(), this.zones());
      if (task && task.status === 'working' && task.workPath && !moving && r.state.path === null) this.finishTask(r, task);
      if (r.state.status === 'moving' || r.state.status === 'working') this.busy.set(r.id, (this.busy.get(r.id) ?? 0) + dt);
      if (r.state.status === 'waiting' && task && task.segments && this.reserveSegments(r.id, task.segments)) {
        task.status = 'working';
        followPath(r, task.workPath!);
        r.state.status = 'working';
      }
      this.events.emit('robotUpdated', { robot: r });
    }
  }

  private finishTask(r: MobileRobot, task: FleetTask): void {
    task.status = 'done';
    task.finishedAt = this.time;
    if (task.workPath && task.workPath.length > 1) this.yield.areaM2 += (pathLength(task.workPath) / 1000) * ((task.meta?.swathM as number) ?? 2);
    if (task.type === 'harvest') {
      const y = this.onHarvestDone?.(task) ?? { fruit: task.meta?.fruit ?? 0, kg: ((task.meta?.fruit ?? 0) * (task.meta?.fruitKg ?? 0.18)) };
      this.yield.fruit += y.fruit;
      this.yield.kg += y.kg;
      task.meta.harvestedKg = y.kg;
    }
    r.state.taskId = null;
    r.state.status = 'idle';
    this.releaseSegments(r.id);
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
      perRobot[r.id] = { busyTime: b, distance: d, tasks: done.filter((t) => t.robotId === r.id).length, battery: r.batteryLevel(), status: r.state.status };
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
      perRobot,
    };
  }

  private logMsg(text: string, level: 'info' | 'warn' | 'error' = 'info') {
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
