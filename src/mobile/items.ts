import { Item, ItemType, SerializedItem, DeserializeContext, registerItemType, GeometryRef } from '../core/items/item';
import { Mat4, transl, rotz, mul, DEG, RAD } from '../core/math/pose';

export type DriveType = 'differential' | 'ackermann' | 'omni' | 'tracked' | 'legged';

export interface MobileKinematics {
  drive: DriveType;
  /** Wheel base / track (mm). */
  wheelBase: number;
  track: number;
  wheelRadius: number;
  /** Max steering angle for Ackermann (deg). */
  maxSteer: number;
  maxSpeed: number; // mm/s
  maxReverseSpeed: number;
  maxAccel: number; // mm/s²
  maxYawRate: number; // deg/s
  /** Footprint (mm): length x width x height. */
  footprint: [number, number, number];
  /** Minimum turning radius (mm) — 0 for differential/omni. */
  minTurnRadius: number;
}

export interface BatteryModel {
  capacityWh: number;
  levelWh: number;
  /** Idle draw (W) and traction draw per m/s (W per m/s). */
  idleW: number;
  drivePerMps: number;
  /** Charging power (W). */
  chargeW: number;
  /** Threshold (0-1) below which the fleet manager sends the robot to charge. */
  lowThreshold: number;
}

export interface MobileState {
  x: number; // mm, in parent frame
  y: number;
  theta: number; // deg
  v: number; // mm/s
  omega: number; // deg/s
  steer: number; // deg (ackermann)
  status: 'idle' | 'moving' | 'working' | 'charging' | 'error' | 'waiting';
  odometer: number; // mm
  /** Current path being followed (points in parent frame) and progress index. */
  path: number[][] | null;
  pathIndex: number;
  taskId: string | null;
}

/** Mobile robot (AMR / AGV / tractor / harvesting platform). Arms and sensors are children. */
export class MobileRobot extends Item {
  kin: MobileKinematics = { drive: 'differential', wheelBase: 800, track: 700, wheelRadius: 150, maxSteer: 30, maxSpeed: 1500, maxReverseSpeed: 500, maxAccel: 800, maxYawRate: 90, footprint: [1200, 800, 600], minTurnRadius: 0 };
  battery: BatteryModel = { capacityWh: 2000, levelWh: 2000, idleW: 60, drivePerMps: 400, chargeW: 1500, lowThreshold: 0.25 };
  state: MobileState = { x: 0, y: 0, theta: 0, v: 0, omega: 0, steer: 0, status: 'idle', odometer: 0, path: null, pathIndex: 0, taskId: null };
  geometry: GeometryRef[] = [];
  /** Sensors: lidar/camera/gnss. */
  sensors: Array<{ kind: 'lidar2d' | 'lidar3d' | 'camera' | 'gnss' | 'imu'; pose: number[]; range?: number; fov?: number }> = [];
  /** Capabilities used by the fleet allocator (e.g. 'harvest', 'spray', 'transport', 'mow'). */
  capabilities: string[] = ['transport'];
  payloadKg = 100;
  /** ROS 2 namespace for the bridge. */
  rosNamespace = '';
  fleetId: string | null = null;
  /** Home / charging position. */
  home: { x: number; y: number; theta: number } | null = null;

  constructor(name = 'Mobile robot', id?: string) {
    super(ItemType.MOBILE_ROBOT, name, id);
    this.color = '#2b8a3e';
  }

  /** Sync the item pose from the 2D state (pose on the ground plane, z from current pose). */
  syncPoseFromState(): void {
    const z = this._pose[14];
    this._pose = mul(transl(this.state.x, this.state.y, z), rotz(this.state.theta * DEG));
    this.notify('pose');
  }

  /** Set 2D state from the item pose. */
  syncStateFromPose(): void {
    this.state.x = this._pose[12];
    this.state.y = this._pose[13];
    this.state.theta = Math.atan2(this._pose[1], this._pose[0]) * RAD;
  }

  override setPose(m: Mat4): this {
    super.setPose(m);
    this.syncStateFromPose();
    return this;
  }

  setPose2D(x: number, y: number, thetaDeg: number): void {
    this.state.x = x; this.state.y = y; this.state.theta = thetaDeg;
    this.syncPoseFromState();
  }

  batteryLevel(): number {
    return this.battery.capacityWh > 0 ? this.battery.levelWh / this.battery.capacityWh : 1;
  }

  protected override serializeExtra() {
    return { kin: this.kin, battery: this.battery, state: this.state, geometry: this.geometry, sensors: this.sensors, capabilities: this.capabilities, payloadKg: this.payloadKg, rosNamespace: this.rosNamespace, fleetId: this.fleetId, home: this.home };
  }
  override deserializeExtra(d: SerializedItem, _ctx: DeserializeContext) {
    this.kin = { ...this.kin, ...((d.kin as any) ?? {}) };
    this.battery = { ...this.battery, ...((d.battery as any) ?? {}) };
    this.state = { ...this.state, ...((d.state as any) ?? {}), path: null, pathIndex: 0 };
    this.geometry = (d.geometry as GeometryRef[]) ?? [];
    this.sensors = (d.sensors as any) ?? [];
    this.capabilities = (d.capabilities as string[]) ?? ['transport'];
    this.payloadKg = (d.payloadKg as number) ?? 100;
    this.rosNamespace = (d.rosNamespace as string) ?? '';
    this.fleetId = (d.fleetId as string | null) ?? null;
    this.home = (d.home as any) ?? null;
    this.syncStateFromPose();
  }
}

/** 2D occupancy grid map (for navigation). Cells: 0 free, 100 occupied, 255 unknown. */
export class MapItem extends Item {
  width = 100;
  height = 100;
  resolution = 100; // mm per cell
  /** Origin of cell (0,0) relative to this item (mm). */
  originX = 0;
  originY = 0;
  cells: Uint8Array = new Uint8Array(100 * 100);
  /** Inflation radius (mm) used for planning. */
  inflation = 400;

  constructor(name = 'Map', id?: string) {
    super(ItemType.MAP, name, id);
  }

  resize(width: number, height: number, resolution: number, originX = 0, originY = 0): void {
    this.width = width; this.height = height; this.resolution = resolution; this.originX = originX; this.originY = originY;
    this.cells = new Uint8Array(width * height);
    this.notify('map');
  }

  worldToCell(x: number, y: number): [number, number] {
    return [Math.floor((x - this.originX) / this.resolution), Math.floor((y - this.originY) / this.resolution)];
  }
  cellToWorld(cx: number, cy: number): [number, number] {
    return [this.originX + (cx + 0.5) * this.resolution, this.originY + (cy + 0.5) * this.resolution];
  }
  get(cx: number, cy: number): number {
    if (cx < 0 || cy < 0 || cx >= this.width || cy >= this.height) return 100;
    return this.cells[cy * this.width + cx];
  }
  set(cx: number, cy: number, v: number): void {
    if (cx < 0 || cy < 0 || cx >= this.width || cy >= this.height) return;
    this.cells[cy * this.width + cx] = v;
  }
  isFree(x: number, y: number): boolean {
    const [cx, cy] = this.worldToCell(x, y);
    return this.get(cx, cy) < 50;
  }
  /** Mark a world-space rectangle as occupied (mm). */
  fillRect(x0: number, y0: number, x1: number, y1: number, v = 100): void {
    const [ax, ay] = this.worldToCell(Math.min(x0, x1), Math.min(y0, y1));
    const [bx, by] = this.worldToCell(Math.max(x0, x1), Math.max(y0, y1));
    for (let cy = ay; cy <= by; cy++) for (let cx = ax; cx <= bx; cx++) this.set(cx, cy, v);
    this.notify('map');
  }
  fillCircle(x: number, y: number, r: number, v = 100): void {
    const [cx0, cy0] = this.worldToCell(x, y);
    const rc = Math.ceil(r / this.resolution);
    for (let cy = cy0 - rc; cy <= cy0 + rc; cy++) for (let cx = cx0 - rc; cx <= cx0 + rc; cx++) {
      const [wx, wy] = this.cellToWorld(cx, cy);
      if (Math.hypot(wx - x, wy - y) <= r) this.set(cx, cy, v);
    }
    this.notify('map');
  }
  private _inflCache: { key: string; grid: Uint8Array } | null = null;
  private _version = 0;
  override notify(what: string): void {
    if (what === 'map') { this._version++; this._inflCache = null; }
    super.notify(what);
  }

  /** Return an inflated copy of the grid for planning (cached until the map changes). */
  inflated(radiusMm = this.inflation): Uint8Array {
    const rc = Math.ceil(radiusMm / this.resolution);
    const key = `${rc}:${this._version}:${this.cells.length}`;
    if (this._inflCache && this._inflCache.key === key) return this._inflCache.grid;
    const out = new Uint8Array(this.cells);
    if (rc <= 0) { this._inflCache = { key, grid: out }; return out; }
    for (let cy = 0; cy < this.height; cy++) for (let cx = 0; cx < this.width; cx++) {
      if (this.cells[cy * this.width + cx] < 50) continue;
      for (let dy = -rc; dy <= rc; dy++) for (let dx = -rc; dx <= rc; dx++) {
        if (dx * dx + dy * dy > rc * rc) continue;
        const x = cx + dx, y = cy + dy;
        if (x >= 0 && y >= 0 && x < this.width && y < this.height && out[y * this.width + x] < 50) out[y * this.width + x] = 99;
      }
    }
    this._inflCache = { key, grid: out };
    return out;
  }

  protected override serializeExtra() {
    return { width: this.width, height: this.height, resolution: this.resolution, originX: this.originX, originY: this.originY, inflation: this.inflation, cells: rle(this.cells) };
  }
  override deserializeExtra(d: SerializedItem) {
    this.width = (d.width as number) ?? 100; this.height = (d.height as number) ?? 100; this.resolution = (d.resolution as number) ?? 100;
    this.originX = (d.originX as number) ?? 0; this.originY = (d.originY as number) ?? 0; this.inflation = (d.inflation as number) ?? 400;
    this.cells = d.cells ? unrle(d.cells as number[], this.width * this.height) : new Uint8Array(this.width * this.height);
  }
}

function rle(a: Uint8Array): number[] {
  const out: number[] = [];
  let i = 0;
  while (i < a.length) {
    const v = a[i];
    let n = 1;
    while (i + n < a.length && a[i + n] === v && n < 65535) n++;
    out.push(v, n);
    i += n;
  }
  return out;
}
function unrle(r: number[], size: number): Uint8Array {
  const out = new Uint8Array(size);
  let i = 0;
  for (let k = 0; k + 1 < r.length; k += 2) {
    out.fill(r[k], i, i + r[k + 1]);
    i += r[k + 1];
  }
  return out;
}

export type ZoneKind = 'work' | 'nogo' | 'charging' | 'loading' | 'unloading' | 'parking' | 'speed_limit' | 'headland' | 'gnss_denied';

/** Polygonal zone on the ground plane (points relative to this item, mm). */
export class ZoneItem extends Item {
  kind: ZoneKind = 'work';
  polygon: number[][] = [];
  speedLimit = 0;
  capacity = 1;
  constructor(name = 'Zone', id?: string) {
    super(ItemType.ZONE, name, id);
  }
  contains(x: number, y: number): boolean {
    const p = this.polygon;
    let inside = false;
    for (let i = 0, j = p.length - 1; i < p.length; j = i++) {
      const xi = p[i][0], yi = p[i][1], xj = p[j][0], yj = p[j][1];
      if (yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) inside = !inside;
    }
    return inside;
  }
  centroid(): [number, number] {
    if (!this.polygon.length) return [0, 0];
    let sx = 0, sy = 0;
    for (const p of this.polygon) { sx += p[0]; sy += p[1]; }
    return [sx / this.polygon.length, sy / this.polygon.length];
  }
  protected override serializeExtra() {
    return { kind: this.kind, polygon: this.polygon, speedLimit: this.speedLimit, capacity: this.capacity };
  }
  override deserializeExtra(d: SerializedItem) {
    this.kind = (d.kind as ZoneKind) ?? 'work'; this.polygon = (d.polygon as number[][]) ?? []; this.speedLimit = (d.speedLimit as number) ?? 0; this.capacity = (d.capacity as number) ?? 1;
  }
}

registerItemType(ItemType.MOBILE_ROBOT, (n, id) => new MobileRobot(n, id));
registerItemType(ItemType.MAP, (n, id) => new MapItem(n, id));
registerItemType(ItemType.ZONE, (n, id) => new ZoneItem(n, id));
