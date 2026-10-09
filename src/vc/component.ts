/**
 * Visual Components-style behaviour components: signals, feeders, conveyors, processes, sinks,
 * buffers and robot pick&place transfers. Products are SceneObjects created at runtime.
 */
import { Item, ItemType, SceneObject, SerializedItem, DeserializeContext, registerItemType, Station, GeometryRef } from '../core/items/item';
import { Mat4, transl, multiply, identity, rotz, DEG, invert } from '../core/math/pose';
import { EventBus } from '../core/events';

export type BehaviourType = 'feeder' | 'conveyor' | 'process' | 'sink' | 'buffer' | 'signal' | 'transfer' | 'sensor' | 'human';

export interface BehaviourBase {
  type: BehaviourType;
  enabled: boolean;
}
export interface FeederBehaviour extends BehaviourBase {
  type: 'feeder';
  interval: number; // s
  productTemplate: { name: string; geometry: GeometryRef; massKg?: number };
  limit: number; // -1 infinite
  /** Output: the connected conveyor/process component id. */
  next: string | null;
  created: number;
}
export interface ConveyorBehaviour extends BehaviourBase {
  type: 'conveyor';
  /** Path points relative to the component (mm). */
  path: number[][];
  speed: number; // mm/s
  spacing: number; // min gap between products (mm)
  next: string | null;
  /** Stop signal name (if set and true, conveyor halts). */
  stopSignal?: string;
}
export interface ProcessBehaviour extends BehaviourBase {
  type: 'process';
  cycleTime: number; // s
  capacity: number;
  next: string | null;
  /** Signal raised while processing. */
  busySignal?: string;
  /** Failure model: mean time between failures / mean time to repair (s). */
  mtbf?: number;
  mttr?: number;
}
export interface SinkBehaviour extends BehaviourBase {
  type: 'sink';
  count: number;
}
export interface BufferBehaviour extends BehaviourBase {
  type: 'buffer';
  capacity: number;
  next: string | null;
  /** Grid arrangement for stacked products (columns, rows, layers, pitch). */
  grid?: { cols: number; rows: number; layers: number; pitch: [number, number, number] };
}
export interface TransferBehaviour extends BehaviourBase {
  type: 'transfer';
  /** Robot id that performs the pick & place. */
  robotId: string | null;
  /** From component (input buffer) to next. */
  from: string | null;
  next: string | null;
  cycleTime: number;
}
export interface SensorBehaviour extends BehaviourBase {
  type: 'sensor';
  /** Raises signal when a product is within range (mm). */
  signal: string;
  range: number;
}
export interface HumanBehaviour extends BehaviourBase {
  type: 'human';
  walkSpeed: number;
  taskTime: number;
  next: string | null;
}
export type Behaviour = FeederBehaviour | ConveyorBehaviour | ProcessBehaviour | SinkBehaviour | BufferBehaviour | TransferBehaviour | SensorBehaviour | HumanBehaviour;

export interface ProductState {
  id: string;
  /** Current holder component id. */
  holder: string | null;
  /** Progress along conveyor (mm) or time in process (s). */
  progress: number;
  createdAt: number;
  enteredAt: number;
}

/** A component with a behaviour and optional geometry, like a Visual Components component. */
export class Component extends Item {
  behaviour: Behaviour;
  geometry: GeometryRef[] = [];
  /** Product ids currently held. */
  products: string[] = [];
  /** Runtime stats. */
  stats = { entered: 0, exited: 0, busyTime: 0, blockedTime: 0, idleTime: 0, failures: 0, downTime: 0 };
  /** Runtime state. */
  runtime: Record<string, any> = {};

  constructor(name: string, behaviour: Behaviour, id?: string) {
    super(ItemType.COMPONENT, name, id);
    this.behaviour = behaviour;
  }

  protected override serializeExtra() {
    return { behaviour: this.behaviour, geometry: this.geometry };
  }
  override deserializeExtra(d: SerializedItem, _ctx: DeserializeContext) {
    this.behaviour = d.behaviour as Behaviour;
    this.geometry = (d.geometry as GeometryRef[]) ?? [];
  }

  /** World pose at a distance along the conveyor path. */
  conveyorPoseAt(dist: number): Mat4 {
    const b = this.behaviour as ConveyorBehaviour;
    const pts = b.path;
    if (pts.length < 2) return this.poseAbs();
    let rem = dist;
    for (let i = 1; i < pts.length; i++) {
      const a = pts[i - 1], c = pts[i];
      const l = Math.hypot(c[0] - a[0], c[1] - a[1], (c[2] ?? 0) - (a[2] ?? 0));
      if (rem <= l || i === pts.length - 1) {
        const t = l > 0 ? Math.min(1, rem / l) : 0;
        const p = [a[0] + (c[0] - a[0]) * t, a[1] + (c[1] - a[1]) * t, (a[2] ?? 0) + ((c[2] ?? 0) - (a[2] ?? 0)) * t];
        const yaw = Math.atan2(c[1] - a[1], c[0] - a[0]);
        return multiply(this.poseAbs(), multiply(transl(p[0], p[1], p[2]), rotz(yaw)));
      }
      rem -= l;
    }
    return this.poseAbs();
  }

  conveyorLength(): number {
    const b = this.behaviour as ConveyorBehaviour;
    let l = 0;
    for (let i = 1; i < b.path.length; i++) l += Math.hypot(b.path[i][0] - b.path[i - 1][0], b.path[i][1] - b.path[i - 1][1], (b.path[i][2] ?? 0) - (b.path[i - 1][2] ?? 0));
    return l;
  }
}
registerItemType(ItemType.COMPONENT, (n, id) => new Component(n, { type: 'sink', enabled: true, count: 0 }, id));

export interface ProcessEvents extends Record<string, unknown> {
  productCreated: { product: SceneObject };
  productConsumed: { product: SceneObject; by: Component };
  signal: { name: string; value: boolean | number };
}

/** Discrete-time process flow simulator for Component behaviours. */
export class ProcessSimulator {
  readonly events = new EventBus<ProcessEvents>();
  time = 0;
  signals = new Map<string, boolean | number>();
  products = new Map<string, ProductState>();
  private productsFolder: Item | null = null;
  private rng = mulberry32(42);

  constructor(readonly station: Station) {}

  components(): Component[] {
    return this.station.itemsOfType<Component>(ItemType.COMPONENT).filter((c) => c.behaviour.enabled);
  }

  reset(): void {
    this.time = 0;
    for (const [id] of this.products) this.station.findById(id)?.delete();
    this.products.clear();
    for (const c of this.components()) {
      c.products = [];
      c.stats = { entered: 0, exited: 0, busyTime: 0, blockedTime: 0, idleTime: 0, failures: 0, downTime: 0 };
      c.runtime = {};
      if (c.behaviour.type === 'feeder') c.behaviour.created = 0;
      if (c.behaviour.type === 'sink') c.behaviour.count = 0;
    }
  }

  setSignal(name: string, value: boolean | number): void {
    this.signals.set(name, value);
    this.events.emit('signal', { name, value });
  }

  private folder(): Item {
    if (!this.productsFolder || !this.productsFolder.parent) {
      this.productsFolder = this.station.find('Products', ItemType.FOLDER) ?? this.station.addChild(new Item(ItemType.FOLDER, 'Products'));
    }
    return this.productsFolder;
  }

  private canAccept(c: Component): boolean {
    const b = c.behaviour;
    switch (b.type) {
      case 'conveyor': {
        // free if last product has moved more than spacing from the start
        const last = c.products[c.products.length - 1];
        if (!last) return true;
        return (this.products.get(last)?.progress ?? 0) >= b.spacing;
      }
      case 'process': return c.products.length < b.capacity && !c.runtime.down;
      case 'buffer': return c.products.length < b.capacity;
      case 'sink': return true;
      case 'transfer': return c.products.length < 1;
      case 'human': return c.products.length < 1;
      default: return false;
    }
  }

  private nextOf(c: Component): Component | null {
    const b = c.behaviour as any;
    if (!b.next) return null;
    const n = this.station.findById(b.next);
    return n instanceof Component ? n : null;
  }

  private transfer(product: SceneObject, from: Component | null, to: Component): void {
    const ps = this.products.get(product.id)!;
    if (from) {
      from.products = from.products.filter((p) => p !== product.id);
      from.stats.exited++;
    }
    to.products.push(product.id);
    to.stats.entered++;
    ps.holder = to.id;
    ps.progress = 0;
    ps.enteredAt = this.time;
    this.placeProduct(product, to, ps);
  }

  private placeProduct(product: SceneObject, holder: Component, ps: ProductState): void {
    const b = holder.behaviour;
    if (b.type === 'conveyor') product.setPoseAbs(holder.conveyorPoseAt(ps.progress));
    else if (b.type === 'buffer' && b.grid) {
      const i = holder.products.indexOf(product.id);
      const { cols, rows, pitch } = b.grid;
      const col = i % cols, row = Math.floor(i / cols) % rows, layer = Math.floor(i / (cols * rows));
      product.setPoseAbs(multiply(holder.poseAbs(), transl(col * pitch[0], row * pitch[1], layer * pitch[2])));
    } else if (b.type === 'sink') {
      /* consumed below */
    } else product.setPoseAbs(multiply(holder.poseAbs(), transl(0, 0, 0)));
  }

  step(dt: number): void {
    this.time += dt;
    const comps = this.components();
    for (const c of comps) {
      const b = c.behaviour;
      switch (b.type) {
        case 'feeder': {
          c.runtime.acc = (c.runtime.acc ?? b.interval) + dt;
          if (c.runtime.acc >= b.interval && (b.limit < 0 || b.created < b.limit)) {
            const next = this.nextOf(c);
            if (next && this.canAccept(next)) {
              c.runtime.acc = 0;
              b.created++;
              const prod = new SceneObject(`${b.productTemplate.name} ${b.created}`);
              prod.geometry = [JSON.parse(JSON.stringify(b.productTemplate.geometry))];
              prod.mass = b.productTemplate.massKg ?? 1;
              prod.color = b.productTemplate.geometry.color;
              this.folder().addChild(prod);
              const ps: ProductState = { id: prod.id, holder: null, progress: 0, createdAt: this.time, enteredAt: this.time };
              this.products.set(prod.id, ps);
              this.transfer(prod, null, next);
              this.events.emit('productCreated', { product: prod });
            } else c.stats.blockedTime += dt;
          }
          break;
        }
        case 'conveyor': {
          const stopped = b.stopSignal ? !!this.signals.get(b.stopSignal) : false;
          const len = c.conveyorLength();
          const next = this.nextOf(c);
          // move products front to back keeping spacing
          let frontLimit = Infinity;
          const ordered = [...c.products];
          for (const pid of ordered) {
            const ps = this.products.get(pid)!;
            const prod = this.station.findById(pid) as SceneObject | null;
            if (!prod) continue;
            let target = stopped ? ps.progress : ps.progress + b.speed * dt;
            target = Math.min(target, frontLimit - b.spacing, len);
            if (target < ps.progress) target = ps.progress;
            ps.progress = target;
            if (ps.progress >= len - 1e-6) {
              if (next && this.canAccept(next)) { this.transfer(prod, c, next); continue; }
              c.stats.blockedTime += dt;
            }
            frontLimit = ps.progress;
            this.placeProduct(prod, c, ps);
          }
          if (c.products.length) c.stats.busyTime += dt; else c.stats.idleTime += dt;
          break;
        }
        case 'process': {
          // failures
          if (b.mtbf && b.mttr) {
            if (c.runtime.down) {
              c.runtime.downLeft -= dt;
              c.stats.downTime += dt;
              if (c.runtime.downLeft <= 0) c.runtime.down = false;
            } else if (this.rng() < dt / b.mtbf) {
              c.runtime.down = true;
              c.runtime.downLeft = b.mttr * (0.5 + this.rng());
              c.stats.failures++;
            }
          }
          if (b.busySignal) this.setSignal(b.busySignal, c.products.length > 0);
          if (c.runtime.down) break;
          const next = this.nextOf(c);
          for (const pid of [...c.products]) {
            const ps = this.products.get(pid)!;
            ps.progress += dt;
            if (ps.progress >= b.cycleTime) {
              const prod = this.station.findById(pid) as SceneObject | null;
              if (!prod) continue;
              if (next && this.canAccept(next)) this.transfer(prod, c, next);
              else c.stats.blockedTime += dt;
            }
          }
          if (c.products.length) c.stats.busyTime += dt; else c.stats.idleTime += dt;
          break;
        }
        case 'buffer': {
          const next = this.nextOf(c);
          if (next && c.products.length && this.canAccept(next)) {
            const pid = c.products[0];
            const prod = this.station.findById(pid) as SceneObject | null;
            if (prod) this.transfer(prod, c, next);
            for (const p of c.products) { const pr = this.station.findById(p) as SceneObject | null; if (pr) this.placeProduct(pr, c, this.products.get(p)!); }
          }
          break;
        }
        case 'transfer':
        case 'human': {
          const next = this.nextOf(c);
          const from = b.type === 'transfer' && b.from ? (this.station.findById(b.from) as Component | null) : null;
          if (c.products.length === 0 && from && from.products.length) {
            const pid = from.products[0];
            const ps = this.products.get(pid)!;
            const prod = this.station.findById(pid) as SceneObject | null;
            // only pick from conveyor end or buffers/processes
            if (prod && (from.behaviour.type !== 'conveyor' || ps.progress >= from.conveyorLength() - 1e-6)) this.transfer(prod, from, c);
          }
          for (const pid of [...c.products]) {
            const ps = this.products.get(pid)!;
            ps.progress += dt;
            const ct = b.type === 'transfer' ? b.cycleTime : b.taskTime;
            if (ps.progress >= ct && next && this.canAccept(next)) {
              const prod = this.station.findById(pid) as SceneObject | null;
              if (prod) this.transfer(prod, c, next);
            }
          }
          if (c.products.length) c.stats.busyTime += dt; else c.stats.idleTime += dt;
          break;
        }
        case 'sink': {
          for (const pid of [...c.products]) {
            const prod = this.station.findById(pid) as SceneObject | null;
            b.count++;
            c.products = c.products.filter((p) => p !== pid);
            this.products.delete(pid);
            if (prod) { this.events.emit('productConsumed', { product: prod, by: c }); prod.delete(); }
          }
          break;
        }
        case 'sensor': {
          const pos = c.poseAbs();
          let hit = false;
          for (const [pid] of this.products) {
            const p = this.station.findById(pid);
            if (!p) continue;
            const pp = p.poseAbs();
            if (Math.hypot(pp[12] - pos[12], pp[13] - pos[13], pp[14] - pos[14]) <= b.range) { hit = true; break; }
          }
          if (this.signals.get(b.signal) !== hit) this.setSignal(b.signal, hit);
          break;
        }
      }
    }
  }

  /** Throughput statistics per component. */
  statistics(): Array<{ id: string; name: string; type: BehaviourType; entered: number; exited: number; utilization: number; blocked: number; failures: number; wip: number }> {
    return this.components().map((c) => ({
      id: c.id,
      name: c.name,
      type: c.behaviour.type,
      entered: c.stats.entered,
      exited: c.behaviour.type === 'sink' ? c.behaviour.count : c.behaviour.type === 'feeder' ? c.behaviour.created : c.stats.exited,
      utilization: this.time > 0 ? c.stats.busyTime / this.time : 0,
      blocked: this.time > 0 ? c.stats.blockedTime / this.time : 0,
      failures: c.stats.failures,
      wip: c.products.length,
    }));
  }
}

function mulberry32(a: number) {
  return () => {
    a |= 0; a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Factory helpers */
export const makeConveyor = (name: string, length: number, speed = 300, width = 400): Component =>
  Object.assign(new Component(name, { type: 'conveyor', enabled: true, path: [[0, 0, 0], [length, 0, 0]], speed, spacing: 300, next: null }), {
    geometry: [{ primitive: { kind: 'box', size: [length, width, 80] }, origin: Array.from(transl(length / 2, 0, -40)), color: '#555c66' } as GeometryRef],
  });
export const makeFeeder = (name: string, interval: number, product: FeederBehaviour['productTemplate']): Component =>
  new Component(name, { type: 'feeder', enabled: true, interval, productTemplate: product, limit: -1, next: null, created: 0 });
export const makeProcess = (name: string, cycleTime: number, capacity = 1): Component =>
  Object.assign(new Component(name, { type: 'process', enabled: true, cycleTime, capacity, next: null }), {
    geometry: [{ primitive: { kind: 'box', size: [600, 600, 900] }, origin: Array.from(transl(0, 0, 450)), color: '#4a6fa5', opacity: 0.6 } as GeometryRef],
  });
export const makeSink = (name: string): Component => new Component(name, { type: 'sink', enabled: true, count: 0 });
export const makeBuffer = (name: string, capacity: number, grid?: BufferBehaviour['grid']): Component => new Component(name, { type: 'buffer', enabled: true, capacity, next: null, grid });

export { identity, invert, DEG };
