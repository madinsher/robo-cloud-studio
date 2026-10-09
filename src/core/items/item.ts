import { Mat4, identity, clone, multiply, invert, fromArray } from '../math/pose';
import { uid } from '../units';
import { EventBus } from '../events';

/** Item types. Numeric values mirror RoboDK's ITEM_TYPE_* constants where they exist. */
export enum ItemType {
  ANY = -1,
  STATION = 1,
  ROBOT = 2,
  FRAME = 3,
  TOOL = 4,
  OBJECT = 5,
  TARGET = 6,
  CURVE = 7,
  PROGRAM = 8,
  INSTRUCTION = 9,
  PROGRAM_PYTHON = 10,
  MACHINING = 11,
  BALLBARVALIDATION = 12,
  CALIBPROJECT = 13,
  VALID_ISO9283 = 14,
  FOLDER = 17,
  ROBOT_ARM = 18,
  CAMERA = 19,
  GENERIC = 20,
  ROBOT_AXES = 21,
  NOTES = 22,
  // --- VerticalBot Studio extensions (>= 100) ---
  MOBILE_ROBOT = 100,
  COMPONENT = 101,
  FLEET = 102,
  MAP = 103,
  FIELD = 104,
  MISSION = 105,
  SENSOR = 106,
  CONVEYOR = 107,
  CROP_ROW = 108,
  ZONE = 109,
  PATH = 110,
  /** Control-design model document (automata, Petri nets, behavior trees, specifications…). */
  CONTROL_MODEL = 111,
}

export type ParamValue = string | number | boolean | null | ParamValue[] | { [k: string]: ParamValue };

export interface SerializedItem {
  id: string;
  type: ItemType;
  name: string;
  pose: number[];
  visible: boolean;
  params: Record<string, ParamValue>;
  children: SerializedItem[];
  [extra: string]: unknown;
}

export interface ItemEvents extends Record<string, unknown> {
  changed: { item: Item; what: string };
  childAdded: { parent: Item; child: Item };
  childRemoved: { parent: Item; child: Item };
}

/**
 * Base class of everything in the station tree (RoboDK "Item").
 * Pose is relative to the parent item. Absolute pose = parent.poseAbs * pose.
 */
export class Item {
  readonly id: string;
  type: ItemType;
  name: string;
  visible = true;
  parent: Item | null = null;
  children: Item[] = [];
  /** Free-form parameters (RoboDK setParam/getParam). */
  params: Record<string, ParamValue> = {};
  /** Pose relative to parent (mm). */
  protected _pose: Mat4 = identity();
  protected _events?: EventBus<ItemEvents>;
  /** Selected in the UI. */
  selected = false;
  /** Colour hint for rendering (CSS colour). */
  color?: string;

  constructor(type: ItemType, name: string, id?: string) {
    this.type = type;
    this.name = name;
    this.id = id ?? uid(ItemType[type]?.toLowerCase() ?? 'item');
  }

  // -- Tree ---------------------------------------------------------------

  get station(): Station | null {
    let p: Item | null = this;
    while (p && !(p instanceof Station)) p = p.parent;
    return (p as Station) ?? null;
  }

  addChild<T extends Item>(child: T, index?: number): T {
    if (child.parent) child.parent.removeChild(child, false);
    child.parent = this;
    if (index === undefined || index >= this.children.length) this.children.push(child);
    else this.children.splice(Math.max(0, index), 0, child);
    this.station?.notifyTree('childAdded', { parent: this, child });
    return child;
  }

  removeChild(child: Item, notify = true): void {
    const i = this.children.indexOf(child);
    if (i >= 0) {
      this.children.splice(i, 1);
      if (notify) this.station?.notifyTree('childRemoved', { parent: this, child });
      child.parent = null;
    }
  }

  /** Remove this item from the tree (RoboDK Item.Delete). */
  delete(): void {
    this.parent?.removeChild(this);
  }

  /** Reparent keeping the absolute pose (RoboDK setParentStatic). */
  setParentStatic(newParent: Item): void {
    const abs = this.poseAbs();
    newParent.addChild(this);
    this.setPoseAbs(abs);
  }

  /** Reparent keeping the relative pose (RoboDK setParent). */
  setParent(newParent: Item): void {
    newParent.addChild(this);
  }

  *walk(): Generator<Item> {
    yield this;
    for (const c of this.children) yield* c.walk();
  }

  find(name: string, type: ItemType = ItemType.ANY): Item | null {
    for (const it of this.walk()) if (it !== this && it.name === name && (type === ItemType.ANY || it.type === type)) return it;
    return null;
  }

  findById(id: string): Item | null {
    for (const it of this.walk()) if (it.id === id) return it;
    return null;
  }

  childrenOfType<T extends Item>(type: ItemType, recursive = true): T[] {
    const out: T[] = [];
    const src = recursive ? [...this.walk()].slice(1) : this.children;
    for (const it of src) if (it.type === type) out.push(it as T);
    return out;
  }

  depth(): number {
    let d = 0;
    let p = this.parent;
    while (p) {
      d++;
      p = p.parent;
    }
    return d;
  }

  isAncestorOf(other: Item): boolean {
    let p = other.parent;
    while (p) {
      if (p === this) return true;
      p = p.parent;
    }
    return false;
  }

  // -- Pose --------------------------------------------------------------

  /** Pose relative to parent. Returns a copy. */
  pose(): Mat4 {
    return clone(this._pose);
  }

  setPose(m: Mat4): this {
    this._pose = clone(m);
    this.notify('pose');
    return this;
  }

  /**
   * Absolute pose w.r.t. the station. Children of a robot (tools, cameras, a robot on a rail/track)
   * are attached to the robot's flange: abs = parentAbs * flange(q) * pose.
   */
  poseAbs(): Mat4 {
    if (!this.parent || this.parent instanceof Station) return this.pose();
    return multiply(this.parentAttachAbs(), this._pose);
  }

  /** Absolute pose of the frame this item is attached to (parent pose, or parent flange for robots). */
  parentAttachAbs(): Mat4 {
    const p = this.parent as any;
    if (!p || p instanceof Station) return identity();
    if (typeof p.solveFKFlange === 'function') return multiply(p.poseAbs(), p.solveFKFlange());
    return p.poseAbs();
  }

  setPoseAbs(m: Mat4): this {
    if (!this.parent || this.parent instanceof Station) return this.setPose(m);
    return this.setPose(multiply(invert(this.parentAttachAbs()), m));
  }

  /** Pose of this item expressed in another item's frame. */
  poseWrt(other: Item): Mat4 {
    return multiply(invert(other.poseAbs()), this.poseAbs());
  }

  // -- Params ------------------------------------------------------------

  setParam(key: string, value: ParamValue): void {
    this.params[key] = value;
    this.notify(`param:${key}`);
  }

  getParam<T extends ParamValue = ParamValue>(key: string, def?: T): T | undefined {
    return (this.params[key] as T) ?? def;
  }

  setVisible(v: boolean): void {
    this.visible = v;
    this.notify('visible');
  }

  setName(n: string): void {
    this.name = n;
    this.notify('name');
  }

  notify(what: string): void {
    this.station?.notifyTree('changed', { item: this, what });
  }

  // -- Serialization ------------------------------------------------------

  serialize(): SerializedItem {
    return {
      id: this.id,
      type: this.type,
      name: this.name,
      pose: Array.from(this._pose),
      visible: this.visible,
      params: JSON.parse(JSON.stringify(this.params)),
      color: this.color,
      children: this.children.map((c) => c.serialize()),
      ...this.serializeExtra(),
    };
  }

  /** Subclasses add their own fields here. */
  protected serializeExtra(): Record<string, unknown> {
    return {};
  }

  /** Subclasses restore their own fields here. */
  deserializeExtra(_data: SerializedItem, _ctx?: DeserializeContext): void {}

  applyBase(data: SerializedItem): void {
    this.name = data.name;
    this.visible = data.visible ?? true;
    this._pose = fromArray(data.pose ?? Array.from(identity()));
    this.params = (data.params as Record<string, ParamValue>) ?? {};
    if (typeof data.color === 'string') this.color = data.color;
  }

  typeName(): string {
    return ItemType[this.type] ?? 'UNKNOWN';
  }
}

export interface DeserializeContext {
  station: Station;
  /** Resolve id references after the whole tree is built. */
  deferred: Array<() => void>;
  byId: Map<string, Item>;
}

/** Registry of item constructors by type for deserialization. */
export const itemFactories = new Map<ItemType, (name: string, id: string) => Item>();

export function registerItemType(type: ItemType, factory: (name: string, id: string) => Item): void {
  itemFactories.set(type, factory);
}

// ---------------------------------------------------------------------------

export class Frame extends Item {
  constructor(name = 'Frame', id?: string) {
    super(ItemType.FRAME, name, id);
  }
}

export class Folder extends Item {
  constructor(name = 'Folder', id?: string) {
    super(ItemType.FOLDER, name, id);
  }
}

export interface GeometryRef {
  /** Asset id in the AssetStore (mesh) */
  mesh?: string;
  primitive?: { kind: 'box'; size: [number, number, number] } | { kind: 'cylinder'; radius: number; length: number } | { kind: 'sphere'; radius: number } | { kind: 'plane'; size: [number, number] } | { kind: 'cone'; radius: number; length: number };
  origin?: number[];
  color?: string;
  scale?: [number, number, number];
  opacity?: number;
}

/** 3D object (workpiece, fixture, tree, crate...). */
export class SceneObject extends Item {
  geometry: GeometryRef[] = [];
  /** Curves/points attached (RoboDK "Object" features) — used for machining / path following. */
  curves: Array<{ name: string; points: number[][] }> = [];
  points: Array<{ name: string; point: number[] }> = [];
  /** Physical properties for VC-like simulation. */
  mass = 0;
  /** Bounding box half extents in local frame (mm), for cheap collision checks. */
  bbox?: { min: [number, number, number]; max: [number, number, number] };

  constructor(name = 'Object', id?: string) {
    super(ItemType.OBJECT, name, id);
  }

  protected override serializeExtra() {
    return { geometry: this.geometry, curves: this.curves, points: this.points, mass: this.mass, bbox: this.bbox };
  }
  override deserializeExtra(d: SerializedItem, _ctx: DeserializeContext) {
    this.geometry = (d.geometry as GeometryRef[]) ?? [];
    this.curves = (d.curves as any) ?? [];
    this.points = (d.points as any) ?? [];
    this.mass = (d.mass as number) ?? 0;
    this.bbox = d.bbox as any;
  }
}

/** Tool (end effector). Pose = TCP relative to the robot flange (RoboDK Tool.PoseTool). */
export class Tool extends SceneObject {
  /** Tool type hint: gripper, sprayer, cutter, camera... */
  toolKind = 'generic';
  /** Gripper state for simulation. */
  closed = false;
  /** Attached (grabbed) object ids. */
  attached: string[] = [];
  /** Payload (kg). */
  payload = 0;

  constructor(name = 'Tool', id?: string) {
    super(name, id);
    this.type = ItemType.TOOL;
  }

  /** TCP pose relative to flange (alias of pose()). */
  poseTool(): Mat4 {
    return this.pose();
  }

  /** Absolute pose of the flange this tool is mounted on. */
  flangeAbs(): Mat4 {
    return this.parentAttachAbs();
  }
  setPoseTool(m: Mat4): void {
    this.setPose(m);
  }

  protected override serializeExtra() {
    return { ...super.serializeExtra(), toolKind: this.toolKind, closed: this.closed, attached: this.attached, payload: this.payload };
  }
  override deserializeExtra(d: SerializedItem, ctx: DeserializeContext) {
    super.deserializeExtra(d, ctx);
    this.toolKind = (d.toolKind as string) ?? 'generic';
    this.closed = !!d.closed;
    this.attached = (d.attached as string[]) ?? [];
    this.payload = (d.payload as number) ?? 0;
  }
}

/** Target: a cartesian pose (relative to its parent frame) and/or a joint configuration. */
export class Target extends Item {
  /** Joint values recorded with the target (deg/mm). */
  joints: number[] | null = null;
  /** True: joint target (motion uses joints). False: cartesian target. */
  isJointTarget = false;
  /** Robot this target was recorded for (id). */
  robotId: string | null = null;
  /** Configuration flags (front/back, elbow, wrist) similar to RoboDK conf_RLF. */
  config?: [number, number, number];

  constructor(name = 'Target', id?: string) {
    super(ItemType.TARGET, name, id);
  }

  setAsJointTarget(): void {
    this.isJointTarget = true;
    this.notify('targetType');
  }
  setAsCartesianTarget(): void {
    this.isJointTarget = false;
    this.notify('targetType');
  }
  setJoints(q: number[]): void {
    this.joints = [...q];
    this.notify('joints');
  }

  protected override serializeExtra() {
    return { joints: this.joints, isJointTarget: this.isJointTarget, robotId: this.robotId, config: this.config };
  }
  override deserializeExtra(d: SerializedItem) {
    this.joints = (d.joints as number[] | null) ?? null;
    this.isJointTarget = !!d.isJointTarget;
    this.robotId = (d.robotId as string | null) ?? null;
    this.config = d.config as any;
  }
}

/** Camera (simulated 2D/3D camera / sensor mount). */
export class Camera extends Item {
  fov = 60;
  near = 10;
  far = 20000;
  width = 640;
  height = 480;
  kind: 'rgb' | 'depth' | 'lidar2d' | 'lidar3d' = 'rgb';
  constructor(name = 'Camera', id?: string) {
    super(ItemType.CAMERA, name, id);
  }
  protected override serializeExtra() {
    return { fov: this.fov, near: this.near, far: this.far, width: this.width, height: this.height, kind: this.kind };
  }
  override deserializeExtra(d: SerializedItem) {
    Object.assign(this, { fov: d.fov ?? 60, near: d.near ?? 10, far: d.far ?? 20000, width: d.width ?? 640, height: d.height ?? 480, kind: d.kind ?? 'rgb' });
  }
}

export class Notes extends Item {
  text = '';
  constructor(name = 'Notes', id?: string) {
    super(ItemType.NOTES, name, id);
  }
  protected override serializeExtra() {
    return { text: this.text };
  }
  override deserializeExtra(d: SerializedItem) {
    this.text = (d.text as string) ?? '';
  }
}

// ---------------------------------------------------------------------------

export interface StationEvents extends ItemEvents {
  selection: { items: Item[] };
  simTime: { t: number };
}

/** Root of the item tree. */
export class Station extends Item {
  readonly events = new EventBus<StationEvents>();
  /** Station-level settings. */
  settings: Record<string, ParamValue> = { units: 'mm', upAxis: 'z', gravity: -9.81 };
  /** File path / name this station was loaded from. */
  filePath: string | null = null;
  /** Simulation clock (s). */
  simTime = 0;
  /** Selection set. */
  selection: Item[] = [];
  /** Suppress notifications during bulk operations. */
  private quiet = 0;

  constructor(name = 'Station', id?: string) {
    super(ItemType.STATION, name, id);
  }

  override get station(): Station {
    return this;
  }

  notifyTree<K extends keyof ItemEvents>(ev: K, payload: ItemEvents[K]): void {
    if (this.quiet > 0) return;
    this.events.emit(ev, payload as any);
  }

  batch<T>(fn: () => T): T {
    this.quiet++;
    try {
      return fn();
    } finally {
      this.quiet--;
      if (this.quiet === 0) this.events.emit('changed', { item: this, what: 'batch' });
    }
  }

  setSelection(items: Item[]): void {
    for (const it of this.selection) it.selected = false;
    this.selection = items.filter((i) => i && i !== this);
    for (const it of this.selection) it.selected = true;
    this.events.emit('selection', { items: this.selection });
  }

  select(item: Item | null, additive = false): void {
    if (!item) return this.setSelection([]);
    if (additive) {
      const has = this.selection.includes(item);
      this.setSelection(has ? this.selection.filter((i) => i !== item) : [...this.selection, item]);
    } else this.setSelection([item]);
  }

  itemsOfType<T extends Item>(type: ItemType): T[] {
    return this.childrenOfType<T>(type, true);
  }

  protected override serializeExtra() {
    return { settings: this.settings, simTime: this.simTime, format: 'vbs-station', version: 1 };
  }
  override deserializeExtra(d: SerializedItem, _ctx?: DeserializeContext) {
    this.settings = (d.settings as any) ?? this.settings;
  }

  /** Build a station from serialized JSON. */
  static deserialize(data: SerializedItem): Station {
    const st = new Station(data.name, data.id);
    st.applyBase(data);
    const ctx: DeserializeContext = { station: st, deferred: [], byId: new Map([[st.id, st]]) };
    st.deserializeExtra(data, ctx);
    const build = (parent: Item, d: SerializedItem) => {
      const factory = itemFactories.get(d.type);
      const item = factory ? factory(d.name, d.id) : new Item(d.type, d.name, d.id);
      item.applyBase(d);
      parent.children.push(item);
      item.parent = parent;
      ctx.byId.set(item.id, item);
      item.deserializeExtra(d, ctx);
      for (const c of d.children ?? []) build(item, c);
    };
    for (const c of data.children ?? []) build(st, c);
    for (const fn of ctx.deferred) fn();
    return st;
  }
}

registerItemType(ItemType.FRAME, (n, id) => new Frame(n, id));
registerItemType(ItemType.FOLDER, (n, id) => new Folder(n, id));
registerItemType(ItemType.OBJECT, (n, id) => new SceneObject(n, id));
registerItemType(ItemType.TOOL, (n, id) => new Tool(n, id));
registerItemType(ItemType.TARGET, (n, id) => new Target(n, id));
registerItemType(ItemType.CAMERA, (n, id) => new Camera(n, id));
registerItemType(ItemType.NOTES, (n, id) => new Notes(n, id));
