import { Item, ItemType, SceneObject, SerializedItem, DeserializeContext, registerItemType } from '../core/items/item';
import { GeoOrigin } from './geo';

export type CropType = 'apple' | 'pear' | 'cherry' | 'citrus' | 'grape' | 'strawberry' | 'tomato' | 'cucumber' | 'blueberry' | 'olive' | 'almond' | 'kiwi' | 'generic_tree' | 'row_crop' | 'greenhouse_bench';
export type TrainingSystem = 'freestanding' | 'spindle' | 'trellis_2d' | 'v_trellis' | 'pergola' | 'hedgerow' | 'greenhouse_gutter';

export interface CropParams {
  crop: CropType;
  training: TrainingSystem;
  rowSpacing: number; // mm
  plantSpacing: number; // mm
  rowHeading: number; // deg
  canopyHeight: number; // mm
  canopyWidth: number;
  canopyBase: number; // mm above ground where the canopy starts
  trunkHeight: number;
  /** Fruit per plant (mean) and fruit size (mm). */
  fruitPerPlant: number;
  fruitDiameter: number;
  /** Fraction of ripe fruit (0-1) for harvesting scenarios. */
  ripeFraction: number;
  headland: number; // mm
}

export const CROP_PRESETS: Record<CropType, Partial<CropParams>> = {
  apple: { training: 'spindle', rowSpacing: 3500, plantSpacing: 1000, canopyHeight: 3200, canopyWidth: 1200, canopyBase: 600, trunkHeight: 600, fruitPerPlant: 80, fruitDiameter: 75, ripeFraction: 0.6 },
  pear: { training: 'trellis_2d', rowSpacing: 3500, plantSpacing: 1200, canopyHeight: 3000, canopyWidth: 900, canopyBase: 500, trunkHeight: 500, fruitPerPlant: 60, fruitDiameter: 70, ripeFraction: 0.5 },
  cherry: { training: 'v_trellis', rowSpacing: 3800, plantSpacing: 1500, canopyHeight: 3000, canopyWidth: 1600, canopyBase: 500, trunkHeight: 500, fruitPerPlant: 400, fruitDiameter: 26, ripeFraction: 0.7 },
  citrus: { training: 'hedgerow', rowSpacing: 6000, plantSpacing: 3000, canopyHeight: 3500, canopyWidth: 2800, canopyBase: 400, trunkHeight: 400, fruitPerPlant: 250, fruitDiameter: 70, ripeFraction: 0.5 },
  grape: { training: 'trellis_2d', rowSpacing: 2500, plantSpacing: 1000, canopyHeight: 1900, canopyWidth: 500, canopyBase: 700, trunkHeight: 700, fruitPerPlant: 30, fruitDiameter: 120, ripeFraction: 0.8 },
  strawberry: { training: 'greenhouse_gutter', rowSpacing: 1500, plantSpacing: 250, canopyHeight: 400, canopyWidth: 350, canopyBase: 1000, trunkHeight: 0, fruitPerPlant: 8, fruitDiameter: 30, ripeFraction: 0.4 },
  tomato: { training: 'greenhouse_gutter', rowSpacing: 1600, plantSpacing: 500, canopyHeight: 2500, canopyWidth: 500, canopyBase: 300, trunkHeight: 0, fruitPerPlant: 25, fruitDiameter: 60, ripeFraction: 0.3 },
  cucumber: { training: 'greenhouse_gutter', rowSpacing: 1600, plantSpacing: 400, canopyHeight: 2400, canopyWidth: 500, canopyBase: 300, trunkHeight: 0, fruitPerPlant: 12, fruitDiameter: 45, ripeFraction: 0.35 },
  blueberry: { training: 'freestanding', rowSpacing: 3000, plantSpacing: 1000, canopyHeight: 1800, canopyWidth: 1200, canopyBase: 200, trunkHeight: 100, fruitPerPlant: 300, fruitDiameter: 15, ripeFraction: 0.5 },
  olive: { training: 'hedgerow', rowSpacing: 4000, plantSpacing: 1500, canopyHeight: 2800, canopyWidth: 1200, canopyBase: 500, trunkHeight: 500, fruitPerPlant: 500, fruitDiameter: 20, ripeFraction: 0.9 },
  almond: { training: 'freestanding', rowSpacing: 6500, plantSpacing: 4500, canopyHeight: 4500, canopyWidth: 4000, canopyBase: 800, trunkHeight: 800, fruitPerPlant: 800, fruitDiameter: 35, ripeFraction: 0.95 },
  kiwi: { training: 'pergola', rowSpacing: 5000, plantSpacing: 3000, canopyHeight: 600, canopyWidth: 5000, canopyBase: 1800, trunkHeight: 1800, fruitPerPlant: 300, fruitDiameter: 60, ripeFraction: 0.9 },
  generic_tree: { training: 'freestanding', rowSpacing: 4000, plantSpacing: 2000, canopyHeight: 3000, canopyWidth: 2000, canopyBase: 600, trunkHeight: 600, fruitPerPlant: 50, fruitDiameter: 60, ripeFraction: 0.5 },
  row_crop: { training: 'freestanding', rowSpacing: 750, plantSpacing: 300, canopyHeight: 600, canopyWidth: 400, canopyBase: 0, trunkHeight: 0, fruitPerPlant: 0, fruitDiameter: 0, ripeFraction: 0 },
  greenhouse_bench: { training: 'greenhouse_gutter', rowSpacing: 1800, plantSpacing: 300, canopyHeight: 300, canopyWidth: 1200, canopyBase: 800, trunkHeight: 0, fruitPerPlant: 5, fruitDiameter: 25, ripeFraction: 0.5 },
};

export function cropParams(crop: CropType, overrides: Partial<CropParams> = {}): CropParams {
  return { crop, training: 'freestanding', rowSpacing: 4000, plantSpacing: 2000, rowHeading: 0, canopyHeight: 3000, canopyWidth: 2000, canopyBase: 600, trunkHeight: 600, fruitPerPlant: 50, fruitDiameter: 60, ripeFraction: 0.5, headland: 6000, ...CROP_PRESETS[crop], ...overrides };
}

/** Field / orchard block: polygon boundary (mm, local), crop parameters, geo origin. */
export class FieldItem extends Item {
  polygon: number[][] = [];
  crop: CropParams = cropParams('apple');
  geoOrigin: GeoOrigin | null = null;
  /** Soil / terrain: slope (deg) and heading (deg) for simple terrain modelling. */
  slopeDeg = 0;
  slopeHeading = 0;
  /** Environment: greenhouse (indoor) flag. */
  indoor = false;
  constructor(name = 'Field', id?: string) {
    super(ItemType.FIELD, name, id);
    this.color = '#6b8e23';
  }
  rows(): CropRow[] {
    return this.children.filter((c) => c instanceof CropRow) as CropRow[];
  }
  protected override serializeExtra() {
    return { polygon: this.polygon, crop: this.crop, geoOrigin: this.geoOrigin, slopeDeg: this.slopeDeg, slopeHeading: this.slopeHeading, indoor: this.indoor };
  }
  override deserializeExtra(d: SerializedItem) {
    this.polygon = (d.polygon as number[][]) ?? []; this.crop = { ...this.crop, ...((d.crop as any) ?? {}) }; this.geoOrigin = (d.geoOrigin as any) ?? null;
    this.slopeDeg = (d.slopeDeg as number) ?? 0; this.slopeHeading = (d.slopeHeading as number) ?? 0; this.indoor = !!d.indoor;
  }
}

export interface PlantRecord {
  /** Position along row (mm) and world pose is derived from the row. */
  s: number;
  height: number;
  width: number;
  /** Fruit positions relative to the plant base (mm) with ripeness 0-1 and picked flag. */
  fruit: Array<{ p: [number, number, number]; ripe: number; picked: boolean; d: number }>;
  health: number; // 0-1 (for scouting / disease scenarios)
}

/** Crop row: a straight segment inside a field with plants placed along it. */
export class CropRow extends Item {
  /** Start/end relative to this item's parent field (mm, ground plane). */
  start: [number, number] = [0, 0];
  end: [number, number] = [10000, 0];
  index = 0;
  plants: PlantRecord[] = [];
  /** Segment id for traffic management. */
  segmentId = '';
  constructor(name = 'Row', id?: string) {
    super(ItemType.CROP_ROW, name, id);
  }
  length(): number {
    return Math.hypot(this.end[0] - this.start[0], this.end[1] - this.start[1]);
  }
  direction(): [number, number] {
    const l = this.length() || 1;
    return [(this.end[0] - this.start[0]) / l, (this.end[1] - this.start[1]) / l];
  }
  /** Point on the row at distance s, offset laterally (mm) — used for robot travel lines beside the row. */
  pointAt(s: number, lateral = 0): [number, number] {
    const [dx, dy] = this.direction();
    return [this.start[0] + dx * s - dy * lateral, this.start[1] + dy * s + dx * lateral];
  }
  /** Parallel travel line at a lateral offset. */
  travelLine(lateral: number): { start: number[]; end: number[] } {
    return { start: this.pointAt(0, lateral), end: this.pointAt(this.length(), lateral) };
  }
  ripeFruitCount(): number {
    let n = 0;
    for (const p of this.plants) for (const f of p.fruit) if (f.ripe >= 0.5 && !f.picked) n++;
    return n;
  }
  protected override serializeExtra() {
    return { start: this.start, end: this.end, index: this.index, plants: this.plants, segmentId: this.segmentId };
  }
  override deserializeExtra(d: SerializedItem) {
    this.start = (d.start as any) ?? [0, 0]; this.end = (d.end as any) ?? [10000, 0]; this.index = (d.index as number) ?? 0; this.plants = (d.plants as any) ?? []; this.segmentId = (d.segmentId as string) ?? '';
  }
}

export type MissionType = 'harvest' | 'spray' | 'mow' | 'prune' | 'scout' | 'transport' | 'pollinate' | 'weed' | 'thin' | 'irrigate';
export type MissionStatus = 'draft' | 'planned' | 'running' | 'paused' | 'done' | 'aborted';

/** Mission: a high-level agricultural job over rows/zones executed by a fleet. */
export class MissionItem extends Item {
  missionType: MissionType = 'harvest';
  fieldId: string | null = null;
  fleetId: string | null = null;
  rowIds: string[] = [];
  status: MissionStatus = 'draft';
  /** Parameters (speed, swath, spray rate, etc). */
  settings: Record<string, any> = { workSpeed: 500, sideOffset: 1500, bothSides: true };
  /** Generated task ids (fleet tasks). */
  taskIds: string[] = [];
  progress = 0;
  constructor(name = 'Mission', id?: string) {
    super(ItemType.MISSION, name, id);
  }
  protected override serializeExtra() {
    return { missionType: this.missionType, fieldId: this.fieldId, fleetId: this.fleetId, rowIds: this.rowIds, status: this.status, settings: this.settings, taskIds: this.taskIds, progress: this.progress };
  }
  override deserializeExtra(d: SerializedItem, _ctx: DeserializeContext) {
    this.missionType = (d.missionType as MissionType) ?? 'harvest'; this.fieldId = (d.fieldId as any) ?? null; this.fleetId = (d.fleetId as any) ?? null; this.rowIds = (d.rowIds as string[]) ?? [];
    this.status = (d.status as MissionStatus) ?? 'draft'; this.settings = (d.settings as any) ?? {}; this.taskIds = (d.taskIds as string[]) ?? []; this.progress = (d.progress as number) ?? 0;
  }
}

/** Sensor item (lidar / camera / GNSS / soil probe) — mounted on robots or posts. */
export class SensorItem extends Item {
  kind: 'lidar2d' | 'lidar3d' | 'camera' | 'depth' | 'gnss' | 'imu' | 'soil' | 'weather' = 'camera';
  range = 10000;
  fov = 90;
  rate = 10;
  constructor(name = 'Sensor', id?: string) {
    super(ItemType.SENSOR, name, id);
  }
  protected override serializeExtra() {
    return { kind: this.kind, range: this.range, fov: this.fov, rate: this.rate };
  }
  override deserializeExtra(d: SerializedItem) {
    this.kind = (d.kind as any) ?? 'camera'; this.range = (d.range as number) ?? 10000; this.fov = (d.fov as number) ?? 90; this.rate = (d.rate as number) ?? 10;
  }
}

registerItemType(ItemType.FIELD, (n, id) => new FieldItem(n, id));
registerItemType(ItemType.CROP_ROW, (n, id) => new CropRow(n, id));
registerItemType(ItemType.MISSION, (n, id) => new MissionItem(n, id));
registerItemType(ItemType.SENSOR, (n, id) => new SensorItem(n, id));
export { SceneObject };
