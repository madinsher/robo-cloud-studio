/**
 * Crop collision world: tree trunks / stems (cylinders from the ground to the canopy top) and canopies (disks of the
 * plant's own width between canopyBase and the canopy top) as circle obstacles with height bands. Registered as an
 * obstacle provider for mobile planning, traffic management and contact detection (a robot lower than canopyBase
 * passes under a pergola, trunks always block), and turned into SceneObjects for the arm collision checker.
 */
import { Station, ItemType, SceneObject, GeometryRef } from '../core/items/item';
import { transl } from '../core/math/pose';
import { FieldItem, CropRow, PlantRecord } from './items';
import { CircleObstacle } from '../mobile/footprint';
import { registerObstacleProvider } from '../mobile/world';

/** Trunk / stem radius (mm) of a crop. */
export function trunkRadius(field: FieldItem): number {
  const c = field.crop;
  if (c.trunkDiameter) return c.trunkDiameter / 2;
  if (c.training === 'greenhouse_gutter') return 15;
  return c.crop === 'blueberry' || c.crop === 'grape' ? 40 : 75;
}

/** World (station-frame) position of a plant base. */
export function plantWorld(row: CropRow, plant: PlantRecord): [number, number] {
  const [lx, ly] = row.pointAt(plant.s);
  const abs = row.parent ? row.parent.poseAbs() : null;
  return abs ? [abs[0] * lx + abs[4] * ly + abs[12], abs[1] * lx + abs[5] * ly + abs[13]] : [lx, ly];
}

/** Circle obstacles of every plant in the station's fields. */
export function cropObstacles(station: Station): CircleObstacle[] {
  const out: CircleObstacle[] = [];
  for (const field of station.itemsOfType<FieldItem>(ItemType.FIELD)) {
    const c = field.crop;
    const tr = trunkRadius(field);
    for (const row of field.rows()) row.plants.forEach((p, i) => {
      const [x, y] = plantWorld(row, p);
      const top = c.canopyBase + p.height;
      out.push({ x, y, r: tr, zMin: 0, zMax: top, kind: 'trunk', ref: { rowId: row.id, plant: i } });
      out.push({ x, y, r: p.width / 2, zMin: c.canopyBase, zMax: top, kind: 'canopy', ref: { rowId: row.id, plant: i } });
    });
  }
  return out;
}

registerObstacleProvider({
  id: 'agri.crop',
  signature: (station) => station.itemsOfType<FieldItem>(ItemType.FIELD).map((f) => { const a = f.poseAbs(); let n = 0, s = 0; for (const r of f.rows()) { n += r.plants.length; s += r.start[0] + r.start[1] * 3 + r.end[0] * 7 + r.end[1] * 11; } return `${f.id}:${n}:${s.toFixed(0)}:${a[12].toFixed(0)},${a[13].toFixed(0)},${a[0].toFixed(4)}`; }).join(','),
  obstacles: cropObstacles,
});

/**
 * Rigid crop parts near a point as a SceneObject for the arm collision checker: trunk + central leader cylinders
 * (foliage is soft — arms reach into it — so canopies are not rigid colliders). Replaces an earlier object of the
 * same name. Returns the object (or null when no plant is within `radius`).
 */
export function cropColliders(station: Station, center: [number, number], radius = 3000, name = 'Crop colliders'): SceneObject | null {
  const old = station.find(name, ItemType.OBJECT);
  if (old) old.delete();
  const geo: GeometryRef[] = [];
  for (const field of station.itemsOfType<FieldItem>(ItemType.FIELD)) {
    const tr = trunkRadius(field);
    const fz = field.poseAbs()[14];
    for (const row of field.rows()) for (const p of row.plants) {
      const [x, y] = plantWorld(row, p);
      if (Math.hypot(x - center[0], y - center[1]) > radius) continue;
      const h = field.crop.canopyBase + p.height * 0.9;
      geo.push({ primitive: { kind: 'cylinder', radius: tr, length: h }, origin: Array.from(transl(x, y, fz + h / 2)), color: '#5c3d1e' });
    }
  }
  if (!geo.length) return null;
  const obj = station.addChild(new SceneObject(name));
  // separate collision geometry when the core supports it (I9), else rigid trunks double as visual geometry
  if ('colliders' in obj) { (obj as any).colliders = geo; obj.geometry = []; } else obj.geometry = geo;
  obj.setParam('agriColliders', true);
  return obj;
}
