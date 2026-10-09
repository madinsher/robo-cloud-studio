/**
 * Simulated perception: project fruit / plants into a camera item and return detections
 * (pixel boxes, range, ripeness), with occlusion approximated by depth ordering and canopy density.
 * Enables "detect → pick" scenarios without real images.
 */
import { Station, Camera as CameraItem, ItemType } from '../core/items/item';
import { CropRow } from './items';
import { fruitWorldPositions } from './orchard';
import { multiply, invert } from '../core/math/pose';

export interface Detection {
  /** World position (mm). */
  p: [number, number, number];
  /** Pixel centre and box size. */
  u: number; v: number; w: number; h: number;
  range: number;
  ripe: number;
  rowId: string;
  fruit: any;
}

/** Project world points into a pinhole camera (camera looks along +Z of the item, X right, Y down). */
export function detectFruit(station: Station, camera: CameraItem, opts: { onlyRipe?: boolean; maxRange?: number; occlusionPerMeter?: number } = {}): Detection[] {
  const inv = invert(camera.poseAbs());
  const fx = (camera.width / 2) / Math.tan(((camera.fov * Math.PI) / 180) / 2);
  const fy = fx;
  const cx = camera.width / 2, cy = camera.height / 2;
  const maxRange = opts.maxRange ?? camera.far;
  const out: Detection[] = [];
  for (const row of station.itemsOfType<CropRow>(ItemType.CROP_ROW)) {
    for (const f of fruitWorldPositions(row, opts.onlyRipe ?? false)) {
      const l = multiply(inv, Float64Array.from([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, f.p[0], f.p[1], f.p[2], 1]));
      const x = l[12], y = l[13], z = l[14];
      if (z < camera.near || z > maxRange) continue;
      const u = cx + (fx * x) / z, v = cy + (fy * y) / z; // optical convention: +X right, +Y down (matches renderFromItem)
      if (u < 0 || v < 0 || u >= camera.width || v >= camera.height) continue;
      const size = (fx * f.fruit.d) / z;
      out.push({ p: f.p, u, v, w: size, h: size, range: z, ripe: f.fruit.ripe, rowId: row.id, fruit: f.fruit });
    }
  }
  // occlusion: sort by range, drop detections whose box is mostly covered by a nearer one, plus
  // a distance-based canopy occlusion probability
  out.sort((a, b) => a.range - b.range);
  const kept: Detection[] = [];
  const occ = opts.occlusionPerMeter ?? 0.25;
  // canopy occlusion is a property of the fruit (leaves around it), not of the frame: hash its position so a
  // hidden fruit stays hidden and a visible one stays visible while the camera moves (approaching reveals more)
  const hash01 = (p: [number, number, number]) => { let h = 2166136261; for (const v of p) { h ^= Math.round(v) & 0xffff; h = Math.imul(h, 16777619) >>> 0; } return (h % 100000) / 100000; };
  for (const d of out) {
    const covered = kept.some((k) => Math.abs(k.u - d.u) < (k.w + d.w) / 4 && Math.abs(k.v - d.v) < (k.h + d.h) / 4);
    if (covered) continue;
    if (hash01(d.p) < Math.min(0.9, occ * (d.range / 1000))) continue;
    kept.push(d);
  }
  return kept;
}
