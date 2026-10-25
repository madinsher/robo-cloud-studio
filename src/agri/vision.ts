/**
 * Simulated perception of fruit: project fruit into a camera item (the studio camera model, cameraIntrinsics) and
 * return detections (pixel boxes, range, ripeness). Occlusion: solid geometry (objects, walls, robots, tools,
 * vehicles — ray cast against the same triangles the camera renders), nearer fruit, and a canopy-density model
 * (leaves around a fruit). Enables "detect → pick" scenarios without real images.
 */
import { Station, Camera as CameraItem, ItemType } from '../core/items/item';
import { CropRow } from './items';
import { fruitWorldPositions } from './orchard';
import { multiply, invert, Vec3 } from '../core/math/pose';
import { cameraIntrinsics } from '../vision/camera_model';
import { sceneMeshes, firstHit, SceneMesh } from '../vision/sensing';

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
export function detectFruit(station: Station, camera: CameraItem, opts: { onlyRipe?: boolean; maxRange?: number; occlusionPerMeter?: number; solids?: SceneMesh[] } = {}): Detection[] {
  const camWorld = camera.poseAbs();
  const inv = invert(camWorld);
  const { fx, fy, cx, cy } = cameraIntrinsics(camera);
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
  // solid occluders between the camera and the fruit centre (fields excluded: canopy occlusion is statistical below)
  if (out.length) {
    const solids = opts.solids ?? sceneMeshes(station, { fields: false });
    if (solids.length) {
      const o: Vec3 = [camWorld[12], camWorld[13], camWorld[14]];
      for (let i = out.length - 1; i >= 0; i--) {
        const d = out[i], dir: Vec3 = [d.p[0] - o[0], d.p[1] - o[1], d.p[2] - o[2]];
        const len = Math.hypot(dir[0], dir[1], dir[2]);
        // from the near plane to just before the fruit surface
        const hit = firstHit(solids, o, dir, Math.max(0, 1 - (d.fruit.d / 2) / len), { tMin: camera.near / len });
        if (hit) out.splice(i, 1);
      }
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
