/**
 * Training-data export from the simulator: rendered camera images with exact labels from the instance-ID buffer.
 *  - YOLO (Ultralytics): images/<split>/*.png, labels/<split>/*.txt (`cls cx cy w h`, normalised; `seg: true` writes
 *    polygon labels from the mask outline), data.yaml with the class names;
 *  - COCO: annotations.json (bbox, area = mask pixels, segmentation as uncompressed RLE, iscrowd 0) + images/*.png.
 */
import { Station, Camera as CameraItem } from '../core/items/item';
import { renderCamera, CameraRender } from './sensing';
import { captureTruth } from './pipeline';
import { encodePNG } from './png';
import type { Mask } from './models';
import type { AssetStore } from '../scene/assets';

export interface DatasetObject { cls: string; box: { x: number; y: number; w: number; h: number }; mask?: Mask; pixels?: number }
export interface DatasetSample { name: string; width: number; height: number; png: Uint8Array; objects: DatasetObject[] }

/** One labelled frame of a camera: shaded render + visible objects (truth from the ID buffer). */
export function captureDatasetSample(station: Station, cam: CameraItem, name: string, opts: { assets?: AssetStore | null; classes?: string[]; render?: CameraRender } = {}): DatasetSample {
  const r = opts.render?.rgba && opts.render.width === cam.width && opts.render.height === cam.height ? opts.render : renderCamera(station, cam, { rgb: true, assets: opts.assets });
  const wanted = new Set((opts.classes ?? []).map((c) => c.toLowerCase()));
  const truth = captureTruth(station, cam, { render: r, assets: opts.assets, fruit: false, trunks: false });
  const objects = truth.filter((t) => t.visible && (!wanted.size || wanted.has(t.cls.toLowerCase()))).map((t) => ({ cls: t.cls, box: t.box, mask: t.mask, pixels: t.pixels }));
  return { name, width: r.width, height: r.height, png: encodePNG(r.rgba!, r.width, r.height), objects };
}

/** Outer boundary of a mask as a polygon (pixel corners, image coordinates) — marching along the boundary of the largest blob's bounding rows. */
function maskPolygon(m: Mask): Array<[number, number]> {
  const x0 = m.x0 ?? 0, y0 = m.y0 ?? 0, pts: Array<[number, number]> = [];
  // left edges top→bottom, then right edges bottom→top (exact for row-convex silhouettes, a good outline otherwise)
  const left: Array<[number, number]> = [], right: Array<[number, number]> = [];
  for (let y = 0; y < m.height; y++) {
    let a = -1, b = -1;
    for (let x = 0; x < m.width; x++) if (m.data[y * m.width + x]) { if (a < 0) a = x; b = x; }
    if (a >= 0) { left.push([x0 + a, y0 + y], [x0 + a, y0 + y + 1]); right.push([x0 + b + 1, y0 + y], [x0 + b + 1, y0 + y + 1]); }
  }
  pts.push(...left, ...right.reverse());
  return pts;
}

export function yoloDataset(samples: DatasetSample[], classes: string[], opts: { split?: string; seg?: boolean } = {}): Record<string, Uint8Array | string> {
  const split = opts.split ?? 'train';
  const idx = new Map(classes.map((c, i) => [c.toLowerCase(), i]));
  const files: Record<string, Uint8Array | string> = {};
  for (const s of samples) {
    files[`images/${split}/${s.name}.png`] = s.png;
    const lines: string[] = [];
    for (const o of s.objects) {
      const k = idx.get(o.cls.toLowerCase()); if (k === undefined) continue;
      if (opts.seg && o.mask) lines.push(`${k} ${maskPolygon(o.mask).map(([x, y]) => `${(x / s.width).toFixed(6)} ${(y / s.height).toFixed(6)}`).join(' ')}`);
      else lines.push(`${k} ${((o.box.x + o.box.w / 2) / s.width).toFixed(6)} ${((o.box.y + o.box.h / 2) / s.height).toFixed(6)} ${(o.box.w / s.width).toFixed(6)} ${(o.box.h / s.height).toFixed(6)}`);
    }
    files[`labels/${split}/${s.name}.txt`] = lines.join('\n') + (lines.length ? '\n' : '');
  }
  files['data.yaml'] = `# VerticalBot Studio synthetic dataset (labels from the instance-ID render)\npath: .\ntrain: images/${split}\nval: images/${split}\nnames:\n${classes.map((c, i) => `  ${i}: ${c}`).join('\n')}\n`;
  return files;
}

/** COCO uncompressed RLE of a mask placed in a width×height image (column-major, starting with zeros). */
export function maskToRle(m: Mask, width: number, height: number): { counts: number[]; size: [number, number] } {
  const x0 = m.x0 ?? 0, y0 = m.y0 ?? 0, counts: number[] = [];
  let cur = 0, run = 0;
  for (let x = 0; x < width; x++) for (let y = 0; y < height; y++) {
    const mx = x - x0, my = y - y0;
    const v = mx >= 0 && my >= 0 && mx < m.width && my < m.height && m.data[my * m.width + mx] ? 1 : 0;
    if (v !== cur) { counts.push(run); run = 0; cur = v; }
    run++;
  }
  counts.push(run);
  return { counts, size: [height, width] };
}

export function cocoDataset(samples: DatasetSample[], classes: string[]): Record<string, Uint8Array | string> {
  const files: Record<string, Uint8Array | string> = {};
  const images: any[] = [], annotations: any[] = [];
  const idx = new Map(classes.map((c, i) => [c.toLowerCase(), i + 1]));
  let aid = 1;
  samples.forEach((s, i) => {
    files[`images/${s.name}.png`] = s.png;
    images.push({ id: i + 1, file_name: `images/${s.name}.png`, width: s.width, height: s.height });
    for (const o of s.objects) {
      const cid = idx.get(o.cls.toLowerCase()); if (!cid) continue;
      annotations.push({ id: aid++, image_id: i + 1, category_id: cid, bbox: [o.box.x, o.box.y, o.box.w, o.box.h], area: o.pixels ?? o.box.w * o.box.h, iscrowd: 0, segmentation: o.mask ? maskToRle(o.mask, s.width, s.height) : [] });
    }
  });
  files['annotations.json'] = JSON.stringify({ info: { description: 'VerticalBot Studio synthetic dataset', version: '1.0' }, images, annotations, categories: classes.map((c, i) => ({ id: i + 1, name: c, supercategory: 'object' })) });
  return files;
}
