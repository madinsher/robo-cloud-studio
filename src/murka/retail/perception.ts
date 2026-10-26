/**
 * Classical product recognition of murom_perception (recognition.py + perception_node.py + calibrate.py), on frames
 * rendered by the studio itself (software rasterizer of src/vision/sensing.ts: pixel-true metric depth along the
 * optical axis and shaded colour — the head camera and the range finder share pose, size and FOV like in Webots):
 *  1. for each slot a search box above the board (±0.21 m around the slot centre, from 2.5 cm above the board up to the
 *     region height or below the next board) — back wall, boards and neighbouring slots are cut off by geometry;
 *  2. range-finder pixels are back-projected to the map; points inside the box are the product, < 25 points = empty;
 *  3. features: median colour (HSV) and the width / height of the visible face (5–95 % percentiles);
 *  4. nearest prototype by a weighted HSV + size distance, confidence by the margin to the second candidate.
 * Prototype colours come from `calibrate()` on scan frames of this simulator (the MURKA calibrate tool), shipped as
 * appearance_studio.json and re-checked by the tests.
 */
import type { Station } from '../../core/items/item';
import type { AssetStore } from '../../scene/assets';
import { sceneMeshes, rasterize, CameraRender } from '../../vision/sensing';
import { cameraIntrinsics } from '../../vision/camera_model';
import type { RetailLayout, RetailSlot, CatalogEntry } from './layout';

export const HALF_XY = 0.21, MIN_POINTS = 25, BOARD_SKIP = 0.025;
export const REGION_HEIGHT: Record<string, number> = { gondola: 0.24, pallet: 0.33, bulk: 0.33, packing: 0.30 };
export const SMALL_KINDS = new Set(['gondola', 'packing']);
export const VIEW_MARGIN = 12, OCCLUSION = 0.35, MAX_RANGE = 4.0;

export interface Prototype { sku: string; size: [number, number, number]; color: [number, number, number]; twoHand: boolean; hsv: [number, number, number] }
export interface SlotView { slotId: string; visible: boolean; sku: string; confidence: number; nPoints: number; position: [number, number, number]; width: number; height: number; bbox: [number, number, number, number]; rgb: [number, number, number]; scores: Record<string, number> }
/** One camera frame: colour (0..1, row-major RGB), planar depth (m, 0 = no return), intrinsics, optical frame in the map. */
export interface Frame { width: number; height: number; rgb: Float32Array; depth: Float32Array; K: { fx: number; fy: number; cx: number; cy: number }; T: number[] /* row-major 4×4, m */ }
export interface Appearance { simulator: string; frames: number; skus: Record<string, { rgb: [number, number, number]; hsv: [number, number, number]; samples: number; width: number; height: number }> }

/** colorsys.rgb_to_hsv */
export function rgbToHsv(r: number, g: number, b: number): [number, number, number] {
  const mx = Math.max(r, g, b), mn = Math.min(r, g, b);
  const v = mx; if (mn === mx) return [0, 0, v];
  const s = (mx - mn) / mx;
  const rc = (mx - r) / (mx - mn), gc = (mx - g) / (mx - mn), bc = (mx - b) / (mx - mn);
  let h: number;
  if (r === mx) h = bc - gc; else if (g === mx) h = 2 + rc - bc; else h = 4 + gc - rc;
  h = ((h / 6) % 1 + 1) % 1;
  return [h, s, v];
}

export function loadCatalog(L: RetailLayout, appearance?: Appearance | null): Record<string, Prototype> {
  const out: Record<string, Prototype> = {};
  for (const c of Object.values(L.catalog) as CatalogEntry[]) out[c.sku] = { sku: c.sku, size: [...c.size], color: [...c.color], twoHand: c.twoHand, hsv: rgbToHsv(...c.color) };
  if (appearance) for (const [sku, d] of Object.entries(appearance.skus)) if (out[sku]) { out[sku].color = [...d.rgb]; out[sku].hsv = rgbToHsv(...d.rgb); }
  return out;
}

function hsvDistance(a: [number, number, number], b: [number, number, number]): number {
  let dh = Math.abs(a[0] - b[0]); dh = Math.min(dh, 1 - dh);
  const sat = Math.min(a[1], b[1]);
  return 2.0 * sat * dh + 1.0 * Math.abs(a[1] - b[1]) + 0.35 * Math.abs(a[2] - b[2]);
}

export function classify(rgb: [number, number, number], width: number, height: number, candidates: Prototype[], along = 0): [string, number, Record<string, number>] {
  const hsv = rgbToHsv(...rgb); const scores: Record<string, number> = {};
  for (const c of candidates) { const colorD = hsvDistance(hsv, c.hsv); const sizeD = Math.abs(width - c.size[along]) / c.size[along] + Math.abs(height - c.size[2]) / c.size[2]; scores[c.sku] = colorD + 0.6 * sizeD; }
  const ranked = Object.entries(scores).sort((a, b) => a[1] - b[1]);
  if (!ranked.length) return ['unknown', 0, scores];
  const [bestSku, best] = ranked[0]; const second = ranked.length > 1 ? ranked[1][1] : best + 1;
  const margin = (second - best) / (second + 1e-6);
  const conf = Math.max(0, Math.min(1, 0.5 + margin)) * Math.max(0.2, Math.min(1, 1.2 - best));
  return [bestSku, conf, scores];
}

const pct = (vals: number[], p: number) => { const s = [...vals].sort((a, b) => a - b); const k = (s.length - 1) * (p / 100); const f = Math.floor(k), c = Math.min(s.length - 1, f + 1); return s[f] + (s[c] - s[f]) * (k - f); };
const median = (vals: number[]) => pct(vals, 50);

/** recognize() of recognition.py on one frame for a set of slots. */
export function recognize(L: RetailLayout, slotIds: string[], f: Frame, catalog: Record<string, Prototype>, opts: { minConf?: number; maxRange?: number } = {}): SlotView[] {
  const { width: w, height: h, K, T } = f; const maxRange = opts.maxRange ?? MAX_RANGE;
  // back-projected points in the map (planar depth)
  const n = w * h; const X = new Float64Array(n), Y = new Float64Array(n), Z = new Float64Array(n); const valid = new Uint8Array(n);
  for (let v = 0; v < h; v++) for (let u = 0; u < w; u++) {
    const i = v * w + u; const d = f.depth[i];
    if (!(d > 0.08 && d < maxRange)) continue;
    const xc = ((u - K.cx) / K.fx) * d, yc = ((v - K.cy) / K.fy) * d;
    X[i] = T[0] * xc + T[1] * yc + T[2] * d + T[3]; Y[i] = T[4] * xc + T[5] * yc + T[6] * d + T[7]; Z[i] = T[8] * xc + T[9] * yc + T[10] * d + T[11];
    valid[i] = 1;
  }
  const toPixel = (p: [number, number, number]): [number, number, number] => {
    const dx = p[0] - T[3], dy = p[1] - T[7], dz = p[2] - T[11];
    const pc = [T[0] * dx + T[4] * dy + T[8] * dz, T[1] * dx + T[5] * dy + T[9] * dz, T[2] * dx + T[6] * dy + T[10] * dz];
    if (pc[2] <= 0.05) return [NaN, NaN, pc[2]];
    return [K.fx * pc[0] / pc[2] + K.cx, K.fy * pc[1] / pc[2] + K.cy, pc[2]];
  };
  return slotIds.map((id) => recognizeSlot(id, L.slots[id], f, X, Y, Z, valid, toPixel, catalog, opts.minConf ?? 0.35));
}

function recognizeSlot(slotId: string, slot: RetailSlot, f: Frame, X: Float64Array, Y: Float64Array, Z: Float64Array, valid: Uint8Array, toPixel: (p: [number, number, number]) => [number, number, number], catalog: Record<string, Prototype>, minConf: number): SlotView {
  const kind = slot.kind ?? 'gondola';
  const { x: sx, y: sy, z: sz } = slot.pose;
  const top = sz + Math.min(REGION_HEIGHT[kind] ?? 0.3, (slot.ceiling ?? 9.0) - sz - 0.01);
  const { width: w, height: h } = f;
  const inside: number[] = [];
  for (let i = 0; i < w * h; i++) if (valid[i] && Math.abs(X[i] - sx) < HALF_XY && Math.abs(Y[i] - sy) < HALF_XY && Z[i] > sz + BOARD_SKIP && Z[i] < top) inside.push(i);
  const nPts = inside.length;
  const [u, v, zc] = toPixel([sx, sy, (sz + BOARD_SKIP + top) / 2]);
  const inFrame = !Number.isNaN(u) && VIEW_MARGIN <= u && u < w - VIEW_MARGIN && VIEW_MARGIN <= v && v < h - VIEW_MARGIN;
  let occluded = false;
  if (inFrame) {
    const patch: number[] = [];
    for (let vv = Math.trunc(v) - 2; vv < Math.trunc(v) + 3; vv++) for (let uu = Math.trunc(u) - 2; uu < Math.trunc(u) + 3; uu++) { if (vv < 0 || uu < 0 || vv >= h || uu >= w) continue; const d = f.depth[vv * w + uu]; if (d > 0 && Number.isFinite(d)) patch.push(d); }
    occluded = patch.length === 0 || median(patch) < zc - OCCLUSION;
  }
  const view: SlotView = { slotId, visible: nPts >= MIN_POINTS || (inFrame && !occluded), sku: '', confidence: 0, nPoints: nPts, position: [0, 0, 0], width: 0, height: 0, bbox: [0, 0, 0, 0], rgb: [0, 0, 0], scores: {} };
  if (nPts < MIN_POINTS) { view.confidence = view.visible ? 0.9 : 0; return view; }
  let u0 = Infinity, u1 = -Infinity, v0 = Infinity, v1 = -Infinity; let cx = 0, cy = 0, cz = 0;
  for (const i of inside) { const uu = i % w, vv = (i - uu) / w; u0 = Math.min(u0, uu); u1 = Math.max(u1, uu); v0 = Math.min(v0, vv); v1 = Math.max(v1, vv); cx += X[i]; cy += Y[i]; cz += Z[i]; }
  view.bbox = [u0, v0, u1 - u0 + 1, v1 - v0 + 1];
  const centroid: [number, number, number] = [cx / nPts, cy / nPts, cz / nPts];
  const ray = [centroid[0] - f.T[3], centroid[1] - f.T[7]];
  const axis = Math.abs(ray[0]) < Math.abs(ray[1]) ? 0 : 1;
  const A = inside.map((i) => (axis === 0 ? X[i] : Y[i])), Zs = inside.map((i) => Z[i]);
  view.width = pct(A, 95) - pct(A, 5);
  view.height = pct(Zs, 95) - sz;
  view.position = centroid;
  view.rgb = [0, 1, 2].map((c) => median(inside.map((i) => f.rgb[i * 3 + c]))) as [number, number, number];
  const cands = Object.values(catalog).filter((c) => SMALL_KINDS.has(kind) !== c.twoHand);
  const [sku, conf, scores] = classify(view.rgb, view.width, view.height, cands, axis);
  view.scores = scores; view.sku = conf >= minConf ? sku : 'unknown'; view.confidence = conf;
  return view;
}

/** perception_node._targets: one slot, all slots of a shelf, or (empty) all gondola slots. */
export function scanTargets(L: RetailLayout, target: string): string[] {
  if (L.slots[target]) return [target];
  if (!target) return Object.keys(L.slots).filter((s) => L.slots[s].kind === 'gondola');
  return Object.keys(L.slots).filter((s) => s.startsWith(`${target}_`));
}

/** Render the head camera (colour) and the range finder (depth) of the station, as the Webots devices deliver them. */
export function renderHeadFrame(station: Station, cam: { poseAbs(): Float64Array; width: number; height: number; fov: number; fovAxis?: 'vertical' | 'horizontal'; near: number; far: number }, depthCam: { near: number; far: number } | null, assets?: AssetStore | null): Frame & { render: CameraRender } {
  const K = cameraIntrinsics(cam as never);
  const meshes = sceneMeshes(station, { assets: assets ?? null, fields: false, ground: true });
  const near = depthCam?.near ?? cam.near, far = depthCam?.far ?? cam.far;
  const r = rasterize(meshes, K, cam.poseAbs(), near, far, { rgb: true });
  const n = r.width * r.height; const rgb = new Float32Array(n * 3); const depth = new Float32Array(n);
  for (let i = 0; i < n; i++) { rgb[i * 3] = r.rgba![i * 4] / 255; rgb[i * 3 + 1] = r.rgba![i * 4 + 1] / 255; rgb[i * 3 + 2] = r.rgba![i * 4 + 2] / 255; depth[i] = r.depth[i] / 1000; }
  const P = cam.poseAbs();
  const T = [P[0], P[4], P[8], P[12] / 1000, P[1], P[5], P[9], P[13] / 1000, P[2], P[6], P[10], P[14] / 1000, 0, 0, 0, 1];
  return { width: r.width, height: r.height, rgb, depth, K: { fx: K.fx, fy: K.fy, cx: K.cx, cy: K.cy }, T, render: r };
}

/** calibrate.py: prototype colour of a SKU = median colour of its labelled samples in the scan frames. */
export function calibrate(L: RetailLayout, dumps: Array<{ ids: string[]; frame: Frame }>, simulator = 'studio'): { appearance: Appearance; before: [number, number]; after: [number, number]; errors: string[] } {
  const truth: Record<string, string> = {}; for (const i of L.instances) truth[i.slot] = i.sku;
  const samples = (cat: Record<string, Prototype>) => dumps.flatMap((d) => recognize(L, d.ids, d.frame, cat).filter((v) => v.visible));
  const acc = (vs: SlotView[]): [number, number, string[]] => { let ok = 0; const errors: string[] = []; for (const v of vs) { const t = truth[v.slotId] ?? ''; if (v.sku === t) ok++; else errors.push(`${v.slotId}: ${v.sku || 'пусто'} вместо ${t || 'пусто'} (${v.confidence.toFixed(2)})`); } return [ok, vs.length, errors]; };
  const cat0 = loadCatalog(L);
  const before = samples(cat0);
  const [ok0, n0] = acc(before);
  const bySku: Record<string, SlotView[]> = {};
  for (const v of before) { const sku = truth[v.slotId] ?? ''; if (sku && v.nPoints >= MIN_POINTS) (bySku[sku] ??= []).push(v); }
  const app: Appearance = { simulator, frames: dumps.length, skus: {} };
  for (const [sku, vs] of Object.entries(bySku).sort((a, b) => (a[0] < b[0] ? -1 : 1))) {
    const rgb = [0, 1, 2].map((c) => median(vs.map((v) => v.rgb[c]))) as [number, number, number];
    const r4 = (x: number) => Math.round(x * 1e4) / 1e4;
    app.skus[sku] = { rgb: rgb.map(r4) as [number, number, number], hsv: rgbToHsv(...rgb).map(r4) as [number, number, number], samples: vs.length, width: r4(median(vs.map((v) => v.width))), height: r4(median(vs.map((v) => v.height))) };
  }
  const after = samples(loadCatalog(L, app));
  const [ok1, n1, errors] = acc(after);
  return { appearance: app, before: [ok0, n0], after: [ok1, n1], errors };
}
