/**
 * Vision pipeline runtime: ties a station camera + `VisionStackConfig` to the pluggable models.
 *
 *  capture  → a pixel-true render of what the camera sees (sensing.ts: metric depth + instance IDs, WebGL in the browser,
 *             software rasterizer in Node), ground truth read from the ID buffer (exact boxes, masks, occlusion by robots,
 *             tools, meshes and same-class objects), the sensor depth image for every depth modality (stereo / RGB-D /
 *             ToF, with the catalogue noise when `simulate`), point clouds, or the pixels the browser rendered;
 *  infer    → one model per task (detect / segment / keypoints / classify / VLM / VLA);
 *  track    → ByteTrack ids and velocities;
 *  locate   → 3D per detection: from the depth image (localize.ts: dominant face, outline, truncation-aware centre,
 *             yaw), truth + noise for model-based 6D estimators in simulation, size prior (mono), LiDAR clusters;
 *  act      → grasp + approach targets on the face (tool Z into the part, yaw folded by symmetry), IK + collision checked
 *             before targets are created, follow commands for a mobile base, TCP deltas for VLA.
 *
 * Everything runs in Node (software render), so picking cycles and fleets can be validated in tests and headless.
 */
import { Station, Camera as CameraItem, ItemType, SceneObject, Frame, Target, Item, Tool } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { Mat4, Vec3, invert, multiply, transformPoint, transl, identity, normalize, sub, dot } from '../core/math/pose';
import { detectFruit } from '../agri/vision';
import { FieldItem } from '../agri/items';
import { plantPosition } from '../agri/orchard';
import type { MobileRobot } from '../mobile/items';
import { VisionStackConfig, VisionTask, VISION_SENSORS, VISION_MODELS, getVisionStack } from './stack';
import { Intrinsics, cameraIntrinsics, project, backproject, depthSigmaMm, rangeFromSize, targetPoseFromPoint } from './camera_model';
import { VisionModel, VisionFrame, VisionResult, Box2D, TruthObject, ByteTracker, createModel, applyVlaAction, ModelContext as BaseModelContext } from './models';
import { visionSummary } from '../ros/publishers';
import { PointCloud, simulateLidar3D, voxelDownsample, fitGroundPlane, removeGround, euclideanCluster, Cluster, detectRows, Line2D, pointCount, selectPoints, fitLine2D, depthToCloud } from './pointcloud';
import { CameraRender, renderCamera, regionsFromRender, sceneMeshes, silhouettePixels, projectedBounds, SceneMesh } from './sensing';
import { localizeFromDepth, surfaceFromTruth, graspFromSurface, SurfaceEstimate } from './localize';
import { checkRobotCollisions } from '../core/collision/collision';
import type { AssetStore } from '../scene/assets';

// ---------------------------------------------------------------------------------------------
// Ground truth capture
// ---------------------------------------------------------------------------------------------

function cropClass(station: Station, rowId: string): string {
  const row = station.findById(rowId);
  const field = row?.parent as FieldItem | undefined;
  return field && field.crop ? String(field.crop.crop) : 'fruit';
}

/** Project a world AABB (8 corners) into the image; returns the box or null when fully outside/behind. */
function projectBox(K: Intrinsics, inv: Mat4, corners: Vec3[]): { box: { x: number; y: number; w: number; h: number }; z: number } | null {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity, zMin = Infinity, n = 0;
  for (const c of corners) { const l = transformPoint(inv, c); const uv = project(K, l); if (!uv) continue; n++; x0 = Math.min(x0, uv[0]); y0 = Math.min(y0, uv[1]); x1 = Math.max(x1, uv[0]); y1 = Math.max(y1, uv[1]); zMin = Math.min(zMin, l[2]); }
  if (n < 4) return null;
  const bx = Math.max(0, x0), by = Math.max(0, y0), ex = Math.min(K.width, x1), ey = Math.min(K.height, y1);
  if (ex <= bx || ey <= by) return null;
  return { box: { x: bx, y: by, w: ex - bx, h: ey - by }, z: zMin };
}

/** Oriented box of an object in its own frame: centre (local) and full size, from bbox or the geometry (primitives / mesh bounds). */
export function objectLocalBox(o: SceneObject, assets?: AssetStore | null): { centre: Vec3; dims: Vec3 } | null {
  if (o.bbox) return { centre: [(o.bbox.max[0] + o.bbox.min[0]) / 2, (o.bbox.max[1] + o.bbox.min[1]) / 2, (o.bbox.max[2] + o.bbox.min[2]) / 2], dims: [o.bbox.max[0] - o.bbox.min[0], o.bbox.max[1] - o.bbox.min[1], o.bbox.max[2] - o.bbox.min[2]] };
  const lo: Vec3 = [Infinity, Infinity, Infinity], hi: Vec3 = [-Infinity, -Infinity, -Infinity];
  for (const g of o.geometry) {
    let mn: Vec3 | null = null, mx: Vec3 | null = null;
    const pr = g.primitive;
    if (pr) { const h: Vec3 = pr.kind === 'box' ? [pr.size[0] / 2, pr.size[1] / 2, pr.size[2] / 2] : pr.kind === 'cylinder' || pr.kind === 'cone' ? [pr.radius, pr.radius, pr.length / 2] : pr.kind === 'sphere' ? [pr.radius, pr.radius, pr.radius] : [pr.size[0] / 2, pr.size[1] / 2, 0.5]; mn = [-h[0], -h[1], -h[2]]; mx = h; }
    else if (g.mesh && assets) { const a = assets.get(g.mesh); if (a?.mesh) { mn = a.mesh.min.slice() as Vec3; mx = a.mesh.max.slice() as Vec3; } }
    if (!mn || !mx) continue;
    const s = g.scale ?? [1, 1, 1];
    const org = g.origin && g.origin.length === 16 ? Float64Array.from(g.origin) as unknown as Mat4 : identity();
    for (const sx of [mn[0], mx[0]]) for (const sy of [mn[1], mx[1]]) for (const sz of [mn[2], mx[2]]) { const p = transformPoint(org, [sx * s[0], sy * s[1], sz * s[2]]); for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], p[k]); hi[k] = Math.max(hi[k], p[k]); } }
  }
  if (!(lo[0] <= hi[0])) return null;
  return { centre: [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2], dims: [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]] };
}

/** Class reported for an object: its `visionClass` / `class` parameter, else the name without the trailing number. */
export function objectClass(o: Item): string {
  const p = (o.params.visionClass ?? o.params.class) as unknown;
  if (typeof p === 'string' && p) return p.toLowerCase();
  return o.name.toLowerCase().replace(/[\s_]+\d*$/, '').replace(/\d+$/, '') || 'object';
}

export interface TruthOptions {
  maxRange?: number; fruit?: boolean; trunks?: boolean; objects?: boolean; vehicles?: boolean;
  /** A render of this camera at its resolution (browser GPU); rendered in software when absent. */
  render?: CameraRender;
  assets?: AssetStore | null;
  /** An object counts as visible with at least this many pixels and this fraction of its unoccluded silhouette. */
  minPixels?: number; minVisibleFraction?: number;
}

/** What a camera sees, as ground truth (simulation). Objects and vehicles come from the instance-ID render (pixel-true occlusion). */
export function captureTruth(station: Station, cam: CameraItem, opts: TruthOptions = {}): TruthObject[] {
  const K = cameraIntrinsics(cam);
  const camWorld = cam.poseAbs();
  const inv = invert(camWorld);
  const maxRange = opts.maxRange ?? cam.far;
  const out: TruthObject[] = [];
  if (opts.fruit !== false) {
    for (const d of detectFruit(station, cam, { onlyRipe: false, maxRange })) {
      // the box is clipped to the image like every detector output (and the ID-buffer boxes of objects)
      const x0 = Math.max(0, d.u - d.w / 2), y0 = Math.max(0, d.v - d.h / 2), x1 = Math.min(K.width, d.u + d.w / 2), y1 = Math.min(K.height, d.v + d.h / 2);
      if (x1 <= x0 || y1 <= y0) continue;
      out.push({ id: `fruit:${d.rowId}:${d.p.map((v) => v.toFixed(0)).join(',')}`, cls: cropClass(station, d.rowId), box: { x: x0, y: y0, w: x1 - x0, h: y1 - y0 }, p: d.p, z: d.range, size: d.fruit.d, visible: true, attrs: { ripe: d.ripe >= 0.5 ? 1 : 0, ripeness: Number(d.ripe.toFixed(2)) }, keypoints: [[d.u, d.v - d.h / 2]] });
    }
  }
  if (opts.trunks !== false) {
    for (const f of station.itemsOfType<FieldItem>(ItemType.FIELD)) {
      const abs = f.poseAbs();
      for (const row of f.rows()) for (const p of row.plants) {
        const [lx, ly] = plantPosition(row, p);
        const wx = abs[0] * lx + abs[4] * ly + abs[12], wy = abs[1] * lx + abs[5] * ly + abs[13], wz = abs[14];
        const l = transformPoint(inv, [wx, wy, wz + f.crop.canopyBase / 2]);
        if (l[2] < cam.near || l[2] > maxRange) continue;
        const r = 60, h = f.crop.canopyBase;
        const pb = projectBox(K, inv, [[wx - r, wy - r, wz], [wx + r, wy + r, wz], [wx - r, wy + r, wz + h], [wx + r, wy - r, wz + h], [wx - r, wy - r, wz + h], [wx + r, wy + r, wz + h], [wx - r, wy + r, wz], [wx + r, wy - r, wz]]);
        if (!pb) continue;
        out.push({ id: `trunk:${row.id}:${p.s.toFixed(0)}`, cls: 'trunk', box: pb.box, p: [wx, wy, wz + h / 2], z: l[2], size: 120, visible: true });
        const cw = p.width, ch = p.height, cz = wz + f.crop.canopyBase;
        const cb = projectBox(K, inv, [[wx - cw / 2, wy - cw / 2, cz], [wx + cw / 2, wy + cw / 2, cz], [wx - cw / 2, wy + cw / 2, cz + ch], [wx + cw / 2, wy - cw / 2, cz + ch], [wx - cw / 2, wy - cw / 2, cz + ch], [wx + cw / 2, wy + cw / 2, cz + ch], [wx - cw / 2, wy + cw / 2, cz], [wx + cw / 2, wy - cw / 2, cz]]);
        if (cb) out.push({ id: `tree:${row.id}:${p.s.toFixed(0)}`, cls: 'tree', box: cb.box, p: [wx, wy, cz + ch / 2], z: cb.z, size: Math.max(cw, ch), visible: true, attrs: { health: p.health } });
      }
    }
  }
  const wantObjects = opts.objects !== false, wantVehicles = opts.vehicles !== false;
  if (wantObjects || wantVehicles) {
    const render = opts.render && opts.render.width === Math.round(K.width) && opts.render.height === Math.round(K.height) ? opts.render : renderCamera(station, cam, { assets: opts.assets });
    const regions = regionsFromRender(render);
    // software meshes per key: unoccluded silhouette area (same tessellation as the render)
    let byKey: Map<string, SceneMesh[]> | null = null;
    const meshesOf = (key: string) => {
      if (!byKey) { byKey = new Map(); for (const m of render.meshes ?? sceneMeshes(station, { assets: opts.assets, fields: false })) { if (m.kind === 'field' || m.kind === 'ground') continue; const l = byKey.get(m.key); if (l) l.push(m); else byKey.set(m.key, [m]); } }
      return byKey.get(key) ?? [];
    };
    const minPx = opts.minPixels ?? 20, minVis = opts.minVisibleFraction ?? 0.5;
    const pixelTruth = (key: string): { box: { x: number; y: number; w: number; h: number }; pixels: number; vf: number; mask?: TruthObject['mask'] } | null => {
      const ms = meshesOf(key);
      if (!ms.length) return null;
      const full = projectedBounds(ms, K, camWorld, cam.near, cam.far);
      if (!full) return null;
      const own = silhouettePixels(ms, K, camWorld, cam.near, cam.far, full);
      const reg = regions.get(key);
      const pixels = reg?.pixels ?? 0;
      return { box: reg ? reg.box : full, pixels, vf: own ? Math.min(1, pixels / own) : 0, mask: reg?.mask };
    };
    if (wantObjects) for (const o of station.itemsOfType<SceneObject>(ItemType.OBJECT)) {
      if (o instanceof Tool) continue;
      const lb = objectLocalBox(o, opts.assets);
      if (!lb) continue;
      const pose = multiply(o.poseAbs(), transl(lb.centre[0], lb.centre[1], lb.centre[2]));
      const c: Vec3 = [pose[12], pose[13], pose[14]];
      const l = transformPoint(inv, c);
      if (l[2] < cam.near || l[2] > maxRange) continue;
      const pt = pixelTruth(`object:${o.id}`);
      if (!pt) continue;
      out.push({ id: `object:${o.id}`, cls: objectClass(o), box: pt.box, p: c, z: l[2], size: Math.max(...lb.dims), visible: pt.pixels >= minPx && pt.vf >= minVis, pose: Array.from(pose), dims: lb.dims, mask: pt.mask, pixels: pt.pixels, visibleFraction: Number(pt.vf.toFixed(3)) });
    }
    if (wantVehicles) for (const r of station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)) {
      const [L, , H] = r.kin.footprint;
      const pa = r.poseAbs();
      const c: Vec3 = [pa[12], pa[13], pa[14] + H / 2];
      const l = transformPoint(inv, c);
      if (l[2] < cam.near || l[2] > maxRange) continue;
      const pt = pixelTruth(`vehicle:${r.id}`);
      if (!pt) continue;
      const [, W] = r.kin.footprint;
      const vp = Array.from(pa); vp[12] = c[0]; vp[13] = c[1]; vp[14] = c[2];
      out.push({ id: `vehicle:${r.id}`, cls: 'vehicle', box: pt.box, p: c, z: l[2], size: L, visible: pt.pixels >= minPx && pt.vf >= minVis, pose: vp, dims: [L, W, H], mask: pt.mask, pixels: pt.pixels, visibleFraction: Number(pt.vf.toFixed(3)) });
    }
  }
  // fruit / trunks: an entry whose box lies mostly inside a nearer entry's box (≥70 %) of another class is hidden
  // (objects and vehicles already have pixel-true visibility from the ID buffer)
  out.sort((a, b) => a.z - b.z);
  for (let i = 0; i < out.length; i++) {
    const a = out[i];
    if (a.cls === 'tree' || a.pixels !== undefined) continue;
    for (let j = 0; j < i; j++) {
      const b = out[j];
      if (b.cls === 'tree' || b.z >= a.z || (b.pixels !== undefined && !b.visible)) continue;
      const ix = Math.max(0, Math.min(a.box.x + a.box.w, b.box.x + b.box.w) - Math.max(a.box.x, b.box.x)), iy = Math.max(0, Math.min(a.box.y + a.box.h, b.box.y + b.box.h) - Math.max(a.box.y, b.box.y));
      if (ix * iy > 0.7 * a.box.w * a.box.h && a.cls !== b.cls) { a.visible = false; break; }
    }
  }
  return out;
}

// ---------------------------------------------------------------------------------------------
// Runtime
// ---------------------------------------------------------------------------------------------

export interface ModelContext extends BaseModelContext {
  /** Called after every pipeline run (webhooks, ROS publishers). */
  onOutput?: (out: PipelineOutput, cam: CameraItem) => void;
  /** Mesh assets for the software render (headless). */
  assets?: AssetStore | null;
}

/** Detection quality vs truth (simulation). The position error covers true positives; false positives that still got a 3D
 *  position (they could become pick targets) are counted separately in `fp3d` — they have no truth to measure against. */
export interface PipelineStats { frames: number; tp: number; fp: number; fn: number; posErrSum: number; posErrN: number; fps: number; lastLatencyMs: number; fp3d: number }

export interface CloudAnalysis { cloud: PointCloud; ground: PointCloud; objects: PointCloud; clusters: Cluster[]; rows: Line2D[]; classes: string[] }

export interface VisionTarget {
  name: string; cls: string;
  /** World poses (mm): TCP on the grasp face (tool Z into the part) and the approach above it. */
  grasp: Mat4; approach: Mat4;
  /** Object centre estimate (world, mm) and its camera depth. */
  p: Vec3; z: number; sigmaMm: number; trackId?: number;
  /** Detector confidence of the detection the target was computed on. */
  score?: number;
  /** Station object id when known (simulation truth or the object under the grasp). */
  objectId?: string;
  surface?: Box2D['surface'];
}

export interface PipelineOutput {
  frame: VisionFrame;
  results: Partial<Record<VisionTask, VisionResult>>;
  /** Final detections after tracking + 3D localisation. */
  detections: Box2D[];
  cloud?: CloudAnalysis;
  /** Targets computed (world poses, mm) — grasp + approach per detection with a 3D position. */
  targets: VisionTarget[];
  vla?: { action: number[]; tcp?: Mat4; gripper?: number; text?: string };
  text?: string;
  warnings: string[];
}

export interface CaptureOptions {
  rgba?: Uint8ClampedArray; dataUrl?: string; time?: number; proprio?: VisionFrame['proprio'];
  lidar?: { channels?: number; hres?: number; range?: number };
  /** Pixel-true buffers of this camera at its resolution (browser: SceneRenderer.renderCameraBuffers). */
  render?: CameraRender;
}

function sensorOf(cfg: VisionStackConfig) { return VISION_SENSORS.find((x) => x.id === cfg.sensor); }

export class VisionRuntime {
  models = new Map<VisionTask, VisionModel>();
  tracker = new ByteTracker({ minHits: 1 });
  stats: PipelineStats = { frames: 0, tp: 0, fp: 0, fn: 0, posErrSum: 0, posErrN: 0, fps: 0, lastLatencyMs: 0, fp3d: 0 };
  last: PipelineOutput | null = null;
  private lastTime = 0;
  private seedCounter = 0;
  constructor(readonly cam: CameraItem, public cfg: VisionStackConfig, readonly ctx: ModelContext = {}) {}

  model(task: VisionTask): VisionModel {
    let m = this.models.get(task);
    if (!m) { m = createModel(this.cfg, task, this.ctx); this.models.set(task, m); }
    return m;
  }
  reset(): void { for (const m of this.models.values()) m.dispose?.(); this.models.clear(); this.tracker.reset(); this.stats = { frames: 0, tp: 0, fp: 0, fn: 0, posErrSum: 0, posErrN: 0, fps: 0, lastLatencyMs: 0, fp3d: 0 }; this.last = null; }

  /** Depth modalities (stereo / RGB-D / ToF) always produce a metric depth image. */
  get depthModality(): boolean { const m = this.cfg.modality; return m === 'stereo' || m === 'rgbd' || m === 'tof'; }

  /** Build the frame: pixel-true render → ground truth (simulation) + sensor depth / cloud + optional rendered pixels. */
  capture(station: Station, opts: CaptureOptions = {}): VisionFrame {
    const cam = this.cam, cfg = this.cfg;
    const K = cameraIntrinsics(cam);
    const camWorld = cam.poseAbs();
    const frame: VisionFrame = { width: cam.width, height: cam.height, K, camWorld, time: opts.time ?? station.simTime ?? 0, rgba: opts.rgba, dataUrl: opts.dataUrl, proprio: opts.proprio };
    const m = cfg.modality;
    const wantTruth = cfg.simulate || cfg.runtime === 'simulated';
    const okRender = (r?: CameraRender) => r && r.width === cam.width && r.height === cam.height ? r : undefined;
    let render = okRender(opts.render);
    if (!render && (wantTruth || this.depthModality) && m !== 'lidar3d') render = renderCamera(station, cam, { ground: this.depthModality, assets: this.ctx.assets });
    if (render) frame.render = render;
    if (wantTruth) frame.truth = captureTruth(station, cam, { maxRange: Math.min(cam.far, cfg.workingDistance * 1000 * 4 + 2000), render, assets: this.ctx.assets });
    const needCloud = cfg.tasks.some((t) => t === 'cloud_detect' || t === 'cloud_segment' || t === 'measure');
    if (m === 'lidar3d') {
      const s = sensorOf(cfg);
      const [v0, v1] = s ? [-s.vfov / 2, s.vfov / 2] : [-15, 15];
      const { cloud } = simulateLidar3D(station, camWorld, { channels: opts.lidar?.channels ?? Math.min(32, s?.channels ?? 16), vfov: [v0, v1], hfov: s?.hfov ?? 360, hres: opts.lidar?.hres ?? 1, range: opts.lidar?.range ?? Math.min(cam.far, 40000), noise: cfg.simulate ? (s?.depthError ? s.depthError.a * 1000 : 20) : 0, fruit: false });
      frame.cloud = cloud;
    } else if (this.depthModality && render) {
      frame.depth = this.sensorDepth(render.depth, frame.K);
      if (needCloud) frame.cloud = depthToCloud(frame.depth, K, camWorld, Math.max(2, Math.round(K.width / 160)));
    }
    return frame;
  }

  /** Render depth → sensor depth: catalogue range limits and, with `simulate`, per-pixel Gaussian noise σ(z) (seeded). */
  sensorDepth(depth: Float32Array, K: Intrinsics): Float32Array {
    const cfg = this.cfg, s = sensorOf(cfg);
    const out = new Float32Array(depth.length);
    const [r0, r1] = s?.depthRange ? [s.depthRange[0] * 1000, s.depthRange[1] * 1000] : [0, Infinity];
    const sigma = (z: number) => depthSigmaMm(cfg.sensor, cfg.modality, z, { baselineMm: (s?.baseline ?? 0.1) * 1000, fxPx: K.fx });
    // N(0,1) samples from a seeded table (fast for full-resolution frames), offset per frame
    const G = gaussTable();
    let gi = (((cfg.seed ?? 1) * 92821 + this.stats.frames * 7919) >>> 0) % G.length;
    for (let i = 0; i < depth.length; i++) {
      const z = depth[i];
      if (!(z > 0) || z < r0 || z > r1) { out[i] = 0; continue; }
      if (!cfg.simulate) { out[i] = z; continue; }
      out[i] = Math.max(1, z + sigma(z) * G[gi]);
      gi = gi + 7 >= G.length ? (gi + 7) - G.length : gi + 7;
    }
    return out;
  }

  /** Run every configured task on a frame. */
  async run(station: Station, frame: VisionFrame, opts: { prompt?: string; instruction?: string; tcp?: Mat4; refAxis?: Vec3 } = {}): Promise<PipelineOutput> {
    const cfg = this.cfg;
    const t0 = typeof performance !== 'undefined' ? performance.now() : Date.now();
    const results: Partial<Record<VisionTask, VisionResult>> = {};
    const warnings: string[] = [];
    const run = async (task: VisionTask, prompt?: string) => {
      try { const r = await this.model(task).run(frame, { task, classes: cfg.classes, confidence: cfg.confidence, iou: cfg.iou, prompt, seed: (cfg.seed ?? 1) + this.seedCounter++, simulate: cfg.simulate }); results[task] = r; return r; }
      catch (e) { warnings.push(`${task}: ${(e as Error).message}`); return null; }
    };
    const imageTasks = cfg.tasks.filter((t) => !['cloud_detect', 'cloud_segment', 'follow', 'track', 'pose', 'grasp', 'measure', 'depth', 'vlm_query', 'vla_policy'].includes(t));
    const needsBoxes = cfg.tasks.some((t) => ['track', 'follow', 'pose', 'grasp', 'measure'].includes(t));
    if (needsBoxes && !imageTasks.includes('detect') && !imageTasks.includes('segment') && cfg.modality !== 'lidar3d') imageTasks.unshift('detect');
    let boxes: Box2D[] = [];
    for (const t of imageTasks) {
      const r = await run(t);
      if (!r) continue;
      if (t === 'segment' || t === 'keypoints') boxes = mergeBoxes(boxes, r.boxes); else if (t === 'detect') boxes = mergeBoxes(r.boxes, boxes);
    }
    // classification of crops: attach the top label as an attribute when a classifier is configured
    if (cfg.tasks.includes('classify') && cfg.models.classify && !imageTasks.includes('classify')) await run('classify');
    if (cfg.models.classify && frame.truth) {
      // a dedicated classifier re-labels the per-detection attributes with its own accuracy (simulation, noise on)
      const acc = VISION_MODELS.find((m) => m.id === cfg.models.classify)?.accuracy ?? 0.7;
      const rndC = makeRng((cfg.seed ?? 1) * 17 + this.stats.frames);
      const byId = new Map(frame.truth.map((t) => [t.id, t]));
      for (const b of boxes) { const t = b.truthId ? byId.get(b.truthId) : undefined; if (t?.attrs && typeof t.attrs.ripe === 'number') { b.attr = { ...(b.attr ?? {}), ripe: cfg.simulate && rndC() < (1 - acc) * 0.5 ? 1 - t.attrs.ripe : t.attrs.ripe }; } }
    }
    // tracking
    const dt = this.lastTime ? Math.max(1e-3, frame.time - this.lastTime) : 1 / 15;
    this.lastTime = frame.time;
    if (cfg.tasks.includes('track') || cfg.tasks.includes('follow')) boxes = this.tracker.update(boxes, dt);
    // 3D localisation
    const s = sensorOf(cfg);
    const sigma = (z: number) => depthSigmaMm(cfg.sensor, cfg.modality, z, { baselineMm: (s?.baseline ?? 0.1) * 1000, fxPx: frame.K.fx });
    const truthById = new Map((frame.truth ?? []).map((t) => [t.id, t]));
    const rnd = makeRng((cfg.seed ?? 1) * 31 + this.stats.frames);
    const noise = cfg.simulate;
    const poseSpec = VISION_MODELS.find((m) => m.id === (cfg.models.pose ?? cfg.models.grasp));
    const modelBased6D = poseSpec?.family === 'pose6d';
    const refAxis = opts.refAxis ?? [1, 0, 0];
    const wantsPose = cfg.tasks.some((t) => ['pose', 'grasp', 'measure', 'follow'].includes(t));
    const camInv = invert(frame.camWorld);
    const depthOf = (p: Vec3) => transformPoint(camInv, p)[2];
    if (wantsPose) for (const b of boxes) {
      if (b.p && b.z) continue;
      const t = b.truthId ? truthById.get(b.truthId) : undefined;
      // object model: truth box (simulation), the configured class model, or a size prior (sphere-like: fruit)
      const prior = t?.size ?? cfg.objectSizeMm;
      const dims: Vec3 | null = t?.dims ?? cfg.objectDims?.[b.cls] ?? (prior ? [prior, prior, prior] : null);
      const round = !t?.dims && !cfg.objectDims?.[b.cls] && !!prior; // only a size prior: a round object (fruit)
      let est: SurfaceEstimate | null = null;
      // sensor path: the depth image (unless a model-based 6D estimator is simulated from truth)
      if (frame.depth && !(modelBased6D && t?.pose)) {
        est = localizeFromDepth({ depth: frame.depth, width: frame.width, height: frame.height, K: frame.K, camWorld: frame.camWorld }, b, { mask: b.mask ?? null, dims, round, sigma: noise ? sigma : undefined, refAxis, seed: (cfg.seed ?? 1) + this.stats.frames });
        if (est) { b.surface = { point: est.point, normal: est.normal, xAxis: est.xAxis, size: est.size, symmetryDeg: est.symmetryDeg, points: est.points, source: 'depth' }; b.p = est.centroid ?? est.point; }
      }
      // truth + noise: model-based 6D estimators, or sensors without depth in simulation
      if (!b.p && t && cfg.modality !== 'mono') {
        const zt = t.z, sz = noise ? sigma(zt) : 0, sl = noise ? (zt / frame.K.fx) * 0.5 : 0;
        const ray = normalize(sub(t.p, [frame.camWorld[12], frame.camWorld[13], frame.camWorld[14]]));
        const ex = [frame.camWorld[0], frame.camWorld[1], frame.camWorld[2]] as Vec3, ey = [frame.camWorld[4], frame.camWorld[5], frame.camWorld[6]] as Vec3;
        const n1 = gauss(rnd) * sz, n2 = gauss(rnd) * sl, n3 = gauss(rnd) * sl;
        const shift: Vec3 = [ray[0] * n1 + ex[0] * n2 + ey[0] * n3, ray[1] * n1 + ex[1] * n2 + ey[1] * n3, ray[2] * n1 + ex[2] * n2 + ey[2] * n3];
        b.p = [t.p[0] + shift[0], t.p[1] + shift[1], t.p[2] + shift[2]];
        if (t.pose && t.dims) {
          const sf = surfaceFromTruth(t.pose, t.dims, { refAxis });
          const yawNoise = noise ? gauss(rnd) * (1 - (poseSpec?.accuracy ?? 0.8)) * 10 * Math.PI / 180 : 0;
          const c = Math.cos(yawNoise), sn = Math.sin(yawNoise), yv: Vec3 = [sf.normal[1] * sf.xAxis[2] - sf.normal[2] * sf.xAxis[1], sf.normal[2] * sf.xAxis[0] - sf.normal[0] * sf.xAxis[2], sf.normal[0] * sf.xAxis[1] - sf.normal[1] * sf.xAxis[0]];
          b.surface = { point: [sf.point[0] + shift[0], sf.point[1] + shift[1], sf.point[2] + shift[2]], normal: sf.normal, xAxis: [sf.xAxis[0] * c + yv[0] * sn, sf.xAxis[1] * c + yv[1] * sn, sf.xAxis[2] * c + yv[2] * sn], size: sf.size, symmetryDeg: sf.symmetryDeg, points: 0, source: 'truth' };
        }
      }
      // mono: range from the known size (catalogue model or prior)
      if (!b.p && cfg.modality === 'mono') {
        const size = cfg.objectSizeMm ?? t?.size;
        if (size) {
          const z = rangeFromSize(frame.K, b.w, size) * (noise ? 1 + 0.05 * gauss(rnd) : 1);
          if (Number.isFinite(z)) b.p = transformPoint(frame.camWorld, backproject(frame.K, b.x + b.w / 2, b.y + b.h / 2, z));
        }
      }
      if (!b.p) continue;
      b.z = depthOf(b.p);
      if (t) { const e = Math.hypot(b.p[0] - t.p[0], b.p[1] - t.p[1], b.p[2] - t.p[2]); this.stats.posErrSum += e; this.stats.posErrN++; }
    }
    // point clouds
    let cloud: CloudAnalysis | undefined;
    if (frame.cloud && cfg.tasks.some((t) => t === 'cloud_detect' || t === 'cloud_segment' || t === 'measure')) {
      cloud = analyseCloud(frame.cloud, { voxel: cfg.modality === 'lidar3d' ? 150 : 40, tol: cfg.modality === 'lidar3d' ? 500 : 120, minPts: cfg.modality === 'lidar3d' ? 6 : 15 });
      if (cfg.tasks.includes('cloud_detect')) {
        const rc: VisionResult = { task: 'cloud_detect', model: cfg.models.cloud_detect ?? 'pcl_pipeline', boxes: [], latencyMs: 0 };
        for (const cl of cloud.clusters) {
          const l = transformPoint(camInv, cl.centroid);
          const uv = project(frame.K, l);
          const b: Box2D = { x: uv ? uv[0] - 10 : 0, y: uv ? uv[1] - 10 : 0, w: 20, h: 20, score: Math.min(0.99, cl.count / 50), cls: cl.shape, p: cl.centroid, z: Math.hypot(...l), attr: { sizeX: cl.size[0], sizeY: cl.size[1], sizeZ: cl.size[2], points: cl.count } };
          rc.boxes.push(b);
        }
        results.cloud_detect = rc;
        if (cfg.modality === 'lidar3d') boxes = mergeBoxes(boxes, rc.boxes);
      }
      if (cfg.tasks.includes('cloud_segment')) results.cloud_segment = { task: 'cloud_segment', model: cfg.models.cloud_segment ?? 'patchwork', boxes: [], latencyMs: 0, labels: [{ cls: 'ground', score: pointCount(cloud.ground) }, { cls: 'objects', score: pointCount(cloud.objects) }, ...cloud.rows.map((r, i) => ({ cls: `row ${i + 1}`, score: r.n }))] };
    }
    // VLM / VLA
    let text: string | undefined, vla: PipelineOutput['vla'] | undefined;
    if (cfg.tasks.includes('vlm_query')) { const r = await run('vlm_query', opts.prompt ?? cfg.vlm?.prompt); if (r) { text = r.text; if (r.boxes.length && !boxes.length) boxes = r.boxes; } }
    if (cfg.tasks.includes('vla_policy')) {
      const r = await run('vla_policy', opts.instruction ?? cfg.vla?.instruction);
      // the action is expressed in the camera frame (x right, y down, z forward): rotate it into the TCP motion
      if (r?.action) { const a = applyVlaAction(opts.tcp ?? frame.camWorld, r.action, { scaleMm: cfg.vla?.actionScaleMm, scaleDeg: cfg.vla?.actionScaleDeg, axes: frame.camWorld }); vla = { action: r.action, tcp: opts.tcp ? a.pose : undefined, gripper: a.gripper, text: r.text }; }
    }
    // targets
    const targets: VisionTarget[] = [];
    if (cfg.tasks.some((t) => t === 'pose' || t === 'grasp')) {
      let i = 0;
      for (const b of boxes) {
        if (!b.p || !b.z) continue;
        const { grasp, approach } = b.surface ? graspFromSurface(b.surface, { approachMm: cfg.approachMm ?? 100, standoffMm: cfg.graspStandoffMm ?? 2 }) : targetPoseFromPoint(b.p, frame.camWorld, { approachMm: cfg.approachMm ?? 100 });
        const objectId = b.truthId?.startsWith('object:') ? b.truthId.slice(7) : undefined;
        targets.push({ name: `${b.cls}${b.id !== undefined ? `#${b.id}` : `_${++i}`}`, cls: b.cls, grasp, approach, p: b.p, z: b.z, sigmaMm: sigma(b.z), trackId: b.id, score: b.score, objectId, surface: b.surface });
      }
    }
    // stats vs truth
    if (frame.truth) {
      const wanted = new Set(cfg.classes.map((c) => c.toLowerCase()));
      const visible = frame.truth.filter((t) => t.visible && (wanted.size === 0 || wanted.has(t.cls.toLowerCase())) && Math.min(t.box.w, t.box.h) >= 10); // objects under ~10 px are not expected from a 640-px detector
      const matched = new Set(boxes.map((b) => b.truthId).filter(Boolean));
      this.stats.tp += visible.filter((t) => matched.has(t.id)).length;
      this.stats.fn += visible.filter((t) => !matched.has(t.id)).length;
      this.stats.fp += boxes.filter((b) => !b.truthId).length;
      this.stats.fp3d += boxes.filter((b) => !b.truthId && b.p).length;
    }
    const lat = (typeof performance !== 'undefined' ? performance.now() : Date.now()) - t0;
    this.stats.frames++; this.stats.lastLatencyMs = lat; this.stats.fps = lat > 0 ? Math.min(1000 / lat, 1 / dt) : 0;
    const out: PipelineOutput = { frame, results, detections: boxes, cloud, targets, vla, text, warnings };
    this.last = out;
    this.cam.setParam('visionLast', visionSummary(this.cam, out) as any);
    try { this.ctx.onOutput?.(out, this.cam); } catch (e) { warnings.push(`onOutput: ${(e as Error).message}`); }
    return out;
  }

  /** capture + run in one call. */
  async step(station: Station, opts: CaptureOptions & { prompt?: string; instruction?: string; tcp?: Mat4; refAxis?: Vec3 } = {}): Promise<PipelineOutput> {
    return this.run(station, this.capture(station, opts), opts);
  }
}

function mergeBoxes(primary: Box2D[], extra: Box2D[]): Box2D[] {
  const out = [...primary];
  for (const e of extra) {
    const m = out.find((p) => p.cls === e.cls && overlap(p, e) > 0.5);
    if (m) { if (e.mask) m.mask = e.mask; if (e.keypoints) m.keypoints = e.keypoints; if (e.p) m.p = e.p; if (e.z) m.z = e.z; }
    else out.push(e);
  }
  return out;
}
function overlap(a: Box2D, b: Box2D): number {
  const ix = Math.max(0, Math.min(a.x + a.w, b.x + b.w) - Math.max(a.x, b.x)), iy = Math.max(0, Math.min(a.y + a.h, b.y + b.h) - Math.max(a.y, b.y));
  const inter = ix * iy; return inter / Math.max(1e-6, a.w * a.h + b.w * b.h - inter);
}
function makeRng(seed: number) { let s = seed >>> 0 || 1; return () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; }; }
function gauss(r: () => number) { return Math.sqrt(-2 * Math.log(r() + 1e-12)) * Math.cos(2 * Math.PI * r()); }
let GTAB: Float32Array | null = null;
/** 65 521 seeded N(0,1) samples (prime length: the stride-7 walk visits every entry). */
function gaussTable(): Float32Array { if (!GTAB) { const r = makeRng(424242); GTAB = new Float32Array(65521); for (let i = 0; i < GTAB.length; i++) GTAB[i] = gauss(r); } return GTAB; }

// ---------------------------------------------------------------------------------------------
// Point-cloud analysis
// ---------------------------------------------------------------------------------------------

export function analyseCloud(cloud: PointCloud, opts: { voxel?: number; tol?: number; minPts?: number; groundThresh?: number; trunkSlice?: [number, number] } = {}): CloudAnalysis {
  const ds = voxelDownsample(cloud, opts.voxel ?? 100);
  const plane = fitGroundPlane(ds, { thresh: opts.groundThresh ?? 120 });
  const { ground, objects } = plane ? removeGround(ds, plane, opts.groundThresh ?? 150) : { ground: { xyz: new Float32Array(0), frame: ds.frame } as PointCloud, objects: ds };
  const clusters = euclideanCluster(objects, opts.tol ?? 400, opts.minPts ?? 6);
  // trunk slice: points between the ground band and the canopy base cluster separately (canopies merge along a row)
  const [z0, z1] = opts.trunkSlice ?? [200, 900];
  const gz = (x: number, y: number) => (plane ? -(plane.normal[0] * x + plane.normal[1] * y + plane.d) / plane.normal[2] : 0);
  const slice = selectPoints(objects, (p) => { const h = objects.xyz[p * 3 + 2] - gz(objects.xyz[p * 3], objects.xyz[p * 3 + 1]); return h >= z0 && h <= z1; });
  const trunks = euclideanCluster(slice, Math.min(opts.tol ?? 400, 350), Math.max(2, Math.round((opts.minPts ?? 6) / 2))).filter((c) => Math.max(c.size[0], c.size[1]) < 1200);
  for (const t of trunks) t.shape = 'trunk';
  const all = [...clusters, ...trunks];
  all.forEach((c, i) => (c.id = i + 1));
  let rows = detectRows(trunks.length >= 3 ? trunks : clusters);
  // long "wall" clusters (merged canopies / hedgerows) are rows themselves
  for (const w of clusters.filter((c) => c.shape === 'wall')) {
    const pts = w.indices.map((i) => [objects.xyz[i * 3], objects.xyz[i * 3 + 1]] as [number, number]);
    const line = fitLine2D(pts);
    if (line && !rows.some((r) => Math.abs(-(line.point[0] - r.point[0]) * r.dir[1] + (line.point[1] - r.point[1]) * r.dir[0]) < 800 && Math.abs(line.dir[0] * r.dir[0] + line.dir[1] * r.dir[1]) > 0.9)) rows.push(line);
  }
  rows = rows.sort((a, b) => b.n - a.n);
  return { cloud: ds, ground, objects, clusters: all, rows, classes: [...new Set(all.map((c) => c.shape))] };
}

// ---------------------------------------------------------------------------------------------
// Acting on results
// ---------------------------------------------------------------------------------------------

/** Station object of the detected class under a grasp point (within `tol` mm of its world AABB), for targets without a truth id (never the table under a false positive). */
function objectNear(station: Station, p: Vec3, cls: string, tol = 25): string | undefined {
  let best: string | undefined, bd = tol;
  for (const m of sceneMeshes(station, { fields: false })) {
    if (m.kind !== 'object') continue;
    const it = station.findById(m.itemId);
    if (!it || objectClass(it) !== cls.toLowerCase()) continue;
    const d = Math.hypot(Math.max(m.min[0] - p[0], 0, p[0] - m.max[0]), Math.max(m.min[1] - p[1], 0, p[1] - m.max[1]), Math.max(m.min[2] - p[2], 0, p[2] - m.max[2]));
    if (d <= bd) { bd = d; best = m.itemId; }
  }
  return best;
}

export interface PickTarget {
  name: string; cls: string;
  /** World poses and the same relative to the robot base (what solveIK takes). */
  grasp: Mat4; approach: Mat4; graspBase: Mat4; approachBase: Mat4;
  joints?: number[]; approachJoints?: number[];
  reachable: boolean; collisionFree: boolean;
  /** Why the target is not feasible. */
  reason?: string;
  objectId?: string; trackId?: number;
}

/**
 * Feasibility of vision targets for a robot (no station change): IK of the grasp and the approach (base-relative),
 * then a collision check with the robot at the grasp joints against everything except the picked object.
 */
export function detectionsToPickTargets(station: Station, robot: Robot, targets: VisionTarget[], opts: { checkCollision?: boolean; max?: number } = {}): PickTarget[] {
  const baseInv = invert(robot.poseAbs());
  const out: PickTarget[] = [];
  const q0 = robot.joints();
  try {
    for (const t of targets.slice(0, opts.max ?? 50)) {
      const graspBase = multiply(baseInv, t.grasp), approachBase = multiply(baseInv, t.approach);
      const objectId = t.objectId ?? objectNear(station, t.surface?.point ?? t.p, t.cls);
      const ikG = robot.solveIK(graspBase, { seed: q0 });
      const ikA = ikG.ok ? robot.solveIK(approachBase, { seed: ikG.joints }) : ikG;
      const pt: PickTarget = { name: t.name, cls: t.cls, grasp: t.grasp, approach: t.approach, graspBase, approachBase, joints: ikG.ok ? ikG.joints : undefined, approachJoints: ikA.ok ? ikA.joints : undefined, reachable: ikG.ok && ikA.ok, collisionFree: false, objectId, trackId: t.trackId };
      if (!pt.reachable) pt.reason = ikG.ok ? 'approach out of reach' : 'grasp out of reach';
      else if (opts.checkCollision !== false) {
        const ignore: Array<[string, string]> = objectId ? [...robot.walk()].map((it) => [it.id, objectId] as [string, string]) : [];
        // contacts that already exist at the current pose (base on its pedestal, part on its table) are not caused by the pick
        const sig = (p: { a: { item: Item; part: string }; b: { item: Item; part: string } }) => [`${p.a.item.id}/${p.a.part}`, `${p.b.item.id}/${p.b.part}`].sort().join('|');
        robot.setJoints(q0);
        const baseline = new Set(checkRobotCollisions(station, robot, { ignore }).map(sig));
        const hits: string[] = [];
        for (const q of [ikG.joints, ikA.joints]) {
          robot.setJoints(q);
          for (const p of checkRobotCollisions(station, robot, { ignore })) if (!baseline.has(sig(p)) && p.a.linkIndex !== 0 && p.b.linkIndex !== 0) hits.push(`${p.a.item.name}${p.a.linkIndex !== undefined ? `[${p.a.part}]` : ''} × ${p.b.item.name}`);
        }
        pt.collisionFree = hits.length === 0;
        if (hits.length) pt.reason = `collision: ${[...new Set(hits)].slice(0, 3).join(', ')}`;
      } else pt.collisionFree = true;
      out.push(pt);
    }
  } finally { robot.setJoints(q0); }
  return out;
}

/**
 * Create (or refresh) a "Vision <camera>" frame with grasp/approach targets from a pipeline output. Targets are updated
 * in place by name (stable ids for programs), and `Pick` / `Pick approach` always hold the best feasible target (highest detector score), so a
 * program can MoveJ(Pick approach) → MoveL(Pick) after a VisionDetect call. With a robot (`robotId`) only targets the
 * robot reaches without collision are kept, unless `keepInfeasible`.
 */
export function createTargetsFromOutput(station: Station, cam: CameraItem, out: PipelineOutput, opts: { parent?: Item; max?: number; robotId?: string | null; frameName?: string; keepInfeasible?: boolean; checkCollision?: boolean; pickTargets?: boolean } = {}): Target[] {
  const robot = opts.robotId ? station.findById(opts.robotId) : null;
  // with a robot the frame lives in the robot's active reference frame (identity pose): a program on that robot moves to
  // the targets without a frame change (RoboDK applies a target's pose in the active reference frame)
  const robotFrame = robot instanceof Robot ? robot.activeFrame() : null;
  const parent = opts.parent ?? (robotFrame && robotFrame !== robot ? robotFrame : station);
  const name = opts.frameName ?? `Vision ${cam.name}`;
  let frame = (parent.children.find((c) => c instanceof Frame && c.name === name) ?? station.find(name, ItemType.FRAME)) as Frame | undefined;
  if (frame && frame.parent !== parent) { parent.addChild(frame); frame.setPose(transl(0, 0, 0)); } // targets are recomputed below
  if (!frame) { frame = parent.addChild(new Frame(name)); frame.setPose(transl(0, 0, 0)); }
  const inv = invert(frame.poseAbs());
  const list = out.targets.slice(0, opts.max ?? 50);
  const feas = robot instanceof Robot ? detectionsToPickTargets(station, robot, list, { checkCollision: opts.checkCollision }) : null;
  const old = new Map(frame.children.filter((c): c is Target => c instanceof Target).map((t) => [t.name, t]));
  const used = new Set<string>();
  const put = (tname: string, pose: Mat4, joints?: number[]): Target => {
    let t = old.get(tname);
    if (!t) { t = frame!.addChild(new Target(tname)); old.set(tname, t); }
    t.setPose(multiply(inv, pose)); t.robotId = opts.robotId ?? null; t.joints = joints ?? null;
    used.add(tname);
    return t;
  };
  const targets: Target[] = [];
  // Pick: the most confident feasible detection (ties: the nearest to the camera, the top of a pile), not simply the
  // first one in detector order
  let first: { t: VisionTarget; f?: PickTarget } | null = null;
  const better = (a: VisionTarget, b: VisionTarget) => (a.score ?? 0) > (b.score ?? 0) + 1e-9 || (Math.abs((a.score ?? 0) - (b.score ?? 0)) <= 1e-9 && a.z < b.z);
  list.forEach((t, k) => {
    const f = feas?.[k];
    if (f && !(f.reachable && f.collisionFree) && !opts.keepInfeasible) return;
    if ((!f || (f.reachable && f.collisionFree)) && (!first || better(t, (first as { t: VisionTarget }).t))) first = { t, f };
    const app = put(`${t.name} approach`, t.approach, f?.approachJoints);
    const g = put(t.name, t.grasp, f?.joints);
    g.setParam('vision', { cls: t.cls, z: t.z, sigmaMm: t.sigmaMm, trackId: t.trackId ?? null, objectId: t.objectId ?? f?.objectId ?? null, feasible: f ? f.reachable && f.collisionFree : null, reason: f?.reason ?? null } as any);
    targets.push(app, g);
  });
  if (opts.pickTargets !== false && first) {
    const fp = first as { t: VisionTarget; f?: PickTarget };
    put('Pick approach', fp.t.approach, fp.f?.approachJoints);
    put('Pick', fp.t.grasp, fp.f?.joints).setParam('vision', { cls: fp.t.cls, objectId: fp.t.objectId ?? fp.f?.objectId ?? null, source: fp.t.name } as any);
  }
  // targets of the previous run that this run did not produce (Pick / Pick approach stay: programs reference them)
  for (const [n, t] of old) if (!used.has(n) && n !== 'Pick' && n !== 'Pick approach') t.delete();
  return targets;
}

/** Joints for an absolute TCP pose (VLA / servo steps): Robot.solveIK takes poses relative to the robot base. */
export function solveTcpAbs(robot: Robot, tcpAbs: Mat4): { ok: boolean; joints: number[] } {
  const r = robot.solveIK(multiply(invert(robot.poseAbs()), tcpAbs));
  return { ok: r.ok, joints: r.joints };
}

/** Velocity command for a mobile base following a tracked box: keep it centred and at `desiredRange` (mm). */
export function followCommand(box: Box2D | null, K: Intrinsics, opts: { desiredRange?: number; maxSpeed?: number; maxYawRate?: number; kAng?: number; kLin?: number } = {}): { v: number; omega: number; state: 'tracking' | 'lost' | 'arrived' } {
  if (!box) return { v: 0, omega: 0, state: 'lost' };
  const desired = opts.desiredRange ?? 2000, maxV = opts.maxSpeed ?? 1000, maxW = opts.maxYawRate ?? 60;
  const bearing = Math.atan2((box.x + box.w / 2 - K.cx), K.fx); // rad, +right
  const omega = Math.max(-maxW, Math.min(maxW, -(opts.kAng ?? 1.5) * bearing * (180 / Math.PI)));
  const z = box.z ?? desired;
  const err = z - desired;
  const v = Math.abs(err) < 150 ? 0 : Math.max(-maxV * 0.3, Math.min(maxV, (opts.kLin ?? 1.5) * err));
  return { v, omega, state: Math.abs(err) < 150 && Math.abs(bearing) < 0.05 ? 'arrived' : 'tracking' };
}

/** Image-based visual servoing for an arm camera: TCP delta (camera frame, mm) to centre the box and reach `standoff`. */
export function servoDelta(box: Box2D | null, K: Intrinsics, opts: { standoff?: number; gain?: number; maxStep?: number } = {}): Vec3 | null {
  if (!box) return null;
  const z = box.z ?? 500, g = opts.gain ?? 0.5, maxStep = opts.maxStep ?? 30;
  const ex = ((box.x + box.w / 2 - K.cx) / K.fx) * z, ey = ((box.y + box.h / 2 - K.cy) / K.fy) * z, ez = z - (opts.standoff ?? 150);
  const clamp = (v: number) => Math.max(-maxStep, Math.min(maxStep, v * g));
  return [clamp(ex), clamp(ey), clamp(ez)];
}

/** Runtime registry per camera (kept off the item so it is not serialised). */
const RUNTIMES = new WeakMap<CameraItem, VisionRuntime>();
export function ensureVisionRuntime(cam: CameraItem, ctx: ModelContext = {}): VisionRuntime | null {
  const cfg = getVisionStack(cam);
  if (!cfg) { RUNTIMES.delete(cam); return null; }
  let rt = RUNTIMES.get(cam);
  if (!rt || rt.cfg !== cfg) { rt?.reset(); rt = new VisionRuntime(cam, cfg, ctx); RUNTIMES.set(cam, rt); }
  return rt;
}
export function visionRuntimeOf(cam: CameraItem): VisionRuntime | undefined { return RUNTIMES.get(cam); }

/** Result of one headless pipeline run (program instruction / API call). */
export interface VisionRunResult { found: number; targets: Target[]; output: PipelineOutput; picks?: PickTarget[]; warnings: string[] }

/**
 * Run a camera's pipeline once (headless, software render unless a GPU renderer is registered), optionally create the
 * targets in the station (frame "Vision <camera>"), and publish the count. Used by the VisionDetect program call and
 * the API. `found` counts feasible targets when a robot is given, else detections with a 3D position.
 */
export async function runVisionOnce(station: Station, cam: CameraItem, opts: { robot?: Robot | null; createTargets?: boolean; frameName?: string; ctx?: ModelContext; time?: number; refAxis?: Vec3 } = {}): Promise<VisionRunResult> {
  const rt = ensureVisionRuntime(cam, opts.ctx ?? {});
  if (!rt) throw new Error(`camera "${cam.name}" has no vision stack (params.visionStack)`);
  const refAxis = opts.refAxis ?? (opts.robot ? ([opts.robot.poseAbs()[0], opts.robot.poseAbs()[1], opts.robot.poseAbs()[2]] as Vec3) : undefined);
  const output = await rt.step(station, { time: opts.time ?? station.simTime ?? 0, refAxis });
  const picks = opts.robot ? detectionsToPickTargets(station, opts.robot, output.targets) : undefined;
  const targets = opts.createTargets === false ? [] : createTargetsFromOutput(station, cam, output, { robotId: opts.robot?.id ?? null, frameName: opts.frameName });
  const found = picks ? picks.filter((p) => p.reachable && p.collisionFree).length : output.targets.length || output.detections.filter((d) => d.p).length;
  return { found, targets, output, picks, warnings: output.warnings };
}

/** False positives of the run statistics; those with a 3D position are named, since they could become pick targets. */
export function fpText(st: { fp: number; fp3d?: number }): string {
  if (!st.fp) return ' · 0 false positives';
  return ` · ${st.fp} false positive${st.fp > 1 ? 's' : ''}${st.fp3d ? ` (${st.fp3d} with a 3D position — check the targets)` : ' (none got a 3D position)'}`;
}

/** Human-readable summary line for logs / the Vision tab. */
export function summarize(out: PipelineOutput, cfg: VisionStackConfig): string {
  const parts: string[] = [];
  const byCls = new Map<string, number>();
  for (const b of out.detections) byCls.set(b.cls, (byCls.get(b.cls) ?? 0) + 1);
  parts.push([...byCls.entries()].map(([c, n]) => `${n} ${c}`).join(', ') || 'no detections');
  if (out.targets.length) parts.push(`${out.targets.length} targets (σ ${out.targets[0].sigmaMm.toFixed(0)} mm)`);
  if (out.cloud) parts.push(`cloud ${pointCount(out.cloud.cloud)} pts → ${out.cloud.clusters.length} clusters, ${out.cloud.rows.length} rows`);
  if (out.text) parts.push(`VLM: ${out.text.slice(0, 120)}`);
  if (out.vla) parts.push(`VLA action [${out.vla.action.map((v) => v.toFixed(2)).join(' ')}] ${out.vla.text ?? ''}`);
  const models = Object.values(cfg.models).map((id) => VISION_MODELS.find((m) => m.id === id)?.name.split(' ')[0]).filter(Boolean);
  return `${models.join('+')}: ${parts.join(' · ')}${out.warnings.length ? ` ⚠ ${out.warnings.join('; ')}` : ''}`;
}

void dot;
