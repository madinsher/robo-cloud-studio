/**
 * Vision pipeline runtime: ties a station camera + `VisionStackConfig` to the pluggable models.
 *
 *  capture  → ground truth of what the camera sees (fruit, trunks, objects, vehicles) with occlusion, plus a
 *             depth image / point cloud when the modality provides one (simulated sensors), or the pixels the
 *             browser rendered for real models;
 *  infer    → one model per task (detect / segment / keypoints / classify / VLM / VLA);
 *  track    → ByteTrack ids and velocities;
 *  locate   → 3D position per detection: depth-centroid (stereo / RGB-D / ToF, with the sensor's error model),
 *             size prior (mono), LiDAR clusters (3D boxes), then camera → world → robot targets (hand-eye);
 *  act      → targets in the station (grasp + approach), follow commands for a mobile base, TCP deltas for VLA.
 *
 * Everything except pixel rendering runs in Node, so fleets and picking cycles can be validated in tests.
 */
import { Station, Camera as CameraItem, ItemType, SceneObject, Frame, Target, Item } from '../core/items/item';
import { Mat4, Vec3, invert, multiply, transformPoint, transl } from '../core/math/pose';
import { detectFruit } from '../agri/vision';
import { FieldItem } from '../agri/items';
import { plantPosition } from '../agri/orchard';
import type { MobileRobot } from '../mobile/items';
import { VisionStackConfig, VisionTask, VISION_SENSORS, VISION_MODELS, getVisionStack } from './stack';
import { Intrinsics, intrinsicsFromCamera, project, backproject, depthSigmaMm, rangeFromSize, targetPoseFromPoint, medianDepthInBox } from './camera_model';
import { VisionModel, VisionFrame, VisionResult, Box2D, TruthObject, ByteTracker, createModel, applyVlaAction, ModelContext as BaseModelContext } from './models';
import { visionSummary } from '../ros/publishers';
import { PointCloud, simulateLidar3D, voxelDownsample, fitGroundPlane, removeGround, euclideanCluster, Cluster, detectRows, Line2D, simulateDepthCamera, pointCount, selectPoints, fitLine2D } from './pointcloud';

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

/** What a camera sees, as ground truth (simulation). */
export function captureTruth(station: Station, cam: CameraItem, opts: { maxRange?: number; fruit?: boolean; trunks?: boolean; objects?: boolean; vehicles?: boolean } = {}): TruthObject[] {
  const K = intrinsicsFromCamera(cam);
  const camWorld = cam.poseAbs();
  const inv = invert(camWorld);
  const maxRange = opts.maxRange ?? cam.far;
  const out: TruthObject[] = [];
  if (opts.fruit !== false) {
    for (const d of detectFruit(station, cam, { onlyRipe: false, maxRange })) {
      out.push({ id: `fruit:${d.rowId}:${d.p.map((v) => v.toFixed(0)).join(',')}`, cls: cropClass(station, d.rowId), box: { x: d.u - d.w / 2, y: d.v - d.h / 2, w: d.w, h: d.h }, p: d.p, z: d.range, size: d.fruit.d, visible: true, attrs: { ripe: d.ripe >= 0.5 ? 1 : 0, ripeness: Number(d.ripe.toFixed(2)) }, keypoints: [[d.u, d.v - d.h / 2]] });
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
  if (opts.objects !== false) {
    for (const o of station.itemsOfType<SceneObject>(ItemType.OBJECT)) {
      const pose = o.poseAbs();
      let half: Vec3 | null = null, centre: Vec3 = [0, 0, 0];
      if (o.bbox) { half = [(o.bbox.max[0] - o.bbox.min[0]) / 2, (o.bbox.max[1] - o.bbox.min[1]) / 2, (o.bbox.max[2] - o.bbox.min[2]) / 2]; centre = [(o.bbox.max[0] + o.bbox.min[0]) / 2, (o.bbox.max[1] + o.bbox.min[1]) / 2, (o.bbox.max[2] + o.bbox.min[2]) / 2]; }
      else for (const g of o.geometry) { const pr = g.primitive; if (!pr) continue; half = pr.kind === 'box' ? [pr.size[0] / 2, pr.size[1] / 2, pr.size[2] / 2] : pr.kind === 'cylinder' ? [pr.radius, pr.radius, pr.length / 2] : pr.kind === 'sphere' ? [pr.radius, pr.radius, pr.radius] : pr.kind === 'cone' ? [pr.radius, pr.radius, pr.length / 2] : [pr.size[0] / 2, pr.size[1] / 2, 1]; if (g.origin && g.origin.length >= 3) centre = [g.origin[0], g.origin[1], g.origin[2]]; break; }
      if (!half) continue;
      const corners: Vec3[] = [];
      for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) corners.push(transformPoint(pose, [centre[0] + sx * half[0], centre[1] + sy * half[1], centre[2] + sz * half[2]]));
      const c = transformPoint(pose, centre);
      const l = transformPoint(inv, c);
      if (l[2] < cam.near || l[2] > maxRange) continue;
      const pb = projectBox(K, inv, corners);
      if (!pb) continue;
      out.push({ id: `object:${o.id}`, cls: o.name.toLowerCase().replace(/[\s_]+\d*$/, '').replace(/\d+$/, '') || 'object', box: pb.box, p: c, z: l[2], size: Math.max(...half) * 2, visible: true });
    }
  }
  if (opts.vehicles !== false) {
    for (const r of station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)) {
      const th = (r.state.theta * Math.PI) / 180, [L, W, H] = r.kin.footprint;
      const corners: Vec3[] = [];
      for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [0, 1]) { const lx = sx * L / 2, ly = sy * W / 2; corners.push([r.state.x + lx * Math.cos(th) - ly * Math.sin(th), r.state.y + lx * Math.sin(th) + ly * Math.cos(th), sz * H]); }
      const c: Vec3 = [r.state.x, r.state.y, H / 2];
      const l = transformPoint(inv, c);
      if (l[2] < cam.near || l[2] > maxRange) continue;
      const pb = projectBox(K, inv, corners);
      if (pb) out.push({ id: `vehicle:${r.id}`, cls: 'vehicle', box: pb.box, p: c, z: l[2], size: L, visible: true });
    }
  }
  // occlusion between non-fruit objects: an object whose centre lies inside a nearer object's box (≥70 % overlap) is hidden
  out.sort((a, b) => a.z - b.z);
  for (let i = 0; i < out.length; i++) {
    const a = out[i];
    if (a.cls === 'tree') continue; // canopies are translucent to detection of what is in front, never occluders here
    for (let j = 0; j < i; j++) {
      const b = out[j];
      if (b.cls === 'tree' || b.z >= a.z) continue;
      const ix = Math.max(0, Math.min(a.box.x + a.box.w, b.box.x + b.box.w) - Math.max(a.box.x, b.box.x)), iy = Math.max(0, Math.min(a.box.y + a.box.h, b.box.y + b.box.h) - Math.max(a.box.y, b.box.y));
      if (ix * iy > 0.7 * a.box.w * a.box.h && a.cls !== b.cls) { a.visible = false; break; }
    }
  }
  return out;
}

// ---------------------------------------------------------------------------------------------
// Runtime
// ---------------------------------------------------------------------------------------------

export interface ModelContext extends BaseModelContext { /** Called after every pipeline run (webhooks, ROS publishers). */ onOutput?: (out: PipelineOutput, cam: CameraItem) => void }

export interface PipelineStats { frames: number; tp: number; fp: number; fn: number; posErrSum: number; posErrN: number; fps: number; lastLatencyMs: number }

export interface CloudAnalysis { cloud: PointCloud; ground: PointCloud; objects: PointCloud; clusters: Cluster[]; rows: Line2D[]; classes: string[] }

export interface PipelineOutput {
  frame: VisionFrame;
  results: Partial<Record<VisionTask, VisionResult>>;
  /** Final detections after tracking + 3D localisation. */
  detections: Box2D[];
  cloud?: CloudAnalysis;
  /** Targets computed (world poses, mm) — grasp + approach per detection with a 3D position. */
  targets: Array<{ name: string; cls: string; grasp: Mat4; approach: Mat4; p: Vec3; z: number; sigmaMm: number; trackId?: number }>;
  vla?: { action: number[]; tcp?: Mat4; gripper?: number; text?: string };
  text?: string;
  warnings: string[];
}

export class VisionRuntime {
  models = new Map<VisionTask, VisionModel>();
  tracker = new ByteTracker({ minHits: 1 });
  stats: PipelineStats = { frames: 0, tp: 0, fp: 0, fn: 0, posErrSum: 0, posErrN: 0, fps: 0, lastLatencyMs: 0 };
  last: PipelineOutput | null = null;
  private lastTime = 0;
  private seedCounter = 0;
  constructor(readonly cam: CameraItem, public cfg: VisionStackConfig, readonly ctx: ModelContext = {}) {}

  model(task: VisionTask): VisionModel {
    let m = this.models.get(task);
    if (!m) { m = createModel(this.cfg, task, this.ctx); this.models.set(task, m); }
    return m;
  }
  reset(): void { for (const m of this.models.values()) m.dispose?.(); this.models.clear(); this.tracker.reset(); this.stats = { frames: 0, tp: 0, fp: 0, fn: 0, posErrSum: 0, posErrN: 0, fps: 0, lastLatencyMs: 0 }; this.last = null; }

  /** Build the frame: ground truth + (simulated) depth / cloud + optional rendered pixels. */
  capture(station: Station, opts: { rgba?: Uint8ClampedArray; dataUrl?: string; time?: number; proprio?: VisionFrame['proprio']; lidar?: { channels?: number; hres?: number; range?: number } } = {}): VisionFrame {
    const K = intrinsicsFromCamera(this.cam);
    const camWorld = this.cam.poseAbs();
    const frame: VisionFrame = { width: this.cam.width, height: this.cam.height, K, camWorld, time: opts.time ?? station.simTime ?? 0, rgba: opts.rgba, dataUrl: opts.dataUrl, proprio: opts.proprio };
    if (this.cfg.simulate || this.cfg.runtime === 'simulated') frame.truth = captureTruth(station, this.cam, { maxRange: Math.min(this.cam.far, this.cfg.workingDistance * 1000 * 4 + 2000) });
    const m = this.cfg.modality;
    const needCloud = this.cfg.tasks.some((t) => t === 'cloud_detect' || t === 'cloud_segment');
    if (m === 'lidar3d') {
      const s = VISION_SENSORS.find((x) => x.id === this.cfg.sensor);
      const [v0, v1] = s ? [-s.vfov / 2, s.vfov / 2] : [-15, 15];
      const { cloud } = simulateLidar3D(station, camWorld, { channels: opts.lidar?.channels ?? Math.min(32, s?.channels ?? 16), vfov: [v0, v1], hfov: s?.hfov ?? 360, hres: opts.lidar?.hres ?? 1, range: opts.lidar?.range ?? Math.min(this.cam.far, 40000), noise: s?.depthError ? s.depthError.a * 1000 : 20, fruit: false });
      frame.cloud = cloud;
    } else if (needCloud && (m === 'stereo' || m === 'rgbd' || m === 'tof')) {
      const { depth, cloud } = simulateDepthCamera(station, camWorld, K, { range: Math.min(this.cam.far, 15000), step: Math.max(2, Math.round(K.width / 160)), noise: 0.01, fruit: true });
      frame.depth = depth; frame.cloud = cloud;
    }
    return frame;
  }

  /** Run every configured task on a frame. */
  async run(station: Station, frame: VisionFrame, opts: { prompt?: string; instruction?: string; tcp?: Mat4 } = {}): Promise<PipelineOutput> {
    const cfg = this.cfg;
    const t0 = typeof performance !== 'undefined' ? performance.now() : Date.now();
    const results: Partial<Record<VisionTask, VisionResult>> = {};
    const warnings: string[] = [];
    const run = async (task: VisionTask, prompt?: string) => {
      try { const r = await this.model(task).run(frame, { task, classes: cfg.classes, confidence: cfg.confidence, iou: cfg.iou, prompt, seed: (cfg.seed ?? 1) + this.seedCounter++ }); results[task] = r; return r; }
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
      // a dedicated classifier re-labels the per-detection attributes with its own accuracy (simulation)
      const acc = VISION_MODELS.find((m) => m.id === cfg.models.classify)?.accuracy ?? 0.7;
      const rndC = makeRng((cfg.seed ?? 1) * 17 + this.stats.frames);
      const byId = new Map(frame.truth.map((t) => [t.id, t]));
      for (const b of boxes) { const t = b.truthId ? byId.get(b.truthId) : undefined; if (t?.attrs && typeof t.attrs.ripe === 'number') { b.attr = { ...(b.attr ?? {}), ripe: rndC() < (1 - acc) * 0.5 ? 1 - t.attrs.ripe : t.attrs.ripe }; } }
    }
    // tracking
    const dt = this.lastTime ? Math.max(1e-3, frame.time - this.lastTime) : 1 / 15;
    this.lastTime = frame.time;
    if (cfg.tasks.includes('track') || cfg.tasks.includes('follow')) boxes = this.tracker.update(boxes, dt);
    // 3D localisation
    const s = VISION_SENSORS.find((x) => x.id === cfg.sensor);
    const sigma = (z: number) => depthSigmaMm(cfg.sensor, cfg.modality, z, { baselineMm: (s?.baseline ?? 0.1) * 1000, fxPx: frame.K.fx });
    const truthById = new Map((frame.truth ?? []).map((t) => [t.id, t]));
    const rnd = makeRng((cfg.seed ?? 1) * 31 + this.stats.frames);
    const wantsPose = cfg.tasks.some((t) => ['pose', 'grasp', 'measure', 'follow'].includes(t));
    if (wantsPose) for (const b of boxes) {
      if (b.p && b.z) continue;
      let z: number | null = null;
      if (frame.depth) z = medianDepthInBox(frame.depth, frame.width, b, b.mask && b.mask.width === frame.width ? b.mask.data : null);
      if (z === null && b.truthId && truthById.has(b.truthId)) {
        const t = truthById.get(b.truthId)!;
        if (cfg.modality === 'mono') { const size = cfg.objectSizeMm ?? t.size; z = rangeFromSize(frame.K, b.w, size) * (1 + 0.05 * gauss(rnd)); }
        else z = t.z + sigma(t.z) * gauss(rnd);
      }
      if (z === null && cfg.modality === 'mono' && cfg.objectSizeMm) z = rangeFromSize(frame.K, b.w, cfg.objectSizeMm);
      if (z === null || !Number.isFinite(z)) continue;
      b.z = z;
      const cx = b.x + b.w / 2, cy = b.y + b.h / 2;
      const local = backproject(frame.K, cx, cy, z);
      // lateral pixel noise ≈ 0.5 px
      b.p = transformPoint(frame.camWorld, [local[0] + (z / frame.K.fx) * 0.5 * gauss(rnd), local[1] + (z / frame.K.fy) * 0.5 * gauss(rnd), local[2]]);
      if (b.truthId && truthById.has(b.truthId)) { const t = truthById.get(b.truthId)!; const e = Math.hypot(b.p[0] - t.p[0], b.p[1] - t.p[1], b.p[2] - t.p[2]); this.stats.posErrSum += e; this.stats.posErrN++; }
    }
    // point clouds
    let cloud: CloudAnalysis | undefined;
    if (frame.cloud && cfg.tasks.some((t) => t === 'cloud_detect' || t === 'cloud_segment' || t === 'measure')) {
      cloud = analyseCloud(frame.cloud, { voxel: cfg.modality === 'lidar3d' ? 150 : 40, tol: cfg.modality === 'lidar3d' ? 500 : 120, minPts: cfg.modality === 'lidar3d' ? 6 : 15 });
      if (cfg.tasks.includes('cloud_detect')) {
        const rc: VisionResult = { task: 'cloud_detect', model: cfg.models.cloud_detect ?? 'pcl_pipeline', boxes: [], latencyMs: 0 };
        for (const cl of cloud.clusters) {
          const l = transformPoint(invert(frame.camWorld), cl.centroid);
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
      if (r?.action) { const a = applyVlaAction(opts.tcp ?? frame.camWorld, r.action, { scaleMm: cfg.vla?.actionScaleMm, scaleDeg: cfg.vla?.actionScaleDeg }); vla = { action: r.action, tcp: opts.tcp ? a.pose : undefined, gripper: a.gripper, text: r.text }; }
    }
    // targets
    const targets: PipelineOutput['targets'] = [];
    if (cfg.tasks.some((t) => t === 'pose' || t === 'grasp')) {
      let i = 0;
      for (const b of boxes) {
        if (!b.p || !b.z) continue;
        const { grasp, approach } = targetPoseFromPoint(b.p, frame.camWorld, { approachMm: 100 });
        targets.push({ name: `${b.cls}${b.id !== undefined ? `#${b.id}` : `_${++i}`}`, cls: b.cls, grasp, approach, p: b.p, z: b.z, sigmaMm: sigma(b.z), trackId: b.id });
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
  async step(station: Station, opts: Parameters<VisionRuntime['capture']>[1] & { prompt?: string; instruction?: string; tcp?: Mat4 } = {}): Promise<PipelineOutput> {
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

/** Create (or refresh) a "Vision targets" frame with grasp/approach targets from a pipeline output. */
export function createTargetsFromOutput(station: Station, cam: CameraItem, out: PipelineOutput, opts: { parent?: Item; max?: number; robotId?: string | null; frameName?: string } = {}): Target[] {
  const parent = opts.parent ?? station;
  const name = opts.frameName ?? `Vision ${cam.name}`;
  let frame = parent.children.find((c) => c instanceof Frame && c.name === name) as Frame | undefined;
  if (!frame) { frame = parent.addChild(new Frame(name)); frame.setPose(transl(0, 0, 0)); }
  for (const c of [...frame.children]) frame.removeChild(c);
  const inv = invert(frame.poseAbs());
  const targets: Target[] = [];
  for (const t of out.targets.slice(0, opts.max ?? 50)) {
    const app = frame.addChild(new Target(`${t.name} approach`));
    app.setPose(multiply(inv, t.approach)); app.robotId = opts.robotId ?? null;
    const g = frame.addChild(new Target(t.name));
    g.setPose(multiply(inv, t.grasp)); g.robotId = opts.robotId ?? null;
    g.setParam('vision', { cls: t.cls, z: t.z, sigmaMm: t.sigmaMm, trackId: t.trackId ?? null } as any);
    targets.push(app, g);
  }
  return targets;
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
