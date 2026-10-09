/**
 * Pluggable vision models. Every adapter implements `VisionModel.run(frame, opts)` and returns boxes / masks /
 * keypoints / labels / text in a common `VisionResult`, so the pipeline, the Vision tab and programs do not care
 * whether a detection came from a simulated ground truth, an ONNX YOLO model running in the browser, the studio
 * server, an OpenAI-compatible VLM, a VLA action server or a ROS 2 topic.
 *
 *  - `SimulatedModel`   — ground-truth projection with recall / false-positive / noise derived from the model's
 *                         catalogue accuracy (works in Node and in the browser, no weights needed).
 *  - `OnnxYoloModel`    — onnxruntime-web (WebGPU → WASM). Any Ultralytics export (`yolo export format=onnx`):
 *                         YOLOv5/8/10/11 detect, -seg, -pose, -cls; RT-DETR and YOLO-World exports with the
 *                         same output layout. Pre/post-processing (letterbox, decode, NMS, mask protos) is pure
 *                         TypeScript and unit-tested.
 *  - `HttpModel`        — POST the image to the studio server (`/vision/infer`, ultralytics / onnxruntime in
 *                         Python), a Roboflow-style or a Triton/TorchServe JSON endpoint.
 *  - `VlmOpenAIModel`   — OpenAI-compatible `chat/completions` with image content (Ollama, vLLM, LM Studio,
 *                         OpenAI/Anthropic-compatible gateways): detection (grounding), classification,
 *                         free-form questions; answers are parsed into boxes/labels/text.
 *  - `VlaHttpModel`     — VLA policy servers (openpi websocket/HTTP JSON, OpenVLA `/act`, LeRobot policies,
 *                         the studio's own format): image + instruction + proprioception → action chunk.
 *  - `Ros2VisionModel`  — `vision_msgs/Detection2DArray` over rosbridge.
 *  - `ByteTracker`      — multi-object tracker (ByteTrack association with a constant-velocity filter).
 */
import type { Vec3, Mat4 } from '../core/math/pose';
import { multiply, transl, rotx, roty, rotz } from '../core/math/pose';
import type { Intrinsics } from './camera_model';
import type { PointCloud } from './pointcloud';
import { VISION_MODELS, VisionModelSpec, VisionTask, VisionStackConfig } from './stack';

// ---------------------------------------------------------------------------------------------
// Common types
// ---------------------------------------------------------------------------------------------

export interface Mask { data: Uint8Array; width: number; height: number }
export interface Box2D {
  x: number; y: number; w: number; h: number;
  score: number;
  cls: string;
  clsId?: number;
  /** Track id (after tracking). */
  id?: number;
  /** Track velocity (px/s). */
  vx?: number; vy?: number;
  /** Mask at image resolution (or downscaled: see width/height). */
  mask?: Mask;
  /** Keypoints in image pixels with confidence. */
  keypoints?: Array<[number, number, number]>;
  /** Depth (mm along camera Z) and 3D position (world, mm) when estimated. */
  z?: number;
  p?: Vec3;
  /** Ground-truth id for evaluation in simulation. */
  truthId?: string;
  /** Secondary classification (e.g. ripeness) */
  attr?: Record<string, number | string>;
}

export interface TruthObject {
  id: string;
  cls: string;
  box: { x: number; y: number; w: number; h: number };
  /** World position (mm) and camera depth (mm). */
  p: Vec3; z: number;
  /** Real size (mm). */
  size: number;
  visible: boolean;
  attrs?: Record<string, number | string>;
  keypoints?: Array<[number, number]>;
}

export interface VisionFrame {
  width: number; height: number;
  /** RGBA pixels (browser render) — optional in Node. */
  rgba?: Uint8ClampedArray;
  /** PNG/JPEG data URL for HTTP / VLM adapters. */
  dataUrl?: string;
  /** Depth image (mm along Z), same size as the image. */
  depth?: Float32Array;
  cloud?: PointCloud;
  K: Intrinsics;
  camWorld: Mat4;
  time: number;
  /** Ground truth (simulation only). */
  truth?: TruthObject[];
  /** Robot proprioception for VLA: joints (deg) + TCP pose. */
  proprio?: { joints?: number[]; tcp?: number[]; gripper?: number };
}

export interface RunOptions {
  task: VisionTask;
  classes: string[];
  confidence: number;
  iou: number;
  /** VLM prompt / VLA instruction. */
  prompt?: string;
  seed?: number;
}

export interface VisionResult {
  task: VisionTask;
  model: string;
  boxes: Box2D[];
  /** Whole-image classification (classify task). */
  labels?: Array<{ cls: string; score: number }>;
  /** VLM free text. */
  text?: string;
  /** VLA action (7-DoF: dx dy dz droll dpitch dyaw gripper) or an action chunk. */
  action?: number[];
  actionChunk?: number[][];
  latencyMs: number;
  raw?: unknown;
}

export interface VisionModel {
  id: string;
  name: string;
  tasks: VisionTask[];
  ready?(): Promise<void>;
  run(frame: VisionFrame, opts: RunOptions): Promise<VisionResult>;
  dispose?(): void;
}

const now = () => (typeof performance !== 'undefined' ? performance.now() : Date.now());

export function iou(a: { x: number; y: number; w: number; h: number }, b: { x: number; y: number; w: number; h: number }): number {
  const x1 = Math.max(a.x, b.x), y1 = Math.max(a.y, b.y), x2 = Math.min(a.x + a.w, b.x + b.w), y2 = Math.min(a.y + a.h, b.y + b.h);
  const inter = Math.max(0, x2 - x1) * Math.max(0, y2 - y1);
  const u = a.w * a.h + b.w * b.h - inter;
  return u > 0 ? inter / u : 0;
}

/** Class-aware greedy NMS. */
export function nms(boxes: Box2D[], iouThresh: number): Box2D[] {
  const sorted = [...boxes].sort((a, b) => b.score - a.score);
  const kept: Box2D[] = [];
  for (const b of sorted) if (!kept.some((k) => k.cls === b.cls && iou(k, b) > iouThresh)) kept.push(b);
  return kept;
}

function makeRng(seed: number) { let s = seed >>> 0 || 1; return () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; }; }
function gauss(r: () => number) { return Math.sqrt(-2 * Math.log(r() + 1e-12)) * Math.cos(2 * Math.PI * r()); }

// ---------------------------------------------------------------------------------------------
// Simulated model
// ---------------------------------------------------------------------------------------------

/**
 * Ground truth → detections with the statistics of the chosen model: recall drops for small boxes (few pixels)
 * and with (1 − accuracy); false positives appear on background; boxes jitter; classification confuses close
 * classes; masks are ellipses inside the box; keypoints are the "stem" (top centre) with noise.
 */
export class SimulatedModel implements VisionModel {
  id: string; name: string; tasks: VisionTask[];
  private frameNo = 0;
  private vlaTarget: string | null = null;
  private vlaLast: number[] | null = null;
  private vlaHold = 0;
  constructor(readonly spec: VisionModelSpec, private seed = 1) { this.id = spec.id; this.name = `${spec.name} (simulated)`; this.tasks = spec.tasks; }
  async run(frame: VisionFrame, opts: RunOptions): Promise<VisionResult> {
    const t0 = now();
    const rnd = makeRng((opts.seed ?? this.seed) * 7919 + this.frameNo++ * 104729 + 1);
    const truth = frame.truth ?? [];
    const acc = this.spec.accuracy;
    const boxes: Box2D[] = [];
    const wanted = new Set(opts.classes.map((c) => c.toLowerCase()));
    const open = this.spec.openVocabulary;
    if (opts.task === 'classify') {
      // whole-image classification: dominant visible class
      const counts = new Map<string, number>();
      for (const t of truth) if (t.visible) counts.set(t.cls, (counts.get(t.cls) ?? 0) + 1);
      const labels = [...counts.entries()].sort((a, b) => b[1] - a[1]).map(([cls, n]) => ({ cls, score: Math.min(0.99, 0.5 + 0.5 * (n / Math.max(1, truth.length)) * acc + 0.1 * (rnd() - 0.5)) }));
      return { task: opts.task, model: this.id, boxes: [], labels: labels.length ? labels : [{ cls: 'background', score: 0.9 }], latencyMs: now() - t0 };
    }
    if (opts.task === 'vlm_query') return this.vlm(frame, opts, t0);
    if (opts.task === 'vla_policy') return this.vla(frame, opts, t0);
    for (const t of truth) {
      if (!t.visible) continue;
      const known = wanted.size === 0 || wanted.has(t.cls.toLowerCase()) || open;
      if (!known) continue;
      const px = Math.min(t.box.w, t.box.h);
      // recall model: sigmoid on box size in pixels (small objects are missed) × model accuracy
      const sizeFactor = 1 / (1 + Math.exp(-(px - 8) / 3)); // ~50 % at 8 px, ~80 % at 12 px, ~98 % at 20 px (640 input)
      const recall = Math.min(0.99, (0.55 + 0.45 * acc) * sizeFactor);
      if (rnd() > recall) continue;
      const jit = (1 - acc) * 0.15 * px + 0.5;
      const b: Box2D = { x: t.box.x + gauss(rnd) * jit, y: t.box.y + gauss(rnd) * jit, w: Math.max(2, t.box.w * (1 + gauss(rnd) * (1 - acc) * 0.2)), h: Math.max(2, t.box.h * (1 + gauss(rnd) * (1 - acc) * 0.2)), score: Math.min(0.99, Math.max(opts.confidence, recall * (0.75 + 0.25 * rnd()))), cls: t.cls, truthId: t.id, attr: t.attrs ? { ...t.attrs } : undefined };
      if (b.score < opts.confidence) continue;
      if (opts.task === 'segment') b.mask = ellipseMask(b, frame.width, frame.height);
      if (opts.task === 'keypoints') { const kps = t.keypoints ?? [[t.box.x + t.box.w / 2, t.box.y]]; b.keypoints = kps.map(([kx, ky]) => [kx + gauss(rnd) * jit, ky + gauss(rnd) * jit, Math.min(0.99, acc + 0.2 * rnd())]); }
      // classification attribute confusion (e.g. ripeness) scales with accuracy
      if (b.attr && typeof b.attr.ripe === 'number' && rnd() < (1 - acc) * 0.5) b.attr.ripe = 1 - (b.attr.ripe as number); // coarse attribute confusion
      boxes.push(b);
    }
    // false positives: (1 - acc) * 2 per frame on average
    const nFP = Math.floor((1 - acc) * 2 + rnd());
    const clsList = opts.classes.length ? opts.classes : ['object'];
    for (let i = 0; i < nFP; i++) {
      const w = 8 + rnd() * 40, h = 8 + rnd() * 40;
      boxes.push({ x: rnd() * (frame.width - w), y: rnd() * (frame.height - h), w, h, score: opts.confidence + rnd() * 0.25, cls: clsList[Math.floor(rnd() * clsList.length)] });
    }
    return { task: opts.task, model: this.id, boxes: nms(boxes, opts.iou), latencyMs: now() - t0 };
  }
  private vlm(frame: VisionFrame, opts: RunOptions, t0: number): VisionResult {
    const truth = (frame.truth ?? []).filter((t) => t.visible);
    const counts = new Map<string, number>();
    for (const t of truth) counts.set(t.cls, (counts.get(t.cls) ?? 0) + 1);
    const q = (opts.prompt ?? '').toLowerCase();
    const boxes: Box2D[] = [];
    let text: string;
    if (/where|find|locate|point|box|detect|ground|show/.test(q) || opts.classes.length) {
      const wanted = opts.classes.length ? opts.classes.map((c) => c.toLowerCase()) : [...counts.keys()];
      for (const t of truth) if (wanted.some((w) => t.cls.toLowerCase().includes(w) || q.includes(t.cls.toLowerCase()))) boxes.push({ ...t.box, score: 0.8, cls: t.cls, truthId: t.id, attr: t.attrs });
      text = boxes.length ? `I can see ${boxes.length} ${wanted.join('/')}: ${boxes.slice(0, 5).map((b) => `${b.cls} at [${Math.round(b.x)},${Math.round(b.y)},${Math.round(b.x + b.w)},${Math.round(b.y + b.h)}]`).join('; ')}.` : `I cannot see any ${wanted.join('/')} in this image.`;
    } else if (/how many|count/.test(q)) text = [...counts.entries()].map(([c, n]) => `${n} ${c}`).join(', ') || 'nothing recognisable';
    else text = `The image shows ${[...counts.entries()].map(([c, n]) => `${n} ${c}${n > 1 ? 's' : ''}`).join(', ') || 'an empty scene'}. The nearest object is ${truth.length ? `${truth.reduce((a, b) => (a.z < b.z ? a : b)).cls} at ${(Math.min(...truth.map((t) => t.z)) / 1000).toFixed(2)} m` : 'not visible'}.`;
    return { task: 'vlm_query', model: this.id, boxes, text: `[simulated ${this.spec.name}] ${text}`, latencyMs: now() - t0 };
  }
  private vla(frame: VisionFrame, opts: RunOptions, t0: number): VisionResult {
    // scripted "policy": move the camera towards the nearest instructed object, close gripper when near
    const q = (opts.prompt ?? '').toLowerCase();
    const cands = (frame.truth ?? []).filter((t) => t.visible && (!q || q.includes(t.cls.toLowerCase()) || opts.classes.some((c) => q.includes(c.toLowerCase()) && t.cls.toLowerCase() === c.toLowerCase())));
    if (!cands.length) return { task: 'vla_policy', model: this.id, boxes: [], action: [0, 0, 0, 0, 0, 0, 0], text: 'no target visible', latencyMs: now() - t0 };
    // lock on the first chosen target (a real policy keeps attending to one object); if it is momentarily occluded,
    // repeat the last action for a few frames instead of switching to another fruit
    const locked = this.vlaTarget ? cands.find((t) => t.id === this.vlaTarget) : undefined;
    if (!locked && this.vlaTarget && this.vlaLast && this.vlaHold < 6) { this.vlaHold++; return { task: 'vla_policy', model: this.id, boxes: [], action: this.vlaLast, text: 'approach (target occluded, holding)', latencyMs: now() - t0 }; }
    const tgt = locked ?? cands.reduce((a, b) => (a.z < b.z ? a : b));
    this.vlaTarget = tgt.id; this.vlaHold = 0;
    const cx = tgt.box.x + tgt.box.w / 2, cy = tgt.box.y + tgt.box.h / 2;
    const ex = (cx - frame.K.cx) / frame.K.fx, ey = (cy - frame.K.cy) / frame.K.fy;
    const near = tgt.z < 150;
    // normalised deltas in the camera frame: x right, y down, z forward
    const action = [Math.max(-1, Math.min(1, ex * 3)), Math.max(-1, Math.min(1, ey * 3)), near ? 0 : Math.min(1, Math.max(0.3, tgt.z / 500)), 0, 0, 0, near ? 1 : 0];
    this.vlaLast = action;
    if (near) { this.vlaTarget = null; this.vlaLast = null; }
    return { task: 'vla_policy', model: this.id, boxes: [{ ...tgt.box, score: 0.9, cls: tgt.cls, truthId: tgt.id }], action, text: near ? 'grasp' : `approach ${tgt.cls}`, latencyMs: now() - t0 };
  }
}

export function ellipseMask(b: { x: number; y: number; w: number; h: number }, width: number, height: number): Mask {
  const x0 = Math.max(0, Math.floor(b.x)), y0 = Math.max(0, Math.floor(b.y)), x1 = Math.min(width, Math.ceil(b.x + b.w)), y1 = Math.min(height, Math.ceil(b.y + b.h));
  const mw = Math.max(1, x1 - x0), mh = Math.max(1, y1 - y0);
  const data = new Uint8Array(mw * mh);
  const cx = b.x + b.w / 2 - x0, cy = b.y + b.h / 2 - y0, rx = b.w / 2, ry = b.h / 2;
  for (let y = 0; y < mh; y++) for (let x = 0; x < mw; x++) { const dx = (x + 0.5 - cx) / rx, dy = (y + 0.5 - cy) / ry; if (dx * dx + dy * dy <= 1) data[y * mw + x] = 1; }
  return { data, width: mw, height: mh };
}

// ---------------------------------------------------------------------------------------------
// ONNX Runtime Web — YOLO family
// ---------------------------------------------------------------------------------------------

export interface LetterboxInfo { scale: number; padX: number; padY: number; size: number }

/** RGBA → NCHW float [0,1] letterboxed to a square input. */
export function letterbox(rgba: Uint8ClampedArray, width: number, height: number, size: number): { data: Float32Array; info: LetterboxInfo } {
  const scale = Math.min(size / width, size / height);
  const nw = Math.round(width * scale), nh = Math.round(height * scale);
  const padX = Math.floor((size - nw) / 2), padY = Math.floor((size - nh) / 2);
  const data = new Float32Array(3 * size * size).fill(114 / 255);
  const plane = size * size;
  for (let y = 0; y < nh; y++) {
    const sy = Math.min(height - 1, Math.floor(y / scale));
    for (let x = 0; x < nw; x++) {
      const sx = Math.min(width - 1, Math.floor(x / scale));
      const si = (sy * width + sx) * 4, di = (y + padY) * size + (x + padX);
      data[di] = rgba[si] / 255; data[plane + di] = rgba[si + 1] / 255; data[2 * plane + di] = rgba[si + 2] / 255;
    }
  }
  return { data, info: { scale, padX, padY, size } };
}

export interface YoloDecodeOptions { classes: string[]; confidence: number; iou: number; task: 'detect' | 'segment' | 'keypoints' | 'classify'; info: LetterboxInfo; imgW: number; imgH: number; numKeypoints?: number }

/**
 * Decode Ultralytics outputs. Handles `[1, 4+nc(+32|+3K), N]` (v8/11), `[1, N, 4+nc]` (transposed / RT-DETR
 * with normalised xywh), `[1, N, 5+nc]` (v5 with objectness) and `[1, N, 6]` end-to-end (v10: xyxy conf cls).
 * `proto` is the `[1, 32, mh, mw]` mask prototype tensor for -seg models.
 */
export function decodeYolo(output: Float32Array, dims: number[], opts: YoloDecodeOptions, proto?: { data: Float32Array; dims: number[] }): Box2D[] {
  const { info, classes } = opts;
  const nc = classes.length;
  const boxes: Box2D[] = [];
  const unmap = (cx: number, cy: number, w: number, h: number): { x: number; y: number; w: number; h: number } => ({ x: (cx - w / 2 - info.padX) / info.scale, y: (cy - h / 2 - info.padY) / info.scale, w: w / info.scale, h: h / info.scale });
  const clip = (b: { x: number; y: number; w: number; h: number }) => { const x0 = Math.max(0, b.x), y0 = Math.max(0, b.y), x1 = Math.min(opts.imgW, b.x + b.w), y1 = Math.min(opts.imgH, b.y + b.h); return { x: x0, y: y0, w: Math.max(0, x1 - x0), h: Math.max(0, y1 - y0) }; };
  if (dims.length === 2 || (dims.length === 3 && dims[1] === 1 && dims[2] === nc)) {
    // classification: probabilities
    const probs = Array.from(output.subarray(0, nc));
    return probs.map((p, i) => ({ x: 0, y: 0, w: opts.imgW, h: opts.imgH, score: p, cls: classes[i] ?? `c${i}` })).sort((a, b) => b.score - a.score);
  }
  const [, d1, d2] = dims;
  const maskCoefs: number[][] = [];
  const nk = opts.numKeypoints ?? 0;
  const segCh = opts.task === 'segment' ? 32 : 0;
  const poseCh = opts.task === 'keypoints' ? nk * 3 : 0;
  const chExpected = 4 + nc + segCh + poseCh;
  let layout: 'ch_first' | 'n_first_v8' | 'n_first_v5' | 'e2e';
  if (d1 === chExpected) layout = 'ch_first';
  else if (d2 === chExpected) layout = 'n_first_v8';
  else if (d2 === 5 + nc) layout = 'n_first_v5';
  else if (d2 === 6) layout = 'e2e';
  else layout = d1 < d2 ? 'ch_first' : 'n_first_v8';
  const N = layout === 'ch_first' ? d2 : d1;
  const C = layout === 'ch_first' ? d1 : d2;
  const at = (n: number, c: number) => (layout === 'ch_first' ? output[c * N + n] : output[n * C + c]);
  for (let n = 0; n < N; n++) {
    if (layout === 'e2e') {
      const score = at(n, 4); if (score < opts.confidence) continue;
      const cid = Math.round(at(n, 5));
      const x0 = at(n, 0), y0 = at(n, 1), x1 = at(n, 2), y1 = at(n, 3);
      boxes.push({ ...clip(unmap((x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0)), score, cls: classes[cid] ?? `c${cid}`, clsId: cid });
      continue;
    }
    let best = -1, bs = 0;
    const off = layout === 'n_first_v5' ? 5 : 4;
    const obj = layout === 'n_first_v5' ? at(n, 4) : 1;
    for (let c = 0; c < nc; c++) { const s = at(n, off + c) * obj; if (s > bs) { bs = s; best = c; } }
    if (best < 0 || bs < opts.confidence) continue;
    let cx = at(n, 0), cy = at(n, 1), w = at(n, 2), h = at(n, 3);
    if (cx <= 1 && cy <= 1 && w <= 1 && h <= 1) { cx *= info.size; cy *= info.size; w *= info.size; h *= info.size; } // normalised (RT-DETR)
    const b: Box2D = { ...clip(unmap(cx, cy, w, h)), score: bs, cls: classes[best] ?? `c${best}`, clsId: best };
    if (poseCh) { b.keypoints = []; for (let k = 0; k < nk; k++) { const kx = at(n, 4 + nc + k * 3), ky = at(n, 4 + nc + k * 3 + 1), ks = at(n, 4 + nc + k * 3 + 2); b.keypoints.push([(kx - info.padX) / info.scale, (ky - info.padY) / info.scale, ks]); } }
    if (segCh) { const coefs: number[] = []; for (let k = 0; k < 32; k++) coefs.push(at(n, 4 + nc + k)); maskCoefs.push(coefs); (b as any)._mi = maskCoefs.length - 1; }
    boxes.push(b);
  }
  const kept = nms(boxes, opts.iou);
  if (segCh && proto) for (const b of kept) { const mi = (b as any)._mi as number | undefined; if (mi !== undefined) b.mask = protoMask(proto, maskCoefs[mi], b, info, opts.imgW, opts.imgH); delete (b as any)._mi; }
  return kept;
}

/** sigmoid(coefs · proto) cropped to the box, upsampled (nearest) to image pixels. */
export function protoMask(proto: { data: Float32Array; dims: number[] }, coefs: number[], b: Box2D, info: LetterboxInfo, imgW: number, imgH: number): Mask {
  const [, nm, mh, mw] = proto.dims;
  const x0 = Math.max(0, Math.floor(b.x)), y0 = Math.max(0, Math.floor(b.y)), x1 = Math.min(imgW, Math.ceil(b.x + b.w)), y1 = Math.min(imgH, Math.ceil(b.y + b.h));
  const w = Math.max(1, x1 - x0), h = Math.max(1, y1 - y0);
  const data = new Uint8Array(w * h);
  const sx = (mw / info.size) * info.scale, sy = (mh / info.size) * info.scale;
  for (let y = 0; y < h; y++) {
    const my = Math.min(mh - 1, Math.max(0, Math.floor((y + y0) * sy + (info.padY * mh) / info.size)));
    for (let x = 0; x < w; x++) {
      const mx = Math.min(mw - 1, Math.max(0, Math.floor((x + x0) * sx + (info.padX * mw) / info.size)));
      let v = 0;
      for (let k = 0; k < nm; k++) v += coefs[k] * proto.data[k * mh * mw + my * mw + mx];
      data[y * w + x] = v > 0 ? 1 : 0; // sigmoid(v) > 0.5
    }
  }
  return { data, width: w, height: h };
}

export interface OnnxOptions { url: string; task: 'detect' | 'segment' | 'keypoints' | 'classify'; classes: string[]; inputSize?: number; numKeypoints?: number; executionProviders?: string[]; wasmPaths?: string }

export class OnnxYoloModel implements VisionModel {
  id: string; name: string; tasks: VisionTask[];
  private session: any = null;
  private ort: any = null;
  private inputName = 'images';
  private loading: Promise<void> | null = null;
  constructor(readonly opts: OnnxOptions, id = 'onnx') { this.id = id; this.name = `ONNX ${opts.url.split('/').pop()}`; this.tasks = [opts.task === 'keypoints' ? 'keypoints' : opts.task]; }
  async ready(): Promise<void> {
    if (this.session) return;
    if (!this.loading) this.loading = (async () => {
      const g = globalThis as any;
      const ort = g.ort ?? (await import('onnxruntime-web'));
      this.ort = ort;
      if (this.opts.wasmPaths) ort.env.wasm.wasmPaths = this.opts.wasmPaths;
      else if (typeof location !== 'undefined' && !g.ort) ort.env.wasm.wasmPaths = `https://cdn.jsdelivr.net/npm/onnxruntime-web@${ort.env.versions?.web ?? '1.30.0'}/dist/`;
      const providers = this.opts.executionProviders ?? (typeof navigator !== 'undefined' && (navigator as any).gpu ? ['webgpu', 'wasm'] : ['wasm']);
      const remote = /^(https?:|data:|blob:)/.test(this.opts.url) || typeof location !== 'undefined';
      const src = remote ? new Uint8Array(await (await fetch(this.opts.url)).arrayBuffer()) : this.opts.url; // Node: onnxruntime reads the file path
      this.session = await ort.InferenceSession.create(src, { executionProviders: providers, graphOptimizationLevel: 'all' });
      this.inputName = this.session.inputNames[0];
    })();
    await this.loading;
  }
  async run(frame: VisionFrame, opts: RunOptions): Promise<VisionResult> {
    await this.ready();
    if (!frame.rgba) throw new Error('ONNX model needs RGBA pixels (render the camera first)');
    const t0 = now();
    const size = this.opts.inputSize ?? 640;
    const { data, info } = letterbox(frame.rgba, frame.width, frame.height, size);
    const feeds: Record<string, any> = { [this.inputName]: new this.ort.Tensor('float32', data, [1, 3, size, size]) };
    const outMap = await this.session.run(feeds);
    const names: string[] = this.session.outputNames;
    const out0 = outMap[names[0]];
    const proto = names.length > 1 ? outMap[names[1]] : undefined;
    const classes = this.opts.classes.length ? this.opts.classes : opts.classes;
    const boxes = decodeYolo(out0.data as Float32Array, out0.dims as number[], { classes, confidence: opts.confidence, iou: opts.iou, task: this.opts.task, info, imgW: frame.width, imgH: frame.height, numKeypoints: this.opts.numKeypoints ?? 17 }, proto ? { data: proto.data as Float32Array, dims: proto.dims as number[] } : undefined);
    if (this.opts.task === 'classify') return { task: 'classify', model: this.id, boxes: [], labels: boxes.map((b) => ({ cls: b.cls, score: b.score })), latencyMs: now() - t0 };
    return { task: opts.task, model: this.id, boxes, latencyMs: now() - t0 };
  }
  dispose(): void { this.session?.release?.(); this.session = null; }
}

// ---------------------------------------------------------------------------------------------
// HTTP inference (studio server / Roboflow / Triton-like JSON)
// ---------------------------------------------------------------------------------------------

export interface HttpOptions { url: string; model?: string; format?: 'studio' | 'roboflow' | 'ultralytics'; headers?: Record<string, string> }

export class HttpModel implements VisionModel {
  id: string; name: string; tasks: VisionTask[] = ['detect', 'segment', 'keypoints', 'classify', 'pose'];
  constructor(readonly opts: HttpOptions, id = 'http') { this.id = id; this.name = `HTTP ${opts.model ?? ''} @ ${opts.url}`; }
  async run(frame: VisionFrame, opts: RunOptions): Promise<VisionResult> {
    if (!frame.dataUrl) throw new Error('HTTP model needs an encoded image (dataUrl)');
    const t0 = now();
    const res = await fetch(this.opts.url, { method: 'POST', headers: { 'content-type': 'application/json', ...(this.opts.headers ?? {}) }, body: JSON.stringify({ model: this.opts.model, task: opts.task, image: frame.dataUrl, confidence: opts.confidence, iou: opts.iou, classes: opts.classes }) });
    if (!res.ok) throw new Error(`inference server ${res.status}: ${(await res.text()).slice(0, 200)}`);
    const j = await res.json();
    return { ...parseDetectionsJson(j, frame.width, frame.height, opts), task: opts.task, model: this.id, latencyMs: now() - t0, raw: j };
  }
}

/** Accepts studio `{detections:[{x,y,w,h|box:[..],class|label,score|confidence,keypoints?,mask?}]}`, Roboflow `{predictions:[{x,y,width,height,class,confidence}]}` (centre-based), Ultralytics `[{name,confidence,box:{x1,y1,x2,y2}}]`. */
export function parseDetectionsJson(j: any, imgW: number, imgH: number, opts: { confidence: number }): { boxes: Box2D[]; labels?: Array<{ cls: string; score: number }>; text?: string } {
  const boxes: Box2D[] = [];
  const list: any[] = Array.isArray(j) ? j : j.detections ?? j.predictions ?? j.results ?? j.objects ?? [];
  for (const d of list) {
    const score = Number(d.score ?? d.confidence ?? d.conf ?? 1);
    if (score < opts.confidence) continue;
    const cls = String(d.class ?? d.label ?? d.name ?? d.cls ?? 'object');
    let x: number, y: number, w: number, h: number;
    if (Array.isArray(d.box) && d.box.length === 4) { [x, y, w, h] = d.box; if (d.format === 'xyxy' || j.format === 'xyxy') { w -= x; h -= y; } }
    else if (Array.isArray(d.bbox) && d.bbox.length === 4) { [x, y, w, h] = d.bbox; }
    else if (d.box && typeof d.box === 'object' && 'x1' in d.box) { x = d.box.x1; y = d.box.y1; w = d.box.x2 - d.box.x1; h = d.box.y2 - d.box.y1; }
    else if ('width' in d && 'height' in d) { w = d.width; h = d.height; x = d.x - w / 2; y = d.y - h / 2; }
    else if ('w' in d && 'h' in d) { x = d.x; y = d.y; w = d.w; h = d.h; }
    else continue;
    if (x <= 1 && y <= 1 && w <= 1 && h <= 1) { x *= imgW; y *= imgH; w *= imgW; h *= imgH; }
    const b: Box2D = { x, y, w, h, score, cls, id: d.track_id ?? d.id };
    if (Array.isArray(d.keypoints)) b.keypoints = d.keypoints.map((k: any) => (Array.isArray(k) ? [k[0], k[1], k[2] ?? 1] : [k.x, k.y, k.score ?? k.confidence ?? 1]));
    if (Array.isArray(d.position) && d.position.length === 3) b.p = d.position as Vec3;
    if (typeof d.z === 'number') b.z = d.z;
    if (Array.isArray(d.points) && d.points.length > 2) b.mask = polygonMask(d.points.map((p: any) => (Array.isArray(p) ? p : [p.x, p.y])), b, imgW, imgH);
    boxes.push(b);
  }
  const labels = j.labels ?? j.classification ?? (j.top5 ? j.top5 : undefined);
  return { boxes, labels: Array.isArray(labels) ? labels.map((l: any) => ({ cls: String(l.class ?? l.label ?? l.name ?? l[0]), score: Number(l.score ?? l.confidence ?? l[1] ?? 0) })) : undefined, text: typeof j.text === 'string' ? j.text : undefined };
}

export function polygonMask(poly: number[][], b: Box2D, imgW: number, imgH: number): Mask {
  const x0 = Math.max(0, Math.floor(b.x)), y0 = Math.max(0, Math.floor(b.y)), x1 = Math.min(imgW, Math.ceil(b.x + b.w)), y1 = Math.min(imgH, Math.ceil(b.y + b.h));
  const w = Math.max(1, x1 - x0), h = Math.max(1, y1 - y0), data = new Uint8Array(w * h);
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    const px = x + x0 + 0.5, py = y + y0 + 0.5; let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) { const [xi, yi] = poly[i], [xj, yj] = poly[j]; if ((yi > py) !== (yj > py) && px < ((xj - xi) * (py - yi)) / (yj - yi) + xi) inside = !inside; }
    if (inside) data[y * w + x] = 1;
  }
  return { data, width: w, height: h };
}

// ---------------------------------------------------------------------------------------------
// VLM — OpenAI-compatible chat completions with images
// ---------------------------------------------------------------------------------------------

export interface VlmOptions { url: string; model: string; apiKey?: string; maxTokens?: number; temperature?: number }

export const VLM_PROMPTS: Record<string, (classes: string[], user?: string) => string> = {
  detect: (c) => `Detect every ${c.join(', ')} in the image. Reply ONLY with JSON: {"detections":[{"label":"<class>","box_2d":[x1,y1,x2,y2],"confidence":0.0-1.0}]} where coordinates are integers in a 0-1000 normalised frame (x to the right, y down).`,
  classify: (c) => `Classify the main content of the image into one of: ${c.join(', ')}. Reply ONLY with JSON: {"labels":[{"label":"<class>","confidence":0.0-1.0}]} sorted by confidence.`,
  segment: (c) => `Find every ${c.join(', ')} and give a polygon outline for each. Reply ONLY with JSON: {"detections":[{"label":"<class>","box_2d":[x1,y1,x2,y2],"polygon":[[x,y],...],"confidence":0.0-1.0}]} in a 0-1000 normalised frame.`,
  vlm_query: (_c, u) => u ?? 'Describe what a robot should know about this scene: objects, their approximate positions, obstacles, and anything unusual.',
  count: (c) => `Count the ${c.join(' and ')} visible. Reply ONLY with JSON: {"counts":{"<class>":n}}`,
};

export class VlmOpenAIModel implements VisionModel {
  id: string; name: string; tasks: VisionTask[] = ['vlm_query', 'detect', 'classify', 'segment'];
  constructor(readonly opts: VlmOptions, id = 'vlm') { this.id = id; this.name = `VLM ${opts.model} @ ${opts.url}`; }
  async run(frame: VisionFrame, opts: RunOptions): Promise<VisionResult> {
    if (!frame.dataUrl) throw new Error('VLM needs an encoded image (dataUrl)');
    const t0 = now();
    const prompt = opts.task === 'vlm_query' ? (opts.prompt || VLM_PROMPTS.vlm_query([], undefined)) : (VLM_PROMPTS[opts.task] ?? VLM_PROMPTS.detect)(opts.classes.length ? opts.classes : ['object'], opts.prompt);
    const body = { model: this.opts.model, max_tokens: this.opts.maxTokens ?? 800, temperature: this.opts.temperature ?? 0, messages: [{ role: 'user', content: [{ type: 'text', text: prompt }, { type: 'image_url', image_url: { url: frame.dataUrl } }] }] };
    const res = await fetch(`${this.opts.url.replace(/\/$/, '')}/chat/completions`, { method: 'POST', headers: { 'content-type': 'application/json', ...(this.opts.apiKey ? { authorization: `Bearer ${this.opts.apiKey}` } : {}) }, body: JSON.stringify(body) });
    if (!res.ok) throw new Error(`VLM endpoint ${res.status}: ${(await res.text()).slice(0, 200)}`);
    const j = await res.json();
    const text: string = j.choices?.[0]?.message?.content ?? j.message?.content ?? j.response ?? '';
    const parsed = parseVlmAnswer(text, frame.width, frame.height, opts);
    return { task: opts.task, model: this.id, boxes: parsed.boxes, labels: parsed.labels, text, latencyMs: now() - t0, raw: j };
  }
}

/** Extract JSON boxes (Qwen/Gemini `box_2d` [y1,x1,y2,x2] or [x1,x2,...] 0-1000, PaliGemma `<loc####>` tokens, Florence `<loc_###>`) and labels from a VLM answer. */
export function parseVlmAnswer(text: string, imgW: number, imgH: number, opts: { confidence: number; classes?: string[] }): { boxes: Box2D[]; labels?: Array<{ cls: string; score: number }> } {
  const boxes: Box2D[] = [];
  let labels: Array<{ cls: string; score: number }> | undefined;
  const jsonText = text.match(/```(?:json)?\s*([\s\S]*?)```/)?.[1] ?? text.slice(text.indexOf('{') >= 0 ? text.indexOf('{') : 0, text.lastIndexOf('}') + 1);
  let j: any = null;
  try { j = JSON.parse(jsonText); } catch { /* fall through to token parsing */ }
  const arr: any[] = j ? (Array.isArray(j) ? j : j.detections ?? j.objects ?? j.boxes ?? []) : [];
  for (const d of arr) {
    const bb: number[] | undefined = d.box_2d ?? d.bbox ?? d.box ?? d.bbox_2d;
    if (!bb || bb.length !== 4) continue;
    const norm = Math.max(...bb) <= 1000 && !(Math.max(...bb) <= 1);
    let [x1, y1, x2, y2] = bb.map(Number);
    if (Math.max(...bb) <= 1) { x1 *= 1000; y1 *= 1000; x2 *= 1000; y2 *= 1000; }
    // Gemini uses [ymin, xmin, ymax, xmax]; detect by key name
    if (d.box_2d && d.format !== 'xyxy' && /gemini/i.test(String(d.model ?? ''))) [y1, x1, y2, x2] = [x1, y1, x2, y2];
    const sx = norm || Math.max(...bb) <= 1 ? imgW / 1000 : 1, sy = norm || Math.max(...bb) <= 1 ? imgH / 1000 : 1;
    const score = Number(d.confidence ?? d.score ?? 0.7);
    if (score < opts.confidence) continue;
    const b: Box2D = { x: Math.min(x1, x2) * sx, y: Math.min(y1, y2) * sy, w: Math.abs(x2 - x1) * sx, h: Math.abs(y2 - y1) * sy, score, cls: String(d.label ?? d.class ?? d.name ?? 'object') };
    if (Array.isArray(d.polygon) && d.polygon.length > 2) b.mask = polygonMask(d.polygon.map((p: number[]) => [p[0] * sx, p[1] * sy]), b, imgW, imgH);
    boxes.push(b);
  }
  if (!arr.length) {
    // PaliGemma: <loc0123><loc0456><loc0789><loc0012> label  (y1 x1 y2 x2 / 1024)
    const re = /<loc(\d{4})><loc(\d{4})><loc(\d{4})><loc(\d{4})>\s*([^<;\n]+)?/g;
    let m: RegExpExecArray | null;
    while ((m = re.exec(text))) { const [y1, x1, y2, x2] = [m[1], m[2], m[3], m[4]].map((v) => Number(v) / 1024); boxes.push({ x: x1 * imgW, y: y1 * imgH, w: (x2 - x1) * imgW, h: (y2 - y1) * imgH, score: 0.7, cls: (m[5] ?? 'object').trim() }); }
  }
  if (j?.labels && Array.isArray(j.labels)) labels = j.labels.map((l: any) => ({ cls: String(l.label ?? l.class ?? l), score: Number(l.confidence ?? l.score ?? 1) }));
  else if (j?.counts && typeof j.counts === 'object') labels = Object.entries(j.counts).map(([cls, n]) => ({ cls, score: Number(n) }));
  return { boxes, labels };
}

// ---------------------------------------------------------------------------------------------
// VLA — action servers
// ---------------------------------------------------------------------------------------------

export interface VlaOptions { url: string; model?: string; format?: 'studio' | 'openpi' | 'openvla'; headers?: Record<string, string> }

export class VlaHttpModel implements VisionModel {
  id: string; name: string; tasks: VisionTask[] = ['vla_policy'];
  constructor(readonly opts: VlaOptions, id = 'vla') { this.id = id; this.name = `VLA ${opts.model ?? opts.format ?? ''} @ ${opts.url}`; }
  async run(frame: VisionFrame, opts: RunOptions): Promise<VisionResult> {
    if (!frame.dataUrl) throw new Error('VLA needs an encoded image (dataUrl)');
    const t0 = now();
    const fmt = this.opts.format ?? 'studio';
    const img = frame.dataUrl;
    const state = frame.proprio ?? {};
    let url = this.opts.url.replace(/\/$/, ''), body: any;
    if (fmt === 'openvla') { url += '/act'; body = { image: img, instruction: opts.prompt ?? '', unnorm_key: this.opts.model ?? 'bridge_orig' }; }
    else if (fmt === 'openpi') { url += '/infer'; body = { observation: { image: img, state: [...(state.joints ?? []), state.gripper ?? 0], prompt: opts.prompt ?? '' } }; }
    else { url += '/act'; body = { model: this.opts.model, image: img, instruction: opts.prompt ?? '', proprio: state }; }
    const res = await fetch(url, { method: 'POST', headers: { 'content-type': 'application/json', ...(this.opts.headers ?? {}) }, body: JSON.stringify(body) });
    if (!res.ok) throw new Error(`VLA server ${res.status}: ${(await res.text()).slice(0, 200)}`);
    const j = await res.json();
    const actions: number[][] = Array.isArray(j.actions) ? j.actions : Array.isArray(j.action) ? (Array.isArray(j.action[0]) ? j.action : [j.action]) : Array.isArray(j) ? (Array.isArray(j[0]) ? j : [j]) : [];
    return { task: 'vla_policy', model: this.id, boxes: [], action: actions[0], actionChunk: actions, text: j.text, latencyMs: now() - t0, raw: j };
  }
}

/**
 * Apply a normalised 7-DoF action (dx dy dz droll dpitch dyaw gripper ∈ [-1,1]) to a TCP pose. Deltas are
 * expressed in the camera frame by default (eye-in-hand policies), scaled to mm/deg per step.
 */
export function applyVlaAction(tcp: Mat4, action: number[], opts: { scaleMm?: number; scaleDeg?: number; frame?: 'tool' | 'world' } = {}): { pose: Mat4; gripper: number } {
  const sMm = opts.scaleMm ?? 20, sDeg = ((opts.scaleDeg ?? 5) * Math.PI) / 180;
  const [dx = 0, dy = 0, dz = 0, dr = 0, dp = 0, dyaw = 0, g = 0] = action;
  const delta = multiply(multiply(multiply(transl(dx * sMm, dy * sMm, dz * sMm), rotx(dr * sDeg)), roty(dp * sDeg)), rotz(dyaw * sDeg));
  const pose = opts.frame === 'world' ? multiply(delta, tcp) : multiply(tcp, delta);
  return { pose, gripper: g };
}

// ---------------------------------------------------------------------------------------------
// ROS 2 — vision_msgs/Detection2DArray via rosbridge
// ---------------------------------------------------------------------------------------------

export interface RosLike { subscribe(topic: string, type: string, cb: (msg: any) => void): void }

export class Ros2VisionModel implements VisionModel {
  id: string; name: string; tasks: VisionTask[] = ['detect', 'track', 'segment', 'pose'];
  private latest: any = null;
  constructor(ros: RosLike, readonly topic: string, id = 'ros2', type = 'vision_msgs/msg/Detection2DArray') { this.id = id; this.name = `ROS 2 ${topic}`; ros.subscribe(topic, type, (m) => { this.latest = m; }); }
  async run(frame: VisionFrame, opts: RunOptions): Promise<VisionResult> {
    const t0 = now();
    const boxes = this.latest ? parseDetection2DArray(this.latest, opts.confidence) : [];
    void frame;
    return { task: opts.task, model: this.id, boxes, latencyMs: now() - t0, raw: this.latest };
  }
}

export function parseDetection2DArray(msg: any, confidence: number): Box2D[] {
  const out: Box2D[] = [];
  for (const d of msg.detections ?? []) {
    const bb = d.bbox ?? {}; const c = bb.center?.position ?? bb.center ?? { x: 0, y: 0 };
    const hyp = (d.results ?? [])[0]?.hypothesis ?? (d.results ?? [])[0] ?? {};
    const score = Number(hyp.score ?? 1);
    if (score < confidence) continue;
    const b: Box2D = { x: c.x - bb.size_x / 2, y: c.y - bb.size_y / 2, w: bb.size_x, h: bb.size_y, score, cls: String(hyp.class_id ?? d.id ?? 'object'), id: d.tracking_id ? Number(String(d.tracking_id).replace(/\D/g, '')) || undefined : undefined };
    const pose = hyp.pose?.pose?.position ?? (d.results ?? [])[0]?.pose?.pose?.position;
    if (pose) b.p = [pose.x * 1000, pose.y * 1000, pose.z * 1000];
    out.push(b);
  }
  return out;
}

// ---------------------------------------------------------------------------------------------
// Tracking — ByteTrack association with a constant-velocity filter
// ---------------------------------------------------------------------------------------------

export interface Track { id: number; cls: string; x: number; y: number; w: number; h: number; vx: number; vy: number; score: number; age: number; hits: number; lost: number; last: Box2D; p?: Vec3 }

export class ByteTracker {
  tracks: Track[] = [];
  private nextId = 1;
  constructor(readonly opts: { highThresh?: number; lowThresh?: number; iouThresh?: number; maxLost?: number; minHits?: number; alpha?: number } = {}) {}
  /** dt in seconds; returns confirmed tracks as boxes with ids. */
  update(dets: Box2D[], dt: number): Box2D[] {
    const hi = this.opts.highThresh ?? 0.5, lo = this.opts.lowThresh ?? 0.1, thr = this.opts.iouThresh ?? 0.3, maxLost = this.opts.maxLost ?? 15, minHits = this.opts.minHits ?? 2, alpha = this.opts.alpha ?? 0.6;
    for (const t of this.tracks) { t.x += t.vx * dt; t.y += t.vy * dt; t.age++; }
    const high = dets.filter((d) => d.score >= hi), low = dets.filter((d) => d.score >= lo && d.score < hi);
    const unmatchedT = new Set(this.tracks.map((_, i) => i));
    const matchSet = (cands: Box2D[], pool: number[], iouMin: number) => {
      const pairs: Array<[number, number, number]> = [];
      pool.forEach((ti) => cands.forEach((d, di) => { const t = this.tracks[ti]; if (t.cls !== d.cls) return; const v = iou(t, d); if (v >= iouMin) pairs.push([v, ti, di]); }));
      pairs.sort((a, b) => b[0] - a[0]);
      const usedD = new Set<number>(), usedT = new Set<number>();
      for (const [, ti, di] of pairs) {
        if (usedD.has(di) || usedT.has(ti)) continue;
        usedD.add(di); usedT.add(ti); unmatchedT.delete(ti);
        const t = this.tracks[ti], d = cands[di];
        const nvx = dt > 0 ? (d.x - t.x) / dt : 0, nvy = dt > 0 ? (d.y - t.y) / dt : 0;
        t.vx = alpha * t.vx + (1 - alpha) * nvx; t.vy = alpha * t.vy + (1 - alpha) * nvy;
        t.x = d.x; t.y = d.y; t.w = d.w; t.h = d.h; t.score = d.score; t.hits++; t.lost = 0; t.last = d; if (d.p) t.p = d.p;
      }
      return cands.filter((_, di) => !usedD.has(di));
    };
    const remainingHigh = matchSet(high, [...unmatchedT], thr);
    matchSet(low, [...unmatchedT].filter((ti) => this.tracks[ti].lost === 0 || this.tracks[ti].hits >= minHits), thr);
    for (const ti of unmatchedT) this.tracks[ti].lost++;
    for (const d of remainingHigh) this.tracks.push({ id: this.nextId++, cls: d.cls, x: d.x, y: d.y, w: d.w, h: d.h, vx: 0, vy: 0, score: d.score, age: 1, hits: 1, lost: 0, last: d, p: d.p });
    this.tracks = this.tracks.filter((t) => t.lost <= maxLost);
    return this.tracks.filter((t) => t.hits >= minHits && t.lost === 0).map((t) => ({ ...t.last, x: t.x, y: t.y, w: t.w, h: t.h, id: t.id, vx: t.vx, vy: t.vy, p: t.p ?? t.last.p }));
  }
  reset(): void { this.tracks = []; this.nextId = 1; }
}

// ---------------------------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------------------------

export interface ModelContext { ros?: RosLike; serverBase?: string | null }

/** Instantiate the executable model for a task from the camera's configuration. */
export function createModel(cfg: VisionStackConfig, task: VisionTask, ctx: ModelContext = {}): VisionModel {
  const specId = cfg.models[task] ?? cfg.models.detect;
  const spec = VISION_MODELS.find((m) => m.id === specId) ?? VISION_MODELS[0];
  if (task === 'vlm_query' && cfg.vlm?.url && cfg.runtime !== 'simulated') return new VlmOpenAIModel({ url: cfg.vlm.url, model: cfg.vlm.model, apiKey: cfg.vlm.apiKey }, spec.id);
  if (task === 'vla_policy' && cfg.vla?.url && cfg.runtime !== 'simulated') return new VlaHttpModel({ url: cfg.vla.url, model: cfg.vla.model, format: cfg.vla.format }, spec.id);
  if ((task === 'detect' || task === 'segment' || task === 'keypoints' || task === 'classify') && cfg.runtime === 'vlm' && cfg.vlm?.url) return new VlmOpenAIModel({ url: cfg.vlm.url, model: cfg.vlm.model, apiKey: cfg.vlm.apiKey }, spec.id);
  if (cfg.runtime === 'onnx' && cfg.modelUrl && (task === 'detect' || task === 'segment' || task === 'keypoints' || task === 'classify')) return new OnnxYoloModel({ url: cfg.modelUrl, task, classes: cfg.classes, inputSize: cfg.inputSize ?? 640 }, spec.id);
  if (cfg.runtime === 'server' && ctx.serverBase) return new HttpModel({ url: `${ctx.serverBase}/vision/infer`, model: cfg.serverModel ?? spec.id }, spec.id);
  if (cfg.runtime === 'ros2' && ctx.ros && cfg.rosTopic) return new Ros2VisionModel(ctx.ros, cfg.rosTopic, spec.id);
  return new SimulatedModel(spec, cfg.seed ?? 1);
}
