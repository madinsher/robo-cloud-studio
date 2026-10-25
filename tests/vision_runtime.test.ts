/**
 * T9 vision — runtime behaviour: tracker (α-β filter + ByteTrack association), real-model runtimes without ground truth
 * (explicit errors, no fabricated detections), the "simulate" switch, ONNX on a downscaled frame, ONNX on an RGB-D camera
 * (depth always produced → targets), VLA steps for a robot that is not at the world origin, the bundled ORT WASM.
 * Ported from tests/_probe/vision_runtime.test.ts (10 probes).
 */
import { it, expect, describe } from 'vitest';
import fs from 'node:fs';
import path from 'node:path';
import { demos } from '../src/demos';
import { Station, Camera, ItemType, SceneObject } from '../src/core/items/item';
import { createRobotFromLibrary } from '../src/core/items/library';
import { mul, transl, rotx, DEG, transformPoint } from '../src/core/math/pose';
import { setVisionStack, getVisionStack, VisionStackConfig } from '../src/vision/stack';
import { VisionRuntime, solveTcpAbs, fpText } from '../src/vision/pipeline';
import { ByteTracker, letterbox, applyVlaAction, UnavailableModel, SimulatedModel, iou } from '../src/vision/models';
import { VISION_MODELS } from '../src/vision/stack';

function cell() {
  const st = demos.find((d) => d.id === 'pickplace')!.build();
  const boxes = st.itemsOfType<SceneObject>(ItemType.OBJECT).filter((o) => /^Box \d$/.test(o.name));
  const cam = st.addChild(new Camera('Cell camera'));
  cam.kind = 'depth'; cam.fov = 60; cam.width = 640; cam.height = 480; cam.near = 100; cam.far = 4000;
  cam.setPose(mul(transl(1020, -150, 1700), rotx(180 * DEG)));
  const cfg: VisionStackConfig = { tasks: ['detect', 'pose', 'grasp'], modality: 'rgbd', environment: 'factory', sensor: 'realsense_d455', compute: 'intel_ipc', models: { detect: 'yolov8s', pose: 'depth_centroid', grasp: 'approach_grasp' }, classes: ['box'], confidence: 0.35, iou: 0.5, runtime: 'simulated', workingDistance: 1.1, simulate: true, seed: 3 };
  setVisionStack(cam, cfg);
  return { st, boxes, cam, cfg };
}

describe('tracker', () => {
  it('velocity converges to the true image velocity (constant 100 px/s)', () => {
    const tr = new ByteTracker({ minHits: 1 });
    let out: any[] = [];
    for (let k = 0; k < 40; k++) out = tr.update([{ x: 50 + 100 * k * 0.1, y: 100, w: 30, h: 30, score: 0.9, cls: 'a' }], 0.1);
    expect(out[0].vx).toBeGreaterThan(97);
    expect(out[0].vx).toBeLessThan(103);
    expect(Math.abs(out[0].x - (50 + 100 * 3.9))).toBeLessThan(1); // the filtered box follows the detection
  });
  it('a fast object (20 px/frame, 30 px box) keeps one id; two crossing classes keep theirs', () => {
    const tr = new ByteTracker({ minHits: 1 });
    const ids = new Set<number>();
    for (let k = 0; k < 20; k++) { const o = tr.update([{ x: 20 * k, y: 100, w: 30, h: 30, score: 0.9, cls: 'a' }], 1 / 15); if (o[0]) ids.add(o[0].id!); }
    expect(ids.size).toBe(1);
    const tr2 = new ByteTracker({ minHits: 1 });
    const byCls = new Map<string, Set<number>>();
    for (let k = 0; k < 15; k++) {
      const o = tr2.update([{ x: 10 * k, y: 100, w: 30, h: 30, score: 0.9, cls: 'a' }, { x: 300 - 10 * k, y: 100, w: 30, h: 30, score: 0.9, cls: 'b' }], 1 / 15);
      for (const b of o) { const s = byCls.get(b.cls) ?? new Set(); s.add(b.id!); byCls.set(b.cls, s); }
    }
    expect([...byCls.values()].map((s) => s.size)).toEqual([1, 1]);
  });
  // V6 F6 (Vision tab overlay 24-64 px off a part): a 34x15 false-positive track 120 px away captured the 246x228 box of a
  // part detected for the first time, and the α-filter pulled the part's box towards the false positive
  it('a small false-positive track does not capture a large part detected next to it', () => {
    const tr = new ByteTracker({ minHits: 1 });
    tr.update([{ x: 1000, y: 432, w: 34, h: 15, score: 0.5, cls: 'box' }], 1 / 15);
    const out = tr.update([{ x: 825, y: 427, w: 246, h: 228, score: 0.7, cls: 'box' }], 1 / 15);
    const part = out.find((b) => b.w > 100)!;
    expect([part.x, part.y, part.w, part.h]).toEqual([825, 427, 246, 228]);
    expect(part.id).toBe(2); // a new track, not the false positive's
  });
});

describe('real-model runtimes', () => {
  it('runtime "onnx" without a model: an explicit warning and no fabricated detections', async () => {
    const { st, cam, cfg } = cell();
    setVisionStack(cam, { ...cfg, runtime: 'onnx', simulate: false });
    const rt = new VisionRuntime(cam, getVisionStack(cam)!);
    const out = await rt.step(st, { time: 0 });
    expect(rt.models.get('detect')).toBeInstanceOf(UnavailableModel);
    expect(out.warnings.join(' ')).toMatch(/no ONNX model configured/);
    expect(out.detections.length).toBe(0);
    expect(out.targets.length).toBe(0);
  });
  it('runtime "server" with no studio server: not the simulated model, an explicit error', async () => {
    const { st, cam, cfg } = cell();
    setVisionStack(cam, { ...cfg, runtime: 'server', simulate: false });
    const rt = new VisionRuntime(cam, getVisionStack(cam)!);
    const out = await rt.step(st, { time: 0 });
    expect(rt.models.get('detect')).not.toBeInstanceOf(SimulatedModel);
    expect(out.warnings.join(' ')).toMatch(/no studio server/);
    expect(out.detections.length).toBe(0);
  });
  it('a simulated model without ground truth reports nothing (no false positives out of thin air)', async () => {
    const m = new SimulatedModel({ id: 'yolov8s', name: 'YOLOv8s', accuracy: 0.6 } as any, 1);
    for (let k = 0; k < 20; k++) {
      const r = await m.run({ width: 640, height: 480, K: { fx: 500, fy: 500, cx: 320, cy: 240, width: 640, height: 480 }, camWorld: new Float64Array(16) as any, time: k } as any, { task: 'detect', classes: ['box'], confidence: 0.1, iou: 0.5, seed: k, simulate: true });
      expect(r.boxes.length).toBe(0);
    }
  });
  it('simulate:false = exact detector: no false positives, boxes equal the ID-buffer truth', async () => {
    const { st, cam, cfg } = cell();
    setVisionStack(cam, { ...cfg, simulate: false });
    const rt = new VisionRuntime(cam, getVisionStack(cam)!);
    let fps = 0, jit = 0, n = 0;
    for (let k = 0; k < 10; k++) {
      const out = await rt.step(st, { time: k / 10 });
      fps += out.detections.filter((d) => !d.truthId).length;
      for (const d of out.detections) { const t = out.frame.truth!.find((x) => x.id === d.truthId)!; jit += Math.abs(d.x - t.box.x) + Math.abs(d.w - t.box.w); n++; }
    }
    expect(fps).toBe(0);
    expect(n).toBe(30);
    expect(jit).toBe(0);
  });
  it('default simulated detector (noise on): boxes stay on the parts like a trained detector (V6 F6: 40-64 px off)', async () => {
    // a 300 px part, the wizard's YOLOv10 (COCO mAP 0.46): IoU with the truth box ~0.9+, edges within a few % of the box
    const spec = VISION_MODELS.find((m) => m.id === 'yolov10')!;
    const model = new SimulatedModel(spec, 1);
    const truth = { id: 'object:a', cls: 'box', box: { x: 400, y: 300, w: 300, h: 300 }, p: [0, 0, 0] as [number, number, number], z: 1000, size: 150, visible: true };
    const ious: number[] = [], edges: number[] = [];
    for (let k = 0; k < 200; k++) {
      const r = await model.run({ width: 1280, height: 1024, K: { fx: 1000, fy: 1000, cx: 640, cy: 512, width: 1280, height: 1024 }, camWorld: new Float64Array(16) as any, time: k, truth: [truth] }, { task: 'detect', classes: ['box'], confidence: 0.4, iou: 0.5, seed: k, simulate: true });
      const d = r.boxes.find((b) => b.truthId === 'object:a');
      if (!d) continue;
      ious.push(iou(d, truth.box));
      edges.push(Math.max(Math.abs(d.x - 400), Math.abs(d.y - 300), Math.abs(d.x + d.w - 700), Math.abs(d.y + d.h - 600)));
    }
    expect(ious.length).toBeGreaterThan(120); // recall ≈ 0.76 (missed detections remain part of the model)
    const mean = ious.reduce((a, b) => a + b, 0) / ious.length;
    expect(mean).toBeGreaterThan(0.9);
    edges.sort((a, b) => a - b);
    expect(edges[Math.floor(0.9 * edges.length)]).toBeLessThan(25); // 90 % of the boxes: every edge within 25 px (8 %)
  });
  it('the run statistics name the false positives that got a 3D position (Vision tab)', () => {
    expect(fpText({ fp: 0, fp3d: 0 })).toBe(' · 0 false positives');
    expect(fpText({ fp: 2, fp3d: 0 })).toMatch(/2 false positives \(none got a 3D position\)/);
    expect(fpText({ fp: 3, fp3d: 1 })).toMatch(/1 with a 3D position/);
  });
  it('letterbox of a 1280-px render for a 1920-px camera reads inside the buffer (0 NaN) and keeps the image', () => {
    const camW = 1920, camH = 1080, w = 1280, h = Math.round((w * camH) / camW);
    const rgba = new Uint8ClampedArray(w * h * 4);
    for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) { const i = (y * w + x) * 4; rgba[i] = x < w / 2 ? 255 : 0; rgba[i + 3] = 255; }
    const { data, info } = letterbox(rgba, camW, camH, 640);
    expect(data.some((v) => Number.isNaN(v))).toBe(false);
    // left half red, right half black, in the letterboxed frame of the 1920-px coordinates
    const row = info.padY + Math.round(info.scale * camH / 2);
    expect(data[row * 640 + info.padX + 10]).toBeCloseTo(1, 5);
    expect(data[row * 640 + info.padX + Math.round(info.scale * camW) - 10]).toBeCloseTo(0, 5);
  });
  it('ONNX model on an RGB-D camera: the studio provides the depth image, so real detections get 3D and targets', async () => {
    const { st, cam, cfg, boxes } = cell();
    // the fixture model always answers [270,120,100,80] "b" and [80,0,40,40] "a": put Box 2 under the first one (its top
    // face projects to u 291..349, v 131..189); the second one stays on the bare table
    boxes[1].setPose(transl(320, 357, 500));
    const url = path.join(process.cwd(), 'tests', 'fixtures', 'fake_yolo.onnx');
    setVisionStack(cam, { ...cfg, runtime: 'onnx', modelUrl: url, classes: ['a', 'b'], simulate: false });
    const rt = new VisionRuntime(cam, getVisionStack(cam)!);
    const out = await rt.step(st, { time: 0, rgba: new Uint8ClampedArray(640 * 480 * 4).fill(128) });
    expect(out.frame.depth).toBeDefined();
    expect(out.frame.truth).toBeUndefined(); // real runtime: no ground truth used
    expect(out.detections.length).toBe(2);
    const onBox = out.detections.find((d) => d.cls === 'b')!, onTable = out.detections.find((d) => d.cls === 'a')!;
    // the real detection gets 3D: the top face of Box 2 (no truth, no object model: the face centre)
    const top = transformPoint(boxes[1].poseAbs(), [0, 0, 120]);
    expect(Math.hypot(onBox.p![0] - top[0], onBox.p![1] - top[1], onBox.p![2] - top[2])).toBeLessThan(10);
    expect(onBox.z!).toBeGreaterThan(0);
    // a box on the bare table: the surface continues all around it, nothing stands out of the table: no 3D, no target
    expect(onTable.p).toBeUndefined();
    expect(out.targets.map((t) => t.cls)).toEqual(['b']);
  }, 60000);
  it('the ONNX Runtime WASM the browser loads is a local package asset (no CDN)', () => {
    const src = fs.readFileSync(path.join(process.cwd(), 'src', 'vision', 'models.ts'), 'utf8');
    expect(src).not.toMatch(/cdn\.jsdelivr\.net\/npm\/onnxruntime-web/);
    const m = /import\('(onnxruntime-web\/[^'?]+)\?url'\)/.exec(src);
    expect(m).toBeTruthy();
    const pkg = JSON.parse(fs.readFileSync(path.join(process.cwd(), 'node_modules', 'onnxruntime-web', 'package.json'), 'utf8'));
    const rel = pkg.exports['./' + m![1].slice('onnxruntime-web/'.length)];
    expect(typeof rel).toBe('string');
    expect(fs.statSync(path.join(process.cwd(), 'node_modules', 'onnxruntime-web', rel)).size).toBeGreaterThan(1e6);
  });
});

describe('VLA on a robot that is not at the world origin', () => {
  it('a 20 mm tool-Z step reaches the commanded TCP (base-relative IK via solveTcpAbs)', () => {
    const st = new Station();
    const robot = st.addChild(createRobotFromLibrary('UR10e'));
    robot.setPose(transl(0, 0, 700));
    robot.setJoints([0, -90, -90, -90, 90, 0]);
    const tcp = robot.poseTCPAbs();
    const { pose } = applyVlaAction(tcp, [0, 0, 1, 0, 0, 0, 0], { scaleMm: 20 });
    const ik = solveTcpAbs(robot, pose);
    expect(ik.ok).toBe(true);
    robot.setJoints(ik.joints);
    const after = robot.poseTCPAbs();
    expect(Math.hypot(after[12] - pose[12], after[13] - pose[13], after[14] - pose[14])).toBeLessThan(0.01);
    expect(Math.hypot(after[12] - tcp[12], after[13] - tcp[13], after[14] - tcp[14])).toBeCloseTo(20, 3);
  });
  it('a camera-frame action (eye-in-hand policy) moves the TCP along the camera axes', () => {
    const tcp = transl(500, 0, 800);
    const camAxes = mul(transl(0, 0, 0), rotx(180 * DEG)); // camera looking down: +x right = world +x, +y down = world -y
    const { pose } = applyVlaAction(tcp, [0, 1, 0, 0, 0, 0, 0], { scaleMm: 10, axes: camAxes });
    expect([pose[12], pose[13], pose[14]].map((v) => +v.toFixed(6))).toEqual([500, -10, 800]);
  });
});
