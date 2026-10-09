import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { Station, Camera, Frame, Target, ItemType, SceneObject } from '../src/core/items/item';
import { FieldItem, cropParams } from '../src/agri/items';
import { generateOrchard, rectPolygon } from '../src/agri/orchard';
import { mul, transl, rotz, roty, rotx, DEG, transformPoint, identity, xyzrpwToPose } from '../src/core/math/pose';
import { VISION_SENSORS, VISION_MODELS, COMPUTE_TARGETS, recommendVisionStacks, configFromRecommendation, setVisionStack, getVisionStack, errorBudget, VisionStackConfig } from '../src/vision/stack';
import { SimulatedModel, ByteTracker, decodeYolo, letterbox, protoMask, parseVlmAnswer, parseDetectionsJson, parseDetection2DArray, applyVlaAction, OnnxYoloModel, nms } from '../src/vision/models';
import { VisionRuntime, captureTruth, createTargetsFromOutput, followCommand, servoDelta, analyseCloud, summarize } from '../src/vision/pipeline';
import { parsePCD, parsePLY, writePCD, writePLY, voxelDownsample, simulateLidar3D, simulateDepthCamera, fitGroundPlane, euclideanCluster, pointCount, makeCloud, fitLine2D } from '../src/vision/pointcloud';
import { rigidTransform, targetPoseFromPoint, intrinsicsFromCamera, depthSigmaMm, rangeFromSize, backproject, project } from '../src/vision/camera_model';
import { generateRosVisionPackage, rosVisionPackageZip } from '../src/vision/vision_ros';
import { unzipSync, strFromU8 } from 'fflate';

function orchardStation(): { st: Station; cam: Camera; field: FieldItem } {
  const st = new Station();
  const field = st.addChild(new FieldItem('F'));
  field.polygon = rectPolygon(20000, 10000);
  field.crop = cropParams('apple', { headland: 2000 });
  const rows = generateOrchard(field, 3);
  const row = rows[0];
  const [x, y] = row.pointAt(row.length() / 2, -1500);
  const cam = st.addChild(new Camera('Cam'));
  cam.fov = 70; cam.width = 640; cam.height = 480; cam.far = 8000; cam.kind = 'depth';
  cam.setPose(mul(transl(x, y, 1500), rotz(90 * DEG), roty(90 * DEG)));
  return { st, cam, field };
}

describe('vision catalogue and recommender', () => {
  it('has consistent catalogue entries', () => {
    const ids = new Set<string>();
    for (const m of VISION_MODELS) { expect(ids.has(m.id)).toBe(false); ids.add(m.id); expect(m.tasks.length).toBeGreaterThan(0); expect(m.source.length).toBeGreaterThan(0); }
    expect(VISION_SENSORS.length).toBeGreaterThan(10);
    expect(COMPUTE_TARGETS.some((c) => c.class === 'edge_low')).toBe(true);
    expect(VISION_MODELS.filter((m) => m.family === 'vlm').length).toBeGreaterThanOrEqual(4);
    expect(VISION_MODELS.filter((m) => m.family === 'vla').length).toBeGreaterThanOrEqual(4);
  });
  it('recommends a sunlight-robust depth camera + YOLO for orchard picking and warns about RealSense in sun', () => {
    const recs = recommendVisionStacks({ tasks: ['detect', 'track', 'pose', 'grasp'], environment: 'orchard', workingDistance: 1.2, needs3D: true, moving: true });
    expect(recs.length).toBeGreaterThan(5);
    expect(['zed_x', 'oak_d_pro']).toContain(recs[0].sensor.id);
    expect(recs[0].models.detect!.family === 'yolo' || recs[0].models.detect!.family === 'detr').toBe(true);
    expect(recs[0].models.track!.id).toBe('bytetrack');
    expect(recs[0].models.grasp!.id).toBe('approach_grasp');
    expect(recs[0].depthErrorM).not.toBeNull();
    const rs = recs.find((r) => r.sensor.id === 'realsense_d455')!;
    expect(rs.warnings.join(' ')).toMatch(/sunlight/);
    expect(rs.score).toBeLessThan(recs[0].score);
    const mono = recs.find((r) => r.sensor.id === 'usb_rgb')!;
    expect(mono.warnings.join(' ')).toMatch(/no depth/);
  });
  it('picks LiDAR + PCL for point-cloud tasks, VLM/VLA models when asked, and open-vocabulary detectors on request', () => {
    const lidar = recommendVisionStacks({ tasks: ['cloud_detect', 'cloud_segment'], environment: 'orchard', moving: true });
    expect(lidar[0].sensor.modality).toBe('lidar3d');
    expect(['pcl_pipeline', 'pointpillars']).toContain(lidar[0].models.cloud_detect!.id);
    const vlm = recommendVisionStacks({ tasks: ['detect', 'vlm_query', 'vla_policy'], environment: 'factory', compute: 'edge_high', workingDistance: 0.8 });
    expect(vlm[0].models.vlm_query!.family).toBe('vlm');
    expect(vlm[0].models.vla_policy!.family).toBe('vla');
    const ov = recommendVisionStacks({ tasks: ['detect'], environment: 'warehouse', openVocabulary: true, compute: 'edge_mid' });
    expect(ov[0].models.detect!.openVocabulary).toBe(true);
    const low = recommendVisionStacks({ tasks: ['detect'], environment: 'warehouse', compute: 'mcu', modalities: ['stereo'] });
    expect(low[0].compute.id).toBe('oak_myriad');
    expect(low[0].sensor.id).toBe('oak_d_pro');
    const night = recommendVisionStacks({ tasks: ['detect'], environment: 'orchard', night: true, modalities: ['stereo'] });
    expect(night.find((r) => r.sensor.id === 'zed_x')!.warnings.join(' ')).toMatch(/darkness/);
  });
  it('stores the configuration on the camera and computes an error budget', () => {
    const cam = new Camera('C');
    const req = { tasks: ['detect', 'pose'] as const, environment: 'orchard' as const, workingDistance: 1.5 };
    const rec = recommendVisionStacks({ ...req, tasks: [...req.tasks] })[0];
    const cfg = configFromRecommendation(rec, { ...req, tasks: [...req.tasks] }, ['apple']);
    setVisionStack(cam, cfg);
    expect(getVisionStack(cam)!.sensor).toBe(rec.sensor.id);
    const eb = errorBudget(cfg);
    expect(eb.depthSigmaMm).toBeGreaterThan(0);
    expect(eb.pixelMm).toBeGreaterThan(0);
    const st = new Station(); st.addChild(cam);
    const copy = Station.deserialize(st.serialize()).itemsOfType<Camera>(ItemType.CAMERA)[0];
    expect(getVisionStack(copy)!.models.detect).toBe(cfg.models.detect);
  });
});

describe('simulated perception pipeline', () => {
  const baseCfg = (over: Partial<VisionStackConfig> = {}): VisionStackConfig => ({ tasks: ['detect', 'track', 'pose', 'grasp'], modality: 'stereo', environment: 'orchard', sensor: 'zed_x', compute: 'jetson_orin_nx', models: { detect: 'yolov8s', track: 'bytetrack', pose: 'depth_centroid', grasp: 'approach_grasp' }, classes: ['apple'], confidence: 0.3, iou: 0.5, runtime: 'simulated', workingDistance: 1.5, simulate: true, seed: 3, ...over });
  it('captures ground truth (fruit, trunks) from the camera and reports detection quality', async () => {
    const { st, cam } = orchardStation();
    const truth = captureTruth(st, cam, { maxRange: 6000 });
    expect(truth.filter((t) => t.cls === 'apple').length).toBeGreaterThan(20);
    expect(truth.some((t) => t.cls === 'trunk')).toBe(true);
    expect(truth.every((t) => t.box.x >= 0 && t.box.x + t.box.w <= 640.01)).toBe(true);
    setVisionStack(cam, baseCfg());
    const rt = new VisionRuntime(cam, getVisionStack(cam)!);
    const out = await rt.step(st, { time: 0 });
    expect(out.detections.length).toBeGreaterThan(5);
    const st1 = rt.stats;
    expect(st1.tp).toBeGreaterThan(st1.fn); // recall > 50 %
    expect(st1.tp / Math.max(1, st1.tp + st1.fp)).toBeGreaterThan(0.6);
    // 3D positions: error consistent with the stereo error model at ~1.5–3 m
    const withP = out.detections.filter((b) => b.p && b.truthId);
    expect(withP.length).toBeGreaterThan(3);
    const truthById = new Map(out.frame.truth!.map((t) => [t.id, t]));
    const errs = withP.map((b) => { const t = truthById.get(b.truthId!)!; return Math.hypot(b.p![0] - t.p[0], b.p![1] - t.p[1], b.p![2] - t.p[2]); });
    const mean = errs.reduce((a, b) => a + b, 0) / errs.length;
    expect(mean).toBeLessThan(120);
    expect(mean).toBeGreaterThan(1);
    expect(summarize(out, rt.cfg)).toMatch(/apple/);
  });
  it('tracks ids across frames and creates grasp/approach targets under a Vision frame', async () => {
    const { st, cam } = orchardStation();
    setVisionStack(cam, baseCfg({ tasks: ['detect', 'track', 'pose', 'grasp', 'follow'] }));
    const rt = new VisionRuntime(cam, getVisionStack(cam)!);
    let out = await rt.step(st, { time: 0 });
    for (let i = 1; i < 5; i++) out = await rt.step(st, { time: i / 15 });
    const ids = out.detections.filter((b) => b.id !== undefined).map((b) => b.id!);
    expect(ids.length).toBeGreaterThan(3);
    expect(new Set(ids).size).toBe(ids.length);
    const before = new Set(ids);
    out = await rt.step(st, { time: 5 / 15 });
    const after = out.detections.filter((b) => b.id !== undefined).map((b) => b.id!);
    expect(after.filter((i) => before.has(i)).length).toBeGreaterThan(after.length * 0.6); // ids stable
    expect(out.targets.length).toBeGreaterThan(3);
    const targets = createTargetsFromOutput(st, cam, out, { robotId: null });
    expect(targets.length).toBe(out.targets.length * 2);
    const frame = st.children.find((c) => c instanceof Frame && c.name === 'Vision Cam') as Frame;
    expect(frame.children.filter((c) => c instanceof Target).length).toBe(targets.length);
    // approach target lies 100 mm behind the grasp along the tool Z
    const g = out.targets[0];
    const d = Math.hypot(g.grasp[12] - g.approach[12], g.grasp[13] - g.approach[13], g.grasp[14] - g.approach[14]);
    expect(d).toBeCloseTo(100, 0);
    // follow command steers towards the tracked box
    const K = intrinsicsFromCamera(cam);
    const right = followCommand({ x: 600, y: 200, w: 20, h: 20, score: 1, cls: 'apple', z: 4000 }, K, { desiredRange: 2000 });
    expect(right.omega).toBeLessThan(0); // target right of centre -> turn right (negative yaw)
    expect(right.v).toBeGreaterThan(0);
    expect(followCommand(null, K).state).toBe('lost');
    const delta = servoDelta({ x: 300, y: 220, w: 20, h: 20, score: 1, cls: 'apple', z: 500 }, K, { standoff: 150 });
    expect(delta![2]).toBeGreaterThan(0);
  });
  it('mono cameras estimate range from the object size prior; classification, segmentation and keypoints produce attributes, masks and points', async () => {
    const { st, cam } = orchardStation();
    cam.kind = 'rgb';
    setVisionStack(cam, baseCfg({ modality: 'mono', sensor: 'gs_rgb', tasks: ['detect', 'segment', 'keypoints', 'classify', 'pose'], models: { detect: 'yolov8s', segment: 'yolov8_seg', keypoints: 'yolov8_pose', classify: 'efficientnet', pose: 'size_prior_position' }, objectSizeMm: 75 }));
    const rt = new VisionRuntime(cam, getVisionStack(cam)!);
    const out = await rt.step(st, { time: 0 });
    expect(out.detections.some((b) => b.mask)).toBe(true);
    expect(out.detections.some((b) => b.keypoints && b.keypoints.length)).toBe(true);
    expect(out.results.classify?.labels?.length).toBeGreaterThan(0);
    const withZ = out.detections.filter((b) => b.z && b.truthId);
    expect(withZ.length).toBeGreaterThan(3);
    const truthById = new Map(out.frame.truth!.map((t) => [t.id, t]));
    const relErr = withZ.map((b) => Math.abs(b.z! - truthById.get(b.truthId!)!.z) / truthById.get(b.truthId!)!.z);
    expect(relErr.reduce((a, b) => a + b, 0) / relErr.length).toBeLessThan(0.35);
    expect(out.detections.some((b) => b.attr && 'ripe' in b.attr)).toBe(true);
  });
  it('VLM queries and VLA policies run through the simulated adapters', async () => {
    const { st, cam } = orchardStation();
    setVisionStack(cam, baseCfg({ tasks: ['vlm_query', 'vla_policy'], models: { vlm_query: 'qwen2_vl', vla_policy: 'pi0' }, vla: { url: '', model: 'pi0', instruction: 'pick the apple', actionScaleMm: 20, actionScaleDeg: 5 } }));
    const rt = new VisionRuntime(cam, getVisionStack(cam)!);
    const out = await rt.step(st, { time: 0, prompt: 'How many apples do you see?', tcp: cam.poseAbs() });
    expect(out.text).toMatch(/apple/);
    expect(out.vla?.action.length).toBe(7);
    expect(out.vla!.tcp).toBeDefined();
    const moved = Math.hypot(out.vla!.tcp![12] - cam.poseAbs()[12], out.vla!.tcp![13] - cam.poseAbs()[13], out.vla!.tcp![14] - cam.poseAbs()[14]);
    expect(moved).toBeGreaterThan(0);
    expect(moved).toBeLessThanOrEqual(20 * Math.sqrt(3) + 1e-6);
  });
  it('3D LiDAR modality yields clusters, trunk shapes and row lines from the simulated cloud', async () => {
    const { st, cam } = orchardStation();
    cam.kind = 'lidar3d'; cam.setPose(mul(transl(cam.poseAbs()[12], cam.poseAbs()[13], 1200), rotz(90 * DEG)));
    setVisionStack(cam, baseCfg({ modality: 'lidar3d', sensor: 'ouster_os1', tasks: ['cloud_detect', 'cloud_segment'], models: { cloud_detect: 'pcl_pipeline', cloud_segment: 'patchwork' } }));
    const rt = new VisionRuntime(cam, getVisionStack(cam)!);
    const out = await rt.step(st, { time: 0, lidar: { channels: 16, hres: 1, range: 15000 } });
    expect(out.cloud).toBeDefined();
    expect(pointCount(out.cloud!.ground)).toBeGreaterThan(100);
    expect(out.cloud!.clusters.length).toBeGreaterThan(3);
    expect(out.cloud!.rows.length).toBeGreaterThanOrEqual(1);
    expect(out.results.cloud_detect!.boxes.every((b) => b.p)).toBe(true);
  });
});

describe('model adapters: YOLO decode, ONNX runtime, VLM / HTTP / ROS parsers, tracker, VLA', () => {
  it('letterboxes and decodes YOLOv8 / v5 / end-to-end layouts with NMS and masks', () => {
    const rgba = new Uint8ClampedArray(320 * 240 * 4).fill(200);
    const { data, info } = letterbox(rgba, 320, 240, 640);
    expect(data.length).toBe(3 * 640 * 640);
    expect(info.scale).toBe(2); expect(info.padY).toBe(80); expect(info.padX).toBe(0);
    const classes = ['a', 'b'];
    // v8 [1, 4+nc, N]
    const N = 4; const v8 = new Float32Array((4 + 2) * N);
    const put = (n: number, vals: number[]) => vals.forEach((v, c) => (v8[c * N + n] = v));
    put(0, [320, 240, 100, 80, 0.1, 0.9]); put(1, [322, 242, 100, 80, 0.1, 0.8]); put(2, [100, 100, 40, 40, 0.7, 0.1]); put(3, [10, 10, 5, 5, 0.01, 0.02]);
    const det = decodeYolo(v8, [1, 6, N], { classes, confidence: 0.25, iou: 0.5, task: 'detect', info, imgW: 320, imgH: 240 });
    expect(det.length).toBe(2);
    expect(det[0].cls).toBe('b'); expect(det[0].x).toBeCloseTo((320 - 50) / 2, 3); expect(det[0].y).toBeCloseTo((240 - 40 - 80) / 2, 3); expect(det[0].w).toBeCloseTo(50, 3);
    // v5 [1, N, 5+nc]
    const v5 = new Float32Array([320, 240, 100, 80, 0.9, 0.1, 0.95]);
    const d5 = decodeYolo(v5, [1, 1, 7], { classes, confidence: 0.25, iou: 0.5, task: 'detect', info, imgW: 320, imgH: 240 });
    expect(d5.length).toBe(1); expect(d5[0].score).toBeCloseTo(0.855, 3);
    // e2e [1, N, 6] xyxy conf cls
    const e2e = new Float32Array([270, 200, 370, 280, 0.8, 0]);
    const de = decodeYolo(e2e, [1, 1, 6], { classes: ['a', 'b', 'c'], confidence: 0.25, iou: 0.5, task: 'detect', info, imgW: 320, imgH: 240 });
    expect(de.length).toBe(1); expect(de[0].cls).toBe('a'); expect(de[0].w).toBeCloseTo(50, 3);
    // seg: [1, 4+nc+32, N] + proto [1,32,160,160]
    const seg = new Float32Array((4 + 2 + 32) * 1); [320, 240, 100, 80, 0.1, 0.9].forEach((v, i) => (seg[i] = v)); seg[6] = 5; // coef 0 positive
    const proto = new Float32Array(32 * 160 * 160); for (let i = 0; i < 160 * 160; i++) proto[i] = 1; // channel 0 = +1 everywhere
    const ds = decodeYolo(seg, [1, 38, 1], { classes, confidence: 0.25, iou: 0.5, task: 'segment', info, imgW: 320, imgH: 240 }, { data: proto, dims: [1, 32, 160, 160] });
    expect(ds[0].mask).toBeDefined();
    expect(ds[0].mask!.data.every((v) => v === 1)).toBe(true);
    // pose: 4+nc+3K
    const K = 2; const pose = new Float32Array(4 + 2 + 3 * K); [320, 240, 100, 80, 0.1, 0.9, 300, 200, 0.9, 340, 260, 0.8].forEach((v, i) => (pose[i] = v));
    const dp = decodeYolo(pose, [1, 12, 1], { classes, confidence: 0.25, iou: 0.5, task: 'keypoints', info, imgW: 320, imgH: 240, numKeypoints: K });
    expect(dp[0].keypoints!.length).toBe(2); expect(dp[0].keypoints![0][0]).toBeCloseTo(150, 3); expect(dp[0].keypoints![0][1]).toBeCloseTo(60, 3);
    // classification [1, nc]
    const cls = decodeYolo(new Float32Array([0.2, 0.7]), [1, 2], { classes, confidence: 0, iou: 0.5, task: 'classify', info, imgW: 320, imgH: 240 });
    expect(cls[0].cls).toBe('b');
    expect(nms([{ x: 0, y: 0, w: 10, h: 10, score: 0.9, cls: 'a' }, { x: 1, y: 1, w: 10, h: 10, score: 0.8, cls: 'a' }, { x: 1, y: 1, w: 10, h: 10, score: 0.8, cls: 'b' }], 0.5).length).toBe(2);
    expect(protoMask({ data: proto, dims: [1, 32, 160, 160] }, [1], { x: 0, y: 0, w: 4, h: 4, score: 1, cls: 'a' }, info, 320, 240).width).toBe(4);
  });
  it('runs a real ONNX model through onnxruntime-web (fixture with a constant YOLOv8 head)', async () => {
    const file = path.join(process.cwd(), 'tests', 'fixtures', 'fake_yolo.onnx');
    const model = new OnnxYoloModel({ url: file, task: 'detect', classes: ['a', 'b'], inputSize: 640, executionProviders: ['wasm'] });
    const rgba = new Uint8ClampedArray(640 * 480 * 4).fill(128);
    const K = intrinsicsFromCamera({ fov: 60, width: 640, height: 480 });
    const res = await model.run({ width: 640, height: 480, rgba, K, camWorld: identity(), time: 0 }, { task: 'detect', classes: ['a', 'b'], confidence: 0.25, iou: 0.5 });
    expect(res.boxes.length).toBe(2);
    expect(res.boxes[0].cls).toBe('b');
    expect(res.boxes[0].x + res.boxes[0].w / 2).toBeCloseTo(320, 2); // centre in image coordinates (letterbox pad removed)
    expect(res.boxes[0].y + res.boxes[0].h / 2).toBeCloseTo(240 - 80, 2);
    model.dispose();
  }, 60000);
  it('parses VLM answers (JSON 0-1000 boxes, PaliGemma loc tokens), HTTP detection JSON and vision_msgs', () => {
    const q = parseVlmAnswer('Here you go:\n```json\n{"detections":[{"label":"apple","box_2d":[100,200,300,400],"confidence":0.9}]}\n```', 1000, 500, { confidence: 0.3 });
    expect(q.boxes.length).toBe(1); expect(q.boxes[0].x).toBeCloseTo(100); expect(q.boxes[0].y).toBeCloseTo(100); expect(q.boxes[0].w).toBeCloseTo(200); expect(q.boxes[0].h).toBeCloseTo(100);
    const pg = parseVlmAnswer('<loc0256><loc0512><loc0768><loc1023> apple ; <loc0000><loc0000><loc0100><loc0100> leaf', 1024, 1024, { confidence: 0.3 });
    expect(pg.boxes.length).toBe(2); expect(pg.boxes[0].cls).toBe('apple'); expect(pg.boxes[0].x).toBeCloseTo(512); expect(pg.boxes[0].y).toBeCloseTo(256);
    const labels = parseVlmAnswer('{"labels":[{"label":"ripe","confidence":0.8},{"label":"unripe","confidence":0.2}]}', 10, 10, { confidence: 0 });
    expect(labels.labels![0].cls).toBe('ripe');
    const rf = parseDetectionsJson({ predictions: [{ x: 100, y: 100, width: 50, height: 40, class: 'apple', confidence: 0.77 }] }, 640, 480, { confidence: 0.3 });
    expect(rf.boxes[0].x).toBe(75); expect(rf.boxes[0].cls).toBe('apple');
    const ul = parseDetectionsJson([{ name: 'box', confidence: 0.6, box: { x1: 10, y1: 20, x2: 60, y2: 70 } }], 640, 480, { confidence: 0.3 });
    expect(ul.boxes[0].w).toBe(50);
    const studio = parseDetectionsJson({ detections: [{ x: 1, y: 2, w: 3, h: 4, score: 0.9, class: 'a', keypoints: [[1, 2, 0.5]], points: [[1, 2], [4, 2], [4, 6]] }] }, 640, 480, { confidence: 0.3 });
    expect(studio.boxes[0].keypoints!.length).toBe(1); expect(studio.boxes[0].mask).toBeDefined();
    const ros = parseDetection2DArray({ detections: [{ bbox: { center: { position: { x: 100, y: 50 } }, size_x: 20, size_y: 10 }, results: [{ hypothesis: { class_id: 'apple', score: 0.8 }, pose: { pose: { position: { x: 1, y: 0.5, z: 2 } } } }], tracking_id: '7' }] }, 0.3);
    expect(ros[0].x).toBe(90); expect(ros[0].id).toBe(7); expect(ros[0].p![2]).toBe(2000);
  });
  it('ByteTracker keeps ids through a missed frame and drops stale tracks', () => {
    const tr = new ByteTracker({ minHits: 1, maxLost: 2 });
    const b = (x: number, score = 0.9) => ({ x, y: 10, w: 20, h: 20, score, cls: 'a' });
    let out = tr.update([b(0)], 0.1); expect(out[0].id).toBe(1);
    out = tr.update([b(2)], 0.1); expect(out[0].id).toBe(1); expect(out[0].vx).toBeGreaterThan(0);
    out = tr.update([], 0.1); expect(out.length).toBe(0);
    out = tr.update([b(6, 0.3)], 0.1); expect(out[0]?.id).toBe(1); // low-score box re-associates
    tr.update([], 0.1); tr.update([], 0.1); tr.update([], 0.1);
    out = tr.update([b(8)], 0.1); expect(out[0].id).toBe(2);
  });
  it('applies VLA actions in the tool frame', () => {
    const tcp = xyzrpwToPose(100, 200, 300, 0, 0, 90);
    const { pose, gripper } = applyVlaAction(tcp, [1, 0, 0, 0, 0, 0, 1], { scaleMm: 10 });
    expect(pose[12]).toBeCloseTo(100, 3); expect(pose[13]).toBeCloseTo(210, 3); // tool X = world Y after 90° yaw
    expect(gripper).toBe(1);
    const sim = new SimulatedModel(VISION_MODELS.find((m) => m.id === 'yolov8n')!, 1);
    expect(sim.tasks).toContain('detect');
  });
});

describe('point clouds and camera geometry', () => {
  it('reads and writes PCD (ascii/binary) and PLY (binary) with metres ↔ mm', () => {
    const c = makeCloud([1000, 2000, 3000, -500, 0, 250, 10, 20, 30]);
    c.intensity = Float32Array.from([0.1, 0.5, 0.9]);
    const bin = parsePCD(writePCD(c, true).buffer as ArrayBuffer);
    expect(pointCount(bin)).toBe(3); expect(bin.xyz[3]).toBeCloseTo(-500, 2); expect(bin.intensity![1]).toBeCloseTo(0.5, 5);
    const asc = parsePCD(writePCD(c, false).buffer as ArrayBuffer);
    expect(asc.xyz[8]).toBeCloseTo(30, 1);
    const ply = parsePLY(writePLY({ ...c, rgb: Uint8Array.from([255, 0, 0, 0, 255, 0, 0, 0, 255]) }).buffer as ArrayBuffer);
    expect(pointCount(ply)).toBe(3); expect(ply.rgb![4]).toBe(255); expect(ply.xyz[2]).toBeCloseTo(3000, 2);
    const asciiPly = new TextEncoder().encode('ply\nformat ascii 1.0\nelement vertex 2\nproperty float x\nproperty float y\nproperty float z\nend_header\n0 0 0\n1 1 1\n');
    expect(pointCount(parsePLY(asciiPly.buffer as ArrayBuffer))).toBe(2);
  });
  it('simulated LiDAR / depth camera: ground plane, trunk clusters, row line, voxel down-sampling', () => {
    const { st, cam } = orchardStation();
    const pose = mul(transl(cam.poseAbs()[12], cam.poseAbs()[13], 1200), rotz(90 * DEG));
    const { cloud, classes } = simulateLidar3D(st, pose, { channels: 16, hres: 1, range: 15000 });
    expect(pointCount(cloud)).toBeGreaterThan(1000);
    expect(classes).toContain('ground'); expect(classes).toContain('trunk');
    const ds = voxelDownsample(cloud, 200);
    expect(pointCount(ds)).toBeLessThan(pointCount(cloud));
    const plane = fitGroundPlane(ds, { thresh: 80 })!;
    expect(plane.normal[2]).toBeGreaterThan(0.95);
    expect(Math.abs(plane.d)).toBeLessThan(150);
    const a = analyseCloud(cloud, { voxel: 150, tol: 500, minPts: 6 });
    expect(a.clusters.some((c) => c.shape === 'trunk' || c.shape === 'canopy')).toBe(true);
    expect(a.rows.length).toBeGreaterThanOrEqual(1);
    const line = fitLine2D([[0, 0], [1000, 10], [2000, -10], [3000, 0]])!;
    expect(Math.abs(line.dir[0])).toBeCloseTo(1, 2); expect(line.rms).toBeLessThan(20);
    const K = intrinsicsFromCamera(cam);
    const d = simulateDepthCamera(st, cam.poseAbs(), K, { step: 8, range: 8000, fruit: true });
    expect(d.depth.some((v) => v > 0)).toBe(true);
    expect(d.classes).toContain('canopy');
    expect(euclideanCluster(d.cloud, 300, 5).length).toBeGreaterThan(0);
  });
  it('camera model: projection round trip, depth error models, size prior, rigid fit, target orientation', () => {
    const K = intrinsicsFromCamera({ fov: 60, width: 640, height: 480 });
    const p = backproject(K, 400, 300, 1500);
    expect(project(K, p)![0]).toBeCloseTo(400, 6);
    expect(depthSigmaMm('zed_x', 'stereo', 1500)).toBeCloseTo((0.01 + 0.004 * 2.25) * 1000, 3);
    expect(depthSigmaMm(undefined, 'stereo', 2000, { baselineMm: 100, fxPx: 700, disparityNoisePx: 0.25 })).toBeCloseTo((4 * 0.25) / (700 * 0.1) * 1000, 3);
    expect(depthSigmaMm('velodyne_vlp16', 'lidar3d', 20000)).toBe(30);
    expect(rangeFromSize(K, K.fx * 75 / 1500, 75)).toBeCloseTo(1500, 6);
    const T = xyzrpwToPose(100, -50, 30, 10, -20, 35);
    const src: [number, number, number][] = [[0, 0, 0], [500, 0, 0], [0, 400, 0], [0, 0, 300], [200, 200, 200]];
    const dst = src.map((s) => transformPoint(T, s));
    const { pose, rms } = rigidTransform(src, dst);
    expect(rms).toBeLessThan(1e-6);
    for (let i = 0; i < 16; i++) expect(pose[i]).toBeCloseTo(T[i], 6);
    const camW = mul(transl(0, 0, 1000), rotx(180 * DEG));
    const { grasp, approach } = targetPoseFromPoint([0, 0, 0], camW, { approachMm: 50 });
    expect(grasp[10]).toBeCloseTo(-1, 6); // tool Z points down towards the point
    expect(approach[14]).toBeCloseTo(50, 6);
  });
});

describe('ROS 2 perception package', () => {
  it('generates launch, configs, bridges and README for the configured stack', () => {
    const cam = new Camera('ArmCam');
    const cfg: VisionStackConfig = { tasks: ['detect', 'track', 'pose', 'cloud_segment', 'vlm_query', 'vla_policy'], modality: 'rgbd', environment: 'orchard', sensor: 'realsense_d455', compute: 'jetson_agx_orin', models: { detect: 'yolov8s', track: 'bytetrack', pose: 'depth_centroid', cloud_segment: 'patchwork', vlm_query: 'qwen2_vl', vla_policy: 'pi0' }, classes: ['apple', 'trunk'], confidence: 0.4, iou: 0.5, runtime: 'simulated', workingDistance: 1.2, simulate: true, vlm: { url: 'http://jetson:11434/v1', model: 'qwen2.5vl:7b' }, vla: { url: 'http://jetson:8000', model: 'pi0_fast', format: 'openpi', instruction: 'pick' }, handEyeMode: 'eye_in_hand', handEye: Array.from(xyzrpwToPose(50, 0, 80, 0, 0, 90)) };
    const pkg = generateRosVisionPackage(cam, cfg);
    expect(pkg.files['launch/perception.launch.py']).toMatch(/realsense2_camera/);
    expect(pkg.files['launch/perception.launch.py']).toMatch(/yolo_node/);
    expect(pkg.files['launch/perception.launch.py']).toMatch(/detect_3d_node/);
    expect(pkg.files['launch/perception.launch.py']).toMatch(/--frame-id', 'tool0'/);
    expect(pkg.files['config/detector.yaml']).toMatch(/yolov8s\.engine/);
    expect(pkg.files['config/detector.yaml']).toMatch(/"apple","trunk"/);
    expect(pkg.files['config/vlm.yaml']).toMatch(/jetson:11434/);
    expect(pkg.files['scripts/vla_bridge.py']).toMatch(/openpi/);
    expect(pkg.files['scripts/cloud_pipeline.py']).toMatch(/segment_plane/);
    expect(pkg.files['config/hand_eye.yaml']).toMatch(/xyz: \[0.0500, 0.0000, 0.0800\]/);
    const zip = unzipSync(rosVisionPackageZip(pkg));
    expect(strFromU8(zip[`${pkg.name}/README.md`])).toMatch(/Perception stack/);
    const lidarPkg = generateRosVisionPackage(cam, { ...cfg, modality: 'lidar3d', sensor: 'ouster_os1', tasks: ['cloud_detect'], models: { cloud_detect: 'pcl_pipeline' } });
    expect(lidarPkg.files['launch/perception.launch.py']).toMatch(/ouster_ros/);
    expect(lidarPkg.files['launch/perception.launch.py']).toMatch(/cloud_pipeline/);
    expect(lidarPkg.files['launch/perception.launch.py']).not.toMatch(/yolo_node/);
  });
  it('vision_infer.py exists and reports its backends', () => {
    const src = readFileSync(path.join(process.cwd(), 'python', 'vision_infer.py'), 'utf8');
    expect(src).toMatch(/run_ultralytics/); expect(src).toMatch(/run_onnxruntime/); expect(src).toMatch(/run_vlm/);
  });
});

it('objects and vehicles are seen by cameras too', () => {
  const st = new Station();
  const box = st.addChild(new SceneObject('Crate'));
  box.geometry = [{ primitive: { kind: 'box', size: [400, 300, 200] } }];
  box.setPose(transl(0, 0, 100));
  const cam = st.addChild(new Camera('C'));
  cam.setPose(mul(transl(0, 0, 2000), rotx(180 * DEG)));
  const truth = captureTruth(st, cam);
  expect(truth.length).toBe(1);
  expect(truth[0].cls).toBe('crate');
  expect(truth[0].z).toBeCloseTo(1900, 0);
  expect(truth[0].box.w).toBeGreaterThan(50);
  expect(st.itemsOfType(ItemType.CAMERA).length).toBe(1);
});
