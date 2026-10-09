import { describe, it, expect, afterAll } from 'vitest';
import http from 'node:http';
import { AddressInfo } from 'node:net';
import { Station, Camera, ItemType } from '../src/core/items/item';
import { identity, transl, rotz, DEG, mul } from '../src/core/math/pose';
import { HttpModel, VlmOpenAIModel, VlaHttpModel, Ros2VisionModel, VisionFrame } from '../src/vision/models';
import { intrinsicsFromCamera } from '../src/vision/camera_model';
import { handleVisionHttp } from '../server/vision';
import { detection2DArrayMsg, detection3DArrayMsg, pointCloud2Msg, publishVisionOutput, publishNavRuntime, odometryMsg, laserScanMsg, occupancyGridMsg, visionSummary, navSummary, toBase64 } from '../src/ros/publishers';
import { VISION_SCENARIOS } from '../src/scenarios';
import { getVisionStack } from '../src/vision/stack';
import { VisionRuntime } from '../src/vision/pipeline';
import { MobileRobot, MapItem } from '../src/mobile/items';
import { setNavStack, stepNavRuntime } from '../src/mobile/navstack';
import { followPath, stepMobile } from '../src/mobile/controller';

/** Mock servers speaking the real wire contracts: studio inference JSON, OpenAI chat/completions, openpi /infer, OpenVLA /act. */
function mockServer(handler: (req: http.IncomingMessage, body: any, res: http.ServerResponse) => void): Promise<{ url: string; close: () => void; calls: any[] }> {
  const calls: any[] = [];
  const srv = http.createServer((req, res) => { let b = ''; req.on('data', (c) => (b += c)); req.on('end', () => { const body = b ? JSON.parse(b) : {}; calls.push({ url: req.url, method: req.method, body, headers: req.headers }); handler(req, body, res); }); });
  return new Promise((resolve) => srv.listen(0, '127.0.0.1', () => resolve({ url: `http://127.0.0.1:${(srv.address() as AddressInfo).port}`, close: () => srv.close(), calls })));
}
const json = (res: http.ServerResponse, code: number, o: unknown) => { res.writeHead(code, { 'content-type': 'application/json' }); res.end(JSON.stringify(o)); };
const frame = (): VisionFrame => ({ width: 640, height: 480, K: intrinsicsFromCamera({ fov: 60, width: 640, height: 480 }), camWorld: identity(), time: 1, dataUrl: 'data:image/jpeg;base64,/9j/4AAQSkZJRg==', proprio: { joints: [0, -90, 90, 0, 90, 0], gripper: 0 } });
const closers: Array<() => void> = [];
afterAll(() => closers.forEach((c) => c()));

describe('inference server contract (HTTP)', () => {
  it('HttpModel posts the image and parses studio / Roboflow / Ultralytics answers; the studio server forwards to STUDIO_VISION_URL', async () => {
    const infer = await mockServer((req, body, res) => {
      if (req.url === '/vision/infer') return json(res, 200, { detections: [{ x: 10, y: 20, w: 100, h: 50, score: 0.9, class: body.classes?.[0] ?? 'apple', keypoints: [[60, 20, 0.9]], position: [1000, 200, 300] }], labels: [{ class: 'ripe', score: 0.8 }], backend: 'mock', echo: { model: body.model, task: body.task } });
      if (req.url === '/roboflow') return json(res, 200, { predictions: [{ x: 60, y: 45, width: 100, height: 50, class: 'apple', confidence: 0.77 }] });
      return json(res, 404, { error: 'nope' });
    });
    closers.push(infer.close);
    const m = new HttpModel({ url: `${infer.url}/vision/infer`, model: 'best.onnx' });
    const r = await m.run(frame(), { task: 'detect', classes: ['apple'], confidence: 0.3, iou: 0.5 });
    expect(r.boxes.length).toBe(1); expect(r.boxes[0].cls).toBe('apple'); expect(r.boxes[0].keypoints![0][0]).toBe(60); expect(r.boxes[0].p).toEqual([1000, 200, 300]);
    expect(r.labels![0].cls).toBe('ripe');
    expect(infer.calls[0].body.image).toMatch(/^data:image/); expect(infer.calls[0].body.model).toBe('best.onnx'); expect(infer.calls[0].body.task).toBe('detect');
    const rf = await new HttpModel({ url: `${infer.url}/roboflow` }).run(frame(), { task: 'detect', classes: [], confidence: 0.3, iou: 0.5 });
    expect(rf.boxes[0].x).toBe(10); expect(rf.boxes[0].w).toBe(100);
    // studio server: forward /vision/infer to an external inference server (STUDIO_VISION_URL)
    process.env.STUDIO_VISION_URL = `${infer.url}/vision/infer`;
    const studio = http.createServer(async (req, res) => { const readJson = () => new Promise<any>((ok) => { let b = ''; req.on('data', (c) => (b += c)); req.on('end', () => ok(b ? JSON.parse(b) : {})); }); if (!(await handleVisionHttp(req, res, readJson))) { res.writeHead(404); res.end(); } });
    await new Promise<void>((ok) => studio.listen(0, '127.0.0.1', () => ok()));
    closers.push(() => studio.close());
    const base = `http://127.0.0.1:${(studio.address() as AddressInfo).port}`;
    const res = await fetch(`${base}/vision/infer`, { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify({ image: 'data:image/png;base64,AAAA', task: 'detect', classes: ['box'] }) });
    expect(res.status).toBe(200);
    const j = await res.json();
    expect(j.detections[0].class).toBe('box'); expect(j.backend).toBe('mock');
    const models = await (await fetch(`${base}/vision/models`)).json();
    expect(models.external).toBe(`${infer.url}/vision/infer`);
    expect('backends' in models).toBe(true);
    const bad = await fetch(`${base}/vision/infer`, { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify({ task: 'detect' }) });
    expect(bad.status).toBe(400);
    delete process.env.STUDIO_VISION_URL;
  });
});

describe('VLM contract (OpenAI-compatible chat/completions)', () => {
  it('sends the image as image_url content with a bearer token and parses the grounded JSON answer', async () => {
    const vlm = await mockServer((req, body, res) => {
      if (req.url !== '/v1/chat/completions') return json(res, 404, {});
      const text = body.messages[0].content.find((c: any) => c.type === 'text').text as string;
      const answer = /Detect every/.test(text) ? '```json\n{"detections":[{"label":"apple","box_2d":[100,100,300,300],"confidence":0.9},{"label":"apple","box_2d":[500,200,600,350],"confidence":0.6}]}\n```' : /Classify/.test(text) ? '{"labels":[{"label":"ripe","confidence":0.7},{"label":"unripe","confidence":0.3}]}' : 'I can see two apples and a trunk. The nearest apple is about 1.2 m away.';
      json(res, 200, { id: 'x', choices: [{ message: { role: 'assistant', content: answer } }] });
    });
    closers.push(vlm.close);
    const m = new VlmOpenAIModel({ url: `${vlm.url}/v1`, model: 'qwen2.5vl:7b', apiKey: 'sk-test' });
    const det = await m.run(frame(), { task: 'detect', classes: ['apple'], confidence: 0.3, iou: 0.5 });
    expect(det.boxes.length).toBe(2);
    expect(det.boxes[0].x).toBeCloseTo(64); expect(det.boxes[0].y).toBeCloseTo(48); expect(det.boxes[0].w).toBeCloseTo(128); // 0-1000 frame → pixels
    const call = vlm.calls[0];
    expect(call.headers.authorization).toBe('Bearer sk-test');
    expect(call.body.model).toBe('qwen2.5vl:7b');
    expect(call.body.messages[0].content.some((c: any) => c.type === 'image_url' && c.image_url.url.startsWith('data:image'))).toBe(true);
    const cls = await m.run(frame(), { task: 'classify', classes: ['ripe', 'unripe'], confidence: 0, iou: 0.5 });
    expect(cls.labels![0].cls).toBe('ripe');
    const q = await m.run(frame(), { task: 'vlm_query', classes: [], confidence: 0, iou: 0.5, prompt: 'What do you see?' });
    expect(q.text).toMatch(/two apples/);
    expect(vlm.calls[2].body.messages[0].content[0].text).toBe('What do you see?');
  });
});

describe('VLA contracts (openpi /infer, OpenVLA /act, studio /act)', () => {
  it('posts observation + instruction and applies the returned action chunk', async () => {
    const vla = await mockServer((req, body, res) => {
      if (req.url === '/infer') return json(res, 200, { actions: [[0.1, 0, 0.5, 0, 0, 0, 0], [0.1, 0, 0.4, 0, 0, 0, 0]] });
      if (req.url === '/act') return json(res, 200, { action: [0, 0.2, 0.3, 0, 0, 0.1, 1] });
      json(res, 404, {});
    });
    closers.push(vla.close);
    const pi = new VlaHttpModel({ url: vla.url, model: 'pi0_fast', format: 'openpi' });
    const r = await pi.run(frame(), { task: 'vla_policy', classes: [], confidence: 0, iou: 0.5, prompt: 'pick the apple' });
    expect(r.action).toEqual([0.1, 0, 0.5, 0, 0, 0, 0]); expect(r.actionChunk!.length).toBe(2);
    expect(vla.calls[0].url).toBe('/infer'); expect(vla.calls[0].body.observation.prompt).toBe('pick the apple'); expect(vla.calls[0].body.observation.state.length).toBe(7);
    const ov = new VlaHttpModel({ url: vla.url, format: 'openvla', model: 'bridge_orig' });
    const r2 = await ov.run(frame(), { task: 'vla_policy', classes: [], confidence: 0, iou: 0.5, prompt: 'pick' });
    expect(r2.action![6]).toBe(1); expect(vla.calls[1].url).toBe('/act'); expect(vla.calls[1].body.unnorm_key).toBe('bridge_orig'); expect(vla.calls[1].body.instruction).toBe('pick');
    const st = new VlaHttpModel({ url: vla.url, format: 'studio', model: 'act_policy' });
    await st.run(frame(), { task: 'vla_policy', classes: [], confidence: 0, iou: 0.5, prompt: 'place' });
    expect(vla.calls[2].body.proprio.joints.length).toBe(6); expect(vla.calls[2].body.model).toBe('act_policy');
  });
});

describe('ROS 2 messages (rosbridge JSON)', () => {
  it('Ros2VisionModel consumes vision_msgs/Detection2DArray from a rosbridge-like subscription', async () => {
    let cb: ((m: any) => void) | null = null;
    const ros = { subscribe: (_t: string, _ty: string, f: (m: any) => void) => { cb = f; } };
    const m = new Ros2VisionModel(ros, '/yolo/detections');
    cb!({ detections: [{ bbox: { center: { position: { x: 100, y: 80 } }, size_x: 40, size_y: 30 }, results: [{ hypothesis: { class_id: 'apple', score: 0.9 } }], id: '3' }] });
    const r = await m.run(frame(), { task: 'detect', classes: [], confidence: 0.3, iou: 0.5 });
    expect(r.boxes[0].x).toBe(80); expect(r.boxes[0].cls).toBe('apple');
  });
  it('publishes Detection2DArray / Detection3DArray / PoseArray / PointCloud2 / CompressedImage for a pipeline output and stores visionLast', async () => {
    const sc = VISION_SCENARIOS.find((s) => s.id === 'vis_lidar_rows')!;
    const b = sc.build(); const cam = b.focus as Camera;
    const rt = new VisionRuntime(cam, getVisionStack(cam)!);
    const out = await rt.step(b.station, { time: 2, lidar: { channels: 8, hres: 2, range: 15000 } });
    const sent: Array<{ topic: string; type: string; msg: any }> = [];
    const ros = { publish: (topic: string, type: string, msg: any) => sent.push({ topic, type, msg }) };
    const topics = publishVisionOutput(ros, cam, out, { namespace: '/platform1' });
    expect(topics).toContain('/platform1/vision/camera/detections');
    expect(topics).toContain('/platform1/vision/camera/cloud');
    const cloud = sent.find((s) => s.type === 'sensor_msgs/msg/PointCloud2')!.msg;
    expect(cloud.fields.map((f: any) => f.name)).toEqual(['x', 'y', 'z', 'intensity']); // simulated LiDAR carries intensity
    expect(cloud.point_step).toBe(16);
    expect(Buffer.from(cloud.data, 'base64').length).toBe(cloud.width * 16);
    expect(cloud.header.stamp.sec).toBe(2);
    const d3 = sent.find((s) => s.type === 'vision_msgs/msg/Detection3DArray')!.msg;
    expect(d3.detections.length).toBe(out.detections.filter((x) => x.p).length);
    expect(d3.detections[0].results[0].hypothesis.class_id).toBeTruthy();
    const last = cam.params.visionLast as any;
    expect(last.cloud.clusters.length).toBe(out.cloud!.clusters.length);
    expect(last.detections.length).toBe(out.detections.length);
    // 2D message geometry
    const o2 = { ...out, detections: [{ x: 10, y: 20, w: 30, h: 40, score: 0.5, cls: 'a', id: 7 }] } as any;
    const d2 = detection2DArrayMsg(o2, 'cam');
    expect(d2.detections[0].bbox.center.position.x).toBe(25); expect(d2.detections[0].id).toBe('7');
    const img = { ...out, frame: { ...out.frame, dataUrl: 'data:image/jpeg;base64,QUJD' } } as any;
    const t2 = publishVisionOutput(ros, cam, img, { cloud: false });
    expect(t2.some((t) => t.endsWith('/image/compressed'))).toBe(true);
    expect(sent[sent.length - 1].msg.data).toBe('QUJD');
    expect(toBase64(new Uint8Array([65, 66, 67]))).toBe('QUJD');
    expect(visionSummary(cam, out).units).toBe('mm');
    void detection3DArrayMsg; void mul; void rotz; void DEG; void transl; void Station; void ItemType;
  });
  it('publishes Odometry (estimate + truth with covariance), LaserScan and OccupancyGrid for a navigation runtime and stores navEstimate', () => {
    const st = new Station('nav');
    const map = new MapItem('m'); map.resize(120, 80, 100, 0, 0); st.addChild(map);
    map.fillRect(0, 0, 12000, 200); map.fillRect(0, 7800, 12000, 8000);
    const r = st.addChild(new MobileRobot('amr')); r.rosNamespace = '/amr1'; r.setPose2D(1000, 4000, 0);
    setNavStack(r, { platform: 'amr', environment: 'warehouse', sensors: ['wheel_odom', 'imu', 'lidar2d'], localization: 'slam_toolbox_2d', navigation: 'nav2_navfn_dwb', fusion: ['wheel_odom'], simulate: true });
    followPath(r, [[1000, 4000], [9000, 4000]]);
    let rt = null as any;
    for (let i = 0; i < 100; i++) { rt = stepNavRuntime(r, 0.05, map, []); stepMobile(r, 0.05); }
    const est = r.params.navEstimate as any;
    expect(est.method).toBe('slam_toolbox_2d'); expect(typeof est.rmse).toBe('number'); expect(est.truth.x).toBeGreaterThan(1000);
    const sent: Array<{ topic: string; type: string; msg: any }> = [];
    const topics = publishNavRuntime({ publish: (topic, type, msg) => sent.push({ topic, type, msg }) }, r, rt, { timeS: 5 });
    expect(topics).toEqual(['/amr1/odom_estimate', '/amr1/ground_truth', '/amr1/scan', '/amr1/slam_map']);
    const odom = sent[0].msg;
    expect(odom.header.frame_id).toBe('map'); expect(odom.child_frame_id).toBe('base_link'); expect(odom.pose.covariance[0]).toBeGreaterThan(0); expect(odom.pose.pose.position.x).toBeCloseTo(est.x / 1000, 3);
    const scan = sent[2].msg;
    expect(scan.ranges.length).toBe(rt.lastScan.ranges.length); expect(scan.range_max).toBe(15);
    const grid = sent[3].msg;
    expect(grid.info.width).toBe(map.width); expect(grid.data.length).toBe(map.width * map.height); expect(grid.data.every((v: number) => v >= -1 && v <= 100)).toBe(true);
    expect(odometryMsg({ x: 1000, y: 0, theta: 90 }, { v: 500, omega: 10 }).pose.pose.orientation.z).toBeCloseTo(Math.SQRT1_2, 5);
    expect(laserScanMsg({ angles: [0, 0.1], ranges: [1000, 2000], maxRange: 5000, hits: [null, null] }).angle_increment).toBeCloseTo(0.1);
    expect(occupancyGridMsg({ width: 2, height: 1, resolution: 100, originX: 0, originY: 0, cells: Uint8Array.from([255, 100]) }).data).toEqual([-1, 100]);
    expect(navSummary(r, rt).slamCoverage).not.toBeNull();
  });
});
