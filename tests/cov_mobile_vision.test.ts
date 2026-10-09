/**
 * Coverage tests for the remaining mobile (items, navstack) and vision (camera model, point clouds, stack,
 * pipeline registry, tracker) functions: serialization round trips, map editing, platform detection, GNSS
 * availability patterns, pinhole helpers, cloud filters and the per-camera runtime registry.
 */
import { describe, it, expect } from 'vitest';
import { Station, Camera, Frame, ItemType } from '../src/core/items/item';
import { MobileRobot, MapItem, ZoneItem } from '../src/mobile/items';
import { platformForRobot, gnssAvailableAt, NavStackConfig } from '../src/mobile/navstack';
import { intrinsicsFromCamera, intrinsicsFromSensor, pixelRay, inImage, mmPerPixel, cameraWorldPose, pixelDepthToWorld, handEyeFromScene, project, backproject } from '../src/vision/camera_model';
import { VISION_SENSORS, VISION_MODELS, modalityForCamera, cameraKindForModality, setVisionStack, getVisionStack, VisionStackConfig } from '../src/vision/stack';
import { makeCloud, pointCount, transformCloud, cropBox, bounds, euclideanCluster, labelClusters, canopyMetrics, depthToCloud, cloudStats, decimate, parsePCD, voxelDownsample, selectPoints } from '../src/vision/pointcloud';
import { ensureVisionRuntime, visionRuntimeOf, summarize, PipelineOutput } from '../src/vision/pipeline';
import { ByteTracker } from '../src/vision/models';
import { transl, rotz, mul, DEG, transformPoint } from '../src/core/math/pose';

// ---------------------------------------------------------------------------------------------
// Mobile items
// ---------------------------------------------------------------------------------------------

describe('mobile items', () => {
  it('keeps the 2D state and the item pose in sync and detects the platform type', () => {
    const r = new MobileRobot('R');
    r.setPose(mul(transl(100, 200, 50), rotz(30 * DEG)));
    expect(r.state.x).toBeCloseTo(100);
    expect(r.state.y).toBeCloseTo(200);
    expect(r.state.theta).toBeCloseTo(30, 6);
    r.setPose2D(-10, 20, -45);
    expect(r.pose()[12]).toBeCloseTo(-10);
    expect(r.pose()[14]).toBeCloseTo(50); // height preserved
    expect(Math.atan2(r.pose()[1], r.pose()[0]) / DEG).toBeCloseTo(-45, 6);
    expect(platformForRobot(r)).toBe('amr');
    r.kin.drive = 'ackermann';
    expect(platformForRobot(r)).toBe('tractor');
    r.kin.drive = 'tracked';
    expect(platformForRobot(r)).toBe('tracked');
    r.kin.drive = 'legged';
    expect(platformForRobot(r)).toBe('legged');
    r.kin.drive = 'omni';
    expect(platformForRobot(r)).toBe('amr');
    r.battery.capacityWh = 0;
    expect(r.batteryLevel()).toBe(1);
  });

  it('serializes robots, maps (run-length encoded cells) and zones through the station file', () => {
    const st = new Station('Yard');
    const r = st.addChild(new MobileRobot('Tractor'));
    r.kin = { ...r.kin, drive: 'ackermann', maxSpeed: 2500, minTurnRadius: 3000 };
    r.battery.levelWh = 1234;
    r.sensors = [{ kind: 'gnss', pose: [0, 0, 1500] }, { kind: 'lidar2d', pose: [500, 0, 300], range: 20000, fov: 270 }];
    r.capabilities = ['mow', 'spray'];
    r.payloadKg = 400;
    r.rosNamespace = '/tractor';
    r.fleetId = 'fleet-1';
    r.home = { x: 1, y: 2, theta: 3 };
    r.setPose2D(1000, 2000, 90);
    r.state.path = [[0, 0], [1, 1]];
    r.state.pathIndex = 1;
    r.state.odometer = 777;
    const map = st.addChild(new MapItem('Field'));
    map.resize(40, 30, 250, -1000, -2000);
    map.inflation = 600;
    map.fillRect(0, 0, 2000, 500);
    map.fillCircle(5000, 3000, 900);
    expect(map.isFree(5000, 3000)).toBe(false);
    expect(map.isFree(5000, 3000 + 1200)).toBe(true);
    let occupied = 0;
    for (const c of map.cells) if (c === 100) occupied++;
    const circleCells = Math.PI * 900 * 900 / (250 * 250);
    expect(occupied).toBeGreaterThan(circleCells * 0.7 + 10);
    expect(occupied).toBeLessThan(circleCells * 1.4 + 30);
    const zone = st.addChild(new ZoneItem('Dock'));
    zone.kind = 'charging';
    zone.polygon = [[0, 0], [1000, 0], [1000, 1000], [0, 1000]];
    zone.speedLimit = 300;
    zone.capacity = 2;
    const json = JSON.stringify(st.serialize());
    expect(json.length).toBeLessThan(map.cells.length * 2); // RLE keeps the map compact
    const copy = Station.deserialize(JSON.parse(json));
    const r2 = copy.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)[0];
    expect(r2).toBeInstanceOf(MobileRobot);
    expect(r2.kin.drive).toBe('ackermann');
    expect(r2.kin.minTurnRadius).toBe(3000);
    expect(r2.battery.levelWh).toBe(1234);
    expect(r2.sensors.length).toBe(2);
    expect(r2.capabilities).toEqual(['mow', 'spray']);
    expect([r2.payloadKg, r2.rosNamespace, r2.fleetId]).toEqual([400, '/tractor', 'fleet-1']);
    expect(r2.home).toEqual({ x: 1, y: 2, theta: 3 });
    expect(r2.state.odometer).toBe(777);
    expect(r2.state.path).toBeNull(); // transient path progress is not restored
    expect(r2.state.pathIndex).toBe(0);
    expect(r2.state.x).toBeCloseTo(1000);
    expect(r2.state.theta).toBeCloseTo(90, 6);
    const m2 = copy.itemsOfType<MapItem>(ItemType.MAP)[0];
    expect([m2.width, m2.height, m2.resolution, m2.originX, m2.originY, m2.inflation]).toEqual([40, 30, 250, -1000, -2000, 600]);
    expect(m2.cells.length).toBe(map.cells.length);
    expect(Array.from(m2.cells)).toEqual(Array.from(map.cells));
    expect(m2.isFree(5000, 3000)).toBe(false);
    const z2 = copy.itemsOfType<ZoneItem>(ItemType.ZONE)[0];
    expect([z2.kind, z2.speedLimit, z2.capacity]).toEqual(['charging', 300, 2]);
    expect(z2.polygon).toEqual(zone.polygon);
    expect(z2.contains(500, 500)).toBe(true);
    // defaults when fields are missing
    const bare = new MobileRobot('bare');
    bare.setPose(transl(5, 6, 0));
    bare.deserializeExtra({} as any, {} as any);
    expect(bare.capabilities).toEqual(['transport']);
    expect(bare.state.x).toBe(5);
    const bareMap = new MapItem('bm');
    bareMap.deserializeExtra({} as any);
    expect(bareMap.cells.length).toBe(100 * 100);
    const bareZone = new ZoneItem('bz');
    bareZone.deserializeExtra({} as any);
    expect([bareZone.kind, bareZone.capacity, bareZone.polygon]).toEqual(['work', 1, []]);
    expect(bareZone.centroid()).toEqual([0, 0]);
  });

  it('GNSS availability follows the configured spatial pattern when no denied zones are drawn', () => {
    const cfg: NavStackConfig = { platform: 'tractor', environment: 'open_field', sensors: ['gnss_rtk'], localization: 'gnss_rtk', navigation: 'gnss_waypoints', fusion: [], simulate: true };
    expect(gnssAvailableAt(1234, 5678, [], { ...cfg, gnssAvailability: 0 })).toBe(false);
    expect(gnssAvailableAt(1234, 5678, [], { ...cfg, gnssAvailability: 1 })).toBe(true);
    let ok = 0;
    for (let i = 0; i < 400; i++) if (gnssAvailableAt(i * 731, i * 397, [], { ...cfg, gnssAvailability: 0.5 })) ok++;
    expect(ok).toBeGreaterThan(100);
    expect(ok).toBeLessThan(300);
    const parking = new ZoneItem('p'); parking.kind = 'parking'; parking.polygon = [[0, 0], [1, 0], [1, 1]];
    expect(gnssAvailableAt(5, 5, [parking], { ...cfg, gnssAvailability: 1 })).toBe(true);
    expect(gnssAvailableAt(5, 5, [parking], cfg)).toBe(true);
  });
});

// ---------------------------------------------------------------------------------------------
// Camera model
// ---------------------------------------------------------------------------------------------

describe('camera model helpers', () => {
  it('builds intrinsics from catalogue sensors and projects rays / pixels', () => {
    const cam = VISION_SENSORS.find((s) => s.resolution[0] > 0 && s.modality === 'mono')!;
    const K = intrinsicsFromSensor(cam);
    expect(K.width).toBe(cam.resolution[0]);
    expect(K.cx).toBe(cam.resolution[0] / 2);
    expect(K.fx).toBeCloseTo((cam.resolution[0] / 2) / Math.tan((cam.hfov * Math.PI) / 360));
    const lidar = VISION_SENSORS.find((s) => !s.resolution[0])!;
    const KL = intrinsicsFromSensor(lidar);
    expect([KL.width, KL.height]).toEqual([1024, 768]);
    const K2 = intrinsicsFromCamera({ fov: 90, width: 640, height: 480 });
    expect(K2.fx).toBeCloseTo(320);
    const ray = pixelRay(K2, 320, 240);
    expect(ray).toEqual([0, 0, 1]);
    const r2 = pixelRay(K2, 640, 240);
    expect(Math.hypot(...r2)).toBeCloseTo(1);
    expect(r2[0]).toBeCloseTo(Math.SQRT1_2);
    expect(inImage(K2, [0, 0])).toBe(true);
    expect(inImage(K2, [640, 10])).toBe(false);
    expect(inImage(K2, [-1, 10])).toBe(false);
    expect(mmPerPixel(K2, 3200)).toBeCloseTo(10);
    const uv = project(K2, backproject(K2, 100, 50, 2000))!;
    expect(uv[0]).toBeCloseTo(100);
    expect(uv[1]).toBeCloseTo(50);
  });

  it('places cameras in the world with or without a hand-eye transform', () => {
    const st = new Station('Cell');
    const flange = st.addChild(new Frame('Flange'));
    flange.setPose(mul(transl(500, 0, 800), rotz(90 * DEG)));
    const cam = flange.addChild(new Camera('Eye'));
    cam.setPose(transl(0, 50, 100));
    const world = cameraWorldPose(cam);
    expect(Array.from(world)).toEqual(Array.from(cam.poseAbs()));
    const handEye = Array.from(transl(0, 0, 200));
    const viaHandEye = cameraWorldPose(cam, handEye, flange.poseAbs());
    expect(viaHandEye[14]).toBeCloseTo(1000);
    expect(viaHandEye[12]).toBeCloseTo(500);
    expect(Array.from(cameraWorldPose(cam, [1, 2, 3], flange.poseAbs()))).toEqual(Array.from(cam.poseAbs()));
    expect(Array.from(cameraWorldPose(cam, handEye))).toEqual(Array.from(cam.poseAbs()));
    const he = handEyeFromScene(cam, flange.poseAbs());
    expect(he.length).toBe(16);
    expect(he[12]).toBeCloseTo(0, 6);
    expect(he[13]).toBeCloseTo(50, 6);
    expect(he[14]).toBeCloseTo(100, 6);
    const K = intrinsicsFromCamera({ fov: 90, width: 640, height: 480 });
    const p = pixelDepthToWorld(K, transl(0, 0, 1000), 320, 240, 500);
    expect(p[0]).toBeCloseTo(0);
    expect(p[2]).toBeCloseTo(1500);
    const q = pixelDepthToWorld(K, transl(0, 0, 1000), 640, 240, 500);
    expect(q[0]).toBeCloseTo(500);
  });
});

// ---------------------------------------------------------------------------------------------
// Point clouds
// ---------------------------------------------------------------------------------------------

describe('point cloud filters and summaries', () => {
  const grid = (): number[] => { const pts: number[] = []; for (let x = 0; x < 10; x++) for (let y = 0; y < 10; y++) pts.push(x * 100, y * 100, 0); return pts; };

  it('transforms, crops, bounds, decimates and summarises clouds', () => {
    const c = makeCloud(grid());
    c.intensity = new Float32Array(pointCount(c)).fill(0.5);
    const m = mul(transl(1000, 0, 50), rotz(90 * DEG));
    const t = transformCloud(c, m, 'sensor');
    expect(t.frame).toBe('sensor');
    expect(pointCount(t)).toBe(100);
    expect(t.intensity).toBe(c.intensity);
    const p9 = transformPoint(m, [900, 0, 0]);
    expect(t.xyz[9 * 10 * 3]).toBeCloseTo(p9[0], 3);
    expect(t.xyz[9 * 10 * 3 + 1]).toBeCloseTo(p9[1], 3);
    expect(t.xyz[2]).toBeCloseTo(50, 5);
    const b = bounds(c);
    expect(b.min).toEqual([0, 0, 0]);
    expect(b.max).toEqual([900, 900, 0]);
    const crop = cropBox(c, [0, 0, -1], [250, 150, 1]);
    expect(pointCount(crop)).toBe(3 * 2);
    expect(crop.intensity!.length).toBe(6);
    const s = cloudStats(c);
    expect(s.n).toBe(100);
    expect(s.centroid[0]).toBeCloseTo(450);
    expect(s.centroid[1]).toBeCloseTo(450);
    expect(s.max).toEqual([900, 900, 0]);
    const d = decimate(c, 25);
    expect(pointCount(d)).toBe(25);
    expect(decimate(c, 100)).toBe(c);
    expect(pointCount(decimate(c, 7))).toBe(7);
    expect(pointCount(voxelDownsample(c, 250))).toBe(16);
    expect(pointCount(selectPoints(c, (i) => i % 2 === 0))).toBe(50);
  });

  it('labels clusters, computes canopy metrics and converts depth images', () => {
    const pts: number[] = [];
    // a trunk-like column and a canopy-like blob
    for (let z = 0; z < 2000; z += 100) pts.push(0, 0, z);
    for (let x = 0; x < 1200; x += 200) for (let y = 0; y < 1200; y += 200) for (let z = 1500; z < 2700; z += 300) pts.push(5000 + x, y, z);
    const c = makeCloud(pts);
    const clusters = euclideanCluster(c, 350, 5);
    expect(clusters.length).toBe(2);
    const labelled = labelClusters(c, clusters);
    expect(labelled.label!.length).toBe(pointCount(c));
    for (const cl of clusters) for (const i of cl.indices) expect(labelled.label![i]).toBe(cl.id);
    expect(labelled.xyz).toBe(c.xyz);
    const canopy = clusters.find((cl) => cl.count > 20)!;
    const metrics = canopyMetrics(canopy);
    expect(metrics.widthM).toBeCloseTo(1.0);
    expect(metrics.heightM).toBeCloseTo(0.9);
    expect(metrics.volumeM3).toBeCloseTo(1.0 * 1.0 * 0.9, 3);
    const K = { fx: 100, fy: 100, cx: 2, cy: 2, width: 4, height: 4 };
    const depth = new Float32Array(16).fill(1000);
    depth[0] = 0; depth[5] = NaN; depth[10] = Infinity;
    const sensor = depthToCloud(depth, K);
    expect(sensor.frame).toBe('sensor');
    expect(pointCount(sensor)).toBe(13);
    expect(sensor.xyz[2]).toBe(1000);
    const world = depthToCloud(depth, K, transl(0, 0, 500), 2);
    expect(world.frame).toBe('world');
    expect(pointCount(world)).toBe(2); // stride 2 samples (0,0) zero depth and (2,2) infinite depth are skipped; (2,0),(0,2) kept
    expect(world.xyz[2]).toBeCloseTo(1500);
  });

  it('drops non-finite points when parsing PCD files', () => {
    const text = ['# .PCD v0.7', 'VERSION 0.7', 'FIELDS x y z', 'SIZE 4 4 4', 'TYPE F F F', 'COUNT 1 1 1', 'WIDTH 3', 'HEIGHT 1', 'VIEWPOINT 0 0 0 1 0 0 0', 'POINTS 3', 'DATA ascii', '1 2 3', 'nan nan nan', '4 5 6', ''].join('\n');
    const cloud = parsePCD(new TextEncoder().encode(text).buffer);
    expect(pointCount(cloud)).toBe(2);
    expect(Array.from(cloud.xyz)).toEqual([1000, 2000, 3000, 4000, 5000, 6000]);
    expect(cloud.frame).toBe('sensor');
  });
});

// ---------------------------------------------------------------------------------------------
// Vision stack / pipeline registry / tracker
// ---------------------------------------------------------------------------------------------

describe('vision stack helpers, runtime registry and tracker reset', () => {
  const detectId = VISION_MODELS.find((m) => m.tasks.includes('detect') && m.family === 'yolo')!.id;
  const trackId = VISION_MODELS.find((m) => m.tasks.includes('track'))!.id;
  const cfg = (): VisionStackConfig => ({ tasks: ['detect', 'track'], modality: 'mono', environment: 'warehouse', sensor: 'usb_rgb', compute: 'jetson_orin_nano', models: { detect: detectId, track: trackId }, classes: ['box'], confidence: 0.4, iou: 0.5, runtime: 'simulated', workingDistance: 1, simulate: true });

  it('maps camera kinds to modalities and back', () => {
    const cam = new Camera('C');
    expect(modalityForCamera(cam)).toBe('mono');
    cam.kind = 'depth';
    expect(modalityForCamera(cam)).toBe('rgbd');
    cam.kind = 'lidar3d';
    expect(modalityForCamera(cam)).toBe('lidar3d');
    cam.kind = 'lidar2d';
    expect(modalityForCamera(cam)).toBe('lidar3d');
    expect(cameraKindForModality('lidar3d')).toBe('lidar3d');
    expect(cameraKindForModality('rgbd')).toBe('depth');
    expect(cameraKindForModality('stereo')).toBe('depth');
    expect(cameraKindForModality('tof')).toBe('depth');
    expect(cameraKindForModality('mono')).toBe('rgb');
    expect(cameraKindForModality('thermal')).toBe('rgb');
  });

  it('keeps one runtime per camera configuration and resets it when the configuration changes', () => {
    const cam = new Camera('C');
    expect(ensureVisionRuntime(cam)).toBeNull();
    expect(visionRuntimeOf(cam)).toBeUndefined();
    const c1 = cfg();
    setVisionStack(cam, c1);
    expect(getVisionStack(cam)).toBe(c1);
    const rt = ensureVisionRuntime(cam)!;
    expect(rt.cam).toBe(cam);
    expect(ensureVisionRuntime(cam)).toBe(rt);
    expect(visionRuntimeOf(cam)).toBe(rt);
    const model = rt.model('detect');
    expect(rt.model('detect')).toBe(model);
    expect(rt.models.size).toBe(1);
    rt.stats.frames = 5;
    rt.tracker.update([{ x: 0, y: 0, w: 10, h: 10, score: 0.9, cls: 'box' }], 0.1);
    expect(rt.tracker.tracks.length).toBe(1);
    rt.reset();
    expect(rt.models.size).toBe(0);
    expect(rt.stats.frames).toBe(0);
    expect(rt.tracker.tracks).toEqual([]);
    expect(rt.last).toBeNull();
    const c2 = cfg();
    setVisionStack(cam, c2);
    const rt2 = ensureVisionRuntime(cam)!;
    expect(rt2).not.toBe(rt);
    expect(rt2.cfg).toBe(c2);
    setVisionStack(cam, null);
    expect(getVisionStack(cam)).toBeNull();
    expect(ensureVisionRuntime(cam)).toBeNull();
    expect(visionRuntimeOf(cam)).toBeUndefined();
  });

  it('summarises pipeline outputs including models, targets, clouds, text, VLA actions and warnings', () => {
    const cam = new Camera('C');
    const K = intrinsicsFromCamera(cam);
    const base: PipelineOutput = { frame: { width: 640, height: 480, K, camWorld: cam.poseAbs(), time: 0 }, results: {}, detections: [], targets: [], warnings: [] };
    const c = cfg();
    expect(summarize(base, c)).toMatch(/no detections/);
    const detectName = VISION_MODELS.find((m) => m.id === detectId)!.name.split(' ')[0];
    expect(summarize(base, c).startsWith(detectName)).toBe(true);
    const cloud = makeCloud([0, 0, 0, 1, 1, 1]);
    const rich: PipelineOutput = {
      ...base,
      detections: [{ x: 0, y: 0, w: 1, h: 1, score: 0.9, cls: 'box' }, { x: 0, y: 0, w: 1, h: 1, score: 0.8, cls: 'box' }, { x: 0, y: 0, w: 1, h: 1, score: 0.8, cls: 'pallet' }],
      targets: [{ name: 't', cls: 'box', grasp: transl(0, 0, 0), approach: transl(0, 0, 0), p: [0, 0, 0], z: 1, sigmaMm: 12.4 }],
      cloud: { cloud, ground: cloud, objects: cloud, clusters: [], rows: [], classes: [] },
      text: 'two boxes and a pallet',
      vla: { action: [0.5, -0.25, 0, 0, 0, 0, 1], text: 'move left' },
      warnings: ['low light'],
    };
    const s = summarize(rich, c);
    expect(s).toContain('2 box, 1 pallet');
    expect(s).toContain('1 targets (σ 12 mm)');
    expect(s).toContain('cloud 2 pts → 0 clusters, 0 rows');
    expect(s).toContain('VLM: two boxes and a pallet');
    expect(s).toContain('VLA action [0.50 -0.25 0.00 0.00 0.00 0.00 1.00] move left');
    expect(s).toContain('low light');
    expect(summarize(base, { ...c, models: {} }).startsWith(': ')).toBe(true);
  });

  it('ByteTracker restarts ids after reset', () => {
    const tr = new ByteTracker({ minHits: 1 });
    const det = [{ x: 10, y: 10, w: 20, h: 20, score: 0.9, cls: 'apple' }];
    expect(tr.update(det, 0.1)[0].id).toBe(1);
    tr.update([{ x: 300, y: 300, w: 20, h: 20, score: 0.9, cls: 'apple' }], 0.1);
    expect(tr.tracks.length).toBe(2);
    tr.reset();
    expect(tr.tracks).toEqual([]);
    expect(tr.update(det, 0.1)[0].id).toBe(1);
  });
});
