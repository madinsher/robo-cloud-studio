/**
 * T9 vision — RoboDK camera API parity (Example-06.c "SimulateCamera", Macros/Depth_Camera.py,
 * Macros/Open3D_Depth_Camera.py, OpenCV_*.py) and solid occlusion of the fruit detector; agricultural detect → pick keeps
 * the fruit state. Ported from tests/_probe/vision_rdkcam.test.ts (4 probes) + pixel/metric checks of the snapshots.
 */
import { it, expect, describe } from 'vitest';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { Station, Camera, SceneObject } from '../src/core/items/item';
import { Robolink, ITEM_TYPE_FRAME, ITEM_TYPE_CAMERA } from '../src/api/robolink';
import { FieldItem, cropParams, CropRow } from '../src/agri/items';
import { generateOrchard, rectPolygon, fruitWorldPositions } from '../src/agri/orchard';
import { detectFruit } from '../src/agri/vision';
import { mul, transl, rotz, roty, rotx, DEG, invert, transformPoint } from '../src/core/math/pose';
import { cameraIntrinsics, project } from '../src/vision/camera_model';
import { decodePNG, decodeGrey32 } from '../src/vision/png';

/** A station with a red 100x100x2 mm plate (top at z = 100) below a RoboDK camera frame at 1 m looking down (+Z of the frame = view direction). A plate parallel to the image plane projects to a similar square: its pixel centroid is the projection of its centre. */
function scene() {
  const st = new Station();
  const RDK = new Robolink(st);
  const ref = RDK.AddFrame('Camera Reference');
  (ref.item as any).setPose(mul(transl(0, 0, 1000), rotx(180 * DEG)));
  const cube = st.addChild(new SceneObject('Part'));
  cube.geometry = [{ primitive: { kind: 'box', size: [100, 100, 2] }, color: '#ff0000' }];
  cube.color = '#ff0000';
  cube.setPose(transl(120, 80, 99)); // top face at z = 100 -> 900 mm from the camera
  return { st, RDK, ref, cube };
}

describe('RoboDK Cam2D parity', () => {
  it('Example-06.c Cam2D_Add parameters: FAR_LENGHT (RoboDK spelling), FOV vertical, SIZE, parent frame', () => {
    const { RDK } = scene();
    RDK.Cam2D_Close();
    const ref = RDK.Item('Camera Reference', ITEM_TYPE_FRAME);
    const cam = RDK.Cam2D_Add(ref, 'FOCAL_LENGHT=6 FOV=32 FAR_LENGHT=1000 SIZE=640x480 BG_COLOR=black LIGHT_AMBIENT=red LIGHT_DIFFUSE=black LIGHT_SPECULAR=white');
    const c = cam.item as Camera;
    expect([c.far, c.fov, c.width, c.height, c.fovAxis]).toEqual([1000, 32, 640, 480, 'vertical']);
    expect(c.parent?.name).toBe('Camera Reference');
    expect(RDK.Item(c.name, ITEM_TYPE_CAMERA).Valid()).toBe(true);
    // Settings string (Open3D_Depth_Camera.py parses it): FOV / FAR_LENGTH / SIZE round trip
    const s = RDK.Cam2D_Settings(cam);
    // RoboDK 5.9.4 wrote "FOV=32.000 PixelSize=7.168635 Size=640x480 SHOW_LIMITS FOCAL_LENGTH=6.00 FAR_LENGTH=1000.00 BG_COLOR=#000000 Persp WindowResize"
    expect(s).toBe('FOV=32.000 PixelSize=7.168635 Size=640x480 SHOW_LIMITS FOCAL_LENGTH=6.00 FAR_LENGTH=1000.00 BG_COLOR=#000000 Persp WindowResize');
  });

  it('FOCAL_LENGTH (mm) + PIXELSIZE (µm) give the focal length in pixels', () => {
    const { RDK, ref } = scene();
    const cam = RDK.Cam2D_Add(ref, 'FOCAL_LENGTH=6 PIXELSIZE=5 SIZE=1280x1024');
    expect(cameraIntrinsics(cam.item as Camera).fy).toBeCloseTo(6000 / 5, 6);
  });

  it('Cam2D_Snapshot(file) saves a PNG and returns 1; Cam2D_Snapshot("") returns the PNG bytes; the plate is where K says', () => {
    const { st, RDK, ref, cube } = scene();
    const cam = RDK.Cam2D_Add(ref, 'FOV=40 SIZE=320x240 BG_COLOR=black');
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'vbs-cam2d-'));
    try {
      const file = path.join(dir, 'snap.png');
      expect(RDK.Cam2D_Snapshot(file, cam)).toBe(1);
      const disk = decodePNG(new Uint8Array(fs.readFileSync(file)));
      const bytes = RDK.Cam2D_Snapshot('', cam) as Uint8Array;
      expect(bytes).toBeInstanceOf(Uint8Array);
      expect(Array.from(bytes.slice(1, 4)).map((c) => String.fromCharCode(c)).join('')).toBe('PNG');
      const img = decodePNG(bytes);
      expect([img.width, img.height, disk.width, disk.height]).toEqual([320, 240, 320, 240]);
      // red pixels: centroid of the cube's top face = projection of its centre with the camera K
      let sx = 0, sy = 0, n = 0, black = 0;
      for (let y = 0; y < 240; y++) for (let x = 0; x < 320; x++) {
        const i = (y * 320 + x) * 4, r = img.rgba[i], g = img.rgba[i + 1], b = img.rgba[i + 2];
        if (r > 60 && g < 30 && b < 30) { sx += x + 0.5; sy += y + 0.5; n++; } else if (r + g + b === 0) black++;
      }
      const c = cam.item as Camera;
      const uv = project(cameraIntrinsics(c), transformPoint(invert(c.poseAbs()), transformPoint(cube.poseAbs(), [0, 0, 1])))!;
      expect(n).toBeGreaterThan(100);
      expect(Math.hypot(sx / n - uv[0], sy / n - uv[1])).toBeLessThan(0.5);
      expect(black).toBe(320 * 240 - n); // BG_COLOR=black everywhere else
    } finally { fs.rmSync(dir, { recursive: true, force: true }); }
    void st;
  });

  it('DEPTH snapshot (grey32, Depth_Camera.py / Open3D macro decoding) is metric: depth/FAR_LENGTH·(2³²−1)', () => {
    const { RDK, ref } = scene();
    const cam = RDK.Cam2D_Add(ref, 'FOV=40 SIZE=160x120 FAR_LENGTH=2000 DEPTH');
    const bytes = RDK.Cam2D_Snapshot('', cam, 'DEPTH') as Uint8Array;
    // numpy: np.frombuffer(bytes, '>u4'); w, h = a[:2]; img = np.flipud(a[2:].reshape(h, w))
    const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    expect([dv.getUint32(0), dv.getUint32(4)]).toEqual([160, 120]);
    const { depth } = decodeGrey32(bytes, 2000);
    const c = cam.item as Camera, K = cameraIntrinsics(c);
    const centre = transformPoint(invert(c.poseAbs()), [120, 80, 100]);
    const [u, v] = project(K, centre)!;
    expect(depth[Math.floor(v) * 160 + Math.floor(u)]).toBeCloseTo(900, 1); // plate top
    expect(depth[5 * 160 + 5]).toBe(0); // nothing behind (no floor in the station) = no measurement
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'vbs-cam2d-'));
    try {
      const f = path.join(dir, 'temp.grey32');
      expect(RDK.Cam2D_Snapshot(f, cam)).toBe(1);
      expect(Buffer.compare(fs.readFileSync(f), Buffer.from(bytes))).toBe(0);
    } finally { fs.rmSync(dir, { recursive: true, force: true }); }
  });
});

describe('headless API: vision run', () => {
  it('Cam2D_Detect runs the stack on the current station, refreshes Pick and sets VISION_FOUND (getParam)', async () => {
    const { demos } = await import('../src/demos');
    const { setVisionStack } = await import('../src/vision/stack');
    const st = demos.find((d) => d.id === 'pickplace')!.build();
    const cam = st.addChild(new Camera('Cell camera'));
    cam.kind = 'depth'; cam.fov = 60; cam.width = 640; cam.height = 480; cam.near = 100; cam.far = 4000;
    cam.setPose(mul(transl(1020, -150, 1700), rotx(180 * DEG)));
    setVisionStack(cam, { tasks: ['detect', 'pose', 'grasp'], modality: 'rgbd', environment: 'factory', sensor: 'realsense_d455', compute: 'intel_ipc', models: { detect: 'yolov8s', pose: 'depth_centroid', grasp: 'approach_grasp' }, classes: ['box'], confidence: 0.35, iou: 0.5, runtime: 'simulated', workingDistance: 1.1, simulate: false });
    const RDK = new Robolink(st);
    const found = await RDK.Cam2D_Detect(RDK.Item('Cell camera', ITEM_TYPE_CAMERA), RDK.Item('UR10e', 2));
    expect(found).toBe(3);
    expect(Number(RDK.getParam('VISION_FOUND'))).toBe(3);
    const pick = RDK.Item('Pick', 6);
    expect(pick.Valid()).toBe(true);
    const obj = st.findById((pick.item!.getParam('vision') as any).objectId)!;
    expect(obj.name).toMatch(/^Box \d$/);
    expect(Math.abs(pick.item!.poseAbs()[14] - 622)).toBeLessThan(1); // top face (620) + 2 mm standoff
  });
});

describe('fruit detector occlusion and harvest state', () => {
  it('detectFruit: a 6 m x 4 m wall between the camera and the row hides the fruit (ray cast on the rendered triangles)', () => {
    const st = new Station();
    const field = st.addChild(new FieldItem('F'));
    field.polygon = rectPolygon(20000, 10000);
    field.crop = cropParams('apple', { headland: 2000 });
    const row = generateOrchard(field, 3)[0];
    const [x, y] = row.pointAt(row.length() / 2, -1500);
    const cam = st.addChild(new Camera('Cam'));
    cam.fov = 70; cam.width = 640; cam.height = 480; cam.far = 8000;
    cam.setPose(mul(transl(x, y, 1500), rotz(90 * DEG), roty(90 * DEG)));
    const before = detectFruit(st, cam, { occlusionPerMeter: 0 }).length;
    expect(before).toBeGreaterThan(20);
    const [wx, wy] = row.pointAt(row.length() / 2, -700);
    const wall = st.addChild(new SceneObject('Wall'));
    wall.geometry = [{ primitive: { kind: 'box', size: [6000, 50, 4000] } }];
    wall.setPose(mul(transl(wx, wy, 2000), rotz(Math.atan2(row.direction()[1], row.direction()[0]))));
    const after = detectFruit(st, cam, { occlusionPerMeter: 0 });
    expect(after.length).toBeLessThan(before * 0.2);
    // whatever remains is seen past the wall's ends or above its top edge (z > 4 m) — never through it
    const half = 3000, dir = row.direction();
    for (const d of after) { const along = (d.p[0] - wx) * dir[0] + (d.p[1] - wy) * dir[1]; expect(Math.abs(along) > half - 600 || d.p[2] > 3000).toBe(true); }
  });

  it('harvest program generation does not mark fruit as picked before the program runs', async () => {
    const { generateHarvestProgram } = await import('../src/agri/missions');
    const { createRobotFromLibrary } = await import('../src/core/items/library');
    const st = new Station();
    const field = st.addChild(new FieldItem('F'));
    field.polygon = rectPolygon(20000, 10000);
    field.crop = cropParams('apple', { headland: 2000 });
    const row = generateOrchard(field, 3)[0];
    const [x, y] = row.pointAt(row.length() / 2, -1200);
    const robot = st.addChild(createRobotFromLibrary('UR10e'));
    robot.setPose(mul(transl(x, y, 600), rotz(Math.atan2(row.direction()[1], row.direction()[0]) + Math.PI / 2)));
    const ripeBefore = fruitWorldPositions(row as CropRow).length;
    generateHarvestProgram(st, robot, [row as CropRow], { limit: 5, maxAttempts: 40 });
    expect(fruitWorldPositions(row as CropRow).length).toBe(ripeBefore);
  });
});
