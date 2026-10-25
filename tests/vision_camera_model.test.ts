/**
 * T9 vision — one camera model (I4): the pinhole K of src/vision/camera_model.ts must agree with what the renderer
 * draws (three.js PerspectiveCamera built from threeFovDeg, exactly as SceneRenderer.renderFromItem /
 * renderCameraBuffers do), with RoboDK Cam2D (vertical FOV), with the catalogue datasheets (horizontal FOV) and with
 * Webots cameras (horizontal fieldOfView, e.g. MURKA's head camera).
 * Ported from the investigation probe tests/_probe/vision_fov.test.ts (4 failing probes) + new checks.
 */
import { it, expect, describe } from 'vitest';
import fs from 'node:fs';
import * as THREE from 'three';
import { Station, Camera, SceneObject, Frame } from '../src/core/items/item';
import { mul, transl, rotx, rotz, DEG, invert, transformPoint, Vec3 } from '../src/core/math/pose';
import { intrinsicsFromCamera, cameraIntrinsics, threeFovDeg, applySensorToCamera, cameraFovs, project, setFocalLengthPx } from '../src/vision/camera_model';
import { captureTruth } from '../src/vision/pipeline';
import { VISION_SENSORS } from '../src/vision/stack';
import { renderCamera } from '../src/vision/sensing';
import { cameraInfoMsg } from '../src/vision/vision_ros';

/** Pixel (image row 0 = top, pixel centres at +0.5) where the studio renderer draws a world point for this camera item. */
function renderedPixel(cam: Camera, p: Vec3, w = cam.width, h = cam.height): [number, number] {
  const tc = new THREE.PerspectiveCamera(threeFovDeg(cam, w, h), w / h, cam.near, cam.far);
  const m = new THREE.Matrix4().fromArray(Array.from(cam.poseAbs())).multiply(new THREE.Matrix4().makeRotationX(Math.PI));
  tc.matrixAutoUpdate = false; tc.matrix.copy(m); tc.matrixWorld.copy(m); tc.matrixWorldInverse.copy(m).invert(); tc.updateProjectionMatrix();
  const v = new THREE.Vector3(p[0], p[1], p[2]).project(tc);
  return [((v.x + 1) / 2) * w, ((1 - v.y) / 2) * h];
}
const uvOf = (cam: Camera, p: Vec3, w?: number, h?: number) => project(cameraIntrinsics(cam, w, h), transformPoint(invert(cam.poseAbs()), p))!;
const dist = (a: number[], b: number[]) => Math.hypot(a[0] - b[0], a[1] - b[1]);

describe('camera model vs renderer (probe vision_fov)', () => {
  it('640x480 fov 60 (vertical, RoboDK/three.js): an off-centre point lands on the same pixel in K and in the render', () => {
    const cam = new Station().addChild(new Camera('Cam'));
    cam.fov = 60; cam.width = 640; cam.height = 480;
    cam.setPose(mul(transl(0, 0, 2000), rotx(180 * DEG)));
    const K = intrinsicsFromCamera(cam);
    expect(K.fy).toBeCloseTo(240 / Math.tan(30 * DEG), 9); // three.js vertical fov
    expect(dist(uvOf(cam, [300, 200, 0]), renderedPixel(cam, [300, 200, 0]))).toBeLessThan(1e-6);
  });

  it('horizontal-fov camera (Webots / URDF): K and the render agree and the rendered hfov equals fov', () => {
    const cam = new Station().addChild(new Camera('Cam'));
    cam.fov = 90; cam.fovAxis = 'horizontal'; cam.width = 640; cam.height = 360;
    cam.setPose(mul(transl(100, -50, 1500), rotx(170 * DEG), rotz(20 * DEG)));
    for (const p of [[300, 200, 0], [-400, 100, 50], [0, 0, 0]] as Vec3[]) expect(dist(uvOf(cam, p), renderedPixel(cam, p))).toBeLessThan(1e-6);
    expect(cameraFovs(cam).h).toBeCloseTo(90, 9);
    expect(cameraIntrinsics(cam).fx).toBeCloseTo(320, 9);
  });

  it('an overlay drawn at another canvas size (Camera tab 640x400 for a 640x480 camera) maps with cameraIntrinsics(cam, w, h)', () => {
    const cam = new Station().addChild(new Camera('Cam'));
    cam.fov = 55; cam.width = 640; cam.height = 480;
    cam.setPose(mul(transl(0, 0, 1800), rotx(180 * DEG)));
    for (const [w, h] of [[640, 400], [320, 240], [1280, 720]]) {
      const p: Vec3 = [350, -220, 0];
      expect(dist(uvOf(cam, p, w, h), renderedPixel(cam, p, w, h))).toBeLessThan(1e-6);
    }
  });

  it('bin-picking camera 1280x720 fov 60: the truth box (instance-ID render) is centred on the rendered object', () => {
    const st = new Station();
    const table = st.addChild(new Frame('Table')); table.setPose(transl(600, 0, 700));
    const o = table.addChild(new SceneObject('Box 1'));
    o.geometry = [{ primitive: { kind: 'box', size: [120, 80, 60] } }];
    o.setPose(mul(transl(-150, -120, 30), rotz(15 * DEG)));
    const cam = st.addChild(new Camera('Cell camera'));
    cam.kind = 'depth'; cam.fov = 60; cam.width = 1280; cam.height = 720; cam.near = 100; cam.far = 3000;
    cam.setPose(mul(transl(600, 0, 1900), rotx(180 * DEG)));
    const truth = captureTruth(st, cam)[0];
    // the silhouette of a convex box is the hull of its 8 projected corners: compare with their bounding box
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    for (const cx of [-60, 60]) for (const cy of [-40, 40]) for (const cz of [-30, 30]) { const [u, v] = renderedPixel(cam, transformPoint(o.poseAbs(), [cx, cy, cz])); x0 = Math.min(x0, u); x1 = Math.max(x1, u); y0 = Math.min(y0, v); y1 = Math.max(y1, v); }
    // pixels whose centre lies inside the silhouette: edges within one pixel of the continuous bounds
    expect(Math.abs(truth.box.x - x0)).toBeLessThan(1); expect(Math.abs(truth.box.x + truth.box.w - x1)).toBeLessThan(1);
    expect(Math.abs(truth.box.y - y0)).toBeLessThan(1); expect(Math.abs(truth.box.y + truth.box.h - y1)).toBeLessThan(1);
    expect(truth.visible).toBe(true);
  });

  it('catalogue wizard (applySensorToCamera): the rendered horizontal FOV equals the datasheet hfov', () => {
    for (const id of ['zed_x', 'realsense_d455', 'usb_rgb']) {
      const s = VISION_SENSORS.find((x) => x.id === id)!;
      const cam = new Camera('C');
      applySensorToCamera(cam, s);
      const vfov = threeFovDeg(cam); // what three.js receives
      const renderedHfov = (2 * Math.atan(Math.tan((vfov * DEG) / 2) * cam.width / cam.height)) / DEG;
      expect(Math.abs(renderedHfov - s.hfov)).toBeLessThan(1e-6);
      expect(cam.width / cam.height).toBeCloseTo(s.resolution[0] / s.resolution[1], 2);
    }
  });

  it('RoboDK Cam2D FOV is vertical: a point at the top edge of a FOV=32 640x480 camera maps to v = 0', () => {
    const cam = new Camera('C'); cam.fov = 32; cam.width = 640; cam.height = 480;
    const z = 1000, yTop = -Math.tan(16 * DEG) * z;
    expect(Math.abs(project(intrinsicsFromCamera(cam), [0, yTop, z])![1])).toBeLessThan(1e-9);
  });

  it('FOCAL_LENGTH / PIXELSIZE → focal length in pixels (setFocalLengthPx keeps the fov axis)', () => {
    const cam = new Camera('C'); cam.width = 640; cam.height = 480;
    setFocalLengthPx(cam, 1000); // 6 mm lens, 6 µm pixels
    expect(cameraIntrinsics(cam).fy).toBeCloseTo(1000, 9);
    cam.fovAxis = 'horizontal'; setFocalLengthPx(cam, 800);
    expect(cameraIntrinsics(cam).fx).toBeCloseTo(800, 9);
  });
});

describe('metric depth render and ROS CameraInfo', () => {
  it('software depth render is metric along the optical axis (plane 1 m in front, tilted plane analytic)', () => {
    const st = new Station();
    const wall = st.addChild(new SceneObject('Wall'));
    wall.geometry = [{ primitive: { kind: 'box', size: [4000, 4000, 10] } }];
    wall.setPose(transl(0, 0, -5)); // top face at z = 0
    const cam = st.addChild(new Camera('Cam'));
    cam.fov = 60; cam.width = 160; cam.height = 120; cam.near = 10; cam.far = 5000;
    cam.setPose(mul(transl(0, 0, 1000), rotx(180 * DEG)));
    const r = renderCamera(st, cam);
    let n = 0, maxErr = 0;
    for (let i = 0; i < r.depth.length; i++) if (r.depth[i] > 0) { n++; maxErr = Math.max(maxErr, Math.abs(r.depth[i] - 1000)); }
    expect(n).toBe(160 * 120);
    expect(maxErr).toBeLessThan(1e-3);
    // tilt the camera 30 deg about its X axis: depth along +Z of the camera at pixel (u, v) = h / (n · ray)
    cam.setPose(mul(transl(0, 0, 1000), rotx(150 * DEG)));
    const r2 = renderCamera(st, cam), K = cameraIntrinsics(cam), Cw = cam.poseAbs();
    for (const [u, v] of [[80, 60], [10, 100], [150, 5]]) {
      const ray = [(u + 0.5 - K.cx) / K.fx, (v + 0.5 - K.cy) / K.fy, 1];
      const rz = Cw[2] * ray[0] + Cw[6] * ray[1] + Cw[10] * ray[2]; // world z component of the ray (per unit optical z)
      expect(r2.depth[v * 160 + u]).toBeCloseTo(1000 / -rz, 2);
    }
  });

  it('CameraInfo K reprojects 3D points onto the rendered pixel (ROS integer pixel centres)', () => {
    const cam = new Camera('C'); cam.fov = 1.2 / DEG; cam.fovAxis = 'horizontal'; cam.width = 320; cam.height = 240;
    const info = cameraInfoMsg(cam, 'head_camera_optical_frame');
    const p: Vec3 = [120, -45, 900];
    const uv = project(cameraIntrinsics(cam), p)!; // studio: pixel centres at +0.5
    const uRos = info.k[0] * p[0] / p[2] + info.k[2], vRos = info.k[4] * p[1] / p[2] + info.k[5];
    expect(uRos).toBeCloseTo(uv[0] - 0.5, 9);
    expect(vRos).toBeCloseTo(uv[1] - 0.5, 9);
    expect(info.p[0]).toBe(info.k[0]);
  });

  const MURKA_PROTO = 'C:/Users/TRITON 300 SE/YandexDisk/Работа/КБНЦ РАН/MURKA_Simulation/robot-retailer-murka-v2/ros2_ws/src/murom_webots/protos/Murom.proto';
  it.skipIf(!fs.existsSync(MURKA_PROTO))('MURKA head_camera (Webots fieldOfView, horizontal): studio K equals the webots_ros2 camera_info K', () => {
    const src = fs.readFileSync(MURKA_PROTO, 'utf8');
    const m = /Camera\s*\{[^}]*?name\s+"head_camera"[^}]*?width\s+(\d+)[^}]*?height\s+(\d+)[^}]*?fieldOfView\s+([\d.]+)/s.exec(src);
    expect(m).toBeTruthy();
    const [W, H, fov] = [Number(m![1]), Number(m![2]), Number(m![3])];
    const cam = new Camera('head_camera'); cam.width = W; cam.height = H; cam.fov = fov / DEG; cam.fovAxis = 'horizontal';
    const K = cameraIntrinsics(cam);
    // webots_ros2_driver Ros2Camera: focalLength = width / (2 tan(fov / 2)), same for x and y
    const fWebots = W / (2 * Math.tan(fov / 2));
    expect(K.fx).toBeCloseTo(fWebots, 9);
    expect(K.fy).toBeCloseTo(fWebots, 9);
    expect(fWebots).toBeCloseTo(233.87, 2); // 320 px, 1.2 rad
    expect(cameraInfoMsg(cam, 'f').k[0]).toBeCloseTo(fWebots, 9);
  });
});

describe('overlays on a canvas of another aspect', () => {
  it('rescalePixel maps a camera-image pixel onto a 640x400 render of a 640x480 / 1280x720 camera (Camera tab)', async () => {
    const { rescalePixel } = await import('../src/vision/camera_model');
    for (const [W, H, axis] of [[640, 480, 'vertical'], [1280, 720, 'horizontal']] as const) {
      const cam = new Station().addChild(new Camera('Cam'));
      cam.fov = 58; cam.fovAxis = axis; cam.width = W; cam.height = H;
      cam.setPose(mul(transl(0, 0, 1800), rotx(180 * DEG), rotz(15 * DEG)));
      for (const p of [[300, 200, 0], [-500, 250, 100]] as Vec3[]) {
        const [u, v] = uvOf(cam, p);
        expect(dist(rescalePixel(cam, u, v, 640, 400), renderedPixel(cam, p, 640, 400))).toBeLessThan(1e-6);
      }
    }
  });
});
