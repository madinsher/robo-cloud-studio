/**
 * T9 vision — P1: hand-eye calibration (eye-to-hand and eye-in-hand, from simulated marker measurements through the
 * camera with occlusion and sensor noise) recovers the real camera pose; synthetic dataset export (YOLO / COCO) labels
 * match the rendered image (labels from the instance-ID buffer, RLE area = mask pixels).
 */
import { it, expect, describe } from 'vitest';
import { demos } from '../src/demos';
import { Camera, ItemType, SceneObject, Tool } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { mul, transl, rotx, roty, rotz, DEG, invert, multiply, transformPoint, Mat4 } from '../src/core/math/pose';
import { setVisionStack, getVisionStack, VisionStackConfig } from '../src/vision/stack';
import { collectHandEyeSamples, solveEyeToHand, solveEyeInHand, applyHandEye, cameraPoseFromHandEye } from '../src/vision/handeye';
import { captureDatasetSample, yoloDataset, cocoDataset } from '../src/vision/dataset';
import { generateRosVisionPackage, handEyeLink } from '../src/vision/vision_ros';
import { decodePNG } from '../src/vision/png';

const cfg: VisionStackConfig = { tasks: ['detect', 'pose', 'grasp'], modality: 'rgbd', environment: 'factory', sensor: 'realsense_d455', compute: 'intel_ipc', models: { detect: 'yolov8s', pose: 'depth_centroid', grasp: 'approach_grasp' }, classes: ['box'], confidence: 0.35, iou: 0.5, runtime: 'simulated', workingDistance: 1.1, simulate: true, seed: 3 };
function cell() {
  const st = demos.find((d) => d.id === 'pickplace')!.build();
  const robot = st.itemsOfType<Robot>(ItemType.ROBOT)[0];
  const boxes = st.itemsOfType<SceneObject>(ItemType.OBJECT).filter((o) => /^Box \d$/.test(o.name));
  return { st, robot, boxes };
}
const rotErrDeg = (a: Mat4, b: Mat4) => { const t = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[4] * b[4] + a[5] * b[5] + a[6] * b[6] + a[8] * b[8] + a[9] * b[9] + a[10] * b[10]; return (Math.acos(Math.max(-1, Math.min(1, (t - 1) / 2))) * 180) / Math.PI; };
const posErr = (a: Mat4, b: Mat4) => Math.hypot(a[12] - b[12], a[13] - b[13], a[14] - b[14]);

describe('hand-eye calibration', () => {
  it('eye-to-hand: the camera pose in the robot base is recovered exactly without noise, within the sensor error with noise', () => {
    const { st, robot, boxes } = cell();
    const cam = st.addChild(new Camera('Cell camera'));
    cam.kind = 'depth'; cam.fov = 60; cam.width = 640; cam.height = 480; cam.near = 100; cam.far = 4000;
    cam.setPose(mul(transl(1020, -150, 1700), rotx(180 * DEG), rotz(7 * DEG)));
    setVisionStack(cam, cfg);
    // start the robot with the TCP above box 2, in the camera view
    const top = transformPoint(boxes[1].poseAbs(), [0, 0, 120]);
    robot.setJoints(robot.solveIK(multiply(invert(robot.poseAbs()), mul(transl(top[0], top[1], top[2] + 250), rotx(180 * DEG)))).joints);
    const truth = multiply(invert(robot.poseAbs()), cam.poseAbs());
    const q0 = robot.joints();
    const exact = solveEyeToHand(collectHandEyeSamples(st, robot, cam, 'eye_to_hand', { n: 12, noise: false }));
    expect(exact.samples).toBe(12);
    expect(posErr(exact.pose, truth)).toBeLessThan(1e-4);
    expect(rotErrDeg(exact.pose, truth)).toBeLessThan(1e-4);
    expect(robot.joints()).toEqual(q0); // the robot is restored
    const noisy = solveEyeToHand(collectHandEyeSamples(st, robot, cam, 'eye_to_hand', { n: 20, noise: true, sensor: 'realsense_d455', seed: 4 }));
    expect(posErr(noisy.pose, truth)).toBeLessThan(5);
    expect(rotErrDeg(noisy.pose, truth)).toBeLessThan(1);
    expect(noisy.rms).toBeGreaterThan(0.1);
    // stored in the stack: the camera pose follows from the calibration, and the ROS export publishes it
    const next = applyHandEye(cam, getVisionStack(cam)!, exact);
    expect(posErr(cameraPoseFromHandEye(robot, next)!, cam.poseAbs())).toBeLessThan(1e-4);
    const he = handEyeLink(cam, getVisionStack(cam)!);
    expect(he.parent).toBe('base_link');
    expect(posErr(he.optical, truth)).toBeLessThan(1e-4);
    expect(generateRosVisionPackage(cam, getVisionStack(cam)!).files['launch/perception.launch.py']).toContain("'--frame-id', 'base_link'");
  });

  it('eye-in-hand: camera pose in the flange and the fixed marker position are recovered', () => {
    const { st, robot, boxes } = cell();
    const tool = st.itemsOfType<Tool>(ItemType.TOOL)[0];
    const cam = tool.addChild(new Camera('Wrist camera'));
    cam.fov = 60; cam.width = 640; cam.height = 480; cam.near = 50; cam.far = 3000;
    cam.setPose(mul(transl(0, 90, 60), rotx(-12 * DEG), roty(4 * DEG))); // beside the gripper, looking down along the tool
    setVisionStack(cam, { ...cfg, handEyeMode: 'eye_in_hand' });
    const top = transformPoint(boxes[1].poseAbs(), [0, 0, 120]);
    robot.setJoints(robot.solveIK(multiply(invert(robot.poseAbs()), mul(transl(top[0], top[1] - 90, top[2] + 450), rotx(180 * DEG)))).joints);
    const flangeAbs = () => multiply(robot.poseAbs(), robot.fk().flange);
    const truth = multiply(invert(flangeAbs()), cam.poseAbs());
    const samples = collectHandEyeSamples(st, robot, cam, 'eye_in_hand', { n: 14, markerWorld: top, noise: false, radiusMm: 80, tiltDeg: 15 });
    expect(samples.length).toBe(14);
    const res = solveEyeInHand(samples);
    expect(posErr(res.pose, truth)).toBeLessThan(1e-3);
    expect(rotErrDeg(res.pose, truth)).toBeLessThan(1e-3);
    const mb = transformPoint(invert(robot.poseAbs()), top);
    expect(Math.hypot(res.marker![0] - mb[0], res.marker![1] - mb[1], res.marker![2] - mb[2])).toBeLessThan(1e-3);
    const noisy = solveEyeInHand(collectHandEyeSamples(st, robot, cam, 'eye_in_hand', { n: 24, markerWorld: top, noise: true, sensor: 'realsense_d455', radiusMm: 80, tiltDeg: 15, seed: 9 }));
    expect(posErr(noisy.pose, truth)).toBeLessThan(5);
    expect(rotErrDeg(noisy.pose, truth)).toBeLessThan(1.5);
  });
});

describe('synthetic dataset export', () => {
  function scene() {
    const { st, boxes } = cell();
    const cam = st.addChild(new Camera('Cell camera'));
    cam.fov = 60; cam.width = 320; cam.height = 240; cam.near = 100; cam.far = 4000;
    cam.setPose(mul(transl(1020, -150, 1700), rotx(180 * DEG)));
    return { st, cam, boxes };
  }
  it('YOLO labels: one line per visible box, normalised centre/size from the ID buffer (bbox + polygon)', () => {
    const { st, cam, boxes } = scene();
    const s = captureDatasetSample(st, cam, 'f0', { classes: ['box'] });
    expect(s.objects.length).toBe(3);
    const files = yoloDataset([s], ['box']);
    const lines = (files['labels/train/f0.txt'] as string).trim().split('\n').map((l) => l.split(' ').map(Number));
    expect(lines.length).toBe(3);
    const img = decodePNG(files['images/train/f0.png'] as Uint8Array);
    expect([img.width, img.height]).toEqual([320, 240]);
    for (const b of boxes) {
      // centre of the box top face projects inside its label box
      const K = { fx: 120 / Math.tan(30 * DEG), cx: 160, cy: 120 };
      const p = transformPoint(invert(cam.poseAbs()), transformPoint(b.poseAbs(), [0, 0, 120]));
      const u = (K.cx + (K.fx * p[0]) / p[2]) / 320, v = (K.cy + (K.fx * p[1]) / p[2]) / 240;
      const l = lines.find((q) => Math.abs(q[1] - u) < q[3] / 2 && Math.abs(q[2] - v) < q[4] / 2)!;
      expect(l).toBeTruthy();
      expect(l[0]).toBe(0);
      expect(Math.abs(l[1] - u) * 320).toBeLessThan(3); // camera straight above: label centre ≈ projected top centre
    }
    expect(files['data.yaml']).toMatch(/names:\n {2}0: box/);
    const seg = yoloDataset([s], ['box'], { seg: true });
    const poly = (seg['labels/train/f0.txt'] as string).trim().split('\n')[0].split(' ').slice(1).map(Number);
    expect(poly.length % 2).toBe(0);
    expect(poly.length).toBeGreaterThan(8);
    expect(poly.every((v) => v >= 0 && v <= 1)).toBe(true);
  });
  it('COCO: bbox, area = mask pixels, uncompressed RLE decodes to the same pixel count', () => {
    const { st, cam } = scene();
    const s = captureDatasetSample(st, cam, 'f0', { classes: ['box'] });
    const coco = JSON.parse(cocoDataset([s], ['box'])['annotations.json'] as string);
    expect(coco.images).toEqual([{ id: 1, file_name: 'images/f0.png', width: 320, height: 240 }]);
    expect(coco.annotations.length).toBe(3);
    for (const a of coco.annotations) {
      const counts: number[] = a.segmentation.counts;
      expect(counts.reduce((x: number, y: number) => x + y, 0)).toBe(320 * 240);
      const ones = counts.filter((_, i) => i % 2 === 1).reduce((x, y) => x + y, 0);
      expect(ones).toBe(a.area);
      expect(a.area).toBeGreaterThan(0.8 * a.bbox[2] * a.bbox[3]); // a box seen from above fills its bbox
    }
  });
});
