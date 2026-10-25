/**
 * T9 vision — closed loop detect → 3D → targets → robot program → attach on the pick & place demo cell, against the
 * real box poses (independent ground truth), plus pixel-true occlusion (same-class objects, other objects, the robot).
 * An eye-to-hand RGB-D camera looks down from 1.7 m on the three boxes of the "pickplace" demo (UR10e + vacuum gripper).
 * Ported from tests/_probe/vision_closedloop.test.ts (5 probes) and the depth-path / clipped-box probes of
 * tests/_probe/vision_runtime.test.ts.
 */
import { it, expect, describe } from 'vitest';
import { demos } from '../src/demos';
import { Camera, ItemType, SceneObject } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { mul, transl, rotx, rotz, DEG, invert, multiply, transformPoint } from '../src/core/math/pose';
import { setVisionStack, getVisionStack, VisionStackConfig } from '../src/vision/stack';
import { VisionRuntime, createTargetsFromOutput, detectionsToPickTargets } from '../src/vision/pipeline';
import { intrinsicsFromCamera } from '../src/vision/camera_model';

function cell(opts: { yaw?: number; seed?: number } = {}) {
  const st = demos.find((d) => d.id === 'pickplace')!.build();
  const robot = st.itemsOfType<Robot>(ItemType.ROBOT)[0];
  const boxes = st.itemsOfType<SceneObject>(ItemType.OBJECT).filter((o) => /^Box \d$/.test(o.name));
  if (opts.yaw) boxes[1].setPose(mul(boxes[1].pose(), rotz(opts.yaw * DEG)));
  const cam = st.addChild(new Camera('Cell camera'));
  cam.kind = 'depth'; cam.fov = 60; cam.width = 640; cam.height = 480; cam.near = 100; cam.far = 4000;
  cam.setPose(mul(transl(1020, -150, 1700), rotx(180 * DEG)));
  const cfg: VisionStackConfig = { tasks: ['detect', 'pose', 'grasp'], modality: 'rgbd', environment: 'factory', sensor: 'realsense_d455', compute: 'intel_ipc', models: { detect: 'yolov8s', pose: 'depth_centroid', grasp: 'approach_grasp' }, classes: ['box'], confidence: 0.35, iou: 0.5, runtime: 'simulated', workingDistance: 1.1, simulate: true, seed: opts.seed ?? 3 };
  setVisionStack(cam, cfg);
  return { st, robot, boxes, cam, cfg };
}
/** Demo box: 150x150x120 mm, geometry origin 60 mm up. */
const boxCentre = (b: SceneObject) => transformPoint(b.poseAbs(), [0, 0, 60]);
const boxTop = (b: SceneObject) => transformPoint(b.poseAbs(), [0, 0, 120]);

describe('closed loop on the pickplace cell', () => {
  it('grasp targets sit on the top face of the real boxes, tool Z down, reachable', async () => {
    const { st, robot, boxes, cam } = cell();
    const out = await new VisionRuntime(cam, getVisionStack(cam)!).step(st, { time: 0 });
    // one target per box (the noise model may add a false positive elsewhere, e.g. on the floor)
    const onBoxes = out.targets.filter((t) => boxes.some((b) => b.id === t.objectId));
    expect(new Set(onBoxes.map((t) => t.objectId)).size).toBe(3);
    expect(onBoxes.length).toBe(3);
    for (const t of onBoxes) {
      const box = boxes.find((b) => t.objectId === b.id)!;
      const top = boxTop(box);
      // TCP on the top face (+2 mm standoff), within the RealSense D455 noise after the plane fit
      expect(Math.hypot(t.grasp[12] - top[0], t.grasp[13] - top[1], t.grasp[14] - top[2])).toBeLessThan(15);
      expect(t.grasp[10]).toBeLessThan(-0.995); // tool Z into the part (down)
      const ik = robot.solveIK(multiply(invert(robot.poseAbs()), t.grasp));
      expect(ik.ok).toBe(true);
      // approach 100 mm above along the tool axis
      expect(t.approach[14] - t.grasp[14]).toBeCloseTo(100, 0);
    }
  });

  it('a program built from the vision targets picks the detected box without driving the tool into it', async () => {
    const { st, robot, boxes, cam } = cell();
    for (const p of st.itemsOfType<Program>(ItemType.PROGRAM)) p.delete();
    const out = await new VisionRuntime(cam, getVisionStack(cam)!).step(st, { time: 0 });
    const targets = createTargetsFromOutput(st, cam, out, { robotId: robot.id });
    const prog = st.addChild(new Program('VisionPick'));
    prog.setRobot(robot);
    const app = targets[0], grasp = targets[1];
    prog.addMoveJ(app); prog.addMoveL(grasp); prog.event('attach'); prog.addMoveL(app);
    const sim = new ProgramSimulator(st);
    sim.collisionOptions = { enabled: true, sampleStep: 0.02 } as any;
    const res = sim.compile(prog);
    sim.runToEnd();
    const expected = st.findById((grasp.getParam('vision') as any).objectId)!;
    expect(boxes).toContain(expected);
    expect([...sim.attachments.keys()].map((id) => st.findById(id)?.name)).toEqual([expected.name]);
    // reachable, and nothing collides (the tool stops 2 mm above the top face, the attached box leaves the table)
    expect(res.problems.filter((p) => p.severity === 'error').map((p) => p.message)).toEqual([]);
    expect(sim.collisions.length).toBe(0);
  });

  it('a box rotated 35 deg: the grasp X axis follows the part yaw (mod 90 deg, RoboDK Example-06.c TX,TY,RZ)', async () => {
    const { st, boxes, cam } = cell({ yaw: 35 });
    const out = await new VisionRuntime(cam, getVisionStack(cam)!).step(st, { time: 0 });
    const t = out.targets.find((x) => x.objectId === boxes[1].id)!;
    expect(t).toBeTruthy();
    const yawTarget = Math.atan2(t.grasp[1], t.grasp[0]) / DEG;
    const bp = boxes[1].poseAbs();
    const diff = (((yawTarget - Math.atan2(bp[1], bp[0]) / DEG) % 90) + 135) % 90 - 45;
    expect(Math.abs(diff)).toBeLessThan(5);
    // symmetric folding: the chosen X axis is the one closest to the robot X (lazy wrist), within ±45 deg
    expect(Math.abs(yawTarget)).toBeLessThanOrEqual(45.5);
  });

  it('pixel-true occlusion: a plate over box 2 and a same-class box stacked on box 1 hide them', async () => {
    const { st, boxes, cam } = cell();
    const plate = st.addChild(new SceneObject('Plate'));
    plate.geometry = [{ primitive: { kind: 'box', size: [400, 400, 10] } }];
    const c = boxTop(boxes[1]);
    plate.setPose(transl(c[0], c[1], c[2] + 100));
    const out = await new VisionRuntime(cam, { ...getVisionStack(cam)!, classes: [] }).step(st, { time: 0 });
    expect(out.frame.truth!.find((t) => t.id === `object:${boxes[1].id}`)?.visible).toBe(false);
    expect(out.frame.truth!.find((t) => t.cls === 'plate')?.visible).toBe(true);
    expect(out.detections.some((d) => d.truthId === `object:${boxes[1].id}`)).toBe(false);
    const c2 = cell(); const b1 = c2.boxes[0];
    const top = c2.st.addChild(new SceneObject('Box 9'));
    top.geometry = [{ primitive: { kind: 'box', size: [200, 200, 120] }, origin: Array.from(transl(0, 0, 60)) }];
    top.bbox = { min: [-100, -100, 0], max: [100, 100, 120] };
    const tp = boxTop(b1); top.setPose(transl(tp[0], tp[1], tp[2] + 5));
    const out2 = await new VisionRuntime(c2.cam, getVisionStack(c2.cam)!).step(c2.st, { time: 0 });
    expect(out2.frame.truth!.find((t) => t.id === `object:${b1.id}`)?.visible).toBe(false);
    expect(out2.frame.truth!.find((t) => t.id === `object:${top.id}`)?.visible).toBe(true);
  });

  it('the robot arm occludes: TCP parked 150 mm above box 2 between box and camera hides it', async () => {
    const { st, robot, boxes, cam } = cell();
    const top = boxTop(boxes[1]);
    const ik = robot.solveIK(multiply(invert(robot.poseAbs()), mul(transl(top[0], top[1], top[2] + 150), rotx(180 * DEG))));
    expect(ik.ok).toBe(true);
    robot.setJoints(ik.joints);
    const out = await new VisionRuntime(cam, getVisionStack(cam)!).step(st, { time: 0 });
    const t2 = out.frame.truth!.find((t) => t.id === `object:${boxes[1].id}`)!;
    expect(t2.visible).toBe(false);
    expect(t2.visibleFraction!).toBeLessThan(0.5);
    // the arm reaches over box 1 too; box 3, on the far side, stays fully visible
    expect(out.frame.truth!.find((t) => t.id === `object:${boxes[2].id}`)?.visibleFraction).toBeGreaterThan(0.95);
  });

  it('feasibility before creating targets: a box out of reach gives no target (IK), the others stay', async () => {
    const { st, robot, boxes, cam } = cell();
    boxes[2].setPose(mul(boxes[2].pose(), transl(900, 0, 0))); // ~1.9 m from the base: beyond the UR10e reach
    cam.setPose(mul(transl(1350, -150, 1900), rotx(180 * DEG)));
    const out = await new VisionRuntime(cam, getVisionStack(cam)!).step(st, { time: 0 });
    const picks = detectionsToPickTargets(st, robot, out.targets);
    const far = picks.find((p) => p.objectId === boxes[2].id)!;
    expect(far.reachable).toBe(false);
    expect(picks.filter((p) => p.reachable && p.collisionFree).length).toBe(2);
    const targets = createTargetsFromOutput(st, cam, out, { robotId: robot.id });
    expect(targets.length).toBe(4);
    expect(targets.every((t) => (t.getParam('vision') as any)?.objectId !== boxes[2].id || /approach/.test(t.name))).toBe(true);
  });
});

describe('depth-image localisation (probe vision_runtime)', () => {
  it('one 3D point definition: the depth-path error statistic vs the object centroid is < 30 mm', async () => {
    const { st, cam, cfg } = cell();
    setVisionStack(cam, { ...cfg, tasks: ['detect', 'pose', 'grasp', 'cloud_segment'] });
    const rt = new VisionRuntime(cam, getVisionStack(cam)!);
    const out = await rt.step(st, { time: 0 });
    expect(out.frame.depth).toBeDefined();
    expect(rt.stats.posErrN).toBe(3);
    expect(rt.stats.posErrSum / rt.stats.posErrN).toBeLessThan(30);
  });

  it('a box cut by the image border: the truncation-aware centre is within 40 mm of the real centre', async () => {
    const { st, cam, cfg, boxes } = cell();
    const K = intrinsicsFromCamera(cam);
    const b1c = boxCentre(boxes[0]);
    const z = 1700 - b1c[2];
    cam.setPose(mul(transl(b1c[0] + (K.cx / K.fx) * z, -150, 1700), rotx(180 * DEG))); // box 1 centre on the left border
    setVisionStack(cam, { ...cfg, seed: 11 });
    const rt = new VisionRuntime(cam, getVisionStack(cam)!);
    const errs: number[] = [];
    for (let k = 0; k < 6; k++) {
      const out = await rt.step(st, { time: k / 10 });
      const d = out.detections.find((x) => x.truthId === `object:${boxes[0].id}`);
      if (d?.p) errs.push(Math.hypot(d.p[0] - b1c[0], d.p[1] - b1c[1], d.p[2] - b1c[2]));
    }
    expect(errs.length).toBeGreaterThan(3);
    expect(Math.max(...errs)).toBeLessThan(40);
  });
});
