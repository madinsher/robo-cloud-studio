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
import { mul, transl, rotx, rotz, DEG, invert, multiply, transformPoint, Vec3 } from '../src/core/math/pose';
import { setVisionStack, getVisionStack, VisionStackConfig } from '../src/vision/stack';
import { VisionRuntime, createTargetsFromOutput, detectionsToPickTargets } from '../src/vision/pipeline';
import { intrinsicsFromCamera, cameraIntrinsics, project, applySensorToCamera } from '../src/vision/camera_model';
import type { VisionModel, VisionFrame, RunOptions, Box2D } from '../src/vision/models';
import { StationRuntime } from '../src/core/motion/runtime';
import { Target, Tool } from '../src/core/items/item';
import { recommendVisionStacks, configFromRecommendation, cameraKindForModality } from '../src/vision/stack';
import { installVisionRunner, runVisionCall } from '../src/vision/program_vision';

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

/** A detector that reports the truth boxes (score `truthScore`) plus extra boxes: what any real detector may answer. */
function injected(extra: Box2D[], truthScore = 0.6): VisionModel {
  return {
    id: 'inj', name: 'injected', tasks: ['detect'],
    async run(frame: VisionFrame, _o: RunOptions) {
      const boxes: Box2D[] = (frame.truth ?? []).filter((t) => t.visible && t.cls === 'box').map((t) => ({ ...t.box, score: truthScore, cls: t.cls, truthId: t.id }));
      return { task: 'detect', model: 'inj', boxes: [...extra.map((b) => ({ ...b })), ...boxes], latencyMs: 0 };
    },
  };
}
const pix = (cam: Camera, p: Vec3) => project(cameraIntrinsics(cam), transformPoint(invert(cam.poseAbs()), p))!;

describe('false positives never become pick targets (V6 F2: the table top was lifted)', () => {
  it("boxes on the bare table, inside a part's top face and across a part's edge get no 3D position; Pick is a real part", async () => {
    const { st, robot, boxes, cam } = cell();
    const b2 = boxTop(boxes[1]);
    const onTable = pix(cam, [b2[0], b2[1] + 300, 500]); // the table top surface, 300 mm beside box 2
    const inTop = pix(cam, b2);
    const edge = pix(cam, transformPoint(boxes[0].poseAbs(), [-75, 0, 120]));
    const fp = (c: number[], w: number, h: number, score: number): Box2D => ({ x: c[0] - w / 2, y: c[1] - h / 2, w, h, score, cls: 'box' });
    const fps = [fp(onTable, 30, 22, 0.95), fp(onTable, 9, 12, 0.9), fp(inTop, 16, 16, 0.93), fp(edge, 22, 22, 0.92)];
    for (const prior of [undefined, 75]) {
      const rt = new VisionRuntime(cam, { ...getVisionStack(cam)!, objectSizeMm: prior });
      rt.models.set('detect', injected(fps));
      const out = await rt.step(st, { time: 0 });
      // the false positives (listed first, scored higher than the parts) have no 3D position, hence no target
      const fpDet = out.detections.filter((d) => !d.truthId);
      expect(fpDet.length).toBe(4);
      expect(fpDet.map((d) => d.p ?? null), `prior ${prior}`).toEqual([null, null, null, null]);
      // every target is on the top face of a real box
      expect(out.targets.length).toBe(3);
      for (const t of out.targets) expect(Math.min(...boxes.map((b) => { const c = boxTop(b); return Math.hypot(t.grasp[12] - c[0], t.grasp[13] - c[1], t.grasp[14] - c[2]); }))).toBeLessThan(15);
      createTargetsFromOutput(st, cam, out, { robotId: robot.id });
      const pick = st.find('Pick', ItemType.TARGET)!;
      expect(boxes.map((b) => b.id)).toContain((pick.getParam('vision') as any).objectId);
    }
  });

  it('Pick holds the most confident feasible target, not the first one in detector order', async () => {
    const { st, robot, boxes, cam } = cell();
    const rt = new VisionRuntime(cam, { ...getVisionStack(cam)!, simulate: false });
    rt.models.set('detect', {
      id: 'inj', name: 'inj', tasks: ['detect'],
      async run(frame: VisionFrame) {
        const tr = (frame.truth ?? []).filter((t) => t.visible && t.cls === 'box').sort((a, b) => a.box.x - b.box.x);
        return { task: 'detect', model: 'inj', boxes: tr.map((t, i) => ({ ...t.box, score: [0.5, 0.6, 0.9][i] ?? 0.4, cls: t.cls, truthId: t.id })), latencyMs: 0 };
      },
    });
    const out = await rt.step(st, { time: 0 });
    expect(out.targets.map((t) => t.score)).toEqual([0.5, 0.6, 0.9]);
    createTargetsFromOutput(st, cam, out, { robotId: robot.id });
    const best = out.targets[2];
    expect((st.find('Pick', ItemType.TARGET)!.getParam('vision') as any).source).toBe(best.name);
    expect(best.objectId).toBe(boxes.slice().sort((a, b) => pix(cam, boxTop(a))[0] - pix(cam, boxTop(b))[0])[2].id);
  });

  it('wizard stack (Zivid, "Simulate detections" on): VisionDetect -> IF -> PickPart picks a box over many layouts, never the table top', async () => {
    let picked = 0;
    for (let k = 0; k < 6; k++) {
      const st = demos.find((d) => d.id === 'pickplace')!.build();
      for (const p of st.itemsOfType<Program>(ItemType.PROGRAM)) p.delete();
      const robot = st.itemsOfType<Robot>(ItemType.ROBOT)[0];
      const boxes = st.itemsOfType<SceneObject>(ItemType.OBJECT).filter((o) => /^Box \d$/.test(o.name));
      const table = st.find('Table top', ItemType.OBJECT) as SceneObject;
      const cam = st.addChild(new Camera('Cell camera'));
      cam.near = 100; cam.far = 4000;
      cam.setPose(mul(transl(1020, -150, 1700), rotx(180 * DEG)));
      const req = { tasks: ['detect', 'track', 'pose', 'grasp'] as any, environment: 'factory' as any, modalities: ['rgbd' as any], workingDistance: 1.2, needs3D: true, minFps: 10 };
      const rec = recommendVisionStacks(req)[0];
      setVisionStack(cam, { ...configFromRecommendation(rec, req, ['box']), seed: 1 + 13 * k });
      cam.kind = cameraKindForModality(rec.sensor.modality); applySensorToCamera(cam, rec.sensor);
      await runVisionCall(st, ['Cell camera'], { robot });
      const pickT = st.find('Pick', ItemType.TARGET) as Target, appT = st.find('Pick approach', ItemType.TARGET) as Target;
      const pick = st.addChild(new Program('PickPart')); pick.setRobot(robot);
      pick.addMoveJ(appT); pick.addMoveL(pickT); pick.event('attach'); pick.addMoveL(appT);
      const none = st.addChild(new Program('NoPart')); none.setRobot(robot); none.setDO('NoPart', 1);
      const main = st.addChild(new Program('VisionMain')); main.setRobot(robot);
      main.runInstruction('VisionDetect("Cell camera")', true);
      main.addInstruction({ kind: 'if', condition: 'VISION_FOUND > 0', thenProgramId: pick.id, elseProgramId: none.id });
      // the parts at random places on the table (>= 225 mm apart)
      let seed = 1000 + (k + 11) * 7919; const rnd = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
      let L: number[][] = [];
      do L = boxes.map(() => [60 + rnd() * 520, rnd() * 380, rnd() * 180 - 90]); while (!L.every((p, i) => L.every((q, j) => j <= i || Math.hypot(p[0] - q[0], p[1] - q[1]) > 225)));
      boxes.forEach((b, i) => b.setPose(mul(transl(L[i][0], L[i][1], 500), rotz(L[i][2] * DEG))));
      const tops = boxes.map((b) => boxTop(b));
      const tz = table.poseAbs()[14];
      const rt = new StationRuntime(st);
      installVisionRunner(rt);
      const h = rt.start(main);
      await rt.runUntilDone(120);
      expect((await h.done).status).toBe('done');
      const carried = st.itemsOfType<Tool>(ItemType.TOOL).flatMap((t) => t.children.filter((c) => c instanceof SceneObject).map((c) => c.name));
      expect(carried).not.toContain('Table top');
      expect(table.poseAbs()[14]).toBe(tz);
      // no detection without a part under it got a 3D position (the default simulated detector adds false positives)
      const out = (await import('../src/vision/pipeline')).visionRuntimeOf(cam)!.last!;
      for (const d of out.detections) if (d.p && !d.truthId) throw new Error(`false positive with a 3D position: ${JSON.stringify([d.x, d.y, d.w, d.h, d.p])}`);
      if (rt.signals.get('VISION_FOUND')) {
        expect(carried.length).toBe(1);
        expect(carried[0]).toMatch(/^Box \d$/);
        const pk = pickT.poseAbs();
        expect(Math.min(...tops.map((c) => Math.hypot(pk[12] - c[0], pk[13] - c[1], pk[14] - c[2])))).toBeLessThan(15);
        expect(pk[10]).toBeLessThan(-0.99);
        picked++;
      } else expect(rt.signals.get('NoPart')).toBeTruthy();
    }
    expect(picked).toBeGreaterThanOrEqual(4);
  }, 300000);
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
