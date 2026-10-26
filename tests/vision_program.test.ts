/**
 * T9 vision — vision inside robot programs (P0 g): a code instruction VisionDetect("<camera>") runs the camera pipeline
 * on the CURRENT station state through the StationRuntime script runner (contract I5), refreshes the "Pick" targets the
 * program moves to, and sets VISION_FOUND for an IF branch. The parts are moved after teaching: the robot must pick them
 * where they are now (RoboDK Example-06.c style closed loop), and branch to "not found" when the table is empty.
 */
import { it, expect, describe } from 'vitest';
import { demos } from '../src/demos';
import { Camera, ItemType, SceneObject, Target, Tool } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import { StationRuntime } from '../src/core/motion/runtime';
import { mul, transl, rotx, rotz, DEG, transformPoint } from '../src/core/math/pose';
import { setVisionStack, VisionStackConfig } from '../src/vision/stack';
import { installVisionRunner, runVisionCall, visionSignalName } from '../src/vision/program_vision';

function cell() {
  const st = demos.find((d) => d.id === 'pickplace')!.build();
  for (const p of st.itemsOfType<Program>(ItemType.PROGRAM)) p.delete();
  const robot = st.itemsOfType<Robot>(ItemType.ROBOT)[0];
  const boxes = st.itemsOfType<SceneObject>(ItemType.OBJECT).filter((o) => /^Box \d$/.test(o.name));
  const cam = st.addChild(new Camera('Cell camera'));
  cam.kind = 'depth'; cam.fov = 60; cam.width = 640; cam.height = 480; cam.near = 100; cam.far = 4000;
  cam.setPose(mul(transl(1020, -150, 1700), rotx(180 * DEG)));
  const cfg: VisionStackConfig = { tasks: ['detect', 'pose', 'grasp'], modality: 'rgbd', environment: 'factory', sensor: 'realsense_d455', compute: 'intel_ipc', models: { detect: 'yolov8s', pose: 'depth_centroid', grasp: 'approach_grasp' }, classes: ['box'], confidence: 0.35, iou: 0.5, runtime: 'simulated', workingDistance: 1.1, simulate: false, seed: 5 }; // exact detector: the closed-loop logic is under test (noise: vision_closedloop)
  setVisionStack(cam, cfg);
  return { st, robot, boxes, cam };
}

/** Teach once (creates Vision frame + Pick targets), then build: VisionDetect → IF VISION_FOUND > 0 → pick, else NoPart. */
async function teach(c: ReturnType<typeof cell>) {
  const { st, robot } = c;
  await runVisionCall(st, ['Cell camera'], { robot });
  const pickT = st.find('Pick', ItemType.TARGET) as Target, appT = st.find('Pick approach', ItemType.TARGET) as Target;
  const pick = st.addChild(new Program('PickPart')); pick.setRobot(robot);
  pick.addMoveJ(appT); pick.addMoveL(pickT); pick.event('attach'); pick.addMoveL(appT);
  const none = st.addChild(new Program('NoPart')); none.setRobot(robot); none.setDO('NoPart', 1);
  const main = st.addChild(new Program('VisionMain')); main.setRobot(robot);
  main.runInstruction('VisionDetect("Cell camera")', true);
  main.addInstruction({ kind: 'if', condition: 'VISION_FOUND > 0', thenProgramId: pick.id, elseProgramId: none.id });
  return { main, pickT };
}

describe('VisionDetect in a robot program (StationRuntime)', () => {
  it('parts moved after teaching are picked where they are now (top face, yaw aligned), without errors', async () => {
    const c = cell();
    const { main, pickT } = await teach(c);
    // move every box by +45 mm in x and turn it 20 deg after teaching
    for (const b of c.boxes) b.setPose(mul(b.pose(), transl(45, -20, 0), rotz(20 * DEG)));
    const placed = new Map(c.boxes.map((b) => [b.id, b.poseAbs()])); // where the parts are when the program starts
    const rt = new StationRuntime(c.st);
    installVisionRunner(rt);
    const h = rt.start(main);
    await rt.runUntilDone(120);
    const res = await h.done;
    expect(res.status).toBe('done');
    expect(rt.problems.filter((p) => p.severity === 'error')).toEqual([]);
    expect(rt.signals.get('VISION_FOUND')).toBe(3);
    expect(rt.signals.get(visionSignalName(c.cam))).toBe(3);
    // the refreshed Pick target names the box it was computed on; that box is now carried by the gripper
    const objectId = (pickT.getParam('vision') as any).objectId;
    const box = c.st.findById(objectId) as SceneObject;
    expect(c.boxes).toContain(box);
    expect(box.parent).toBeInstanceOf(Tool);
    // where the robot grasped: Pick holds the moved box's top face centre (+2 mm standoff) and its yaw (mod 90)
    const top = transformPoint(placed.get(box.id)!, [0, 0, 120]);
    const pk = pickT.poseAbs();
    expect(Math.hypot(pk[12] - top[0], pk[13] - top[1], pk[14] - top[2])).toBeLessThan(15);
    const bp = placed.get(box.id)!;
    const dyaw = ((((Math.atan2(pk[1], pk[0]) - Math.atan2(bp[1], bp[0])) / DEG) % 90) + 135) % 90 - 45;
    expect(Math.abs(dyaw)).toBeLessThan(5);
    expect(rt.signals.get('NoPart')).toBeUndefined();
    // and it left with the gripper: 100 mm above where it was (MoveL back to Pick approach)
    expect(box.poseAbs()[14] - bp[14]).toBeCloseTo(100, 0);
  }, 120000);

  it('an empty table: VISION_FOUND = 0 and the program takes the "not found" branch', async () => {
    const c = cell();
    const { main } = await teach(c);
    for (const b of c.boxes) b.delete();
    const rt = new StationRuntime(c.st);
    installVisionRunner(rt);
    const h = rt.start(main);
    await rt.runUntilDone(60);
    expect((await h.done).status).toBe('done');
    expect(rt.signals.get('VISION_FOUND')).toBe(0);
    expect(rt.signals.get('NoPart')).toBeTruthy();
    expect(c.st.getParam('VISION_TARGET')).toBe('');
  }, 120000);

  it('an unknown camera fails the run at the call (explicit error, no silent success)', async () => {
    const c = cell();
    const p = c.st.addChild(new Program('Bad')); p.setRobot(c.robot);
    p.runInstruction('VisionDetect("No such camera")', true);
    const rt = new StationRuntime(c.st);
    installVisionRunner(rt);
    const h = rt.start(p);
    await rt.runUntilDone(10);
    const res = await h.done;
    expect(res.status).toBe('error');
    expect(String(res.error?.message ?? '')).toMatch(/camera "No such camera" not found/);
  });
});
