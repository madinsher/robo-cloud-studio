// T3 (sim): RoboDK motion semantics in the program simulator / runtime (sim_kin report S1-S14):
// blending (rounding zones at constant path speed), dense paths, MoveJ IK on the pose, joint velocity limits,
// RunInstruction calls, synchronised external axes (turntable, rail), named-tool attach, joint targets.
import { describe, it, expect } from 'vitest';
import { Station, Frame, Target, Tool, SceneObject, ItemType } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import { createRobotFromLibrary } from '../src/core/items/library';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { StationRuntime } from '../src/core/motion/runtime';
import { buildMechanism, MAKE_ROBOT_1R, MAKE_ROBOT_1T } from '../src/core/kinematics/mechanism';
import { getPos, multiply, invert, transl, mul, rotx, DEG, distance, rotationAngle } from '../src/core/math/pose';
import { sampleAt } from '../src/core/motion/trajectory';
import { generateCurveFollow } from '../src/core/motion/pathfollow';
import { generateMachining } from '../src/core/motion/machining';

const tcpErr = (robot: Robot, target: Target, tool?: Tool) => {
  const tcp = multiply(robot.poseAbs(), robot.solveFK(robot.joints(), (tool ?? robot.activeTool())?.poseTool()));
  return { pos: distance(getPos(tcp), getPos(target.poseAbs())), rotDeg: rotationAngle(tcp, target.poseAbs()) / DEG };
};
function ur5Cell() {
  const st = new Station('sim');
  const robot = st.addChild(createRobotFromLibrary('UR5e', 'UR5e'));
  const tool = robot.addChild(new Tool('T150')); tool.setPoseTool(transl(0, 0, 150)); robot.setTool(tool);
  robot.setJoints([0, -90, 90, -90, -90, 0]);
  const frame = st.addChild(new Frame('Work')); frame.setPose(transl(400, -200, 0));
  return { st, robot, tool, frame };
}
function teach(robot: Robot, frame: Frame, name: string, pose = mul(transl(0, 0, 300), rotx(180 * DEG))) {
  const t = frame.addChild(new Target(name));
  t.setPose(pose);
  const r = robot.solveIK(multiply(invert(robot.poseAbs()), t.poseAbs()), { seed: robot.joints() });
  if (r.ok) t.setJoints(r.joints);
  return t;
}
/** Closest approach of the TCP path (sampled every 5 ms) to a point. */
function closest(sim: ProgramSimulator, robot: Robot, p: [number, number, number]) {
  let m = Infinity;
  for (const s of sim.steps) {
    const tr = s.trajectory;
    if (!tr || !tr.samples.length) continue;
    for (let t = 0; t <= tr.duration + 1e-9; t += 0.005) m = Math.min(m, distance(getPos(multiply(robot.poseAbs(), sampleAt(tr, t).pose)), p));
  }
  return m;
}

describe('motion semantics (RoboDK parity)', () => {
  it('S1: a rounding zone cuts the corner by up to the radius and shortens the cycle; fine points stop at the target', () => {
    const run = (radius: number) => {
      const { st, robot, frame } = ur5Cell();
      const a = teach(robot, frame, 'A'), b = teach(robot, frame, 'B', mul(transl(200, 0, 300), rotx(180 * DEG))), c = teach(robot, frame, 'C', mul(transl(200, 200, 300), rotx(180 * DEG)));
      const p = st.addChild(new Program('P')); p.setRobot(robot); p.setSpeed(200); p.setRounding(radius);
      p.addMoveJ(a); p.addMoveL(b); p.addMoveL(c);
      const sim = new ProgramSimulator(st);
      const res = sim.compile(p);
      expect(res.ok, JSON.stringify(res.problems)).toBe(true);
      const corner = closest(sim, robot, getPos(b.poseAbs()));
      sim.runToEnd();
      return { duration: res.duration, corner, end: tcpErr(robot, c).pos };
    };
    const fine = run(-1), r50 = run(50);
    expect(fine.corner).toBeLessThan(0.05);
    // a 90 deg corner blended with r = 50 mm: the quadratic zone passes 50/(2*sqrt 2) = 17.7 mm from the corner
    expect(r50.corner).toBeGreaterThan(10);
    expect(r50.corner).toBeLessThan(50);
    expect(r50.duration).toBeLessThan(fine.duration - 0.1);
    expect(fine.end).toBeLessThan(0.05); expect(r50.end).toBeLessThan(0.05); // the last point is a fine point
  });

  it('S14: 200 x 1 mm MoveL at 100 mm/s with rounding keeps the path speed (about 2 s, not stop-and-go)', () => {
    const { st, robot, frame } = ur5Cell();
    const p = st.addChild(new Program('P')); p.setRobot(robot);
    p.addMoveJ(teach(robot, frame, 'S'));
    p.setSpeed(100, undefined, 1000); p.setRounding(1);
    for (let i = 1; i <= 200; i++) p.addMoveL(teach(robot, frame, `P${i}`, mul(transl(i, 0.3 * Math.sin(i / 10), 300), rotx(180 * DEG))));
    const sim = new ProgramSimulator(st);
    const res = sim.compile(p);
    expect(res.ok, JSON.stringify(res.problems.slice(0, 3))).toBe(true);
    const moveL = sim.steps.filter((s) => s.trajectory && (s.instruction.data as any).moveType === 'MoveL');
    const t = moveL[moveL.length - 1].t1 - moveL[0].t0;
    expect(t).toBeGreaterThan(2.0); // >= 200 mm / 100 mm/s
    expect(t).toBeLessThan(2.5);
    // the same path with fine points stops at every point
    for (const ins of p.instructions()) if (ins.data.kind === 'rounding') ins.data.radius = -1;
    const fine = new ProgramSimulator(st).compile(p);
    expect(fine.duration).toBeGreaterThan(res.duration + 5);
  });

  it('S2/S3: MoveJ to a Cartesian target solves IK on the pose with the active tool (stored joints only pick the configuration)', () => {
    const { st, robot, frame } = ur5Cell();
    const t = teach(robot, frame, 'T');
    frame.setPose(transl(500, -100, 0)); // fixture moved by 100 mm in X and Y after teaching
    const t300 = robot.addChild(new Tool('T300')); t300.setPoseTool(transl(0, 0, 300));
    const p = st.addChild(new Program('P')); p.setRobot(robot); p.addMoveJ(t); p.setTool(t300); p.addMoveJ(t);
    const sim = new ProgramSimulator(st);
    const res = sim.compile(p);
    expect(res.ok, JSON.stringify(res.problems)).toBe(true);
    const first = sim.steps.find((s) => s.trajectory)!;
    const qFirst = sampleAt(first.trajectory!, first.trajectory!.duration).joints;
    robot.setJoints(qFirst);
    expect(tcpErr(robot, t, robot.activeTool()!).pos).toBeLessThan(0.05);
    // same configuration as the stored joints (wrist/elbow flags unchanged)
    expect(robot.configFlags(qFirst)).toEqual(robot.configFlags(t.joints!));
    sim.runToEnd();
    expect(robot.activeTool()).toBe(t300);
    const e = tcpErr(robot, t, t300);
    expect(e.pos).toBeLessThan(0.05);
    expect(e.rotDeg).toBeLessThan(0.01);
  });

  it('RoboDK reference frames: after Set Ref. a target is its pose in its own frame applied to the active frame (01.c duplicate part)', () => {
    const { st, robot, frame } = ur5Cell();
    const part2 = st.addChild(new Frame('Part 2')); part2.setPose(transl(400, 100, 0)); // second fixture, 300 mm further in Y
    const t = teach(robot, frame, 'Drill', mul(transl(50, 0, 300), rotx(180 * DEG)));
    const drill = st.addChild(new Program('Drill')); drill.setRobot(robot); drill.addMoveJ(t);
    const p1 = st.addChild(new Program('P1')); p1.setRobot(robot); p1.setFrame(frame); p1.callProgram(drill);
    const p2 = st.addChild(new Program('P2')); p2.setRobot(robot); p2.setFrame(part2); p2.callProgram(drill);
    const tcp = () => getPos(multiply(robot.poseAbs(), robot.solveFK(robot.joints(), robot.activeTool()!.poseTool())));
    const s1 = new ProgramSimulator(st); expect(s1.compile(p1).ok).toBe(true); s1.runToEnd();
    expect(distance(tcp(), getPos(t.poseAbs()))).toBeLessThan(0.05);
    const s2 = new ProgramSimulator(st); expect(s2.compile(p2).ok).toBe(true); s2.runToEnd();
    expect(distance(tcp(), getPos(multiply(part2.poseAbs(), t.pose())))).toBeLessThan(0.05); // drilled on part 2
    // a program that never sets a reference keeps its targets where they are, whatever frame the UI made active
    robot.setFrame(part2);
    robot.setJoints([0, -90, 90, -90, -90, 0]);
    const s3 = new ProgramSimulator(st); expect(s3.compile(drill).ok).toBe(true); s3.runToEnd();
    expect(distance(tcp(), getPos(t.poseAbs()))).toBeLessThan(0.05);
  });

  it('a MoveL that starts exactly at a wrist singularity (J5 = 0) is rejected like RoboDK (01.d RetractInspectA); a MoveJ first is fine', () => {
    const st = new Station('ws');
    const robot = st.addChild(createRobotFromLibrary('KUKA_KR6_R900', 'KR6'));
    const tool = robot.addChild(new Tool('T')); tool.setPoseTool(transl(0, 0, 100)); robot.setTool(tool);
    robot.setJoints([0, -90, 90, 0, 0, 0]);
    const tcp0 = multiply(robot.poseAbs(), robot.solveFK(robot.joints(), tool.poseTool()));
    const t = st.addChild(new Target('Side')); t.setPose(multiply(transl(0, 100, -50), tcp0));
    const p = st.addChild(new Program('P')); p.setRobot(robot); p.addMoveL(t);
    const res = new ProgramSimulator(st).compile(p);
    expect(res.ok).toBe(false);
    expect(res.problems.find((x) => x.severity === 'error')!.message).toMatch(/starts at a wrist singularity/);
    const q1 = st.addChild(new Target('Bent')); q1.setJoints([0, -45, 90, 0, 45, 0]); q1.setAsJointTarget();
    const t2 = st.addChild(new Target('Side2')); t2.setPose(multiply(transl(0, 0, 60), multiply(robot.poseAbs(), robot.solveFK(q1.joints!, tool.poseTool()))));
    const p2 = st.addChild(new Program('P2')); p2.setRobot(robot); p2.addMoveJ(q1); p2.addMoveL(t2);
    expect(new ProgramSimulator(st).compile(p2).problems.filter((x) => x.severity === 'error')).toEqual([]);
  });

  it('S11: MoveJ honours per-joint maximum velocities even when the program speed is higher', () => {
    const { st, robot } = ur5Cell();
    robot.chain.joints[0].maxVelocity = 180;
    const p = st.addChild(new Program('P')); p.setRobot(robot); p.setSpeed(undefined, 1000, undefined, 10000); p.addMoveJ([180, -90, 90, -90, -90, 0]);
    const res = new ProgramSimulator(st).compile(p);
    expect(res.ok).toBe(true);
    expect(res.duration).toBeGreaterThanOrEqual(1.0); // 180 deg at 180 deg/s
    expect(res.duration).toBeLessThan(1.2);
  });

  it('S5: RunInstruction(name, call) of a station program executes it (RoboDK INSTRUCTION_CALL_PROGRAM)', () => {
    const { st, robot, frame } = ur5Cell();
    const a = teach(robot, frame, 'A'), b = teach(robot, frame, 'B', mul(transl(200, 0, 300), rotx(180 * DEG)));
    const sub = st.addChild(new Program('Sub')); sub.setRobot(robot); sub.addMoveL(b);
    const main = st.addChild(new Program('Main')); main.setRobot(robot); main.addMoveJ(a); main.runInstruction('Sub', true);
    const sim = new ProgramSimulator(st);
    const res = sim.compile(main);
    expect(res.ok, JSON.stringify(res.problems)).toBe(true);
    sim.runToEnd();
    expect(tcpErr(robot, b).pos).toBeLessThan(0.05);
  });

  it('S13: MoveL to a joint target ends at the stored joints (its Cartesian pose is ignored)', () => {
    const { st, robot, frame } = ur5Cell();
    const a = teach(robot, frame, 'A');
    robot.setJoints(a.joints!);
    const t = teach(robot, frame, 'J', mul(transl(100, 0, 300), rotx(180 * DEG)));
    robot.setJoints([0, -90, 90, -90, -90, 0]);
    t.setAsJointTarget();
    t.setPose(mul(transl(50, 0, 300), rotx(180 * DEG))); // stale pose
    const p = st.addChild(new Program('P')); p.setRobot(robot); p.addMoveJ(a); p.addMoveL(t);
    const sim = new ProgramSimulator(st);
    expect(sim.compile(p).ok).toBe(true);
    sim.runToEnd();
    robot.joints().forEach((v, i) => expect(v).toBeCloseTo(t.joints![i], 3));
  });

  it('S6: a target with 6+1 joints on a synchronised turntable moves the turntable with the arm (no clamp warning)', () => {
    const { st, robot } = ur5Cell();
    const table = st.addChild(buildMechanism({ type: MAKE_ROBOT_1R, parameters: [100], name: 'Turntable', lower: [-360], upper: [360] }));
    table.setPose(transl(450, 0, 0));
    const tf = table.addChild(new Frame('Table frame')); tf.setPose(transl(0, 0, 100));
    robot.externalAxes = [table.id];
    table.setJoints([90]);
    const t = tf.addChild(new Target('OnTable')); t.setPose(mul(transl(100, 0, 200), rotx(180 * DEG)));
    const r = robot.solveIK(multiply(invert(robot.poseAbs()), t.poseAbs()), { seed: robot.joints() });
    expect(r.ok).toBe(true);
    t.setJoints([...r.joints, 90]);
    const t2 = tf.addChild(new Target('OnTable2')); t2.setPose(mul(transl(100, 80, 200), rotx(180 * DEG)));
    const r2 = robot.solveIK(multiply(invert(robot.poseAbs()), t2.poseAbs()), { seed: r.joints });
    t2.setJoints([...r2.joints, 90]);
    table.setJoints([0]);
    const p = st.addChild(new Program('P')); p.setRobot(robot); p.addMoveJ(t); p.addMoveL(t2);
    const sim = new ProgramSimulator(st);
    const res = sim.compile(p);
    expect(res.problems).toEqual([]);
    // halfway through the MoveJ the turntable is between 0 and 90 deg (synchronised)
    const mj = sim.steps.find((s) => s.trajectory)!;
    sim.seek((mj.t0 + mj.t1) / 2);
    expect(table.joints()[0]).toBeGreaterThan(10); expect(table.joints()[0]).toBeLessThan(80);
    sim.runToEnd();
    expect(table.joints()[0]).toBeCloseTo(90, 6);
    expect(tcpErr(robot, t2).pos).toBeLessThan(0.05);
  });

  it('S10: a MoveL 2.5 m along a synchronised rail moves the rail (the arm alone cannot reach)', () => {
    const st = new Station('rail');
    const rail = st.addChild(buildMechanism({ type: MAKE_ROBOT_1T, parameters: [3000], name: 'Rail', lower: [0], upper: [3000] }));
    const robot = rail.addChild(createRobotFromLibrary('KUKA_KR6_R900', 'KR6'));
    robot.setJoints([0, -90, 90, 0, 45, 0]);
    robot.externalAxes = [rail.id];
    const tool = robot.addChild(new Tool('T')); tool.setPoseTool(transl(0, 0, 100)); robot.setTool(tool);
    const start = st.addChild(new Target('Start')); start.setPose(multiply(rail.poseAbs(), multiply(robot.poseAbs(), robot.solveFK(robot.joints(), tool.poseTool()))));
    const t = st.addChild(new Target('Far')); t.setPose(multiply(transl(2500, 0, 0), start.poseAbs()));
    const p = st.addChild(new Program('P')); p.setRobot(robot); p.addMoveL(t);
    const sim = new ProgramSimulator(st);
    const res = sim.compile(p);
    expect(res.problems.filter((x) => x.severity === 'error')).toEqual([]);
    sim.runToEnd();
    expect(rail.joints()[0]).toBeGreaterThan(1500);
    expect(tcpErr(robot, t).pos).toBeLessThan(0.1);
    // the TCP follows the straight line (rail + arm interpolated together)
    expect(res.distance).toBeCloseTo(2500, -1);
  });

  it('S8: attach to a named tool on a gripper mechanism keeps the grasp offset (no jump)', () => {
    const st = new Station('grip');
    const arm = st.addChild(createRobotFromLibrary('UR5e', 'Arm'));
    arm.setJoints([0, -90, 90, -90, -90, 0]);
    const grip = arm.addChild(buildMechanism({ type: MAKE_ROBOT_1T, parameters: [50], name: 'Gripper mech' }));
    grip.setPose(transl(0, 0, 50));
    const gtool = grip.addChild(new Tool('Gripper TCP')); gtool.setPoseTool(transl(0, 0, 100));
    const armTool = arm.addChild(new Tool('Flange TCP')); armTool.setPoseTool(transl(0, 0, 150)); arm.setTool(armTool);
    const part = st.addChild(new SceneObject('Part'));
    part.geometry = [{ primitive: { kind: 'box', size: [40, 40, 40] } }];
    part.setPoseAbs(multiply(arm.poseAbs(), arm.solveFK(arm.joints(), transl(0, 0, 150))));
    const up = st.addChild(new Target('Up')); up.setJoints([0, -80, 80, -90, -90, 0]); up.setAsJointTarget();
    const p = st.addChild(new Program('P')); p.setRobot(arm); p.event('attach', part.id, 'Gripper TCP'); p.addMoveJ(up);
    const before = getPos(part.poseAbs());
    const rel0 = multiply(invert(gtool.poseAbs()), part.poseAbs());
    const sim = new ProgramSimulator(st);
    sim.compile(p);
    sim.seek(1e-6);
    expect(distance(getPos(part.poseAbs()), before)).toBeLessThan(1e-6);
    sim.runToEnd();
    expect(part.parent).toBe(gtool);
    expect(distance(getPos(part.poseAbs()), getPos(multiply(gtool.poseAbs(), rel0)))).toBeLessThan(1e-6);
    expect(distance(getPos(part.poseAbs()), before)).toBeGreaterThan(20);
  });

  it('RoboDK Item.Update() estimate mode ignores Set speed instructions; playback honours them', () => {
    const { st, robot, frame } = ur5Cell();
    robot.motion = { ...robot.motion, speedLinear: 400, accelLinear: 4000 };
    const p = st.addChild(new Program('P')); p.setRobot(robot);
    p.addMoveJ(teach(robot, frame, 'A')); p.setSpeed(20); p.addMoveL(teach(robot, frame, 'B', mul(transl(200, 0, 300), rotx(180 * DEG))));
    const play = new ProgramSimulator(st).compile(p);
    const est = new ProgramSimulator(st); est.speedInstructions = 'ignore';
    const upd = est.compile(p);
    const mj = play.duration - 200 / 20; // MoveJ part (same in both)
    expect(play.duration).toBeGreaterThan(10); // 200 mm at 20 mm/s
    expect(upd.duration - mj).toBeGreaterThan(200 / 400); // at the robot default 400 mm/s
    expect(upd.duration - mj).toBeLessThan(200 / 400 + 0.2);
  });

  it('inline 6+N targets (machining moves) drive the synchronised turntable to their stored external values', () => {
    const { st, robot } = ur5Cell();
    const table = st.addChild(buildMechanism({ type: MAKE_ROBOT_1R, parameters: [100], name: 'Turntable', lower: [-360], upper: [360] }));
    table.setPose(transl(450, 0, 0));
    const tf = table.addChild(new Frame('Table frame')); tf.setPose(transl(0, 0, 100));
    robot.externalAxes = [table.id];
    const p = st.addChild(new Program('P')); p.setRobot(robot); p.setFrame(tf);
    const poses: Array<[number, number]> = [[0, 0], [20, 30], [40, 60]];
    for (const [x, ang] of poses) {
      table.setJoints([ang]);
      const rel = mul(transl(100 + x, 0, 200), rotx(180 * DEG));
      const r = robot.solveIK(multiply(invert(robot.poseAbs()), multiply(tf.poseAbs(), rel)), { seed: robot.joints() });
      expect(r.ok).toBe(true);
      robot.setJoints(r.joints);
      p.addInstruction({ kind: 'move', moveType: x === 0 ? 'MoveJ' : 'MoveL', targetId: null, pose: Array.from(rel), joints: [...r.joints, ang] });
    }
    table.setJoints([0]); robot.setJoints([0, -90, 90, -90, -90, 0]);
    const sim = new ProgramSimulator(st);
    const res = sim.compile(p);
    expect(res.problems.filter((x) => x.severity === 'error')).toEqual([]);
    sim.runToEnd();
    expect(table.joints()[0]).toBeCloseTo(60, 6);
    const tcp = multiply(robot.poseAbs(), robot.solveFK(robot.joints(), robot.activeTool()!.poseTool()));
    expect(distance(getPos(tcp), getPos(multiply(tf.poseAbs(), mul(transl(140, 0, 200), rotx(180 * DEG)))))).toBeLessThan(0.05);
  });

  it('S9: curve follow / machining generated with tool-axis freedom store the reached poses and simulate without errors', () => {
    const mk = () => {
      const st = new Station('cf');
      const robot = st.addChild(createRobotFromLibrary('KUKA_KR6_R900', 'KR6'));
      const tool = robot.addChild(new Tool('Spindle')); tool.setPoseTool(transl(0, 0, 120)); robot.setTool(tool);
      robot.setJoints([0, -90, 90, 0, 45, 0]);
      const part = st.addChild(new SceneObject('Disk')); part.setPose(transl(550, 0, 200));
      return { st, robot, part };
    };
    // two full turns of a circle: the path direction (X hint) turns 720 deg about tool Z
    const pts: number[][] = [];
    for (let i = 0; i <= 96; i++) { const a = (i / 48) * 2 * Math.PI; pts.push([100 * Math.cos(a), 100 * Math.sin(a), 0]); }
    const c1 = mk();
    const gen = generateCurveFollow(c1.st, c1.robot, c1.part, { points: pts }, { zMode: 'down', freeToolZ: true, approach: 30 });
    expect(gen.points).toBeGreaterThan(80); // 89 of 97 points reachable with this disk position
    const sim = new ProgramSimulator(c1.st);
    const res = sim.compile(gen.program);
    expect(res.problems.filter((x) => x.severity === 'error')).toEqual([]);
    // every generated target is consistent: FK(stored joints, tool) == stored pose
    for (const t of c1.st.itemsOfType<Target>(ItemType.TARGET)) if (t.joints && !t.isJointTarget) expect(distance(getPos(multiply(c1.robot.poseAbs(), c1.robot.solveFK(t.joints, c1.robot.activeTool()!.poseTool()))), getPos(t.poseAbs()))).toBeLessThan(0.05);
    const c2 = mk();
    c2.part.curves = [{ name: 'spiral', points: pts, kind: 'cut', feed: 600 } as any];
    const m = generateMachining(c2.st, c2.robot, c2.part, { freeToolZ: true });
    expect(m.points).toBeGreaterThan(80);
    const res2 = new ProgramSimulator(c2.st).compile(m.program);
    expect(res2.problems.filter((x) => x.severity === 'error')).toEqual([]);
  });

  it('live runtime and compiled timeline agree on cycle time and final joints', () => {
    const { st, robot, frame } = ur5Cell();
    const p = st.addChild(new Program('P')); p.setRobot(robot); p.setSpeed(300); p.setRounding(20);
    p.addMoveJ(teach(robot, frame, 'A'));
    for (const [x, y] of [[200, 0], [200, 200], [0, 200]]) p.addMoveL(teach(robot, frame, `${x},${y}`, mul(transl(x, y, 300), rotx(180 * DEG))));
    const sim = new ProgramSimulator(st);
    const res = sim.compile(p);
    expect(res.ok).toBe(true);
    const rt = new StationRuntime(st);
    const h = rt.start(p);
    rt.runFor(res.duration - 0.05);
    expect(h.status).toBe('running');
    rt.runFor(0.1);
    expect(h.status).toBe('done');
    expect(h.endTime! - h.startTime!).toBeCloseTo(res.duration, 9);
    sim.runToEnd();
    const qSim = robot.joints();
    rt.reset();
    rt.start(p); rt.runFor(res.duration + 1);
    robot.joints().forEach((v, i) => expect(v).toBeCloseTo(qSim[i], 6));
  });
});
