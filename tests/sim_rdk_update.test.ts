// R1 (rdk-sim): RoboDK simulation parity rules measured on RoboDK 5.9.4 itself (scratch probes rdk_speed*.py,
// rdk_movej.py, rdk_tt.py; ground truth tests/fixtures/robodk_gt/sim, dump_sim.py):
// - Item.Update() cycle time (ProgramRunResult.updateTime): moves rest to rest at the ROBOT's motion state, program
//   speed / rounding instructions ignored, robot rounding > 0 = no acceleration, calls into other robots' programs
//   start from those robots' joints at the start;
// - MoveJ to a Cartesian target with stored joints outside the limits keeps the current configuration;
// - linear moves passing within 2.5 deg of a spherical-wrist singularity fail;
// - Comau J3 limits apply to q3 - q2; imported robots use the active reference frame from the start;
// - a run holds the robots it moves, not the robot its program is linked to (threads of 06.e).
import { describe, it, expect } from 'vitest';
import { existsSync, readFileSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { Station, Frame, Target, Tool, ItemType } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import { createRobotFromLibrary } from '../src/core/items/library';
import { ProgramSimulator, estimateProgram } from '../src/core/motion/simulator';
import { restToRest, updateModelOf } from '../src/core/motion/planner';
import { multiply, invert, transl, mul, rotx, DEG } from '../src/core/math/pose';
import { importRdkBestEffort } from '../src/io/robodk/rdk_container';
import { AssetStore } from '../src/scene/assets';

/** UR5e cell with the motion state of the Library Fanuc M-10iA (v 1000 mm/s, a 1500 mm/s², vj 200 deg/s, aj 150 deg/s²). */
function cell(rounding = -1) {
  const st = new Station('update');
  const robot = st.addChild(createRobotFromLibrary('UR5e', 'UR5e'));
  robot.setMotionDefaults({ speedLinear: 1000, accelLinear: 1500, speedJoints: 200, accelJoints: 150, rounding });
  const tool = robot.addChild(new Tool('T100')); tool.setPoseTool(transl(0, 0, 100)); robot.setTool(tool);
  robot.setJoints([0, -90, 90, -90, -90, 0]);
  const frame = st.addChild(new Frame('Work')); frame.setPose(transl(400, -200, 0));
  return { st, robot, frame };
}
function teach(robot: Robot, frame: Frame, name: string, pose = mul(transl(0, 0, 300), rotx(180 * DEG))) {
  const t = frame.addChild(new Target(name));
  t.setPose(pose);
  const r = robot.solveIK(multiply(invert(robot.poseAbs()), t.poseAbs()), { seed: robot.joints() });
  if (r.ok) t.setJoints(r.joints);
  return t;
}

describe('RoboDK Item.Update() cycle-time model (updateTime)', () => {
  it('rest-to-rest trapezoids at the robot motion state (values measured on RoboDK 5.9.4)', () => {
    // RoboDK, Fanuc M-10iA (v 1000, a 1500): MoveL 100 mm 0.5164 s, 400 mm 1.0328 s, 10 x 10 mm 1.6330 s (no blending)
    expect(restToRest(100, 1000, 1500)).toBeCloseTo(0.5164, 4);
    expect(restToRest(400, 1000, 1500)).toBeCloseTo(1.0328, 4);
    expect(restToRest(1200, 1000, 1500)).toBeCloseTo(1.8667, 4); // saturates: d/v + v/a
    expect(restToRest(90, 200, 2000)).toBeCloseTo(0.55, 4); // MoveJ 90 deg with aj 2000
    expect(restToRest(100, 1000, 1500, true)).toBeCloseTo(0.1, 6); // robot rounding > 0: length / speed
    const { st, robot, frame } = cell();
    const a = teach(robot, frame, 'A'), b = teach(robot, frame, 'B', mul(transl(100, 0, 300), rotx(180 * DEG)));
    const plain = st.addChild(new Program('Plain')); plain.setRobot(robot);
    plain.addMoveJ(a.joints!); plain.addMoveL(b);
    // the same moves with speed / rounding instructions: Item.Update() ignores them, the playback honours the speed
    const slow = st.addChild(new Program('Slow')); slow.setRobot(robot);
    slow.setSpeed(5); slow.setRounding(20); slow.addMoveJ(a.joints!); slow.addMoveL(b); slow.pause(1000);
    robot.setJoints(a.joints!);
    const r1 = new ProgramSimulator(st).compile(plain);
    const r2 = new ProgramSimulator(st).compile(slow);
    expect(r1.ok && r2.ok, JSON.stringify([r1.problems, r2.problems])).toBe(true);
    expect(r1.updateTime!).toBeCloseTo(2 * Math.sqrt(100 / 1500), 3);
    expect(r2.updateTime!).toBeCloseTo(2 * Math.sqrt(100 / 1500) + 1, 3); // + the 1 s pause
    expect(r2.duration).toBeGreaterThan(19); // 100 mm at the programmed 5 mm/s
    expect(estimateProgram(st, slow, { roboDKUpdate: true }).duration).toBeCloseTo(r2.updateTime!, 6);
  });

  it('joint moves: largest joint travel at vj / aj; robot rounding > 0 removes the acceleration phases', () => {
    for (const rounding of [-1, 5]) {
      const { st, robot } = cell(rounding);
      const q0 = robot.joints();
      const p = st.addChild(new Program('J')); p.setRobot(robot);
      p.addMoveJ([q0[0] + 45, q0[1] + 10, ...q0.slice(2)]);
      const r = new ProgramSimulator(st).compile(p);
      expect(r.ok).toBe(true);
      // RoboDK: J1 +45 deg -> 1.0954 s (2 sqrt(45 / 150)); with robot rounding 5 -> 0.225 s (45 / 200)
      expect(r.updateTime!).toBeCloseTo(rounding > 0 ? 45 / 200 : 2 * Math.sqrt(45 / 150), 4);
    }
  });

  it('a robot table without joint speed / acceleration moves the joints at 2 v and 2 a (RoboDK 05.c / 03.h)', () => {
    const r = createRobotFromLibrary('UR5e', 'UR5e');
    r.motion = { speedLinear: 16.667, accelLinear: 20000, speedJoints: 0, accelJoints: 0, rounding: -1 };
    const m = updateModelOf(r);
    expect(m.vj).toBeCloseTo(33.334, 3);
    expect(m.aj).toBe(40000);
    // RoboDK IRB 6640 of 05.c: J1 +5 deg -> 0.1508 s
    expect(restToRest(5, m.vj, m.aj)).toBeCloseTo(0.1508, 4);
  });
});

describe('runtime: a run holds the robots it moves (RoboDK 06.e threads)', () => {
  it('a thread of a program linked to robot A starts at once when the caller only moves robot B', () => {
    const st = new Station('hold');
    const ra = st.addChild(createRobotFromLibrary('UR5e', 'A')); ra.setPose(transl(0, 0, 0));
    const rb = st.addChild(createRobotFromLibrary('UR5e', 'B')); rb.setPose(transl(2000, 0, 0));
    for (const r of [ra, rb]) { r.setJoints([0, -90, 90, -90, -90, 0]); r.setMotionDefaults({ speedJoints: 30, accelJoints: 100 }); }
    const feed = st.addChild(new Program('Feed')); feed.setRobot(ra); feed.addMoveJ([40, -90, 90, -90, -90, 0]);
    const take = st.addChild(new Program('Take')); take.setRobot(rb); take.addMoveJ([-40, -90, 90, -90, -90, 0]);
    // linked to A, but A is only moved by its thread (StartSimulation of 06.e)
    const main = st.addChild(new Program('Start')); main.setRobot(ra); main.startThread(feed); main.callProgram(take); main.pause(500);
    const sim = new ProgramSimulator(st);
    const res = sim.compile(main);
    expect(res.ok, JSON.stringify(res.problems)).toBe(true);
    const fs = sim.steps.find((s) => s.trajectory && s.robot === ra)!, ts = sim.steps.find((s) => s.trajectory && s.robot === rb)!;
    expect(fs.t0).toBeLessThan(1e-9); // not queued behind Start (which would make it start after ~1.8 s)
    expect(ts.t0).toBeLessThan(1e-9);
    expect(fs.t1).toBeGreaterThan(1); // both robots really move at the same time
    expect(ts.t1).toBeGreaterThan(1);
  });
  it('runtime.events "tick" is emitted on every advance with the simulation time (per-tick hook of scripted fleets)', async () => {
    const { StationRuntime } = await import('../src/core/motion/runtime');
    const st = new Station('tick');
    const ra = st.addChild(createRobotFromLibrary('UR5e', 'A'));
    ra.setJoints([0, -90, 90, -90, -90, 0]); ra.setMotionDefaults({ speedJoints: 30, accelJoints: 100 });
    const p = st.addChild(new Program('P')); p.setRobot(ra); p.addMoveJ([30, -90, 90, -90, -90, 0]);
    const rt = new StationRuntime(st);
    const ts: number[] = [];
    let q1AtTick: number[] = [];
    rt.events.on('tick', ({ t }) => { ts.push(t); q1AtTick.push(ra.joints()[0]); });
    rt.start(p);
    rt.runFor(2, 0.1);
    expect(ts.length).toBe(20);
    for (let i = 1; i < ts.length; i++) expect(ts[i]).toBeGreaterThan(ts[i - 1]);
    expect(ts[ts.length - 1]).toBeCloseTo(2, 9);
    // the hook sees the robot state of that time (the arm moves during the run)
    expect(q1AtTick[0]).toBeLessThan(q1AtTick[q1AtTick.length - 1]);
    q1AtTick = [];
  });
  it('a second program moving the same robot still waits for the first', () => {
    const st = new Station('hold2');
    const ra = st.addChild(createRobotFromLibrary('UR5e', 'A'));
    ra.setJoints([0, -90, 90, -90, -90, 0]); ra.setMotionDefaults({ speedJoints: 30, accelJoints: 100 });
    const p1 = st.addChild(new Program('P1')); p1.setRobot(ra); p1.addMoveJ([40, -90, 90, -90, -90, 0]);
    const p2 = st.addChild(new Program('P2')); p2.setRobot(ra); p2.addMoveJ([0, -90, 90, -90, -90, 0]);
    const main = st.addChild(new Program('Main')); main.startThread(p1); main.startThread(p2);
    const sim = new ProgramSimulator(st);
    expect(sim.compile(main).ok).toBe(true);
    const [s1, s2] = sim.steps.filter((s) => s.trajectory && s.trajectory.samples.length);
    expect(s2.t0).toBeGreaterThanOrEqual(s1.t1 - 1e-9);
  });
});

// ---- Library stations vs RoboDK (needs C:/RoboDK/Library) -------------------------------------------------------------
const LIB = 'C:/RoboDK/Library';
const simGt = (f: string) => { const p = `tests/fixtures/robodk_gt/sim/${f.replace(/ /g, '_')}_.json.gz`; return existsSync(p) ? JSON.parse(gunzipSync(readFileSync(p)).toString().replace(/\bNaN\b/g, 'null')) : null; };
async function load(f: string) {
  const b = readFileSync(`${LIB}/${f}`);
  const assets = new AssetStore();
  const { station } = await importRdkBestEffort(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength), f, assets);
  return { station, assets, gt: simGt(f) };
}
const prog = (st: Station, name: string) => st.itemsOfType<Program>(ItemType.PROGRAM).find((p) => p.name === name)!;
const firstError = (st: Station, p: Program, r: { problems: Array<{ severity: string; instructionId: string; message: string }> }) => {
  const e = r.problems.find((x) => x.severity === 'error');
  return e ? { index: p.instructions().findIndex((i) => i.id === e.instructionId), message: e.message } : null;
};
const has = (f: string) => existsSync(`${LIB}/${f}`);

describe('Library programs vs RoboDK 5.9.4 (simulation ground truth)', () => {
  const F06d = 'Example-06.d-Pick and stamp.rdk';
  it.skipIf(!has(F06d))('06.d: a call into another robot\'s program starts from that robot\'s joints (Stamp1 17.925 s like RoboDK)', async () => {
    const { station, gt } = await load(F06d);
    for (const name of ['Stamp1', 'Stamp2', 'LoadMachine', 'UnloadMachine', 'MainProgram']) {
      const want = gt.programs.find((p: any) => p.name === name).update[1];
      const r = new ProgramSimulator(station).compile(prog(station, name));
      expect(r.ok, name).toBe(true);
      expect(Math.abs(r.updateTime! - want) / want, `${name}: ${r.updateTime} vs RoboDK ${want}`).toBeLessThan(name === 'MainProgram' ? 0.1 : 0.001);
    }
  }, 120000);

  const F03a = 'Example-03.a-Liquid dispensing - Kawasaki.rdk', F03d = 'Example-03.d-Polishing Canadian Canoe - KUKA.rdk';
  it.skipIf(!has(F03a) || !has(F03d))('03.a / 03.d: linear moves through the wrist singularity fail at the instruction RoboDK stops at', async () => {
    for (const [f, name] of [[F03a, 'Oil'], [F03d, 'Polish']] as const) {
      const { station, gt } = await load(f);
      const p = prog(station, name);
      const rdkValid = gt.programs.find((x: any) => x.name === name).update[0]; // RoboDK: valid instructions before the failure
      const e = firstError(station, p, new ProgramSimulator(station).compile(p));
      expect(e, name).not.toBeNull();
      expect(e!.index, `${name}: ${e!.message}`).toBe(rdkValid);
      expect(e!.message).toMatch(/wrist singularity/);
    }
  }, 180000);

  const F03g = 'Example-03.g-Glue Dispensing Car Windshield - Fanuc.rdk';
  it.skipIf(!has(F03g))('03.g: MoveJ to a target whose stored joints are outside the limits keeps the current configuration', async () => {
    const { station, gt } = await load(F03g);
    for (const name of ['Place WindShield 3', 'Place WindShield 4']) {
      const sim = new ProgramSimulator(station);
      const r = sim.compile(prog(station, name));
      const sp = gt.programs.find((x: any) => x.name === name);
      expect(r.ok, name).toBe(true);
      expect(Math.abs(r.updateTime! - sp.update[1]) / sp.update[1]).toBeLessThan(0.001);
      const end = sim.steps.find((s) => s.trajectory)!.qTarget!;
      const row = sp.jl[0]; // first listed move end (placement at the IK RoboDK chose)
      for (let i = 0; i < 6; i++) expect(Math.abs(end[i] - row[i]), `${name} J${i + 1}`).toBeLessThan(0.05);
    }
  }, 120000);

  const F09c = 'Example-09.c-PRM Automated Collision Avoidance - Comau.rdk';
  it.skipIf(!has(F09c))('09.c: Comau J3 limits apply to q3 - q2 (T1toT2 fails at its second move like RoboDK)', async () => {
    const { station, gt } = await load(F09c);
    for (const name of ['T1toT2', 'T2toT1']) {
      const p = prog(station, name);
      const e = firstError(station, p, new ProgramSimulator(station).compile(p));
      expect(e?.index, name).toBe(gt.programs.find((x: any) => x.name === name).update[0]);
    }
  }, 120000);

  const F01c = 'Example-01.c-Robot Drilling duplicate part - KUKA.rdk';
  it.skipIf(!has(F01c))('01.c: an imported robot uses its active reference frame for Cartesian targets (DrillingPart alone fails like RoboDK)', async () => {
    const { station, gt } = await load(F01c);
    const p = prog(station, 'DrillingPart');
    const e = firstError(station, p, new ProgramSimulator(station).compile(p));
    expect(e?.index).toBe(gt.programs.find((x: any) => x.name === 'DrillingPart').update[0]);
    // with its Set Ref. (DrillProgram1 / 2) the same targets are reachable and the cycle time is RoboDK's
    for (const name of ['DrillProgram1', 'DrillProgram2']) {
      const r = new ProgramSimulator(station).compile(prog(station, name));
      expect(r.ok).toBe(true);
      expect(r.updateTime!).toBeCloseTo(gt.programs.find((x: any) => x.name === name).update[1], 2);
    }
  }, 120000);

  const F06f = 'Example-06.f-Pick and Place CNC and Dual RobotiQ Gripper-UR10.rdk';
  it.skipIf(!has(F06f))('06.f: a call to a program whose name has blanks ("Pick 1") is a program call', async () => {
    const { station } = await load(F06f);
    const main = prog(station, 'MainProgram');
    const calls = main.instructions().map((i) => i.data).filter((d: any) => d.kind === 'call').map((d: any) => d.programName);
    expect(calls).toEqual(expect.arrayContaining(['Pick 1', 'Pick 2', 'Drop 1', 'Work 1']));
    expect(main.instructions().some((i) => i.data.kind === 'code')).toBe(false);
  }, 120000);
});
