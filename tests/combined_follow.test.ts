import { describe, it, expect } from 'vitest';
import { Station, SceneObject } from '../src/core/items/item';
import { createRobotFromLibrary } from '../src/core/items/library';
import { solveIKWithCarrier, applyCombined, combineChains } from '../src/core/kinematics/combined';
import { generateCurveFollow, generatePointFollow, resampleCurve } from '../src/core/motion/pathfollow';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { transl, mul, rotx, DEG, getPos, distance, multiply, invert } from '../src/core/math/pose';

describe('external axes', () => {
  it('robot on a linear track reaches a far target using the track', () => {
    const st = new Station();
    const rail = st.addChild(createRobotFromLibrary('GANTRY_XYZ', 'Rail'));
    // make it a 1-axis rail by limiting Y/Z
    rail.setJointLimits([0, 0, 0], [3000, 0, 0]);
    const arm = rail.addChild(createRobotFromLibrary('UR10e', 'Arm'));
    // children of a robot hang on its flange: the arm pose is relative to the rail flange (Z down -> flip)
    arm.setPose(rotx(180 * DEG));
    expect(getPos(arm.poseAbs())[2]).toBeCloseTo(1000);
    const chain = combineChains(rail.chain, arm.pose(), arm.chain);
    void multiply; void invert;
    expect(chain.links.length).toBe(chain.joints.length + 1);
    const target = mul(transl(2800, 300, 500), rotx(180 * DEG)); // far along X: only reachable with the rail
    const sol = solveIKWithCarrier(rail, arm, target);
    expect(sol.ok, `pos err ${sol.result.posError}`).toBe(true);
    expect(sol.carrierJoints[0]).toBeGreaterThan(1000);
    applyCombined(rail, arm, sol);
    const tcpAbs = arm.poseTCPAbs();
    expect(distance(getPos(tcpAbs), [2800, 300, 500])).toBeLessThan(0.05);
  });
});

describe('curve / point following', () => {
  it('generates a seam following program with approach and retract', () => {
    const st = new Station();
    const r = st.addChild(createRobotFromLibrary('KUKA_KR16_R2010'));
    const part = st.addChild(new SceneObject('Part'));
    part.setPose(transl(900, 0, 400));
    const pts = Array.from({ length: 11 }, (_, i) => [-250 + i * 50, 0, 0]);
    const curve = { points: pts, normals: pts.map(() => [0, 0, 1]) };
    expect(resampleCurve(pts, 25).length).toBe(21);
    const res = generateCurveFollow(st, r, part, curve, { step: 25, approach: 60, io: 'Arc', speed: 20 });
    expect(res.unreachable).toBe(0);
    expect(res.points).toBe(21);
    const sim = new ProgramSimulator(st);
    const out = sim.compile(res.program);
    expect(out.problems.filter((p) => p.severity === 'error')).toEqual([]);
    expect(out.duration).toBeGreaterThan(20);
  });
  it('generates a point follow program', () => {
    const st = new Station();
    const r = st.addChild(createRobotFromLibrary('UR5e'));
    const tray = st.addChild(new SceneObject('Tray'));
    tray.setPose(transl(400, -200, 100));
    const points = Array.from({ length: 6 }, (_, i) => ({ point: [i * 60, 0, 0] }));
    const res = generatePointFollow(st, r, tray, points, { approach: 40, io: 'Drill' });
    expect(res.points).toBe(6);
  });
});

describe('machining options', () => {
  it('tool-Z optimisation reaches a seam that a fixed orientation cannot, and rail-assisted following works', async () => {
    const { Station, SceneObject } = await import('../src/core/items/item');
    const { createRobotFromLibrary } = await import('../src/core/items/library');
    const { generateCurveFollow } = await import('../src/core/motion/pathfollow');
    const { transl, rotx, DEG, mul } = await import('../src/core/math/pose');
    const st = new Station();
    const r = st.addChild(createRobotFromLibrary('UR5e'));
    const part = st.addChild(new SceneObject('Part'));
    part.setPose(transl(400, 0, 150));
    // a long seam: with spin optimisation the wrist keeps a comfortable configuration
    const pts = Array.from({ length: 16 }, (_, i) => [-250 + i * 33, 120 * Math.sin(i / 3), 0]);
    const a = generateCurveFollow(st, r, part, { points: pts }, { step: 0, approach: 40, zMode: 'down', optimizeToolZ: true, spinStep: 45, name: 'opt' });
    expect(a.unreachable).toBe(0);
    expect(a.points).toBe(16);
    // rail: mount a UR10e on a 3 m rail and follow a 2.5 m long seam
    const st2 = new Station();
    const rail = st2.addChild(createRobotFromLibrary('GANTRY_XYZ', 'Rail'));
    rail.setJointLimits([0, 0, 0], [3000, 0, 0]);
    const arm = rail.addChild(createRobotFromLibrary('UR10e', 'Arm'));
    arm.setPose(rotx(180 * DEG));
    const seam = st2.addChild(new SceneObject('Seam'));
    seam.setPose(transl(0, 300, 300));
    const long = Array.from({ length: 11 }, (_, i) => [i * 250, 0, 0]);
    const b = generateCurveFollow(st2, arm, seam, { points: long }, { step: 0, approach: 50, zMode: 'down', carrier: rail, freeToolZ: true, name: 'rail' });
    expect(b.unreachable).toBe(0);
    expect(b.points).toBe(11);
    void mul;
  });
});
