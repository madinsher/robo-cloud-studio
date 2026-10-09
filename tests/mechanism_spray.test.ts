import { describe, it, expect } from 'vitest';
import { buildMechanism, MAKE_ROBOT_6DOF, MAKE_ROBOT_1T, MAKE_ROBOT_3T, chainFromDHM, MAKE_ROBOT_2R } from '../src/core/kinematics/mechanism';
import { Robot } from '../src/core/items/robot';
import { Station, SceneObject, Tool } from '../src/core/items/item';
import { createRobotFromLibrary } from '../src/core/items/library';
import { SpraySimulator } from '../src/core/motion/spray';
import { transl, mul, rotx, DEG, getPos, distance } from '../src/core/math/pose';

describe('mechanism builder', () => {
  it('builds 6DOF, rail and gantry mechanisms with working IK', () => {
    const r6 = buildMechanism({ type: MAKE_ROBOT_6DOF, parameters: [400, 25, 455, 35, 420, 80], name: 'Built 6R' });
    expect(r6.dof).toBe(6);
    const target = r6.solveFK([10, -20, 30, 0, 40, 0]);
    expect(r6.solveIK(target, { seed: [0, 0, 0, 0, 0, 0] }).ok).toBe(true);
    const rail = buildMechanism({ type: MAKE_ROBOT_1T, parameters: [5000], name: 'Rail' });
    expect(rail.dof).toBe(1);
    rail.setJoints([1234]);
    expect(getPos(rail.solveFKFlange())[0]).toBe(1234);
    const g = buildMechanism({ type: MAKE_ROBOT_3T, parameters: [2000, 1000, 500] });
    g.setJoints([100, 200, 300]);
    expect(getPos(g.solveFKFlange())).toEqual([100, 200, 300]);
    const p = buildMechanism({ type: MAKE_ROBOT_2R, parameters: [300, 200] });
    expect(p.dof).toBe(2);
  });
  it('modified DH chain matches a standard DH robot converted by hand (PUMA-like check)', () => {
    // simple 2-link planar: standard DH a1=300, a2=200 -> DHM rows [0,0,th1,0],[0,300,th2,0] + flange a2
    const c = chainFromDHM('planar', [[0, 0, 0, 0], [0, 300, 0, 0]]);
    c.flange = transl(200, 0, 0);
    const r = new Robot('planar', c);
    r.setJoints([90, 0]);
    const p = getPos(r.solveFKFlange());
    expect(distance(p, [0, 500, 0])).toBeLessThan(1e-9);
  });
});

describe('spray simulation', () => {
  it('deposits paint on a box facing the gun and reports coverage', () => {
    const st = new Station();
    const r = st.addChild(createRobotFromLibrary('UR10e'));
    const tool = r.addChild(new Tool('Gun'));
    tool.setPoseTool(transl(0, 0, 150));
    r.setTool(tool);
    const part = st.addChild(new SceneObject('Panel'));
    part.geometry = [{ primitive: { kind: 'box', size: [400, 400, 20] }, origin: Array.from(transl(0, 0, 10)), color: '#ccc' }];
    part.setPose(transl(700, 0, 0));
    const spray = new SpraySimulator(st);
    const gun = spray.add(tool, part, { angleDeg: 20, range: 600, sampleSpacing: 25 });
    // point the gun down onto the panel from 300 mm above
    const res = r.solveIK(mul(transl(700, 0, 320), rotx(180 * DEG)));
    expect(res.ok).toBe(true);
    r.setJoints(res.joints);
    spray.setState(gun.id, true);
    for (let i = 0; i < 20; i++) spray.step(0.1);
    const s = spray.stats();
    expect(s.samples).toBeGreaterThan(100);
    expect(s.coverage).toBeGreaterThan(5);
    expect(s.coverage).toBeLessThan(60);
    expect(s.max).toBeGreaterThan(0);
    spray.clear();
    expect(spray.stats().coverage).toBe(0);
  });
});
