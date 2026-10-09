import { describe, it, expect } from 'vitest';
import { createRobotFromLibrary, ROBOT_LIBRARY } from '../src/core/items/library';
import { getPos, distance, rotationAngle, mul, transl, rotx, roty, rotz, DEG } from '../src/core/math/pose';
import { Station, Frame, Target, Tool } from '../src/core/items/item';
import { planMoveJ, planMoveL } from '../src/core/motion/trajectory';

describe('kinematics', () => {
  it('UR5e FK at home matches known pose (approx reach)', () => {
    const r = createRobotFromLibrary('UR5e');
    const p = r.solveFK([0, 0, 0, 0, 0, 0]);
    // UR5e with all joints at zero: arm fully stretched along -X: x = a2 + a3 = -817.2, y = -(d4 + d6) = -232.9, z = d1 - d5 = 62.8
    const pos = getPos(p);
    expect(Math.abs(pos[0] - -817.2)).toBeLessThan(1e-6);
    expect(Math.abs(pos[1] - -232.9)).toBeLessThan(1e-6);
    expect(Math.abs(pos[2] - 62.8)).toBeLessThan(1e-6);
    expect(r.dof).toBe(6);
  });

  it('IK recovers a FK pose for every library robot', () => {
    for (const e of ROBOT_LIBRARY) {
      const r = createRobotFromLibrary(e.id);
      const { lower, upper } = r.jointLimits();
      // a mid-range configuration
      const q = lower.map((lo, i) => {
        const hi = upper[i];
        const c = (lo + hi) / 2;
        return c + (hi - lo) * 0.1 * ((i % 2) ? 1 : -1);
      });
      const target = r.solveFK(q);
      const res = r.solveIK(target, { seed: q.map((v, i) => v + (i === 0 ? 5 : -3)) });
      expect(res.ok, `${e.id} IK failed: pos ${res.posError} rot ${res.rotError}`).toBe(true);
      const back = r.solveFK(res.joints);
      expect(distance(getPos(back), getPos(target))).toBeLessThan(0.01);
      if (r.dof >= 6) expect(rotationAngle(back, target), e.id).toBeLessThan(1e-4);
      else if (r.dof >= 4) {
        // orientation relaxed about the tool Z axis: Z directions must still agree
        const dz = Math.hypot(back[8] - target[8], back[9] - target[9], back[10] - target[10]);
        expect(dz, e.id).toBeLessThan(1e-3);
      }
    }
  });

  it('IK from a far seed still converges (UR10e)', () => {
    const r = createRobotFromLibrary('UR10e');
    const target = mul(transl(600, 200, 400), rotx(180 * DEG));
    const res = r.solveIK(target, { seed: [0, -90, 0, -90, 90, 0] });
    expect(res.ok).toBe(true);
    expect(distance(getPos(r.solveFK(res.joints)), [600, 200, 400])).toBeLessThan(0.01);
  });

  it('tool and frame transforms compose correctly', () => {
    const st = new Station();
    const r = st.addChild(createRobotFromLibrary('KUKA_KR6_R900'));
    r.setPose(transl(1000, 0, 0));
    const tool = r.addChild(new Tool('Gripper'));
    tool.setPoseTool(transl(0, 0, 150));
    r.setTool(tool);
    const frame = st.addChild(new Frame('Table'));
    frame.setPose(mul(transl(1500, 300, 200), rotz(90 * DEG)));
    r.setFrame(frame);
    const targetInFrame = mul(transl(100, 50, 200), rotx(180 * DEG));
    const res = r.solveIKFrame(targetInFrame);
    expect(res.ok).toBe(true);
    r.setJoints(res.joints);
    const tcpAbs = r.poseTCPAbs();
    const expectedAbs = mul(frame.poseAbs(), targetInFrame);
    expect(distance(getPos(tcpAbs), getPos(expectedAbs))).toBeLessThan(0.01);
    // Target item relative to frame
    const t = frame.addChild(new Target('T1'));
    t.setPose(targetInFrame);
    const q = r.jointsForTarget(t);
    expect(q).not.toBeNull();
  });

  it('MoveJ and MoveL trajectories are generated', () => {
    const r = createRobotFromLibrary('UR5e');
    const q0 = [0, -90, 90, -90, -90, 0];
    r.setJoints(q0);
    const p0 = r.solveFK(q0);
    const p1 = mul(p0, transl(0, 0, 100));
    const l = planMoveL(r, q0, p1, 250, 1000);
    expect(l.ok, l.error).toBe(true);
    expect(l.length).toBeGreaterThan(99);
    expect(l.length).toBeLessThan(101);
    const q1 = l.samples[l.samples.length - 1].joints;
    const j = planMoveJ(r, q1, q0, 90, 360);
    expect(j.ok).toBe(true);
    expect(j.duration).toBeGreaterThan(0);
  });

  it('station serializes and deserializes with robots', () => {
    const st = new Station('Cell');
    const r = st.addChild(createRobotFromLibrary('ABB_IRB120'));
    r.setJoints([10, 20, -30, 0, 45, 0]);
    const json = JSON.stringify(st.serialize());
    const st2 = Station.deserialize(JSON.parse(json));
    const r2 = st2.children[0] as any;
    expect(r2.joints()).toEqual([10, 20, -30, 0, 45, 0]);
    expect(distance(getPos(r2.solveFK()), getPos(r.solveFK()))).toBeLessThan(1e-9);
  });
});
