/**
 * T2 kinematics: Robot API semantics (RoboDK SolveFK / SolveIK / SolveIK_All / JointsConfig with explicit tool and
 * reference, flange-based), joint speed / acceleration limits, robot default speeds, reach and serialisation of the
 * RoboDK chain features (base offset, flange link, coupling, delta).
 */
import { describe, it, expect } from 'vitest';
import { Robot, estimateReach, serializeChain, deserializeChain } from '../src/core/items/robot';
import { Station, Tool, Frame } from '../src/core/items/item';
import { createRobotFromLibrary, ROBOT_LIBRARY } from '../src/core/items/library';
import { flangePose } from '../src/core/kinematics/chain';
import { chainFromRobodk } from '../src/core/kinematics/rdk_model';
import { analyzeChain } from '../src/core/kinematics/ik_analytic';
import { mul, transl, rotx, roty, rotz, DEG, multiply, invert, Mat4 } from '../src/core/math/pose';
import { haveLib, libraryChains, posDiff, rotDiff } from './kin_gt_util';

const close = (a: number[], b: number[], tol: number) => a.every((v, i) => Math.abs(v - b[i]) < tol);

function cell() {
  const st = new Station('api');
  const r = st.addChild(createRobotFromLibrary('KUKA_KR6_R900', 'R'));
  const tool = r.addChild(new Tool('Spindle'));
  tool.setPoseTool(mul(transl(0, 0, 200), roty(30 * DEG)));
  r.setTool(tool);
  const frame = st.addChild(new Frame('Ref'));
  frame.setPose(mul(transl(500, -300, 100), rotz(40 * DEG)));
  r.setFrame(frame);
  return { st, r, tool: tool.poseTool(), ref: frame.pose() };
}

describe('kin: Robot API (RoboDK semantics)', () => {
  it('SolveFK is the flange w.r.t. the base, independent of the active tool; tool / reference arguments apply explicitly', () => {
    const { r, tool, ref } = cell();
    const q = [10, -70, 100, 20, 40, -30];
    const flange = flangePose(r.chain, q);
    expect(posDiff(r.solveFKRef(q), flange)).toBeLessThan(1e-12); // active 200 mm tool ignored
    expect(posDiff(r.solveFKRef(q, tool), multiply(flange, tool))).toBeLessThan(1e-12);
    expect(posDiff(r.solveFKRef(q, tool, ref), multiply(invert(ref), multiply(flange, tool)))).toBeLessThan(1e-9);
    // legacy helper keeps its TCP meaning (active tool)
    expect(posDiff(r.solveFK(q), multiply(flange, tool))).toBeLessThan(1e-12);
    expect(posDiff(r.poseTCP(q), multiply(invert(ref), multiply(flange, tool)))).toBeLessThan(1e-9);
  });

  it('SolveIK(SolveFK(q, tool, ref), seed, tool, ref) returns q; with flags it returns the requested configuration', () => {
    const { r, tool, ref } = cell();
    const q = [10, -70, 100, 20, 40, -30];
    const pose = r.solveFKRef(q, tool, ref);
    const back = r.solveIKRef(pose, { tool, reference: ref, seed: q.map((v) => v + 3) });
    expect(back.ok).toBe(true);
    expect(back.method).toBe('analytic');
    expect(close(back.joints, q, 1e-6)).toBe(true);
    // flange pose without arguments
    expect(close(r.solveIKRef(r.solveFKRef(q), { seed: q }).joints, q, 1e-6)).toBe(true);
    // every configuration of SolveIK_All is reachable through the config flags
    const all = r.solveIKAllRef(pose, tool, ref);
    expect(all.some((s) => close(s, q, 1e-6))).toBe(true);
    const cfgs = new Set(all.map((s) => r.jointsConfig(s).slice(0, 3).join('')));
    expect(cfgs.size).toBeGreaterThan(1);
    for (const c of cfgs) {
      const flags = c.split('').map(Number);
      const s = r.solveIKRef(pose, { tool, reference: ref, seed: q, config: flags });
      expect(s.ok, c).toBe(true);
      expect(r.jointsConfig(s.joints).slice(0, 3)).toEqual(flags);
      expect(posDiff(r.solveFKRef(s.joints, tool, ref), pose)).toBeLessThan(1e-6);
    }
    // the TCP helpers stay consistent with the explicit form
    expect(close(r.solveIK(multiply(ref, pose), { seed: q }).joints, q, 1e-6)).toBe(true);
    expect(close(r.solveIKFrame(pose, { seed: q }).joints, q, 1e-6)).toBe(true);
  });

  it('an unreachable pose is reported as unreachable by the closed form (never as a singularity)', () => {
    const { r } = cell();
    const res = r.solveIKRef(transl(3000, 0, 500));
    expect(res.ok).toBe(false);
    expect(res.unreachable).toBe(true);
    expect(res.posError).toBeGreaterThan(1000);
  });

  it('joint speed / acceleration limits and default speeds live on the robot', () => {
    const { r } = cell();
    r.setJointMaxVelocities([156, 156, 156, 343, 362, 659]);
    r.setJointMaxAccelerations([600, 600, 600, 1200, 1200, 2400]);
    expect(r.jointMaxVelocities()).toEqual([156, 156, 156, 343, 362, 659]);
    expect(r.jointMaxAccelerations()).toEqual([600, 600, 600, 1200, 1200, 2400]);
    r.setMotionDefaults({ speedLinear: 1000, speedJoints: 120, accelLinear: 3000, accelJoints: 500, rounding: 5 });
    r.motion.speedLinear = 50;
    r.resetMotion();
    expect(r.motion).toEqual({ speedLinear: 1000, speedJoints: 120, accelLinear: 3000, accelJoints: 500, rounding: 5 });
    // round trip through serialisation
    const copy = new Robot('copy', { name: 'empty', joints: [], links: [{ name: 'base', visuals: [] }], flange: transl(0, 0, 0) });
    copy.deserializeExtra(JSON.parse(JSON.stringify((r as any).serializeExtra())), {} as any);
    expect(copy.jointMaxVelocities()).toEqual([156, 156, 156, 343, 362, 659]);
    expect(copy.jointMaxAccelerations()[5]).toBe(2400);
    expect(copy.motionDefaults.speedLinear).toBe(1000);
    expect(copy.reach).toBeCloseTo(r.reach, 6);
  });

  it('reach of the built-in spherical-wrist robots equals the catalogue value (wrist-centre radius) within 3 %', () => {
    // (UR catalogue reach uses another definition: 850 mm for a UR5e whose a2 + a3 + d5 is 917 mm)
    for (const id of ['KUKA_KR6_R900', 'KUKA_KR210_R2700', 'ABB_IRB120', 'ABB_IRB2600', 'ABB_IRB6700']) {
      const e = ROBOT_LIBRARY.find((x) => x.id === id)!;
      const reach = estimateReach(e.build());
      expect(Math.abs(reach / e.reach - 1), `${id}: ${reach.toFixed(1)} vs ${e.reach}`).toBeLessThan(0.03);
    }
  });

  it.skipIf(!haveLib)('reach of the RoboDK Library arms equals the RoboDK robot table value (2.5 %: nominal catalogue values; most agree to 0.2 %)', { timeout: 120000 }, () => {
    let n = 0;
    for (const { gt, chain, table } of libraryChains()) {
      const t = table[101];
      if (!(t > 100) || chain.parallel || (chain.dof ?? 0) !== 6 || analyzeChain(chain).kind !== 'spherical6') continue; // UR catalogue values use other definitions; offset wrists: flange radius
      // RoboDK copies catalogue values that use other definitions (to the flange: Stäubli, Doosan, Meca500, Comau NM;
      // and wrong table entries: KR 16-2 = 901, M-710iC/50 and /70 = 1360)
      if (/Staubli|Doosan|Meca500|JAKA|Comau|KR 16 2|M-710iC\/(50|70)/.test(gt.name)) continue;
      expect(Math.abs(estimateReach(chain) / t - 1), `${gt.name}: ${estimateReach(chain).toFixed(1)} vs ${t}`).toBeLessThan(0.025);
      n++;
    }
    expect(n).toBeGreaterThan(50);
  });

  it('delta reach is the platform workspace radius, not the sum of the link lengths', () => {
    const chain = chainFromRobodk({ name: 'IRB 360', robotClass: 5, jointsType: 0, rows: [[0, 0, 0, 0, 0, 692.82, 155.88, 350, 800, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]], lower: [-46, -46, -46], upper: [100, 100, 100] });
    const reach = estimateReach(chain);
    expect(reach).toBeGreaterThan(500);
    expect(reach).toBeLessThan(chain.parallel!.baseRadius + 350 + 800 - chain.parallel!.platformRadius - 100);
  });

  it('serialisation keeps base offset, flange link, coupling, joints type and the delta model', () => {
    const arm = chainFromRobodk({ name: 'Fanuc', jointsType: 2, rows: [[0, 0, 0, 450, 0], [-Math.PI / 2, 150, -Math.PI / 2, 0, 0], [0, 600, 0, 0, 0], [-Math.PI / 2, 200, 0, 640, 0], [Math.PI / 2, 0, 0, 0, 0], [-Math.PI / 2, 0, Math.PI, 100, 0]], base: [0, 0, -450, 0, 0, 0], flange: [0, 0, 0, 0, 0, Math.PI] });
    const back = deserializeChain(JSON.parse(JSON.stringify(serializeChain(arm))));
    const q = [10, 20, -30, 40, 50, 60];
    expect(back.joints[2].couple).toEqual(arm.joints[2].couple);
    expect(back.jointsType).toBe(2);
    expect(posDiff(flangePose(back, q), flangePose(arm, q))).toBeLessThan(1e-12);
    expect(rotDiff(flangePose(back, q), flangePose(arm, q))).toBeLessThan(1e-12);
    const delta = chainFromRobodk({ name: 'IRB 360', robotClass: 5, jointsType: 0, rows: [[0, 0, 0, 0, 0, 692.82, 155.88, 350, 800, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, Math.PI, 0]] });
    const d2 = deserializeChain(JSON.parse(JSON.stringify(serializeChain(delta))));
    expect(d2.parallel?.kind).toBe('delta');
    expect(posDiff(flangePose(d2, [20, 30, 40, 50]), flangePose(delta, [20, 30, 40, 50]))).toBeLessThan(1e-12);
    const grip = chainFromRobodk({ name: 'Gripper', robotClass: 15, jointsType: 0, rows: [[0, 0, 0, 0, 1]], senses: [0.5], flange: [0, 0, 120, 0, 0, 0] });
    const g2 = deserializeChain(JSON.parse(JSON.stringify(serializeChain(grip))));
    expect(g2.flangeLink).toBe(0);
    expect(posDiff(flangePose(g2, [50]), transl(0, 0, 120))).toBeLessThan(1e-12); // the flange does not move with the fingers
  });
});

export type { Mat4 };
void rotx;
