/**
 * T2 kinematics: synchronised external axes (combined.ts) against RoboDK 5.9.4 ground truth.
 *  - Library 05.c: ABB IRB 6640 on an IRBT 6004 rail, synchronised with an IRBP L turntable (8 joints)
 *  - Library 05.d: Comau NM 45 hanging from a Gudel FP-6 gantry (9 joints)
 * RoboDK SolveFK of a synchronised robot = flange relative to the positioner flange (05.c) / the carrier base (05.d).
 */
import { describe, it, expect } from 'vitest';
import { ChainDef, flangePose, jointLimits } from '../src/core/kinematics/chain';
import { chainFromRobodk, rdkParamsFromTable } from '../src/core/kinematics/rdk_model';
import { configFlags } from '../src/core/kinematics/ik_analytic';
import { SyncedSystem, syncedFK, syncedIK, syncedIKAll, syncedPoseAbs, syncedSystemFor, robotSolveFK, solveIKWithCarrier, applySynced, syncedJoints } from '../src/core/kinematics/combined';
import { Robot } from '../src/core/items/robot';
import { Station, Frame } from '../src/core/items/item';
import { chainFromDH } from '../src/core/kinematics/chain';
import { multiply, invert, transl, rotx, mul, DEG, Mat4, identity } from '../src/core/math/pose';
import { haveLib, loadGt, robotTables, m16, posDiff, rotDiff, GtRobot } from './kin_gt_util';

interface Cell { sys: SyncedSystem; gt: GtRobot; chains: Record<string, ChainDef>; abs: (n: string) => Mat4 }

function cell(file: string, armName: string): Cell {
  const gt = loadGt(file)!;
  const tabs = robotTables(gt.file);
  const chains: Record<string, ChainDef> = {};
  for (const r of gt.robots) chains[r.name] = chainFromRobodk(rdkParamsFromTable(tabs.find((t) => t.name === r.name)!.t, r.name)!);
  const items: Array<{ name: string; poseAbs: number[] }> = (gt as any).items;
  const abs = (n: string) => m16(items.find((i) => i.name === n)!.poseAbs);
  const cur = (n: string) => gt.robots.find((r) => r.name === n)!.joints;
  const armGt = gt.robots.find((r) => r.name === armName)!;
  const mech = (n: string, role: 'carrier' | 'positioner') => {
    const base = abs(n);
    const ax = { chain: chains[n], role, base } as SyncedSystem['axes'][number];
    if (role === 'carrier') ax.mount = multiply(invert(multiply(base, flangePose(chains[n], cur(n)))), abs(armName));
    return ax;
  };
  const axes = file.includes('05.c')
    ? [mech('ABB IRBT 6004 Standard 3.5m', 'carrier'), mech('ABB IRBP L2000 L4000', 'positioner')]
    : [mech('Gudel FP-6-300', 'carrier')];
  // RoboDK keeps the synchronised limits with the arm (JointLimits of the 8/9-joint robot)
  let k = armGt.lower.length - axes.reduce((s, a) => s + (a.chain.dof ?? 0), 0);
  for (const a of axes) { const n = a.chain.dof ?? 0; a.lower = armGt.lower.slice(k, k + n); a.upper = armGt.upper.slice(k, k + n); k += n; }
  return { sys: { arm: chains[armName], armBase: abs(armName), axes }, gt: armGt, chains, abs };
}

const CELLS = [
  { file: 'Example-05.c-Robot_machining_with_a_rail_and_turntable_-_ABB.rdk_.json.gz', arm: 'ABB IRB 6640-130/3.2', nExt: 2 },
  { file: 'Example-05.d-H_bot_gantry_-_Comau.rdk_.json.gz', arm: 'Comau Smart NM 45-2.0', nExt: 3 },
];

describe('kin: synchronised external axes vs RoboDK', () => {
  for (const c of CELLS) {
    it.skipIf(!haveLib)(`${c.arm}: SolveFK over arm + external axes equals RoboDK (12 samples, < 0.01 mm)`, { timeout: 120000 }, () => {
      const { sys, gt } = cell(c.file, c.arm);
      expect(gt.fk.length).toBe(12);
      for (const s of gt.fk) {
        expect(s.q.length).toBe(6 + c.nExt);
        const ours = syncedFK(sys, s.q);
        expect(posDiff(ours, m16(s.flange!)), `q=${s.q}`).toBeLessThan(0.01);
        expect(rotDiff(ours, m16(s.flange!)) * 180 / Math.PI).toBeLessThan(0.01);
      }
      // at the current joints the arm base is where RoboDK draws it
      expect(posDiff(multiply(syncedPoseAbs(sys, gt.joints), invert(flangePose(sys.arm, gt.joints.slice(0, 6)))), sys.armBase)).toBeLessThan(1e-6);
    });

    it.skipIf(!haveLib)(`${c.arm}: IK with the external values given by the target recovers every RoboDK sample and its configuration`, { timeout: 120000 }, () => {
      const { sys, gt } = cell(c.file, c.arm);
      for (const s of gt.fk) {
        const pose = m16(s.flange!);
        const ext = s.q.slice(6);
        const want = configFlags(sys.arm, s.q.slice(0, 6))!;
        const r = syncedIK(sys, pose, { ext, config: want, seed: [...s.q.slice(0, 6).map((v) => v + 5), ...ext] });
        expect(r.ok, `q=${s.q}`).toBe(true);
        expect(r.extJoints).toEqual(ext); // honoured exactly
        expect(posDiff(syncedFK(sys, r.joints), pose)).toBeLessThan(1e-6);
        expect(configFlags(sys.arm, r.armJoints)).toEqual(want);
        // the RoboDK joints are one of the solutions listed for these external values
        const all = syncedIKAll(sys, pose, ext);
        // 6-decimal GT poses: near the wrist singularity the recovered joints differ from RoboDK by ~1e-3 deg
        expect(all.some((q) => q.every((v, i) => Math.abs(v - s.q[i]) < 0.01)), `q=${s.q} not in SolveIK_All`).toBe(true);
      }
    });

    it.skipIf(!haveLib)(`${c.arm}: free carrier axes are chosen so the arm reaches poses outside its own workspace`, { timeout: 120000 }, () => {
      const { sys, gt } = cell(c.file, c.arm);
      const lim = sys.axes.flatMap((a) => a.lower!.map((lo, i) => [lo, a.upper![i]]));
      let n = 0;
      for (const s of gt.fk) {
        const pose = m16(s.flange!);
        const ext = s.q.slice(6);
        // keep positioner values (they define where the part is), free the carrier
        const given = sys.axes.flatMap((a, k) => {
          const off = sys.axes.slice(0, k).reduce((acc, b) => acc + (b.chain.dof ?? 0), 0);
          return Array.from({ length: a.chain.dof ?? 0 }, (_, i) => (a.role === 'positioner' ? ext[off + i] : undefined));
        });
        // start from the carrier home: the arm alone cannot reach most samples from there
        const seed = [...gt.joints.slice(0, 6), ...given.map((v) => v ?? 0)];
        const r = syncedIK(sys, pose, { ext: given, seed });
        if (!s.q.slice(6).every((v, i) => v >= lim[i][0] - 1e-6 && v <= lim[i][1] + 1e-6)) continue; // GT sample outside the limits
        expect(r.ok, `q=${s.q}`).toBe(true);
        expect(r.method).toBe('analytic');
        expect(posDiff(syncedFK(sys, r.joints), pose)).toBeLessThan(1e-6);
        expect(rotDiff(syncedFK(sys, r.joints), pose)).toBeLessThan(2e-6); // the GT rotation carries 6 decimals
        r.extJoints.forEach((v, i) => { expect(v).toBeGreaterThanOrEqual(lim[i][0] - 1e-9); expect(v).toBeLessThanOrEqual(lim[i][1] + 1e-9); });
        n++;
      }
      expect(n).toBeGreaterThan(6);
    });
  }

  it.skipIf(!haveLib)('SolveIK_All of the synchronised arms at the current external values equals the RoboDK count', { timeout: 120000 }, () => {
    for (const c of CELLS) {
      const { sys, gt } = cell(c.file, c.arm);
      const pose = syncedFK(sys, gt.joints);
      const all = syncedIKAll(sys, pose, gt.joints.slice(6), { seed: gt.joints });
      expect(all.length, c.arm).toBe(gt.ikAllCount);
    }
  });
});

describe('kin: synchronised axes at item level', () => {
  /** Synthetic cell: a 6R arm on a 4 m rail along X, a turntable 1.5 m in front of the rail. */
  function station() {
    const st = new Station('sync');
    const railChain = chainFromDH('Rail', [{ theta: 0, d: 0, a: 0, alpha: 0, type: 'prismatic', lower: 0, upper: 4000 }]);
    railChain.joints[0].axis = [1, 0, 0];
    railChain.joints[0].origin = identity();
    railChain.joints[0].post = identity();
    railChain.flange = transl(0, 0, 300);
    const rail = st.addChild(new Robot('Rail', railChain));
    const arm = rail.addChild(new Robot('Arm', chainFromDH('Arm', [
      { theta: 0, d: 400, a: 150, alpha: -90 }, { theta: -90, d: 0, a: 600, alpha: 0 }, { theta: 0, d: 0, a: 120, alpha: -90 },
      { theta: 0, d: 620, a: 0, alpha: 90 }, { theta: 0, d: 0, a: 0, alpha: -90 }, { theta: 0, d: 100, a: 0, alpha: 0 },
    ])));
    arm.setPose(transl(0, 0, 300));
    const ttChain = chainFromDH('Turntable', [{ theta: 0, d: 0, a: 0, alpha: 0, lower: -360, upper: 360 }]);
    ttChain.flange = transl(0, 0, 500);
    const tt = st.addChild(new Robot('Turntable', ttChain));
    tt.setPose(transl(2000, 1500, 0));
    arm.externalAxes = [rail.id, tt.id];
    return { st, rail, arm, tt };
  }

  it('robot SolveFK with 8 joints is relative to the turntable flange; IK honours the turntable value and moves the rail', () => {
    const { arm, rail, tt } = station();
    const sys = syncedSystemFor(arm)!;
    expect(sys.axes.map((a) => a.role)).toEqual(['carrier', 'positioner']);
    const q = [10, -20, 30, 40, 50, 60, 1200, 35];
    // SolveFK = inverse(turntable flange) · arm base moved with the rail · arm flange (children of a mechanism hang on its flange)
    const armBase = multiply(multiply(rail.poseAbs(), rail.solveFKFlange([1200])), arm.pose());
    expect(posDiff(arm.poseAbs(), multiply(multiply(rail.poseAbs(), rail.solveFKFlange()), arm.pose()))).toBeLessThan(1e-12);
    const want = multiply(invert(multiply(tt.poseAbs(), tt.solveFKFlange([35]))), multiply(armBase, arm.solveFKFlange(q.slice(0, 6))));
    expect(posDiff(robotSolveFK(arm, q), want)).toBeLessThan(1e-9);
    expect(rotDiff(robotSolveFK(arm, q), want)).toBeLessThan(1e-12);
    // six joints: the plain flange w.r.t. the robot base
    expect(posDiff(robotSolveFK(arm, q.slice(0, 6)), arm.solveFKFlange(q.slice(0, 6)))).toBeLessThan(1e-12);
    // a pose on the table, far from the rail start: the rail must move, the turntable stays at the given -20 deg
    const pose = mul(transl(0, -300, 150), rotx(180 * DEG));
    const r = syncedIK(sys, pose, { ext: [undefined, -20], seed: syncedJoints(arm) });
    expect(r.ok).toBe(true);
    expect(r.method).toBe('analytic');
    expect(r.extJoints[1]).toBe(-20);
    expect(r.extJoints[0]).toBeGreaterThan(500);
    expect(posDiff(syncedFK(sys, r.joints), pose)).toBeLessThan(1e-6);
    applySynced(arm, r.joints);
    expect(rail.joints()[0]).toBeCloseTo(r.extJoints[0], 9);
    expect(tt.joints()[0]).toBe(-20);
    // the applied state reproduces the pose in the world
    const world = multiply(multiply(tt.poseAbs(), tt.solveFKFlange()), pose);
    expect(posDiff(multiply(arm.poseAbs(), arm.solveFKFlange()), world)).toBeLessThan(1e-6);
  });

  it('solveIKWithCarrier (legacy API) solves in closed form and respects the arm base offset', () => {
    const { arm, rail } = station();
    arm.chain.base = transl(0, 0, 50); // RoboDK base offset inside the kinematics
    const target = mul(transl(3000, 300, 500), rotx(180 * DEG));
    const sol = solveIKWithCarrier(rail, arm, target, { seed: [0, ...arm.joints()] });
    expect(sol.ok).toBe(true);
    expect(sol.result.method).toBe('analytic');
    const mount = multiply(invert(multiply(rail.poseAbs(), rail.solveFKFlange())), arm.poseAbs());
    const flange = multiply(multiply(rail.solveFKFlange(sol.carrierJoints), mount), arm.solveFKFlange(sol.robotJoints));
    expect(posDiff(flange, target)).toBeLessThan(1e-6);
    expect(rotDiff(flange, target)).toBeLessThan(1e-8);
    expect(sol.carrierJoints[0]).toBeGreaterThan(1500);
    expect(sol.carrierJoints[0]).toBeLessThanOrEqual(jointLimits(rail.chain).upper[0]);
    // the numeric path (5-axis relaxation) reaches the same target through the combined chain
    const num = solveIKWithCarrier(rail, arm, target, { seed: [0, ...arm.joints()], freeToolZ: true });
    expect(num.ok).toBe(true);
    const f2 = multiply(multiply(rail.solveFKFlange(num.carrierJoints), mount), arm.solveFKFlange(num.robotJoints));
    expect(posDiff(f2, target)).toBeLessThan(1e-2);
  });
});
