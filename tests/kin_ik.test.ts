/**
 * T2 kinematics: closed-form IK (spherical-wrist 6R, UR-type 6R, delta) on the real RoboDK Library robot models.
 *  - RoboDK JointsConfig flags and SolveIK_All counts against RoboDK 5.9.4 ground truth (tests/fixtures/robodk_gt)
 *  - round trips on 1000 random reachable poses per robot family, honouring the requested configuration
 *  - continuous branch tracking along a linear path, also through the wrist singularity
 *  - numeric fallback for chains without a closed form (Denso Cobotta, KUKA iiwa)
 */
import { describe, it, expect } from 'vitest';
import { ChainDef, flangePose, jointLimits } from '../src/core/kinematics/chain';
import { analyzeChain, configFlags, solveAnalytic, wrapDeg } from '../src/core/kinematics/ik_analytic';
import { ikAll, solveIKBest, inverseKinematics } from '../src/core/kinematics/ik';
import { multiply, transl, Mat4 } from '../src/core/math/pose';
import { haveLib, libraryChains, rotDiff, posDiff } from './kin_gt_util';

let cache: ReturnType<typeof libraryChains> | null = null;
const lib = () => (cache ??= libraryChains());
const robot = (name: string): ChainDef => {
  const c = lib().find((x) => x.gt.name === name);
  if (!c) throw new Error(`no Library robot ${name}`);
  return c.chain;
};

/** Deterministic PRNG (mulberry32). */
function rng(seed: number) {
  let a = seed >>> 0;
  return () => { a = (a + 0x6d2b79f5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
}
const randomQ = (chain: ChainDef, r: () => number): number[] => {
  const { lower, upper } = jointLimits(chain);
  for (;;) {
    const q = lower.map((lo, i) => { const l = Math.max(lo, -360), h = Math.min(upper[i], 360); return l + (h - l) * r(); });
    // Comau (RoboDK joints type 11): the J3 limits also bound the parallelogram angle q2 + q3
    if (Math.round(chain.jointsType ?? -1) === 11 && (q[1] + q[2] < lower[2] || q[1] + q[2] > upper[2])) continue;
    return q;
  }
};

const FAMILIES: Array<{ name: string; kind: string }> = [
  { name: 'KUKA KR 6 R900 sixx', kind: 'spherical6' },
  { name: 'ABB IRB 1600-6/1.45', kind: 'spherical6' },
  { name: 'Fanuc LR Mate 200iD', kind: 'spherical6' },   // J2/J3 coupled
  { name: 'Motoman GP8', kind: 'spherical6' },
  { name: 'Kawasaki BA006L', kind: 'spherical6' },
  { name: 'Staubli RX160', kind: 'spherical6' },
  { name: 'Comau Smart5 NJ 165-3.4 SH', kind: 'spherical6' }, // parallelogram coupled
  { name: 'Mecademic Meca500 R3', kind: 'spherical6' },
  { name: 'Doosan M1013', kind: 'spherical6' },
  { name: 'UR10e', kind: 'ur6' },
  { name: 'JAKA Zu5', kind: 'ur6' },
  { name: 'Omron TM12', kind: 'ur6' },
  { name: 'ABB IRB 360-1/1130 4D', kind: 'delta' },
  { name: 'ABB IRB 360-3/800 3D', kind: 'delta' },
];

describe('kin: closed-form IK on RoboDK Library robots', () => {
  it.skipIf(!haveLib)('detects the closed-form family of every robot', { timeout: 180000 }, () => {
    for (const f of FAMILIES) expect(analyzeChain(robot(f.name)).kind, f.name).toBe(f.kind);
    expect(analyzeChain(robot('Denso Cobotta')).kind).toBeNull(); // offset wrist: numeric
    expect(analyzeChain(robot('KUKA LBR iiwa 14 R820')).kind).toBeNull(); // 7 axes: numeric
  });

  it.skipIf(!haveLib)('JointsConfig flags equal RoboDK for every 6-axis Library robot', { timeout: 180000 }, () => {
    const bad: string[] = [];
    let n = 0;
    for (const { gt, chain } of lib()) {
      if (!gt.jointsConfig || gt.joints.length !== chain.dof) continue;
      const f = configFlags(chain, gt.joints);
      if (!f) continue;
      n++;
      const want = gt.jointsConfig.slice(0, 3);
      // FLIP is undefined at q5 = 0 exactly (RoboDK reports either value)
      const cmp = Math.abs(gt.joints[4]) < 1e-6 ? [0, 1] : [0, 1, 2];
      if (cmp.some((i) => f[i] !== want[i])) bad.push(`${gt.name} q=${gt.joints}: ${f} != ${want}`);
    }
    expect(bad, bad.join('\n')).toEqual([]);
    expect(n).toBeGreaterThan(80);
  });

  it.skipIf(!haveLib)('the closed form recovers the RoboDK joints of every robot (pose -> IK contains q)', { timeout: 180000 }, () => {
    let n = 0;
    for (const { gt, chain } of lib()) {
      if (gt.joints.length !== chain.dof || !analyzeChain(chain).kind) continue;
      const sols = solveAnalytic(chain, flangePose(chain, gt.joints), gt.joints)!;
      expect(sols.some((s) => s.q.every((v, i) => Math.abs(wrapDeg(v - gt.joints[i])) < 1e-6)), gt.name).toBe(true);
      n++;
    }
    expect(n).toBeGreaterThan(80);
  });

  it.skipIf(!haveLib)('SolveIK_All counts equal RoboDK (spherical-wrist and delta robots)', { timeout: 180000 }, () => {
    const bad: string[] = [];
    let n = 0;
    for (const { gt, chain } of lib()) {
      if (gt.ikAllCount == null || gt.joints.length !== chain.dof) continue;
      const kind = analyzeChain(chain).kind;
      // Known gaps (see docs/kinematics.md): RoboDK's Fanuc J2/J3 interaction limits are not decoded yet, and RoboDK's own
      // count for UR-type arms is not deterministic (UR10 at the same pose: 12 in 01.a, 14 in Tutorial-UR).
      if (kind !== 'spherical6' && kind !== 'delta') continue;
      if (Math.round(chain.jointsType ?? -1) === 2) continue;
      const pose = flangePose(chain, gt.joints);
      const { lower, upper } = jointLimits(chain);
      const sols = solveAnalytic(chain, pose, gt.joints)!;
      // a ±360° variant exactly on a limit is decided by RoboDK's floating-point noise: skip those samples
      const ambiguous = sols.some((s) => s.q.some((v, i) => [-360, 360].some((k) => Math.abs(v + k - lower[i]) < 1e-3 || Math.abs(v + k - upper[i]) < 1e-3)));
      if (ambiguous) continue;
      n++;
      const all = ikAll(chain, pose, { seed: gt.joints });
      if (kind === 'delta') {
        // the 4-axis delta lists q4 turns within ±4 (RoboDK) — compare the single 3-axis solution only
        if (chain.dof === 3 && all.length !== gt.ikAllCount) bad.push(`${gt.name}: ${all.length} != ${gt.ikAllCount}`);
        continue;
      }
      if (all.length !== gt.ikAllCount) bad.push(`${gt.name} q=${gt.joints.map((v) => v.toFixed(1))}: ${all.length} != ${gt.ikAllCount}`);
      // every listed solution reaches the pose
      for (const q of all) expect(posDiff(flangePose(chain, q), pose)).toBeLessThan(1e-6);
    }
    expect(bad, bad.join('\n')).toEqual([]);
    expect(n).toBeGreaterThan(45);
  });

  for (const fam of FAMILIES) {
    it.skipIf(!haveLib)(`${fam.name}: 1000 random reachable poses solve with the requested configuration`, { timeout: 180000 }, () => {
      const chain = robot(fam.name);
      const r = rng(fam.name.length * 7919);
      const home = chain.joints.filter((j) => j.type !== 'fixed' && !j.mimic).map((j) => j.home ?? 0);
      let checked = 0;
      for (let k = 0; k < 1000; k++) {
        const q = randomQ(chain, r);
        const pose = flangePose(chain, q);
        const want = configFlags(chain, q);
        const res = solveIKBest(chain, pose, { seed: home, config: want ?? undefined });
        expect(res.ok, `${fam.name} q=${q}`).toBe(true);
        expect(res.method).toBe('analytic');
        const back = flangePose(chain, res.joints);
        expect(posDiff(back, pose), `${fam.name} q=${q}`).toBeLessThan(1e-6);
        expect(rotDiff(back, pose)).toBeLessThan(1e-8);
        if (want) expect(configFlags(chain, res.joints), `${fam.name} q=${q}`).toEqual(want);
        // the solution set lists q itself (joint turns included)
        const all = ikAll(chain, pose, { seed: q });
        expect(all.some((s) => s.every((v, i) => Math.abs(v - q[i]) < 1e-6)), `${fam.name} q=${q} not in ikAll`).toBe(true);
        checked++;
      }
      expect(checked).toBe(1000);
    });
  }

  it.skipIf(!haveLib)('6R arms give 8 distinct solutions in a stable configuration order', { timeout: 180000 }, () => {
    for (const name of ['KUKA KR 6 R900 sixx', 'UR10e', 'Fanuc LR Mate 200iD']) {
      const chain = robot(name);
      const q = name === 'UR10e' ? [20, -100, 80, -60, 70, 30] : [20, -60, 70, 30, 50, -40];
      const sols = solveAnalytic(chain, flangePose(chain, q), q)!;
      expect(sols.length, name).toBe(8);
      const flags = sols.map((s) => configFlags(chain, s.q)!.join(''));
      expect(new Set(flags).size, name).toBe(8);
      // no turn variants: one representative per configuration, sorted by [REAR, LOWER, FLIP]
      const a = ikAll(chain, flangePose(chain, q), { turns: 'none' }).map((s) => configFlags(chain, s)!.join(''));
      expect(a).toEqual([...a].sort());
      expect(ikAll(chain, flangePose(chain, q), { turns: 'none' })).toEqual(ikAll(chain, flangePose(chain, q), { turns: 'none' }));
    }
  });

  it.skipIf(!haveLib)('MoveL tracks one continuous branch, also through the wrist singularity (q5 = 0)', { timeout: 180000 }, () => {
    for (const name of ['KUKA KR 6 R900 sixx', 'KUKA KR 210 R2700 extra', 'Fanuc LR Mate 200iD', 'ABB IRB 4600-40/2.55', 'UR10e']) {
      const chain = robot(name);
      // start in the standard home posture with the wrist exactly singular
      const q0 = name === 'UR10e' ? [0, -90, -90, 0, 90, 0] : name.startsWith('KUKA') ? [0, -90, 90, 0, 0, 0] : [0, 0, 0, 0, 0, 0];
      const p0 = flangePose(chain, q0);
      for (const dir of [[100, 0, 0], [0, 100, 0], [0, 0, -100], [60, -60, 40]]) {
        let q = q0.slice();
        let maxStep = 0;
        for (let s = 1; s <= 100; s++) {
          const t = s / 100;
          const target = multiply(transl(dir[0] * t, dir[1] * t, dir[2] * t), p0);
          const r = solveIKBest(chain, target, { seed: q });
          expect(r.ok, `${name} ${dir} at ${s}%`).toBe(true);
          expect(posDiff(flangePose(chain, r.joints), target)).toBeLessThan(1e-6);
          // joints 1-3 move smoothly; at the singular start the wrist may redistribute q4/q6 but never jumps a turn
          const step = Math.max(...r.joints.map((v, i) => Math.abs(v - q[i])));
          maxStep = Math.max(maxStep, step);
          q = r.joints;
        }
        expect(maxStep, `${name} ${dir}`).toBeLessThan(90);
      }
      // exactly at the singularity: solved, flagged, exact
      const r0 = solveIKBest(chain, p0, { seed: q0 });
      expect(r0.ok).toBe(true);
      if (name !== 'UR10e') expect(r0.nearSingular, name).toBe(true);
      expect(posDiff(flangePose(chain, r0.joints), p0)).toBeLessThan(1e-6);
    }
  });

  it.skipIf(!haveLib)('numeric fallback: Denso Cobotta and KUKA iiwa (7 axes) solve from a nearby seed', { timeout: 180000 }, () => {
    for (const name of ['Denso Cobotta', 'KUKA LBR iiwa 14 R820']) {
      const chain = robot(name);
      const r = rng(42);
      let ok = 0;
      for (let k = 0; k < 50; k++) {
        const q = randomQ(chain, r).map((v) => v * 0.6);
        const pose = flangePose(chain, q);
        const res = solveIKBest(chain, pose, { seed: q.map((v, i) => v + (i % 2 ? 4 : -4)) });
        if (res.ok && posDiff(flangePose(chain, res.joints), pose) < 1e-3) ok++;
        expect(res.method).toBe('numeric');
      }
      expect(ok, name).toBe(50);
    }
  });

  it.skipIf(!haveLib)('performance: 20 000 closed-form solves per second order (no per-call re-analysis)', { timeout: 180000 }, () => {
    const chain = robot('KUKA KR 6 R900 sixx');
    const r = rng(7);
    const poses: Mat4[] = [];
    const seeds: number[][] = [];
    for (let k = 0; k < 4000; k++) { const q = randomQ(chain, r); poses.push(flangePose(chain, q)); seeds.push(q.map((v) => v + 1)); }
    solveIKBest(chain, poses[0], { seed: seeds[0] });
    const t0 = performance.now();
    for (let k = 0; k < poses.length; k++) solveIKBest(chain, poses[k], { seed: seeds[k] });
    const dt = performance.now() - t0;
    // generous bound for loaded CI machines: < 0.5 ms per solve (8 branches, verification and limit handling)
    expect(dt / poses.length).toBeLessThan(0.5);
  });

  it('numeric solver: unscaled damping no longer blows up at a stretched arm (sub-tolerance residual is a warning)', () => {
    // planar 3R arm fully stretched: target slightly beyond reach along the arm
    const chain: ChainDef = {
      name: '3R', links: [{ name: 'b', visuals: [] }, { name: 'l1', visuals: [] }, { name: 'l2', visuals: [] }, { name: 'l3', visuals: [] }],
      joints: [0, 1, 2].map((i) => ({ name: `J${i + 1}`, type: 'revolute' as const, origin: i ? transl(500, 0, 0) : new Float64Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]), axis: [0, 0, 1] as [number, number, number], lower: -180, upper: 180 })),
      flange: transl(500, 0, 0),
    };
    const res = inverseKinematics(chain, transl(1499.9, 0, 0), { positionOnly: true, seed: [5, -5, 5] });
    expect(res.ok).toBe(true);
    expect(res.posError).toBeLessThan(1e-3);
    const far = inverseKinematics(chain, transl(1600, 0, 0), { positionOnly: true, seed: [5, -5, 5] });
    expect(far.ok).toBe(false);
    expect(far.posError).toBeGreaterThan(99); // the best it can do is the stretched arm, 100 mm short
    expect(far.posError).toBeLessThan(101);
  });
});
