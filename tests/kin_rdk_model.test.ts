/**
 * T2 kinematics: the RoboDK robot model (rdk_model.ts) reproduces RoboDK 5.9.4 SolveFK for every robot of the Library.
 * Ground truth: tests/fixtures/robodk_gt (12 random joint samples per robot, flange w.r.t. the robot base).
 * The robot tables are read straight from C:/RoboDK/Library (skipped when RoboDK is not installed).
 */
import { describe, it, expect } from 'vitest';
import { ChainDef, flangePose, forwardKinematics } from '../src/core/kinematics/chain';
import { haveLib, libraryChains, m16, rotDiff, posDiff } from './kin_gt_util';

describe('kin: RoboDK robot model vs RoboDK SolveFK', () => {
  it.skipIf(!haveLib)('FK of every Library robot matches RoboDK within 0.01 mm / 0.01 deg', { timeout: 180000 }, () => {
    const all = libraryChains();
    expect(all.length).toBeGreaterThan(60);
    const bad: string[] = [];
    let checked = 0, synced = 0;
    for (const { file, gt, chain } of all) {
      const dof = chain.dof ?? 0;
      // robots synchronised with external axes report FK over arm + carrier: covered by kin_combined
      if (gt.fk.some((s) => s.q.length !== dof)) { synced++; continue; }
      for (const s of gt.fk) {
        if (!s.flange) continue;
        const ours = flangePose(chain, s.q);
        const g = m16(s.flange);
        const pe = posDiff(ours, g);
        const re = rotDiff(ours, g) * 180 / Math.PI;
        // the GT matrices carry 6 decimals: 1e-6 rad rounding ~ 6e-5 deg
        if (pe > 0.01 || re > 0.01) { bad.push(`${gt.name} (${file}) q=${s.q.join(',')}: ${pe.toFixed(4)} mm ${re.toFixed(4)} deg`); break; }
        checked++;
      }
      // the full-FK path (link poses for rendering) agrees with the fast flange path
      const q = gt.fk[0]?.q;
      if (q && q.length === dof) { const a = forwardKinematics(chain, q).flange, b = flangePose(chain, q); expect(Math.max(...Array.from(a).map((v, i) => Math.abs(v - b[i])))).toBeLessThan(1e-9); }
    }
    expect(bad, bad.join('\n')).toEqual([]);
    expect(checked).toBeGreaterThan(700);
    expect(synced).toBe(3); // ABB IRB 6640 on rail + turntable (05.c), Comau NM 45 on the Gudel gantry (05.d), ABB IRB 2600 on the turntable (02.f)
  });

  it.skipIf(!haveLib)('brand conventions: KUKA, Fanuc, Yaskawa, Kawasaki and Comau chains carry the RoboDK senses and couplings', { timeout: 180000 }, () => {
    const all = libraryChains();
    const by = (n: string) => all.find((c) => c.gt.name === n)!.chain;
    const signs = (c: ChainDef) => c.joints.filter((j) => j.type !== 'fixed').map((j) => Math.sign(j.axis[2]));
    expect(signs(by('KUKA KR 6 R900 sixx'))).toEqual([-1, 1, 1, -1, 1, -1]);
    expect(signs(by('Fanuc LR Mate 200iD'))).toEqual([1, 1, -1, -1, -1, -1]);
    expect(by('Fanuc LR Mate 200iD').joints[2].couple).toEqual({ joint: 'J2', multiplier: 1 });
    expect(signs(by('Motoman GP8'))).toEqual([1, 1, -1, -1, -1, -1]);
    expect(by('Motoman GP8').joints[2].couple).toBeUndefined();
    expect(signs(by('Kawasaki BA006L'))).toEqual([-1, 1, -1, 1, -1, 1]);
    expect(by('Comau Smart5 NJ 165-3.4 SH').joints[2].couple?.multiplier).toBe(1);
    // base offsets: Yaskawa / Fanuc / Stäubli base frames sit at J2 height
    expect(by('Motoman GP8').base?.[14]).toBeCloseTo(-330, 6);
    expect(by('Staubli RX160').base?.[14]).toBeLessThan(-500);
    // delta robots become parallel models
    expect(by('ABB IRB 360-1/1130 4D').parallel?.kind).toBe('delta');
  });
});
