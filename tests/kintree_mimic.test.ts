/**
 * Kinematic trees with cross-chain mimic joints (the repository's palletizer: parallelogram links modelled as URDF
 * mimics of joints in other branches). Ground truth: urdf_parser_py FK of the xacro-expanded URDF with every revolute /
 * continuous joint at 0 and at 0.3 rad, mimics following their masters (tests/fixtures/kintree_gt/palletizer_fk.json).
 */
import { describe, it, expect } from 'vitest';
import * as fs from 'node:fs';
import { Station } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { importURDF } from '../src/io/urdf/urdf';
import { multiply, invert } from '../src/core/math/pose';
import type { Mat4 } from '../src/core/math/pose';

const URDF = fs.readFileSync('tests/fixtures/kintree_gt/palletizer.urdf', 'utf8');
const GT = JSON.parse(fs.readFileSync('tests/fixtures/kintree_gt/palletizer_fk.json', 'utf8')) as { fk0: Record<string, number[][]>; fk1: Record<string, number[][]> };
const mat = (r: number[][]): Mat4 => Float64Array.from([r[0][0], r[1][0], r[2][0], 0, r[0][1], r[1][1], r[2][1], 0, r[0][2], r[1][2], r[2][2], 0, r[0][3] * 1000, r[1][3] * 1000, r[2][3] * 1000, 1]) as Mat4;

/** World pose (mm) of every chain link of the imported robots, after setting the independent joints by name. */
function linkPoses(q: Record<string, number>) {
  const st = new Station('s'); const res = importURDF(URDF, { name: 'palletizer' }); st.addChild(res.root);
  const out = new Map<string, Mat4>();
  for (const r of res.robots) {
    const names = r.jointNames(); r.setJoints(names.map((n) => q[n] ?? 0));
    const fk = r.fk(r.joints()); const base = r.poseAbs();
    r.chain.links.forEach((l, i) => { if (i > 0 && !l.name.endsWith('__return')) out.set(l.name, multiply(base, fk.linkPoses[i])); });
  }
  return { out, res };
}
const errMm = (a: Mat4, b: Mat4) => Math.hypot(a[12] - b[12], a[13] - b[13], a[14] - b[14]);
const errRad = (a: Mat4, b: Mat4) => { const R = multiply(invert(a), b); return Math.acos(Math.max(-1, Math.min(1, (R[0] + R[5] + R[10] - 1) / 2))); };

describe('cross-chain mimic joints (palletizer linkage)', () => {
  it('imports as one mechanism with the 4 independent joints of ROS, no joint imported as independent', () => {
    const { res } = linkPoses({});
    expect(res.report.join('\n')).not.toMatch(/imported as independent/);
    expect(res.robots).toHaveLength(1);
    expect(res.robots[0].jointNames().sort()).toEqual(['pal_joint0', 'pal_joint1', 'pal_joint2', 'pal_joint3']);
  });
  for (const [label, q, gt] of [['q = 0', {}, GT.fk0], ['every joint 0.3 rad (mimics follow)', { pal_joint0: 0.3, pal_joint1: 0.3, pal_joint2: 0.3, pal_joint3: 0.3 }, GT.fk1]] as const) {
    it(`every link pose equals the URDF tree FK at ${label}`, () => {
      const deg = Object.fromEntries(Object.entries(q).map(([k, v]) => [k, (v as number) * 180 / Math.PI]));
      const { out } = linkPoses(deg);
      const names = Object.keys(gt).filter((l) => out.has(l));
      expect(names.sort()).toEqual(['pal_link1', 'pal_link2', 'pal_link3', 'pal_link4', 'pal_link4_1', 'pal_link5', 'pal_link6', 'pal_link7', 'pal_link8', 'pal_link9']);
      for (const l of names) {
        const a = out.get(l)!, b = mat(gt[l]);
        expect(errMm(a, b), `${l} position`).toBeLessThan(0.01);
        expect(errRad(a, b), `${l} rotation`).toBeLessThan(1e-5);
      }
    });
  }
  it('jogging only pal_joint1 moves the gripper mount like ROS (mimics of pal_joint1 in other branches follow)', () => {
    const { out } = linkPoses({ pal_joint1: 0.3 * 180 / Math.PI });
    const { out: o0 } = linkPoses({});
    // pal_link4_1 (gripper mount) moved; pal_link5 / pal_link9 (mimics of pal_joint1 on other branches) rotated by 0.3 rad
    expect(errMm(out.get('pal_link4_1')!, o0.get('pal_link4_1')!)).toBeGreaterThan(5);
    expect(errRad(out.get('pal_link5')!, o0.get('pal_link5')!)).toBeCloseTo(0.3, 3);
    expect(errRad(out.get('pal_link9')!, o0.get('pal_link9')!)).toBeCloseTo(0.3, 3);
    const r = importURDF(URDF).robots[0]; expect(r).toBeInstanceOf(Robot);
  });
});
