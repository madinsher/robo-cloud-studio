/**
 * MURKA retail — arm kinematics. The TS port of murom_manipulation/arm_kinematics.py must (1) equal the kinematic
 * chain the studio imported from the Webots PROTO (independent source: Murom.proto joints, not the Python code), and
 * (2) return the same IK / palm IK / IK selection as the original Python module on the scenario targets (run with
 * the MURKA sources and numpy; skipped visibly when they are absent).
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { buildMurkaRetailStation, RetailStationResult } from '../src/murka/retail';
import { ArmKinematics, BASKET_BOX, V3 } from '../src/murka/retail/kinematics';
import { AssetStore } from '../src/scene/assets';
import { invert, multiply, transformPoint } from '../src/core/math/pose';
import { Robot } from '../src/core/items/robot';
import { HAS_MURKA, HAS_PY, runPython, V2_SRC } from './murka_retail_helpers';

const RAD = Math.PI / 180;
/** Deterministic pseudo-random joints within the MURKA limits. */
function* joints(n: number, seed = 7): Generator<number[]> { let s = seed; const r = () => ((s = (s * 1103515245 + 12345) % 2147483648) / 2147483648); for (let i = 0; i < n; i++) yield [r() * 5.2 - 2.6, r() * 4.2 - 2.1, r() * 3.2 - 1.6, r() * 4.8 - 2.4, r() * 5.2 - 2.6, r() * 3.1 - 1.55]; }

describe.skipIf(!HAS_MURKA)('MURKA arm kinematics vs the imported Webots robot', () => {
  let v2: RetailStationResult, v1: RetailStationResult;
  beforeAll(async () => {
    v2 = await buildMurkaRetailStation({ variant: 'v2', assets: new AssetStore(), loadMeshes: false });
    v1 = await buildMurkaRetailStation({ variant: 'v1', assets: new AssetStore(), loadMeshes: false });
  }, 300000);

  it('v2: palm frame of the port = flange of the imported 6-DOF chain (both arms, 200 poses, 1e-9 m / 1e-12)', () => {
    for (const side of ['left', 'right'] as const) {
      const arm: Robot = v2.robot[side]; const k = new ArmKinematics(side);
      expect(arm.jointNames()).toEqual([...k.jointNames, ...k.wristNames]);
      for (const q of joints(200, side === 'left' ? 3 : 11)) {
        const F = multiply(invert(v2.robot.base.poseAbs()), multiply(arm.poseAbs(), arm.solveFKFlange(q.map((v) => v / RAD))));
        const [R, o] = k.palmFrame(q.slice(0, 4), q.slice(4));
        expect(Math.hypot(F[12] / 1000 - o[0], F[13] / 1000 - o[1], F[14] / 1000 - o[2])).toBeLessThan(1e-9);
        for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) expect(Math.abs(F[j * 4 + i] - R[i * 3 + j])).toBeLessThan(1e-12);
      }
    }
  });

  it('v2: the studio TCP of each arm is the magnetic grasp point (palm surface centre of arm_kinematics.palm_point)', () => {
    for (const side of ['left', 'right'] as const) {
      const arm = v2.robot[side]; const k = new ArmKinematics(side);
      for (const q of joints(20, 5)) {
        arm.setJoints(q.map((v) => v / RAD));
        const T = multiply(invert(v2.robot.base.poseAbs()), arm.poseTCPAbs());
        const p = k.palmPoint(q.slice(0, 4), q.slice(4));
        expect(Math.hypot(T[12] / 1000 - p[0], T[13] / 1000 - p[1], T[14] / 1000 - p[2])).toBeLessThan(1e-9);
      }
    }
  });

  it('v1 classic: 4-DOF chain, forearm tip of the port = imported flange + (0, 0, -0.248)', () => {
    for (const side of ['left', 'right'] as const) {
      const arm = v1.robot[side]; const k = new ArmKinematics(side);
      expect(arm.dof).toBe(4);
      for (const q of joints(100, 17)) {
        const F = multiply(invert(v1.robot.base.poseAbs()), multiply(arm.poseAbs(), arm.solveFKFlange(q.slice(0, 4).map((v) => v / RAD))));
        const tip = transformPoint(F, [0, 0, -248]); const p = k.fk(q);
        expect(Math.hypot(tip[0] / 1000 - p[0], tip[1] / 1000 - p[1], tip[2] / 1000 - p[2])).toBeLessThan(1e-9);
      }
    }
  });

  it('MURKA joint limits and start poses are applied (v2 travel pose [0.3, 0, 0, -2.2], v1 hanging arms, head ±1.6 / -0.9..0.6)', () => {
    expect(v2.robot.left.jointsHome().map((d) => d * RAD)).toEqual([0.3, 0, 0, -2.2, 0, 0].map((v) => expect.closeTo(v, 12)));
    expect(v1.robot.right.jointsHome()).toEqual([0, 0, 0, 0]);
    const lim = v2.robot.head.jointLimits();
    expect(lim.lower.map((d) => d * RAD)).toEqual([expect.closeTo(-1.6, 12), expect.closeTo(-0.9, 12)]);
    expect(lim.upper.map((d) => d * RAD)).toEqual([expect.closeTo(1.6, 12), expect.closeTo(0.6, 12)]);
    expect(v2.robot.left.jointLimits().upper.map((d) => d * RAD)).toEqual([2.6, 2.1, 1.6, 2.4, 2.6, 1.55].map((v) => expect.closeTo(v, 12)));
  });
});

const PY_KIN = String.raw`
import json, sys, numpy as np
inp = json.load(sys.stdin)
sys.path.insert(0, inp['pkg'])
from murom_manipulation import arm_kinematics as ak
out = []
for c in inp['cases']:
    k = ak.ArmKinematics(c['side'])
    t = np.array(c['target'])
    if c['kind'] == 'ik':
        q, err = k.ik(t, seed=None if c.get('seed') is None else np.array(c['seed']))
        out.append({'q': [float(v) for v in q], 'err': float(err)})
    elif c['kind'] == 'ik_palm':
        q, w, err = k.ik_palm(t, c['normal'], seed=None if c.get('seed') is None else np.array(c['seed']))
        out.append({'q': [float(v) for v in q], 'w': [float(v) for v in w], 'err': float(err)})
    else:
        ref = None if c.get('ref') is None else (np.array(c['ref'][0]), np.array(c['ref'][1]))
        obst = [(np.array(lo), np.array(hi)) for lo, hi in c.get('obstacles', [])]
        sel = k.ik_palm_select(t, c['normal'], ref=ref, tol=0.02, max_normal_deg=3.0, obstacles=obst)
        out.append(None if sel is None else {'q': [float(v) for v in sel[0]], 'w': [float(v) for v in sel[1]], 'err': float(sel[2]), 'path': float(sel[3])})
print(json.dumps(out))
`;

describe.skipIf(!HAS_PY)('MURKA arm kinematics: port = original Python (numpy) on the scenario targets', () => {
  // targets in base_link: gondola grasp / pre / stage (left), basket slot, packing, two-hand box edges (palms inwards)
  const cases: Array<{ kind: string; side: 'left' | 'right'; target: V3; seed?: number[] | null; normal?: V3; ref?: [number[], number[]] | null; obstacles?: Array<[V3, V3]> }> = [
    { kind: 'ik', side: 'left', target: [-0.056, 0.644, 1.055] },
    { kind: 'ik', side: 'right', target: [0.3, -0.45, 1.2], seed: [0.3, 0, 0, -2.2] },
    { kind: 'ik', side: 'left', target: [0.2, 0.3, 0.9] },
    { kind: 'ik_palm', side: 'left', target: [-0.056, 0.644, 1.055], normal: [0, 0, -1] },
    { kind: 'ik_palm', side: 'left', target: [-0.26, 0.12, 1.012], normal: [0, 0, -1], seed: [0.3, 0, 0, -2.2] },
    { kind: 'ik_palm', side: 'right', target: [0.43, -0.18, 1.33], normal: [0, 1, 0], seed: [0.3, 0, 0, -2.2] },
    { kind: 'select', side: 'left', target: [-0.056, 0.644, 1.055], normal: [0, 0, -1], ref: [[0.3, 0, 0, -2.2], [0, 0]], obstacles: [[[-1.056, 0.524, 0], [0.944, 1.324, 0.89]], [[-1.056, 0.524, 1.21], [0.944, 1.324, 1.24]]] },
    { kind: 'select', side: 'left', target: [-0.26, 0.12, 1.162], normal: [0, 0, -1], ref: [[0.3, 0, 0, -2.2], [0, 0]], obstacles: [[[-1, 0.495, 0], [1, 1.5, 2]], [[...BASKET_BOX[0]], [...BASKET_BOX[1]]]] },
    { kind: 'select', side: 'right', target: [0.43, -0.18, 1.33], normal: [0, 1, 0], ref: [[0.3, 0, 0, -2.2], [0, 0]] },
    { kind: 'select', side: 'left', target: [0.05, 0.40, 1.25], normal: [0, 0, -1], ref: [[0.3, 0, 0, -2.2], [0, 0]] },
  ];
  let py: Array<{ q: number[]; w?: number[]; err: number; path?: number } | null>;
  beforeAll(() => { py = runPython(PY_KIN, { pkg: `${V2_SRC}/murom_manipulation`, cases }); }, 300000);

  it('ik / ik_palm / ik_palm_select: same joints (1e-6 rad), same residual and path penetration', { timeout: 120000 }, () => {
    cases.forEach((c, i) => {
      const k = new ArmKinematics(c.side); const want = py[i];
      if (c.kind === 'ik') { const [q, err] = k.ik(c.target, c.seed ?? null); expect(q.map((v, j) => Math.abs(v - want!.q[j])).every((d) => d < 1e-6), `${i} ${q} vs ${want!.q}`).toBe(true); expect(err).toBeCloseTo(want!.err, 8); return; }
      if (c.kind === 'ik_palm') { const [q, w, err] = k.ikPalm(c.target, c.normal!, c.seed ?? null); expect(q.map((v, j) => Math.abs(v - want!.q[j])).every((d) => d < 1e-6), `${i}`).toBe(true); expect(w[0]).toBeCloseTo(want!.w![0], 6); expect(w[1]).toBeCloseTo(want!.w![1], 6); expect(err).toBeCloseTo(want!.err, 8); return; }
      const sel = k.ikPalmSelect(c.target, c.normal!, { ref: c.ref ?? null, obstacles: c.obstacles });
      if (want === null) { expect(sel).toBeNull(); return; }
      expect(sel, `${i}`).not.toBeNull();
      expect(sel!.q.map((v, j) => Math.abs(v - want.q[j])).every((d) => d < 1e-6), `${i} ${sel!.q} vs ${want.q}`).toBe(true);
      expect(sel!.err).toBeCloseTo(want.err, 8);
      expect(sel!.pathPen).toBeCloseTo(want.path!, 8);
    });
  });
});
