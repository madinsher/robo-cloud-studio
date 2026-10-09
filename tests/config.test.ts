import { it, expect } from 'vitest';
import { createRobotFromLibrary } from '../src/core/items/library';
import { transl, mul, rotx, DEG } from '../src/core/math/pose';

it('IK keeps the elbow-up configuration of the seed', () => {
  const r = createRobotFromLibrary('UR10e');
  const seed = [0, -90, 90, -90, -90, 0];
  expect(r.configFlags(seed).elbowUp).toBe(true);
  const target = mul(transl(800, -150, -60), rotx(180 * DEG));
  const res = r.solveIK(target, { seed }, transl(0, 0, 160));
  expect(res.ok).toBe(true);
  expect(r.configFlags(res.joints).elbowUp).toBe(true);
});
