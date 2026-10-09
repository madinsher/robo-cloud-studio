import { it, expect } from 'vitest';
import { createRobotFromLibrary } from '../src/core/items/library';
import { planMoveL } from '../src/core/motion/trajectory';
import { multiply, transl } from '../src/core/math/pose';

it('MoveL from a fully stretched (singular) UR pose is reported as singularity', () => {
  const r = createRobotFromLibrary('UR10e');
  const q0 = [0, -90, 0, -90, 90, 0];
  const p0 = r.solveFK(q0);
  const tr = planMoveL(r, q0, multiply(p0, transl(150, 0, 0)), 300, 2000);
  expect(tr.ok).toBe(false);
  expect(tr.error).toMatch(/singularity/);
});

it('MoveL from the library home works in all tool directions', () => {
  const r = createRobotFromLibrary('UR10e');
  const q0 = r.jointsHome();
  const p0 = r.solveFK(q0);
  for (const d of [[150, 0, 0], [0, 150, 0], [0, 0, 150], [-100, -100, 50]]) {
    const tr = planMoveL(r, q0, multiply(p0, transl(d[0], d[1], d[2])), 300, 2000);
    expect(tr.ok, `${d}: ${tr.error}`).toBe(true);
  }
});
