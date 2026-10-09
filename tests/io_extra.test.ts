import { describe, it, expect } from 'vitest';
import { parseOBJ } from '../src/io/mesh/obj';
import { parseDHText, robotFromDH, robotToDHText } from '../src/io/robodk/dh';
import { createRobotFromLibrary } from '../src/core/items/library';
import { getPos, distance } from '../src/core/math/pose';

describe('extra importers', () => {
  it('parses OBJ with quads', () => {
    const obj = `v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nvn 0 0 1\nf 1//1 2//1 3//1 4//1\n`;
    const m = parseOBJ(obj, 1000);
    expect(m.triangles).toBe(2);
    expect(m.max[0]).toBe(1000);
  });
  it('DH text round trip reproduces the library UR5e', () => {
    const ur = createRobotFromLibrary('UR5e');
    const text = robotToDHText(ur)!;
    expect(text).toContain('# name');
    const r2 = robotFromDH(parseDHText(text));
    expect(r2.dof).toBe(6);
    const q = [10, -70, 50, -60, 80, 20];
    expect(distance(getPos(ur.solveFK(q)), getPos(r2.solveFK(q)))).toBeLessThan(1e-9);
  });
});
