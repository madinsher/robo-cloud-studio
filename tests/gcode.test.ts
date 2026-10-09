import { describe, it, expect } from 'vitest';
import { parseGcode, gcodeToCurves } from '../src/io/programs/gcode';
import { Station, SceneObject } from '../src/core/items/item';
import { createRobotFromLibrary } from '../src/core/items/library';
import { generateCurveFollow } from '../src/core/motion/pathfollow';
import { transl } from '../src/core/math/pose';

describe('G-code import', () => {
  it('parses linear and arc moves and builds a follow program', () => {
    const nc = `%\nG21 G90\nG0 X0 Y0 Z10\nM3 S12000\nG1 Z0 F300\nG1 X100 Y0 F600\nG2 X150 Y50 I0 J50\nG1 X150 Y150\nG0 Z10\nM5\n%`;
    const g = parseGcode(nc);
    expect(g.units).toBe('mm');
    const curves = gcodeToCurves(g);
    expect(curves.length).toBe(1);
    const pts = curves[0].points;
    expect(pts[pts.length - 1]).toEqual([150, 150, 0]);
    // arc end point and roughly quarter circle length (78.5 mm) + lines
    expect(g.length).toBeGreaterThan(100 + 78 + 100 + 10);
    const st = new Station();
    const r = st.addChild(createRobotFromLibrary('UR10e'));
    const part = st.addChild(new SceneObject('Part'));
    part.setPose(transl(500, -100, 200));
    part.curves = curves;
    const res = generateCurveFollow(st, r, part, curves[0], { step: 10, approach: 30, io: 'Spindle', zMode: 'down' });
    expect(res.unreachable).toBe(0);
    expect(res.points).toBeGreaterThan(30);
  });
});
