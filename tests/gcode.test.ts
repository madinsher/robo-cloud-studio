import { describe, it, expect } from 'vitest';
import { parseGcode, gcodeToCurves } from '../src/io/programs/gcode';
import { existsSync, readFileSync } from 'node:fs';
import path from 'node:path';
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

describe('G-code fixes and APT / CL data (P0f)', () => {
  it('R-format arcs are centred on the chord bisector (G2 X20 Y0 R10 from the origin stays in X 0..20)', () => {
    const g = parseGcode('G21 G90\nG0 X0 Y0 Z0\nG2 X20 Y0 R10 F100\n');
    const pts = g.segments.filter((s) => s.kind === 'cut').flatMap((s) => s.points);
    expect(Math.min(...pts.map((p) => p[0]))).toBeGreaterThanOrEqual(-1e-9);
    expect(Math.max(...pts.map((p) => p[0]))).toBeLessThanOrEqual(20 + 1e-9);
    // clockwise half circle from (0,0) to (20,0) around (10,0) passes through (10, 10)
    expect(pts.some((p) => Math.abs(p[0] - 10) < 0.5 && Math.abs(p[1] - 10) < 0.05)).toBe(true);
    for (const p of pts) expect(Math.hypot(p[0] - 10, p[1])).toBeCloseTo(10, 6);
    // negative R: the arc longer than 180 deg (G3 from (0,0) to (10,10), R-10: centre (10,0), three quarters)
    const g2 = parseGcode('G0 X0 Y0\nG3 X10 Y10 R-10\n');
    const q = g2.segments.filter((s) => s.kind === 'cut').flatMap((s) => s.points);
    for (const p of q) expect(Math.hypot(p[0] - 10, p[1])).toBeCloseTo(10, 6);
    expect(g2.length).toBeCloseTo((3 / 4) * 2 * Math.PI * 10, 0);
  });
  it('inch programs convert feeds to mm/min (Library Smilepath: F15 / F300 in/min)', () => {
    const g = parseGcode('G20 G90\nG0 X0 Y0\nG1 X1 F15\nG1 X2 F300\n');
    expect(g.segments.filter((s) => s.kind === 'cut').map((s) => s.feed)).toEqual([15 * 25.4, 300 * 25.4]);
    expect(g.segments.at(-1)!.points.at(-1)).toEqual([50.8, 0, 0]);
  });
  it('APT: GOTO / RAPID / FEDRAT / LOADTL / continuation lines / 5-axis tool axes / CIRCLE arcs', () => {
    const apt = ['UNITS/MM', 'RAPID', 'GOTO / 0, 0, 100', 'FEDRAT/ 500.0,MMPM', 'LOADTL/2,1', 'GOTO / 0 0 0 0 0 1', 'GOTO / 100, $', '  0, 0, 0, 0.6, 0.8', 'CIRCLE/ 100, 50, 0, 0, 0, 1, 50', 'GOTO / 150, 50, 0', '$$ comment', 'FEDRAT/ 10, IPM', 'GODLTA/ 0, 0, 10'].join('\n');
    const g = parseGcode(apt, { arcStep: 1 });
    // the first GOTO only defines the start point (APT has no implicit origin); the RAPID applied to it
    const cuts = g.segments.filter((s) => s.kind === 'cut');
    expect(cuts[0].points[0]).toEqual([0, 0, 100]);
    expect(cuts[0].feed).toBe(500);
    expect(cuts[0].tool).toBe(2);
    expect(cuts[0].normals![1]).toEqual([0, 0, 1]);
    expect(cuts[0].points[2]).toEqual([100, 0, 0]);
    expect(cuts[0].normals![2]).toEqual([0, 0.6, 0.8]);
    // counter-clockwise quarter circle about +Z around (100,50) from (100,0) to (150,50)
    const arc = cuts[0].points.slice(3);
    for (const p of arc) expect(Math.hypot(p[0] - 100, p[1] - 50)).toBeCloseTo(50, 6);
    expect(arc.at(-1)).toEqual([150, 50, 0]);
    expect(arc.every((p) => p[0] >= 100 - 1e-9 && p[1] <= 50 + 1e-9)).toBe(true);
    expect(cuts.at(-1)!.feed).toBeCloseTo(254, 9);
    expect(cuts.at(-1)!.points.at(-1)).toEqual([150, 50, 10]);
  });
  const LIB = 'C:/RoboDK/Library';
  const lib = (f: string) => readFileSync(path.join(LIB, f), 'utf8');
  it.skipIf(!existsSync(LIB))('Library NC / APT files give their tool paths (RoboDK examples 02.c, 02.f, 02.e, 02.g, APT instructions)', () => {
    const bowl = parseGcode(lib('Example-02.c-Robot Machining Bowl.nc'));
    const bowlPts = bowl.segments.reduce((a, s) => a + s.points.length, 0);
    expect(bowlPts).toBeGreaterThan(800);
    expect(bowl.segments.find((s) => s.kind === 'cut')!.feed).toBe(300);
    expect(bowl.bounds.min[2]).toBeCloseTo(12, 3);
    expect(bowl.bounds.max[2]).toBeCloseTo(61.2, 3);
    const sph = parseGcode(lib('Example-02.f-Robot Machining Spherical Part.nc'));
    const sc = sph.segments.filter((s) => s.kind === 'cut');
    expect(sc.length).toBeGreaterThan(10);
    expect(sc.every((s) => s.normals && s.normals.length === s.points.length)).toBe(true);
    for (const n of sc[0].normals!) expect(Math.hypot(n[0], n[1], n[2])).toBeCloseTo(1, 3);
    const machCurves = gcodeToCurves(sph);
    expect(machCurves[0].normals!.length).toBe(machCurves[0].points.length);
    const rook = parseGcode(lib('Example-02.e-Robot Machining Chess Rook.apt'));
    expect(rook.segments.reduce((a, s) => a + s.points.length, 0)).toBeGreaterThan(3000);
    const eiffel = parseGcode(lib('Example-02.g-Robot Machining Tour Eiffel.apt'));
    expect(eiffel.segments.reduce((a, s) => a + s.points.length, 0)).toBeGreaterThan(3000);
    const ex = parseGcode(lib('Example-APT Instructions.apt'));
    expect(ex.segments.reduce((a, s) => a + s.points.length, 0)).toBeGreaterThanOrEqual(4);
    expect(ex.segments.some((s) => s.tool === 2 && s.feed === 500)).toBe(true);
    const smile = parseGcode(lib('Example-G-code-Smilepath.nc'));
    expect(smile.units).toBe('inch'); // source units; coordinates and feeds are converted to mm
    expect(new Set(smile.segments.map((s) => s.feed).filter((f) => f !== undefined))).toEqual(new Set([15 * 25.4, 300 * 25.4]));
  });
});
