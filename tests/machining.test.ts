import { describe, it, expect } from 'vitest';
import { parseGcode, gcodeToMachiningCurves, gcodeStats } from '../src/io/programs/gcode';
import { generateMachining } from '../src/core/motion/machining';
import { createRobotFromLibrary } from '../src/core/items/library';
import { Station, SceneObject, Tool, ItemType } from '../src/core/items/item';
import { Program } from '../src/core/items/program';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { transl, mul, rotx, DEG } from '../src/core/math/pose';
import { compileForPost } from '../src/posts/base';
import { loadStation } from '../src/io/station-file';
import { postProcessRdkExport } from '../src/io/robodk/rdk_import';

const NC = `%
O1000 (pocket)
G21 G90 G17
T1 M6
S12000 M3
G0 X0 Y0 Z10
G1 Z-2 F300
G1 X40 F800
G1 Y30
G2 X30 Y40 I-10 J0
G1 X0
G1 Y0
G0 Z10
M5
G0 X60 Y0
M3
G1 Z-1 F200
G1 X80
G0 Z10
M5
M30
`;

const PRINT = `; 3D print
G21 G90 M82
G92 E0
G1 Z0.2 F3000
G1 X10 Y0 E0.5 F1200
G1 X10 Y10 E1.0
G1 E0.2 F2400 ; retract
G0 X20 Y20
G1 X30 Y20 E1.4 F1200
`;

describe('G-code parsing for machining', () => {
  it('extracts feeds, spindle state, tools and rapids', () => {
    const g = parseGcode(NC);
    const st = gcodeStats(g);
    expect(st.tools).toEqual([1]);
    expect(st.feeds).toEqual([300, 800, 200]);
    expect(st.cuts).toBeGreaterThanOrEqual(3);
    expect(st.rapids).toBeGreaterThanOrEqual(3);
    expect(st.cutLength).toBeGreaterThan(120);
    const curves = gcodeToMachiningCurves(g);
    expect(curves.some((c) => c.kind === 'rapid')).toBe(true);
    expect(curves.filter((c) => c.kind === 'cut').every((c) => c.spindle)).toBe(true);
  });
  it('detects extrusion moves and retractions', () => {
    const g = parseGcode(PRINT);
    const st = gcodeStats(g);
    expect(st.extruding).toBeCloseTo(30, 3);
    const curves = gcodeToMachiningCurves(g);
    expect(curves.filter((c) => c.extrude).length).toBeGreaterThanOrEqual(2);
    expect(curves.some((c) => c.kind === 'rapid')).toBe(true);
  });
});

describe('robot machining program generation', () => {
  it('builds a milling program with speeds, spindle IO, approach and retract that simulates', () => {
    const st = new Station('Mill');
    const r = createRobotFromLibrary('UR5e'); st.addChild(r);
    const tool = new Tool('Spindle'); tool.setPoseTool(mul(transl(0, 0, 150), rotx(0 * DEG))); r.addChild(tool); r.setTool(tool);
    const part = new SceneObject('Pocket'); part.setPose(transl(450, -150, 100)); part.curves = gcodeToMachiningCurves(parseGcode(NC)); st.addChild(part);
    r.setJoints([0, -90, 90, -90, -90, 0]);
    const res = generateMachining(st, r, part, { spindleIO: 'Spindle', approach: 30 });
    expect(res.unreachable).toBe(0);
    expect(res.points).toBeGreaterThan(10);
    expect(res.cutLength).toBeGreaterThan(120);
    const ins = res.program.instructions();
    const kinds = ins.map((i) => i.data.kind);
    expect(kinds.filter((k) => k === 'speed').length).toBeGreaterThanOrEqual(3); // 300, 800, 200 + rapid
    const ios = ins.filter((i) => i.data.kind === 'io').map((i) => (i.data as any).value);
    expect(ios[0]).toBe(true);
    expect(ios[ios.length - 1]).toBe(false);
    expect(ins[0].name).toBe('MoveJ');
    expect(ins.some((i) => i.data.kind === 'print')).toBe(true);
    const sim = new ProgramSimulator(st);
    const rr = sim.compile(res.program);
    expect(rr.problems.filter((p) => p.severity === 'error')).toEqual([]);
    expect(sim.duration).toBeGreaterThan(1);
    // cut at 800 mm/min = 13.3 mm/s -> the 40 mm line alone takes ~3 s
    expect(res.estimatedTime).toBeGreaterThan(10);
    const post = compileForPost(st, res.program);
    expect(post.events.filter((e) => e.kind === 'setDO').length).toBe(ios.length);
  });
});

describe('joint path replay instruction', () => {
  it('simulates a recorded joint path and exports it through posts', () => {
    const st = new Station('Replay');
    const r = createRobotFromLibrary('UR5e'); st.addChild(r);
    const p = st.addChild(new Program('Replay'));
    p.setRobot(r);
    const rows = Array.from({ length: 50 }, (_, i) => [i * 0.5, -90 + i * 0.2, 90, -90, -90, 0]);
    p.addInstruction({ kind: 'jointPath', joints: rows, source: 'test' });
    const sim = new ProgramSimulator(st);
    const res = sim.compile(p);
    expect(res.ok).toBe(true);
    expect(sim.duration).toBeGreaterThan(0.1);
    sim.seek(sim.duration);
    expect(r.joints()[0]).toBeCloseTo(24.5, 3);
    const post = compileForPost(st, p);
    expect(post.events.filter((e) => e.kind === 'moveJ').length).toBe(50);
  });
  it('is emitted by the RoboDK importer when the robot kinematics are unknown', () => {
    const data = {
      format: 'vbstation', version: 1, assets: {},
      station: { id: 's', type: 1, name: 'RDK', pose: Array.from(transl(0, 0, 0)), visible: true, params: { source: 'rdk_export.py' }, children: [
        { id: 'r', type: 2, name: 'Zorg', pose: Array.from(transl(0, 0, 0)), visible: true, params: { rdk_type: 2 }, children: [] },
        { id: 'p', type: 8, name: 'Prog', pose: Array.from(transl(0, 0, 0)), visible: true, params: { rdk_type: 8, robotName: 'Zorg', instructions: [{ name: 'T1', type: 1, moveType: 1, isJointTarget: true, pose: Array.from(transl(400, 0, 400)), joints: [0, 0, 0, 0, 0, 0] }], jointsList: [[0, 0, 0, 0, 0, 0], [5, 0, 0, 0, 0, 0], [10, 0, 0, 0, 0, 0]], jointsListStepMm: 10 }, children: [] },
      ] },
    };
    const st = loadStation(data as any);
    const rep = postProcessRdkExport(st);
    expect(rep.robots[0].source).toBe('none');
    const p = st.itemsOfType<Program>(ItemType.PROGRAM)[0];
    expect(p.instructions().map((i) => i.data.kind)).toEqual(['jointPath']);
  });
});
