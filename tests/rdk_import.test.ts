import { describe, it, expect } from 'vitest';
import { loadStation } from '../src/io/station-file';
import { isRdkExport, postProcessRdkExport } from '../src/io/robodk/rdk_import';
import { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { identity, transl, mul, rotx, DEG } from '../src/core/math/pose';

describe('rdk_export.py import', () => {
  it('rebuilds robots and programs from an rdk_export station', () => {
    const cols = (m: Float64Array) => Array.from(m);
    const data = {
      format: 'vbstation', version: 1, savedAt: '', app: 'rdk_export.py',
      station: {
        id: 'station', type: 1, name: 'RoboDK cell', pose: cols(identity()), visible: true, params: { source: 'rdk_export.py' }, children: [
          { id: 'f1', type: 3, name: 'Frame 2', pose: cols(transl(500, 0, 0)), visible: true, params: { rdk_type: 3 }, children: [
            { id: 't1', type: 6, name: 'Target 1', pose: cols(mul(transl(100, 0, 300), rotx(180 * DEG))), visible: true, params: { rdk_type: 6 }, children: [], joints: [0, -90, 90, -90, -90, 0], isJointTarget: false },
          ] },
          { id: 'r1', type: 2, name: 'UR10e', pose: cols(identity()), visible: true, params: { rdk_type: 2, library_hint: 'UR10e', joints: [10, -80, 90, -100, -90, 5], lower: [-360, -360, -360, -360, -360, -360], upper: [360, 360, 360, 360, 360, 360], activeFrame: 'Frame 2', activeTool: 'Tool 1' }, children: [
            { id: 'tl1', type: 4, name: 'Tool 1', pose: cols(transl(0, 0, 120)), visible: true, params: { rdk_type: 4 }, children: [] },
          ] },
          { id: 'p1', type: 8, name: 'MainProg', pose: cols(identity()), visible: true, params: { rdk_type: 8, robotName: 'UR10e', instructions: [
            { name: 'Home', type: 1, moveType: 1, isJointTarget: true, pose: cols(identity()), joints: [10, -80, 90, -100, -90, 5] },
            { name: 'Target 1', type: 1, moveType: 1, isJointTarget: false, pose: cols(mul(transl(100, 0, 300), rotx(180 * DEG))), joints: [] },
            { name: 'Target 1b', type: 1, moveType: 2, isJointTarget: false, pose: cols(mul(transl(100, 0, 250), rotx(180 * DEG))), joints: [] },
            { name: 'Pause', type: 7, pauseMs: 500 },
          ] }, children: [] },
        ],
        settings: {}, format: 'vbs-station',
      },
      assets: {},
    };
    const st = loadStation(data as any);
    expect(isRdkExport(st)).toBe(true);
    const rep = postProcessRdkExport(st);
    expect(rep.robots[0].source).toBe('library');
    expect(rep.robots[0].matched).toBe('UR10e');
    const robot = st.find('UR10e') as Robot;
    expect(robot.dof).toBe(6);
    expect(robot.joints()[0]).toBe(10);
    expect(robot.activeFrame()?.name).toBe('Frame 2');
    expect(robot.activeTool()?.name).toBe('Tool 1');
    const prog = st.find('MainProg') as Program;
    expect(prog.instructions().length).toBe(4);
    expect(prog.instructions().map((i) => (i.data as any).moveType ?? i.data.kind)).toEqual(['MoveJ', 'MoveJ', 'MoveL', 'pause']);
    const res = new ProgramSimulator(st).compile(prog);
    expect(res.problems.filter((p) => p.severity === 'error')).toEqual([]);
  });
});
