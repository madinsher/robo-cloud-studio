import { describe, it, expect } from 'vitest';
import { existsSync, mkdtempSync, readFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { spawnSync } from 'node:child_process';
import { loadStation } from '../src/io/station-file';
import { isRdkExport, postProcessRdkExport, importRdkExport } from '../src/io/robodk/rdk_import';
import { importRdkBestEffort } from '../src/io/robodk/rdk_container';
import { AssetStore } from '../src/scene/assets';
import { Station } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { identity, transl, mul, rotx, DEG, getPos } from '../src/core/math/pose';

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

describe('rdk_export.py import: instruction mapping and library matching (API walk)', () => {
  const cols = (m: Float64Array) => Array.from(m);
  const node = (id: string, type: number, name: string, extra: Record<string, unknown> = {}, children: unknown[] = []) => ({ id, type, name, pose: cols(identity()), visible: true, params: { rdk_type: type, ...((extra.params as object) ?? {}) }, children, ...Object.fromEntries(Object.entries(extra).filter(([k]) => k !== 'params')) });
  const station = (children: unknown[]) => loadStation({ format: 'vbstation', version: 1, savedAt: '', app: 'rdk_export.py', station: { ...node('station', 1, 'Cell', { params: { source: 'rdk_export.py' } }, children), settings: {}, format: 'vbs-station' }, assets: {} } as any);

  it('maps every official INS_TYPE (RoboDK labels) to the studio instruction', () => {
    const st = station([
      node('f2', 3, 'Frame 2', {}, [
        node('t2', 6, 'Target 2', { pose: cols(mul(transl(100, 0, 0), rotx(180 * DEG))), joints: [0, -90, 90, -90, -90, 0], isJointTarget: false }),
        node('t3', 6, 'Target 3', { pose: cols(mul(transl(100, 100, 0), rotx(180 * DEG))), joints: [0, -90, 90, -90, -90, 0], isJointTarget: false }),
      ]),
      node('r1', 2, 'UR10e', { params: { library_hint: 'UR10e', joints: [0, -90, 90, -90, -90, 0], lower: [-360, -360, -360, -360, -360, -360], upper: [360, 360, 360, 360, 360, 360] } }, [node('tl', 4, 'Gun', { pose: cols(transl(0, 0, 100)) })]),
      node('sub', 8, 'Sub', { params: { robotName: 'UR10e', instructions: [] } }),
      node('p1', 8, 'Main', { params: { robotName: 'UR10e', instructions: [
        { name: 'Set Ref.: Frame 2', insType: 3 }, { name: 'Set Tool: Gun', insType: 4 },
        { name: 'Set speed (150.0 mm/s)', insType: 2 }, { name: 'Set speed (45.0 deg/s)', insType: 2 }, { name: 'Set accel. (200.0 deg/s2)', insType: 2 }, { name: 'Set accel. (800.0 mm/s2)', insType: 2 },
        { name: 'Smooth(5)', insType: 10 }, { name: 'Accurate', insType: 10 },
        { name: 'MoveL (Target 2)', insType: 0, moveType: 2, isJointTarget: false, pose: cols(identity()), joints: [] },
        { name: 'MoveC (Target 2, Target 3)', insType: 1, moveType: null, isJointTarget: false },
        { name: 'Pause 500 ms', insType: 6 }, { name: 'Pause 2 s', insType: 6 }, { name: 'Stop', insType: 6 },
        { name: 'Set DO_5=1', insType: 11 }, { name: 'Wait DI_3=1 T2.0s', insType: 11 },
        { name: 'Sub', insType: 8 }, { name: 'SetRPM(70)', insType: 8 }, { name: 'Print: hello', insType: 9 },
      ] } }),
    ]);
    postProcessRdkExport(st);
    const f2 = st.find('Frame 2')!, gun = st.find('Gun')!, sub = st.find('Sub')!;
    const t2 = st.find('Target 2')!, t3 = st.find('Target 3')!;
    const d = (st.find('Main') as Program).instructions().map((i) => i.data);
    expect(d).toEqual([
      { kind: 'frame', frameId: f2.id }, { kind: 'tool', toolId: gun.id },
      { kind: 'speed', speedLinear: 150 }, { kind: 'speed', speedJoints: 45 }, { kind: 'speed', accelJoints: 200 }, { kind: 'speed', accelLinear: 800 },
      { kind: 'rounding', radius: 5 }, { kind: 'rounding', radius: -1 },
      { kind: 'move', moveType: 'MoveL', targetId: t2.id },
      { kind: 'move', moveType: 'MoveC', targetId: t3.id, viaTargetId: t2.id },
      { kind: 'pause', timeMs: 500 }, { kind: 'pause', timeMs: 2000 }, { kind: 'pause', timeMs: -1 },
      { kind: 'io', io: 'DO_5', value: 1, wait: false }, { kind: 'io', io: 'DI_3', value: 1, wait: true, timeoutMs: 2000 },
      { kind: 'call', programId: sub.id, programName: 'Sub' }, { kind: 'code', code: 'SetRPM(70)', asFunctionCall: true }, { kind: 'print', message: 'hello', isComment: false },
    ]);
  });

  it('never gives a robot the kinematics of another brand, model number or axis count', () => {
    const robots = [['Fanuc M-710iC/50', 6], ['KUKA LBR iiwa 14 R820', 7], ['RobotiQ 2F85', 1], ['ABB IRBT 6004 Standard 3.5m', 1], ['KUKA KR 6 R900 sixx', 6], ['ABB IRB 120-3/0.6', 6]] as const;
    const st = station(robots.map(([n, dof], i) => node(`r${i}`, 2, n, { params: { library_hint: n, joints: new Array(dof).fill(0), lower: new Array(dof).fill(-100), upper: new Array(dof).fill(100) } })));
    const rep = postProcessRdkExport(st);
    expect(rep.robots.map((r) => [r.name, r.matched])).toEqual([['Fanuc M-710iC/50', null], ['KUKA LBR iiwa 14 R820', null], ['RobotiQ 2F85', null], ['ABB IRBT 6004 Standard 3.5m', null], ['KUKA KR 6 R900 sixx', 'KUKA_KR6_R900'], ['ABB IRB 120-3/0.6', 'ABB_IRB120']]);
  });
});

// Lossless path: RoboDK itself re-saves the station (python/rdk2vbs.py, as POST /convert/rdk runs it) and the studio
// decodes the embedded file. Starts a hidden RoboDK on a free port 21500+ and closes it.
const RDK_PY = 'C:/RoboDK/Python37/python.exe', LIB_01A = 'C:/RoboDK/Library/Example-01.a-Robot Painting - Universal Robots.rdk';
describe('lossless conversion through RoboDK (rdk2vbs.py)', () => {
  it.skipIf(!existsSync(RDK_PY) || !existsSync(LIB_01A))('converts Example 01.a with RoboDK and decodes it exactly like the direct import', async () => {
    const dir = mkdtempSync(join(tmpdir(), 't1-rdk2vbs-'));
    try {
      const out = join(dir, 'out.vbstation');
      const r = spawnSync(RDK_PY, ['-I', 'python/rdk2vbs.py', LIB_01A, out], { encoding: 'utf8', timeout: 240_000 });
      expect(r.status, r.stdout + '\n' + r.stderr).toBe(0);
      expect(r.stdout).toMatch(/port 21[5-9]\d\d/);
      const data = JSON.parse(readFileSync(out, 'utf8'));
      // API walk: the robot is exported at its base (not its TCP) under its exact RoboDK name
      const walkRobot = data.station.children.find((c: any) => c.name === 'UR10 Base').children.find((c: any) => c.type === 2);
      expect(walkRobot.name).toBe('UR10');
      expect(walkRobot.pose).toEqual(Array.from(identity()));
      const assets = new AssetStore();
      const conv = await importRdkExport(data, assets);
      expect(conv).not.toBeNull();
      expect(conv!.report.fidelity, JSON.stringify(conv!.report.dropped)).toBe('full');
      expect(conv!.report.notes[0]).toMatch(/^Converted by RoboDK 5\.9/);
      const b = readFileSync(LIB_01A);
      const direct = await importRdkBestEffort(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength), 'x.rdk', new AssetStore());
      const sig = (s: Station) => [...s.walk()].map((i) => [i.type, i.name, getPos(i.poseAbs()).map((v) => v.toFixed(3)).join(','), i instanceof Program ? i.instructions().map((x) => x.data.kind).join(',') : ''].join('|'));
      expect(sig(conv!.station)).toEqual(sig(direct.station));
      const robot = conv!.station.find('UR10') as Robot;
      expect(robot.chain.links.every((l) => l.visuals.length > 0 && l.visuals.every((v) => (assets.get(v.mesh!)?.mesh?.triangles ?? 0) > 0))).toBe(true);
    } finally { rmSync(dir, { recursive: true, force: true }); }
  }, 300_000);
});
