import { describe, it, expect, vi, afterEach } from 'vitest';
import { detectLanguage, importProgram } from '../src/io/programs/import';
import { parseGcode, gcodeToCurves, gcodeToMachiningCurves, gcodeStats } from '../src/io/programs/gcode';
import { parseTargetsText, importTargets, exportTargets } from '../src/io/robodk/targets';
import { parseDHText, robotFromDH, robotToDHText } from '../src/io/robodk/dh';
import { stationToRoboDKScript } from '../src/io/robodk/station_script';
import { fetchOnlineRobot, fetchRobotFromUrl, fetchPackageMeshes, ONLINE_ROBOT_LIBRARY, ONLINE_REPOS, rawBase } from '../src/io/library/online_library';
import { AssetStore } from '../src/scene/assets';
import { writeBinarySTL } from '../src/io/mesh/stl';
import { Station, Frame, Target, Tool, SceneObject, Folder } from '../src/core/items/item';
import { Program } from '../src/core/items/program';
import { createRobotFromLibrary } from '../src/core/items/library';
import { transl, mul, rotx, rotz, DEG, getPos, poseToXyzrpw, poseToKuka, distance, identity } from '../src/core/math/pose';

afterEach(() => vi.unstubAllGlobals());

const kinds = (p: Program) => p.instructions().map((i) => (i.data.kind === 'move' ? i.data.moveType : i.data.kind));

describe('program import', () => {
  it('detects languages from extension and content', () => {
    expect(detectLanguage('', 'a.src')).toBe('krl');
    expect(detectLanguage('DEF main( )\nPTP HOME\nEND')).toBe('krl');
    expect(detectLanguage('', 'a.mod')).toBe('rapid');
    expect(detectLanguage('MODULE x\nENDMODULE')).toBe('rapid');
    expect(detectLanguage('/PROG X\n/MN')).toBe('ls');
    expect(detectLanguage('movej([0,0,0,0,0,0])')).toBe('urscript');
    expect(detectLanguage('', 'p.csv')).toBe('csv');
    expect(detectLanguage('hello')).toBe('auto');
  });
  it('imports KRL with axis/frame targets, CIRC, speed, waits, IO and calls', () => {
    const st = new Station();
    const robot = st.addChild(createRobotFromLibrary('KUKA_KR6_R900'));
    const base = st.addChild(new Frame('Base'));
    robot.setFrame(base);
    const src = `&ACCESS RVP
DEF demo( )
;FOLD INI
BAS (#INITMOV,0 )
;ENDFOLD (INI)
; a comment
$VEL.CP = 0.25
$APO.CDIS = 3
PTP {A1 10,A2 -90,A3 90,A4 0,A5 45,A6 0} ; home
PTP {X 500,Y 0,Z 400,A 0,B 90,C 0}
LIN {X 500,Y 100,Z 400,A 0,B 90,C 0} ; lin1
CIRC {X 550,Y 150,Z 400,A 0,B 90,C 0}, {X 600,Y 100,Z 400,A 0,B 90,C 0} ; arc
PTP HOME
WAIT SEC 1.5
$OUT[3] = TRUE
WAIT FOR $IN[7] == FALSE
HALT
MYSUB ( )
$BWDSTART = FALSE
END`;
    const res = importProgram(src, { station: st, robot, filename: 'demo.src' });
    expect(res.program.name).toBe('demo');
    expect(res.program.robot()).toBe(robot);
    expect(res.warnings).toContain('CIRC imported as two MoveL');
    expect(kinds(res.program)).toEqual(['print', 'speed', 'rounding', 'MoveJ', 'MoveJ', 'MoveL', 'MoveL', 'MoveL', 'code', 'pause', 'io', 'io', 'pause', 'code', 'code']);
    expect(res.targets.length).toBe(5);
    expect(res.targets[0].name).toBe('home');
    expect(res.targets[0].isJointTarget).toBe(true);
    expect(res.targets[0].joints).toEqual([10, -90, 90, 0, 45, 0]);
    expect(res.targets[0].parent).toBe(base);
    expect(getPos(res.targets[2].pose())).toEqual([500, 100, 400]);
    expect(res.targets[3].name).toBe('arc_via');
    const d = res.program.instructions().map((i) => i.data as any);
    expect(d[1].speedLinear).toBe(250);
    expect(d[2].radius).toBe(3);
    expect(d[9].timeMs).toBe(1500);
    expect(d[10]).toMatchObject({ io: '3', value: true, wait: false });
    expect(d[11]).toMatchObject({ io: '7', value: false, wait: true });
    expect(d[12].timeMs).toBe(-1);
    expect(d[13]).toMatchObject({ code: 'MYSUB', asFunctionCall: true });
    expect(d[14]).toMatchObject({ code: '$BWDSTART = FALSE', asFunctionCall: false });
  });
  it('imports RAPID with named targets, MoveC, MoveAbsJ, IO and messages', () => {
    const st = new Station();
    const robot = st.addChild(createRobotFromLibrary('ABB_IRB120'));
    const mod = `MODULE Mod1
  CONST robtarget pHome:=[[300,0,400],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
  CONST robtarget pVia:=[[350,50,400],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
  CONST jointtarget jHome:=[[0,0,0,0,90,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
  PROC main()
    ConfL \\Off;
    ! comment here
    MoveAbsJ jHome,v1000,fine,tool0;
    MoveJ pHome,v500,z10,tool0\\WObj:=wobj0;
    MoveL [[300,100,400],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],v200,fine,tool0;  ! inline
    MoveC pVia,pHome,v100,fine,tool0;
    WaitTime 0.5;
    SetDO do1, 1;
    WaitDI di2, 0;
    TPWrite "Done";
    Stop;
    Sub1;
    x := 1;
  ENDPROC
ENDMODULE`;
    const res = importProgram(mod, { station: st, robot, language: 'rapid', name: 'Mod' });
    expect(res.warnings).toContain('MoveC imported as two MoveL');
    expect(kinds(res.program)).toEqual(['print', 'speed', 'MoveJ', 'speed', 'MoveJ', 'speed', 'MoveL', 'speed', 'MoveL', 'MoveL', 'pause', 'io', 'io', 'print', 'pause', 'code', 'code']);
    expect(res.targets[0].joints).toEqual([0, 0, 0, 0, 90, 0]);
    expect(res.targets[0].name).toBe('jHome');
    expect(res.targets[1].name).toBe('pHome');
    expect(getPos(res.targets[1].pose())).toEqual([300, 0, 400]);
    expect(res.targets[2].name).toBe('inline');
    expect(res.targets[3].name).toBe('pVia_via');
    const d = res.program.instructions().map((i) => i.data as any);
    expect(d[1].speedLinear).toBe(1000);
    expect(d[10].timeMs).toBe(500);
    expect(d[13]).toMatchObject({ message: 'Done', isComment: false });
    expect(d[15]).toMatchObject({ code: 'Sub1', asFunctionCall: true });
    expect(d[16]).toMatchObject({ code: 'x := 1', asFunctionCall: false });
    // targets go to the station when the robot has no active frame
    expect(res.targets[0].parent).toBe(st);
  });
  it('imports Fanuc LS positions (joint + cartesian) and TP instructions', () => {
    const st = new Station();
    const robot = st.addChild(createRobotFromLibrary('FANUC_LRMATE200iD'));
    const ls = `/PROG  DEMO
/ATTR
/MN
   1:  UFRAME_NUM=1 ;
   2:  UTOOL_NUM=1 ;
   3:J P[1] 100% FINE ;
   4:L P[2:"pick"] 500mm/sec CNT50 ;
   5:L P[3] 200mm/sec FINE ;
   6:  WAIT 2.00(sec) ;
   7:  DO[5]=ON ;
   8:  WAIT DI[6]=OFF ;
   9:  CALL SUB2 ;
  10:  !note ;
  11:  PAUSE ;
  12:  R[1]=0 ;
/POS
P[1]{
   GP1:
	UF : 1, UT : 1,
	J1=  10.000 deg,	J2=  20.000 deg,	J3=  -5.000 deg,
	J4=  0.000 deg,	J5=  -60.000 deg,	J6=  0.000 deg
};
P[2:"pick"]{
   GP1:
	UF : 1, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =  400.000  mm,	Y =  50.000  mm,	Z =  300.000  mm,
	W =  180.000 deg,	P =  0.000 deg,	R =  0.000 deg
};
/END`;
    const res = importProgram(ls, { station: st, robot, filename: 'DEMO.LS' });
    expect(res.warnings).toEqual(['P[3] not found']);
    // FINE / CNT termination is emitted as a rounding instruction before the move it belongs to
    expect(kinds(res.program)).toEqual(['print', 'print', 'rounding', 'MoveJ', 'speed', 'rounding', 'MoveL', 'pause', 'io', 'io', 'code', 'print', 'pause', 'code']);
    expect(res.targets[0].joints).toEqual([10, 20, -5, 0, -60, 0]);
    expect(res.targets[1].name).toBe('pick');
    expect(getPos(res.targets[1].pose())).toEqual([400, 50, 300]);
    const d = res.program.instructions().map((i) => i.data as any);
    expect(d[2].radius).toBe(-1);
    expect(d[4].speedLinear).toBe(500);
    expect(d[5].radius).toBe(50);
    expect(d[7].timeMs).toBe(2000);
    expect(d[8]).toMatchObject({ io: '5', value: true });
    expect(d[9]).toMatchObject({ io: '6', value: false, wait: true });
    expect(d[10]).toMatchObject({ code: 'SUB2', asFunctionCall: true });
    expect(d[11].message).toBe('note');
    expect(d[12].timeMs).toBe(-1);
    expect(d[13]).toMatchObject({ code: 'R[1]=0', asFunctionCall: false });
  });
  it('imports URScript (pose + joint movej, movec, io, messages) and CSV', () => {
    const st = new Station();
    const robot = st.addChild(createRobotFromLibrary('UR5e'));
    const script = `def prog():
  set_tcp(p[0,0,0.1,0,0,0])  # tcp
  movej([0, -1.5708, 1.5708, -1.5708, -1.5708, 0], a=1.4, v=1.05)  # home
  movej(p[0.4, 0.1, 0.3, 3.1416, 0, 0], a=1.2, v=0.25)
  movel(p[0.4, 0.2, 0.3, 3.1416, 0, 0], a=1.2, v=0.1)  # lin
  movec(p[0.45, 0.25, 0.3, 3.1416, 0, 0], p[0.5, 0.2, 0.3, 3.1416, 0, 0], a=1.2, v=0.1)
  sleep(0.25)
  set_digital_out(2, True)
  set_standard_digital_out(3, False)
  textmsg("hi")
  # only a comment
  mysub()
  popup("x")
end
prog()`;
    const res = importProgram(script, { station: st, robot, filename: 'prog.script' });
    // bare `name()` lines (program entry calls) are skipped by design
    expect(kinds(res.program)).toEqual(['print', 'speed', 'MoveJ', 'speed', 'MoveJ', 'speed', 'MoveL', 'speed', 'MoveL', 'MoveL', 'pause', 'io', 'io', 'print', 'print', 'code']);
    expect(res.targets[0].joints!.map((v) => Math.round(v))).toEqual([0, -90, 90, -90, -90, 0]);
    expect(res.targets[0].isJointTarget).toBe(true);
    expect(getPos(res.targets[1].pose()).map((v) => +v.toFixed(3))).toEqual([400, 100, 300]);
    const d = res.program.instructions().map((i) => i.data as any);
    expect(d[0].message).toBe('set_tcp tcp');
    expect(d[1].speedLinear).toBe(1050);
    expect(d[10].timeMs).toBe(250);
    expect(d[11]).toMatchObject({ io: '2', value: true });
    expect(d[12]).toMatchObject({ io: '3', value: false });
    expect(d[13].message).toBe('hi');
    expect(d[14]).toMatchObject({ message: 'only a comment', isComment: true });
    expect(d[15]).toMatchObject({ code: 'popup("x")', asFunctionCall: false });

    const csv = `type,name,x,y,z,rx,ry,rz,j1,j2,j3,j4,j5,j6\nmoveJ,A,100,0,200,0,180,0,1,2,3,4,5,6\nmoveL,B,150,0,200,0,180,0\nmoveC,C,x,0,200,0,180,0,1,2,3,4,5,6\n# c\nspeed,1\n`;
    const r2 = importProgram(csv, { station: st, language: 'csv', name: 'csv' });
    expect(kinds(r2.program)).toEqual(['MoveJ', 'MoveL', 'MoveL']);
    expect(r2.targets[0].joints).toEqual([1, 2, 3, 4, 5, 6]);
    expect(r2.targets[0].isJointTarget).toBe(false);
    expect(getPos(r2.targets[1].pose())).toEqual([150, 0, 200]);
    expect(r2.targets[2].isJointTarget).toBe(true); // bad pose -> joints only
    expect(r2.program.robot()).toBeNull();
    const r3 = importProgram('foo\n\nbar', { station: st, name: 'raw' });
    expect(r3.warnings).toEqual(['Unknown language; imported as raw code']);
    expect(r3.program.instructions().map((i) => (i.data as any).code)).toEqual(['foo', 'bar']);
  });
});

describe('G-code', () => {
  it('handles inch units, incremental mode, arcs in all planes, R arcs, tools and extrusion', () => {
    const g = parseGcode(`%\nO1000\nN10 G20 G17 G90 (inch)\nN20 T3 M6\nG0 X1 Y0 Z0.5\nM3 S1000\nG1 Z0 F10\nG2 X0 Y1 I-1 J0\nG21\nG91\nG1 X10 Y0 F200\nG18\nG3 X10 Z10 I5 K5\nG19\nG2 Y10 Z10 R5\nG90\nM5\nG0 Z100\nG28\n; comment only\nM30`, { arcStep: 1 });
    expect(g.units).toBe('mm');
    expect(g.segments[0].kind).toBe('rapid');
    expect(g.segments[0].tool).toBe(3);
    expect(g.segments[0].points[1]).toEqual([25.4, 0, 12.7]);
    const cut = g.segments.find((s) => s.kind === 'cut')!;
    expect(cut.feed).toBe(10);
    expect(cut.spindle).toBe(true);
    // the G2 quarter circle in inches ends at (0, 25.4)
    const arcEnd = g.segments.filter((s) => s.kind === 'cut').flatMap((s) => s.points).find((p) => Math.abs(p[0]) < 1e-6 && Math.abs(p[1] - 25.4) < 1e-6);
    expect(arcEnd).toBeTruthy();
    expect(g.length).toBeGreaterThan(25.4 * Math.PI / 2 + 10 + 10);
    expect(g.bounds.max[2]).toBe(100);
    const st = gcodeStats(g);
    expect(st.tools).toEqual([3]);
    expect(st.feeds).toEqual(expect.arrayContaining([10, 200]));
    expect(st.rapids).toBeGreaterThan(0);
    expect(st.cuts).toBeGreaterThan(0);
    expect(st.cutLength + st.rapidLength).toBeCloseTo(g.length, 6);
    // 3D printing: relative E moves extrude, retractions do not
    const p = parseGcode(`G21 G90\nM83\nG1 X0 Y0 Z0.2 F1200\nG1 X10 E0.5\nG1 E-1\nG1 X20 E0.5\nM82\nG92 E0\nG1 X30 E1\nG1 X40 E0.5\nG1 X50`);
    const ext = p.segments.filter((s) => s.extrude);
    // the retraction (E only) does not break the extruding segment; X40 with decreasing E starts a travel segment
    expect(ext.length).toBe(1);
    expect(ext[0].points.map((q) => q[0])).toEqual([0, 10, 20, 30]);
    const travel = p.segments.filter((s) => s.kind === 'cut' && !s.extrude);
    expect(travel.at(-1)!.points.map((q) => q[0])).toEqual([30, 40, 50]);
    expect(gcodeStats(p).extruding).toBeGreaterThan(0);
    const mc = gcodeToMachiningCurves(p);
    expect(mc.some((c) => c.name.endsWith(' E'))).toBe(true);
    expect(mc.every((c) => c.points.length >= 2)).toBe(true);
  });
  it('merges consecutive cuts into curves and skips rapids', () => {
    const g = parseGcode(`G0 X0 Y0\nG1 X10 F100\nG1 X20 F200\nG0 X30\nG1 X40 F200\nG1 Y0`);
    const curves = gcodeToCurves(g);
    expect(curves.length).toBe(2);
    expect(curves[0].name).toBe('cut 1 F100');
    expect(curves[0].points).toEqual([[0, 0, 0], [10, 0, 0], [20, 0, 0]]);
    expect(curves[1].points[0]).toEqual([30, 0, 0]);
    expect(gcodeToMachiningCurves(g).map((c) => c.kind)).toEqual(['rapid', 'cut', 'cut', 'rapid', 'cut']);
  });
});

describe('RoboDK targets', () => {
  it('parses text with names, comments and separators; imports all formats; exports back', () => {
    const rows = parseTargetsText(`# header\n% also\n// and\nP1, 100, 200, 300, 0, 180, 0\n400;0;300;0;90;0\n1\t2\t3\nbad line\n10 20 30 0 0 0 1 2 3 4 5 6\n`);
    expect(rows.length).toBe(4);
    expect(rows[0].name).toBe('P1');
    expect(rows[3].values.length).toBe(12);
    const st = new Station();
    const f = st.addChild(new Frame('F'));
    const ts = importTargets(`P1, 100, 200, 300, 0, 180, 0\n400;0;300;0;90;0\n1 2 3\n10 20 30 0 0 0 1 2 3 4 5 6`, { parent: f, namePrefix: 'T' });
    expect(ts.map((t) => t.name)).toEqual(['P1', 'T 2', 'T 3', 'T 4']);
    expect(ts.every((t) => t.parent === f)).toBe(true);
    expect(getPos(ts[0].pose())).toEqual([100, 200, 300]);
    // rotation 180° about Y: X and Z axes flipped (Euler decomposition may return an equivalent triple)
    expect(ts[0].pose()[0]).toBeCloseTo(-1, 9);
    expect(ts[0].pose()[10]).toBeCloseTo(-1, 9);
    expect(poseToXyzrpw(ts[0].pose()).slice(0, 3)).toEqual([100, 200, 300]);
    // 3 columns: point with tool Z pointing down
    expect(getPos(ts[2].pose())).toEqual([1, 2, 3]);
    expect(ts[2].pose()[10]).toBeCloseTo(-1, 9);
    expect(ts[3].joints).toEqual([1, 2, 3, 4, 5, 6]);
    const approach = mul(rotz(45 * DEG), rotx(180 * DEG));
    const pts = importTargets('5 6 7', { parent: f, pointApproachPose: approach });
    expect(distance(getPos(pts[0].pose()), [5, 6, 7])).toBeLessThan(1e-9);
    expect(pts[0].pose()[0]).toBeCloseTo(Math.cos(45 * DEG), 9);
    const kuka = importTargets('100 0 0 90 0 0', { parent: f, format: 'kuka' })[0];
    expect(poseToKuka(kuka.pose()).map((v) => +v.toFixed(3))).toEqual([100, 0, 0, 90, 0, 0]);
    const fanuc = importTargets('100 0 0 0 0 90', { parent: f, format: 'fanuc' })[0];
    expect(poseToKuka(fanuc.pose()).map((v) => +v.toFixed(3))).toEqual([100, 0, 0, 90, 0, 0]);
    const out = exportTargets(ts, 'xyzrpw', true);
    const lines = out.trim().split('\n');
    expect(lines[0]).toMatch(/^# name,x,y,z,rx,ry,rz,j1/);
    expect(lines[1]).toMatch(/^P1,100.000,200.000,300.000,(0.000,180.000,0.000|-180.000,0.000,-180.000)$/); // equivalent Euler triples
    expect(lines[4]).toBe('T_4,10.000,20.000,30.000,0.000,0.000,0.000,1.000,2.000,3.000,4.000,5.000,6.000');
    expect(exportTargets([kuka], 'kuka', false).split('\n')[1]).toBe('Target_1,100.000,0.000,0.000,90.000,0.000,0.000');
    expect(exportTargets([fanuc], 'fanuc').split('\n')[1]).toMatch(/^Target_1,100.000,0.000,0.000,0.000,0.000,90.000$/);
    // round trip: re-import the export
    const again = importTargets(out, { parent: f });
    expect(again.length).toBe(4);
    expect(again[3].joints).toEqual([1, 2, 3, 4, 5, 6]);
  });
});

describe('DH robot definitions', () => {
  it('parses text tables with names, JSON specs, prismatic joints and flange', () => {
    const spec = parseDHText(`# name: My SCARA\n# theta, d, a, alpha\n0, 200, 250, 0\n0, 0, 150, 180, -150, 150\n0 0 0 0 -200 0 1 -50\n0 100 0 0`);
    expect(spec.name).toBe('My SCARA');
    expect(spec.dh.length).toBe(4);
    const r = robotFromDH(spec);
    expect(r.name).toBe('My SCARA');
    expect(r.dof).toBe(4);
    expect(r.chain.dh![2].type).toBe('prismatic');
    expect(r.chain.dh![2].home).toBe(-50);
    expect(r.chain.dh![1].lower).toBe(-150);
    expect(r.chain.dh![0].lower).toBe(-180);
    r.setJoints([0, 0, -100, 0]);
    expect(getPos(r.solveFKFlange())).toEqual([400, 0, 200]);
    const json = robotFromDH(JSON.parse(JSON.stringify(parseDHText(JSON.stringify({ name: 'J', brand: 'Fanuc', post: 'Fanuc_R30iA', flange: [0, 0, 50], dh: [{ theta: 0, d: 100, a: 0, alpha: 0 }, [0, 0, 300, 0]] })))), 'fallback');
    expect(json.name).toBe('J');
    expect(json.brand).toBe('Fanuc');
    expect(json.postProcessor).toBe('Fanuc_R30iA');
    expect(json.params.source).toBe('dh');
    expect(getPos(json.solveFKFlange())).toEqual([300, 0, 150]);
    expect(robotFromDH({ dh: [[0, 0, 100, 0]] }).name).toBe('DH robot');
    expect(robotFromDH({ dh: [[0, 0, 100, 0]] }, 'X').name).toBe('X');
    const text = robotToDHText(r)!;
    expect(text.split('\n')[2]).toBe('0, 200, 250, 0, -180, 180, 0, 0');
    expect(text.split('\n')[4]).toBe('0, 0, 0, 0, -200, 0, 1, -50');
    expect(robotToDHText(robotFromDH(parseDHText(text)))).toBe(text);
    const urdfBot = createRobotFromLibrary('UR5e');
    (urdfBot.chain as any).dh = undefined;
    expect(robotToDHText(urdfBot)).toBeNull();
  });
});

describe('RoboDK station script', () => {
  it('emits every item type, hidden items and embedded programs', () => {
    const st = new Station('Cell "A"');
    const robot = st.addChild(createRobotFromLibrary('UR10e', 'UR10e'));
    robot.setPose(transl(0, 0, 500));
    const tool = robot.addChild(new Tool('Gripper'));
    tool.setPoseTool(transl(0, 0, 100));
    robot.setTool(tool);
    const frame = st.addChild(new Frame('Table'));
    frame.setPose(transl(600, 0, 0));
    robot.setFrame(frame);
    const t1 = frame.addChild(new Target('T1'));
    t1.setPose(mul(transl(0, 0, 300), rotx(180 * DEG)));
    const tj = frame.addChild(new Target('J1'));
    tj.setJoints([0, -90, 90, -90, -90, 0]);
    tj.setAsJointTarget();
    const obj = frame.addChild(new SceneObject('Part 1'));
    obj.setVisible(false);
    const folder = st.addChild(new Folder('Extras'));
    const cam = folder.addChild(new Frame('Cam'));
    void cam;
    const notes = st.addChild(new (Object.getPrototypeOf(frame).constructor as any)('Generic'));
    (notes as any).type = 99;
    const prog = st.addChild(new Program('Main'));
    prog.setRobot(robot);
    prog.addMoveJ(tj);
    prog.addMoveL(t1);
    prog.pause(100);
    const script = stationToRoboDKScript(st);
    expect(script).toContain('station = RDK.AddStation("Cell \\"A\\"")');
    expect(script).toContain('it1 = lib_robot("Universal Robots UR10e", station)');
    expect(script).toContain('it1.setJoints([');
    expect(script).toMatch(/it2 = it1\.AddTool\(Mat\(\[\[1,0,0,0\],\[0,1,0,0\],\[0,0,1,100\],\[0,0,0,1\]\]\), "Gripper"\) if it1\.Type\(\) == ITEM_TYPE_ROBOT else RDK\.AddFrame\("Gripper", it1\)/);
    expect(script).toContain('it3 = RDK.AddFrame("Table", station)');
    expect(script).toContain('it4 = RDK.AddTarget("T1", it3)');
    expect(script).toContain('it4.setAsCartesianTarget()');
    expect(script).toContain('it5.setJoints([0,-90,90,-90,-90,0])');
    expect(script).toContain('it5.setAsJointTarget()');
    expect(script).toContain('"Part_1.stl"');
    expect(script).toContain('it6.setVisible(False)');
    expect(script).toContain('it7 = RDK.AddFolder("Extras", station)');
    expect(script).toContain('it8 = RDK.AddFrame("Cam", it7)');
    expect(script).toContain('it9 = RDK.AddFrame("Generic", station)');
    expect(script).toContain('# ---- Program Main ----');
    expect(script).toContain('prog.MoveJ(t1)');
    expect(script).toContain('prog.MoveL(t2)');
    expect(script).toContain('prog.Pause(100)');
    // program instructions are not emitted as station items, and the embedded program header is stripped
    expect(script).not.toContain('"Pause 100 ms"');
    expect(script).not.toContain('RDK.AddFrame("MoveJ"');
    expect((script.match(/from robodk\.robolink import/g) ?? []).length).toBe(1);
    expect(script.trim().endsWith('.rdk file")')).toBe(true);
  });
});

describe('online robot library (mocked network)', () => {
  const repo = ONLINE_REPOS.find((r) => r.id === 'ur')!;
  const base = rawBase(repo);
  const files: Record<string, string> = {
    [`${base}ur_description/urdf/ur5.xacro`]: `<?xml version="1.0"?><robot name="ur5" xmlns:xacro="http://wiki.ros.org/xacro">
      <xacro:include filename="$(find ur_description)/urdf/ur5.urdf.xacro"/>
      <xacro:include filename="\${dynamic}.xacro"/>
      <xacro:ur5_robot prefix="\$(arg prefix)"/></robot>`,
    [`${base}ur_description/urdf/ur5.urdf.xacro`]: `<robot xmlns:xacro="http://wiki.ros.org/xacro"><xacro:include filename="materials.xacro"/><xacro:macro name="ur5_robot" params="prefix">
      <link name="\${prefix}base_link"><visual><geometry><mesh filename="package://ur_description/meshes/ur5/visual/base.dae"/></geometry></visual></link>
      <link name="\${prefix}shoulder_link"><visual><geometry><mesh filename="package://ur_description/meshes/ur5/visual/shoulder.stl"/></geometry></visual></link>
      <link name="\${prefix}upper_arm_link"><visual><geometry><mesh filename="package://ur_description/meshes/ur5/visual/upperarm.obj"/></geometry></visual></link>
      <link name="\${prefix}forearm_link"><visual><geometry><mesh filename="package://ur_description/meshes/ur5/visual/forearm.ply"/></geometry></visual></link>
      <link name="\${prefix}wrist_link"><visual><geometry><mesh filename="package://ur_description/meshes/ur5/visual/missing.stl"/></geometry></visual></link>
      <link name="\${prefix}tool0"/>
      <joint name="\${prefix}shoulder_pan_joint" type="revolute"><parent link="\${prefix}base_link"/><child link="\${prefix}shoulder_link"/><origin xyz="0 0 0.089"/><axis xyz="0 0 1"/><limit lower="-6.28" upper="6.28" effort="150" velocity="3.15"/></joint>
      <joint name="\${prefix}shoulder_lift_joint" type="revolute"><parent link="\${prefix}shoulder_link"/><child link="\${prefix}upper_arm_link"/><origin xyz="0 0.135 0" rpy="0 1.5708 0"/><axis xyz="0 1 0"/><limit lower="-6.28" upper="6.28" effort="150" velocity="3.15"/></joint>
      <joint name="\${prefix}elbow_joint" type="revolute"><parent link="\${prefix}upper_arm_link"/><child link="\${prefix}forearm_link"/><origin xyz="0 -0.119 0.425"/><axis xyz="0 1 0"/><limit lower="-3.14" upper="3.14" effort="150" velocity="3.15"/></joint>
      <joint name="\${prefix}wrist_joint" type="revolute"><parent link="\${prefix}forearm_link"/><child link="\${prefix}wrist_link"/><origin xyz="0 0 0.392"/><axis xyz="0 1 0"/><limit lower="-6.28" upper="6.28" effort="28" velocity="3.2"/></joint>
      <joint name="\${prefix}tool_joint" type="fixed"><parent link="\${prefix}wrist_link"/><child link="\${prefix}tool0"/><origin xyz="0 0.1 0" rpy="-1.5708 0 0"/></joint>
      </xacro:macro></robot>`,
    [`${base}ur_description/urdf/materials.xacro`]: `<robot><material name="grey"><color rgba="0.5 0.5 0.5 1"/></material></robot>`,
  };
  const stlBytes = new Uint8Array(writeBinarySTL(new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0])));
  const dae = `<COLLADA><asset><unit meter="1"/></asset><library_geometries><geometry id="g"><mesh><source id="p"><float_array id="pa" count="9">0 0 0 1 0 0 0 1 0</float_array><technique_common><accessor source="#pa" count="3" stride="3"/></technique_common></source><vertices id="v"><input semantic="POSITION" source="#p"/></vertices><triangles count="1"><input semantic="VERTEX" source="#v" offset="0"/><p>0 1 2</p></triangles></mesh></geometry></library_geometries></COLLADA>`;
  const binaries: Record<string, Uint8Array> = {
    [`${base}ur_description/meshes/ur5/visual/shoulder.stl`]: stlBytes,
    [`${base}ur_description/meshes/ur5/visual/base.dae`]: new TextEncoder().encode(dae),
    [`${base}ur_description/meshes/ur5/visual/upperarm.obj`]: new TextEncoder().encode('v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n'),
    [`${base}ur_description/meshes/ur5/visual/forearm.ply`]: new Uint8Array([1]),
  };
  function stubFetch() {
    const calls: string[] = [];
    vi.stubGlobal('fetch', vi.fn(async (url: string) => {
      calls.push(url);
      if (url.endsWith('/boom.xacro')) throw new Error('network down');
      if (files[url]) return { ok: true, text: async () => files[url], arrayBuffer: async () => new ArrayBuffer(0) };
      if (binaries[url]) return { ok: true, text: async () => '', arrayBuffer: async () => binaries[url].buffer.slice(binaries[url].byteOffset, binaries[url].byteOffset + binaries[url].byteLength) };
      return { ok: false, status: 404, text: async () => '', arrayBuffer: async () => new ArrayBuffer(0) };
    }));
    return calls;
  }
  it('fetches a catalogue robot through includes, picks tool0 and downloads meshes of every format', async () => {
    const calls = stubFetch();
    const entry = { id: 'ur_test', brand: 'UR' as const, name: 'UR5 test', repo: 'ur', file: 'ur_description/urdf/ur5.xacro', dof: 4, payload: 5, reach: 850, category: 'collaborative' as const, postProcessor: 'Universal_Robots', args: { prefix: 'r_' } };
    const assets = new AssetStore();
    const progress: string[] = [];
    const res = await fetchOnlineRobot(entry, { assets, onProgress: (m) => progress.push(m), concurrency: 2 });
    expect(res.robot.name).toBe('UR5 test');
    expect(res.robot.dof).toBe(4);
    expect(res.robot.brand).toBe('UR');
    expect(res.robot.params.libraryId).toBe('ur_test');
    expect(res.robot.params.licence).toBe('BSD-3');
    expect(res.robot.params.payload).toBe(5);
    expect(res.model.links.has('r_tool0')).toBe(true);
    // tool0 is the flange: chain ends with the fixed tool joint folded in
    expect(res.robot.chain.joints.at(-1)!.name).toBe('r_tool_joint');
    expect(res.files.length).toBe(3);
    expect(res.meshes.loaded.sort()).toEqual(['package://ur_description/meshes/ur5/visual/base.dae', 'package://ur_description/meshes/ur5/visual/shoulder.stl', 'package://ur_description/meshes/ur5/visual/upperarm.obj']);
    expect(res.meshes.failed.sort()).toEqual(['package://ur_description/meshes/ur5/visual/forearm.ply', 'package://ur_description/meshes/ur5/visual/missing.stl']);
    expect(res.warnings).toEqual(expect.arrayContaining([expect.stringMatching(/forearm.ply: unsupported mesh format .ply/), expect.stringMatching(/missing.stl: 404/)]));
    expect(assets.get('package://ur_description/meshes/ur5/visual/base.dae')?.units).toBe('m');
    expect(assets.get('package://ur_description/meshes/ur5/visual/upperarm.obj')?.mesh?.triangles).toBe(1);
    expect(progress.some((p) => /Downloading 5 meshes/.test(p))).toBe(true);
    expect(progress.filter((p) => p.startsWith('Downloaded')).length).toBe(5);
    // the dynamic include is skipped from prefetch and reported through the resolver
    expect(calls.filter((c) => c.includes('undefined') || c.includes('${'))).toEqual([]);
    // fetching again into the same store skips already-loaded meshes
    const again = await fetchOnlineRobot('ur_test' as any, { assets }).catch((e) => e);
    expect(again).toBeInstanceOf(Error);
    expect(String(again)).toMatch(/Unknown online robot/);
  });
  it('fetches by URL with custom fetchers, no meshes, and reports failures', async () => {
    stubFetch();
    const url = `${base}ur_description/urdf/ur5.xacro`;
    const res = await fetchRobotFromUrl(url, { fetchText: async (u) => files[u] ?? null, args: { prefix: '' } });
    expect(res.robot.brand).toBe('ROS2');
    expect(res.entry.id).toBe('ur5');
    expect(res.entry.reach).toBe(res.robot.reach);
    expect(res.meshes.loaded).toEqual([]);
    expect(res.url).toBe(url);
    expect(res.robot.params.sourceUrl).toBe(url);
    await expect(fetchRobotFromUrl(`${base}nothing.xacro`)).rejects.toThrow(/Could not download/);
    await expect(fetchRobotFromUrl(`${base}x/boom.xacro`)).rejects.toThrow(/Could not download/);
    const noJoints = await fetchRobotFromUrl('https://h/only_links.urdf', { fetchText: async () => '<robot name="e"><link name="a"/></robot>' }).catch((e) => e);
    expect(String(noJoints)).toMatch(/No joints found/);
    const bad = await fetchRobotFromUrl('https://h/bad.urdf', { fetchText: async () => '<notarobot/>' }).catch((e) => e);
    expect(String(bad)).toMatch(/Not a URDF/);
    // include that can never be resolved (file:// and unknown package) is reported as a warning, expansion continues
    const res2 = await fetchRobotFromUrl('https://h/r.xacro', { fetchText: async (u) => (u === 'https://h/r.xacro' ? `<robot name="r"><xacro:include filename="file:///abs/x.xacro"/><xacro:include filename="package://nope/x.xacro"/><link name="a"/><link name="b"/><joint name="j" type="revolute"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/></joint></robot>` : null) });
    expect(res2.robot.dof).toBe(1);
    await expect(fetchOnlineRobot({ ...ONLINE_ROBOT_LIBRARY[0], repo: 'ghost' })).rejects.toThrow(/Unknown repository/);
  });
  it('fetchPackageMeshes resolves package URIs and reports unknown/unsupported ones', async () => {
    stubFetch();
    const assets = new AssetStore();
    assets.registerMesh('package://ur_description/meshes/ur5/visual/base.dae', { positions: new Float32Array(), normals: new Float32Array(), min: [0, 0, 0], max: [0, 0, 0], triangles: 0 });
    const done: string[] = [];
    const r = await fetchPackageMeshes(['package://ur_description/meshes/ur5/visual/base.dae', 'package://ur_description/meshes/ur5/visual/shoulder.stl', 'package://ur_description/meshes/ur5/visual/upperarm.obj', 'package://ur_description/meshes/ur5/visual/forearm.ply', 'package://ur_description/meshes/ur5/visual/missing.stl', 'package://unknown_pkg/a.stl'], assets, { onProgress: (n) => done.push(n), concurrency: 3 });
    expect(r.loaded.sort()).toEqual(['package://ur_description/meshes/ur5/visual/shoulder.stl', 'package://ur_description/meshes/ur5/visual/upperarm.obj']);
    expect(r.failed.length).toBe(3);
    expect(done.length).toBe(5); // the already-present dae is skipped
    expect(assets.get('package://ur_description/meshes/ur5/visual/shoulder.stl')?.mesh?.triangles).toBe(1);
    const viaBytes = await fetchPackageMeshes(['package://ur_description/meshes/ur5/visual/base.dae'], new AssetStore(), { fetchBytes: async () => new TextEncoder().encode(dae) });
    expect(viaBytes.loaded.length).toBe(1);
  });
});

export { identity };
