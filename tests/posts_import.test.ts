// Program importers (P0f): RoboDK's own post outputs (tests/fixtures/posts_gt) re-imported must give back every move
// of the job (MoveC as MoveC), with the same joints / poses in the same frames, plus frames, tools, IO and waits;
// language specifics: KRL .src + .dat, Library KUKA_Cutting.src, RAPID multi-PROC / Offs / RelTool / zones / wobj,
// LS continuation lines / UFRAME / WAIT TIMEOUT, URScript sub-programs / units / threads, RoboDK CSV.
import { describe, it, expect } from 'vitest';
import { existsSync, readdirSync, readFileSync } from 'node:fs';
import path from 'node:path';
import { importProgram } from '../src/io/programs/import';
import { Station, Frame, Target } from '../src/core/items/item';
import { Program } from '../src/core/items/program';
import { createRobotFromLibrary } from '../src/core/items/library';
import { getPos, Mat4, fromArray, DEG } from '../src/core/math/pose';

const CASES = path.resolve(__dirname, 'fixtures/posts_gt/cases');
const kinds = (p: Program) => p.instructions().map((i) => (i.data.kind === 'move' ? i.data.moveType : i.data.kind));
const moves = (p: Program) => p.instructions().filter((i) => i.data.kind === 'move').map((i) => i.data as any);

/** Compare imported moves with the job's move events (main program). */
function checkAgainstJob(st: Station, prog: Program, events: any[], label: string) {
  const ev = events.filter((e) => /^move/.test(e.kind));
  const mv = moves(prog);
  expect(mv.map((m) => m.moveType), `${label}: move types`).toEqual(ev.map((e) => (e.kind === 'moveJ' ? 'MoveJ' : e.kind === 'moveL' ? 'MoveL' : 'MoveC')));
  ev.forEach((e, k) => {
    const t = st.findById(mv[k].targetId) as Target;
    if (e.kind === 'moveJ') {
      expect(t.joints!.slice(0, 6).map((v) => +v.toFixed(2)), `${label} #${k} joints`).toEqual(e.joints.slice(0, 6).map((v: number) => +v.toFixed(2)));
      return;
    }
    const want: Mat4 = fromArray(e.pose);
    const got = t.pose();
    for (let i = 0; i < 16; i++) expect(Math.abs(got[i] - want[i]), `${label} #${k} pose[${i}] ${Array.from(getPos(got))} vs ${Array.from(getPos(want))}`).toBeLessThan(i >= 12 ? 0.01 : 2e-4);
    if (e.kind === 'moveC') {
      const via = st.findById(mv[k].viaTargetId) as Target;
      expect(Math.hypot(...getPos(via.pose()).map((v, i) => v - e.via[12 + i])), `${label} #${k} via`).toBeLessThan(0.01);
    }
  });
}

const GT: Array<[string, string, string, string]> = [
  // case, RoboDK post, main file, library robot
  ['feature_KUKA_KR_6_R900_sixx', 'KUKA_KRC2', 'FeatureTest.src', 'KUKA_KR6_R900'],
  ['feature_KUKA_KR_6_R900_sixx', 'KUKA_KRC4', 'FeatureTest.src', 'KUKA_KR6_R900'],
  ['synth_misc_KUKA_KR_6_R900_sixx', 'KUKA_KRC4', 'Prog_Name_That_I.src', 'KUKA_KR6_R900'],
  ['feature_ABB_IRB_120_3_0_6', 'ABB_RAPID_IRC5', 'FeatureTest.mod', 'ABB_IRB120'],
  ['synth_misc_ABB_IRB_120_3_0_6', 'ABB_RAPID_IRC5', 'Prog_Name_That_Is_Very_Long_123.mod', 'ABB_IRB120'],
  ['feature_Fanuc_LR_Mate_200iD', 'Fanuc_R30i', 'FeatureTes.LS', 'FANUC_LRMATE200iD'],
  ['feature_Fanuc_LR_Mate_200iD', 'Fanuc_RJ3', 'Featur.LS', 'FANUC_LRMATE200iD'],
  ['synth_misc_Fanuc_LR_Mate_200iD', 'Fanuc_R30i', 'Prog_Name_.LS', 'FANUC_LRMATE200iD'],
  ['feature_UR5e', 'Universal_Robots', 'FeatureTest.script', 'UR5e'],
  ['synth_misc_UR5e', 'Universal_Robots', 'Prog_Name_That_Is_Very_Long_123.script', 'UR5e'],
  ['feature_Denso_Cobotta', 'CSV', 'FeatureTest.csv', 'UR5e'],
];

describe('RoboDK post outputs re-imported', () => {
  for (const [c, post, file, robotId] of GT) {
    it(`${c} / ${post}`, () => {
      const dir = path.join(CASES, c, post);
      const job = JSON.parse(readFileSync(path.join(CASES, c, 'job.json'), 'utf8'));
      const st = new Station();
      const robot = st.addChild(createRobotFromLibrary(robotId));
      const files: Record<string, string> = {};
      for (const f of readdirSync(dir)) if (!f.startsWith('_') && f !== file) files[f] = readFileSync(path.join(dir, f), 'utf8');
      const res = importProgram(readFileSync(path.join(dir, file), 'utf8'), { station: st, robot, filename: file, files });
      checkAgainstJob(st, res.program, job.events, `${c}/${post}`);
      const k = kinds(res.program);
      const evk = job.events.map((e: any) => e.kind);
      // frames and tools are recognised (RoboDK writes them as data + activation)
      if (evk.includes('setFrame') && post !== 'CSV') expect(k, 'setFrame').toContain('frame');
      if (evk.includes('setTool') && !/Fanuc/.test(post) && post !== 'CSV') expect(k, 'setTool').toContain('tool');
      // IO: every numbered / named output and input wait comes back as an io instruction
      const io = res.program.instructions().filter((i) => i.data.kind === 'io').map((i) => i.data as any);
      expect(io.filter((d) => !d.wait).length, 'setDO').toBe(job.events.filter((e: any) => e.kind === 'setDO').length);
      expect(io.filter((d) => d.wait).length, 'waitDI').toBe(job.events.filter((e: any) => e.kind === 'waitDI').length);
      expect(io.filter((d) => !d.wait).map((d) => String(d.io))).toEqual(job.events.filter((e: any) => e.kind === 'setDO').map((e: any) => String(e.io)));
      // sub-programs in companion files are imported and called
      if (job.subprograms?.length && post !== 'CSV') {
        expect(res.programs.map((p) => p.name)).toEqual(expect.arrayContaining(['SubPick'].map((n) => (post === 'Fanuc_RJ3' ? 'SubPic' : n))));
        expect(k).toContain('call');
      }
    });
  }
});

describe('language specifics', () => {
  it.skipIf(!existsSync('C:/RoboDK/Library/KUKA_Cutting.src'))('Library KUKA_Cutting.src: 1 PTP + 159 LIN in the $BASE frame with the $TOOL tool', () => {
    const st = new Station();
    const robot = st.addChild(createRobotFromLibrary('KUKA_KR6_R900'));
    const res = importProgram(readFileSync('C:/RoboDK/Library/KUKA_Cutting.src', 'utf8'), { station: st, robot, filename: 'KUKA_Cutting.src' });
    const mv = moves(res.program);
    expect(mv.filter((m) => m.moveType === 'MoveJ').length).toBe(1);
    expect(mv.filter((m) => m.moveType === 'MoveL').length).toBe(159);
    expect(res.frames.length).toBe(1);
    expect(getPos(res.frames[0].poseAbs()).map((v) => +v.toFixed(3))).toEqual([1480.965, -767.411, 500]);
    expect(res.tools.length).toBe(1);
    expect(getPos(res.tools[0].poseTool()).map((v) => +v.toFixed(3))).toEqual([557.2, 0, 176]);
    const lin = res.targets.find((t) => !t.isJointTarget)!;
    expect(lin.parent).toBe(res.frames[0]);
    expect(getPos(lin.pose()).map((v) => +v.toFixed(3))).toEqual([1671.189, -581.324, -193.775]);
    const speeds = res.program.instructions().filter((i) => i.data.kind === 'speed').map((i) => (i.data as any).speedLinear);
    expect(speeds.slice(0, 2)).toEqual([1000, 50]); // approach at 1 m/s, cut at 50 mm/s, alternating per contour
    expect(new Set(speeds)).toEqual(new Set([1000, 50]));
  });
  it('KRL .src with its .dat: E6POS / E6AXIS points, FDAT tool / base, C_DIS rounding, timer wait, messages', () => {
    const st = new Station();
    const robot = st.addChild(createRobotFromLibrary('KUKA_KR6_R900'));
    const src = ['DEF Job()', ';FOLD PTP HOME Vel=100 % DEFAULT;%{PE}', 'PTP XHOME', ';ENDFOLD', 'FDAT_ACT=FP1', 'BASE_DATA[2] = {X 10,Y 20,Z 30,A 0,B 0,C 0}', '$BASE = BASE_DATA[2]', '$APO.CDIS = 4',
      'LIN XP1 C_DIS', 'CIRC XP2, XP3', 'START_TIMER:', '$TIMER_STOP[1]=TRUE', '$TIMER_FLAG[1]=FALSE', '$TIMER[1]=1.500', '$TIMER_STOP[1]=FALSE', 'WAIT FOR ($IN[4]==TRUE OR $TIMER_FLAG[1])', '$TIMER_STOP[1]=TRUE',
      'IF $TIMER_FLAG[1]== TRUE THEN', '    HALT ; Timed out!', '    GOTO START_TIMER', 'ENDIF', 'Wait for StrClear($LOOP_MSG[])', '$LOOP_CONT = TRUE', '$LOOP_MSG[] = "hello"', 'SetRPM(5000)', 'END'].join('\n');
    const dat = ['DEFDAT Job', 'DECL E6AXIS XHOME={A1 0.0,A2 -90.0,A3 90.0,A4 0.0,A5 0.0,A6 0.0,E1 250.0}', 'DECL E6POS XP1={X 500.0,Y 0.0,Z 400.0,A 0.0,B 90.0,C 0.0,S 6,T 27}', 'DECL E6POS XP2={X 550.0,Y 50.0,Z 400.0,A 0.0,B 90.0,C 0.0}',
      'DECL E6POS XP3={X 600.0,Y 0.0,Z 400.0,A 0.0,B 90.0,C 0.0}', 'DECL FDAT FP1={TOOL_NO 1,BASE_NO 0,IPO_FRAME #BASE}', 'ENDDAT'].join('\n');
    const res = importProgram(src, { station: st, robot, filename: 'Job.src', files: { 'Job.dat': dat } });
    expect(kinds(res.program)).toEqual(['rounding', 'MoveJ', 'frame', 'tool', 'frame', 'rounding', 'MoveL', 'rounding', 'MoveC', 'io', 'print', 'code']);
    expect(res.targets[0].joints).toEqual([0, -90, 90, 0, 0, 0, 250]); // external axis E1 kept
    const d = res.program.instructions().map((i) => i.data as any);
    expect(d[5].radius).toBe(4);
    expect(d[9]).toMatchObject({ io: '4', value: true, wait: true, timeoutMs: 1500 });
    expect(d[10]).toMatchObject({ message: 'hello', isComment: false });
    expect(d[11]).toMatchObject({ code: 'SetRPM(5000)', asFunctionCall: true });
    expect(getPos(res.frames.find((f) => f.name === 'Base 2')!.poseAbs())).toEqual([10, 20, 30]);
    expect(res.warnings.some((w) => /TOOL_DATA\[1\]/.test(w))).toBe(true);
  });
  it('RAPID: main + sub PROCs, Offs / RelTool, zones, speeddata, wobjdata frame, \\MaxTime, Set / Reset, calls with arguments', () => {
    const st = new Station();
    const robot = st.addChild(createRobotFromLibrary('ABB_IRB120'));
    const mod = ['MODULE MOD_Cell', '  PERS tooldata tGrip := [TRUE,[[0,0,100],[1,0,0,0]],[1,[0,0,20],[1,0,0,0],0,0,0]];',
      '  PERS wobjdata wTable := [FALSE, TRUE, "", [[0,0,0],[1,0,0,0]],[[400,0,100],[1,0,0,0]]];', '  CONST robtarget p10 := [[100,0,50],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];',
      '  VAR speeddata vSlow := [50,500,5000,1000];', '  PROC Sub1()', '    MoveL Offs(p10,0,0,100),v200,z5,tGrip\\WObj:=wTable;', '  ENDPROC', '  PROC main()', '    MoveJ p10,v1000,fine,tGrip\\WObj:=wTable;',
      '    MoveL RelTool(p10,0,0,-20\\Rz:=90),vSlow,z10,tGrip\\WObj:=wTable;', '    Sub1;', '    SetRPM 5000;', '    Set doGrip;', '    Reset doGrip;', '    WaitDI diPart, 1\\MaxTime:=2.5;', '  ENDPROC', 'ENDMODULE'].join('\n');
    const res = importProgram(mod, { station: st, robot, filename: 'Cell.mod' });
    expect(res.programs.map((p) => p.name)).toEqual(['Cell', 'Sub1']);
    expect(kinds(res.program)).toEqual(['tool', 'frame', 'rounding', 'speed', 'MoveJ', 'rounding', 'speed', 'MoveL', 'call', 'code', 'io', 'io', 'io']);
    const d = res.program.instructions().map((i) => i.data as any);
    expect(d[3].speedLinear).toBe(1000);
    expect(d[6].speedLinear).toBe(50);
    expect(d[5].radius).toBe(10);
    expect(d[8].programName).toBe('Sub1');
    expect(d[9]).toMatchObject({ code: 'SetRPM(5000)', asFunctionCall: true });
    expect(d[10]).toMatchObject({ io: 'doGrip', value: true });
    expect(d[12]).toMatchObject({ io: 'diPart', value: true, wait: true, timeoutMs: 2500 });
    const wobj = res.frames.find((f) => f.name === 'wTable')!;
    expect(getPos(wobj.poseAbs())).toEqual([400, 0, 100]);
    // RelTool: 20 mm back along the tool Z (tool Z points down: +20 in the frame) and 90 deg about tool Z
    const rel = st.findById(moves(res.program)[1].targetId) as Target;
    expect(getPos(rel.pose()).map((v) => +v.toFixed(6))).toEqual([100, 0, 70]);
    const sub = res.programs[1];
    const offs = st.findById(moves(sub)[0].targetId) as Target;
    expect(getPos(offs.pose())).toEqual([100, 0, 150]);
    expect(kinds(sub)).toEqual(['tool', 'frame', 'rounding', 'speed', 'MoveL']);
    expect(res.tools.map((t) => t.name)).toEqual(['tGrip']);
    expect(getPos(res.tools[0].poseTool())).toEqual([0, 0, 100]);
  });
  it('LS: C P[i] + continuation line is one MoveC, cm/min speeds, UFRAME from PR[], WAIT TIMEOUT block, Arc Start', () => {
    const st = new Station();
    const robot = st.addChild(createRobotFromLibrary('FANUC_LRMATE200iD'));
    const ls = ['/PROG  CIRC1', '/ATTR', '/MN', '   1:  PR[9,1]=100.000 ;', '   2:  PR[9,2]=0.000 ;', '   3:  PR[9,3]=50.000 ;', '   4:  PR[9,4]=0.000 ;', '   5:  PR[9,5]=0.000 ;', '   6:  PR[9,6]=0.000 ;',
      '   7:  UFRAME[9]=PR[9] ;', '   8:  UFRAME_NUM=9 ;', '   9:L P[1] 600cm/min CNT10 ;', '  10:C P[2] ', '       P[3] 200mm/sec FINE ;', '  11:  $WAITTMOUT=1000 ;', '  12:  WAIT DI[5]=ON TIMEOUT, LBL[1] ;',
      '  13:  MESSAGE[Timed out for LBL 1] ;', '  14:  PAUSE ;', '  15:  LBL[1] ;', '  16:L P[1] 100mm/sec FINE Arc Start[11] ;', '  17:  Weave Sine[2.0Hz,8.0mm,0.075s,0.075] ;', '/POS',
      ...[1, 2, 3].flatMap((k) => [`P[${k}]{`, '   GP1:', "    UF : 9, UT : 1,        CONFIG : 'N U T, 0, 0, 0',", `\tX =   ${100 + 50 * (k - 1)}.000  mm,\tY =   ${k === 2 ? 50 : 0}.000  mm,\tZ =   10.000  mm,`, '\tW =   180.000 deg,\tP =     0.000 deg,\tR =     0.000 deg', '};']), '/END'].join('\n');
    const res = importProgram(ls, { station: st, robot, filename: 'CIRC1.LS' });
    expect(kinds(res.program)).toEqual(['frame', 'speed', 'rounding', 'MoveL', 'speed', 'rounding', 'MoveC', 'io', 'speed', 'MoveL', 'code']);
    const d = res.program.instructions().map((i) => i.data as any);
    expect(d[1].speedLinear).toBe(100); // 600 cm/min = 100 mm/s
    expect(d[7]).toMatchObject({ io: '5', value: true, wait: true, timeoutMs: 1000 });
    expect(d[10]).toMatchObject({ code: 'ArcStart', asFunctionCall: true });
    expect(getPos(res.frames[0].poseAbs())).toEqual([100, 0, 50]);
    const mc = moves(res.program)[1];
    expect(getPos((st.findById(mc.viaTargetId) as Target).pose())).toEqual([150, 50, 10]);
    expect(getPos((st.findById(mc.targetId) as Target).pose())).toEqual([200, 0, 10]);
  });
  it('URScript: sub-programs and their calls, movej v in rad/s -> joint speed, movel units, movec, threads, IO ranges, waits', () => {
    const st = new Station();
    const robot = st.addChild(createRobotFromLibrary('UR5e'));
    const script = ['def main():', '  def sub_one():', '    movel(p[0.4, 0.0, 0.3, 3.141593, 0, 0], a=1.2, v=0.25, r=0.005)', '  end', '  thread Thread_sub_one():', '    sub_one()', '  end',
      '  movej([0, -1.5708, 1.5708, -1.5708, -1.5708, 0], a=1.4, v=1.05)', '  ref_frame = p[0.1, 0.2, 0.0, 0, 0, 0]', '  movel(pose_trans(ref_frame,p[0.3, 0.0, 0.2, 3.141593, 0, 0]),1.2,0.1,0,0.002)',
      '  movec(pose_trans(ref_frame,p[0.35, 0.05, 0.2, 3.141593, 0, 0]),pose_trans(ref_frame,p[0.4, 0.0, 0.2, 3.141593, 0, 0]),1.2,0.1,0)', '  sub_one()', '  thrd_sub_one = run Thread_sub_one()',
      '  set_tool_digital_out(1, True)', '  while (get_standard_digital_in(4) != True):', '    sync()', '  end', '  SprayOn(1)', 'end', 'main()'].join('\n');
    const res = importProgram(script, { station: st, robot, filename: 'main.script' });
    expect(res.programs.map((p) => p.name)).toEqual(['main', 'sub_one']);
    expect(kinds(res.program)).toEqual(['speed', 'MoveJ', 'frame', 'speed', 'rounding', 'MoveL', 'rounding', 'MoveC', 'call', 'thread', 'io', 'io', 'code']);
    const d = res.program.instructions().map((i) => i.data as any);
    expect(d[0].speedJoints).toBeCloseTo(1.05 / DEG, 6); // 60.16 deg/s, not 1050 mm/s
    expect(d[0].speedLinear).toBeUndefined();
    expect(d[3].speedLinear).toBeCloseTo(100, 9);
    expect(d[4].radius).toBeCloseTo(2, 9);
    expect(d[10]).toMatchObject({ io: '11', value: true, wait: false }); // tool output 1 -> RoboDK IO 11
    expect(d[11]).toMatchObject({ io: '4', value: true, wait: true });
    expect(d[12]).toMatchObject({ code: 'SprayOn(1)', asFunctionCall: true });
    const lin = st.findById(moves(res.program)[1].targetId) as Target;
    expect(lin.parent).toBe(res.frames[0]);
    expect(getPos(lin.pose()).map((v) => +v.toFixed(6))).toEqual([300, 0, 200]);
    expect(kinds(res.programs[1])).toEqual(['speed', 'rounding', 'MoveL']);
  });
});
