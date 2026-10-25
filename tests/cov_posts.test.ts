import { describe, it, expect, vi, afterEach, beforeEach } from 'vitest';
import { listPosts, getPost, compileForPost, PostProgram, PostEvent, f, safeName } from '../src/posts/index';
import { poseToZyz, zyzToPose } from '../src/posts/euler';
import { eventsToJSON, registerPythonPost, runPythonPost } from '../src/posts/python_post';
import { listUserPosts, addUserPost, removeUserPost, restoreUserPosts, guessBrand, looksLikeRoboDKPost, postIdFor, importPostFiles } from '../src/posts/user_posts';
import { demos } from '../src/demos';
import { Station, Frame, Target, Tool, SceneObject, ItemType } from '../src/core/items/item';
import { Program } from '../src/core/items/program';
import { createRobotFromLibrary } from '../src/core/items/library';
import { transl, mul, rotx, rotz, roty, DEG, getPos, identity, distance } from '../src/core/math/pose';

afterEach(() => vi.unstubAllGlobals());

const pose = (x: number, y: number, z: number) => mul(transl(x, y, z), rotx(180 * DEG));

/** A synthetic program touching every PostEvent kind (with a sub program). */
function allEventsProgram(): PostProgram {
  const robot = createRobotFromLibrary('UR10e');
  const q0 = [10, -90, 90, -90, -90, 0];
  const events: PostEvent[] = [
    { kind: 'comment', text: 'start' },
    { kind: 'setFrame', pose: transl(500, 0, 0), name: 'Table 1' },
    { kind: 'setTool', pose: transl(0, 0, 120), name: 'Gripper', toolId: 'tool-1' },
    { kind: 'setSpeed', speedLinear: 250, speedJoints: 60, accelLinear: 1200, accelJoints: 300 },
    { kind: 'setSpeed', speedJoints: 30 },
    { kind: 'setRounding', radius: 10 },
    { kind: 'moveJ', joints: q0, pose: robot.solveFK(q0), name: 'Home' },
    { kind: 'moveL', joints: [12, -85, 95, -100, -90, 5], pose: pose(300, 100, 200), name: 'Pick 1', speed: 100 },
    { kind: 'moveC', joints: null, via: pose(350, 150, 200), pose: pose(400, 100, 200), name: 'Arc', viaName: 'Arc via' },
    { kind: 'setRounding', radius: 0 },
    { kind: 'moveL', joints: null, pose: pose(400, -100, 300), name: 'Place' },
    { kind: 'pause', timeMs: 500 },
    { kind: 'pause', timeMs: -1 },
    { kind: 'setDO', io: 'DO3', value: true },
    { kind: 'setDO', io: 'gripper', value: 0 },
    { kind: 'waitDI', io: 'DI7', value: false, timeoutMs: 2000 },
    { kind: 'runCode', code: 'MyRoutine', isCall: true },
    { kind: 'runCode', code: 'x = 1', isCall: false },
    { kind: 'message', text: 'Cycle "done"' },
    { kind: 'callProgram', name: 'Sub Routine' },
    { kind: 'gripper', close: true },
    { kind: 'gripper', close: false },
    { kind: 'navigate', x: 1500, y: -200, heading: 90, speed: 500, label: 'dock' },
    { kind: 'task', task: 'harvest', params: { rows: 2 } },
    { kind: 'moveJ', joints: [0, -90, 90, -90, -90, 0, 100], pose: null, name: 'Home 2' },
  ];
  const sub: PostProgram = { name: 'Sub Routine', robot, robotName: robot.name, dof: 6, jointNames: robot.jointNames(), events: [{ kind: 'moveJ', joints: q0, pose: robot.solveFK(q0), name: 'SubHome' }, { kind: 'moveL', joints: q0, pose: pose(300, 0, 200), name: 'SubL' }], subprograms: [], frame: { pose: identity(), name: 'World' }, tool: { pose: identity(), name: 'Flange' }, problems: [] };
  return { name: 'All Events 1', robot, robotName: robot.name, dof: 6, jointNames: robot.jointNames(), events, subprograms: [sub], frame: { pose: transl(500, 0, 0), name: 'Table 1' }, tool: { pose: transl(0, 0, 120), name: 'Gripper' }, problems: [] };
}

/** Posts that echo target names into their output (RoboDK's KUKA / ABB / Fanuc / UR / Motoman posts do not). */
const LABELLED = ['RoboDK_Python', 'Generic', 'JSON'];

describe('post processor registry', () => {
  it('registers every vendor post with a unique id and extension', () => {
    const posts = listPosts();
    const ids = posts.map((p) => p.id);
    expect(new Set(ids).size).toBe(ids.length);
    for (const id of ['KUKA_KRC4', 'ABB_RAPID_IRC5', 'Fanuc_R30iA', 'Universal_Robots', 'Motoman', 'Staubli_VAL3', 'Doosan_Robotics', 'Mecademic', 'Generic', 'JSON', 'RoboDK_Python', 'ROS2', 'Denso_RC8', 'Kawasaki', 'Nachi_AX_FD', 'Comau_C5G', 'Epson_RC', 'Techman_TM', 'Hanwha_HCR', 'Kinova_Kortex', 'Mitsubishi_Melfa', 'KUKA_KRC2', 'AUBO', 'JAKA', 'Elite_EC', 'Dobot_CR', 'Fanuc_RJ3', 'ABB_RAPID_S4C']) expect(ids, id).toContain(id);
    expect(posts.length).toBeGreaterThanOrEqual(28);
    for (const p of posts) { expect(p.extension).toMatch(/^[A-Za-z0-9]+$/); expect(p.brand).toBeTruthy(); expect(p.name).toBeTruthy(); }
    expect(getPost('nope')).toBeUndefined();
  });

  it('every post turns the all-events program into non-empty files with the declared extension', () => {
    const prog = allEventsProgram();
    for (const post of listPosts()) {
      const files = post.generate(prog, { topic: '/traj', uframe: 2, utool: 3, toolIndex: 2, baseIndex: 2 });
      expect(files.length, post.id).toBeGreaterThan(0);
      const main = files[0];
      expect(main.name.toLowerCase(), post.id).toMatch(post.id === 'Nachi_AX_FD' ? /-01-a[.][0-9]{3}$/ : new RegExp(`\\.${post.extension.toLowerCase()}$`));
      expect(main.content.length, post.id).toBeGreaterThan(100);
      const all = files.map((x) => x.content).join('\n');
      // posts that label their moves carry every target name; all posts carry the home joints (-90) and the pick X (300 mm / 0.3 m)
      if (LABELLED.includes(post.id)) for (const label of ['Home', 'Pick 1', 'Place']) expect(all, `${post.id} ${label}`).toContain(label);
      // no NaN or undefined leaks
      expect(main.content, post.id).not.toMatch(/NaN|undefined/);
      // sub program is emitted (as extra file or inline)
      // ROS2 inlines sub-programs in its trajectory; JAKA's controller script has no program calls (RoboDK: same)
      if (post.id !== 'ROS2' && post.id !== 'JAKA') expect(all, post.id).toMatch(/Sub[_ ]?Ro/i); // Fanuc / Motoman truncate names
    }
  });

  it('vendor specifics: KUKA, ABB, Fanuc, UR, ROS2, RoboDK, Generic, JSON', () => {
    const prog = allEventsProgram();
    // KUKA / ABB / Fanuc / UR reproduce RoboDK's own posts (token-equal output in tests/posts_gt.test.ts); here: the
    // semantics RoboDK's GT does not cover (names with blanks, events outside a compiled program, per-move overrides).
    const kuka = getPost('KUKA_KRC4')!.generate(prog);
    expect(kuka.map((x) => x.name)).toEqual(['AllEvents1.src', 'SubRoutine.src']);
    const src = kuka[0].content;
    expect(src).toContain('DEF AllEvents1 ( )');
    expect(src).toContain('$BASE = {FRAME: X 500.000,Y 0.000,Z 0.000,A 0.000,B 0.000,C 0.000}');
    expect(src).not.toMatch(/\{FRAME: \{/); // RoboDK aggregate syntax, not a nested one
    expect(src).toContain('$TOOL = {FRAME: X 0.000,Y 0.000,Z 120.000,A 0.000,B 0.000,C 0.000}');
    expect(src).toContain('$VEL.CP = 0.25000');
    expect(src).toContain('$ACC.CP = 1.20000');
    expect(src).toContain('$VEL.ORI1 = 30.00000'); // joint speed -> orientation speed like RoboDK
    expect(src).toContain('$APO.CDIS = 10.000');
    expect(src).toContain('PTP {A1 10.00000,A2 -90.00000,A3 90.00000,A4 -90.00000,A5 -90.00000,A6 0.00000} C_PTP');
    // per-move speed 100 applied to Pick 1 only, then the program speed is restored
    expect(src).toMatch(/\$VEL\.CP = 0\.10000\r?\n(\$APO.*\r?\n)*LIN \{X 300\.000,Y 100\.000,Z 200\.000,A 0\.000,B 0\.000,C 180\.000\} C_DIS\r?\n\$VEL\.CP = 0\.25000\r?\nCIRC \{X 350\.000/);
    expect(src).toContain('WAIT SEC 0.500');
    expect(src).toContain('HALT');
    expect(src).toContain('DO3=TRUE'); // named IO kept verbatim
    expect(src).toContain('MyRoutine()');
    expect(src).toContain('SubRoutine()');
    expect(kuka[1].content).toContain('DEF SubRoutine ( )');
    // simulation events (attach / gripper) never become outputs
    expect(src).not.toMatch(/\$OUT\[/);
    expect(src).toContain('; gripper close');
    expect(src).toContain('; NAVIGATE X 1500.000 Y -200.000');

    const abbF = getPost('ABB_RAPID_IRC5')!.generate(prog);
    expect(abbF.map((x) => x.name)).toEqual(['AllEvents1.mod']);
    const abb = abbF[0].content;
    expect(abb).toContain('PERS tooldata Gripper := [TRUE,[[0.000,0.000,120.000],[1.00000000,0.00000000,0.00000000,0.00000000]]');
    expect(abb).toContain('PERS wobjdata Table1 := [FALSE, TRUE, "", [[0,0,0],[1,0,0,0]],[[500.000,0.000,0.000]');
    // every declaration once, sub-program as its own PROC in the same module, called by name
    for (const decl of abb.match(/PERS \w+ \w+/g) ?? []) expect(abb.split(decl).length - 1, decl).toBe(1);
    expect(abb).toContain('PROC SubRoutine()');
    expect(abb).toMatch(/^\t\tSubRoutine;$/m);
    expect(abb).toMatch(/MoveAbsJ \[\[10\.000000,-90\.000000,90\.000000,-90\.000000,-90\.000000,0\.000000\],\[9E\+09,9E\+09,9E\+09,9E\+09,9E\+09,9E\+09\]\],\[250\.00,500,5000,1000\],z10,Gripper \\WObj:=Table1;/);
    expect(abb).toMatch(/MoveL \[\[300\.000,100\.000,200\.000\],\[0\.00000000,1\.00000000,0\.00000000,0\.00000000\],\[0,-2,0,0\],.*\],v100,z10,Gripper \\WObj:=Table1;/);
    expect(abb).toMatch(/MoveC \[\[350\.000,150\.000,200\.000\].*\[\[400\.000,100\.000,200\.000\].*,\[250\.00,500,5000,1000\],z10,Gripper/);
    expect(abb).toMatch(/MoveL \[\[400\.000,-100\.000,300\.000\].*,z0,Gripper/);
    expect(abb).toContain('WaitTime 0.500;');
    expect(abb).toContain('SetDO DO3, 1;');
    expect(abb).toContain('WaitDI DI7, 0, \\MaxTime:=2.0;');
    expect(abb).toContain('MyRoutine;');
    expect(abb).not.toMatch(/doGripper/);
    expect(abb.trim().endsWith('ENDMODULE')).toBe(true);

    const fanuc = getPost('Fanuc_R30iA')!.generate(prog);
    expect(fanuc.map((x) => x.name)).toEqual(['AllEvents1.LS', 'SubRoutine.LS']);
    const ls = fanuc[0].content;
    expect(ls).toContain('/PROG  AllEvents1');
    // un-numbered frame / tool are written into PR[9] and activated
    expect(ls).toMatch(/PR\[9,1\]=500\.000 ;[\s\S]*UFRAME\[9\]=PR\[9\] ;\r\n\s+\d+:\s+UFRAME_NUM=9 ;/);
    expect(ls).toMatch(/PR\[9,3\]=120\.000 ;[\s\S]*UTOOL\[9\]=PR\[9\] ;\r\n\s+\d+:\s+UTOOL_NUM=9 ;/);
    expect(ls).toMatch(/J P\[1\] 15% CNT10 ACC100 ;/); // speedJoints 30 -> 15 %
    expect(ls).toMatch(/L P\[2\] 100mm\/sec CNT10 ACC100 ;/); // per-move speed
    expect(ls).toMatch(/C P\[3\] \r\n\s+P\[4\] 250mm\/sec CNT10 ;/); // continuation line has no number
    expect(ls).toContain('WAIT  0.50(sec)');
    expect(ls).toContain('PAUSE');
    expect(ls).toContain('WAIT DI7=OFF TIMEOUT, LBL[1]');
    expect(ls).toContain('CALL MyRoutine');
    expect(ls).toContain('CALL SubRoutine ;'); // calls the program by its file name
    expect(ls).toMatch(/J1=\s+10\.000 deg,\tJ2=\s+-90\.000 deg,\tJ3=\s+90\.000 deg/); // J3 as given (RoboDK)
    expect(ls).toContain("CONFIG : 'N U T, 0, 0, 0'");
    expect(ls).not.toMatch(/RO\[|DO\[1\]/); // no fabricated gripper outputs
    expect(ls).toMatch(/LINE_COUNT\t= \d+;/);

    const urF = getPost('Universal_Robots')!.generate(prog);
    expect(urF.map((x) => x.name)).toEqual(['AllEvents1.script', 'AllEvents1.urp']);
    const ur = urF[0].content;
    expect(ur).toContain('def AllEvents1():');
    expect(ur).toContain('ref_frame = p[0.500000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000]');
    expect(ur).toContain('set_tcp(p[0.000000, 0.000000, 0.120000, 0.000000, 0.000000, 0.000000])');
    // a speed change right after a blended move stops the blend like RoboDK ("# end trace")
    expect(ur).toContain('movej([0.174533, -1.570796, 1.570796, -1.570796, -1.570796, 0.000000],accel_radss,speed_rads,0,0) # end trace');
    expect(ur).toMatch(/speed_ms\s+= 0\.100\r?\n\s+movel\(pose_trans\(ref_frame,p\[0\.300000, 0\.100000, 0\.200000, 3\.141593, 0\.000000, 0\.000000\]\),accel_mss,speed_ms,0,0\) # end trace/);
    expect(ur).toMatch(/movec\(pose_trans\(ref_frame,p\[0\.350000.*\]\),pose_trans\(ref_frame,p\[0\.400000.*\]\),accel_mss,speed_ms,0/);
    expect(ur).toContain('sleep(0.500)');
    expect(ur).toContain('MyRoutine()');
    // the sub-program is defined inside the main program and called once from it
    expect(ur.indexOf('def SubRoutine():')).toBeGreaterThan(ur.indexOf('def AllEvents1():'));
    expect(ur.match(/^\s*SubRoutine\(\)$/gm)?.length).toBe(1);
    expect(ur.trim().endsWith('AllEvents1()')).toBe(true);
    expect(ur).not.toMatch(/set_tool_digital_out/);
    // ROS 2: ament_python package with a FollowJointTrajectory client (system test in tests/posts_ros2.test.ts)
    const ros = getPost('ROS2')!.generate(prog, { controller: 'arm_controller' });
    expect(ros.map((x) => x.name)).toEqual(expect.arrayContaining(['allevents1_ros2/package.xml', 'allevents1_ros2/setup.py', 'allevents1_ros2/allevents1_ros2/allevents1_node.py', 'allevents1_ros2/trajectory.json']));
    const traj = JSON.parse(ros.find((f) => f.name.endsWith('trajectory.json'))!.content);
    expect(traj.joint_names).toEqual(prog.robot!.jointNames());
    expect(traj.controller).toBe('arm_controller');
    const tsegs = traj.segments.filter((x: any) => x.type === 'traj');
    expect(tsegs[0].points[0].positions[0]).toBeCloseTo(10 * DEG, 6);
    expect(tsegs[0].points[0].first).toBe(true); // timed from the current state at run time
    expect(traj.segments.some((x: any) => x.type === 'io' && x.name === 'DO3' && x.value === 1)).toBe(true);
    expect(traj.segments.some((x: any) => x.type === 'pause' && x.seconds === 0.5)).toBe(true);
    expect(ros[0].content).toContain("'/%s/follow_joint_trajectory' % controller");
    expect(ros.find((f) => f.name.endsWith('trajectory.json'))!.mime).toBe('application/json');

    const rdk = getPost('RoboDK_Python')!.generate(prog);
    expect(rdk.map((x) => x.name)).toEqual(['All_Events_1_robodk.py', 'Sub_Routine_robodk.py']);
    const py = rdk[0].content;
    expect(py).toContain('robot = RDK.Item("Universal Robots UR10e", ITEM_TYPE_ROBOT)');
    expect(py).toContain('t1 = RDK.AddTarget("Home", frame, robot)');
    expect(py).toContain('t1.setAsJointTarget()');
    expect(py).toContain('t2.setAsCartesianTarget()');
    expect(py).toContain('t2.setJoints([12,-85,95,-100,-90,5])');
    expect(py).toContain('prog.MoveC(t3, t4)');
    expect(py).toContain('prog.setSpeed(250, 60, 1200, 300)');
    expect(py).toContain('prog.setSpeed(-1, 30, -1, -1)');
    expect(py).toContain('prog.setRounding(10.00)');
    expect(py).toContain('prog.Pause(-1)');
    expect(py).toContain('prog.setDO("DO3", "1")');
    expect(py).toContain('prog.setDO("gripper", "0")');
    expect(py).toContain('prog.waitDI("DI7", "0", 2000)');
    expect(py).toContain('prog.RunInstruction("MyRoutine", INSTRUCTION_CALL_PROGRAM)');
    expect(py).toContain('prog.RunInstruction("x = 1", INSTRUCTION_INSERT_CODE)');
    expect(py).toContain('INSTRUCTION_SHOW_MESSAGE');
    expect(py).toContain('prog.RunInstruction("Attach", INSTRUCTION_CALL_PROGRAM)');
    expect(py).toContain('fr = RDK.Item("Table 1", ITEM_TYPE_FRAME)');
    expect(py).toContain('tl = RDK.Item("Gripper", ITEM_TYPE_TOOL)');

    const csv = getPost('Generic')!.generate(prog)[0].content;
    expect(csv.split('\n')[1]).toBe('type,name,x,y,z,rx,ry,rz,' + prog.jointNames.join(','));
    expect(csv).toMatch(/^moveJ,Home,.*,10.000,-90.000,90.000,-90.000,-90.000,0.000$/m);
    expect(csv).toMatch(/^moveL,Pick 1,300.000,100.000,200.000,(-?180.000|0.000),/m);
    expect(csv).toMatch(/^moveC,Arc,400.000,/m);
    expect(csv).toContain('# start');
    expect(csv).toMatch(/^pause,\{"kind":"pause";"timeMs":500\}$/m);
    const json = JSON.parse(getPost('JSON')!.generate(prog)[0].content);
    expect(json.program).toBe('All Events 1');
    expect(json.joint_names.length).toBe(6);
    expect(json.events.length).toBe(prog.events.length);
    expect(json.events[7].xyzrpw.slice(0, 3)).toEqual([300, 100, 200]);
    expect(json.events[7].position_m).toEqual([0.3, 0.1, 0.2]);
    expect(json.events[7].quat_wxyz[0]).toBeCloseTo(0, 9);
    expect(Array.isArray(json.events[8].via)).toBe(true);
    expect(json.subprograms[0].program).toBe('Sub Routine');
  });

  it('Elite (RoboDK EC V3): definitions, Lua base conversion, then the motions; IO-before-motion warning; RoboDK split', () => {
    const post = getPost('Elite_Robots_EC_V3')!;
    expect(post.id).toBe('Elite_EC');
    const prog = allEventsProgram();
    const files = post.generate(prog, { date: new Date(2026, 9, 25, 12, 0, 0) });
    expect(files.map((x) => x.name)).toEqual(['AllEvents1.jbi', 'SubRoutine.jbi']);
    const lines = files[0].content.split('\r\n');
    expect(lines.slice(0, 3)).toEqual(['NOP', '// HEADER', '// DATE 2026/10/25 12:00']);
    const firstMove = lines.findIndex((l) => /^MOV[JLC] /.test(l));
    const lua = lines.indexOf('//set total pose number and use lua to tranf pose in user to in_base');
    expect(lua).toBeGreaterThan(0);
    expect(firstMove).toBeGreaterThan(lua);
    expect(lines.findIndex((l) => l.startsWith('DOUT '))).toBeLessThan(lua);
    expect(lines.findIndex((l) => l.startsWith('WAIT ') && l.includes(' T=2'))).toBeLessThan(lua);
    expect(lines).toContain('CALL JOB:SubRoutine');
    expect(lines).toContain('SETUSERFRAME USER #(1) V000');
    expect(lines).toContain('SETTOOLNUMBER TF=1');
    // MoveC: last linear target, via, end; rounding clamped to 7 mm
    const movc = lines.filter((l) => l.startsWith('MOVC '));
    expect(movc.length).toBe(3);
    expect(movc[0]).toMatch(/^MOVC V000 V=250MM\/S CR=7.0MM ACC=50 DEC=50$/);
    expect(lines[lines.length - 2]).toBe('END');
    expect(prog.problems.some((p) => /executed before the motion block/.test(p))).toBe(true);

    const split = post.generate(allEventsProgram(), { maxLines: 8 });
    const main = split[0].content.split('\r\n');
    expect(split[0].name).toBe('M_AllEvents1.jbi');
    const parts = main.filter((l) => l.startsWith('CALL JOB:')).map((l) => l.slice(9));
    expect(parts.length).toBeGreaterThan(1);
    expect(parts[0]).toBe('AllEvents1');
    for (const [i, name] of parts.entries()) expect(split.map((x) => x.name)).toContain(`${name}.jbi`), expect(name).toBe(i ? `AllEvents1${i + 1}` : 'AllEvents1');
    // every part re-runs the motions so far; the last part and the main run all of them
    const motions = (txt: string) => txt.split('\r\n').filter((l) => /^MOV[JLC] /.test(l));
    const last = split.find((x) => x.name === `${parts[parts.length - 1]}.jbi`)!;
    expect(motions(last.content)).toEqual(motions(split[0].content));
    expect(motions(split[0].content)).toEqual(motions(files[0].content));
    expect(motions(split.find((x) => x.name === 'AllEvents1.jbi')!.content).length).toBeLessThan(motions(split[0].content).length);
  });

  it('runs every post on the demo pick & place program compiled from the station', () => {
    const st = demos.find((d) => d.id === 'pickplace')!.build();
    const prog = st.itemsOfType<Program>(ItemType.PROGRAM)[0];
    const compiled = compileForPost(st, prog);
    expect(compiled.problems).toEqual([]);
    expect(compiled.robotName).toBe('UR10e');
    expect(compiled.frame.name).toBe('Table');
    expect(compiled.tool.name).toBe('Vacuum gripper');
    expect(getPos(compiled.tool.pose)).toEqual([0, 0, 160]);
    const nJ = compiled.events.filter((e) => e.kind === 'moveJ').length;
    const nL = compiled.events.filter((e) => e.kind === 'moveL').length;
    // Home + 3×(above, place above) + Home = 8 MoveJ; 3×(pick, above, place, place above) = 12 MoveL
    expect(nJ).toBe(8);
    expect(nL).toBe(12);
    expect(compiled.events.filter((e) => e.kind === 'gripper').length).toBe(6);
    expect(compiled.events.filter((e) => e.kind === 'setDO').length).toBe(6);
    expect(compiled.events.filter((e) => e.kind === 'pause').length).toBe(3);
    // the exported program starts in its own frame with its own tool, then the program's speed instruction
    expect(compiled.events.slice(0, 3)).toMatchObject([{ kind: 'setFrame', name: 'Table' }, { kind: 'setTool', name: 'Vacuum gripper' }, { kind: 'setSpeed', speedLinear: 600, speedJoints: 120 }]);
    // every moveL has a joint solution and its pose is expressed in the Table frame
    const first = compiled.events.find((e) => e.kind === 'moveL') as any;
    expect(first.joints).toBeTruthy();
    expect(getPos(first.pose).map((v) => Math.round(v))).toEqual([100, 150, 622]);
    for (const post of listPosts()) {
      const files = post.generate(compiled);
      expect(files.length, post.id).toBeGreaterThan(0);
      expect(files[0].name.toLowerCase(), post.id).toMatch(post.id === 'Nachi_AX_FD' ? /-01-a[.][0-9]{3}$/ : new RegExp(`\\.${post.extension.toLowerCase()}$`));
      const all = files.map((x) => x.content).join('\n');
      expect(all.length, post.id).toBeGreaterThan(200);
      if (LABELLED.includes(post.id)) { expect(all, post.id).toContain('Home'); expect(all, post.id).toContain('Pick 1'); }
      expect(all, post.id).not.toMatch(/NaN/);
    }
    // move counts survive in the KUKA and UR outputs
    const krl = getPost('KUKA_KRC4')!.generate(compiled)[0].content;
    expect((krl.match(/^PTP /gm) ?? []).length).toBe(nJ);
    expect((krl.match(/^LIN /gm) ?? []).length).toBe(nL);
    const ur = getPost('Universal_Robots')!.generate(compiled)[0].content;
    expect((ur.match(/movej\(/g) ?? []).length).toBe(nJ);
    expect((ur.match(/movel\(/g) ?? []).length).toBe(nL);
    const ls = getPost('Fanuc_R30iA')!.generate(compiled)[0].content;
    expect((ls.match(/:J P\[/g) ?? []).length).toBe(nJ);
    expect((ls.match(/:L P\[/g) ?? []).length).toBe(nL);
  });
});

describe('compileForPost resolves every instruction kind', () => {
  function cell() {
    const st = new Station('c');
    const robot = st.addChild(createRobotFromLibrary('UR5e'));
    robot.setPose(transl(0, 0, 100));
    const tool = robot.addChild(new Tool('T'));
    tool.setPoseTool(transl(0, 0, 50));
    robot.setTool(tool);
    const frame = st.addChild(new Frame('F'));
    frame.setPose(transl(300, 0, 0));
    robot.setFrame(frame);
    const t = frame.addChild(new Target('T1'));
    t.setPose(mul(transl(0, 0, 400), rotx(180 * DEG)));
    const far = frame.addChild(new Target('Far'));
    far.setPose(transl(5000, 0, 0));
    const via = frame.addChild(new Target('Via'));
    via.setPose(mul(transl(50, 50, 400), rotx(180 * DEG)));
    const jt = frame.addChild(new Target('JT'));
    jt.setJoints([0, -90, 90, -90, -90, 0]);
    jt.setAsJointTarget();
    const box = frame.addChild(new SceneObject('Box'));
    return { st, robot, tool, frame, t, far, via, jt, box };
  }
  it('emits events for moves, frames, tools, io, calls, threads, waits and mobile instructions', () => {
    const { st, robot, tool, frame, t, far, via, jt, box } = cell();
    const sub = st.addChild(new Program('Sub'));
    sub.setRobot(robot);
    sub.addMoveJ(jt);
    const prog = st.addChild(new Program('Main'));
    prog.setRobot(robot);
    prog.addMoveJ(jt);
    prog.addMoveL(t, { speed: 80, rounding: 2 });
    prog.addMoveC(via, t);
    prog.addMoveJ([10, -80, 80, -90, -90, 10]);
    prog.addMoveL(mul(transl(0, 0, 350), rotx(180 * DEG))); // inline pose in frame
    prog.addInstruction({ kind: 'move', moveType: 'MoveJ', targetId: null, pose: Array.from(mul(transl(0, 0, 380), rotx(180 * DEG))) });
    prog.addInstruction({ kind: 'move', moveType: 'MoveC', targetId: t.id, viaPose: Array.from(mul(transl(20, 20, 400), rotx(180 * DEG))) });
    prog.addInstruction({ kind: 'move', moveType: 'MoveC', targetId: t.id }); // via missing -> problem
    prog.addMoveL(far); // unreachable
    prog.addInstruction({ kind: 'move', moveType: 'MoveJ', targetId: null }); // no data -> problem
    prog.addInstruction({ kind: 'move', moveType: 'MoveL', targetId: null }); // no pose -> problem
    prog.addInstruction({ kind: 'jointPath', joints: Array.from({ length: 5 }, (_, i) => [i, -90, 90, -90, -90, 0]), source: 'log' });
    prog.setFrame(null);
    prog.setFrame(frame);
    prog.setTool(null);
    prog.setTool(tool);
    prog.setSpeed(100, 50);
    prog.setRounding(3);
    prog.pause(100);
    prog.setDO('D1', 1);
    prog.waitDI('D2', 0, 500);
    prog.runInstruction('foo');
    prog.showMessage('hello');
    prog.comment('note');
    prog.event('attach', box.id);
    prog.event('gripper_open');
    prog.event('show', box.id);
    prog.callProgram(sub);
    prog.callProgram(sub);
    prog.addInstruction({ kind: 'call', programId: null, programName: 'Sub' });
    prog.addInstruction({ kind: 'call', programId: 'ghost' });
    prog.addInstruction({ kind: 'signal', signal: 's1', value: 1, wait: false });
    prog.addInstruction({ kind: 'signal', signal: 's2', value: true, wait: true });
    prog.navigateTo(1000, 2000, 45, { speed: 300, label: 'dock' });
    prog.addInstruction({ kind: 'mobile_follow', pathId: 'p1' });
    prog.addInstruction({ kind: 'mission_task', task: 'spray', params: { rate: 1 } });
    prog.addInstruction({ kind: 'loop', count: 2, bodyProgramId: sub.id });
    prog.addInstruction({ kind: 'if', condition: 'x', thenProgramId: sub.id });
    prog.startThread(sub);
    prog.addInstruction({ kind: 'wait', what: 'time', timeMs: 250 });
    prog.waitSignal('s3', 1, 100);
    prog.addInstruction({ kind: 'wait', what: 'move_done' });
    const disabled = prog.pause(999);
    disabled.enabled = false;
    const c = compileForPost(st, prog);
    const kinds = c.events.map((e) => e.kind);
    const moves = c.events.filter((e) => /^move/.test(e.kind)) as any[];
    expect(moves.slice(0, 7).map((e) => e.kind)).toEqual(['moveJ', 'moveL', 'moveC', 'moveJ', 'moveL', 'moveJ', 'moveC']);
    // RoboDK keeps every move it can express: only the MoveC without via and the MoveL without pose are skipped
    expect(c.problems).toEqual(expect.arrayContaining([expect.stringMatching(/T1: MoveC needs via/), expect.stringMatching(/Far: unreachable/), expect.stringMatching(/MoveJ: no joint values for MoveJ, previous joints kept/), expect.stringMatching(/MoveL without a pose skipped/)]));
    expect(c.problems.length).toBe(4);
    // the exported program starts with its frame and tool; the per-move override becomes explicit speed / zone events
    // around the move (posts have no per-move parameters, like RoboDK)
    expect(kinds.slice(0, 9)).toEqual(['setFrame', 'setTool', 'moveJ', 'setSpeed', 'setRounding', 'moveL', 'setSpeed', 'setRounding', 'moveC']);
    expect(c.events.slice(3, 8)).toMatchObject([{ speedLinear: 80 }, { radius: 2 }, { kind: 'moveL' }, { speedLinear: 500 }, { radius: -1 }]);
    const moveL = moves[1];
    expect(moveL.speed).toBe(80);
    // pose w.r.t. frame F: target is 400 up in F
    expect(getPos(moveL.pose).map(Math.round)).toEqual([0, 0, 400]);
    expect(moveL.joints.length).toBe(6);
    const mc = moves[2];
    expect(mc.viaName).toBe('Via');
    expect(getPos(mc.via).map(Math.round)).toEqual([50, 50, 400]);
    expect(mc.viaJoints.length).toBe(6); // MoveC carries the via joints (RoboDK passes both joint sets)
    expect(moves[6].viaName).toBe('via');
    // the unreachable target still produces a moveL (pose kept, joints from seed)
    expect(kinds.filter((k) => k === 'moveL').length).toBe(3);
    // joint path -> 5 joint moves named "Joint path ... i"
    const jp = c.events.filter((e) => e.kind === 'moveJ' && /Joint path/.test((e as any).name));
    expect(jp.length).toBe(5);
    expect((jp[4] as any).joints[0]).toBe(4);
    expect(kinds).toEqual(expect.arrayContaining(['setFrame', 'setTool', 'setSpeed', 'setRounding', 'pause', 'setDO', 'waitDI', 'runCode', 'message', 'comment', 'gripper', 'callProgram', 'navigate', 'task', 'loop', 'if', 'thread']));
    const frames = c.events.filter((e) => e.kind === 'setFrame') as any[];
    expect(frames.map((x) => x.name)).toEqual(['F', 'World', 'F']);
    expect(getPos(frames[1].pose)).toEqual([0, 0, 0]);
    expect(getPos(frames[2].pose)).toEqual([300, 0, -100]); // frame in robot base
    const tools = c.events.filter((e) => e.kind === 'setTool') as any[];
    expect(tools.map((x) => x.name)).toEqual(['T', 'Flange', 'T']);
    expect(tools[2].toolId).toBe(tool.id);
    expect(c.events.find((e) => e.kind === 'waitDI' && (e as any).io === 'D2')).toMatchObject({ value: 0, timeoutMs: 500 });
    expect(c.events.filter((e) => e.kind === 'gripper').map((e: any) => e.close)).toEqual([true, false]);
    expect(c.events.filter((e) => e.kind === 'comment').map((e: any) => e.text)).toEqual(['note', 'show Box', 'follow path p1', 'wait move done']);
    // control flow stays structured for the posts (I6)
    expect(c.events.find((e) => e.kind === 'loop')).toMatchObject({ count: 2, body: 'Sub' });
    expect(c.events.find((e) => e.kind === 'if')).toMatchObject({ condition: 'x', then: 'Sub', else: null });
    expect(c.events.find((e) => e.kind === 'thread')).toMatchObject({ name: 'Sub' });
    expect(c.events.filter((e) => e.kind === 'callProgram').length).toBe(3);
    // a call to a program that is not in the station is still emitted as a call (RoboDK "Call program")
    expect(c.events.find((e) => e.kind === 'runCode' && (e as any).code === 'ghost')).toMatchObject({ isCall: true });
    expect(c.subprograms.length).toBe(1);
    expect(c.subprograms[0].name).toBe('Sub');
    expect(c.events.find((e) => e.kind === 'setDO' && (e as any).io === 's1')).toMatchObject({ value: 1 });
    expect(c.events.find((e) => e.kind === 'waitDI' && (e as any).io === 's2')).toMatchObject({ value: true, timeoutMs: -1 });
    expect(c.events.find((e) => e.kind === 'navigate')).toMatchObject({ x: 1000, y: 2000, heading: 45, speed: 300, label: 'dock' });
    expect(c.events.find((e) => e.kind === 'task')).toMatchObject({ task: 'spray', params: { rate: 1 } });
    expect(c.events.filter((e) => e.kind === 'pause').map((e: any) => e.timeMs)).toEqual([100, 250]);
    expect(c.events.find((e) => e.kind === 'waitDI' && (e as any).io === 's3')).toMatchObject({ value: 1, timeoutMs: 100 });
    expect(c.events.some((e) => e.kind === 'pause' && (e as any).timeMs === 999)).toBe(false);
  });
  it('compiles programs without a robot and with explicit frame/tool ids', () => {
    const { st, frame, t, tool, robot } = cell();
    const prog = st.addChild(new Program('NoRobot'));
    prog.addMoveL(t);
    prog.addMoveJ(t);
    prog.addMoveJ([1, 2, 3]);
    const c = compileForPost(st, prog);
    expect(c.robot).toBeNull();
    expect(c.dof).toBe(6);
    expect(c.jointNames).toEqual([]);
    // without a robot the targets stay relative to their frame and the frame is given w.r.t. the station
    expect(c.frame.name).toBe('F');
    expect(getPos(c.frame.pose)).toEqual([300, 0, 0]);
    expect(c.events.map((e) => e.kind)).toEqual(['setFrame', 'setTool', 'moveL', 'moveJ']); // target MoveJ has no joints
    expect(getPos((c.events[2] as any).pose)).toEqual([0, 0, 400]);
    expect((c.events[3] as any).joints).toEqual([1, 2, 3]);
    expect(c.problems).toEqual(['T1: no joint values for MoveJ (no robot), move skipped']);
    const p2 = st.addChild(new Program('Explicit'));
    p2.setRobot(robot);
    p2.frameId = frame.id;
    p2.toolId = tool.id;
    p2.addMoveL(t);
    const c2 = compileForPost(st, p2);
    expect(c2.frame.name).toBe('F');
    expect(c2.tool.name).toBe('T');
    expect(c2.events.map((e) => e.kind)).toEqual(['setFrame', 'setTool', 'moveL']);
  });
  it('shared helpers format numbers and names', () => {
    expect(f(-0.0001)).toBe('0.000');
    expect(f(-0.001, 2)).toBe('0.00');
    expect(f(-0.4, 0)).toBe('0');
    expect(f(1.23456)).toBe('1.235');
    expect(safeName('1 Pick & place!')).toBe('_1_Pick___place_');
  });
});

describe('euler ZYZ', () => {
  it('round-trips generic and degenerate poses', () => {
    const m = mul(transl(1, 2, 3), rotz(30 * DEG), roty(60 * DEG), rotz(-45 * DEG));
    const v = poseToZyz(m);
    expect(v.slice(0, 3)).toEqual([1, 2, 3]);
    expect(v[3]).toBeCloseTo(30, 9);
    expect(v[4]).toBeCloseTo(60, 9);
    expect(v[5]).toBeCloseTo(-45, 9);
    const back = zyzToPose(...v);
    for (let i = 0; i < 16; i++) expect(back[i]).toBeCloseTo(m[i], 9);
    // b = 0 / 180 singularities
    // RoboDK Pose_2_Adept puts a pure Z rotation into the last angle
    const up = poseToZyz(mul(transl(0, 0, 0), rotz(20 * DEG)));
    expect(up[3]).toBe(0);
    expect(up[4]).toBe(0);
    expect(up[5]).toBeCloseTo(20, 9);
    const down = poseToZyz(rotx(180 * DEG));
    expect(down[4]).toBeCloseTo(180, 9);
    expect(distance(getPos(zyzToPose(...down)), [0, 0, 0])).toBe(0);
  });
});

describe('Python (RoboDK) posts', () => {
  const POST_SRC = `from robodk.robomath import *\nPROG_EXT = 'jbi'\nclass RobotPost(object):\n    def ProgStart(self, name):\n        pass\n    def MoveJ(self, pose, joints, conf_RLF=None):\n        pass\n    def MoveL(self, pose, joints, conf_RLF=None):\n        pass\n`;
  it('serialises events with poses as plain arrays', () => {
    const prog = allEventsProgram();
    const arr = JSON.parse(eventsToJSON(prog));
    expect(arr.length).toBe(prog.events.length);
    expect(Array.isArray(arr[6].pose)).toBe(true);
    expect(arr[6].pose.length).toBe(16);
    expect(Array.isArray(arr[8].via)).toBe(true);
    expect(arr[8].pose.length).toBe(16);
    expect(arr[7].speed).toBe(100);
  });
  it('registers a python post whose sync generate() is a stub and runs it through a (fake) Pyodide', async () => {
    const written: Record<string, string> = {};
    const globals = new Map<string, any>();
    const runs: string[] = [];
    const fakePy = {
      FS: { mkdirTree: vi.fn(), writeFile: (p: string, c: string) => { written[p] = c; } },
      globals: { set: (k: string, v: any) => globals.set(k, v) },
      runPythonAsync: async (code: string) => {
        runs.push(code);
        if (/post_shim\.run_job_source\(__post_src, __job/.test(code)) {
          // the job carries the main program and its sub-programs (RoboDK names without blanks)
          const job = JSON.parse(globals.get('__job'));
          const files = [job, ...job.subprograms].map((p: any) => ({ name: `${p.name}.jbi`, encoding: 'utf-8', content: `PROGRAM ${p.name} for ${job.robot} dof=${job.dof}\n${p.events.map((e: any) => e.kind).join('\n')}` }));
          return JSON.stringify({ ok: true, files, log: job.subprograms.length ? 'ok' : '' });
        }
        return '';
      },
    };
    const loadPyodide = vi.fn(async (opts: any) => { expect(opts.indexURL).toMatch(/pyodide/); return fakePy; });
    vi.stubGlobal('window', { loadPyodide });
    const post = registerPythonPost('PY_test', 'TestPost', POST_SRC);
    expect(getPost('PY_test')).toBe(post);
    expect(post.name).toBe('TestPost (Python post)');
    expect(post.generate(allEventsProgram())[0].name).toBe('README.txt');
    const files = await post.generateAsync(allEventsProgram());
    // every file the post writes comes back (main + sub-program in one job, ProgSave), the post log as post_log.txt
    expect(files.map((x) => x.name)).toEqual(['AllEvents1.jbi', 'SubRoutine.jbi', 'post_log.txt']);
    expect(files[0].content).toContain('PROGRAM AllEvents1 for Universal Robots UR10e dof=6');
    expect(files[0].content).toContain('moveC');
    expect(files[0].content).toContain('callProgram');
    expect(files[2].content).toBe('ok');
    // the robodk shim modules were installed once and the shim imported
    expect(Object.keys(written)).toEqual(expect.arrayContaining(['/studio/robodk/__init__.py', '/studio/robodk/robomath.py', '/studio/robodk/robodialogs.py', '/studio/robodk/robofileio.py', '/studio/robodk/robolink.py', '/studio/robolink.py', '/studio/post_shim.py']));
    expect(written['/studio/robodk/robomath.py'].length).toBeGreaterThan(1000);
    expect(written['/studio/robodk/robolink.py']).toContain('class Robolink');
    expect(runs[0]).toContain('import post_shim');
    // second run reuses the cached Pyodide instance
    await runPythonPost(POST_SRC, allEventsProgram().subprograms[0]);
    expect(loadPyodide).toHaveBeenCalledTimes(1);
  });
  describe('user posts persisted in localStorage', () => {
    let store: Record<string, string>;
    beforeEach(() => {
      store = {};
      vi.stubGlobal('localStorage', { getItem: (k: string) => store[k] ?? null, setItem: (k: string, v: string) => { store[k] = v; }, removeItem: (k: string) => { delete store[k]; } });
    });
    it('guesses brands, validates sources and builds ids', () => {
      expect(guessBrand('KUKA_KRC4.py')).toBe('KUKA');
      expect(guessBrand('ABB_RAPID_IRC5.py')).toBe('ABB');
      expect(guessBrand('Fanuc_R30iA.py')).toBe('Fanuc');
      expect(guessBrand('Universal_Robots.py')).toBe('UR');
      expect(guessBrand('Motoman.py')).toBe('Yaskawa');
      expect(guessBrand('Staubli_VAL3.py')).toBe('Staubli');
      expect(guessBrand('Doosan.py')).toBe('Doosan');
      expect(guessBrand('Siemens_GCode.py')).toBe('CNC');
      expect(guessBrand('MyThing.py')).toBe('Custom');
      expect(looksLikeRoboDKPost(POST_SRC)).toBe(true);
      expect(looksLikeRoboDKPost('class RobotPost: pass')).toBe(false);
      expect(postIdFor('KUKA KRC4 v2.py')).toBe('PY_KUKA_KRC4_v2');
    });
    it('adds, lists, restores and removes posts', async () => {
      expect(listUserPosts()).toEqual([]);
      const post = addUserPost('Motoman_DX200.py', POST_SRC);
      expect(post.id).toBe('PY_Motoman_DX200');
      expect(post.brand).toBe('Yaskawa');
      expect(post.extension).toBe('jbi');
      expect(post.name).toBe('Motoman_DX200 (RoboDK post)');
      expect(getPost('PY_Motoman_DX200')).toBe(post);
      const list = listUserPosts();
      expect(list.length).toBe(1);
      expect(list[0]).toMatchObject({ id: 'PY_Motoman_DX200', file: 'Motoman_DX200.py', brand: 'Yaskawa', extension: 'jbi' });
      // re-adding replaces the entry; posts without PROG_EXT default to txt
      addUserPost('Motoman_DX200.py', POST_SRC.replace("PROG_EXT = 'jbi'", ''));
      expect(listUserPosts().length).toBe(1);
      expect(listUserPosts()[0].extension).toBe('txt');
      // restore: already registered ids are skipped, new ones re-registered
      store['studio.userPosts.v1'] = JSON.stringify([...listUserPosts(), { id: 'PY_Other', name: 'Other', file: 'Other.py', source: POST_SRC, addedAt: '', brand: 'Custom', extension: 'txt' }]);
      expect(restoreUserPosts()).toBe(1);
      expect(getPost('PY_Other')).toBeTruthy();
      expect(restoreUserPosts()).toBe(0);
      removeUserPost('PY_Other');
      expect(listUserPosts().map((p) => p.id)).toEqual(['PY_Motoman_DX200']);
      // import from File objects: non-python and non-post files are skipped
      const res = await importPostFiles([new File([POST_SRC], 'Comau_C5G.py'), new File(['print(1)'], 'notes.py'), new File(['x'], 'readme.txt')]);
      expect(res).toEqual({ added: ['Comau_C5G.py'], skipped: ['notes.py', 'readme.txt'] });
      expect(listUserPosts().map((p) => p.id)).toEqual(['PY_Motoman_DX200', 'PY_Comau_C5G']);
      // storage failures are tolerated
      vi.stubGlobal('localStorage', { getItem: () => { throw new Error('denied'); }, setItem: () => { throw new Error('quota'); } });
      expect(listUserPosts()).toEqual([]);
      expect(() => addUserPost('X.py', POST_SRC)).not.toThrow();
      vi.stubGlobal('localStorage', undefined);
      expect(listUserPosts()).toEqual([]);
    });
  });
});
