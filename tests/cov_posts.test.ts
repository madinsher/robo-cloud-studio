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

/** Posts that echo target names into their output. */
const LABELLED = ['KUKA_KRC4', 'ABB_RAPID_IRC5', 'Fanuc_R30iA', 'Universal_Robots', 'RoboDK_Python', 'Generic', 'JSON', 'KUKA_KRC2', 'Fanuc_RJ3', 'ABB_RAPID_S4C'];

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
      expect(main.name.toLowerCase(), post.id).toMatch(new RegExp(`\\.${post.extension.toLowerCase()}$`));
      expect(main.content.length, post.id).toBeGreaterThan(100);
      const all = files.map((x) => x.content).join('\n');
      // posts that label their moves carry every target name; all posts carry the home joints (-90) and the pick X (300 mm / 0.3 m)
      if (LABELLED.includes(post.id)) for (const label of ['Home', 'Pick 1', 'Place']) expect(all, `${post.id} ${label}`).toContain(label);
      // no NaN or undefined leaks
      expect(main.content, post.id).not.toMatch(/NaN|undefined/);
      // sub program is emitted (as extra file or inline)
      if (post.id !== 'ROS2') expect(all, post.id).toMatch(/Sub_Routine|SubRoutine|SUB_ROUTINE|Sub Routine/i); // ROS2 emits one flat trajectory
    }
  });

  it('vendor specifics: KUKA, ABB, Fanuc, UR, ROS2, RoboDK, Generic, JSON', () => {
    const prog = allEventsProgram();
    const kuka = getPost('KUKA_KRC4')!.generate(prog);
    expect(kuka.map((x) => x.name)).toEqual(['ALL_EVENTS_1.src', 'ALL_EVENTS_1.dat', 'SUB_ROUTINE.src', 'SUB_ROUTINE.dat']);
    const src = kuka[0].content;
    expect(src).toContain('DEF ALL_EVENTS_1 ( )');
    expect(src).toContain('$BASE = {FRAME: {X 500.000,Y 0.000,Z 0.000,A 0.000,B 0.000,C 0.000}} ; Table 1');
    expect(src).toContain('$VEL.CP = 0.250');
    expect(src).toContain('$ACC.CP = 1.200');
    expect(src).toContain('$VEL_AXIS[1] = 17');
    expect(src).toContain('$APO.CDIS = 10.0');
    expect(src).toContain('PTP {A1 10.000,A2 -90.000,A3 90.000,A4 -90.000,A5 -90.000,A6 0.000,E1 0.000,E2 0.000,E3 0,E4 0,E5 0,E6 0} C_PTP ; Home');
    expect(src).toContain('LIN {X 300.000,Y 100.000,Z 200.000,A 0.000,B 0.000,C 180.000} C_DIS ; Pick 1');
    expect(src).toMatch(/CIRC \{X 350.000.*\}, \{X 400.000.*\} C_DIS ; Arc/);
    expect(src).toMatch(/LIN \{X 400.000,Y -100.000.*\} ; Place/); // rounding 0 -> no C_DIS
    expect(src).toContain('WAIT SEC 0.500');
    expect(src).toContain('HALT');
    expect(src).toContain('$OUT[3] = TRUE');
    expect(src).toContain('$OUT[gripper] = FALSE');
    expect(src).toContain('WAIT FOR $IN[7] == FALSE');
    expect(src).toContain('MyRoutine()');
    expect(src).toContain("MsgNotify(\"Cycle 'done'\")");
    expect(src).toContain('SUB_ROUTINE()');
    expect(src).toContain('$OUT[1] = TRUE ; gripper close');
    expect(src).toContain('; NAVIGATE X 1500.000 Y -200.000');
    expect(src).toContain('; TASK harvest {"rows":2}');
    expect(src).toContain('E1 100.000'); // 7th axis

    const abb = getPost('ABB_RAPID_IRC5')!.generate(prog)[0].content;
    expect(abb).toContain('MODULE All_Events_1');
    expect(abb).toContain('PERS tooldata Gripper := [TRUE,[[0.000,0.000,120.000],[1.000000,0.000000,0.000000,0.000000]],[1,[0,0,50],[1,0,0,0],0,0,0]];');
    expect(abb).toContain('PERS wobjdata Table_1 :=');
    expect(abb).toContain('PERS speeddata vbs_v250 := [250,60,5000,1000];');
    expect(abb).toMatch(/CONST jointtarget jt_Home_1 := \[\[10.000,-90.000,90.000,-90.000,-90.000,0.000\],\[9E9,9E9,9E9,9E9,9E9,9E9\]\];/);
    expect(abb).toMatch(/CONST jointtarget jt_Home_2_\d+ := \[\[.*\],\[100.000,9E9/); // external axis
    expect(abb).toContain('MoveAbsJ jt_Home_1,vbs_v250,z10,Gripper\\WObj:=Table_1;');
    expect(abb).toMatch(/MoveL \[\[300.000,100.000,200.000\],\[0.000000,1.000000,0.000000,0.000000\],\[0,-2,0,0\],\[9E9,9E9,9E9,9E9,9E9,9E9\]\],vbs_v250,z10,Gripper\\WObj:=Table_1;  ! Pick 1/);
    expect(abb).toMatch(/MoveC .*,vbs_v250,z10,Gripper/);
    expect(abb).toMatch(/MoveL \[\[400.000,-100.000,300.000\].*,fine,Gripper/);
    expect(abb).toContain('WaitTime 0.50;');
    expect(abb).toContain('Stop;');
    expect(abb).toContain('SetDO DO3, 1;');
    expect(abb).toContain('WaitDI DI7, 0;');
    expect(abb).toContain('MyRoutine;');
    expect(abb).toContain('x = 1');
    expect(abb).toContain("TPWrite \"Cycle 'done'\";");
    expect(abb).toContain('Sub_Routine;');
    expect(abb).toContain('PROC Sub_Routine()');
    expect(abb).toContain('SetDO doGripper, 1;');
    expect(abb.trim().endsWith('ENDMODULE')).toBe(true);

    const fanuc = getPost('Fanuc_R30iA')!.generate(prog, { uframe: 2, utool: 3 });
    expect(fanuc.length).toBe(2);
    const ls = fanuc[0].content;
    expect(ls).toContain('/PROG  ALL_EVENTS_1');
    expect(ls).toContain('UFRAME_NUM=2');
    expect(ls).toContain('UFRAME_NUM=3'); // setFrame increments
    expect(ls).toContain('UTOOL_NUM=4');
    expect(ls).toMatch(/J P\[1\] 15% CNT10 ;/); // speedJoints 30 -> 15 %
    expect(ls).toMatch(/L P\[2\] 100mm\/sec CNT10 ;/);
    expect(ls).toMatch(/C P\[3\]\s+;\n\s*\d+:\s+P\[4\] 250mm\/sec CNT10 ;/);
    expect(ls).toMatch(/L P\[5\] 250mm\/sec FINE ;/);
    expect(ls).toContain('WAIT 0.50(sec)');
    expect(ls).toContain('PAUSE');
    expect(ls).toContain('DO[3]=ON');
    expect(ls).toContain('DO[1]=OFF');
    expect(ls).toContain('WAIT DI[7]=OFF');
    expect(ls).toContain('CALL MYROUTINE');
    expect(ls).toContain('CALL SUB_ROUTINE');
    expect(ls).toContain('RO[1]=ON');
    expect(ls).toContain('P[1:"Home"]{');
    expect(ls).toMatch(/J3=\s+0.000 deg/); // J3 = j3 + j2 = 0
    expect(ls).toContain('/POS');
    expect(ls).toContain("CONFIG : 'F U B', 0, -1, 0"); // Pick 1 joints: j5 -90 -> F/B, j4 -100
    expect(ls).toMatch(/LINE_COUNT\t= \d+;/);

    const ur = getPost('Universal_Robots')!.generate(prog)[0].content;
    expect(ur).toContain('def All_Events_1():');
    expect(ur).toContain('set_tcp(p[0.000000, 0.000000, 0.120000, 0.000000, 0.000000, 0.000000])  # Gripper');
    expect(ur).toContain('frame = p[0.500000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000]  # Table 1');
    expect(ur).toMatch(/movej\(\[0.174533, -1.570796, 1.570796, -1.570796, -1.570796, 0.000000\], a=5.236, v=0.524, r=0.0100\)  # Home/);
    expect(ur).toMatch(/movel\(pose_trans\(frame, p\[0.300000, 0.100000, 0.200000, 3.141593, 0.000000, 0.000000\]\), a=1.200, v=0.100, r=0.0100\)  # Pick 1/);
    expect(ur).toMatch(/movec\(pose_trans\(frame, p\[0.350000.*\), pose_trans\(frame, p\[0.400000.*\), a=1.200, v=0.250, r=0.0100\)  # Arc/);
    expect(ur).toMatch(/movel\(.*Place/);
    expect(ur).toContain('sleep(0.500)');
    expect(ur).toContain('popup("Paused: press continue", blocking=True)');
    expect(ur).toContain('set_standard_digital_out(3, True)');
    expect(ur).toContain('set_standard_digital_out(0, False)');
    expect(ur).toContain('while get_standard_digital_in(7) != False:');
    expect(ur).toContain('  MyRoutine()');
    expect(ur).toContain('  x = 1');
    expect(ur).toContain("textmsg(\"Cycle 'done'\")");
    expect(ur).toContain('  Sub_Routine()');
    expect(ur).toContain('set_tool_digital_out(0, True)  # gripper close');
    expect(ur.indexOf('def Sub_Routine():')).toBeLessThan(ur.indexOf('def All_Events_1():'));
    expect(ur.trim().endsWith('All_Events_1()')).toBe(true);

    const ros = getPost('ROS2')!.generate(prog, { topic: '/arm/traj' });
    expect(ros.map((x) => x.name)).toEqual(['All_Events_1_ros2.py', 'All_Events_1_trajectory.json']);
    const traj = JSON.parse(ros[1].content);
    expect(traj.joint_names.length).toBe(6);
    expect(traj.points.length).toBe(3); // Home, Pick 1, Home 2 (moveC / Place carry no joints)
    expect(traj.points[0].positions[0]).toBeCloseTo(10 * DEG, 6);
    expect(traj.points[0].time_from_start).toEqual({ sec: 0, nanosec: 500000000 });
    expect(traj.points[1].time_from_start.sec).toBeGreaterThanOrEqual(0);
    expect(traj.waypoints.some((w: any) => w.navigate && w.navigate.x === 1.5)).toBe(true);
    expect(traj.waypoints.some((w: any) => w.set_io === 'DO3' && w.value === true)).toBe(true);
    expect(traj.waypoints[0].position.x).toBeCloseTo(getPos(prog.robot!.solveFK([10, -90, 90, -90, -90, 0]))[0] / 1000, 6);
    expect(ros[0].content).toContain("'/arm/traj'");
    expect(ros[0].content).toContain('class All_Events_1Node(Node):');
    expect(ros[1].mime).toBe('application/json');

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
    expect(compiled.events[0]).toMatchObject({ kind: 'setSpeed', speedLinear: 600, speedJoints: 120 });
    // every moveL has a joint solution and its pose is expressed in the Table frame
    const first = compiled.events.find((e) => e.kind === 'moveL') as any;
    expect(first.joints).toBeTruthy();
    expect(getPos(first.pose).map((v) => Math.round(v))).toEqual([100, 150, 622]);
    for (const post of listPosts()) {
      const files = post.generate(compiled);
      expect(files.length, post.id).toBeGreaterThan(0);
      expect(files[0].name.toLowerCase(), post.id).toMatch(new RegExp(`\\.${post.extension.toLowerCase()}$`));
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
    expect(kinds.slice(0, 7)).toEqual(['moveJ', 'moveL', 'moveC', 'moveJ', 'moveL', 'moveJ', 'moveC']);
    expect(c.problems).toEqual(expect.arrayContaining([expect.stringMatching(/T1: MoveC needs via/), expect.stringMatching(/Far: unreachable/), expect.stringMatching(/no joint solution for MoveJ/), expect.stringMatching(/no pose for MoveL/), 'Program ghost not found']));
    expect(c.problems.length).toBe(5);
    const moveL = c.events[1] as any;
    expect(moveL.speed).toBe(80);
    expect(moveL.rounding).toBe(2);
    // pose w.r.t. frame F: target is 400 up in F
    expect(getPos(moveL.pose).map(Math.round)).toEqual([0, 0, 400]);
    expect(moveL.joints.length).toBe(6);
    const mc = c.events[2] as any;
    expect(mc.viaName).toBe('Via');
    expect(getPos(mc.via).map(Math.round)).toEqual([50, 50, 400]);
    expect((c.events[6] as any).viaName).toBe('via');
    // the unreachable target still produces a moveL (pose kept, joints from seed)
    expect(kinds.filter((k) => k === 'moveL').length).toBe(3);
    // joint path -> 5 joint moves named "Joint path ... i"
    const jp = c.events.filter((e) => e.kind === 'moveJ' && /Joint path/.test((e as any).name));
    expect(jp.length).toBe(5);
    expect((jp[4] as any).joints[0]).toBe(4);
    expect(kinds).toEqual(expect.arrayContaining(['setFrame', 'setTool', 'setSpeed', 'setRounding', 'pause', 'setDO', 'waitDI', 'runCode', 'message', 'comment', 'gripper', 'callProgram', 'navigate', 'task']));
    const frames = c.events.filter((e) => e.kind === 'setFrame') as any[];
    expect(frames.map((x) => x.name)).toEqual(['World', 'F']);
    expect(getPos(frames[0].pose)).toEqual([0, 0, 0]);
    expect(getPos(frames[1].pose)).toEqual([300, 0, -100]); // frame in robot base
    const tools = c.events.filter((e) => e.kind === 'setTool') as any[];
    expect(tools.map((x) => x.name)).toEqual(['Flange', 'T']);
    expect(tools[1].toolId).toBe(tool.id);
    expect(c.events.find((e) => e.kind === 'waitDI' && (e as any).io === 'D2')).toMatchObject({ value: 0, timeoutMs: 500 });
    expect(c.events.filter((e) => e.kind === 'gripper').map((e: any) => e.close)).toEqual([true, false]);
    expect(c.events.filter((e) => e.kind === 'comment').map((e: any) => e.text)).toEqual(['note', 'event show', 'follow path p1', 'loop (not supported by post)', 'if (not supported by post)', 'start thread Sub', 'wait move done']);
    expect(c.events.filter((e) => e.kind === 'callProgram').length).toBe(3);
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
    expect(c.frame.name).toBe('World');
    expect(c.events.length).toBe(2); // MoveL (pose relative to station) + inline joints; target MoveJ has no joints
    expect(getPos((c.events[0] as any).pose)).toEqual([300, 0, 400]);
    expect(c.problems).toEqual(['T1: no joint solution for MoveJ']);
    const p2 = st.addChild(new Program('Explicit'));
    p2.setRobot(robot);
    p2.frameId = frame.id;
    p2.toolId = tool.id;
    p2.addMoveL(t);
    const c2 = compileForPost(st, p2);
    expect(c2.frame.name).toBe('F');
    expect(c2.tool.name).toBe('T');
    expect(c2.events.length).toBe(1);
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
    const up = poseToZyz(mul(transl(0, 0, 0), rotz(20 * DEG)));
    expect(up[3]).toBeCloseTo(20, 9);
    expect(up[4]).toBe(0);
    expect(up[5]).toBe(0);
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
        if (/post_shim\.run_from_json/.test(code)) {
          const events = JSON.parse(globals.get('__events'));
          return JSON.stringify({ program: `PROGRAM ${globals.get('__prog')} for ${globals.get('__robot')} dof=${globals.get('__dof')}\n${events.map((e: any) => e.kind).join('\n')}`, extension: 'jbi', log: globals.get('__prog') === 'Sub Routine' ? '' : 'ok' });
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
    expect(files.map((x) => x.name)).toEqual(['All_Events_1.jbi', 'All_Events_1.log', 'Sub_Routine.jbi']);
    expect(files[0].content).toContain('PROGRAM All Events 1 for Universal Robots UR10e dof=6');
    expect(files[0].content).toContain('moveC');
    expect(files[1].content).toBe('ok');
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
