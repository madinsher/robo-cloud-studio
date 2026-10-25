// ROS 2 post (P1): a real ament_python package with a FollowJointTrajectory action client. Unit level: package
// layout, URDF joint names, Cartesian MoveL sampled on the straight line (FK of every point on the segment), time
// stamps from distance / speed, sub-programs inlined, IO / pauses between goals. System level (WSL Ubuntu-22.04 with
// ROS 2 Jazzy): colcon build + the node against a fake FollowJointTrajectory server that records the goals.
import { describe, it, expect } from 'vitest';
import { execFileSync } from 'node:child_process';
import { existsSync, mkdirSync, readFileSync, realpathSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { Station, Frame, Target, Tool } from '../src/core/items/item';
import { Program } from '../src/core/items/program';
import { createRobotFromLibrary } from '../src/core/items/library';
import { compileForPost, getPost } from '../src/posts/index';
import { ros2Segments } from '../src/posts/ros2';
import { transl, rotx, mul, DEG, multiply, invert } from '../src/core/math/pose';

function cell() {
  const st = new Station('ros');
  const robot = st.addChild(createRobotFromLibrary('UR5e'));
  const tool = robot.addChild(new Tool('Pen'));
  tool.setPoseTool(transl(0, 0, 100));
  robot.setTool(tool);
  const frame = st.addChild(new Frame('Table'));
  frame.setPose(transl(300, -200, 0));
  const t = (name: string, x: number, y: number, z: number) => { const tg = frame.addChild(new Target(name)); tg.setPose(mul(transl(x, y, z), rotx(180 * DEG))); return tg; };
  const a = t('A', 100, 0, 300), b = t('B', 100, 300, 300), via = t('Via', 150, 350, 300), c = t('C', 200, 300, 300);
  const sub = st.addChild(new Program('Retract'));
  sub.setRobot(robot);
  sub.addMoveL(t('Up', 200, 300, 400));
  const p = st.addChild(new Program('Draw Line'));
  p.setRobot(robot); p.setFrame(frame); p.setTool(tool);
  p.setSpeed(100, 60);
  p.addMoveJ([0, -90, 90, -90, -90, 0]);
  p.addMoveJ(a);
  p.addMoveL(b);
  p.addMoveC(via, c);
  p.setDO('3', 1);
  p.pause(250);
  p.callProgram(sub);
  return { st, robot, p, frame, tool };
}

describe('ROS 2 package post', () => {
  it('writes an ament_python package with the robot joint names and a FollowJointTrajectory client', () => {
    const { st, robot, p } = cell();
    const files = getPost('ROS2')!.generate(compileForPost(st, p));
    const names = files.map((f) => f.name);
    expect(names).toEqual(expect.arrayContaining(['drawline_ros2/package.xml', 'drawline_ros2/setup.py', 'drawline_ros2/setup.cfg', 'drawline_ros2/resource/drawline_ros2', 'drawline_ros2/drawline_ros2/__init__.py', 'drawline_ros2/drawline_ros2/drawline_node.py', 'drawline_ros2/launch/DrawLine.launch.py', 'drawline_ros2/trajectory.json']));
    const node = files.find((f) => f.name.endsWith('_node.py'))!.content;
    expect(node).toContain('from control_msgs.action import FollowJointTrajectory');
    expect(node).toContain("'/%s/follow_joint_trajectory' % controller");
    expect(node).toContain(`JOINT_NAMES = ${JSON.stringify(robot.jointNames())}`);
    expect(files.find((f) => f.name.endsWith('package.xml'))!.content).toContain('<build_type>ament_python</build_type>');
    expect(files.find((f) => f.name.endsWith('setup.py'))!.content).toContain("'drawline_node = drawline_ros2.drawline_node:main'");
  });
  it('samples MoveL / MoveC in Cartesian space: every point lies on the programmed line / arc, timed by the linear speed', () => {
    const { st, robot, p, frame, tool } = cell();
    const c = compileForPost(st, p);
    const { segs, warnings } = ros2Segments(c);
    expect(warnings.filter((w) => /not reachable/.test(w))).toEqual([]);
    expect(segs.map((s) => s.type)).toEqual(['traj', 'io', 'pause', 'traj']);
    const tr = segs[0] as any;
    expect(tr.points[0].first).toBe(true); // first point timed from the current robot state at run time
    const fk = (q: number[]) => multiply(invert(multiply(invert(robot.poseAbs()), frame.poseAbs())), robot.solveFK(q.map((v) => v / DEG), tool.poseTool()));
    // points of the MoveL A -> B: x = 100, z = 300, y from 0 to 300 (straight line in the Table frame)
    const iB = tr.points.findIndex((pt: any) => pt.name === 'B');
    const iA = tr.points.findIndex((pt: any) => pt.name === 'A');
    expect(iB - iA).toBeGreaterThanOrEqual(300 / 5);
    for (let k = iA + 1; k <= iB; k++) {
      const m = fk(tr.points[k].positions);
      expect(Math.abs(m[12] - 100), `line point ${k}`).toBeLessThan(0.05);
      expect(Math.abs(m[14] - 300), `line point ${k}`).toBeLessThan(0.05);
      expect(m[13]).toBeGreaterThan(-0.05);
      expect(m[13]).toBeLessThan(300.05);
    }
    // 300 mm at 100 mm/s = 3 s for the segment
    const dur = tr.points.slice(iA + 1, iB + 1).reduce((a: number, pt: any) => a + pt.dt, 0);
    expect(dur).toBeCloseTo(3, 1);
    // MoveC B -> C through Via: all points at the circle's radius from its centre (150, 300, 300), r = 50
    const iC = tr.points.findIndex((pt: any) => pt.name === 'C');
    for (let k = iB + 1; k <= iC; k++) {
      const m = fk(tr.points[k].positions);
      expect(Math.hypot(m[12] - 150, m[13] - 300), `arc point ${k}`).toBeCloseTo(50, 1);
    }
    // the called sub-program is inlined after the IO and the pause
    expect((segs[3] as any).points.at(-1).name).toBe('Up');
    expect(segs[1]).toEqual({ type: 'io', name: '3', value: 1 });
    expect(segs[2]).toEqual({ type: 'pause', seconds: 0.25 });
  });

  const WSL = process.platform === 'win32' && existsSync('C:/Windows/System32/wsl.exe');
  let jazzy = false;
  if (WSL) { try { jazzy = execFileSync('wsl.exe', ['-d', 'Ubuntu-22.04', '--', 'bash', '-lc', 'test -f /opt/ros/jazzy/setup.bash && echo yes'], { encoding: 'utf8', timeout: 60_000 }).includes('yes'); } catch { jazzy = false; } }
  it('finds WSL Ubuntu-22.04 with ROS 2 Jazzy for the system test', () => { expect(jazzy, 'WSL / ROS 2 Jazzy not available: colcon test below is skipped').toBe(true); });
  it.skipIf(!jazzy)('colcon builds the package and the node drives a FollowJointTrajectory server from the current state', () => {
    const { st, robot, p } = cell();
    const files = getPost('ROS2')!.generate(compileForPost(st, p), { controller: 'fake_controller' });
    const dir = path.join(realpathSync.native(tmpdir()), `vbs_ros2_${process.pid}`);
    rmSync(dir, { recursive: true, force: true });
    for (const f of files) { const fp = path.join(dir, f.name); mkdirSync(path.dirname(fp), { recursive: true }); writeFileSync(fp, f.content); }
    writeFileSync(path.join(dir, 'names.json'), JSON.stringify(robot.jointNames()));
    writeFileSync(path.join(dir, 'fake_jtc.py'), [
      'import json, sys, rclpy', 'from rclpy.node import Node', 'from rclpy.action import ActionServer', 'from control_msgs.action import FollowJointTrajectory', 'from sensor_msgs.msg import JointState',
      'names = json.load(open(sys.argv[1])); out = sys.argv[2]', 'class Fake(Node):', '    def __init__(self):', "        super().__init__('fake_jtc')", '        self.goals = []; self.q = [0.0] * len(names)',
      "        self.pub = self.create_publisher(JointState, '/joint_states', 10)", '        self.create_timer(0.05, self.tick)', "        self._srv = ActionServer(self, FollowJointTrajectory, '/fake_controller/follow_joint_trajectory', self.execute)",
      '    def tick(self):', '        m = JointState(); m.name = names; m.position = self.q; m.header.stamp = self.get_clock().now().to_msg(); self.pub.publish(m)',
      '    def execute(self, gh):', '        t = gh.request.trajectory', "        self.goals.append({'joint_names': list(t.joint_names), 'points': [[list(p.positions), p.time_from_start.sec + p.time_from_start.nanosec * 1e-9] for p in t.points]})",
      '        if t.points: self.q = list(t.points[-1].positions)', "        json.dump(self.goals, open(out, 'w'))", '        gh.succeed()', '        r = FollowJointTrajectory.Result(); r.error_code = 0; return r',
      'rclpy.init(); n = Fake()', 'try:', '    rclpy.spin(n)', 'except KeyboardInterrupt:', '    pass'].join('\n'));
    // the fake controller and the generated node run in ONE rclpy process: FastDDS does not deliver between processes in
    // this WSL (scratchpad reports/ros.md 9); the node code is the installed package module, unchanged
    writeFileSync(path.join(dir, 'inproc.py'), "import json, sys, threading, time\nimport rclpy\nfrom rclpy.executors import MultiThreadedExecutor\nsys.path.insert(0, sys.argv[3])\nimport importlib\nfake_src = open(sys.argv[1]).read().split('rclpy.init()')[0]\nns = {}\nnames = json.load(open(sys.argv[2]))\nsys.argv = ['x', sys.argv[2], sys.argv[4]]\nexec(fake_src, ns)\nrclpy.init()\nfake = ns['Fake']()\nex = MultiThreadedExecutor()\nex.add_node(fake)\nth = threading.Thread(target=ex.spin, daemon=True); th.start()\nmod = importlib.import_module('drawline_ros2.drawline_node')\nfrom rclpy.parameter import Parameter\nrunner = mod.ProgramRunner()\nrunner.set_parameters([Parameter('controller', Parameter.Type.STRING, 'fake_controller')])\nrunner.client = mod.ActionClient(runner, mod.FollowJointTrajectory, '/fake_controller/follow_joint_trajectory')\ncode = runner.run()\nprint('NODE_EXIT=%d' % code)\nex.shutdown(); rclpy.shutdown()\n");
    const wdir = `/mnt/${dir[0].toLowerCase()}${dir.slice(2).split(path.sep).join('/')}`;
    writeFileSync(path.join(dir, 'run.sh'), ['source /opt/ros/jazzy/setup.bash', 'WS=$HOME/vbs_t4_ws; rm -rf "$WS"; mkdir -p "$WS/src"', `cp -r "${wdir}/drawline_ros2" "$WS/src/"`, 'cd "$WS"',
      'colcon build --packages-select drawline_ros2 > build.log 2>&1 || { tail -30 build.log; exit 3; }', 'source install/setup.bash',
      `timeout 120 python3 "${wdir}/inproc.py" "${wdir}/fake_jtc.py" "${wdir}/names.json" "$WS/install/drawline_ros2/lib/python3.12/site-packages" "${wdir}/goals.json" 2>&1 | tail -15`].join(String.fromCharCode(10)));
    const out = execFileSync('wsl.exe', ['-d', 'Ubuntu-22.04', '--', 'bash', `${wdir}/run.sh`], { encoding: 'utf8', timeout: 400_000 });
    expect(out, out).toContain('NODE_EXIT=0');
    const goals = JSON.parse(readFileSync(path.join(dir, 'goals.json'), 'utf8'));
    const traj = JSON.parse(files.find((f) => f.name.endsWith('trajectory.json'))!.content);
    const trajSegs = traj.segments.filter((s: any) => s.type === 'traj');
    expect(goals.length).toBe(trajSegs.length);
    goals.forEach((g: any, k: number) => {
      expect(g.joint_names).toEqual(robot.jointNames());
      expect(g.points.length).toBe(trajSegs[k].points.length);
      for (let i = 1; i < g.points.length; i++) expect(g.points[i][1]).toBeGreaterThan(g.points[i - 1][1]);
    });
    // first point: from the fake robot's zero pose at 60 deg/s
    const q0 = goals[0].points[0][0];
    expect(goals[0].points[0][1]).toBeCloseTo(Math.max(0.5, Math.max(...q0.map((v: number) => Math.abs(v))) / (60 * DEG)), 2);
    rmSync(dir, { recursive: true, force: true });
  }, 600_000);
});
