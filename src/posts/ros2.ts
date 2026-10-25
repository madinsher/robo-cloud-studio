/**
 * ROS 2 post: a complete ament_python package that executes the program on a ros2_control robot through the
 * FollowJointTrajectory action of its joint trajectory controller:
 *   <pkg>/package.xml, setup.py, setup.cfg, resource/<pkg>, <pkg>/__init__.py, <pkg>/<prog>_node.py,
 *   launch/<prog>.launch.py, trajectory.json (inspection copy), README.md
 * The node waits for the action server and for /joint_states, starts from the CURRENT robot state (the first point's
 * time is computed at run time from the distance and the joint speed), sends one goal per motion segment, and executes
 * pauses, IO (ur_msgs SetIO when available), input waits (ur_msgs IOStates) and messages between segments.
 * MoveL / MoveC are sampled in Cartesian space and solved with the robot model (dense joint points on the straight line /
 * arc); joint names are the robot's (URDF) joint names, overridable with the `joint_names` parameter.
 * Validated with `colcon build` and a fake FollowJointTrajectory server in ROS 2 Jazzy (tests/posts_ros2.test.ts).
 */
import { PostProcessor, PostProgram, PostFile, PostEvent, registerPost } from './base';
import { Mat4, DEG, multiply, invert, identity } from '../core/math/pose';
import { arcPoses, postProgName } from './robopost';
import { Robot } from '../core/items/robot';

type Seg =
  | { type: 'traj'; points: Array<{ positions: number[]; dt: number; first?: boolean; name?: string }>; vmax: number }
  | { type: 'pause'; seconds: number }
  | { type: 'io'; name: string; value: number }
  | { type: 'wait'; name: string; value: number; timeout: number }
  | { type: 'msg'; text: string };

const pkgName = (prog: string) => { const s = postProgName(prog).toLowerCase().replace(/[^a-z0-9_]/g, '_').replace(/^[^a-z]+/, ''); return `${s || 'program'}_ros2`; };
const pyStr = (s: string) => JSON.stringify(s);

function interpPose(a: Mat4, b: Mat4, t: number): Mat4 {
  // position lerp + rotation slerp (via arcPoses' quaternion code would need 3 points; use matrix log-free nlerp)
  const qa = quat(a), qb0 = quat(b);
  const dot = qa[0] * qb0[0] + qa[1] * qb0[1] + qa[2] * qb0[2] + qa[3] * qb0[3];
  const qb = dot < 0 ? qb0.map((v) => -v) : qb0;
  const th = Math.acos(Math.min(1, Math.abs(dot)));
  const s0 = th < 1e-9 ? 1 - t : Math.sin((1 - t) * th) / Math.sin(th), s1 = th < 1e-9 ? t : Math.sin(t * th) / Math.sin(th);
  const q = qa.map((v, i) => v * s0 + qb[i] * s1);
  const m = fromQuat(q);
  m[12] = a[12] + (b[12] - a[12]) * t; m[13] = a[13] + (b[13] - a[13]) * t; m[14] = a[14] + (b[14] - a[14]) * t;
  return m;
}
function quat(m: Mat4): number[] {
  const tr = m[0] + m[5] + m[10];
  if (tr > 0) { const s = Math.sqrt(tr + 1) * 2; return [0.25 * s, (m[6] - m[9]) / s, (m[8] - m[2]) / s, (m[1] - m[4]) / s]; }
  if (m[0] > m[5] && m[0] > m[10]) { const s = Math.sqrt(1 + m[0] - m[5] - m[10]) * 2; return [(m[6] - m[9]) / s, 0.25 * s, (m[4] + m[1]) / s, (m[8] + m[2]) / s]; }
  if (m[5] > m[10]) { const s = Math.sqrt(1 + m[5] - m[0] - m[10]) * 2; return [(m[8] - m[2]) / s, (m[4] + m[1]) / s, 0.25 * s, (m[9] + m[6]) / s]; }
  const s = Math.sqrt(1 + m[10] - m[0] - m[5]) * 2; return [(m[1] - m[4]) / s, (m[8] + m[2]) / s, (m[9] + m[6]) / s, 0.25 * s];
}
function fromQuat(q: number[]): Mat4 {
  const n = Math.hypot(...q) || 1, [w, x, y, z] = q.map((v) => v / n);
  const m = identity();
  m[0] = 1 - 2 * (y * y + z * z); m[4] = 2 * (x * y - w * z); m[8] = 2 * (x * z + w * y);
  m[1] = 2 * (x * y + w * z); m[5] = 1 - 2 * (x * x + z * z); m[9] = 2 * (y * z - w * x);
  m[2] = 2 * (x * z - w * y); m[6] = 2 * (y * z + w * x); m[10] = 1 - 2 * (x * x + y * y);
  return m;
}

/** Turn the compiled program (sub-programs inlined, counted loops expanded) into ROS execution segments. */
export function ros2Segments(p: PostProgram, opts: { stepMm?: number; stepDeg?: number } = {}): { segs: Seg[]; warnings: string[]; jointNames: string[] } {
  const robot: Robot | null = p.robot;
  const dof = p.dof;
  const jointNames = p.jointNames.length >= dof ? p.jointNames.slice(0, dof) : Array.from({ length: dof }, (_, i) => `joint_${i + 1}`);
  const prism = robot ? robot.chain.joints.filter((j) => j.type !== 'fixed' && !j.mimic).map((j) => j.type === 'prismatic') : jointNames.map(() => false);
  const conv = (q: number[]) => q.slice(0, dof).map((v, i) => +(prism[i] ? v / 1000 : v * DEG).toFixed(9));
  const warnings: string[] = [];
  const segs: Seg[] = [];
  const subs = new Map(p.subprograms.map((s) => [s.name, s]));
  let vj = robot?.motion?.speedJoints ?? 90, vl = robot?.motion?.speedLinear ?? 250;
  let tool: Mat4 = p.tool.pose, frame: Mat4 = p.frame.pose;
  let prev: number[] | null = null;
  let cur: Extract<Seg, { type: 'traj' }> | null = null;
  const stepMm = opts.stepMm ?? 5, stepDeg = opts.stepDeg ?? 2;
  const flush = () => { if (cur && cur.points.length) segs.push(cur); cur = null; };
  const traj = () => (cur ??= { type: 'traj', points: [], vmax: vj * DEG });
  const jointDt = (a: number[] | null, b: number[]) => (a ? Math.max(0.05, ...b.slice(0, dof).map((v, i) => Math.abs(v - a[i]) / (prism[i] ? vl : vj))) : 0);
  const addPoint = (q: number[], name: string, dt?: number) => {
    const t = traj();
    t.points.push({ positions: conv(q), dt: dt ?? jointDt(prev, q), ...(prev ? {} : { first: true }), name });
    prev = q.slice(0, dof);
  };
  /** Dense joint points along Cartesian poses (in the program frame); joint interpolation when IK fails. */
  const cartesian = (poses: Mat4[], qEnd: number[] | null, name: string) => {
    if (!robot || !prev) { if (qEnd) addPoint(qEnd, name); return; }
    const start = multiply(invert(frame), robot.solveFK(prev, tool));
    const pts: Mat4[] = [];
    let last = start;
    for (const target of poses) {
      const d = Math.hypot(target[12] - last[12], target[13] - last[13], target[14] - last[14]);
      const qa = quat(last), qb = quat(target), ang = 2 * Math.acos(Math.min(1, Math.abs(qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3]))) / DEG;
      const n = Math.max(1, Math.ceil(Math.max(d / stepMm, ang / stepDeg)));
      for (let k = 1; k <= n; k++) pts.push(interpPose(last, target, k / n));
      last = target;
    }
    let seed = prev, q: number[] | null = null, prevPose = start;
    const out: Array<{ q: number[]; dt: number }> = [];
    for (const m of pts) {
      const r = robot.solveIK(multiply(frame, m), { seed }, tool);
      if (!r.ok) { warnings.push(`${name}: Cartesian path not reachable, joint interpolation used`); out.length = 0; break; }
      q = r.joints;
      const d = Math.hypot(m[12] - prevPose[12], m[13] - prevPose[13], m[14] - prevPose[14]);
      out.push({ q, dt: Math.max(d / vl, jointDt(seed, q), 0.01) });
      seed = q; prevPose = m;
    }
    if (!out.length) { if (qEnd) addPoint(qEnd, name); return; }
    // keep the programmed end joints (configuration) for the last point
    if (qEnd) out[out.length - 1].q = qEnd.slice(0, dof);
    out.forEach((o, i) => addPoint(o.q, i === out.length - 1 ? name : `${name} ${i + 1}`, o.dt));
  };
  const walk = (events: PostEvent[], depth: number) => {
    if (depth > 16) { warnings.push('call depth > 16: recursion stopped'); return; }
    for (const ev of events) {
      switch (ev.kind) {
        case 'moveJ': addPoint(ev.joints, ev.name); break;
        case 'moveL': if (ev.pose) cartesian([ev.pose], ev.joints, ev.name); else if (ev.joints) addPoint(ev.joints, ev.name); break;
        case 'moveC': {
          const start = robot && prev ? multiply(invert(frame), robot.solveFK(prev, tool)) : null;
          const arc = start ? arcPoses(start, ev.via, ev.pose, stepDeg) : null;
          cartesian(arc ? arc.poses : [ev.via, ev.pose], ev.joints, ev.name);
          break;
        }
        case 'setSpeed': if (ev.speedJoints !== undefined) vj = ev.speedJoints; if (ev.speedLinear !== undefined) vl = ev.speedLinear; break;
        case 'setFrame': frame = ev.pose; break;
        case 'setTool': tool = ev.pose; break;
        case 'pause': flush(); segs.push({ type: 'pause', seconds: ev.timeMs < 0 ? -1 : ev.timeMs / 1000 }); break;
        case 'setDO': flush(); segs.push({ type: 'io', name: String(ev.io), value: Number(ev.value) ? 1 : 0 }); break;
        case 'waitDI': flush(); segs.push({ type: 'wait', name: String(ev.io), value: Number(ev.value) ? 1 : 0, timeout: ev.timeoutMs > 0 ? ev.timeoutMs / 1000 : -1 }); break;
        case 'message': flush(); segs.push({ type: 'msg', text: ev.text }); break;
        case 'callProgram': { const s = subs.get(ev.name); if (s) walk(s.events, depth + 1); else warnings.push(`call ${ev.name}: program not available`); break; }
        case 'loop': { const s = subs.get(ev.body); if (!s) break; const n = ev.count > 0 ? Math.min(ev.count, 1000) : 1; if (ev.count <= 0) warnings.push(`endless loop ${ev.body}: executed once`); for (let k = 0; k < n; k++) walk(s.events, depth + 1); break; }
        case 'runCode': if (ev.isCall) { const s = subs.get(ev.code); if (s) walk(s.events, depth + 1); else warnings.push(`call ${ev.code}: not executable in ROS (skipped)`); } break;
        case 'if': warnings.push(`IF ${ev.condition}: not evaluated in ROS (skipped)`); break;
        case 'thread': warnings.push(`thread ${ev.name}: not supported (skipped)`); break;
        default: break;
      }
    }
  };
  walk(p.events, 0);
  flush();
  return { segs, warnings, jointNames };
}

const NODE_PY = (o: { pkg: string; prog: string; controller: string; jointNames: string[]; segs: Seg[]; warnings: string[] }) => `#!/usr/bin/env python3
"""${o.prog}: generated by VerticalBot Studio. Executes the program through the FollowJointTrajectory action of a
ros2_control joint trajectory controller (default /${o.controller}/follow_joint_trajectory), starting from the robot's
current /joint_states. Parameters: controller, joint_names, speed_scale, min_start_time.
"""
import json
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

JOINT_NAMES = ${JSON.stringify(o.jointNames)}
SEGMENTS = json.loads(r'''${JSON.stringify(o.segs)}''')
WARNINGS = ${JSON.stringify(o.warnings)}


def to_duration(t):
    sec = int(t)
    return Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1000000000)


class ProgramRunner(Node):
    def __init__(self):
        super().__init__(${pyStr(`${postProgName(o.prog).toLowerCase().replace(/[^a-z0-9_]/g, '_')}_runner`)})
        self.declare_parameter('controller', ${pyStr(o.controller)})
        self.declare_parameter('joint_names', JOINT_NAMES)
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('min_start_time', 0.5)
        self.joint_names = list(self.get_parameter('joint_names').value)
        controller = self.get_parameter('controller').value
        self.client = ActionClient(self, FollowJointTrajectory, '/%s/follow_joint_trajectory' % controller)
        self.positions = {}
        self.inputs = {}
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.io_client = None
        try:  # Universal Robots IO (ur_robot_driver); other robots: IO is only logged
            from ur_msgs.srv import SetIO
            from ur_msgs.msg import IOStates
            self.SetIO = SetIO
            self.io_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
            self.create_subscription(IOStates, '/io_and_status_controller/io_states', self.on_io_states, 10)
        except ImportError:
            pass

    def on_joint_states(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.positions[n] = p

    def on_io_states(self, msg):
        for d in msg.digital_in_states:
            self.inputs[str(d.pin)] = 1 if d.state else 0

    def current(self, timeout=5.0):
        end = time.time() + timeout
        while time.time() < end:
            if all(n in self.positions for n in self.joint_names):
                return [self.positions[n] for n in self.joint_names]
            rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def run_trajectory(self, seg):
        scale = max(1e-3, float(self.get_parameter('speed_scale').value))
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        t = 0.0
        q0 = self.current()
        for p in seg['points']:
            dt = p['dt']
            if p.get('first') or dt <= 0:
                # first point of the program: time from the CURRENT state at the programmed joint speed
                dist = max(abs(a - b) for a, b in zip(p['positions'], q0)) if q0 else 0.0
                dt = max(float(self.get_parameter('min_start_time').value), dist / seg['vmax'])
            t += dt / scale
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in p['positions']]
            pt.time_from_start = to_duration(t)
            traj.points.append(pt)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Trajectory goal rejected')
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        code = res.result().result.error_code
        if code != 0:
            self.get_logger().error('Trajectory failed (error code %d)' % code)
            return False
        return True

    def set_io(self, name, value):
        if self.io_client is not None and name.isdigit() and self.io_client.wait_for_service(timeout_sec=2.0):
            req = self.SetIO.Request(fun=1, pin=int(name), state=float(value))
            f = self.io_client.call_async(req)
            rclpy.spin_until_future_complete(self, f)
            return
        self.get_logger().warning('Digital output %s = %d: no IO service, not set' % (name, value))

    def wait_input(self, name, value, timeout):
        end = None if timeout < 0 else time.time() + timeout
        if self.io_client is None:
            self.get_logger().warning('Wait for input %s = %d: no IO topic, skipped' % (name, value))
            return
        while self.inputs.get(name) != value:
            if end is not None and time.time() > end:
                self.get_logger().warning('Timeout waiting for input %s' % name)
                return
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        for w in WARNINGS:
            self.get_logger().warning(w)
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('FollowJointTrajectory action server not available')
            return 1
        for seg in SEGMENTS:
            kind = seg['type']
            if kind == 'traj':
                if not self.run_trajectory(seg):
                    return 2
            elif kind == 'pause':
                if seg['seconds'] < 0:
                    self.get_logger().info('Program paused (stop): continuing')
                else:
                    time.sleep(seg['seconds'])
            elif kind == 'io':
                self.set_io(seg['name'], seg['value'])
            elif kind == 'wait':
                self.wait_input(seg['name'], seg['value'], seg['timeout'])
            elif kind == 'msg':
                self.get_logger().info(seg['text'])
        self.get_logger().info('Program ${o.prog} done')
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = ProgramRunner()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    main()
`;

export const ROS2_POST: PostProcessor = {
  id: 'ROS2',
  name: 'ROS 2 (ament_python package, FollowJointTrajectory)',
  brand: 'ROS2',
  extension: 'py',
  description: 'ament_python package with an rclpy FollowJointTrajectory action client (ros2_control).',
  generate(p: PostProgram, options: Record<string, any> = {}): PostFile[] {
    const pkg = options.package ?? pkgName(p.name);
    const prog = postProgName(p.name).replace(/[^A-Za-z0-9_]/g, '_');
    const { segs, warnings, jointNames } = ros2Segments(p, options);
    const names = Array.isArray(options.jointNames) && options.jointNames.length === jointNames.length ? options.jointNames : jointNames;
    const controller = options.controller ?? 'joint_trajectory_controller';
    const exe = `${prog.toLowerCase()}_node`;
    const nPts = segs.reduce((a, s) => a + (s.type === 'traj' ? s.points.length : 0), 0);
    const files: PostFile[] = [
      { name: `${pkg}/package.xml`, content: `<?xml version="1.0"?>\n<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>\n<package format="3">\n  <name>${pkg}</name>\n  <version>0.1.0</version>\n  <description>Robot program ${p.name} generated by VerticalBot Studio (FollowJointTrajectory client)</description>\n  <maintainer email="user@example.com">VerticalBot Studio</maintainer>\n  <license>MIT</license>\n  <exec_depend>rclpy</exec_depend>\n  <exec_depend>control_msgs</exec_depend>\n  <exec_depend>trajectory_msgs</exec_depend>\n  <exec_depend>sensor_msgs</exec_depend>\n  <exec_depend>builtin_interfaces</exec_depend>\n  <test_depend>python3-pytest</test_depend>\n  <export>\n    <build_type>ament_python</build_type>\n  </export>\n</package>\n` },
      { name: `${pkg}/setup.py`, content: `from setuptools import setup\n\npackage_name = '${pkg}'\n\nsetup(\n    name=package_name,\n    version='0.1.0',\n    packages=[package_name],\n    data_files=[\n        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),\n        ('share/' + package_name, ['package.xml', 'trajectory.json']),\n        ('share/' + package_name + '/launch', ['launch/${prog}.launch.py']),\n    ],\n    install_requires=['setuptools'],\n    zip_safe=True,\n    maintainer='VerticalBot Studio',\n    maintainer_email='user@example.com',\n    description='Robot program ${p.name} (FollowJointTrajectory client)',\n    license='MIT',\n    entry_points={\n        'console_scripts': [\n            '${exe} = ${pkg}.${prog.toLowerCase()}_node:main',\n        ],\n    },\n)\n` },
      { name: `${pkg}/setup.cfg`, content: `[develop]\nscript_dir=$base/lib/${pkg}\n[install]\ninstall_scripts=$base/lib/${pkg}\n` },
      { name: `${pkg}/resource/${pkg}`, content: `ament index marker of the ${pkg} package (content is not read)` + String.fromCharCode(10) },
      { name: `${pkg}/${pkg}/__init__.py`, content: `"""Robot program ${p.name.replace(/["\\]/g, '_')}: FollowJointTrajectory client node."""` + String.fromCharCode(10) },
      { name: `${pkg}/${pkg}/${prog.toLowerCase()}_node.py`, content: NODE_PY({ pkg, prog, controller, jointNames: names, segs, warnings }) },
      { name: `${pkg}/launch/${prog}.launch.py`, content: `from launch import LaunchDescription\nfrom launch.actions import DeclareLaunchArgument\nfrom launch.substitutions import LaunchConfiguration\nfrom launch_ros.actions import Node\n\n\ndef generate_launch_description():\n    return LaunchDescription([\n        DeclareLaunchArgument('controller', default_value='${controller}'),\n        DeclareLaunchArgument('speed_scale', default_value='1.0'),\n        Node(package='${pkg}', executable='${exe}', output='screen',\n             parameters=[{'controller': LaunchConfiguration('controller'), 'speed_scale': LaunchConfiguration('speed_scale')}]),\n    ])\n` },
      { name: `${pkg}/trajectory.json`, content: JSON.stringify({ joint_names: names, controller, segments: segs, warnings }, null, 1), mime: 'application/json' },
      { name: `${pkg}/README.md`, content: `# ${pkg}\n\nRobot program **${p.name}** generated by VerticalBot Studio: ${segs.length} segments, ${nPts} trajectory points.\n\n\`\`\`bash\ncd ~/ros2_ws/src && cp -r <this folder> . && cd .. && colcon build --packages-select ${pkg}\nsource install/setup.bash\nros2 launch ${pkg} ${prog}.launch.py controller:=${controller}\n\`\`\`\n\nThe node waits for \`/${controller}/follow_joint_trajectory\` and \`/joint_states\` and starts from the current robot state.\n${warnings.length ? `\nWarnings:\n${warnings.map((w) => `- ${w}`).join('\n')}\n` : ''}` },
    ];
    // the node first: files[0] carries the declared extension
    return [files[5], ...files.filter((_, i) => i !== 5)];
  },
};
registerPost(ROS2_POST);
