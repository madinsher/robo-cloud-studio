/**
 * T5 P0 e): the studio as a ROS 2 simulator backend (src/ros/twin.ts, rosbridge 'sim' mode). Checked against the
 * MURKA interface (robot-retailer-murka-v2/docs/interfaces.md §1-2, the topic set webots_ros2 drivers expose) with the
 * ROS-expanded murom.urdf: /joint_states carries every movable joint by its URDF name, /robot_description is the kept
 * URDF, /cmd_vel drives the base with the 0.6 s watchdog, /odom + TF odom->base_link follow the motion, wheel angles
 * integrate the wheel speeds, /murom/joint_trajectory interpolates linearly and clamps to the limits, LaserScan
 * angles are sensor-relative, /clock is sim time.
 */
import { describe, it, expect } from 'vitest';
import { importURDF, parseURDF } from '../src/io/urdf/urdf';
import { RosTwin, RosIO, twinDescription } from '../src/ros/twin';
import { laserScanMsg } from '../src/ros/publishers';
import { Station } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { MobileRobot, MapItem } from '../src/mobile/items';
import { createRobotFromLibrary } from '../src/core/items/library';
import { simulateLidar2D } from '../src/mobile/navstack';
import { DEG } from '../src/core/math/pose';
import { FIX, read, join } from './ros_gt_util';

function fakeIO() {
  const sent: { topic: string; type: string; msg: any; latch: boolean }[] = [];
  const subs = new Map<string, { type: string; cb: (m: any) => void }>();
  const io: RosIO = { publish: (topic, type, msg, o) => sent.push({ topic, type, msg, latch: !!o?.latch }), subscribe: (topic, type, cb) => subs.set(topic, { type, cb }) };
  const last = (topic: string) => [...sent].reverse().find((s) => s.topic === topic)?.msg;
  return { io, sent, subs, last };
}

describe('ROS 2 digital twin (webots_ros2 / MURKA topic set)', () => {
  const text = read(join(FIX, 'xacro', 'murom.urdf'));
  const gt = parseURDF(text);
  const movable = gt.joints.filter((j) => ['revolute', 'continuous', 'prismatic'].includes(j.type)).map((j) => j.name);
  const setup = () => {
    const st = new Station('shop');
    const res = importURDF(text);
    st.addChild(res.root);
    const f = fakeIO();
    const twin = new RosTwin(st, f.io);
    twin.start();
    return { st, res, twin, ...f };
  };

  it('publishes the kept description (latched), every movable joint by URDF name and sim time', () => {
    const { res, twin, sent, subs, last } = setup();
    const rd = sent.find((s) => s.topic === '/robot_description')!;
    expect(rd.latch).toBe(true);
    expect([...parseURDF(rd.msg.data).links.keys()].sort()).toEqual([...gt.links.keys()].sort());
    expect([...subs.keys()].sort()).toEqual(['/cmd_vel', '/joint_trajectory_controller/joint_trajectory', '/murom/joint_trajectory']);
    expect(subs.get('/cmd_vel')!.type).toBe('geometry_msgs/msg/Twist');
    const arm = res.robots.find((r) => r.jointNames().includes('l_elbow_joint'))!;
    const q = arm.joints(); q[arm.jointNames().indexOf('l_elbow_joint')] = 30; arm.setJoints(q);
    for (let i = 0; i < 3; i++) twin.step(0.05);
    const js = last('/joint_states');
    expect(movable.length).toBe(16);
    expect([...js.name].sort()).toEqual([...movable].sort());
    expect(js.position[js.name.indexOf('l_elbow_joint')]).toBeCloseTo(30 * DEG, 9);
    expect(last('/clock').clock).toEqual({ sec: 0, nanosec: 150000000 });
    // every published topic carries the ROS type of the interface table
    const types = Object.fromEntries(sent.map((s) => [s.topic, s.type]));
    expect(types).toMatchObject({ '/joint_states': 'sensor_msgs/msg/JointState', '/clock': 'rosgraph_msgs/msg/Clock', '/tf': 'tf2_msgs/msg/TFMessage', '/odom': 'nav_msgs/msg/Odometry', '/imu': 'sensor_msgs/msg/Imu', '/murom/ground_truth_pose': 'geometry_msgs/msg/PoseStamped' });
  });

  it('cmd_vel drives the base (watchdog 0.6 s); /odom, TF odom->base_link and wheel angles follow', () => {
    const { res, twin, subs, last } = setup();
    const m = res.mobile as MobileRobot;
    expect(m).toBeDefined();
    const x0 = m.state.x;
    for (let i = 0; i < 40; i++) { if (i % 5 === 0) subs.get('/cmd_vel')!.cb({ linear: { x: 0.4 }, angular: { z: 0 } }); twin.step(0.05); } // 2 s, cmd every 0.25 s
    const moved = m.state.x - x0;
    expect(moved).toBeGreaterThan(400); // accelerating to 0.4 m/s
    const odom = last('/odom');
    expect(odom.header.frame_id).toBe('odom');
    expect(odom.child_frame_id).toBe(gt.rootLink); // MURKA: odom -> base_link
    expect(gt.rootLink).toBe('base_link');
    expect(odom.pose.pose.position.x).toBeCloseTo(moved / 1000, 2);
    expect(odom.twist.twist.linear.x).toBeCloseTo(m.state.v / 1000, 6);
    const tf = last('/tf').transforms[0];
    expect([tf.header.frame_id, tf.child_frame_id]).toEqual(['odom', 'base_link']);
    const js = last('/joint_states');
    const wheel = js.position[js.name.indexOf('left_wheel_joint')];
    expect(wheel * m.kin.wheelRadius).toBeCloseTo(moved, -1); // rolling without slip (within 10 mm)
    expect(js.velocity[js.name.indexOf('left_wheel_joint')]).toBeGreaterThan(0);
    // no command for > 0.6 s: the watchdog stops the base
    for (let i = 0; i < 40; i++) twin.step(0.05);
    expect(Math.abs(m.state.v)).toBeLessThan(1);
    const stopped = m.state.x;
    for (let i = 0; i < 10; i++) twin.step(0.05);
    expect(m.state.x).toBeCloseTo(stopped, 6);
  });

  it('/murom/joint_trajectory: linear interpolation, replaces per joint, clamps to the URDF limits', () => {
    const { res, twin, subs, last } = setup();
    const traj = subs.get('/murom/joint_trajectory')!.cb;
    traj({ joint_names: ['head_pan_joint', 'not_a_joint'], points: [{ positions: [0.5, 1], time_from_start: { sec: 1, nanosec: 0 } }] });
    for (let i = 0; i < 10; i++) twin.step(0.05);
    let js = last('/joint_states');
    expect(js.position[js.name.indexOf('head_pan_joint')]).toBeCloseTo(0.25, 6);
    for (let i = 0; i < 12; i++) twin.step(0.05);
    js = last('/joint_states');
    expect(js.position[js.name.indexOf('head_pan_joint')]).toBeCloseTo(0.5, 6);
    traj({ joint_names: ['head_pan_joint'], points: [{ positions: [3.0], time_from_start: { sec: 0, nanosec: 500000000 } }] }); // beyond +1.6 rad
    for (let i = 0; i < 12; i++) twin.step(0.05);
    js = last('/joint_states');
    expect(js.position[js.name.indexOf('head_pan_joint')]).toBeCloseTo(1.6, 6);
    const head = res.robots.find((r) => r.jointNames().includes('head_pan_joint'))!;
    expect(head.joints()[head.jointNames().indexOf('head_pan_joint')]).toBeCloseTo(1.6 / DEG, 6);
  });

  it('native station: description has a floating mobile base and every /joint_states name is a URDF joint', () => {
    const st = new Station('cell');
    const arm = st.addChild(createRobotFromLibrary('UR5e', 'Arm')) as Robot;
    const amr = st.addChild(new MobileRobot('AMR')) as MobileRobot;
    amr.setPose2D(2000, 1000, 90);
    const carried = amr.addChild(createRobotFromLibrary('UR5e', 'Mobile arm')) as Robot;
    const d = twinDescription(st);
    const m = parseURDF(d.xml);
    const base = d.baseLinks.get(amr)!;
    expect(m.joints.find((j) => j.child === base)!.type).toBe('floating');
    // the carried arm hangs from the mobile base link, the fixed arm from world
    const names = [...d.jointNames.get(carried)!, ...d.jointNames.get(arm)!];
    for (const n of names) expect(m.joints.some((j) => j.name === n), n).toBe(true);
    const f = fakeIO();
    const twin = new RosTwin(st, f.io);
    twin.start();
    twin.step(0.05);
    const js = f.last('/joint_states');
    expect(js.name.length).toBe(12);
    for (const n of js.name) expect(m.joints.some((j) => j.name === n), n).toBe(true);
    expect(f.last('/tf').transforms[0].child_frame_id).toBe(base);
  });

  it('LaserScan angles are relative to the sensor, not the world', () => {
    const map = new MapItem('m');
    map.resize(100, 100, 100, -5000, -5000);
    const a = laserScanMsg(simulateLidar2D(map, 0, 0, 0, { beams: 91, fov: 270 }), 'laser', 0, 0);
    const b = laserScanMsg(simulateLidar2D(map, 0, 0, 90, { beams: 91, fov: 270 }), 'laser', 0, 90 * DEG);
    expect(a.angle_min).toBeCloseTo(-135 * DEG, 9);
    expect(b.angle_min).toBeCloseTo(-135 * DEG, 9);
    expect(b.angle_max).toBeCloseTo(135 * DEG, 9);
  });
});
