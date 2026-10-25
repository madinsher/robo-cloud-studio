/**
 * R5 / contract C3: the studio as the MURKA simulator backend (src/ros/murom_sim.ts) checked topic by topic and
 * service by service through a fake rosbridge (real websocket, rosbridge v2 ops, Python json dialect) on the imported
 * MURKA retail store (retail_store.wbt + murom_world/config). Reference: robot-retailer-murka-v2 docs/interfaces.md
 * §1-2, murom_webots murom_driver.py / supervisor_plugin.py / murom_webots.urdf and murom_mujoco sim_node.py.
 * Ground truth: the MURKA config files (product slots), murom.urdf FK (l/r_tool_link, l/r_palm_link — T5 verified the
 * studio URDF FK against urdf_parser_py), analytic ray/box intersection with the hall boxes, the webots_ros2 camera
 * formula, kinematic invariants of the differential drive (wheel radius 0.177 m, track 0.441 m).
 */
import { describe, it, expect, beforeAll, afterAll } from 'vitest';
import fs from 'node:fs';
import path from 'node:path';
import { parseYaml } from '../src/io/webots';
import { AssetStore } from '../src/scene/assets';
import { Station, SceneObject, Item } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { MobileRobot } from '../src/mobile/items';
import { MuromSimInterface, MUROM, MUROM_WHEEL_JOINTS, MUROM_ARM_HEAD_JOINTS } from '../src/ros/murom_sim';
import { RosbridgeClient } from '../src/ros/rosbridge_client';
import { parseURDF, urdfLinkPoses, importURDF } from '../src/io/urdf/urdf';
import { sceneMeshes, firstHit } from '../src/vision/sensing';
import { Mat4, multiply, invert, transformPoint, transformDir, transl, DEG, RAD } from '../src/core/math/pose';
import { HAS_MURKA } from './webots_helpers';
import { buildRetailStation } from './ros_murka_helpers';
import { FakeRosbridge, until } from './ros_fake_rosbridge';
import { FIX, read, join } from './ros_gt_util';

const DT = 0.032;

const buildStation = buildRetailStation;

async function connect(station: Station, assets: AssetStore, opts: ConstructorParameters<typeof MuromSimInterface>[2] = {}) {
  const fake = await FakeRosbridge.start();
  const client = new RosbridgeClient(fake.url);
  await client.connect();
  const sim = new MuromSimInterface(station, client, { assets, lidarNoise: false, ...opts });
  sim.start(0);
  /** rosbridge -> studio message, delivered before the next frame. */
  const send = async (topic: string, msg: unknown) => { const n = client.stats.received; fake.send(topic, msg); await until(() => client.stats.received > n); };
  const run = async (frames: number) => { for (let i = 0; i < frames; i++) sim.step(DT, Math.round((sim.time + DT) * 1e6) / 1e6); await until(() => client.stats.sent === fake.ops.length, 10000); };
  return { fake, client, sim, run, send };
}
const tSec = (s: { sec: number; nanosec: number }) => s.sec + s.nanosec * 1e-9;
const b64 = (s: string) => Buffer.from(s, 'base64');
const pos = (m: Mat4) => [m[12], m[13], m[14]];

/** Independent ray / oriented-box intersection (slab method), mm. */
function rayBox(o: number[], d: number[], world: Mat4, size: number[]): number {
  const I = invert(world);
  const lo = transformPoint(I, o as [number, number, number]), ld = transformDir(I, d as [number, number, number]);
  let t0 = 0, t1 = Infinity;
  for (let k = 0; k < 3; k++) {
    const h = size[k] / 2;
    if (Math.abs(ld[k]) < 1e-12) { if (Math.abs(lo[k]) > h) return Infinity; continue; }
    let a = (-h - lo[k]) / ld[k], b = (h - lo[k]) / ld[k];
    if (a > b) [a, b] = [b, a];
    t0 = Math.max(t0, a); t1 = Math.min(t1, b);
    if (t0 > t1) return Infinity;
  }
  return t0 > 0 ? t0 : Infinity;
}
/** Every box (visual geometry) of the hall and the products, world pose + size (mm). */
function hallBoxes(station: Station, skip: Set<Item>): { world: Mat4; size: number[]; name: string }[] {
  const out: { world: Mat4; size: number[]; name: string }[] = [];
  for (const it of station.walk()) {
    if (!(it instanceof SceneObject) || skip.has(it) || it.name === 'floor') continue;
    for (const g of it.geometry) if (g.primitive?.kind === 'box') out.push({ world: multiply(it.poseAbs(), g.origin ? Float64Array.from(g.origin) as Mat4 : transl(0, 0, 0)), size: g.primitive.size.map((v, k) => v * (g.scale?.[k] ?? 1)), name: it.name });
  }
  return out;
}

describe.skipIf(!HAS_MURKA)('C3 MURKA simulator interface (fake rosbridge)', () => {
  let S: ReturnType<typeof buildRetailStation>;
  let C: Awaited<ReturnType<typeof connect>>;
  beforeAll(async () => { S = buildStation(); C = await connect(S.station, S.assets, { cameraRate: 5 }); await C.run(125); }, 120000);
  afterAll(async () => { C?.client.close(); await C?.fake.close(); });

  it('advertises every topic with the interface type, subscribes the commands, advertises the simulator services', async () => {
    const adv = C.fake.advertised();
    const types = Object.fromEntries([...adv].map(([k, v]) => [k, v.type]));
    expect(types).toEqual({
      '/clock': 'rosgraph_msgs/msg/Clock', '/odom': 'nav_msgs/msg/Odometry', '/tf': 'tf2_msgs/msg/TFMessage',
      '/murom/ground_truth_pose': 'geometry_msgs/msg/PoseStamped', '/murom/sim/tool_tips': 'geometry_msgs/msg/PoseArray',
      '/joint_states': 'sensor_msgs/msg/JointState', '/murom/sim/objects': 'murom_msgs/msg/ProductArray', '/imu': 'sensor_msgs/msg/Imu',
      '/scan': 'sensor_msgs/msg/LaserScan', '/head_camera/image_color': 'sensor_msgs/msg/Image', '/head_camera/camera_info': 'sensor_msgs/msg/CameraInfo',
      '/head_depth/image': 'sensor_msgs/msg/Image', '/head_depth/camera_info': 'sensor_msgs/msg/CameraInfo',
    });
    // advertise precedes the first publish of the topic
    for (const [topic, a] of adv) expect(C.fake.ops.findIndex((o) => o.op === 'publish' && o.topic === topic)).toBeGreaterThan(a.index);
    expect(Object.fromEntries(C.fake.subscriptions())).toEqual({ '/cmd_vel': 'geometry_msgs/msg/Twist', '/murom/joint_trajectory': 'trajectory_msgs/msg/JointTrajectory' });
    expect(Object.fromEntries(C.fake.services())).toEqual({ '/murom/sim/attach': 'murom_msgs/srv/AttachObject', '/murom/sim/detach': 'murom_msgs/srv/DetachObject', '/murom/sim/screenshot': 'std_srvs/srv/SetBool', '/murom/sim/record': 'std_srvs/srv/SetBool' });
  });

  it('rates in sim time (32 ms frame): clock/odom/tf/gt/tips/imu every frame, joint_states 20 Hz, scan 10 Hz, objects 2 Hz, cameras 5 Hz; stamps = /clock', () => {
    const n = (t: string) => C.fake.published(t).length;
    const clocks = C.fake.published('/clock').map((m) => tSec(m.clock));
    expect(clocks.length).toBe(125);
    clocks.forEach((t, i) => expect(t).toBeCloseTo((i + 1) * DT, 6));
    for (const t of ['/odom', '/tf', '/murom/ground_truth_pose', '/murom/sim/tool_tips', '/imu']) expect(n(t), t).toBe(125);
    const span = 125 * DT; // 4.0 s
    expect(Math.abs(n('/joint_states') - span * 20)).toBeLessThanOrEqual(1);
    expect(Math.abs(n('/scan') - span * 10)).toBeLessThanOrEqual(1);
    expect(Math.abs(n('/murom/sim/objects') - span * 2)).toBeLessThanOrEqual(1);
    for (const t of ['/head_camera/image_color', '/head_camera/camera_info', '/head_depth/image', '/head_depth/camera_info']) expect(Math.abs(n(t) - span * 5), t).toBeLessThanOrEqual(1);
    // every stamped message carries the /clock time of its frame
    const clockSet = new Set(clocks.map((t) => Math.round(t * 1e6)));
    for (const t of ['/odom', '/joint_states', '/scan', '/imu', '/murom/ground_truth_pose', '/murom/sim/objects', '/head_camera/image_color']) for (const m of C.fake.published(t)) expect(clockSet.has(Math.round(tSec(m.header.stamp) * 1e6)), t).toBe(true);
    // /odom and the ground truth of one frame share the stamp (localization_node: map->odom from the GT stamp)
    expect(C.fake.published('/odom').map((m) => tSec(m.header.stamp))).toEqual(C.fake.published('/murom/ground_truth_pose').map((m) => tSec(m.header.stamp)));
  });

  it('frames and units: odom->base_link (m, start pose = origin), map ground truth = robot pose, imu_link, laser_frame, camera_link_optical', () => {
    const odom = C.fake.last('/odom');
    expect([odom.header.frame_id, odom.child_frame_id]).toEqual(['odom', 'base_link']);
    expect([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.orientation.w]).toEqual([0, 0, 1]);
    const tf = C.fake.last('/tf').transforms;
    expect(tf).toHaveLength(1);
    expect([tf[0].header.frame_id, tf[0].child_frame_id]).toEqual(['odom', 'base_link']);
    const gt = C.fake.last('/murom/ground_truth_pose');
    expect(gt.header.frame_id).toBe('map');
    const B = (S.station.find('murom') as MobileRobot).poseAbs();
    expect([gt.pose.position.x, gt.pose.position.y, gt.pose.position.z]).toEqual(pos(B).map((v) => v / 1000));
    expect(gt.pose.position.x).toBeCloseTo(-7.199, 9); // dock pose of retail_store.wbt
    expect(Math.atan2(2 * gt.pose.orientation.w * gt.pose.orientation.z, 1 - 2 * gt.pose.orientation.z ** 2)).toBeCloseTo(3.1416 - 2 * Math.PI, 9); // rotation 0 0 1 3.1416 of retail_store.wbt, wrapped
    const imu = C.fake.last('/imu');
    expect(imu.header.frame_id).toBe('imu_link');
    expect(imu.linear_acceleration.z).toBeCloseTo(9.81, 9);
    expect(imu.orientation).toEqual(gt.pose.orientation);
    expect(C.fake.last('/scan').header.frame_id).toBe('laser_frame');
    for (const t of ['/head_camera/image_color', '/head_camera/camera_info', '/head_depth/image', '/head_depth/camera_info']) expect(C.fake.last(t).header.frame_id, t).toBe('camera_link_optical');
    const tips = C.fake.last('/murom/sim/tool_tips');
    expect(tips.header.frame_id).toBe('map');
    expect(tips.poses).toHaveLength(2);
  });

  it('/joint_states: 16 joints in the driver order (wheels first), rad, v2 start pose, wheel velocity = commanded wheel speed', () => {
    const js = C.fake.last('/joint_states');
    expect(js.name).toEqual([...MUROM_WHEEL_JOINTS, ...MUROM_ARM_HEAD_JOINTS]);
    expect(js.position).toHaveLength(16); expect(js.velocity).toHaveLength(16);
    const at = (n: string) => js.position[js.name.indexOf(n)];
    expect(at('l_shoulder_pitch_joint')).toBeCloseTo(0.3, 9);
    expect(at('r_elbow_joint')).toBeCloseTo(-2.2, 9);
    expect(at('head_pan_joint')).toBeCloseTo(0, 9);
    const left = (S.station.find('murom_left_arm') as Robot);
    expect(at('l_elbow_joint')).toBeCloseTo(left.joints()[left.jointNames().indexOf('l_elbow_joint')] * DEG, 12);
    expect(js.velocity.slice(0, 2)).toEqual([0, 0]);
  });

  it('/scan: 640 beams over 240 deg left to right (webots_ros2 Ros2Lidar), inf outside [0.15, 8] m, ranges = analytic ray/box hits on the hall', () => {
    const s = C.fake.last('/scan');
    expect(s.ranges).toHaveLength(640);
    expect(s.angle_min).toBeCloseTo(4.189 / 2, 12);
    expect(s.angle_max).toBeCloseTo(-4.189 / 2, 12);
    expect(s.angle_increment).toBeCloseTo(-4.189 / 639, 12);
    expect([s.range_min, s.range_max, s.scan_time]).toEqual([0.15, 8, 0.1]);
    expect(s.time_increment).toBeCloseTo(0.1 / 640, 12);
    // the wire text carries bare Infinity tokens (rosbridge's Python json reads them; JSON null would be rejected)
    expect(C.fake.raws('/scan')[0]).toMatch(/,Infinity[,\]]/);
    const finite = s.ranges.filter((r: number) => Number.isFinite(r));
    expect(finite.length).toBeGreaterThan(50);
    // analytic ground truth: the laser frame from murom.urdf (laser_joint xyz 0.30 0 0.66 on base_link)
    const base = (S.station.find('murom') as MobileRobot).poseAbs();
    const o = transformPoint(base, [300, 0, 660]);
    const own = new Set<Item>([...(S.station.find('murom') as Item).walk()]);
    const boxes = hallBoxes(S.station, own);
    let compared = 0;
    s.ranges.forEach((r: number, i: number) => {
      const a = 4.189 / 2 - (i * 4.189) / 639;
      const d = transformDir(base, [Math.cos(a), Math.sin(a), 0]);
      const t = Math.min(...boxes.map((b) => rayBox(o, d, b.world, b.size))) / 1000;
      const expected = t >= 0.15 && t <= 8 ? t : Infinity;
      if (!Number.isFinite(expected)) expect(r, `beam ${i}`).toBe(Infinity);
      else { expect(Math.abs(r - expected), `beam ${i}`).toBeLessThan(0.001); compared++; }
    });
    expect(compared).toBe(finite.length);
  });

  it('cameras: bgra8 320x240 and 32FC1 depth (m, inf outside [0.1, 5] m); K of the unified camera model (fx = W / 2 tan(fov/2))', () => {
    const img = C.fake.last('/head_camera/image_color');
    expect([img.width, img.height, img.encoding, img.is_bigendian, img.step]).toEqual([320, 240, 'bgra8', 0, 1280]);
    const px = b64(img.data);
    expect(px.length).toBe(320 * 240 * 4);
    for (let i = 3; i < px.length; i += 4) if (px[i] !== 255) throw new Error('alpha must be 255');
    const info = C.fake.last('/head_camera/camera_info');
    const f = 320 / (2 * Math.tan(1.2 / 2));
    expect(f).toBeCloseTo(233.87, 2); // webots_ros2 Ros2Camera / murom_mujoco
    expect(info.k[0]).toBeCloseTo(f, 9); expect(info.k[4]).toBeCloseTo(f, 9);
    expect([info.k[2], info.k[5]]).toEqual([159.5, 119.5]); // ROS integer pixel centres (webots_ros2: W/2, H/2 — half a pixel)
    expect([info.width, info.height, info.distortion_model]).toEqual([320, 240, 'plumb_bob']);
    expect(info.p.slice(0, 4)).toEqual([info.k[0], 0, info.k[2], 0]);
    const dep = C.fake.last('/head_depth/image');
    expect([dep.width, dep.height, dep.encoding, dep.step]).toEqual([320, 240, '32FC1', 1280]);
    const buf = b64(dep.data);
    const z = new Float32Array(buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength));
    expect(z.length).toBe(320 * 240);
    for (const v of z) if (!(v === Infinity || (v >= 0.1 && v <= 5))) throw new Error(`depth ${v} outside [0.1, 5] and not inf`);
    // centre pixel depth = ray cast along the optical axis through that pixel centre (depth along +Z)
    const cam = S.station.find('head_depth') as unknown as { poseAbs(): Mat4 };
    const P = cam.poseAbs();
    const u = 160, v = 120, dl: [number, number, number] = [(u + 0.5 - 160) / f, (v + 0.5 - 120) / f, 1];
    const hit = firstHit(sceneMeshes(S.station, { assets: S.assets, ground: true }), [P[12], P[13], P[14]], transformDir(P, dl), 20000)!;
    expect(Math.abs(z[v * 320 + u] - hit.t / 1000)).toBeLessThan(1e-3);
  });

  it('/murom/sim/objects: the 50 products of products.yaml at their slots (slot pose + size/2 + 2 mm, map frame)', () => {
    const msg = C.fake.last('/murom/sim/objects');
    expect(msg.header.frame_id).toBe('map');
    const prod = parseYaml(S.productsYaml) as { catalog: Record<string, { size: number[] }>; instances: { name: string; sku: string; slot: string }[] };
    const slots = (parseYaml(S.locationsYaml) as { slots: Record<string, { pose: { x: number; y: number; z: number } }> }).slots;
    expect(msg.products).toHaveLength(prod.instances.length);
    expect(prod.instances.length).toBe(50);
    for (const inst of prod.instances) {
      const p = msg.products.find((x: any) => x.name === inst.name);
      expect(p, inst.name).toBeTruthy();
      expect([p.sku, p.location_id]).toEqual([inst.sku, inst.slot]);
      const s = slots[inst.slot].pose, h = prod.catalog[inst.sku].size[2];
      expect(Math.abs(p.pose.position.x - s.x)).toBeLessThan(1e-6);
      expect(Math.abs(p.pose.position.y - s.y)).toBeLessThan(1e-6);
      expect(Math.abs(p.pose.position.z - (s.z + h / 2 + 0.002))).toBeLessThan(1e-6);
    }
  });
});

describe.skipIf(!HAS_MURKA)('C3 MURKA kinematics: tool tips and palms vs murom.urdf', () => {
  it('tool tips (forearm + 0.248 m) and palm frames equal murom.urdf FK of l/r_tool_link, l/r_palm_link (< 1 mm) for random arm poses', async () => {
    const S = buildStation();
    const C = await connect(S.station, S.assets, { cameraRate: 0 });
    try {
      const urdf = parseURDF(read(join(FIX, 'xacro', 'murom.urdf')));
      const base = (S.station.find('murom') as MobileRobot).poseAbs();
      let rnd = 3;
      const rand = () => { rnd = (rnd * 16807) % 2147483647; return rnd / 2147483647; };
      for (let k = 0; k < 6; k++) {
        const q: Record<string, number> = {};
        for (const s of C.sim.servos) { const v = (rand() - 0.5) * 2 * Math.min(1.4, s.upper); q[s.name] = v; const arr = s.robot.joints(); arr[s.indep] = v * RAD; s.robot.setJoints(arr); }
        const gt = urdfLinkPoses(urdf, Object.fromEntries(Object.entries(q).map(([n, v]) => [n, v * RAD])), base);
        const tips = [C.sim.toolTip('left'), C.sim.toolTip('right')];
        const want = [gt.get('l_tool_link')!, gt.get('r_tool_link')!].map((m) => pos(m).map((v) => v / 1000));
        for (let a = 0; a < 2; a++) for (let i = 0; i < 3; i++) expect(Math.abs(tips[a][i] - want[a][i])).toBeLessThan(1e-3);
        for (const [link, side] of [['l_palm_link', 'l_palm'], ['r_palm_link', 'r_palm']] as const) {
          const P = C.sim.linkPose(side)!, G = gt.get(link)!;
          for (let i = 0; i < 3; i++) expect(Math.abs(P[12 + i] - G[12 + i] / 1000)).toBeLessThan(1e-3);
          for (const i of [0, 1, 2, 4, 5, 6, 8, 9, 10]) expect(Math.abs(P[i] - G[i])).toBeLessThan(1e-6);
        }
      }
    } finally { C.client.close(); await C.fake.close(); }
  }, 120000);
});

describe.skipIf(!HAS_MURKA)('C3 MURKA commands and services', () => {
  let S: { station: Station; assets: AssetStore };
  let C: Awaited<ReturnType<typeof connect>>;
  beforeAll(async () => { S = buildStation(); C = await connect(S.station, S.assets, { cameraRate: 0 }); await C.run(2); }, 120000);
  afterAll(async () => { C?.client.close(); await C?.fake.close(); });

  it('/cmd_vel drives the differential base exactly (v, w), wheel speeds (v -+ w b/2)/r, stops 0.6 s after the last command', async () => {
    const m = S.station.find('murom') as MobileRobot;
    const x0 = m.state.x, y0 = m.state.y, th0 = m.state.theta * DEG;
    await C.send('/cmd_vel', { linear: { x: -0.25, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0.4 } });
    await C.run(25); // 0.8 s: 0.6 s of motion (watchdog), then stopped
    const js = C.fake.published('/joint_states');
    const moving = js.find((x) => x.velocity[0] !== 0)!;
    expect(moving.velocity[0]).toBeCloseTo((-0.25 - 0.4 * 0.441 / 2) / 0.177, 9);
    expect(moving.velocity[1]).toBeCloseTo((-0.25 + 0.4 * 0.441 / 2) / 0.177, 9);
    expect(js[js.length - 1].velocity.slice(0, 2)).toEqual([0, 0]);
    // the command is live for the frames with t - t_cmd <= 0.6 s (driver watchdog): 18 frames of 32 ms
    const T = 18 * DT, R = -0.25 / 0.4;
    const th1 = th0 + 0.4 * T;
    const ex = x0 / 1000 + R * (Math.sin(th1) - Math.sin(th0)), ey = y0 / 1000 - R * (Math.cos(th1) - Math.cos(th0));
    expect(Math.abs(m.state.x / 1000 - ex)).toBeLessThan(1e-6);
    expect(Math.abs(m.state.y / 1000 - ey)).toBeLessThan(1e-6);
    // odometry in the start frame agrees with the ground truth displacement
    const gt = C.fake.last('/murom/ground_truth_pose').pose.position, odom = C.fake.last('/odom').pose.pose.position;
    const dx = gt.x - x0 / 1000, dy = gt.y - y0 / 1000;
    expect(Math.abs(odom.x - (Math.cos(th0) * dx + Math.sin(th0) * dy))).toBeLessThan(1e-9);
    expect(Math.abs(odom.y - (-Math.sin(th0) * dx + Math.cos(th0) * dy))).toBeLessThan(1e-9);
    // wheel angles integrate the wheel speeds
    const last = js[js.length - 1];
    expect(last.position[0]).toBeCloseTo(moving.velocity[0] * T, 9);
  });

  it('wheel speed saturation at 18 rad/s keeps the commanded curvature', async () => {
    await C.send('/cmd_vel', { linear: { x: 5, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 2 } });
    await C.run(2);
    const js = C.fake.last('/joint_states') ?? C.fake.published('/joint_states').pop();
    const [wl, wr] = js.velocity;
    expect(Math.max(Math.abs(wl), Math.abs(wr))).toBeCloseTo(18, 9);
    const v = ((wl + wr) / 2) * 0.177, w = ((wr - wl) * 0.177) / 0.441;
    expect(w / v).toBeCloseTo(2 / 5, 9);
    await C.run(25);
  });

  it('/murom/joint_trajectory: linear interpolation from the current position, Webots servo (P = 10, maxVelocity), replace, unknown joints ignored, wrist clamp', async () => {
    const head = S.station.find('murom_head') as Robot;
    const pan = () => head.joints()[0] * DEG;
    await C.send('/murom/joint_trajectory', { joint_names: ['head_pan_joint'], points: [{ positions: [0.5], time_from_start: { sec: 1, nanosec: 0 } }] });
    const servo = C.sim.servos.find((s) => s.name === 'head_pan_joint')!;
    const t0 = C.sim.time;
    // target follows the linear interpolation; the joint follows the target with q += clip(P e, +-vmax) dt
    let q = pan();
    for (let i = 0; i < 40; i++) {
      await C.run(1);
      const t = C.sim.time - t0;
      const target = Math.min(1, t) * 0.5;
      expect(servo.target).toBeCloseTo(target, 9);
      const step = Math.max(-servo.vmax, Math.min(servo.vmax, 10 * (target - q))) * DT;
      q = Math.abs(target - q) < Math.abs(step) ? target : q + step;
      expect(pan()).toBeCloseTo(q, 9);
    }
    expect(pan()).toBeCloseTo(0.5, 2);
    // unknown joint: the whole message is ignored with the driver's warning
    await C.send('/murom/joint_trajectory', { joint_names: ['head_pan_joint', 'no_such_joint'], points: [{ positions: [0, 0], time_from_start: { sec: 1, nanosec: 0 } }] });
    await C.run(40);
    expect(pan()).toBeCloseTo(0.5, 3);
    expect(C.sim.warnings.some((w) => /Неизвестные суставы.*no_such_joint/.test(w))).toBe(true);
    // wrist targets beyond the motor limits are clipped (+-2.6 rad roll)
    await C.send('/murom/joint_trajectory', { joint_names: ['l_wrist_roll_joint'], points: [{ positions: [3.5], time_from_start: { sec: 0, nanosec: 100000000 } }] });
    await C.run(80);
    const left = S.station.find('murom_left_arm') as Robot;
    expect(left.joints()[left.jointNames().indexOf('l_wrist_roll_joint')] * DEG).toBeCloseTo(2.6, 6);
  });

  it('services over rosbridge: attach out of reach refused; palm on the top face -> attached, follows the palm; detach with place_pose keeps the yaw upright; basket items ride with the robot', async () => {
    const name = 'tea_box_1';
    const far = await C.fake.call('/murom/sim/attach', { object_name: name, arm: 'left' });
    expect(far.result).toBe(true);
    expect(far.values.success).toBe(false);
    expect(far.values.message).toMatch(/^Ладонь слишком далеко от tea_box_1: \d+\.\d{3} м/);
    expect((await C.fake.call('/murom/sim/attach', { object_name: name, arm: 'middle' })).values).toEqual({ success: false, message: "Неизвестная рука: 'middle'" });
    expect((await C.fake.call('/murom/sim/attach', { object_name: 'ghost', arm: 'left' })).values.success).toBe(false);
    // put the product under the left palm surface (top face 1 cm below the grip point)
    const item = S.station.find(name) as SceneObject;
    const g = C.sim.gripPoint('left').p;
    const h = (item.params.size as number[])[2];
    const T0 = Float64Array.from(item.poseAbs()) as Mat4;
    T0[12] = g[0] * 1000; T0[13] = g[1] * 1000; T0[14] = g[2] * 1000 - h / 2 - 10;
    item.setPoseAbs(T0);
    const ok = await C.fake.call('/murom/sim/attach', { object_name: name, arm: 'left' });
    expect(ok.values).toEqual({ success: true, message: 'tea_box_1 захвачен рукой left (dist 0.010)' });
    expect((await C.fake.call('/murom/sim/attach', { object_name: 'tea_box_2', arm: 'left' })).values.message).toBe('Рука left уже держит tea_box_1');
    // the product keeps its pose relative to the palm while the arm moves
    const palm0 = C.sim.linkPose('l_palm')!;
    const rel0 = multiply(invert(palm0), C.sim.productPose(C.sim.products.get(name)!));
    await C.send('/murom/joint_trajectory', { joint_names: ['l_shoulder_pitch_joint', 'l_elbow_joint'], points: [{ positions: [0.6, -1.6], time_from_start: { sec: 1, nanosec: 0 } }] });
    await C.run(60);
    const palm1 = C.sim.linkPose('l_palm')!;
    expect(Math.hypot(palm1[12] - palm0[12], palm1[13] - palm0[13], palm1[14] - palm0[14])).toBeGreaterThan(0.05);
    const rel1 = multiply(invert(palm1), C.sim.productPose(C.sim.products.get(name)!));
    for (let i = 0; i < 16; i++) expect(Math.abs(rel1[i] - rel0[i])).toBeLessThan(1e-9);
    await C.run(16);
    let objs = C.fake.last('/murom/sim/objects').products;
    expect(objs.find((p: any) => p.name === name).location_id).toBe('carried_left');
    // detach into the basket: placed upright at place_pose with the yaw it had in the hand, then rides with the robot
    const yawHand = Math.atan2(C.sim.productPose(C.sim.products.get(name)!)[1], C.sim.productPose(C.sim.products.get(name)!)[0]);
    const base = C.sim.basePose();
    const place = transformPoint(base, [-0.3, 0.1, 0.95]);
    const det = await C.fake.call('/murom/sim/detach', { object_name: name, use_place_pose: true, place_pose: { position: { x: place[0], y: place[1], z: place[2] }, orientation: { x: 0, y: 0, z: 0, w: 1 } }, location_id: 'basket_0' });
    expect(det.values.success).toBe(true);
    expect(det.values.message).toMatch(/^tea_box_1 отпущен \(basket_0, скачок \d+\.\d{3} м \d+\.\d°\)$/);
    const P = C.sim.productPose(C.sim.products.get(name)!);
    expect(pos(P).map((v, i) => Math.abs(v - place[i])).every((e) => e < 1e-9)).toBe(true);
    expect(P[10]).toBeCloseTo(1, 12); // upright
    expect(Math.atan2(P[1], P[0])).toBeCloseTo(yawHand, 9);
    expect((await C.fake.call('/murom/sim/detach', { object_name: name, use_place_pose: false, location_id: '' })).values).toEqual({ success: false, message: 'tea_box_1 не захвачен' });
    await C.send('/cmd_vel', { linear: { x: 0.3, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0.3 } });
    await C.run(20);
    const inRobot = (M: Mat4) => transformPoint(invert(C.sim.basePose()), [M[12], M[13], M[14]]);
    const r = inRobot(C.sim.productPose(C.sim.products.get(name)!));
    expect(r.map((v, i) => Math.abs(v - [-0.3, 0.1, 0.95][i])).every((e) => e < 1e-9)).toBe(true);
    await C.run(30);
    objs = C.fake.last('/murom/sim/objects').products;
    expect(objs.find((p: any) => p.name === name).location_id).toBe('basket_0');
    // two hands: refused while a hand... both hands free now; palms far from the box -> refused with the measured values
    const both = await C.fake.call('/murom/sim/attach', { object_name: 'box_blue_1', arm: 'both' });
    expect(both.values.success).toBe(false);
    expect(both.values.message).toMatch(/^Ладони далеко от box_blue_1: середина-центр \d+\.\d{3} м, между ладонями \d+\.\d{3} при ширине 0\.\d{3}$/);
    expect((await C.fake.call('/murom/sim/record', { data: true })).values.success).toBe(false);
  });
});

// C1 (R3) builds the reference retail station: the MURKA interface must run on it unchanged
const C1_PATHS = ['../src/murka/retail/index.ts', '../src/murka/retail/station.ts'].filter((p) => fs.existsSync(path.resolve(__dirname, p)));
describe.skipIf(!HAS_MURKA || !C1_PATHS.length)('C3 on the C1 station (buildMurkaRetailStation)', () => {
  it('finds the decomposed MUROM, serves the 50 products and the 16 joints, drives by /cmd_vel', async () => {
    const mod = await import(/* @vite-ignore */ C1_PATHS[0]) as { buildMurkaRetailStation?: (o?: { assets?: AssetStore }) => Promise<{ station: Station }> };
    expect(typeof mod.buildMurkaRetailStation).toBe('function');
    const assets = new AssetStore();
    const { station } = await mod.buildMurkaRetailStation!({ assets });
    const C = await connect(station, assets, { cameraRate: 0 });
    try {
      await C.run(16);
      expect(C.fake.last('/murom/sim/objects').products).toHaveLength(50);
      expect(C.fake.last('/joint_states').name).toEqual([...MUROM_WHEEL_JOINTS, ...MUROM_ARM_HEAD_JOINTS]);
      const x0 = C.sim.base.state.x;
      await C.send('/cmd_vel', { linear: { x: -0.25, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
      await C.run(10);
      // yaw pi at the dock: reversing moves the robot towards +x, 10 frames x 32 ms x 0.25 m/s
      expect((C.sim.base.state.x - x0) / 1000).toBeCloseTo(10 * DT * 0.25, 6);
    } finally { C.client.close(); await C.fake.close(); }
  }, 180000);
});

// C3 on a URDF-decomposed MUROM (murom.urdf of murom_description, repository fixture: runs without the MURKA folder)
describe('C3 on the URDF import of murom.urdf', () => {
  it('detects the robot (wrists as separate chains), serves 16 joints in the driver order, tool tips = murom.urdf FK', async () => {
    const text = read(join(FIX, 'xacro', 'murom.urdf'));
    const station = new Station('murom_urdf');
    station.addChild(importURDF(text).root);
    const C = await connect(station, new AssetStore(), { cameraRate: 0 });
    try {
      await C.run(4);
      const js = C.fake.last('/joint_states');
      expect(js.name).toEqual([...MUROM_WHEEL_JOINTS, ...MUROM_ARM_HEAD_JOINTS]);
      expect(js.position[js.name.indexOf('l_elbow_joint')]).toBeCloseTo(-2.2, 9); // v2 START_POSE
      const base = C.sim.base.poseAbs();
      const q = Object.fromEntries(js.name.map((n: string, i: number) => [n, js.position[i] * RAD]));
      const gt = urdfLinkPoses(parseURDF(text), q, base);
      const tips = C.fake.last('/murom/sim/tool_tips').poses;
      ['l_tool_link', 'r_tool_link'].forEach((l, k) => { const P = gt.get(l)!; expect(Math.hypot(tips[k].position.x - P[12] / 1000, tips[k].position.y - P[13] / 1000, tips[k].position.z - P[14] / 1000)).toBeLessThan(1e-6); });
      expect(C.fake.last('/murom/sim/objects').products).toEqual([]);
    } finally { C.client.close(); await C.fake.close(); }
  });
});
