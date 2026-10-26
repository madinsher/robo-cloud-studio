/**
 * R5 ROS / vision leftovers (round 2):
 *  - estimated <inertial> of studio-native links (src/io/urdf/inertia.ts): exact solid formulas as ground truth
 *    (box, cylinder), physical invariants of the exported robot (positive definite, triangle inequality, total mass);
 *  - MoveIt 2 config skeleton (src/io/urdf/moveit_export.ts): files MoveItConfigsBuilder reads, typed YAML, the group
 *    joints everywhere; opt-in WSL runtime (ROS_WSL=1): colcon build + headless bring-up with mock hardware, a plan
 *    from /plan_kinematic_path and its execution through follow_joint_trajectory;
 *  - generic twin (non-MURKA stations) publishing camera images: rgb8 / 32FC1 / CameraInfo / camera TF;
 *  - URDF gazebo cameras: fovAxis 'horizontal' and the ROS K;
 *  - SRDF disabled pairs reach the collision checker under the collider part names;
 *  - detectFruit no longer reports harvested fruit.
 */
import { describe, it, expect } from 'vitest';
import fs from 'node:fs';
import path from 'node:path';
import os from 'node:os';
import { spawnSync } from 'node:child_process';
import { meshMoments, massProps, linkMassProps, visualMoments } from '../src/io/urdf/inertia';
import { exportRobotURDF } from '../src/io/urdf/urdf_export';
import { exportMoveItConfig } from '../src/io/urdf/moveit_export';
import { parseURDF, importURDF } from '../src/io/urdf/urdf';
import { createRobotFromLibrary } from '../src/core/items/library';
import { Station, Camera, SceneObject } from '../src/core/items/item';
import { RosTwin } from '../src/ros/twin';
import { parseYaml as parseRosYaml } from '../src/ros/yaml';
import { checkRobotCollisions } from '../src/core/collision/collision';
import { importRosPackageFiles } from '../src/io/urdf/package';
import { AssetStore } from '../src/scene/assets';
import { FieldItem, cropParams } from '../src/agri/items';
import { generateOrchard, rectPolygon } from '../src/agri/orchard';
import { detectFruit } from '../src/agri/vision';
import { cameraIntrinsics } from '../src/vision/camera_model';
import { mul, transl, rotz, roty, rotx, DEG } from '../src/core/math/pose';

/** Closed box soup (outward normals), metres. */
function boxSoup(a: number, b: number, c: number, o: number[] = [0, 0, 0]): number[] {
  const v = (x: number, y: number, z: number) => [o[0] + x * a / 2, o[1] + y * b / 2, o[2] + z * c / 2];
  const quad = (p: number[][]) => [...p[0], ...p[1], ...p[2], ...p[0], ...p[2], ...p[3]];
  return [
    ...quad([v(-1, -1, -1), v(-1, 1, -1), v(1, 1, -1), v(1, -1, -1)]), ...quad([v(-1, -1, 1), v(1, -1, 1), v(1, 1, 1), v(-1, 1, 1)]),
    ...quad([v(-1, -1, -1), v(1, -1, -1), v(1, -1, 1), v(-1, -1, 1)]), ...quad([v(-1, 1, -1), v(-1, 1, 1), v(1, 1, 1), v(1, 1, -1)]),
    ...quad([v(-1, -1, -1), v(-1, -1, 1), v(-1, 1, 1), v(-1, 1, -1)]), ...quad([v(1, -1, -1), v(1, 1, -1), v(1, 1, 1), v(1, -1, 1)]),
  ];
}
/** Closed cylinder soup along z, n segments, metres. */
function cylSoup(r: number, h: number, n: number): number[] {
  const out: number[] = [];
  for (let i = 0; i < n; i++) {
    const a0 = (2 * Math.PI * i) / n, a1 = (2 * Math.PI * (i + 1)) / n;
    const p0 = [r * Math.cos(a0), r * Math.sin(a0)], p1 = [r * Math.cos(a1), r * Math.sin(a1)];
    out.push(0, 0, h / 2, p0[0], p0[1], h / 2, p1[0], p1[1], h / 2); // top (ccw from +z)
    out.push(0, 0, -h / 2, p1[0], p1[1], -h / 2, p0[0], p0[1], -h / 2); // bottom
    out.push(p0[0], p0[1], -h / 2, p1[0], p1[1], -h / 2, p1[0], p1[1], h / 2, p0[0], p0[1], -h / 2, p1[0], p1[1], h / 2, p0[0], p0[1], h / 2);
  }
  return out;
}

describe('estimated inertials', () => {
  it('closed meshes integrate exactly: box mass / centre / inertia = m/12 (b²+c², a²+c², a²+b²); cylinder converges to the solid formulas', () => {
    const p = massProps(meshMoments(boxSoup(0.3, 0.2, 0.1, [0.5, -0.2, 1])), 1000)!;
    expect(p.mass).toBeCloseTo(6, 12);
    p.com.forEach((v, i) => expect(v).toBeCloseTo([0.5, -0.2, 1][i], 12));
    const m = 6;
    expect(p.I[0]).toBeCloseTo((m / 12) * (0.04 + 0.01), 12);
    expect(p.I[3]).toBeCloseTo((m / 12) * (0.09 + 0.01), 12);
    expect(p.I[5]).toBeCloseTo((m / 12) * (0.09 + 0.04), 12);
    [p.I[1], p.I[2], p.I[4]].forEach((v) => expect(Math.abs(v)).toBeLessThan(1e-12));
    // the same box as a primitive visual rotated 90 deg about z at 1 m: the x / y moments swap
    const q = massProps(visualMoments({ primitive: { kind: 'box', size: [300, 200, 100] }, origin: Array.from(mul(transl(1000, 0, 0), rotz(90 * DEG))) } as never)!, 1000)!;
    expect(q.com[0]).toBeCloseTo(1, 12);
    expect(q.I[0]).toBeCloseTo(p.I[3], 12);
    expect(q.I[3]).toBeCloseTo(p.I[0], 12);
    const c = massProps(meshMoments(cylSoup(0.05, 0.4, 512)), 2700)!;
    const mc = 2700 * Math.PI * 0.05 ** 2 * 0.4;
    expect(Math.abs(c.mass / mc - 1)).toBeLessThan(1e-4);
    expect(Math.abs(c.I[0] / ((mc * (3 * 0.05 ** 2 + 0.4 ** 2)) / 12) - 1)).toBeLessThan(1e-3);
    expect(Math.abs(c.I[5] / ((mc * 0.05 ** 2) / 2) - 1)).toBeLessThan(1e-3);
  });

  it('URDF export of a native robot (procedural links): every link has an inertial, positive definite, triangle inequality, total = params.massKg', () => {
    const r = createRobotFromLibrary('UR5e', 'UR5e');
    r.params.massKg = 20.6;
    const urdf = exportRobotURDF(r).urdf;
    const model = parseURDF(urdf);
    // KDL: no inertia on the root link — it sits on the fixed child <root>_inertia (as in ur_description)
    expect(model.links.get(model.rootLink)!.inertial).toBeFalsy();
    const links = [...model.links.values()].filter((l) => !/tool0/.test(l.name) && l.name !== model.rootLink);
    expect(links.map((l) => l.name)).toContain(`${model.rootLink}_inertia`);
    expect(links.length).toBe(7);
    let total = 0;
    for (const l of links) {
      const inert = l.inertial!;
      expect(inert, l.name).toBeTruthy();
      expect(inert.mass).toBeGreaterThan(0);
      total += inert.mass;
      const [ixx, ixy, ixz, iyy, iyz, izz] = inert.inertia;
      // positive definite (leading principal minors) and the triangle inequality of principal moments
      expect(ixx).toBeGreaterThan(0);
      expect(ixx * iyy - ixy * ixy).toBeGreaterThan(0);
      expect(ixx * (iyy * izz - iyz * iyz) - ixy * (ixy * izz - iyz * ixz) + ixz * (ixy * iyz - iyy * ixz)).toBeGreaterThan(0);
      expect(ixx + iyy).toBeGreaterThanOrEqual(izz * (1 - 1e-6)); expect(iyy + izz).toBeGreaterThanOrEqual(ixx * (1 - 1e-6)); expect(ixx + izz).toBeGreaterThanOrEqual(iyy * (1 - 1e-6));
    }
    expect(Math.abs(total - 20.6)).toBeLessThan(1e-4); // toPrecision(7) per link
    // without a mass: default density 1200 kg/m³, link masses follow the link volumes
    const plain = parseURDF(exportRobotURDF(createRobotFromLibrary('UR5e', 'UR5e')).urdf);
    const ratio = plain.links.get('link_2')!.inertial!.mass / model.links.get('link_2')!.inertial!.mass;
    expect(plain.links.get('link_3')!.inertial!.mass / model.links.get('link_3')!.inertial!.mass).toBeCloseTo(ratio, 5);
    expect(exportRobotURDF(r, undefined, { inertials: false }).urdf).not.toMatch(/<inertial>/);
  });

  it('mesh visuals: a link with an STL box gets the inertial of that box (density 1200)', () => {
    const r = createRobotFromLibrary('UR5e', 'UR5e');
    const assets = new AssetStore();
    const soup = Float32Array.from(boxSoup(100, 80, 60)); // mm
    assets.registerMesh('box.stl', { positions: soup, normals: new Float32Array(soup.length), min: [-50, -40, -30], max: [50, 40, 30], triangles: soup.length / 9 });
    r.chain.links[2].visuals = [{ mesh: 'box.stl', origin: Array.from(transl(0, 0, 200)), scale: [1, 1, 1] } as never];
    const l2 = parseURDF(exportRobotURDF(r, assets).urdf).links.get('link_2')!.inertial!;
    expect(l2.mass).toBeCloseTo(1200 * 0.1 * 0.08 * 0.06, 9);
    expect(l2.origin[14]).toBeCloseTo(200, 6); // com in mm in the parsed model
    expect(l2.inertia[0]).toBeCloseTo((l2.mass / 12) * (0.08 ** 2 + 0.06 ** 2), 9);
    expect(linkMassProps([], () => null)).toBeNull();
  });
});

describe('MoveIt 2 config skeleton', () => {
  const r = createRobotFromLibrary('UR5e', 'UR5e');
  const res = exportMoveItConfig(r);
  const f = (p: string) => String(res.files[`${res.packages.moveit}/${p}`]);

  it('writes the MoveItConfigsBuilder layout with the group joints in SRDF, controllers, limits and ros2_control', () => {
    expect(res.packages).toEqual({ description: 'ur5e_description', moveit: 'ur5e_moveit_config' });
    for (const p of ['package.xml', 'CMakeLists.txt', 'config/UR5e.urdf.xacro', 'config/UR5e.srdf', 'config/kinematics.yaml', 'config/joint_limits.yaml', 'config/ros2_controllers.yaml', 'config/moveit_controllers.yaml', 'config/initial_positions.yaml', 'config/pilz_cartesian_limits.yaml', 'launch/bringup.launch.py', 'launch/demo.launch.py', 'launch/move_group.launch.py']) expect(res.files[`ur5e_moveit_config/${p}`], p).toBeTruthy();
    expect(res.files['ur5e_description/urdf/UR5e.urdf']).toBeTruthy();
    expect(res.joints).toEqual(['J1', 'J2', 'J3', 'J4', 'J5', 'J6']);
    expect(res.group).toBe('manipulator');
    const urdf = parseURDF(f('config/UR5e.urdf.xacro'));
    expect([...urdf.joints.values()].filter((j) => j.type !== 'fixed').map((j) => j.name)).toEqual(res.joints);
    expect(f('config/UR5e.urdf.xacro')).toMatch(/<plugin>mock_components\/GenericSystem<\/plugin>/);
    expect((f('config/UR5e.urdf.xacro').match(/<command_interface name="position"\/>/g) ?? []).length).toBe(6);
    expect(f('config/UR5e.urdf.xacro')).toMatch(/<joint name="J2">[\s\S]*?<param name="initial_value">-1\.5707963267948966<\/param>/);
    // the chain group ends at the last link (tool0 is its fixed child); named pose home in rad
    expect(f('config/UR5e.srdf')).toMatch(/<group name="manipulator">\s*<chain base_link="base" tip_link="link_6"\/>/);
    expect(f('config/UR5e.srdf')).toMatch(/<joint name="J2" value="-1\.570796327"\/>/);
    const ctrl = parseRosYaml(f('config/ros2_controllers.yaml')) as any;
    expect(ctrl.controller_manager.ros__parameters.update_rate).toBe(100);
    expect(f('config/ros2_controllers.yaml')).toMatch(/update_rate: 100\n/);
    expect(ctrl.controller_manager.ros__parameters.manipulator_controller.type).toBe('joint_trajectory_controller/JointTrajectoryController');
    expect(ctrl.manipulator_controller.ros__parameters.joints).toEqual(res.joints);
    const mc = parseRosYaml(f('config/moveit_controllers.yaml')) as any;
    expect(mc.moveit_simple_controller_manager.manipulator_controller).toMatchObject({ type: 'FollowJointTrajectory', action_ns: 'follow_joint_trajectory', default: true, joints: res.joints });
    const lim = parseRosYaml(f('config/joint_limits.yaml')) as any;
    expect(lim.joint_limits.J1.max_velocity).toBeCloseTo(Math.PI, 12); // 180 deg/s
    expect(lim.joint_limits.J1.max_acceleration).toBeCloseTo(4 * Math.PI, 12);
    expect(f('config/joint_limits.yaml')).toMatch(/default_velocity_scaling_factor: 0\.1\n/);
    expect(f('config/kinematics.yaml')).toMatch(/kinematics_solver: kdl_kinematics_plugin\/KDLKinematicsPlugin/);
    expect(f('launch/bringup.launch.py')).toMatch(/MoveItConfigsBuilder\('UR5e', package_name='ur5e_moveit_config'\)/);
  });

  const WSL = process.env.ROS_WSL === '1';
  it.skipIf(!WSL)('WSL Jazzy: colcon build, move_group ready with mock hardware, joint-space plan, execution through the controller', () => {
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'r5_moveit_'));
    try {
      for (const [p, c] of Object.entries(res.files)) { const out = path.join(dir, p); fs.mkdirSync(path.dirname(out), { recursive: true }); fs.writeFileSync(out, c); }
      const target = [0.5, -1.2, 1.4, -1.6, -1.5, 0.3];
      const jc = res.joints.map((n, i) => `{joint_name: ${n}, position: ${target[i]}, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0}`).join(', ');
      fs.writeFileSync(path.join(dir, 'plan_request.yaml'), `{motion_plan_request: {group_name: ${res.group}, num_planning_attempts: 1, allowed_planning_time: 5.0, max_velocity_scaling_factor: 0.5, max_acceleration_scaling_factor: 0.5, goal_constraints: [{joint_constraints: [${jc}]}]}}`);
      fs.writeFileSync(path.join(dir, 'goal.yaml'), `{trajectory: {joint_names: [${res.joints.join(', ')}], points: [{positions: [${target.join(', ')}], time_from_start: {sec: 3, nanosec: 0}}]}}`);
      const toWsl = (p: string) => `/mnt/${p[0].toLowerCase()}${p.slice(2).split(path.sep).join('/')}`;
      const run = spawnSync('wsl.exe', ['-d', 'Ubuntu-22.04', '--', 'bash', toWsl(path.resolve(__dirname, 'fixtures/ros_gt/tools/moveit_smoke.sh')), toWsl(dir), res.packages.moveit, res.controller], { encoding: 'utf8', timeout: 900000, env: { ...process.env, MSYS_NO_PATHCONV: '1' }, windowsHide: true });
      const out = `${run.stdout}${run.stderr}`;
      fs.mkdirSync(path.resolve(__dirname, '_probe'), { recursive: true });
      fs.writeFileSync(path.resolve(__dirname, '_probe/r5_moveit_out.txt'), out);
      expect(out).toContain('BUILD_OK');
      expect(out).toContain('MOVE_GROUP_READY');
      expect(out).toContain('CONTROLLER_ACTIVE');
      expect(out).toMatch(/PLAN_CODE=1\b/); // MoveItErrorCodes.SUCCESS
      expect(out).toContain('EXECUTE_SUCCEEDED');
      const js = /JOINT_STATES (.*)/.exec(out)![1];
      res.joints.forEach((n, i) => { const v = Number(new RegExp(`${n}=([-0-9.]+)`).exec(js)![1]); expect(Math.abs(v - target[i]), n).toBeLessThan(0.01); });
    } finally { fs.rmSync(dir, { recursive: true, force: true }); }
  }, 1200000);
});

describe('generic twin: camera images for non-MURKA stations', () => {
  it('rgb camera -> rgb8 Image + CameraInfo (render K) + TF map -> <cam>_link -> <cam>_optical_frame; depth camera -> 32FC1 in m', () => {
    const st = new Station('cell');
    const box = st.addChild(new SceneObject('Box'));
    box.geometry = [{ primitive: { kind: 'box', size: [400, 400, 400] }, color: '#ff0000' }];
    box.setPose(transl(2000, 0, 200));
    const cam = st.addChild(new Camera('Cell cam'));
    cam.width = 64; cam.height = 48; cam.fov = 40; cam.near = 50; cam.far = 10000;
    cam.setPose(mul(transl(0, 0, 200), rotz(-90 * DEG), rotx(-90 * DEG))); // optical +Z = world +X
    const dcam = st.addChild(new Camera('Depth cam'));
    dcam.kind = 'depth'; dcam.width = 32; dcam.height = 24; dcam.fov = 40; dcam.near = 100; dcam.far = 5000;
    dcam.setPose(cam.pose());
    const sent: { topic: string; type: string; msg: any }[] = [];
    const twin = new RosTwin(st, { publish: (topic, type, msg) => sent.push({ topic, type, msg }), subscribe: () => {} });
    twin.start(0);
    for (let i = 0; i < 10; i++) twin.step(0.02); // 0.2 s at 5 Hz -> one camera frame
    const last = (t: string) => [...sent].reverse().find((s) => s.topic === t);
    const img = last('/cell_cam/image_raw')!;
    expect(img.type).toBe('sensor_msgs/msg/Image');
    expect([img.msg.width, img.msg.height, img.msg.encoding, img.msg.step, img.msg.header.frame_id]).toEqual([64, 48, 'rgb8', 192, 'cell_cam_optical_frame']);
    const px = Buffer.from(img.msg.data, 'base64');
    const c = (24 * 64 + 32) * 3; // centre pixel: the red box face
    expect(px[c]).toBeGreaterThan(150); expect(px[c + 1]).toBeLessThan(60); expect(px[c + 2]).toBeLessThan(60);
    const info = last('/cell_cam/camera_info')!.msg;
    const K = cameraIntrinsics(cam);
    expect(info.k).toEqual([K.fx, 0, K.cx - 0.5, 0, K.fy, K.cy - 0.5, 0, 0, 1]);
    const dep = last('/depth_cam/depth/image_raw')!.msg;
    expect(dep.encoding).toBe('32FC1');
    const b = Buffer.from(dep.data, 'base64');
    const z = new Float32Array(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength));
    expect(z[12 * 32 + 16]).toBeCloseTo(1.8, 3); // box face at x = 1.8 m, depth along the optical axis
    const tf = sent.filter((s) => s.topic === '/tf').flatMap((s) => s.msg.transforms).filter((t: any) => t.child_frame_id.startsWith('cell_cam'));
    expect(tf.map((t: any) => `${t.header.frame_id}->${t.child_frame_id}`)).toEqual(expect.arrayContaining(['map->cell_cam_link', 'cell_cam_link->cell_cam_optical_frame']));
    // publish-only twins (RosBridge 'publish' mode passes commands: false) do not render
    const sent2: string[] = [];
    const ro = new RosTwin(st, { publish: (t) => sent2.push(t), subscribe: () => {} }, { commands: false });
    ro.start(0); for (let i = 0; i < 10; i++) ro.step(0.02);
    expect(sent2.filter((t) => t.includes('image'))).toEqual([]);
  });
});

describe('URDF / package imports feeding vision and collision', () => {
  it('a gazebo camera (horizontal_fov) becomes a Camera with fovAxis horizontal: K fx = W / (2 tan(hfov/2))', () => {
    const urdf = `<robot name="r"><link name="base_link"/><link name="cam_link"/>
      <joint name="cj" type="fixed"><parent link="base_link"/><child link="cam_link"/><origin xyz="0 0 1"/></joint>
      <gazebo reference="cam_link"><sensor type="camera" name="front"><camera><horizontal_fov>1.2</horizontal_fov>
      <image><width>320</width><height>240</height></image><clip><near>0.05</near><far>30</far></clip></camera></sensor></gazebo></robot>`;
    const res = importURDF(urdf);
    const cams = [...res.root.walk()].filter((c): c is Camera => c instanceof Camera);
    expect(cams).toHaveLength(1);
    expect(cams[0].fovAxis).toBe('horizontal');
    expect(cams[0].fov).toBeCloseTo(1.2 / DEG, 9);
    expect(cameraIntrinsics(cams[0]).fx).toBeCloseTo(320 / (2 * Math.tan(0.6)), 9);
    expect([cams[0].near, cams[0].far]).toEqual([50, 30000]);
  });

  it('SRDF <disable_collisions> reach the collision checker: a disabled pair in contact is skipped, other contacts still reported', async () => {
    const box = (name: string, x: number, len: number) => `<link name="${name}"><visual><origin xyz="${x} 0 0"/><geometry><box size="${len} 0.04 0.04"/></geometry></visual></link>`;
    const joint = (n: string, p: string, c: string, x: number) => `<joint name="${n}" type="revolute"><parent link="${p}"/><child link="${c}"/><origin xyz="${x} 0 0"/><axis xyz="0 0 1"/><limit lower="-3.2" upper="3.2" effort="1" velocity="1"/></joint>`;
    const urdf = `<robot name="folder">${box('base_link', 0, 0.1)}${box('link1', 0.2, 0.4)}${box('link2', 0.2, 0.4)}${box('link3', 0.2, 0.4)}${joint('j1', 'base_link', 'link1', 0)}${joint('j2', 'link1', 'link2', 0.4)}${joint('j3', 'link2', 'link3', 0.4)}</robot>`;
    const srdf = '<robot name="folder"><group name="arm"><chain base_link="base_link" tip_link="link3"/></group><disable_collisions link1="base_link" link2="link3" reason="Never"/></robot>';
    const load = async (withSrdf: boolean) => {
      const files = new Map<string, Uint8Array>([['folder/folder.urdf', new TextEncoder().encode(urdf)]]);
      if (withSrdf) files.set('folder/folder.srdf', new TextEncoder().encode(srdf));
      const res = await importRosPackageFiles(files, new AssetStore());
      const st = new Station(); st.addChild(res.items[0]);
      const r = res.robots[0];
      r.setJoints([0, 180, 180]); // link2 folds back over link1, link3 folds forward again: through the base and along link1
      return checkRobotCollisions(st, r, { selfOnly: true }).map((p) => [p.a.part, p.b.part].sort().join('|')).sort();
    };
    const all = await load(false);
    expect(all).toContain('base_link|link3');
    expect(all).toContain('link1|link3');
    const filtered = await load(true);
    expect(filtered).not.toContain('base_link|link3');
    expect(filtered).toContain('link1|link3');
  });
});

describe('detectFruit and harvesting', () => {
  it('harvested fruit (picked) is no longer detected; every unpicked detection stays (fruit behind picked ones may appear)', () => {
    const st = new Station();
    const field = st.addChild(new FieldItem('F'));
    field.polygon = rectPolygon(20000, 10000);
    field.crop = cropParams('apple', { headland: 2000 });
    const row = generateOrchard(field, 3)[0];
    const [x, y] = row.pointAt(row.length() / 2, -1500);
    const cam = st.addChild(new Camera('Cam'));
    cam.fov = 70; cam.width = 640; cam.height = 480; cam.far = 8000;
    cam.setPose(mul(transl(x, y, 1500), rotz(90 * DEG), roty(90 * DEG)));
    const before = detectFruit(st, cam, { onlyRipe: false, occlusionPerMeter: 0 });
    expect(before.length).toBeGreaterThan(20);
    const picked = before.filter((_, i) => i % 3 === 0);
    for (const d of picked) d.fruit.picked = true;
    const after = detectFruit(st, cam, { onlyRipe: false, occlusionPerMeter: 0 });
    expect(after.some((d) => d.fruit.picked)).toBe(false);
    const seen = new Set(after.map((d) => d.fruit));
    for (const d of before) if (!d.fruit.picked) expect(seen.has(d.fruit)).toBe(true);
    expect(after.length).toBeGreaterThanOrEqual(before.length - picked.length);
  });
});
