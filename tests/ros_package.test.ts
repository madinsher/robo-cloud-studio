/**
 * T5 P0 c): ROS package / workspace import (interface I7, src/io/urdf/package.ts importRosPackageFiles).
 * A dropped folder / zip of ROS packages: package.xml discovery, package:// + $(find) inside the drop, robot entry points
 * from launch files (xacro command + DeclareLaunchArgument defaults), macro-only / included xacros are not robots, only
 * referenced meshes are registered (DAE honours <unit>), expansion errors are surfaced instead of a 0-DOF robot,
 * SRDF groups / named poses / disabled collisions applied. Ground truth: ROS 2 Jazzy xacro output in
 * tests/fixtures/ros_gt/xacro (same args as the launch defaults).
 */
import { describe, it, expect } from 'vitest';
import { readFileSync, readdirSync, statSync, existsSync } from 'node:fs';
import { join, relative } from 'node:path';
import { zipSync, strToU8 } from 'fflate';
import { importRosPackageFiles, discoverPackages, launchEntries, normalizeDrop } from '../src/io/urdf/package';
import { parseURDF } from '../src/io/urdf/urdf';
import { AssetStore } from '../src/scene/assets';
import { FIX, MURKA, read } from './ros_gt_util';

function readTree(root: string, prefix: string, out = new Map<string, Uint8Array>(), skip = /(^|\/)(build|install|log|\.git)(\/|$)/): Map<string, Uint8Array> {
  const walk = (d: string) => {
    for (const f of readdirSync(d)) {
      const p = join(d, f);
      const rel = `${prefix}/${relative(root, p).replace(/\\/g, '/')}`;
      if (skip.test(rel)) continue;
      if (statSync(p).isDirectory()) walk(p); else out.set(rel, new Uint8Array(readFileSync(p)));
    }
  };
  walk(root);
  return out;
}
const movableNames = (xml: string) => parseURDF(xml).joints.filter((j) => ['revolute', 'continuous', 'prismatic'].includes(j.type) && !j.mimic).map((j) => j.name).sort();

const MUROM = `${MURKA}/robot-retailer-murka-v2/ros2_ws/src/murom_description`;
const GW = `${MURKA}/simulation_ws/src/agromarca/murca_garden_ws`;
const MURCA = `${GW}/src/murca_description`;
const TB4 = join(FIX, 'pkgs', 'nav2_minimal_tb4_description');

describe('ROS package import (I7)', () => {
  it.skipIf(!existsSync(MUROM))('murom_description: one robot from the launch file, every movable joint, meshes under package:// only', async () => {
    const files = readTree(MUROM, 'ws/src/murom_description');
    const assets = new AssetStore();
    const res = await importRosPackageFiles(files, assets);
    expect(res.errors).toEqual([]);
    expect(res.packages).toEqual({ murom_description: 'ws/src/murom_description' });
    expect(res.entries.map((e) => [e.file, e.source])).toEqual([['ws/src/murom_description/urdf/murom.urdf', 'ws/src/murom_description/launch/description.launch.py']]);
    expect(res.items.length).toBe(1); // no stray mesh objects, no extra robots
    const gt = read(join(FIX, 'xacro', 'murom.urdf'));
    const jointsKept = new Set(res.robots.flatMap((r) => r.jointNames()));
    for (const it of res.items[0].walk()) for (const k of ['wheelJoints', 'steerJoints']) for (const j of (it.params[k] as string[]) ?? []) jointsKept.add(j);
    expect(movableNames(gt).length).toBe(16);
    for (const j of movableNames(gt)) expect(jointsKept.has(j), `joint ${j}`).toBe(true);
    // every mesh of the robot resolved inside the drop and registered by its package URI (STL in metres)
    expect(res.report.some((r) => /not in the drop/.test(r))).toBe(false);
    const meshIds = res.models[0].meshes;
    expect(meshIds.length).toBeGreaterThan(10);
    for (const m of meshIds) expect(assets.has(m), m).toBe(true);
    const base = assets.get('package://murom_description/meshes/v2/base_link.stl');
    expect(base?.mesh).toBeDefined();
    const ext = [0, 1, 2].map((i) => (base!.mesh!.max[i] - base!.mesh!.min[i]) * 1000);
    for (const e of ext) { expect(e).toBeGreaterThan(400); expect(e).toBeLessThan(1000); } // base ~774 x 772 x 632 mm
  }, 60000);

  it.skipIf(!existsSync(MURCA))('murca_description: launch defaults drive xacro, macro-only xacros are not robots, tree == ROS xacro', async () => {
    const files = readTree(MURCA, 'src/murca_description');
    const res = await importRosPackageFiles(files, new AssetStore());
    expect(res.errors).toEqual([]);
    const launch = res.entries.filter((e) => e.source.endsWith('display.launch.py'));
    expect(launch.length).toBe(1);
    expect(launch[0].args).toEqual({ robot_name: 'utp_g_01', slot_top: 'manipulator', slot_front: 'rgbd_camera', slot_cargo: 'crate', use_manipulator: 'true' });
    expect(res.items.length).toBe(1);
    const gt = parseURDF(read(join(FIX, 'xacro', 'murca_robot.urdf')));
    const m = res.models[0];
    expect([...m.links.keys()].sort()).toEqual([...gt.links.keys()].sort());
    for (const j of gt.joints) {
      const o = m.joints.find((x) => x.name === j.name)!;
      expect(o, j.name).toBeDefined();
      for (let k = 0; k < 16; k++) expect(Math.abs(o.origin[k] - j.origin[k]), `${j.name} origin[${k}]`).toBeLessThan(1e-6);
    }
    // the aerial platform is a second robot description of the package: offered, not imported
    expect(res.report.some((r) => /Also available: .*aerial_platform\.urdf\.xacro/.test(r))).toBe(true);
  }, 60000);

  it('TurtleBot4 description (fixture copy of /opt/ros/jazzy): PathJoinSubstitution entry, 32 links like ROS, drop as zip', async () => {
    const files = readTree(TB4, 'nav2_minimal_tb4_description');
    const zipped = new Map([['tb4.zip', zipSync(Object.fromEntries(files))]]);
    for (const drop of [files, zipped]) {
      const res = await importRosPackageFiles(drop, new AssetStore());
      expect(res.errors).toEqual([]);
      expect(res.entries.filter((e) => e.source !== 'top-level').map((e) => e.file.replace(/^tb4\//, ''))).toEqual(['nav2_minimal_tb4_description/urdf/standard/turtlebot4.urdf.xacro']);
      expect(res.items.length).toBe(1);
      const gt = parseURDF(read(join(FIX, 'xacro', 'tb4.urdf')));
      expect([...res.models[0].links.keys()].sort()).toEqual([...gt.links.keys()].sort());
      expect(res.report.some((r) => /mesh file\(s\) not in the drop/.test(r))).toBe(true); // the fixture has no meshes: reported, not fatal
      expect(res.report.join('\n')).toMatch(/mobile base \(diff/);
    }
  });

  it('a xacro whose include is missing is an error, never a 0-DOF robot', async () => {
    const files = new Map<string, Uint8Array>([
      ['p/package.xml', strToU8('<package format="3"><name>bad_description</name></package>')],
      ['p/urdf/robot.urdf.xacro', strToU8('<robot name="r" xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:include filename="$(find bad_description)/urdf/missing.xacro"/><link name="base"/></robot>')],
    ]);
    const res = await importRosPackageFiles(files, new AssetStore());
    expect(res.items).toEqual([]);
    expect(res.errors.length).toBe(1);
    expect(res.errors[0]).toMatch(/robot\.urdf\.xacro.*missing\.xacro/);
  });

  // V5 F7 (nav2_minimal_tb3_sim): spawn_tb3.launch.py expands gz_waffle.sdf.xacro for Gazebo; the URDF next to it is the robot
  it('a launch entry that expands to an SDF (Gazebo) is skipped; the package URDF is still imported', async () => {
    const files = new Map<string, Uint8Array>([
      ['tb3_sim/package.xml', strToU8('<package format="3"><name>tb3_sim</name></package>')],
      ['tb3_sim/launch/spawn.launch.py', strToU8("import os\nfrom ament_index_python.packages import get_package_share_directory\nfrom launch_ros.actions import Node\ndef generate_launch_description():\n    d = get_package_share_directory('tb3_sim')\n    sdf = os.path.join(d, 'urdf', 'gz_waffle.sdf.xacro')\n    return LaunchDescription([Node(package='ros_gz_sim', executable='create', arguments=['-string', Command(['xacro', ' ', sdf, ' namespace:=', ''])])])\n")],
      ['tb3_sim/urdf/gz_waffle.sdf.xacro', strToU8('<?xml version="1.0"?>\n<sdf version="1.8" xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:arg name="namespace" default=""/><model name="waffle"><link name="base_footprint"/><link name="base_link"/><joint name="j" type="fixed"><parent>base_footprint</parent><child>base_link</child></joint></model></sdf>')],
      ['tb3_sim/urdf/waffle.urdf', strToU8('<?xml version="1.0"?>\n<!-- TurtleBot3 -->\n<robot name="turtlebot3_waffle"><link name="base_footprint"/><link name="base_link"/><link name="arm"/><joint name="base_joint" type="fixed"><parent link="base_footprint"/><child link="base_link"/></joint><joint name="lift" type="prismatic"><parent link="base_link"/><child link="arm"/><axis xyz="0 0 1"/><limit lower="0" upper="0.2" effort="1" velocity="1"/></joint></robot>')],
    ]);
    const res = await importRosPackageFiles(files, new AssetStore());
    expect(res.errors).toEqual([]);
    expect(res.items.map((i) => i.name)).toEqual(['turtlebot3_waffle']);
    expect(res.robots.length).toBe(1);
    expect(res.report.some((r) => /gz_waffle\.sdf\.xacro: <sdf> document \(not a URDF robot\), skipped/.test(r))).toBe(true);
  });

  it("a launch robot that cannot be expanded is an error, and the package's other robot is still imported", async () => {
    const files = new Map<string, Uint8Array>([
      ['p/package.xml', strToU8('<package format="3"><name>p_description</name></package>')],
      ['p/launch/display.launch.py', strToU8("import os\nfrom ament_index_python.packages import get_package_share_directory\ndef generate_launch_description():\n    f = os.path.join(get_package_share_directory('p_description'), 'urdf', 'broken.urdf.xacro')\n")],
      ['p/urdf/broken.urdf.xacro', strToU8('<robot name="broken" xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="a"/><xacro:wheel side="left"/></robot>')],
      ['p/urdf/good.urdf', strToU8('<robot name="good"><link name="a"/><link name="b"/><joint name="j" type="revolute"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>')],
    ]);
    const res = await importRosPackageFiles(files, new AssetStore());
    expect(res.items.map((i) => i.name)).toEqual(['good']);
    expect(res.errors.length).toBe(1);
    expect(res.errors[0]).toMatch(/broken\.urdf\.xacro.*wheel/);
  });

  // V5 F10 (webots_ros2_universal_robot): multirobot_launch.py expands ur5e_with_gripper.urdf.xacro name:=..., another
  // launch file loads the pre-expanded ur5e_with_gripper.urdf — the same robot, imported once, no spurious xacro error
  it('the same robot from its xacro and its pre-expanded URDF (two launch files) is imported once, without an arg error', async () => {
    const urdf = (name: string) => `<robot name="${name}"><link name="base_link"/><link name="tool0"/><joint name="shoulder_pan_joint" type="revolute"><parent link="base_link"/><child link="tool0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint></robot>`;
    const files = new Map<string, Uint8Array>([
      ['ws/ur_pkg/package.xml', strToU8('<package format="3"><name>ur_pkg</name></package>')],
      ['ws/ur_pkg/launch/multirobot_launch.py', strToU8("import os\nfrom ament_index_python.packages import get_package_share_directory\ndef generate_launch_description():\n    d = get_package_share_directory('ur_pkg')\n    x = xacro.process_file(os.path.join(d, 'resource', 'ur5e.urdf.xacro'), mappings={'name': 'UR5eWithGripper'})\n")],
      ['ws/ur_pkg/launch/moveit_launch.py', strToU8("import os\nfrom ament_index_python.packages import get_package_share_directory\ndef generate_launch_description():\n    d = get_package_share_directory('ur_pkg')\n    description = open(os.path.join(d, 'resource', 'ur5e.urdf')).read()\n")],
      // the xacro needs $(arg name) (no default): ROS rejects it without the launch file's mapping
      ['ws/ur_pkg/resource/ur5e.urdf.xacro', strToU8('<robot name="UR5e" xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="base_link"><visual><geometry><box size="0.1 0.1 0.1"/></geometry><material name="$(arg name)_grey"/></visual></link><link name="tool0"/><joint name="shoulder_pan_joint" type="revolute"><parent link="base_link"/><child link="tool0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint></robot>')],
      ['ws/ur_pkg/resource/ur5e.urdf', strToU8(urdf('UR5e'))],
    ]);
    const res = await importRosPackageFiles(files, new AssetStore());
    expect(res.errors).toEqual([]);
    expect(res.items.length).toBe(1);
    expect(res.robots.length).toBe(1);
    expect(res.items[0].params.urdfFile).toBe('ws/ur_pkg/resource/ur5e.urdf.xacro');
    expect(res.report.some((r) => /Same robot as ws\/ur_pkg\/resource\/ur5e\.urdf\.xacro .*ur5e\.urdf .*not imported twice/.test(r))).toBe(true);
    // two different robots from two launch files are both kept
    files.set('ws/ur_pkg/resource/ur5e.urdf', strToU8(urdf('UR10e')));
    const res2 = await importRosPackageFiles(files, new AssetStore());
    expect(res2.items.length).toBe(2);
  });

  it('an unknown macro call is an error', async () => {
    const files = new Map<string, Uint8Array>([
      ['p/package.xml', strToU8('<package format="3"><name>m_description</name></package>')],
      ['p/urdf/robot.urdf.xacro', strToU8('<robot name="r" xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="base"/><xacro:wheel side="left"/></robot>')],
    ]);
    const res = await importRosPackageFiles(files, new AssetStore());
    expect(res.items).toEqual([]);
    expect(res.errors[0]).toMatch(/wheel/);
  });

  it.skipIf(!existsSync(`${MURCA}/meshes/vehicle/hokuyo.dae`))('COLLADA meshes honour <unit> (hokuyo 50 x 50 x 70 mm, not metres)', async () => {
    const files = new Map<string, Uint8Array>([
      ['murca_description/package.xml', strToU8('<package format="3"><name>murca_description</name></package>')],
      ['murca_description/urdf/l.urdf', strToU8('<robot name="l"><link name="base"><visual><geometry><mesh filename="package://murca_description/meshes/vehicle/hokuyo.dae"/></geometry></visual></link></robot>')],
      ['murca_description/meshes/vehicle/hokuyo.dae', new Uint8Array(readFileSync(`${MURCA}/meshes/vehicle/hokuyo.dae`))],
      ['murca_description/meshes/vehicle/unused.stl', new Uint8Array(84)],
    ]);
    const assets = new AssetStore();
    const res = await importRosPackageFiles(files, assets);
    expect(res.errors).toEqual([]);
    const a = assets.get('package://murca_description/meshes/vehicle/hokuyo.dae')!;
    const vis = [...res.models[0].links.values()][0].visuals[0];
    const ext = [0, 1, 2].map((i) => Math.round((a.mesh!.max[i] - a.mesh!.min[i]) * vis.scale![i])).sort((x, y) => x - y);
    expect(ext).toEqual([50, 50, 70]);
    expect(assets.has('package://murca_description/meshes/vehicle/unused.stl')).toBe(false); // unreferenced meshes are not imported
  });

  it.skipIf(!existsSync(`${GW}/assets/urp_reference/urp.srdf`))('SRDF of the same robot: planning group, named poses (deg) and disabled collision pairs', async () => {
    const dir = `${GW}/assets/urp_reference`;
    const files = new Map<string, Uint8Array>(['urp.urdf', 'urp.srdf'].map((f) => [`urp_reference/${f}`, new Uint8Array(readFileSync(`${dir}/${f}`))]));
    const res = await importRosPackageFiles(files, new AssetStore());
    expect(res.errors).toEqual([]);
    expect(res.items.length).toBe(1);
    const arm = res.robots.find((r) => r.jointNames().includes('Joint_1'))!;
    expect(arm.params.planningGroups).toEqual(['manipulator']);
    const poses = arm.params.namedPoses as Record<string, number[]>;
    expect(Object.keys(poses).sort()).toEqual(['collect_apple_pose', 'home']);
    const i1 = arm.jointNames().indexOf('Joint_1');
    expect(poses.collect_apple_pose[i1]).toBeCloseTo(0.1735 * 180 / Math.PI, 6);
    const pairs = res.items[0].params.disabledCollisions as string[][];
    expect(pairs).toContainEqual(['Link_1', 'Link_2', 'Adjacent']);
    expect(res.items[0].params.srdf).toBe('urp');
  });

  it('launch parsing: os.path.join / pathlib / PathJoinSubstitution / ROS 1 XML entries with arg defaults', () => {
    const files = normalizeDrop(new Map<string, Uint8Array>([
      ['a/package.xml', strToU8('<package><name>a_desc</name></package>')],
      ['a/urdf/r.urdf.xacro', strToU8('<robot name="r"/>')],
      ['a/launch/one.launch.py', strToU8("pkg = get_package_share_directory('a_desc')\nx = os.path.join(pkg, 'urdf', 'r.urdf.xacro')\nDeclareLaunchArgument('side', default_value='left')\nside = LaunchConfiguration('side')\nCommand(['xacro ', x, ' side:=', side, ' mode:=fast'])\n")],
      ['a/launch/two.launch', strToU8('<launch><arg name="s" default="right"/><param name="robot_description" command="$(find xacro)/xacro $(find a_desc)/urdf/r.urdf.xacro side:=$(arg s)"/></launch>')],
    ]));
    const pk = discoverPackages(files);
    expect(pk).toEqual({ a_desc: 'a' });
    const e = launchEntries(files, pk, (p) => (files.has(p) ? p : null));
    expect(e).toEqual([
      { file: 'a/urdf/r.urdf.xacro', package: 'a_desc', args: { side: 'left', mode: 'fast' }, source: 'a/launch/one.launch.py' },
      { file: 'a/urdf/r.urdf.xacro', package: 'a_desc', args: { side: 'right' }, source: 'a/launch/two.launch' },
    ]);
  });
});
