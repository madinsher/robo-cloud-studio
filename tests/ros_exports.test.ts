/**
 * T5 P0 d): runnable ROS 2 exports. Static checks (always): the typed YAML writer, Nav2 / EKF / SLAM parameter files
 * typed like the ROS nodes declare them (no int for a double, no mixed sequences), launch files without template
 * leftovers and with consistent absolute topics. Runtime check (WSL ROS 2 Jazzy, opt-in with ROS_WSL=1 because it takes
 * 3-10 min): `colcon build` + `ros2 launch <pkg> bringup.launch.py` until Nav2 reports every node active, with no process dying
 * (tests/fixtures/ros_gt/tools/smoke_exports.sh).
 */
import { describe, it, expect } from 'vitest';
import { mkdirSync, writeFileSync, rmSync, existsSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { tmpdir } from 'node:os';
import { spawnSync } from 'node:child_process';
import { generateRosNavPackage } from '../src/mobile/navstack_ros';
import { LOCALIZATION_METHODS, NAVIGATION_METHODS, NavStackConfig } from '../src/mobile/navstack';
import { MobileRobot } from '../src/mobile/items';
import { toRosYaml, parseYaml, ydouble, yint, formatDouble } from '../src/ros/yaml';
import { importURDF, parseURDF, urdfLinkPoses, URDFModel } from '../src/io/urdf/urdf';
import { exportRobotURDF, exportDescriptionURDF, exportStationURDF } from '../src/io/urdf/urdf_export';
import { parseSRDF, srdfGroupJoints } from '../src/io/urdf/srdf';
import { AssetStore } from '../src/scene/assets';
import { Station } from '../src/core/items/item';
import { transl, RAD } from '../src/core/math/pose';
import { createRobotFromLibrary } from '../src/core/items/library';
import { sceneMeshes } from '../src/vision/sensing';
import { parseSTL } from '../src/io/mesh/stl';
import { FIX, read } from './ros_gt_util';

/** Parameters ROS nodes declare as double (rclcpp throws InvalidParameterTypeException on an integer literal). */
const DOUBLE_PARAMS = ['controller_frequency', 'expected_planner_frequency', 'frequency', 'smoothing_frequency', 'update_frequency', 'publish_frequency', 'max_planning_time', 'minimum_turning_radius',
  'transform_tolerance', 'inflation_radius', 'cost_scaling_factor', 'resolution', 'desired_linear_vel', 'lookahead_dist', 'max_vel_x', 'vx_max', 'model_dt', 'max_velocity', 'min_velocity', 'max_accel', 'max_decel',
  'delay', 'yaw_offset', 'xy_goal_tolerance', 'movement_time_allowance', 'max_laser_range', 'fov_degree', 'tolerance'];
/** Parameters declared as integer. */
const INT_PARAMS = ['bt_loop_duration', 'default_server_timeout', 'max_iterations', 'angle_quantization_bins', 'width', 'height', 'max_particles', 'min_particles', 'time_steps', 'batch_size', 'vx_samples', 'max_its', 'loop_rate', 'port', 'min_points', 'max_retries'];

function walk(v: any, key: string, raw: string, out: string[]) {
  if (Array.isArray(v)) {
    const kinds = new Set(v.map((x) => (typeof x === 'number' ? 'n' : typeof x)));
    if (kinds.size > 1) out.push(`${key}: mixed sequence ${JSON.stringify(v)}`);
    v.forEach((x) => walk(x, key, raw, out));
  } else if (v && typeof v === 'object') for (const [k, x] of Object.entries(v)) walk(x, k, raw, out);
}
/** Every textual scalar of `key` in the YAML text (block style `key: value` and flow sequences). */
const literals = (text: string, key: string) => [...text.matchAll(new RegExp(`^\\s*${key}:\\s*(.+)$`, 'gm'))].map((m) => m[1].trim());

const robots = () => {
  const amr = new MobileRobot('amr');
  const sprayer = new MobileRobot('Sprayer'); sprayer.kin.drive = 'ackermann'; sprayer.kin.maxSteer = 30; sprayer.kin.wheelBase = 1800; sprayer.rosNamespace = 'sprayer/';
  return { amr, sprayer };
};

describe('typed ROS YAML writer', () => {
  it('doubles always carry a decimal point, ints stay ints, sequences are homogeneous, re-typable strings are quoted', () => {
    expect(formatDouble(20)).toBe('20.0');
    expect(formatDouble(1e-10)).toBe('1.0e-10');
    expect(formatDouble(-0)).toBe('-0.0');
    const y = toRosYaml({ node: { ros__parameters: { f: 20, l: [1.5, 0, 1.57], i: yint(3), d: ydouble(2), s: 'true', on: 'on', t: '0.5', mode: 'mapping', b: true, a: [] } } }, { intKeys: ['k'] });
    expect(y).toBe('node:\n  ros__parameters:\n    f: 20.0\n    l: [1.5, 0.0, 1.57]\n    i: 3\n    d: 2.0\n    s: "true"\n    "on": "on"\n    t: "0.5"\n    mode: mapping\n    b: true\n    a: []\n');
    // round trip through the PyYAML-compatible reader keeps the types
    expect(parseYaml(y)).toEqual({ node: { ros__parameters: { f: 20, l: [1.5, 0, 1.57], i: 3, d: 2, s: 'true', on: 'on', t: '0.5', mode: 'mapping', b: true, a: [] } } });
    expect(() => toRosYaml({ x: [1, 'a'] })).toThrow(/mixed-type sequence/);
  });
});

describe('Nav2 / EKF / SLAM package export (static)', () => {
  const { amr, sprayer } = robots();
  const combos: [MobileRobot, NavStackConfig][] = [];
  for (const loc of LOCALIZATION_METHODS) for (const nav of NAVIGATION_METHODS) combos.push([loc.id.includes('rtk') || loc.family === 'lio' ? sprayer : amr, { platform: 'amr', environment: 'warehouse', sensors: ['wheel_odom', 'imu', 'lidar2d', 'lidar3d', 'gnss_rtk', 'rgbd_camera', 'stereo_camera'], localization: loc.id, navigation: nav.id, fusion: ['wheel_odom', 'imu'], simulate: false } as NavStackConfig]);
  it(`every localization x navigation combination (${combos.length}) writes ROS-typed parameter files`, () => {
    for (const [r, cfg] of combos) {
      const pkg = generateRosNavPackage(r, cfg);
      for (const [path, text] of Object.entries(pkg.files)) {
        if (!/\.ya?ml$/.test(path) || text.startsWith('%YAML:1.0')) continue; // OpenCV-style files are not ROS params
        const errs: string[] = [];
        walk(parseYaml(text), '', text, errs);
        for (const k of DOUBLE_PARAMS) for (const lit of literals(text, k)) if (!/^\[\]$/.test(lit) && !/^["']/.test(lit)) for (const n of lit.replace(/[[\]]/g, '').split(',')) if (!/[.e]|inf|nan/i.test(n.trim())) errs.push(`${k}: ${lit} (double expected)`);
        for (const k of INT_PARAMS) for (const lit of literals(text, k)) if (!/^-?\d+$/.test(lit)) errs.push(`${k}: ${lit} (int expected)`);
        expect(errs, `${cfg.localization}/${cfg.navigation} ${path}`).toEqual([]);
      }
      const launch = pkg.files['launch/bringup.launch.py'];
      expect(launch, `${cfg.localization} launch`).not.toMatch(/\$\{/);
      // remaps / topics are valid absolute ROS names; the namespace is normalized ('sprayer/' -> '/sprayer')
      for (const m of launch.matchAll(/\('([^']+)', '([^']+)'\)/g)) expect(m[2], `${cfg.localization} remap ${m[1]}`).toMatch(/^\/?[A-Za-z_][\w/]*$/);
      expect(launch).not.toMatch(/\/\//);
    }
  });

  it('RTK stacks: GNSS only in the map EKF with dense odomN numbering, navsat publishes where the EKF listens, hybrid LIO launched and fused', () => {
    const pkg = generateRosNavPackage(sprayer, { platform: 'tractor', environment: 'orchard', sensors: ['wheel_odom', 'imu', 'lidar3d', 'gnss_rtk'], localization: 'hybrid_rtk_lio', navigation: 'nav2_smac_hybrid_rpp', fusion: ['wheel_odom'], simulate: false });
    const odom = parseYaml(pkg.files['config/ekf_odom.yaml']).ekf_filter_node_odom.ros__parameters;
    const map = parseYaml(pkg.files['config/ekf_map.yaml']).ekf_filter_node_map.ros__parameters;
    expect(Object.keys(odom).filter((k) => /^odom\d+$/.test(k)).sort()).toEqual(['odom0', 'odom1']);
    expect(odom.odom0).toBe('/sprayer/wheel/odom');
    expect(odom.odom1).toBe('/Odometry'); // FAST-LIO2 odometry
    expect(odom.odom1_differential).toBe(true);
    expect(Object.values(odom)).not.toContain('/odometry/gps');
    expect(map.odom2).toBe('/odometry/gps');
    expect(map.world_frame).toBe('map');
    const launch = pkg.files['launch/bringup.launch.py'];
    expect(launch).toMatch(/\('odometry\/gps', '\/odometry\/gps'\)/);
    expect(launch).toMatch(/\('gps\/fix', '\/sprayer\/gps\/fix'\)/);
    expect(launch).toMatch(/opt\('fast_lio', lambda: Node\(package='fast_lio'/);
    expect(pkg.files['config/fast_lio.yaml']).toMatch(/lid_topic: \/sprayer\/points/);
  });

  it('costmap inflation covers the circumscribed footprint (Smac planners log an error otherwise) and plugins use pluginlib class types', () => {
    for (const [r, cfg] of combos) {
      const p = parseYaml(generateRosNavPackage(r, cfg).files['config/nav2_params.yaml']);
      const circ = Math.hypot(r.kin.footprint[0] / 2, r.kin.footprint[1] / 2) / 1000;
      for (const c of [p.local_costmap.local_costmap, p.global_costmap.global_costmap]) expect(c.ros__parameters.inflation_layer.inflation_radius).toBeGreaterThanOrEqual(circ);
      for (const plug of [p.controller_server.ros__parameters.FollowPath.plugin, p.planner_server.ros__parameters.GridBased.plugin, p.behavior_server.ros__parameters.spin.plugin]) expect(plug).toMatch(/^\w+::\w+$/);
    }
  });

  const python = spawnSync('python3', ['-c', 'import ast'], { encoding: 'utf8' }).status === 0;
  it.skipIf(!python)('generated launch files are valid Python', () => {
    const sources = combos.map(([r, cfg]) => generateRosNavPackage(r, cfg).files['launch/bringup.launch.py']);
    const res = spawnSync('python3', ['-c', 'import ast,sys,json\nsrc = json.load(sys.stdin)\nfor i, s in enumerate(src): ast.parse(s, "launch%d" % i)\nprint("parsed", len(src))'], { input: JSON.stringify(sources), encoding: 'utf8' });
    expect(res.stderr).toBe('');
    expect(res.stdout.trim()).toBe(`parsed ${sources.length}`);
  }, 60000);
});

describe('URDF package export keeps the imported description', () => {
  const qOf = (m: URDFModel) => Object.fromEntries(m.joints.filter((j) => ['revolute', 'continuous', 'prismatic'].includes(j.type)).map((j) => [j.name, j.type === 'prismatic' ? 30 : 0.4 * RAD]));
  for (const name of ['murca_robot_manip', 'palletizer', 'ur5e_gripper', 'tb4', 'vertical_robot']) {
    it(`${name}: names, inertials, mimic / off-chain joints, ros2_control survive; re-import has identical tree FK`, () => {
      const text = read(join(FIX, 'xacro', `${name}.urdf`));
      const res = importURDF(text);
      const exp = res.robots.length ? exportRobotURDF(res.robots[res.robots.length - 1], new AssetStore()) : exportDescriptionURDF(res.root, new AssetStore());
      const a = parseURDF(text), b = parseURDF(exp.urdf);
      expect([...b.links.keys()].sort()).toEqual([...a.links.keys()].sort());
      expect(b.joints.map((j) => `${j.name}:${j.type}:${j.mimic?.joint ?? ''}`).sort()).toEqual(a.joints.map((j) => `${j.name}:${j.type}:${j.mimic?.joint ?? ''}`).sort());
      const count = (s: string, tag: string) => (s.match(new RegExp(`<${tag}[\\s>]`, 'g')) ?? []).length;
      for (const tag of ['inertial', 'ros2_control', 'collision', 'visual', 'gazebo', 'transmission']) expect(count(exp.urdf, tag), tag).toBe(count(text, tag));
      for (const q of [{}, qOf(a)]) {
        const pa = urdfLinkPoses(a, q), pb = urdfLinkPoses(b, q);
        for (const [l, T] of pa) for (let k = 0; k < 16; k++) expect(Math.abs(pb.get(l)![k] - T[k]), `${l}[${k}]`).toBeLessThan(1e-6);
      }
      // runnable package: launch file + SRDF whose groups resolve to the robot joints; meshes not loaded are reported
      expect(Object.keys(exp.files)).toEqual(expect.arrayContaining(['package.xml', 'CMakeLists.txt', 'launch/display.launch.py']));
      if (res.robots.length) {
        const srdf = parseSRDF(exp.files[`config/${a.name.replace(/[^A-Za-z0-9_]+/g, '_')}.srdf`] as string);
        expect(srdf.groups.length).toBe(res.robots.length);
        // the group's independent joints are the robot's (a mechanism with cross-chain mimics adds its off-chain joints to the chain group)
        const indepOf = (names: string[]) => names.filter((n) => { const j = b.joints.find((x) => x.name === n); return j && j.type !== 'fixed' && !j.mimic; }).sort().join();
        for (const r of res.robots) expect(srdf.groups.some((g) => indepOf(srdfGroupJoints(srdf, b, g.name)) === r.chain.joints.filter((j) => !j.mimic).map((j) => j.name).sort().join()), r.name).toBe(true);
        expect(srdf.states.filter((s) => s.name === 'home').length).toBe(res.robots.length);
      }
      if (a.meshes.length) expect(exp.warnings.some((w) => /not loaded/.test(w))).toBe(true);
      expect(exp.files['CMakeLists.txt']).not.toMatch(/meshes/); // no mesh folder -> not installed (colcon would fail)
    });
  }

  // V5 F13: Add › Robot from library › UR10e (drawn procedurally) exported with 0 <visual> / 0 <collision> — invisible
  // in RViz — and an inertia on the root link (KDL warning); station packages had no SRDF
  it('library robot without meshes: every link gets the visual + collision the 3D view draws, no root inertia; station packages carry an SRDF', () => {
    for (const model of ['UR10e', 'KUKA_KR6_R900', 'GENERIC_PALLETIZER_4']) {
      const st = new Station('Cell');
      const robot = st.addChild(createRobotFromLibrary(model, model)) as any;
      if (!robot.chain.links.every((l: any) => !l.visuals.length)) continue; // not a procedural robot
      const exp = exportRobotURDF(robot, new AssetStore());
      const m = parseURDF(exp.urdf);
      // every chain link (not the flange / inertia helper frames) has a mesh visual and the same mesh as collision
      for (const l of robot.chain.links) {
        const ln = String(l.name).replace(/[^A-Za-z0-9_/]+/g, '_');
        const link = m.links.get(ln)!;
        expect(link, ln).toBeDefined();
        expect(link.visuals.length, `${model} ${ln} visual`).toBeGreaterThan(0);
        expect(exp.urdf).toMatch(new RegExp(`<link name="${ln}">[\\s\\S]*?<collision>[\\s\\S]*?</link>`));
      }
      // KDL: the root link has no <inertial>; its mass sits on a fixed child link
      expect(m.links.get(m.rootLink)!.inertial).toBeFalsy();
      expect(m.links.get(`${m.rootLink}_inertia`)!.inertial!.mass).toBeGreaterThan(0);
      // the exported meshes, placed by the URDF FK, cover the same volume as the studio drawing, at two poses
      const qs = [robot.joints().map(() => 0), robot.joints().map((_: number, i: number) => [20, -35, 50, -40, 60, 30][i] ?? 0)];
      for (const q of qs) {
        robot.setJoints(q);
        const studio = sceneMeshes(st, {}).filter((x) => x.kind === 'robot');
        const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
        for (const s of studio) { const W = s.world, T = s.tris; for (let i = 0; i + 2 < T.length; i += 3) { const p = [W[0] * T[i] + W[4] * T[i + 1] + W[8] * T[i + 2] + W[12], W[1] * T[i] + W[5] * T[i + 1] + W[9] * T[i + 2] + W[13], W[2] * T[i] + W[6] * T[i + 1] + W[10] * T[i + 2] + W[14]]; for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], p[k]); hi[k] = Math.max(hi[k], p[k]); } } }
        const indep = robot.chain.joints.filter((j: any) => !j.mimic);
        const qn = Object.fromEntries(indep.map((j: any, i: number) => [j.name, q[i]]));
        const poses = urdfLinkPoses(m, qn);
        const lo2 = [Infinity, Infinity, Infinity], hi2 = [-Infinity, -Infinity, -Infinity];
        for (const [ln, link] of m.links) for (const v of link.visuals) {
          const file = String(v.mesh).replace(/^package:\/\/[^/]+\//, '');
          const bytes = exp.files[file] as Uint8Array;
          const mesh = parseSTL(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) as ArrayBuffer);
          const T = poses.get(ln)!;
          for (let i = 0; i < mesh.positions.length; i += 3) {
            const x = mesh.positions[i] * 1000, y = mesh.positions[i + 1] * 1000, z = mesh.positions[i + 2] * 1000;
            const p = [T[0] * x + T[4] * y + T[8] * z + T[12], T[1] * x + T[5] * y + T[9] * z + T[13], T[2] * x + T[6] * y + T[10] * z + T[14]];
            for (let k = 0; k < 3; k++) { lo2[k] = Math.min(lo2[k], p[k]); hi2[k] = Math.max(hi2[k], p[k]); }
          }
        }
        for (let k = 0; k < 3; k++) { expect(Math.abs(lo2[k] - lo[k]), `${model} q=${q} min[${k}]`).toBeLessThan(1); expect(Math.abs(hi2[k] - hi[k]), `${model} max[${k}]`).toBeLessThan(1); }
      }
      // whole-station package: config/<station>.srdf with one chain group per robot, joints and links of the URDF
      const se = exportStationURDF(st, new AssetStore());
      const srdfText = se.files['config/Cell.srdf'] as string;
      expect(srdfText, Object.keys(se.files).join()).toBeDefined();
      const sm = parseURDF(se.urdf), srdf = parseSRDF(srdfText);
      expect(srdf.groups.length).toBe(1);
      const gj = srdfGroupJoints(srdf, sm, srdf.groups[0].name).filter((n) => { const j = sm.joints.find((x) => x.name === n); return j && j.type !== 'fixed' && !j.mimic; });
      expect(gj.length).toBe(robot.chain.joints.filter((j: any) => !j.mimic).length);
      expect(srdf.states.find((s) => s.name === 'home')).toBeDefined();
    }
  });

  it('copies the original mesh files into the package and rewrites the URIs; station export keeps TF names', () => {
    const text = '<robot name="r"><link name="ns/base_link"><inertial><mass value="2"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial><visual><geometry><mesh filename="package://src_pkg/meshes/body.stl" scale="0.001 0.001 0.001"/></geometry></visual></link>'
      + '<joint name="ns/j1" type="revolute"><parent link="ns/base_link"/><child link="ns/l1"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint><link name="ns/l1"/></robot>';
    const assets = new AssetStore();
    const stl = new Uint8Array(84 + 50); new DataView(stl.buffer).setUint32(80, 1, true);
    assets.registerRaw('package://src_pkg/meshes/body.stl', 'stl', stl);
    const res = importURDF(text, { assets });
    const exp = exportRobotURDF(res.robots[0], assets, { packageName: 'out_description' });
    expect(exp.files['meshes/body.stl']).toEqual(stl);
    expect(exp.urdf).toContain('<mesh filename="package://out_description/meshes/body.stl" scale="0.001 0.001 0.001"/>');
    expect(exp.files['CMakeLists.txt']).toMatch(/install\(DIRECTORY urdf meshes launch config /);
    const st = new Station('cell'); st.addChild(res.root); res.root.setPose(transl(1000, 0, 0));
    const se = exportStationURDF(st, assets);
    const m = parseURDF(se.urdf);
    expect(m.links.has('ns/base_link') && m.links.has('ns/l1')).toBe(true);
    expect(m.joints.find((j) => j.child === 'ns/base_link')!.parent).toBe('world');
    expect(m.joints.find((j) => j.child === 'ns/base_link')!.origin[12]).toBeCloseTo(1000, 6);
    expect(m.links.get('ns/base_link')!.inertial!.mass).toBe(2);
  });
});

// ---------------------------------------------------------------------------------------------------------------------
const WSL = process.env.ROS_WSL === '1' && process.platform === 'win32';
describe('generated packages run in ROS 2 Jazzy (WSL)', () => {
  it.skipIf(!WSL)('colcon build + ros2 launch: every node of amr_nav (slam_toolbox + Nav2) and sprayer_nav (RTK + LIO + Smac/RPP) stays alive', () => {
    const ws = join(tmpdir(), 't5_ros_smoke');
    rmSync(ws, { recursive: true, force: true });
    const { amr, sprayer } = robots();
    const pkgs = [
      generateRosNavPackage(amr, { platform: 'amr', environment: 'warehouse', sensors: ['wheel_odom', 'imu', 'lidar2d'], localization: 'slam_toolbox_2d', navigation: 'nav2_navfn_dwb', fusion: ['wheel_odom', 'imu'], simulate: false }),
      generateRosNavPackage(sprayer, { platform: 'tractor', environment: 'orchard', sensors: ['wheel_odom', 'imu', 'lidar3d', 'gnss_rtk'], localization: 'hybrid_rtk_lio', navigation: 'nav2_smac_hybrid_rpp', fusion: ['wheel_odom'], simulate: false }),
      generateRosNavPackage(amr, { platform: 'amr', environment: 'warehouse', sensors: ['wheel_odom', 'imu', 'lidar2d'], localization: 'slam_toolbox_2d', navigation: 'nav2_smac2d_mppi', fusion: ['wheel_odom', 'imu'], simulate: false }, { packageName: 'amr_mppi_nav' }),
    ];
    for (const p of pkgs) for (const [f, c] of Object.entries(p.files)) { const fp = join(ws, 'src', p.name, f); mkdirSync(dirname(fp), { recursive: true }); writeFileSync(fp, c); }
    // URDF description packages (kept MURKA description with ros2_control; a native UR5e) -> display.launch.py
    const urdfPkgs = [exportDescriptionURDF(importURDF(read(join(FIX, 'xacro', 'murca_robot_manip.urdf'))).root), exportRobotURDF(createRobotFromLibrary('UR5e', 'UR5e'))];
    for (const p of urdfPkgs) for (const [f, c] of Object.entries(p.files)) { const fp = join(ws, 'src', p.packageName, f); mkdirSync(dirname(fp), { recursive: true }); writeFileSync(fp, c); }
    const toWsl = (p: string) => `/mnt/${p[0].toLowerCase()}${p.slice(2).replace(/\\/g, '/')}`;
    const res = spawnSync('wsl.exe', ['-d', 'Ubuntu-22.04', '--', 'bash', toWsl(join(FIX, 'tools', 'smoke_exports.sh')), toWsl(ws), ...pkgs.map((p) => p.name), ...urdfPkgs.map((p) => `${p.packageName}:display`)], { encoding: 'utf8', timeout: 1200000, env: { ...process.env, MSYS_NO_PATHCONV: '1' } });
    const out = `${res.stdout}\n${res.stderr}`;
    expect(out, out).toMatch(/BUILD OK/);
    for (const p of pkgs) expect(out, out).toMatch(new RegExp(`LAUNCH ${p.name} OK`));
    // robot_state_publisher started on each exported URDF and urdfdom (check_urdf) sees every link of the tree
    for (const p of urdfPkgs) {
      const m = new RegExp(`DISPLAY ${p.packageName} OK \\((\\d+) child links\\)`).exec(out);
      expect(m, out).not.toBeNull();
      expect(+m![1]).toBe(parseURDF(p.urdf).links.size - 1);
    }
    expect(out).not.toMatch(/process has died|InvalidParameterTypeException|RCLInvalidROSArgsError/);
  }, 1300000);
});
