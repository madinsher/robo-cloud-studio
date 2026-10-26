/**
 * T5 acceptance: xacro expansion of real ROS packages equals the real ROS 2 Jazzy `xacro` output (element trees with
 * every attribute string). Inputs: MURKA packages (murca_description, murca_webots, realsense2_description) in place,
 * vertical_bot_ros palletizer, and text copies of /opt/ros/jazzy packages in tests/fixtures/ros_gt/pkgs
 * (ur_description, nav2_minimal_tb4_description, webots_ros2_universal_robot, robot_state_publisher,
 * nav2_minimal_tb3_sim, moveit_ros_planning, launch_param_builder, moveit_setup_controllers).
 * Regenerate the ground truth with tests/fixtures/ros_gt/tools/gen_gt.sh (WSL).
 */
import { describe, it, expect } from 'vitest';
import { existsSync } from 'node:fs';
import { expandXacro, XacroError } from '../src/io/urdf/xacro';
import { parseURDF } from '../src/io/urdf/urdf';
import { FIX, MURKA, VBOT, diffXml, diskResolver, read, join } from './ros_gt_util';

const AG = `${MURKA}/simulation_ws/src/agromarca`;
export const PKG_ROOTS: Record<string, string> = {
  murca_description: `${AG}/murca_garden_ws/src/murca_description`,
  murca_webots: `${AG}/murca_garden_ws/src/murca_webots`,
  realsense2_description: `${AG}/urp_gazebo_apples/ws/src/realsense2_description`,
  murom_description: `${MURKA}/robot-retailer-murka-v2/ros2_ws/src/murom_description`,
  palletizer_model_pkg: `${VBOT}/palletizer_model_pkg`,
};
for (const p of ['nav2_minimal_tb4_description', 'ur_description', 'webots_ros2_universal_robot', 'robot_state_publisher', 'nav2_minimal_tb3_sim', 'moveit_ros_planning', 'launch_param_builder', 'moveit_setup_controllers']) PKG_ROOTS[p] = join(FIX, 'pkgs', p);

const cases = read(join(FIX, 'xacro', 'cases.txt')).trim().split('\n').map((l) => { const [name, pkg, file, args] = l.split('|'); return { name, pkg, file, args: Object.fromEntries((args ?? '').split(/\s+/).filter(Boolean).map((a) => a.split(':='))) as Record<string, string> }; });

function expandCase(c: (typeof cases)[number]) {
  const root = PKG_ROOTS[c.pkg];
  return expandXacro(read(join(root, c.file)), { filename: `package://${c.pkg}/${c.file}`, args: c.args, packages: PKG_ROOTS, resolveInclude: diskResolver(root, PKG_ROOTS), env: {} });
}

describe('xacro on real ROS packages == ROS 2 xacro', () => {
  it('covers at least 10 distinct packages and 25 entry points', () => {
    const pkgs = new Set(cases.filter((c) => existsSync(join(FIX, 'xacro', `${c.name}.urdf`))).map((c) => c.pkg));
    expect(pkgs.size).toBeGreaterThanOrEqual(10);
    expect(cases.length).toBeGreaterThanOrEqual(25);
  });
  for (const c of cases) {
    const present = existsSync(join(PKG_ROOTS[c.pkg], c.file));
    const gt = join(FIX, 'xacro', `${c.name}.urdf`);
    it.skipIf(!present)(`${c.name} (${c.pkg}/${c.file}${Object.keys(c.args).length ? ` ${Object.entries(c.args).map(([k, v]) => `${k}:=${v}`).join(' ')}` : ''})`, () => {
      if (!existsSync(gt)) {
        const err = read(join(FIX, 'xacro', `${c.name}.err`));
        expect(() => expandCase(c), err.split('\n')[0]).toThrow(XacroError);
        expect(() => expandCase(c)).toThrow(/error: (.*)/.exec(err)![1].slice(0, 30));
        return;
      }
      const res = expandCase(c);
      expect(diffXml(res.xml, read(gt))).toEqual([]);
    });
  }
});

describe('regressions found by the investigators (MURKA murca_robot)', () => {
  const c = cases.find((x) => x.name === 'murca_robot')!;
  it.skipIf(!existsSync(join(PKG_ROOTS.murca_description, c.file)))('places IMU / camera like ROS and omits the manipulator (bool literal semantics)', () => {
    const m = parseURDF(expandCase(c).xml);
    expect(m.links.size).toBe(14);
    expect([...m.links.keys()].some((l) => l.includes('manipulator'))).toBe(false);
    const imu = m.joints.find((j) => j.name === 'utp_g_01/imu_joint')!;
    expect([imu.origin[12], imu.origin[13], imu.origin[14]].map((v) => Math.round(v))).toEqual([0, -1250, 300]);
    const cam = m.joints.find((j) => j.name === 'utp_g_01/camera_joint')!;
    expect([cam.origin[12], cam.origin[13], cam.origin[14]].map((v) => Math.round(v))).toEqual([0, -2600, 600]);
  });
  it('UR5e via xacro.load_yaml has finite limits and the ROS tool0 position', () => {
    const ur = cases.find((x) => x.name === 'ur5e')!;
    const m = parseURDF(expandCase(ur).xml);
    for (const j of m.joints.filter((x) => x.type === 'revolute')) { expect(Number.isFinite(j.lower)).toBe(true); expect(Number.isFinite(j.upper)).toBe(true); }
    expect(m.joints.find((j) => j.name === 'shoulder_pan_joint')!.upper).toBeCloseTo(360, 6); // !degrees 360.0
  });
});
