/**
 * MoveIt 2 configuration skeleton for a studio robot: the robot description package (./urdf_export.ts, with the
 * estimated inertials) plus `<robot>_moveit_config` in the layout MoveIt Setup Assistant writes and
 * moveit_configs_utils.MoveItConfigsBuilder reads:
 *   config/<robot>.urdf.xacro      the URDF + a ros2_control block (mock_components/GenericSystem, position command,
 *                                  position/velocity state, initial values = the robot's home joints)
 *   config/<robot>.srdf            planning group (chain base -> tip), named poses, adjacent pairs disabled
 *   config/kinematics.yaml         KDL plugin; config/joint_limits.yaml (rad/s, rad/s², scaling 0.1)
 *   config/ros2_controllers.yaml   joint_state_broadcaster + <group>_controller (JointTrajectoryController)
 *   config/moveit_controllers.yaml MoveItSimpleControllerManager -> follow_joint_trajectory
 *   config/initial_positions.yaml, config/pilz_cartesian_limits.yaml
 *   launch/bringup.launch.py       headless: robot_state_publisher + ros2_control_node (mock hardware) + spawners +
 *                                  move_group;  launch/demo.launch.py  MoveIt demo with RViz (generate_demo_launch)
 * Every YAML is written with typed numbers (src/ros/yaml.ts toRosYaml).
 */
import type { Robot } from '../../core/items/robot';
import type { AssetStore } from '../../scene/assets';
import { exportRobotURDF, sanitizeName, frameName, type URDFExportOptions } from './urdf_export';
import { parseXML, childElements } from './xml';
import { toRosYaml } from '../../ros/yaml';

export interface MoveItExportResult {
  /** `<package>/<path>` -> content for both packages (zip or write as a colcon src folder). */
  files: Record<string, Uint8Array | string>;
  packages: { description: string; moveit: string };
  robotName: string;
  group: string;
  controller: string;
  joints: string[];
  warnings: string[];
}

export function exportMoveItConfig(robot: Robot, assets?: AssetStore, opts: URDFExportOptions & { moveitPackage?: string } = {}): MoveItExportResult {
  const rn = sanitizeName(robot.name);
  const descPkg = sanitizeName(opts.packageName ?? `${robot.name}_description`).toLowerCase();
  const pkg = sanitizeName(opts.moveitPackage ?? `${robot.name}_moveit_config`).toLowerCase();
  const desc = exportRobotURDF(robot, assets, { ...opts, packageName: descPkg, extras: true });
  const srdf = String(desc.files[`config/${rn}.srdf`] ?? '');
  const group = /<group name="([^"]+)"/.exec(srdf)?.[1] ?? 'manipulator';
  const controller = `${group}_controller`;
  // the group joints = the movable, non-mimic joints of the URDF (chain order)
  const doc = parseXML(desc.urdf);
  const urdfJoints = childElements(doc, 'joint').filter((j) => ['revolute', 'continuous', 'prismatic'].includes(j.attrs.type) && !childElements(j, 'mimic').length);
  const joints = urdfJoints.map((j) => j.attrs.name);
  const indep = robot.chain.joints.filter((j) => j.type !== 'fixed' && !j.mimic);
  const home = robot.jointsHome();
  const vmax = robot.jointMaxVelocities(), amax = robot.jointMaxAccelerations();
  const si = (i: number, v: number) => (indep[i]?.type === 'prismatic' ? v / 1000 : (v * Math.PI) / 180);
  const byName = new Map(indep.map((j, i) => [frameName(j.name), i]));
  const idx = (n: string) => byName.get(n) ?? joints.indexOf(n);

  const ros2Control = [
    `  <ros2_control name="${rn}_mock" type="system">`,
    '    <hardware>', '      <plugin>mock_components/GenericSystem</plugin>', '    </hardware>',
    ...joints.map((n) => `    <joint name="${n}">\n      <command_interface name="position"/>\n      <state_interface name="position">\n        <param name="initial_value">${si(idx(n), home[idx(n)] ?? 0)}</param>\n      </state_interface>\n      <state_interface name="velocity"/>\n    </joint>`),
    '  </ros2_control>',
  ].join('\n');
  const urdfXacro = desc.urdf.replace(/<robot name="([^"]*)">/, '<robot name="$1" xmlns:xacro="http://www.ros.org/wiki/xacro">').replace(/<\/robot>\s*$/, `${ros2Control}\n</robot>\n`);

  const limits: Record<string, unknown> = {};
  for (const n of joints) {
    const i = idx(n);
    const v = vmax[i] > 0 ? si(i, vmax[i]) : 1.0;
    const a = amax[i] > 0 ? si(i, amax[i]) : 2 * v;
    limits[n] = { has_velocity_limits: true, max_velocity: v, has_acceleration_limits: true, max_acceleration: a };
  }
  const y = (o: unknown, ints: string[] = []) => toRosYaml(o, { intKeys: ints });
  const cfg: Record<string, string> = {
    [`config/${rn}.urdf.xacro`]: urdfXacro,
    [`config/${rn}.srdf`]: srdf,
    'config/kinematics.yaml': y({ [group]: { kinematics_solver: 'kdl_kinematics_plugin/KDLKinematicsPlugin', kinematics_solver_search_resolution: 0.005, kinematics_solver_timeout: 0.005 } }),
    'config/joint_limits.yaml': `# rad/s, rad/s² (m/s for prismatic joints); MoveIt scales trajectories by the default factors\n${y({ default_velocity_scaling_factor: 0.1, default_acceleration_scaling_factor: 0.1, joint_limits: limits })}`,
    'config/ros2_controllers.yaml': y({
      controller_manager: { ros__parameters: { update_rate: 100, [controller]: { type: 'joint_trajectory_controller/JointTrajectoryController' }, joint_state_broadcaster: { type: 'joint_state_broadcaster/JointStateBroadcaster' } } },
      [controller]: { ros__parameters: { joints, command_interfaces: ['position'], state_interfaces: ['position', 'velocity'], allow_nonzero_velocity_at_trajectory_end: true } },
    }, ['update_rate']),
    'config/moveit_controllers.yaml': y({ moveit_controller_manager: 'moveit_simple_controller_manager/MoveItSimpleControllerManager', moveit_simple_controller_manager: { controller_names: [controller], [controller]: { type: 'FollowJointTrajectory', action_ns: 'follow_joint_trajectory', default: true, joints } } }),
    'config/initial_positions.yaml': y({ initial_positions: Object.fromEntries(joints.map((n) => [n, si(idx(n), home[idx(n)] ?? 0)])) }),
    'config/pilz_cartesian_limits.yaml': y({ cartesian_limits: { max_trans_vel: 1.0, max_trans_acc: 2.25, max_trans_dec: -5.0, max_rot_vel: 1.57 } }),
  };
  const builder = `MoveItConfigsBuilder('${rn}', package_name='${pkg}')`;
  cfg['launch/demo.launch.py'] = [
    `"""MoveIt 2 demo for ${robot.name} (RViz, mock hardware). Exported by VerticalBot Studio."""`,
    'from moveit_configs_utils import MoveItConfigsBuilder', 'from moveit_configs_utils.launches import generate_demo_launch', '', '',
    'def generate_launch_description():', `    return generate_demo_launch(${builder}.to_moveit_configs())`, '',
  ].join('\n');
  cfg['launch/move_group.launch.py'] = [
    `"""move_group of ${robot.name} (exported by VerticalBot Studio)."""`,
    'from moveit_configs_utils import MoveItConfigsBuilder', 'from moveit_configs_utils.launches import generate_move_group_launch', '', '',
    'def generate_launch_description():', `    return generate_move_group_launch(${builder}.to_moveit_configs())`, '',
  ].join('\n');
  cfg['launch/bringup.launch.py'] = [
    `"""Headless MoveIt 2 bring-up of ${robot.name}: robot_state_publisher, ros2_control (mock_components/GenericSystem),`,
    `joint_state_broadcaster + ${controller}, move_group. Exported by VerticalBot Studio."""`,
    'import os', 'from ament_index_python.packages import get_package_share_directory', 'from launch_ros.actions import Node',
    'from moveit_configs_utils import MoveItConfigsBuilder', 'from moveit_configs_utils.launches import generate_move_group_launch', '', '',
    'def generate_launch_description():',
    `    mc = ${builder}.to_moveit_configs()`,
    `    controllers = os.path.join(get_package_share_directory('${pkg}'), 'config', 'ros2_controllers.yaml')`,
    '    ld = generate_move_group_launch(mc)',
    "    ld.add_action(Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[mc.robot_description]))",
    "    ld.add_action(Node(package='controller_manager', executable='ros2_control_node', output='screen', parameters=[controllers],",
    "                       remappings=[('~/robot_description', '/robot_description')]))",
    `    for c in ['joint_state_broadcaster', '${controller}']:`,
    "        ld.add_action(Node(package='controller_manager', executable='spawner', arguments=[c, '--controller-manager', '/controller_manager'], output='screen'))",
    '    return ld', '',
  ].join('\n');
  cfg['package.xml'] = `<?xml version="1.0"?>
<package format="3">
  <name>${pkg}</name>
  <version>0.1.0</version>
  <description>MoveIt 2 configuration of ${robot.name} (exported by VerticalBot Studio)</description>
  <maintainer email="user@example.com">VerticalBot Studio</maintainer>
  <license>BSD-3-Clause</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
${[descPkg, 'moveit_ros_move_group', 'moveit_kinematics', 'moveit_planners', 'moveit_simple_controller_manager', 'moveit_configs_utils', 'robot_state_publisher', 'controller_manager', 'joint_state_broadcaster', 'joint_trajectory_controller', 'xacro'].map((d) => `  <exec_depend>${d}</exec_depend>`).join('\n')}
  <export><build_type>ament_cmake</build_type></export>
</package>
`;
  cfg['CMakeLists.txt'] = `cmake_minimum_required(VERSION 3.8)\nproject(${pkg})\nfind_package(ament_cmake REQUIRED)\ninstall(DIRECTORY config launch DESTINATION share/\${PROJECT_NAME})\nament_package()\n`;

  const files: Record<string, Uint8Array | string> = {};
  for (const [p, c] of Object.entries(desc.files)) files[`${descPkg}/${p}`] = c;
  for (const [p, c] of Object.entries(cfg)) files[`${pkg}/${p}`] = c;
  return { files, packages: { description: descPkg, moveit: pkg }, robotName: rn, group, controller, joints, warnings: desc.warnings };
}
