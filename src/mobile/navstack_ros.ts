/**
 * ROS 2 configuration export for a selected navigation/SLAM stack: Nav2 parameters, the SLAM / odometry
 * package configuration, robot_localization EKF fusion, GNSS (NTRIP) setup, a bringup launch file and a README
 * with the apt/rosdep packages. The output is a starting point that matches the vehicle's kinematics,
 * footprint, speed limits and sensor set as configured in the studio.
 */
import { zipSync, strToU8 } from 'fflate';
import type { MobileRobot } from './items';
import { NavStackConfig, LOCALIZATION_METHODS, NAVIGATION_METHODS, LocalizationMethod, NavigationMethod, SensorKind } from './navstack';

export interface RosNavPackage { name: string; files: Record<string, string>; packages: string[]; notes: string[] }

const yaml = (obj: any, indent = 0): string => {
  const pad = '  '.repeat(indent);
  if (Array.isArray(obj)) return obj.map((v) => (typeof v === 'object' && v !== null ? `${pad}-\n${yaml(v, indent + 1)}` : `${pad}- ${fmt(v)}`)).join('\n');
  return Object.entries(obj).map(([k, v]) => (typeof v === 'object' && v !== null && !Array.isArray(v) ? `${pad}${k}:\n${yaml(v, indent + 1)}` : Array.isArray(v) ? (v.length && typeof v[0] === 'object' ? `${pad}${k}:\n${yaml(v, indent + 1)}` : `${pad}${k}: [${v.map(fmt).join(', ')}]`) : `${pad}${k}: ${fmt(v)}`)).join('\n');
};
const fmt = (v: any): string => (typeof v === 'string' ? (/^[A-Za-z0-9_./:-]+$/.test(v) && !/^(true|false|null)$/.test(v) ? v : JSON.stringify(v)) : String(v));
const M = 0.001;

export function generateRosNavPackage(robot: MobileRobot, cfg: NavStackConfig, opts: { packageName?: string } = {}): RosNavPackage {
  const loc = LOCALIZATION_METHODS.find((m) => m.id === cfg.localization)!;
  const nav = NAVIGATION_METHODS.find((n) => n.id === cfg.navigation)!;
  const name = (opts.packageName ?? `${robot.name}_nav`).toLowerCase().replace(/[^a-z0-9_]+/g, '_');
  const ns = robot.rosNamespace || '';
  const k = robot.kin;
  const files: Record<string, string> = {};
  const packages = new Set<string>(['navigation2', 'nav2_bringup', 'robot_localization', 'tf2_ros', 'ros2_control']);
  const notes: string[] = [];
  const holonomic = k.drive === 'omni';
  const ackermann = k.drive === 'ackermann' || k.minTurnRadius > 0;
  const rMin = ackermann ? (k.minTurnRadius || k.wheelBase / Math.tan((k.maxSteer * Math.PI) / 180)) * M : 0;
  const footprint = `[[${(k.footprint[0] / 2 * M).toFixed(3)}, ${(k.footprint[1] / 2 * M).toFixed(3)}], [${(k.footprint[0] / 2 * M).toFixed(3)}, ${(-k.footprint[1] / 2 * M).toFixed(3)}], [${(-k.footprint[0] / 2 * M).toFixed(3)}, ${(-k.footprint[1] / 2 * M).toFixed(3)}], [${(-k.footprint[0] / 2 * M).toFixed(3)}, ${(k.footprint[1] / 2 * M).toFixed(3)}]]`;
  const has = (s: SensorKind) => cfg.sensors.includes(s) || cfg.fusion.includes(s);
  const obstacleSources = [has('lidar2d') ? 'scan' : '', has('lidar3d') ? 'points' : '', has('rgbd_camera') || has('stereo_camera') ? 'depth' : ''].filter(Boolean);
  const globalFrame = loc.global || loc.family === 'lio' || loc.family === 'lidar3d_slam' ? 'map' : 'odom';

  // ---------------- Nav2 -----------------------------------------------------------------------
  const controllerPlugin = nav.controller.split('/').pop()?.split('::').pop() ?? 'FollowPath';
  const controller: any = nav.controller.includes('RegulatedPurePursuit') ? {
    plugin: nav.controller, desired_linear_vel: +(k.maxSpeed * M).toFixed(2), lookahead_dist: +(Math.max(0.6, k.wheelBase * 1.2 * M)).toFixed(2), min_lookahead_dist: 0.4, max_lookahead_dist: +(Math.max(1.5, k.wheelBase * 3 * M)).toFixed(2), lookahead_time: 1.5, use_velocity_scaled_lookahead_dist: true, rotate_to_heading_angular_vel: +(k.maxYawRate * Math.PI / 180).toFixed(2), transform_tolerance: 0.2, use_regulated_linear_velocity_scaling: true, use_cost_regulated_linear_velocity_scaling: true, regulated_linear_scaling_min_radius: +(Math.max(0.9, rMin)).toFixed(2), regulated_linear_scaling_min_speed: 0.25, use_rotate_to_heading: !ackermann, allow_reversing: ackermann, max_robot_pose_search_dist: 10.0, ...(ackermann ? { min_turning_radius: +rMin.toFixed(2) } : {}),
  } : nav.controller.includes('MPPI') ? {
    plugin: nav.controller, time_steps: 56, model_dt: 0.05, batch_size: 2000, vx_std: 0.2, vy_std: holonomic ? 0.2 : 0.0, wz_std: 0.4, vx_max: +(k.maxSpeed * M).toFixed(2), vx_min: +(-k.maxReverseSpeed * M).toFixed(2), vy_max: holonomic ? +(k.maxSpeed * M).toFixed(2) : 0.0, wz_max: +(k.maxYawRate * Math.PI / 180).toFixed(2), motion_model: holonomic ? 'Omni' : ackermann ? 'Ackermann' : 'DiffDrive', ...(ackermann ? { AckermannConstraints: { min_turning_r: +rMin.toFixed(2) } } : {}), critics: ['ConstraintCritic', 'ObstaclesCritic', 'GoalCritic', 'GoalAngleCritic', 'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic', 'PreferForwardCritic'],
  } : {
    plugin: nav.controller, min_vel_x: 0.0, max_vel_x: +(k.maxSpeed * M).toFixed(2), max_vel_theta: +(k.maxYawRate * Math.PI / 180).toFixed(2), acc_lim_x: +(k.maxAccel * M).toFixed(2), acc_lim_theta: 3.2, vx_samples: 20, vtheta_samples: 20, sim_time: 1.7, critics: ['RotateToGoal', 'Oscillation', 'BaseObstacle', 'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist'],
  };
  const planner: any = nav.planner.includes('SmacPlannerHybrid') ? { plugin: nav.planner, tolerance: 0.25, downsample_costmap: false, allow_unknown: true, max_iterations: 1000000, max_planning_time: 5.0, motion_model_for_search: ackermann ? 'REEDS_SHEPP' : 'DUBIN', angle_quantization_bins: 72, minimum_turning_radius: +(Math.max(0.4, rMin)).toFixed(2), reverse_penalty: ackermann ? 2.0 : 100.0, change_penalty: 0.0, non_straight_penalty: 1.2, cost_penalty: 2.0, smooth_path: true }
    : nav.planner.includes('SmacPlanner2D') ? { plugin: nav.planner, tolerance: 0.125, downsample_costmap: false, allow_unknown: true, max_iterations: 1000000, max_planning_time: 2.0 }
    : { plugin: 'nav2_navfn_planner/NavfnPlanner', tolerance: 0.5, use_astar: false, allow_unknown: true };
  const costmapCommon = (rolling: boolean) => ({ update_frequency: rolling ? 5.0 : 1.0, publish_frequency: rolling ? 2.0 : 1.0, global_frame: rolling ? 'odom' : globalFrame, robot_base_frame: 'base_link', footprint, footprint_padding: 0.05, resolution: 0.05, ...(rolling ? { rolling_window: true, width: 8, height: 8 } : {}), plugins: [rolling ? 'obstacle_layer' : 'static_layer', ...(rolling ? [] : ['obstacle_layer']), 'inflation_layer'],
    static_layer: { plugin: 'nav2_costmap_2d::StaticLayer', map_subscribe_transient_local: true },
    obstacle_layer: { plugin: 'nav2_costmap_2d::ObstacleLayer', enabled: true, observation_sources: obstacleSources.join(' ') || 'scan',
      ...(obstacleSources.includes('scan') || !obstacleSources.length ? { scan: { topic: `${ns}/scan`, max_obstacle_height: 2.0, clearing: true, marking: true, data_type: 'LaserScan', raytrace_max_range: 12.0, obstacle_max_range: 10.0 } } : {}),
      ...(obstacleSources.includes('points') ? { points: { topic: `${ns}/points`, max_obstacle_height: 2.0, min_obstacle_height: 0.15, clearing: true, marking: true, data_type: 'PointCloud2', raytrace_max_range: 20.0, obstacle_max_range: 15.0 } } : {}),
      ...(obstacleSources.includes('depth') ? { depth: { topic: `${ns}/camera/depth/points`, max_obstacle_height: 1.8, min_obstacle_height: 0.1, clearing: true, marking: true, data_type: 'PointCloud2', raytrace_max_range: 5.0, obstacle_max_range: 4.0 } } : {}) },
    inflation_layer: { plugin: 'nav2_costmap_2d::InflationLayer', cost_scaling_factor: 3.0, inflation_radius: +(Math.max(0.55, k.footprint[1] * 0.75 * M)).toFixed(2) } });
  const nav2 = {
    bt_navigator: { ros__parameters: { global_frame: globalFrame, robot_base_frame: 'base_link', odom_topic: `${ns}/odom`, bt_loop_duration: 10, default_server_timeout: 20 } },
    controller_server: { ros__parameters: { controller_frequency: 20.0, min_x_velocity_threshold: 0.001, min_theta_velocity_threshold: 0.001, progress_checker_plugin: 'progress_checker', goal_checker_plugins: ['general_goal_checker'], controller_plugins: ['FollowPath'], progress_checker: { plugin: 'nav2_controller::SimpleProgressChecker', required_movement_radius: 0.5, movement_time_allowance: 10.0 }, general_goal_checker: { plugin: 'nav2_controller::SimpleGoalChecker', xy_goal_tolerance: 0.15, yaw_goal_tolerance: 0.15 }, FollowPath: controller } },
    planner_server: { ros__parameters: { expected_planner_frequency: 1.0, planner_plugins: ['GridBased'], GridBased: planner } },
    local_costmap: { local_costmap: { ros__parameters: costmapCommon(true) } },
    global_costmap: { global_costmap: { ros__parameters: costmapCommon(false) } },
    smoother_server: { ros__parameters: { smoother_plugins: ['simple_smoother'], simple_smoother: { plugin: 'nav2_smoother::SimpleSmoother', tolerance: 1.0e-10, max_its: 1000 } } },
    behavior_server: { ros__parameters: { behavior_plugins: ['spin', 'backup', 'drive_on_heading', 'wait'], spin: { plugin: 'nav2_behaviors/Spin' }, backup: { plugin: 'nav2_behaviors/BackUp' }, drive_on_heading: { plugin: 'nav2_behaviors/DriveOnHeading' }, wait: { plugin: 'nav2_behaviors/Wait' }, global_frame: 'odom', robot_base_frame: 'base_link', max_rotational_vel: +(k.maxYawRate * Math.PI / 180).toFixed(2), min_rotational_vel: 0.2, rotational_acc_lim: 1.0 } },
    velocity_smoother: { ros__parameters: { smoothing_frequency: 20.0, feedback: 'OPEN_LOOP', max_velocity: [+(k.maxSpeed * M).toFixed(2), holonomic ? +(k.maxSpeed * M).toFixed(2) : 0.0, +(k.maxYawRate * Math.PI / 180).toFixed(2)], min_velocity: [+(-k.maxReverseSpeed * M).toFixed(2), holonomic ? +(-k.maxSpeed * M).toFixed(2) : 0.0, +(-k.maxYawRate * Math.PI / 180).toFixed(2)], max_accel: [+(k.maxAccel * M).toFixed(2), holonomic ? +(k.maxAccel * M).toFixed(2) : 0.0, 3.2], max_decel: [-(+(k.maxAccel * M).toFixed(2)), holonomic ? -(+(k.maxAccel * M).toFixed(2)) : 0.0, -3.2] } },
    ...(nav.id === 'gnss_waypoints' || nav.id === 'row_following' ? { waypoint_follower: { ros__parameters: { loop_rate: 20, stop_on_failure: false, waypoint_task_executor_plugin: 'wait_at_waypoint', wait_at_waypoint: { plugin: 'nav2_waypoint_follower::WaitAtWaypoint', enabled: true, waypoint_pause_duration: 200 } } } } : {}),
  };
  files[`config/nav2_params.yaml`] = `# Nav2 parameters generated by VerticalBot Studio for ${robot.name}\n# platform: ${cfg.platform}, environment: ${cfg.environment}, drive: ${k.drive}, stack: ${loc.name} + ${nav.name}\n${yaml(nav2)}\n`;

  // ---------------- localization / SLAM ---------------------------------------------------------
  const ekfInputs: string[] = [];
  const ekf: any = { frequency: 30.0, two_d_mode: true, publish_tf: true, map_frame: 'map', odom_frame: 'odom', base_link_frame: 'base_link', world_frame: 'odom' };
  let i = 0;
  if (has('wheel_odom')) { ekf[`odom${i}`] = `${ns}/wheel/odom`; ekf[`odom${i}_config`] = [false, false, false, false, false, false, true, true, false, false, false, true, false, false, false]; ekf[`odom${i}_differential`] = false; ekfInputs.push('wheel odometry (vx, vy, vyaw)'); i++; }
  if (has('imu')) { ekf.imu0 = `${ns}/imu/data`; ekf.imu0_config = [false, false, false, false, false, true, false, false, false, false, false, true, true, false, false]; ekf.imu0_differential = false; ekf.imu0_remove_gravitational_acceleration = true; ekfInputs.push('IMU (yaw, yaw rate, ax)'); }
  const slamFiles: Record<string, string> = {};
  switch (loc.id) {
    case 'slam_toolbox_2d':
      slamFiles['config/slam_toolbox.yaml'] = yaml({ slam_toolbox: { ros__parameters: { solver_plugin: 'solver_plugins::CeresSolver', odom_frame: 'odom', map_frame: 'map', base_frame: 'base_link', scan_topic: `${ns}/scan`, mode: 'mapping', use_scan_matching: true, minimum_travel_distance: 0.3, minimum_travel_heading: 0.3, resolution: 0.05, max_laser_range: 20.0, do_loop_closing: true, loop_search_maximum_distance: 5.0, transform_publish_period: 0.02 } } });
      slamFiles['config/amcl.yaml'] = yaml({ amcl: { ros__parameters: { base_frame_id: 'base_link', odom_frame_id: 'odom', global_frame_id: 'map', scan_topic: `${ns}/scan`, robot_model_type: holonomic ? 'nav2_amcl::OmniMotionModel' : 'nav2_amcl::DifferentialMotionModel', max_particles: 3000, min_particles: 500, update_min_d: 0.2, update_min_a: 0.2, laser_max_range: 20.0, set_initial_pose: true } } });
      packages.add('slam-toolbox'); packages.add('nav2-amcl');
      notes.push('Map once with slam_toolbox (mode: mapping), save with `ros2 run nav2_map_server map_saver_cli`, then localize with AMCL (or slam_toolbox localization mode).');
      break;
    case 'cartographer_2d':
      slamFiles['config/cartographer_2d.lua'] = `include "map_builder.lua"\ninclude "trajectory_builder.lua"\noptions = {\n  map_builder = MAP_BUILDER, trajectory_builder = TRAJECTORY_BUILDER,\n  map_frame = "map", tracking_frame = "${has('imu') ? 'imu_link' : 'base_link'}", published_frame = "odom", odom_frame = "odom",\n  provide_odom_frame = false, publish_frame_projected_to_2d = true, use_odometry = ${has('wheel_odom')}, use_nav_sat = false, use_landmarks = false,\n  num_laser_scans = 1, num_multi_echo_laser_scans = 0, num_subdivisions_per_laser_scan = 1, num_point_clouds = 0,\n  lookup_transform_timeout_sec = 0.2, submap_publish_period_sec = 0.3, pose_publish_period_sec = 5e-3, trajectory_publish_period_sec = 30e-3,\n  rangefinder_sampling_ratio = 1., odometry_sampling_ratio = 1., fixed_frame_pose_sampling_ratio = 1., imu_sampling_ratio = 1., landmarks_sampling_ratio = 1.,\n}\nMAP_BUILDER.use_trajectory_builder_2d = true\nTRAJECTORY_BUILDER_2D.use_imu_data = ${has('imu')}\nTRAJECTORY_BUILDER_2D.min_range = 0.2\nTRAJECTORY_BUILDER_2D.max_range = 20.\nTRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true\nPOSE_GRAPH.optimize_every_n_nodes = 35\nreturn options\n`;
      packages.add('cartographer-ros'); break;
    case 'lio_sam':
      slamFiles['config/lio_sam.yaml'] = yaml({ lio_sam: { ros__parameters: { pointCloudTopic: `${ns}/points`, imuTopic: `${ns}/imu/data`, odomTopic: 'odometry/imu', gpsTopic: has('gnss_rtk') || has('gnss') ? `${ns}/odometry/gps` : 'odometry/gps', lidarFrame: 'lidar_link', baselinkFrame: 'base_link', odometryFrame: 'odom', mapFrame: 'map', useImuHeadingInitialization: has('gnss_rtk'), useGpsElevation: false, gpsCovThreshold: 2.0, poseCovThreshold: 25.0, sensor: 'velodyne', N_SCAN: 16, Horizon_SCAN: 1800, downsampleRate: 1, lidarMinRange: 1.0, lidarMaxRange: 100.0, imuAccNoise: 3.9939570888238808e-03, imuGyrNoise: 1.5636343949698187e-03, imuAccBiasN: 6.4356659353532566e-05, imuGyrBiasN: 3.5640318696367613e-05, imuGravity: 9.80511, mappingCornerLeafSize: 0.2, mappingSurfLeafSize: 0.4, surroundingkeyframeAddingDistThreshold: 1.0, surroundingkeyframeAddingAngleThreshold: 0.2, loopClosureEnableFlag: true, loopClosureFrequency: 1.0, historyKeyframeSearchRadius: 15.0 } } });
      packages.add('lio_sam (build from source)'); break;
    case 'fast_lio2':
      slamFiles['config/fast_lio.yaml'] = yaml({ common: { lid_topic: `${ns}/points`, imu_topic: `${ns}/imu/data`, time_sync_en: false }, preprocess: { lidar_type: 1, scan_line: 16, blind: 0.5 }, mapping: { acc_cov: 0.1, gyr_cov: 0.1, b_acc_cov: 0.0001, b_gyr_cov: 0.0001, fov_degree: 180, det_range: 100.0, extrinsic_est_en: true, extrinsic_T: [0, 0, 0], extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1] }, publish: { path_en: true, scan_publish_en: true, dense_publish_en: false }, pcd_save: { pcd_save_en: false } });
      packages.add('fast_lio (build from source)'); notes.push('FAST-LIO2 gives odometry only: add hdl_localization or KISS-ICP + a saved map for global localization, or fuse RTK.'); break;
    case 'kiss_icp':
      slamFiles['config/kiss_icp.yaml'] = yaml({ odometry_node: { ros__parameters: { topic: `${ns}/points`, odom_frame: 'odom', base_frame: 'base_link', max_range: 100.0, min_range: 1.0, deskew: true, voxel_size: 1.0, initial_threshold: 2.0, min_motion_th: 0.1 } } });
      packages.add('kiss-icp'); break;
    case 'rtabmap_rgbd':
      slamFiles['config/rtabmap.yaml'] = yaml({ rtabmap: { ros__parameters: { frame_id: 'base_link', subscribe_depth: has('rgbd_camera'), subscribe_stereo: !has('rgbd_camera'), subscribe_scan: has('lidar2d'), approx_sync: true, 'Rtabmap/DetectionRate': '1', 'RGBD/NeighborLinkRefining': 'true', 'RGBD/ProximityBySpace': 'true', 'Reg/Strategy': has('lidar2d') ? '1' : '0', 'Grid/RangeMax': '5.0', 'Grid/FromDepth': String(has('rgbd_camera')), 'Mem/IncrementalMemory': 'true' } } });
      packages.add('rtabmap-ros'); break;
    case 'orb_slam3_mono': case 'orb_slam3_vi':
      slamFiles['config/orb_slam3.yaml'] = `%YAML:1.0\n---\nFile.version: "1.0"\nCamera.type: "PinHole"\nCamera1.fx: 458.654\nCamera1.fy: 457.296\nCamera1.cx: 367.215\nCamera1.cy: 248.375\nCamera1.k1: -0.28340811\nCamera1.k2: 0.07395907\nCamera1.p1: 0.00019359\nCamera1.p2: 1.76187114e-05\nCamera.width: 752\nCamera.height: 480\nCamera.fps: 20\nCamera.RGB: 1\n${loc.id === 'orb_slam3_vi' ? 'IMU.NoiseGyro: 1.7e-4\nIMU.NoiseAcc: 2.0e-3\nIMU.GyroWalk: 1.9393e-05\nIMU.AccWalk: 3.0000e-03\nIMU.Frequency: 200.0\nIMU.T_b_c1: !!opencv-matrix\n  rows: 4\n  cols: 4\n  dt: f\n  data: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]\n' : ''}ORBextractor.nFeatures: 1500\nORBextractor.scaleFactor: 1.2\nORBextractor.nLevels: 8\nORBextractor.iniThFAST: 20\nORBextractor.minThFAST: 7\nViewer.on: 0\n# Replace the intrinsics with your calibration (camera_calibration / kalibr).\n`;
      packages.add('orb_slam3_ros2 (build from source)'); notes.push('Calibrate the camera (and camera-IMU extrinsics with kalibr) before use; monocular scale is not metric without IMU.'); break;
    case 'openvins':
      slamFiles['config/openvins_estimator.yaml'] = yaml({ use_fej: true, use_imuavg: true, integration: 'rk4', use_stereo: has('stereo_camera'), max_cameras: has('stereo_camera') ? 2 : 1, calib_cam_extrinsics: true, calib_cam_intrinsics: true, calib_cam_timeoffset: true, max_clones: 11, max_slam: 50, max_msckf: 40, dt_slam_delay: 2, init_window_time: 2.0, init_imu_thresh: 1.5, num_pts: 200, fast_threshold: 20, grid_x: 5, grid_y: 5, min_px_dist: 10, knn_ratio: 0.7, relative_config_imu: 'kalibr_imu_chain.yaml', relative_config_imucam: 'kalibr_imucam_chain.yaml' });
      packages.add('open_vins (build from source)'); break;
    case 'vins_fusion':
      slamFiles['config/vins_fusion.yaml'] = `%YAML:1.0\nimu: 1\nnum_of_cam: ${has('stereo_camera') ? 2 : 1}\nimu_topic: "${ns}/imu/data"\nimage0_topic: "${ns}/camera/left/image_raw"\n${has('stereo_camera') ? `image1_topic: "${ns}/camera/right/image_raw"\n` : ''}output_path: "/tmp/vins"\ncam0_calib: "cam0.yaml"\n${has('stereo_camera') ? 'cam1_calib: "cam1.yaml"\n' : ''}image_width: 752\nimage_height: 480\nestimate_extrinsic: 1\nmultiple_thread: 1\nmax_cnt: 150\nmin_dist: 30\nfreq: 10\nF_threshold: 1.0\nshow_track: 0\nflow_back: 1\nmax_solver_time: 0.04\nmax_num_iterations: 8\nkeyframe_parallax: 10.0\nacc_n: 0.1\ngyr_n: 0.01\nacc_w: 0.001\ngyr_w: 0.0001\ng_norm: 9.805\nestimate_td: 1\ntd: 0.0\nload_previous_pose_graph: 0\npose_graph_save_path: "/tmp/vins/pose_graph/"\nsave_image: 0\n`;
      packages.add('VINS-Fusion (build from source)'); if (has('gnss') || has('gnss_rtk')) notes.push('Run the VINS-Fusion global_fusion node to fuse GPS into the pose graph.'); break;
    case 'gnss_rtk': case 'gnss_ins': case 'hybrid_rtk_lio': case 'hybrid_rtk_vio':
      break;
    case 'uwb': case 'hybrid_uwb_lidar':
      slamFiles['config/uwb_anchors.yaml'] = yaml({ uwb_localization: { ros__parameters: { anchors: [{ id: 1, x: 0.0, y: 0.0, z: 2.5 }, { id: 2, x: 20.0, y: 0.0, z: 2.5 }, { id: 3, x: 20.0, y: 30.0, z: 2.5 }, { id: 4, x: 0.0, y: 30.0, z: 2.5 }], tag_topic: `${ns}/uwb/ranges`, frame_id: 'map', min_anchors: 3 } } });
      if (loc.id === 'hybrid_uwb_lidar') slamFiles['config/slam_toolbox.yaml'] = yaml({ slam_toolbox: { ros__parameters: { odom_frame: 'odom', map_frame: 'map', base_frame: 'base_link', scan_topic: `${ns}/scan`, mode: 'localization', use_scan_matching: true, resolution: 0.05, max_laser_range: 20.0 } } });
      notes.push('Survey the UWB anchors (x, y, z in the map frame) and fill config/uwb_anchors.yaml.'); break;
    case 'magnetic_tape': case 'qr_grid': case 'laser_reflectors': case 'rail_encoder':
      notes.push(`${loc.name}: localization is provided by the vehicle controller / infrastructure; Nav2 is configured for lane following with obstacle stop only.`); break;
  }
  Object.assign(files, slamFiles);
  // GNSS
  if (loc.requires.includes('gnss_rtk') || loc.requires.includes('gnss')) {
    files['config/navsat_transform.yaml'] = yaml({ navsat_transform: { ros__parameters: { frequency: 30.0, delay: 3.0, magnetic_declination_radians: 0.0, yaw_offset: 0.0, zero_altitude: true, broadcast_cartesian_transform: true, publish_filtered_gps: true, use_odometry_yaw: !has('imu'), wait_for_datum: false } } });
    if (loc.requires.includes('gnss_rtk')) { files['config/ntrip.yaml'] = yaml({ ntrip_client: { ros__parameters: { host: 'rtk.example.com', port: 2101, mountpoint: 'RTCM3_MSM', username: 'user', password: 'pass', authenticate: true, ssl: false, rtcm_topic: `${ns}/rtcm` } } }); packages.add('ntrip-client'); }
    ekf.odom1 = `${ns}/odometry/gps`; ekf.odom1_config = [true, true, false, false, false, false, false, false, false, false, false, false, false, false, false]; ekfInputs.push('GNSS (x, y from navsat_transform)');
    const ekfMap = { ...ekf, world_frame: 'map' };
    files['config/ekf_map.yaml'] = yaml({ ekf_filter_node_map: { ros__parameters: ekfMap } });
    packages.add('robot-localization');
    notes.push('Dual EKF: ekf_odom (continuous, wheel+IMU) and ekf_map (with GNSS) as in the robot_localization navsat example; RTK corrections via ntrip_client.');
  }
  files['config/ekf_odom.yaml'] = yaml({ ekf_filter_node_odom: { ros__parameters: ekf } });

  // ---------------- launch + docs ---------------------------------------------------------------
  const slamLaunch = loc.id === 'slam_toolbox_2d' ? `Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', parameters=[cfg('slam_toolbox.yaml')], output='screen')`
    : loc.id === 'cartographer_2d' ? `Node(package='cartographer_ros', executable='cartographer_node', arguments=['-configuration_directory', os.path.join(pkg, 'config'), '-configuration_basename', 'cartographer_2d.lua'], output='screen'),\n        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node', parameters=[{'resolution': 0.05}])`
    : loc.id === 'lio_sam' ? `IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('lio_sam'), 'launch', 'run.launch.py')), launch_arguments={'params_file': cfg('lio_sam.yaml')}.items())`
    : loc.id === 'fast_lio2' ? `Node(package='fast_lio', executable='fastlio_mapping', parameters=[cfg('fast_lio.yaml')], output='screen')`
    : loc.id === 'kiss_icp' ? `Node(package='kiss_icp', executable='odometry_node', parameters=[cfg('kiss_icp.yaml')], output='screen')`
    : loc.id === 'rtabmap_rgbd' ? `Node(package='rtabmap_slam', executable='rtabmap', parameters=[cfg('rtabmap.yaml')], output='screen')`
    : loc.id.startsWith('orb_slam3') ? `Node(package='orb_slam3_ros2', executable='${loc.id === 'orb_slam3_vi' ? 'mono-inertial' : 'mono'}', arguments=[vocab, cfg('orb_slam3.yaml')], output='screen')`
    : loc.id === 'openvins' ? `Node(package='ov_msckf', executable='run_subscribe_msckf', parameters=[{'config_path': cfg('openvins_estimator.yaml')}], output='screen')`
    : loc.id === 'vins_fusion' ? `Node(package='vins', executable='vins_node', arguments=[cfg('vins_fusion.yaml')], output='screen')`
    : loc.id.includes('uwb') ? `Node(package='uwb_localization', executable='uwb_localization_node', parameters=[cfg('uwb_anchors.yaml')], output='screen')`
    : `# ${loc.name}: localization from the vehicle controller / infrastructure`;
  files['launch/bringup.launch.py'] = `"""Bringup for ${robot.name}: ${loc.name} + ${nav.name}. Generated by VerticalBot Studio."""\nimport os\nfrom ament_index_python.packages import get_package_share_directory\nfrom launch import LaunchDescription\nfrom launch.actions import IncludeLaunchDescription\nfrom launch.launch_description_sources import PythonLaunchDescriptionSource\nfrom launch_ros.actions import Node\n\n\ndef generate_launch_description():\n    pkg = get_package_share_directory('${name}')\n    cfg = lambda f: os.path.join(pkg, 'config', f)\n    vocab = os.path.expanduser('~/ORB_SLAM3/Vocabulary/ORBvoc.txt')\n    nodes = [\n        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node_odom', parameters=[cfg('ekf_odom.yaml')], remappings=[('odometry/filtered', 'odometry/local')]),\n${files['config/ekf_map.yaml'] ? "        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node_map', parameters=[cfg('ekf_map.yaml')], remappings=[('odometry/filtered', 'odometry/global')]),\n        Node(package='robot_localization', executable='navsat_transform_node', parameters=[cfg('navsat_transform.yaml')], remappings=[('imu', '${ns}/imu/data'), ('gps/fix', '${ns}/gps/fix'), ('odometry/filtered', 'odometry/global')]),\n" : ''}${files['config/ntrip.yaml'] ? "        Node(package='ntrip_client', executable='ntrip_ros.py', parameters=[cfg('ntrip.yaml')]),\n" : ''}        ${slamLaunch},\n        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')), launch_arguments={'params_file': cfg('nav2_params.yaml'), 'use_sim_time': 'false'}.items()),\n    ]\n    return LaunchDescription(nodes)\n`;
  files['package.xml'] = `<?xml version="1.0"?>\n<package format="3">\n  <name>${name}</name>\n  <version>0.1.0</version>\n  <description>Navigation stack for ${robot.name}: ${loc.name} + ${nav.name} (generated by VerticalBot Studio)</description>\n  <maintainer email="user@example.com">VerticalBot Studio</maintainer>\n  <license>Apache-2.0</license>\n  <buildtool_depend>ament_cmake</buildtool_depend>\n  <exec_depend>navigation2</exec_depend>\n  <exec_depend>nav2_bringup</exec_depend>\n  <exec_depend>robot_localization</exec_depend>\n  <export><build_type>ament_cmake</build_type></export>\n</package>\n`;
  files['CMakeLists.txt'] = `cmake_minimum_required(VERSION 3.8)\nproject(${name})\nfind_package(ament_cmake REQUIRED)\ninstall(DIRECTORY config launch DESTINATION share/\${PROJECT_NAME})\nament_package()\n`;
  files['README.md'] = `# ${name}\n\nNavigation & localization stack for **${robot.name}** generated by VerticalBot Studio.\n\n| | |\n|---|---|\n| Platform | ${cfg.platform} (${k.drive}, ${(k.footprint[0] * M).toFixed(2)} × ${(k.footprint[1] * M).toFixed(2)} m, ${(k.maxSpeed * M).toFixed(2)} m/s) |\n| Environment | ${cfg.environment} |\n| Localization | ${loc.name} — ${loc.notes} |\n| Navigation | ${nav.name} — ${nav.notes} |\n| Sensors | ${cfg.sensors.join(', ')} |\n| EKF inputs | ${ekfInputs.join('; ') || 'none'} |\n\n## Install\n\n\`\`\`bash\nsudo apt install ${[...packages].filter((p) => !p.includes('source')).map((p) => `ros-$ROS_DISTRO-${p.replace(/_/g, '-')}`).join(' ')}\n# from source: ${[...packages].filter((p) => p.includes('source')).join(', ') || '—'}\ncolcon build --packages-select ${name}\nros2 launch ${name} bringup.launch.py\n\`\`\`\n\n## Frames and topics\n\n\`map → odom → base_link\`; sensors on \`${ns || ''}/scan\`, \`${ns || ''}/points\`, \`${ns || ''}/imu/data\`, \`${ns || ''}/wheel/odom\`, \`${ns || ''}/gps/fix\` — remap to your drivers.\n\n## Notes\n\n${notes.map((n) => `- ${n}`).join('\n') || '- none'}\n- Software behind the choice: ${loc.software.join(', ')}; ${nav.software.join(', ')}.\n- Tune costmap inflation, controller lookahead and speeds on the vehicle; values here follow the studio model.\n`;
  return { name, files, packages: [...packages], notes };
}

export function rosNavPackageZip(pkg: RosNavPackage): Uint8Array {
  const entries: Record<string, Uint8Array> = {};
  for (const [p, c] of Object.entries(pkg.files)) entries[`${pkg.name}/${p}`] = strToU8(c);
  return zipSync(entries, { level: 6 });
}

export type { LocalizationMethod, NavigationMethod };
