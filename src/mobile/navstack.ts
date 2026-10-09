/**
 * Navigation & SLAM stack selection and simulation for mobile platforms.
 *
 * A "stack" is the combination of (1) how the vehicle localises and maps — 2D/3D LiDAR SLAM, visual SLAM,
 * visual-inertial or LiDAR-inertial odometry, GNSS-RTK/INS, UWB, magnetic tape / QR grid / reflectors for
 * AGVs, and fusion of several of them — and (2) how it navigates — free navigation (Nav2 planners and
 * controllers), crop-row following, line following, GNSS waypoints. The module provides:
 *
 *  - a catalogue of localization and navigation methods with sensor requirements, environment suitability,
 *    accuracy/drift figures and the real ROS 2 packages that implement them;
 *  - a recommender: platform + environment + available sensors + constraints → ranked stacks with reasons;
 *  - error models and a `LocalizationEstimator` that produces the *estimated* pose of a simulated robot from
 *    its true pose (drift, noise, GNSS outages under canopy, loop closures, scale drift, localization loss), so
 *    fleets can be debugged against realistic localization behaviour before hardware exists;
 *  - a 2D LiDAR ray-cast on occupancy maps and an incremental SLAM map (explored/occupied cells).
 *
 * ROS 2 configuration export (Nav2, slam_toolbox, Cartographer, RTAB-Map, ORB-SLAM3, OpenVINS, LIO-SAM,
 * FAST-LIO, robot_localization EKF, NTRIP…) lives in `navstack_ros.ts`.
 */
import type { MobileRobot } from './items';
import type { MapItem, ZoneItem } from './items';
import { navSummary } from '../ros/publishers';

// ---------------------------------------------------------------------------------------------
// Catalogue
// ---------------------------------------------------------------------------------------------

export type PlatformType = 'agv_line' | 'agv_natural' | 'amr' | 'tractor' | 'tracked' | 'legged' | 'rail';
export type Environment = 'warehouse' | 'factory' | 'greenhouse' | 'orchard' | 'open_field' | 'vineyard' | 'forest' | 'urban' | 'underground' | 'mixed';
export type SensorKind = 'wheel_odom' | 'imu' | 'lidar2d' | 'lidar3d' | 'mono_camera' | 'stereo_camera' | 'rgbd_camera' | 'gnss' | 'gnss_rtk' | 'uwb' | 'magnetic_tape' | 'qr_grid' | 'reflectors' | 'radar' | 'encoder_rail';

export type LocalizationFamily = 'lidar2d_slam' | 'lidar3d_slam' | 'lio' | 'mono_vslam' | 'stereo_vslam' | 'rgbd_slam' | 'vio' | 'gnss_rtk' | 'gnss_ins' | 'uwb' | 'tape' | 'qr' | 'reflector' | 'rail_encoder' | 'hybrid';

export interface LocalizationMethod {
  id: string;
  name: string;
  family: LocalizationFamily;
  /** Sensors that must be present. */
  requires: SensorKind[];
  /** Any one of these groups satisfies the requirement (alternatives). */
  requiresAny?: SensorKind[][];
  /** Sensors that improve the result (fused). */
  benefitsFrom: SensorKind[];
  /** 0..1 suitability per environment. */
  suitability: Partial<Record<Environment, number>>;
  /** Typical position error (m, 1σ) when the method works nominally. */
  accuracy: number;
  /** Drift per metre travelled without corrections (m/m); 0 for globally referenced methods. */
  driftPerMeter: number;
  /** Heading drift (deg per metre). */
  headingDriftPerMeter: number;
  /** Needs a prebuilt map / infrastructure (tape, markers, beacons, reflectors, survey). */
  infrastructure: 'none' | 'map' | 'markers' | 'beacons' | 'tape' | 'reflectors' | 'base_station';
  /** Provides a global, repeatable frame (vs. relative odometry). */
  global: boolean;
  compute: 'low' | 'mid' | 'high';
  /** Relative hardware cost 1..5. */
  cost: number;
  /** Behaviour in the dark / at night. */
  worksAtNight: boolean;
  /** Real software: ROS 2 packages / products. */
  software: string[];
  notes: string;
}

export interface NavigationMethod {
  id: string;
  name: string;
  platforms: PlatformType[];
  environments: Environment[];
  /** Nav2 planner / controller plugins (or non-Nav2 behaviour). */
  planner: string;
  controller: string;
  requiresGlobalLocalization: boolean;
  needsCostmap: boolean;
  software: string[];
  notes: string;
}

export const LOCALIZATION_METHODS: LocalizationMethod[] = [
  { id: 'slam_toolbox_2d', name: '2D LiDAR SLAM (slam_toolbox)', family: 'lidar2d_slam', requires: ['lidar2d'], benefitsFrom: ['wheel_odom', 'imu'], suitability: { warehouse: 1, factory: 1, greenhouse: 0.9, underground: 0.8, urban: 0.5, orchard: 0.35, vineyard: 0.4, open_field: 0.05, forest: 0.3, mixed: 0.5 }, accuracy: 0.05, driftPerMeter: 0, headingDriftPerMeter: 0, infrastructure: 'none', global: true, compute: 'low', cost: 2, worksAtNight: true, software: ['slam_toolbox', 'nav2_amcl (localization on the saved map)'], notes: 'Structured, mostly planar environments with walls; degrades in open fields and in orchards where trunks are thin and foliage changes.' },
  { id: 'cartographer_2d', name: '2D LiDAR SLAM (Cartographer)', family: 'lidar2d_slam', requires: ['lidar2d'], benefitsFrom: ['wheel_odom', 'imu'], suitability: { warehouse: 1, factory: 1, greenhouse: 0.9, underground: 0.8, urban: 0.5, orchard: 0.35, vineyard: 0.4, open_field: 0.05, forest: 0.3, mixed: 0.5 }, accuracy: 0.05, driftPerMeter: 0, headingDriftPerMeter: 0, infrastructure: 'none', global: true, compute: 'mid', cost: 2, worksAtNight: true, software: ['cartographer_ros'], notes: 'Submap-based, good loop closure; heavier than slam_toolbox.' },
  { id: 'lio_sam', name: '3D LiDAR-inertial SLAM (LIO-SAM)', family: 'lio', requires: ['lidar3d', 'imu'], benefitsFrom: ['gnss_rtk', 'gnss', 'wheel_odom'], suitability: { orchard: 0.95, vineyard: 0.9, forest: 0.9, urban: 0.9, factory: 0.9, warehouse: 0.85, greenhouse: 0.8, underground: 0.9, open_field: 0.5, mixed: 0.9 }, accuracy: 0.1, driftPerMeter: 0.002, headingDriftPerMeter: 0.01, infrastructure: 'none', global: false, compute: 'high', cost: 4, worksAtNight: true, software: ['lio_sam', 'nav2 + 3D→2D costmap projection'], notes: 'Tightly coupled LiDAR + IMU with loop closure and optional GPS factors — the workhorse for orchards and forests.' },
  { id: 'fast_lio2', name: '3D LiDAR-inertial odometry (FAST-LIO2)', family: 'lio', requires: ['lidar3d', 'imu'], benefitsFrom: ['gnss_rtk', 'wheel_odom'], suitability: { orchard: 0.9, vineyard: 0.9, forest: 0.9, urban: 0.9, factory: 0.9, warehouse: 0.85, greenhouse: 0.8, underground: 0.9, open_field: 0.5, mixed: 0.9 }, accuracy: 0.1, driftPerMeter: 0.003, headingDriftPerMeter: 0.015, infrastructure: 'none', global: false, compute: 'mid', cost: 4, worksAtNight: true, software: ['fast_lio', 'hdl_localization / KISS-ICP for map-based localization'], notes: 'Very fast and robust odometry (also solid-state LiDARs); loop closure through separate packages.' },
  { id: 'kiss_icp', name: '3D LiDAR odometry (KISS-ICP)', family: 'lidar3d_slam', requires: ['lidar3d'], benefitsFrom: ['imu', 'wheel_odom'], suitability: { orchard: 0.75, vineyard: 0.75, forest: 0.7, urban: 0.85, factory: 0.85, warehouse: 0.8, greenhouse: 0.7, underground: 0.85, open_field: 0.4, mixed: 0.8 }, accuracy: 0.15, driftPerMeter: 0.005, headingDriftPerMeter: 0.02, infrastructure: 'none', global: false, compute: 'mid', cost: 4, worksAtNight: true, software: ['kiss_icp'], notes: 'Sensor-agnostic ICP odometry without IMU; pair with a global reference for long missions.' },
  { id: 'rtabmap_rgbd', name: 'RGB-D / stereo SLAM (RTAB-Map)', family: 'rgbd_slam', requires: [], requiresAny: [['rgbd_camera'], ['stereo_camera']], benefitsFrom: ['imu', 'wheel_odom', 'lidar2d'], suitability: { greenhouse: 0.85, warehouse: 0.8, factory: 0.8, urban: 0.6, orchard: 0.55, vineyard: 0.6, forest: 0.4, open_field: 0.2, underground: 0.5, mixed: 0.6 }, accuracy: 0.1, driftPerMeter: 0.01, headingDriftPerMeter: 0.05, infrastructure: 'none', global: true, compute: 'high', cost: 2, worksAtNight: false, software: ['rtabmap_ros'], notes: 'Appearance-based loop closure with dense maps; sensitive to lighting and dynamic foliage; limited depth range outdoors.' },
  { id: 'orb_slam3_mono', name: 'Monocular visual SLAM (ORB-SLAM3)', family: 'mono_vslam', requires: ['mono_camera'], benefitsFrom: ['imu'], suitability: { warehouse: 0.5, factory: 0.5, greenhouse: 0.5, urban: 0.5, orchard: 0.35, vineyard: 0.4, forest: 0.3, open_field: 0.15, underground: 0.3, mixed: 0.4 }, accuracy: 0.3, driftPerMeter: 0.02, headingDriftPerMeter: 0.05, infrastructure: 'none', global: false, compute: 'high', cost: 1, worksAtNight: false, software: ['orb_slam3_ros2'], notes: 'Cheapest sensor; metric scale is unobservable without IMU (scale drift), fails in low texture, low light and fast rotations.' },
  { id: 'orb_slam3_vi', name: 'Visual-inertial SLAM (ORB-SLAM3 mono/stereo + IMU)', family: 'vio', requires: ['imu'], requiresAny: [['mono_camera'], ['stereo_camera']], benefitsFrom: ['wheel_odom', 'gnss_rtk'], suitability: { warehouse: 0.7, factory: 0.7, greenhouse: 0.7, urban: 0.7, orchard: 0.55, vineyard: 0.6, forest: 0.45, open_field: 0.3, underground: 0.5, mixed: 0.6 }, accuracy: 0.15, driftPerMeter: 0.01, headingDriftPerMeter: 0.03, infrastructure: 'none', global: false, compute: 'high', cost: 2, worksAtNight: false, software: ['orb_slam3_ros2'], notes: 'IMU makes scale observable and bridges short feature losses; still lighting dependent.' },
  { id: 'openvins', name: 'Visual-inertial odometry (OpenVINS)', family: 'vio', requires: ['imu'], requiresAny: [['mono_camera'], ['stereo_camera']], benefitsFrom: ['wheel_odom', 'gnss_rtk'], suitability: { warehouse: 0.7, factory: 0.7, greenhouse: 0.7, urban: 0.75, orchard: 0.6, vineyard: 0.65, forest: 0.5, open_field: 0.35, underground: 0.5, mixed: 0.65 }, accuracy: 0.2, driftPerMeter: 0.01, headingDriftPerMeter: 0.03, infrastructure: 'none', global: false, compute: 'mid', cost: 2, worksAtNight: false, software: ['open_vins'], notes: 'Filter-based VIO, light on compute; no loop closure — an odometry source for fusion, not a map.' },
  { id: 'vins_fusion', name: 'VINS-Fusion (stereo/mono + IMU, optional GPS)', family: 'vio', requires: ['imu'], requiresAny: [['stereo_camera'], ['mono_camera']], benefitsFrom: ['gnss', 'gnss_rtk', 'wheel_odom'], suitability: { warehouse: 0.7, factory: 0.7, greenhouse: 0.7, urban: 0.8, orchard: 0.6, vineyard: 0.65, forest: 0.5, open_field: 0.4, underground: 0.5, mixed: 0.7 }, accuracy: 0.15, driftPerMeter: 0.008, headingDriftPerMeter: 0.03, infrastructure: 'none', global: false, compute: 'high', cost: 2, worksAtNight: false, software: ['VINS-Fusion'], notes: 'Optimisation-based VIO with loop closure and global GPS fusion.' },
  { id: 'gnss_rtk', name: 'GNSS RTK (+ heading from dual antenna / IMU)', family: 'gnss_rtk', requires: ['gnss_rtk'], benefitsFrom: ['imu', 'wheel_odom'], suitability: { open_field: 1, vineyard: 0.7, orchard: 0.35, urban: 0.5, forest: 0.15, greenhouse: 0.05, warehouse: 0, factory: 0, underground: 0, mixed: 0.5 }, accuracy: 0.02, driftPerMeter: 0, headingDriftPerMeter: 0, infrastructure: 'base_station', global: true, compute: 'low', cost: 3, worksAtNight: true, software: ['ublox_dgnss / septentrio_gnss_driver', 'ntrip_client', 'robot_localization navsat_transform'], notes: 'Centimetre accuracy with sky view; multipath and outages under canopy, near buildings and indoors.' },
  { id: 'gnss_ins', name: 'GNSS/INS dead reckoning (standard GNSS + IMU + odometry)', family: 'gnss_ins', requires: ['gnss', 'imu'], benefitsFrom: ['wheel_odom'], suitability: { open_field: 0.8, vineyard: 0.5, orchard: 0.25, urban: 0.4, forest: 0.1, greenhouse: 0, warehouse: 0, factory: 0, underground: 0, mixed: 0.4 }, accuracy: 1.0, driftPerMeter: 0.01, headingDriftPerMeter: 0.02, infrastructure: 'none', global: true, compute: 'low', cost: 1, worksAtNight: true, software: ['robot_localization (ekf + navsat_transform)'], notes: 'Metre-level; fine for field coverage with wide tolerances, not for row entries.' },
  { id: 'uwb', name: 'UWB beacon positioning', family: 'uwb', requires: ['uwb'], benefitsFrom: ['imu', 'wheel_odom'], suitability: { greenhouse: 0.9, warehouse: 0.8, factory: 0.8, orchard: 0.3, vineyard: 0.3, open_field: 0.2, urban: 0.2, forest: 0.1, underground: 0.6, mixed: 0.4 }, accuracy: 0.2, driftPerMeter: 0, headingDriftPerMeter: 0, infrastructure: 'beacons', global: true, compute: 'low', cost: 2, worksAtNight: true, software: ['uwb_localization / pozyx / decawave drivers', 'robot_localization ekf'], notes: 'Anchors on the greenhouse structure give a repeatable global frame independent of lighting and plant growth.' },
  { id: 'magnetic_tape', name: 'Magnetic / optical tape following', family: 'tape', requires: ['magnetic_tape'], benefitsFrom: ['wheel_odom', 'qr_grid'], suitability: { warehouse: 0.9, factory: 0.9, greenhouse: 0.6, orchard: 0, vineyard: 0, open_field: 0, urban: 0, forest: 0, underground: 0.5, mixed: 0.3 }, accuracy: 0.01, driftPerMeter: 0, headingDriftPerMeter: 0, infrastructure: 'tape', global: false, compute: 'low', cost: 1, worksAtNight: true, software: ['vendor AGV controller', 'ros2 line follower node'], notes: 'Classic AGV: fixed routes, centimetre lateral accuracy, position along the route from RFID/QR tags or odometry.' },
  { id: 'qr_grid', name: 'QR / fiducial floor grid', family: 'qr', requires: ['qr_grid'], benefitsFrom: ['wheel_odom', 'imu'], suitability: { warehouse: 1, factory: 0.9, greenhouse: 0.5, orchard: 0, vineyard: 0, open_field: 0, urban: 0, forest: 0, underground: 0.4, mixed: 0.3 }, accuracy: 0.01, driftPerMeter: 0.005, headingDriftPerMeter: 0.02, infrastructure: 'markers', global: true, compute: 'low', cost: 1, worksAtNight: true, software: ['apriltag_ros / fiducial_slam', 'vendor grid controllers'], notes: 'Goods-to-person warehouses: absolute fix at every tag, odometry between tags.' },
  { id: 'laser_reflectors', name: 'Laser triangulation with reflectors', family: 'reflector', requires: ['lidar2d', 'reflectors'], benefitsFrom: ['wheel_odom'], suitability: { warehouse: 1, factory: 1, greenhouse: 0.6, orchard: 0.1, vineyard: 0.1, open_field: 0, urban: 0.2, forest: 0, underground: 0.6, mixed: 0.4 }, accuracy: 0.01, driftPerMeter: 0, headingDriftPerMeter: 0, infrastructure: 'reflectors', global: true, compute: 'low', cost: 3, worksAtNight: true, software: ['SICK NAV / vendor', 'reflector_localization'], notes: 'Industrial AGV standard: millimetre repeatability with surveyed reflectors.' },
  { id: 'rail_encoder', name: 'Rail / pipe-rail encoder', family: 'rail_encoder', requires: ['encoder_rail'], benefitsFrom: ['qr_grid'], suitability: { greenhouse: 1, warehouse: 0.3, factory: 0.3, orchard: 0, vineyard: 0, open_field: 0, urban: 0, forest: 0, underground: 0.2, mixed: 0.2 }, accuracy: 0.005, driftPerMeter: 0.001, headingDriftPerMeter: 0, infrastructure: 'tape', global: false, compute: 'low', cost: 1, worksAtNight: true, software: ['ros2_control encoder', 'greenhouse trolley controller'], notes: '1-D localization along greenhouse pipe rails; row identity from tags or the concrete path.' },
  { id: 'hybrid_rtk_lio', name: 'Hybrid: RTK-GNSS + LiDAR-inertial odometry', family: 'hybrid', requires: ['gnss_rtk', 'lidar3d', 'imu'], benefitsFrom: ['wheel_odom'], suitability: { orchard: 1, vineyard: 1, forest: 0.85, open_field: 1, urban: 0.9, mixed: 1, greenhouse: 0.6, warehouse: 0.6, factory: 0.6, underground: 0.5 }, accuracy: 0.03, driftPerMeter: 0.002, headingDriftPerMeter: 0.01, infrastructure: 'base_station', global: true, compute: 'high', cost: 5, worksAtNight: true, software: ['lio_sam (GPS factor)', 'robot_localization dual EKF', 'ntrip_client'], notes: 'RTK on headlands, LiDAR-inertial odometry under the canopy; the loop closes at the next fix. Reference stack for orchard fleets.' },
  { id: 'hybrid_rtk_vio', name: 'Hybrid: RTK-GNSS + visual-inertial odometry', family: 'hybrid', requires: ['gnss_rtk', 'imu'], requiresAny: [['stereo_camera'], ['mono_camera']], benefitsFrom: ['wheel_odom'], suitability: { orchard: 0.8, vineyard: 0.85, forest: 0.6, open_field: 0.95, urban: 0.8, mixed: 0.85, greenhouse: 0.6, warehouse: 0.5, factory: 0.5, underground: 0.3 }, accuracy: 0.03, driftPerMeter: 0.008, headingDriftPerMeter: 0.03, infrastructure: 'base_station', global: true, compute: 'high', cost: 3, worksAtNight: false, software: ['VINS-Fusion (GPS)', 'robot_localization dual EKF'], notes: 'Lower cost than LiDAR; daytime only unless illuminated.' },
  { id: 'hybrid_uwb_lidar', name: 'Hybrid: UWB + 2D LiDAR SLAM', family: 'hybrid', requires: ['uwb', 'lidar2d'], benefitsFrom: ['imu', 'wheel_odom'], suitability: { greenhouse: 1, warehouse: 0.9, factory: 0.9, orchard: 0.3, vineyard: 0.3, open_field: 0.2, urban: 0.3, forest: 0.1, underground: 0.7, mixed: 0.5 }, accuracy: 0.05, driftPerMeter: 0, headingDriftPerMeter: 0, infrastructure: 'beacons', global: true, compute: 'mid', cost: 3, worksAtNight: true, software: ['slam_toolbox', 'uwb driver', 'robot_localization ekf'], notes: 'Greenhouses: UWB for the global frame between crop rows, LiDAR for local structure and obstacle avoidance.' },
  { id: 'wheel_imu_dr', name: 'Wheel odometry + IMU dead reckoning', family: 'gnss_ins', requires: ['wheel_odom', 'imu'], benefitsFrom: [], suitability: { warehouse: 0.2, factory: 0.2, greenhouse: 0.3, orchard: 0.1, vineyard: 0.1, open_field: 0.1, urban: 0.1, forest: 0.05, underground: 0.2, mixed: 0.1 }, accuracy: 0.5, driftPerMeter: 0.02, headingDriftPerMeter: 0.05, infrastructure: 'none', global: false, compute: 'low', cost: 0, worksAtNight: true, software: ['robot_localization ekf'], notes: 'Baseline only: drift and wheel slip make it unusable alone beyond a few tens of metres.' },
];

export const NAVIGATION_METHODS: NavigationMethod[] = [
  { id: 'nav2_smac_hybrid_rpp', name: 'Nav2: Smac Hybrid-A* + Regulated Pure Pursuit', platforms: ['tractor', 'tracked', 'amr', 'agv_natural'], environments: ['orchard', 'vineyard', 'open_field', 'forest', 'urban', 'mixed', 'factory', 'warehouse'], planner: 'nav2_smac_planner/SmacPlannerHybrid', controller: 'nav2_regulated_pure_pursuit_controller/RegulatedPurePursuitController', requiresGlobalLocalization: true, needsCostmap: true, software: ['navigation2'], notes: 'Kinematically feasible paths for Ackermann / large footprints; smooth tracking with speed regulation on curves and near obstacles.' },
  { id: 'nav2_navfn_dwb', name: 'Nav2: NavFn + DWB', platforms: ['amr', 'agv_natural'], environments: ['warehouse', 'factory', 'greenhouse', 'urban', 'mixed'], planner: 'nav2_navfn_planner/NavfnPlanner', controller: 'dwb_core::DWBLocalPlanner', requiresGlobalLocalization: true, needsCostmap: true, software: ['navigation2'], notes: 'Default differential-drive stack; good in cluttered indoor spaces.' },
  { id: 'nav2_smac2d_mppi', name: 'Nav2: Smac 2D + MPPI', platforms: ['amr', 'agv_natural', 'tracked'], environments: ['warehouse', 'factory', 'greenhouse', 'urban', 'mixed', 'orchard'], planner: 'nav2_smac_planner/SmacPlanner2D', controller: 'nav2_mppi_controller::MPPIController', requiresGlobalLocalization: true, needsCostmap: true, software: ['navigation2'], notes: 'Predictive controller handling dynamic obstacles; needs a capable CPU.' },
  { id: 'row_following', name: 'Crop-row following (LiDAR / camera) + headland turns', platforms: ['tractor', 'tracked', 'amr'], environments: ['orchard', 'vineyard', 'open_field', 'greenhouse'], planner: 'row_traversal (studio) / nav2 waypoint follower on headlands', controller: 'row_follower (lidar centreline or crop-row detection)', requiresGlobalLocalization: false, needsCostmap: false, software: ['studio row traversal', 'nav2_waypoint_follower', 'crop row detection (OpenCV / PCL)'], notes: 'In the row the vehicle centres itself between trunks/canopy and does not need global accuracy; global localization is needed only for row entry and headland turns.' },
  { id: 'line_following', name: 'Fixed-route line following (tape / QR grid)', platforms: ['agv_line'], environments: ['warehouse', 'factory', 'greenhouse'], planner: 'route graph (fixed lanes)', controller: 'line follower', requiresGlobalLocalization: false, needsCostmap: false, software: ['vendor AGV controller', 'VDA 5050 order graph'], notes: 'Deterministic lanes; obstacle stop by safety scanner; fleet orders as node/edge graphs (VDA 5050).' },
  { id: 'gnss_waypoints', name: 'GNSS waypoint following (coverage paths)', platforms: ['tractor', 'tracked', 'amr'], environments: ['open_field', 'vineyard', 'orchard'], planner: 'coverage planner (studio) / nav2 waypoint follower', controller: 'nav2_regulated_pure_pursuit_controller', requiresGlobalLocalization: true, needsCostmap: false, software: ['nav2_waypoint_follower', 'opennav_coverage'], notes: 'Field coverage with RTK; add a local obstacle layer from LiDAR or radar.' },
  { id: 'rail_1d', name: 'Rail trolley (1-D position control)', platforms: ['rail'], environments: ['greenhouse'], planner: 'row schedule', controller: 'ros2_control position controller', requiresGlobalLocalization: false, needsCostmap: false, software: ['ros2_control'], notes: 'Pipe-rail trolleys: position along the rail from the encoder, row selection at the concrete path.' },
];

/** Sensors usually found on each platform preset (used when the robot has no explicit sensor list). */
export const PLATFORM_DEFAULT_SENSORS: Record<PlatformType, SensorKind[]> = {
  agv_line: ['wheel_odom', 'magnetic_tape'],
  agv_natural: ['wheel_odom', 'lidar2d', 'reflectors'],
  amr: ['wheel_odom', 'imu', 'lidar2d', 'rgbd_camera'],
  tractor: ['wheel_odom', 'imu', 'gnss_rtk', 'lidar3d'],
  tracked: ['wheel_odom', 'imu', 'lidar3d', 'stereo_camera', 'gnss_rtk'],
  legged: ['imu', 'stereo_camera', 'lidar3d'],
  rail: ['encoder_rail'],
};

export const SENSOR_LABELS: Record<SensorKind, string> = {
  wheel_odom: 'Wheel odometry', imu: 'IMU', lidar2d: '2D LiDAR', lidar3d: '3D LiDAR', mono_camera: 'Mono camera', stereo_camera: 'Stereo camera', rgbd_camera: 'RGB-D camera', gnss: 'GNSS (standard)', gnss_rtk: 'GNSS RTK', uwb: 'UWB tag', magnetic_tape: 'Magnetic/optical tape sensor', qr_grid: 'Floor QR / fiducial reader', reflectors: 'Reflector survey', radar: 'Radar', encoder_rail: 'Rail encoder',
};

// ---------------------------------------------------------------------------------------------
// Recommender
// ---------------------------------------------------------------------------------------------

export interface StackRequest {
  platform: PlatformType;
  environment: Environment;
  /** Sensors available on the vehicle (defaults from the platform preset). */
  sensors?: SensorKind[];
  /** Share of the mission area with GNSS sky view (0..1); orchards under canopy are ~0.2. */
  gnssAvailability?: number;
  /** Night / low-light operation required. */
  night?: boolean;
  /** Global repeatable frame required (row entries, fleet coordination, docking). */
  needsGlobal?: boolean;
  /** Maximum hardware cost 1..5. */
  maxCost?: number;
  /** Compute budget on the vehicle. */
  compute?: 'low' | 'mid' | 'high';
  /** Infrastructure the site can provide. */
  infrastructureAllowed?: LocalizationMethod['infrastructure'][];
  /** Required accuracy (m). */
  accuracy?: number;
}

export interface StackRecommendation {
  localization: LocalizationMethod;
  navigation: NavigationMethod;
  /** Sensors fused in addition to the required ones. */
  fusion: SensorKind[];
  /** Sensors the vehicle would need to add. */
  missingSensors: SensorKind[];
  score: number;
  reasons: string[];
  warnings: string[];
}

const computeRank = { low: 0, mid: 1, high: 2 };

function hasSensors(available: Set<SensorKind>, m: LocalizationMethod): { ok: boolean; missing: SensorKind[] } {
  const missing = m.requires.filter((s) => !available.has(s));
  if (m.requiresAny) for (const group of m.requiresAny) if (!group.some((s) => available.has(s))) missing.push(group[0]);
  return { ok: missing.length === 0, missing };
}

/** Rank localization + navigation combinations for a request. Methods needing missing sensors are kept but penalised (so the user sees what to add). */
export function recommendStacks(req: StackRequest): StackRecommendation[] {
  const available = new Set<SensorKind>(req.sensors ?? PLATFORM_DEFAULT_SENSORS[req.platform]);
  const gnss = req.gnssAvailability ?? ({ open_field: 0.95, vineyard: 0.7, orchard: 0.25, urban: 0.5, forest: 0.15, greenhouse: 0.05, warehouse: 0, factory: 0, underground: 0, mixed: 0.5 } as Record<Environment, number>)[req.environment];
  const out: StackRecommendation[] = [];
  for (const loc of LOCALIZATION_METHODS) {
    const reasons: string[] = [], warnings: string[] = [];
    let score = (loc.suitability[req.environment] ?? 0) * 100;
    if (score <= 0) continue;
    reasons.push(`${Math.round(loc.suitability[req.environment]! * 100)}% suited to ${req.environment}`);
    const sens = hasSensors(available, loc);
    if (!sens.ok) { score -= 35; warnings.push(`needs ${sens.missing.map((s) => SENSOR_LABELS[s]).join(', ')}`); }
    // GNSS dependence
    if (loc.family === 'gnss_rtk' || loc.family === 'gnss_ins') { score *= 0.3 + 0.7 * gnss; if (gnss < 0.5) warnings.push(`GNSS available only ${Math.round(gnss * 100)}% of the time — outages without inertial/LiDAR fallback`); }
    if (loc.family === 'hybrid' && loc.requires.includes('gnss_rtk')) { score += 10 * (1 - Math.abs(gnss - 0.5) * 2) ; reasons.push('bridges GNSS outages with onboard odometry'); }
    if (req.night && !loc.worksAtNight) { score -= 40; warnings.push('camera-based: not usable at night without illumination'); }
    if (req.needsGlobal && !loc.global) { score -= 25; warnings.push('relative odometry only — drifts without a global reference'); }
    if (req.maxCost !== undefined && loc.cost > req.maxCost) { score -= 15 * (loc.cost - req.maxCost); warnings.push(`hardware cost ${loc.cost}/5 above budget ${req.maxCost}/5`); }
    if (req.compute && computeRank[loc.compute] > computeRank[req.compute]) { score -= 20; warnings.push(`needs ${loc.compute} compute`); }
    if (req.infrastructureAllowed && loc.infrastructure !== 'none' && !req.infrastructureAllowed.includes(loc.infrastructure)) { score -= 30; warnings.push(`requires ${loc.infrastructure.replace('_', ' ')} infrastructure`); }
    if (req.accuracy !== undefined && loc.accuracy > req.accuracy) { score -= 20; warnings.push(`typical error ${loc.accuracy} m above the required ${req.accuracy} m`); }
    const fusion = loc.benefitsFrom.filter((s) => available.has(s));
    if (fusion.length) { score += 4 * fusion.length; reasons.push(`fuses ${fusion.map((s) => SENSOR_LABELS[s]).join(', ')}`); }
    if (loc.driftPerMeter === 0 && loc.global) reasons.push('drift-free global positioning');
    // navigation choice per platform/environment
    const navs = NAVIGATION_METHODS.filter((n) => n.platforms.includes(req.platform) && n.environments.includes(req.environment));
    const nav = navs.find((n) => !n.requiresGlobalLocalization || loc.global || loc.family === 'lio' || loc.family === 'lidar3d_slam') ?? navs[0] ?? NAVIGATION_METHODS[0];
    if (nav.requiresGlobalLocalization && !loc.global && !(loc.family === 'lio' || loc.family === 'lidar3d_slam')) warnings.push(`${nav.name} expects a global frame; use ${loc.name} only with row/line following`);
    out.push({ localization: loc, navigation: nav, fusion, missingSensors: sens.missing, score: Math.round(score), reasons, warnings });
  }
  return out.sort((a, b) => b.score - a.score);
}

// ---------------------------------------------------------------------------------------------
// Robot configuration
// ---------------------------------------------------------------------------------------------

export interface NavStackConfig {
  platform: PlatformType;
  environment: Environment;
  sensors: SensorKind[];
  localization: string;
  navigation: string;
  fusion: SensorKind[];
  /** Simulate localization errors and use the estimate for control (digital-twin debugging). */
  simulate: boolean;
  /** GNSS availability override (0..1) when no gnss_denied zones exist. */
  gnssAvailability?: number;
  /** Loop-closure interval in metres for SLAM methods (drift reset). */
  loopClosureEvery?: number;
  /** Optional Nav2 tuning stored with the robot. */
  params?: Record<string, number | string | boolean>;
}

export function defaultNavStack(platform: PlatformType = 'amr', environment: Environment = 'warehouse'): NavStackConfig {
  const rec = recommendStacks({ platform, environment })[0];
  return { platform, environment, sensors: [...PLATFORM_DEFAULT_SENSORS[platform]], localization: rec.localization.id, navigation: rec.navigation.id, fusion: rec.fusion, simulate: false };
}

export function getNavStack(robot: MobileRobot): NavStackConfig | null {
  const c = robot.params.navStack as unknown as NavStackConfig | undefined;
  return c && c.localization ? c : null;
}

export function setNavStack(robot: MobileRobot, cfg: NavStackConfig): void {
  robot.setParam('navStack', cfg as any);
}

export function platformForRobot(robot: MobileRobot): PlatformType {
  const d = robot.kin.drive;
  if (d === 'ackermann') return 'tractor';
  if (d === 'tracked') return 'tracked';
  if (d === 'legged') return 'legged';
  return 'amr';
}

// ---------------------------------------------------------------------------------------------
// Sensor simulation: 2D LiDAR and incremental SLAM map
// ---------------------------------------------------------------------------------------------

export interface LidarScan { angles: number[]; ranges: number[]; maxRange: number; hits: Array<[number, number] | null> }

/** Cast rays on an occupancy map from (x, y, theta deg). Returns ranges in mm (maxRange when nothing hit). */
export function simulateLidar2D(map: MapItem, x: number, y: number, thetaDeg: number, opts: { beams?: number; fov?: number; maxRange?: number; noise?: number } = {}): LidarScan {
  const beams = opts.beams ?? 180, fov = opts.fov ?? 270, maxRange = opts.maxRange ?? 20000, noise = opts.noise ?? 15;
  const step = map.resolution * 0.5;
  const angles: number[] = [], ranges: number[] = [], hits: Array<[number, number] | null> = [];
  for (let i = 0; i < beams; i++) {
    const a = (thetaDeg + (-fov / 2 + (fov * i) / Math.max(1, beams - 1))) * Math.PI / 180;
    const cx = Math.cos(a), sy = Math.sin(a);
    let r = 0, hit: [number, number] | null = null;
    while (r < maxRange) {
      r += step;
      const px = x + cx * r, py = y + sy * r;
      const [gx, gy] = map.worldToCell(px, py);
      const v = map.get(gx, gy);
      if (v >= 50) { hit = [px, py]; break; }
    }
    const measured = hit ? Math.min(maxRange, r + (Math.random() - 0.5) * 2 * noise) : maxRange;
    angles.push(a); ranges.push(measured); hits.push(hit);
  }
  return { angles, ranges, maxRange, hits };
}

/** Occupancy grid built from scans: 0 free, 100 occupied, 255 unknown (same conventions as MapItem). */
export class SlamMap {
  cells: Uint8Array;
  constructor(public width: number, public height: number, public resolution: number, public originX: number, public originY: number) {
    this.cells = new Uint8Array(width * height).fill(255);
  }
  static like(map: MapItem): SlamMap { return new SlamMap(map.width, map.height, map.resolution, map.originX, map.originY); }
  idx(x: number, y: number): number { const cx = Math.floor((x - this.originX) / this.resolution), cy = Math.floor((y - this.originY) / this.resolution); return cx < 0 || cy < 0 || cx >= this.width || cy >= this.height ? -1 : cy * this.width + cx; }
  /** Integrate a scan taken from the *estimated* pose (so localization error shows up as map distortion). */
  integrate(scan: LidarScan, x: number, y: number, thetaDeg: number, trueTheta: number): void {
    const dTheta = (thetaDeg - trueTheta) * Math.PI / 180;
    for (let i = 0; i < scan.angles.length; i++) {
      const a = scan.angles[i] + dTheta;
      const r = scan.ranges[i];
      const n = Math.max(1, Math.floor(r / this.resolution));
      for (let k = 1; k < n; k++) { const j = this.idx(x + Math.cos(a) * k * this.resolution, y + Math.sin(a) * k * this.resolution); if (j >= 0 && this.cells[j] !== 100) this.cells[j] = 0; }
      if (scan.hits[i]) { const j = this.idx(x + Math.cos(a) * r, y + Math.sin(a) * r); if (j >= 0) this.cells[j] = 100; }
    }
  }
  coverage(): number { let k = 0; for (const c of this.cells) if (c !== 255) k++; return k / this.cells.length; }
  /** Fraction of true occupied cells that the SLAM map also marks occupied (map quality proxy). */
  agreement(truth: MapItem): number {
    let occ = 0, ok = 0;
    for (let cy = 0; cy < Math.min(this.height, truth.height); cy++) for (let cx = 0; cx < Math.min(this.width, truth.width); cx++) {
      if (truth.get(cx, cy) >= 50) { occ++; const c = this.cells[cy * this.width + cx]; if (c === 100 || [-1, 1, -this.width, this.width].some((d) => this.cells[cy * this.width + cx + d] === 100)) ok++; }
    }
    return occ ? ok / occ : 0;
  }
}

// ---------------------------------------------------------------------------------------------
// Localization error model & estimator
// ---------------------------------------------------------------------------------------------

export interface EstimatorState {
  x: number; y: number; theta: number;
  /** Accumulated drift vector (mm, deg) since the last global correction. */
  driftX: number; driftY: number; driftTheta: number;
  distanceSinceFix: number;
  /** Scale factor for monocular SLAM (drifts around 1). */
  scale: number;
  lost: boolean;
  lostEvents: number;
  fixes: number;
  gnssAvailable: boolean;
  /** Error statistics (mm). */
  error: number; rmse: number; maxError: number; samples: number;
  covariance: number;
}

/** Deterministic pseudo-random (seeded) so replays are reproducible. */
function mulberry32(seed: number) { let a = seed >>> 0; return () => { a = (a + 0x6d2b79f5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }
function gauss(rnd: () => number): number { const u = 1 - rnd(), v = rnd(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }

export class LocalizationEstimator {
  state: EstimatorState;
  method: LocalizationMethod;
  private rnd: () => number;
  private lastTrue: { x: number; y: number; theta: number } | null = null;
  private sumSq = 0;
  constructor(public config: NavStackConfig, seed = 1) {
    this.method = LOCALIZATION_METHODS.find((m) => m.id === config.localization) ?? LOCALIZATION_METHODS[0];
    this.rnd = mulberry32(seed);
    this.state = { x: 0, y: 0, theta: 0, driftX: 0, driftY: 0, driftTheta: 0, distanceSinceFix: 0, scale: 1, lost: false, lostEvents: 0, fixes: 0, gnssAvailable: true, error: 0, rmse: 0, maxError: 0, samples: 0, covariance: 0 };
  }

  reset(x: number, y: number, theta: number): void {
    this.state = { ...this.state, x, y, theta, driftX: 0, driftY: 0, driftTheta: 0, distanceSinceFix: 0, scale: 1, lost: false, error: 0 };
    this.lastTrue = { x, y, theta };
  }

  /** Effective drift and noise for the current context (mm/mm, deg/mm, mm). */
  private model(ctx: { gnss: boolean; featureRich: boolean }): { drift: number; hdrift: number; noise: number; global: boolean; hnoise: number } {
    const m = this.method;
    const fusionGain = 1 / (1 + 0.5 * this.config.fusion.length);
    let drift = m.driftPerMeter * fusionGain, hdrift = m.headingDriftPerMeter * fusionGain, noise = m.accuracy * 1000, global = m.global;
    if (m.family === 'gnss_rtk' || m.family === 'gnss_ins' || (m.family === 'hybrid' && m.requires.includes('gnss_rtk'))) {
      if (!ctx.gnss) {
        // outage: fall back to the onboard odometry of the stack
        global = false;
        drift = m.family === 'hybrid' ? (m.requires.includes('lidar3d') ? 0.002 : 0.008) : (this.config.fusion.includes('wheel_odom') ? 0.015 : 0.03);
        hdrift = m.family === 'hybrid' ? 0.01 : 0.05;
        noise = 50;
      }
    }
    if ((m.family === 'mono_vslam' || m.family === 'vio' || m.family === 'rgbd_slam' || m.family === 'stereo_vslam') && !ctx.featureRich) { drift *= 4; hdrift *= 4; }
    if (m.family === 'lidar2d_slam' && !ctx.featureRich) { noise *= 3; drift = 0.01; global = false; }
    return { drift, hdrift, noise, global, hnoise: m.global ? 0.3 : 0.05 };
  }

  /**
   * Advance the estimate. `truePose` is the simulated ground truth; `ctx.gnss` tells whether the sky is visible
   * here (false under canopy / indoors), `ctx.featureRich` whether the surroundings give the sensors structure.
   */
  update(truePose: { x: number; y: number; theta: number }, dt: number, ctx: { gnss: boolean; featureRich: boolean; velocity?: number }): EstimatorState {
    const s = this.state;
    if (!this.lastTrue) { this.reset(truePose.x, truePose.y, truePose.theta); return s; }
    const dx = truePose.x - this.lastTrue.x, dy = truePose.y - this.lastTrue.y;
    const dist = Math.hypot(dx, dy);
    const dTheta = ((truePose.theta - this.lastTrue.theta + 540) % 360) - 180;
    this.lastTrue = { ...truePose };
    const m = this.model(ctx);
    s.gnssAvailable = ctx.gnss;
    // monocular scale drift
    if (this.method.family === 'mono_vslam') s.scale = Math.max(0.7, Math.min(1.3, s.scale + gauss(this.rnd) * 0.002 * dist / 1000));
    // dead-reckon the increment with drift
    const heading = (s.theta) * Math.PI / 180;
    const scale = this.method.family === 'mono_vslam' ? s.scale : 1;
    const driftStep = m.drift * dist * (1 + 0.3 * gauss(this.rnd));
    const hStep = m.hdrift * dist / 1000 * (1 + 0.3 * gauss(this.rnd)) * (this.rnd() < 0.5 ? -1 : 1);
    s.driftTheta += hStep;
    s.driftX += driftStep * Math.cos(heading + Math.PI / 2 * (this.rnd() < 0.5 ? 1 : -1));
    s.driftY += driftStep * Math.sin(heading + Math.PI / 2 * (this.rnd() < 0.5 ? 1 : -1));
    s.distanceSinceFix += dist;
    // localization loss: feature-poor context with camera/lidar2d methods, or long outage
    const lossRisk = !ctx.featureRich && (this.method.family === 'mono_vslam' || this.method.family === 'vio' || this.method.family === 'rgbd_slam') ? 0.02 * dt : !ctx.gnss && this.method.family === 'gnss_rtk' && s.distanceSinceFix > 30000 ? 0.05 * dt : 0;
    if (!s.lost && this.rnd() < lossRisk) { s.lost = true; s.lostEvents++; }
    if (s.lost && (ctx.featureRich || (ctx.gnss && this.method.global)) && this.rnd() < 0.5 * dt) { s.lost = false; }
    // global correction (fix) when available: GNSS fix, map-based SLAM match, markers, beacons
    const canFix = m.global && !s.lost;
    const loop = this.config.loopClosureEvery ?? (this.method.family === 'lio' || this.method.family === 'lidar3d_slam' || this.method.family === 'vio' ? 60000 : 0);
    const loopClosure = loop > 0 && s.distanceSinceFix >= loop && ctx.featureRich;
    if (canFix || loopClosure) {
      const decay = loopClosure ? 0.2 : 0.05 * Math.min(1, dt * 10);
      s.driftX *= 1 - decay; s.driftY *= 1 - decay; s.driftTheta *= 1 - decay;
      if (loopClosure) { s.distanceSinceFix = 0; s.fixes++; }
      else if (dist > 0 || dt > 0) s.fixes += dt > 0 ? 0 : 0;
      if (canFix) s.distanceSinceFix = Math.max(0, s.distanceSinceFix - dist * 2);
    }
    const noise = (s.lost ? 5 : 1) * m.noise;
    s.x = truePose.x + s.driftX + gauss(this.rnd) * noise * scale;
    s.y = truePose.y + s.driftY + gauss(this.rnd) * noise * scale;
    s.theta = ((truePose.theta + s.driftTheta + gauss(this.rnd) * m.hnoise + dTheta * 0 + 540) % 360) - 180;
    if (s.lost) { s.x += gauss(this.rnd) * 500; s.y += gauss(this.rnd) * 500; }
    s.error = Math.hypot(s.x - truePose.x, s.y - truePose.y);
    s.samples++;
    this.sumSq += s.error * s.error;
    s.rmse = Math.sqrt(this.sumSq / s.samples);
    s.maxError = Math.max(s.maxError, s.error);
    s.covariance = noise * noise + Math.hypot(s.driftX, s.driftY) ** 2;
    return s;
  }
}

/** GNSS sky view at a point: false inside `gnss_denied`/canopy zones or when the environment is indoor. */
export function gnssAvailableAt(x: number, y: number, zones: ZoneItem[], cfg: NavStackConfig): boolean {
  if (['warehouse', 'factory', 'greenhouse', 'underground'].includes(cfg.environment)) return false;
  for (const z of zones) if ((z.kind as string) === 'gnss_denied' && z.contains(x, y)) return false;
  if (cfg.gnssAvailability !== undefined && zones.every((z) => (z.kind as string) !== 'gnss_denied')) {
    // no zones drawn: pseudo-random spatial pattern with the configured availability
    const h = Math.abs(Math.sin(x * 0.00037 + y * 0.00051) * 43758.5453) % 1;
    return h < cfg.gnssAvailability;
  }
  return true;
}

/** Feature richness proxy: distance to the nearest obstacle cell in the map (structure for LiDAR/vision). */
export function featureRichAt(map: MapItem | null, x: number, y: number, radius = 8000): boolean {
  if (!map) return true;
  const [cx, cy] = map.worldToCell(x, y);
  const r = Math.ceil(radius / map.resolution);
  for (let dy = -r; dy <= r; dy += 2) for (let dx = -r; dx <= r; dx += 2) if (map.get(cx + dx, cy + dy) >= 50 && map.get(cx + dx, cy + dy) !== 255) return true;
  return false;
}

/** Runtime state attached to a simulated robot (not serialized). */
export interface NavRuntime {
  estimator: LocalizationEstimator;
  slam: SlamMap | null;
  lastScan: LidarScan | null;
  trueTrail: number[][];
  estTrail: number[][];
  scanTimer: number;
}

export function ensureNavRuntime(robot: MobileRobot, map: MapItem | null): NavRuntime | null {
  const cfg = getNavStack(robot);
  if (!cfg || !cfg.simulate) { (robot as any)._nav = undefined; return null; }
  let rt = (robot as any)._nav as NavRuntime | undefined;
  if (!rt || rt.estimator.config !== cfg) {
    const est = new LocalizationEstimator(cfg, robot.id.length);
    est.reset(robot.state.x, robot.state.y, robot.state.theta);
    rt = { estimator: est, slam: map && cfg.sensors.includes('lidar2d') ? SlamMap.like(map) : null, lastScan: null, trueTrail: [], estTrail: [], scanTimer: 0 };
    (robot as any)._nav = rt;
  }
  return rt;
}

/** Step localization simulation for one robot (called by the fleet manager / world clock). */
export function stepNavRuntime(robot: MobileRobot, dt: number, map: MapItem | null, zones: ZoneItem[]): NavRuntime | null {
  const rt = ensureNavRuntime(robot, map);
  if (!rt) return null;
  const cfg = rt.estimator.config;
  const st = robot.state;
  const ctx = { gnss: gnssAvailableAt(st.x, st.y, zones, cfg), featureRich: featureRichAt(map, st.x, st.y), velocity: st.v };
  rt.estimator.update({ x: st.x, y: st.y, theta: st.theta }, dt, ctx);
  rt.scanTimer += dt;
  if (rt.slam && map && rt.scanTimer >= 0.5 && (Math.abs(st.v) > 1 || rt.trueTrail.length === 0)) {
    rt.scanTimer = 0;
    rt.lastScan = simulateLidar2D(map, st.x, st.y, st.theta, { beams: 120, maxRange: 15000 });
    const e = rt.estimator.state;
    rt.slam.integrate(rt.lastScan, e.x, e.y, e.theta, st.theta);
  }
  robot.params.navEstimate = navSummary(robot, rt) as any; // readable through the API (getParam) and webhooks
  if (Math.abs(st.v) > 1) {
    const e = rt.estimator.state;
    if (!rt.trueTrail.length || Math.hypot(st.x - rt.trueTrail[rt.trueTrail.length - 1][0], st.y - rt.trueTrail[rt.trueTrail.length - 1][1]) > 200) { rt.trueTrail.push([st.x, st.y]); rt.estTrail.push([e.x, e.y]); if (rt.trueTrail.length > 3000) { rt.trueTrail.shift(); rt.estTrail.shift(); } }
  }
  return rt;
}
