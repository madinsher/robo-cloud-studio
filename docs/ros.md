# ROS / ROS 2 compatibility

Everything below is checked against the real ROS 2 Jazzy tools (WSL Ubuntu 22.04) — the ground truth lives in
`tests/fixtures/ros_gt/` and is regenerated with `tests/fixtures/ros_gt/tools/gen_gt.sh`.

## xacro (`src/io/urdf/xacro.ts`, `src/io/urdf/pyexpr.ts`)

A port of the ROS 2 `xacro` semantics with a sandboxed Python-expression evaluator (no JavaScript evaluation of
file content): `_eval_literal` typing (int / float / bool / quoted strings), lazy properties, `scope="parent|global"`,
namespaced includes (`ns=`) resolved relative to the including file and through the package map, `xacro.load_yaml`
(PyYAML-compatible reader in `src/ros/yaml.ts`, `!degrees` / `!radians` tags), `$(arg|find|env|optenv|eval|dirname)`,
`^` / `^|default`, `*` / `**` blocks, `xacro:element` / `xacro:attribute`, `$${` escapes, Python builtins / `math`,
`%`-formatting, conditionals that must be boolean, and hard errors (missing include, unknown macro, undefined
property / arg) like ROS.

Tests: `tests/ros_xacro_gt.test.ts` — 29 entry points of 13 real packages (MURKA `murca_description`, `murca_webots`,
`realsense2_description`, `ur_description`, TurtleBot 3 / 4, `webots_ros2_universal_robot`, `robot_state_publisher`,
MoveIt test data, `launch_param_builder`, `moveit_setup_controllers`, palletizer) produce the same element tree with
every attribute string as ROS `xacro`; `tests/ros_xacro_features.test.ts` — feature cases vs ROS (including the cases
ROS rejects).

## URDF → station items (`src/io/urdf/urdf.ts`)

`importURDF` converts the URDF into a kinematic tree (`src/io/kintree.ts`, shared with the Webots importer) and
decomposes it into a mobile base (wheels / steering detected, floating base) plus one robot per serial chain of
actuated joints; mimic joints follow their master, fixed links fold into their parent, every joint keeps its URDF name,
the chain flange is chosen by tip link (SRDF chain tips, `tool0` / `flange` / `tcp`…). urdfdom validation is
reproduced (duplicate links, dangling joints, multiple roots).
Tests: `tests/ros_urdf_fk.test.ts` — the pose of every link of 25+ robots equals urdf_parser_py FK (< 0.1 mm) at q = 0
and q ≠ 0, and no movable joint is lost.

## ROS package / workspace import (`src/io/urdf/package.ts`)

`importRosPackageFiles(files, assets)` takes a dropped folder or zip: packages come from `package.xml`,
`package://` and `$(find)` resolve inside the drop, robot entry points come from launch files (`xacro` commands built
with `os.path.join`, pathlib, `PathJoinSubstitution`, `DeclareLaunchArgument` defaults, ROS 1 `<param command>`), or
from the top-level URDF / xacro files when there is no launch file. Macro-only and included xacros are not robots;
only meshes referenced by the imported robot are registered (under their `package://` URI, STL / OBJ in metres,
COLLADA by its `<unit>`); expansion errors are reported instead of a 0-DOF robot; an SRDF of the same robot adds
planning groups, named poses (`params.namedPoses`) and the disabled collision pairs.
Tests: `tests/ros_package.test.ts`.

## Exports

- **URDF package** (`src/io/urdf/urdf_export.ts`): robots imported from URDF export their kept description — link /
  joint names (TF frames such as `utp_g_01/base_link`), inertials, mimic and off-chain joints, `ros2_control` /
  `gazebo` / `transmission` blocks unchanged, meshes copied into the package. Every package has
  `launch/display.launch.py` (robot_state_publisher + joint_state_publisher) and a basic SRDF (`config/*.srdf`: one
  chain group per robot, `home` and imported named poses, adjacent links disabled). Tests: `tests/ros_exports.test.ts`
  (re-import gives identical link / joint sets and FK; in WSL the packages build and `ros2 launch <pkg> display.launch.py`
  starts robot_state_publisher, check_urdf sees every link).
- **Nav2 / SLAM / EKF package** (`src/mobile/navstack_ros.ts`): ROS-typed YAML (`toRosYaml`: doubles always with a
  decimal point, homogeneous sequences, integer parameters from a table), Jazzy plugin class names, absolute topic names,
  dense robot_localization inputs (GNSS only in the map EKF), hybrid RTK + LIO / VIO stacks launch and fuse their
  odometry, optional / source-built packages are skipped with a log line. Verified with `colcon build` + `ros2 launch`
  in ROS 2 Jazzy: `ROS_WSL=1 npx vitest run tests/ros_exports.test.ts -t WSL` (all lifecycle nodes become active, no
  node dies or rejects its parameters).

## Live interface (`src/ros/twin.ts`, `src/ros/rosbridge.ts`, `server/drivers/ros2.ts`)

rosbridge mode **`sim`** makes the studio the simulator backend of a ROS 2 stack with the topic set of webots_ros2 /
the MURKA robots (`docs/interfaces.md` §1–2 of robot-retailer-murka-v2):

| Topic | Type | |
|---|---|---|
| `/clock` | rosgraph_msgs/Clock | sim time, every step |
| `/joint_states` | sensor_msgs/JointState | every joint by URDF name (wheels with velocity, mimic joints), 20 Hz |
| `/robot_description` | std_msgs/String | latched; the kept URDF, or the station URDF with floating mobile bases |
| `/tf` | tf2_msgs/TFMessage | `odom → <base link>` of each mobile robot, 31 Hz |
| `/tf_static` | tf2_msgs/TFMessage | `map → odom` when `twin.mapFrame` is set |
| `<ns>/odom` | nav_msgs/Odometry | from the start pose, base-frame twist, 31 Hz |
| `<ns>/imu` | sensor_msgs/Imu | `imu_link`, 31 Hz |
| `<ns>/scan` | sensor_msgs/LaserScan | sensor-relative angles, 10 Hz (when the navigation stack simulates a 2D LiDAR) |
| `<ns or /robot>/ground_truth_pose` | geometry_msgs/PoseStamped | map frame |
| `<ns>/cmd_vel` (sub) | geometry_msgs/Twist | drives the base through the drive model; 0.6 s watchdog |
| `/<robot>/joint_trajectory`, `/joint_trajectory_controller/joint_trajectory` (sub) | trajectory_msgs/JointTrajectory | linear interpolation, replaces per joint, clamped to limits |

Mode `publish` mirrors the same state topics (without `/clock` and commands) next to the repository's legacy topics;
`follow` applies `/joint_states` (by name) and `/odom` to the sim. Tests: `tests/ros_twin.test.ts` (murom.urdf).

The server driver (`server/drivers/ros2.ts`) maps joints by name: the command joint list comes from
`ROS2_JOINT_NAMES`, else from the trajectory controller's `joints` parameter, else from `/joint_states` (warned);
prismatic joints are converted m ↔ mm from the robot_description joint types; DO topics are advertised as
`std_msgs/Bool`, DI topics subscribed. Tests: `tests/ros_driver.test.ts`.

## Not covered

- Camera images (`sensor_msgs/Image` bgra8 / depth 32FC1) and the MURKA simulator services (`/murom/sim/attach`,
  `/murom/sim/objects` …) need custom message packages and rendering on the bridge side — not published yet.
- Native DDS from the browser is out of scope; everything goes through rosbridge.
