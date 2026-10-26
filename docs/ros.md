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
A launch entry that expands to something other than a URDF `<robot>` (the SDF / world xacros a Gazebo launch file
spawns, e.g. `gz_waffle.sdf.xacro` in nav2_minimal_tb3_sim) is skipped with a report line, and when no launch robot can
be imported the package's other robot descriptions are imported instead. The same robot reached twice (its xacro with
the launch arguments and its pre-expanded URDF from another launch file, as in webots_ros2_universal_robot; identity =
robot name + links + joints) is imported once; an expansion error of a file is only reported when no other entry of
that file expanded (no spurious "Undefined substitution argument" for the bare top-level expansion).
Tests: `tests/ros_package.test.ts`.

## Exports

- **URDF package** (`src/io/urdf/urdf_export.ts`): library robots without meshes export the shapes the 3D view draws
  as STL visual + collision meshes per link (at the URDF link frames: RViz shows the robot where the studio does), and
  the root link's estimated inertia moves to a fixed `<root>_inertia` child (KDL rejects an inertia on the root); a
  whole-station package has `config/<station>.srdf` (one chain group per robot, station link / joint names). Robots
  imported from URDF export their kept description — link /
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

rosbridge mode **`sim`** makes the studio the simulator backend of a ROS 2 stack. Stations with the MURKA MUROM
robot get the exact MURKA interface (next section); any other station gets the webots_ros2-style twin:

| Topic | Type | |
|---|---|---|
| `/clock` | rosgraph_msgs/Clock | sim time, every step |
| `/joint_states` | sensor_msgs/JointState | every joint by URDF name (wheels with velocity, mimic joints), 20 Hz; in a multi-robot station a joint is prefixed (`<robot>_`) only when its name clashes with another robot's joint |
| `/robot_description` | std_msgs/String | latched; the kept URDF, or the station URDF with floating mobile bases |
| `/tf` | tf2_msgs/TFMessage | `odom → <base link>` of each mobile robot, 31 Hz |
| `/tf_static` | tf2_msgs/TFMessage | `map → odom` when `twin.mapFrame` is set |
| `<ns>/odom` | nav_msgs/Odometry | from the start pose, base-frame twist, 31 Hz |
| `<ns>/imu` | sensor_msgs/Imu | `imu_link`, 31 Hz |
| `<ns>/scan` | sensor_msgs/LaserScan | sensor-relative angles, 10 Hz (when the navigation stack simulates a 2D LiDAR) |
| `<ns or /robot>/ground_truth_pose` | geometry_msgs/PoseStamped | map frame |
| `<ns>/cmd_vel` (sub) | geometry_msgs/Twist | drives the base through the drive model; 0.6 s watchdog |
| `/<robot>/joint_trajectory`, `/joint_trajectory_controller/joint_trajectory` (sub) | trajectory_msgs/JointTrajectory | linear interpolation, replaces per joint, clamped to limits; a joint is found by its `/joint_states` name or, when unique (inside the robot for `/<robot>/…`), by the name in its own URDF |

The combined station description prefixes the *links* of a URDF import whose link names clash with another item's
(TurtleBot 4 and a UR5e both have `base_link`), so TF frames stay unique, but keeps its *joint* names unless those clash
too: standard `JointTrajectory` commands with the URDF joint names reach the arm in a multi-robot station.

Mode `publish` mirrors the same state topics (without `/clock` and commands) next to the repository's legacy topics;
`follow` applies `/joint_states` (by name) and `/odom` to the sim. Tests: `tests/ros_twin.test.ts` (murom.urdf).

The server driver (`server/drivers/ros2.ts`) maps joints by name: the command joint list comes from
`ROS2_JOINT_NAMES`, else from the trajectory controller's `joints` parameter, else from `/joint_states` (warned);
prismatic joints are converted m ↔ mm from the robot_description joint types; DO topics are advertised as
`std_msgs/Bool`, DI topics subscribed. Tests: `tests/ros_driver.test.ts`.

Camera items of non-MURKA stations are published by the twin in `sim` mode (5 Hz, option `twin.cameras`):
`<ns>/<cam>/image_raw` (rgb8), `/camera_info` (render K), `/depth/image_raw` (32FC1 metres, NaN = no return) and TF
`map → <cam>_link → <cam>_optical_frame` (`tests/ros_exports_r5.test.ts`). A depth camera (the wizard's RGB-D / stereo /
ToF, a Gazebo `rgbd_camera`) publishes the colour image too; depth-only devices (Webots RangeFinder, Gazebo `depth` /
`depth_camera`, `params.depthOnly`) publish depth only (`cameraHasColour`). The exported perception package
(Tools › Export ROS 2 perception package) subscribes to exactly these topics with `ros2 launch <pkg>
perception.launch.py source:=studio` (`studioCameraTopics`), to the driver's topics by default; the driver and the
yolo_ros nodes are optional (skipped with a log line when not installed) and yolo_ros is an `exec_depend`
(`tests/vision_ros.test.ts`; in WSL the launch starts on a stock Jazzy in both sources).

## Studio as the MURKA simulator backend

A station holding the decomposed MUROM robot (Webots import of `retail_store.wbt`: `MobileRobot "murom"` with
`left/right_wheel_joint`, `Robot` `murom_left_arm` / `murom_right_arm` / `murom_head`; products from
`murom_world/config` with `params.sku` / `params.slot`, e.g. `buildMurkaRetailStation()` of docs/murka-retail.md) is
served with **exactly the interface of the Webots backend** (`murom_webots`: MuromDriver + SupervisorPlugin + the
webots_ros2 sensor plugins of `murom_webots.urdf`) and of `murom_mujoco/sim_node.py` — robot-retailer-murka-v2
`docs/interfaces.md` §1–2 (`src/ros/murom_sim.ts`, chosen automatically by rosbridge mode `sim` and by the headless
backend). The ROS side runs exactly what `webots.launch.py` starts besides Webots: rosbridge_server (with rosapi, as
`rosbridge_websocket_launch.xml` starts it) and robot_state_publisher with `murom.urdf` (all robot TF frames,
`laser_frame`, `imu_link`, `camera_link_optical`).

| Interface | Type | Behaviour (sim time, control frame = world `basicTimeStep` 32 ms) |
|---|---|---|
| `/clock` | rosgraph_msgs/Clock | every frame (31 Hz) |
| `/odom` + `/tf` `odom → base_link` | nav_msgs/Odometry | every frame; odom = start pose; exact (no wheel slip) |
| `/joint_states` | sensor_msgs/JointState | 20 Hz; the 16 joints in the driver order: wheels (angle, commanded wheel speed), `l_*`, `r_*`, head, wrists |
| `/imu` | sensor_msgs/Imu | every frame, `imu_link`: world orientation, gyro, acceleration + g |
| `/scan` | sensor_msgs/LaserScan | 10 Hz, `laser_frame`, 640 beams over 4.189 rad **left to right** like webots_ros2 Ros2Lidar (`angle_min = +fov/2`, negative increment), `+inf` outside [0.15, 8] m, Webots noise (std = noise × maxRange); ray cast on the scene geometry except the robot (hall + products, like `LIDAR_GROUPS` of murom_mujoco) |
| `/head_camera/image_color` + `/camera_info` | Image bgra8 + CameraInfo | 5 Hz (`cameraRate`) **while subscribed**, 320×240, `camera_link_optical`; K of the unified camera model (fx = W / 2 tan(fov/2) = 233.87 px; principal point W/2 − 0.5, the ROS integer-pixel convention — webots_ros2 writes W/2) |
| `/head_depth/image` + `/camera_info` | Image 32FC1 | **while subscribed**; depth along the optical axis (m), `+inf` outside [0.1, 5] m (geometry nearer than 0.1 m is `inf`, not see-through) |
| `/murom/ground_truth_pose` | geometry_msgs/PoseStamped | every frame, `map`, same stamp as `/odom` |
| `/murom/sim/tool_tips` | geometry_msgs/PoseArray | every frame, `map`, [left, right] forearm tips (forearm + 0.248 m = `l/r_tool_link`) |
| `/murom/sim/objects` | murom_msgs/ProductArray | 2 Hz; sku, name, `location_id` (slot, `carried_*`, `basket_*`, `packing_*`), pose in `map` |
| `/cmd_vel` (sub) | geometry_msgs/Twist | differential drive r = 0.177 m, b = 0.441 m, wheel speed ≤ 18 rad/s (curvature kept), 0.6 s watchdog |
| `/murom/joint_trajectory` (sub) | trajectory_msgs/JointTrajectory | linear interpolation from the current position; a new message replaces the active trajectory; unknown joint → warning, message ignored; Webots motor servo v = clip(10 (target − q), ±maxVelocity); wrist limits clip |
| `/murom/sim/attach`, `/murom/sim/detach` | murom_msgs/srv/AttachObject, DetachObject | SupervisorPlugin rules and messages: v2 grip point = palm surface centre within `MUROM_ATTACH_MAX_DIST` (0.18 m) of the top face; two hands: palm midpoint ≤ 0.15 m from the centre and the palm distance = box width ± 0.03 m; the product follows the palm (or the tips); detach with `place_pose` sets it upright with the yaw it had in the hand; `basket_*` items ride with the robot |
| `/murom/sim/screenshot`, `/murom/sim/record` | std_srvs/srv/SetBool | answered `success: false` (use the studio window) unless the host provides a screenshot hook |

Cameras on demand: the head camera and the range finder have no `<alwaysOn>` in `murom_webots.urdf`, so webots_ros2
renders them only while a node subscribes; the studio does the same. The backend asks rosapi
(`/rosapi/subscribers`, every second) whether the image or camera_info topics have subscribers; images start within
about a second of the first subscription and stop when it goes. Until rosapi has answered the devices stay off; when
rosapi is not running (three failed calls) they publish at `cameraRate` all the time (`cameraOnDemand: false` /
`--camera-always` force that). Lidar, IMU and the rest are always on (`<alwaysOn>true</alwaysOn>`).
Back pressure: rosbridge (Python) has to decode about 0.8 MB of JSON per bgra8 + 32FC1 frame pair. The backend
subscribes to its own `/clock` through rosbridge (throttled to 10 Hz); the echo's lag over its smallest value of the
last 30 s says whether rosbridge's queue keeps growing. Above 1 s camera frames are dropped until it is back below
0.4 s; the clock, odometry, lidar and the rest are never dropped. `backendStats()` reports `rosLag` and
`camera: {rendered, skipped (no subscriber), dropped}`.
Images come from the software rasterizer of `src/vision/sensing.ts` (same camera model; flat shading); the
page-thread fallback of the browser uses the WebGL buffers of the 3D view when registered. Transport (`src/ros/rosbridge_client.ts`, also used by
`src/ros/rosbridge.ts`): rosbridge v2 with `advertise_service` (ROS service calls arrive as `call_service` ops and are
answered with `service_response`), non-finite floats written as the `Infinity` / `NaN` tokens that rosbridge's
JSON backends read (Python json, and ujson 5.9 of the Ubuntu 24.04 package — checked; JSON `null` is rejected for float
fields), `uint8[]` as base64. QoS: rosbridge publishers are reliable / volatile (latched topics transient local), which matches
the `SensorDataQoS().reliable()` of the webots_ros2 sensor plugins and serves best-effort subscribers (Nav2 costmaps).
The interface also runs on a URDF import of `murom.urdf` (wrist chains as separate robots; no products).

### In the browser (Connect › ROS 2 via rosbridge… › Simulator backend)

1. File › Open folder `ros2_ws/src/murom_webots` (the Webots world; the folder is a ROS package, choose "Webots world"),
   or `ros2_ws/src` (world and `murom_world/config` together).
2. Connect › ROS 2 via rosbridge…: when the MUROM world has no products, the dialog offers **Load retail layout
   (murom_world/config)…** — the supervisor's `$MUROM_CONFIG_DIR`; it spawns the 50 products of `products.yaml` in the
   `locations.yaml` slots exactly as a folder import with the YAML files does (`src/ros/murka_layout.ts`; products as the
   coloured boxes `RetailProduct.proto` is). Without it the backend warns and `/murom/sim/objects` is empty.
3. Mode "Simulator backend", the rosbridge URL, OK.

The backend then runs in a **Web Worker** (`src/ros/sim_worker.ts`) on a structured clone of the station (item tree +
the meshes it references, `stationTransfer`): the same `sim_backend.ts` loop as the headless CLI, paced to the wall
clock with catch-up frames, with its own rosbridge connection. The 3D view, the UI and the WebGL renders cannot stall
it. What it moves — base state, joints, product poses — comes back to the page as state diffs by item id at up to
30 Hz (`src/ros/sim_mirror.ts`), so the 3D view and the tree follow the backend. `app.ros.backendStats()` reports sim
time, RTF, frames, messages and the time spent in frames. The station is copied at Connect: edits made in the
studio while connected are not seen by the backend (reconnect); opening another station disconnects it. Without
workers (Node, `?worker=off`, or a worker that fails to load) the same loop runs on the page thread (catch-up keeps
the sim clock on the wall clock across a 400 ms stall; the former `setInterval` loop reached 0.43 of real time).

Headless backend (`src/ros/sim_backend.ts`, CLI `src/ros/sim_cli.ts`):

```bash
npx vite build --ssr src/ros/sim_cli.ts --outDir dist/ros-sim            # once
# WSL / Linux: rosbridge_server + robot_state_publisher (+ localization / Nav2 / the whole bringup with sim:=none)
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090 -p use_sim_time:=true &
ros2 launch murom_description description.launch.py use_sim_time:=true &
ros2 launch murom_bringup retail_simulation.launch.py sim:=none use_rviz:=false enable_perception:=false &
node dist/ros-sim/sim_cli.js --murka "<MURKA_Simulation>" --url ws://127.0.0.1:9090 [--rtf 1] [--camera-rate 5]
```

Verified (`tests/fixtures/ros_gt/tools/murka_sim/`: `setup_ws.sh` builds rosbridge_suite (branch `jazzy`, Python deps
with `pip --target`, no sudo) and a copy of the MURKA `ros2_ws`; `run_stack.sh` / `stop_stack.sh` / `smoke.sh` /
`mission.sh`):

- `tests/ros_murka_sim.test.ts` — contract tests through a fake rosbridge over a real websocket: types, advertise
  before publish, subscriptions, services, rates and stamps in sim time, frames, units, joint order, `/scan` vs an
  analytic ray / box intersection with the hall (all 640 beams < 1 mm), camera layouts and K, depth vs a ray cast,
  the 50 products at their `locations.yaml` slots, tool tips and palms vs `murom.urdf` FK (< 1 mm), `/cmd_vel`
  kinematics + watchdog + saturation, trajectory interpolation + servo law, attach / detach / basket / two-hand
  messages; the same interface on the C1 station (`buildMurkaRetailStation`).
- `ROS_WSL=1 npx vitest run tests/ros_murka_e2e.test.ts` (headless backend) — MURKA's own `scripts/smoke_test.py`
  (unmodified) prints **SMOKE PASS** against the studio (all topics, 640 beams, 16 joints, 50 products, 0.75 m reverse
  drive, |odom − gt| 0.000 m, RTF 1.0); and the real **MURKA FSM + Nav2 + navigate_server** (`murom_bringup` with
  `sim:=none`) run `mission_test.py --mission undock` and `--mission dock`: both **MISSION PASS**, the studio robot
  ends 1.6 cm from the dock pose (last run, RTF 1.0).
- `npm run build`, then `ROS_WSL=1 npm run test:e2e -- tests/e2e/ros_murka_browser.e2e.test.ts` (**the studio in the
  browser**, system Chrome with the hardware GPU; `E2E_GL=swiftshader` for software GL) — the steps above done in the
  UI (Open folder `murom_webots`, Load retail layout, Simulator backend), then `smoke_test.py`, then a camera
  subscriber (`cam_rate.py`, 12 s). With the Intel GPU the smoke test passed in all 5 runs (50 products, 0.752 m,
  backend RTF 1.00 in the worker); the last run, with the final back-pressure thresholds, passed whole: bgra8 / 32FC1
  320×240 at 5.0 Hz in sim time, 0 frames dropped, `/clock` on the ROS side at 0.99 of real time. With SwiftShader
  the smoke test passed in 3 of 4 runs; the software-rendered 3D view loads every core, and with other heavy jobs on
  the machine the WSL VM fell behind (`/clock` seen at 0.2–0.3 of real time, once during the drive, once during the
  camera check) — use a GPU. Before this round the same flow failed (0 products; with SwiftShader also
  "робот не поехал", RTF 0.12).
- `tests/ros_murka_browser_backend.test.ts` — the layout loader equals the folder import (50 products, poses
  1e-6); the worker on a structured clone through a fake rosbridge (the mirrored base equals the published ground
  truth to 1e-9, joints to 1e-6; no images until rosapi reports a subscriber, then 6- or 7-frame gaps); state diffs by
  id; `SubscriberWatch` with and without rosapi; the page-thread loop across a stall.

## MoveIt 2 config and inertials (`src/io/urdf/moveit_export.ts`, `src/io/urdf/inertia.ts`)

- URDF export of studio-native robots writes `<inertial>` estimated from the geometry: every visual (or, for robots
  drawn procedurally, the capsules of the 3D view) is a uniform solid — closed meshes by exact tetrahedral integration,
  primitives analytically, parallel-axis combination; density 1200 kg/m³ or scaled to `robot.params.massKg`.
- `exportMoveItConfig(robot)` writes `<robot>_description` + `<robot>_moveit_config` in the MoveItConfigsBuilder layout:
  `config/<robot>.urdf.xacro` (+ `ros2_control` with `mock_components/GenericSystem`), SRDF, `kinematics.yaml` (KDL),
  `joint_limits.yaml`, `ros2_controllers.yaml` (joint_state_broadcaster + JointTrajectoryController),
  `moveit_controllers.yaml`, `initial_positions.yaml`, `pilz_cartesian_limits.yaml`, `launch/bringup.launch.py`
  (headless, mock hardware), `demo.launch.py` (RViz), `move_group.launch.py`. Verified in WSL Jazzy
  (`ROS_WSL=1 npx vitest run tests/ros_exports_r5.test.ts -t "WSL Jazzy"`): colcon build, move_group ready, controller
  active, `/plan_kinematic_path` SUCCESS, `follow_joint_trajectory` SUCCEEDED and `/joint_states` at the target.
- SRDF `<disable_collisions>` of an imported package are honoured by the collision checker (the pair is skipped, the
  other contacts are still reported — `tests/ros_exports_r5.test.ts`).

## Not covered

- MURKA `perception_node` on the studio images: the colour prototypes are calibrated per simulator
  (`config/appearance_<sim>.yaml` by its `calibrate.py`); a studio appearance file is not provided, so the `scan`
  stage of the mission was not run (the image / depth formats are verified).
- `/murom/sim/record` (video) and `/murom/sim/viewpoint` are not implemented by the backend.
- A studio tab in the background: the worker's timers are not subject to the page's timer throttling, but this was
  not measured (headless Chrome pages stay "visible").
- Native DDS from the browser is out of scope; everything goes through rosbridge.
