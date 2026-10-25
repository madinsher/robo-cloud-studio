#!/bin/bash
# Ground truth for the T5 ROS tests, produced with the real ROS 2 Jazzy tools in WSL (Ubuntu 22.04):
#   MSYS_NO_PATHCONV=1 wsl.exe -d Ubuntu-22.04 -- bash "/mnt/c/robo-cloud-studio/tests/fixtures/ros_gt/tools/gen_gt.sh"
# 1. copies the text sources (xacro/urdf/yaml/srdf/package.xml/launch) of the /opt/ros packages used as inputs into pkgs/
# 2. expands every entry point of CASES with `xacro` -> xacro/<case>.urdf (or .err when ROS rejects it)
# 3. tree FK of every link with urdf_parser_py (gt_fk.py) -> fk/<case>.json
set +u
source /opt/ros/jazzy/setup.bash
FIX="$(cd "$(dirname "$0")/.." && pwd)"
M="/mnt/c/Users/TRITON 300 SE/YandexDisk/Работа/КБНЦ РАН/MURKA_Simulation"
AG="$M/simulation_ws/src/agromarca"
J=/opt/ros/jazzy/share
mkdir -p "$FIX/pkgs" "$FIX/xacro" "$FIX/fk"

copy_pkg() { # name src-dir
  local n="$1" src="$2"
  rm -rf "$FIX/pkgs/$n"; mkdir -p "$FIX/pkgs/$n"
  (cd "$src" && find . -type f \( -name '*.xacro' -o -name '*.urdf' -o -name '*.yaml' -o -name '*.srdf' -o -name 'package.xml' -o -name '*.launch.py' -o -name '*.sdf' \) \
     -not -path './meshes/*' -not -path '*/meshes/*' -print0 | xargs -0 -I{} cp --parents {} "$FIX/pkgs/$n/")
}
if [ "${SKIP_COPY:-0}" != "1" ]; then
  copy_pkg nav2_minimal_tb4_description $J/nav2_minimal_tb4_description
  copy_pkg ur_description $J/webots_ros2_universal_robot/resource/Universal_Robots_ROS2_Driver/ur_description
  copy_pkg webots_ros2_universal_robot $J/webots_ros2_universal_robot
  copy_pkg robot_state_publisher $J/robot_state_publisher
  copy_pkg nav2_minimal_tb3_sim $J/nav2_minimal_tb3_sim
  copy_pkg moveit_ros_planning $J/moveit_ros_planning
  copy_pkg launch_param_builder $J/launch_param_builder
  copy_pkg moveit_setup_controllers $J/moveit_setup_controllers
  # ur_description inside the share folder has no package.xml of its own
  [ -f "$FIX/pkgs/ur_description/package.xml" ] || printf '<?xml version="1.0"?>\n<package format="3"><name>ur_description</name><version>2.1.0</version><description>UR description (copied from webots_ros2_universal_robot)</description><maintainer email="x@x">x</maintainer><license>BSD-3-Clause</license></package>\n' > "$FIX/pkgs/ur_description/package.xml"
fi

PFX=$HOME/t5gt/prefix; rm -rf $HOME/t5gt; mkdir -p $PFX/share/ament_index/resource_index/packages
link() { touch $PFX/share/ament_index/resource_index/packages/$1; ln -s "$2" $PFX/share/$1; }
for p in nav2_minimal_tb4_description ur_description webots_ros2_universal_robot robot_state_publisher nav2_minimal_tb3_sim moveit_ros_planning launch_param_builder moveit_setup_controllers; do link $p "$FIX/pkgs/$p"; done
if [ -d "$M" ]; then
  link murca_description "$AG/murca_garden_ws/src/murca_description"
  link murca_webots "$AG/murca_garden_ws/src/murca_webots"
  link realsense2_description "$AG/urp_gazebo_apples/ws/src/realsense2_description"
  link murom_description "$M/robot-retailer-murka-v2/ros2_ws/src/murom_description"
fi
[ -d /mnt/c/vertical_bot_ros/palletizer_model_pkg ] && link palletizer_model_pkg /mnt/c/vertical_bot_ros/palletizer_model_pkg
export AMENT_PREFIX_PATH=$PFX:$AMENT_PREFIX_PATH

# case | package | file inside package | args
CASES="
murca_robot|murca_description|urdf/murca_robot.urdf.xacro|
murca_robot_alt|murca_description|urdf/murca_robot.urdf.xacro|slot_front:=lidar2d use_manipulator:=false slot_top:=lidar2d slot_cargo:=spray_tank
murca_robot_manip|murca_description|urdf/murca_robot.urdf.xacro|use_manipulator:=true slot_cargo:=crate
aerial_platform|murca_description|urdf/aerial_platform.urdf.xacro|
utp_g|murca_webots|urdf/utp_g.urdf.xacro|
utp_a|murca_webots|urdf/utp_a.urdf.xacro|
rs_d415|realsense2_description|urdf/test_d415_camera.urdf.xacro|
rs_d435|realsense2_description|urdf/test_d435_camera.urdf.xacro|
rs_d435i|realsense2_description|urdf/test_d435i_camera.urdf.xacro|
rs_d455|realsense2_description|urdf/test_d455_camera.urdf.xacro|
rs_l515|realsense2_description|urdf/test_l515_camera.urdf.xacro|
rs_r410|realsense2_description|urdf/test_r410_camera.urdf.xacro|
rs_r430|realsense2_description|urdf/test_r430_camera.urdf.xacro|
rs_multi|realsense2_description|urdf/test_d435_multiple_cameras.urdf.xacro|
rs_one_of_each|realsense2_description|tests/one_of_each.xacro|
tb4|nav2_minimal_tb4_description|urdf/standard/turtlebot4.urdf.xacro|
ur5e|ur_description|urdf/ur.urdf.xacro|ur_type:=ur5e name:=ur
ur5e_prefixed|ur_description|urdf/ur.urdf.xacro|ur_type:=ur5e name:=ur tf_prefix:=left_ use_fake_hardware:=true
ur5e_gripper_noargs|webots_ros2_universal_robot|resource/ur5e_with_gripper.urdf.xacro|
ur5e_gripper|webots_ros2_universal_robot|resource/ur5e_with_gripper.urdf.xacro|name:=UR5e ur_type:=ur5e
rsp_test_desc|robot_state_publisher|urdf/test-desc.urdf.xacro|
palletizer|palletizer_model_pkg|urdf/palletizer_model.urdf.xacro|
tb3_gz_waffle|nav2_minimal_tb3_sim|urdf/gz_waffle.sdf.xacro|
tb3_sandbox_world|nav2_minimal_tb3_sim|worlds/tb3_sandbox.sdf.xacro|
robin_srdf|moveit_ros_planning|rdf_loader/test/data/robin.srdf.xacro|
lpb_parameter|launch_param_builder|config/parameter.xacro|
msc_ros2_control|moveit_setup_controllers|templates/config/ros2_control.xacro|
msc_modified|moveit_setup_controllers|templates/config/modified.urdf.xacro|
"
rm -f "$FIX/xacro/cases.txt"
echo "$CASES" | while IFS='|' read -r name pkg file args; do
  [ -z "$name" ] && continue
  d=$(ros2 pkg prefix $pkg >/dev/null 2>&1 && echo "$PFX/share/$pkg")
  [ -z "$d" ] && { echo "skip $name (no $pkg)"; continue; }
  echo "$name|$pkg|$file|$args" >> "$FIX/xacro/cases.txt"
  rm -f "$FIX/xacro/$name.urdf" "$FIX/xacro/$name.err"
  if (cd "$d" && xacro "$file" $args > "$FIX/xacro/$name.urdf" 2> "$FIX/xacro/$name.err"); then echo "OK   $name"; else echo "FAIL $name: $(head -c 200 "$FIX/xacro/$name.err")"; rm -f "$FIX/xacro/$name.urdf"; fi
  [ -s "$FIX/xacro/$name.err" ] || rm -f "$FIX/xacro/$name.err"
done
# plain URDFs used for FK ground truth
cp "$M/robot-retailer-murka-v2/ros2_ws/src/murom_description/urdf/murom.urdf" "$FIX/xacro/murom.urdf" 2>/dev/null
cp "$AG/murca_garden_ws/assets/urp_reference/urp.urdf" "$FIX/xacro/urp.urdf" 2>/dev/null
cp /mnt/c/vertical_bot_ros/vertical_robot_model/urdf/vertical_robot.urdf "$FIX/xacro/vertical_robot.urdf" 2>/dev/null
cp $J/nav2_minimal_tb3_sim/urdf/turtlebot3_waffle.urdf "$FIX/xacro/turtlebot3_waffle.urdf"
cp $J/webots_ros2_tiago/resource/tiago_webots.urdf "$FIX/xacro/tiago_webots.urdf"
cp $J/webots_ros2_universal_robot/resource/webots_abb_description.urdf "$FIX/xacro/webots_abb_description.urdf"
cd "$FIX/xacro"
for f in *.urdf; do
  case "$f" in tb3_*|robin_srdf*|lpb_*|msc_*) continue;; esac
  grep -q "<robot" "$f" && grep -q "<link" "$f" || continue
  python3 "$FIX/tools/gt_fk.py" "$FIX/fk/${f%.urdf}.json" "$f" 2>&1 | grep -v "Unknown tag\|Unknown attribute" | tail -1
  check_urdf "$f" > /dev/null 2> "$FIX/fk/${f%.urdf}.check_err" && rm -f "$FIX/fk/${f%.urdf}.check_err"
done
