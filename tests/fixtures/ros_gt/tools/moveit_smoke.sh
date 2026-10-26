#!/usr/bin/env bash
# R5: build an exported <robot>_description + <robot>_moveit_config pair in a fresh colcon workspace and bring it up
# headless (mock hardware): move_group ready, controllers active, a joint-space plan from /plan_kinematic_path and
# its execution through <controller>/follow_joint_trajectory, then the joint states.
# Usage: moveit_smoke.sh <dir with both packages> <moveit pkg> <controller>
# (the request / goal YAMLs are files next to the packages: plan_request.yaml, goal.yaml)
set -o pipefail
source /opt/ros/jazzy/setup.bash
SRC="$1"; PKG="$2"; CTRL="$3"
export ROS_DOMAIN_ID=${R5_DOMAIN_MOVEIT:-78} FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-SHM} RCUTILS_COLORIZED_OUTPUT=0
WS=$(mktemp -d "$HOME/r5_moveit_XXXX")
mkdir -p "$WS/src" && cp -r "$SRC"/*/ "$WS/src/" && cp "$SRC"/*.yaml "$WS/"
cd "$WS" && colcon build --event-handlers console_direct- > build.log 2>&1 || { echo "BUILD_FAIL"; tail -30 build.log; exit 1; }
echo "BUILD_OK"
source install/setup.bash
setsid ros2 launch "$PKG" bringup.launch.py > launch.log 2>&1 < /dev/null &
SID=$!
cleanup() { kill -INT -- -"$SID" 2>/dev/null; sleep 2; pkill -KILL -s "$SID" 2>/dev/null; rm -rf "$WS"; }
trap cleanup EXIT
for i in $(seq 1 240); do grep -q "You can start planning now" launch.log && grep -q "Configured and activated $CTRL" launch.log && break; sleep 0.5; done
grep -q "You can start planning now" launch.log && echo "MOVE_GROUP_READY" || { echo "MOVE_GROUP_NOT_READY"; tail -40 launch.log; exit 1; }
grep -q "Configured and activated $CTRL" launch.log && echo "CONTROLLER_ACTIVE" || { echo "CONTROLLER_NOT_ACTIVE"; grep -i "error\|fail" launch.log | head; exit 1; }
grep -E "\[ERROR\]|process has died" launch.log | grep -v "No 3D sensor plugin" | head -5
timeout 60 ros2 service call /plan_kinematic_path moveit_msgs/srv/GetMotionPlan "$(cat "$WS/plan_request.yaml")" > plan.log 2>&1
grep -o "error_code=moveit_msgs.msg.MoveItErrorCodes(val=[-0-9]*" plan.log | head -1 | sed 's/.*val=/PLAN_CODE=/'
echo "PLAN_POINTS=$(grep -o "JointTrajectoryPoint(" plan.log | wc -l)"
timeout 60 ros2 action send_goal "/$CTRL/follow_joint_trajectory" control_msgs/action/FollowJointTrajectory "$(cat "$WS/goal.yaml")" > goal.log 2>&1
grep -q "Goal finished with status: SUCCEEDED" goal.log && echo "EXECUTE_SUCCEEDED" || { echo "EXECUTE_FAILED"; tail -10 goal.log; }
timeout 45 python3 - <<'PY'
import time, rclpy
from sensor_msgs.msg import JointState
rclpy.init()
n = rclpy.create_node('r5_js_probe')
got = []
n.create_subscription(JointState, '/joint_states', lambda m: got.append(m), 10)
t0 = time.monotonic()
while not got and time.monotonic() - t0 < 30:
    rclpy.spin_once(n, timeout_sec=0.2)
if got:
    m = got[-1]
    print('JOINT_STATES ' + ' '.join(f'{a}={b:.4f}' for a, b in zip(m.name, m.position)))
else:
    print('JOINT_STATES_TIMEOUT')
PY
