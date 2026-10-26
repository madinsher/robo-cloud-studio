#!/bin/bash
# Smoke test of generated ROS 2 packages in WSL ROS 2 Jazzy (used by tests/ros_exports.test.ts with ROS_WSL=1):
#   smoke_exports.sh <workspace-with-src (WSL path)> <nav-package>... [<urdf-package>:display]...
# Builds the workspace copy with colcon.
# <nav-package>: `ros2 launch <pkg> bringup.launch.py` until the Nav2 lifecycle manager reports every node active (or
#   LAUNCH_MAX_S, default 240 s), kept running 10 s more, then stopped. Passes when no node died / rejected its parameters
#   / logged an error and Nav2 brought every node up -> "LAUNCH <pkg> OK".
# <urdf-package>:display: `ros2 launch <pkg> display.launch.py`; passes when robot_state_publisher parsed the URDF
#   ("Robot initialized") without errors -> "DISPLAY <pkg> OK (<n> child links)", n = links below the root in the
#   urdfdom tree printed by check_urdf for the installed URDF.
# Only the log before the final Ctrl-C counts (nodes still starting are killed at shutdown). A Nav2 bringup that stalls
# without any error (a lifecycle change_state response lost by DDS on a loaded machine) is retried once.
# Allowed upstream message: SmacPlanner2D in Jazzy always passes 0 as the circumscribed cost to its collision checker
# and logs "Inflation layer either not found ..." at configure whatever the parameters are (the real inflation check,
# computeCircumscribedCost "inflation radius ... smaller than the circumscribed radius", is still an error).
set +u
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=87
SRC="$1"; shift
TMAX=${LAUNCH_MAX_S:-240}
W=$HOME/t5_smoke_ws; rm -rf "$W"; mkdir -p "$W"; cp -r "$SRC/src" "$W/"
cd "$W" && colcon build --event-handlers console_direct- > build.log 2>&1 && echo "BUILD OK" || { echo "BUILD FAILED"; tail -30 build.log; exit 1; }
source install/setup.bash
STP=/opt/ros/jazzy/lib/tf2_ros/static_transform_publisher
BAD="process has died|InvalidParameterTypeException|RCLInvalidROSArgsError|Traceback|Caught exception|what\(\)|Failed to (configure|activate|bring up)|\[ERROR\]|\[FATAL\]"
ESC=$(printf '\033')
strip() { sed "s/${ESC}\[[0-9;]*m//g" "$1" | sed '/user interrupted with ctrl-c/,$d'; }

display() { # <pkg>
  local p="$1" t=0 n bad
  timeout -k 10 -s INT 120 ros2 launch "$p" display.launch.py > "$W/$p.log" 2>&1 & local L=$!
  while [ $t -lt 90 ] && ! grep -q "Robot initialized\|process has died" "$W/$p.log" 2>/dev/null; do sleep 2; t=$((t + 2)); done
  # urdfdom tree of the installed URDF (cross-process DDS delivery is unreliable in this WSL, so no topic echo)
  n=$(check_urdf "$(ls "$W"/install/"$p"/share/"$p"/urdf/*.urdf | head -1)" 2>&1 | grep -c "child([0-9]*):")
  kill -INT $L 2>/dev/null; wait $L 2>/dev/null
  pkill -9 -f "$W/install" 2>/dev/null
  strip "$W/$p.log" > "$W/$p.run.log"
  bad=$(grep -E "$BAD" "$W/$p.run.log" | sort | uniq | head -20)
  if [ -z "$bad" ] && grep -q "Robot initialized" "$W/$p.run.log"; then echo "DISPLAY $p OK ($n child links)"; else echo "DISPLAY $p FAILED ($n child links)"; echo "$bad"; fi
}

bringup() { # <pkg> -> sets RESULT (ok | fail | stall)
  local p="$1" t=0 bad active
  # no robot / simulator here: static map -> odom -> base_link so the costmaps can activate
  timeout -k 5 $((TMAX + 60)) $STP --frame-id map --child-frame-id odom > /dev/null 2>&1 & local S1=$!
  timeout -k 5 $((TMAX + 60)) $STP --frame-id odom --child-frame-id base_link > /dev/null 2>&1 & local S2=$!
  timeout -k 20 -s INT $((TMAX + 30)) ros2 launch "$p" bringup.launch.py > "$W/$p.log" 2>&1 & local L=$!
  while [ $t -lt $TMAX ] && ! grep -q "Managed nodes are active\|Failed to bring up\|process has died" "$W/$p.log" 2>/dev/null; do sleep 2; t=$((t + 2)); done
  sleep 10
  kill -INT $L 2>/dev/null; wait $L 2>/dev/null
  kill -9 $S1 $S2 2>/dev/null; pkill -9 -f "$W/install" 2>/dev/null; sleep 2
  strip "$W/$p.log" > "$W/$p.run.log"
  bad=$(grep -E "$BAD" "$W/$p.run.log" | grep -v "Inflation layer either not found or inflation is not set sufficiently" | sort | uniq | head -20)
  active=$(grep -c "Managed nodes are active" "$W/$p.run.log")
  if [ -z "$bad" ] && [ "$active" -ge 1 ]; then echo "LAUNCH $p OK (${t}s to active)"; RESULT=ok
  elif [ -z "$bad" ]; then RESULT=stall
  else echo "LAUNCH $p FAILED (nav2 active: $active after ${t}s)"; echo "$bad"; RESULT=fail; fi
  grep -E "skipped|Managed nodes are active" "$W/$p.run.log" | sed 's/^\[[^]]*\] //' | sort | uniq -c | head -10
}

for arg in "$@"; do
  p=${arg%%:*}
  if [ "$arg" != "$p" ]; then display "$p"; continue; fi
  bringup "$p"
  if [ "$RESULT" = stall ]; then echo "  (retrying $p: bringup stalled without errors)"; bringup "$p"; fi
  if [ "$RESULT" = stall ]; then echo "LAUNCH $p FAILED (stalled twice without errors)"; fi
done
