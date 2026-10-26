#!/usr/bin/env bash
# R5: start the ROS side of "studio as the MURKA simulator backend" — what murom_webots/webots.launch.py starts
# besides Webots itself: rosbridge_server (the studio connects to it) and robot_state_publisher (murom.urdf); plus
#   loc   MURKA localization_node (/murom/pose, map->odom from /murom/ground_truth_pose)   [default; smoke_test.py]
#   nav   murom_navigation/navigation.launch.py (Nav2 + localization + navigate_server + battery)
#   full  murom_bringup/retail_simulation.launch.py sim:=none (navigation, manipulation, FSM; perception with
#         R5_PERCEPTION=true)
# All nodes run in one new session (setsid); this script prints STACK_READY and stays in the foreground until the
# session ends (stop_stack.sh), so the wsl.exe that runs it keeps the stack alive.
source "$(dirname "$0")/env.sh"
MODE="${1:-loc}"
LOG="$R5_RUN/stack.log"
: > "$LOG"
setsid bash "$(dirname "$0")/stack_inner.sh" "$MODE" >>"$LOG" 2>&1 < /dev/null &
SID=$!
echo "$SID" > "$R5_RUN/stack.sid"
ready=0
for i in $(seq 1 180); do
  if grep -q "Rosbridge WebSocket server started" "$LOG" 2>/dev/null; then ready=1; break; fi
  kill -0 "$SID" 2>/dev/null || break
  sleep 0.5
done
if [ "$ready" = 1 ]; then echo "STACK_READY port=$R5_PORT mode=$MODE sid=$SID"; else echo "STACK_TIMEOUT"; tail -30 "$LOG"; fi
wait "$SID"
echo "STACK_EXIT"
