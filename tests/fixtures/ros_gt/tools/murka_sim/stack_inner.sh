#!/usr/bin/env bash
source "$(dirname "$0")/env.sh"
MODE="${1:-loc}"
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=$R5_PORT -p address:=127.0.0.1 -p use_sim_time:=true &
ros2 launch murom_description description.launch.py use_sim_time:=true &
case "$MODE" in
  loc) ros2 run murom_navigation localization_node --ros-args -p use_sim_time:=true & ;;
  nav) ros2 launch murom_navigation navigation.launch.py & ;;
  full) ros2 launch murom_bringup retail_simulation.launch.py sim:=none use_rviz:=false enable_perception:=${R5_PERCEPTION:-false} & ;;
esac
wait
