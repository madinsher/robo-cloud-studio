# R5: environment of the MURKA-on-studio stack (sourced). ROS_DOMAIN_ID / port are private to these tests.
source /opt/ros/jazzy/setup.bash
source "$HOME/r5_murom_ws/install/local_setup.bash"
source "$HOME/r5_rosbridge_ws/install/local_setup.bash"
export PYTHONPATH="$HOME/r5_rosbridge_ws/pydeps:$PYTHONPATH"
export ROS_DOMAIN_ID="${R5_DOMAIN:-77}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-SHM}"
export RCUTILS_COLORIZED_OUTPUT=0
export R5_PORT="${R5_PORT:-9097}"
export R5_RUN="${R5_RUN:-/tmp/r5_murka_sim}"
mkdir -p "$R5_RUN"
