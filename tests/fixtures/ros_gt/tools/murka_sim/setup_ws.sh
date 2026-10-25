#!/usr/bin/env bash
# R5: WSL workspaces for "studio as the MURKA simulator backend" (no sudo, nothing written into the MURKA folder).
#   ~/r5_rosbridge_ws  rosbridge_suite (branch jazzy) built from source + its Python deps (tornado, pymongo/bson, cbor2)
#                      installed with pip --target into ~/r5_rosbridge_ws/pydeps (no change to the user's site-packages)
#   ~/r5_murom_ws      COPY of robot-retailer-murka-v2/ros2_ws/src (rsync like scripts/build.sh), colcon build
# Usage: bash setup_ws.sh "<MURKA v2 repo path in WSL>"   (idempotent; rebuilds only when sources changed)
set -eo pipefail
MURKA_V2="${1:?path of robot-retailer-murka-v2 (WSL path)}"
source /opt/ros/jazzy/setup.bash
RB=$HOME/r5_rosbridge_ws
MW=$HOME/r5_murom_ws
mkdir -p "$RB/src" "$MW/src"

if [ ! -f "$RB/install/rosbridge_server/share/rosbridge_server/package.xml" ]; then
  [ -d "$RB/src/rosbridge_suite" ] || git clone --quiet --depth 1 -b jazzy https://github.com/RobotWebTools/rosbridge_suite.git "$RB/src/rosbridge_suite"
  (cd "$RB" && colcon build --cmake-args -DBUILD_TESTING=OFF --event-handlers console_direct- 2>&1 | tail -5)
  [ -f "$RB/install/rosbridge_server/share/rosbridge_server/package.xml" ] || { echo "rosbridge build failed"; exit 1; }
fi
if [ ! -d "$RB/pydeps/tornado" ] || [ ! -d "$RB/pydeps/cbor2" ]; then
  pip3 install --quiet --upgrade --target "$RB/pydeps" tornado pymongo cbor2 2>&1 | tail -2
fi

# MURKA workspace copy (yasmin comes from apt here; setup_deps.sh logic: clone 6.1.0 only when missing)
rsync -a --delete --exclude '__pycache__' --exclude 'external' "$MURKA_V2/ros2_ws/src/" "$MW/src/"
if ! python3 -c "import yasmin" 2>/dev/null && [ ! -d "$MW/src/external/yasmin" ]; then
  git clone --quiet --depth 1 --branch 6.1.0 https://github.com/uleroboticsgroup/yasmin "$MW/src/external/yasmin"
  for d in "$MW"/src/external/yasmin/yasmin_*; do [ -d "$d" ] && touch "$d/COLCON_IGNORE"; done
fi
(cd "$MW" && colcon build --event-handlers console_direct- 2>&1 | tail -8)
echo "R5_SETUP_OK rosbridge=$RB murom=$MW"
