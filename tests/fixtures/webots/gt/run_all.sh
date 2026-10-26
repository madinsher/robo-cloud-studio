#!/bin/bash
# Pose ground truth for the Webots importer. Usage: run_all.sh <tools dir> <world.wbt>...   (worlds = COPIES of the
# MURKA worlds with their protos/assets at the same relative paths; never the originals).
# For each world: installs controllers/pose_dump (+ runtime.ini selecting a Python >= 3.8) in its project, writes
# gt_<world>.wbt (prep_world.py) and runs Webots in batch mode; the supervisor writes gt_<world>.wbt.json next to it.
TOOLS="$1"; shift
WB="${WEBOTS_EXE:-/c/Program Files/Webots/msys64/mingw64/bin/webots.exe}"
PY="${GT_PYTHON:-C:\\RoboDK\\Python-Embedded\\python.exe}"
for w in "$@"; do
  proj="$(dirname "$(dirname "$w")")"
  mkdir -p "$proj/controllers/pose_dump"
  cp "$TOOLS/pose_dump.py" "$proj/controllers/pose_dump/"
  printf '[python]\nCOMMAND = %s\n' "$PY" > "$proj/controllers/pose_dump/runtime.ini"
  g="$(dirname "$w")/gt_$(basename "$w")"
  "$(cygpath -u "$PY")" -I "$TOOLS/prep_world.py" "$w" "$g" Murom Murka MurkaClassic UTP_G UTP_A
  rm -f "$g.json"
  timeout 300 "$WB" --batch --mode=fast --no-rendering --minimize --stdout --stderr "$(cygpath -w "$g")" > "$g.log" 2>&1
  echo "$(basename "$g") exit=$? json=$(test -f "$g.json" && wc -c < "$g.json")"
  grep -v "^INFO" "$g.log" | sort | uniq -c | sort -rn | head -5
done
