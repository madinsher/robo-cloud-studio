#!/usr/bin/env bash
# R5: run MURKA's scripts/mission_test.py (copied, unmodified) for one mission against the running 'full' stack.
# Usage: bash mission.sh <MURKA v2 path> <mission> [wall timeout s]
source "$(dirname "$0")/env.sh"
MURKA_V2="${1:?MURKA v2 path}"; MISSION="${2:?mission}"; TO="${3:-600}"
cp "$MURKA_V2/scripts/mission_test.py" "$R5_RUN/mission_test.py"
cd "$R5_RUN" && timeout $((TO + 240)) python3 -B mission_test.py --mission "$MISSION" --timeout "$TO" 2>&1
