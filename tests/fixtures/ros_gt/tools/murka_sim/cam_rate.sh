#!/usr/bin/env bash
# G3: camera topic rates (cam_rate.py) against the running stack. Usage: bash cam_rate.sh [wall seconds]
source "$(dirname "$0")/env.sh"
cd "$R5_RUN" && timeout 120 python3 -B "$(dirname "$0")/cam_rate.py" "${1:-10}" 2>&1
