#!/usr/bin/env bash
# R5: run MURKA's scripts/smoke_test.py (copied, unmodified) against the running stack.
source "$(dirname "$0")/env.sh"
MURKA_V2="${1:?MURKA v2 path}"
cp "$MURKA_V2/scripts/smoke_test.py" "$R5_RUN/smoke_test.py"
cd "$R5_RUN" && timeout 120 python3 -B smoke_test.py 2>&1
