#!/usr/bin/env bash
# R5: stop the session run_stack.sh started (only its own processes), keep the log.
source "$(dirname "$0")/env.sh"
SID=$(cat "$R5_RUN/stack.sid" 2>/dev/null)
if [ -n "$SID" ]; then
  kill -INT -- -"$SID" 2>/dev/null
  for i in $(seq 1 20); do pgrep -s "$SID" >/dev/null 2>&1 || break; sleep 0.5; done
  pkill -KILL -s "$SID" 2>/dev/null
fi
echo STACK_STOPPED
