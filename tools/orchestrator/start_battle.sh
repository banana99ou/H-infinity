#!/bin/bash
# Bootstrap the LIMO battle station. Brings up:
#   - rosbridge_websocket on :9090
#   - limo_orchestrator (process supervisor)
#
# Everything else (limo_base / limo_base_gnss / estop_cli / path_follower_node)
# is then started/stopped from the browser UI via /orchestrator/start and
# /orchestrator/kill topics.
#
# Usage: bash ~/H-infinity/tools/orchestrator/start_battle.sh

set -e

# Do NOT 'set -u' — ROS setup.bash references unset vars.
source /opt/ros/humble/setup.bash
[ -f /home/agilex/agilex_ws/install/setup.bash ] && source /home/agilex/agilex_ws/install/setup.bash

cleanup() {
  echo
  echo "[start_battle] shutting down rosbridge (pid $ROSBRIDGE_PID)"
  if [ -n "$ROSBRIDGE_PID" ] && kill -0 "$ROSBRIDGE_PID" 2>/dev/null; then
    kill "$ROSBRIDGE_PID" 2>/dev/null || true
    sleep 1
    kill -9 "$ROSBRIDGE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[start_battle] starting rosbridge_websocket on :9090"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/limo_orchestrator/rosbridge.log 2>&1 &
ROSBRIDGE_PID=$!
mkdir -p /tmp/limo_orchestrator
sleep 2

echo "[start_battle] starting limo_orchestrator (Ctrl+C to stop everything)"
exec ros2 run limo_path_follower orchestrator_node
