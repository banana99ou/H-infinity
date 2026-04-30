#!/usr/bin/env bash
# Run the LIMO around an indoor 5x5 m sinusoidal path.
# Usage: run_indoor.sh [speed_mps]   (default 0.3)
#
# Starts the path_follower_node with v_const=<speed>, then publishes a
# latched nav_msgs/Path on /reference_path. Ctrl+C stops both.
#
# Workspace: 5x5 m, 0.5 m margin -> usable 4x4 m.
# Path: start (0.5, 2.5) heading +x, end (4.5, 2.5).
#       y(x) = 2.5 + 0.7 * sin(2*pi*(x-0.5)/4)
#       R_min ~= 0.58 m (above 0.5 m mechanical limit).

set -e

SPEED="${1:-0.3}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source /home/agilex/agilex_ws/install/setup.bash

cleanup() {
  echo
  echo "[indoor] stopping..."
  if [ -n "${FOLLOWER_PID:-}" ] && kill -0 "$FOLLOWER_PID" 2>/dev/null; then
    kill "$FOLLOWER_PID" 2>/dev/null || true
    wait "$FOLLOWER_PID" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

echo "[indoor] launching path_follower_node with v_const=$SPEED"
ros2 run limo_path_follower path_follower_node \
  --ros-args -p v_const:="$SPEED" &
FOLLOWER_PID=$!

# Give the node a moment to declare its subscription before we publish.
sleep 2.0

echo "[indoor] publishing /reference_path (latched)"
python3 "$DIR/publish_indoor_path.py"

# publish_indoor_path.py spins; when user Ctrl+Cs, trap fires and kills follower.
