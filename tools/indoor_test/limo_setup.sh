#!/bin/bash
# Sandbox setup script run on the NUC. Do not 'set -u' (ROS setup.bash references unset vars).
set -e

echo "=== 1. Verify colcon symlink ==="
SYMLINK=/home/agilex/agilex_ws/src/limo_path_follower
TARGET=/home/agilex/H-infinity/scalecar-vfg-h-infinite/ros2_bridge
if [ ! -L "$SYMLINK" ] || [ "$(readlink "$SYMLINK")" != "$TARGET" ]; then
  echo "Recreating symlink: $SYMLINK -> $TARGET"
  ln -sfn "$TARGET" "$SYMLINK"
else
  echo "OK: $SYMLINK -> $(readlink $SYMLINK)"
fi

echo
echo "=== 2. Check rosbridge_suite ==="
if dpkg -l | grep -q ros-humble-rosbridge-suite; then
  echo "OK: rosbridge_suite installed"
else
  echo "MISSING: ros-humble-rosbridge-suite (will install — sudo will prompt)"
  sudo apt-get install -y ros-humble-rosbridge-suite
fi

echo
echo "=== 3. Source ROS env ==="
source /opt/ros/humble/setup.bash
[ -f /home/agilex/agilex_ws/install/setup.bash ] && source /home/agilex/agilex_ws/install/setup.bash

echo
echo "=== 4. Build limo_path_follower ==="
cd /home/agilex/agilex_ws
colcon build --packages-select limo_path_follower 2>&1 | tail -20

echo
echo "=== 5. Re-source after build ==="
source /home/agilex/agilex_ws/install/setup.bash

echo
echo "=== 6. Smoke import: path_follower_node module ==="
python3 -c "
import importlib
m = importlib.import_module('limo_path_follower.path_follower_node')
n = m.PathFollowerNode
print('PathFollowerNode imported OK from', m.__file__)
print('has _publish_status:', hasattr(n, '_publish_status'))
"

echo
echo "=== 7. Smoke import: estop_cli ==="
python3 -c "
import sys
sys.path.insert(0, '/home/agilex/H-infinity')
import estop_cli
print('estop_cli imported OK from', estop_cli.__file__)
print('has estop_trigger_callback:', hasattr(estop_cli.EstopCliNode, 'estop_trigger_callback'))
"

echo
echo "=== 8. List rosbridge launch targets ==="
ls /opt/ros/humble/share/rosbridge_server/launch/ 2>/dev/null || echo "(no rosbridge launch dir found)"

echo
echo "=== ALL DONE ==="
