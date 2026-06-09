# tools/launch — NUC launch files (manual deploy)

These launch files live in the **NUC vendor package** (`limo_base`), which is
outside the `sync.sh` path, so they are version-controlled here but deployed by
hand.

## `MAVROS+RTK_Node_Launcher.launch.py`

The **GNSS half** of the vendor `LIMO+MAVROS+RTK_Node_Launcher.launch.py`
(mavros + F9P RTK, chassis node removed). Pairs with the chassis-only `base`
PROC so the `odom_watchdog` can respawn the chassis driver on a base-serial
(CP2102) dropout **without** dropping RTK/compass.

Deploy on the NUC:

```bash
scp "tools/launch/MAVROS+RTK_Node_Launcher.launch.py" \
    agilex@<nuc>:/home/agilex/agilex_ws/src/limo_ros2/limo_base/launch/
ssh agilex@<nuc> 'cd ~/agilex_ws && colcon build --packages-select limo_base'
```

Then it resolves as `ros2 launch limo_base MAVROS+RTK_Node_Launcher.launch.py`
(orchestrator `gnss` PROC). It finds `GPS-RTK_ROS2_pub_node.py` by walking up
from the launch dir (resolves to `~/agilex_ws/GPS-RTK_ROS2_pub_node.py`), so it
must stay in that launch dir.

The chassis half is the stock `limo_base.launch.py` started as
`ros2 launch limo_base limo_base.launch.py port_name:=limo_base` (the `base`
PROC) — `port_name:=limo_base` pins the udev symlink instead of the launch's
bare `ttyUSB1` default (dual-CP2102 hazard).
