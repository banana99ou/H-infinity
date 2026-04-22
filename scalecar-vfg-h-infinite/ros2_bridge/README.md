# LIMO Path Follower -- ROS2 Bridge

ROS2 node that wraps the `vfg_pathfollowing` Python package for real-time
path following on the AgileX LIMO robot.

## Prerequisites

- **ROS2 Humble** (Ubuntu 22.04)
- **vfg_pathfollowing** Python package installed in the ROS2 Python environment:
  ```bash
  pip install -e /path/to/limo-vfg-pathfollowing
  ```

## Build

```bash
# From your colcon workspace root (e.g., ~/ros2_ws)
ln -s /path/to/limo-vfg-pathfollowing/ros2_bridge src/limo_path_follower
colcon build --packages-select limo_path_follower
source install/setup.bash
```

## Run

```bash
# Default parameters
ros2 run limo_path_follower path_follower_node

# With parameter file
ros2 run limo_path_follower path_follower_node --ros-args \
    --params-file $(ros2 pkg prefix limo_path_follower)/share/limo_path_follower/config/params.yaml

# Override individual parameters
ros2 run limo_path_follower path_follower_node --ros-args \
    -p controller_type:=pid-ff \
    -p v_const:=1.5
```

## Topics

### Subscribed

| Topic    | Type                     | Description              |
|----------|--------------------------|--------------------------|
| `/odom`  | `nav_msgs/msg/Odometry`  | Vehicle odometry (x, y, yaw, v) |

### Published

| Topic      | Type                      | Description                     |
|------------|---------------------------|---------------------------------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | linear.x = v, angular.z = omega |

## Parameters

| Parameter         | Type   | Default      | Description                                  |
|-------------------|--------|--------------|----------------------------------------------|
| `controller_type` | string | `lpv-hinf`   | Controller: `lpv-hinf` or `pid-ff`           |
| `v_const`         | double | `1.0`        | Constant forward speed [m/s]                 |
| `k_e`             | double | `3.0`        | VFG convergence gain                         |
| `dt_ctrl`         | double | `0.05`       | Control period [s] (node runs at 1/dt_ctrl Hz) |
| `wheelbase`       | double | `0.2`        | LIMO wheelbase [m]                           |
| `K_P`             | double | `2.0`        | PID proportional gain (pid-ff only)          |
| `K_D`             | double | `0.3`        | PID derivative gain (pid-ff only)            |

## Architecture

```
/odom ──> [path_follower_node] ──> /cmd_vel
              │
              ├── VectorFieldGuidance  (psi_des, kappa, e_d)
              └── LPVHinfController or PIDFeedforward  (delta_cmd)
                     │
                     └── delta_cmd → omega = v * tan(delta) / L
```

The node uses a hardcoded `StepCurvaturePath` for demonstration.
Replace it with your own path (e.g., from a `/path` topic or a waypoint file)
for real experiments.
