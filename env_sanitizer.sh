# ROS 2 + workspace
source /opt/ros/humble/setup.bash
source ~/agilex_ws/install/setup.bash

# Use a fixed domain for all robot-related terminals
export ROS_DOMAIN_ID=0

# Use default RMW implementation (FastDDS)
unset RMW_IMPLEMENTATION

# CRITICAL for reliability on WiFi:
# Set to 1 to isolate ROS 2 traffic to the NUC only. 
# This prevents the laptop's laggy WiFi from stalling the robot's control loop.
# Set to 0 only when you need to visualize data on your laptop (e.g., via Rviz).
export ROS_LOCALHOST_ONLY=1

# Restart daemon to apply discovery changes
ros2 daemon stop
ros2 daemon start
