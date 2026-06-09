import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_repo_file_upwards(start: Path, filename: str, max_depth: int = 12) -> Path:
    """Find a file by walking up parent directories (works from source or install)."""
    cur = start.resolve()
    for _ in range(max_depth + 1):
        candidate = cur / filename
        if candidate.exists():
            return candidate
        if cur.parent == cur:
            break
        cur = cur.parent
    raise RuntimeError(
        f"Could not find '{filename}' by searching parent directories from '{start}'.")


def generate_launch_description():
    """mavros + F9P RTK ONLY — the GNSS half of LIMO+MAVROS+RTK_Node_Launcher,
    with the limo_base chassis node removed. Pair with the chassis 'base' PROC
    (limo_base.launch.py port_name:=limo_base). Splitting them lets odom_watchdog
    respawn the chassis driver on a base-serial dropout WITHOUT dropping RTK or
    compass (separate USB: CP2102 chassis vs ttyACM* Pixhawk/F9P)."""
    fcu_url = LaunchConfiguration("fcu_url")
    gps_rtk_script = _find_repo_file_upwards(Path(__file__).parent, "GPS-RTK_ROS2_pub_node.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fcu_url",
                default_value="serial:///dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00:115200",
                description="Pixhawk FCU connection URL for MAVROS",
            ),
            # --- MAVROS node (Pixhawk + F9P Rover GPS) ---
            Node(
                package="mavros",
                executable="mavros_node",
                namespace="pixhawk",
                output="screen",
                parameters=[
                    {
                        "fcu_url": fcu_url,
                    }
                ],
            ),
            # --- GPS RTK node (standalone script; rclpy node under /gps_rtk_f9p_helical) ---
            ExecuteProcess(
                cmd=[
                    sys.executable,
                    str(gps_rtk_script),
                    "--ros-args",
                    "-r", "__ns:=/gps_rtk_f9p_helical",
                ],
                output="screen",
                respawn=True,
                respawn_delay=2.0,
            ),
        ]
    )
