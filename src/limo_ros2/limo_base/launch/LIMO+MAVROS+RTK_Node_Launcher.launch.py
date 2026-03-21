import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_repo_file_upwards(start: Path, filename: str, max_depth: int = 12) -> Path:
    """
    Find a file by walking up parent directories.

    Works both when this launch file is executed from the source tree and when it is
    executed from the colcon install space (via `ros2 launch`).
    """
    cur = start.resolve()
    for _ in range(max_depth + 1):
        candidate = cur / filename
        if candidate.exists():
            return candidate
        if cur.parent == cur:
            break
        cur = cur.parent
    raise RuntimeError(
        f"Could not find '{filename}' by searching parent directories from '{start}'. "
        f"Expected it in the workspace root. If you moved/renamed it, update this launch file."
    )


def generate_launch_description():
    # --- Launch arguments ---
    port_name = LaunchConfiguration("port_name")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    pub_odom_tf = LaunchConfiguration("pub_odom_tf")
    fcu_url = LaunchConfiguration("fcu_url")

    gps_rtk_script = _find_repo_file_upwards(Path(__file__).parent, "GPS-RTK_ROS2_pub_node.py")

    return LaunchDescription(
        [
            # --- LIMO base args (mirrors limo_with_mavros.launch.py) ---
            DeclareLaunchArgument(
                "port_name",
                default_value="limo_base",
                description="USB device name for LIMO base (e.g. ttyUSB1 or a udev symlink like limo_base)",
            ),
            DeclareLaunchArgument(
                "odom_frame",
                default_value="odom",
                description="Odometry frame id",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_link",
                description="Base link frame id",
            ),
            DeclareLaunchArgument(
                "pub_odom_tf",
                default_value="true",
                description="Whether to publish odom -> base_link TF",
            ),
            # --- Pixhawk / MAVROS arg ---
            DeclareLaunchArgument(
                "fcu_url",
                # default_value='serial:///dev/ttyACM1:115200',
                default_value="serial:///dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00:115200",
                description="Pixhawk FCU connection URL for MAVROS",
            ),
            # --- LIMO base driver node ---
            Node(
                package="limo_base",
                executable="limo_base",
                name="limo_base_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "port_name": port_name,
                        "odom_frame": odom_frame,
                        "base_frame": base_frame,
                        "pub_odom_tf": pub_odom_tf,
                        "use_mcnamu": False,
                    }
                ],
                remappings=[
                    ("odom", "/wheel/odom"),
                ],
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
            # --- GPS RTK node (standalone script) ---
            #
            # This script is not a ROS2 "package executable" in the ament index, so we run it as a process.
            # It still creates an rclpy Node and publishes to:
            #   - /gps/fix
            #   - /gps/nmea
            #   - /gps/rtk_status
            ExecuteProcess(
                cmd=[
                    sys.executable,
                    str(gps_rtk_script),
                    "--ros-args",
                    "-r", "__ns:=/gps_rtk_f9p_helical",
                ],
                output="screen",
            ),
        ]
    )