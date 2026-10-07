from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Launch arguments ---
    port_name = LaunchConfiguration('port_name')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    pub_odom_tf = LaunchConfiguration('pub_odom_tf')
    fcu_url = LaunchConfiguration('fcu_url')

    return LaunchDescription([
        # LIMO base args
        DeclareLaunchArgument(
            'port_name',
            default_value='limo_base',
            description='USB device name for LIMO base (e.g. ttyUSB1 or a udev symlink like limo_base)'
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Odometry frame id'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Base link frame id'
        ),
        DeclareLaunchArgument(
            'pub_odom_tf',
            default_value='true',
            description='Whether to publish odom -> base_link TF'
        ),
        # Pixhawk / MAVROS arg
        DeclareLaunchArgument(
            'fcu_url',
            # default_value='serial:///dev/ttyACM1:115200',
            default_value='serial:///dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00:115200',
            description='Pixhawk FCU connection URL for MAVROS'
        ),

        # --- LIMO base driver node ---
        Node(
            package='limo_base',
            executable='limo_base',   # for foxy/galactic/humble this is correct
            name='limo_base_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'port_name': port_name,         # e.g. "ttyUSB1"
                'odom_frame': odom_frame,       # e.g. "odom"
                'base_frame': base_frame,       # e.g. "base_link"
                'pub_odom_tf': pub_odom_tf,     # "true"/"false"
                'use_mcnamu': False
            }],
            remappings=[
                ('odom', '/wheel/odom'),
            ]
        ),

        # --- MAVROS node (Pixhawk + F9P Rover GPS) ---
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='pixhawk',
            output='screen',
            parameters=[{
                'fcu_url': fcu_url,
                # you can add more MAVROS params here later if needed
                # 'gcs_url': '',
                # 'target_system_id': 1,
            }]
        ),
    ])

