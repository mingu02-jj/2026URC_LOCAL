from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('urc_node')
    mux_yaml = os.path.join(pkg_share, 'config', 'twist_mux.yaml')

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[mux_yaml],
        remappings=[
            ('/cmd_vel_out', '/cmd_vel_mux'),
        ],
    )

    urc_drive_node = Node(
        package='urc_node',
        executable='urc_drive_controller',
        name='urc_drive_controller',
        output='screen',
        parameters=[{
            'cmd_vel_topic': '/cmd_vel_mux',
            'serial_port': '/dev/portenta',
            'baudrate': 115200,
        }],
    )

    return LaunchDescription([
        twist_mux_node,
        urc_drive_node,
    ])
