from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('lora_rover')
    params_file = os.path.join(pkg_share, 'config', 'rover_lora.yaml')

    return LaunchDescription([
        Node(
            package='lora_rover',
            executable='lora_rover_duplex',
            output='screen',
            parameters=[params_file],
        )
    ])
