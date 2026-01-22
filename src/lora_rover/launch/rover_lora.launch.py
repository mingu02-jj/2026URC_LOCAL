from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lora_rover',
            executable='lora_rover_rx',
            name='lora_rover_rx',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baudrate': 115200,
                'cmd_vel_out': '/cmd_vel_lora',
                'estop_out': '/estop_lora',
                'failsafe_stop_after_sec': 0.7,
            }]
        )
    ])
