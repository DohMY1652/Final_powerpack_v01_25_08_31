from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='powerpack_controller',
            executable='controller',
            name='controller',
            output='screen',
        ),
    ])
