# path: powerpack_controller/launch/control.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('powerpack_controller')
    
    default_config_path = os.path.join(pkg_share, 'config', 'Config.yaml') 

    teensy_bridge_node = Node(
        package='powerpack_controller',
        executable='teensy_bridge_node',
        output='screen',
        parameters=[default_config_path],
    )

    controller_node = Node(
        package='powerpack_controller',
        executable='controller_node',
        output='screen',
        parameters=[default_config_path],
    )

    return LaunchDescription([
        teensy_bridge_node,
        controller_node,
    ])