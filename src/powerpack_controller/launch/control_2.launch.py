from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('powerpack_controller')
    config_path = os.path.join(pkg_share, 'config', '2nd_powerpack_config.yaml') 

    # 2번 세트는 CAN 브릿지를 사용
    can_bridge_node = Node(
        package='powerpack_controller', # CAN 브릿지가 있는 패키지 이름
        executable='can_bridge_node',   # CAN 브릿지 실행 파일 이름
        name='can_bridge',
        namespace='pack2',
        output='screen',
        parameters=[config_path],
    )

    controller_node = Node(
        package='powerpack_controller',
        executable='controller_node',
        name='pp_controller',
        namespace='pack2',
        output='screen',
        parameters=[config_path],
    )

    return LaunchDescription([
        can_bridge_node,
        controller_node,
    ])