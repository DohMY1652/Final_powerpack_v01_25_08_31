from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('powerpack_controller')
    
    # 1번 전용 설정 파일 경로
    config_path = os.path.join(pkg_share, 'config', '1st_powerpack_config.yaml') 

    return LaunchDescription([
        Node(
            package='powerpack_controller',
            executable='teensy_bridge_node',
            name='teensy_bridge', # 노드 이름 명시
            namespace='pack1',    # 네임스페이스 지정
            output='screen',
            parameters=[config_path],
        ),
        Node(
            package='powerpack_controller',
            executable='controller_node',
            name='pp_controller', # YAML의 키값과 일치해야 함
            namespace='pack1',    # 네임스페이스 지정
            output='screen',
            parameters=[config_path],
        ),
    ])