# path: powerpack_controller/launch/control.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지의 공유 디렉토리 경로를 찾습니다.
    pkg_share = get_package_share_directory('powerpack_controller')

    # 공유 디렉토리 안의 설정 파일 경로를 지정합니다.
    # 이 방식이 훨씬 더 안정적입니다.
    default_config_path = os.path.join(pkg_share, 'config', 'Config.yaml')

    controller_node = Node(
        package='powerpack_controller',
        executable='controller',
        name='pp_controller',  # 코드와 일관성을 위해 노드 이름을 명시적으로 지정
        output='screen',
        parameters=[default_config_path],
    )

    return LaunchDescription([
        controller_node
    ])