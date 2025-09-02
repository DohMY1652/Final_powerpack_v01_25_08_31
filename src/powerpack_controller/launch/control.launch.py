# path: powerpack_controller/launch/control.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('powerpack_controller')
    default_config_path = os.path.join(pkg_share, 'config', 'Config.yaml')

    # 두 개의 노드가 하나의 실행 파일에 포함되어 있으므로,
    # 런치 파일에서 name을 지정하지 않아야 각 노드가 자신의 이름을 사용할 수 있습니다.
    # 파라미터 파일은 YAML 내부에 지정된 'pp_controller' 노드에만 적용됩니다.
    main_executable_node = Node(
        package='powerpack_controller',
        executable='controller',
        output='screen',
        parameters=[default_config_path],
        # [수정됨] 이 줄을 삭제(또는 주석 처리)하여 이름 강제 지정을 해제합니다.
        # name='pp_controller',
    )

    return LaunchDescription([
        main_executable_node
    ])