import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'powerpack_controller'
    pkg_share = get_package_share_directory(pkg_name)
    
    # 설정 파일 경로 정의
    config_path_1 = os.path.join(pkg_share, 'config', '1st_powerpack_config.yaml') 
    config_path_2 = os.path.join(pkg_share, 'config', '2nd_powerpack_config.yaml') 

    return LaunchDescription([
        # =======================================================
        # [Pack 1] Teensy Bridge + Controller (Namespace: pack1)
        # =======================================================
        Node(
            package=pkg_name,
            executable='teensy_bridge_node',
            name='teensy_bridge',
            namespace='pack1',
            output='screen',
            parameters=[config_path_1],
        ),
        Node(
            package=pkg_name,
            executable='controller_node',
            name='pp_controller',
            namespace='pack1',
            output='screen',
            parameters=[config_path_1],
        ),

        # =======================================================
        # [Pack 2] CAN Bridge + Controller (Namespace: pack2)
        # =======================================================
        Node(
            package=pkg_name,
            executable='can_bridge_node',
            name='can_bridge',
            namespace='pack2',
            output='screen',
            parameters=[config_path_2],
        ),
        Node(
            package=pkg_name,
            executable='controller_node',
            name='pp_controller',
            namespace='pack2',
            output='screen',
            parameters=[config_path_2],
        ),

        # =======================================================
        # [Dashboard] GUI 모니터링 노드 (Global Scope)
        # =======================================================
        # 주의: setup.py의 entry_points에 'dashboard'가 등록되어 있어야 함
        Node(
            package=pkg_name,
            executable='dashboard', 
            name='pressure_dashboard_gui',
            output='screen'
        ),
    ])