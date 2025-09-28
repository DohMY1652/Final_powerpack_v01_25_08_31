import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('topic_csv_converter'),
        'config',
        'topics_wide.yaml' # 새 설정 파일을 가리킴
    )

    return LaunchDescription([
        Node(
            package='topic_csv_converter',
            executable='wide_logger', # 새 실행 파일 이름
            name='wide_logger_node',
            parameters=[config],
            output='screen'
        )
    ])