import os
from glob import glob
from setuptools import setup

package_name = 'topic_csv_converter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 파일들을 설치 경로에 포함
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Config 파일들을 설치 경로에 포함
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',  # 사용자 이름으로 변경하세요
    maintainer_email='your_email@email.com',  # 이메일로 변경하세요
    description='A ROS2 package to subscribe to topics and save data to CSV files in various formats.',
    license='Apache License 2.0', # 원하는 라이선스로 변경 가능
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 1. 단일 토픽 -> 단일 CSV
            'topic_to_csv = topic_csv_converter.topic_to_csv:main',
            # 2. 다중 토픽 -> 다중 CSV (사용 중단, multi_topic_to_csv로 대체됨)
            'multi_topic_to_csv = topic_csv_converter.multi_topic_to_csv:main',
            # 3. 다중 토픽 -> 단일 통합 CSV (Long Format)
            'unified_logger = topic_csv_converter.unified_logger:main',
            # 4. 다중 토픽 -> 단일 통합 CSV (Wide Format, 동기화)
            'wide_logger = topic_csv_converter.wide_logger:main',
        ],
    },
)