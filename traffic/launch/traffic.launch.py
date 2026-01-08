import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 노드 설정
    traffic_signal_node = Node(
        package='traffic',           # 본인의 패키지 이름
        executable='traffic_node_exe',       # setup.py에 등록할 이름
        name='traffic_signal_publisher',     # 노드 이름
        output='screen',                     # 로그 출력
        parameters=[{
            'red_threshold': 500,            # 빨간색 픽셀 임계값 (예시)
            'blue_threshold': 1500,          # 파란색 픽셀 임계값 (예시)
        }]
    )

    # 2. LaunchDescription에 노드 추가
    return LaunchDescription([
        traffic_signal_node
    ])
