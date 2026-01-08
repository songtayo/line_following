import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 노드 설정
    obstacle_detection_node = Node(
        package='obstacle',       # 본인의 패키지 이름
        executable='obstacle_node_exe',   # setup.py의 console_scripts에 적은 이름
        name='obstacle_detection_node',   # 실행 시 부여할 노드 이름
        output='screen',                  # 로그를 터미널에 출력
        parameters=[{
            'detection_range': 0.3        # 필요 시 코드 내 변수를 외부 파라미터로 관리 가능
        }]
    )

    # 2. LaunchDescription에 노드 추가
    return LaunchDescription([
        obstacle_detection_node
    ])
