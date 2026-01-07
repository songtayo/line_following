import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control',          # 패키지 이름
            executable='cam_viewer', # setup.py에 정의한 entry_point 이름
            name='lane_follower_node',  # 실행될 노드의 이름
            output='screen',            # 터미널에 로그(print) 출력
            parameters=[{
                # 'linear_speed': 0.1,    # 필요 시 파라미터로 관리 가능
                # 'p_gain_dual': 0.01
            }]
        ),
        # 여기에 다른 패키지의 노드도 추가할 수 있습니다.
        # 예: 신호등 인식 노드
        # Node(
        #     package='traffic',
        #     executable='traffic_node',
        #     name='traffic_node'
        # )
    ])
