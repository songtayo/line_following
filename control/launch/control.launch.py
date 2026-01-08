import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 노드 설정
    lane_follower_node = Node(
        package='control',                # 실제 패키지 이름
        executable='control_node_exe',    # setup.py의 console_scripts에 적은 이름
        name='lane_follower_node',        # 노드 이름
        output='screen',
        parameters=[{
            'linear_speed': 0.1,          # 필요 시 속도 파라미터를 밖으로 뺄 수 있습니다
            'p_gain_dual': 0.01
        }]
    )

    # 2. LaunchDescription에 노드 추가
    return LaunchDescription([
        lane_follower_node
    ])
