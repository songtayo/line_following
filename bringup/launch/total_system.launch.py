import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. 각 패키지의 launch 파일 경로 설정
    # (주의: 패키지 이름이 다르면 실제 이름으로 수정하세요)
    
    obstacle_launch_path = os.path.join(
        get_package_share_directory('obstacle'),
        'launch',
        'obstacle.launch.py'
    )
    
    traffic_launch_path = os.path.join(
        get_package_share_directory('traffic'),
        'launch',
        'traffic.launch.py'
    )
    
    control_launch_path = os.path.join(
        get_package_share_directory('control'),
        'launch',
        'control.launch.py'
    )

    # 2. 각 launch 파일 포함시키기
    obstacle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(obstacle_launch_path)
    )

    traffic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(traffic_launch_path)
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch_path)
    )

    # 3. 모든 노드를 동시에 실행
    return LaunchDescription([
        obstacle_launch,
        traffic_launch,
        control_launch
    ])
