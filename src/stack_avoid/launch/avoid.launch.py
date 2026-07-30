"""stack_avoid 실행 — config/params.yaml 로드 + 노드 기동.

LiDAR 드라이버는 별도로 실행한다(다른 워크스페이스). 이 노드가
base_link→laser_frame static TF를 params 기준으로 발행하므로,
벤더 launch의 플레이스홀더 TF와 겹치지 않도록 드라이버는 노드로 실행 권장:

  ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args \
      --params-file $HOME/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/Tmini.yaml

파라미터 파일 오버라이드:
  ros2 launch stack_avoid avoid.launch.py params_file:=/path/to/other.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('stack_avoid'), 'config', 'params.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='stack_avoid 파라미터 YAML 경로'),
        Node(
            package='stack_avoid',
            executable='stack_avoid_node',
            name='stack_avoid_node',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),
    ])
