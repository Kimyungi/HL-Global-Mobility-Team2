"""adas_mgm 10ms 루프 — 인지 노드와 별도 프로세스로 실행할 것 (CLAUDE.md §5.2)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('adas_mgm'), 'config', 'params.yaml')
    return LaunchDescription([
        Node(
            package='adas_mgm',
            executable='mgm_node',
            parameters=[params],
            output='screen',
        ),
    ])
