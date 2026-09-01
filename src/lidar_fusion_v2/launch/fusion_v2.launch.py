import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lidar_fusion_v2')
    params = os.path.join(share, 'config', 'fixed_geometry.yaml')
    rviz = os.path.join(share, 'rviz', 'fusion_v2.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='lidar_fusion_v2', executable='fusion_node',
             name='unified_lidar_v2', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
        Node(package='rviz2', executable='rviz2', name='rviz_fusion_v2',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', rviz], output='screen'),
    ])
