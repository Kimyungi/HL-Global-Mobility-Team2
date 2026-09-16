"""Perception/planning only; uses the running GPS node's map->base_link TF."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('waypoint_csv'),
        DeclareLaunchArgument('cloud_topic', default_value='/unified_lidar/cloud'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('stack_avoid'), 'config', 'waypoint_avoid.yaml')),
        Node(package='stack_avoid', executable='waypoint_avoid_node', name='stack_avoid_node',
             parameters=[LaunchConfiguration('params_file'),
                         {'waypoint_csv': LaunchConfiguration('waypoint_csv'),
                          'cloud_topic': LaunchConfiguration('cloud_topic')}], output='screen'),
    ])
