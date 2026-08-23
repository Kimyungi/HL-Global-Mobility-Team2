"""Run the parking stack, optionally bringing up the existing four-LiDAR stack.

Examples:
  ros2 launch stack_parking parking.launch.py
  ros2 launch stack_parking parking.launch.py start_multi_lidar:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('stack_parking'), 'config', 'parking_params.yaml'])
    multi_share = FindPackageShare('multi_lidar_fusion')
    start_multi = LaunchConfiguration('start_multi_lidar')
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_multi_lidar', default_value='false',
            description=(
                'Start existing four drivers and multi_lidar_fusion; '
                'false if already running')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                multi_share, 'launch', 'multi_lidar_drivers.launch.py'])),
            condition=IfCondition(start_multi)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                multi_share, 'launch', 'multi_lidar_fusion.launch.py'])),
            condition=IfCondition(start_multi)),
        Node(
            package='stack_parking',
            executable='stack_parking_node',
            name='stack_parking_node',
            output='screen',
            parameters=[params],
        ),
    ])
