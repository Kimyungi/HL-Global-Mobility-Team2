"""GPS-free parking bench launch with an isolated synthetic MGM parking gate.

Do not run this launch together with stack_gps: both would publish GpsPath.
The vehicle may be moved manually while SCANNING, or an existing lane source
may keep it straight. The parking mission starts with /parking/manual_command.
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
            description='Start four LiDAR drivers and fusion if not already running'),
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
            parameters=[params, {
                'auto_trigger_gps_zone': False,
                'manual_test_publish_gps_gate': True,
            }],
        ),
    ])
