"""Run the parking stack, optionally bringing up the v2 four-LiDAR stack.

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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('stack_parking'), 'config', 'parking_params.yaml'])
    fusion_share = FindPackageShare('lidar_fusion_v2')
    merged_cloud_topic = '/parking/nearest_merged_cloud'
    start_multi = LaunchConfiguration('start_multi_lidar')
    return LaunchDescription([
        DeclareLaunchArgument('t_reference_enabled', default_value='false'),
        DeclareLaunchArgument('t_reference_origin_csv', default_value=''),
        DeclareLaunchArgument('t_reference_route_csv', default_value=''),
        DeclareLaunchArgument('t_reference_reverse_1_csv', default_value=''),
        DeclareLaunchArgument('t_reference_reverse_2_csv', default_value=''),
        DeclareLaunchArgument(
            'start_multi_lidar', default_value='false',
            description=(
                'Start the v2 four-driver profiles and lidar_fusion_v2; '
                'false if already running')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                fusion_share, 'launch', 'drivers.launch.py'])),
            condition=IfCondition(start_multi)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                fusion_share, 'launch', 'fusion_v2.launch.py'])),
            launch_arguments={'rviz': 'false'}.items(),
            condition=IfCondition(start_multi)),
        Node(
            package='stack_parking',
            executable='scan_to_cloud',
            name='parking_scan_to_cloud',
            output='screen',
            parameters=[{
                'input_scan_topic': '/unified_lidar/scan',
                'output_cloud_topic': merged_cloud_topic,
            }],
        ),
        Node(
            package='stack_parking',
            executable='stack_parking_node',
            name='stack_parking_node',
            output='screen',
            parameters=[params, {
                'merged_cloud_topic': merged_cloud_topic,
                'auto_trigger_gps_zone': False,
                'manual_test_publish_gps_gate': False,
                't_reference_enabled': ParameterValue(LaunchConfiguration('t_reference_enabled'), value_type=bool),
                't_reference_origin_csv': ParameterValue(LaunchConfiguration('t_reference_origin_csv'), value_type=str),
                't_reference_route_csv': ParameterValue(LaunchConfiguration('t_reference_route_csv'), value_type=str),
                't_reference_reverse_1_csv': ParameterValue(LaunchConfiguration('t_reference_reverse_1_csv'), value_type=str),
                't_reference_reverse_2_csv': ParameterValue(LaunchConfiguration('t_reference_reverse_2_csv'), value_type=str),
            }],
        ),
        Node(
            package='stack_parking',
            executable='parking_lateral_wall',
            name='parking_lateral_wall',
            output='screen',
        ),
    ])
