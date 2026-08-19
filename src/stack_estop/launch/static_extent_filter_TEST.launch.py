"""TEST ONLY: synthetic end-to-end validation of static extent filtering."""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    package_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    test_tools = os.path.join(package_root, 'test_tools')
    return LaunchDescription([
        LogInfo(msg=(
            'STATIC EXTENT FILTER TEST ONLY: synthetic /scan; '
            'no YDLIDAR, recovery, MGM, CAN, or vehicle nodes')),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_extent_filter_test_laser_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '1.57079632679',
                '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'],
            output='screen'),
        Node(
            package='stack_estop',
            executable='stack_estop_node',
            name='stack_estop_node',
            parameters=[{
                'static_min_obstacle_extent_m': 0.07,
                'dynamic_enabled': False,
                'laser_yaw_in_base_rad': 1.57079632679,
            }],
            output='screen'),
        ExecuteProcess(
            cmd=['python3', os.path.join(
                test_tools, 'static_extent_filter_validator.py')],
            output='screen'),
        ExecuteProcess(
            cmd=['python3', os.path.join(
                test_tools, 'static_extent_filter_synthetic_source.py')],
            output='screen'),
    ])
