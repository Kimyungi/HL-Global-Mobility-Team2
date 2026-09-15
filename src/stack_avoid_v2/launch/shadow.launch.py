"""Observe existing sensors. Starts no drivers, MGM, CAN bridge or operator/go."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('stack_avoid_v2'))
    geometry_file = Path(get_package_share_directory('lidar_fusion_v2')) / 'config/fixed_geometry.yaml'
    geometry = yaml.safe_load(geometry_file.read_text())['/**']['ros__parameters']
    settings_file = share / 'config/shadow.yaml'
    settings = yaml.safe_load(settings_file.read_text())['avoid_v2_node']['ros__parameters']
    # Select avoidance inputs independently of the shared four-sensor fusion.
    params = {'sensor_ids': settings['sensor_ids']}
    for sensor_id in settings['sensor_ids']:
        for key, value in geometry['sensors'][sensor_id].items():
            params[f'sensors.{sensor_id}.{key}'] = value
    nodes = [Node(package='stack_avoid_v2', executable='avoid_v2_node',
                  parameters=[str(settings_file), params], output='screen'),
             Node(package='stack_avoid_v2', executable='watchdog.py', output='screen')]
    course = LaunchConfiguration('course_file').perform(context)
    if course:
        nodes.append(Node(package='stack_avoid_v2', executable='publish_course.py',
                          arguments=[course], output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('course_file', default_value='',
                              description='Explicit JSON course; absent means HOLD'),
        OpaqueFunction(function=setup),
    ])
