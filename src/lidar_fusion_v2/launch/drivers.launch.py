"""Start the four field-verified YDLiDAR drivers, without any fusion node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from lidar_fusion_v2.driver_profiles import DEFAULT_PORTS
from lidar_fusion_v2.driver_profiles import SENSOR_IDS
from lidar_fusion_v2.driver_profiles import parameters


def _driver(sensor_id):
    return Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name=f'ydlidar_{sensor_id}',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration(f'enable_{sensor_id}')),
        parameters=[parameters(
            sensor_id, LaunchConfiguration(f'{sensor_id}_port'))],
        remappings=[('/scan', f'/lidar/{sensor_id}/scan')],
    )


def generate_launch_description():
    arguments = []
    for sensor_id in SENSOR_IDS:
        arguments.extend([
            DeclareLaunchArgument(
                f'{sensor_id}_port', default_value=DEFAULT_PORTS[sensor_id],
                description=f'{sensor_id} udev device link'),
            DeclareLaunchArgument(
                f'enable_{sensor_id}', default_value='true',
                description=f'start {sensor_id} driver'),
        ])
    return LaunchDescription(arguments + [_driver(sid) for sid in SENSOR_IDS])
