"""Start four YDLiDAR drivers, the v2 unifier, and optionally RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from lidar_fusion_v2.driver_profiles import DEFAULT_PORTS, SENSOR_IDS


def generate_launch_description():
    share = get_package_share_directory('lidar_fusion_v2')
    arguments = [
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                share, 'config', 'fixed_geometry.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]
    driver_arguments = {}
    for sensor_id in SENSOR_IDS:
        port_name = f'{sensor_id}_port'
        enable_name = f'enable_{sensor_id}'
        arguments.extend([
            DeclareLaunchArgument(
                port_name, default_value=DEFAULT_PORTS[sensor_id]),
            DeclareLaunchArgument(enable_name, default_value='true'),
        ])
        driver_arguments[port_name] = LaunchConfiguration(port_name)
        driver_arguments[enable_name] = LaunchConfiguration(enable_name)

    drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'drivers.launch.py')),
        launch_arguments=driver_arguments.items(),
    )
    fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'fusion_v2.launch.py')),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )
    return LaunchDescription(arguments + [drivers, fusion])
