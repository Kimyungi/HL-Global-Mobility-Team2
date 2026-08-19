"""No-CAN bench launcher for the experimental generated MGM backend."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('adas_mgm'), 'config', 'params.yaml')
    backend = LaunchConfiguration('backend')
    acknowledge = LaunchConfiguration(
        'generated_backend_acknowledge_limited_scope')

    return LaunchDescription([
        DeclareLaunchArgument(
            'backend',
            default_value='core',
            description='MGM backend: core or generated (bench only)'),
        DeclareLaunchArgument(
            'generated_backend_acknowledge_limited_scope',
            default_value='false',
            description=(
                'Must be true to run the limited two-state generated backend')),
        Node(
            package='adas_mgm',
            executable='mgm_node',
            remappings=[('/adas/target_ref', '/bench/adas/target_ref')],
            parameters=[
                params,
                {
                    'backend': ParameterValue(backend, value_type=str),
                    'generated_backend_acknowledge_limited_scope': ParameterValue(
                        acknowledge, value_type=bool),
                },
            ],
            output='screen',
        ),
    ])
