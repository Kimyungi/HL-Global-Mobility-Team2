"""Safe E-Stop perception/MGM chain without any vehicle command bridge.

Included processes:
  - external YDLIDAR driver + its base_link -> laser_frame static TF
  - Team2-1 stack_estop_node
  - Team2-1 adas_mgm mgm_node

Intentionally excluded:
  - bridge_dspace / can_bridge_node
  - dummy_ref_publisher / dspace_sim_node
  - test_mgm_inputs.py
  - any UDP, CAN, or vehicle-control node
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_ydlidar_ws = os.path.join(os.path.expanduser('~'), 'ydlidar_ws')
    default_ydlidar_launch = os.path.join(
        default_ydlidar_ws, 'src', 'ydlidar_ros2_driver', 'launch',
        'ydlidar_launch.py')
    default_ydlidar_params = os.path.join(
        default_ydlidar_ws, 'src', 'ydlidar_ros2_driver', 'params',
        'Tmini-Plus-SH.yaml')

    # The external launch contains only the YDLIDAR lifecycle node and its
    # base_link -> laser_frame static TF. It does not contain bridge_dspace.
    ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(LaunchConfiguration('ydlidar_launch')),
        launch_arguments={
            'params_file': LaunchConfiguration('ydlidar_params')
        }.items(),
    )

    mgm_launch = os.path.join(
        get_package_share_directory('adas_mgm'),
        'launch',
        'mgm.launch.py',
    )

    # This is the official Team2-1 E-Stop node. It publishes only
    # /perception/estop and never creates a CAN/UDP/vehicle command.
    stack_estop = Node(
        package='stack_estop',
        executable='stack_estop_node',
        name='stack_estop_node',
        output='screen',
    )

    # Existing MGM launch: publishes /adas/target_ref, but no bridge is
    # included here, so the target ref cannot become a CAN transmission.
    mgm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mgm_launch),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ydlidar_launch', default_value=default_ydlidar_launch,
            description='Path to the external YDLIDAR launch file.'),
        DeclareLaunchArgument(
            'ydlidar_params', default_value=default_ydlidar_params,
            description='Path to the external YDLIDAR parameter YAML.'),
        ydlidar,
        stack_estop,
        mgm,
    ])
