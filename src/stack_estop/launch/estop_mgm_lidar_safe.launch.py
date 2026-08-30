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

from ament_index_python.packages import (
    PackageNotFoundError, get_package_share_directory)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def ydlidar_file(*parts):
    """`ydlidar_ros2_driver` 안의 파일 경로 — **저장소 설치본이 1순위**.

    2026-08-29: 기본값이 `~/ydlidar_ws/src/...` 로 박혀 있었는데 이 PC 의 실제
    워크스페이스는 `~/ydlidar_ros2_ws` 라 파일이 없었다. 없으면 드라이버가 뜨자마자
    죽고 `/scan` 이 0Hz 가 되는데 증상은 "go 가 안 통과한다"로만 보인다.
    저장소가 드라이버를 직접 갖고 있으므로 설치본을 기본값으로 쓰고, 외부
    워크스페이스는 옛 세팅 호환용 폴백으로만 남긴다.
    """
    cands = []
    try:
        cands.append(os.path.join(
            get_package_share_directory('ydlidar_ros2_driver'), *parts))
    except PackageNotFoundError:
        pass
    home = os.path.expanduser('~')
    cands += [os.path.join(home, ws, 'src', 'ydlidar_ros2_driver', *parts)
              for ws in ('ydlidar_ros2_ws', 'ydlidar_ws')]
    return next((p for p in cands if os.path.isfile(p)), cands[0])


def generate_launch_description():
    default_ydlidar_launch = ydlidar_file(
        'launch', 'ydlidar_launch.py')
    default_ydlidar_params = ydlidar_file(
        'params', 'Tmini-Plus-SH.yaml')

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
