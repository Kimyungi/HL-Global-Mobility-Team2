"""multi_lidar_fusion 메인 launch.

이 파일의 역할:
    융합 노드 하나를 두 YAML(lidar_extrinsics.yaml, fusion_params.yaml)과 함께 띄운다.
    센서 드라이버는 별도 launch(multi_lidar_drivers.launch.py)로 분리했다 —
    드라이버가 죽어도 융합 노드는 살아 있어야 하고(요구 §20), rosbag 재생 시에는
    드라이버 없이 융합만 돌려야 하기 때문이다.

실행:
    ros2 launch multi_lidar_fusion multi_lidar_fusion.launch.py
    ros2 launch multi_lidar_fusion multi_lidar_fusion.launch.py sim:=true rviz:=true
    ros2 launch multi_lidar_fusion multi_lidar_fusion.launch.py \
        params_file:=/path/to/my_fusion_params.yaml

인자:
    extrinsics_file  센서 정의·장착 위치·FOV      (기본: config/lidar_extrinsics.yaml)
    params_file      융합 알고리즘 파라미터        (기본: config/fusion_params.yaml)
    sim              합성 라이다 4대를 함께 띄움   (기본: false)
    sim_params_file  시뮬레이터 파라미터           (기본: config/sim_lidars.yaml)
    rviz             RViz2 동시 실행               (기본: false)
    log_level        노드 로그 레벨                (기본: info)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('multi_lidar_fusion')

    extrinsics_file = LaunchConfiguration('extrinsics_file')
    params_file = LaunchConfiguration('params_file')
    sim_params_file = LaunchConfiguration('sim_params_file')
    sim = LaunchConfiguration('sim')
    rviz = LaunchConfiguration('rviz')
    log_level = LaunchConfiguration('log_level')

    args = [
        DeclareLaunchArgument(
            'extrinsics_file',
            default_value=os.path.join(pkg, 'config', 'lidar_extrinsics.yaml'),
            description='센서 정의 + 장착 위치(extrinsic) + FOV'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'fusion_params.yaml'),
            description='동기화/보상/필터/가상스캔 파라미터'),
        DeclareLaunchArgument(
            'sim_params_file',
            default_value=os.path.join(pkg, 'config', 'sim_lidars.yaml'),
            description='합성 라이다 시뮬레이터 파라미터'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='실 센서 없이 합성 라이다 4대로 검증'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='RViz2 동시 실행'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    fusion_node = Node(
        package='multi_lidar_fusion',
        executable='multi_lidar_fusion_node',
        name='multi_lidar_fusion',
        output='screen',
        emulate_tty=True,
        # 순서가 중요하다: 뒤 파일이 앞 파일을 덮어쓴다.
        parameters=[extrinsics_file, params_file],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # 시뮬레이터에도 같은 extrinsics 를 먹인다 — 합성 데이터와 융합 노드가
    # 같은 장착 위치를 보게 해서, 어긋남이 보이면 그건 순수하게 알고리즘 문제다.
    sim_node = Node(
        package='multi_lidar_fusion',
        executable='test_scan_publisher',
        name='test_scan_publisher',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(sim),
        parameters=[extrinsics_file, sim_params_file],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', os.path.join(pkg, 'rviz', 'multi_lidar.rviz')],
    )

    return LaunchDescription(args + [fusion_node, sim_node, rviz_node])
