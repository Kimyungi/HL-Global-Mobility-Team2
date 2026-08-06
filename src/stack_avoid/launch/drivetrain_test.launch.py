"""구동계 검증 (dummy ref → CAN → dSPACE) + 로깅. ★실차 구동 — 안전 주의★

MGM 없이 dummy_ref_publisher가 직진 ref + 고정 v_ref를 /adas/target_ref로 10ms 발행 →
bridge_dspace가 CAN(can0)으로 송신 → dSPACE가 바퀴/조향 구동. RX(/vehicle/vector) 회신 로깅.

  ros2 launch stack_avoid drivetrain_test.launch.py v_ref:=0.2 curvature:=0.0 bag_dir:=/경로/bag
※ v_ref 낮게 시작(0.2). curvature 0=직진, 0.3 정도면 완만한 좌선회.
※ 바퀴 들고(스탠드) 먼저, 물리 비상정지 준비, 주변 통제.
"""
import os

from ament_index_python.packages import get_package_share_directory  # noqa: F401
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    v_ref = LaunchConfiguration('v_ref')
    curvature = LaunchConfiguration('curvature')
    can_if = LaunchConfiguration('can_interface')
    bag_dir = LaunchConfiguration('bag_dir')
    use_can = LaunchConfiguration('can')

    return LaunchDescription([
        DeclareLaunchArgument('v_ref', default_value='0.2',
                              description='목표 속도 [m/s] — 낮게 시작'),
        DeclareLaunchArgument('curvature', default_value='0.0',
                              description='곡률 [1/m] (0=직진, +좌선회)'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('can', default_value='true'),
        DeclareLaunchArgument('bag_dir', default_value='drivetrain_bag'),

        # CAN 브리지: /adas/target_ref → CAN TX, dSPACE RX → /vehicle/vector
        Node(package='bridge_dspace', executable='can_bridge_node', name='can_bridge_node',
             output='screen', parameters=[{'can_interface': can_if}],
             condition=IfCondition(use_can)),

        # dummy ref: 직진 ref + 고정 v_ref (10ms). MGM 대체(부트스트랩용).
        Node(package='bridge_dspace', executable='dummy_ref_publisher', name='dummy_ref_publisher',
             output='screen',
             parameters=[{'v_ref': v_ref, 'curvature': curvature,
                          'n_points': 1, 'period_ms': 10}]),

        # 로그: 보낸 명령(TX) + dSPACE 상태회신(RX) + 노드로그
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_dir,
                 '/adas/target_ref', '/vehicle/vector', '/rosout'],
            output='screen'),
    ])
