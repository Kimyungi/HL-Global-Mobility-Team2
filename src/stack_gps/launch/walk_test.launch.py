"""도보 검증 한 방 실행 — 노드 + 횡오차 CSV + rosbag 기록을 전부 자동으로.

  ros2 launch stack_gps walk_test.launch.py                  # 최신 트랙 CSV 자동 선택
  ros2 launch stack_gps walk_test.launch.py rtcm_host:=...   # 필요 시 재정의

기록물 (src/stack_gps/logs/, git 제외):
  errlog_<시각>.csv — 매 100ms: 위치·품질·최근접 idx·횡오차·fix age
  bag_<시각>/       — /perception/gps_path(ref points), /perception/gps_fix(새 측정만
                      발행 → 메시지 간격이 곧 GPS 갱신 주기)
"""
import glob
import os
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_src = os.path.expanduser('~/FMA_ws/src/stack_gps')
    csvs = sorted(glob.glob(os.path.join(pkg_src, 'waypoints', 'waypoints_*.csv')))
    if not csvs:
        raise RuntimeError('waypoints/*.csv 없음 — 먼저 record_waypoints.py로 기록할 것')
    logdir = os.path.join(pkg_src, 'logs')
    os.makedirs(logdir, exist_ok=True)
    stamp = time.strftime('%m%d_%H%M%S')

    return LaunchDescription([
        DeclareLaunchArgument('waypoint_csv', default_value=csvs[-1]),
        DeclareLaunchArgument('rtcm_host', default_value='100.70.198.29'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyRover'),
        Node(
            package='stack_gps', executable='stack_gps_node', output='screen',
            parameters=[{
                # str 강제 — YAML이 'off'/'no' 등을 불리언으로 바꾸는 것 방지
                'waypoint_csv': ParameterValue(
                    LaunchConfiguration('waypoint_csv'), value_type=str),
                'rtcm_host': ParameterValue(
                    LaunchConfiguration('rtcm_host'), value_type=str),
                'serial_port': ParameterValue(
                    LaunchConfiguration('serial_port'), value_type=str),
                'error_log_csv': os.path.join(logdir, f'errlog_{stamp}.csv'),
            }],
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', os.path.join(logdir, f'bag_{stamp}'),
                 '/perception/gps_path', '/perception/gps_fix'],
            output='log',
        ),
    ])
