"""라이다 회피 통합 확인 — RViz(스캔+회피방향) + (선택)실차 조향/구동 + 로깅.

drive:=false → 인지+시각화만 (안전): LiDAR → stack_avoid → avoid_viz + RViz. 차 안 움직임.
drive:=true  → 위 + avoid_to_ref(회피점→/adas/target_ref) + bridge(CAN) → dSPACE 실제 조향/구동.

  ros2 launch stack_avoid avoid_drive_full.launch.py drive:=false        # 먼저 이걸로 방향 확인
  ros2 launch stack_avoid avoid_drive_full.launch.py drive:=true v_ref:=0.2  # 실제 회피 기동
★ drive:=true 는 실차 조향/구동 — 바퀴 들고(스탠드)·비상정지 준비. dummy_ref 테스트는 먼저 종료.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from stack_avoid.launch_parts import can_bridge_with_zero_guard, ydlidar_driver


def generate_launch_description():
    pkg = get_package_share_directory('stack_avoid')
    params = os.path.join(pkg, 'config', 'params.yaml')
    rviz_cfg = os.path.join(pkg, 'config', 'avoid_test.rviz')

    drive = LaunchConfiguration('drive')
    v_ref = LaunchConfiguration('v_ref')
    bag_dir = LaunchConfiguration('bag_dir')

    bag_topics = ['/scan', '/scan_front', '/perception/avoid', '/avoid_markers',
                  '/adas/target_ref', '/vehicle/vector', '/tf', '/tf_static', '/rosout']

    return LaunchDescription([
        DeclareLaunchArgument('drive', default_value='false',
                              description='true=실차 조향/구동(avoid_to_ref+bridge), false=인지+RViz만'),
        DeclareLaunchArgument('v_ref', default_value='0.2', description='회피 주행 속도 [m/s]'),
        DeclareLaunchArgument('bag_dir', default_value='avoid_drive_bag'),

        # LiDAR → /scan  (드라이버 노드만 — launch 를 include 하면 placeholder static TF 가
        # 딸려와 stack_avoid_node 의 실측 TF 와 충돌한다. 사유는 field_session.launch.py 참조)
        ydlidar_driver(),
        # 회피 인지 (방향 270 고정) + 시각화
        Node(package='stack_avoid', executable='stack_avoid_node', name='stack_avoid_node',
             output='screen', parameters=[params]),
        Node(package='stack_avoid', executable='avoid_viz', name='avoid_viz', output='screen',
             parameters=[{'lidar_x_m': 0.76, 'vehicle_width_m': 0.62, 'lateral_margin_m': 0.15,
                          'detect_range_m': 3.0, 'offset_max_m': 1.0, 'roi_angle_deg': 180.0}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg], output='log'),

        # ── drive:=true 일 때만: 회피점 → dSPACE 실제 조향/구동 ──
        Node(package='stack_avoid', executable='avoid_to_ref', name='avoid_to_ref', output='screen',
             parameters=[{'target_speed_mps': v_ref, 'straight_when_clear': False}],
             condition=IfCondition(drive)),
        # ── CAN 브리지 + 종료 시 dSPACE 목표값 0 복귀 (안전 가드) ──
        # 공용 조각. 예전에는 이 launch 가 상주 가드만 갖고 있어, 브리지 종료 순서를
        # 보장하는 수정(팀장 리뷰 ⑤)이 field_session 에만 들어가 있었다 — 복붙의 전형적
        # 피해다. 이제 세 launch 가 같은 구현을 쓴다.
        *can_bridge_with_zero_guard(condition=IfCondition(drive)),

        # 로그
        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', bag_dir] + bag_topics, output='screen'),
    ])
