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
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg = get_package_share_directory('stack_avoid')
    params = os.path.join(pkg, 'config', 'params.yaml')
    rviz_cfg = os.path.join(pkg, 'config', 'avoid_test.rviz')
    ydlidar_params = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'), 'params', 'ydlidar.yaml')

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
        LifecycleNode(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
                      name='ydlidar_ros2_driver_node', namespace='/', output='screen',
                      emulate_tty=True, parameters=[ydlidar_params]),
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
        Node(package='bridge_dspace', executable='can_bridge_node', name='can_bridge_node',
             output='screen', parameters=[{'can_interface': 'can0'}],
             condition=IfCondition(drive)),

        # ── 종료 시 dSPACE 목표값 0 복귀 (안전 가드) ──
        # dSPACE 에 watchdog 이 없다(2026-08-09 실측) — PC 송신이 끊겨도 마지막 v_ref 를
        # 무기한 유지한다. launch 를 끄는 것만으로는 정지 상태가 되지 않으므로, 이 가드가
        # SIGINT 를 받아 SocketCAN 에 직접 0 을 쓴다(브리지가 이미 죽어도 동작).
        # ★ `ros2 run` 으로 감싸면 래퍼가 SIGINT 를 삼켜 안 돈다 — Node 액션이어야 한다.
        Node(package='stack_avoid', executable='can_zero', name='can_zero', output='screen',
             condition=IfCondition(drive)),

        # 로그
        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', bag_dir] + bag_topics, output='screen'),
    ])
