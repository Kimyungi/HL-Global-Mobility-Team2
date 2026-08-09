"""회피↔긴급정지 경계 시험 (ⓐⓑⓒ) — stack_avoid + stack_estop 합동.  이기돈 · 박찬미

한 대의 앞 LiDAR(/scan)를 두 스택이 같이 구독한다:
  - stack_avoid  : 무엇을 어떻게 피할지 (회피 목표점, /perception/avoid)
  - stack_estop  : 언제 무조건 설지  (안전 바닥, /perception/estop)
avoid_to_ref(MGM 대체 하네스)가 회피점을 /adas/target_ref로 내보내되,
estop=true면 v_ref=0으로 덮어쓴다 — 두 스택의 인계가 실차에서 맞물리는지 확인.

시험 3케이스:
  ⓐ 3m 전방 콘 회피 기동 중 estop이 걸리지 않는가        → 정상 회피 (임계가 과보수적이지 않음)
  ⓑ 1m 앞 갑자기 투입 시 estop이 걸리는가                → 안전망 작동
  ⓒ 연석 같은 연속 경계 접근 시 avoid 진입 없이 서는가    → 오판하지 않음

  ros2 launch stack_avoid avoid_estop_joint.launch.py drive:=false   # ← 먼저 이걸로 (차 안 움직임)
  ros2 launch stack_avoid avoid_estop_joint.launch.py drive:=true v_ref:=0.2

★ drive:=true 는 실차 조향/구동. 바퀴 들고(스탠드) 먼저 · 물리 비상정지 준비.
★ mgm_node·dummy_ref_publisher가 떠 있으면 /adas/target_ref 이중 발행 — 먼저 종료할 것.

판정 근거는 로그와 bag의 다음 토픽으로 구분한다:
  /perception/static_estop   정적 거리(estop_on_distance_m=0.70m) 기준이 걸렸나
  /perception/dynamic_estop  동적 물체 기준(dynamic_stop_distance_m=1.00m)이 걸렸나
  /perception/avoid          narrow_gap(통과 불가)인가, 회피 목표점이 나왔나
  avoid_to_ref 콘솔 로그      v_ref=0을 세운 사유 (ESTOP / narrow_gap / clear)
⚠ ⓐ에서 회피 기동 중 자차가 움직이므로 정지한 콘도 상대운동으로 보인다 —
  dynamic_estop이 오탐하면 dynamic:=false로 한 번 더 돌려 정적 기준만으로 재확인할 것.
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
    dynamic = LaunchConfiguration('dynamic')
    estop_on = LaunchConfiguration('estop_on_distance_m')

    bag_topics = ['/scan', '/scan_front', '/perception/avoid', '/avoid_markers',
                  '/perception/estop', '/perception/estop/status',
                  '/perception/static_estop', '/perception/dynamic_estop',
                  '/perception/dynamic_obstacle_detected',
                  '/adas/target_ref', '/vehicle/vector', '/tf', '/tf_static', '/rosout']

    return LaunchDescription([
        DeclareLaunchArgument('drive', default_value='false',
                              description='true=실차 조향/구동(avoid_to_ref+CAN), false=인지+RViz만'),
        DeclareLaunchArgument('v_ref', default_value='0.2', description='회피 주행 속도 [m/s]'),
        DeclareLaunchArgument('bag_dir', default_value='avoid_estop_joint_bag'),
        DeclareLaunchArgument(
            'dynamic', default_value='true',
            description='stack_estop 동적 물체 기준 사용 여부 (ⓐ 오탐 확인 시 false로 재시험)'),
        DeclareLaunchArgument(
            'estop_on_distance_m', default_value='0.70',
            description='estop 정지 거리 [m] — 바꾸면 stack_avoid 측방여유 부등식 재확인 필요'),

        # LiDAR → /scan  (두 스택이 같은 스캔을 구독)
        # 드라이버 노드만 — launch 를 include 하면 placeholder static TF 가 딸려와
        # stack_avoid_node 의 실측 TF 와 충돌한다. 사유는 field_session.launch.py 참조.
        LifecycleNode(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
                      name='ydlidar_ros2_driver_node', namespace='/', output='screen',
                      emulate_tty=True, parameters=[ydlidar_params]),

        # 회피 인지 (방향 raw 270° 고정)
        Node(package='stack_avoid', executable='stack_avoid_node', name='stack_avoid_node',
             output='screen', parameters=[params]),
        # 긴급 정지 (박찬미) — laser_yaw_in_base_rad=π/2 가 위 forward_angle_deg=270 과 짝.
        # 한쪽 라이다를 재장착하면 두 값을 같이 고쳐야 한다.
        Node(package='stack_estop', executable='stack_estop_node', name='stack_estop_node',
             output='screen',
             parameters=[{'estop_on_distance_m': estop_on, 'dynamic_enabled': dynamic}]),

        Node(package='stack_avoid', executable='avoid_viz', name='avoid_viz', output='screen',
             parameters=[{'lidar_x_m': 0.76, 'vehicle_width_m': 0.62, 'lateral_margin_m': 0.15,
                          'detect_range_m': 3.0, 'offset_max_m': 1.0, 'roi_angle_deg': 180.0}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg], output='log'),

        # ── drive:=true 일 때만: 회피점 → dSPACE 실제 조향/구동 (estop 게이트 ON) ──
        # straight_when_clear=true — ⓐ는 "장애물을 향해 접근하다 비켜 가는" 시험이라
        # clear 구간에서 차가 움직여야 성립한다.
        Node(package='stack_avoid', executable='avoid_to_ref', name='avoid_to_ref',
             output='screen',
             parameters=[{'target_speed_mps': v_ref, 'straight_when_clear': True,
                          'estop_gate': True, 'estop_stale_s': 0.25}],
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

        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', bag_dir] + bag_topics, output='screen'),
    ])
