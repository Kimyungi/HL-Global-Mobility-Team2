"""실차 측정 세션 — stage2 실측 ①②③ + 경계시험 ⓐⓑⓒ 를 한 구성으로.  이기돈

인지·안전·로깅 부분은 항상 같고, **/adas/target_ref 를 누가 내느냐**만 mode로 바꾼다.
어떤 mode든 stack_estop이 함께 뜨고 명령 노드는 estop 게이트를 통과하므로,
세션 내내 안전 바닥은 동일하다.

  mode:=perception   아무도 명령 안 냄 (차 안 움직임)   → ③ 감지 신뢰 거리
  mode:=step         step_injector (측방 스텝 계단)     → ① 조향 응답, ② 측방 이동 곡선
  mode:=avoid        avoid_to_ref (회피 하네스)         → ⓐⓑⓒ 경계 시험

  # ③ 감지 (가장 안전 — 먼저)
  ros2 launch stack_avoid field_session.launch.py mode:=perception
  # ① 조향 응답 — ★스탠드(바퀴 듦)에서
  ros2 launch stack_avoid field_session.launch.py mode:=step v_ref:=0.3
  # ② 측방 이동 곡선 — 지상, 속도별 2회
  ros2 launch stack_avoid field_session.launch.py mode:=step v_ref:=0.3 hold_s:=6.0
  ros2 launch stack_avoid field_session.launch.py mode:=step v_ref:=0.5 hold_s:=6.0
  # ⓐⓑⓒ 경계 시험
  ros2 launch stack_avoid field_session.launch.py mode:=avoid v_ref:=0.2

별도 터미널에서 `ros2 run stack_avoid mark` 를 띄워 구간 라벨을 남길 것 —
bag의 /test/event 로 나중에 "이 구간이 무슨 시험이었나"를 복원한다.

★ mode:=step·avoid 는 실차 조향/구동이다. mgm_node·dummy_ref_publisher가 떠 있으면
  /adas/target_ref 이중 발행 — 먼저 종료할 것 (run_field_session.sh가 검사한다).
★ v_ref=0으로는 조향 측정 불가 — MPC 지평 = 0.2×v_ref 라 지평이 붕괴한다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg = get_package_share_directory('stack_avoid')
    params = os.path.join(pkg, 'config', 'params.yaml')
    rviz_cfg = os.path.join(pkg, 'config', 'avoid_test.rviz')
    ydlidar_params = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'), 'params', 'ydlidar.yaml')

    v_ref = LaunchConfiguration('v_ref')
    bag_dir = LaunchConfiguration('bag_dir')
    dynamic = LaunchConfiguration('dynamic')
    estop_on = LaunchConfiguration('estop_on_distance_m')
    offsets = LaunchConfiguration('offsets')
    hold_s = LaunchConfiguration('hold_s')
    repeats = LaunchConfiguration('repeats')

    # /test/event 는 구간 라벨 — 이게 없으면 한 세션 bag을 나중에 못 자른다.
    bag_topics = ['/scan', '/scan_front', '/perception/avoid', '/avoid_markers',
                  '/perception/estop', '/perception/estop/status',
                  '/perception/static_estop', '/perception/dynamic_estop',
                  '/perception/dynamic_obstacle_detected',
                  '/adas/target_ref', '/vehicle/vector', '/test/event',
                  '/tf', '/tf_static', '/rosout']

    drives = ['step', 'avoid']      # 실차를 움직이는 모드 — CAN 브리지 필요

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='perception',
            description='perception(③ 감지) | step(①② 스텝) | avoid(ⓐⓑⓒ 경계)'),
        DeclareLaunchArgument('v_ref', default_value='0.3', description='주행/시험 속도 [m/s]'),
        DeclareLaunchArgument('bag_dir', default_value='field_bag'),
        DeclareLaunchArgument(
            'dynamic', default_value='true',
            description='stack_estop 동적 물체 기준 (ⓐ 오탐 확인 시 false로 재시험)'),
        DeclareLaunchArgument(
            'estop_on_distance_m', default_value='0.70',
            description='estop 정지 거리 [m] — 바꾸면 측방여유 부등식 재확인 (찬미와 공유)'),
        DeclareLaunchArgument('offsets', default_value='[0.46, -0.46, 0.30, -0.30]',
                              description='mode:=step 측방 스텝 [m]'),
        DeclareLaunchArgument('hold_s', default_value='3.0',
                              description='mode:=step 스텝 유지 [s] (② 지상은 6s 권장)'),
        DeclareLaunchArgument('repeats', default_value='3',
                              description='mode:=step 반복 횟수'),

        # ── 항상 동일: 인지 + 안전 + 시각화 + 로깅 ──
        # ★ 드라이버 launch를 include하지 않고 드라이버 노드만 직접 띄운다.
        # ydlidar_launch.py 는 base_link→laser_frame 을 (0,0,0.02, yaw 0°) placeholder 로
        # 함께 발행하는데, stack_avoid_node 도 params.yaml 실측값
        # (0.76,0,0.065, yaw 90° = forward_angle 270 반영)으로 같은 쌍을 발행한다.
        # 같은 parent→child 에 static TF publisher 가 둘이면 tf2 버퍼가 나중 도착분을
        # 잡아, 어느 쪽이 이길지가 RViz 기동 타이밍에 따라 매번 달라진다. placeholder 가
        # 이기면 스캔이 원점에 90° 틀어져 그려져 마커와 안 맞는다 (2026-08-09 규명).
        # 드라이버 워크스페이스는 stack_parking 과 공유하므로 그쪽을 고치지 않고
        # 여기서 TF publisher 만 뺀다. 실측 TF 의 단일 소스는 params.yaml 이다.
        LifecycleNode(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
                      name='ydlidar_ros2_driver_node', namespace='/', output='screen',
                      emulate_tty=True, parameters=[ydlidar_params]),
        Node(package='stack_avoid', executable='stack_avoid_node', name='stack_avoid_node',
             output='screen', parameters=[params]),
        # 박찬미 stack_estop — laser_yaw_in_base_rad=π/2 가 우리 forward_angle_deg=270 과 짝.
        # 라이다를 재장착하면 두 값을 같이 고칠 것.
        Node(package='stack_estop', executable='stack_estop_node', name='stack_estop_node',
             output='screen',
             parameters=[{'estop_on_distance_m': estop_on, 'dynamic_enabled': dynamic}]),
        Node(package='stack_avoid', executable='avoid_viz', name='avoid_viz', output='screen',
             parameters=[{'lidar_x_m': 0.76, 'vehicle_width_m': 0.62, 'lateral_margin_m': 0.15,
                          'detect_range_m': 3.0, 'offset_max_m': 1.0, 'roi_angle_deg': 180.0}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg], output='log'),

        # ── mode:=step — ①② 측방 스텝 계단 ──
        Node(package='stack_avoid', executable='step_injector', name='step_injector',
             output='screen',
             parameters=[{'v_ref': v_ref, 'offsets': offsets, 'hold_s': hold_s,
                          'repeats': repeats, 'estop_gate': True}],
             condition=LaunchConfigurationEquals('mode', 'step')),

        # ── mode:=avoid — ⓐⓑⓒ 경계 시험 ──
        # straight_when_clear=true: ⓐ가 "접근하다 비켜 간다"를 보는 시험이라
        # clear 구간에 차가 움직여야 성립한다. 통제된 공간에서만.
        Node(package='stack_avoid', executable='avoid_to_ref', name='avoid_to_ref',
             output='screen',
             parameters=[{'target_speed_mps': v_ref, 'straight_when_clear': True,
                          'estop_gate': True, 'estop_stale_s': 0.25}],
             condition=LaunchConfigurationEquals('mode', 'avoid')),

        # ── CAN 브리지 — 실차를 움직이는 모드에서만 ──
        *[Node(package='bridge_dspace', executable='can_bridge_node', name='can_bridge_node',
               output='screen', parameters=[{'can_interface': 'can0'}],
               condition=LaunchConfigurationEquals('mode', d)) for d in drives],

        # ── 종료 시 dSPACE 목표값 0 복귀 (안전 가드) ──
        # dSPACE 에 watchdog 이 없다(2026-08-09 실측) — PC 송신이 끊겨도 마지막 v_ref 를
        # 무기한 유지한다. launch 를 끄는 것만으로는 정지 상태가 되지 않으므로,
        # 이 가드가 SIGINT 를 받아 SocketCAN 에 직접 0 을 쓴다(브리지가 이미 죽어도 동작).
        # ★ `ros2 run` 으로 감싸면 안 된다 — 래퍼가 SIGINT 를 삼켜 가드가 안 돈다(실측).
        #   Node 액션은 실행 파일을 직접 띄우므로 신호가 그대로 전달된다.
        *[Node(package='stack_avoid', executable='can_zero', name='can_zero', output='screen',
               condition=LaunchConfigurationEquals('mode', d)) for d in drives],

        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', bag_dir] + bag_topics, output='screen'),
    ])
