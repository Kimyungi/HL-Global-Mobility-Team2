"""MBD(Simulink 생성 C) MGM 실차 시험 launch — 차선 ↔ GPS 2상태만.

**REAL_VEHICLE_lane_gps_can.launch.py 와는 별개 파일이다.** 그쪽(V2)은 베이스
좌표부터 지정 구간 3종까지 다 물려 있는 운영 런치이므로 건드리지 않는다.
이 파일은 김재민의 `ADAS_MGR2` v1.68 생성 C 를 `mgm_step()` 자리에 끼워
(CLAUDE.md §5.5 이중 트랙) **레퍼런스 C++ 코어와 같은 차를 같은 코스에서**
굴려 보기 위한 것이다.

무엇이 바뀌는가 — **판단 코어 하나뿐이다.**
  인지 스택 · ref 포맷 · bridge_dspace · CAN 프레임 · dSPACE 는 전부 그대로다.
  생성 C 도 `(ref_points, v_ref, flags)` 까지만 내놓는다 (§5.5 인터페이스).
  CAN 은 MBD 모델의 몫이 아니다 — 양자화·프레임 분할은 계속 bridge_dspace 다.

v1.68 이 **갖고 있지 않은 것** (그래서 이 런치가 빼는 것):
  · AVOID / PARKING 스테이트  → stack_avoid 를 아예 안 띄운다
  · TTC 안전 바닥 · narrow_gap 감속
  · 종점(at_end) 래치 · 역방향 래치 · estop_latch_release
  · 지정 구간 3종(gps_only_zone · stop_zone · avoid_zone — 2026-08-18 main)
  DecisionBackend 가 이 입력들을 감시하다가 하나라도 들어오면 **영구 fail-stop
  래치**(v_ref 0)를 건다. 즉 "몰래 다르게 굴러가는" 일은 없고, 대신 트랙 종점을
  지나면 그 자리에서 서고 **노드를 재시작해야** 다시 움직인다.

그래서 **LANE ↔ WAYPOINT 전이는 차선 신뢰도 히스테리시스 + 재합류 게이트로만**
일어난다. 구간 파일에 GPS 전용 구간을 찍어 두었어도 MBD 는 그것을 못 본다
(validate() 가 경고를 찍는다).

2단계로 쓴다:

  ① 정지 상태 전이 확인 — CAN 없음 (기본값). 바퀴 안 움직인다.
       ros2 launch adas_mgm MBD_lane_gps_can.launch.py \
           waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/<코스>.csv \
           usb_speed:=high camera_fps:=10
       확인:  ros2 run adas_mgm state          (스테이트 전이 이력)
              ros2 topic echo /adas/target_ref  (v_ref·ref points)

  ② 실주행 — 확인 토큰을 주면 bridge_dspace + can_zero 가드가 붙는다.
       ros2 launch adas_mgm MBD_lane_gps_can.launch.py \
           REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
           waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/<코스>.csv \
           usb_speed:=high camera_fps:=10

출발 인가는 **`ros2 run adas_mgm go --skip-avoid`** 다 — stack_avoid 를 안 띄우므로
회피 점검을 건너뛰어야 한다.

빌드는 별도 opt-in 이 필요하다 (생성 C 는 기본 빌드에 링크되지 않는다):
    colcon build --packages-up-to adas_mgm \
        --cmake-args -DADAS_MGM_ENABLE_GENERATED_BACKEND=ON

로깅 — run 마다 ~/FMA_ws/drive_logs/run_mbd_<시각>/ :
  rosbag/            MGM 입출력 전 토픽 + /scan + /rosout
  transitions.csv    **스테이트 전이 이유** — 바뀐 틱·규칙·결정 변수·스펙 대조
  mgm_snapshots.bin  매 10ms CoreSnapshot 덤프 — 같은 덤프를 parity_replay 에 물리면
                     **레퍼런스 C++ 코어와 back-to-back 비교**가 된다 (§5.5 검증)
  mgm_jitter.csv     10ms 루프 주기 실측 (§7)
  lateral.csv        GPS 횡오차
"""
import csv
import os
from datetime import datetime

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, SetLaunchConfiguration, Shutdown)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue

from stack_avoid.launch_parts import can_bridge_with_zero_guard


def die_hard(what, why):
    """비정상 종료일 때만 launch 전체를 내리는 on_exit 핸들러 (V2 와 동일 규약)."""
    def _on_exit(event, context):
        if event.returncode in (0, -2, -15):
            return []
        return [LogInfo(msg=f'[launch] ✖ {what} 비정상 종료(코드 {event.returncode}) — {why}'),
                Shutdown(reason=f'{what} died')]
    return _on_exit


CONFIRM_TOKEN = 'I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX'

# CAN 을 붙일지 — 이 문자열 비교 하나가 ①bench / ②실주행 을 가른다.
CAN_ON = PythonExpression(
    ["'", LaunchConfiguration('REAL_VEHICLE_CONFIRM'), "' == '", CONFIRM_TOKEN, "'"])

LOG_DIR = os.path.expanduser(
    '~/FMA_ws/drive_logs/run_mbd_' + datetime.now().strftime('%m%d_%H%M%S'))

RECORD_TOPICS = [
    '/perception/lane_path', '/perception/gps_path', '/perception/gps_fix',
    '/perception/estop', '/perception/avoid', '/perception/parking',
    '/perception/traffic_stop', '/adas/target_ref', '/vehicle/vector',
    '/scan', '/rosout', '/tf', '/tf_static',
]

DEFAULT_HOMOGRAPHY = os.path.expanduser(
    '~/FMA_ws/src/stack_lane/config/homography.json')

DEFAULT_YDLIDAR_PARAMS = os.path.join(
    os.path.expanduser('~'), 'ydlidar_ws', 'src', 'ydlidar_ros2_driver',
    'params', 'Tmini-Plus-SH.yaml')


def zones_path_for(waypoint_csv):
    """트랙 CSV → 같은 폴더의 구간 파일 경로 (stack_gps.mark_zone 과 같은 규약)."""
    d, base = os.path.split(waypoint_csv)
    stem = base[:-4] if base.endswith('.csv') else base
    if stem.startswith('waypoints_'):
        stem = stem[len('waypoints_'):]
    return os.path.join(d, f'zones_{stem}.yaml')


def validate(context):
    confirm = LaunchConfiguration('REAL_VEHICLE_CONFIRM').perform(context)
    if confirm not in ('NOT_CONFIRMED', CONFIRM_TOKEN):
        raise RuntimeError(
            f'REAL_VEHICLE_CONFIRM 값이 잘못됨: {confirm}\n'
            f'  실주행하려면 정확히 {CONFIRM_TOKEN}\n'
            '  CAN 없이 전이만 보려면 인자 자체를 빼세요.')
    can_on = confirm == CONFIRM_TOKEN

    print('[launch] ════════ MBD(생성 C) MGM 시험 ════════')
    print('[launch] backend = generated (ADAS_MGR2 v1.68) — LANE/WAYPOINT 2상태만')
    if can_on:
        print('[launch] ⚠ 실주행 모드 — CAN TX 나갑니다. 물리 비상정지에 손 올릴 것')
    else:
        print('[launch] bench 모드 — bridge_dspace 미기동, 바퀴 안 움직입니다')
        print('[launch]   실주행하려면: REAL_VEHICLE_CONFIRM:=' + CONFIRM_TOKEN)

    waypoint_csv = LaunchConfiguration('waypoint_csv').perform(context)
    if not waypoint_csv:
        raise RuntimeError('waypoint_csv:=<코스 CSV 경로> 를 지정하세요 (stack_gps 필수)')
    # V2 와 같은 이유로 CSV 를 여기서 실제로 읽는다 — 1~4점짜리 잔여 파일을 주면
    # stack_gps_node 만 조용히 죽고 카메라만 보고 주행하게 된다 (2026-08-15 3연속).
    _MIN_POINTS = 10
    try:
        with open(waypoint_csv, newline='') as f:
            rows = [r for r in csv.DictReader(f) if r.get('lat') and r.get('lon')]
    except OSError as e:
        raise RuntimeError(f'waypoint_csv 를 열 수 없음 — {e}') from e
    if len(rows) < _MIN_POINTS:
        raise RuntimeError(
            f'waypoint_csv 점이 {len(rows)}개뿐 (최소 {_MIN_POINTS}) — 트랙이 아니다: '
            f'{waypoint_csv}')
    print(f'[launch] 웨이포인트 {len(rows)}점 확인: {os.path.basename(waypoint_csv)}')

    # 구간 파일은 stack_gps 에 그대로 넘긴다 (GpsPath 에 실려 bag 에 남고, 나중에
    # 같은 덤프를 레퍼런스 코어로 재생할 때 필요하다). 다만 **MBD 는 못 본다** —
    # 여기서 크게 경고하지 않으면 "언덕에서 왜 안 서지?" 로 현장에서 헤맨다.
    zones_file = (LaunchConfiguration('zones_file').perform(context)
                  or zones_path_for(waypoint_csv))
    if os.path.isfile(zones_file):
        try:
            with open(zones_file) as f:
                z = yaml.safe_load(f) or {}
            n_stop = len(z.get('stop_points') or [])
            n_avoid = len([a for a in (z.get('avoid_zones') or []) if 'end' in a])
            n_gonly = len([a for a in (z.get('gps_only_zones') or []) if 'end' in a])
            print(f'[launch] 구간 파일: {os.path.basename(zones_file)} '
                  f'(정지 {n_stop} · 회피 {n_avoid} · GPS전용 {n_gonly})')
            if n_stop or n_avoid or n_gonly:
                print('[launch] ⚠ ADAS_MGR2 v1.68 은 지정 구간 3종을 구현하지 않는다 — '
                      '이 run 에서는 **전부 무시**된다')
                if n_stop:
                    print('[launch]   · 지정 지점에서 자동 정차하지 않는다')
                if n_gonly:
                    print('[launch]   · GPS 전용 구간에서도 차선 신뢰도가 높으면 LANE 으로 간다 '
                          '(run 전체를 GPS 로만 가려면 gps_only:=true)')
        except Exception as e:                                # noqa: BLE001
            print(f'[launch] ⚠ 구간 파일을 못 읽음 — 지정 구간 없이 진행: {e}')
    else:
        print(f'[launch] 구간 파일 없음 ({os.path.basename(zones_file)}) — 지정 구간 없이 주행')

    print('[launch] ⚠ stack_avoid 미기동 — 장애물은 회피가 아니라 stack_estop 정지로만 대응.'
          ' 출발 인가는 `ros2 run adas_mgm go --skip-avoid`')
    print('[launch] ⚠ 트랙 종점(at_end)에 닿으면 생성 backend 가 영구 fail-stop 래치를 건다 —'
          ' 정지는 정상, 다시 달리려면 launch 재시작')

    if LaunchConfiguration('lane_enabled').perform(context) == 'true':
        homography = LaunchConfiguration('homography_path').perform(context)
        if not os.path.isfile(homography):
            raise RuntimeError(
                f'호모그래피 파일 없음: {homography} — placeholder 실주행 금지 '
                '(stack_lane CALIBRATION_GUIDE.md)')
    else:
        print('[launch] ⚠ stack_lane 미기동 — LANE 전이 없음, '
              '`ros2 run adas_mgm go --skip-lane --skip-avoid`')

    os.makedirs(LOG_DIR, exist_ok=True)
    print(f'[record] 로그 디렉터리: {LOG_DIR}')
    return [SetLaunchConfiguration('zones_file_resolved', zones_file)]


def generate_launch_description():
    mgm_params = os.path.join(
        get_package_share_directory('adas_mgm'), 'config', 'params.yaml')

    # gps_only 오버라이드의 "평상시" 값은 params.yaml 에서 읽는다 (V2 와 같은 이유 —
    # 여기 숫자를 박으면 params.yaml 튜닝이 이 launch 에서만 조용히 무시된다).
    with open(mgm_params) as _f:
        _yaml = yaml.safe_load(_f)['mgm_node']['ros__parameters']
    lane_exit_default = float(_yaml['lane_conf_exit'])
    lane_return_default = float(_yaml['lane_conf_return'])

    return LaunchDescription([
        # 인자를 **주지 않으면** CAN 없는 bench 로 뜬다 (V2 는 없으면 기동 거부 —
        # 이 파일은 정지 전이 확인 단계를 1급으로 두므로 규약이 다르다).
        DeclareLaunchArgument('REAL_VEHICLE_CONFIRM', default_value='NOT_CONFIRMED'),
        DeclareLaunchArgument('can_interface', default_value='can0'),

        # ── stack_gps (V2 와 동일 인자·동일 기본값)
        DeclareLaunchArgument('waypoint_csv', default_value='',
                              description='코스 웨이포인트 CSV (필수)'),
        DeclareLaunchArgument('rtcm_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('ref_lookahead_m', default_value='1.0'),
        DeclareLaunchArgument('rejoin_rate_damp_s', default_value='0.0'),
        DeclareLaunchArgument('rejoin_full_cross_m', default_value='0.5'),
        DeclareLaunchArgument('rejoin_target_max_m', default_value='1.8'),
        DeclareLaunchArgument('rejoin_target_min_m', default_value='1.8'),
        DeclareLaunchArgument('rejoin_e_lpf_s', default_value='0.15'),
        DeclareLaunchArgument(
            'zones_file', default_value='',
            description='구간 파일 경로 (빈 값 = waypoint_csv 옆 zones_*.yaml 자동). '
                        'MBD 는 구간을 무시하지만 bag 기록·재생 비교용으로 넘긴다'),
        DeclareLaunchArgument('stop_zone_span_m', default_value='1.0'),

        # LANE 전이 차단 (임계 2.0 = confidence 최대 1.0 이라 도달 불가).
        # 생성 backend 의 adapter 도 이 2.0/2.0 을 "GPS 전용 sentinel" 로 인정한다.
        DeclareLaunchArgument('gps_only', default_value='false'),
        DeclareLaunchArgument('lane_enabled', default_value='true'),

        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('gps_error_log_csv',
                              default_value=os.path.join(LOG_DIR, 'lateral.csv')),

        # ── stack_lane (V2 와 동일 — 튜닝 근거는 V2 주석 참조)
        DeclareLaunchArgument('homography_path', default_value=DEFAULT_HOMOGRAPHY),
        DeclareLaunchArgument('lane_weights', default_value=os.path.expanduser(
            '~/FMA_ws/src/stack_lane/models/yolopv2.pt')),
        DeclareLaunchArgument('camera_mxid', default_value='14442C105157D3D200'),
        # OAK-D USB3 가 RTK 를 죽인다 — usb_speed:=high 는 반드시 camera_fps:=10 과
        # 함께 (CLAUDE.md §6, 2026-08-14 실측 -16.5dB).
        DeclareLaunchArgument('camera_fps', default_value='10'),
        # ★ 기본값을 'high'/10 으로 뒤집었다 (2026-08-24, 인수인계). USB3 로
        #   열거되면 GNSS L1 이 덮여 RTK 가 죽는데, 그걸 피하려면 매 launch 마다
        #   인자를 손으로 붙여야 했다. 한 번 잊으면 위성 수도 HDOP 도 RTCM 도
        #   정상으로 보이는 채 FIXED 만 안 잡혀 원인 찾기가 어렵다 — C/N0(GSV)를
        #   봐야 보인다. 안전한 쪽을 기본으로 두고, USB3 가 필요하면 그때 올린다:
        #     ros2 launch ... usb_speed:=super camera_fps:=30
        DeclareLaunchArgument('usb_speed', default_value='high'),
        DeclareLaunchArgument('lane_device', default_value='xpu'),
        DeclareLaunchArgument('ref_point0_lookahead_m', default_value='2.0'),
        DeclareLaunchArgument('ref_point0_extrap_mode', default_value='linear'),
        DeclareLaunchArgument('ref_point0_min_confidence', default_value='0.30'),
        DeclareLaunchArgument('coeff_smoothing_alpha', default_value='0.3'),

        # ── stack_estop (V2 와 동일 — v_base 1.0 세트의 문턱값)
        DeclareLaunchArgument('ydlidar_params', default_value=DEFAULT_YDLIDAR_PARAMS),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB_LIDAR'),
        DeclareLaunchArgument('laser_yaw_in_base_rad', default_value='1.57079632679'),
        DeclareLaunchArgument('dynamic_enabled', default_value='true'),
        DeclareLaunchArgument('dynamic_stop_distance_m', default_value='1.35'),
        DeclareLaunchArgument('estop_on_distance_m', default_value='1.20'),
        DeclareLaunchArgument('estop_off_distance_m', default_value='1.35'),
        DeclareLaunchArgument('dynamic_tracking_max_distance_m', default_value='3.00'),

        OpaqueFunction(function=validate),

        LifecycleNode(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            namespace='/',
            parameters=[LaunchConfiguration('ydlidar_params'), {
                'port': LaunchConfiguration('lidar_port'),
                'baudrate': 230400,
                'lidar_type': 1,
                'intensity_bit': 8,
                # 좌우 규약은 V2 와 동일하게 고정 (2026-08-13 규명).
                'reversion': False,
                'inverted': False,
            }],
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=2.0,
        ),

        # ★ stack_avoid 는 띄우지 않는다. v1.68 에 AVOID 가 없어서만이 아니라,
        #   장애물이 한 번이라도 잡히면 DecisionBackend 가 "지원 범위 이탈" 로
        #   영구 fail-stop 을 래치해 run 이 끝나기 때문이다.
        #   ⚠ 그 대가로 base_link→laser_frame TF 발행자가 없다 (평소엔 stack_avoid
        #     노드가 실측값으로 발행). stack_estop 은 laser_yaw_in_base_rad 를 직접
        #     받으므로 동작에 영향 없고, RViz 로 /scan 을 보려면 그때만 TF 를 따로
        #     띄울 것.

        Node(
            package='stack_estop',
            executable='stack_estop_node',
            name='stack_estop_node',
            parameters=[{
                'laser_yaw_in_base_rad': ParameterValue(
                    LaunchConfiguration('laser_yaw_in_base_rad'), value_type=float),
                'dynamic_enabled': ParameterValue(
                    LaunchConfiguration('dynamic_enabled'), value_type=bool),
                'dynamic_stop_distance_m': ParameterValue(
                    LaunchConfiguration('dynamic_stop_distance_m'), value_type=float),
                'dynamic_tracking_max_distance_m': ParameterValue(
                    LaunchConfiguration('dynamic_tracking_max_distance_m'), value_type=float),
                'estop_on_distance_m': ParameterValue(
                    LaunchConfiguration('estop_on_distance_m'), value_type=float),
                'estop_off_distance_m': ParameterValue(
                    LaunchConfiguration('estop_off_distance_m'), value_type=float),
            }],
            output='screen',
        ),

        Node(
            package='stack_gps',
            executable='stack_gps_node',
            name='stack_gps_node',
            parameters=[{
                'waypoint_csv': LaunchConfiguration('waypoint_csv'),
                'rtcm_host': LaunchConfiguration('rtcm_host'),
                'error_log_csv': LaunchConfiguration('gps_error_log_csv'),
                'ref_lookahead_m': ParameterValue(
                    LaunchConfiguration('ref_lookahead_m'), value_type=float),
                'rejoin_rate_damp_s': ParameterValue(
                    LaunchConfiguration('rejoin_rate_damp_s'), value_type=float),
                'rejoin_full_cross_m': ParameterValue(
                    LaunchConfiguration('rejoin_full_cross_m'), value_type=float),
                'rejoin_target_max_m': ParameterValue(
                    LaunchConfiguration('rejoin_target_max_m'), value_type=float),
                'rejoin_target_min_m': ParameterValue(
                    LaunchConfiguration('rejoin_target_min_m'), value_type=float),
                'rejoin_e_lpf_s': ParameterValue(
                    LaunchConfiguration('rejoin_e_lpf_s'), value_type=float),
                'zones_file': LaunchConfiguration('zones_file_resolved'),
                'stop_zone_span_m': ParameterValue(
                    LaunchConfiguration('stop_zone_span_m'), value_type=float),
            }],
            output='screen',
            on_exit=die_hard('stack_gps_node',
                             'GPS 없이 주행 불가 — waypoint_csv·RTK·빌드 확인'),
        ),

        Node(
            package='stack_lane',
            executable='stack_lane_node',
            name='stack_lane_node',
            condition=IfCondition(LaunchConfiguration('lane_enabled')),
            parameters=[{
                'homography_path': LaunchConfiguration('homography_path'),
                'weights': LaunchConfiguration('lane_weights'),
                'camera_mxid': LaunchConfiguration('camera_mxid'),
                'device': LaunchConfiguration('lane_device'),
                'camera_fps': ParameterValue(
                    LaunchConfiguration('camera_fps'), value_type=int),
                'usb_speed': LaunchConfiguration('usb_speed'),
                'ref_point0_lookahead_m': ParameterValue(
                    LaunchConfiguration('ref_point0_lookahead_m'), value_type=float),
                'ref_point0_extrap_mode': LaunchConfiguration('ref_point0_extrap_mode'),
                'ref_point0_min_confidence': ParameterValue(
                    LaunchConfiguration('ref_point0_min_confidence'), value_type=float),
                'coeff_smoothing_alpha': ParameterValue(
                    LaunchConfiguration('coeff_smoothing_alpha'), value_type=float),
            }],
            output='screen',
        ),

        Node(
            package='adas_mgm',
            executable='mgm_node',
            name='mgm_node',
            parameters=[mgm_params, {
                # ★ 이 두 줄이 MBD 시험의 전부다 — 나머지는 V2 와 같은 구성.
                #   acknowledge 를 빼면 노드가 기동 실패한다 (core 로 몰래 폴백하지 않음).
                'backend': 'generated',
                'generated_backend_acknowledge_limited_scope': True,

                'snapshot_dump_path': os.path.join(LOG_DIR, 'mgm_snapshots.bin'),
                'jitter_csv_path': os.path.join(LOG_DIR, 'mgm_jitter.csv'),
                # 스테이트가 바뀔 때마다 **왜** 바뀌었는지 — 그 틱의 결정 변수와
                # §4 규칙 대조 결과. 콘솔에도 같은 줄이 뜬다.
                'transition_csv_path': os.path.join(LOG_DIR, 'transitions.csv'),
                'wait_go': True,
                'lane_conf_exit': ParameterValue(PythonExpression(
                    ["2.0 if '", LaunchConfiguration('gps_only'),
                     f"' == 'true' else {lane_exit_default}"]),
                    value_type=float),
                'lane_conf_return': ParameterValue(PythonExpression(
                    ["2.0 if '", LaunchConfiguration('gps_only'),
                     f"' == 'true' else {lane_return_default}"]),
                    value_type=float),
                # 지정 구간 기구는 전부 끈다 — v1.68 이 구현하지 않으므로 켜 두면
                # 레퍼런스 코어와 비교할 때 "왜 다르지" 의 원인만 늘어난다.
                'stop_zone_hold_cycles': 0,
                'avoid_zone_only': False,
            }],
            output='screen',
            on_exit=die_hard('mgm_node',
                             '목표값 송신 중단 — can_zero로 0 복귀 후 전체 종료'),
        ),

        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('record')),
            cmd=['ros2', 'bag', 'record', '-o', os.path.join(LOG_DIR, 'rosbag')]
                + RECORD_TOPICS,
            output='screen',
        ),

        # CAN 브리지 + 종료 시 목표값 0 복귀 — 확인 토큰을 준 run 에서만 붙는다.
        # 토큰이 없으면 이 셋이 통째로 빠져 /adas/target_ref 를 아무도 안 읽는다
        # (= 바퀴가 안 움직인다). 김재민의 PR #39 는 대신 토픽 이름을 remap 해
        # 격리했는데, 그러면 실주행 단계로 못 넘어가므로 조건부 기동으로 바꿨다.
        *can_bridge_with_zero_guard(
            condition=IfCondition(CAN_ON),
            can_interface=LaunchConfiguration('can_interface')),
    ])
