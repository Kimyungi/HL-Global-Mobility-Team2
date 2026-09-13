"""차선+GPS+회피 통합 실차 주행 launch — lane ↔ waypoint ↔ avoid 자동 전이 (CLAUDE.md §4).

한 번에 띄우는 노드 (2026-08-11 통합 점검 §3의 "터미널 5개" 조합을 대체):
  ydlidar + stack_estop  (REAL_VEHICLE_stack_estop_mgm_can과 동일 구성)
  stack_avoid  — 장애물 회피 (2026-08-12 통합). base_link→laser_frame TF도 이 노드가
                 실측값(stack_avoid params.yaml)으로 발행 — 예전의 placeholder
                 laser_static_tf는 제거 (같은 TF 2중 발행 시 비결정적, PR #23·2026-08-09 규명)
  stack_gps    — waypoint_csv 필수 인자
  stack_lane   — 실측 호모그래피·MxID 핀닝·오실레이션 잠정 튜닝(TESTING_LOG §7.3) 기본 적용
  stack_traffic — 신호등·정지선 (2026-08-29 통합). **기본 꺼짐** — traffic_enabled:=true
                 로 켠다. 2번째 OAK-D 를 쓰므로 USB2 대역폭을 차선과 나눠 쓴다
                 (traffic_width/height 주석 참조). 끄면 거동은 통합 전과 동일하다.
  adas_mgm     — config/params.yaml 적용 (기존 REAL_VEHICLE launch는 params 누락이었음)
  bridge_dspace — 실제 CAN TX + 종료 시 can_zero로 목표값 0 복귀
                 (dSPACE watchdog 미구현 실측 2026-08-09 — CLAUDE.md §3 주의 참조)

주의:
- stack_estop/launch/REAL_VEHICLE_stack_estop_mgm_can.launch.py 와 동시 실행 금지
  (estop/mgm/bridge 중복). 이 파일 하나만 띄운다.
- 실제 CAN TX가 나가므로 동일한 확인 토큰을 요구한다.

로깅 — run마다 ~/FMA_ws/drive_logs/run_<시각>/ 에 모아 저장:
  rosbag/            MGM 입출력 전 토픽 + /scan + /rosout (record:=false로만 끔)
  mgm_snapshots.bin  매 10ms CoreSnapshot 덤프 — core_replay로 판단 재현 (§5.5, 항상)
  mgm_jitter.csv     10ms 루프 주기 실측 (§7 판정 근거, 항상)
  lateral.csv        GPS 횡오차 (DRIVE_GUIDE와 동일 포맷, 항상)
  vehicle_vector.csv dSPACE RX 피드백 {x,y,yaw,v,str,counter} (2026-08-25 신설).
                     counter로 dSPACE 측 로그와 틱 단위 정합 (CLAUDE.md §3)

사용:
  ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py \
      REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
      waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/<코스>.csv

  GPS 구간별 T자/평행 주차까지 통합:
  ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py \
      REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
      waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/<코스>.csv \
      parking_enabled:=true \
      t_parking_zone_ranges:="[120,140]" \
      parallel_parking_zone_ranges:="[260,285]"

  신호등까지 함께 (카메라 2대):
  ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py \
      REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
      waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/<코스>.csv \
      traffic_enabled:=true
  ★ 켜기 전에 `ros2 run stack_traffic stack_traffic_ml_preflight` 가
    ML_RUNTIME_READY 인지 확인할 것 (HANDOVER §2.3).
"""
import csv
import math
import os
from datetime import datetime
from typing import List

import yaml

from ament_index_python.packages import (
    PackageNotFoundError, get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            SetLaunchConfiguration, Shutdown)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue

# CAN 브리지 + 종료 시 dSPACE 목표값 0 복귀(can_zero) 공용 조각 — 근거·순서 보장은
# stack_avoid/launch_parts.py 주석 참조 (dSPACE watchdog 미구현 실측 2026-08-09)
from stack_avoid.launch_parts import can_bridge_with_zero_guard


def die_hard(what, why):
    """비정상 종료일 때만 launch 전체를 내리는 on_exit 핸들러.

    무조건 Shutdown을 걸면 Ctrl-C 정상 종료 때도 다시 Shutdown이 발행돼
    `Cannot shutdown a ROS adapter that is not running` 에러가 뜬다 —
    실패처럼 보여 현장에서 혼란스럽다 (2026-08-15). returncode 0(정상)과
    -2/-15(SIGINT/SIGTERM = 우리가 내린 종료)은 그냥 통과시킨다.
    """
    def _on_exit(event, context):
        if event.returncode in (0, -2, -15):
            return []
        return [LogInfo(msg=f'[launch] ✖ {what} 비정상 종료(코드 {event.returncode}) — {why}'),
                Shutdown(reason=f'{what} died')]
    return _on_exit


CONFIRM_TOKEN = 'I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX'

# run 단위 로그 디렉터리 — launch 파일은 실행마다 새로 파싱되므로 매 run 고유
LOG_DIR = os.path.expanduser(
    '~/FMA_ws/drive_logs/run_' + datetime.now().strftime('%m%d_%H%M%S'))

# 버그 판단에 필요한 전 토픽 — MGM 입력(인지 6종)·출력·dSPACE 회신·라이다 원본·
# 노드 로그(/rosout — watchdog "estop 강제" 경고 등이 여기 남는다)·TF.
# 미발행 토픽(avoid/parking/traffic 미탑재 시)은 그냥 비어 있게 기록된다.
RECORD_TOPICS = [
    '/perception/lane_path', '/perception/gps_path', '/perception/gps_fix',
    '/perception/estop', '/perception/avoid', '/perception/parking',
    '/perception/traffic_stop', '/adas/target_ref', '/vehicle/vector',
    '/adas/mgm_state', '/parking/mission_command', '/operator/cancel_mission',
    '/scan', '/lidar/a1/scan', '/unified_lidar/scan',
    '/parking/local_map', '/parking/slam_pose', '/parking/pipeline_stage',
    '/parking/left_wall/status', '/parking/left_wall/diagnostics', '/parking/left_wall/markers',
    '/rosout', '/tf', '/tf_static',
]

# 실측 호모그래피 (2026-08-11 캘리브레이션, LOO RMS 0.041m) — 소스 트리 절대경로로
# 지정해야 한다: 노드 기본 경로는 설치본 내부로 해석돼 파일을 못 찾는다
# (stack_lane CALIBRATION_GUIDE.md §6).
DEFAULT_HOMOGRAPHY = os.path.expanduser(
    '~/FMA_ws/src/stack_lane/config/homography.json')

# 지정 구간(정지 지점·회피 허용 구간)은 **트랙 CSV 옆의 구간 파일**에서 온다.
#   waypoints_<이름>.csv  →  zones_<이름>.yaml   (같은 폴더)
# 파일은 `ros2 run stack_gps mark_zone` 이 현장에서 기록한다 — 실차 launch 가 도는
# 중에 원하는 자리에 정차하고 찍으면 된다. 위경도로 남기므로 트랙을 다시 기록해도
# 같은 장소를 가리키고, 엉뚱한 코스에 쓰면 stack_gps 가 스냅 거리로 걸러 낸다.
# 파일이 없으면 지정 구간 없이 그냥 주행한다 (에러 아님).


def zones_path_for(waypoint_csv):
    """트랙 CSV → 같은 폴더의 구간 파일 경로 (stack_gps.mark_zone 과 같은 규약).

    launch 파싱은 셸 PYTHONPATH 에 좌우돼 stack_gps 를 import 하면 터질 수 있으므로
    (2026-08-15 실측, validate() 주석 참조) 규약을 여기서 자립적으로 복제한다.
    """
    d, base = os.path.split(waypoint_csv)
    stem = base[:-4] if base.endswith('.csv') else base
    if stem.startswith('waypoints_'):
        stem = stem[len('waypoints_'):]
    return os.path.join(d, f'zones_{stem}.yaml')


def ydlidar_file(*parts):
    """`ydlidar_ros2_driver` 안의 파일 경로 — **저장소 설치본이 1순위**.

    2026-08-29: 기본값이 `~/ydlidar_ws/src/...` 로 박혀 있었는데 이 PC 의 실제
    워크스페이스는 `~/ydlidar_ros2_ws` 라 파일이 없었다. 파일이 없으면 드라이버가
    뜨자마자 죽고 `respawn` 으로 2초마다 되살아나기만 해 `/scan` 이 영영 0Hz 인데,
    화면에는 "go 가 안 통과한다"로만 보인다 (원인이 라이다로 안 보인다).

    저장소가 드라이버를 직접 갖고 있으므로(`src/ydlidar_ros2_driver`, 설치본의
    params 는 외부 워크스페이스본과 바이트 동일) **설치본을 기본값으로 쓴다.**
    외부 워크스페이스는 옛 세팅 호환용 폴백으로만 남긴다.
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


DEFAULT_YDLIDAR_PARAMS = ydlidar_file('params', 'Tmini-Plus-SH.yaml')


def validate(context, log_dir=LOG_DIR, lidar_estop_enabled=True):
    if LaunchConfiguration('REAL_VEHICLE_CONFIRM').perform(context) != CONFIRM_TOKEN:
        raise RuntimeError(
            'REAL VEHICLE launch refused. '
            'Set REAL_VEHICLE_CONFIRM:=' + CONFIRM_TOKEN)
    if not lidar_estop_enabled:
        if int(LaunchConfiguration('escape_after_cycles').perform(context)) != 0:
            raise RuntimeError('LiDAR E-stop excluded test requires escape_after_cycles:=0')
        print('[launch] LiDAR E-stop input DISABLED — separate test configuration')
    traffic_enabled = LaunchConfiguration('traffic_enabled').perform(context).lower() == 'true'
    traffic_require_stop_gate = (
        LaunchConfiguration('traffic_require_stop_gate').perform(context).lower() == 'true')
    try:
        traffic_stop_y_ratio = float(
            LaunchConfiguration('traffic_stop_y_ratio').perform(context))
    except ValueError as e:
        raise RuntimeError('traffic_stop_y_ratio는 숫자여야 합니다.') from e
    if traffic_require_stop_gate and not traffic_enabled:
        raise RuntimeError(
            'traffic_require_stop_gate:=true 이면 traffic_enabled:=true 가 필요합니다.')
    if traffic_require_stop_gate and not 0.0 < traffic_stop_y_ratio <= 1.10:
        raise RuntimeError(
            '운영 신호등 정지 게이트가 비활성입니다. 검증된 '
            'traffic_stop_y_ratio:=<0보다 크고 1.10 이하>를 지정하세요.')
    waypoint_csv = LaunchConfiguration('waypoint_csv').perform(context)
    route_file = LaunchConfiguration('route_sequence_file').perform(context)
    start_id = LaunchConfiguration('route_start_id').perform(context)
    end_id = LaunchConfiguration('route_end_id').perform(context)
    if (start_id or end_id) and not route_file:
        raise RuntimeError('route_start_id/route_end_id require route_sequence_file')
    if route_file:
        from stack_gps.route_plan import RoutePlan
        plan = RoutePlan(route_file, start_id, end_id)
        first = plan.files[0]
        if waypoint_csv and os.path.realpath(waypoint_csv) != str(first.csv):
            raise RuntimeError('waypoint_csv must match the first route in route_sequence_file')
        explicit_zones = LaunchConfiguration('zones_file').perform(context)
        if explicit_zones and os.path.realpath(explicit_zones) != str(first.zones):
            raise RuntimeError('sequence Zone overrides belong in each manifest entry')
        waypoint_csv = str(first.csv)
        context.launch_configurations['waypoint_csv'] = waypoint_csv
        context.launch_configurations['zones_file'] = str(first.zones)
        print(f'[launch] route sequence: {" -> ".join(r.id for r in plan.files)}, identity={plan.sequence_id}')
    if not waypoint_csv:
        raise RuntimeError('waypoint_csv:=<코스 CSV 경로> 를 지정하세요 (stack_gps 필수)')
    # CSV를 여기서 실제로 읽어본다. 경로 오타나 1~4점짜리 잔여 파일(FIXED 확인용
    # 기록)을 주면 stack_gps_node가 0.05초 만에 exit 1로 죽는데, launch는 나머지
    # 노드를 그대로 띄우고 계속 돈다 — 화면에 트레이스백이 한 번 스치고 묻힌다.
    # 그러면 gps_path가 아예 없어 FIXED가 잡힐 수 없고, 스테이트가 LANE이면
    # MGM의 gps watchdog도 안 걸려 **카메라만 보고 주행**하게 된다
    # (2026-08-15 run_0815_150224·150408·151100 3연속 실사례).
    # 검사는 launch 파일 안에서 자립적으로 한다. stack_gps.path_engine을 import해
    # 같은 로더를 쓰려 했으나, launch 파싱은 `ros2 launch` 프로세스의 PYTHONPATH에
    # 의존해 셸 환경에 따라 `No module named 'stack_gps'`로 launch 자체가 죽는다
    # (2026-08-15 실측). 검증 하나 때문에 패키지 간 파이썬 의존을 만들지 않는다.
    _MIN_POINTS = 10
    try:
        with open(waypoint_csv, newline='') as f:
            rows = [r for r in csv.DictReader(f) if r.get('lat') and r.get('lon')]
    except OSError as e:
        raise RuntimeError(f'waypoint_csv 를 열 수 없음 — {e}') from e
    if len(rows) < _MIN_POINTS:
        raise RuntimeError(
            f'waypoint_csv 점이 {len(rows)}개뿐 (최소 {_MIN_POINTS}) — 트랙이 아니다: '
            f'{waypoint_csv}\n'
            '  실코스 예: waypoints_wonju_license_20260818_160511.csv (721점)\n'
            '  1~4점 파일은 FIXED 확인용 잔여물이다.')
    print(f'[launch] 웨이포인트 {len(rows)}점 확인: {os.path.basename(waypoint_csv)}')
    hold_s = LaunchConfiguration('stop_hold_sec').perform(context)
    zones_file = (LaunchConfiguration('zones_file').perform(context)
                  or zones_path_for(waypoint_csv))
    n_stop, n_avoid = 0, 0
    if os.path.isfile(zones_file):
        try:
            with open(zones_file) as f:
                z = yaml.safe_load(f) or {}
            stops = z.get('stop_points') or []
            avoids = [a for a in (z.get('avoid_zones') or []) if 'end' in a]
            gonly = [a for a in (z.get('gps_only_zones') or []) if 'end' in a]
            parking = z.get('parking_points') or []
            n_stop, n_avoid = len(stops), len(avoids)
            print(f'[launch] 구간 파일: {os.path.basename(zones_file)} '
                  f'(정지 {n_stop} · 회피 {n_avoid} · GPS전용 {len(gonly)} · '
                  f'주차 {len(parking)})')
            if gonly:
                print(f'[launch]   GPS 전용 구간 {len(gonly)}개 — 그 안에서는 차선 전이 없음 '
                      '(run 전체를 GPS로만 가려면 gps_only:=true)')
            for i, e in enumerate(stops):
                note = f"  ({e.get('note')})" if e.get('note') else ''
                print(f'[launch]   정지 {i + 1}: {e.get("lat")},{e.get("lon")}{note}')
            for i, e in enumerate(parking):
                print(f'[launch]   주차 {i + 1}: {e.get("mode")} '
                      f'{e.get("lat")},{e.get("lon")}')
        except Exception as e:                                # noqa: BLE001
            print(f'[launch] ⚠ 구간 파일을 못 읽음 — 지정 구간 없이 진행: {e}')
    else:
        print(f'[launch] 구간 파일 없음 ({os.path.basename(zones_file)}) — 지정 구간 없이 주행. '
              '만들려면 주행 중 그 자리에 정차하고 `ros2 run stack_gps mark_zone stop`')
    stop_pts = LaunchConfiguration('stop_points_latlon').perform(context)
    if n_stop or stop_pts:
        print(f'[launch] 지정 정지: 각 지점에서 {hold_s}s 정차 후 자동 재출발'
              + (f' (+ 인자 지정 {stop_pts})' if stop_pts else ''))
    zone_only = LaunchConfiguration('avoid_zone_only').perform(context) == 'true'
    avoid_zone = LaunchConfiguration('avoid_zone_latlon').perform(context)
    if zone_only and not (n_avoid or avoid_zone):
        print('[launch] ══════════════════════════════════════════════════')
        print('[launch] ⚠⚠ 회피 전면 차단 (avoid_zone_only:=true + 구간 없음) — '
              '장애물은 회피 없이 stack_estop 정지로만 대응한다')
        print('[launch]    구간은 `mark_zone avoid_start` / `avoid_end` 로 찍는다. '
              '인자를 빼면 기본값 false = 어디서나 회피')
        print('[launch] ══════════════════════════════════════════════════')
    elif zone_only:
        print(f'[launch] 회피 허용 구간에서만 회피 (구간 {n_avoid}개)')
    else:
        print('[launch] 회피 구간 제한 없음 (기본) — 어디서나 회피')
    # 카메라를 안 띄우는 run(lane_enabled:=false)에선 호모그래피 유무가 무의미
    if LaunchConfiguration('lane_enabled').perform(context) == 'true':
        homography = LaunchConfiguration('homography_path').perform(context)
        if not os.path.isfile(homography):
            raise RuntimeError(
                f'호모그래피 파일 없음: {homography} — placeholder 실주행 금지 '
                '(stack_lane CALIBRATION_GUIDE.md)')
    else:
        print('[launch] ⚠ stack_lane 미기동 (lane_enabled:=false) — '
              '차선 전이 없음, 출발 인가는 `ros2 run adas_mgm go --skip-lane`')
    # 라이다 파라미터 파일은 여기서 반드시 확인한다. 없으면 드라이버가 뜨자마자
    # 죽고 respawn=True 로 2초마다 되살아나기만 해 /scan 이 영영 0Hz 인데, 화면에는
    # "go 가 안 통과한다"로만 보여 원인이 라이다로 안 보인다 (2026-08-29: 기본값이
    # 없는 워크스페이스를 가리키고 있었다 — ydlidar_file() 주석 참조).
    parking_enabled = (
        LaunchConfiguration('parking_enabled').perform(context).lower()
        in ('true', '1', 'yes', 'on'))
    ydlidar_params = LaunchConfiguration('ydlidar_params').perform(context)
    if not parking_enabled and not os.path.isfile(ydlidar_params):
        raise RuntimeError(
            f'라이다 파라미터 파일 없음: {ydlidar_params}\n'
            '  ydlidar_ros2_driver 를 빌드했는지 확인하거나 '
            'ydlidar_params:=<경로> 로 직접 지정하세요.')
    os.makedirs(log_dir, exist_ok=True)
    print(f'[record] 로그 디렉터리: {log_dir}')
    # 확정한 구간 파일 경로를 노드에 넘긴다. 이 OpaqueFunction 은 LaunchDescription
    # 목록에서 노드들보다 **앞**에 있으므로 여기서 설정한 값이 아래 Node 에 잡힌다.
    return [SetLaunchConfiguration('zones_file_resolved', zones_file),
            SetLaunchConfiguration('route_sequence_enabled_resolved', 'true' if route_file else 'false')]


def build_launch_description(
        log_dir=LOG_DIR, default_homography=DEFAULT_HOMOGRAPHY,
        default_lane_weights=os.path.expanduser('~/FMA_ws/src/stack_lane/models/yolopv2.pt'),
        *, lidar_estop_enabled=True):
    mgm_params = os.path.join(
        get_package_share_directory('adas_mgm'), 'config', 'params.yaml')

    # gps_only 오버라이드의 "평상시" 값은 params.yaml에서 읽어온다.
    # 여기 숫자를 하드코딩하면 이 dict가 mgm_params보다 뒤에 있어 params.yaml을
    # 덮어써, 임계를 튜닝해도 이 launch에서만 조용히 무시된다 (2026-08-14 발견 —
    # 0.4/0.6이 박혀 있어 0.35/0.70 상향이 무효화될 뻔했다).
    with open(mgm_params) as _f:
        _yaml = yaml.safe_load(_f)['mgm_node']['ros__parameters']
    lane_exit_default = float(_yaml['lane_conf_exit'])
    lane_return_default = float(_yaml['lane_conf_return'])
    v_base_default = float(_yaml['v_base'])
    # The no-estop entry must remain recovery-disabled even when the RC profile enables it.
    escape_after_cycles_default = int(_yaml['escape_after_cycles']) if lidar_estop_enabled else 0

    # Four-LiDAR a1 uses reversion=true and the calibrated raw forward angle
    # is +87 deg. The single-LiDAR /scan profile uses raw -90 deg instead.
    # Reusing its +90 deg yaw for a1 turns rear returns into front obstacles.
    geometry_file = os.path.join(get_package_share_directory('lidar_fusion_v2'),
                                 'config', 'fixed_geometry.yaml')
    with open(geometry_file) as stream:
        a1 = yaml.safe_load(stream)['/**']['ros__parameters']['sensors']['a1']
    a1_yaw_rad = math.radians(float(a1['yaw_deg']))
    avoid_params = os.path.join(
        get_package_share_directory('stack_avoid'), 'config', 'params.yaml')
    with open(avoid_params) as stream:
        legacy_forward = yaml.safe_load(stream)['/**'][
            'ros__parameters']['lidar_mount']['forward_angle_deg']

    return LaunchDescription([
        DeclareLaunchArgument('REAL_VEHICLE_CONFIRM', default_value='NOT_CONFIRMED'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        # Unset calibration: do not assign operational search limits from guesses.
        DeclareLaunchArgument('zone_enter_confirm_samples', default_value=str(_yaml['zone_enter_confirm_samples'])),
        DeclareLaunchArgument('route_sequence_file', default_value='',
                              description='Ordered CSV/Zone manifest; empty keeps single-route operation'),
        DeclareLaunchArgument('route_start_id', default_value='', description='Explicit sequence start ID: 01 or 02 for Halla'),
        DeclareLaunchArgument('route_end_id', default_value='', description='Explicit sequence end ID: 06 or 07 for Halla'),
        DeclareLaunchArgument('zone_exit_confirm_samples', default_value=str(_yaml['zone_exit_confirm_samples'])),
        DeclareLaunchArgument('parking_zone_entry_active', default_value=str(_yaml['parking_zone_entry_active']).lower(),
                              description='Enter Parking/GPS search at Zone entry; maneuver after ready; done or CSV endpoint releases'),
        DeclareLaunchArgument('parking_search_zone_only', default_value=str(_yaml['parking_search_zone_only']).lower(),
                              description='Search only inside the source Mission Zone; exit records failure'),
        DeclareLaunchArgument('parking_search_timeout', default_value='-1.0'),
        DeclareLaunchArgument('max_parking_search_distance', default_value='-1.0'),
        DeclareLaunchArgument(
            'v_base', default_value=str(v_base_default),
            description='MGM normal target speed [m/s]'),
        DeclareLaunchArgument(
            'escape_after_cycles', default_value=str(escape_after_cycles_default),
            description='Consecutive E-stop ticks before reverse escape; 0 disables escape'),

        # ── stack_gps (DRIVE_GUIDE.md V2와 동일 인자)
        DeclareLaunchArgument('waypoint_csv', default_value='',
                              description='코스 웨이포인트 CSV (필수)'),
        DeclareLaunchArgument('rtcm_host', default_value='127.0.0.1'),
        DeclareLaunchArgument(
            'parking_enabled', default_value='true',
            description=(
                'Enable integrated four-LiDAR parking and replace this '
                "launch's single front-LiDAR driver; false restores the "
                'legacy lane/GPS/avoid-only sensor setup')),
        DeclareLaunchArgument(
            't_parking_zone_ranges', default_value='[0]',
            description='T/perpendicular waypoint ranges [start,end,...]'),
        DeclareLaunchArgument(
            'parallel_parking_zone_ranges', default_value='[0]',
            description='Parallel-parking waypoint ranges [start,end,...]'),

        # GPS: bounded station search and one fixed +2.5m preview.

        # ── 지정 지점 정지 (2026-08-18) — 트랙 위 특정 장소에서 자동으로 서고
        # 다시 출발한다 (언덕 정차 시험). 지점은 **구간 파일**(mark_zone 이 기록)에서
        # 오고, stack_gps 가 위경도 → 웨이포인트 인덱스 구간으로 바꿔
        # GpsPath.stop_zone 에 싣는다. "정지하고 N초 뒤 재출발"이라는 판단은 MGM
        # 스테이트 머신에만 있다 (CLAUDE.md §4·§5.1).
        # 정차는 지점당 한 번이다 — 언덕에서 밀려 구간을 다시 밟아도 재정지하지
        # 않는다(정차를 마친 번호는 소진 처리). 지점 **안에서** launch 하면 그
        # 지점은 임시 억제로 시작하고, 구간을 벗어나면 억제가 풀린다.
        DeclareLaunchArgument(
            'zones_file', default_value='',
            description='구간 파일 경로 (빈 값 = waypoint_csv 옆 zones_*.yaml 자동)'),
        DeclareLaunchArgument(
            'stop_points_latlon', default_value='',
            description='구간 파일 외에 추가할 정지 지점 "lat,lon;lat,lon" (보통 비움)'),
        DeclareLaunchArgument('stop_hold_sec', default_value='3.0'),
        # dSPACE RX 피드백(/vehicle/vector) CSV. 토픽은 rosbag 에도 들어가지만
        # 실차 분석은 run 폴더 CSV 를 먼저 보므로 같은 자리에 둔다 (2026-08-25).
        DeclareLaunchArgument('vehicle_csv_path',
                              default_value=os.path.join(log_dir, 'vehicle_vector.csv')),
        DeclareLaunchArgument('stop_zone_span_m', default_value='1.0',
                              description='정지 지점 구간 폭 [m] (진입 판정 여유)'),
        DeclareLaunchArgument('parking_zone_span_m', default_value='1.0',
                              description='주차 지점 구간 폭 [m] (진입 판정 여유)'),

        # ── 회피 허용 구간 (2026-08-18). avoid_zone_only:=true 면 이 구간 **안에서만**
        # AVOID 전이가 일어난다.
        #
        # ★ 기본값은 **끔**이다 (2026-08-25 복구). CLAUDE.md §4·params.yaml 이 정한
        #   기본이 "어디서나 회피"인데, 2026-08-19 에 이 launch 만 true 로 켜 두었다가
        #   **원주 전용 운용 선택이 전 코스의 기본이 되어 버렸다.** 한라대에서 그대로
        #   터졌다 — run_mbd_0825_162752: stack_avoid 가 장애물을 잡고 회피 경로까지
        #   냈는데(avoidable 1.49s) 구간을 안 찍었다는 이유로 AVOID 에 못 들어가고
        #   그대로 직진하다 estop 정지. 구간 제한이 필요한 코스에서 **명시적으로 켤 것**:
        #     avoid_zone_only:=true
        #   (원주 운전면허시험장 절차: RUNBOOK_avoid_field_test.md)
        #   구간 지정: avoid_zone_latlon:="lat1,lon1,lat2,lon2" (구간의 시작·끝 좌표)
        # ⚠ 켠 상태에서 구간 밖 장애물을 만나면 회피가 아니라 stack_estop 정지로 대응한다
        #   (MGM 의 TTC 안전 바닥은 AVOID 스테이트 안에서만 걸리기 때문).
        DeclareLaunchArgument('avoid_zone_latlon', default_value=''),
        DeclareLaunchArgument('avoidance_enabled', default_value=str(_yaml['avoidance_enabled']).lower(),
                              description='Enable ordinary avoidance authority and its TTC stop; perception stays on'),
        DeclareLaunchArgument('avoid_zone_only', default_value='false'),

        # ── GPS 전용 모드: LANE 전이 차단 (히스테리시스 임계를 2.0으로 — confidence는
        # 최대 1.0이라 절대 도달 불가 → 항상 WAYPOINT). 야간 등 차선 오검출이 위험한
        # 조건에서 사용 (2026-08-12: 야간 오검출 conf 0.71로 LANE 전이 → 벽 방향 조향).
        # stack_lane은 그대로 떠서 데이터는 기록됨 — 오검출 사후 분석용.
        DeclareLaunchArgument('gps_only', default_value='false'),

        # ── 카메라 프로세스 자체를 띄우지 않음 (gps_only와 별개!).
        # gps_only는 LANE '전이'만 막고 stack_lane은 그대로 떠서 OAK-D가 USB3로
        # 1280x720 무압축 30fps(≈83MB/s)를 계속 흘린다. 이 트래픽이 GNSS L1을
        # 덮어 RTK FIXED가 무너지는 정황이 강하다 — `dai.Device()` 오픈 +2.5s에
        # FIXED 사망(2026-08-14 6/6 run, 편차 0.2s. 8/11~8/14 26 run 중 카메라
        # 오픈 전 사망 0건 / +8s 내 19건). RTCM 570B/s·GGA 8Hz·위성 12개는
        # 정상 run과 동일 = PC 소프트웨어 무죄, 물리계층(RF/전원) 문제.
        # lane_enabled:=false 로 카메라를 빼면 원인 확정 + GPS/회피 시험 가능.
        # 출발 인가는 `ros2 run adas_mgm go --skip-lane`.
        # ⚠ 차선 주행 자체를 시험하려면 이 인자로는 못 피한다 — 안테나 이격·
        # USB2 포트·camera_fps 하향 같은 물리 대책이 필요.
        DeclareLaunchArgument('lane_enabled', default_value='true'),

        # ── 로깅 (record:=false 는 rosbag만 끔 — CSV·스냅샷 덤프는 가벼워서 항상)
        DeclareLaunchArgument('record', default_value='true'),
        # 차선 검출 **진단 run 전용** (2026-08-15). 켜면 두 가지가 추가된다:
        #   ① /perception/lane_debug_image 발행 + rosbag 기록 (720p 10Hz ≈ 28MB/s
        #      — 200초에 약 5.5GB. 분석 끝나면 지울 것)
        #   ② stack_lane 프레임 CSV (width_m·mode·좌우 후보 수·피팅 잔차·계수)
        # 근거: 두 바퀴를 트랙 인덱스로 겹치니 헤딩오차 상관 +0.79, 차선 ly0 상관
        # +0.80, 부호 일치 14/17 — 위빙이 제어 진동이 아니라 **코스 위치에 고정된
        # 인지 바이어스**로 확정됐다(2026-08-15 run_0815_175044·175602). 원인이
        # 호모그래피인지·검출인지·웨이포인트 기준선 차이인지 가르려면 검출 내부가
        # 필요한데, 지금은 최종 20점만 남아 판별이 불가능하다.
        # ⚠ 상시로 켜지 말 것 — 디스크와 CPU를 크게 먹는다. 켠 run에서는 반드시
        #   mgm_jitter.csv(주기 지터)와 stack_lane의 파이프라인 지연 로그를 확인해
        #   기록 부하가 제어 루프를 건드리지 않았는지 확인할 것.
        DeclareLaunchArgument('lane_debug', default_value='false'),
        DeclareLaunchArgument('traffic_show_debug', default_value='false',
                              description='Show the traffic camera detection window'),
        DeclareLaunchArgument('lane_csv', default_value='false',
                              description='Frame CSV without debug image overhead'),
        DeclareLaunchArgument('gps_error_log_csv',
                              default_value=os.path.join(log_dir, 'lateral.csv')),

        # ── stack_lane
        DeclareLaunchArgument('homography_path', default_value=default_homography),
        # 가중치도 소스 트리 절대경로 필수 (기본값은 설치본 내부로 해석 — 파일 없음).
        # yolopv2.pt는 gitignore 대상(156MB) — 새 PC엔 공식 릴리즈에서 수동 다운로드.
        DeclareLaunchArgument('lane_weights', default_value=default_lane_weights),
        DeclareLaunchArgument('camera_mxid', default_value='14442C105157D3D200',
                              description='차선용 OAK-D MxID (2026-08-11 실측)'),
        # USB3 트래픽 = 1280x720x3 x fps. 기본 30fps는 ≈83MB/s인데 YOLOPv2 추론은
        # XPU에서 172ms(5.8Hz)라 실사용량의 5배를 흘리는 셈 — RTK 간섭이 확인되면
        # camera_fps:=10 (≈28MB/s)이 성능 손실 없는 1차 완화책 (2026-08-14).
        DeclareLaunchArgument('camera_fps', default_value='10'),
        # 'high' = 카메라를 USB2로 강제 → SuperSpeed 신호가 사라져 GPS 간섭의
        # 주 원인이 제거된다. 반드시 camera_fps:=10 과 함께 쓸 것 (2026-08-14).
        # ★ 기본값을 'high'/10 으로 뒤집었다 (2026-08-24, 인수인계). USB3 로
        #   열거되면 GNSS L1 이 덮여 RTK 가 죽는데, 그걸 피하려면 매 launch 마다
        #   인자를 손으로 붙여야 했다. 한 번 잊으면 위성 수도 HDOP 도 RTCM 도
        #   정상으로 보이는 채 FIXED 만 안 잡혀 원인 찾기가 어렵다 — C/N0(GSV)를
        #   봐야 보인다. 안전한 쪽을 기본으로 두고, USB3 가 필요하면 그때 올린다:
        #     ros2 launch ... usb_speed:=super camera_fps:=30
        DeclareLaunchArgument('usb_speed', default_value='high'),

        # ── 신호등·정지선 (stack_traffic, OAK-D 2번째 대) ─────────────────────
        # ★ 기본 꺼짐이다. 켜면 두 가지가 바뀐다:
        #   ① 카메라가 2대가 되어 USB2 대역폭을 나눠 쓴다 (아래 대역폭 주석)
        #   ② 적색+정지선에서 v_ref 0 이 걸린다 (MGM §5.7 ③ traffic watchdog 도
        #      "수신 이력이 있은 뒤" 활성화되므로, 안 띄우면 지금과 완전히 같다)
        # 회피 구간 게이트가 원주 전용 선택인 채 전 코스 기본이 돼 한라대에서
        # 회피를 통째로 막았던 사고(2026-08-25, CLAUDE.md §4)와 같은 계열이라,
        # 시나리오 기능은 인자로 켠다.
        #   ros2 launch ... traffic_enabled:=true
        DeclareLaunchArgument('traffic_enabled', default_value='false'),
        # 운영 런북 전용 fail-closed 가드. 측정 런북은 false로 두어 y gate 0을 허용한다.
        DeclareLaunchArgument('traffic_require_stop_gate', default_value='false'),
        # 신호등용 OAK-D MxID (CLAUDE.md §6 정본표). 차선용과 반드시 달라야 한다 —
        # 핀닝이 없거나 겹치면 어느 노드가 어느 카메라를 잡을지 부팅 순서에 좌우된다.
        DeclareLaunchArgument('traffic_mxid', default_value='14442C10B167CFD200'),
        # 신호등 RGB 자동 노출 보정: -2=두 단계 어둡게, 0=원복 (SDK -9..9).
        DeclareLaunchArgument('traffic_exposure_compensation', default_value='-2'),
        # ⚠ USB2 공유 대역폭 — 두 카메라가 같은 허브(2026-08-27 확정 배치의 허브 A)에
        #   물려 있고 둘 다 USB2(480Mbps, 실효 ~40MB/s)다. 비압축 BGR 3B/px 기준:
        #     차선   1280x720@10 = 27.65 MB/s
        #     신호등 1280x720@10 = 27.65 MB/s   → 합계 55.3 MB/s  ★ 실효치 초과
        #     신호등  640x360@10 =  6.91 MB/s   → 합계 34.6 MB/s  (여유 있음)
        #   stack_traffic 의 대역폭 검사는 **카메라 한 대씩만** 본다(36MB/s 상한)
        #   → 각각은 통과하지만 합계는 못 본다. 2대 동시 fps 는 실차 미검증이므로
        #   기본을 640x360 으로 둔다. 신호등이 멀어 안 잡히면 그때 올리고
        #   (traffic_width:=1280 traffic_height:=720) 양쪽 fps 를 실측할 것.
        DeclareLaunchArgument('traffic_width', default_value='640'),
        DeclareLaunchArgument('traffic_height', default_value='360'),
        # 정지선 depth는 진단값일 뿐 정지 조건이 아니다. 통합 주행은 RGB-only로
        # USB2 여유와 처리 지연을 우선하고, optical-Z 현장 진단 때만 켠다.
        DeclareLaunchArgument('traffic_depth_enabled', default_value='false'),
        # 2026-09-13 저조도 추가학습 모델은 입력 640에서 검증했다.
        # 320에서는 작은 신호등을 놓치므로 검증된 입력 크기를 사용한다.
        # 적색 전에는 격프레임, 적색 뒤에는 정지선을 우선하고 3프레임마다 신호등을 재확인한다.
        # 한 callback에서는 두 YOLO 중 하나만 실행한다.
        DeclareLaunchArgument('traffic_yolo_image_size', default_value='640'),
        DeclareLaunchArgument(
            'traffic_yolo_inference_interval', default_value='2'),
        DeclareLaunchArgument(
            'traffic_red_phase_yolo_inference_interval', default_value='3'),
        DeclareLaunchArgument(
            'traffic_stopline_yolo_image_size', default_value='320'),
        # 정지 게이트. 0 = **측정 전용**(정지 요구를 만들지 않는다). 첫 실차 세션은
        # 이 상태로 돌려 로그의 y_ratio 분포를 보고 값을 정한다 — 현장값 0.98 은
        # 옛 ROI·고정 장착·0.28m/s 이하에서만 검증됐고, 카메라 장착·ROI·속도가
        # 바뀌면 재보정 대상이다 (stack_traffic/REQUIREMENTS.md).
        DeclareLaunchArgument('traffic_stop_y_ratio', default_value='0.0'),
        # NVIDIA 없음 — 인텔 iGPU를 XPU 백엔드로 쓴다.
        #   산업용 PC (Arc 140V)      fp16 172ms/frame ≈ 5.8Hz   (2026-08-11)
        #   Xanadu-book5 (Lunar Lake) fp16  35ms/frame ≈ 28.7Hz  (2026-08-25)
        #   같은 노트북 CPU            fp32 617ms/frame ≈ 1.6Hz   (17.6배 차이)
        #
        # ⚠ XPU 는 컴퓨트 런타임이 있어야 뜬다. resolve_device()는 없을 때 폴백하지
        # 않고 RuntimeError 를 던지므로(yolopv2_infer.py:26) stack_lane 이 기동 즉시
        # 죽고 /perception/lane_path 퍼블리셔가 0이 된다 — go 점검 ②가 막힌다.
        # 새 PC 에서 `torch.xpu.is_available() == False` 면 런타임 미설치다:
        #   ~/intel_gpu_runtime/README.md  (22.04 는 compute-runtime 25.13 이 상한 —
        #   25.18 부터 glibc 2.38 을 요구해 설치 자체가 안 된다)
        # 급하면 lane_device:=cpu 로 폴백 (느리지만 뜨긴 한다).
        DeclareLaunchArgument('lane_device', default_value='xpu',
                              description="YOLOPv2 추론 장치: 'xpu'(인텔 GPU)/'cpu'/cuda 인덱스"),
        # Camera preview is fixed at station +2.5m; only coefficient smoothing remains tunable.
        DeclareLaunchArgument('coeff_smoothing_alpha', default_value='0.3'),

        # ── stack_estop (REAL_VEHICLE_stack_estop_mgm_can과 동일)
        DeclareLaunchArgument('ydlidar_params', default_value=DEFAULT_YDLIDAR_PARAMS),
        # /dev/ttyUSB0 고정 금지 — 이 PC에선 USB0=무전기, USB1=IMU, USB2=라이다로
        # 열거된다 (2026-08-11 확인). udev 별칭(ttyUSB_LIDAR, MODE 0666)으로 고정.
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB_LIDAR'),
        DeclareLaunchArgument('laser_yaw_in_base_rad', default_value=PythonExpression([
            str(a1_yaw_rad), " if '", LaunchConfiguration('parking_enabled'),
            "' == 'true' else 1.57079632679",
        ]), description='Scan yaw: calibrated a1 for four LiDARs; shared with avoidance in that mode'),
        DeclareLaunchArgument('dynamic_enabled', default_value='true'),
        DeclareLaunchArgument('dynamic_stop_distance_m', default_value='1.35'),
        # ── 정적 장애물 estop 문턱 [m] (2026-08-18, v_base 0.6→1.0 과 세트).
        # §5-1c 실측식: 필요거리 = 0.303·v + 1.19·(0.13·v + v²/(2·0.94))
        #   0.6 → 0.49m | 1.0 → 1.09m.  0.70/0.80 은 **0.6 m/s 전용**이었다.
        # 1.0 m/s 로 달리면서 그대로 두면 문턱에서 멈추기 전에 닿는다.
        # ⚠ 속도를 되돌릴 땐 이 값도 함께 되돌릴 것.
        DeclareLaunchArgument('estop_on_distance_m', default_value='1.20'),
        DeclareLaunchArgument('estop_off_distance_m', default_value='1.35'),
        DeclareLaunchArgument('dynamic_tracking_max_distance_m', default_value='3.00'),
        DeclareLaunchArgument('estop_corridor_max_x_m', default_value='1.50'),
        DeclareLaunchArgument('dynamic_roi_max_x_m', default_value='1.50'),
        DeclareLaunchArgument('avoid_target_speed_mps', default_value='1.0'),
        DeclareLaunchArgument('ttc_stop', default_value=str(_yaml['ttc_stop'])),
        DeclareLaunchArgument('v_accel_zone', default_value=str(_yaml['v_accel_zone'])),

        OpaqueFunction(function=validate, args=[log_dir, lidar_estop_enabled]),

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
                # ★ 스캔 좌우 방향 규약 — stack_avoid가 검증된 규약(ydlidar.yaml,
                # reversion/inverted=false)으로 고정. Tmini-Plus-SH.yaml의 true/true는
                # 전방은 보존하면서 좌우를 거울상으로 만들어, 회피가 열린 쪽의 반대
                # (막힌 쪽)로 조향했다 (2026-08-13 run_0813_001140 — 우측 벽인데 우조향,
                # bag /scan 재구성으로 규명). estop은 전방 거리만 봐서 영향 없음.
                'reversion': False,
                'inverted': False,
            }],
            output='screen',
            emulate_tty=True,
            # USB 허브 재열거 순간에 launch가 뜨면 드라이버가 포트 열다 SIGABRT로
            # 죽는다 (2026-08-13 00:23 실측 — 파라미터 문제 아님, 단독 재실행 정상).
            # 재시작으로 자가 회복. 출발 인가는 go 점검 ③(scan 수신)이 계속 막는다.
            respawn=True,
            respawn_delay=2.0,
            condition=UnlessCondition(LaunchConfiguration('parking_enabled')),
        ),

        # Four-LiDAR mapping + parking producer. It publishes
        # /perception/parking only; MGM remains the sole TargetRef owner.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                get_package_share_directory('stack_parking'),
                'launch', 'parking.launch.py'])),
            launch_arguments={'start_multi_lidar': 'true'}.items(),
            condition=IfCondition(LaunchConfiguration('parking_enabled')),
        ),

        # laser_static_tf(placeholder)는 제거 — base_link→laser_frame 은 stack_avoid_node가
        # 실측값(0.76, 0, 0.065 + forward_angle 반영)으로 발행한다. 같은 TF를 두 곳이
        # 발행하면 어느 쪽이 이길지 RViz 기동 타이밍에 따라 달라진다 (2026-08-09 규명).

        # ── stack_avoid (2026-08-12 통합) — 장애물 감지·회피 목표점 → MGM avoid 스테이트.
        # 파라미터 단일 소스 = stack_avoid/config/params.yaml (target_speed_mps 1.0 =
        # MGM v_base와 일치 유지할 것 — 2026-08-18에 둘 다 0.6→1.0).
        # 현장 튜닝: ros2 param set /stack_avoid_node ...
        Node(
            package='stack_avoid',
            executable='stack_avoid_node',
            name='stack_avoid_node',
            parameters=[avoid_params, {
                    'target_speed_mps': ParameterValue(
                        LaunchConfiguration('avoid_target_speed_mps'), value_type=float),
                    'scan_topic': PythonExpression([
                        "'/lidar/a1/scan' if '",
                        LaunchConfiguration('parking_enabled'),
                        "' == 'true' else '/scan'",
                    ]),
                    'lidar_mount.forward_angle_deg': ParameterValue(PythonExpression([
                        "(-float('", LaunchConfiguration('laser_yaw_in_base_rad'),
                        "') * ", str(180.0 / math.pi), ") % 360.0 if '",
                        LaunchConfiguration('parking_enabled'),
                        "' == 'true' else ", str(float(legacy_forward)),
                    ]), value_type=float),
                }],
            output='screen',
        ),

        Node(
            package='stack_estop',
            executable='stack_estop_node',
            name='stack_estop_node',
            condition=IfCondition(str(lidar_estop_enabled).lower()),
            remappings=[('/scan', PythonExpression([
                "'/lidar/a1/scan' if '",
                LaunchConfiguration('parking_enabled'),
                "' == 'true' else '/scan'",
            ]))],
            parameters=[{
                'laser_yaw_in_base_rad': ParameterValue(
                    LaunchConfiguration('laser_yaw_in_base_rad'), value_type=float),
                'corridor_max_x_m': ParameterValue(
                    LaunchConfiguration('estop_corridor_max_x_m'), value_type=float),
                'dynamic_roi_max_x_m': ParameterValue(
                    LaunchConfiguration('dynamic_roi_max_x_m'), value_type=float),
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
                'n_points': 1,
                'parking_zone_ranges': ParameterValue(
                    LaunchConfiguration('t_parking_zone_ranges'),
                    value_type=List[int]),
                'parallel_parking_zone_ranges': ParameterValue(
                    LaunchConfiguration('parallel_parking_zone_ranges'),
                    value_type=List[int]),
                # 지정 구간 (위경도 문자열 → 노드가 인덱스 구간으로 변환)
                # validate() 가 확정한 경로 (인자 지정 또는 트랙 CSV 옆 zones_*.yaml)
                'zones_file': LaunchConfiguration('zones_file_resolved'),
                'stop_points_latlon': LaunchConfiguration('stop_points_latlon'),
                'avoid_zone_latlon': LaunchConfiguration('avoid_zone_latlon'),
                'stop_zone_span_m': ParameterValue(
                    LaunchConfiguration('stop_zone_span_m'), value_type=float),
                'parking_zone_span_m': ParameterValue(
                    LaunchConfiguration('parking_zone_span_m'), value_type=float),
                'route_sequence_file': LaunchConfiguration('route_sequence_file'),
                'route_start_id': ParameterValue(LaunchConfiguration('route_start_id'), value_type=str),
                'route_end_id': ParameterValue(LaunchConfiguration('route_end_id'), value_type=str),
            }],
            output='screen',
            # 이 노드가 죽으면 launch 전체를 내린다 (2026-08-15). 예전에는 혼자
            # 죽어도 나머지가 계속 돌아, gps_path 없이 **카메라만 보고 주행**하는
            # 상태가 됐다 — 스테이트가 LANE이면 MGM의 gps watchdog도 안 걸린다.
            # 종료 경로를 타야 can_zero가 dSPACE 목표값 0을 송신한다(§3 주의).
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
                'camera_fps': ParameterValue(
                    LaunchConfiguration('camera_fps'), value_type=int),
                'usb_speed': LaunchConfiguration('usb_speed'),
                'device': LaunchConfiguration('lane_device'),
                # Camera returns one point at current station +2.5m (fixed in stack_lane).
                # 진단 run 전용 (lane_debug 인자 주석 참조)
                'publish_debug_image': ParameterValue(
                    LaunchConfiguration('lane_debug'), value_type=bool),
                'log_csv': PythonExpression(
                    ["'", os.path.join(log_dir, 'lane_frames.csv'),
                     "' if ('", LaunchConfiguration('lane_debug'),
                     "' == 'true' or '", LaunchConfiguration('lane_csv'),
                     "' == 'true') else ''"]),
                'coeff_smoothing_alpha': ParameterValue(
                    LaunchConfiguration('coeff_smoothing_alpha'), value_type=float),
            }],
            output='screen',
        ),

        # 신호등·정지선 — 2번째 OAK-D. traffic_enabled:=true 일 때만 뜬다.
        # MGM 은 /perception/traffic_stop 을 구독만 하고(§5.7 ③), 이 노드가
        # 없으면 watchdog 도 잠들어 있으므로 껐을 때 거동은 지금과 동일하다.
        Node(
            package='stack_traffic',
            executable='stack_traffic_node',
            name='stack_traffic_node',
            condition=IfCondition(LaunchConfiguration('traffic_enabled')),
            # lane/traffic OAK-D를 동시에 열 때 DepthAI 장치 열거 경쟁으로 traffic이
            # 시작 직후 exit 1 하는 실차 사례가 있다. MGM watchdog은 traffic을 한 번도
            # 수신하지 못한 시작 실패에는 개입하지 못하므로 launch가 반드시 복구한다.
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                # 노드 기본은 'opencv'(USB 웹캠) — 실차는 반드시 oak 로 바꾼다.
                'camera_backend': 'oak',
                'show_debug': ParameterValue(
                    LaunchConfiguration('traffic_show_debug'), value_type=bool),
                'oak_mxid': LaunchConfiguration('traffic_mxid'),
                'oak_exposure_compensation': ParameterValue(
                    LaunchConfiguration('traffic_exposure_compensation'), value_type=int),
                # 차선 카메라와 같은 대책을 공유한다 — USB3 로 열거되면 GNSS L1 이
                # 덮여 RTK 가 죽는다 (CLAUDE.md §6). 두 카메라가 따로 놀면 안 된다.
                'oak_usb_speed': LaunchConfiguration('usb_speed'),
                'oak_fps': ParameterValue(
                    LaunchConfiguration('camera_fps'), value_type=float),
                'oak_width': ParameterValue(
                    LaunchConfiguration('traffic_width'), value_type=int),
                'oak_height': ParameterValue(
                    LaunchConfiguration('traffic_height'), value_type=int),
                'oak_depth_enabled': ParameterValue(
                    LaunchConfiguration('traffic_depth_enabled'), value_type=bool),
                'yolo_image_size': ParameterValue(
                    LaunchConfiguration('traffic_yolo_image_size'),
                    value_type=int),
                'yolo_inference_interval': ParameterValue(
                    LaunchConfiguration('traffic_yolo_inference_interval'),
                    value_type=int),
                'red_phase_yolo_inference_interval': ParameterValue(
                    LaunchConfiguration(
                        'traffic_red_phase_yolo_inference_interval'),
                    value_type=int),
                # y gate 를 쓰려면 정지선 검출이 켜져 있어야 한다(노드가 검증).
                'stopline_detection_enabled': True,
                'stopline_yolo_image_size': ParameterValue(
                    LaunchConfiguration(
                        'traffic_stopline_yolo_image_size'), value_type=int),
                'stopline_stop_y_ratio': ParameterValue(
                    LaunchConfiguration('traffic_stop_y_ratio'), value_type=float),
                # 시연 신호등은 적색=정지 / 초록=재출발 타입 (2026-08-09 팀장 결정,
                # CLAUDE.md §6). 패키지 기본은 자동 해제 없음이라 실차에서 켠다.
                'resume_on_green': True,
                'resume_on_red_clear': False,
            }],
            output='screen',
        ),

        Node(
            package='adas_mgm',
            executable='mgm_node',
            name='mgm_node',
            parameters=[mgm_params, {   # 기존 REAL_VEHICLE launch의 params 누락 수정
                'lidar_estop_enabled': lidar_estop_enabled,
                # run별 진단 산출물 — back-to-back 재현(§5.5)과 지터 판정(§7)
                'snapshot_dump_path': os.path.join(log_dir, 'mgm_snapshots.bin'),
                'jitter_csv_path': os.path.join(log_dir, 'mgm_jitter.csv'),
                # 스테이트 전이 이유 (판단 아님 — 관찰 기록). MBD 시험과 같은
                # 포맷이라 두 run 을 그대로 대조할 수 있다.
                'transition_csv_path': os.path.join(log_dir, 'transitions.csv'),
                'mission_events_csv_path': os.path.join(log_dir, 'mission_events.csv'),
                'zone_enter_confirm_samples': ParameterValue(
                    LaunchConfiguration('zone_enter_confirm_samples'), value_type=int),
                'zone_exit_confirm_samples': ParameterValue(
                    LaunchConfiguration('zone_exit_confirm_samples'), value_type=int),
                'parking_search_zone_only': ParameterValue(LaunchConfiguration('parking_search_zone_only'), value_type=bool),
                'parking_zone_entry_active': ParameterValue(LaunchConfiguration('parking_zone_entry_active'), value_type=bool),
                'zone_observations_csv_path': os.path.join(log_dir, 'zone_observations.csv'),
                'parking_search_timeout': ParameterValue(
                    LaunchConfiguration('parking_search_timeout'), value_type=float),
                'max_parking_search_distance': ParameterValue(
                    LaunchConfiguration('max_parking_search_distance'), value_type=float),
                # 출발 인가 게이트 — launch 직후 정지 대기, `ros2 run adas_mgm go`
                # (RTK FIXED 등 점검 통과 시)로 출발 (2026-08-11)
                'wait_go': True,
                'route_sequence_enabled': ParameterValue(
                    LaunchConfiguration('route_sequence_enabled_resolved'), value_type=bool),
                # 시험별 목표속도. 기본은 params.yaml 값을 그대로 따르며, 실차 시험에서
                # 명시적으로 낮출 때만 launch 인자로 덮어쓴다.
                'ttc_stop': ParameterValue(
                    LaunchConfiguration('ttc_stop'), value_type=float),
                'v_accel_zone': ParameterValue(
                    LaunchConfiguration('v_accel_zone'), value_type=float),
                'v_base': ParameterValue(
                    LaunchConfiguration('v_base'), value_type=float),
                # E-stop 자체를 실패로 판정하는 시험에서는 반드시 0으로 두어, 장시간
                # 정지 후 후진 탈출이 시험 결과를 바꾸지 못하게 한다.
                'escape_after_cycles': ParameterValue(
                    LaunchConfiguration('escape_after_cycles'), value_type=int),
                # gps_only 시 LANE 전이 불가 임계로 상향 (위 gps_only 인자 참조).
                # 평상시 값은 params.yaml에서 읽은 것 — 여기 숫자를 박지 말 것.
                'lane_conf_exit': ParameterValue(PythonExpression(
                    ["2.0 if '", LaunchConfiguration('gps_only'),
                     f"' == 'true' else {lane_exit_default}"]),
                    value_type=float),
                'lane_conf_return': ParameterValue(PythonExpression(
                    ["2.0 if '", LaunchConfiguration('gps_only'),
                     f"' == 'true' else {lane_return_default}"]),
                    value_type=float),
                # 지정 지점 정차 시간 [틱] = stop_hold_sec × 100 (10ms 루프)
                'stop_zone_hold_cycles': ParameterValue(PythonExpression(
                    ["int(round(float('", LaunchConfiguration('stop_hold_sec'), "') * 100))"]),
                    value_type=int),
                # 회피 허용 구간 밖 AVOID 전이 금지 (기본 끔 — 위 인자 주석 참조)
                'avoidance_enabled': ParameterValue(
                    LaunchConfiguration('avoidance_enabled'), value_type=bool),
                'avoid_zone_only': ParameterValue(
                    LaunchConfiguration('avoid_zone_only'), value_type=bool),
            }],
            output='screen',
            # MGM이 죽으면 TargetRef 송신이 끊긴다. dSPACE watchdog이 아직
            # 미구현이라(§3 ⚠) 마지막 v_ref를 무기한 유지하며 계속 굴러간다 —
            # 반드시 종료 경로를 타서 can_zero가 목표값 0을 보내게 한다.
            on_exit=die_hard('mgm_node',
                             '목표값 송신 중단 — can_zero로 0 복귀 후 전체 종료'),
        ),

        # ── rosbag — 버그 사후 분석·재생용. 토픽 명시 목록(RECORD_TOPICS)만 기록
        # 평상시 — 명시 토픽 목록만
        ExecuteProcess(
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('record'), "' == 'true' and '",
                 LaunchConfiguration('lane_debug'), "' != 'true'"])),
            cmd=['ros2', 'bag', 'record', '-o', os.path.join(log_dir, 'rosbag')]
                + RECORD_TOPICS,
            output='screen',
        ),
        # 차선 진단 run — 디버그 영상 추가 (lane_debug 인자 주석 참조).
        # 720p 10Hz raw = 약 28MB/s. 압축은 일부러 쓰지 않는다 — 기록 중 CPU를
        # 더 먹으면 MGM 10ms 루프 지터에 영향이 갈 수 있고, 어차피 분석 뒤 지운다.
        ExecuteProcess(
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('record'), "' == 'true' and '",
                 LaunchConfiguration('lane_debug'), "' == 'true'"])),
            cmd=['ros2', 'bag', 'record', '-o', os.path.join(log_dir, 'rosbag')]
                + RECORD_TOPICS + ['/perception/lane_debug_image'],
            output='screen',
        ),

        # CAN 브리지 + 종료 시 목표값 0 복귀 (2026-08-12 — 기존엔 브리지만 있어서
        # Ctrl-C 후 dSPACE가 마지막 v_ref를 latch한 채 계속 굴러갈 수 있었다)
        *can_bridge_with_zero_guard(
            can_interface=LaunchConfiguration('can_interface'),
            vehicle_csv_path=LaunchConfiguration('vehicle_csv_path')),
    ])


def generate_launch_description():
    return build_launch_description()
