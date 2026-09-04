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
    '/scan', '/lidar/a1/scan', '/unified_lidar/scan',
    '/parking/local_map', '/parking/slam_pose', '/parking/pipeline_stage',
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


def validate(context):
    if LaunchConfiguration('REAL_VEHICLE_CONFIRM').perform(context) != CONFIRM_TOKEN:
        raise RuntimeError(
            'REAL VEHICLE launch refused. '
            'Set REAL_VEHICLE_CONFIRM:=' + CONFIRM_TOKEN)
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
            n_stop, n_avoid = len(stops), len(avoids)
            print(f'[launch] 구간 파일: {os.path.basename(zones_file)} '
                  f'(정지 {n_stop} · 회피 {n_avoid} · GPS전용 {len(gonly)})')
            if gonly:
                print(f'[launch]   GPS 전용 구간 {len(gonly)}개 — 그 안에서는 차선 전이 없음 '
                      '(run 전체를 GPS로만 가려면 gps_only:=true)')
            for i, e in enumerate(stops):
                note = f"  ({e.get('note')})" if e.get('note') else ''
                print(f'[launch]   정지 {i + 1}: {e.get("lat")},{e.get("lon")}{note}')
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
    os.makedirs(LOG_DIR, exist_ok=True)
    print(f'[record] 로그 디렉터리: {LOG_DIR}')
    # 확정한 구간 파일 경로를 노드에 넘긴다. 이 OpaqueFunction 은 LaunchDescription
    # 목록에서 노드들보다 **앞**에 있으므로 여기서 설정한 값이 아래 Node 에 잡힌다.
    return [SetLaunchConfiguration('zones_file_resolved', zones_file)]


def generate_launch_description():
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
    escape_after_cycles_default = int(_yaml['escape_after_cycles'])

    return LaunchDescription([
        DeclareLaunchArgument('REAL_VEHICLE_CONFIRM', default_value='NOT_CONFIRMED'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
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

        # ── GPS 재합류 기하 (2026-08-17). 셋 다 "dSPACE 조향이 느리던 시절"의
        # 보상값이라 조향 PI 도입 후 재조정 대상이다. 주행 중에도 바꿀 수 있다:
        #   ros2 param set /stack_gps_node rejoin_full_cross_m 1.0
        # 빈 값 = stack_gps 노드 기본값 사용(= path_engine.py 의 REJOIN_*).
        DeclareLaunchArgument('ref_lookahead_m', default_value='1.0'),
        DeclareLaunchArgument('rejoin_rate_damp_s', default_value='0.0'),
        DeclareLaunchArgument('rejoin_full_cross_m', default_value='0.5'),
        DeclareLaunchArgument('rejoin_target_max_m', default_value='1.8'),
        # 하한 — 실측상 ref[0] 거리의 82%가 하한에 붙으므로 **실효 이득은 이 값이
        # 정한다**. 1.267(기하 바닥) → 1.8 로 올린 것이 2026-08-17 잡음 대책의 핵심.
        DeclareLaunchArgument('rejoin_target_min_m', default_value='1.8'),
        # 접근각이 쓰는 횡오차의 저역통과 [s]. 0 = 끔. ψₑ 쪽엔 절대 걸지 말 것.
        DeclareLaunchArgument('rejoin_e_lpf_s', default_value='0.15'),

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
                              default_value=os.path.join(LOG_DIR, 'vehicle_vector.csv')),
        DeclareLaunchArgument('stop_zone_span_m', default_value='1.0',
                              description='정지 지점 구간 폭 [m] (진입 판정 여유)'),

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
        DeclareLaunchArgument('gps_error_log_csv',
                              default_value=os.path.join(LOG_DIR, 'lateral.csv')),

        # ── stack_lane
        DeclareLaunchArgument('homography_path', default_value=DEFAULT_HOMOGRAPHY),
        # 가중치도 소스 트리 절대경로 필수 (기본값은 설치본 내부로 해석 — 파일 없음).
        # yolopv2.pt는 gitignore 대상(156MB) — 새 PC엔 공식 릴리즈에서 수동 다운로드.
        DeclareLaunchArgument('lane_weights', default_value=os.path.expanduser(
            '~/FMA_ws/src/stack_lane/models/yolopv2.pt')),
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
        # CPU에서 100ms 처리 예산을 지키는 통합 주행 프로필. 적색 전에는 신호등을
        # 격프레임으로, 적색 뒤에는 정지선을 우선하고 3프레임마다 신호등을 재확인한다.
        # 한 callback에서는 두 YOLO 중 하나만 실행한다.
        DeclareLaunchArgument('traffic_yolo_image_size', default_value='320'),
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
        # 1.8 → 0.0(외삽 끔, ref[0] = points_x_start 2.5m 균일) — 2026-08-15.
        # 구 값 1.8은 TESTING_LOG §7.3의 v_base 0.5 시절 잠정 최적값이다(표본 51초).
        # **차선 추종 루프는 이득 과다다** — 목표를 당길수록(=이득↑) 위빙이 커진다:
        #   ref[0] 2.08m (run_0815_163614)  |str| 0.0566  헤딩 표준편차  9.22°
        #   ref[0] 1.88m (run_0815_170539)  |str| 0.0875  헤딩 표준편차 10.83°
        # 목표를 0.2m 당겼더니 조향량 +55%, 위빙 +17%로 되레 나빠졌다. 지연은
        # 0.44→0.27s로 줄었는데도 그렇다 — 지연이 아니라 이득이 지배한다.
        # v_base를 0.5→0.6으로 올리면서 실현율이 14%→24%로 올라간 것(§3 ①)도
        # 같은 방향으로 이득을 밀어올렸다. 미리보기 시간 = L/v 로 보면 1.8m@0.5는
        # 3.6s인데 0.6에서 같은 3.6s를 쓰려면 2.16m가 필요하다 — 즉 속도를 올린
        # 만큼 lookahead도 나갔어야 했다.
        # 0.0으로 두면 외삽 자체가 꺼져 ref[0]이 카메라 최소 가시거리 2.5m로
        # **균일**해진다(이득 최저 + 변조 없음). 이득 가설의 깨끗한 검증이다.
        # ⚠ 이 값을 다시 당길 땐 v_base와 함께 볼 것 — 짝지어 움직여야 한다.
        # 0.0(외삽 끔) → 2.0 (2026-08-16). PR #35로 이 인자의 의미가 **고정 거리에서
        # "가장 공격적인 기준값"으로 바뀌었다** — 실제 거리는 c0(드리프트)·c2(곡률)에
        # 따라 이 값 ~ points_x_start(2.5m) 사이에서 매 프레임 정해지고, R_min 하한이
        # 하드 가드로 걸린다. 이탈이 클수록 물러나므로 "가까운 점을 조준해 조향이
        # 포화되던" 실패 모드가 구조적으로 막힌다.
        #
        # ⚠ 값은 이현준 기본값 1.15가 아니라 **2.0**을 쓴다. 차선 추종 루프는
        # 이득 과다이며 목표를 당길수록 위빙이 커진다는 게 실측이다:
        #     고정 1.88m → 헤딩 표준편차 10.83°  (run_0815_170539)
        #     고정 2.08m → 9.22°                (run_0815_163614)
        #     고정 2.54m → 7.86°                (run_0815_172512)
        # 실측 5336프레임에 PR #35 알고리즘을 오프라인으로 돌려본 결과, base별로
        # **1.9m 미만이 차지하는 비율**이 이렇게 갈린다:
        #     base 1.15 → 84.9%   base 1.50 → 81.1%   base 1.80 → 69.3%
        #     base 2.00 →  0.0%   base 2.20 →  0.0%
        # 1.15는 위빙이 가장 심했던 1.88m보다 더 공격적인 영역에서 85%를 보낸다.
        # 2.0이면 가장 공격적일 때조차 실측으로 확인된 안전 구간(≥2.0m) 안이면서,
        # 이탈 시 2.5m로 물러나는 적응 동작은 그대로 시험된다.
        # 다음 run에서 위빙이 줄면 1.8 → 1.5 순으로 낮춰가며 최적점을 찾을 것.
        DeclareLaunchArgument('ref_point0_lookahead_m', default_value='2.0'),
        DeclareLaunchArgument('ref_point0_extrap_mode', default_value='linear'),
        # 0.5(stack_lane 기본) → 0.30 (2026-08-15). 이 게이트가 풀렸다 걸렸다 하면
        # ref[0] 거리가 1.8m ↔ points_x_start 2.5m 로 **0.70m 계단 점프**하고,
        # 조향 응답이 거리에 강하게 의존하므로(CLAUDE.md §3 ③) 루프 이득이 같이
        # 튄다. run_0815_163614 실측: 신뢰도 분포가 임계 0.5 바로 양옆에 최대 밀집
        # (0.4~0.5 1887틱 / 0.5~0.6 2186틱)이라 **120초에 82회, 평균 1.5초마다** 전환.
        # 그 구간 LANE 헤딩 표준편차 9.22°(GPS 6.07°)·명령→조향 지연 0.60s(GPS 0.18s).
        # 0.30이면 미적용이 33.1% → 1.1%로 떨어져 전환이 사실상 사라진다
        # (0.45→19.8%, 0.40→10.5%, 0.35→4.1%, 0.25→1.0% — 0.30이 평탄부 시작).
        # 안전성: 이 외삽은 **가시구간(2.5~6.0m) 폴리곤을 그대로 뒤로 평가**하는 것이라
        # 별도 추정이 아니다 — 실측 8213프레임에서 |가시구간 2차피팅 예측 − 실제 ref[0].y|
        # 중앙값 1.8mm·p90 7.6mm, 임계 바로 위(0.5~0.6) 구간도 2.5mm로 열화 없음.
        # 게다가 미적용 프레임이 오히려 더 단순한 경로였다(2차피팅 잔차 0.00003 vs
        # 0.00025m, |ly19| 0.34 vs 0.84m) — 신뢰도는 피팅 품질보다 차선 곡률·복잡도를
        # 따라간다. stack_lane 패키지 기본값(0.5)은 단독 시험용으로 그대로 두었다.
        DeclareLaunchArgument('ref_point0_min_confidence', default_value='0.30'),
        DeclareLaunchArgument('coeff_smoothing_alpha', default_value='0.3'),

        # ── stack_estop (REAL_VEHICLE_stack_estop_mgm_can과 동일)
        DeclareLaunchArgument('ydlidar_params', default_value=DEFAULT_YDLIDAR_PARAMS),
        # /dev/ttyUSB0 고정 금지 — 이 PC에선 USB0=무전기, USB1=IMU, USB2=라이다로
        # 열거된다 (2026-08-11 확인). udev 별칭(ttyUSB_LIDAR, MODE 0666)으로 고정.
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB_LIDAR'),
        DeclareLaunchArgument('laser_yaw_in_base_rad', default_value='1.57079632679'),
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
            parameters=[os.path.join(
                get_package_share_directory('stack_avoid'), 'config', 'params.yaml'), {
                    'scan_topic': PythonExpression([
                        "'/lidar/a1/scan' if '",
                        LaunchConfiguration('parking_enabled'),
                        "' == 'true' else '/scan'",
                    ]),
                }],
            output='screen',
        ),

        Node(
            package='stack_estop',
            executable='stack_estop_node',
            name='stack_estop_node',
            remappings=[('/scan', PythonExpression([
                "'/lidar/a1/scan' if '",
                LaunchConfiguration('parking_enabled'),
                "' == 'true' else '/scan'",
            ]))],
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
                'ref_point0_lookahead_m': ParameterValue(
                    LaunchConfiguration('ref_point0_lookahead_m'), value_type=float),
                'ref_point0_extrap_mode': LaunchConfiguration('ref_point0_extrap_mode'),
                'ref_point0_min_confidence': ParameterValue(
                    LaunchConfiguration('ref_point0_min_confidence'), value_type=float),
                # 진단 run 전용 (lane_debug 인자 주석 참조)
                'publish_debug_image': ParameterValue(
                    LaunchConfiguration('lane_debug'), value_type=bool),
                'log_csv': PythonExpression(
                    ["'", os.path.join(LOG_DIR, 'lane_frames.csv'),
                     "' if '", LaunchConfiguration('lane_debug'), "' == 'true' else ''"]),
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
                'oak_mxid': LaunchConfiguration('traffic_mxid'),
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
                # run별 진단 산출물 — back-to-back 재현(§5.5)과 지터 판정(§7)
                'snapshot_dump_path': os.path.join(LOG_DIR, 'mgm_snapshots.bin'),
                'jitter_csv_path': os.path.join(LOG_DIR, 'mgm_jitter.csv'),
                # 스테이트 전이 이유 (판단 아님 — 관찰 기록). MBD 시험과 같은
                # 포맷이라 두 run 을 그대로 대조할 수 있다.
                'transition_csv_path': os.path.join(LOG_DIR, 'transitions.csv'),
                # 출발 인가 게이트 — launch 직후 정지 대기, `ros2 run adas_mgm go`
                # (RTK FIXED 등 점검 통과 시)로 출발 (2026-08-11)
                'wait_go': True,
                # 시험별 목표속도. 기본은 params.yaml 값을 그대로 따르며, 실차 시험에서
                # 명시적으로 낮출 때만 launch 인자로 덮어쓴다.
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
            cmd=['ros2', 'bag', 'record', '-o', os.path.join(LOG_DIR, 'rosbag')]
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
            cmd=['ros2', 'bag', 'record', '-o', os.path.join(LOG_DIR, 'rosbag')]
                + RECORD_TOPICS + ['/perception/lane_debug_image'],
            output='screen',
        ),

        # CAN 브리지 + 종료 시 목표값 0 복귀 (2026-08-12 — 기존엔 브리지만 있어서
        # Ctrl-C 후 dSPACE가 마지막 v_ref를 latch한 채 계속 굴러갈 수 있었다)
        *can_bridge_with_zero_guard(
            can_interface=LaunchConfiguration('can_interface'),
            vehicle_csv_path=LaunchConfiguration('vehicle_csv_path')),
    ])
