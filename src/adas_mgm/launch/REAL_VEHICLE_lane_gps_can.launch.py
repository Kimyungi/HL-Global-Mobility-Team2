"""차선+GPS+회피 통합 실차 주행 launch — lane ↔ waypoint ↔ avoid 자동 전이 (CLAUDE.md §4).

한 번에 띄우는 노드 (2026-08-11 통합 점검 §3의 "터미널 5개" 조합을 대체):
  ydlidar + stack_estop  (REAL_VEHICLE_stack_estop_mgm_can과 동일 구성)
  stack_avoid  — 장애물 회피 (2026-08-12 통합). base_link→laser_frame TF도 이 노드가
                 실측값(stack_avoid params.yaml)으로 발행 — 예전의 placeholder
                 laser_static_tf는 제거 (같은 TF 2중 발행 시 비결정적, PR #23·2026-08-09 규명)
  stack_gps    — waypoint_csv 필수 인자
  stack_lane   — 실측 호모그래피·MxID 핀닝·오실레이션 잠정 튜닝(TESTING_LOG §7.3) 기본 적용
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

사용:
  ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py \
      REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
      waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/<코스>.csv
"""
import csv
import os
from datetime import datetime

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, Shutdown)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    '/scan', '/rosout', '/tf', '/tf_static',
]

# 실측 호모그래피 (2026-08-11 캘리브레이션, LOO RMS 0.041m) — 소스 트리 절대경로로
# 지정해야 한다: 노드 기본 경로는 설치본 내부로 해석돼 파일을 못 찾는다
# (stack_lane CALIBRATION_GUIDE.md §6).
DEFAULT_HOMOGRAPHY = os.path.expanduser(
    '~/FMA_ws/src/stack_lane/config/homography.json')

DEFAULT_YDLIDAR_PARAMS = os.path.join(
    os.path.expanduser('~'), 'ydlidar_ws', 'src', 'ydlidar_ros2_driver',
    'params', 'Tmini-Plus-SH.yaml')


def validate(context):
    if LaunchConfiguration('REAL_VEHICLE_CONFIRM').perform(context) != CONFIRM_TOKEN:
        raise RuntimeError(
            'REAL VEHICLE launch refused. '
            'Set REAL_VEHICLE_CONFIRM:=' + CONFIRM_TOKEN)
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
            '  실코스 예: waypoints_straight_1_20260811_193556.csv (288점)\n'
            '  1~4점 파일은 FIXED 확인용 잔여물이다.')
    print(f'[launch] 웨이포인트 {len(rows)}점 확인: {os.path.basename(waypoint_csv)}')
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
    os.makedirs(LOG_DIR, exist_ok=True)
    print(f'[record] 로그 디렉터리: {LOG_DIR}')
    return []


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

    return LaunchDescription([
        DeclareLaunchArgument('REAL_VEHICLE_CONFIRM', default_value='NOT_CONFIRMED'),
        DeclareLaunchArgument('can_interface', default_value='can0'),

        # ── stack_gps (DRIVE_GUIDE.md V2와 동일 인자)
        DeclareLaunchArgument('waypoint_csv', default_value='',
                              description='코스 웨이포인트 CSV (필수)'),
        DeclareLaunchArgument('rtcm_host', default_value='127.0.0.1'),

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
        DeclareLaunchArgument('camera_fps', default_value='30'),
        # 'high' = 카메라를 USB2로 강제 → SuperSpeed 신호가 사라져 GPS 간섭의
        # 주 원인이 제거된다. 반드시 camera_fps:=10 과 함께 쓸 것 (2026-08-14).
        DeclareLaunchArgument('usb_speed', default_value='super'),
        # 이 PC(산업용)는 NVIDIA 없음 — 인텔 Arc iGPU를 XPU 백엔드로 사용
        # (fp16 172ms/frame ≈ 5.8Hz, CPU 390ms 대비 2.3배 — 2026-08-11 실측).
        # XPU 초기화 실패 시(드라이버 문제 등) lane_device:=cpu 로 폴백.
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
        DeclareLaunchArgument('dynamic_stop_distance_m', default_value='1.20'),
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
        ),

        # laser_static_tf(placeholder)는 제거 — base_link→laser_frame 은 stack_avoid_node가
        # 실측값(0.76, 0, 0.065 + forward_angle 반영)으로 발행한다. 같은 TF를 두 곳이
        # 발행하면 어느 쪽이 이길지 RViz 기동 타이밍에 따라 달라진다 (2026-08-09 규명).

        # ── stack_avoid (2026-08-12 통합) — 장애물 감지·회피 목표점 → MGM avoid 스테이트.
        # 파라미터 단일 소스 = stack_avoid/config/params.yaml (target_speed_mps 1.0 =
        # MGM v_base와 일치 유지할 것 — 2026-08-15에 둘 다 0.5→0.6, 08-17에 0.6→1.0).
        # 현장 튜닝: ros2 param set /stack_avoid_node ...
        Node(
            package='stack_avoid',
            executable='stack_avoid_node',
            name='stack_avoid_node',
            parameters=[os.path.join(
                get_package_share_directory('stack_avoid'), 'config', 'params.yaml')],
            output='screen',
        ),

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

        Node(
            package='adas_mgm',
            executable='mgm_node',
            name='mgm_node',
            parameters=[mgm_params, {   # 기존 REAL_VEHICLE launch의 params 누락 수정
                # run별 진단 산출물 — back-to-back 재현(§5.5)과 지터 판정(§7)
                'snapshot_dump_path': os.path.join(LOG_DIR, 'mgm_snapshots.bin'),
                'jitter_csv_path': os.path.join(LOG_DIR, 'mgm_jitter.csv'),
                # 출발 인가 게이트 — launch 직후 정지 대기, `ros2 run adas_mgm go`
                # (RTK FIXED 등 점검 통과 시)로 출발 (2026-08-11)
                'wait_go': True,
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
            can_interface=LaunchConfiguration('can_interface')),
    ])
