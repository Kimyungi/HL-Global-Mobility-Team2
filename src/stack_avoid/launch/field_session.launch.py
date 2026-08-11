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
  # 회피 주행
  ros2 launch stack_avoid field_session.launch.py mode:=avoid v_ref:=0.4 dynamic:=true

튜닝은 **이 한 줄 안에서** 끝낸다 — 별도 터미널의 `ros2 param set` 은 필요 없다.
안 준 인자는 params.yaml 값이 그대로 쓰이므로 거동이 변하지 않는다.

  # 측방 오프셋 상한을 올려 더 멀리 있는 열림도 목표로 잡게 (N절)
  ros2 launch stack_avoid field_session.launch.py mode:=avoid v_ref:=0.4 dynamic:=true \
      offset_max:=1.3
  # 조향 크기를 키우기 (M-2 실행률 부족 대응)
  ros2 launch stack_avoid field_session.launch.py mode:=avoid v_ref:=0.4 lateral_margin:=0.25
  # 회차 메모 — 폴더 이름에 붙는다
  ros2 launch stack_avoid field_session.launch.py mode:=avoid v_ref:=0.4 \
      note:="콘 2개 3m 간격"
  # estop 정지거리 변경 — estop_off 는 안 줘도 on+0.10 으로 자동 파생된다
  ros2 launch stack_avoid field_session.launch.py mode:=avoid estop_on_distance_m:=0.90

★ 안전 노드(stack_estop)가 뜨지 못하면 **세션 전체가 중단된다** — estop 없이 굴러가는
  구성을 허용하지 않는다. 예전에는 그 노드만 조용히 죽고 나머지는 정상 기동해,
  "차가 안 움직인다" 하나로만 드러나 원인 찾기가 어려웠다 (2026-08-10).

별도 터미널에서 `ros2 run stack_avoid mark` 를 띄워 구간 라벨을 남길 것 —
bag의 /test/event 로 나중에 "이 구간이 무슨 시험이었나"를 복원한다.

로깅 (bag_dir 를 지정하지 않으면 전부 자동):
  <src/stack_avoid>/field_logs/<mode>_<YYYYmmdd_HHMMSS>            ← rosbag
  <src/stack_avoid>/field_logs/<mode>_<YYYYmmdd_HHMMSS>_params.yaml ← 노드 파라미터 스냅샷
  <src/stack_avoid>/field_logs/<mode>_<YYYYmmdd_HHMMSS>_can.log     ← raw CAN (step·avoid 만)
bag 은 "무엇을 publish 했나"만 남는다 — 어떤 설정이었나(_params.yaml)와 버스에 실제로
무엇이 나갔나(_can.log)를 같은 이름으로 옆에 남겨야 세션이 완결된다 (nsweep 세션에서
손으로 하던 것의 자동화). field_logs/ 는 .gitignore 대상.

★ mode:=step·avoid 는 실차 조향/구동이다. mgm_node·dummy_ref_publisher가 떠 있으면
  /adas/target_ref 이중 발행 — 먼저 종료할 것 (run_field_session.sh가 검사한다).
★ v_ref=0으로는 조향 측정 불가 — MPC 지평 = 0.2×v_ref 라 지평이 붕괴한다.
"""
import os
from datetime import datetime
from pathlib import Path
from typing import List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from stack_avoid.launch_parts import (can_bridge_with_zero_guard, safety_node,
                                      ydlidar_driver)

# /test/event 는 구간 라벨 — 이게 없으면 한 세션 bag을 나중에 못 자른다.
BAG_TOPICS = ['/scan', '/scan_front', '/perception/avoid', '/avoid_markers',
              '/perception/estop', '/perception/estop/status',
              '/perception/static_estop', '/perception/dynamic_estop',
              '/perception/dynamic_obstacle_detected',
              '/adas/target_ref', '/vehicle/vector', '/test/event',
              '/tf', '/tf_static', '/rosout']

DRIVES = ['step', 'avoid']      # 실차를 움직이는 모드 — CAN 브리지 필요


def _src_pkg_dir():
    """워크스페이스 소스 트리의 src/stack_avoid 절대경로.

    로그를 install/이 아니라 소스 트리 밑(field_logs/)에 남기기 위해, launch 파일
    위치에서 조상으로 올라가며 src/stack_avoid/package.xml 을 찾는다. symlink-install
    여부와 무관하게 동작한다 (install/share/... 도 조상에 워크스페이스 루트가 있다).
    """
    for p in Path(__file__).resolve().parents:
        cand = p / 'src' / 'stack_avoid'
        if (cand / 'package.xml').exists():
            return cand
    return None                 # 워크스페이스 밖 설치본 — 홈 폴백


def _yaml_default(params_file, *keys):
    """params.yaml 의 현재 값을 launch 인자 기본값으로 읽어 온다.

    launch 파일에 숫자를 다시 적으면 params.yaml("모든 설정값의 단일 소스")과
    두 곳이 되어 언젠가 어긋난다. 기본값은 항상 yaml 에서 가져오고, launch 인자는
    **사용자가 준 경우에만** 덮어쓰는 용도로만 쓴다.
    """
    try:
        with open(params_file) as f:
            node = yaml.safe_load(f)['/**']['ros__parameters']
        for k in keys:
            node = node[k]
        return str(node)
    except (OSError, KeyError, TypeError, yaml.YAMLError):
        return ''


def _slug(text, limit=40):
    """메모 → 디렉터리 이름에 쓸 수 있는 조각. 한글은 그대로 두고 구분자만 정리."""
    out = []
    for ch in text.strip():
        if ch.isalnum() or ch in '-_':
            out.append(ch)
        elif ch in ' \t/\\:':
            out.append('_')
    return ''.join(out).strip('_')[:limit]


def _safety_node(context):
    """stack_estop 기동 — 공용 조각(stack_avoid.launch_parts.safety_node).

    estop 거리 3중 안전장치(자동 파생·기동 전 중단·사망 시 세션 중단)는 공용 모듈에
    있다. avoid_estop_joint 도 같은 것을 쓴다 — 예전에는 이 launch 에만 있어서
    joint launch 로는 여전히 "안전 노드 없는 세션"이 만들어질 수 있었다.
    """
    return safety_node(context)


def _can_actions(mode):
    """CAN 브리지 + 종료 시 목표값 0 복귀 — 공용 조각(stack_avoid.launch_parts).

    액션 구성과 그 근거(브리지 종료 후 0 송신, 폴백 가드, ros2 run 금지)는 공용
    모듈에 있다. 여기서는 이 mode 에서만 돌도록 조건만 붙인다.
    """
    return can_bridge_with_zero_guard(condition=LaunchConfigurationEquals('mode', mode))


def _logging_actions(context):
    """세션 로깅 3종 — rosbag + 파라미터 스냅샷 + raw CAN. 이름은 전부 동일 스템.

    bag_dir 를 비우면 field_logs/<mode>_<YYYYmmdd_HHMMSS>[_<메모>] 로 자동 명명 —
    "언제 무슨 시험이었나"가 파일명에서 바로 읽힌다. bag_dir 지정 시 그대로 쓴다.

    ★ 시험 조건 메모는 `note:=` 로 **출발 전에** 준다 (2026-08-09).
      원래는 `mark` 노드로 주행 중 타이핑하는 설계였는데, 회피 시험은 20초 만에
      끝나고 그동안 운전자는 물리 비상정지에 손을 올리고 차를 봐야 한다 —
      키보드를 칠 손이 없다. 시험 1회 = launch 1회 = bag 1개인 지금 구조에서는
      라벨이 실행 인자로 들어가는 게 맞다. (`mark` 는 한 세션에 여러 항목을
      몰아 돌릴 때만 쓴다.)
    """
    mode = LaunchConfiguration('mode').perform(context)
    user_dir = LaunchConfiguration('bag_dir').perform(context)
    note = LaunchConfiguration('note').perform(context).strip()
    src_pkg = _src_pkg_dir()
    if user_dir:
        bag = Path(user_dir).expanduser()
    else:
        root = (src_pkg / 'field_logs') if src_pkg else Path.home() / 'avoid_logs'
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        slug = _slug(note)
        bag = root / (f'{mode}_{stamp}_{slug}' if slug else f'{mode}_{stamp}')
    bag.parent.mkdir(parents=True, exist_ok=True)

    acts = [ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', str(bag)] + BAG_TOPICS,
                           output='screen')]

    # 메모 전문은 별도 파일로 — 디렉터리 이름은 40자에서 잘리고 특수문자도 빠진다.
    if note:
        acts.append(ExecuteProcess(
            cmd=['bash', '-c', f'printf %s\\\\n "$0" > "{bag}_note.txt"', note],
            name='note', output='log'))

    # 노드 파라미터 스냅샷 1회 — bag 에는 "어떤 설정으로 돌렸나"가 안 남는다.
    # 기동 대기 후 덤프하므로 라이브 튜닝(ros2 param set)의 최초값 기준이다.
    nodes = ['/stack_avoid_node', '/stack_estop_node']
    nodes += {'step': ['/step_injector'], 'avoid': ['/avoid_to_ref']}.get(mode, [])
    dump = '; '.join(f"echo '# ---- {n}'; ros2 param dump {n}" for n in nodes)
    acts.append(ExecuteProcess(
        cmd=['bash', '-c', f'sleep 8; {{ {dump}; }} > "{bag}_params.yaml" 2>&1'],
        name='params_snapshot', output='log'))

    # raw CAN 로그 (candump 포맷) — 버스에 실제로 나간 프레임·드롭은 bag 에 없다.
    # tools/ 는 미설치 패키지라 소스 트리에서 직접 실행한다. 실차 모드에서만.
    can_log = src_pkg / 'tools' / 'can_log.py' if src_pkg else None
    if mode in DRIVES and can_log and can_log.exists():
        acts.append(ExecuteProcess(
            cmd=['python3', str(can_log), '--iface', 'can0', '--out', f'{bag}_can.log'],
            name='can_log', output='log'))
    return acts


def generate_launch_description():
    pkg = get_package_share_directory('stack_avoid')
    params = os.path.join(pkg, 'config', 'params.yaml')
    rviz_cfg = os.path.join(pkg, 'config', 'avoid_test.rviz')

    # estop_on/off·dynamic 은 _safety_node 안에서 직접 읽는다(검증·자동 파생 때문).
    v_ref = LaunchConfiguration('v_ref')
    offset_max = LaunchConfiguration('offset_max')
    detect_range = LaunchConfiguration('detect_range')
    lateral_margin = LaunchConfiguration('lateral_margin')
    offsets = LaunchConfiguration('offsets')
    hold_s = LaunchConfiguration('hold_s')
    repeats = LaunchConfiguration('repeats')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='perception',
            description='perception(③ 감지) | step(①② 스텝) | avoid(ⓐⓑⓒ 경계)'),
        DeclareLaunchArgument('v_ref', default_value='0.3', description='주행/시험 속도 [m/s]'),
        DeclareLaunchArgument(
            'bag_dir', default_value='',
            description='비우면 자동: <src/stack_avoid>/field_logs/<mode>_<YYYYmmdd_HHMMSS>'),
        DeclareLaunchArgument(
            'note', default_value='',
            description='시험 조건 메모(선택). 파일명 뒤에 붙고 _note.txt 로도 남는다. '
                        '예: note:="콘 정면 3m 2회차"'),
        DeclareLaunchArgument(
            'dynamic', default_value='true',
            description='stack_estop 동적 물체 기준 (ⓐ 오탐 확인 시 false로 재시험)'),
        DeclareLaunchArgument(
            'estop_on_distance_m', default_value='0.70',
            description='estop 정지 거리 [m] — 바꾸면 측방여유 부등식 재확인 (찬미와 공유). '
                        'estop_off 를 따로 안 주면 자동으로 on+0.10 이 된다'),
        DeclareLaunchArgument(
            'estop_off_distance_m', default_value='',
            description='estop 해제 거리 [m] (히스테리시스 상단). '
                        '비우면 estop_on + 0.10 으로 자동 파생 — on 만 올려도 항상 on < off'),
        DeclareLaunchArgument('offsets', default_value='[0.46, -0.46, 0.30, -0.30]',
                              description='mode:=step 측방 스텝 [m]'),
        DeclareLaunchArgument('hold_s', default_value='3.0',
                              description='mode:=step 스텝 유지 [s] (② 지상은 6s 권장)'),
        DeclareLaunchArgument('repeats', default_value='3',
                              description='mode:=step 반복 횟수'),

        # ── 현장 튜닝 노브 — 기본값은 params.yaml 에서 읽는다(단일 소스 유지) ──
        # 안 주면 yaml 값 그대로. 별도 터미널에서 `ros2 param set` 할 필요 없이
        # 실행 명령 한 줄에 담기게 하려고 뺐다 (2026-08-09 이기돈 요구).
        DeclareLaunchArgument(
            'offset_max', default_value=_yaml_default(params, 'avoid', 'offset_max_m'),
            description='측방 회피 오프셋 상한 [m]. 열림 중심이 이보다 멀면 목표점을 '
                        '포기하고 정지(narrow_gap) — N절 2번 장애물 정지의 원인'),
        DeclareLaunchArgument(
            'detect_range', default_value=_yaml_default(params, 'avoid', 'detect_range_m'),
            description='장애물 감지 거리 [m]'),
        # ★ 통과 이격을 벌리는 실제 레버. 목표점 = 장애물 끝 + (차폭/2 + 이 값).
        #   0.15 → 0.25 로 키우면 편측 여유가 0.46 → 0.56m 가 된다.
        #   (감지 통로 반폭도 같은 식이라 함께 넓어진다 — 의도된 동작.)
        DeclareLaunchArgument(
            'lateral_margin', default_value=_yaml_default(params, 'avoid', 'lateral_margin_m'),
            description='편측 안전 여유 [m] — 장애물을 얼마나 벌리고 지나갈지. '
                        '실측 이격이 부족하면 이 값을 키운다'),
        # ※ `clear_before`·`ray_pull` 인자는 2026-08-11 에 제거했다.
        #   clear_before 는 역산 경로(_required_curvature)에서만 읽혔고 ray_pull 은 그
        #   경로로 가는 스위치였는데, 역산·직송·gps_style 이 전부 기각·삭제되면서
        #   둘 다 "줘도 아무 일이 안 일어나는 인자"가 됐다 (MEASUREMENTS V절).
        #   O 절에 그 무효 실험 1건이 기록돼 있다 — 같은 사고를 인자 삭제로 원천 차단한다.

        # ★ 아래 노드들의 수치 파라미터는 전부 `ParameterValue(..., value_type=...)` 로
        #   감싼다 (팀장 리뷰 2026-08-10 ④). launch 는 인자 문자열의 타입을 추론하므로
        #   `v_ref:=1` 은 INTEGER 가 되는데 노드는 DOUBLE 로 선언해 두어
        #   `InvalidParameterTypeException` 으로 **그 노드만 조용히 죽는다.**
        #   라이다·RViz·bag 은 정상이라 화면상 멀쩡한데 차만 안 움직여, 현장에서
        #   원인 찾다 세션을 통째로 날리기 쉽다. 실측으로 재현 확인:
        #     ros2 run stack_avoid avoid_to_ref --ros-args -p target_speed_mps:=1
        #     → Trying to set parameter 'target_speed_mps' to '1' of type 'INTEGER',
        #       expecting type 'DOUBLE' / Process exited with failure 1
        #   value_type 을 명시하면 "1" 도 1.0 으로 변환돼 들어간다.
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
        ydlidar_driver(),
        # params.yaml 이 기본, 뒤의 dict 가 launch 인자로 덮어쓴다(뒤가 우선).
        # 인자를 안 주면 yaml 값이 그대로 들어가므로 거동은 변하지 않는다.
        Node(package='stack_avoid', executable='stack_avoid_node', name='stack_avoid_node',
             output='screen',
             parameters=[params, {
                 'avoid.offset_max_m': ParameterValue(offset_max, value_type=float),
                 'avoid.detect_range_m': ParameterValue(detect_range, value_type=float),
                 'avoid.lateral_margin_m': ParameterValue(lateral_margin, value_type=float),
             }]),
        # stack_estop 은 estop 거리 검증·자동 파생·사망 시 세션 중단이 붙어 있어
        # OpaqueFunction 으로 뺐다 (_safety_node 주석 참조).
        OpaqueFunction(function=_safety_node),
        Node(package='stack_avoid', executable='avoid_viz', name='avoid_viz', output='screen',
             parameters=[{'lidar_x_m': 0.76, 'vehicle_width_m': 0.62, 'lateral_margin_m': 0.15,
                          'detect_range_m': 3.0, 'offset_max_m': 1.0, 'roi_angle_deg': 180.0}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg], output='log'),

        # ── mode:=step — ①② 측방 스텝 계단 ──
        Node(package='stack_avoid', executable='step_injector', name='step_injector',
             output='screen',
             parameters=[{
                 'v_ref': ParameterValue(v_ref, value_type=float),
                 'offsets': ParameterValue(offsets, value_type=List[float]),
                 'hold_s': ParameterValue(hold_s, value_type=float),
                 'repeats': ParameterValue(repeats, value_type=int),
                 'estop_gate': True}],
             condition=LaunchConfigurationEquals('mode', 'step')),

        # ── mode:=avoid — ⓐⓑⓒ 경계 시험 ──
        # straight_when_clear=true: ⓐ가 "접근하다 비켜 간다"를 보는 시험이라
        # clear 구간에 차가 움직여야 성립한다. 통제된 공간에서만.
        Node(package='stack_avoid', executable='avoid_to_ref', name='avoid_to_ref',
             output='screen',
             parameters=[{'target_speed_mps': ParameterValue(v_ref, value_type=float),
                          'straight_when_clear': True,
                          'estop_gate': True, 'estop_stale_s': 0.25,
                          }],
             condition=LaunchConfigurationEquals('mode', 'avoid')),

        # ── CAN 브리지 + 종료 시 목표값 0 복귀 — 실차를 움직이는 모드에서만 ──
        # 두 가지가 짝이라 _can_actions() 한 곳에서 만든다 (순서 보장 때문).
        *[a for d in DRIVES for a in _can_actions(d)],

        # ── 로깅 3종 (bag + params + raw CAN) — 이름·경로는 _logging_actions 가 정한다 ──
        OpaqueFunction(function=_logging_actions),
    ])
