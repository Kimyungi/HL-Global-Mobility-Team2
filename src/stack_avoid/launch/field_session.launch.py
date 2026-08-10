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
  ros2 launch stack_avoid field_session.launch.py mode:=avoid v_ref:=0.4 clear_before:=1.2
  # 회차 메모 — 폴더 이름에 붙는다
  ros2 launch stack_avoid field_session.launch.py mode:=avoid v_ref:=0.4 \
      note:="콘 2개 3m 간격"

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

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue

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
    ydlidar_params = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'), 'params', 'ydlidar.yaml')

    v_ref = LaunchConfiguration('v_ref')
    dynamic = LaunchConfiguration('dynamic')
    offset_max = LaunchConfiguration('offset_max')
    detect_range = LaunchConfiguration('detect_range')
    lateral_margin = LaunchConfiguration('lateral_margin')
    clear_before = LaunchConfiguration('clear_before')
    ray_pull = LaunchConfiguration('ray_pull')
    estop_on = LaunchConfiguration('estop_on_distance_m')
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
            description='estop 정지 거리 [m] — 바꾸면 측방여유 부등식 재확인 (찬미와 공유)'),
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
        # ⚠ ray_pull(기본 true) 경로에서는 **읽히지 않는다** — 2026-08-09 실측 확인.
        #   clear_before 는 _required_curvature() 안에서만 쓰이고 그 함수는
        #   _scale_matched_point() 에서만 불린다(= ray_pull:=false 일 때만).
        #   ray_pull 을 끄지 않고 이 값만 바꾸면 아무 일도 일어나지 않는다.
        DeclareLaunchArgument(
            'clear_before', default_value='0.85',
            description='세로 여유 [m]. ⚠ ray_pull:=false 일 때만 유효 '
                        '(기본 ray_pull=true 에서는 무시됨)'),
        DeclareLaunchArgument(
            'ray_pull', default_value='true',
            description='true=초록점 방향 보존·거리만 당김(기본). '
                        'false=당김+역산 경로(clear_before 가 살아난다)'),

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
        # params.yaml 이 기본, 뒤의 dict 가 launch 인자로 덮어쓴다(뒤가 우선).
        # 인자를 안 주면 yaml 값이 그대로 들어가므로 거동은 변하지 않는다.
        Node(package='stack_avoid', executable='stack_avoid_node', name='stack_avoid_node',
             output='screen',
             parameters=[params, {
                 'avoid.offset_max_m': ParameterValue(offset_max, value_type=float),
                 'avoid.detect_range_m': ParameterValue(detect_range, value_type=float),
                 'avoid.lateral_margin_m': ParameterValue(lateral_margin, value_type=float),
             }]),
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
                          'estop_gate': True, 'estop_stale_s': 0.25,
                          'clear_before_m': ParameterValue(clear_before, value_type=float),
                          'ray_pull': ParameterValue(ray_pull, value_type=bool)}],
             condition=LaunchConfigurationEquals('mode', 'avoid')),

        # ── CAN 브리지 — 실차를 움직이는 모드에서만 ──
        *[Node(package='bridge_dspace', executable='can_bridge_node', name='can_bridge_node',
               output='screen', parameters=[{'can_interface': 'can0'}],
               condition=LaunchConfigurationEquals('mode', d)) for d in DRIVES],

        # ── 종료 시 dSPACE 목표값 0 복귀 (안전 가드) ──
        # dSPACE 에 watchdog 이 없다(2026-08-09 실측) — PC 송신이 끊겨도 마지막 v_ref 를
        # 무기한 유지한다. launch 를 끄는 것만으로는 정지 상태가 되지 않으므로,
        # 이 가드가 SIGINT 를 받아 SocketCAN 에 직접 0 을 쓴다(브리지가 이미 죽어도 동작).
        # ★ `ros2 run` 으로 감싸면 안 된다 — 래퍼가 SIGINT 를 삼켜 가드가 안 돈다(실측).
        #   Node 액션은 실행 파일을 직접 띄우므로 신호가 그대로 전달된다.
        *[Node(package='stack_avoid', executable='can_zero', name='can_zero', output='screen',
               condition=LaunchConfigurationEquals('mode', d)) for d in DRIVES],

        # ── 로깅 3종 (bag + params + raw CAN) — 이름·경로는 _logging_actions 가 정한다 ──
        OpaqueFunction(function=_logging_actions),
    ])
