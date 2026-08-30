"""launch 파일들이 공유하는 액션 조각.  이기돈

같은 블록이 launch 4개에 복붙돼 있던 것을 한곳으로 모은다 (팀장 리뷰 2026-08-10 비차단).
복붙의 실제 피해는 코드량이 아니라 **한쪽만 고쳐지는 것**이었다 — can_zero 종료 순서
수정(⑤)은 field_session 에만 들어갔고, 나머지 launch 는 예전 상태로 남아 있었다.

launch 파일에서 `from stack_avoid.launch_parts import ...` 로 쓴다. 설치된 파이썬
패키지라 launch 파싱 시점에 그대로 import 된다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def ydlidar_driver(condition=None):
    """LiDAR 드라이버 노드 → /scan.

    ★ 드라이버 **노드만** 띄운다. `ydlidar_launch.py` 를 include 하면 placeholder
      static TF(laser_frame)가 딸려와 stack_avoid_node 가 발행하는 실측 TF 와 충돌한다
      (같은 프레임을 두 곳이 발행 → TF 가 튄다). PR #23 에서 확정된 구성이다.
    """
    params = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'), 'params', 'ydlidar.yaml')
    return LifecycleNode(
        package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node', namespace='/', output='screen',
        emulate_tty=True, parameters=[params], condition=condition)


def can_bridge_with_zero_guard(condition=None, can_interface='can0', vehicle_csv_path='',
                               can_fd=True):
    """CAN 브리지 + 종료 시 dSPACE 목표값 0 복귀. 실차를 움직이는 구성에서만 쓴다.

    dSPACE 에 watchdog 이 없다(2026-08-09 실측, J-6) — PC 송신이 끊겨도 마지막 v_ref 를
    무기한 유지한다. launch 를 끄는 것만으로는 정지 상태가 되지 않는다.

    ★ 순서 문제 (팀장 리뷰 2026-08-10 ⑤). `ros2 launch` 는 SIGINT 를 **전 프로세스에
      동시** 전달한다. 파이썬 teardown 이 수백 ms 걸리므로, can_zero 의 0 버스트
      (30×10ms=0.3s)가 끝난 **뒤에** 브리지가 마지막 큐를 비우며 nonzero v_ref 를 한
      프레임이라도 더 실으면 dSPACE 가 그것을 latch 한다 — 가드가 막으려던 바로 그 상황.

    그래서 두 겹으로 둔다:
      ⓐ `OnProcessExit(브리지)` → 브리지가 **완전히 죽은 뒤** `can_zero --once`.
         버스에 더 쓸 주체가 없는 시점이라 순서가 보장된다. 브리지가 세션 중간에
         죽는 경우(크래시)에도 동작한다 — 오히려 그때가 더 필요하다.
      ⓑ 상주 가드는 폴백으로 남긴다. 이벤트가 안 걸리는 경우(강제 종료 등) 대비이고,
         같은 0 을 쓰므로 ⓐ 와 겹쳐도 무해하다.

    브리지 프로세스 자체가 예외 종료하면 ⓐ의 0 버스트(약 0.3초)가 끝난 뒤가 되도록
    1초 지연 후 자동 respawn한다. USB 재열거는 노드 내부에서 소켓만 다시 열기 때문에
    보통 respawn까지 가지 않으며, 프로세스 장애까지 사람 개입 없이 복구하는 2차 경로다.

    ★ `ros2 run` 으로 감싸면 안 된다 — 래퍼가 SIGINT 를 삼켜 가드가 안 돈다(실측).
      Node 액션은 실행 파일을 직접 띄우므로 신호가 그대로 전달된다.

    `vehicle_csv_path` 를 주면 dSPACE RX 피드백(/vehicle/vector)을 CSV 로도 남긴다
    (2026-08-25). 토픽은 원래부터 rosbag 에 기록됐지만 RX 가 0건이라 늘 비어 있었고,
    실차 분석은 run 폴더의 CSV 를 먼저 보므로 같은 자리에 둔다. 빈 값 = 끔.

    `can_fd` (2026-08-28 Kvaser Leaf v3 이관) — 팀 표준은 CAN FD 다. False 면 classic
    프레임으로 송신한다(A/B 대조용). 프레임 레이아웃·ID·스케일은 양쪽이 동일하므로
    이 값은 **와이어 포맷만** 고른다. can_zero 가드는 인터페이스 MTU 로 자동 판정하니
    별도 인자가 없다 — 종료 안전 경로에서 인자를 빠뜨리는 사고를 만들지 않기 위해서다.
    """
    bridge = Node(package='bridge_dspace', executable='can_bridge_node',
                  name='can_bridge_node', output='screen',
                  respawn=True, respawn_delay=1.0,
                  parameters=[{'can_interface': can_interface,
                               'vehicle_csv_path': vehicle_csv_path,
                               'can_fd': can_fd}],
                  condition=condition)
    zero_exe = os.path.normpath(os.path.join(
        get_package_share_directory('stack_avoid'), '..', '..',
        'lib', 'stack_avoid', 'can_zero'))
    return [
        bridge,
        # ⓐ 브리지 종료 후 — 이 시점엔 버스에 쓸 주체가 없다
        RegisterEventHandler(
            OnProcessExit(
                target_action=bridge,
                on_exit=[
                    LogInfo(msg='can_bridge 종료 확인 → dSPACE 목표값 0 송신'),
                    ExecuteProcess(cmd=[zero_exe, '--once'],
                                   name='can_zero_after_bridge', output='screen'),
                ]),
            condition=condition),
        # ⓑ 폴백 가드 (SIGINT 동시 수신 — 순서 보장 없음)
        Node(package='stack_avoid', executable='can_zero', name='can_zero',
             output='screen', condition=condition),
    ]


ESTOP_HYSTERESIS_M = 0.10       # estop_off 를 자동 파생할 때의 간격


def safety_node(context):
    """stack_estop(박찬미) 기동 — estop 거리 3중 안전장치.

    ★ 배경 (2026-08-10). `estop_on_distance_m:=0.80` 이상을 주면 stack_estop 이
      `Require 0 < estop_on_distance_m < estop_off_distance_m` 로 **즉사**한다
      (off 기본 0.80). 그런데 launch 는 멈추지 않아 라이다·RViz·bag 은 정상 기동하고
      **안전 노드만 없는 세션**이 된다. avoid_to_ref 의 estop 게이트가 미수신
      페일세이프로 v_ref=0 을 걸어 차는 안 움직이지만, 증상이 "차가 안 감" 하나뿐이라
      원인을 찾는 데 세션을 통째로 날린다.

    그래서 세 겹으로 막는다 — 값 노출만으로는 못 막는다(on 만 올리면 그대로 재현):
      ① off 를 안 주면 on + 0.10 으로 **자동 파생** → on 만 만져도 항상 on < off
      ② 그래도 모순되면(둘 다 명시) launch 를 **기동 전에** 읽히는 메시지로 중단
      ③ 그 밖의 이유로 stack_estop 이 죽으면 `on_exit=Shutdown` 으로 **세션 전체 중단**
         — estop 없이 굴러가는 구성 자체를 허용하지 않는다. ①② 는 이 한 가지 원인만
         막지만 ③ 은 원인과 무관하게 "안전 노드 없는 주행"을 막는다.
    """
    on_s = LaunchConfiguration('estop_on_distance_m').perform(context).strip()
    off_s = LaunchConfiguration('estop_off_distance_m').perform(context).strip()
    try:
        on = float(on_s)
    except ValueError:
        return [LogInfo(msg=f'✗ estop_on_distance_m="{on_s}" 은 숫자가 아닙니다.'),
                Shutdown(reason='estop_on_distance_m 파싱 실패')]
    if off_s:
        try:
            off = float(off_s)
        except ValueError:
            return [LogInfo(msg=f'✗ estop_off_distance_m="{off_s}" 은 숫자가 아닙니다.'),
                    Shutdown(reason='estop_off_distance_m 파싱 실패')]
        derived = False
    else:
        # round: 0.70+0.10 이 0.7999999999999999 로 나와 로그·params 스냅샷에 남는다.
        off = round(on + ESTOP_HYSTERESIS_M, 4)     # ① 자동 파생
        derived = True

    if on <= 0.0:                              # ② 기동 전 중단 (양수 아님)
        return [LogInfo(msg=(
            f'✗ estop_on_distance_m={on:.2f} — 0 보다 커야 합니다.\n'
            f'   이 값은 "장애물이 이 거리 안에 들어오면 정지"라는 뜻이라 '
            f'0 이하면 영원히 정지하지 않습니다.')),
            Shutdown(reason='estop_on_distance_m 이 0 이하')]
    if on >= off:                              # ② 기동 전 중단 (히스테리시스 역전)
        return [LogInfo(msg=(
            f'✗ estop 거리 설정이 모순입니다: on={on:.2f} off={off:.2f}\n'
            f'   estop_on < estop_off 여야 합니다 (on 에서 정지, off 에서 해제 — '
            f'해제 거리가 더 멀어야 채터링이 없습니다).\n'
            f'   해결: estop_off_distance_m 을 더 크게 주거나(예: '
            f'estop_off_distance_m:={on + ESTOP_HYSTERESIS_M:.2f}), '
            f'estop_on_distance_m 을 {off:.2f} 미만으로 낮추세요.\n'
            f'   estop_off 를 아예 주지 않으면 on+{ESTOP_HYSTERESIS_M:.2f} 로 '
            f'자동 계산되어 이 오류가 나지 않습니다.')),
            Shutdown(reason='estop 거리 설정 모순 (on >= off)')]

    note = ' (estop_off 자동 파생)' if derived else ' (estop_off 직접 지정)'
    return [
        LogInfo(msg=f'estop 거리: 정지 {on:.2f}m / 해제 {off:.2f}m{note}'),
        # 박찬미 stack_estop — laser_yaw_in_base_rad=π/2 가 우리 forward_angle_deg=270 과 짝.
        # 라이다를 재장착하면 두 값을 같이 고칠 것.
        Node(package='stack_estop', executable='stack_estop_node',
             name='stack_estop_node', output='screen',
             parameters=[{
                 'estop_on_distance_m': float(on),
                 'estop_off_distance_m': float(off),
                 'dynamic_enabled': ParameterValue(
                     LaunchConfiguration('dynamic'), value_type=bool)}],
             # ③ 안전 노드가 죽으면 세션 전체를 내린다.
             on_exit=Shutdown(reason='stack_estop 종료 — 안전 노드 없이 주행 불가')),
    ]
