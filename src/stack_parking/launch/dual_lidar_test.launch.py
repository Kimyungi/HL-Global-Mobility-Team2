"""라이다 2대 동시 구동 — 상호 간섭 시험용.

두 유닛을 각각 별도 네임스페이스·frame_id 로 띄워 한 RViz 에서 같이 본다.
개체 구분은 `/dev/serial/by-path/` 안정 이름으로 한다 (두 유닛 다 시리얼이 `0001`이라
시리얼로는 구분 불가 — CLAUDE.md 스택 문서 및 memo 참조).

  ros2 launch stack_parking dual_lidar_test.launch.py
  ros2 launch stack_parking dual_lidar_test.launch.py dy_m:=0.15 yaw_b_deg:=0

인자 (A 를 원점으로 본 B 의 상대 위치):
  port_a / port_b   각 유닛의 시리얼 포트 (기본: by-path 안정 이름)
  dx_m              전후 방향 거리 [m] (+ = A 의 정면 쪽)
  dy_m              좌우 방향 거리 [m] (+ = A 의 좌측)
  yaw_b_deg         B 유닛의 방향 [deg]. 0 = 같은 방향, 180 = 마주봄

주의: 이 launch 는 **시험용**이다. 실차 4대 구성은 장착 위치 확정 후 별도 launch 로 만든다.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BY_PATH = '/dev/serial/by-path'
DEFAULT_A = f'{BY_PATH}/pci-0000:00:14.0-usb-0:1.1:1.0-port0'
DEFAULT_B = f'{BY_PATH}/pci-0000:00:14.0-usb-0:1.2:1.0-port0'


def _driver(ns, port, frame, params_file, auto_reconnect):
    """드라이버 1인스턴스. params 파일을 읽되 port/frame_id 등은 인자로 덮어쓴다."""
    return Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        namespace=ns,
        name='ydlidar_driver',
        output='screen',
        emulate_tty=True,
        parameters=[params_file, {'port': port, 'frame_id': frame,
                                  'auto_reconnect': auto_reconnect}],
    )


def _setup(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    dx = float(LaunchConfiguration('dx_m').perform(context))
    dy = float(LaunchConfiguration('dy_m').perform(context))
    yaw_b = float(LaunchConfiguration('yaw_b_deg').perform(context))
    yaw_b_rad = yaw_b * 3.14159265358979 / 180.0

    # auto_reconnect 는 기본 off. 이 드라이버는 USB 재삽입 후 자동 복구하면
    # 설정 주기(10Hz)를 회복하지 못하고 ~6Hz 로 계속 돈다 (2026-07-31 실측).
    # 재접속은 드라이버를 새로 띄워서 하는 것이 안전하다.
    reconnect = LaunchConfiguration('auto_reconnect').perform(context).lower() in (
        'true', '1', 'yes')

    return [
        _driver('lidar_a', LaunchConfiguration('port_a').perform(context),
                'lidar_a', params_file, reconnect),
        _driver('lidar_b', LaunchConfiguration('port_b').perform(context),
                'lidar_b', params_file, reconnect),
        # 벤치 기준계 — A 를 원점에, B 를 (dx, dy) 만큼 떨어뜨려 배치.
        # 간섭 시험에서는 정확한 외부 파라미터가 필요 없다. 두 스캔을 한 화면에서
        # 구분해 보기 위한 배치일 뿐이며, 실제 외부 캘리브는 장착 후 별도로 한다.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_bench_a', output='log',
             arguments=['0', '0', '0', '0', '0', '0', 'bench', 'lidar_a']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_bench_b', output='log',
             arguments=[str(dx), str(dy), '0', str(yaw_b_rad), '0', '0',
                        'bench', 'lidar_b']),
    ]


def _default_params():
    """드라이버 params 파일의 기본 경로.

    설치된 `ydlidar_ros2_driver` 의 share 에서 찾는다 — 워크스페이스에 벤더링된
    드라이버든 `~/ydlidar_ros2_ws` 오버레이든, 소스한 쪽이 잡힌다(양쪽 다
    `params/` 를 share 에 설치한다). 어느 쪽도 소스하지 않은 상태에서 launch
    파싱이 죽지 않도록 예전 고정 경로로 폴백한다.
    """
    try:
        return os.path.join(get_package_share_directory('ydlidar_ros2_driver'),
                            'params', 'Tmini.yaml')
    except PackageNotFoundError:
        return os.path.join(os.path.expanduser('~'), 'ydlidar_ros2_ws', 'src',
                            'ydlidar_ros2_driver', 'params', 'Tmini.yaml')


def generate_launch_description():
    default_params = _default_params()

    return LaunchDescription([
        DeclareLaunchArgument('port_a', default_value=DEFAULT_A),
        DeclareLaunchArgument('port_b', default_value=DEFAULT_B),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('dx_m', default_value='0.0',
                              description='B 의 전후 오프셋 [m] (+ = A 정면 쪽)'),
        DeclareLaunchArgument('dy_m', default_value='0.15',
                              description='B 의 좌우 오프셋 [m] (+ = A 좌측)'),
        DeclareLaunchArgument('yaw_b_deg', default_value='0.0',
                              description='B 유닛 방향 [deg]. 0 = 같은 방향'),
        DeclareLaunchArgument('auto_reconnect', default_value='false',
                              description='USB 자동 재접속. 켜면 재접속 후 ~6Hz 로 떨어짐'),
        OpaqueFunction(function=_setup),
    ])
