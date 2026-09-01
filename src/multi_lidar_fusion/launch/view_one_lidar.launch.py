"""라이다를 **한 대만** 띄우고 RViz 로 보여준다 — 어느 유닛이 어디에 달렸는지 눈으로 확인용.

이 파일의 역할:
    시리얼 포트만으로는 어느 물리 유닛이 앞/뒤/좌/우인지 알 수 없다
    (네 YDLiDAR가 같은 USB 계열이라 장치명만으로는 위치를 구분하기 어렵다).
    한 대씩 순서대로 띄워 놓고 사람이 눈으로 확인해서 위치를 정한다.

    어느 유닛을 띄우든 토픽·frame 이름은 항상 같다(`/probe/scan`, `probe_frame`)
    → RViz 설정 하나로 4대를 돌아가며 볼 수 있다.

출력 topic : /probe/scan  (LaserScan)
출력 frame : probe_frame  (map -> probe_frame 항등 static TF 를 같이 띄운다.
             안 그러면 RViz 가 "Fixed Frame does not exist" 로 아무것도 안 그린다)

실행:
    # 새 좌/우 YDLiDAR를 각각 확인
    ros2 launch multi_lidar_fusion view_one_lidar.launch.py unit:=b1
    ros2 launch multi_lidar_fusion view_one_lidar.launch.py unit:=b2

    # 아직 udev 위치 링크를 만들기 전인 장치 확인
    ros2 launch multi_lidar_fusion view_one_lidar.launch.py \
        unit:=custom port:=/dev/serial/by-path/...

    ros2 launch multi_lidar_fusion view_one_lidar.launch.py unit:=yd0
    ros2 launch multi_lidar_fusion view_one_lidar.launch.py unit:=yd1

    RViz 없이 값만 보고 싶으면:  rviz:=false

    포트가 바뀌었으면 (허브 자리를 옮겼거나 재부팅 후):
        ls -l /dev/serial/by-path/ /dev/serial/by-id/
        ros2 launch multi_lidar_fusion view_one_lidar.launch.py unit:=yd0 \
            port_yd0:=/dev/serial/by-path/...

RViz 화면에서 이 유닛이 어디에 달렸는지 판별하는 법:
    화면 가운데(원점)가 그 라이다다. 손을 라이다 앞에 대면 그 방향의 점이 원점 쪽으로
    확 붙는다 — 손을 차량 앞쪽/뒤쪽에서 넣어보면 어느 유닛인지 바로 갈린다.
    빨간 축(X)이 그 라이다의 0도 방향이다.

★ 장착 yaw 측정
    화면에 **0~360도 눈금**이 함께 뜬다 (0도=빨강 화살표, 90도=초록 화살표).
    눈금은 이 라이다의 **스캔 원(raw) 각도**다 — 차량 좌표가 아니다.

    차량 기준 방향을 아는 물체(예: 차량 정면에 사람이 서기)를 이 라이다로 보고,
    그 물체가 몇 도에 찍히는지 눈금에서 읽는다. 그러면:

        yaw_deg = (그 물체의 차량 기준 방위) − (읽은 센서 각도)

    예) 차량 정면(0도)에 선 사람이 눈금 90도에 찍혔다  ->  yaw = 0 − 90 = −90도

    읽은 값은 `stack_parking/config/lidar_mounts.yaml` 의 해당 항목에
    `yaw_deg:` 로 적는다. 그러면 multi_lidar_fusion launch 가 TF yaw 와 FOV 변환에
    함께 쓰고 "장착 yaw 미실측" 경고가 사라진다.

    눈금 간격/크기 조정:
        ros2 launch multi_lidar_fusion view_one_lidar.launch.py unit:=b1 \
            label_step_deg:=15 label_r:=3.0
    눈금이 필요 없으면 labels:=false
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 각도 눈금 노드는 stack_avoid 것을 재사용한다 (0~360도 방사선 + 숫자 + 0/90도 축).
# 없는 환경(그 패키지를 안 받은 사람)에서도 드라이버 확인은 되어야 하므로,
# 하드 의존으로 걸지 않고 있으면 쓰고 없으면 안내만 남긴다.
try:
    get_package_share_directory('stack_avoid')
    HAVE_ANGLE_LABELS = True
except Exception:      # PackageNotFoundError
    HAVE_ANGLE_LABELS = False

# 실기에서 확인한 네 USB 위치를 고정한다. 미배정 장치는 unit:=custom으로 확인한다.
PORTS = {
    'yd0': '/dev/lidar_front',
    'yd1': '/dev/lidar_rear',
    'b1': '/dev/lidar_left',
    'b2': '/dev/lidar_right',
}

FRAME = 'probe_frame'
TOPIC = '/probe/scan'


def generate_launch_description():
    pkg = get_package_share_directory('multi_lidar_fusion')

    args = [
        DeclareLaunchArgument(
            'unit', default_value='yd0',
            choices=['custom', 'yd0', 'yd1', 'b1', 'b2'],
            description=('띄울 YDLiDAR 위치 (yd0/yd1=기존 앞·뒤, b1=좌, b2=우, '
                         'custom=port 인자로 지정)')),
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyUSB0',
            description=(
                'unit:=custom 전용 포트. /dev/serial/by-path/의 신규 장치 지정')),
        DeclareLaunchArgument('rviz', default_value='true'),
        # ── 각도 눈금 (장착 yaw 측정용) ──
        DeclareLaunchArgument(
            'labels', default_value='true',
            description='0~360도 각도 눈금 표시 (장착 yaw 를 눈으로 읽을 때 필요)'),
        DeclareLaunchArgument(
            'label_step_deg', default_value='10',
            description='눈금 간격 [deg] (좌/우 라이다 장착 방향 판독 기본값: 10)'),
        DeclareLaunchArgument(
            'label_r', default_value='2.0',
            description='눈금 반지름 [m]. 숫자가 화면 밖이면 줄일 것'),
    ]

    actions = []

    # 기존 위치 심링크에 아직 배정하지 않은 신규 YDLiDAR 확인용.
    actions.append(Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='probe_lidar',
        output='screen',
        emulate_tty=True,
        condition=LaunchConfigurationEquals('unit', 'custom'),
        parameters=[{
            'port': LaunchConfiguration('port'),
            'frame_id': FRAME,
            'baudrate': 230400,
            'lidar_type': 1,
            'device_type': 0,
            'sample_rate': 4,
            'abnormal_check_count': 4,
            'fixed_resolution': False,
            'reversion': True,
            'inverted': False,
            'auto_reconnect': True,
            'isSingleChannel': False,
            'intensity': True,
            'intensity_bit': 8,
            'support_motor_dtr': False,
            'frequency': 10.0,
            'angle_max': 180.0,
            'angle_min': -180.0,
            'range_max': 12.0,
            'range_min': 0.03,
            'invalid_range_is_inf': False,
            'ignore_array': '',
            'debug': False,
        }],
        remappings=[('/scan', TOPIC)],
    ))
    actions.append(LogInfo(
        condition=LaunchConfigurationEquals('unit', 'custom'),
        msg=['[view_one_lidar] 신규 YDLiDAR  port=', LaunchConfiguration('port'),
             '  topic=', TOPIC, '  angle grid=10deg (기본값)']))

    # unit 별로 드라이버 노드를 만들되, LaunchConfigurationEquals 조건으로 하나만 산다.
    for key, default_port in PORTS.items():
        # 유닛별 포트는 각각 port_<unit> 인자로 덮어쓸 수 있다.
        args.append(
            DeclareLaunchArgument('port_' + key, default_value=default_port,
                                  description=key + ' 표준 포트'))
        chosen = LaunchConfiguration('port_' + key)
        stable_stream = key in ('yd1', 'b1', 'b2')
        drv = Node(
                package='ydlidar_ros2_driver',
                executable='ydlidar_ros2_driver_node',
                name='probe_lidar',
                output='screen',
                emulate_tty=True,
                condition=LaunchConfigurationEquals('unit', key),
                parameters=[{
                    'port': chosen,
                    'frame_id': FRAME,
                    'baudrate': 230400,
                    'lidar_type': 1,
                    'device_type': 0,
                    'sample_rate': 4 if stable_stream else 9,
                    'abnormal_check_count': 4,
                    'fixed_resolution': False if stable_stream else True,
                    # ★ multi_lidar_drivers.launch.py 와 **반드시 같아야** 한다.
                    #   reversion/inverted 는 각도 규약 그 자체라, 다르면 이 화면에서
                    #   읽은 각도가 융합 파이프라인이 보는 각도와 거울상이 된다 —
                    #   측정 화면이 거짓말을 하게 된다.
                    'reversion': True,
                    'inverted': False,
                    'auto_reconnect': True,
                    'isSingleChannel': False,
                    'intensity': True,
                    'intensity_bit': 8 if stable_stream else 16,
                    'support_motor_dtr': False,
                    'frequency': 10.0,
                    'angle_max': 180.0,
                    'angle_min': -180.0,
                    'range_max': 12.0,
                    'range_min': 0.03,
                    'invalid_range_is_inf': False if stable_stream else True,
                    'ignore_array': '',
                    'debug': False,
                }],
                remappings=[('/scan', TOPIC)],
            )
        actions.append(drv)
        actions.append(
            LogInfo(condition=LaunchConfigurationEquals('unit', key),
                    msg=['[view_one_lidar] unit=', key, '  port=', chosen,
                         '  topic=', TOPIC]))

    # RViz 의 Fixed Frame 이 TF 트리에 존재해야 그림이 나온다.
    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='probe_frame_tf',
        output='log',
        arguments=['0', '0', '0', '0', '0', '0', 'map', FRAME],
    ))

    # 0~360도 각도 눈금 — 이 라이다의 스캔 raw 각도를 화면에서 바로 읽기 위한 것.
    # 장착 yaw 는 이 눈금으로만 확정된다 (아래 안내 참조).
    if HAVE_ANGLE_LABELS:
        actions.append(Node(
            package='stack_avoid',
            executable='angle_labels',
            name='angle_labels',
            output='log',
            condition=IfCondition(LaunchConfiguration('labels')),
            parameters=[{
                'frame_id': FRAME,
                'radius_m': ParameterValue(
                    LaunchConfiguration('label_r'), value_type=float),
                'step_deg': ParameterValue(
                    LaunchConfiguration('label_step_deg'), value_type=int),
            }],
        ))
        actions.append(LogInfo(
            condition=IfCondition(LaunchConfiguration('labels')),
            msg=('[view_one_lidar] 각도 눈금 = 이 라이다의 스캔 raw 각도 '
                 '(0도=빨강 화살표, 90도=초록).\n'
                 '                 장착 yaw 측정: 차량 기준 방향을 아는 물체가 눈금 몇 도에'
                 ' 찍히는지 읽고\n'
                 '                     yaw_deg = 차량 기준 방위 - 센서 각도\n'
                 '                 결과는 stack_parking/config/lidar_mounts.yaml의'
                 ' 해당 항목에 yaw_deg 로 기록.')))
    else:
        actions.append(LogInfo(
            msg=('[view_one_lidar] ! stack_avoid 패키지가 없어 각도 눈금을 띄우지 못한다 '
                 '— 장착 yaw 를 읽으려면 그 패키지를 빌드할 것')))

    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg, 'rviz', 'one_lidar.rviz')],
    ))

    return LaunchDescription(args + actions)
