"""라이다를 **한 대만** 띄우고 RViz 로 보여준다 — 어느 유닛이 어디에 달렸는지 눈으로 확인용.

이 파일의 역할:
    시리얼 포트만으로는 어느 물리 유닛이 앞/뒤/좌/우인지 알 수 없다
    (YDLiDAR 2대는 시리얼이 둘 다 `0001` 이라 by-id 로도 구분되지 않는다).
    한 대씩 순서대로 띄워 놓고 사람이 눈으로 확인해서 위치를 정한다.

    어느 유닛을 띄우든 토픽·frame 이름은 항상 같다(`/probe/scan`, `probe_frame`)
    → RViz 설정 하나로 4대를 돌아가며 볼 수 있다.

출력 topic : /probe/scan  (LaserScan)
출력 frame : probe_frame  (map -> probe_frame 항등 static TF 를 같이 띄운다.
             안 그러면 RViz 가 "Fixed Frame does not exist" 로 아무것도 안 그린다)

실행:
    ros2 launch multi_lidar_fusion view_one_lidar.launch.py unit:=yd0
    ros2 launch multi_lidar_fusion view_one_lidar.launch.py unit:=yd1
    ros2 launch multi_lidar_fusion view_one_lidar.launch.py unit:=rp0
    ros2 launch multi_lidar_fusion view_one_lidar.launch.py unit:=rp1

    RViz 없이 값만 보고 싶으면:  rviz:=false

    포트가 바뀌었으면 (허브 자리를 옮겼거나 재부팅 후):
        ls -l /dev/serial/by-path/ /dev/serial/by-id/
        ros2 launch multi_lidar_fusion view_one_lidar.launch.py unit:=yd0 \
            port_yd0:=/dev/serial/by-path/...

RViz 화면에서 이 유닛이 어디에 달렸는지 판별하는 법:
    화면 가운데(원점)가 그 라이다다. 손을 라이다 앞에 대면 그 방향의 점이 원점 쪽으로
    확 붙는다 — 손을 차량 앞쪽/뒤쪽에서 넣어보면 어느 유닛인지 바로 갈린다.
    빨간 축(X)이 그 라이다의 0도 방향이다.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

BY_PATH = '/dev/serial/by-path/'
BY_ID = '/dev/serial/by-id/'

# 2026-08-13 실측. YDLiDAR 2대는 시리얼이 겹쳐 by-id 를 못 쓰므로 by-path.
PORTS = {
    'yd0': BY_PATH + 'pci-0000:00:14.0-usb-0:1.2.4:1.0-port0',
    'yd1': BY_PATH + 'pci-0000:00:14.0-usb-0:1.2.3:1.0-port0',
    'rp0': BY_ID + ('usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                    '76d341fd291ef1118e6dbee40f0f12f8-if00-port0'),
    'rp1': BY_ID + ('usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                    'f2ee467bfb1df111a7b6c4e40f0f12f8-if00-port0'),
}

FRAME = 'probe_frame'
TOPIC = '/probe/scan'


def generate_launch_description():
    pkg = get_package_share_directory('multi_lidar_fusion')

    args = [
        DeclareLaunchArgument(
            'unit', default_value='yd0',
            choices=['yd0', 'yd1', 'rp0', 'rp1'],
            description='띄울 라이다 (yd0/yd1 = YDLiDAR T-mini Plus, rp0/rp1 = RPLiDAR C1M1)'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    actions = []

    # unit 별로 드라이버 노드를 만들되, LaunchConfigurationEquals 조건으로 하나만 산다.
    for key, default_port in PORTS.items():
        # 유닛별 포트는 각각 port_<unit> 인자로 덮어쓸 수 있다.
        args.append(
            DeclareLaunchArgument('port_' + key, default_value=default_port,
                                  description=key + ' 표준 포트'))
        chosen = LaunchConfiguration('port_' + key)

        if key.startswith('yd'):
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
                    'lidar_type': 0,
                    'device_type': 0,
                    'sample_rate': 4,
                    'abnormal_check_count': 4,
                    'fixed_resolution': True,
                    'reversion': True,
                    'inverted': True,
                    'auto_reconnect': True,
                    'isSingleChannel': False,
                    'intensity': True,
                    'intensity_bit': 16,
                    'support_motor_dtr': False,
                    'frequency': 10.0,
                    'angle_max': 180.0,
                    'angle_min': -180.0,
                    'range_max': 12.0,
                    'range_min': 0.03,
                    'invalid_range_is_inf': True,
                    'ignore_array': '',
                    'debug': False,
                }],
                remappings=[('/scan', TOPIC)],
            )
        else:
            drv = Node(
                package='rplidar_ros',
                executable='rplidar_node',
                name='probe_lidar',
                output='screen',
                emulate_tty=True,
                condition=LaunchConfigurationEquals('unit', key),
                parameters=[{
                    'channel_type': 'serial',
                    'serial_port': chosen,
                    'serial_baudrate': 460800,
                    'frame_id': FRAME,
                    'inverted': False,
                    'angle_compensate': True,
                    'scan_mode': 'Standard',
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

    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg, 'rviz', 'one_lidar.rviz')],
    ))

    return LaunchDescription(args + actions)
