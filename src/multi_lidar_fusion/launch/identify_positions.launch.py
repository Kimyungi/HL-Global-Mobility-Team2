"""라이다 4대를 동시에 띄우고 위치 식별 도구를 실행한다.

이 파일의 역할:
    어느 물리 유닛이 앞/뒤/좌/우인지 모르는 상태에서, 4대를 **포트 이름 그대로**
    (yd0/yd1/rp0/rp1) 띄운 뒤 tools/identify_lidar_positions.py 로 손 가림 식별을 한다.
    여기서 나온 매핑을 multi_lidar_drivers.launch.py 의 DEFAULT_PORTS 와
    config/lidar_extrinsics.yaml 에 반영하면 그 뒤로는 이 launch 를 쓸 일이 없다.

출력 topic: /probe/yd0/scan, /probe/yd1/scan, /probe/rp0/scan, /probe/rp1/scan

실행:
    ros2 launch multi_lidar_fusion identify_positions.launch.py

포트가 바뀌었으면 (허브 자리를 옮겼거나 재부팅 후):
    ls -l /dev/serial/by-path/ /dev/serial/by-id/
    ros2 launch multi_lidar_fusion identify_positions.launch.py \
        yd0_port:=/dev/serial/by-path/...  rp0_port:=/dev/serial/by-id/...

★ YDLiDAR 2대는 시리얼이 둘 다 `0001` 이라 by-id 로 구분되지 않는다(실측 확인).
  반드시 **by-path** 를 쓴다. RPLiDAR 2대는 시리얼이 고유해 by-id 로 충분하다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

BY_PATH = '/dev/serial/by-path/'
BY_ID = '/dev/serial/by-id/'

# 2026-08-13 실측 기본값. 허브 자리를 옮기면 by-path 는 바뀐다.
# ★ 2026-08-27: by-path 를 실측값으로 갱신 (…1.2.4/1.2.3 -> …3.4/3.3).
#   옛 값은 2026-08-25 udev 슬롯 고정(tools/99-fma-lidars.rules) 시점에 이미
#   낡아 있었고, 그대로면 YDLiDAR 두 대가 안 열린다.
#
#   ⚠ 이 파일은 **일부러 슬롯 심링크(/dev/lidar_front 등)를 쓰지 않는다.**
#     여기의 목적이 "어느 물리 유닛이 어느 자리인가"를 알아내는 것이라,
#     자리 이름이 붙은 링크를 쓰면 답을 미리 가정하는 꼴이 된다.
#     드라이버 본 launch(multi_lidar_drivers.launch.py)만 심링크를 쓴다.
#
#   허브 자리를 옮겼으면 다시 확인할 것:
#     udevadm info -q property -n /dev/ttyUSBx | grep ID_PATH
DEFAULTS = {
    'yd0': BY_PATH + 'pci-0000:00:14.0-usb-0:3.4:1.0-port0',
    'yd1': BY_PATH + 'pci-0000:00:14.0-usb-0:3.3:1.0-port0',
    'rp0': BY_ID + ('usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                    '76d341fd291ef1118e6dbee40f0f12f8-if00-port0'),
    'rp1': BY_ID + ('usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                    'f2ee467bfb1df111a7b6c4e40f0f12f8-if00-port0'),
}


def _ydlidar(key):
    return Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='probe_' + key,
        output='log',
        parameters=[{
            'port': LaunchConfiguration(key + '_port'),
            'frame_id': 'probe_' + key,
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
        remappings=[('/scan', '/probe/' + key + '/scan')],
    )


def _rplidar(key):
    return Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='probe_' + key,
        output='log',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration(key + '_port'),
            'serial_baudrate': 460800,
            'frame_id': 'probe_' + key,
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        remappings=[('/scan', '/probe/' + key + '/scan')],
    )


def generate_launch_description():
    args = [
        DeclareLaunchArgument(k + '_port', default_value=v,
                              description=k + ' 시리얼 포트')
        for k, v in DEFAULTS.items()
    ]

    identifier = Node(
        package='multi_lidar_fusion',
        executable='identify_lidar_positions.py',
        name='identify_lidar_positions',
        output='screen',
        emulate_tty=True,
        # 포트는 키마다 개별 파라미터로 넘긴다 — LaunchConfiguration 여러 개를
        # 리스트에 담으면 launch_ros 가 한 문자열로 이어붙인다(STRING_ARRAY 안 됨).
        parameters=[{
            'keys': ['yd0', 'yd1', 'rp0', 'rp1'],
            'positions': ['front', 'rear', 'left', 'right'],
            'port_yd0': LaunchConfiguration('yd0_port'),
            'port_yd1': LaunchConfiguration('yd1_port'),
            'port_rp0': LaunchConfiguration('rp0_port'),
            'port_rp1': LaunchConfiguration('rp1_port'),
        }],
    )

    return LaunchDescription(args + [
        _ydlidar('yd0'),
        _ydlidar('yd1'),
        _rplidar('rp0'),
        _rplidar('rp1'),
        identifier,
    ])
