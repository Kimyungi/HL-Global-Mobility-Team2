"""라이다 4대를 동시에 띄우고 위치 식별 도구를 실행한다.

이 파일의 역할:
    어느 물리 유닛이 앞/뒤/좌/우인지 모르는 상태에서, 4대를 **포트 이름 그대로**
    (yd0/yd1/b1/b2) 띄운 뒤 tools/identify_lidar_positions.py 로 손 가림 식별을 한다.
    여기서 나온 매핑을 multi_lidar_drivers.launch.py 의 DEFAULT_PORTS 와
    config/lidar_extrinsics.yaml 에 반영하면 그 뒤로는 이 launch 를 쓸 일이 없다.

출력 topic: /probe/yd0/scan, /probe/yd1/scan, /probe/b1/scan, /probe/b2/scan

실행:
    ros2 launch multi_lidar_fusion identify_positions.launch.py

포트가 바뀌었으면 (허브 자리를 옮겼거나 재부팅 후):
    ls -l /dev/serial/by-path/ /dev/serial/by-id/
    ros2 launch multi_lidar_fusion identify_positions.launch.py \
        yd0_port:=/dev/serial/by-path/...  b1_port:=/dev/serial/by-path/...

★ 네 대 모두 YDLiDAR다. 앞/뒤 경로는 기존값을 유지하고 새 좌/우는 udev 위치
  심링크를 쓴다. 좌/우 케이블을 옮겼으면 99-fma-lidars.rules도 함께 갱신한다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# 실기 확인 USB 위치는 고정한다. 원시 포트 재식별은 view_one_lidar의 custom을 쓴다.
DEFAULTS = {
    'yd0': '/dev/lidar_front',
    'yd1': '/dev/lidar_rear',
    'b1': '/dev/lidar_left',
    'b2': '/dev/lidar_right',
}


def _ydlidar(key):
    stable_stream = key != 'yd0'
    return Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='probe_' + key,
        output='log',
        parameters=[{
            'port': LaunchConfiguration(key + '_port'),
            'frame_id': 'probe_' + key,
            'baudrate': 230400,
            'lidar_type': 1,
            'device_type': 0,
            'sample_rate': 4 if stable_stream else 9,
            'abnormal_check_count': 4,
            'fixed_resolution': not stable_stream,
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
            'invalid_range_is_inf': not stable_stream,
            'ignore_array': '',
            'debug': False,
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
            'keys': ['yd0', 'yd1', 'b1', 'b2'],
            'positions': ['front', 'rear', 'left', 'right'],
            'port_yd0': LaunchConfiguration('yd0_port'),
            'port_yd1': LaunchConfiguration('yd1_port'),
            'port_b1': LaunchConfiguration('b1_port'),
            'port_b2': LaunchConfiguration('b2_port'),
        }],
    )

    return LaunchDescription(args + [
        _ydlidar('yd0'),
        _ydlidar('yd1'),
        _ydlidar('b1'),
        _ydlidar('b2'),
        identifier,
    ])
