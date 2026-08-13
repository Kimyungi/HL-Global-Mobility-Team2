"""multi_lidar_fusion — 센서 드라이버 4대 launch.

이 파일의 역할:
    YDLiDAR T-mini Plus 2대 + SLAMTEC RPLiDAR C1M1 2대를 각각 다른 토픽/frame 으로
    띄운다. 융합 노드와 분리한 이유는 드라이버가 죽어도 융합 노드는 살아 있어야 하고
    (요구 §20 Case1), rosbag 재생 때는 드라이버 없이 융합만 돌려야 하기 때문이다.

    ★ /dev/ttyUSB* 를 직접 쓰지 않는다. 부팅·재연결마다 번호가 바뀌고, 이 PC 의
      /etc/udev/rules.d/99-ydlidar.rules 는 CP210x(10c4:ea60) 전체를 /dev/ydlidar 로
      묶어버린다 — RPLiDAR C1M1 과 IMU 도 같은 칩이다.
      (2026-08-13 확인: /dev/ydlidar 와 /dev/rplidar 가 둘 다 같은 장치를 가리켰음)

    어느 유닛이 어느 위치인지 다시 확인해야 하면:
        ros2 launch multi_lidar_fusion view_one_lidar.launch.py unit:=yd0
      (한 대만 띄워 RViz 로 본다. unit = yd0|yd1|rp0|rp1)
    포트 목록만 보려면:
        ros2 run multi_lidar_fusion identify_lidars.sh

출력 topic: /lidar/a1/scan, /lidar/a2/scan, /lidar/b1/scan, /lidar/b2/scan
출력 frame: lidar_a1_link, lidar_a2_link, lidar_b1_link, lidar_b2_link
            (base_link 로의 TF 는 융합 노드가 낸다)

실행:
    ros2 launch multi_lidar_fusion multi_lidar_drivers.launch.py
    ros2 launch multi_lidar_fusion multi_lidar_drivers.launch.py a1_port:=/dev/ttyUSB1
    ros2 launch multi_lidar_fusion multi_lidar_drivers.launch.py enable_a2:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

BY_ID = '/dev/serial/by-id/'
BY_PATH = '/dev/serial/by-path/'

# ▼ 2026-08-13 실차 확정 (view_one_lidar.launch.py 로 한 대씩 띄워 눈으로 확인).
#
#   YDLiDAR 2대는 **by-path** 를 쓴다. CP210x 시리얼이 둘 다 `0001` 이라 by-id 이름이
#   완전히 같아져서, 나중에 붙은 쪽이 먼저 것의 링크를 덮어쓴다(실측: by-path 6개 vs
#   by-id 5개, /dev/ydlidar 와 /dev/rplidar 가 같은 장치를 가리킴).
#   RPLiDAR 2대는 시리얼이 고유해 by-id 로 충분하다.
#
#   ★ by-path 는 "허브의 그 구멍"이 주소다. 케이블을 다른 포트에 옮겨 꽂으면
#     앞/뒤가 조용히 뒤바뀐다. 옮겼으면 view_one_lidar 로 다시 확인할 것.
DEFAULT_PORTS = {
    # YDLiDAR T-mini Plus — 각분해능 0.839deg, range 12m
    'a1': BY_PATH + 'pci-0000:00:14.0-usb-0:1.2.4:1.0-port0',   # 전방 (unit yd0)
    'a2': BY_PATH + 'pci-0000:00:14.0-usb-0:1.2.3:1.0-port0',   # 후방 (unit yd1)
    # SLAMTEC RPLiDAR C1M1 — 각분해능 0.499deg, range 16m
    'b1': BY_ID + ('usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                   'f2ee467bfb1df111a7b6c4e40f0f12f8-if00-port0'),   # 좌측 (unit rp1)
    'b2': BY_ID + ('usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                   '76d341fd291ef1118e6dbee40f0f12f8-if00-port0'),   # 우측 (unit rp0)
}


def _ydlidar(sensor_id):
    """YDLiDAR T-mini Plus 1대. 파라미터는 params/Tmini-Plus-SH.yaml 기준."""
    return Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_' + sensor_id,
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_' + sensor_id)),
        parameters=[{
            'port': LaunchConfiguration(sensor_id + '_port'),
            'frame_id': 'lidar_' + sensor_id + '_link',
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
            # true 로 두면 미반사가 inf 로 온다 — 융합 노드가 NaN/Inf 를 제거하므로
            # 어느 쪽이든 동작하지만, 관례에 맞춰 inf 로 받는다.
            'invalid_range_is_inf': True,
            'ignore_array': '',
            'debug': False,
        }],
        remappings=[('/scan', '/lidar/' + sensor_id + '/scan')],
    )


def _rplidar(sensor_id):
    """SLAMTEC RPLiDAR C1M1 1대."""
    return Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_' + sensor_id,
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_' + sensor_id)),
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration(sensor_id + '_port'),
            'serial_baudrate': 460800,      # C1 은 460800 (A1 의 115200 아님)
            'frame_id': 'lidar_' + sensor_id + '_link',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        remappings=[('/scan', '/lidar/' + sensor_id + '/scan')],
    )


def generate_launch_description():
    args = []
    for sid, port in DEFAULT_PORTS.items():
        args.append(
            DeclareLaunchArgument(
                sid + '_port', default_value=port,
                description=sid + ' 시리얼 포트 (/dev/serial/by-id/ 권장)'))
        args.append(
            DeclareLaunchArgument(
                'enable_' + sid, default_value='true',
                description=sid + ' 드라이버 실행 여부'))

    return LaunchDescription(args + [
        _ydlidar('a1'),
        _ydlidar('a2'),
        _rplidar('b1'),
        _rplidar('b2'),
    ])
