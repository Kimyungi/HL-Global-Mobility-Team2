"""multi_lidar_fusion — 센서 드라이버 4대 launch.

이 파일의 역할:
    YDLiDAR T-mini Plus 4대를 각각 다른 토픽/frame 으로
    띄운다. 융합 노드와 분리한 이유는 드라이버가 죽어도 융합 노드는 살아 있어야 하고
    (요구 §20 Case1), rosbag 재생 때는 드라이버 없이 융합만 돌려야 하기 때문이다.

    ★ /dev/ttyUSB* 를 직접 쓰지 않는다. 부팅·재연결마다 번호가 바뀌고, 이 PC 의
      /etc/udev/rules.d/99-ydlidar.rules 는 CP210x(10c4:ea60) 전체를 /dev/ydlidar 로
      묶어버릴 수 있다 — IMU도 같은 칩을 쓸 수 있으므로 위치 링크만 사용한다.

    어느 유닛이 어느 위치인지 다시 확인해야 하면:
        ros2 launch multi_lidar_fusion view_one_lidar.launch.py unit:=b1
      (한 대만 띄워 RViz 로 본다. unit = yd0|yd1|b1|b2)
    포트 목록만 보려면:
        ros2 run multi_lidar_fusion identify_lidars.sh

출력 topic: /lidar/a1/scan, /lidar/a2/scan, /lidar/b1/scan, /lidar/b2/scan
출력 frame: lidar_a1_link, lidar_a2_link, lidar_b1_link, lidar_b2_link
            (base_link 로의 TF 는 융합 노드가 낸다)

실행:
    ros2 launch multi_lidar_fusion multi_lidar_drivers.launch.py
    ros2 launch multi_lidar_fusion multi_lidar_drivers.launch.py \
        a1_port:=/dev/ttyUSB1
    ros2 launch multi_lidar_fusion multi_lidar_drivers.launch.py \
        enable_a2:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# 네 대 모두 같은 YDLiDAR/CP2102 계열이므로 udev의 허브 위치 기반 심링크를 쓴다.
# 실기에서 확인한 네 USB 허브 위치가 고정 배선 계약이다. 케이블을 옮기지 않는다.
#
#   ⚠ 전제: 규칙이 설치돼 있어야 한다. 없으면 네 링크가 아예 없어서
#     "포트를 못 연다"로 나타난다 — 증상만 보면 라이다 고장과 구분이 안 된다.
#       ls -l /dev/lidar_front /dev/lidar_rear /dev/lidar_left /dev/lidar_right
#       sudo cp ../tools/99-fma-lidars.rules /etc/udev/rules.d/
#       sudo udevadm control --reload-rules && sudo udevadm trigger
#
#   허브나 메인보드를 교체한 경우에만 네 위치를 다시 식별하고 규칙을 갱신한다:
#       udevadm info -q property -n /dev/ttyUSBx | grep ID_PATH
#
#   되돌리려면 인자로 넘기면 된다 — 예: a1_port:=/dev/serial/by-path/...
DEFAULT_PORTS = {
    # YDLiDAR T-mini Plus — 네 위치 모두 같은 드라이버/각도 옵션 사용.
    'a1': '/dev/lidar_front',   # 전방 (unit yd0) — /dev/ttyUSB_LIDAR 와 같은 장치
    'a2': '/dev/lidar_rear',    # 후방 (unit yd1)
    'b1': '/dev/lidar_left',    # 좌측 신규 YD
    'b2': '/dev/lidar_right',   # 우측 신규 YD
}


def _ydlidar(sensor_id):
    """앞/뒤 YDLiDAR. 각도 옵션은 실기 확정값과 한 세트다."""
    # 후면 a2 실기(2026-09-01): SDK 보고값은 4K/8bit이며, 기존 9K/고정 430점/16bit
    # 조합에서는 checksum 연속 오류 뒤 YdDataStream out_of_range로 종료됐다.
    # FOV/TF는 건드리지 않고 수집 파싱 설정만 실기값으로 맞춘다.
    rear = sensor_id == 'a2'
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
            'lidar_type': 1,
            'device_type': 0,
            'sample_rate': 4 if rear else 9,
            'abnormal_check_count': 4,
            'fixed_resolution': False if rear else True,
            # ★ inverted = 각도 **부호 반전(거울상)**. reversion = 180도 회전.
            #   2026-08-14: inverted=True 때문에 병합 화면 전체가 좌우 거울상이었다.
            #   정면에 판을 두고 yaw 를 맞췄던 탓에 정면에서는 상쇄되고 좌우로만 드러났다:
            #       a_rep = Y - b,  yaw_cfg = -Y  ->  b_pipe = -b  (좌우만 반전)
            #   반전을 끄고 yaw_deg 를 부호 반전시켜 함께 정정했다.
            'reversion': True,
            'inverted': False,
            'auto_reconnect': True,
            'isSingleChannel': False,
            'intensity': True,
            'intensity_bit': 8 if rear else 16,
            'support_motor_dtr': False,
            'frequency': 10.0,
            'angle_max': 180.0,
            'angle_min': -180.0,
            'range_max': 12.0,
            'range_min': 0.03,
            # true 로 두면 미반사가 inf 로 온다 — 융합 노드가 NaN/Inf 를 제거하므로
            # 어느 쪽이든 동작하지만, 관례에 맞춰 inf 로 받는다.
            'invalid_range_is_inf': False if rear else True,
            'ignore_array': '',
            'debug': False,
        }],
        remappings=[('/scan', '/lidar/' + sensor_id + '/scan')],
    )


def _side_ydlidar(sensor_id):
    """교체된 좌/우 YDLiDAR. a1/a2 기존 드라이버 설정과 분리한다."""
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
            'lidar_type': 1,
            'device_type': 0,
            'sample_rate': 4,
            'abnormal_check_count': 4,
            # 전체 회전을 보존하고 융합 단계에서 raw 270 중심 110도만 사용한다.
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
        remappings=[('/scan', '/lidar/' + sensor_id + '/scan')],
    )


def generate_launch_description():
    args = []
    for sid, port in DEFAULT_PORTS.items():
        args.append(
            DeclareLaunchArgument(
                sid + '_port', default_value=port,
                description=sid + ' 시리얼 포트 (기본 = udev 슬롯 심링크 /dev/lidar_*)'))
        args.append(
            DeclareLaunchArgument(
                'enable_' + sid, default_value='true',
                description=sid + ' 드라이버 실행 여부'))

    return LaunchDescription(args + [
        _ydlidar('a1'),
        _ydlidar('a2'),
        _side_ydlidar('b1'),
        _side_ydlidar('b2'),
    ])
