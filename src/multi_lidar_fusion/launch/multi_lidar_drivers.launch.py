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
#
# ★★ 2026-08-27: 기본값을 **udev 심링크**로 바꿨다.
#
#   왜 — 여기 적혀 있던 by-path(...1.2.4 / ...1.2.3)는 2026-08-25 에 이미 낡아
#   있었다. 그날 udev 슬롯 고정(tools/99-fma-lidars.rules)을 넣으면서 이 표를
#   같이 안 고쳤고, 실제 경로는 ...3.4 / ...3.3 이라 **launch 가 인자 없이는
#   YDLiDAR 두 대를 못 연다.** 값이 두 곳에 갈라져 있는 한 같은 일이 또 난다.
#
#   심링크는 그 갈라짐을 없앤다 — 슬롯의 단일 원천이 udev 규칙 한 곳이 되고,
#   규칙이 ID_PATH(YD)·시리얼(RP)로 매칭하므로 **ttyUSB 번호가 바뀌어도,
#   RPLiDAR 는 구멍을 옮겨 꽂아도 따라간다.**
#
#   ⚠ 전제: 규칙이 설치돼 있어야 한다. 없으면 네 링크가 아예 없어서
#     "포트를 못 연다"로 나타난다 — 증상만 보면 라이다 고장과 구분이 안 된다.
#       ls -l /dev/lidar_front /dev/lidar_rear /dev/lidar_left /dev/lidar_right
#       sudo cp ../tools/99-fma-lidars.rules /etc/udev/rules.d/
#       sudo udevadm control --reload-rules && sudo udevadm trigger
#
#   ⚠ YD 두 대는 시리얼이 **둘 다 "0001"** 이라 규칙이 ID_PATH(허브 구멍)로
#     가른다. 즉 **YD 를 다른 구멍에 옮겨 꽂으면 앞/뒤가 뒤바뀐다** — by-path
#     시절과 같은 함정이고, 옮겼으면 규칙의 ID_PATH 를 고쳐야 한다:
#       udevadm info -q property -n /dev/ttyUSBx | grep ID_PATH
#
#   되돌리려면 인자로 넘기면 된다 — 예: a1_port:=/dev/serial/by-path/...
DEFAULT_PORTS = {
    # YDLiDAR T-mini Plus — 각분해능 0.839deg, range 12m
    # 규칙이 허브 구멍(ID_PATH)으로 가른다: 전방 3.4 / 후방 3.3
    'a1': '/dev/lidar_front',   # 전방 (unit yd0) — /dev/ttyUSB_LIDAR 와 같은 장치
    'a2': '/dev/lidar_rear',    # 후방 (unit yd1)
    # SLAMTEC RPLiDAR C1M1 — 각분해능 0.499deg, range 16m
    # 규칙이 시리얼로 가른다 (좌 f2ee467b… / 우 76d341fd…) — 구멍을 옮겨도 따라간다.
    # 2026-08-13 현장 식별값(view_one_lidar 로 한 대씩 확인). 8/14 에 한 번 맞바꿨다가
    # 되돌렸다 — "좌우가 서로의 자리"로 보이던 증상의 원인은 포트가 아니라 **각도 반전**
    # (아래 inverted 주석 참조)이었다.
    'b1': '/dev/lidar_left',    # 좌측 (unit rp1)
    'b2': '/dev/lidar_right',   # 우측 (unit rp0)
}


def _ydlidar(sensor_id):
    """YDLiDAR T-mini Plus 1대. 파라미터는 params/Tmini-Plus-SH.yaml 기준.

    ★ reversion / inverted 는 **각도 규약 그 자체**다 (reversion = 180도 회전,
      inverted = 각도 부호 반전). 이 두 값이 다르면 같은 물체가 다른 각도로 찍힌다.

      ⚠ 2026-08-14: 이 두 값을 false 로 바꿔 봤다가 되돌렸다. 근거는 "회피 스택이
        쓰는 ydlidar.yaml 이 false/false 이고 전방=raw 270 이 거기서 측정됐다" 였는데,
        **그 파일은 lidar_type=1 / sample_rate=9 이고 여기는 0 / 4 다** — 각도 관련
        두 개만 가져오고 나머지 절반은 다른 상태로 둔 셈이라 전제가 성립하지 않았다.
        실제로 판을 앞에 두고 재니 **설정된 정면(raw 270)에서 179도 어긋나** 보였다
        (reversion 이 정확히 180도 회전이라는 것과 일치).
        따라서 실차에서 실제로 돌던 값(true/true)으로 되돌린다. 이 값을 다시 건드릴
        때는 반드시 **판을 앞에 두고 raw 각도를 확인한 뒤**에 할 것.
    """
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
            'sample_rate': 9,
            'abnormal_check_count': 4,
            'fixed_resolution': True,
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
    """SLAMTEC RPLiDAR C1M1 1대.

    ⚠ 부트 배너가 `RP S2 LIDAR System.` 으로 나온다 — **모델명이 아니라 펌웨어
      플랫폼 배너다. C1M1 이 맞다.** 2026-08-30 에 실측으로 확정했다:

          모델 바이트 0x41 · FW 1.02 · HW 18 · 보드 460800
          샘플레이트 5 kHz · 10 Hz · 물리 분해능 0.72°(511점/회전)
          지원 모드  Standard(16.0m) / DenseBoost(40.0m)

      S2 라면 0.12° · 32 kHz · 1 Mbps 여야 하므로 하나도 맞지 않는다.
      배너만 보고 모델을 바꿔 잡지 말 것 (아래 `serial_baudrate` 도 그래서 460800).

      참고: 아래 `angle_compensate=True` 때문에 드라이버가 내는 `angle_increment` 는
      0.499°(720 bin)로 보간된 값이다. 물리 분해능 0.72° 와 다른 수치인 게 정상이며,
      융합의 `scan.angle_increment` 하한(1.0°) 근거는 **성긴 쪽인 T-mini 0.839°** 다.

    ⚠ **먹통 상태는 드라이버가 못 푼다 — `RESET(0xA5 0x40)` 이 필요하다.**
      이 노드는 STOP → GET_INFO → GET_HEALTH → SCAN 만 보내고 RESET 을 보내지 않는다.
      그래서 한 번 걸리면 재기동·재연결·`/start_motor` 무엇으로도 안 풀린다.
      증상: **health OK 인데 `/scan` 0 Hz** (SCAN 을 걸면 디스크립터 7바이트만 오고
      측정 데이터가 0 = 모터가 안 돈다).

          python3 - <<'PY'
          import serial, time
          for d in ['/dev/lidar_left', '/dev/lidar_right']:
              s = serial.Serial(d, 460800, timeout=1)
              s.write(b'\\xA5\\x25'); time.sleep(0.3); s.reset_input_buffer()
              s.write(b'\\xA5\\x40'); time.sleep(2.5)   # RESET
              print(d, b'LIDAR System' in s.read(s.in_waiting or 1)); s.close()
          PY

      한 번 풀리면 드라이버를 SIGTERM/SIGKILL 어느 쪽으로 죽여도 재발하지 않는다
      (2026-08-30, 재기동 4회 연속 확인). 들어가는 원인은 전원으로 보인다 —
      스캔 도중 전압이 끊기면 컨트롤러는 살아남고 모터만 latch-off 된다.
      HANDOVER §3.7 의 "전류를 갈라라"와 **같은 고장의 앞뒤**다: 전류 분리는 재발을
      막고, 이 RESET 은 이미 걸린 것을 푼다.

      ※ 위 스크립트가 `False` 를 내도 그 자체로 고장은 아니다 — 배너는 읽기 창을
        놓치면 안 잡히고, **다른 프로세스가 포트를 잡고 있으면 반드시 실패한다.**
        최종 판정은 `/lidar/*/scan` 이 도느냐로 한다. 특히 **이 launch 를 두 번
        띄우면** 포트마다 프로세스가 둘씩 붙어 서로를 죽이므로, 새로 띄우기 전에
        `fuser $(readlink -f /dev/lidar_left)` 로 비었는지 볼 것.
    """
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
            # ★ YD 와 같은 이유 (위 _ydlidar 주석). 네 대가 같은 방향 규약을 써야 한다.
            'inverted': True,
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
                description=sid + ' 시리얼 포트 (기본 = udev 슬롯 심링크 /dev/lidar_*)'))
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
