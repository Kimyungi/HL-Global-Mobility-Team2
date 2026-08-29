"""라이다 2대 동시 표시 — 스캔 평면 높이(z) 맞추기용.  이기돈

로컬라이제이션을 위해 서로 다른 라이다 2대의 **스캔 높이를 일치시키는** 작업용 구성.
두 스캔을 다른 색으로 RViz 에 겹쳐 띄우고, 장착 TF 의 z 를 인자로 바꿔가며 맞춘다.

  A(파랑)  YDLidar T-mini Plus   /scan_a   frame laser_a
  B(빨강)  Slamtec RPLidar C1M1  /scan_b   frame laser_b
  공통 부모 frame: base_link (RViz Fixed Frame)

  ros2 launch stack_avoid dual_lidar.launch.py

높이만 맞추는 중이면 z 두 개만 만지면 된다 (단위 m, base_link 기준):

  ros2 launch stack_avoid dual_lidar.launch.py z_a:=0.065 z_b:=0.180

★ 포트 뒤바뀜 방지 — 기본값은 **시리얼 번호 기반 by-id 경로**다. /dev/ttyUSB* 번호는
  연결 순서마다 바뀌지만 by-id 는 장비에 고정된다. 2026-08-11 실측 식별 (장비 자체 보고):
      T-mini Plus : 구형 CP2102, 시리얼 0001
                    → Model "Tmini Plus", FW 1.2, S/N 2025110300090660
      RPLidar C1M1: 신형 CP2102N, 시리얼 f2ee467bfb1df111a7b6c4e40f0f12f8
                    → S/N D669E0F8C8EA9CCCA4939FF84AC2480D, FW 1.02, HW 18
  ※ T-mini 쪽 CP2102 는 시리얼이 "0001" 이라 같은 칩을 쓰는 다른 장비를 함께 꽂으면
    by-id 가 충돌할 수 있다. 그때는 port_a/port_b 로 직접 지정할 것.

★ 통신 설정이 서로 다르다. 틀리면 드라이버가 **에러 없이 빈 스캔만** 낸다.
      T-mini Plus : 230400
      RPLidar C1  : 460800  (scan mode DenseBoost, 5kHz, 10Hz)

★★ 장착 yaw 는 **고정값이다. 세팅이 바뀌어도 이 값을 쓸 것.**
      A T-mini Plus : +π/2 (90°)  — 프로젝트 확정 규약
          params.yaml forward_angle_deg=270 (필드검증 2026-08-05) 에서
          stack_avoid_node 가 TF yaw = 0 − 270° ≡ +90° 로 발행하는 값과 같고,
          stack_estop 의 laser_yaw_in_base_rad=1.57079632679 과도 일치한다.
      B RPLidar C1M1: +π (180°)   — 2026-08-11 실물 확인 후 확정
          C1 의 0° 기준이 T-mini 와 반대쪽을 향해, 보정 없이 두면 두 스캔이
          180° 어긋나 같은 벽이 서로 반대편에 그려진다.
      두 값 중 하나만 바꿔도 정합이 깨진다. 실차 4대 구성으로 옮길 때도 그대로 쓸 것.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue

# 실측 식별자 기반 고정 경로 (2026-08-11). ★ 칩 세대가 서로 반대라 헷갈리기 쉽다:
#   T-mini Plus 가 구형 CP2102(시리얼 0001), C1 이 신형 CP2102N 이다.
PORT_A = ('/dev/serial/by-id/'
          'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
PORT_B = ('/dev/serial/by-id/'
          'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
          'f2ee467bfb1df111a7b6c4e40f0f12f8-if00-port0')

# T-mini Plus 실측 기준값 (ydlidar.yaml 과 동일)
YD = dict(ignore_array='', device_type=0, isSingleChannel=False, intensity=False,
          intensity_bit=0, abnormal_check_count=4, fixed_resolution=True,
          reversion=False, inverted=False, auto_reconnect=True,
          support_motor_dtr=False, angle_max=180.0, angle_min=-180.0,
          range_max=64.0, range_min=0.01, frequency=10.0,
          invalid_range_is_inf=False, debug=False,
          lidar_type=1, sample_rate=9)


def _tf(tag):
    """장착 TF. 높이 맞추기는 z 만 만지면 된다."""
    return Node(
        package='tf2_ros', executable='static_transform_publisher',
        name=f'tf_laser_{tag}', output='log',
        arguments=['--x', LaunchConfiguration(f'x_{tag}'),
                   '--y', LaunchConfiguration(f'y_{tag}'),
                   '--z', LaunchConfiguration(f'z_{tag}'),
                   '--yaw', LaunchConfiguration(f'yaw_{tag}'),
                   '--frame-id', 'base_link',
                   '--child-frame-id', f'laser_{tag}'])


def generate_launch_description():
    rviz_cfg = os.path.join(get_package_share_directory('stack_avoid'),
                            'config', 'dual_lidar.rviz')
    yd = dict(YD)
    yd.update({'port': ParameterValue(LaunchConfiguration('port_a'), value_type=str),
               'frame_id': 'laser_a',
               'baudrate': ParameterValue(LaunchConfiguration('baud_a'), value_type=int)})

    return LaunchDescription([
        DeclareLaunchArgument('port_a', default_value=PORT_A,
                              description='A: YDLidar T-mini Plus 포트'),
        DeclareLaunchArgument('port_b', default_value=PORT_B,
                              description='B: RPLidar C1M1 포트'),
        DeclareLaunchArgument('baud_a', default_value='230400'),
        DeclareLaunchArgument('baud_b', default_value='460800'),
        # ── 장착 TF. ★ 높이 맞추기는 z_a / z_b 만 조정 ──
        DeclareLaunchArgument('x_a', default_value='0.0'),
        DeclareLaunchArgument('y_a', default_value='0.0'),
        DeclareLaunchArgument('z_a', default_value='0.0',
                              description='A(T-mini) 스캔 평면 높이 [m]'),
        # ★ T-mini Plus 방향 규약 = +π/2 (90°). 임의값이 아니라 프로젝트 확정값이다:
        #   params.yaml 의 forward_angle_deg=270(필드검증 2026-08-05) 에서
        #   stack_avoid_node 가 TF yaw = radians(lidar_yaw) − front_center
        #                            = 0 − 270° = −270° ≡ +90° 로 발행하고,
        #   stack_estop 의 laser_yaw_in_base_rad=1.57079632679 과도 일치한다.
        #   여기서 다른 값을 쓰면 이 도구로 맞춘 방향이 실차 구성과 어긋난다.
        DeclareLaunchArgument('yaw_a', default_value='1.5707963',
                              description='A(T-mini) 장착 yaw [rad]. 기본 +π/2 = 프로젝트 규약'),
        DeclareLaunchArgument('x_b', default_value='0.0'),
        DeclareLaunchArgument('y_b', default_value='0.0'),
        DeclareLaunchArgument('z_b', default_value='0.0',
                              description='B(C1) 스캔 평면 높이 [m]'),
        # ★ RPLidar C1M1 방향 = π (180°). 2026-08-11 실물 확인 후 이 방향으로 고정한다.
        #   C1 은 T-mini 와 0° 기준이 반대쪽을 향해, 그대로 두면 두 스캔이 180° 어긋난다.
        #   **다른 세팅에서도 이 값을 쓸 것** — 바꾸면 두 라이다 정합이 깨진다.
        DeclareLaunchArgument('yaw_b', default_value='3.1415927',
                              description='B(C1) 장착 yaw [rad]. 기본 π = 2026-08-11 확정'),
        DeclareLaunchArgument('label_r', default_value='2.5',
                              description='각도 눈금 반지름 [m]'),

        # A: YDLidar T-mini Plus → /scan_a (파랑)
        LifecycleNode(package='ydlidar_ros2_driver',
                      executable='ydlidar_ros2_driver_node',
                      name='ydlidar_a', namespace='/', output='screen',
                      emulate_tty=True, parameters=[yd],
                      remappings=[('/scan', '/scan_a')]),
        _tf('a'),

        # B: Slamtec RPLidar C1M1 → /scan_b (빨강)
        Node(package='rplidar_ros', executable='rplidar_composition',
             name='rplidar_b', output='screen',
             parameters=[{
                 'channel_type': 'serial',
                 'serial_port': ParameterValue(LaunchConfiguration('port_b'),
                                               value_type=str),
                 'serial_baudrate': ParameterValue(LaunchConfiguration('baud_b'),
                                                   value_type=int),
                 'frame_id': 'laser_b',
                 'inverted': False,
                 'angle_compensate': True,
             }],
             remappings=[('/scan', '/scan_b')]),
        _tf('b'),

        # ── 각도 눈금 (0/90/180/270°) — 각 라이다 프레임 기준, 라이다 색과 동일 ──
        # 라이다 원(raw) 각도를 그 라이다 좌표계에 그린다. 두 라이다의 0° 가 서로
        # 어디를 향하는지(장착 회전 차이) 눈으로 바로 비교할 수 있다.
        Node(package='stack_avoid', executable='angle_labels', name='angle_labels_a',
             output='log',
             parameters=[{'frame_id': 'laser_a',
                          'radius_m': ParameterValue(LaunchConfiguration('label_r'),
                                                     value_type=float),
                          'step_deg': 90,
                          'color': [0.0, 0.47, 1.0]}],      # 파랑 (A)
             remappings=[('angle_labels', '/angle_labels_a')]),
        Node(package='stack_avoid', executable='angle_labels', name='angle_labels_b',
             output='log',
             parameters=[{'frame_id': 'laser_b',
                          'radius_m': ParameterValue(LaunchConfiguration('label_r'),
                                                     value_type=float),
                          'step_deg': 90,
                          'color': [1.0, 0.16, 0.16]}],     # 빨강 (B)
             remappings=[('angle_labels', '/angle_labels_b')]),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg], output='log'),
    ])
