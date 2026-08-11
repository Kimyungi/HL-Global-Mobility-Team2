"""차선+GPS 통합 실차 주행 launch — lane ↔ waypoint 자동 전이 (CLAUDE.md §4).

한 번에 띄우는 노드 (2026-08-11 통합 점검 §3의 "터미널 5개" 조합을 대체):
  ydlidar + laser static tf + stack_estop   (REAL_VEHICLE_stack_estop_mgm_can과 동일 구성)
  stack_gps    — waypoint_csv 필수 인자
  stack_lane   — 실측 호모그래피·MxID 핀닝·오실레이션 잠정 튜닝(TESTING_LOG §7.3) 기본 적용
  adas_mgm     — config/params.yaml 적용 (기존 REAL_VEHICLE launch는 params 누락이었음)
  bridge_dspace — 실제 CAN TX

주의:
- stack_estop/launch/REAL_VEHICLE_stack_estop_mgm_can.launch.py 와 동시 실행 금지
  (estop/mgm/bridge 중복). 이 파일 하나만 띄운다.
- 실제 CAN TX가 나가므로 동일한 확인 토큰을 요구한다.

사용:
  ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py \
      REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
      waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/<코스>.csv
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


CONFIRM_TOKEN = 'I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX'

# 실측 호모그래피 (2026-08-11 캘리브레이션, LOO RMS 0.041m) — 소스 트리 절대경로로
# 지정해야 한다: 노드 기본 경로는 설치본 내부로 해석돼 파일을 못 찾는다
# (stack_lane CALIBRATION_GUIDE.md §6).
DEFAULT_HOMOGRAPHY = os.path.expanduser(
    '~/FMA_ws/src/stack_lane/config/homography.json')

DEFAULT_YDLIDAR_PARAMS = os.path.join(
    os.path.expanduser('~'), 'ydlidar_ws', 'src', 'ydlidar_ros2_driver',
    'params', 'Tmini-Plus-SH.yaml')


def validate(context):
    if LaunchConfiguration('REAL_VEHICLE_CONFIRM').perform(context) != CONFIRM_TOKEN:
        raise RuntimeError(
            'REAL VEHICLE launch refused. '
            'Set REAL_VEHICLE_CONFIRM:=' + CONFIRM_TOKEN)
    if not LaunchConfiguration('waypoint_csv').perform(context):
        raise RuntimeError('waypoint_csv:=<코스 CSV 경로> 를 지정하세요 (stack_gps 필수)')
    homography = LaunchConfiguration('homography_path').perform(context)
    if not os.path.isfile(homography):
        raise RuntimeError(
            f'호모그래피 파일 없음: {homography} — placeholder 실주행 금지 '
            '(stack_lane CALIBRATION_GUIDE.md)')
    return []


def generate_launch_description():
    mgm_params = os.path.join(
        get_package_share_directory('adas_mgm'), 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('REAL_VEHICLE_CONFIRM', default_value='NOT_CONFIRMED'),
        DeclareLaunchArgument('can_interface', default_value='can0'),

        # ── stack_gps (DRIVE_GUIDE.md V2와 동일 인자)
        DeclareLaunchArgument('waypoint_csv', default_value='',
                              description='코스 웨이포인트 CSV (필수)'),
        DeclareLaunchArgument('rtcm_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('gps_error_log_csv', default_value=''),

        # ── stack_lane
        DeclareLaunchArgument('homography_path', default_value=DEFAULT_HOMOGRAPHY),
        DeclareLaunchArgument('camera_mxid', default_value='14442C105157D3D200',
                              description='차선용 OAK-D MxID (2026-08-11 실측)'),
        DeclareLaunchArgument('lane_device', default_value='0',
                              description="YOLOPv2 추론 장치: cuda 인덱스 또는 'cpu'"),
        # TESTING_LOG §7.3 잠정 최적값 — 1.8m가 오실레이션 최저(잔차 std 1.92°),
        # 단 표본 51초라 미확정. 재튜닝 시 인자만 바꿔 재실행.
        DeclareLaunchArgument('ref_point0_lookahead_m', default_value='1.8'),
        DeclareLaunchArgument('ref_point0_extrap_mode', default_value='linear'),
        DeclareLaunchArgument('coeff_smoothing_alpha', default_value='0.3'),

        # ── stack_estop (REAL_VEHICLE_stack_estop_mgm_can과 동일)
        DeclareLaunchArgument('ydlidar_params', default_value=DEFAULT_YDLIDAR_PARAMS),
        # /dev/ttyUSB0 고정 금지 — 이 PC에선 USB0=무전기, USB1=IMU, USB2=라이다로
        # 열거된다 (2026-08-11 확인). udev 별칭(ttyUSB_LIDAR, MODE 0666)으로 고정.
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB_LIDAR'),
        DeclareLaunchArgument('laser_yaw_in_base_rad', default_value='1.57079632679'),
        DeclareLaunchArgument('dynamic_enabled', default_value='true'),
        DeclareLaunchArgument('dynamic_stop_distance_m', default_value='1.20'),
        DeclareLaunchArgument('dynamic_tracking_max_distance_m', default_value='3.00'),

        OpaqueFunction(function=validate),

        LifecycleNode(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            namespace='/',
            parameters=[LaunchConfiguration('ydlidar_params'), {
                'port': LaunchConfiguration('lidar_port'),
                'baudrate': 230400,
                'lidar_type': 1,
                'intensity_bit': 8,
            }],
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_static_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.02',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '1.57079632679',
                '--frame-id', 'base_link', '--child-frame-id', 'laser_frame',
            ],
            output='screen',
        ),

        Node(
            package='stack_estop',
            executable='stack_estop_node',
            name='stack_estop_node',
            parameters=[{
                'laser_yaw_in_base_rad': ParameterValue(
                    LaunchConfiguration('laser_yaw_in_base_rad'), value_type=float),
                'dynamic_enabled': ParameterValue(
                    LaunchConfiguration('dynamic_enabled'), value_type=bool),
                'dynamic_stop_distance_m': ParameterValue(
                    LaunchConfiguration('dynamic_stop_distance_m'), value_type=float),
                'dynamic_tracking_max_distance_m': ParameterValue(
                    LaunchConfiguration('dynamic_tracking_max_distance_m'), value_type=float),
            }],
            output='screen',
        ),

        Node(
            package='stack_gps',
            executable='stack_gps_node',
            name='stack_gps_node',
            parameters=[{
                'waypoint_csv': LaunchConfiguration('waypoint_csv'),
                'rtcm_host': LaunchConfiguration('rtcm_host'),
                'error_log_csv': LaunchConfiguration('gps_error_log_csv'),
            }],
            output='screen',
        ),

        Node(
            package='stack_lane',
            executable='stack_lane_node',
            name='stack_lane_node',
            parameters=[{
                'homography_path': LaunchConfiguration('homography_path'),
                'camera_mxid': LaunchConfiguration('camera_mxid'),
                'device': LaunchConfiguration('lane_device'),
                'ref_point0_lookahead_m': ParameterValue(
                    LaunchConfiguration('ref_point0_lookahead_m'), value_type=float),
                'ref_point0_extrap_mode': LaunchConfiguration('ref_point0_extrap_mode'),
                'coeff_smoothing_alpha': ParameterValue(
                    LaunchConfiguration('coeff_smoothing_alpha'), value_type=float),
            }],
            output='screen',
        ),

        Node(
            package='adas_mgm',
            executable='mgm_node',
            name='mgm_node',
            parameters=[mgm_params],   # 기존 REAL_VEHICLE launch의 params 누락 수정
            output='screen',
        ),

        Node(
            package='bridge_dspace',
            executable='can_bridge_node',
            name='can_bridge_node_REAL_VEHICLE',
            parameters=[{
                'can_interface': LaunchConfiguration('can_interface'),
            }],
            output='screen',
        ),
    ])
