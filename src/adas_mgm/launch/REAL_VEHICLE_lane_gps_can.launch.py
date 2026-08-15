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

로깅 — run마다 ~/FMA_ws/drive_logs/run_<시각>/ 에 모아 저장:
  rosbag/            MGM 입출력 전 토픽 + /scan + /rosout (record:=false로만 끔)
  mgm_snapshots.bin  매 10ms CoreSnapshot 덤프 — core_replay로 판단 재현 (§5.5, 항상)
  mgm_jitter.csv     10ms 루프 주기 실측 (§7 판정 근거, 항상)
  lateral.csv        GPS 횡오차 (DRIVE_GUIDE와 동일 포맷, 항상)

사용:
  ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py \
      REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
      waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/<코스>.csv
"""
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


CONFIRM_TOKEN = 'I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX'
DEFAULT_LANE_CAMERA_FPS = '10'
DEFAULT_LANE_USB_SPEED = 'high'

# run 단위 로그 디렉터리 — launch 파일은 실행마다 새로 파싱되므로 매 run 고유
LOG_DIR = os.path.expanduser(
    '~/FMA_ws/drive_logs/run_' + datetime.now().strftime('%m%d_%H%M%S'))

# 버그 판단에 필요한 전 토픽 — MGM 입력(인지 6종)·출력·dSPACE 회신·라이다 원본·
# 노드 로그(/rosout — watchdog "estop 강제" 경고 등이 여기 남는다)·TF.
# 미발행 토픽(avoid/parking/traffic 미탑재 시)은 그냥 비어 있게 기록된다.
RECORD_TOPICS = [
    '/perception/lane_path', '/perception/gps_path', '/perception/gps_fix',
    '/perception/estop', '/perception/avoid', '/perception/parking',
    '/perception/traffic_stop', '/adas/target_ref', '/vehicle/vector',
    '/scan', '/rosout', '/tf', '/tf_static',
]

# 실측 호모그래피 (2026-08-11 캘리브레이션, LOO RMS 0.041m) — 소스 트리 절대경로로
# 지정해야 한다: 노드 기본 경로는 설치본 내부로 해석돼 파일을 못 찾는다
# (stack_lane CALIBRATION_GUIDE.md §6).
DEFAULT_HOMOGRAPHY = os.path.expanduser(
    '~/FMA_ws/src/stack_lane/config/homography.json')

DEFAULT_YDLIDAR_PARAMS = os.path.join(
    os.path.expanduser('~'), 'ydlidar_ws', 'src', 'ydlidar_ros2_driver',
    'params', 'Tmini-Plus-SH.yaml')


def build_lane_camera_parameters():
    """REAL_VEHICLE에서 사용하는 lane OAK 안전 프로필."""
    return {
        'camera_mxid': LaunchConfiguration('camera_mxid'),
        'camera_fps': ParameterValue(
            LaunchConfiguration('camera_fps'),
            value_type=int,
        ),
        'usb_speed': LaunchConfiguration('usb_speed'),
    }


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
    os.makedirs(LOG_DIR, exist_ok=True)
    print(f'[record] 로그 디렉터리: {LOG_DIR}')
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

        # ── GPS 전용 모드: LANE 전이 차단 (히스테리시스 임계를 2.0으로 — confidence는
        # 최대 1.0이라 절대 도달 불가 → 항상 WAYPOINT). 야간 등 차선 오검출이 위험한
        # 조건에서 사용 (2026-08-12: 야간 오검출 conf 0.71로 LANE 전이 → 벽 방향 조향).
        # stack_lane은 그대로 떠서 데이터는 기록됨 — 오검출 사후 분석용.
        DeclareLaunchArgument('gps_only', default_value='false'),

        # ── 로깅 (record:=false 는 rosbag만 끔 — CSV·스냅샷 덤프는 가벼워서 항상)
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('gps_error_log_csv',
                              default_value=os.path.join(LOG_DIR, 'lateral.csv')),

        # ── stack_lane
        DeclareLaunchArgument('homography_path', default_value=DEFAULT_HOMOGRAPHY),
        # 가중치도 소스 트리 절대경로 필수 (기본값은 설치본 내부로 해석 — 파일 없음).
        # yolopv2.pt는 gitignore 대상(156MB) — 새 PC엔 공식 릴리즈에서 수동 다운로드.
        DeclareLaunchArgument('lane_weights', default_value=os.path.expanduser(
            '~/FMA_ws/src/stack_lane/models/yolopv2.pt')),
        DeclareLaunchArgument('camera_mxid', default_value='14442C105157D3D200',
                              description='차선용 OAK-D MxID (2026-08-11 실측)'),
        DeclareLaunchArgument(
            'camera_fps',
            default_value=DEFAULT_LANE_CAMERA_FPS,
            description='차량 lane OAK 입력 FPS; USB2 기본은 10',
        ),
        DeclareLaunchArgument(
            'usb_speed',
            default_value=DEFAULT_LANE_USB_SPEED,
            description='lane OAK USB 상한: high=USB2, super=USB3',
        ),
        # 이 PC(산업용)는 NVIDIA 없음 — 인텔 Arc iGPU를 XPU 백엔드로 사용
        # (fp16 172ms/frame ≈ 5.8Hz, CPU 390ms 대비 2.3배 — 2026-08-11 실측).
        # XPU 초기화 실패 시(드라이버 문제 등) lane_device:=cpu 로 폴백.
        DeclareLaunchArgument('lane_device', default_value='xpu',
                              description="YOLOPv2 추론 장치: 'xpu'(인텔 GPU)/'cpu'/cuda 인덱스"),
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
                'weights': LaunchConfiguration('lane_weights'),
                **build_lane_camera_parameters(),
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
            parameters=[mgm_params, {   # 기존 REAL_VEHICLE launch의 params 누락 수정
                # run별 진단 산출물 — back-to-back 재현(§5.5)과 지터 판정(§7)
                'snapshot_dump_path': os.path.join(LOG_DIR, 'mgm_snapshots.bin'),
                'jitter_csv_path': os.path.join(LOG_DIR, 'mgm_jitter.csv'),
                # 출발 인가 게이트 — launch 직후 정지 대기, `ros2 run adas_mgm go`
                # (RTK FIXED 등 점검 통과 시)로 출발 (2026-08-11)
                'wait_go': True,
                # gps_only 시 LANE 전이 불가 임계로 상향 (위 gps_only 인자 참조)
                'lane_conf_exit': ParameterValue(PythonExpression(
                    ["2.0 if '", LaunchConfiguration('gps_only'), "' == 'true' else 0.4"]),
                    value_type=float),
                'lane_conf_return': ParameterValue(PythonExpression(
                    ["2.0 if '", LaunchConfiguration('gps_only'), "' == 'true' else 0.6"]),
                    value_type=float),
            }],
            output='screen',
        ),

        # ── rosbag — 버그 사후 분석·재생용. 토픽 명시 목록(RECORD_TOPICS)만 기록
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('record')),
            cmd=['ros2', 'bag', 'record', '-o', os.path.join(LOG_DIR, 'rosbag')]
                + RECORD_TOPICS,
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
