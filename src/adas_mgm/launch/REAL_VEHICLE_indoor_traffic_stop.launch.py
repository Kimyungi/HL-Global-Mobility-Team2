"""Indoor straight-line test for red-light + stop-line autonomous stopping."""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue

from stack_avoid.launch_parts import can_bridge_with_zero_guard


CONFIRM_TOKEN = 'I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX'
LOG_DIR = os.path.expanduser(
    '~/FMA_ws/drive_logs/indoor_traffic_' + datetime.now().strftime('%m%d_%H%M%S'))
RECORD_TOPICS = [
    '/perception/lane_path', '/perception/traffic_stop', '/perception/estop',
    '/perception/estop/status', '/adas/target_ref', '/vehicle/vector',
    '/scan', '/rosout',
]


def validate(context):
    if LaunchConfiguration('REAL_VEHICLE_CONFIRM').perform(context) != CONFIRM_TOKEN:
        raise RuntimeError(
            'REAL VEHICLE launch refused. Set REAL_VEHICLE_CONFIRM:=' + CONFIRM_TOKEN)
    on = float(LaunchConfiguration('estop_on_distance_m').perform(context))
    off = float(LaunchConfiguration('estop_off_distance_m').perform(context))
    speed = float(LaunchConfiguration('v_base').perform(context))
    if not 0.0 < on < off:
        raise RuntimeError('Require 0 < estop_on_distance_m < estop_off_distance_m')
    if not 0.0 < speed <= 1.0:
        raise RuntimeError('Indoor test requires 0 < v_base <= 1.0 m/s')
    lidar_params = LaunchConfiguration('ydlidar_params').perform(context)
    if not os.path.isfile(lidar_params):
        raise RuntimeError(f'LiDAR parameter file not found: {lidar_params}')
    os.makedirs(LOG_DIR, exist_ok=True)
    print('[indoor traffic test] GPS/lane camera/avoid disabled')
    print(f'[indoor traffic test] speed={speed:.6f}m/s, LiDAR E-stop={on:.2f}m')
    print(f'[indoor traffic test] logs={LOG_DIR}')
    return []


def generate_launch_description():
    mgm_params = os.path.join(
        get_package_share_directory('adas_mgm'), 'config', 'params.yaml')
    lidar_params = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'params', 'Tmini-Plus-SH.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('REAL_VEHICLE_CONFIRM', default_value='NOT_CONFIRMED'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB_LIDAR'),
        DeclareLaunchArgument('ydlidar_params', default_value=lidar_params),
        DeclareLaunchArgument('traffic_mxid', default_value='14442C10B167CFD200'),
        DeclareLaunchArgument('usb_speed', default_value='high'),
        DeclareLaunchArgument('v_base', default_value='1.0'),
        DeclareLaunchArgument('estop_on_distance_m', default_value='1.0'),
        DeclareLaunchArgument('estop_off_distance_m', default_value='1.15'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.20'),
        DeclareLaunchArgument('tracking_confidence_threshold', default_value='0.10'),
        OpaqueFunction(function=validate),

        LifecycleNode(
            package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node', namespace='/', output='screen',
            emulate_tty=True, respawn=True, respawn_delay=2.0,
            parameters=[LaunchConfiguration('ydlidar_params'), {
                'port': LaunchConfiguration('lidar_port'),
                'baudrate': 230400, 'lidar_type': 1, 'intensity_bit': 8,
                'reversion': False, 'inverted': False,
            }]),

        Node(
            package='stack_estop', executable='stack_estop_node',
            name='stack_estop_node', output='screen',
            parameters=[{
                'laser_yaw_in_base_rad': 1.57079632679,
                'estop_on_distance_m': ParameterValue(
                    LaunchConfiguration('estop_on_distance_m'), value_type=float),
                'estop_off_distance_m': ParameterValue(
                    LaunchConfiguration('estop_off_distance_m'), value_type=float),
                'dynamic_enabled': False,
            }],
            on_exit=Shutdown(reason='stack_estop stopped')),

        Node(
            package='adas_mgm', executable='straight_lane_publisher.py',
            name='straight_lane_publisher', output='screen',
            parameters=[{'lookahead_m': 2.0, 'publish_hz': 20.0}],
            on_exit=Shutdown(reason='straight path source stopped')),

        Node(
            package='stack_traffic', executable='stack_traffic_node',
            name='stack_traffic_node', output='screen',
            parameters=[{
                'camera_backend': 'oak',
                'oak_mxid': LaunchConfiguration('traffic_mxid'),
                'oak_usb_speed': LaunchConfiguration('usb_speed'),
                'oak_fps': 10.0, 'oak_width': 640, 'oak_height': 360,
                'confidence_threshold': ParameterValue(
                    LaunchConfiguration('confidence_threshold'), value_type=float),
                'tracking_confidence_threshold': ParameterValue(
                    LaunchConfiguration('tracking_confidence_threshold'), value_type=float),
                # depth는 정지 조건이 아닌 진단값이다. 단일 정지 시험도 RGB-only로
                # 두어 USB·CPU 부하와 depth 불량의 영향을 제거한다.
                'oak_depth_enabled': False,
                # CPU 직렬 추론이 traffic watchdog(0.5s)을 넘지 않도록 기존
                # 저해상도·간격 실행 경로를 사용한다. 적색 확정 뒤에는 아래
                # resume 설정에 따라 신호등 YOLO가 멈추고 정지선에 집중한다.
                'yolo_image_size': 320,
                'yolo_inference_interval': 2,
                'red_phase_yolo_inference_interval': 3,
                'stopline_detection_enabled': True,
                'stopline_yolo_confidence_threshold': 0.10,
                'stopline_yolo_image_size': 320,
                'stopline_stop_y_ratio': 0.0,
                'resume_on_green': False,
                'resume_on_red_clear': False,
                'show_debug': True,
            }],
            on_exit=Shutdown(reason='stack_traffic stopped')),

        Node(
            package='adas_mgm', executable='mgm_node', name='mgm_node', output='screen',
            parameters=[mgm_params, {
                'v_base': ParameterValue(LaunchConfiguration('v_base'), value_type=float),
                'wait_go': True,
                'escape_after_cycles': 0,
                'snapshot_dump_path': os.path.join(LOG_DIR, 'mgm_snapshots.bin'),
                'jitter_csv_path': os.path.join(LOG_DIR, 'mgm_jitter.csv'),
                'transition_csv_path': os.path.join(LOG_DIR, 'transitions.csv'),
            }],
            on_exit=Shutdown(reason='mgm stopped')),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', os.path.join(LOG_DIR, 'rosbag')]
                + RECORD_TOPICS,
            output='screen'),

        *can_bridge_with_zero_guard(
            can_interface=LaunchConfiguration('can_interface'),
            vehicle_csv_path=os.path.join(LOG_DIR, 'vehicle_vector.csv')),
    ])
