"""REAL VEHICLE Stack E-Stop, MGM, and reverse-recovery launch.

This launch can send real CAN commands. It requires both the real-vehicle and
reverse-actuation confirmations. MGM is remapped away from the final
/adas/target_ref topic, so reverse_recovery_node is its only publisher.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


REAL_CONFIRM_TOKEN = 'I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX'


def validate_real_vehicle_confirmation(context):
    supplied = LaunchConfiguration('REAL_VEHICLE_CONFIRM').perform(context)
    if supplied != REAL_CONFIRM_TOKEN:
        raise RuntimeError(
            'REAL VEHICLE launch refused. Set REAL_VEHICLE_CONFIRM:='
            + REAL_CONFIRM_TOKEN)
    return []


def generate_launch_description():
    default_ydlidar_params = os.path.join(
        os.path.expanduser('~'), 'ydlidar_ws', 'src',
        'ydlidar_ros2_driver', 'params', 'Tmini-Plus-SH.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'REAL_VEHICLE_CONFIRM', default_value='NOT_CONFIRMED'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument(
            'ydlidar_params', default_value=default_ydlidar_params),
        DeclareLaunchArgument(
            'front_lidar_port',
            default_value=(
                '/dev/serial/by-path/'
                'pci-0000:00:14.0-usb-0:3.4:1.0-port0')),
        DeclareLaunchArgument(
            'rear_lidar_port',
            default_value=(
                '/dev/serial/by-path/'
                'pci-0000:00:14.0-usb-0:3.3:1.0-port0')),
        DeclareLaunchArgument(
            'laser_yaw_in_base_rad', default_value='1.57079632679'),
        DeclareLaunchArgument(
            'static_min_obstacle_extent_m', default_value='0.07',
            description=(
                'Static cluster minimum physical extent in metres; 0 disables '
                'small-object filtering.')),
        DeclareLaunchArgument('dynamic_enabled', default_value='true'),
        DeclareLaunchArgument(
            'dynamic_stop_distance_m', default_value='1.20'),
        DeclareLaunchArgument(
            'dynamic_tracking_max_distance_m', default_value='3.00'),
        DeclareLaunchArgument(
            'reverse_actuation_enabled', default_value='false'),
        DeclareLaunchArgument(
            'reverse_confirm_token',
            default_value='NOT_CONFIRMED',
            description=(
                'Set to I_CONFIRM_REVERSE_RECOVERY_ACTUATION to authorize '
                'negative reverse v_ref.')),
        DeclareLaunchArgument('reverse_wait_sec', default_value='10.0'),
        DeclareLaunchArgument('reverse_speed_mps', default_value='-0.30'),
        DeclareLaunchArgument(
            'max_abs_reverse_speed_mps', default_value='0.30'),
        DeclareLaunchArgument(
            'reverse_max_duration_sec', default_value='0.0'),
        DeclareLaunchArgument(
            'post_reverse_stop_hold_sec', default_value='0.5'),
        DeclareLaunchArgument(
            'status_stale_timeout_sec', default_value='0.50'),

        OpaqueFunction(function=validate_real_vehicle_confirmation),

        LogInfo(msg=[
            '[YDLIDAR FRONT CONFIG] port=',
            LaunchConfiguration('front_lidar_port'),
            ', topic=/scan, frame=laser_frame, auto_reconnect=true',
        ]),

        LifecycleNode(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            namespace='/',
            parameters=[LaunchConfiguration('ydlidar_params'), {
                'port': LaunchConfiguration('front_lidar_port'),
                'baudrate': 230400,
                'lidar_type': 1,
                'intensity_bit': 8,
                'auto_reconnect': True,
            }],
            output='screen',
            emulate_tty=True,
        ),
        # base_link -> laser_frame is published by stack_avoid_node from its
        # verified config. Do not add a second static TF publisher here.
        LogInfo(msg=[
            '[YDLIDAR REAR CONFIG] port=',
            LaunchConfiguration('rear_lidar_port'),
            ', topic=/rear/scan, frame=rear_laser_frame, '
            'auto_reconnect=true',
        ]),
        Node(
            package='stack_avoid',
            executable='stack_avoid_node',
            name='stack_avoid_node',
            parameters=[os.path.join(
                get_package_share_directory('stack_avoid'),
                'config', 'params.yaml')],
            output='screen',
        ),
        Node(
            package='stack_estop',
            executable='stack_estop_node',
            name='stack_estop_node',
            parameters=[{
                'laser_yaw_in_base_rad': ParameterValue(
                    LaunchConfiguration('laser_yaw_in_base_rad'),
                    value_type=float),
                'static_min_obstacle_extent_m': ParameterValue(
                    LaunchConfiguration('static_min_obstacle_extent_m'),
                    value_type=float),
                'dynamic_enabled': ParameterValue(
                    LaunchConfiguration('dynamic_enabled'), value_type=bool),
                'dynamic_stop_distance_m': ParameterValue(
                    LaunchConfiguration('dynamic_stop_distance_m'),
                    value_type=float),
                'dynamic_tracking_max_distance_m': ParameterValue(
                    LaunchConfiguration('dynamic_tracking_max_distance_m'),
                    value_type=float),
            }],
            output='screen',
        ),
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            namespace='rear',
            name='rear_ydlidar_ros2_driver_node',
            parameters=[{
                'port': LaunchConfiguration('rear_lidar_port'),
                'frame_id': 'rear_laser_frame',
                'baudrate': 230400,
                'lidar_type': 1,
                'device_type': 0,
                'intensity': True,
                'intensity_bit': 8,
                'isSingleChannel': False,
                'frequency': 10.0,
                'sample_rate': 4,
                'abnormal_check_count': 4,
                'fixed_resolution': True,
                'reversion': True,
                'inverted': True,
                'auto_reconnect': True,
                'support_motor_dtr': False,
                'angle_max': 180.0,
                'angle_min': -180.0,
                'range_max': 12.0,
                'range_min': 0.03,
                'invalid_range_is_inf': False,
            }],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='stack_estop',
            executable='reverse_recovery_node',
            name='reverse_recovery_node',
            parameters=[{
                'reverse_wait_sec': ParameterValue(
                    LaunchConfiguration('reverse_wait_sec'),
                    value_type=float),
                'reverse_actuation_enabled': ParameterValue(
                    LaunchConfiguration('reverse_actuation_enabled'),
                    value_type=bool),
                'reverse_confirm_token': LaunchConfiguration(
                    'reverse_confirm_token'),
                'reverse_speed_mps': ParameterValue(
                    LaunchConfiguration('reverse_speed_mps'),
                    value_type=float),
                'max_abs_reverse_speed_mps': ParameterValue(
                    LaunchConfiguration('max_abs_reverse_speed_mps'),
                    value_type=float),
                'reverse_max_duration_sec': ParameterValue(
                    LaunchConfiguration('reverse_max_duration_sec'),
                    value_type=float),
                'post_reverse_stop_hold_sec': ParameterValue(
                    LaunchConfiguration('post_reverse_stop_hold_sec'),
                    value_type=float),
                'rear_scan_topic': '/rear/scan',
                'rear_scan_timeout_sec': 0.25,
                'front_scan_timeout_sec': 0.25,
                'status_stale_timeout_sec': ParameterValue(
                    LaunchConfiguration('status_stale_timeout_sec'),
                    value_type=float),
                'rear_lidar_x_m': -0.055,
                'rear_lidar_y_m': 0.0,
                'rear_lidar_z_m': 0.065,
                'rear_lidar_yaw_rad': -1.51354952733,
                'rear_roi_min_x_m': -0.80,
                'rear_roi_max_x_m': -0.15,
                'rear_roi_half_width_m': 0.30,
                'rear_cluster_min_points': 3,
            }],
            output='screen',
        ),
        Node(
            package='adas_mgm',
            executable='mgm_node',
            name='mgm_node',
            remappings=[
                ('/adas/target_ref', '/adas/target_ref_mgm'),
            ],
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
