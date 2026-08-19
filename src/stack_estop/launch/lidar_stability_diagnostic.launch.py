"""TEST ONLY: run FRONT/REAR YDLIDAR and a scan-gap monitor; no actuation."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    default_params = os.path.join(
        os.path.expanduser('~'), 'ydlidar_ws', 'src',
        'ydlidar_ros2_driver', 'params', 'Tmini-Plus-SH.yaml')
    front_port = LaunchConfiguration('front_lidar_port')
    rear_port = LaunchConfiguration('rear_lidar_port')
    return LaunchDescription([
        DeclareLaunchArgument('ydlidar_params', default_value=default_params),
        DeclareLaunchArgument('enable_front', default_value='true'),
        DeclareLaunchArgument('enable_rear', default_value='true'),
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
        LogInfo(msg=[
            '[FRONT DIAGNOSTIC] port=', front_port,
            ', topic=/scan, frame=laser_frame, auto_reconnect=true'],
            condition=IfCondition(LaunchConfiguration('enable_front'))),
        LifecycleNode(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            namespace='/',
            parameters=[LaunchConfiguration('ydlidar_params'), {
                'port': front_port,
                'baudrate': 230400,
                'lidar_type': 1,
                'intensity_bit': 8,
                'auto_reconnect': True,
            }],
            output='screen', emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('enable_front'))),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_laser_static_tf',
            arguments=[
                '--x', '0.76', '--y', '0.0', '--z', '0.065',
                '--roll', '0.0', '--pitch', '0.0',
                '--yaw', '1.57079632679',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser_frame'],
            condition=IfCondition(LaunchConfiguration('enable_front'))),
        LogInfo(msg=[
            '[REAR DIAGNOSTIC] port=', rear_port,
            ', topic=/rear/scan, frame=rear_laser_frame, '
            'auto_reconnect=true'],
            condition=IfCondition(LaunchConfiguration('enable_rear'))),
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            namespace='rear',
            name='rear_ydlidar_ros2_driver_node',
            parameters=[{
                'port': rear_port,
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
            output='screen', emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('enable_rear'))),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rear_laser_static_tf',
            arguments=[
                '--x', '-0.055', '--y', '0.0', '--z', '0.065',
                '--roll', '0.0', '--pitch', '0.0',
                '--yaw', '-1.51354952733',
                '--frame-id', 'base_link',
                '--child-frame-id', 'rear_laser_frame'],
            condition=IfCondition(LaunchConfiguration('enable_rear'))),
        Node(
            package='stack_estop',
            executable='lidar_scan_gap_monitor',
            name='front_lidar_scan_gap_monitor',
            parameters=[{'topic': '/scan'}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_front'))),
        Node(
            package='stack_estop',
            executable='lidar_scan_gap_monitor',
            name='rear_lidar_scan_gap_monitor',
            parameters=[{'topic': '/rear/scan'}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_rear'))),
    ])
