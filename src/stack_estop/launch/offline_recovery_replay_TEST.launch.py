"""OFFLINE TEST ONLY: current recovery/avoidance chain without hardware."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    avoid_params = os.path.join(
        get_package_share_directory('stack_avoid'), 'config', 'params.yaml')
    return LaunchDescription([
        LogInfo(msg=[
            'OFFLINE RECOVERY TEST ONLY: no YDLIDAR driver, CAN bridge, '
            'dSPACE, or vehicle actuation']),
        Node(
            package='stack_avoid',
            executable='stack_avoid_node',
            name='stack_avoid_node',
            parameters=[avoid_params],
            output='screen'),
        Node(
            package='stack_estop',
            executable='stack_estop_node',
            name='stack_estop_node',
            parameters=[{
                'laser_yaw_in_base_rad': 1.57079632679,
                'dynamic_enabled': True,
                'dynamic_stop_distance_m': 1.20,
                'dynamic_tracking_max_distance_m': 3.00,
            }],
            output='screen'),
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
            output='screen'),
        Node(
            package='adas_mgm',
            executable='mgm_node',
            name='mgm_node',
            remappings=[('/adas/target_ref', '/adas/target_ref_mgm')],
            output='screen'),
        Node(
            package='stack_estop',
            executable='reverse_recovery_node',
            name='reverse_recovery_node',
            parameters=[{
                'reverse_wait_sec': 10.0,
                'reverse_actuation_enabled': True,
                'reverse_confirm_token': (
                    'I_CONFIRM_REVERSE_RECOVERY_ACTUATION'),
                'reverse_speed_mps': -0.30,
                'max_abs_reverse_speed_mps': 0.30,
                'reverse_max_duration_sec': 0.0,
                'post_reverse_stop_hold_sec': 0.5,
                'rear_scan_topic': '/rear/scan',
                'rear_scan_timeout_sec': 0.25,
                'front_scan_timeout_sec': 0.25,
                'status_stale_timeout_sec': 0.50,
                'rear_lidar_x_m': -0.055,
                'rear_lidar_y_m': 0.0,
                'rear_lidar_z_m': 0.065,
                'rear_lidar_yaw_rad': -1.51354952733,
                'rear_roi_min_x_m': -0.80,
                'rear_roi_max_x_m': -0.15,
                'rear_roi_half_width_m': 0.30,
                'rear_cluster_min_points': 3,
            }],
            output='screen'),
    ])
