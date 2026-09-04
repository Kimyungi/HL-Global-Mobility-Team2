"""Single-command real-stack test for the left-wall parallel-parking case.

With control enabled, switching the vehicle from joystick to auto lets this
node command the same 1.5m preview and 0.75m/s speeds as the T test. Existing
SLAM map data is retained. A clear 1.5m wall-parallel x 0.7m inward rectangle
defines P0. Once the vehicle passes P0, the node shifts the arc origin 0.5m
forward and 0.25m clockwise-normal, then creates the R=2.0m, 50deg+50deg
point-symmetric S path with 1.5m
lines at both ends. It then runs the full S forward, a single-arc
line-arc-line (entry straight parallel_entry_straight_m, default 2m) in
reverse, and its same-radius symmetric arc in forward
(straight shortened to parallel_opposite_straight_m,
default 1m). The last reverse and forward phases reuse the original symmetric
full S (both arcs stay at parallel_turn_radius_m) without moving it. Every
preview-at-end transition holds for one second. The logger flushes after the
final hold.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('stack_parking'), 'config', 'parking_params.yaml'])
    config_dir = PathJoinSubstitution([FindPackageShare('stack_parking'), 'config'])
    driver_share = FindPackageShare('multi_lidar_fusion')
    fusion_share = FindPackageShare('lidar_fusion_v2')
    bridge_share = FindPackageShare('bridge_dspace')
    merged_cloud_topic = '/parking/nearest_merged_cloud'

    start_multi_lidar = LaunchConfiguration('start_multi_lidar')
    start_estop = LaunchConfiguration('start_estop')
    start_can = LaunchConfiguration('start_can')
    can_interface = LaunchConfiguration('can_interface')
    enable_control = LaunchConfiguration('enable_control')
    search_speed_mps = LaunchConfiguration('search_speed_mps')
    forward_speed_mps = LaunchConfiguration('forward_speed_mps')
    reverse_speed_mps = LaunchConfiguration('reverse_speed_mps')
    preview_distance_m = LaunchConfiguration('preview_distance_m')
    direction_change_hold_s = LaunchConfiguration('direction_change_hold_s')
    rectangle_wall_length_m = LaunchConfiguration('rectangle_wall_length_m')
    rectangle_inward_depth_m = LaunchConfiguration('rectangle_inward_depth_m')
    parallel_turn_radius_m = LaunchConfiguration('parallel_turn_radius_m')
    parallel_end_straight_m = LaunchConfiguration('parallel_end_straight_m')
    parallel_arc_start_offset_m = LaunchConfiguration(
        'parallel_arc_start_offset_m')
    parallel_arc_clockwise_offset_m = LaunchConfiguration(
        'parallel_arc_clockwise_offset_m')
    parallel_entry_straight_m = LaunchConfiguration(
        'parallel_entry_straight_m')
    parallel_opposite_straight_m = LaunchConfiguration(
        'parallel_opposite_straight_m')
    rviz = LaunchConfiguration('rviz')
    logging = LaunchConfiguration('logging')

    return LaunchDescription([
        DeclareLaunchArgument('start_multi_lidar', default_value='true'),
        DeclareLaunchArgument(
            'start_estop', default_value='true',
            description=(
                'Start stack_estop on /unified_lidar/scan; set false only '
                'when /perception/estop already has an owner')),
        DeclareLaunchArgument(
            'start_can', default_value='false',
            description=(
                'Explicit opt-in for the real dSPACE CAN bridge; keep false '
                'when another bridge/MGM stack is running')),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('enable_control', default_value='false'),
        DeclareLaunchArgument('search_speed_mps', default_value='0.5'),
        DeclareLaunchArgument('forward_speed_mps', default_value='0.5'),
        DeclareLaunchArgument('reverse_speed_mps', default_value='0.5'),
        DeclareLaunchArgument('preview_distance_m', default_value='1.5'),
        DeclareLaunchArgument('direction_change_hold_s', default_value='1.0'),
        DeclareLaunchArgument('rectangle_wall_length_m', default_value='1.5'),
        DeclareLaunchArgument('rectangle_inward_depth_m', default_value='0.7'),
        DeclareLaunchArgument('parallel_turn_radius_m', default_value='2.0'),
        DeclareLaunchArgument('parallel_end_straight_m', default_value='1.5'),
        DeclareLaunchArgument(
            'parallel_arc_start_offset_m', default_value='0.5'),
        DeclareLaunchArgument(
            'parallel_arc_clockwise_offset_m', default_value='0.25'),
        DeclareLaunchArgument(
            'parallel_entry_straight_m', default_value='2.0',
            description='Straight length of the entry line-arc-line (backing in)'),
        DeclareLaunchArgument(
            'parallel_opposite_straight_m', default_value='1.0',
            description=(
                'Straight length of the forward nudge that follows the '
                'entry (OPPOSITE_ARC_FORWARD)')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'logging', default_value='true',
            description='Log until the final one-second stop completes'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                driver_share, 'launch', 'multi_lidar_drivers.launch.py'])),
            condition=IfCondition(start_multi_lidar)),
        GroupAction(
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    fusion_share, 'launch', 'fusion_v2.launch.py'])),
                launch_arguments={'rviz': 'false'}.items())],
            scoped=True,
            condition=IfCondition(start_multi_lidar)),
        Node(
            package='stack_estop',
            executable='stack_estop_node',
            name='parallel_parking_stack_estop_node',
            output='screen',
            remappings=[('/scan', '/unified_lidar/scan')],
            parameters=[{'laser_yaw_in_base_rad': 0.0}],
            condition=IfCondition(start_estop),
        ),
        Node(
            package='stack_parking',
            executable='scan_to_cloud',
            name='parallel_parking_scan_to_cloud',
            output='screen',
            parameters=[{
                'input_scan_topic': '/unified_lidar/scan',
                'output_cloud_topic': merged_cloud_topic,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bridge_share, 'launch', 'bridge.launch.py'])),
            launch_arguments={'can_interface': can_interface}.items(),
            condition=IfCondition(start_can)),
        Node(
            package='stack_parking',
            executable='stack_parking_node',
            name='stack_parking_node',
            output='screen',
            parameters=[params, {
                'auto_trigger_gps_zone': False,
                'manual_test_publish_gps_gate': False,
                'merged_cloud_topic': merged_cloud_topic,
                # The parallel test intentionally accepts pre-auto map data.
                'reset_map_on_mission_start': False,
            }],
        ),
        Node(
            package='stack_parking',
            executable='parallel_parking_node',
            name='parallel_parking_node',
            output='screen',
            parameters=[{
                'search_side': 'left',
                'enable_control': ParameterValue(
                    enable_control, value_type=bool),
                'search_speed_mps': ParameterValue(
                    search_speed_mps, value_type=float),
                'forward_speed_mps': ParameterValue(
                    forward_speed_mps, value_type=float),
                'reverse_speed_mps': ParameterValue(
                    reverse_speed_mps, value_type=float),
                'preview_distance_m': ParameterValue(
                    preview_distance_m, value_type=float),
                'direction_change_hold_s': ParameterValue(
                    direction_change_hold_s, value_type=float),
                'rectangle_wall_length_m': ParameterValue(
                    rectangle_wall_length_m, value_type=float),
                'rectangle_inward_depth_m': ParameterValue(
                    rectangle_inward_depth_m, value_type=float),
                'parallel_turn_radius_m': ParameterValue(
                    parallel_turn_radius_m, value_type=float),
                'parallel_end_straight_m': ParameterValue(
                    parallel_end_straight_m, value_type=float),
                'parallel_arc_start_offset_m': ParameterValue(
                    parallel_arc_start_offset_m, value_type=float),
                'parallel_arc_clockwise_offset_m': ParameterValue(
                    parallel_arc_clockwise_offset_m, value_type=float),
                'parallel_arc_angle_deg': 50.0,
                'parallel_entry_straight_m': ParameterValue(
                    parallel_entry_straight_m, value_type=float),
                'parallel_opposite_straight_m': ParameterValue(
                    parallel_opposite_straight_m, value_type=float),
            }],
        ),
        Node(
            package='stack_parking',
            executable='parallel_parking_logger',
            name='parallel_parking_logger',
            output='screen',
            condition=IfCondition(logging),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_parallel_parking',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                config_dir, 'parallel_parking.rviz'])],
            condition=IfCondition(rviz),
        ),
    ])
