"""Wall-gap parking-space search — single-command bench test.

Consolidates what used to be three separate steps (parking_mapping_bench.launch.py
in one terminal, wall_gap_node.py run by hand in a second, and manually picking
the one RViz config — config/wall_gap.rviz — that actually subscribes to
/wall_gap/markers) into one launch. Brings up the real 4-LiDAR stack
(lidar_fusion_v2 nearest-wins merge) + the real CAN bridge so
stack_parking_node's ICP SLAM gets real /vehicle/vector feedback, plus
wall_gap_node (left-wall gap search + auto SLAM-map reset on every run, see
wall_gap_node.py's docstring) and the wall_gap RViz view.

Driving is manual (RC controller into dSPACE directly) — this launch does not
publish any drive command. The old square-confirmation stop has been removed;
/wall_gap/stop remains false until the planned rear-LiDAR final-stop logic is
implemented. Do not run tools/T_Parking.py unattended expecting this node to
stop it.

Examples:
  ros2 launch stack_parking wall_gap_test.launch.py
  ros2 launch stack_parking wall_gap_test.launch.py search_side:=right
  ros2 launch stack_parking wall_gap_test.launch.py wall_line_offset_m:=0.15
  ros2 launch stack_parking wall_gap_test.launch.py inside_straight_m:=2.0 parallel_straight_m:=3.0
  ros2 launch stack_parking wall_gap_test.launch.py start_multi_lidar:=false
  ros2 launch stack_parking wall_gap_test.launch.py start_can:=false
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
    start_can = LaunchConfiguration('start_can')
    can_interface = LaunchConfiguration('can_interface')
    search_side = LaunchConfiguration('search_side')
    wall_line_offset_m = LaunchConfiguration('wall_line_offset_m')
    inside_straight_m = LaunchConfiguration('inside_straight_m')
    parallel_straight_m = LaunchConfiguration('parallel_straight_m')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_multi_lidar', default_value='true',
            description=(
                'Start the four real LiDAR drivers + lidar_fusion_v2 '
                '(false if already running)')),
        DeclareLaunchArgument(
            'start_can', default_value='true',
            description=(
                'Start can_bridge_node against the real dSPACE link so '
                '/vehicle/vector carries real actuator v/str (false if already running)')),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument(
            'search_side', default_value='left',
            description='Which side to search for a gap: left | right | both'),
        DeclareLaunchArgument(
            'wall_line_offset_m', default_value='0.12',
            description=(
                'Half-width in metres of the fixed reference-wall offset band')),
        DeclareLaunchArgument(
            'inside_straight_m', default_value='2.0',
            description='Reference-path length from P0 into the parking bay'),
        DeclareLaunchArgument(
            'parallel_straight_m', default_value='3.0',
            description='Wall-parallel reference-path length before the arc'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Open the wall_gap RViz view'),

        # Real 4-LiDAR drivers (a1/a2/b1/b2 raw scans) — fusion algorithm choice
        # doesn't matter here, this launch just brings the sensors up.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                driver_share, 'launch', 'multi_lidar_drivers.launch.py'])),
            condition=IfCondition(start_multi_lidar)),
        # scoped=True is load-bearing: fusion_v2.launch.py also declares a
        # 'rviz' argument, and an unscoped launch_arguments override here
        # clobbers *this* launch's own 'rviz' LaunchConfiguration for
        # everything that runs after it (silently — no error, the rviz2 Node
        # below just never starts). GroupAction keeps the override local to
        # the included launch.
        GroupAction(
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    fusion_share, 'launch', 'fusion_v2.launch.py'])),
                launch_arguments={'rviz': 'false'}.items())],
            scoped=True,
            condition=IfCondition(start_multi_lidar)),
        Node(
            package='stack_parking',
            executable='scan_to_cloud',
            name='parking_scan_to_cloud',
            output='screen',
            parameters=[{
                'input_scan_topic': '/unified_lidar/scan',
                'output_cloud_topic': merged_cloud_topic,
            }],
        ),

        # Real CAN bridge: /vehicle/vector.v and .str are the actual
        # actuator_velocity / actuator_steering readback from dSPACE — driving
        # is manual (RC controller into dSPACE), this launch only reads back.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bridge_share, 'launch', 'bridge.launch.py'])),
            launch_arguments={'can_interface': can_interface}.items(),
            condition=IfCondition(start_can)),

        # Full stack_parking_node so the steering-integrated motion prior is
        # exercised with real feedback. No mission trigger from here —
        # wall_gap_node sends its own start/cancel on startup to reset the map
        # (see wall_gap_node.py's docstring) and runs its own independent
        # detection alongside space_detector.py's mission state machine.
        Node(
            package='stack_parking',
            executable='stack_parking_node',
            name='stack_parking_node',
            output='screen',
            parameters=[params, {
                'auto_trigger_gps_zone': False,
                'manual_test_publish_gps_gate': False,
                'merged_cloud_topic': merged_cloud_topic,
                # wall_gap_node seeds its fixed reference only after this
                # launch-specific fresh-map reset sequence has completed.
                'reset_map_on_mission_start': True,
            }],
        ),

        Node(
            package='stack_parking',
            executable='wall_gap_node',
            name='wall_gap_node',
            output='screen',
            parameters=[{
                'search_side': search_side,
                'wall_line_offset_m': ParameterValue(
                    wall_line_offset_m, value_type=float),
                'inside_straight_m': ParameterValue(
                    inside_straight_m, value_type=float),
                'parallel_straight_m': ParameterValue(
                    parallel_straight_m, value_type=float),
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_wall_gap',
            output='screen',
            arguments=['-d', PathJoinSubstitution([config_dir, 'wall_gap.rviz'])],
            condition=IfCondition(rviz),
        ),
    ])
