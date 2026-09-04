"""4-LiDAR SLAM/mapping bench test — manual driving, real dSPACE actuator feedback.

The vehicle is driven by hand with a controller (not MGM). This launch brings
up the real 4-LiDAR drivers with the `lidar_fusion_v2` fusion (PR #70) and the
real CAN bridge so `stack_parking_node` builds its ICP motion prior from the
actual `/vehicle/vector` feedback (`VehicleVector.v`/`.str` — real actuator
velocity/steering), exactly the signal path used on the vehicle.

`lidar_fusion_v2` publishes two competing merges of the four sensors:
  - `/unified_lidar/cloud` — raw concatenation, overlaps kept as-is
  - `/unified_lidar/scan`  — one LaserScan, nearest return per angle bin
    wins on overlap (`points_to_virtual_scan`, `np.minimum.at`)
This bench feeds ICP the second one (nearest-wins on overlap) via a small
`scan_to_cloud` converter, since `stack_parking_node` only takes a
PointCloud2. No parking mission is triggered: pure SLAM/mapping runs
unconditionally in `stack_parking_node` on every scan, independent of the
mission state machine.

Opens two RViz2 windows:
  - parking_1_slam.rviz    live pose + current scan against the local map
  - parking_2_mapping.rviz accumulated map only

Examples:
  ros2 launch stack_parking parking_mapping_bench.launch.py
  ros2 launch stack_parking parking_mapping_bench.launch.py start_multi_lidar:=false
  ros2 launch stack_parking parking_mapping_bench.launch.py start_can:=false
  ros2 launch stack_parking parking_mapping_bench.launch.py can_interface:=can0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_multi_lidar', default_value='true',
            description=(
                'Start the four real LiDAR drivers + lidar_fusion_v2 '
                '(false if already running)')),
        DeclareLaunchArgument(
            'start_can', default_value='false',
            description=(
                'Explicit opt-in: start can_bridge_node against real dSPACE '
                'for actuator v/str feedback; keep false if already running')),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Open the SLAM and mapping RViz2 windows'),

        # Real 4-LiDAR drivers (a1/a2/b1/b2 raw scans) — fusion algorithm choice
        # doesn't matter here, this launch just brings the sensors up.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                driver_share, 'launch', 'multi_lidar_drivers.launch.py'])),
            condition=IfCondition(start_multi_lidar)),
        # lidar_fusion_v2 (PR #70) instead of multi_lidar_fusion: it publishes
        # both the raw concatenated cloud and the nearest-per-bin merged scan.
        # Its own RViz is off — the two windows below cover debugging.
        # scoped=True is load-bearing: fusion_v2.launch.py also declares a
        # 'rviz' argument, and an unscoped launch_arguments override here
        # clobbers *this* launch's own 'rviz' LaunchConfiguration for
        # everything that runs after it (silently — no error, the two
        # rviz2 Nodes below just never start). GroupAction keeps the
        # override local to the included launch.
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
        # actuator_velocity / actuator_steering readback from dSPACE, driven
        # here by hand with a controller rather than by MGM ref points.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bridge_share, 'launch', 'bridge.launch.py'])),
            launch_arguments={'can_interface': can_interface}.items(),
            condition=IfCondition(start_can)),

        # Full stack_parking_node (not slam_only) so the steering-integrated
        # motion prior (§ commit "enhance motion prior with steering
        # integration") is exercised with real feedback. No mission trigger
        # is sent — SLAM/mapping runs regardless of mission state.
        Node(
            package='stack_parking',
            executable='stack_parking_node',
            name='stack_parking_node',
            output='screen',
            parameters=[params, {
                'auto_trigger_gps_zone': False,
                'manual_test_publish_gps_gate': False,
                'merged_cloud_topic': merged_cloud_topic,
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_parking_slam',
            output='screen',
            arguments=['-d', PathJoinSubstitution([config_dir, 'parking_1_slam.rviz'])],
            condition=IfCondition(rviz),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_parking_mapping',
            output='screen',
            arguments=['-d', PathJoinSubstitution([config_dir, 'parking_2_mapping.rviz'])],
            condition=IfCondition(rviz),
        ),
    ])
