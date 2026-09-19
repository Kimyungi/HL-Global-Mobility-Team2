"""Resolve the production profile without executing sensors or CAN."""
import importlib.util
from pathlib import Path
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT=Path(__file__).resolve().parents[1]


def test_runbook_profile_excludes_legacy_estop_and_starts_traffic_supervisor():
    path=ROOT/'src/adas_mgm/launch/REAL_VEHICLE_lane_gps_can.launch.py'
    spec=importlib.util.spec_from_file_location('revised_base',path)
    mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    topics=['/lidar/a1/scan','/lidar/a2/scan','/lidar/b1/scan','/lidar/b2/scan']
    description=mod.build_launch_description(lidar_estop_enabled=False,
        revised_v2_enabled=True,required_lidar_topics=topics)
    context=LaunchContext()
    for item in description.entities:
        if isinstance(item,DeclareLaunchArgument): item.execute(context)
    context.launch_configurations.update(avoid_v2_enabled='false',waypoint_avoid='true',traffic_enabled='true',
        parking_enabled='true',route_sequence_enabled_resolved='true',zones_file_resolved='')
    nodes=[n for n in description.entities if isinstance(n,Node)]
    estop=next(n for n in nodes if n.node_package=='stack_estop')
    assert not estop.condition.evaluate(context)
    traffic=next(n for n in nodes if n.node_package=='stack_traffic')
    traffic._perform_substitutions(context)
    assert traffic.node_executable == 'traffic_zone_supervisor'
    lane=next(n for n in nodes if n.node_package=='stack_lane')
    assert evaluate_parameters(context,lane._Node__parameters)[0]['zone_gated'] is True
    assert evaluate_parameters(context,lane._Node__parameters)[0]['camera_only'] is True
    mgm=next(n for n in nodes if n.node_package=='adas_mgm')
    params=evaluate_parameters(context,mgm._Node__parameters)[1]
    assert list(params['required_lidar_topics'])==topics
    assert params['revised_v2_enabled'] and not params['lidar_estop_enabled']
    assert params['escape_after_cycles']==0 and params['traffic_stop_offset_m']==1.1
    assert len(params['estop_mount.front'])==8
    assert params['estop_station_zone_id']==0
    assert not any(isinstance(n,Node) and n.node_executable=='estop_recovery_node.py' for n in description.entities)
    gps=next(n for n in nodes if n.node_package=='stack_gps')
    assert evaluate_parameters(context,gps._Node__parameters)[0]['turn_zone_policy']
    assert evaluate_parameters(context,gps._Node__parameters)[0]['initial_heading_from_waypoint']


def test_waypoint_provider_is_exclusive_and_uses_selected_origin():
    path=ROOT/'src/adas_mgm/launch/REAL_VEHICLE_lane_gps_can.launch.py'
    spec=importlib.util.spec_from_file_location('waypoint_base',path)
    mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    description=mod.build_launch_description(lidar_estop_enabled=False,revised_v2_enabled=True,
        required_lidar_topics=['/lidar/a1/scan','/lidar/a2/scan','/lidar/b1/scan','/lidar/b2/scan'])
    context=LaunchContext()
    for item in description.entities:
        if isinstance(item,DeclareLaunchArgument): item.execute(context)
    origin=str(ROOT/'src/stack_gps/waypoints/halla_0919_path_01.csv')
    context.launch_configurations.update(waypoint_avoid='true',avoid_v2_enabled='false',
        avoid_waypoint_csv=origin,avoid_route_origin_csv=origin,route_sequence_enabled_resolved='true',zones_file_resolved='')
    providers=[n for n in description.entities if isinstance(n,Node) and n.node_package in ('stack_avoid','stack_avoid_v2') and n.node_executable != 'can_zero']
    active=[n for n in providers if n.condition.evaluate(context)]
    assert len(active)==1
    active[0]._perform_substitutions(context)
    assert active[0].node_executable=='waypoint_avoid_node'
    params=evaluate_parameters(context,active[0]._Node__parameters)[1]
    assert params['waypoint_csv']==params['route_origin_csv']==origin
    mgm=next(n for n in description.entities if isinstance(n,Node) and n.node_package=='adas_mgm')
    params=evaluate_parameters(context,mgm._Node__parameters)[1]
    assert not params['avoid_v2_enabled'] and params['avoid_unblended']


def test_drive_defaults_select_new_provider_and_keep_t_parking():
    path=ROOT/'src/adas_mgm/launch/REAL_VEHICLE_integration_v2_drive.launch.py'
    spec=importlib.util.spec_from_file_location('waypoint_drive',path)
    mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    context=LaunchContext()
    context.launch_configurations['start_waypoint']='03'
    context.launch_configurations['v_base']='2.0'
    for item in mod.generate_launch_description().entities:
        if isinstance(item,DeclareLaunchArgument): item.execute(context)
    assert context.launch_configurations['waypoint_avoid']=='true'
    assert context.launch_configurations['avoid_v2_enabled']=='false'
    assert context.launch_configurations['t_reference_enabled']=='true'
    assert context.launch_configurations['escape_after_cycles']=='0'
