"""Resolve no-parking launch actions with hardware/service operations mocked."""
import importlib.util
from pathlib import Path

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT=Path(__file__).resolve().parents[1]


def test_no_parking_launch_uses_snapshot_and_keeps_latest_conditions(tmp_path,monkeypatch):
    path=ROOT/'src/adas_mgm/launch/REAL_VEHICLE_integration_v2_drive.launch.py'
    spec=importlib.util.spec_from_file_location('no_parking_launch_test',path)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    context=LaunchContext()
    context.launch_configurations.update(
        REAL_VEHICLE_CONFIRM='I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX',
        course='yongin_no_parking',start_waypoint='01',end_waypoint='06',
        v_base='2.0',rviz='false',run_log_dir=str(tmp_path/'run'))
    for action in mod.generate_launch_description().entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    monkeypatch.setenv('FMA_V2_WORKSPACE',str(ROOT))
    monkeypatch.setattr(mod,'check_lidar_devices',lambda:None)
    import stack_gps.persistent_service as persistent
    monkeypatch.setattr(persistent,'configuration',lambda *a,**k:{})
    monkeypatch.setattr(persistent,'ensure_running',lambda *a,**k:None)
    actions=mod.start_stack(context)
    for action in actions:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    for action in actions:
        if isinstance(action,OpaqueFunction):
            for setting in action.execute(context) or []:
                setting.execute(context)
    assert context.launch_configurations['t_reference_enabled']=='false'
    assert context.launch_configurations['parking_course_catalog']==''
    manifest=yaml.safe_load((tmp_path/'run/route_selected.yaml').read_text())
    assert [r['id'] for r in manifest['routes']]==['01','03','04','05','06','07']
    assert all('no_parking_route' in r['file'] for r in manifest['routes'])
    assert manifest['exit_branches']==dict(source='05',left='06',right='07')
    nodes=[n for n in actions if isinstance(n,Node)]
    mgm=next(n for n in nodes if n.node_package=='adas_mgm' and n.node_executable=='mgm_node')
    params=evaluate_parameters(context,mgm._Node__parameters)[1]
    assert params['estop_station_zone_id']==-1
    assert params['revised_v2_enabled'] and not params['lidar_estop_enabled']
    assert not any(n.node_executable=='estop_recovery_node.py' for n in nodes)
    lane=next(n for n in nodes if n.node_package=='stack_lane')
    assert evaluate_parameters(context,lane._Node__parameters)[0]['camera_only']
    assert context.launch_configurations['traffic_enabled']=='true'
