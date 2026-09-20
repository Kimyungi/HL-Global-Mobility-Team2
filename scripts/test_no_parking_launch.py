"""Resolve Yongin parking/no-parking launch actions with hardware mocked."""
import importlib.util
from pathlib import Path

import yaml
import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('course,explicit_course,start',
    [('yongin_no_parking',True,'01')] +
    [('yongin_0920',explicit,start) for explicit in (False,True) for start in ('01','02','03','04','05','06','07')])
def test_yongin_launch_uses_snapshot_and_keeps_latest_conditions(tmp_path,monkeypatch,course,explicit_course,start):
    path=ROOT/'src/adas_mgm/launch/REAL_VEHICLE_integration_v2_drive.launch.py'
    spec=importlib.util.spec_from_file_location('no_parking_launch_test',path)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    context=LaunchContext()
    context.launch_configurations.update(
        REAL_VEHICLE_CONFIRM='I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX',
        start_waypoint=start,end_waypoint=start if start in ('06','07') else '06',
        v_base='2.0',rviz='false',run_log_dir=str(tmp_path/'run'))
    if explicit_course:
        context.launch_configurations['course']=course
    for action in mod.generate_launch_description().entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    assert context.launch_configurations['course']==course
    assert context.launch_configurations['traffic_image_brightness_scale']=='0.7'
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
    assert context.launch_configurations['t_reference_enabled']==('false' if course=='yongin_no_parking' else 'true')
    if course=='yongin_no_parking':
        assert context.launch_configurations['parking_course_catalog']==''
    manifest=yaml.safe_load((tmp_path/'run/route_selected.yaml').read_text())
    expected=([start,'03','04','05','06','07'] if start in ('01','02') else
              ['03','04','05','06','07'][['03','04','05','06','07'].index(start):])
    if start in ('06','07'):expected=[start]
    assert [r['id'] for r in manifest['routes']]==expected
    directory='no_parking_route' if course=='yongin_no_parking' else 'yongin_0920_route'
    assert all(directory in r['file'] for r in manifest['routes'])
    if '05' in expected:
        assert manifest['exit_branches']==dict(source='05',left='06',right='07')
    else:
        assert 'exit_branches' not in manifest
    if course=='yongin_0920':
        from stack_parking.parking_courses import load_catalog
        courses=load_catalog(context.launch_configurations['parking_course_catalog'],manifest['routes'][0]['file'])
        for mode,route_id in ((1,'03'),(2,'04')):
            assert courses[mode].route_csv == tmp_path/'run/yongin_0920_route'/f'yongin_0920_{route_id}.csv'
            assert len(courses[mode].course[0])==2
        assert courses[1].exits is not None
    nodes=[n for n in actions if isinstance(n,Node)]
    mgm=next(n for n in nodes if n.node_package=='adas_mgm' and n.node_executable=='mgm_node')
    params=evaluate_parameters(context,mgm._Node__parameters)[1]
    assert params['stop_zone_hold_cycles']==500  # 5 seconds at 10 ms.
    assert params['estop_station_zone_id']==-1
    assert params['revised_v2_enabled'] and not params['lidar_estop_enabled']
    assert not any(n.node_executable=='estop_recovery_node.py' for n in nodes)
    traffic=next(n for n in nodes if n.node_package=='stack_traffic')
    assert evaluate_parameters(context,traffic._Node__parameters)[0]['image_brightness_scale']==0.7
    exit_detector=next(n for n in nodes if n.node_package=='stack_exit_decision')
    exit_params=evaluate_parameters(context,exit_detector._Node__parameters)[0]
    assert exit_params['image_topic']=='/perception/lane_image_raw'
    assert 'image_brightness_scale' not in exit_params
    lane=next(n for n in nodes if n.node_package=='stack_lane')
    assert evaluate_parameters(context,lane._Node__parameters)[0]['camera_only']
    assert context.launch_configurations['traffic_enabled']=='true'
    gps=next(n for n in nodes if n.node_package=='stack_gps')
    assert evaluate_parameters(context,gps._Node__parameters)[0]['imu_steering_recovery_enabled']
