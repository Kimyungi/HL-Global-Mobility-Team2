"""Preparation launch structure; never starts GPS, cameras, CAN or ROS processes."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from stack_gps import persistent_service

ROOT = Path(__file__).resolve().parents[1]


def module():
    path = ROOT / 'src/adas_mgm/launch/REAL_VEHICLE_integration_v2_drive.launch.py'
    spec = importlib.util.spec_from_file_location('prepare_drive', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def context(mod, tmp_path):
    ctx = LaunchContext()
    for item in mod.generate_launch_description().entities:
        if isinstance(item, DeclareLaunchArgument):
            item.execute(ctx)
    ctx.launch_configurations['run_log_dir'] = str(tmp_path / 'run')
    ctx.launch_configurations['rviz'] = 'false'
    return ctx


def test_show_args_and_invalid_token_do_not_start_receiver(monkeypatch, tmp_path):
    monkeypatch.setattr(persistent_service, 'ensure_running', lambda *a, **kw: pytest.fail('hardware touched'))
    mod = module()
    ctx = context(mod, tmp_path)
    with pytest.raises(RuntimeError, match='token required'):
        mod.start_stack(ctx)
    assert not (tmp_path / 'run').exists()


def test_drive_reuses_persistent_link_and_does_not_own_rtcm_child(monkeypatch, tmp_path):
    mod = module()
    monkeypatch.setenv('FMA_V2_WORKSPACE', str(ROOT))
    # Build a minimal isolated install share for the launch import; only return actions.
    share = ROOT / 'install_v2/adas_mgm/share/adas_mgm'
    monkeypatch.setattr(mod, 'get_package_share_directory', lambda _: str(share))
    monkeypatch.setattr(mod, 'check_lidar_devices', lambda: None)
    calls = []
    monkeypatch.setattr(persistent_service, 'ensure_running', lambda cfg, script: calls.append((cfg, script)))
    ctx = context(mod, tmp_path)
    ctx.launch_configurations['REAL_VEHICLE_CONFIRM'] = 'I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX'
    actions = mod.start_stack(ctx)
    assert len(calls) == 1 and calls[0][0]['start_relay'] is True
    assert ctx.launch_configurations['gps_link_mode'] == 'persistent'
    assert not any('rtcm_server.py' in str(getattr(a, 'cmd', '')) for a in actions if isinstance(a, ExecuteProcess))


def test_stop_command_cannot_shut_down_gps_and_prepare_is_explicit():
    script = (ROOT / 'scripts/v2').read_text()
    stop = script.split('  stop)', 1)[1].split(';;', 1)[0]
    assert '/operator/stop' in stop and 'persistent_service' not in stop
    assert 'gps-off) V2_GPS_ACTION=stop' in script
    assert 'gps-start) V2_GPS_ACTION=start' in script


def test_prepare_runs_full_wait_go_launcher():
    script = (ROOT / 'scripts/v2').read_text()
    assert '  prepare|drive)' in script
    assert 'REAL_VEHICLE_integration_v2_drive.launch.py' in script


def test_missing_lidar_aborts_before_hardware(monkeypatch,tmp_path):
    mod=module()
    monkeypatch.setenv('FMA_V2_WORKSPACE',str(ROOT))
    monkeypatch.setattr(mod,'get_package_share_directory',lambda _: str(ROOT/'install_v2/adas_mgm/share/adas_mgm'))
    monkeypatch.setattr(mod.os,'access',lambda *a:False)
    monkeypatch.setattr(persistent_service,'ensure_running',lambda *a,**kw:pytest.fail('receiver touched'))
    ctx=context(mod,tmp_path)
    ctx.launch_configurations['REAL_VEHICLE_CONFIRM']='I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX'
    with pytest.raises(RuntimeError,match='라이다 장치'):
        mod.start_stack(ctx)
    assert not (tmp_path/'run').exists()






def test_runbook_prepare_selects_waypoint_provider_without_legacy_backend(monkeypatch, tmp_path):
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters
    from stack_avoid import compute_backend
    mod = module()
    ctx = context(mod, tmp_path)
    ctx.launch_configurations.update(
        REAL_VEHICLE_CONFIRM='I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX',
        start_waypoint='04', end_waypoint='07', parking_enabled='true',
        parking_zone_entry_active='true', avoidance_enabled='true', avoid_zone_only='true',
        zone_enter_confirm_samples='5',
        zone_exit_confirm_samples='5', traffic_enabled='true', v_base='2.0')
    monkeypatch.setenv('FMA_V2_WORKSPACE', str(ROOT))
    monkeypatch.setattr(mod, 'check_lidar_devices', lambda: None)
    monkeypatch.setattr(persistent_service, 'ensure_running', lambda *a: None)
    monkeypatch.setattr(compute_backend, 'compute_functions',
                        lambda *a: pytest.fail('waypoint provider loaded legacy cubic backend'))
    actions = mod.start_stack(ctx)
    for action in actions:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(ctx)
    ctx.launch_configurations['route_sequence_enabled_resolved'] = 'true'
    nodes = {}
    for action in actions:
        if isinstance(action, Node):
            nodes.setdefault(action.node_package, action)  # stack_avoid also owns can_zero.
    assert 'stack_avoid_v2' not in nodes
    assert nodes['stack_avoid'].condition.evaluate(ctx)
    waypoint = evaluate_parameters(ctx, nodes['stack_avoid']._Node__parameters)[1]
    assert waypoint['waypoint_csv'].endswith('waypoints_halla_20260916_path_04.csv')
    assert waypoint['route_origin_csv'] == waypoint['waypoint_csv']
    assert waypoint['target_speed_mps'] == 1.0
    mgm = evaluate_parameters(ctx, nodes['adas_mgm']._Node__parameters)[1]
    assert mgm['avoid_v2_enabled'] is False and mgm['wait_go'] is True
    assert mgm['avoid_zone_only'] is True and mgm['avoidance_enabled'] is True
    assert mgm['zone_enter_confirm_samples'] == 5
    assert mgm['v_base'] == 2.0 and mgm['v_avoid'] == 1.0
    assert (tmp_path / 'run' / 'avoid_planner_mode.txt').read_text().strip() == 'Waypoint_Avoid_PR103'
    assert (tmp_path / 'run' / 'avoid_compute_backend.txt').read_text().strip() == 'stack_avoid.waypoint_planner'






@pytest.mark.parametrize('retired,value', [
    ('waypoint_avoid','false'), ('avoid_v2_enabled','true'),
    ('avoid_planner_mode','fixed_goals'), ('avoid_planner_mode','main_gap'),
    ('avoid_planner_mode','main_gap_path'), ('avoid_compute_backend','native'),
    ('avoid_compute_backend','python')])
def test_retired_selection_fails_before_hardware(monkeypatch,tmp_path,retired,value):
    mod=module();ctx=context(mod,tmp_path)
    ctx.launch_configurations['REAL_VEHICLE_CONFIRM']='I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX'
    ctx.launch_configurations[retired]=value
    monkeypatch.setattr(mod,'check_lidar_devices',lambda:pytest.fail('hardware touched'))
    monkeypatch.setattr(persistent_service,'ensure_running',lambda *a:pytest.fail('GPS touched'))
    with pytest.raises(RuntimeError,match='Legacy avoidance'):
        mod.start_stack(ctx)
