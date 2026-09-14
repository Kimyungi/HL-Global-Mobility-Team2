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
