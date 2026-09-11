"""Inspect launch graphs and refusal paths without starting ROS processes."""
import importlib.util
import re
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'src/adas_mgm/launch'


def load(name):
    spec = importlib.util.spec_from_file_location(name, LAUNCH / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_bench_has_only_mgm_and_remaps_every_driving_topic():
    description = load('integration_v2_bench.launch.py').generate_launch_description()
    assert len(description.entities) == 1
    node = description.entities[0]
    assert isinstance(node, Node)
    context = LaunchContext()
    assert node.node_package == 'adas_mgm'
    # Resolve substitutions only; execute() is deliberately never called on a Node.
    node._perform_substitutions(context)
    code = (ROOT / 'src/adas_mgm/src/mgm_node.cpp').read_text()
    topics = set(re.findall(r'"(/(?:operator|parking|adas|perception|bridge|vehicle)/[^" ]+)"', code))
    remaps = dict(node.expanded_remapping_rules)
    assert topics <= remaps.keys()
    assert all(remaps[topic] == '/integration_v2' + topic for topic in topics)


def test_vehicle_defaults_use_v2_and_refuse_before_hardware(monkeypatch):
    module = load('REAL_VEHICLE_integration_v2.launch.py')
    monkeypatch.setenv('FMA_V2_WORKSPACE', str(ROOT))
    description = module.generate_launch_description()
    context = LaunchContext()
    for item in description.entities:
        if isinstance(item, DeclareLaunchArgument):
            item.execute(context)
    values = context.launch_configurations
    assert Path(values['lane_weights']).is_relative_to(ROOT)
    assert Path(values['homography_path']).is_relative_to(ROOT)
    assert Path(values['gps_error_log_csv']).is_relative_to(ROOT / 'drive_logs')
    assert values['zone_enter_confirm_samples'] == values['zone_exit_confirm_samples'] == '0'
    assert values['parking_search_timeout'] == values['max_parking_search_distance'] == '-1.0'
    before = set((ROOT / 'drive_logs').glob('*'))
    validation = next(item for item in description.entities if isinstance(item, OpaqueFunction))
    with pytest.raises(RuntimeError, match='REAL VEHICLE launch refused'):
        validation.execute(context)
    assert set((ROOT / 'drive_logs').glob('*')) == before


def test_vehicle_rejects_foreign_install(monkeypatch, tmp_path):
    module = load('REAL_VEHICLE_integration_v2.launch.py')
    monkeypatch.setenv('FMA_V2_WORKSPACE', str(ROOT))
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(tmp_path))
    with pytest.raises(RuntimeError, match='must come from this workspace'):
        module.generate_launch_description()


def test_vehicle_requires_workspace_entry(monkeypatch):
    monkeypatch.delenv('FMA_V2_WORKSPACE', raising=False)
    with pytest.raises(RuntimeError, match='scripts/v2 vehicle'):
        load('REAL_VEHICLE_integration_v2.launch.py').generate_launch_description()
