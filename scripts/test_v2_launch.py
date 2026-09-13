"""Inspect launch graphs and refusal paths without starting ROS processes."""
import importlib.util
import re
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
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
    assert values['zone_enter_confirm_samples'] == values['zone_exit_confirm_samples'] == '5'
    assert values['parking_zone_entry_active'] == 'true'
    assert values['parking_search_zone_only'] == 'false'
    assert values['avoidance_enabled'] == 'false'
    assert values['parking_search_timeout'] == values['max_parking_search_distance'] == '-1.0'
    before = set((ROOT / 'drive_logs').glob('*'))
    validation = next(item for item in description.entities if isinstance(item, OpaqueFunction))
    with pytest.raises(RuntimeError, match='REAL VEHICLE launch refused'):
        validation.execute(context)
    assert set((ROOT / 'drive_logs').glob('*')) == before


@pytest.mark.parametrize('entry', [
    'REAL_VEHICLE_integration_v2.launch.py',
    'REAL_VEHICLE_integration_v2_no_estop.launch.py',
])
@pytest.mark.parametrize('enabled', [None, 'true'])
def test_avoidance_off_default_and_explicit_enable_reach_mgm(monkeypatch, entry, enabled):
    from launch_ros.utilities import evaluate_parameters
    monkeypatch.setenv('FMA_V2_WORKSPACE', str(ROOT))
    description = load(entry).generate_launch_description()
    context = LaunchContext()
    if enabled is not None:
        context.launch_configurations['avoidance_enabled'] = enabled
    for item in description.entities:
        if isinstance(item, DeclareLaunchArgument):
            item.execute(context)
    context.launch_configurations['route_sequence_enabled_resolved'] = 'false'
    mgm = next(item for item in description.entities
               if isinstance(item, Node) and item.node_package == 'adas_mgm')
    # Resolve only parameter values; do not execute nodes or validation actions.
    params = evaluate_parameters(context, mgm._Node__parameters)[1]
    assert params['avoidance_enabled'] is (enabled == 'true')
    assert params['parking_zone_entry_active'] is True
    assert params['lidar_estop_enabled'] is ('no_estop' not in entry)
    gps = next(item for item in description.entities
               if isinstance(item, Node) and item.node_package == 'stack_gps')
    context.launch_configurations['zones_file_resolved'] = ''
    gps_params = evaluate_parameters(context, gps._Node__parameters)[0]
    assert gps_params['n_points'] == 1
    assert not any(key.startswith('rejoin_') or key == 'ref_lookahead_m' for key in gps_params)


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


def test_integrated_parking_uses_v2_stream_profiles_for_all_four_lidars():
    import yaml
    path = ROOT / 'src/stack_parking/launch/parking.launch.py'
    spec = importlib.util.spec_from_file_location('parking_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations['start_multi_lidar'] = 'true'
    included = [item for item in module.generate_launch_description().entities
                if isinstance(item, IncludeLaunchDescription)]
    assert len(included) == 2
    driver = included[0]
    assert driver.condition.evaluate(context)
    description = driver.launch_description_source.get_launch_description(context)
    location = Path(driver.launch_description_source.location)
    assert location.name == 'drivers.launch.py'
    assert location.parent.parent.name == 'lidar_fusion_v2'
    nodes = []
    for item in description.entities:
        if isinstance(item, DeclareLaunchArgument):
            item.execute(context)
        elif isinstance(item, Node):
            nodes.append(item)
    assert len(nodes) == 4
    for node, sensor in zip(nodes, ('a1', 'a2', 'b1', 'b2')):
        node._perform_substitutions(context)  # materialize params, never execute a process
        files = node._Node__expanded_parameter_arguments
        assert len(files) == 1 and files[0][1]
        generated = Path(files[0][0])
        try:
            params = yaml.safe_load(generated.read_text())['/**']['ros__parameters']
        finally:
            generated.unlink()
        assert (params['sample_rate'], params['intensity_bit'], params['fixed_resolution']) == (4, 8, False)
        assert params['frame_id'] == f'lidar_{sensor}_link'
        assert ('/scan', f'/lidar/{sensor}/scan') in node.expanded_remapping_rules


def route_context(tmp_path):
    module = load('REAL_VEHICLE_lane_gps_can.launch.py')
    description = module.build_launch_description(log_dir=str(tmp_path / 'run'))
    context = LaunchContext()
    for item in description.entities:
        if isinstance(item, DeclareLaunchArgument):
            item.execute(context)
    context.launch_configurations.update({
        'REAL_VEHICLE_CONFIRM': module.CONFIRM_TOKEN,
        'route_sequence_file': str(ROOT / 'src/stack_gps/waypoints/halla_route_sequence.yaml'),
        'lane_enabled': 'false', 'parking_enabled': 'true',
        'route_start_id': '01', 'route_end_id': '07',
    })
    return module, context


@pytest.mark.parametrize('start', ['01','02'])
@pytest.mark.parametrize('end', ['06','07'])
def test_sequence_resolves_first_csv_and_enables_mgm_without_starting_nodes(tmp_path, start, end):
    module, context = route_context(tmp_path)
    context.launch_configurations.update(route_start_id=start, route_end_id=end)
    for setting in module.validate(context, str(tmp_path / 'run')):
        assert not isinstance(setting, Node)
        setting.execute(context)
    assert context.launch_configurations['waypoint_csv'].endswith(f'path_{start}.csv')
    assert context.launch_configurations['zones_file_resolved'].endswith(f'path_{start}.yaml')
    assert context.launch_configurations['route_sequence_enabled_resolved'] == 'true'


@pytest.mark.parametrize('key,value', [('waypoint_csv', '/wrong.csv'), ('zones_file', '/wrong.yaml')])
def test_sequence_rejects_conflicting_single_file_arguments(tmp_path, key, value):
    module, context = route_context(tmp_path)
    context.launch_configurations[key] = value
    with pytest.raises(RuntimeError):
        module.validate(context, str(tmp_path / 'run'))
    assert not (tmp_path / 'run').exists()


@pytest.mark.parametrize('key,value', [('route_start_id',''),('route_end_id',''),('route_start_id','03'),('route_end_id','05'),('route_sequence_file','')])
def test_sequence_rejects_missing_or_invalid_selection_before_nodes(tmp_path, key, value):
    module, context = route_context(tmp_path)
    context.launch_configurations[key] = value
    with pytest.raises((RuntimeError,ValueError)):
        module.validate(context, str(tmp_path / 'run'))
    assert not (tmp_path / 'run').exists()
