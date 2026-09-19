"""Resolve main launch parameters without executing hardware actions."""
import importlib.util
from pathlib import Path

import pytest
import yaml
from ament_index_python import packages
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[3]


def graph(monkeypatch, relative, overrides=None):
    original = packages.get_package_share_directory

    def share(package):
        source = ROOT / 'src' / package
        return str(source) if (source / 'package.xml').is_file() else original(package)

    monkeypatch.setattr(packages, 'get_package_share_directory', share)
    spec = importlib.util.spec_from_file_location('launch_under_test', ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(route_sequence_enabled_resolved='false', zones_file_resolved='')
    # Never execute Node, IncludeLaunchDescription, OpaqueFunction or CAN guards.
    return context, description.entities


def node_parameters(context, entities, package, required_parameter=None):
    for node in entities:
        if not isinstance(node, Node) or node.node_package != package:
            continue
        values = evaluate_parameters(context, node._Node__parameters)
        params = {key: value for item in values if isinstance(item, dict)
                  for key, value in item.items()}
        if required_parameter is None or required_parameter in params:
            return params
    raise AssertionError(f'No {package} node with parameter {required_parameter}')


MAIN = 'src/adas_mgm/launch/REAL_VEHICLE_lane_gps_can.launch.py'


def test_default_profile_keeps_main_speed_and_requires_go(monkeypatch):
    context, entities = graph(monkeypatch, MAIN)
    baseline = yaml.safe_load((ROOT / 'src/adas_mgm/config/params.yaml').read_text())
    params = node_parameters(context, entities, 'adas_mgm')
    for name in ('v_base', 'v_accel_zone', 'ttc_stop'):
        assert params[name] == baseline['mgm_node']['ros__parameters'][name]
    assert params['wait_go'] is True
    assert context.launch_configurations['lane_csv'] == 'false'
    assert params['zone_enter_confirm_samples'] == int(context.launch_configurations['zone_enter_confirm_samples'])
    assert params['parking_search_timeout'] == float(context.launch_configurations['parking_search_timeout'])


def test_trial_overrides_reach_speed_and_obstacle_consumers(monkeypatch):
    context, entities = graph(monkeypatch, MAIN, {
        'v_base': '2.0', 'v_accel_zone': '2.0', 'ttc_stop': '2.0',
        'avoid_target_speed_mps': '2.0', 'estop_corridor_max_x_m': '4.2',
        'dynamic_roi_max_x_m': '4.2', 'escape_after_cycles': '0',
    })
    mgm = node_parameters(context, entities, 'adas_mgm')
    assert (mgm['v_base'], mgm['v_accel_zone'], mgm['ttc_stop']) == (2.0, 2.0, 2.0)
    assert mgm['escape_after_cycles'] == 0
    assert mgm['wait_go'] is True
    assert node_parameters(context, entities, 'stack_avoid')['target_speed_mps'] == 2.0
    estop = node_parameters(context, entities, 'stack_estop')
    assert estop['corridor_max_x_m'] == estop['dynamic_roi_max_x_m'] == 4.2


@pytest.mark.parametrize('csv,debug,enabled', [
    ('false', 'false', False), ('true', 'false', True), ('false', 'true', True),
])
def test_lane_csv_does_not_require_debug_images(monkeypatch, csv, debug, enabled):
    context, entities = graph(monkeypatch, MAIN, {'lane_csv': csv, 'lane_debug': debug})
    lane = node_parameters(context, entities, 'stack_lane')
    assert bool(lane['log_csv']) is enabled
    if enabled:
        assert lane['log_csv'].endswith('/lane_frames.csv')
    assert lane['publish_debug_image'] is (debug == 'true')


@pytest.mark.parametrize('override,expected', [(None, .25), ('1.0', 1.0)])
def test_parallel_trial_short_straight_reaches_node(monkeypatch, override, expected):
    overrides = {} if override is None else {'parallel_opposite_straight_m': override}
    context, entities = graph(
        monkeypatch, 'src/stack_parking/launch/parallel_parking_test.launch.py', overrides)
    params = node_parameters(context, entities, 'stack_parking',
                             required_parameter='parallel_opposite_straight_m')
    assert params['parallel_opposite_straight_m'] == expected
