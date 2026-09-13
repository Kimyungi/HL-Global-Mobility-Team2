"""Exercise the launched front-scan consumers without starting ROS nodes."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

from stack_avoid.node import StackAvoidNode
from stack_estop.node import DistanceEstopController, analyze_corridor_scan

ROOT = Path(__file__).resolve().parents[1]


def consumer_parameters(monkeypatch, parking, *, lidar_estop_enabled=True, yaw=None):
    path = ROOT / 'src/adas_mgm/launch/REAL_VEHICLE_lane_gps_can.launch.py'
    spec = importlib.util.spec_from_file_location('front_lidar_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # Resolve repository assets; no installed v2 workspace or hardware is needed.
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda package: str(ROOT / 'src' / package))
    description = module.build_launch_description(lidar_estop_enabled=lidar_estop_enabled)
    context = LaunchContext()
    context.launch_configurations['parking_enabled'] = str(parking).lower()
    if yaw is not None:
        context.launch_configurations['laser_yaw_in_base_rad'] = str(yaw)
    for item in description.entities:
        if isinstance(item, DeclareLaunchArgument):
            item.execute(context)
    result = {}
    for item in description.entities:
        if not isinstance(item, Node) or item.node_package not in ('stack_avoid', 'stack_estop'):
            continue
        if item.node_package in result:
            continue
        params = {}
        for source in evaluate_parameters(context, item._Node__parameters):
            if isinstance(source, dict):
                params.update(source)
            else:
                loaded = yaml.safe_load(Path(source).read_text())
                raw = next(iter(loaded.values()))['ros__parameters']
                for key, value in raw.items():
                    if isinstance(value, dict):
                        params.update({f'{key}.{subkey}': subvalue
                                       for subkey, subvalue in value.items()})
                    else:
                        params[key] = value
        result[item.node_package] = params
    return result


def scan_cluster(raw_angle_deg, distance=0.6):
    return SimpleNamespace(
        ranges=[distance] * 5,
        angle_min=math.radians(raw_angle_deg - 2.0),
        angle_increment=math.radians(1.0), range_min=0.03, range_max=12.0)


def detect(params, scan):
    estop = analyze_corridor_scan(
        scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
        laser_yaw_in_base_rad=params['stack_estop']['laser_yaw_in_base_rad'])
    avoid_params = params['stack_avoid']
    avoid = SimpleNamespace(
        front_center=math.radians(avoid_params['lidar_mount.forward_angle_deg']),
        front_half_angle=math.radians(avoid_params['avoid.roi_angle_deg'] / 2),
        max_range=avoid_params['avoid.max_range_m'],
        lidar_x=avoid_params['lidar_mount.x_m'],
        lidar_y=avoid_params['lidar_mount.y_m'],
        cluster_dist=avoid_params['avoid.cluster_dist_m'],
        surface_link_scale=avoid_params['avoid.surface_link_scale'],
        surface_max_link=avoid_params['avoid.surface_max_link_m'],
        corridor_half_width=(avoid_params['vehicle.width_m'] / 2
                             + avoid_params['avoid.lateral_margin_m']))
    avoid._surfaces = StackAvoidNode._scan_surfaces(avoid, scan)
    return estop, StackAvoidNode._nearest_front_obstacle(avoid, scan)


def front_yaw():
    data = yaml.safe_load((ROOT / 'src/lidar_fusion_v2/config/fixed_geometry.yaml').read_text())
    return data['/**']['ros__parameters']['sensors']['a1']['yaw_deg']


@pytest.mark.parametrize('estop_enabled', [True, False])
def test_four_lidar_consumers_share_calibrated_front(monkeypatch, estop_enabled):
    params = consumer_parameters(monkeypatch, True, lidar_estop_enabled=estop_enabled)
    assert params['stack_avoid']['scan_topic'] == '/lidar/a1/scan'
    assert params['stack_estop']['laser_yaw_in_base_rad'] == pytest.approx(math.radians(front_yaw()))
    assert params['stack_avoid']['lidar_mount.forward_angle_deg'] == pytest.approx(-front_yaw())


def test_rear_returns_do_not_block_empty_front(monkeypatch):
    params = consumer_parameters(monkeypatch, True)
    # Raw -93 degrees is behind the fixed a1, although legacy +90 sees it ahead.
    result, obstacle = detect(params, scan_cluster(-front_yaw() - 180))
    assert result['nearest_cluster_min_x'] is None
    assert obstacle is None
    controller = DistanceEstopController()
    for _ in range(3):
        controller.update_from_scan(result['nearest_cluster_min_x'])
    assert not controller.current_final_estop


@pytest.mark.parametrize('relative_angle', [-15, 0, 15])
def test_real_front_obstacle_still_stops_and_keeps_left_right(monkeypatch, relative_angle):
    params = consumer_parameters(monkeypatch, True)
    result, obstacle = detect(params, scan_cluster(-front_yaw() + relative_angle))
    assert result['nearest_cluster_min_x'] == pytest.approx(
        0.6 * math.cos(math.radians(abs(relative_angle) + 2)))
    assert obstacle is not None
    if relative_angle:
        assert obstacle[1] * relative_angle > 0
    controller = DistanceEstopController()
    for _ in range(3):
        controller.update_from_scan(None)
    assert not controller.current_final_estop
    controller.update_from_scan(result['nearest_cluster_min_x'])
    assert controller.current_final_estop


def test_single_lidar_retains_legacy_orientation(monkeypatch):
    params = consumer_parameters(monkeypatch, False)
    assert params['stack_avoid']['scan_topic'] == '/scan'
    assert params['stack_avoid']['lidar_mount.forward_angle_deg'] == 270.0
    assert params['stack_estop']['laser_yaw_in_base_rad'] == pytest.approx(math.pi / 2)
    result, obstacle = detect(params, scan_cluster(-90))
    assert result['nearest_cluster_min_x'] is not None
    assert obstacle is not None


def test_explicit_estop_yaw_override_is_preserved(monkeypatch):
    params = consumer_parameters(monkeypatch, True, yaw=0.25)
    assert params['stack_estop']['laser_yaw_in_base_rad'] == 0.25
    assert params['stack_avoid']['lidar_mount.forward_angle_deg'] == pytest.approx(
        -math.degrees(0.25) % 360)
