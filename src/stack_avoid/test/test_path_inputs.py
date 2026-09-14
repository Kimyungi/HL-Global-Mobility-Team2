import math

import pytest
from rclpy.time import Time
from fma_interfaces.msg import GpsPath, RefPoint, VehicleVector
from std_msgs.msg import Bool

from test_surface_node import node, scene_scan  # noqa: F401


def gps(stamp=99.9):
    msg = GpsPath()
    msg.header.stamp = msg.reference_stamp = Time(seconds=stamp).to_msg()
    msg.header.frame_id = 'base_link'
    msg.fix_quality = 4
    msg.position_valid = msg.vehicle_heading_valid = True
    msg.heading_source = GpsPath.HEADING_FUSED
    msg.position_x, msg.position_y, msg.vehicle_heading_rad = 12., -3., .2
    msg.points = [RefPoint(x=2.5, y=0., yaw=.1, curvature=.05)]
    msg.waypoint_window_valid = True
    msg.waypoint_station_m = 10.
    msg.waypoint_stations = [10., 12., 15., 18.]
    msg.waypoint_points = [RefPoint(x=x) for x in (0., 2., 5., 8.)]
    return msg


def test_gps_window_is_anchored_with_same_pose_and_sensor_stamp(node):
    msg = gps(); node._on_gps_path(msg)
    window = node._gps_waypoints
    assert window.at(11.)[:2] == pytest.approx((12.+math.cos(.2), -3.+math.sin(.2)))
    msg.waypoint_points[1].x = 9.
    node._on_gps_path(msg)
    assert node._gps_waypoints is window


@pytest.mark.parametrize('bad', ['heading', 'tangent', 'window', 'order'])
def test_bad_gps_input_cannot_use_single_navigation_preview(node, bad):
    msg = gps()
    if bad == 'heading': msg.vehicle_heading_valid = False
    elif bad == 'tangent': msg.heading_source = GpsPath.HEADING_TANGENT
    elif bad == 'window': msg.waypoint_window_valid = False
    else: msg.waypoint_stations = [10., 15., 12., 18.]
    node._on_gps_path(msg)
    node.on_scan(scene_scan([((2., -.2), (2., .2))]))
    assert not node.messages[-1].points
    assert not node._planner.path


def test_fresh_vehicle_pose_is_not_a_gps_fallback(node):
    node._poses.clear()
    msg = VehicleVector(x=12., y=-3., yaw=.2)
    msg.header.stamp = Time(seconds=99.9).to_msg()
    node._store_vehicle_pose(msg)
    assert node._path_pose() == (None, 0)


def test_route_change_discards_old_goal_even_for_same_fix_generation(node):
    msg = gps(); node._on_gps_path(msg)
    node._planner.goal = (99., 99., 0., 0.)
    node._planner.episode_active = True
    msg.route.index = 1
    node._on_gps_path(msg)
    assert node._planner.goal is None and not node._planner.episode_active
    assert node._gps_waypoints is not None


def test_scan_loss_and_gps_loss_clear_visual_path_and_reference(node):
    node.on_scan(scene_scan([((2., -.2), (2., .2))]))
    assert node._planner.path is not None
    node._poses.clear()
    scan = scene_scan([]); scan.header.stamp = Time(seconds=100.).to_msg()
    node.on_scan(scan)
    assert not node.messages[-1].points and node._planner.path is None
    scan.ranges = [math.nan]*len(scan.ranges)
    scan.header.stamp = Time(seconds=100.1).to_msg()
    node.on_scan(scan)
    assert not node.messages[-1].scan_valid and node._planner.path is None


def test_session_reset_keeps_pose_but_requires_waypoint_window_again(node):
    node.on_scan(scene_scan([((2., -.2), (2., .2))]))
    node._on_path_session(Bool(data=True))
    assert node._planner.path is None and node._gps_waypoints is None
    assert 'gps' in node._poses
