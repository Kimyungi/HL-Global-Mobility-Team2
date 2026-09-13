import math

import pytest
from rclpy.time import Time
from fma_interfaces.msg import GpsPath, RefPoint, TargetRef, VehicleVector
from std_msgs.msg import Bool

from test_surface_node import node, scene_scan  # noqa: F401 (shared fixture)


def gps(stamp=99.9):
    msg = GpsPath()
    msg.header.stamp = msg.reference_stamp = Time(seconds=stamp).to_msg()
    msg.header.frame_id = 'base_link'
    msg.fix_quality = 4
    msg.position_valid = msg.vehicle_heading_valid = True
    msg.position_x, msg.position_y, msg.vehicle_heading_rad = 12., -3., .2
    msg.points = [RefPoint(x=2.5, y=0., yaw=.1, curvature=.05)]
    return msg


def test_gps_pose_and_return_goal_are_anchored_once_per_sensor_stamp(node):
    msg = gps()
    node._on_gps_path(msg)
    goal = node._gps_goals['gps']
    assert goal[0][0] == pytest.approx(12.+2.5*math.cos(.2))
    msg.points[0].x = 8.
    node._on_gps_path(msg)
    assert node._gps_goals['gps'] == goal


def test_invalid_or_tangent_heading_clears_previous_return_goal(node):
    msg = gps()
    node._on_gps_path(msg)
    msg.vehicle_heading_valid = False
    node._on_gps_path(msg)
    assert not node._gps_goals and 'gps' not in node._poses


def test_active_path_does_not_switch_pose_frames_after_gps_loss(node):
    node._on_gps_path(gps())
    # Create a valid path and explicitly pin GPS coordinates for this episode.
    from stack_avoid.station_path import StationPath, straight
    node._planner.path = StationPath(straight((12., -3., .2, 0.), 4., .05))
    node._pose_source = 'gps'
    node._poses.pop('gps')
    assert node._path_pose() == (None, 0)  # Fresh vehicle pose is a different origin.


def test_command_freshness_uses_final_v_ref_and_unknown_is_zero(node):
    command = TargetRef(v_ref=.7)
    command.header.stamp = Time(seconds=99.9).to_msg()
    node._on_path_command(command)
    assert node._command_v == pytest.approx(.7)
    command.v_ref = math.nan
    node._on_path_command(command)
    assert node._command_v == 0 and node._command_stamp == 0


def test_station_updates_with_localization_generation_only(node):
    scan = scene_scan([((2., -.2), (2., .2))])
    node.on_scan(scan)
    path = node._planner.path
    node._poses['vehicle'] = ((.3, 0., 0.), 99_950_000_000)
    scan.header.stamp = Time(seconds=100.).to_msg()
    node.on_scan(scan)
    station = path.station
    assert 0 < station <= .5
    scan.header.stamp = Time(seconds=100.01).to_msg()
    node.get_clock = lambda: type('Clock', (), {'now': lambda self: Time(seconds=100.01)})()
    node.on_scan(scan)
    assert path.station == station


def test_scan_or_pose_loss_cannot_publish_fresh_reference_or_completion(node):
    scan = scene_scan([((2., -.2), (2., .2))])
    node.on_scan(scan)
    station = node._planner.path.station
    node._poses.clear()
    scan.header.stamp = Time(seconds=100.).to_msg()
    node.on_scan(scan)
    msg = node.messages[-1]
    assert not msg.points and not msg.maneuver_done and not msg.reference_stamp.sec
    assert node._planner.path.station == station
    scan.ranges = [math.nan]*len(scan.ranges)
    scan.header.stamp = Time(seconds=100.1).to_msg()
    node.on_scan(scan)
    assert not node.messages[-1].scan_valid and not node.messages[-1].maneuver_done


def test_session_reset_discards_path_and_station_without_deleting_pose_inputs(node):
    node.on_scan(scene_scan([((2., -.2), (2., .2))]))
    assert node._planner.path is not None
    node._on_path_session(Bool(data=True))
    assert node._planner.path is None and node._pose_source is None
    assert 'vehicle' in node._poses


def test_nonfinite_vehicle_pose_is_not_used(node):
    msg = VehicleVector(x=math.nan)
    msg.header.stamp = Time(seconds=99.9).to_msg()
    node._store_vehicle_pose(msg)
    assert 'vehicle' not in node._poses
