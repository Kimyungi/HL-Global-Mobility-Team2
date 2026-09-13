"""Run production scan callbacks against synthetic scenes without ROS hardware."""
import math
from types import MethodType, SimpleNamespace as NS

import pytest
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

from stack_avoid.node import StackAvoidNode
from stack_avoid.path_planner import AvoidPathPlanner
from stack_avoid.surfaces import Surface, surface_clearance


@pytest.fixture
def node():
    n = NS(lidar_x=.76, lidar_y=0., front_center=0., front_half_angle=math.pi / 2,
           roi_angle=180.,
           cluster_dist=.1, surface_link_scale=3., surface_max_link=.3,
           max_range=12., vehicle_width=.62, lateral_margin=.15, depth_band=.6,
           offset_max=1.6, corridor_half_width=.46, target_rate_mps=3., scan_rate_hz=10.,
           _surfaces=[], _prev_center=None, _prev_center_t=None,
           _detected_prev=False, detect_range=3., detect_hysteresis=.4,
           ttc_stop=1.5, target_speed=1., _maneuver_armed=False, _done_until=0.,
           _clear_since=None, clear_gap_max=3., clear_margin=.3, vehicle_len=.85,
           get_clock=lambda: NS(now=lambda: Time(seconds=100.)),
           get_logger=lambda: NS(debug=lambda msg: None, info=lambda msg: None),
           _ego_speed=lambda: 1., _publish_static_tf=lambda: None,
           _recompute_derived=lambda: None)
    n.path_sample_time, n.path_spacing, n.path_preview, n.path_tail = .1, .05, 1., 3.
    n.path_pose_timeout = n.path_command_timeout = n.path_gps_timeout = .5
    n._planner = AvoidPathPlanner(width=.62, length=.85, front=.76, margin=.15, min_radius=1.15)
    n._poses = {'vehicle': ((0., 0., 0.), 99_900_000_000)}
    n._pose_source = None
    n._gps_goals, n._gps_stamp = {}, 0
    n._command_stamp, n._command_v, n._path_last_scan = 0, 0., 0
    n._completed = False
    n._publish_path = lambda scan, pose: None
    n.messages = []
    n.pub = NS(publish=n.messages.append)
    n.front_scan_pub = NS(publish=lambda msg: None)
    for name in ('on_scan', '_scan_surfaces', '_nearest_front_obstacle',
                 '_target_clear', '_behind_surface', '_front_only_scan',
                 '_fresh_stamp', '_path_pose', '_path_goals', '_station_reference',
                 '_on_gps_path', '_store_vehicle_pose', '_on_path_command', '_on_path_session',
                 '_on_set_params'):
        setattr(n, name, MethodType(getattr(StackAvoidNode, name), n))
    n._rp = StackAvoidNode._rp
    n._valid_surface_params = StackAvoidNode._valid_surface_params
    n.BEHIND_TOL_M = StackAvoidNode.BEHIND_TOL_M
    n.BEHIND_WIN_DEG = StackAvoidNode.BEHIND_WIN_DEG
    return n


def scene_scan(segments):
    """Independent ray/line equation for a one-degree forward scan (LiDAR frame)."""
    ranges = []
    for degrees in range(-90, 91):
        dx, dy = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
        hits = []
        for (ax, ay), (bx, by) in segments:
            sx, sy = bx - ax, by - ay
            determinant = dx * sy - dy * sx
            if abs(determinant) < 1e-10:
                continue
            r = (ax * sy - ay * sx) / determinant
            u = (ax * dy - ay * dx) / determinant
            if r > 0 and 0 <= u <= 1:
                hits.append(r)
        ranges.append(min(hits, default=math.inf))
    scan = LaserScan(angle_min=-math.pi / 2, angle_max=math.pi / 2,
                     angle_increment=math.pi / 180, range_min=.05, range_max=12.,
                     ranges=ranges)
    scan.header.stamp = Time(seconds=99.9).to_msg()
    return scan


def test_finite_obstacle_produces_one_target_with_surface_clearance(node):
    scan = scene_scan([((2., -.2), (2., .2))])
    node.on_scan(scan)
    msg = node.messages[-1]
    assert msg.obstacle_detected and msg.avoidable and not msg.narrow_gap
    assert msg.ttc == pytest.approx(2.)
    assert len(msg.points) == 1
    p = msg.points[0]
    assert 0.0 < p.x <= 1.0
    assert n_preview_distance(node) == pytest.approx(1.)
    assert surface_clearance((p.x, p.y), node._surfaces) >= .46 - 1e-6
    assert msg.reference_stamp == scan.header.stamp


def test_wide_wall_reports_no_target_and_narrow_gap(node):
    node.on_scan(scene_scan([((2., -3.), (2., 3.))]))
    msg = node.messages[-1]
    assert msg.obstacle_detected and msg.narrow_gap
    assert not msg.avoidable and not msg.points


def test_diagonal_wall_cannot_create_sliced_left_edge(node):
    scan = scene_scan([((.74, 1.4), (3.24, .2))])
    node.on_scan(scan)
    msg = node.messages[-1]
    assert msg.obstacle_detected and msg.avoidable
    assert msg.points[0].y < 0  # Only the actual right side is available.


def test_obstacles_on_both_sides_keep_real_opening(node):
    scan = scene_scan([((2., -3.), (2., -.6)), ((2., .6), (2., 3.))])
    node._surfaces = node._scan_surfaces(scan)
    # Exercise gap selection independently: this opening does not block straight travel.
    assert node._nearest_front_obstacle(scan) is None
    goals = node._path_goals(scan, 2.)
    assert goals and goals[0][1] == pytest.approx(0., abs=.03)


def test_small_single_return_is_not_filtered_out(node):
    scan = scene_scan([])
    scan.ranges[90] = 2.
    node.on_scan(scan)
    msg = node.messages[-1]
    assert msg.obstacle_detected and msg.avoidable and len(msg.points) == 1


def test_new_clear_scan_does_not_reuse_old_surfaces_or_complete_immediately(node):
    node.on_scan(scene_scan([((2., -.2), (2., .2))]))
    clear = scene_scan([])
    clear.header.stamp = Time(seconds=100.).to_msg()
    node.on_scan(clear)
    msg = node.messages[-1]
    assert node._surfaces == []
    assert not msg.obstacle_detected and not msg.maneuver_done
    assert msg.ttc == pytest.approx(1e9)


def n_preview_distance(node):
    path = node._planner.path
    return min(path.s[-1], path.station+node.path_preview)-path.station


def test_duplicate_scan_does_not_advance_or_republish_station(node):
    scan = scene_scan([((2., -.2), (2., .2))])
    node.on_scan(scan)
    count = len(node.messages)
    station = node._planner.path.station
    node.on_scan(scan)
    assert len(node.messages) == count and node._planner.path.station == station


def test_target_on_surface_rejected_even_if_not_behind_it(node):
    node._surfaces = [Surface(((2.76, -.15), (2.76, .15)))]
    assert not node._target_clear(scene_scan([]), 2.76, 0., [], .46)


def test_invalid_parameter_batch_is_rejected_before_other_updates(node):
    result = node._on_set_params([
        Parameter('avoid.depth_band_m', value=.8),
        Parameter('avoid.surface_max_link_m', value=.01)])
    assert not result.successful
    assert node.depth_band == .6 and node.surface_max_link == .3


def test_valid_parameter_update_changes_surface_connection(node):
    result = node._on_set_params([Parameter('avoid.cluster_dist_m', value=.2)])
    assert result.successful and node.cluster_dist == .2


@pytest.mark.parametrize('value', [0., -1., math.nan, math.inf])
def test_surface_parameter_validation(value):
    assert not StackAvoidNode._valid_surface_params(.1, value, .3)
