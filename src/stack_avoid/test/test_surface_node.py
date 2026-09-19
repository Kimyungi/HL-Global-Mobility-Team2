"""Run production scan callbacks against synthetic scenes without ROS hardware."""
import math
from types import MethodType, SimpleNamespace as NS

import pytest
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

from stack_avoid.node import StackAvoidNode
from stack_avoid.gps_cubic_path import GpsCubicPlanner, WaypointWindow
from stack_avoid.surfaces import Surface, surface_clearance


@pytest.fixture
def node():
    n = NS(lidar_x=.76, lidar_y=0., front_center=0., front_half_angle=math.pi / 2,
           roi_angle=180.,
           cluster_dist=.1, surface_link_scale=3., surface_max_link=.3,
           max_range=12., vehicle_width=.62, lateral_margin=.15, depth_band=.6,
           offset_max=1.6, corridor_half_width=.46, detect_half_width=.46, target_rate_mps=3., scan_rate_hz=10.,
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
    n._planner = GpsCubicPlanner(width=.62, length=.85, front=.76, margin=.15, min_radius=1.15)
    n._poses = {'gps': ((0., 0., 0.), 99_900_000_000)}
    n._pose_source = None
    n._gps_stamp, n._gps_route_key = 0, None
    n._gps_waypoints = WaypointWindow(list(map(float, range(13))),
                                    [(float(i), 0., 0., 0.) for i in range(13)], 0.)
    n._command_stamp, n._command_v, n._path_last_scan = 0, 0., 0
    n._completed = False
    n.require_mgm_active, n._mgm_active, n._mgm_stamp = False, False, 0
    n._publish_path = lambda scan, pose: None
    n.messages = []
    n.pub = NS(publish=n.messages.append)
    n.front_scan_pub = NS(publish=lambda msg: None)
    for name in ('on_scan', '_scan_surfaces', '_nearest_front_obstacle',
                 '_target_clear', '_behind_surface', '_front_only_scan',
                 '_fresh_stamp', '_path_pose', '_path_goals', '_station_reference',
                 '_on_gps_path', '_on_mgm_state', '_store_vehicle_pose', '_on_path_session',
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
    assert p.x == pytest.approx(2.76)  # actual side point, not a preview sample
    assert node._planner.return_station == pytest.approx(5.46)
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
    goals = node._path_goals(scan)
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


@pytest.mark.parametrize('gap,y,detected', [
    (1.999999, 0., True), (2.000001, 0., False), (2.01, 0., False),
    (1.8, .99, True), (1.8, -.99, True),
    (1.8, 1.01, False), (1.8, -1.01, False),
])
def test_two_meter_one_meter_detection_area(node, gap, y, detected):
    result = node._on_set_params([
        Parameter('avoid.detect_range_m', value=2.),
        Parameter('avoid.detect_half_width_m', value=1.),
    ])
    assert result.successful
    assert node.vehicle_width == .62 and node.lateral_margin == .15
    node._scan_surfaces = lambda scan: [Surface(((node.lidar_x+gap, y),))]
    node.on_scan(scene_scan([]))
    assert node.messages[-1].obstacle_detected == detected


def test_detection_release_retains_existing_point_four_meter_hysteresis(node):
    node.detect_range, node.detect_half_width = 2., 1.
    node._detected_prev = True
    node._scan_surfaces = lambda scan: [Surface(((node.lidar_x+2.3, .8),))]
    node.on_scan(scene_scan([]))
    assert node.messages[-1].obstacle_detected


@pytest.mark.parametrize('x,width,angle', [(2.3, 0.4, 20), (2.3, 0.6, 15), (2.3, 0.6, 20), (2.5, 0.6, 15), (2.5, 0.6, 20), (2.7, 0.6, 15), (2.7, 0.6, 20), (2.86, 0.6, 15), (2.86, 0.6, 20), (3.05, 0.6, 15), (3.05, 0.6, 20)])
def test_tilted_box_finds_nearby_feasible_target(node, x, width, angle):
    # These scenes had a path at 0 degrees, but lost it at a small rotation.
    node.offset_max, node.detect_range, node.detect_half_width = 2.5, 2., .5
    yaw = math.radians(angle)
    corners = [(x+math.cos(yaw)*dx-math.sin(yaw)*dy-node.lidar_x,
                math.sin(yaw)*dx+math.cos(yaw)*dy)
               for dx, dy in ((-.3, -width/2), (.3, -width/2),
                              (.3, width/2), (-.3, width/2))]
    scan = scene_scan(list(zip(corners, corners[1:]+corners[:1])))
    node.on_scan(scan)
    msg = node.messages[-1]
    assert msg.obstacle_detected and len(msg.points) == 1
    target = msg.points[0]
    gap = node._nearest_front_obstacle(scan)[0]
    assert node.lidar_x+gap-1e-6 <= target.x <= max(p[0] for s in node._surfaces for p in s.points)+.46+1e-6
    assert abs(target.y) <= node.offset_max
    assert node._planner.return_station-node._planner.goal_station == pytest.approx(2.7)
    # The entire published curve still satisfies the original physical checks.
    assert node._planner._safe(node._planner.path.points, node._surfaces)


@pytest.mark.parametrize('angle', [-30., 30.])
def test_face_based_goal_uses_observed_far_end_instead_of_nearest_band(node, angle):
    node.offset_max, node.detect_range, node.detect_half_width = 2.5, 2., .5
    yaw = math.radians(angle)
    corners = [(3.05+math.cos(yaw)*dx-math.sin(yaw)*dy-node.lidar_x,
                math.sin(yaw)*dx+math.cos(yaw)*dy)
               for dx, dy in ((-.3, -.3), (.3, -.3), (.3, .3), (-.3, .3))]
    scan = scene_scan(list(zip(corners, corners[1:]+corners[:1])))
    node.on_scan(scan)
    msg = node.messages[-1]
    assert msg.obstacle_detected and len(msg.points) == 1
    target = msg.points[0]
    nearest = node.lidar_x+node._nearest_front_obstacle(scan)[0]
    assert target.x > nearest+.3
    assert target.x == pytest.approx(max(p[0] for s in node._surfaces for p in s.points))
    assert node._planner._safe(node._planner.path.points, node._surfaces)
    assert node._planner.return_station-node._planner.goal_station == pytest.approx(2.7)


@pytest.mark.parametrize('x,width', [(3.05, .6), (2.4, .7)])
@pytest.mark.parametrize('return_distance,expected', [(2., False), (2.7, True)])
def test_45_degree_obstacle_recovers_with_longer_return(node, x, width, return_distance, expected):
    node.offset_max, node.detect_range, node.detect_half_width = 2.5, 2., .5
    node._planner.return_distance = return_distance
    yaw = math.pi/4
    corners = [(x+math.cos(yaw)*dx-math.sin(yaw)*dy-node.lidar_x,
                math.sin(yaw)*dx+math.cos(yaw)*dy)
               for dx, dy in ((-.3, -width/2), (.3, -width/2),
                              (.3, width/2), (-.3, width/2))]
    scan = scene_scan(list(zip(corners, corners[1:]+corners[:1])))
    node.on_scan(scan)
    assert bool(node.messages[-1].points) is expected
    if expected:
        assert node._planner.return_station-node._planner.goal_station == pytest.approx(2.7)
        assert node._planner._safe(node._planner.path.points, node._surfaces)


@pytest.mark.parametrize('gap,y,previous,expected', [
    (3.499, 0., False, True), (3.501, 0., False, False),
    (3.8, 0., False, False), (3.8, 0., True, True),
    (3.899, 0., True, True), (3.901, 0., True, False),
    (3.4, .499, False, True), (3.4, .501, False, False),
])
def test_three_point_five_meter_detection_and_hysteresis(node, gap, y, previous, expected):
    node.detect_range, node.detect_half_width = 3.5, .5
    node._detected_prev = previous
    node._scan_surfaces = lambda scan: [Surface(((node.lidar_x+gap, y),))]
    node.on_scan(scene_scan([]))
    assert node.messages[-1].obstacle_detected is expected


def test_early_detection_at_two_mps_can_produce_avoidable_reference(node):
    node.detect_range, node.detect_half_width = 3.5, .5
    node.offset_max = 2.5
    node._ego_speed = lambda: 2.
    node.on_scan(scene_scan([((3.4, -.2), (3.4, .2))]))
    msg = node.messages[-1]
    assert msg.obstacle_detected and msg.points and msg.avoidable
    assert msg.ttc == pytest.approx(1.7)
    assert node._planner._safe(node._planner.path.points, node._surfaces)
