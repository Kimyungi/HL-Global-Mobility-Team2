import math

import pytest

from stack_avoid.gps_cubic_path import GpsCubicPlanner, WaypointWindow, cubic_connector
from stack_avoid.station_path import to_world, to_local
from stack_avoid.surfaces import Surface


def planner(return_distance=2.7):
    return GpsCubicPlanner(width=.62, length=.85, front=.76, margin=.15, min_radius=1.15, return_distance=return_distance)


def route(station=0., heading=0., origin=(0., 0.), end=12):
    pose = (*origin, heading)
    return WaypointWindow([float(i) for i in range(end+1)],
                          [to_world((float(i), 0., 0., 0.), pose) for i in range(end+1)], station)


def step(p, *, station=0., pose=None, goals=((2.66, .76),), detected=True, surfaces=()):
    return p.step(pose=(station, 0., 0.) if pose is None else pose,
                  waypoints=route(station), surfaces=list(surfaces), goals=goals, detected=detected)


def test_connector_is_cubic_and_has_exact_endpoints_and_tangents():
    a, b = (1., .2, .1, 0.), (4., .9, -.2, 0.)
    points = cubic_connector(a, b)
    assert points[0][:3] == pytest.approx(a[:3])
    assert points[-1][:3] == pytest.approx(b[:3])
    # Uniform parameter samples of a degree-three polynomial have zero 4th difference.
    for axis in (0, 1):
        d = [p[axis] for p in points]
        for _ in range(4): d = [y-x for x, y in zip(d, d[1:])]
        assert max(abs(v) for v in d) < 1e-10


def test_station_is_ordered_arc_length_not_index_or_euclidean_distance():
    w = WaypointWindow([0., 3., 7.], [(0., 0., 0., 0.), (3., 0., 0., 0.),
                                    (3., 4., math.pi/2, 0.)], 2.)
    assert w.at(w.station+1)[:2] == pytest.approx((3., 0.))
    goal_station = w.project((3.5, 1.))
    assert goal_station == pytest.approx(4.)
    assert w.at(goal_station+2)[:2] == pytest.approx((3., 3.))


def test_clear_scene_follows_gps_station_plus_one_without_vehicle_pose_fallback():
    p = planner()
    target, done = step(p, station=3., detected=False, goals=())
    assert target[:2] == pytest.approx((1., 0.)) and not done
    assert not p.episode_active and p.mode == 'gps'
    target, done = p.step(pose=None, waypoints=route(), surfaces=[], goals=[], detected=True)
    assert target is None and p.path is None and not done


def test_rotated_enu_waypoints_preserve_the_same_vehicle_frame_reference():
    p = planner(); yaw = math.radians(-135)
    w = route(station=2., heading=yaw, origin=(40., -20.))
    pose = to_world((2., 0., 0., 0.), (40., -20., yaw))[:3]
    target, _ = p.step(pose=pose, waypoints=w, surfaces=[], goals=[], detected=False)
    assert target[:3] == pytest.approx((1., 0., 0.))


def test_side_point_is_control_target_and_return_is_station_plus_2_7():
    p = planner()
    target, done = step(p)
    assert target is not None and target[:2] == pytest.approx((2.66, .76)) and not done
    assert p.goal_station == pytest.approx(2.66)
    assert p.return_station == pytest.approx(5.36)
    assert p.path.points[-1][:2] == pytest.approx((5.36, 0.))
    assert any(math.dist(q[:2], target[:2]) < 1e-8 for q in p.path.points)


def test_two_meter_detection_scene_uses_current_pose_only_if_anchor_curve_fails():
    p = planner()
    obstacle = Surface(((2.66, -.2), (2.66, .2)))
    target, _ = step(p, surfaces=[obstacle])
    assert target is not None and p.anchor_fallback
    assert p.path.points[0][:2] == pytest.approx((0., 0.))
    assert p._safe(p.path.points, [obstacle])


def test_roomy_scene_keeps_waypoint_plus_one_anchor():
    p = planner()
    target, _ = step(p, goals=((4., .8),))
    assert target is not None and not p.anchor_fallback
    assert p.path.points[0][:2] == pytest.approx((1., 0.))


def test_each_scan_rebuilds_and_drops_traveled_prefix():
    p = planner(); step(p)
    old = p.path
    target, _ = step(p, station=1.5, goals=((2.5, -.8),))
    assert p.path is not old and target[:2] == pytest.approx((2.5, -.8))
    assert min(q[0] for q in p.path.points) >= 1.5-1e-8
    assert not any(q in p.path.points for q in old.points[:5])


def test_no_stale_path_when_new_scene_blocks_all_cubics():
    p = planner(); assert step(p)[0] is not None
    target, _ = step(p, surfaces=[Surface(((1.5, -5.), (1.5, 5.)))])
    assert target is None and p.path is None


def test_unavailable_configured_return_waypoint_does_not_become_straight_tail():
    p = planner()
    target, done = p.step(pose=(0., 0., 0.), waypoints=route(end=5), surfaces=[],
                          goals=[(3., .8)], detected=True)
    assert target is None and p.path is None and not done


def test_return_target_is_fixed_waypoint_after_side_point_is_passed():
    p = planner(); assert step(p)[0] is not None
    pose = next(q[:3] for q in p.path.points if q[0] >= 2.8)
    target, done = step(p, station=pose[0], pose=pose, detected=False, goals=())
    assert target is not None and not done and p.mode == 'return'
    assert to_world(target, pose)[:2] == pytest.approx((5.36, 0.))
    assert p.path.points[0][:2] == pytest.approx(pose[:2])
    target, done = step(p, station=5.2, detected=False, goals=())
    assert target is not None and done


def test_obstacle_memory_remains_in_collision_checks_after_detection_clears():
    p = planner(); assert step(p)[0] is not None
    p.initial_surfaces = [Surface(((3.6, -4.), (3.6, 4.)))]
    assert step(p, station=2.8, pose=(2.8, .76, 0.), detected=False, goals=())[0] is None
    assert p.path is None


def test_fresh_same_side_goal_nearest_previous_world_goal_has_priority():
    p = planner()
    assert step(p, goals=((3.5, .9),))[0] is not None
    target, _ = step(p, goals=((3.5, -.89), (4., .9), (3.51, .91)))
    assert target[:2] == pytest.approx((3.51, .91))
    assert p.goal_side == 1 and not p.side_switched


def test_blocked_preferred_side_switches_only_to_collision_free_opposite():
    p = planner()
    goals = ((3.5, .9), (3.5, -.9))
    assert step(p, goals=goals)[0][1] > 0
    wall = Surface(((2.5, .6), (4.5, .6)))
    target, _ = step(p, goals=goals, surfaces=[wall])
    assert target is not None and target[1] < 0
    assert p.goal_side == -1 and p.side_switched
    assert p._safe(p.path.points, [wall])
    assert step(p, goals=goals, surfaces=[wall])[0][1] < 0
    assert not p.side_switched


def test_both_sides_blocked_clears_reference_but_retains_preference():
    p = planner(); goals = ((3.5, .9), (3.5, -.9))
    step(p, goals=goals)
    target, _ = step(p, goals=goals, surfaces=[Surface(((2., -5.), (2., 5.)))])
    assert target is None and p.path is None and p.control_target is None
    assert p.goal_side == 1 and not p.side_switched


def test_reset_discards_previous_direction_preference():
    p = planner(); step(p, goals=((3.5, .9),))
    p.reset()
    assert p.goal is None and p.goal_side is None and not p.side_switched
    assert step(p, goals=((3.5, -.9), (3.5, .9)))[0][1] < 0


@pytest.mark.parametrize('yaw', [0., math.radians(130.)])
def test_passing_side_is_gps_relative_even_when_vehicle_crosses_goal_lateral(yaw):
    p = planner(); frame = (40., -20., yaw)
    w = route(heading=yaw, origin=frame[:2])
    assert p.step(pose=frame, waypoints=w, surfaces=[], goals=[(3.5, .9)], detected=True)[0]
    # Vehicle is now further left than the old left goal: BOTH candidates
    # have negative vehicle Y, but GPS route sides remain distinct.
    pose = to_world((.1, 1., 0., 0.), frame)[:3]
    target, _ = p.step(pose=pose, waypoints=w, surfaces=[],
                       goals=[(3.4, -1.9), (3.4, -.1)], detected=True)
    assert target is not None and target[:2] == pytest.approx((3.4, -.1))
    assert p.goal_side == 1 and not p.side_switched
