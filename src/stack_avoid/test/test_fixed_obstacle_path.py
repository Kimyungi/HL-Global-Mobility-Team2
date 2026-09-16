import math
from types import MethodType

import pytest

from stack_avoid.compute_backend import compute_functions
from stack_avoid.fixed_obstacle_path import FixedObstaclePlanner, GoalGroup, obstacle_bands
from stack_avoid.gps_cubic_path import WaypointWindow
from stack_avoid.node import StackAvoidNode
from stack_avoid.station_path import to_local, to_world
from stack_avoid.surfaces import Surface
from test_surface_node import node, scene_scan  # noqa: F401


def route(station=0., frame=(0., 0., 0.)):
    return WaypointWindow(list(range(15)),
                          [to_world((float(i), 0., 0., 0.), frame) for i in range(15)], station)


@pytest.fixture(params=['python', 'native'])
def planner(request):
    connector, checker = compute_functions(request.param)
    return FixedObstaclePlanner(width=.62, length=.85, front=.76, margin=.15,
                                min_radius=1.15, connector=connector, collision_check=checker)


def groups():
    return [GoalGroup(2.5, 2.7, ((2.66, .76),)), GoalGroup(5., 5.2, ((5.1, .76),))]


def step(p, station=0., pose=None, detected=True, goals=None, surfaces=()):
    return p.step(pose=pose or (station, 0., 0.), waypoints=route(station),
                  surfaces=list(surfaces), detected=detected,
                  goal_groups=groups() if goals is None else goals)


def test_two_goals_are_fixed_across_jitter_and_detection_loss(planner):
    assert step(planner)[0]
    fixed, end = planner.fixed_goals, planner.return_world
    noise = [GoalGroup(2.5, 2.7, ((2.8, -.9),)), GoalGroup(5., 5.2, ((5.5, -.9),))]
    for detected in (True, False, True):
        target, done = step(planner, station=.2, goals=noise, detected=detected)
        assert target and not done
        assert planner.fixed_goals == fixed and planner.return_world == end
        assert to_world(target, (.2, 0., 0.))[:2] == pytest.approx(fixed[0].point[:2])


def test_station_passes_first_then_second_before_final_return(planner):
    assert step(planner)[0]
    fixed = planner.fixed_goals
    target, done = step(planner, station=2.66, pose=(2.66, .76, 0.))
    assert target and not done and planner.passed_goals == 1
    assert to_world(target, (2.66, .76, 0.))[:2] == pytest.approx(fixed[1].point[:2])
    target, done = step(planner, station=5.1, pose=(5.1, .76, 0.), detected=False)
    assert target and not done and planner.passed_goals == 2
    assert to_world(target, (5.1, .76, 0.))[:2] == pytest.approx((7.8, 0.))
    target, done = step(planner, station=7.7, detected=False)
    assert target and done and planner.fixed_goals == fixed


def test_reverse_and_invalid_pose_do_not_unlock_emitted_points(planner):
    step(planner)
    fixed = planner.fixed_goals
    step(planner, station=2.7, pose=(2.7, .76, 0.))
    step(planner, station=2.4, pose=(2.4, .76, 0.))
    assert planner.passed_goals == 1 and planner.fixed_goals == fixed
    assert planner.step(pose=None, waypoints=route(), surfaces=[], detected=False) == (None, False)
    assert planner.fixed_goals == fixed and planner.path is None


def test_new_wall_stops_without_switching_fixed_points(planner):
    step(planner)
    fixed = planner.fixed_goals
    target, done = step(planner, surfaces=[Surface(((2., -5.), (2., 5.)))])
    assert target is None and not done and planner.path is None
    assert planner.fixed_goals == fixed


def test_last_sample_before_station_does_not_stop_or_consume_goal_early(planner):
    assert step(planner)[0]
    target, done = step(planner, station=2.64, pose=(2.64, .76, 0.))
    assert target and not done and planner.passed_goals == 0
    assert to_world(target, (2.64, .76, 0.))[:2] == pytest.approx((2.66, .76))


def test_second_obstacle_can_be_added_without_moving_first(planner):
    assert step(planner, goals=groups()[:1])[0]
    first = planner.fixed_goals[0]
    assert step(planner, goals=groups())[0]
    assert len(planner.fixed_goals) == 2 and planner.fixed_goals[0] == first
    assert planner.return_station == pytest.approx(7.8)


def test_unfeasible_new_second_obstacle_does_not_replace_first(planner):
    step(planner, goals=groups()[:1])
    fixed = planner.fixed_goals
    target, _ = step(planner, goals=[GoalGroup(5., 5.2, ())])
    assert target is None and planner.fixed_goals == fixed


def test_missing_final_waypoint_does_not_publish_partial_two_goal_chain(planner):
    w = WaypointWindow(list(range(7)), [(float(i), 0., 0., 0.) for i in range(7)], 0.)
    target, done = planner.step(pose=(0., 0., 0.), waypoints=w, surfaces=[],
                                detected=True, goal_groups=groups())
    assert target is None and not done and not planner.fixed_goals


def test_rotated_frame_preserves_world_fixed_goals(planner):
    frame = (40., -20., math.radians(-135))
    assert planner.step(pose=frame, waypoints=route(frame=frame), surfaces=[],
                        detected=True, goal_groups=groups())[0]
    first = planner.fixed_goals[0].point
    pose = to_world((.2, .1, .02, 0.), frame)[:3]
    target, _ = planner.step(pose=pose, waypoints=route(.2, frame), surfaces=[], detected=True)
    assert target and to_world(target, pose)[:2] == pytest.approx(first[:2])
    planner.reset()
    assert planner.fixed_goals == () and planner.passed_goals == 0


def test_station_overlapping_faces_are_one_obstacle_band():
    surfaces = [Surface(((2., -.2), (2., .2))), Surface(((2.1, .2), (2.1, .4))),
                Surface(((4., -.2), (4., .2)))]
    bands = obstacle_bands(surfaces, (0., 0., 0.), route(), .5)
    assert len(bands) == 2 and len(bands[0][2]) == 2


def test_ros_callback_skips_candidate_search_once_two_goals_are_fixed(node, planner):
    node._planner = planner
    node._path_goal_groups = MethodType(StackAvoidNode._path_goal_groups, node)
    # Use already-emitted points; raw scan still passes through live collision checks.
    step(planner)
    node._path_goals = lambda *a: pytest.fail('fixed pair must not regenerate candidates')
    node.on_scan(scene_scan([]))
    assert node.messages[-1].points and len(planner.fixed_goals) == 2


def test_real_contour_candidates_form_two_obstacle_chain(node, planner):
    node._planner = planner
    node._path_goal_groups = MethodType(StackAvoidNode._path_goal_groups, node)
    node.detect_range, node.offset_max = 3.5, 2.5
    node.on_scan(scene_scan([((2., -.2), (2., .2)), ((4., .15), (4., .55))]))
    assert node.messages[-1].points
    assert len(planner.fixed_goals) == 2
    assert planner.fixed_goals[0].station < planner.fixed_goals[1].station
