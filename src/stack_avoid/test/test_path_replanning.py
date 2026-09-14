"""Off-path recovery must produce a new certified target or remain unavailable."""
import math

import pytest

from stack_avoid.path_planner import AvoidPathPlanner
from stack_avoid.station_path import StationPath, straight
from stack_avoid.surfaces import Surface


def planner():
    p = AvoidPathPlanner(width=.62, length=.85, front=.76, margin=.15, min_radius=1.15)
    p.path = StationPath(straight((0., 0., 0., 0.), 10., .05))
    p.episode_active = True
    p.mode = 'return'
    p.return_station = 3.
    return p


def step(p, pose=(0., .8, 0.), *, detected=True, goals=((3., .8),), generation=1):
    return p.step(pose=pose, surfaces=[], scan={}, lidar=(.76, 0.), goals=goals,
                  detected=detected, gps_goal=None, v_ref=0., sample_time=.1,
                  generation=generation)


def test_off_path_replans_at_current_pose_instead_of_rejecting_forever():
    p = planner()
    old = p.path
    target, done = step(p)
    assert target is not None and not done
    assert p.path is not old and p.path.points[0][:2] == pytest.approx((0., .8))
    assert p.replan_count == 1 and p.last_replan_distance == pytest.approx(.8)
    assert p.return_station is None and p.mode == 'bypass'
    assert all(abs(point[3]) <= 1/p.min_radius+1e-6 for point in p.path.points)


def test_failed_replan_keeps_known_obstacle_and_never_reuses_old_path():
    p = planner()
    remembered = Surface(((1.4, -3.), (1.4, 3.)))
    p.initial_surfaces = [remembered]
    for generation in (1, 2):
        target, done = step(p, goals=((3., 0.),), generation=generation)
        assert target is None and not done
        assert p.path is None and p.episode_active
        assert p.initial_surfaces == [remembered]
    assert p.replan_count == 1  # failure retries without resetting the episode


def test_failed_replan_can_retry_when_feasible_goal_arrives():
    p = planner()
    assert step(p, goals=())[0] is None
    assert p.path is None
    assert step(p, generation=2)[0] is not None


def test_detection_clear_after_failed_replan_does_not_orphan_episode():
    p = planner()
    assert step(p, goals=())[0] is None
    target, done = step(p, detected=False, goals=(), generation=2)
    assert target is not None and not done
    assert p.mode == 'straight'


def test_initial_planning_failure_also_retries_after_detection_clears():
    p = AvoidPathPlanner(width=.62, length=.85, front=.76, margin=.15, min_radius=1.15)
    assert step(p, goals=())[0] is None
    assert step(p, detected=False, goals=(), generation=2)[0] is not None


def test_idle_without_obstacle_does_not_invent_a_path():
    p = AvoidPathPlanner(width=.62, length=.85, front=.76, margin=.15, min_radius=1.15)
    assert step(p, detected=False)[0] is None
    assert not p.episode_active


def test_logged_last_target_and_gps_displacement_replan_at_threshold_crossing():
    p = planner()
    # Last valid target at tick 9360, assuming its zero-curvature preview is straight.
    x, y, yaw = .85945189, -.8983977437, -.171277076
    start = (x-math.cos(yaw), y-math.sin(yaw), yaw, 0.)
    origin = (start[0]-5*math.cos(yaw), start[1]-5*math.sin(yaw), yaw, 0.)
    p.path = StationPath(straight(origin, 15., .05))
    p.path.station = 5.
    p.mode = 'straight'; p.return_station = None
    assert step(p, pose=(0., 0., 0.), detected=False)[0] is not None
    assert p.replan_count == 0
    heading = math.radians(-124.2)
    dx, dy = -.05903146, -.11132
    pose = (math.cos(heading)*dx+math.sin(heading)*dy,
            -math.sin(heading)*dx+math.cos(heading)*dy, math.radians(-.2))
    assert step(p, pose=pose, generation=2)[0] is not None
    assert p.last_replan_distance == pytest.approx(.77366069)
    assert p.path.points[0][:2] == pytest.approx(pose[:2])
