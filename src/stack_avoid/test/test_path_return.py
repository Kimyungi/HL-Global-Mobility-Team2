"""Follow the generated path through a real occlusion/return sequence."""
import math

import pytest

from stack_avoid.path_planner import AvoidPathPlanner
from stack_avoid.station_path import to_local
from stack_avoid.surfaces import scan_surfaces
from test_surface_node import scene_scan


def observation(pose):
    edges = [((2.76, -.2), (2.76, .2))]
    local_edges = [[(to_local((*p, 0., 0.), pose)[0]-.76,
                     to_local((*p, 0., 0.), pose)[1]) for p in edge] for edge in edges]
    msg = scene_scan(local_edges)
    scan = dict(ranges=msg.ranges, angle_min=msg.angle_min, increment=msg.angle_increment,
                range_min=msg.range_min, range_max=12., front_center=0., half_angle=math.pi/2)
    surfaces = scan_surfaces(msg.ranges, msg.angle_min, msg.angle_increment, .05, 12.,
                             0., math.pi/2, .76, 0., .1, 3., .3)
    return surfaces, scan


def planner():
    return AvoidPathPlanner(width=.62, length=.85, front=.76, margin=.15, min_radius=1.15)


def step(p, pose, generation, *, gps=True, detected=False, unknown=False):
    surfaces, scan = observation(pose)
    if unknown:
        scan['ranges'] = [math.nan]*len(scan['ranges'])
    goal = to_local((pose[0]+2.5, 0., 0., 0.), pose) if gps else None
    return p.step(pose=pose, surfaces=surfaces, scan=scan, lidar=(.76, 0.),
                  goals=[(2.76, .76)], detected=detected, gps_goal=goal,
                  v_ref=1., sample_time=.1, generation=generation)


def test_occluded_obstacle_first_gets_bypass_and_straight_tail():
    p = planner()
    target, done = step(p, (0., 0., 0.), 1, detected=True)
    assert target is not None and not done
    assert p.mode == 'bypass' and p.return_station is None
    assert target[0] < 1. and target[1] > 0
    assert p.path.points[-1][2] == pytest.approx(0.)
    assert p.path.points[-1][1] == pytest.approx(.76)


def test_unknown_space_cannot_certify_return_and_station_keeps_its_origin():
    p = planner()
    step(p, (0., 0., 0.), 1, detected=True)
    original_prefix = p.path.points[:3]
    for generation in range(2, 35):
        pose = p.path.at(p.path.station+.1)[:3]
        previous = p.path.station
        target, done = step(p, pose, generation, unknown=True)
        assert not done and p.mode != 'return'
        assert target is not None
        assert previous <= p.path.station <= previous+.6+1e-6
    assert p.path.station > 3.
    assert p.path.points[:3] == original_prefix


def test_visible_return_is_latched_followed_and_finishes_only_at_station():
    p = planner()
    step(p, (0., 0., 0.), 1, detected=True)
    return_station = None
    for generation in range(2, 110):
        pose = p.path.at(p.path.station+.1)[:3]
        target, done = step(p, pose, generation)
        assert target is not None, (generation, p.reason)
        if p.mode == 'return':
            if return_station is None:
                return_station = p.return_station
                assert p.path.station < return_station-p.return_tolerance
                assert not done
            else:
                assert p.return_station == return_station
        if done:
            assert return_station is not None
            assert p.path.station >= return_station-p.return_tolerance
            break
    else:
        pytest.fail(f'return never completed: {p.mode} {p.reason}')


def test_no_gps_does_not_turn_or_finish_on_elapsed_time():
    p = planner()
    step(p, (0., 0., 0.), 1, detected=True)
    for generation in range(2, 65):
        pose = p.path.at(p.path.station+.1)[:3]
        point, done = step(p, pose, generation, gps=False)
        assert point is not None and not done and p.mode != 'return'
    assert p.path.station > 6.
    assert p.path.points[-1][1] == pytest.approx(.76)


def test_pose_loss_does_not_advance_existing_station():
    p = planner()
    step(p, (0., 0., 0.), 1, detected=True)
    point, done = p.step(pose=None, surfaces=[], scan={}, lidar=(.76, 0.), goals=[],
                         detected=False, gps_goal=None, v_ref=1., sample_time=.1, generation=2)
    assert point is None and not done and p.path.station == 0.


@pytest.mark.parametrize('loss', ['gps', 'visibility', 'obstacle'])
def test_active_return_is_revoked_when_its_clearance_is_lost(loss):
    from stack_avoid.surfaces import Surface
    p = planner()
    step(p, (0., 0., 0.), 1, detected=True)
    for generation in range(2, 90):
        pose = p.path.at(p.path.station+.1)[:3]
        step(p, pose, generation)
        if p.mode == 'return':
            break
    assert p.mode == 'return'
    surfaces, scan = observation(pose)
    goal = to_local((pose[0]+2.5, 0., 0., 0.), pose)
    if loss == 'gps':
        goal = None
    elif loss == 'visibility':
        scan['ranges'] = [math.nan]*len(scan['ranges'])
    else:
        # A newly measured wall intersects both the latched return and straight fallback.
        surfaces.append(Surface(((1.4, -2.), (1.4, 2.))))
    point, done = p.step(pose=pose, surfaces=surfaces, scan=scan, lidar=(.76, 0.),
                         goals=[], detected=False, gps_goal=goal, v_ref=1.,
                         sample_time=.1, generation=generation+1)
    assert not done and p.mode != 'return' and p.return_station is None
    if loss == 'obstacle':
        assert point is None
