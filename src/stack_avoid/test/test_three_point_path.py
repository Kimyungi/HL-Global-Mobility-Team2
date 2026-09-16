import math

import pytest
from stack_avoid.compute_backend import compute_functions
from stack_avoid.three_point_path import ThreePointPlanner
from stack_avoid.station_path import to_local, to_world, transform_surfaces
from stack_avoid.surfaces import Surface


@pytest.fixture(params=['python', 'native'])
def planner(request):
    connector, checker = compute_functions(request.param)
    return ThreePointPlanner(width=.62, length=.85, front=.76, margin=.15,
        min_radius=1.15, connector=connector, collision_check=checker)


def create(p, frame=(0., 0., 0.)):
    assert p.create((2.66, .66), frame, [Surface(((2.66, -.2), (2.66, .2)))], 1_000_000_000, 1.6)


def test_three_anchors_forward_and_footprint_checked(planner):
    create(planner)
    a, b, c = planner.anchors
    assert a == (0., 0., 0., 0.)
    assert b[0] == 2.66 and b[1] >= .66
    assert c == (5.36, 0., 0., 0.)
    assert any(math.dist(p[:2], b[:2]) < 1e-8 for p in planner.path.points)
    assert planner.path.points[-1][:3] == pytest.approx(c[:3])
    assert planner._safe(planner.path.points, [Surface(((2.66, -.2), (2.66, .2)))])


def test_fixed_frame_curve_tracking_loss_and_completion(planner):
    frame = (14., -7., 1.1)
    create(planner, frame)
    frozen = tuple(planner.path.points)
    assert to_local(frozen[-1], frame)[:3] == pytest.approx((5.36, 0., 0.))
    stamp = 1_000_000_000
    target, done, points = planner.track(frame, stamp, 2., [], False)
    assert target is not None and not done and len(points) > 50
    # A different detection must not move C or replace the path.
    assert planner.create((3., -1.), (15., -6., 1.2), [], stamp, 1.6)
    assert tuple(planner.path.points) == frozen
    steps = math.ceil(planner.path.s[-1]/.2)
    for i in range(1, steps+1):
        point = planner.path.at(planner.path.s[-1]*i/steps)
        stamp += 100_000_000
        target, done, _ = planner.track(point[:3], stamp, 2., [], False)
        if i < steps-1:
            assert not done and target is not None
    assert done


def test_reverse_and_invalid_pose_keep_fixed_geometry(planner):
    create(planner)
    frozen = tuple(planner.path.points)
    for i in range(1, 5):
        pose = planner.path.at(i*.2)[:3]
        planner.track(pose, 1_000_000_000+i*100_000_000, 2., [], True)
    forward = planner.path.station
    planner.track(planner.path.at(.6)[:3], 1_500_000_000, -.8, [], True)
    assert planner.path.station < forward
    assert tuple(planner.path.points) == frozen
    assert planner.track(None, 1_600_000_000, 0., [], False) == (None, False, [])


def test_new_obstacle_keeps_reference_without_retarget(planner):
    create(planner)
    frozen = tuple(planner.path.points)
    target, done, path = planner.track((0., 0., 0.), 1_100_000_000, 0.,
        [Surface(((1., -5.), (1., 5.)))], True)
    assert target is not None and not done and path
    assert tuple(planner.path.points) == frozen


def test_unfeasible_curve_does_not_publish_or_latch(planner):
    assert not planner.create((2.66, .66), (0., 0., 0.),
        [Surface(((1., -5.), (1., 5.)))], 1_000_000_000, 1.6)
    assert planner.path is None and planner.anchors is None


def test_pose_jump_and_reset(planner):
    create(planner)
    assert planner.track((100., 0., 0.), 1_100_000_000, 2., [], False) == (None, False, [])
    planner.reset()
    assert planner.path is None and planner.anchors is None and not planner.finished


def test_lateral_error_does_not_invalidate_fixed_path(planner):
    create(planner)
    frozen = tuple(planner.path.points)
    target, done, path = planner.track((0., -.7, 0.), 1_100_000_000, 0., [], True)
    assert target is not None and path and not done
    assert tuple(planner.path.points) == frozen
