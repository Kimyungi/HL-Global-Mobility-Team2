import math

import pytest

from stack_avoid.station_path import (
    StationPath, connector, straight, to_world, to_local,
    footprint_clear, observed_free,
)
from stack_avoid.surfaces import Surface


def test_station_window_and_exact_arc_length_preview():
    path = StationPath(straight((0., 0., 0., 0.), 10., .1))
    path.update((4., 0.), 1., .1, 1)
    assert path.station == pytest.approx(.6)
    assert path.window == pytest.approx((0., .6))  # No old 1.5 multiplier.
    assert path.at(path.station+1.)[0] == pytest.approx(1.6)
    path.update((4., 0.), 1., .1, 1)
    assert path.station == pytest.approx(.6)
    path.update((4., 0.), 1., .1, 2)
    assert path.station == pytest.approx(1.2)


def test_self_crossing_cannot_jump_to_later_branch():
    path = StationPath([(0., 0., 0., 0.), (3., 0., 0., 0.), (3., 3., 0., 0.),
                        (0., 3., 0., 0.), (0., .01, 0., 0.)])
    path.update((0., .01), 1., .1, 1)
    assert path.station < .6 and path.index == 0


@pytest.mark.parametrize('speed', [0., math.nan, math.inf])
def test_unknown_or_zero_command_retains_half_meter_search(speed):
    path = StationPath(straight((0., 0., 0., 0.), 3., .1))
    path.update((2., 0.), speed, .1, 1)
    assert path.station == pytest.approx(.5)


def test_reverse_command_uses_distance_magnitude_and_bounded_backward_projection():
    path = StationPath(straight((0., 0., 0., 0.), 3., .1))
    path.station = 1.
    path.update((0., 0.), -1., .1, 1)
    assert path.station == pytest.approx(.4)


def test_replan_preserves_prefix_station_and_index_origin():
    path = StationPath(straight((0., 0., 0., 0.), 5., .1))
    path.update((.35, 0.), 1., .1, 1)
    old_station, prefix = path.station, path.points[:3]
    tail = connector(path.at(path.station), (3., .8, 0., 0.), .05)
    path.splice(tail)
    assert path.station == old_station
    assert path.points[:3] == prefix
    assert path.s[path.index] == pytest.approx(old_station)
    assert path.at(old_station+1.)[1] > 0


def test_preview_is_one_meter_on_a_bent_path_not_x_or_straight_line():
    path = StationPath([(0., 0., 0., 0.), (.5, 0., 0., 0.), (.5, 1., math.pi/2, 0.)])
    assert path.at(1.)[:2] == pytest.approx((.5, .5))
    assert math.hypot(*path.at(1.)[:2]) < 1.


def test_coordinate_round_trip_includes_yaw_and_preserves_curvature():
    p, pose = (1., .3, .2, .1), (12., -4., 1.1)
    assert to_local(to_world(p, pose), pose) == pytest.approx(p)


def test_connector_heading_and_curvature_boundaries():
    points = connector((0., 0., .1, 0.), (3., .7, -.1, 0.), .05)
    assert points[0] == pytest.approx((0., 0., .1, 0.))
    assert points[-1] == pytest.approx((3., .7, -.1, 0.))


def test_footprint_detects_front_corner_collision_with_clear_center():
    obstacle = [Surface(((.7, .3), (.7, .4)))]
    assert not footprint_clear([(0., 0., 0., 0.)], obstacle, .62, .76, .09, .15)
    assert footprint_clear([(0., 0., 0., 0.)], obstacle, .2, .3, .09, 0.)


def test_vectorized_footprint_matches_scalar_segment_clipping():
    import random
    from stack_avoid.surfaces import clip_segment
    rng = random.Random(92)
    for _ in range(200):
        pose = (rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-math.pi, math.pi), 0.)
        edge = tuple((rng.uniform(-3, 3), rng.uniform(-3, 3)) for _ in range(2))
        if rng.random() < .2:
            edge = (edge[0], edge[0])
        local = [to_local((*p, 0., 0.), pose)[:2] for p in edge]
        clipped = clip_segment(*local, 0, -.24, .91)
        collides = clipped is not None and clip_segment(*clipped, 1, -.46, .46) is not None
        assert footprint_clear([pose], [Surface(edge)], .62, .76, .09, .15) == (not collides)


def scan(ranges):
    return dict(ranges=ranges, angle_min=-math.pi/2, increment=math.pi/180,
                range_min=.05, range_max=12., front_center=0., half_angle=math.pi/2)


def test_return_needs_observed_free_space_not_nan_or_occluded_surface():
    path = straight((0., 0., 0., 0.), 2., .1)
    assert observed_free(path, scan([math.inf]*181), (.76, 0.), .46, .76, .025)
    assert not observed_free(path, scan([math.nan]*181), (.76, 0.), .46, .76, .025)
    assert not observed_free(path, scan([.8]*181), (.76, 0.), .46, .76, .025)
