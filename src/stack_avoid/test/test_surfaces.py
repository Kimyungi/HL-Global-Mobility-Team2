"""Continuous obstacle geometry regressions; no ROS or vehicle needed."""
import math

import pytest

from stack_avoid.surfaces import (
    Surface, scan_surfaces, nearest_in_corridor, blocked_intervals, gap_centers,
    outside_intervals, surface_clearance, occluded, surface_intervals, surface_goal_sections,
)


def build(ranges, start=-0.02, step=0.01, **kwargs):
    options = dict(range_min=0.05, range_max=12.0, front_center=0.0,
                   half_angle=math.pi / 2, lidar_x=0.76, lidar_y=0.0,
                   link_distance=0.10, link_scale=3.0, max_link=0.30)
    options.update(kwargs)
    return scan_surfaces(ranges, start, step, **options)


def test_neighbor_returns_form_continuous_surface():
    surfaces = build([2.0] * 5)
    assert len(surfaces) == 1
    assert len(list(surfaces[0].segments())) == 4
    assert surface_clearance((2.76, 0.01), surfaces) < 0.001


@pytest.mark.parametrize('invalid', [math.inf, math.nan, 0.0, 13.0])
def test_invalid_ray_breaks_surface_without_dropping_small_obstacles(invalid):
    surfaces = build([2.0, 2.0, invalid, 2.0, 2.0])
    assert [len(s.points) for s in surfaces] == [2, 2]


def test_range_discontinuity_does_not_join_distinct_objects():
    assert [len(s.points) for s in build([2., 2., 4., 4., 2.])] == [2, 2, 1]


def test_resolution_adjustment_is_bounded():
    assert len(build([8., 8.], step=0.02)) == 1  # 16cm beam spacing
    assert len(build([8., 8.], step=0.1)) == 2  # 80cm must remain separate


def test_full_scan_seam_merges_front_contour_and_negative_increment_matches():
    ranges = [math.inf] * 360
    ranges[-1] = ranges[0] = ranges[1] = 2.0
    positive = build(ranges, start=0., step=math.tau / 360)
    negative = build(list(reversed(ranges)), start=math.tau * 359 / 360,
                     step=-math.tau / 360)
    assert len(positive) == len(negative) == 1
    assert len(positive[0].points) == len(negative[0].points) == 3
    # Equal-depth left/right endpoints may win the tie in either scan order.
    assert nearest_in_corridor(positive, .76, .46)[0] == pytest.approx(
        nearest_in_corridor(negative, .76, .46)[0])
    for a, b in zip(positive[0].points, reversed(negative[0].points)):
        assert a == pytest.approx(b)


def test_partial_scan_must_not_join_array_ends():
    surfaces = build([2., math.inf, 2.], start=-.01, step=.01)
    assert len(surfaces) == 2


def test_sensor_forward_angle_and_lateral_offset():
    surfaces = build([2.], start=math.radians(87), front_center=math.radians(87),
                     lidar_y=.2)
    assert surfaces[0].points[0] == pytest.approx((2.76, .2))
    assert nearest_in_corridor(surfaces, .76, .46) == pytest.approx((2., .2))


def test_detection_interpolates_surface_crossing_corridor():
    # Neither endpoint is inside this narrow corridor; the edge crosses it.
    surfaces = [Surface(((2., -.1), (2.2, .1)))]
    assert nearest_in_corridor(surfaces, .76, .02) == pytest.approx((1.32, -.02))


def test_no_surface_behind_bumper_or_outside_corridor_is_detected():
    surfaces = [Surface(((.1, 0.), (.5, .1))), Surface(((2., 1.), (3., 1.)))]
    assert nearest_in_corridor(surfaces, .76, .46) is None


def test_diagonal_wall_keeps_extent_outside_depth_and_lateral_windows():
    wall = Surface(((1.5, 1.4), (3.2, .58), (4., .2)))
    intervals = blocked_intervals([wall], 3.02, 4.22, .46)
    assert intervals[0] == pytest.approx((-.26, 1.86))
    # Old depth clipping offered a false left edge around y=1.17.
    assert not outside_intervals(1.17, intervals)
    assert gap_centers(intervals, 1.6) == pytest.approx([-.26])


def test_wide_front_wall_has_no_way_around_inside_offset_limit():
    wall = Surface(((2.76, -3.), (2.76, 3.)))
    assert gap_centers(blocked_intervals([wall], 2.16, 3.36, .46), 1.6) == []


@pytest.mark.parametrize('gap, passable', [(.90, False), (.92, True), (1.20, True)])
def test_passage_uses_vehicle_width_and_both_margins(gap, passable):
    walls = [Surface(((2.76, -3.), (2.76, -gap / 2))),
             Surface(((2.76, gap / 2), (2.76, 3.)))]
    candidates = gap_centers(blocked_intervals(walls, 2.16, 3.36, .46), 1.6)
    assert bool(candidates) is passable
    if passable:
        assert candidates == pytest.approx([0.])


def test_overlapping_surface_intervals_are_unioned():
    surfaces = [Surface(((2., -.2), (2., .2))), Surface(((2., .1), (2., .6)))]
    assert blocked_intervals(surfaces, 1., 3., .46)[0] == pytest.approx((-.66, 1.06))
    assert len(blocked_intervals(surfaces, 1., 3., .46)) == 1


def test_distant_side_wall_does_not_remove_reachable_outer_passage():
    assert gap_centers([(-.66, .66), (6.5, 7.5)], 1.6) == [-.66, .66]
    assert gap_centers([(-7.5, -6.5), (-.66, .66)], 1.6) == [-.66, .66]


def test_target_guard_interpolates_between_sparse_returns():
    wall = [Surface(((2., -.15), (2., .15)))]
    assert occluded((0., 0.), (3., 0.), wall, .1)
    assert not occluded((0., 0.), (1.5, 0.), wall, .1)
    assert surface_clearance((2.04, 0.), wall) == pytest.approx(.04)


def test_collinear_and_singleton_occlusion():
    assert occluded((0., 0.), (3., 0.), [Surface(((2., 0.),))], .1)
    assert occluded((0., 0.), (3., 0.), [Surface(((1., 0.), (4., 0.)))], .1)
    assert not occluded((0., 0.), (3., 0.), [Surface(((-2., 0.),))], .1)


@pytest.mark.parametrize('step', [0., math.nan, math.inf])
def test_invalid_scan_geometry_is_empty(step):
    assert build([2.], step=step) == []


def test_empty_scene():
    assert build([]) == build([math.inf] * 5) == []
    assert nearest_in_corridor([], .76, .46) is None
    assert surface_clearance((1., 0.), []) == math.inf


@pytest.mark.parametrize('angle', [0, 15, 45, 90, 135])
def test_inflated_face_slices_match_distance_to_segment(angle):
    yaw = math.radians(angle)
    face = Surface(((2., -.2), (2.+math.cos(yaw), -.2+math.sin(yaw))))
    # Independent distance query checks occupancy including the round ends.
    for i in range(41):
        x = 1.+i*.075
        intervals = surface_intervals([face], x, .46)
        for j in range(41):
            y = -1.5+j*.075
            distance = surface_clearance((x, y), [face])
            if abs(distance-.46) > 1e-8:
                assert (not outside_intervals(y, intervals)) == (distance < .46)


def test_face_end_clearance_is_round_and_two_faces_keep_real_gap():
    faces = [Surface(((2., -1.), (2.5, -.5))),
             Surface(((2., 1.), (2.5, .5)))]
    intervals = surface_intervals(faces, 2.8, .4)
    edge = .5-math.sqrt(.4**2-.3**2)
    assert intervals[0][1] == pytest.approx(-edge)
    assert intervals[1][0] == pytest.approx(edge)
    assert outside_intervals(0., intervals)
    assert surface_intervals([Surface(((2., 0.),))], 2., .4) == [(-.4, .4)]


def test_sections_use_each_face_extent_even_if_nearest_distance_is_same():
    near = Surface(((2., -.1), (2., .1)))
    slanted = Surface(((2.4, .8), (3., .4)))
    sections = surface_goal_sections([near, slanted], .76, 3.16, 2.96, .46)
    assert 3. in sections and 2.7 in sections and 2.4 in sections and 2. in sections
    assert sections[0] == 3.
    longer = Surface(((2.4, .8), (3.5, .4)))
    updated = surface_goal_sections([near, longer], .76, 3.16, 2.96, .46)
    assert nearest_in_corridor([near, slanted], .76, .5) == nearest_in_corridor([near, longer], .76, .5)
    assert updated[0] == 3.5 and updated != sections
    # The window edge is not an observed obstacle endpoint.
    assert 3.16 not in updated


def test_sections_exclude_unrelated_faces_and_do_not_join_gaps():
    faces = [Surface(((.1, 0.), (.3, .1))), Surface(((4., 0.), (5., .1))),
             Surface(((2., 4.), (3., 4.))), Surface(((2.5, 0.),))]
    assert surface_goal_sections(faces, .76, 3.16, 2.96, .46) == pytest.approx([2.5, 2.73, 2.96])
