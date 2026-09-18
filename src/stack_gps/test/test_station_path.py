"""Bounded station progression and one preview; no ROS or sensors."""
import math
from pathlib import Path

import pytest

from stack_gps.station_path import StationPath
from stack_gps.path_engine import PathEngine, M_PER_DEG_LAT, load_waypoints_csv
from stack_gps.route_plan import copy_geometry, RoutePlan
from stack_gps.zones import ZoneMap


def track(points, yaw=None, curvature=None):
    return StationPath([p[0] for p in points], [p[1] for p in points],
                       yaw or [0.] * len(points), curvature or [0.] * len(points))


def update(path, x, y=0., *, speed=1., period=.1, generation=1.):
    path.update(x, y, v_ref=speed, sample_time=period, generation=generation)


def test_initial_global_nearest_then_sparse_segment_progresses_with_small_window():
    path = track([(float(i), 0.) for i in range(21)])
    update(path, 10.1)
    assert path.index == 10
    assert path.station == pytest.approx(10.1)
    for k in range(1, 15):
        before = path.station
        update(path, 10.1 + .1*k, generation=1.+k*.1)
        assert path.station-before == pytest.approx(.1)
        assert path.window_high-before == pytest.approx(.65)
    assert path.index == 11
    assert path.station == pytest.approx(11.5)


def test_parallel_return_leg_nearby_cannot_steal_station():
    path = track([(0., 0.), (10., 0.), (10., .1), (0., .1)])
    update(path, 5., .01)
    update(path, 5.1, .1, generation=2.)  # now physically closer to the other leg
    assert path.station == pytest.approx(5.1)
    assert path.index in (0, 1)


def test_crossing_does_not_jump_to_later_segment_at_same_position():
    path = track([(-5., 0.), (0., 0.), (5., 0.), (5., 5.), (0., 0.), (-5., -5.)])
    update(path, -.1)
    update(path, .05, .05, generation=2.)
    assert path.station == pytest.approx(5.05)
    assert path.index == 1


@pytest.mark.parametrize('speed,period,expected_station', [(1., .1, 1.65), (2., .1, 1.8), (1., .2, 1.8)])
def test_large_position_jump_is_clipped_without_global_reacquisition(speed, period, expected_station):
    path = track([(0., 0.), (10., 0.), (20., 0.)])
    update(path, 1.)
    update(path, 19., speed=speed, period=period, generation=1000.)  # outage does not enlarge sample_time
    assert path.station == pytest.approx(expected_station)
    assert path.index == 0


def test_zero_speed_tracks_local_motion_and_index_with_half_meter_margin():
    path = track([(i * .25, 0.) for i in range(41)])
    update(path, 1., speed=0.)
    update(path, 1.25, speed=0., generation=2.)
    assert path.station == pytest.approx(1.25)
    assert path.index == 5
    assert (path.window_low, path.window_high) == pytest.approx((.5, 1.5))
    update(path, 9., speed=0., generation=3.)
    assert path.station == pytest.approx(1.75)
    assert path.index == 7
    update(path, 0., speed=0., generation=4.)
    assert path.station == pytest.approx(1.25)
    assert path.index == 5


@pytest.mark.parametrize('speed', [math.nan, math.inf, -math.inf])
def test_invalid_speed_freezes_station_and_index(speed):
    path = track([(0., 0.), (10., 0.), (20., 0.)])
    update(path, 1.)
    update(path, 19., speed=speed, generation=2.)
    assert path.station == pytest.approx(1.)
    assert path.index == 0


def test_reverse_uses_speed_magnitude_and_allows_local_backward_search():
    path = track([(0., 0.), (10., 0.)])
    update(path, 5.)
    update(path, 4., speed=-.5, period=.2, generation=2.)
    assert path.station == pytest.approx(4.35)


def test_duplicate_and_regressing_fix_do_not_slide_the_window():
    path = track([(0., 0.), (10., 0.)])
    update(path, 1., generation=10.)
    for generation in [10., 10., 9., 8., 10.]:
        update(path, 9., generation=generation)
        assert path.station == pytest.approx(1.)
    update(path, 9., generation=11.)
    assert path.station == pytest.approx(1.65)


def test_reset_allows_one_new_global_initialization():
    path = track([(0., 0.), (10., 0.), (20., 0.)])
    update(path, 1.)
    path.reset()
    update(path, 19., speed=0.)
    assert path.station == pytest.approx(19.)
    assert path.index == 2


@pytest.mark.parametrize('fraction,weight,nearest_index', [
    (.09, 0., 1), (.1, 0., 1), (.1001, .1001, 1), (.5, .5, 1),
    (.8999, .8999, 2), (.9, 1., 2), (.91, 1., 2)])
def test_preview_ninety_percent_boundaries(fraction, weight, nearest_index):
    # Starting at 5+fraction puts station+2.5 at fraction along the [7.5, 8.5] segment.
    path = track([(0., 0.), (7.5, 0.), (8.5, 0.), (10., 0.)],
                 yaw=[0., .2, .6, .8], curvature=[0., .1, .3, .4])
    update(path, 5.+fraction)
    point, diag = path.preview()
    assert point == pytest.approx((7.5+weight, 0., .2+.4*weight, .1+.2*weight))
    assert diag['preview_requested_station_m'] == pytest.approx(7.5+fraction)
    assert diag['preview_station_m'] == pytest.approx(7.5+weight)
    assert diag['preview_index'] == nearest_index
    assert diag['preview_snapped'] is (weight in (0., 1.))


def test_preview_uses_arc_length_around_corner_and_interpolates_all_fields():
    path = track([(0., 0.), (2., 0.), (2., 4.)],
                 yaw=[0., 0., math.pi/2], curvature=[0., .1, .3])
    update(path, 1.)
    point, diag = path.preview()
    assert diag['preview_requested_station_m'] == pytest.approx(3.5)
    assert point == pytest.approx((2., 1.5, .375*math.pi/2, .175))
    assert math.hypot(point[0]-1., point[1]) < 2.5  # chord is not station distance


def test_yaw_interpolation_wraps_across_pi_without_flipping_to_zero():
    path = track([(0., 0.), (5., 0.)], yaw=[math.radians(179.), math.radians(-179.)])
    update(path, 0.)
    point, _ = path.preview()
    assert abs(point[2]) == pytest.approx(math.pi)


def test_end_of_track_clamps_preview_and_zero_speed_window():
    path = track([(0., 0.), (1., 0.), (2., 0.)])
    update(path, 2.)
    update(path, 10., speed=0., generation=2.)
    point, diag = path.preview()
    assert point[:2] == (2., 0.)
    assert diag['preview_station_m'] == 2.
    assert (path.window_low, path.window_high) == pytest.approx((1.5, 2.))


@pytest.mark.parametrize('generation,period', [(None, .1), (math.nan, .1), (1., 0.), (1., -1.)])
def test_bad_sample_does_not_initialize(generation, period):
    path = track([(0., 0.), (10., 0.)])
    with pytest.raises(ValueError):
        update(path, 1., generation=generation, period=period)
    assert path.station is None


def latlon(x, y=0.):
    return 37.5 + y/M_PER_DEG_LAT, 127. + x/(M_PER_DEG_LAT*math.cos(math.radians(37.5)))


def test_engine_gps_only_uses_station_or_nearest_preview_but_parking_uses_station_only():
    eng = PathEngine([latlon(float(i)) for i in range(21)], station_tracking=True,
                     n_points=30, gps_only_ranges=[(7, 9)], parking_ranges=[(7, 9)])
    snap = eng.snapshot(*latlon(5.), heading=.2, v_ref=1., generation=1.)
    point, = snap['points']
    assert point == pytest.approx((2.5*math.cos(.2), -2.5*math.sin(.2), -.2, 0.), abs=1e-7)
    assert snap['idx'] == 5 and snap['preview_index'] == 7
    assert snap['gps_only_zone'] and not snap['parking_zone']
    assert snap['preview_station_m'] == pytest.approx(7.5, abs=1e-7)
    again = eng.snapshot(*latlon(15.), heading=.2, v_ref=1., generation=2.)
    assert again['idx'] == 6 and again['preview_index'] == 8
    assert again['gps_only_zone'] and not again['parking_zone']
    assert again['station_m'] == pytest.approx(snap['station_m']+.65, abs=1e-9)


def test_preloaded_route_copies_station_mode_and_keeps_history_independent():
    first = PathEngine([latlon(float(i)) for i in range(21)], station_tracking=True)
    other = copy_geometry(first, [latlon(float(i)) for i in range(10, 31)])
    first.snapshot(*latlon(5.), generation=1.)
    snap = other.snapshot(*latlon(25.), generation=1.)
    assert other.station_tracking
    assert snap['station_m'] == pytest.approx(15., abs=1e-7)
    assert first.station_path.station == pytest.approx(5., abs=1e-7)


@pytest.mark.parametrize('number', range(1, 8))
def test_uploaded_halla_csv_supports_single_station_preview(number):
    files = Path(__file__).parents[1] / 'waypoints'
    points = load_waypoints_csv(files / f'waypoints_halla_reference_path_{number:02}.csv')
    eng = PathEngine(points, station_tracking=True)
    for index in (0, len(points)//2, len(points)-1):
        eng.reset_station()
        snap = eng.snapshot(*points[index], heading=eng.yaw[index], generation=1.)
        point, = snap['points']
        assert all(math.isfinite(v) for v in point)
        assert snap['preview_requested_station_m'] == pytest.approx(
            min(eng.station_path.s[-1], snap['station_m']+2.5))


def test_accepted_route_handoff_resets_station_but_duplicate_command_does_not():
    files = Path(__file__).parents[1] / 'waypoints'
    plan = RoutePlan(files / 'halla_route_sequence.yaml', '01', '07')
    plan.bind(lambda route: (PathEngine(route.points, station_tracking=True), ZoneMap([], len(route.points))))
    destination = plan.engines[1]
    destination.snapshot(*plan.files[1].points[-1], generation=1.)
    assert destination.station_path.station is not None
    assert plan.apply(plan.sequence_id, 22, 22, 1, 1)
    assert plan.active_engine.station_path.station is None
    initial = plan.active_engine.snapshot(*plan.files[1].points[0], generation=2.)
    assert not plan.apply(plan.sequence_id, 22, 22, 1, 1)
    assert plan.active_engine.station_path.station == initial['station_m']


def test_station_heading_interpolates_wrap_without_preview_snap():
    path = track([(0., 0.), (10., 0.), (20., 0.)],
                 yaw=[math.radians(179), math.radians(-179), math.radians(-160)])
    update(path, 5.)
    assert abs(path.heading()) == pytest.approx(math.pi)
    assert abs(path.preview()[0][2]) != pytest.approx(abs(path.heading()))


def test_engine_station_error_is_separate_from_preview_yaw_and_rejects_fallback():
    eng = PathEngine([latlon(float(i)) for i in range(21)], station_tracking=True)
    snap = eng.snapshot(*latlon(5.), heading=.3, v_ref=1., generation=1.)
    assert snap['station_error_valid']
    assert snap['station_yaw_error_rad'] == pytest.approx(-.3, abs=1e-7)
    snap = eng.snapshot(*latlon(5.), heading=None, v_ref=1., generation=2.)
    assert not snap['station_error_valid']


@pytest.mark.parametrize('ranges', ['parking_ranges','parallel_parking_ranges','stop_ranges','avoid_ranges'])
def test_station_mission_range_blocks_preview_only_gps_zone(ranges):
    eng = PathEngine([latlon(float(i)) for i in range(21)], station_tracking=True,
                     gps_only_ranges=[(7,9)], **{ranges:[(4,6)]})
    snap = eng.snapshot(*latlon(5.), heading=0., generation=1.)
    assert snap['idx'] == 5 and snap['preview_index'] == 7
    assert not snap['gps_only_zone']
    assert snap['points'][0][0] == pytest.approx(2.5, abs=1e-7)


def test_speed_zone_preview_entry_station_hold_and_exit():
    def snapshot(x, **kwargs):
        engine = PathEngine([latlon(float(i)) for i in range(21)],
                            station_tracking=True, accel_ranges=[(7, 9)], **kwargs)
        return engine.snapshot(*latlon(x), heading=0., v_ref=2., generation=1.)
    assert not snapshot(4.)['accel_zone']
    assert snapshot(5.)['accel_zone']  # preview reaches index 7
    assert snapshot(9.)['accel_zone']  # station remains inside after preview exits
    assert not snapshot(10.)['accel_zone']
    assert not snapshot(5., gps_only_ranges=[(4, 6)])['accel_zone']  # station zone wins


def test_physical_zone_ranges_follow_filtered_geometry(tmp_path):
    from stack_gps.path_engine import csv_zone_ranges
    path = tmp_path / 'zones.csv'
    path.write_text('lat,lon,quality,zone_id\n37,127,4,0\n'
                    '37,127.001,5,4\n37,127.002,4,4\n'
                    '37,127.002,4,4\n37,127.003,4,4\n37,127.004,4,0\n')
    assert csv_zone_ranges(path, 4) == [(1, 2)]


def test_physical_csv_zone_forces_waypoint_from_preview_through_station_exit():
    def snapshot(x):
        engine = PathEngine([latlon(float(i)) for i in range(21)], station_tracking=True)
        engine.physical_waypoint_ranges = [(7, 9)]
        return engine.snapshot(*latlon(x), heading=0., v_ref=2., generation=1.)
    for x in (5., 7., 9.):
        snap = snapshot(x)
        assert snap['physical_gps_only'] and snap['gps_only_zone']
    assert not snapshot(10.)['gps_only_zone']


@pytest.mark.parametrize('zone_id', [1, 2, 3, 4, 9, 254])
def test_every_csv_zone_id_enables_waypoint_tracking(tmp_path, zone_id):
    from stack_gps.path_engine import csv_zone_ranges
    file = tmp_path / 'physical.csv'
    file.write_text(f'lat,lon,zone_id\n37,127,0\n37,127.001,{zone_id}\n37,127.002,{zone_id}\n')
    assert csv_zone_ranges(file) == [(1, 2)]
