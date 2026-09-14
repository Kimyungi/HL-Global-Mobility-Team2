import math
from pathlib import Path

import pytest

from stack_gps.station_path import StationPath
from stack_gps.path_engine import PathEngine, load_waypoints_csv


def test_station_window_preserves_csv_arc_lengths_and_exact_boundaries():
    t = StationPath([0., 3., 3., 9.], [0., 0., 4., 4.], [0.]*4, [0.]*4)
    t.station = 4.
    stations, points = t.window()
    assert stations == pytest.approx([2., 3., 7., 12.])
    assert points[0][:2] == pytest.approx((2., 0.))
    assert t.sample(5.)[:2] == pytest.approx((3., 2.))
    assert points[-1][:2] == pytest.approx((8., 4.))


def test_route_end_window_does_not_extrapolate():
    t = StationPath([0., 10.], [0., 0.], [0., 0.], [0., 0.])
    t.station = 9.5
    assert t.window()[0] == pytest.approx([7.5, 10.])
    with pytest.raises(ValueError): t.sample(10.01)


@pytest.mark.parametrize('route', ['01', '03', '04'])
def test_real_csv_waypoints_and_vehicle_heading_share_one_coordinate_transform(route):
    file = Path(__file__).resolve().parents[1]/'waypoints'/f'waypoints_halla_reference_path_{route}.csv'
    points = load_waypoints_csv(file)
    engine = PathEngine(points, station_tracking=True)
    lat, lon = points[min(10, len(points)-2)]
    heading = math.radians(-135.)
    snap = engine.snapshot(lat, lon, heading, generation=1., v_ref=1.)
    ev, nv = engine.to_enu(lat, lon)
    for station, p in zip(snap['waypoint_stations'], snap['waypoint_points']):
        world = engine.station_path.sample(station)
        east = ev+math.cos(heading)*p[0]-math.sin(heading)*p[1]
        north = nv+math.sin(heading)*p[0]+math.cos(heading)*p[1]
        assert (east, north) == pytest.approx(world[:2])
    assert snap['waypoint_stations'][0] <= snap['station_m'] <= snap['waypoint_stations'][-1]
