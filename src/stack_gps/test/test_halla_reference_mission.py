import csv
import math
from pathlib import Path

import yaml


WAYPOINT_DIR = Path(__file__).parents[1] / 'waypoints'
LAT0 = 37.3041743
LON0 = 127.9075328
ZONES = {
    1: (-70.0, -57.8346, -80.0, -66.5645),
    2: (-43.1667, -38.1339, -50.1861, -40.1591),
    3: (-20.0, -5.16, -21.0, -9.58929),
}


def _load_path(path_id):
    path = WAYPOINT_DIR / f'waypoints_halla_reference_path_{path_id:02d}.csv'
    with path.open(newline='', encoding='utf-8-sig') as stream:
        return list(csv.DictReader(stream))


def _enu_to_latlon(east, north):
    a = 6378137.0
    e2 = 6.69437999014e-3
    sin_lat = math.sin(math.radians(LAT0))
    n_radius = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    m_radius = a * (1.0 - e2) / (1.0 - e2 * sin_lat * sin_lat) ** 1.5
    return (
        LAT0 + math.degrees(north / m_radius),
        LON0 + math.degrees(east / (n_radius * math.cos(math.radians(LAT0)))),
    )


def _xy(row):
    return float(row['east_m']), float(row['north_m'])


def test_all_seven_paths_have_consistent_latlon_and_zone_flags():
    for path_id in range(1, 8):
        rows = _load_path(path_id)
        assert len(rows) >= 10
        assert all(row['lat'] and row['lon'] for row in rows)
        for row in rows:
            east, north = _xy(row)
            lat, lon = _enu_to_latlon(east, north)
            assert abs(float(row['lat']) - lat) < 1e-7
            assert abs(float(row['lon']) - lon) < 1e-7
            expected_zone = 0
            for zone_id, (emin, emax, nmin, nmax) in ZONES.items():
                if emin <= east <= emax and nmin <= north <= nmax:
                    expected_zone = zone_id
            assert int(row['zone_id']) == expected_zone
            assert int(row['inside_zone']) == int(expected_zone != 0)


def test_path_graph_connects_at_exact_endpoints():
    paths = {path_id: _load_path(path_id) for path_id in range(1, 8)}
    assert _xy(paths[1][-1]) == _xy(paths[2][-1]) == _xy(paths[3][0])
    assert _xy(paths[3][-1]) == _xy(paths[4][0])
    assert _xy(paths[4][-1]) == _xy(paths[5][0])
    assert _xy(paths[5][-1]) == _xy(paths[6][0]) == _xy(paths[7][0])
    assert _xy(paths[6][-1]) == _xy(paths[1][0])
    assert _xy(paths[7][-1]) == _xy(paths[2][0])


def test_path_2_is_waypoint_only():
    rows = _load_path(2)
    assert {row['drive_mode'] for row in rows} == {'waypoint_only'}
    zones_path = WAYPOINT_DIR / 'zones_halla_reference_path_02.yaml'
    with zones_path.open(encoding='utf-8') as stream:
        zones = yaml.safe_load(stream)
    assert len(zones['gps_only_zones']) == 1
    interval = zones['gps_only_zones'][0]
    assert abs(interval['start']['lat'] - float(rows[0]['lat'])) < 1e-8
    assert abs(interval['end']['lat'] - float(rows[-1]['lat'])) < 1e-8


def test_parking_markers_match_requested_positions():
    expected = {
        3: ('perpendicular', -31.8932, -33.4497),
        4: ('parallel', -62.7363, -65.911),
    }
    for path_id, (mode, east, north) in expected.items():
        marked = [row for row in _load_path(path_id) if row['parking_mode'] != 'none']
        assert len(marked) == 1
        assert marked[0]['parking_mode'] == mode
        actual_east, actual_north = _xy(marked[0])
        assert math.hypot(actual_east - east, actual_north - north) < 0.15


def test_route_zone_files_reference_the_matching_track():
    for path_id in range(1, 8):
        zone_path = WAYPOINT_DIR / f'zones_halla_reference_path_{path_id:02d}.yaml'
        with zone_path.open(encoding='utf-8') as stream:
            zones = yaml.safe_load(stream)
        assert zones['track'] == f'waypoints_halla_reference_path_{path_id:02d}.csv'
        assert isinstance(zones['gps_only_zones'], list)
        assert isinstance(zones['parking_points'], list)
