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
            # 현장에서 Path 3의 Zone 3 종료점을 idx 90→116으로 연장했다.
            if path_id == 3 and 28 <= int(row['idx']) <= 116:
                expected_zone = 3
            if path_id == 3 and 133 <= int(row['idx']) <= 145:
                expected_zone = 4
            assert int(row['zone_id']) == expected_zone
            assert int(row['inside_zone']) == int(expected_zone != 0)


def test_path_graph_connects_except_for_the_parking_handoff():
    paths = {path_id: _load_path(path_id) for path_id in range(1, 8)}
    assert _xy(paths[1][-1]) == _xy(paths[2][-1]) == _xy(paths[3][0])
    assert _xy(paths[3][-1]) == _xy(paths[4][0])
    # Path 4 finishes the parallel-parking maneuver. Path 5 deliberately
    # resumes from its separate post-parking start point.
    assert _xy(paths[4][-1]) == (-62.335, -72.648)
    assert _xy(paths[5][0]) == (-64.335, -70.648)
    assert _xy(paths[5][-1]) == _xy(paths[6][0]) == _xy(paths[7][0])
    assert _xy(paths[6][-1]) == _xy(paths[1][0])
    assert _xy(paths[7][-1]) == _xy(paths[2][0])


def test_updated_parking_control_points_are_sampled():
    path_4 = {_xy(row) for row in _load_path(4)}
    path_5 = {_xy(row) for row in _load_path(5)}
    assert {
        (-35.02, -36.958),
        (-64.335, -67.648),
        (-64.335, -70.648),
        (-62.335, -72.648),
    }.issubset(path_4)
    assert {
        (-64.335, -70.648),
        (-61.335, -70.648),
        (-43.0, -50.0),
    }.issubset(path_5)


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
        3: ('perpendicular', -29.231453, -30.797922),
        4: ('parallel', -63.127912, -66.384294),
    }
    for path_id, (mode, east, north) in expected.items():
        marked = [row for row in _load_path(path_id) if row['parking_mode'] != 'none']
        assert len(marked) == 1
        assert marked[0]['parking_mode'] == mode
        actual_east, actual_north = _xy(marked[0])
        assert math.hypot(actual_east - east, actual_north - north) < 1e-6
        zones_path = WAYPOINT_DIR / f'zones_halla_reference_path_{path_id:02d}.yaml'
        with zones_path.open(encoding='utf-8') as stream:
            parking_points = yaml.safe_load(stream)['parking_points']
        assert len(parking_points) == 1
        assert parking_points[0]['mode'] == mode
        assert abs(parking_points[0]['lat'] - float(marked[0]['lat'])) < 1e-8
        assert abs(parking_points[0]['lon'] - float(marked[0]['lon'])) < 1e-8


def test_mission_state_codes_and_signal_marker():
    expected = {
        1: (3, 145, 'perpendicular'),
        2: (4, 163, 'parallel'),
        3: (5, 300, 'none'),
    }
    marked = []
    for path_id in range(1, 8):
        for row in _load_path(path_id):
            state = int(row['state'])
            assert state in (0, 1, 2, 3)
            if state:
                marked.append((state, path_id, int(row['idx']), row['parking_mode']))
    assert sorted(marked) == [
        (state, path_id, index, mode)
        for state, (path_id, index, mode) in expected.items()
    ]


def test_path_3_zone_3_extension_matches_yaml():
    rows = _load_path(3)
    zone_3 = [row for row in rows if row['zone_id'] == '3']
    assert int(zone_3[0]['idx']) == 28
    assert int(zone_3[-1]['idx']) == 116
    assert rows[117]['zone_id'] == '0'

    zone_path = WAYPOINT_DIR / 'zones_halla_reference_path_03.yaml'
    with zone_path.open(encoding='utf-8') as stream:
        zones = yaml.safe_load(stream)
    interval = zones['gps_only_zones'][0]
    assert abs(interval['start']['lat'] - float(rows[28]['lat'])) < 1e-8
    assert abs(interval['end']['lat'] - float(rows[116]['lat'])) < 1e-8

    approach = [row for row in rows if row['zone_id'] == '4']
    assert [int(row['idx']) for row in approach] == list(range(133, 146))
    assert all(row['drive_mode'] == 'waypoint_only' for row in approach)
    assert abs(float(rows[145]['s_m']) - float(rows[133]['s_m']) - 3.) < .025
    interval = zones['gps_only_zones'][1]
    for side, idx in [('start', 133), ('end', 145)]:
        for coord in ('lat', 'lon'):
            assert abs(interval[side][coord] - float(rows[idx][coord])) < 1e-8
    assert rows[132]['zone_id'] == rows[146]['zone_id'] == '0'


def test_mission_yaml_state_definitions_match_csv():
    mission_path = WAYPOINT_DIR / 'halla_reference_mission.yaml'
    with mission_path.open(encoding='utf-8') as stream:
        mission = yaml.safe_load(stream)
    assert mission['state_definitions'] == {
        0: 'normal',
        1: 'perpendicular_parking',
        2: 'parallel_parking',
        3: 'traffic_signal',
    }
    assert [(point['state'], point['path_id'], point['idx'])
            for point in mission['state_points']] == [
                (1, 3, 145),
                (2, 4, 163),
                (3, 5, 300),
            ]


def test_route_zone_files_reference_the_matching_track():
    for path_id in range(1, 8):
        zone_path = WAYPOINT_DIR / f'zones_halla_reference_path_{path_id:02d}.yaml'
        with zone_path.open(encoding='utf-8') as stream:
            zones = yaml.safe_load(stream)
        assert zones['track'] == f'waypoints_halla_reference_path_{path_id:02d}.csv'
        assert isinstance(zones['gps_only_zones'], list)
        assert isinstance(zones['parking_points'], list)
