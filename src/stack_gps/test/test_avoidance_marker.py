from pathlib import Path

import pytest

from stack_gps.path_engine import PathEngine, avoidance_marker_range, load_waypoints_csv


WAYPOINTS = Path(__file__).parents[1] / 'waypoints'


def test_latest_halla_state_four_starts_only_path_four_at_index_55():
    for route in range(1, 8):
        csv_path = WAYPOINTS / f'waypoints_halla_reference_path_{route:02}.csv'
        intervals = avoidance_marker_range(csv_path)
        assert intervals == ([(55, 191)] if route == 4 else [])
        if route == 4:
            engine = PathEngine(load_waypoints_csv(csv_path), avoid_ranges=intervals)
            assert not engine._in_ranges(54, engine.avoid_ranges)
            assert engine._in_ranges(55, engine.avoid_ranges)
            assert engine._in_ranges(80, engine.avoid_ranges)


def test_marker_indices_follow_quality_filter_and_duplicate_removal(tmp_path):
    path = tmp_path / 'track.csv'
    path.write_text('lat,lon,quality,state\n37,127,5,4\n37,127,4,0\n'
                    '37,127,4,4\n37.00001,127,4,0\n')
    assert avoidance_marker_range(path) == [(0, 1)]


def test_unmarked_csv_keeps_existing_geometry(tmp_path):
    path = tmp_path / 'track.csv'
    path.write_text('lat,lon\n37,127\n37.00001,127\n')
    assert avoidance_marker_range(path) == []
    assert load_waypoints_csv(path) == [(37., 127.), (37.00001, 127.)]


def test_ambiguous_multiple_markers_do_not_silently_drop_the_second(tmp_path):
    path = tmp_path / 'track.csv'
    path.write_text('lat,lon,state\n37,127,4\n37.00001,127,4\n')
    with pytest.raises(ValueError, match='multiple state=4'):
        avoidance_marker_range(path)
