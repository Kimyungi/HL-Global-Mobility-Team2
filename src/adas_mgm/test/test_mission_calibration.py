import csv
import importlib.util
import json
from pathlib import Path
import pytest

PATH = Path(__file__).parents[1] / 'tools/analyze_mission_calibration.py'
spec = importlib.util.spec_from_file_location('mission_calibration', PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def row(request, event, elapsed, distance, kind=1, reason=0):
    return dict(event=event, request_id=request, mission_id=8, mission_type=kind,
                source_zone_id=10, time_ns=int((100 + elapsed)*1e9), position_valid=1,
                x=2, y=3, track_index=12, speed_valid=1, actual_speed=.2,
                zone_id=1, travel_distance=distance, elapsed_s=elapsed, cancel_reason=reason)


def write(tmp_path, rows):
    path = tmp_path / 'events.csv'
    with path.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader(); writer.writerows(rows)
    return path


def test_p6_elapsed_distance_statistics_and_outcomes(tmp_path):
    rows = []
    for request, elapsed, distance in ((100, 2, .4), (101, 6, 1.2)):
        rows += [row(request, 'zone_entry', 0, 0), row(request, 'search_start', .1, .02),
                 row(request, 'space_found', 1, .2), row(request, 'ready', elapsed, distance),
                 row(request, 'handoff', elapsed, distance), row(request, 'done', 10, distance)]
    rows += [row(102, 'zone_entry', 0, 0), row(102, 'cancel', 8, 1.6, reason=1),
             row(103, 'zone_entry', 0, 0, kind=2), row(103, 'cancel', 9, 2, kind=2, reason=2)]
    result = module.analyze([write(tmp_path, rows)])
    group = result['groups']['T_PARKING']
    assert group['outcomes'] == {'success': 2, 'timeout': 1}
    assert group['metrics']['entry_to_ready_s'] == {
        'count': 2, 'mean': 4, 'max': 6, 'p50': 4, 'p90': 5.6, 'p95': 5.8, 'p99': 5.96}
    assert group['metrics']['entry_to_handoff_m']['mean'] == pytest.approx(.8)
    assert result['groups']['PARALLEL_PARKING']['outcomes'] == {'distance_cancel': 1}
    assert result['operating_parameters_generated'] is False
    assert result['status'] == 'CALIBRATION_REQUIRED'
    assert 'CALIBRATION_REQUIRED' in module.markdown(result)
    assert result['requests'][0]['events']['ready']['track_index'] == 12


def test_missing_entry_and_sparse_data_do_not_become_zero_or_defaults(tmp_path):
    result = module.analyze([write(tmp_path, [row(100, 'ready', 2, .4)])])
    assert result['requests'][0]['missing_entry']
    assert result['groups']['T_PARKING']['metrics'] == {}
    assert result['groups']['T_PARKING']['outcomes'] == {'incomplete': 1}
    assert not result['operating_parameters_generated']


@pytest.mark.parametrize('bad', ['schema', 'conflicting_event', 'negative_distance', 'identity', 'both_terminal'])
def test_malformed_logs_are_rejected(tmp_path, bad):
    rows = [row(100, 'zone_entry', 0, 0)]
    if bad == 'schema':
        rows = [{'elapsed_s': 37.92, 'travel_distance': 2.188}]
    elif bad == 'conflicting_event':
        rows += [row(100, 'zone_entry', 1, 0)]
    elif bad == 'negative_distance':
        rows += [row(100, 'ready', 1, -1)]
    elif bad == 'identity':
        rows += [row(100, 'ready', 1, .2, kind=2)]
    else:
        rows += [row(100, 'done', 2, .2), row(100, 'cancel', 2, .2)]
    with pytest.raises(ValueError):
        module.analyze([write(tmp_path, rows)])


def test_monotonic_lifetime_not_ros_clock_and_invalid_position(tmp_path):
    rows = [row(100, 'zone_entry', 0, 0), row(100, 'ready', 2, .4)]
    rows[1].update(time_ns=10, position_valid=0, x=float('nan'), y=float('nan'))
    result = module.analyze([write(tmp_path, rows)])
    assert result['groups']['T_PARKING']['metrics']['entry_to_ready_s']['mean'] == 2
    assert result['requests'][0]['events']['ready']['x'] is None
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('reason,outcome', [(9, 'zone_exit_failure'), (10, 'route_end_failure')])
def test_mission_boundary_is_a_failure_not_success_or_timeout(tmp_path, reason, outcome):
    rows = [row(101, 'zone_entry', 0, 0), row(101, 'cancel', 10, 2, reason=reason)]
    result = module.analyze([write(tmp_path, rows)])
    assert result['groups']['T_PARKING']['outcomes'] == {outcome: 1}
    assert result['requests'][0]['cancel_reason'] == reason
