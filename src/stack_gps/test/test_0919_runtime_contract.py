"""Verify the 0919 CSV through the same zone loader used by the GPS node."""
import csv
from pathlib import Path
from types import SimpleNamespace as NS

from stack_gps.node import StackGpsNode
from stack_gps.path_engine import PathEngine
from stack_gps.route_plan import RoutePlan
from stack_gps.zones import ZoneType

DATA = Path(__file__).parents[1] / 'waypoints'


def load_plan():
    plan = RoutePlan(DATA / 'halla_route_sequence.yaml', '01', '06')
    def factory(route):
        params = dict(waypoint_csv=str(route.csv), zones_file=str(route.zones),
                      stop_zone_snap_max_m=3., stop_zone_span_m=1., parking_zone_span_m=1.,
                      stop_points_latlon='', avoid_zone_latlon='', gps_only_zone_latlon='',
                      avoid_zone_lead_m=5.)
        def error(msg):
            raise AssertionError(msg)
        log = NS(info=lambda msg: None, warn=lambda msg: None, error=error)
        node = NS(engine=PathEngine(route.points), turn_zone_policy=True, get_logger=lambda: log)
        StackGpsNode._setup_zones(node, lambda key: NS(value=params[key]))
        return node.engine, node.zone_map
    plan.bind(factory)
    return plan


def test_runtime_traffic_bounds_equal_csv_and_old_early_entry_is_gone():
    plan = load_plan()
    for route, engine, zones in zip(plan.files, plan.engines, plan.zone_maps):
        rows = list(csv.DictReader(route.csv.open()))
        expected = {i for i, row in enumerate(rows)
                    if int(row['zone_id']) == 3 and int(row['inside_zone'])}
        actual = {i for i in range(len(rows)) if any(z.zone_id == 3 and active
                  for z, active in zones.snapshot(i))}
        assert actual == expected
        if route.id == '03':
            assert expected == set(range(75, 117))
            assert not any(z.zone_id == 3 and active for z, active in zones.snapshot(40, 50))
            assert any(z.zone_id == 3 and active for z, active in zones.snapshot(65, 75))
            assert engine.accel_ranges == [(117, 145)]
        assert [(a, b) for a, b in engine.physical_waypoint_ranges] == physical_ranges(rows)


def physical_ranges(rows):
    result = []
    for i, row in enumerate(rows):
        if not (int(row['zone_id']) and int(row['inside_zone'])):
            continue
        if result and result[-1][1] == i - 1:
            result[-1] = (result[-1][0], i)
        else:
            result.append((i, i))
    return result


def test_csv_state_missions_and_distinct_exit_zone_are_connected():
    plan = load_plan()
    indices = {route.id: i for i, route in enumerate(plan.files)}
    assert plan.exit_branches == {'source': '05', 'left': '06', 'right': '07'}
    assert plan.required[indices['03']]  # CSV state=1 still owns T parking.
    assert plan.required[indices['04']] == ()
    assert plan.engines[indices['04']].parallel_parking_ranges == []
    assert plan.engines[indices['04']].avoid_ranges == [(77, 205)]
    zones = plan.zone_maps[indices['05']]
    exit_zone = next(z for z in zones.definitions if z.zone_id == 2)
    assert exit_zone.zone_type == ZoneType.LAST_MISSION_ZONE
    assert (exit_zone.start_index, exit_zone.end_index) == (300, 316)
    assert not any(active for z, active in zones.snapshot(299, 300))
    assert any(z.zone_id == 2 and active for z, active in zones.snapshot(300))
