"""PR124 parking CSV through the actual GPS zone loader and route sequencer."""
import csv
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from stack_gps.no_parking_route import prepare_csv_catalog, prepare_no_parking_catalog
from stack_gps.node import StackGpsNode
from stack_gps.path_engine import PathEngine
from stack_gps.route_plan import RoutePlan
from stack_gps.zones import MissionType, ZoneType

SOURCE = Path(__file__).parents[1] / 'waypoints/yongin_0920.csv'


@pytest.mark.parametrize('start', ['01', '02'])
def test_parking_course_preserves_csv_and_all_mission_zones(tmp_path, start):
    catalog = prepare_csv_catalog(SOURCE, tmp_path/'route', parking=True)
    assert (catalog.parent/'source.csv').read_bytes() == SOURCE.read_bytes()
    meta = json.loads((catalog.parent/'source_metadata.json').read_text())
    assert meta['sha256'] == hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    assert meta['rows'] == 2584
    assert meta['estop_stations'] == [dict(route='04', first=621, last=828)]
    plan = RoutePlan(catalog, start, '06')
    def factory(route):
        values = dict(waypoint_csv=str(route.csv), zones_file=str(route.zones),
                      stop_zone_snap_max_m=3., stop_zone_span_m=1., parking_zone_span_m=1.,
                      stop_points_latlon='', avoid_zone_latlon='', gps_only_zone_latlon='',
                      avoid_zone_lead_m=5.)
        def fail(message):
            raise AssertionError(message)
        node = NS(engine=PathEngine(route.points), turn_zone_policy=True,
                  get_logger=lambda: NS(info=lambda _:None, warn=lambda _:None, error=fail))
        StackGpsNode._setup_zones(node, lambda k:NS(value=values[k]))
        return node.engine, node.zone_map
    plan.bind(factory)
    assert plan.exit_branches == dict(source='05',left='06',right='07')
    assert [len(missions) for missions in plan.required] == [0,1,1,0,0,0]
    originals = list(csv.DictReader(SOURCE.open()))
    for route, engine, zones in zip(plan.files, plan.engines, plan.zone_maps):
        rows = list(csv.DictReader(route.csv.open()))
        assert rows == [r for r in originals if int(r['path_id']) == int(route.id)]
        for marker, mode in ((1,MissionType.T_PARKING),(2,MissionType.PARALLEL_PARKING)):
            indices = [i for i,r in enumerate(rows) if int(r['state']) == marker]
            definitions = [z for z in zones.definitions if z.mission_type == mode]
            assert len(indices) == len(definitions)
            assert all(any(z.contains(i) for z in definitions) for i in indices)
        if route.id == '03':
            assert engine.avoid_ranges == [(337,462)]
        if route.id == '04':
            estop = [z for z in zones.definitions if z.zone_type == ZoneType.ESTOP_ZONE]
            assert len(estop) == 1 and estop[0].contains(621) and estop[0].contains(828)
            assert not estop[0].contains(620) and not estop[0].contains(829)
        if route.id == '05':
            assert engine.exit_stop_index == 92


def test_parking_csv_cannot_silently_select_no_parking(tmp_path):
    with pytest.raises(ValueError, match='without parking states'):
        prepare_no_parking_catalog(SOURCE, tmp_path/'route')
