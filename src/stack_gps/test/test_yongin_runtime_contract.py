"""Yongin CSV metadata must survive the production node's zone loader."""
import csv
from pathlib import Path
from types import SimpleNamespace as NS
from stack_gps.node import StackGpsNode
from stack_gps.path_engine import PathEngine
from stack_gps.route_plan import RoutePlan
from stack_gps.zones import MissionType, ZoneType

DATA = Path(__file__).parents[1] / 'waypoints'


def plan():
    result = RoutePlan(DATA/'yongin_route_sequence.yaml', '01', '06')
    def factory(route):
        params = dict(waypoint_csv=str(route.csv), zones_file=str(route.zones),
                      stop_zone_snap_max_m=3., stop_zone_span_m=1., parking_zone_span_m=1.,
                      stop_points_latlon='', avoid_zone_latlon='', gps_only_zone_latlon='',
                      avoid_zone_lead_m=5.)
        def fail(msg): raise AssertionError(msg)
        node = NS(engine=PathEngine(route.points), turn_zone_policy=True,
                  get_logger=lambda: NS(info=lambda _:None, warn=lambda _:None, error=fail))
        StackGpsNode._setup_zones(node, lambda k:NS(value=params[k]))
        return node.engine,node.zone_map
    result.bind(factory)
    return result


def test_csv_zones_and_mission_modes():
    p=plan()
    assert p.exit_branches == {'source':'05','left':'06','right':'07'}
    for route,engine,zones in zip(p.files,p.engines,p.zone_maps):
        rows=list(csv.DictReader(route.csv.open()))
        for zone_id in (1,3):
            expected={i for i,r in enumerate(rows) if int(r['zone_id'])==zone_id and int(r['inside_zone'])}
            actual={i for i in range(len(rows)) if any(z.zone_id==zone_id and active for z,active in zones.snapshot(i))}
            assert expected==actual
        for state,mode in [(1,MissionType.T_PARKING),(2,MissionType.PARALLEL_PARKING)]:
            markers=[i for i,r in enumerate(rows) if int(r['state'])==state]
            configured=[z for z in zones.definitions if z.mission_type==mode]
            assert len(markers)==len(configured)
            for i in markers: assert any(z.contains(i) for z in configured)
    route3=next(z for r,z in zip(p.files,p.zone_maps) if r.id=='03')
    assert not any(z.zone_id==3 and active for z,active in route3.snapshot(300,302))
    assert any(z.zone_id==3 and active for z,active in route3.snapshot(564,565))

    assert any(z.zone_id==1 and active for z,active in route3.snapshot(336,340))
    assert not any(z.zone_id==1 and active for z,active in route3.snapshot(450,452))
    assert any(z.zone_id==1 and active for z,active in route3.snapshot(462,463))
