"""Configured map identities must survive selected-route composition."""
from pathlib import Path
import pytest
import yaml
from stack_gps.path_engine import PathEngine
from stack_gps.route_plan import RoutePlan
from stack_gps.zones import (ZoneDefinition, ZoneMap, ZoneType, MissionType,
                             load_zone_definitions, turn_zone_map)

WAYPOINTS = Path(__file__).parents[1] / 'waypoints'


def plan_for(tmp_path, start):
    catalog = yaml.safe_load((WAYPOINTS / 'halla_route_sequence.yaml').read_text())
    ids = [start, *[f'{i:02}' for i in range(max(3, int(start) + 1), 7)]]
    entries = []
    for route_id in ids:
        row = next(dict(r) for r in catalog['routes'] if r['id'] == route_id)
        row['file'] = str(WAYPOINTS / row['file'])
        row['zones_file'] = str(WAYPOINTS / row['zones_file'])
        entries.append(row)
    path = tmp_path / 'selected.yaml'
    path.write_text(yaml.safe_dump({'routes': entries}))
    return RoutePlan(path)


@pytest.mark.parametrize('start', ['01', '02', '03', '04', '05', '06', '07'])
def test_halla_traffic_id_stays_three_for_every_start(tmp_path, start):
    plan = plan_for(tmp_path, start)
    def factory(files):
        engine = PathEngine(files.points, parking_ranges=[(2, 4)] if files.id == '03' else [])
        zones = ZoneMap.from_engine(engine, load_zone_definitions(files.zones, engine, 3.))
        return engine, turn_zone_map(files.zones, engine, 3., zones)
    plan.bind(factory)
    traffic = [z for zones in plan.zone_maps for z in zones.definitions
               if z.zone_type == ZoneType.GPS_ONLY_ZONE]
    assert traffic and {z.zone_id for z in traffic} == {3}
    assert all(z.explicit_id for z in traffic)
    assert all(z.zone_id != 3 for zones in plan.zone_maps for z in zones.definitions
               if z.zone_type == ZoneType.MISSION_ZONE)


def test_distinct_signal_ids_and_mission_ids_survive_composition(tmp_path):
    plan = plan_for(tmp_path, '03')
    def factory(files):
        engine = PathEngine(files.points, parking_ranges=[(6, 8)])
        zones = ZoneMap.from_engine(engine, [
            ZoneDefinition(2, ZoneType.GPS_ONLY_ZONE, 0, 1, explicit_id=True),
            ZoneDefinition(3, ZoneType.GPS_ONLY_ZONE, 2, 3, explicit_id=True)])
        return engine, zones
    plan.bind(factory)
    generated = []
    for zones in plan.zone_maps:
        assert [z.zone_id for z, inside in zones.snapshot(0) if inside] == [2]
        assert [z.zone_id for z, inside in zones.snapshot(2) if inside] == [3]
        generated.extend(z.zone_id for z in zones.definitions if not z.explicit_id)
    assert len(set(generated)) == len(generated)
    assert not {2, 3}.intersection(generated)
    assert plan.required == [(i,) for i in range(len(plan.files))]


def test_same_configured_id_cannot_change_type_between_routes(tmp_path):
    plan = plan_for(tmp_path, '03')
    def factory(files):
        engine = PathEngine(files.points)
        kind = ZoneType.GPS_ONLY_ZONE if files.id == '03' else ZoneType.NORMAL_ZONE
        return engine, ZoneMap([ZoneDefinition(3, kind, 0, 1, explicit_id=True)], len(files.points))
    with pytest.raises(ValueError, match='conflicting types'):
        plan.bind(factory)
