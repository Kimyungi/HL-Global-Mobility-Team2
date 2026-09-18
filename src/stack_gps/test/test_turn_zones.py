import yaml
import pytest
from stack_gps.path_engine import PathEngine
from stack_gps.zones import ZoneDefinition, ZoneType, MissionType, ZoneMap, turn_zone_map


def fixture():
    engine=PathEngine([(37.+i*.00001,127.) for i in range(20)],gps_only_ranges=[(0,19)],parking_ranges=[(10,12)])
    return engine, ZoneMap.from_engine(engine)


def test_missing_turns_excludes_legacy_and_preserves_parking(tmp_path):
    engine, old=fixture()
    path=tmp_path/'zones.yaml'; path.write_text('gps_only_zones: []\n')
    result=turn_zone_map(path,engine,2.,old)
    assert engine.gps_only_ranges==[]
    assert result.definitions==tuple(z for z in old.definitions if z.zone_type==ZoneType.MISSION_ZONE)


def test_explicit_shared_turns_replace_legacy_ranges(tmp_path):
    engine, old=fixture()
    path=tmp_path/'zones.yaml'; path.write_text(yaml.safe_dump({'turn_zones':[{'zone_id':7,'index_range':[3,6]}]}))
    result=turn_zone_map(path,engine,2.,old)
    assert engine.gps_only_ranges==[(3,6)]
    assert any(z.zone_id==7 and inside for z,inside in result.snapshot(4))
    assert not any(z.zone_type==ZoneType.GPS_ONLY_ZONE and inside for z,inside in result.snapshot(0))


def test_conflicting_mission_id_or_bad_bounds_rejected(tmp_path):
    engine, old=fixture()
    mission=next(z for z in old.definitions if z.zone_type==ZoneType.MISSION_ZONE)
    path=tmp_path/'zones.yaml'
    for entry in [{'zone_id':mission.zone_id,'index_range':[3,6]}, {'zone_id':7,'index_range':[3,60]}]:
        path.write_text(yaml.safe_dump({'turn_zones':[entry]}))
        with pytest.raises(ValueError): turn_zone_map(path,engine,2.,old)


def test_revised_gps_go_telemetry_uses_upper_controller():
    from types import SimpleNamespace
    from stack_gps.node import StackGpsNode
    state = SimpleNamespace(turn_zone_policy=True, _route_plan=None, _go_t=None)
    StackGpsNode._on_route_control(state, SimpleNamespace(go_authorized=True))
    assert state._go_t is not None
    StackGpsNode._on_route_control(state, SimpleNamespace(go_authorized=False))
    assert state._go_t is None


def test_halla_shared_zone_matches_csv_zone_3_and_keeps_missions():
    import csv
    from pathlib import Path
    root = Path(__file__).parents[1] / 'waypoints'
    for number in range(1, 8):
        csv_path = root / f'halla_0919_path_{number:02d}.csv'
        rows = list(csv.DictReader(csv_path.open()))
        engine = PathEngine([(float(r['lat']), float(r['lon'])) for r in rows])
        zone_path = root / f'zones_halla_20260916_path_{number:02d}.yaml'
        config = yaml.safe_load(zone_path.read_text())
        assert config['gps_only_zones'] == []
        for point in config['parking_points']:
            first, last, distance = engine.range_from_latlon(point['lat'], point['lon'], 1.)
            ranges = engine.parking_ranges if point['mode'] == 'perpendicular' else engine.parallel_parking_ranges
            ranges.append((first, last))
        existing = ZoneMap.from_engine(engine)
        result = turn_zone_map(zone_path, engine, 2., existing)
        expected = {i for i, row in enumerate(rows) if int(row['zone_id']) == 3 and int(row['inside_zone']) == 1}
        actual = {i for i in range(len(rows)) if any(z.zone_type == ZoneType.GPS_ONLY_ZONE and inside for z, inside in result.snapshot(i))}
        assert actual == expected, number
        assert [z for z in result.definitions if z.zone_type == ZoneType.MISSION_ZONE] == list(existing.definitions)
