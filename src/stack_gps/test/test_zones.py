"""Zone metadata/classification uses synthetic GPS tracks, no vehicle boundaries."""
import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stack_gps.path_engine import PathEngine
from stack_gps.zones import MissionType, ZoneDefinition, ZoneMap, ZoneType, load_zone_definitions
from test_path_engine import en_to_latlon, make_track


def engine(**kwargs):
    return PathEngine(make_track([(i * .2, 0) for i in range(100)]), **kwargs)


def test_existing_gps_and_parking_ranges_are_reused_and_overlap_retained():
    eng = engine(gps_only_ranges=[(10, 30)], parking_ranges=[(20, 25)],
                 parallel_parking_ranges=[(40, 50)])
    zones = ZoneMap.from_engine(eng)
    current = [(d, inside) for d, inside in zones.snapshot(22) if inside]
    assert {d.zone_type for d, _ in current} == {ZoneType.GPS_ONLY_ZONE, ZoneType.MISSION_ZONE}
    assert len({d.zone_id for d in zones.definitions}) == 3
    missions = [d for d in zones.definitions if d.zone_type == ZoneType.MISSION_ZONE]
    assert {d.mission_type for d in missions} == {MissionType.T_PARKING, MissionType.PARALLEL_PARKING}
    assert len({d.mission_id for d in missions}) == 2
    assert zones.definitions == ZoneMap.from_engine(eng).definitions
    assert not any(inside for _, inside in zones.snapshot(0))


def test_range_edges_are_inclusive_and_preview_extends_only_gps_only_zone():
    eng = engine(gps_only_ranges=[(10, 20)], parking_ranges=[(10, 20)], lookahead_m=4.)
    zones = ZoneMap.from_engine(eng)
    gps_only = next(z for z in zones.definitions if z.zone_type == ZoneType.GPS_ONLY_ZONE)
    mission = next(z for z in zones.definitions if z.zone_type == ZoneType.MISSION_ZONE)
    membership = dict(zones.snapshot(9, 10))
    assert membership[gps_only]
    assert not membership[mission]
    membership = dict(zones.snapshot(20, 21))
    assert membership[gps_only]
    assert membership[mission]
    membership = dict(zones.snapshot(21, 20))
    assert membership[gps_only]
    assert not membership[mission]
    membership = dict(zones.snapshot(9, 9))
    assert not membership[gps_only] and not membership[mission]
    snap = eng.snapshot(*en_to_latlon(0, 0), heading=0)
    assert snap['idx'] == 0
    assert snap['points'][0][0] > 0
    assert not any(inside for _, inside in zones.snapshot(snap['idx']))


def test_explicit_ids_replace_same_legacy_range_and_share_mission_memory_key():
    eng = engine(parking_ranges=[(10, 20)])
    explicit = [ZoneDefinition(50, ZoneType.MISSION_ZONE, 10, 20, 7, MissionType.T_PARKING),
                ZoneDefinition(60, ZoneType.MISSION_ZONE, 30, 40, 7, MissionType.T_PARKING)]
    zones = ZoneMap.from_engine(eng, explicit)
    assert zones.definitions == tuple(explicit)
    assert {d.mission_id for d in zones.definitions} == {7}


def test_explicit_yaml_indices_and_latlon_use_existing_snap_limit(tmp_path):
    eng = engine()
    start, end = en_to_latlon(2, 0), en_to_latlon(4, 0)
    path = tmp_path / 'zones.yaml'
    path.write_text(f'''zones:
- zone_id: 6
  zone_type: MISSION_ZONE
  mission_id: 11
  mission_type: T_PARKING
  index_range: [10, 20]
- zone_id: 8
  zone_type: GPS_ONLY_ZONE
  start: {{lat: {start[0]}, lon: {start[1]}}}
  end: {{lat: {end[0]}, lon: {end[1]}}}
''')
    definitions = load_zone_definitions(path, eng, 5.)
    assert [(d.zone_id, d.start_index, d.end_index) for d in definitions] == [(6, 10, 20), (8, 10, 20)]
    assert definitions[0].mission_id == 11
    assert definitions[0].mission_type == MissionType.T_PARKING
    far = en_to_latlon(2, 100)
    path.write_text(f'''zones:
- zone_id: 8
  zone_type: GPS_ONLY_ZONE
  start: {{lat: {far[0]}, lon: {far[1]}}}
  end: {{lat: {end[0]}, lon: {end[1]}}}
''')
    with pytest.raises(ValueError, match='snap_max'):
        load_zone_definitions(path, eng, 5.)


@pytest.mark.parametrize('entries', [
    [ZoneDefinition(0, ZoneType.GPS_ONLY_ZONE, 0, 10)],
    [ZoneDefinition(256, ZoneType.GPS_ONLY_ZONE, 0, 10)],
    [ZoneDefinition(1, ZoneType.GPS_ONLY_ZONE, 0, 10)] * 2,
    [ZoneDefinition(1, ZoneType.GPS_ONLY_ZONE, -1, 10)],
    [ZoneDefinition(1, ZoneType.GPS_ONLY_ZONE, 0, 100)],
    [ZoneDefinition(1, ZoneType.GPS_ONLY_ZONE, 10, 0)],
    [ZoneDefinition(1, ZoneType.MISSION_ZONE, 0, 10, 0, MissionType.NONE)],
    [ZoneDefinition(1, ZoneType.MISSION_ZONE, 0, 10, 256, MissionType.T_PARKING)],
    [ZoneDefinition(1, ZoneType.MISSION_ZONE, 0, 10, 0, MissionType.T_PARKING),
     ZoneDefinition(2, ZoneType.MISSION_ZONE, 20, 30, 0, MissionType.PARALLEL_PARKING)],
    [ZoneDefinition(1, ZoneType.GPS_ONLY_ZONE, 0, 10, 0, MissionType.T_PARKING)],
])
def test_invalid_zone_definitions_are_rejected(entries):
    with pytest.raises(ValueError):
        ZoneMap(entries, 100)


def test_missing_zone_configuration_does_not_invent_coordinates_or_missions(tmp_path):
    eng = engine()
    assert load_zone_definitions(tmp_path / 'absent.yaml', eng, 5.) == ()
    assert ZoneMap.from_engine(eng).definitions == ()


def test_existing_course_zones_file_is_not_reinterpreted_as_new_mission_list():
    eng = engine()
    path = Path(__file__).resolve().parents[1] / 'waypoints/zones_wonju_license_20260818_160511.yaml'
    assert load_zone_definitions(path, eng, 5.) == ()


def test_id_allocation_reserves_explicit_mission_ids():
    eng = engine(parking_ranges=[(0, 10)])
    explicit = [ZoneDefinition(1, ZoneType.MISSION_ZONE, 20, 30, 0, MissionType.PARALLEL_PARKING)]
    definitions = ZoneMap.from_engine(eng, explicit).definitions
    assert definitions[0] == explicit[0]
    assert definitions[1].zone_id == 2 and definitions[1].mission_id == 1


def test_station_zone_wins_over_different_preview_zone():
    current = ZoneDefinition(9, ZoneType.MISSION_ZONE, 10, 20, 0, MissionType.T_PARKING)
    ahead = ZoneDefinition(3, ZoneType.GPS_ONLY_ZONE, 21, 30)
    zones = ZoneMap([ahead, current], 40)
    assert dict(zones.snapshot(20, 23)) == {ahead:False, current:True}
    assert dict(zones.snapshot(9, 23)) == {ahead:True, current:False}
    assert dict(zones.snapshot(21, 23)) == {ahead:True, current:False}
    assert dict(zones.snapshot(9, 23, station_priority=True)) == {ahead:False, current:False}


def test_station_gps_zone_wins_regardless_of_zone_id_sort_order():
    current = ZoneDefinition(9, ZoneType.GPS_ONLY_ZONE, 10, 20)
    ahead = ZoneDefinition(3, ZoneType.GPS_ONLY_ZONE, 21, 30)
    assert dict(ZoneMap([ahead,current],40).snapshot(20,23)) == {ahead:False,current:True}


def test_true_station_overlap_keeps_both_current_memberships():
    current = ZoneDefinition(9, ZoneType.MISSION_ZONE, 10, 20, 0, MissionType.T_PARKING)
    gps = ZoneDefinition(3, ZoneType.GPS_ONLY_ZONE, 15, 25)
    assert all(inside for _,inside in ZoneMap([current,gps],40).snapshot(17,27))
