import csv
from pathlib import Path
from stack_gps.path_engine import estop_station_ranges, load_waypoints_csv, PathEngine
from stack_gps.zones import ZoneMap, ZoneType, ZoneDefinition


def test_zone_6_filters_quality_and_duplicates_and_ignores_state(tmp_path):
    p=tmp_path/'route.csv'
    p.write_text('lat,lon,state,path_id,quality,zone_id,inside_zone\n'
                 '37,127,6,4,4,0,0\n'
                 '37.001,127,0,4,4,6,1\n'
                 '37.002,127,0,4,5,6,1\n'
                 '37.003,127,0,4,4,6,1\n'
                 '37.003,127,0,4,4,6,1\n'
                 '37.004,127,0,4,4,6,0\n'
                 '37.005,127,0,4,4,5,1\n'
                 '37.006,127,0,4,4,6,1\n'
                 '37.007,127,0,4,4,0,0\n')
    assert estop_station_ranges(p)==[(1,2),(5,5)]


def test_state_6_alone_never_arms(tmp_path):
    p=tmp_path/'route.csv'
    p.write_text('lat,lon,state\n37,127,6\n37.001,127,0\n')
    assert estop_station_ranges(p)==[]


def test_production_no_parking_zone():
    p=Path(__file__).resolve().parents[1]/'waypoints/yongin_no_parking.csv'
    spans=estop_station_ranges(p)
    membership={}
    load_waypoints_csv(p,zone_indices=membership)
    assert len(spans)==1
    first,last=spans[0]
    assert all(membership.get(i)==6 for i in range(first,last+1))
    assert membership.get(first-1)!=6 and membership.get(last+1)!=6
    rows=list(csv.DictReader(p.open()))
    selected=[r for r in rows if r['zone_id']=='6' and r['inside_zone']=='1']
    assert {r['path_id'] for r in selected}=={'4'}
    assert (selected[0]['idx'],selected[-1]['idx'])==('622','828')


def test_station_is_current_only_and_does_not_hide_other_memberships():
    engine=PathEngine([(37+i*.00001,127) for i in range(20)])
    engine.estop_ranges=[(8,12)]
    zone_map=ZoneMap.from_engine(engine,[ZoneDefinition(3,ZoneType.GPS_ONLY_ZONE,5,15)])
    station=next(z for z in zone_map.definitions if z.zone_type==ZoneType.ESTOP_ZONE)
    assert not dict(zone_map.snapshot(7,10))[station]
    members=dict(zone_map.snapshot(8,10))
    assert members[station]
    assert next(v for z,v in members.items() if z.zone_type==ZoneType.GPS_ONLY_ZONE)
    assert dict(zone_map.snapshot(12))[station]
    assert not dict(zone_map.snapshot(13))[station]
