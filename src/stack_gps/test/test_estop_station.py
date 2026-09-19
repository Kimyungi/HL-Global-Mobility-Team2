import csv
from pathlib import Path
from stack_gps.path_engine import estop_station_ranges, load_waypoints_csv, PathEngine
from stack_gps.zones import ZoneMap, ZoneType, ZoneDefinition


def test_marker_ends_at_containing_path_and_uses_filtered_indices(tmp_path):
    p=tmp_path/'route.csv'
    with p.open('w') as f:
        writer=csv.writer(f);writer.writerow(['lat','lon','state','path_id','quality'])
        writer.writerows([(37,127,0,3,4),(37.001,127,6,4,4),
                          (37.002,127,0,4,5),(37.003,127,0,4,4),
                          (37.004,127,0,5,4),(37.005,127,6,5,4)])
    assert estop_station_ranges(p)==[(1,2),(4,4)]


def test_production_no_parking_marker():
    p=Path(__file__).resolve().parents[1]/'waypoints/yongin_no_parking.csv'
    spans=estop_station_ranges(p)
    assert len(spans)==1
    ids=[];_,_,states=load_waypoints_csv(p,include_states=True,path_ids=ids)
    first,last=spans[0]
    assert states[first]==6 and ids[first]==ids[last]=='4'
    assert ids[last+1]=='5'


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
