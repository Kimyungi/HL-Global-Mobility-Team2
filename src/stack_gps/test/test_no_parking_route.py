import csv
import json
from pathlib import Path

import pytest
import yaml
from stack_gps.no_parking_route import prepare_no_parking_catalog
from stack_gps.path_engine import PathEngine, load_waypoints_csv, estop_station_ranges
from stack_gps.route_plan import RoutePlan
from stack_gps.zones import ZoneMap, ZoneType, load_zone_definitions, turn_zone_map

SOURCE=Path(__file__).resolve().parents[1]/'waypoints/yongin_no_parking.csv'


def test_snapshot_preserves_all_points_and_station_and_exit_branch(tmp_path):
    catalog=prepare_no_parking_catalog(SOURCE,tmp_path/'prepared')
    assert (catalog.parent/'source.csv').read_bytes()==SOURCE.read_bytes()
    meta=json.loads((catalog.parent/'source_metadata.json').read_text())
    assert meta['rows']==2497
    assert [s['route'] for s in meta['estop_stations']]==['04']
    for start in ('01','02'):
        plan=RoutePlan(catalog,start,'06')
        assert [f.id for f in plan.files]==[start,'03','04','05','06','07']
        def factory(files):
            engine=PathEngine(files.points)
            engine.estop_ranges=estop_station_ranges(files.csv)
            explicit=(*load_zone_definitions(files.zones,engine,5.),
                      *load_zone_definitions(files.zones,engine,5.,key='turn_zones',turn_only=True))
            return engine,turn_zone_map(files.zones,engine,5.,ZoneMap.from_engine(engine,explicit))
        plan.bind(factory)
        assert all(not missions for missions in plan.required)
        idx=next(i for i,f in enumerate(plan.files) if f.id=='04')
        stations=[z for z in plan.zone_maps[idx].definitions if z.zone_type==ZoneType.ESTOP_ZONE]
        assert len(stations)==1
        station=stations[0]
        rows=list(csv.DictReader(plan.files[idx].csv.open()))
        assert rows[station.start_index]['zone_id']=='6'
        assert rows[station.start_index]['idx']=='622'
        assert rows[station.end_index]['idx']=='828'
        assert station.end_index < len(plan.engines[idx].e)-1
        assert not dict(plan.zone_maps[idx].snapshot(station.end_index+1))[station]
        assert not dict(plan.zone_maps[idx].snapshot(station.start_index-1,station.start_index))[station]
        assert dict(plan.zone_maps[idx].snapshot(station.start_index))[station]
    for i in range(1,8):
        rows=list(csv.DictReader((catalog.parent/f'yongin_no_parking_{i:02}.csv').open()))
        expected=[r for r in csv.DictReader(SOURCE.open()) if int(r['path_id'])==i]
        assert rows==expected
        z=yaml.safe_load((catalog.parent/f'zones_no_parking_{i:02}.yaml').read_text())
        assert not z['parking_points']
        if i in (1,2):assert len(z['stop_points'])==1
    assert plan.exit_branches=={'source':'05','left':'06','right':'07'}


def test_snapshot_never_overwrites_existing_run(tmp_path):
    prepare_no_parking_catalog(SOURCE,tmp_path/'prepared')
    with pytest.raises(FileExistsError):prepare_no_parking_catalog(SOURCE,tmp_path/'prepared')
