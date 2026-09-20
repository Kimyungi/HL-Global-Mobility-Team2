"""Create per-session route files from a combined Yongin CSV, preserving metadata."""
import csv
import hashlib
import io
import json
from pathlib import Path

import yaml
from .path_engine import csv_zone_ranges, load_waypoints_csv, estop_station_ranges
from .route_plan import RoutePlan


def prepare_no_parking_catalog(source, output):
    return prepare_csv_catalog(source, output, parking=False)


def prepare_csv_catalog(source, output, *, parking):
    source, output = Path(source).resolve(), Path(output).resolve()
    data = source.read_bytes()
    reader = csv.DictReader(io.StringIO(data.decode('utf-8-sig')))
    fields = reader.fieldnames
    if not fields or not {'path_id', 'lat', 'lon', 'state', 'zone_id', 'inside_zone'} <= set(fields):
        raise ValueError('no-parking CSV requires path_id, lat, lon, state, zone_id, inside_zone')
    groups = {str(i): [] for i in range(1, 8)}
    for row in reader:
        path_id = str(int(row['path_id']))
        if path_id not in groups or (not parking and int(row['state']) in (1, 2)):
            raise ValueError('no-parking course requires paths 01..07 without parking states 1/2')
        groups[path_id].append(row)
    if any(len(rows) < 10 for rows in groups.values()):
        raise ValueError('all seven no-parking paths require at least ten points')
    if parking:
        markers = [(int(pid), int(row['state'])) for pid, rows in groups.items()
                   for row in rows if int(row['state']) in (1, 2)]
        if markers != [(3, 1), (4, 2)]:
            raise ValueError('parking course requires one T marker on 03 and one parallel marker on 04')
    stem = source.stem if parking else 'yongin_no_parking'
    output.mkdir(parents=True, exist_ok=False)
    (output/'source.csv').write_bytes(data)
    routes = []
    station_info = []
    for i in range(1, 8):
        route_id = f'{i:02}'
        csv_path = output/f'{stem}_{route_id}.csv'
        with csv_path.open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader(); writer.writerows(groups[str(i)])
        # Use the exact filtering/index space used by GPS, not the original idx column.
        points, _, states = load_waypoints_csv(csv_path, include_states=True)
        zones = dict(track=csv_path.name, stop_points=[], avoid_zones=[],
                     gps_only_zones=[], parking_points=[], turn_zones=[], zones=[])
        for index, state in enumerate(states):
            if state == 5:
                lat, lon = points[index]
                zones['stop_points'].append(dict(lat=lat, lon=lon))
            elif parking and state in (1, 2):
                lat, lon = points[index]
                zones['parking_points'].append(dict(
                    mode='perpendicular' if state == 1 else 'parallel', lat=lat, lon=lon))
        for zid in (1, 3, 4):
            ranges = csv_zone_ranges(csv_path, zid)
            if ranges:
                zones['turn_zones'].append(dict(zone_id=zid, index_ranges=[list(r) for r in ranges]))
        exit_ranges = csv_zone_ranges(csv_path, 2)
        if exit_ranges:
            if i != 5:
                raise ValueError('exit zone [2] must belong to path 05')
            zones['zones'].append(dict(zone_id=2, zone_type='LAST_MISSION_ZONE',
                                       index_ranges=[list(r) for r in exit_ranges]))
        zone_file = output/f'zones_{stem if parking else "no_parking"}_{route_id}.yaml'
        zone_file.write_text(yaml.safe_dump(zones, sort_keys=False))
        routes.append(dict(id=route_id, file=csv_path.name, zones_file=zone_file.name,
                           completion='endpoint_and_missions'))
        station_info.extend(dict(route=route_id, first=a, last=b)
                            for a,b in estop_station_ranges(csv_path))
    catalog = output/'route_sequence.yaml'
    catalog.write_text(yaml.safe_dump(dict(
        sequence=dict(start=['01','02'], via=['03','04','05'], end=['06','07']),
        routes=routes, exit_branches=dict(source='05',left='06',right='07')), sort_keys=False))
    RoutePlan(catalog, '01', '06')  # Validate branches, exit zone and all files before hardware.
    (output/'source_metadata.json').write_text(json.dumps(dict(
        source=str(source), sha256=hashlib.sha256(data).hexdigest(), rows=sum(map(len,groups.values())),
        estop_stations=station_info), ensure_ascii=False, indent=2))
    return catalog
