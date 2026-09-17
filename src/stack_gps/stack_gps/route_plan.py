"""Immutable route files and GPS command application; MGM owns progression."""
from dataclasses import dataclass, replace
import hashlib
import json
import math
import statistics
from pathlib import Path

import yaml
from .path_engine import PathEngine, load_waypoints_csv
from .zones import ZoneMap, ZoneType


@dataclass(frozen=True)
class RouteFiles:
    id: str
    csv: Path
    zones: Path
    points: tuple
    completion: int
    entry_connection: bool


def _zone_schema(path, course):
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or data.get('track') != course.name:
        raise ValueError(f'{path}: track must match {course.name}')
    def coordinate(point):
        lat, lon = float(point['lat']), float(point['lon'])
        if not (math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError(f'{path}: invalid coordinate')
    for key in ('stop_points', 'avoid_zones', 'gps_only_zones', 'parking_points', 'zones'):
        entries = data.get(key, [])
        if not isinstance(entries, list):
            raise ValueError(f'{path}: {key} must be a list')
        for entry in entries:
            if not isinstance(entry, dict):
                raise ValueError(f'{path}: invalid {key} entry')
            if key in ('stop_points', 'parking_points'):
                coordinate(entry)
                if key == 'parking_points' and entry.get('mode') not in (
                        't', 't_parking', 'perpendicular', 'parallel', 'parallel_parking'):
                    raise ValueError(f'{path}: unsupported parking mode')
            elif key in ('avoid_zones', 'gps_only_zones'):
                coordinate(entry['start']); coordinate(entry['end'])
    return data


class RoutePlan:
    def __init__(self, manifest, start_id='', end_id=''):
        self.path = Path(manifest).expanduser().resolve()
        data = yaml.safe_load(self.path.read_text())
        entries = data.get('routes') if isinstance(data, dict) else None
        if not isinstance(entries, list) or not 1 <= len(entries) <= 256:
            raise ValueError('route plan requires 1..256 routes')
        catalog = {}
        for entry in entries:
            if not isinstance(entry, dict) or not {'id', 'file', 'zones_file'} <= entry.keys():
                raise ValueError('each route requires id, file and zones_file')
            route_id = str(entry['id'])
            if not route_id or route_id in catalog:
                raise ValueError('route ids must be nonempty and unique')
            catalog[route_id] = entry
        branches = data.get('exit_branches')
        selection = data.get('sequence')
        if selection is not None:
            if not isinstance(selection, dict) or set(selection) != {'start', 'via', 'end'}:
                raise ValueError('sequence requires start, via and end lists')
            for key, choices in selection.items():
                if not isinstance(choices, list) or (key != 'via' and not choices):
                    raise ValueError(f'sequence {key} must be a list of route IDs')
                if any(not isinstance(v, str) or v not in catalog for v in choices) or len(set(choices)) != len(choices):
                    raise ValueError(f'sequence {key} contains unknown or duplicate route IDs')
            if start_id not in selection['start'] or end_id not in selection['end']:
                raise ValueError(f'Explicit route_start_id {selection["start"]} and route_end_id {selection["end"]} required')
            selected = [start_id, *selection['via'], end_id]
            if len(set(selected)) != len(selected):
                raise ValueError('selected sequence must not repeat a route')
            # An explicitly defined exit zone enables both final alternatives.
            # No zone in the selected prefix means the existing fixed end is retained.
            sources = [key for key in selected[:-1] if any(
                z.get('zone_type') == 'LAST_MISSION_ZONE' for z in
                (yaml.safe_load((self.path.parent / catalog[key]['zones_file']).read_text()) or {}).get('zones', []))]
            if sources:
                if len(sources) != 1 or sources[0] != selected[-2] or set(selection['end']) != {'06', '07'}:
                    raise ValueError('last mission requires one zone-bearing final common route and ends 06/07')
                branches = {'source': sources[0], 'left': '06', 'right': '07'}
                selected = [*selected[:-1], '06', '07']
            entries = [catalog[key] for key in selected]
        elif start_id or end_id:
            raise ValueError('route_start_id/route_end_id require a selectable sequence manifest')
        self.exit_branches = branches
        self.exit_source = self.exit_left = self.exit_right = -1
        if branches is not None:
            if not isinstance(branches, dict) or set(branches) != {'source', 'left', 'right'}:
                raise ValueError('exit_branches requires source/left/right route IDs')
            ids = [str(entry['id']) for entry in entries]
            if branches['left'] != '06' or branches['right'] != '07':
                raise ValueError('exit directions must map Left to 06 and Right to 07')
            try:
                source, left, right = (ids.index(branches[k]) for k in ('source', 'left', 'right'))
            except ValueError as error:
                raise ValueError('exit branch route missing from preloaded plan') from error
            if (left, right) != (source + 1, source + 2) or right != len(ids) - 1:
                raise ValueError('exit branches must follow the final common route as 06,07')
            self.exit_source, self.exit_left, self.exit_right = source, left, right
        self.files = []
        self._contents = {self.path: self.path.read_bytes()}
        ids = set()
        for entry in entries:
            if not isinstance(entry, dict) or not {'id', 'file', 'zones_file'} <= entry.keys():
                raise ValueError('each route requires id, file and zones_file')
            condition = entry.get('completion', 'endpoint_and_missions')
            if condition not in ('endpoint_and_missions', 'missions_complete'):
                raise ValueError('unsupported route completion condition')
            completion = int(condition == 'missions_complete')
            entry_connection = entry.get('entry_connection', 'none')
            if entry_connection not in ('none', 'straight'):
                raise ValueError('entry_connection must be none or straight')
            if entry_connection == 'straight' and not self.files:
                raise ValueError('first route has no predecessor to connect')
            route_id = str(entry['id'])
            if not route_id or route_id in ids:
                raise ValueError('route ids must be nonempty and unique')
            ids.add(route_id)
            course = (self.path.parent / entry['file']).resolve()
            zones = (self.path.parent / entry['zones_file']).resolve()
            # No tolerant/skipping path parser may silently alter this plan.
            points = tuple(load_waypoints_csv(course, log=self._reject))
            if len(points) < 10 or not all(math.isfinite(v) for p in points for v in p):
                raise ValueError(f'{course}: requires at least 10 finite points')
            zone_data = _zone_schema(zones, course)
            has_exit_zone = any(z.get('zone_type') == 'LAST_MISSION_ZONE' for z in zone_data.get('zones', []))
            if has_exit_zone != (len(self.files) == self.exit_source):
                raise ValueError('LAST_MISSION_ZONE must be on the configured exit source route only')
            if len(self.files) in (self.exit_left, self.exit_right) and entry_connection != 'none':
                raise ValueError('exit branches use their preloaded CSV without an implicit connector')
            # CSV gaps may be traversed by a Zone/Mission. Preserve the configured order.
            self.files.append(RouteFiles(route_id, course, zones, points, completion, entry_connection == 'straight'))
            self._contents[course] = course.read_bytes()
            self._contents[zones] = zones.read_bytes()
        digest = hashlib.sha256()
        # The same catalog with a different branch is a different run contract.
        if selection is not None:
            digest.update(json.dumps([r.id for r in self.files]).encode('utf-8'))
        for content in self._contents.values():
            digest.update(len(content).to_bytes(8, 'big')); digest.update(content)
        self.sequence_id = int.from_bytes(digest.digest()[:8], 'big') or 1
        self.index = 0
        self.connecting = False
        self.connections = []
        self.connection_points = []
        self.acknowledged_request = 0
        self.engines = []
        self.zone_maps = []
        self.required = []
        self.stop_offsets = []

    @staticmethod
    def _reject(message):
        raise ValueError(message)

    def bind(self, factory):
        """Use the existing engine/Zone conversion; assign course-wide wire IDs."""
        zone_id, mission_id, stop_offset = 1, 0, 0
        for files in self.files:
            engine, zones = factory(files)
            local_missions = {}
            definitions = []
            for zone in zones.definitions:
                mapped_mission = 0
                if zone.zone_type == ZoneType.MISSION_ZONE:
                    if zone.mission_id not in local_missions:
                        local_missions[zone.mission_id] = mission_id
                        mission_id += 1
                    mapped_mission = local_missions[zone.mission_id]
                definitions.append(replace(zone, zone_id=zone_id, mission_id=mapped_mission))
                zone_id += 1
            if zone_id > 256 or mission_id > 256 or stop_offset + len(engine.stop_ranges) > 255:
                raise ValueError('sequence exceeds uint8 Zone/Mission/stop ID capacity')
            if files.completion == 1 and not local_missions:
                raise ValueError(f'{files.id}: missions_complete requires at least one Mission Zone')
            self.engines.append(engine)
            self.zone_maps.append(ZoneMap(definitions, len(engine.e)))
            self.required.append(tuple(sorted(local_missions.values())))
            self.stop_offsets.append(stop_offset)
            stop_offset += len(engine.stop_ranges)
        for i, files in enumerate(self.files):
            points, engine = (), None
            if files.entry_connection:
                start, end = self.files[i-1].points[-1], files.points[0]
                if start != end:
                    destination = self.engines[i]
                    spacing = statistics.median(math.hypot(b-a, d-c)
                        for a,b,c,d in zip(destination.e, destination.e[1:], destination.n, destination.n[1:])
                        if math.hypot(b-a, d-c) > 0)
                    distance = math.hypot(*destination.to_enu(*start))
                    count = max(3, math.ceil(distance / spacing) + 1)
                    points = tuple(start if k == 0 else end if k == count-1 else
                        tuple(a + (b-a)*k/(count-1) for a,b in zip(start,end)) for k in range(count))
                    engine = copy_geometry(destination, points)
            self.connections.append(engine)
            self.connection_points.append(points)
        for path, content in self._contents.items():
            if path.read_bytes() != content:
                raise ValueError(f'route configuration changed during preload: {path}')

    def apply(self, sequence_id, instance_id, own_instance, request_id, requested_index, connecting=False):
        """Idempotent acknowledgement; no autonomous endpoint/Mission decision here."""
        if sequence_id != self.sequence_id or instance_id != own_instance:
            return False
        if request_id <= self.acknowledged_request or request_id <= 0:
            return False
        if not 0 <= requested_index < len(self.files):
            return False
        resetting = requested_index == 0 and not connecting
        advancing = (self.index != self.exit_source and self.index not in (self.exit_left, self.exit_right)
                     and not self.connecting and requested_index == self.index + 1
                     and connecting == bool(self.connections[requested_index]))
        branching = (self.index == self.exit_source and not self.connecting and not connecting
                     and requested_index in (self.exit_left, self.exit_right))
        entering = self.connecting and requested_index == self.index and not connecting
        if not (resetting or advancing or entering or branching):
            return False
        self.index = requested_index
        self.connecting = connecting
        self.acknowledged_request = request_id
        self.active_engine.reset_station()
        return True

    @property
    def active_engine(self):
        return self.connections[self.index] if self.connecting else self.engines[self.index]

    @property
    def active_points(self):
        return self.connection_points[self.index] if self.connecting else self.files[self.index].points

    @property
    def active_zones(self):
        return ZoneMap([], len(self.active_points)) if self.connecting else self.zone_maps[self.index]

    @property
    def next_connecting(self):
        if self.index in (self.exit_source, self.exit_left, self.exit_right):
            return False
        return self.index + 1 < len(self.files) and self.files[self.index+1].entry_connection and \
            self.files[self.index].points[-1] != self.files[self.index+1].points[0]

    def position(self, lat, lon):
        """One ENU frame for TF, mission observations and GNSS pose delta."""
        return self.engines[0].to_enu(lat, lon)


def copy_geometry(template, points):
    """Reuse the configured PathEngine geometry for CSV and direct entry segments."""
    return PathEngine(points, n_points=template.n_points,
        lookahead_m=template.lookahead_m, rate_damp_s=template.rate_damp_s,
        full_cross_m=template.full_cross_m, target_max_m=template.target_max_m,
        target_min_m=template.target_min_m, e_lpf_s=template.e_lpf_s,
        curve_ff=template.curve_ff, curve_margin=template.curve_margin,
        station_tracking=template.station_tracking)


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest')
    parser.add_argument('--start', default='', help='Explicit start route ID, e.g. 01')
    parser.add_argument('--end', default='', help='Explicit end route ID, e.g. 07')
    args = parser.parse_args()
    plan = RoutePlan(args.manifest, args.start, args.end)
    print(f'ROUTE_FILES_READY: sequence={plan.sequence_id}, routes={len(plan.files)}')
    for i, route in enumerate(plan.files):
        print(i, route.id, len(route.points), ('endpoint_and_missions', 'missions_complete')[route.completion], route.csv, route.zones)


if __name__ == '__main__':
    main()
