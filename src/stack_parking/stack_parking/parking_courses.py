"""Explicit CSV route/entry/exit bindings for each parking mission mode."""
from dataclasses import dataclass
import math
from pathlib import Path
import yaml
from .t_parking_sequence import csv_rows, load_course, metric_path


@dataclass(frozen=True)
class ParkingCourse:
    route_id: str
    route_csv: Path
    course: tuple
    exits: tuple | None


def load_catalog(path, origin_csv):
    path = Path(path).resolve()
    data = yaml.safe_load(path.read_text())['missions']
    first = csv_rows(origin_csv)[0]
    origin = float(first['lat']), float(first['lon'])
    result = {}
    for key, entry in data.items():
        mode = int(key)
        if mode not in (1, 2):
            raise ValueError('Unsupported parking mission mode')
        route_id = str(entry['route_id']).zfill(2)
        route_csv = (path.parent / entry['route_csv']).resolve()
        entries = [(path.parent / p).resolve() for p in entry['entries']]
        if len(entries) != 2:
            raise ValueError('Each parking mode requires two candidates')
        course = load_course(origin_csv, route_csv, entries,
                             route_id=int(route_id), mission_state=mode)
        exit_files = entry.get('exits', [])
        exits = None
        if exit_files:
            if len(exit_files) != len(entries):
                raise ValueError('Each entry needs its corresponding exit')
            exits = tuple(metric_path(csv_rows(path.parent / p), origin) for p in exit_files)
            for candidate, (points, station) in zip(course[0], exits):
                end, start = candidate.path[-1], points[0]
                if math.hypot(end.x-start.x, end.y-start.y) > .14:
                    raise ValueError('Parking entry end and exit start do not match')
        result[mode] = ParkingCourse(route_id, route_csv, course, exits)
    if set(result) != {1, 2}:
        raise ValueError('Catalog requires both T and parallel parking')
    return result
