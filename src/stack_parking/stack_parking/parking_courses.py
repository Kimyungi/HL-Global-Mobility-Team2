"""Explicit CSV route/entry/exit bindings for each parking mission mode."""
from dataclasses import dataclass
import math
from pathlib import Path
import hashlib
import json
import yaml
from .t_parking_sequence import csv_rows, load_course, metric_path


@dataclass(frozen=True)
class ParkingCourse:
    route_id: str
    route_csv: Path
    course: tuple
    exits: tuple | None


def snapshot_catalog(template, route_csvs, output, origin_csv):
    """Bind parking to the same session CSV identities published by GPS.

    Copy the existing entry/exit references unchanged and validate geometry
    before a launcher opens hardware. Never generate an unverified connector.
    """
    template, output = Path(template).resolve(), Path(output).resolve()
    data = yaml.safe_load(template.read_text())
    output.mkdir(parents=True, exist_ok=False)
    provenance = []
    for mode, entry in data['missions'].items():
        route_id = str(entry['route_id']).zfill(2)
        entry['route_csv'] = str(Path(route_csvs[route_id]).resolve())
        for kind in ('entries', 'exits'):
            copies = []
            for index, relative in enumerate(entry.get(kind, [])):
                source = (template.parent / relative).resolve()
                content = source.read_bytes()
                name = f'{mode}_{kind}_{index}_{source.name}'
                (output / name).write_bytes(content)
                copies.append(name)
                provenance.append(dict(source=str(source), copy=name,
                                       sha256=hashlib.sha256(content).hexdigest()))
            entry[kind] = copies
    catalog = output / 'parking_courses.yaml'
    catalog.write_text(yaml.safe_dump(data, sort_keys=False))
    load_catalog(catalog, origin_csv)
    (output / 'source_metadata.json').write_text(json.dumps(provenance, indent=2))
    return catalog


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
