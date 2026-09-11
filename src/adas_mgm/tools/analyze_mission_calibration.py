#!/usr/bin/env python3
"""Summarize MGM mission_events.csv observations; never writes operating parameters."""
import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

EVENTS = ('zone_entry', 'search_start', 'space_found', 'ready', 'handoff', 'cancel', 'done')
KINDS = {1: 'T_PARKING', 2: 'PARALLEL_PARKING'}
NUMBERS = ('elapsed_s', 'travel_distance', 'actual_speed', 'x', 'y')
INTEGERS = ('request_id', 'mission_id', 'mission_type', 'source_zone_id', 'time_ns',
            'track_index', 'zone_id', 'cancel_reason')


def percentile(values, percent):
    """Linear interpolation between sorted samples; same definition for every group."""
    ordered = sorted(values)
    rank = (len(ordered) - 1) * percent / 100
    lo, hi = math.floor(rank), math.ceil(rank)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (rank - lo)


def statistics(values):
    if not values:
        return {'count': 0}
    return {'count': len(values), 'mean': sum(values) / len(values), 'max': max(values),
            **{f'p{p}': percentile(values, p) for p in (50, 90, 95, 99)}}


def analyze(paths):
    requests = defaultdict(dict)
    for path in paths:
        with Path(path).open(newline='', encoding='utf-8') as stream:
            reader = csv.DictReader(stream)
            required = {'event', 'position_valid', 'speed_valid', *NUMBERS, *INTEGERS}
            if not required.issubset(reader.fieldnames or ()):
                raise ValueError(f'{path}: expected MGM mission event schema, not a maneuver tick log')
            for line, row in enumerate(reader, 2):
                if row['event'] not in EVENTS:
                    raise ValueError(f'{path}:{line}: unknown mission event')
                for field in NUMBERS:
                    row[field] = float(row[field])
                    # Invalid position/speed can legitimately carry NaN; make absence explicit.
                    if not math.isfinite(row[field]):
                        validity = 'position_valid' if field in ('x', 'y') else 'speed_valid'
                        if field in ('elapsed_s', 'travel_distance') or row[validity] in ('1', 'true', 'True'):
                            raise ValueError(f'{path}:{line}: nonfinite {field}')
                        row[field] = None
                for field in INTEGERS:
                    row[field] = int(row[field])
                if row['elapsed_s'] < 0 or row['travel_distance'] < 0 or row['request_id'] <= 0:
                    raise ValueError(f'{path}:{line}: invalid lifetime/distance/request')
                for field in ('position_valid', 'speed_valid'):
                    if row[field] not in ('0', '1', 'false', 'true', 'False', 'True'):
                        raise ValueError(f'{path}:{line}: invalid boolean {field}')
                    row[field] = row[field] in ('1', 'true', 'True')
                # File is a run boundary. Split logs must be joined explicitly by the operator.
                key = (str(Path(path).resolve()), row['request_id'])
                events = requests[key]
                if events:
                    first = next(iter(events.values()))
                    if any(first[f] != row[f] for f in ('mission_id', 'mission_type', 'source_zone_id')):
                        raise ValueError(f'{path}:{line}: conflicting request identity')
                previous = events.get(row['event'])
                if previous is not None and previous != row:
                    raise ValueError(f'{path}:{line}: conflicting duplicate event')
                events[row['event']] = row

    groups = defaultdict(lambda: {'outcomes': Counter(), 'metrics': defaultdict(list)})
    details = []
    for (path, request_id), events in requests.items():
        first = next(iter(events.values()))
        kind = KINDS.get(first['mission_type'], f"UNKNOWN_{first['mission_type']}")
        if 'done' in events and 'cancel' in events:
            raise ValueError(f'{path}: request {request_id} has both done and cancel')
        reason = events.get('cancel', {}).get('cancel_reason', 0)
        outcome = ('success' if 'done' in events else 'timeout' if reason == 1 else
                   'distance_cancel' if reason == 2 else 'cancel' if 'cancel' in events else 'incomplete')
        group = groups[kind]
        group['outcomes'][outcome] += 1
        metrics = {}
        entry = events.get('zone_entry')
        if entry:
            for event in ('search_start', 'space_found', 'ready', 'handoff'):
                if event not in events:
                    continue
                observed = events[event]
                # elapsed_s is monotonic lifetime; ROS time_ns is retained for log correlation only.
                elapsed = observed['elapsed_s'] - entry['elapsed_s']
                distance = observed['travel_distance'] - entry['travel_distance']
                if elapsed < 0 or distance < 0:
                    raise ValueError(f'{path}: request {request_id} has regressing observations')
                metrics[f'entry_to_{event}_s'] = elapsed
                if event in ('ready', 'handoff'):
                    metrics[f'entry_to_{event}_m'] = distance
            for key, value in metrics.items():
                group['metrics'][key].append(value)
        details.append({'file': path, 'request_id': request_id, 'mission_type': kind,
                        'outcome': outcome, 'cancel_reason': reason, 'missing_entry': entry is None,
                        'metrics': metrics, 'events': events})
    return {'status': 'CALIBRATION_REQUIRED', 'operating_parameters_generated': False,
            'note': 'Observed samples only. Missing milestones are excluded, not replaced by zero. '
                    'Percentiles of sparse or selected successes do not certify timeout/distance limits.',
            'groups': {kind: {'outcomes': dict(group['outcomes']),
                             'metrics': {key: statistics(values) for key, values in group['metrics'].items()}}
                       for kind, group in sorted(groups.items())}, 'requests': details}


def markdown(report):
    lines = ['# MGM Parking calibration observations', '', report['status'], '', report['note'], '',
             '운영 파라미터를 생성하거나 YAML에 기록하지 않았습니다.', '']
    for kind, group in report['groups'].items():
        lines += [f'## {kind}', '', ', '.join(f'{k}: {v}' for k, v in group['outcomes'].items()), '',
                  '| Metric | N | Mean | Max | p50 | p90 | p95 | p99 |',
                  '|---|---:|---:|---:|---:|---:|---:|---:|']
        for key, values in group['metrics'].items():
            lines.append('| ' + key + ' | ' + ' | '.join(
                str(values[k]) if k == 'count' else f'{values[k]:.6f}'
                for k in ('count', 'mean', 'max', 'p50', 'p90', 'p95', 'p99')) + ' |')
        lines.append('')
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', nargs='+', type=Path)
    parser.add_argument('--json', type=Path)
    parser.add_argument('--markdown', type=Path)
    args = parser.parse_args()
    try:
        report = analyze(args.csv)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    if args.json:
        args.json.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    if args.markdown:
        args.markdown.write_text(markdown(report) + '\n')
    if not args.json and not args.markdown:
        print(markdown(report))


if __name__ == '__main__':
    main()
