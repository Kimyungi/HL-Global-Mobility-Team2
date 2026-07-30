#!/usr/bin/env python3
"""Analyze stack_estop → MGM timing using rosbag record timestamps."""

import argparse
import json
import math
from pathlib import Path
import sqlite3

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from stack_estop.node import nearest_corridor_cluster_x


SCAN_TOPIC = '/scan'
ESTOP_TOPIC = '/perception/estop'
MGM_TOPIC = '/adas/target_ref'
ZERO_EPSILON = 1e-4
EXPECTED_A_UP_MPS2 = 0.5
RATE_TOLERANCE_MPS2 = 0.15


def load_topics(bag_path):
    events = {SCAN_TOPIC: [], ESTOP_TOPIC: [], MGM_TOPIC: []}
    databases = sorted(bag_path.glob('*.db3'))
    if not databases:
        raise FileNotFoundError(f'No sqlite3 .db3 files found in {bag_path}')

    for database in databases:
        connection = sqlite3.connect(str(database))
        try:
            topics = {
                row[0]: (row[1], row[2])
                for row in connection.execute(
                    'SELECT name, id, type FROM topics'
                )
            }
            for topic_name in events:
                if topic_name not in topics:
                    continue
                topic_id, type_name = topics[topic_name]
                message_class = get_message(type_name)
                rows = connection.execute(
                    'SELECT timestamp, data FROM messages '
                    'WHERE topic_id=? ORDER BY timestamp',
                    (topic_id,),
                )
                for timestamp_ns, data in rows:
                    events[topic_name].append((
                        int(timestamp_ns),
                        deserialize_message(data, message_class),
                    ))
        finally:
            connection.close()

    for topic_name, values in events.items():
        values.sort(key=lambda item: item[0])
        if not values:
            raise ValueError(f'Required topic is missing: {topic_name}')
    return events


def nearest_x(scan):
    valid, nearest = nearest_corridor_cluster_x(
        scan.ranges,
        float(scan.angle_min),
        float(scan.angle_increment),
        float(scan.range_min),
        float(scan.range_max),
    )
    return nearest if valid else None


def first_at_or_after(values, timestamp_ns, predicate):
    return next(
        ((stamp, message) for stamp, message in values
         if stamp >= timestamp_ns and predicate(message)),
        None,
    )


def milliseconds(later, earlier):
    if later is None or earlier is None:
        return None
    return (later - earlier) / 1e6


def transitions(estop_events):
    result = []
    previous = None
    for timestamp_ns, message in estop_events:
        value = bool(message.estop)
        if previous is None or value != previous:
            result.append((timestamp_ns, value))
            previous = value
    return result


def analyze(bag_path):
    events = load_topics(bag_path)
    scans = events[SCAN_TOPIC]
    estops = events[ESTOP_TOPIC]
    targets = events[MGM_TOPIC]

    scan_danger = next(
        (
            (timestamp_ns, distance)
            for timestamp_ns, scan in scans
            for distance in [nearest_x(scan)]
            if distance is not None and distance <= 0.70
        ),
        None,
    )
    danger_scan_ns = scan_danger[0] if scan_danger else None
    estop_true = (
        first_at_or_after(
            estops, danger_scan_ns, lambda message: bool(message.estop)
        )
        if danger_scan_ns is not None else None
    )
    estop_true_ns = estop_true[0] if estop_true else None
    mgm_zero = (
        first_at_or_after(
            targets,
            estop_true_ns,
            lambda message: abs(float(message.v_ref)) <= ZERO_EPSILON,
        )
        if estop_true_ns is not None else None
    )
    mgm_zero_ns = mgm_zero[0] if mgm_zero else None

    estop_false = (
        first_at_or_after(
            estops,
            estop_true_ns + 1,
            lambda message: not bool(message.estop),
        )
        if estop_true_ns is not None else None
    )
    estop_false_ns = estop_false[0] if estop_false else None
    reaccel = (
        first_at_or_after(
            targets,
            estop_false_ns,
            lambda message: float(message.v_ref) > ZERO_EPSILON,
        )
        if estop_false_ns is not None else None
    )
    reaccel_ns = reaccel[0] if reaccel else None

    next_true_ns = next(
        (
            stamp for stamp, value in transitions(estops)
            if estop_false_ns is not None
            and stamp > estop_false_ns and value
        ),
        None,
    )
    accel_samples = [
        (stamp, float(message.v_ref))
        for stamp, message in targets
        if estop_false_ns is not None
        and stamp >= estop_false_ns
        and (next_true_ns is None or stamp < next_true_ns)
    ]
    acceleration_rates = []
    monotonic_increases = []
    for (t0, v0), (t1, v1) in zip(accel_samples, accel_samples[1:]):
        dt = (t1 - t0) * 1e-9
        if dt > 0.0 and v1 > v0 + ZERO_EPSILON:
            acceleration_rates.append((v1 - v0) / dt)
            monotonic_increases.append(v1 - v0)
    max_acceleration = max(acceleration_rates, default=None)
    rate_limited = bool(
        acceleration_rates
        and max_acceleration
        <= EXPECTED_A_UP_MPS2 + RATE_TOLERANCE_MPS2
    )

    estop_interval_end = estop_false_ns or (targets[-1][0] + 1)
    states_during_estop = [
        int(message.state)
        for stamp, message in targets
        if estop_true_ns is not None
        and estop_true_ns <= stamp < estop_interval_end
    ]
    state_transition_count = sum(
        current != previous
        for previous, current in zip(
            states_during_estop, states_during_estop[1:]
        )
    )

    estop_transitions = transitions(estops)
    rapid_transition_count = sum(
        current[0] - previous[0] < 250_000_000
        for previous, current in zip(
            estop_transitions, estop_transitions[1:]
        )
    )

    last_scan_ns = scans[-1][0]
    timeout_deadline_ns = last_scan_ns + 250_000_000
    timeout_estop = first_at_or_after(
        estops, timeout_deadline_ns, lambda message: bool(message.estop)
    )
    timeout_estop_ns = timeout_estop[0] if timeout_estop else None
    timeout_mgm_zero = (
        first_at_or_after(
            targets,
            timeout_estop_ns,
            lambda message: abs(float(message.v_ref)) <= ZERO_EPSILON,
        )
        if timeout_estop_ns is not None else None
    )
    timeout_mgm_zero_ns = (
        timeout_mgm_zero[0] if timeout_mgm_zero else None
    )

    return {
        'bag_path': str(bag_path),
        'event_times_epoch_sec': {
            'scan_first_le_0_70': (
                danger_scan_ns * 1e-9 if danger_scan_ns else None
            ),
            'estop_true_after_danger': (
                estop_true_ns * 1e-9 if estop_true_ns else None
            ),
            'mgm_v_ref_zero_after_estop': (
                mgm_zero_ns * 1e-9 if mgm_zero_ns else None
            ),
            'estop_false_after_danger': (
                estop_false_ns * 1e-9 if estop_false_ns else None
            ),
            'mgm_reacceleration_start': (
                reaccel_ns * 1e-9 if reaccel_ns else None
            ),
            'last_scan': last_scan_ns * 1e-9,
            'scan_timeout_estop_true': (
                timeout_estop_ns * 1e-9 if timeout_estop_ns else None
            ),
            'scan_timeout_mgm_zero': (
                timeout_mgm_zero_ns * 1e-9
                if timeout_mgm_zero_ns else None
            ),
        },
        'latency_ms': {
            'scan_to_estop_ms': milliseconds(
                estop_true_ns, danger_scan_ns
            ),
            'estop_to_mgm_zero_ms': milliseconds(
                mgm_zero_ns, estop_true_ns
            ),
            'scan_to_mgm_zero_ms': milliseconds(
                mgm_zero_ns, danger_scan_ns
            ),
            'estop_false_to_reacceleration_ms': milliseconds(
                reaccel_ns, estop_false_ns
            ),
            'timeout_deadline_to_estop_ms': milliseconds(
                timeout_estop_ns, timeout_deadline_ns
            ),
            'timeout_estop_to_mgm_zero_ms': milliseconds(
                timeout_mgm_zero_ns, timeout_estop_ns
            ),
        },
        'distance_at_first_danger_m': (
            scan_danger[1] if scan_danger else None
        ),
        'reacceleration': {
            'sample_increase_count': len(monotonic_increases),
            'max_observed_acceleration_mps2': max_acceleration,
            'expected_a_up_mps2': EXPECTED_A_UP_MPS2,
            'rate_limited': rate_limited,
        },
        'state_during_estop': {
            'observed_states': sorted(set(states_during_estop)),
            'transition_count': state_transition_count,
            'state_unchanged': state_transition_count == 0,
        },
        'estop_chattering': {
            'transition_count': max(0, len(estop_transitions) - 1),
            'rapid_transition_count_under_250ms': rapid_transition_count,
            'transitions': [
                {'epoch_sec': stamp * 1e-9, 'estop': value}
                for stamp, value in estop_transitions
            ],
        },
        'immediate_stop_observable_in_target_ref': False,
        'immediate_stop_note': (
            'TargetRef has no immediate_stop field. Verify it with MGM '
            'snapshot_dump_path + core_replay CSV; v_ref=0 and unchanged '
            'state are the rosbag-visible integration evidence.'
        ),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'bag',
        type=Path,
        help='Integration rosbag directory containing sqlite3 .db3 files',
    )
    args = parser.parse_args()
    print(json.dumps(
        analyze(args.bag.resolve()),
        ensure_ascii=False,
        indent=2,
        allow_nan=False,
    ))


if __name__ == '__main__':
    main()
