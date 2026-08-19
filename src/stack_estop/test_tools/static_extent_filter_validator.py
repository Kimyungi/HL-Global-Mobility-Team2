#!/usr/bin/env python3
"""TEST ONLY: validate static extent filtering through ROS publications."""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from fma_interfaces.msg import EstopRequest


EXPECTED = {
    'EXTENT_0.050': (False, 0.050),
    'EXTENT_0.069': (False, 0.069),
    'EXTENT_0.070': (True, 0.070),
    'EXTENT_0.080': (True, 0.080),
    'EXTENT_0.150': (True, 0.150),
    'GRASS_0.040_DUMMY_0.150': (True, None),
}


class StaticExtentFilterValidator(Node):
    def __init__(self):
        super().__init__('static_extent_filter_validator')
        self.current_case = None
        self.estop = None
        self.samples = {name: [] for name in EXPECTED}
        self.done = False
        self.reported = False
        self.create_subscription(
            String, '/test/static_extent_filter/case', self.case_callback, 10)
        self.create_subscription(
            EstopRequest, '/perception/estop', self.estop_callback, 10)
        self.create_subscription(
            String, '/perception/estop/status', self.status_callback, 10)
        print('STATIC EXTENT FILTER TEST', flush=True)

    def case_callback(self, message):
        self.current_case = message.data
        if self.current_case == 'DONE' and not self.reported:
            self.report()
            self.reported = True
            self.done = True

    def estop_callback(self, message):
        self.estop = bool(message.estop)

    def status_callback(self, message):
        if self.current_case not in self.samples:
            return
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        status['_estop_topic'] = self.estop
        self.samples[self.current_case].append(status)

    @staticmethod
    def close(value, expected, tolerance=0.0015):
        return value is not None and math.isclose(
            float(value), expected, abs_tol=tolerance)

    def evaluate_single(self, label, accepted, expected_extent):
        samples = self.samples[label]
        if accepted:
            matches = [s for s in samples if (
                s.get('_estop_topic') is True
                and s.get('static_estop') is True
                and s.get('static_nearest_cluster_min_x') is not None
                and self.close(s.get('nearest_accepted_extent_m'), expected_extent)
                and s.get('static_cluster_filter_reason') == 'CLUSTER_ACCEPTED'
            )]
        else:
            matches = [s for s in samples if (
                s.get('_estop_topic') is False
                and s.get('static_estop') is False
                and s.get('static_nearest_cluster_min_x') is None
                and self.close(s.get('nearest_candidate_extent_m'), expected_extent)
                and s.get('small_cluster_rejected_count', 0) >= 1
                and s.get('static_cluster_filter_reason') == 'SMALL_CLUSTER_EXTENT'
            )]
        return bool(matches), (matches[-1] if matches else (samples[-1] if samples else {}))

    def evaluate_mixed(self):
        samples = self.samples['GRASS_0.040_DUMMY_0.150']
        matches = []
        for sample in samples:
            extents = sample.get('static_cluster_extents_m') or []
            if (
                sample.get('_estop_topic') is True
                and sample.get('static_estop') is True
                and self.close(sample.get('nearest_candidate_extent_m'), 0.040, 0.002)
                and self.close(sample.get('nearest_accepted_extent_m'), 0.150, 0.002)
                and self.close(sample.get('static_nearest_cluster_min_x'), 0.60, 0.01)
                and sample.get('small_cluster_rejected_count') == 1
                and any(self.close(v, 0.040, 0.002) for v in extents)
                and any(self.close(v, 0.150, 0.002) for v in extents)
            ):
                matches.append(sample)
        return bool(matches), (matches[-1] if matches else (samples[-1] if samples else {}))

    def report(self):
        all_passed = True
        print('\nSTATIC EXTENT FILTER TEST', flush=True)
        for label, (accepted, extent) in EXPECTED.items():
            if label == 'GRASS_0.040_DUMMY_0.150':
                passed, sample = self.evaluate_mixed()
            else:
                passed, sample = self.evaluate_single(label, accepted, extent)
            all_passed &= passed
            print(
                f'{label}: {"PASS" if passed else "FAIL"} '
                f'estop={sample.get("_estop_topic")} '
                f'candidate_extent={sample.get("nearest_candidate_extent_m")} '
                f'accepted_extent={sample.get("nearest_accepted_extent_m")} '
                f'nearest_x={sample.get("static_nearest_cluster_min_x")} '
                f'rejected={sample.get("small_cluster_rejected_count")} '
                f'reason={sample.get("static_cluster_filter_reason")}',
                flush=True)
        print(f'FINAL: {"PASS" if all_passed else "FAIL"}', flush=True)


def main():
    rclpy.init()
    node = StaticExtentFilterValidator()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
