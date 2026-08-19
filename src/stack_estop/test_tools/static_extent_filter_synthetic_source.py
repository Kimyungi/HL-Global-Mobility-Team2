#!/usr/bin/env python3
"""TEST ONLY: deterministic LaserScan cases for the static extent filter."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


YAW = math.pi / 2.0


class StaticExtentFilterSyntheticSource(Node):
    CASES = (
        ('EXTENT_0.050', 0.050),
        ('EXTENT_0.069', 0.069),
        ('EXTENT_0.070', 0.070),
        ('EXTENT_0.080', 0.080),
        ('EXTENT_0.150', 0.150),
        ('GRASS_0.040_DUMMY_0.150', None),
    )

    def __init__(self):
        super().__init__('static_extent_filter_synthetic_source')
        self.scan_pub = self.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self.case_pub = self.create_publisher(
            String, '/test/static_extent_filter/case', 10)
        self.started = time.monotonic()
        self.last_label = None
        self.create_timer(0.1, self.publish_case)
        print('STATIC EXTENT FILTER SYNTHETIC SOURCE (TEST ONLY)', flush=True)

    def current_case(self, elapsed):
        # Initial and inter-case clear periods supply at least three safe scans.
        if elapsed < 1.0:
            return 'CLEAR', None
        cursor = elapsed - 1.0
        for label, extent in self.CASES:
            if cursor < 1.2:
                return label, extent
            cursor -= 1.2
            if cursor < 0.8:
                return 'CLEAR', None
            cursor -= 0.8
        return 'DONE', None

    def base_scan(self, ranges, angle_min, angle_increment):
        message = LaserScan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'laser_frame'
        message.angle_min = float(angle_min)
        message.angle_increment = float(angle_increment)
        message.angle_max = float(angle_min + angle_increment * (len(ranges) - 1))
        message.scan_time = 0.1
        message.time_increment = message.scan_time / max(1, len(ranges))
        message.range_min = 0.03
        message.range_max = 12.0
        message.ranges = list(ranges)
        message.intensities = [100.0 if math.isfinite(v) else 0.0 for v in ranges]
        return message

    def single_cluster_scan(self, extent):
        distance = 0.50
        half_angle = math.asin(extent / (2.0 * distance))
        return self.base_scan(
            [distance, distance, distance],
            -math.pi / 2.0 - half_angle,
            half_angle,
        )

    def mixed_scan(self):
        # Dense 0.001-rad geometry. The near cluster is a 4 cm wide surface at
        # x=0.40 m; the farther cluster is a 15 cm surface at x=0.60 m.
        increment = 0.001
        angle_min = -math.pi
        ranges = [math.inf] * (round(2.0 * math.pi / increment) + 1)
        for x_value, center_y, extent in (
            (0.40, -0.15, 0.040),
            (0.60, 0.15, 0.150),
        ):
            y_min = center_y - extent / 2.0
            y_max = center_y + extent / 2.0
            base_min = math.atan2(y_min, x_value)
            base_max = math.atan2(y_max, x_value)
            first = math.ceil((base_min - YAW - angle_min) / increment)
            last = math.floor((base_max - YAW - angle_min) / increment)
            for index in range(first, last + 1):
                base_angle = angle_min + index * increment + YAW
                ranges[index] = x_value / math.cos(base_angle)
        return self.base_scan(ranges, angle_min, increment)

    def clear_scan(self):
        return self.base_scan([math.inf] * 361, -math.pi, math.pi / 180.0)

    def publish_case(self):
        elapsed = time.monotonic() - self.started
        label, extent = self.current_case(elapsed)
        if label != self.last_label:
            print(f'[SOURCE] case={label} t={elapsed:.3f}s', flush=True)
            self.last_label = label
        case_message = String()
        case_message.data = label
        self.case_pub.publish(case_message)
        if label == 'GRASS_0.040_DUMMY_0.150':
            scan = self.mixed_scan()
        elif extent is not None:
            scan = self.single_cluster_scan(extent)
        else:
            scan = self.clear_scan()
        self.scan_pub.publish(scan)


def main():
    rclpy.init()
    node = StaticExtentFilterSyntheticSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
