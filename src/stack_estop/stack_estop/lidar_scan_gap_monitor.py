#!/usr/bin/env python3
"""TEST ONLY: report FRONT/REAR LaserScan rate and message gaps."""

import json
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class TopicStats:
    def __init__(self, topic):
        self.topic = topic
        self.count = 0
        self.last_time = None
        self.max_gap = 0.0
        self.gaps_over_025 = 0
        self.gaps_over_1 = 0
        self.recent_gaps = deque(maxlen=50)

    def update(self, now_sec):
        if self.last_time is not None:
            gap = max(0.0, now_sec - self.last_time)
            self.recent_gaps.append(gap)
            self.max_gap = max(self.max_gap, gap)
            if gap > 0.25:
                self.gaps_over_025 += 1
            if gap > 1.0:
                self.gaps_over_1 += 1
        self.last_time = now_sec
        self.count += 1

    def snapshot(self, now_sec):
        mean_gap = (
            sum(self.recent_gaps) / len(self.recent_gaps)
            if self.recent_gaps else None)
        return {
            'topic': self.topic,
            'message_count': self.count,
            'current_hz': (
                None if not mean_gap or mean_gap <= 0.0 else 1.0 / mean_gap),
            'max_inter_message_gap_sec': self.max_gap,
            'gaps_over_0_25_sec': self.gaps_over_025,
            'gaps_over_1_0_sec': self.gaps_over_1,
            'last_message_age_sec': (
                None if self.last_time is None
                else max(0.0, now_sec - self.last_time)),
        }


class LidarScanGapMonitor(Node):
    def __init__(self):
        super().__init__('lidar_scan_gap_monitor')
        topic = str(self.declare_parameter('topic', '/scan').value)
        self.stats = TopicStats(topic)
        self.create_subscription(
            LaserScan, topic, self._scan_callback, qos_profile_sensor_data)
        self.create_timer(1.0, self._report)
        self.get_logger().warning(
            f'TEST ONLY LiDAR gap monitor: {topic}')

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _scan_callback(self, _message):
        self.stats.update(self._now_sec())

    def _report(self):
        now_sec = self._now_sec()
        self.get_logger().info(json.dumps(
            self.stats.snapshot(now_sec),
            separators=(',', ':'), allow_nan=False))


def main(args=None):
    rclpy.init(args=args)
    node = LidarScanGapMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
