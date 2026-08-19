#!/usr/bin/env python3
"""TEST ONLY: deterministic LaserScan source for offline recovery validation."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class OfflineRecoverySyntheticSource(Node):
    def __init__(self):
        super().__init__('offline_recovery_synthetic_source')
        self.front_pub = self.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self.rear_pub = self.create_publisher(
            LaserScan, '/rear/scan', qos_profile_sensor_data)
        self.started = time.monotonic()
        self.last_phase = None
        self.create_timer(0.1, self.publish_scans)
        print('OFFLINE RECOVERY SYNTHETIC SOURCE (TEST ONLY)', flush=True)

    @staticmethod
    def make_scan(frame_id, obstacle_range=None, background_range=None):
        msg = LaserScan()
        msg.header.frame_id = frame_id
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = math.radians(1.0)
        msg.scan_time = 0.1
        msg.time_increment = msg.scan_time / 361.0
        msg.range_min = 0.03
        msg.range_max = 12.0
        fill = math.inf if background_range is None else float(background_range)
        msg.ranges = [fill] * 361
        msg.intensities = [0.0] * 361
        if obstacle_range is not None:
            # Vehicle forward is laser -90 degrees. Use a nine-point cluster.
            center = round((-math.pi / 2.0 - msg.angle_min) / msg.angle_increment)
            for offset in range(-4, 5):
                msg.ranges[center + offset] = float(obstacle_range)
                msg.intensities[center + offset] = 100.0
        return msg

    def publish_scans(self):
        elapsed = time.monotonic() - self.started
        if elapsed < 2.0:
            phase, front_range = 'CLEAR_PREROLL', None
        elif elapsed < 3.0:
            # Make the obstacle-present status fresh before hard E-Stop ON.
            phase, front_range = 'OBSTACLE_PREARM_1.50M', 1.50
        elif elapsed < 14.0:
            phase, front_range = 'HARD_STOP_0.50M', 0.50
        else:
            # Outside hard-stop range but still visible to stack_avoid.
            phase, front_range = 'AVOIDABLE_1.50M', 1.50

        if phase != self.last_phase:
            print(f'[SOURCE] phase={phase} t={elapsed:.3f}s', flush=True)
            self.last_phase = phase

        stamp = self.get_clock().now().to_msg()
        front = self.make_scan('laser_frame', front_range)
        # A real rear scan contains valid returns outside the short rear ROI.
        # Keeping finite far returns also lets the recovery node distinguish a
        # healthy, clear scan from a scan containing no usable measurements.
        rear = self.make_scan('rear_laser_frame', None, background_range=5.0)
        front.header.stamp = stamp
        rear.header.stamp = stamp
        self.front_pub.publish(front)
        self.rear_pub.publish(rear)


def main():
    rclpy.init()
    node = OfflineRecoverySyntheticSource()
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
