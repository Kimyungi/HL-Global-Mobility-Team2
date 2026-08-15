#!/usr/bin/env python3
"""Publish the minimum synthetic perception input needed to hold MGM in LANE."""

import argparse

import rclpy
from rclpy.node import Node

from fma_interfaces.msg import LanePath, RefPoint


class TestMgmInputs(Node):
    def __init__(self, publish_hz):
        super().__init__('test_mgm_inputs')
        self.publisher = self.create_publisher(
            LanePath, '/perception/lane_path', 1
        )
        self.timer = self.create_timer(1.0 / publish_hz, self.publish_lane)
        self.get_logger().warning(
            'TEST INPUT ONLY: publishing a synthetic straight LanePath. '
            'This node does not send CAN directly, but MGM and a running '
            'bridge can convert this path into a vehicle command.'
        )

    def publish_lane(self):
        msg = LanePath()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.confidence = 1.0
        for index in range(20):
            point = RefPoint()
            # x=0인 첫 점은 lookahead가 없어 저속에서도 위빙을 유발했다.
            point.x = 0.5 * (index + 1)
            point.y = 0.0
            point.yaw = 0.0
            point.curvature = 0.0
            msg.points.append(point)
        self.publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rate', type=float, default=10.0)
    args, ros_args = parser.parse_known_args()
    if args.rate <= 0.0:
        parser.error('--rate must be > 0')

    rclpy.init(args=ros_args)
    node = TestMgmInputs(args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
