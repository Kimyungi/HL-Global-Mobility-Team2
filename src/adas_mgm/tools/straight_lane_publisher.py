#!/usr/bin/env python3
"""Indoor road-test source: publish one fixed straight LanePath."""

import rclpy
from rclpy.node import Node

from fma_interfaces.msg import LanePath, RefPoint


class StraightLanePublisher(Node):
    def __init__(self):
        super().__init__('straight_lane_publisher')
        self.declare_parameter('lookahead_m', 2.0)
        self.declare_parameter('publish_hz', 20.0)
        lookahead = float(self.get_parameter('lookahead_m').value)
        hz = float(self.get_parameter('publish_hz').value)
        if lookahead <= 0.0 or hz <= 0.0:
            raise ValueError('lookahead_m and publish_hz must be > 0')
        self.lookahead = lookahead
        self.publisher = self.create_publisher(LanePath, '/perception/lane_path', 1)
        self.timer = self.create_timer(1.0 / hz, self.publish_path)
        self.get_logger().info(
            f'fixed straight LanePath: x={lookahead:.2f}m, y/yaw/curvature=0')

    def publish_path(self):
        msg = LanePath()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.confidence = 1.0
        point = RefPoint()
        point.x = self.lookahead
        point.y = 0.0
        point.yaw = 0.0
        point.curvature = 0.0
        msg.points = [point]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StraightLanePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
