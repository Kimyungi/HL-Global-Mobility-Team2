#!/usr/bin/env python3
"""Independent shadow diagnostic; deliberately not wired to the vehicle E-stop."""
import time

import rclpy
from rclpy.node import Node
from fma_interfaces.msg import AvoidPlan
from std_msgs.msg import Bool


class Watchdog(Node):
    def __init__(self):
        super().__init__('avoid_v2_watchdog')
        self.received = None
        self.valid = False
        self.until_ns = 0
        self.generation = 0
        self.episode = 0
        self.generation_received = None
        self.sub = self.create_subscription(AvoidPlan, '/avoid_v2/plan', self.observe, 1)
        self.pub = self.create_publisher(Bool, '/avoid_v2/watchdog_stop', 1)
        self.timer = self.create_timer(.01, self.tick)

    def observe(self, msg):
        self.received = time.monotonic()
        self.valid = (msg.plan_valid if msg.perception_active else
                      msg.gps_follow and msg.phase in (msg.READY, msg.COMPLETE))
        self.until_ns = msg.valid_until.sec * 10**9 + msg.valid_until.nanosec
        if (msg.episode_id, msg.observation_generation) != (self.episode, self.generation):
            self.episode, self.generation = msg.episode_id, msg.observation_generation
            self.generation_received = self.received

    def tick(self):
        now = time.monotonic()
        stop = (not self.valid or self.received is None or now-self.received > .15
                or self.generation_received is None or now-self.generation_received > .15
                or self.get_clock().now().nanoseconds >= self.until_ns)
        self.pub.publish(Bool(data=stop))


def main():
    rclpy.init()
    node = Watchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
