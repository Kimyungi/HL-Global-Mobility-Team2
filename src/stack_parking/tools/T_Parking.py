#!/usr/bin/env python3
"""T_Parking — bench lead-in: synthetic straight-line CAN drive.

Publishes ``fma_interfaces/TargetRef`` on ``/adas/target_ref`` at the
protocol's 10ms cadence (PROTOCOL.md §3 TX), standing in for adas_mgm so
bridge_dspace/dSPACE can be driven without the full MGM stack:
  - state = STATE_LANE (0) — stateUsesPoseDelta() in can_protocol.hpp
    forwards its synthetic pose delta over CAN.
  - ref_points = one point x=1.0, y=0.0, yaw=0.0, curvature=0.0 — a
    straight-ahead target 1m out, matching the v5 single-point contract.
  - dx=1.0, dy=0.0, dyaw=0.0 — the equivalent straight-line GNSS delta.
  - update increments by 1 every 10 ticks (10Hz fix rate inside the 100Hz
    TX loop, same ratio as the real GPS-vs-CAN cadence).
  - v_ref = target_speed_mps (0.3 by default) until wall_gap_node confirms a
    square and asserts ``/wall_gap/stop``.

On that first stop edge this node sends one final zero-speed frame and then
permanently stops publishing. That handoff leaves an enabled wall_gap_node as
the sole ``/adas/target_ref`` publisher for the hold/alignment/reverse phases.
Restart this process for a new run; do not run adas_mgm at the same time.

``counter`` is not part of TargetRef — bridge_dspace assigns it at send
time (see can_bridge_node.cpp sendFrames()), one CAN cycle per call.

This node only provides the straight-line lead-in. It does not decide when
stopping or reversing is safe.
"""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from fma_interfaces.msg import RefPoint, TargetRef
from std_msgs.msg import Bool


class TParkingNode(Node):

    def __init__(self):
        super().__init__('t_parking_test')
        self.declare_parameter('target_speed_mps', 0.3)

        self.target_speed = float(self.get_parameter('target_speed_mps').value)
        self.stopped = False
        self.handed_off = False

        self.ref_pub = self.create_publisher(TargetRef, '/adas/target_ref', 1)
        self.stop_sub = self.create_subscription(
            Bool, '/wall_gap/stop', self._on_stop, 10)

        self.tick_count = 0
        self.timer = self.create_timer(0.01, self._tick)  # 10ms = protocol TX rate
        self.get_logger().info(
            'T_Parking ready: v=%.2fm/s, waiting on /wall_gap/stop' % self.target_speed)

    def _on_stop(self, msg: Bool) -> None:
        if msg.data and not self.handed_off:
            self.stopped = True
            self._publish_once(0.0)
            self.handed_off = True
            self.timer.cancel()
            self.get_logger().info(
                'wall_gap_node confirmed a square — sent v_ref=0 once and '
                'handed /adas/target_ref ownership to wall_gap_node')

    def _publish_once(self, speed: float) -> None:
        self.tick_count += 1
        msg = TargetRef()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.ref_points = [RefPoint(
            x=1.0, y=0.0, yaw=0.0, curvature=0.0)]
        msg.v_ref = float(speed)
        msg.state = TargetRef.STATE_LANE
        msg.dx = 1.0
        msg.dy = 0.0
        msg.dyaw = 0.0
        msg.update = self.tick_count // 10
        self.ref_pub.publish(msg)

    def _tick(self) -> None:
        if not self.handed_off:
            self._publish_once(self.target_speed)


def main(args=None):
    rclpy.init(args=args)
    node = TParkingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
