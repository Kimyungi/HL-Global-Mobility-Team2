#!/usr/bin/env python3
"""T_Parking — bench test: synthetic straight-line CAN drive.

WARNING: wall_gap_node no longer asserts ``/wall_gap/stop`` when a square is
confirmed. Rear-LiDAR final-stop logic is pending, so do not run this tool
unattended with the current wall-gap test stack.

Publishes ``fma_interfaces/TargetRef`` on ``/adas/target_ref`` at the
protocol's 10ms cadence (PROTOCOL.md §3 TX), standing in for adas_mgm so
bridge_dspace/dSPACE can be driven without the full MGM stack:
  - state = STATE_LANE (0) — stateUsesGpsDelta() in can_protocol.hpp only
    forwards dx/dy/dyaw/update for LANE/WAYPOINT/TRAFFIC, so LANE is the
    state that actually carries them over CAN.
  - ref_points = one point x=1.0, y=0.0, yaw=0.0, curvature=0.0 — a
    straight-ahead target 1m out, matching the v5 single-point contract.
  - dx=1.0, dy=0.0, dyaw=0.0 — the equivalent straight-line GNSS delta.
  - update increments by 1 every 10 ticks (10Hz fix rate inside the 100Hz
    TX loop, same ratio as the real GPS-vs-CAN cadence).
  - v_ref = target_speed_mps (0.3 by default) until an independent publisher
    asserts ``/wall_gap/stop``, then 0.0.

``counter`` is not part of TargetRef — bridge_dspace assigns it at send
time (see can_bridge_node.cpp sendFrames()), one CAN cycle per call.

This node only provides the straight-line drive and reacts to a stop signal;
it does not decide when stopping is safe.
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

        self.ref_pub = self.create_publisher(TargetRef, '/adas/target_ref', 1)
        self.stop_sub = self.create_subscription(
            Bool, '/wall_gap/stop', self._on_stop, 10)

        self.tick_count = 0
        self.timer = self.create_timer(0.01, self._tick)  # 10ms = protocol TX rate
        self.get_logger().info(
            'T_Parking ready: v=%.2fm/s, waiting on /wall_gap/stop' % self.target_speed)

    def _on_stop(self, msg: Bool) -> None:
        if msg.data and not self.stopped:
            self.stopped = True
            self.get_logger().info('wall_gap_node reports space confirmed — stopping (v_ref -> 0)')
        elif not msg.data and self.stopped:
            # wall_gap_node restarted its search — resume driving.
            self.stopped = False
            self.get_logger().info('wall_gap_node cleared stop — resuming drive')

    def _tick(self) -> None:
        self.tick_count += 1
        msg = TargetRef()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.ref_points = [RefPoint(x=1.0, y=0.0, yaw=0.0, curvature=0.0)]
        msg.v_ref = 0.0 if self.stopped else self.target_speed
        msg.state = TargetRef.STATE_LANE
        msg.dx = 1.0
        msg.dy = 0.0
        msg.dyaw = 0.0
        msg.update = self.tick_count // 10
        self.ref_pub.publish(msg)


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
