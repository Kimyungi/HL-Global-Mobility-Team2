#!/usr/bin/env python3
"""Publish an explicit, atomic course; never infer boundaries from a GPS preview."""
import json
import math
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from fma_interfaces.msg import AvoidCourse, RefPoint


def read_course(path):
    data = json.loads(Path(path).read_text())
    msg = AvoidCourse()
    msg.header.frame_id = data['frame_id']
    msg.route_id = data['route_id']
    if not isinstance(msg.route_id, str) or not msg.route_id or not msg.header.frame_id:
        raise ValueError('route identity and fixed frame required')
    for field, minimum, maximum in [('boundary', 3, 2000)]:
        rows = data[field]
        if not minimum <= len(rows) <= maximum:
            raise ValueError(f'invalid {field} length')
        for row in rows:
            if len(row) != 2 or not all(math.isfinite(v) for v in row):
                raise ValueError(f'invalid {field} point')
            getattr(msg, field).append(RefPoint(x=float(row[0]), y=float(row[1])))
    msg.entry_station_m = float(data['entry_station_m'])
    msg.exit_station_m = float(data['exit_station_m'])
    if not 0 <= msg.entry_station_m < msg.exit_station_m or not math.isfinite(msg.exit_station_m):
        raise ValueError('invalid entry/exit station')
    return msg


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)
    if len(args) != 2:
        raise SystemExit('usage: publish_course.py course.json')
    msg = read_course(args[1])
    rclpy.init()
    node = Node('avoid_v2_course_source')
    pub = node.create_publisher(AvoidCourse, '/avoid_v2/course',
                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
