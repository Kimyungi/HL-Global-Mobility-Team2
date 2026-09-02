#!/usr/bin/env python3
"""Record /parking/local_map + /parking/slam_pose to a .npz fixture.

There was no existing way to replay a mapped scene offline — every check
this session was a live query against the running stack, lost as soon as
the process died. This exists so a scene (e.g. a specific room layout) can
be captured once and reused to test detector changes without the full
lidar/CAN/SLAM stack running.

Usage:
  python3 record_map.py --out room1.npz
  python3 record_map.py --out room1.npz --seconds 10 --map-topic /parking/local_map

Replay (in Python): np.load('room1.npz') -> 'map' (Nx2), 'pose' (x,y,yaw).
"""

from __future__ import annotations

import argparse
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True, help='output .npz path')
    parser.add_argument('--map-topic', default='/parking/local_map')
    parser.add_argument('--pose-topic', default='/parking/slam_pose')
    parser.add_argument('--seconds', type=float, default=5.0,
                        help='how long to wait for fresh map+pose messages')
    args = parser.parse_args()

    rclpy.init()
    node = Node('record_map')
    result: dict = {}

    def on_map(msg: PointCloud2) -> None:
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y'), skip_nans=True)
        result['map'] = (
            np.column_stack((pts['x'], pts['y'])) if pts.dtype.names
            else np.asarray(pts).reshape((-1, 2)))

    def on_pose(msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        result['pose'] = np.array(
            [msg.pose.position.x, msg.pose.position.y, yaw], dtype=np.float64)

    node.create_subscription(PointCloud2, args.map_topic, on_map, qos_profile_sensor_data)
    node.create_subscription(PoseStamped, args.pose_topic, on_pose, qos_profile_sensor_data)

    import time
    t0 = time.time()
    while ('map' not in result or 'pose' not in result) and time.time() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if 'map' not in result or 'pose' not in result:
        print('timed out waiting for %s / %s — is the parking stack running?'
              % (args.map_topic, args.pose_topic))
        raise SystemExit(1)

    np.savez(args.out, map=result['map'], pose=result['pose'])
    print('saved %s: %d map points, pose=%s' % (
        args.out, len(result['map']), result['pose']))


if __name__ == '__main__':
    main()
