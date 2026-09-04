"""Republish a LaserScan as PointCloud2 for the ICP node's cloud subscriber.

`lidar_fusion_v2` resolves overlapping sensor FOVs itself: `/unified_lidar/scan`
keeps only the nearest return per 1-degree bin (`points_to_virtual_scan`,
`np.minimum.at`), while `/unified_lidar/cloud` is the raw unfiltered
concatenation of all four sensors. `stack_parking_node` only takes a
PointCloud2, so this node exists solely to carry the already-deduplicated
`/unified_lidar/scan` into that input instead of the raw cloud.
"""

from __future__ import annotations

import laser_geometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2


class ScanToCloudNode(Node):
    def __init__(self):
        super().__init__('parking_scan_to_cloud')
        self.declare_parameter('input_scan_topic', '/unified_lidar/scan')
        self.declare_parameter('output_cloud_topic', '/parking/nearest_merged_cloud')
        self.projector = laser_geometry.LaserProjection()
        self.pub = self.create_publisher(
            PointCloud2, str(self.get_parameter('output_cloud_topic').value), 1)
        self.sub = self.create_subscription(
            LaserScan, str(self.get_parameter('input_scan_topic').value),
            self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg: LaserScan) -> None:
        # channel_options=NONE: x/y/z only (all float32). The default adds an
        # intensity channel in a different datatype, which trips
        # sensor_msgs_py.point_cloud2.read_points_numpy's same-dtype
        # assertion in stack_parking_node's _cloud_xy.
        self.pub.publish(self.projector.projectLaser(
            msg, channel_options=laser_geometry.LaserProjection.ChannelOption.NONE))


def main(args=None):
    rclpy.init(args=args)
    node = ScanToCloudNode()
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
