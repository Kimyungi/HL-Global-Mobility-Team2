"""Read-only four-LiDAR ICP SLAM viewer.

This node deliberately has no parking mission, GPS, vehicle, or MGM interfaces.
It only consumes the merged point cloud and publishes SLAM debug products.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from .geometry import transform_points
from .icp_slam import IcpConfig, IcpSlam, voxel_downsample


class SlamOnlyNode(Node):
    def __init__(self):
        super().__init__('parking_slam_only')
        self.declare_parameter('merged_cloud_topic', '/lidar/merged_cloud')
        self.declare_parameter('map_frame', 'parking_map')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('scan_voxel_m', 0.06)
        self.declare_parameter('max_scan_points', 900)
        self.declare_parameter('map_voxel_m', 0.08)
        self.declare_parameter('max_correspondence_m', 0.35)
        self.declare_parameter('max_iterations', 18)
        self.declare_parameter('min_correspondences', 24)
        self.declare_parameter('max_rmse_m', 0.16)
        self.declare_parameter('local_map_radius_m', 8.0)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.slam = IcpSlam(IcpConfig(
            scan_voxel_m=float(self.get_parameter('scan_voxel_m').value),
            max_scan_points=int(self.get_parameter('max_scan_points').value),
            map_voxel_m=float(self.get_parameter('map_voxel_m').value),
            max_correspondence_m=float(self.get_parameter('max_correspondence_m').value),
            max_iterations=int(self.get_parameter('max_iterations').value),
            min_correspondences=int(self.get_parameter('min_correspondences').value),
            max_rmse_m=float(self.get_parameter('max_rmse_m').value),
            local_map_radius_m=float(self.get_parameter('local_map_radius_m').value),
        ))
        self.pose_pub = self.create_publisher(PoseStamped, '/parking/slam_pose', 1)
        self.scan_pub = self.create_publisher(PointCloud2, '/parking/slam_scan', 1)
        self.map_pub = self.create_publisher(PointCloud2, '/parking/local_map', 1)
        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/parking/slam_diagnostics', 10)
        self.last_publish_ns = 0
        self.publish_period_ns = int(
            1.0e9 / max(0.1, float(self.get_parameter('publish_rate_hz').value)))
        topic = str(self.get_parameter('merged_cloud_topic').value)
        self.create_subscription(
            PointCloud2, topic, self._on_cloud, qos_profile_sensor_data)
        self.get_logger().info('SLAM-only ready: %s -> parking_map (no MGM outputs)' % topic)

    @staticmethod
    def _cloud_xy(msg: PointCloud2) -> np.ndarray:
        points = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y'), skip_nans=True)
        array = np.asarray(points)
        if array.dtype.names:
            return np.column_stack((array['x'], array['y'])).astype(np.float64)
        return np.asarray(array, dtype=np.float64).reshape((-1, 2))

    def _on_cloud(self, msg: PointCloud2) -> None:
        points = self._cloud_xy(msg)
        result = self.slam.update(points)
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_publish_ns < self.publish_period_ns:
            return
        self.last_publish_ns = now_ns
        header = Header(stamp=msg.header.stamp, frame_id=self.map_frame)
        prepared = voxel_downsample(points[np.isfinite(points).all(axis=1)], 0.05)
        registered = transform_points(prepared, result.pose)
        self.scan_pub.publish(point_cloud2.create_cloud_xyz32(
            header, [(float(x), float(y), 0.0) for x, y in registered]))
        map_points = self.slam.map_points()
        self.map_pub.publish(point_cloud2.create_cloud_xyz32(
            header, [(float(x), float(y), 0.0) for x, y in map_points]))
        pose = PoseStamped(header=header)
        pose.pose.position.x = float(result.pose.x)
        pose.pose.position.y = float(result.pose.y)
        pose.pose.orientation.z = math.sin(0.5 * result.pose.yaw)
        pose.pose.orientation.w = math.cos(0.5 * result.pose.yaw)
        self.pose_pub.publish(pose)

        status = DiagnosticStatus(
            level=DiagnosticStatus.OK if result.accepted else DiagnosticStatus.WARN,
            name='stack_parking/slam_only', hardware_id='four_lidar_icp',
            message=result.reason,
            values=[
                KeyValue(key='accepted', value=str(result.accepted)),
                KeyValue(key='rmse_m', value=str(result.rmse_m)),
                KeyValue(key='matches', value=str(result.correspondences)),
                KeyValue(key='map_points', value=str(len(map_points))),
            ])
        self.diag_pub.publish(DiagnosticArray(header=header, status=[status]))


def main(args=None):
    rclpy.init(args=args)
    node = SlamOnlyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
