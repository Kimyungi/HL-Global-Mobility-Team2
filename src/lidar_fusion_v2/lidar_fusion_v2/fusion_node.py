"""Four fixed LaserScans in, one base_link cloud and one virtual LaserScan out."""

import math
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from .geometry import SensorGeometry, points_to_virtual_scan, scan_to_base


class FusionNode(Node):
    def __init__(self):
        super().__init__('unified_lidar_v2')
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.sensor_ids = list(self.declare_parameter(
            'sensor_ids', ['a1', 'a2', 'b1', 'b2']).value)
        self.publish_rate = float(self.declare_parameter('publish_rate_hz', 10.0).value)
        self.max_age = float(self.declare_parameter('max_age_s', 0.30).value)
        self.scan_increment = math.radians(float(self.declare_parameter(
            'scan.angle_increment_deg', 1.0).value))
        self.scan_range_min = float(self.declare_parameter('scan.range_min', 0.15).value)
        self.scan_range_max = float(self.declare_parameter('scan.range_max', 12.0).value)

        self.geometry = {}
        self.latest = {}
        self.received_monotonic = {}
        self._scan_subscriptions = []
        for sid in self.sensor_ids:
            p = 'sensors.' + sid + '.'
            topic = self.declare_parameter(p + 'topic', f'/lidar/{sid}/scan').value
            geom = SensorGeometry(
                sensor_id=sid,
                x=float(self.declare_parameter(p + 'x', 0.0).value),
                y=float(self.declare_parameter(p + 'y', 0.0).value),
                yaw_deg=float(self.declare_parameter(p + 'yaw_deg', 0.0).value),
                fov_min_deg=float(self.declare_parameter(p + 'fov_min_deg', -180.0).value),
                fov_max_deg=float(self.declare_parameter(p + 'fov_max_deg', 180.0).value),
                min_range=float(self.declare_parameter(p + 'min_range', 0.15).value),
                max_range=float(self.declare_parameter(p + 'max_range', 12.0).value),
                range_offset=float(self.declare_parameter(p + 'range_offset_m', 0.0).value),
            )
            self.geometry[sid] = geom
            self._scan_subscriptions.append(self.create_subscription(
                LaserScan, topic, lambda msg, key=sid: self._on_scan(key, msg),
                qos_profile_sensor_data))
            self.get_logger().info(
                f'{sid}: {topic} pose=({geom.x:+.3f},{geom.y:+.3f},'
                f'{geom.yaw_deg:+.1f}deg) FOV={geom.fov_min_deg:+.1f}..'
                f'{geom.fov_max_deg:+.1f}deg')

        self.cloud_pub = self.create_publisher(
            PointCloud2, '/unified_lidar/cloud', qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(
            LaserScan, '/unified_lidar/scan', qos_profile_sensor_data)
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        self.raw_pubs = {
            sid: self.create_publisher(
                PointCloud2, f'/unified_lidar/raw/{sid}', qos_profile_sensor_data)
            for sid in self.sensor_ids
        }
        self.timer = self.create_timer(1.0 / max(self.publish_rate, 1.0), self._publish)
        self.last_active = None

    def _on_scan(self, sensor_id, msg):
        self.latest[sensor_id] = msg
        self.received_monotonic[sensor_id] = time.monotonic()

    def _publish(self):
        now_mono = time.monotonic()
        clouds = []
        sensor_clouds = []
        active = []
        for sid in self.sensor_ids:
            msg = self.latest.get(sid)
            age = now_mono - self.received_monotonic.get(sid, -1e9)
            if msg is None or age > self.max_age:
                continue
            cloud = scan_to_base(msg.ranges, msg.angle_min, msg.angle_increment,
                                 self.geometry[sid])
            if cloud.size:
                clouds.append(cloud)
                sensor_clouds.append((sid, cloud))
                active.append(sid)
        if not clouds:
            if self.last_active != ():
                self.get_logger().warning('active sensors 0; unified output paused')
                self.last_active = ()
            return

        points = np.concatenate(clouds, axis=0)
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self.base_frame)
        for sid, cloud in sensor_clouds:
            raw_xyz = np.column_stack(
                (cloud, np.zeros(cloud.shape[0], dtype=np.float32)))
            self.raw_pubs[sid].publish(
                point_cloud2.create_cloud(header, self.fields, raw_xyz.tolist()))
        xyz = np.column_stack((points, np.zeros(points.shape[0], dtype=np.float32)))
        self.cloud_pub.publish(point_cloud2.create_cloud(header, self.fields, xyz.tolist()))

        ranges = points_to_virtual_scan(
            points, self.scan_increment, self.scan_range_min, self.scan_range_max)
        scan = LaserScan()
        scan.header = header
        scan.angle_min = -math.pi
        scan.angle_increment = self.scan_increment
        scan.angle_max = scan.angle_min + (len(ranges) - 1) * scan.angle_increment
        scan.scan_time = 1.0 / max(self.publish_rate, 1.0)
        scan.time_increment = 0.0
        scan.range_min = self.scan_range_min
        scan.range_max = self.scan_range_max
        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)

        state = tuple(active)
        if state != self.last_active:
            self.get_logger().info(
                f'active={list(active)} points={points.shape[0]} bins={len(ranges)}')
            self.last_active = state


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
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
