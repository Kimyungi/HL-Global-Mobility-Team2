"""Four-LiDAR cloud -> GPS-frame fixed avoidance path -> 1 m preview."""
import math
import time
from collections import deque
from dataclasses import fields

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from fma_interfaces.msg import AvoidStatus, GpsPath, RefPoint

from stack_gps.path_engine import PathEngine, load_waypoints_csv
from .waypoint_planner import Config, Detector, FixedPlanner, AvoidSession, filter_cloud, to_global, to_vehicle


def pose_from_transform(tf):
    q, t = tf.transform.rotation, tf.transform.translation
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
    return t.x, t.y, yaw


class WaypointAvoidNode(Node):
    def __init__(self):
        super().__init__('stack_avoid_node')
        defaults = Config()
        kwargs = {}
        for field in fields(defaults):
            value = getattr(defaults, field.name)
            if isinstance(value, tuple):
                value = list(value)
            kwargs[field.name] = self.declare_parameter('waypoint_avoid.'+field.name, value).value
        kwargs['obstacle_offsets'] = tuple(kwargs['obstacle_offsets'])
        self.cfg = Config(**kwargs)
        csv = str(self.declare_parameter('waypoint_csv', '').value)
        pts, yaws = load_waypoints_csv(csv, include_yaw=True)
        if yaws is None:
            raise ValueError('waypoint avoidance requires CSV yaw_rad or yaw_deg on every point')
        self.route = PathEngine(pts, waypoint_yaws=yaws)
        self.planner = FixedPlanner(self.route, self.cfg)
        self.detector = Detector(self.route, self.cfg)
        self.cloud_topic = str(self.declare_parameter('cloud_topic', '/unified_lidar/cloud').value)
        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self.base_frame = str(self.declare_parameter('base_frame', 'base_link').value)
        self.stale_s = float(self.declare_parameter('input_stale_s', .35).value)
        self.target_speed = float(self.declare_parameter('target_speed_mps', 1.0).value)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(GpsPath, '/perception/gps_path', self.on_gps, 1)
        self.pub = self.create_publisher(AvoidStatus, '/perception/avoid', 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/perception/avoid_path_map', qos)
        self.controls_pub = self.create_publisher(Path, '/perception/avoid_controls_map', qos)
        self.preview_pub = self.create_publisher(PoseStamped, '/perception/avoid_preview_map', 1)
        self.diagnostic_pub = self.create_publisher(String, '/perception/avoid_diagnostic', 1)
        self.pending = deque(maxlen=5)
        self.gps = None
        self.gps_received = 0.0
        self.cloud_received = 0.0
        self.cloud_stamp = None
        self.last_cloud_key = None
        self.map_points = np.empty((0, 2))
        self.report = {'valid': False, 'reason': 'no path'}
        self.published_revision = -1
        self.session = AvoidSession()
        self.last_diagnostic = None
        self.create_timer(.05, self.tick)

    def on_cloud(self, msg):
        # Fused cloud is already in base_link. Do not apply a single-LiDAR
        # mount offset a second time or silently accept an unknown frame.
        if msg.header.frame_id != self.base_frame:
            self.diagnostic('cloud frame must be '+self.base_frame)
            return
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if key != self.last_cloud_key:
            self.pending.append((msg, time.monotonic()))
            self.last_cloud_key = key

    def on_gps(self, msg):
        self.gps, self.gps_received = msg, time.monotonic()

    def diagnostic(self, text):
        if text != self.last_diagnostic:
            self.last_diagnostic = text
            self.get_logger().info(text)
            self.diagnostic_pub.publish(String(data=text))

    def fresh_stamp(self, stamp):
        age = (self.get_clock().now()-Time.from_msg(stamp)).nanoseconds * 1e-9
        return -.05 <= age <= self.stale_s

    def consume_cloud(self):
        if not self.pending:
            return False
        while self.pending:
            msg, received = self.pending[0]
            if time.monotonic()-received <= self.stale_s and self.fresh_stamp(msg.header.stamp):
                break
            self.pending.popleft()
            self.detector.previous = []
        if not self.pending:
            return False
        try:
            # Wait in the timer for GPS TF interpolation at the cloud stamp.
            # No fallback to a different pose/time: that would move obstacles.
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame,
                                                 Time.from_msg(msg.header.stamp))
        except TransformException:
            return False
        points = np.array([(float(p[0]), float(p[1])) for p in
                           point_cloud2.read_points(msg, field_names=('x', 'y'), skip_nans=True)])
        self.map_points = to_global(filter_cloud(points, self.cfg), pose_from_transform(tf))
        self.cloud_received, self.cloud_stamp = received, msg.header.stamp
        self.pending.popleft()
        return True

    @staticmethod
    def pose_message(point, header):
        msg = PoseStamped()
        msg.header = header
        msg.pose.position.x, msg.pose.position.y = float(point[0]), float(point[1])
        msg.pose.orientation.z = math.sin(point[2]/2)
        msg.pose.orientation.w = math.cos(point[2]/2)
        return msg

    def publish_geometry(self):
        if self.published_revision == self.planner.revision:
            return
        path, controls = Path(), Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        controls.header = path.header
        path.poses = [self.pose_message(p, path.header) for p in self.planner.samples]
        for m in self.planner.maneuvers:
            controls.poses.extend(self.pose_message((p.x, p.y, p.yaw), path.header) for p in m.points)
        self.path_pub.publish(path)
        self.controls_pub.publish(controls)
        self.report = self.planner.geometry_report()
        self.published_revision = self.planner.revision

    def tick(self):
        now = time.monotonic()
        msg = AvoidStatus()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ttc = 1.0e9
        reason, pose, new_cloud = '', None, False
        gps_ok = (self.gps is not None and now-self.gps_received <= self.stale_s and
                  self.fresh_stamp(self.gps.header.stamp) and self.gps.fix_quality == 4 and
                  self.gps.heading_source != GpsPath.HEADING_TANGENT)
        if gps_ok:
            self.session.observe_zone(self.gps.avoid_zone)
        if not gps_ok:
            reason = 'waiting for fresh RTK FIXED and measured heading'
        else:
            try:
                tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
                if self.fresh_stamp(tf.header.stamp):
                    pose = pose_from_transform(tf)
                else:
                    reason = 'stale GPS pose TF'
            except TransformException:
                reason = 'waiting for GPS pose TF'
            if pose is not None:
                new_cloud = self.consume_cloud()
                if self.cloud_stamp is None or now-self.cloud_received > self.stale_s or not self.fresh_stamp(self.cloud_stamp):
                    reason = 'waiting for cloud and timestamp-matched GPS TF'
        if pose is not None and not reason and self.session.active:
            ego_s = self.route.project_station(*pose[:2])[0]
            if self.planner.advance(pose):
                self.session.passed_path()
            if new_cloud:
                for obstacle in self.detector.observe(self.map_points):
                    if self.planner.accept(obstacle, ego_s):
                        self.session.accepted()
            self.publish_geometry()
            self.session.finish(self.planner, pose)
            if self.session.active:
                if self.planner.samples and not self.report['valid']:
                    reason = self.report['reason']
                elif self.planner.path_blocked(self.map_points, ego_s):
                    reason = 'observed obstacle intersects the fixed vehicle corridor'
                preview = self.planner.preview(pose)
                if preview is None:
                    reason = reason or 'no forward 1 m preview intersection'
                else:
                    x, y, yaw, curvature = to_vehicle(preview, pose)
                    ref = RefPoint()
                    ref.x, ref.y, ref.yaw, ref.curvature = map(float, (x, y, yaw, curvature))
                    msg.points = [ref]
                    header = Path().header
                    header.frame_id, header.stamp = self.map_frame, msg.header.stamp
                    self.preview_pub.publish(self.pose_message(preview, header))
        msg.obstacle_detected = self.session.active
        msg.avoidable = msg.obstacle_detected and not reason and bool(msg.points)
        msg.narrow_gap = msg.obstacle_detected and bool(reason)
        msg.maneuver_done = self.session.done and gps_ok and pose is not None and not reason
        msg.v_suggest = self.target_speed if msg.avoidable else 0.0
        if msg.obstacle_detected and reason:
            # MGM owns stop decisions; an invalid active corridor supplies its
            # existing TTC safety floor. The global curve remains stored.
            msg.ttc = 0.0
        self.pub.publish(msg)
        self.diagnostic(reason or self.planner.last_reason or
                        ('fixed path active' if self.planner.samples else
                         'returning to waypoint' if self.session.returning else
                         'zone armed, following waypoint' if self.session.active else
                         'waiting for state=4 marker'))


def main(args=None):
    rclpy.init(args=args)
    node = WaypointAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
