#!/usr/bin/env python3
"""ROS wrapper for wall_gap_detector.py — runs against the live SLAM map,
publishes RViz markers for the wall lines, gap candidates, and the
inscribed-square feasibility test.

Does not touch stack_parking_node or space_detector.py — independent
experiment, reads the same /parking/local_map + /parking/slam_pose that are
already being published.

Markers on /wall_gap/markers (ns):
  walls      — one line per detected wall segment, left=blue right=orange,
               projected onto the map-fixed reference wall
  wall_reference — the first fitted wall, extended over the observed span
  wall_offset — the two boundaries of the accepted wall-point offset band
  candidates — one small disc per tracked gap candidate, yellow=untested,
               green=clear, red=blocked
  squares    — the 1m x 1m inscribed-square outline for every *tested*
               candidate (green outline if clear, red if blocked)
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import Pose2
from .reference_path import ReferencePath, build_reference_path
from .wall_gap_detector import (
    SIDE_LEFT,
    SIDE_RIGHT,
    WallGapConfig,
    WallGapDetector,
    candidate_square_corners,
)


def _rgba(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    return ColorRGBA(r=r, g=g, b=b, a=a)


class WallGapNode(Node):

    def __init__(self):
        super().__init__('wall_gap_node')
        self.declare_parameter('map_topic', '/parking/local_map')
        self.declare_parameter('pose_topic', '/parking/slam_pose')
        self.declare_parameter('map_frame', 'parking_map')
        self.declare_parameter('near_m', 0.3)
        self.declare_parameter('far_m', 1.6)
        self.declare_parameter('wall_line_offset_m', 0.12)
        self.declare_parameter('initial_wall_min_points', 6)
        self.declare_parameter('initial_wall_min_length_m', 0.5)
        self.declare_parameter('initial_wall_max_angle_deg', 45.0)
        self.declare_parameter('join_gap_m', 0.3)
        self.declare_parameter('min_segment_points', 3)
        self.declare_parameter('min_gap_m', 1.2)
        self.declare_parameter('square_size_m', 1.0)
        self.declare_parameter('candidate_max_ahead_m', 5.0)
        self.declare_parameter('candidate_max_behind_m', 1.0)
        self.declare_parameter('reach_tolerance_m', 0.3)
        self.declare_parameter('dedup_tolerance_m', 0.5)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('min_turn_radius_m', 1.15)
        self.declare_parameter('inside_straight_m', 2.0)
        self.declare_parameter('parallel_straight_m', 3.0)
        self.declare_parameter('search_side', 'left')

        search_side = str(self.get_parameter('search_side').value).strip().lower()
        search_sides = (
            (SIDE_LEFT,) if search_side == 'left'
            else (SIDE_RIGHT,) if search_side == 'right'
            else (SIDE_LEFT, SIDE_RIGHT))
        cfg = WallGapConfig(
            near_m=float(self.get_parameter('near_m').value),
            far_m=float(self.get_parameter('far_m').value),
            wall_line_offset_m=float(
                self.get_parameter('wall_line_offset_m').value),
            initial_wall_min_points=int(
                self.get_parameter('initial_wall_min_points').value),
            initial_wall_min_length_m=float(
                self.get_parameter('initial_wall_min_length_m').value),
            initial_wall_max_angle_deg=float(
                self.get_parameter('initial_wall_max_angle_deg').value),
            join_gap_m=float(self.get_parameter('join_gap_m').value),
            min_segment_points=int(self.get_parameter('min_segment_points').value),
            min_gap_m=float(self.get_parameter('min_gap_m').value),
            square_size_m=float(self.get_parameter('square_size_m').value),
            candidate_max_ahead_m=float(self.get_parameter('candidate_max_ahead_m').value),
            candidate_max_behind_m=float(self.get_parameter('candidate_max_behind_m').value),
            reach_tolerance_m=float(self.get_parameter('reach_tolerance_m').value),
            dedup_tolerance_m=float(self.get_parameter('dedup_tolerance_m').value),
            search_sides=search_sides,
        )
        self.detector = WallGapDetector(cfg)
        # Seeded once from the first pose received (see _tick). The detector
        # uses this pose only to acquire its initial side wall; the fitted
        # line is then fixed in parking_map for the rest of the run.
        self.seeded = False
        self.seeded_at_s = 0.0
        self.last_reset_send_s = -1
        self.final_cancel_sent = False
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.min_turn_radius_m = float(self.get_parameter('min_turn_radius_m').value)
        self.inside_straight_m = float(
            self.get_parameter('inside_straight_m').value)
        self.parallel_straight_m = float(
            self.get_parameter('parallel_straight_m').value)

        self.latest_map = np.empty((0, 2), dtype=np.float64)
        self.latest_pose = None

        self.confirmed_candidate = None
        self.reference_path: ReferencePath | None = None

        self.map_sub = self.create_subscription(
            PointCloud2, str(self.get_parameter('map_topic').value),
            self._on_map, qos_profile_sensor_data)
        self.pose_sub = self.create_subscription(
            PoseStamped, str(self.get_parameter('pose_topic').value),
            self._on_pose, qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/wall_gap/markers', 1)
        self.stop_pub = self.create_publisher(Bool, '/wall_gap/stop', 1)
        # /wall_gap/stop is intentionally kept false for now. Final parking
        # stop will be implemented from the rear LiDAR independently of path
        # creation; confirming a square must not stop the forward alignment.
        # Every fresh test run must map from scratch (user directive,
        # 2026-09-02) — stack_parking_node already wipes its SLAM map on a
        # manual_command trigger when reset_map_on_mission_start is true
        # (the node.py default since the same date), so reuse that instead
        # of adding a second reset mechanism.
        self.command_pub = self.create_publisher(String, '/parking/manual_command', 10)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(1.0, rate), self._tick)
        self.get_logger().info(
            'wall_gap_node ready: initial_band=%.2f..%.2f wall_offset=+/-%.2f '
            'min_gap=%.2f square=%.2f'
            % (cfg.near_m, cfg.far_m, cfg.wall_line_offset_m,
               cfg.min_gap_m, cfg.square_size_m))

    def _clock_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_map(self, msg: PointCloud2) -> None:
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y'), skip_nans=True)
        self.latest_map = (
            np.column_stack((pts['x'], pts['y'])) if pts.dtype.names
            else np.asarray(pts).reshape((-1, 2)))

    def _on_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.latest_pose = Pose2(msg.pose.position.x, msg.pose.position.y, yaw)

    def _tick(self) -> None:
        if self.latest_pose is None or len(self.latest_map) == 0:
            return
        pose = self.latest_pose

        if not self.seeded:
            self.seeded = True
            self.seeded_at_s = self._clock_s()
            self.get_logger().info(
                'requesting a fresh SLAM map before reference-wall acquisition')

        # Resend for a few seconds, not once — a single early publish can be
        # lost to DDS discovery latency before stack_parking_node's
        # subscriber has matched (no error, it's just gone; same issue
        # T_Parking.py hit earlier this session). "cancel" first: if
        # stack_parking_node's *own* mission (space_detector.py, separate
        # from wall_gap_node's own detection) already ran past SCANNING
        # from an earlier trigger, "start" alone is ignored (only
        # IDLE/COMPLETE accept a new trigger) and the map never resets.
        elapsed_since_seed = self._clock_s() - self.seeded_at_s
        # Once per second for ~1.5s (2 sends), not every tick and not for
        # the full DDS-discovery-safety window: each cancel+start pair also
        # re-arms stack_parking_node's *own* mission/space_detector.py
        # (separate from wall_gap_node's own detection), which in this
        # small room finds and freezes a plan within ~250ms of every reset
        # — resending for 5s made the map cycle reset/freeze every second
        # and barely grow. 2 sends is enough for the discovery race without
        # fighting the map that badly.
        if elapsed_since_seed <= 1.5 and int(elapsed_since_seed) != self.last_reset_send_s:
            self.last_reset_send_s = int(elapsed_since_seed)
            self.command_pub.publish(String(data='cancel'))
            self.command_pub.publish(String(data='start perpendicular'))
        elif not self.final_cancel_sent and elapsed_since_seed > 1.5:
            # One more "cancel" right after the reset window: it returns
            # stack_parking_node's own mission (space_detector.py, separate
            # from wall_gap_node's own detection) to IDLE and *keeps it
            # there* — since observe_map() only ever runs while
            # state==SCANNING, IDLE means that mission can never freeze the
            # shared map again, so wall_gap_node's map keeps growing.
            self.final_cancel_sent = True
            self.command_pub.publish(String(data='cancel'))
            # stack_parking_node resets both the map and its published pose to
            # zero. Capture the seed only after that reset window; otherwise
            # a pre-reset pose could be mixed with post-reset map points.
            seed_side = self.detector.config.search_sides[0]
            self.detector.set_seed(pose, side=seed_side)
            self.get_logger().info(
                'reference-wall acquisition seeded in fresh %s at '
                '(%.2f,%.2f), side=%s'
                % (self.map_frame, pose.x, pose.y, seed_side))

        if self.detector.seed_pose is None:
            self.stop_pub.publish(Bool(data=False))
            self._publish_markers()
            return

        reference_sides_before = set(self.detector.reference_walls)
        cleared = self.detector.update(self.latest_map, pose)
        for side in set(self.detector.reference_walls) - reference_sides_before:
            wall = self.detector.reference_walls[side]
            self.get_logger().info(
                'reference wall locked in %s: side=%s yaw=%.1fdeg '
                'distance=%.2fm offset=+/-%.2fm'
                % (self.map_frame, side, math.degrees(wall.yaw),
                   wall.distance_from_seed_m,
                   self.detector.config.wall_line_offset_m))
        if cleared is not None and self.confirmed_candidate is None:
            self.confirmed_candidate = cleared
            self.reference_path = build_reference_path(
                cleared, pose, self.min_turn_radius_m,
                inside_straight_m=self.inside_straight_m,
                parallel_straight_m=self.parallel_straight_m,
            )
            if self.reference_path is None:
                self.get_logger().error(
                    'usable space confirmed, but reference-path parameters '
                    'must all be positive')
            else:
                self.get_logger().info(
                    'usable space confirmed: side=%s map=(%.2f,%.2f) width=%.2fm '
                    '— path ready immediately (parallel=%.2fm radius=%.2fm '
                    'inside=%.2fm); rear-LiDAR stop pending'
                    % (cleared.side, cleared.map_x, cleared.map_y, cleared.width_m,
                       self.parallel_straight_m, self.min_turn_radius_m,
                       self.inside_straight_m))

        self.stop_pub.publish(Bool(data=False))
        self._publish_markers()

    def _line_marker(self, marker_id: int, p0, p1, color: ColorRGBA, ns: str,
                     width: float = 0.04) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = width
        marker.color = color
        marker.points = [
            Point(x=float(p0[0]), y=float(p0[1]), z=0.0),
            Point(x=float(p1[0]), y=float(p1[1]), z=0.0),
        ]
        return marker

    def _publish_markers(self) -> None:
        array = MarkerArray()
        delete = Marker()
        delete.header.frame_id = self.map_frame
        delete.header.stamp = self.get_clock().now().to_msg()
        delete.action = Marker.DELETEALL
        array.markers.append(delete)

        cfg = self.detector.config
        marker_id = 0
        for side, segments in self.detector.last_segments.items():
            color = _rgba(0.2, 0.5, 1.0) if side == SIDE_LEFT else _rgba(1.0, 0.6, 0.1)
            for seg in segments:
                endpoints = self.detector.segment_map_points(side, seg)
                if endpoints is None:
                    continue
                p0, p1 = endpoints
                array.markers.append(self._line_marker(
                    marker_id, p0, p1, color, 'walls', width=0.06))
                marker_id += 1

        # Draw the infinitely extended model over the currently observed
        # span, plus both offset-band limits. All three are generated directly
        # in parking_map and therefore never follow the vehicle's yaw.
        for reference_id, (side, wall) in enumerate(
                self.detector.reference_walls.items()):
            segments = self.detector.last_segments.get(side, [])
            if segments:
                start_s = min(seg.start_s for seg in segments) - 1.0
                end_s = max(seg.end_s for seg in segments) + 1.0
            else:
                start_s, end_s = -1.0, 1.0
            center_line = wall.to_map(np.array([
                [start_s, 0.0], [end_s, 0.0]]))
            array.markers.append(self._line_marker(
                reference_id, center_line[0], center_line[1],
                _rgba(0.1, 1.0, 0.8, 0.9), 'wall_reference', width=0.025))
            for offset_id, offset in enumerate((
                    -cfg.wall_line_offset_m, cfg.wall_line_offset_m)):
                boundary = wall.to_map(np.array([
                    [start_s, offset], [end_s, offset]]))
                array.markers.append(self._line_marker(
                    2 * reference_id + offset_id,
                    boundary[0], boundary[1],
                    _rgba(0.1, 1.0, 0.8, 0.35),
                    'wall_offset', width=0.015))

        for i, cand in enumerate(self.detector.tracked):
            if cand.clear is True:
                color = _rgba(0.1, 0.9, 0.2)
            elif cand.clear is False:
                color = _rgba(0.9, 0.1, 0.1)
            else:
                color = _rgba(0.9, 0.9, 0.1)
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'candidates'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = cand.map_x
            marker.pose.position.y = cand.map_y
            marker.scale.x = marker.scale.y = marker.scale.z = 0.18
            marker.color = color
            array.markers.append(marker)

            if cand.tested:
                corners_map = candidate_square_corners(cand, cfg)
                sq_marker = Marker()
                sq_marker.header.frame_id = self.map_frame
                sq_marker.header.stamp = self.get_clock().now().to_msg()
                sq_marker.ns = 'squares'
                sq_marker.id = i
                sq_marker.type = Marker.LINE_STRIP
                sq_marker.action = Marker.ADD
                sq_marker.scale.x = 0.05
                sq_marker.color = color
                sq_marker.points = [
                    Point(x=float(x), y=float(y), z=0.0) for x, y in corners_map]
                array.markers.append(sq_marker)

        if self.reference_path is not None:
            path = self.reference_path
            segs = [
                ('path_straight1', path.straight1_map, _rgba(0.2, 0.4, 1.0)),
                ('path_arc', path.arc_map, _rgba(0.1, 0.9, 0.2)),
                ('path_straight2', path.straight2_map, _rgba(1.0, 0.1, 0.1)),
            ]
            for j, (ns, pts, color) in enumerate(segs):
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = ns
                marker.id = 0
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.06
                marker.color = color
                marker.points = [
                    Point(x=float(x), y=float(y), z=0.02) for x, y in pts]
                array.markers.append(marker)
            for j, (label, pt, color) in enumerate((
                ('P0', path.p0_map, _rgba(0.9, 0.2, 0.9)),
                ('E', path.e_map, _rgba(0.0, 0.0, 0.0)),
                ('goal', path.goal_map, _rgba(1.0, 0.1, 0.1)),
            )):
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'path_points'
                marker.id = j
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = float(pt[0])
                marker.pose.position.y = float(pt[1])
                marker.pose.position.z = 0.02
                marker.scale.x = marker.scale.y = marker.scale.z = 0.15
                marker.color = color
                array.markers.append(marker)

        self.marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = WallGapNode()
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
