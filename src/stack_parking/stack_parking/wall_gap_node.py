#!/usr/bin/env python3
"""Live fixed-wall gap detection, reference generation, and parking control.

The detector runs against the LiDAR SLAM map.  Once the first reachable
square is confirmed, the node asserts an immediate stop, creates the fixed
3m-line/arc/2m-line path, holds for one second, and can publish the 100Hz
``/adas/target_ref`` stream used by bridge_dspace.  Motion output is guarded
by the explicit ``enable_control`` parameter; visualization remains usable
with control disabled.

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
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray
from fma_interfaces.msg import RefPoint, TargetRef

from .geometry import Pose2
from .reference_path import ReferencePath, build_reference_path
from .wall_gap_controller import (
    ControlState,
    PoseDeltaTracker,
    WallGapControlConfig,
    WallGapControlOutput,
    WallGapController,
)
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
        self.declare_parameter('base_frame', 'base_link')
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
        self.declare_parameter('enable_control', False)
        self.declare_parameter('target_topic', '/adas/target_ref')
        self.declare_parameter('command_rate_hz', 100.0)
        self.declare_parameter('hold_after_detection_s', 1.0)
        self.declare_parameter('preview_distance_m', 1.0)
        self.declare_parameter('forward_speed_mps', 0.3)
        self.declare_parameter('reverse_speed_mps', 0.3)
        self.declare_parameter('stop_clearance_m', 0.20)
        self.declare_parameter('path_end_tolerance_m', 0.10)
        self.declare_parameter('path_sample_step_m', 0.05)
        self.declare_parameter('require_rear_clearance', True)
        self.declare_parameter('pose_stale_timeout_s', 0.35)
        self.declare_parameter('rear_scan_topic', '/lidar/a2/scan')
        self.declare_parameter('rear_scan_stale_timeout_s', 0.35)
        self.declare_parameter('rear_scan_center_deg', -90.0)
        self.declare_parameter('rear_scan_half_width_deg', 12.0)
        self.declare_parameter('rear_range_offset_m', 0.069)
        self.declare_parameter('rear_wall_min_points', 5)
        self.declare_parameter('rear_wall_cluster_m', 0.04)
        self.declare_parameter(
            'current_cloud_topic', '/parking/nearest_merged_cloud')
        self.declare_parameter('fused_cloud_stale_timeout_s', 0.35)
        self.declare_parameter('rear_lidar_x_m', -0.055)

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
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.min_turn_radius_m = float(self.get_parameter('min_turn_radius_m').value)
        self.inside_straight_m = float(
            self.get_parameter('inside_straight_m').value)
        self.parallel_straight_m = float(
            self.get_parameter('parallel_straight_m').value)
        self.enable_control = bool(self.get_parameter('enable_control').value)
        self.controller = WallGapController(WallGapControlConfig(
            hold_s=float(self.get_parameter('hold_after_detection_s').value),
            preview_distance_m=float(
                self.get_parameter('preview_distance_m').value),
            forward_speed_mps=float(
                self.get_parameter('forward_speed_mps').value),
            reverse_speed_mps=float(
                self.get_parameter('reverse_speed_mps').value),
            stop_clearance_m=float(
                self.get_parameter('stop_clearance_m').value),
            path_end_tolerance_m=float(
                self.get_parameter('path_end_tolerance_m').value),
            sample_step_m=float(
                self.get_parameter('path_sample_step_m').value),
            require_rear_clearance=bool(
                self.get_parameter('require_rear_clearance').value),
        ))
        self.delta_tracker = PoseDeltaTracker()

        self.latest_map = np.empty((0, 2), dtype=np.float64)
        self.latest_pose = None
        self.latest_pose_s = -math.inf
        self.latest_rear_clearance_m = None
        self.latest_rear_scan_s = -math.inf
        self.latest_fused_clearance_m = None
        self.latest_fused_cloud_s = -math.inf
        self.last_control_output: WallGapControlOutput | None = None
        self.last_control_state = ControlState.IDLE
        self.fused_frame_warned = False

        self.confirmed_candidate = None
        self.reference_path: ReferencePath | None = None

        self.map_sub = self.create_subscription(
            PointCloud2, str(self.get_parameter('map_topic').value),
            self._on_map, qos_profile_sensor_data)
        self.pose_sub = self.create_subscription(
            PoseStamped, str(self.get_parameter('pose_topic').value),
            self._on_pose, qos_profile_sensor_data)
        self.rear_scan_sub = self.create_subscription(
            LaserScan, str(self.get_parameter('rear_scan_topic').value),
            self._on_rear_scan, qos_profile_sensor_data)
        self.current_cloud_sub = self.create_subscription(
            PointCloud2, str(self.get_parameter('current_cloud_topic').value),
            self._on_current_cloud, qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/wall_gap/markers', 1)
        self.stop_pub = self.create_publisher(Bool, '/wall_gap/stop', 1)
        self.state_pub = self.create_publisher(String, '/wall_gap/state', 10)
        self.target_pub = self.create_publisher(
            TargetRef, str(self.get_parameter('target_topic').value), 1)
        # Every fresh test run must map from scratch (user directive,
        # 2026-09-02) — stack_parking_node already wipes its SLAM map on a
        # manual_command trigger when reset_map_on_mission_start is true
        # (the node.py default since the same date), so reuse that instead
        # of adding a second reset mechanism.
        self.command_pub = self.create_publisher(String, '/parking/manual_command', 10)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(1.0, rate), self._tick)
        command_rate = float(self.get_parameter('command_rate_hz').value)
        self.command_timer = self.create_timer(
            1.0 / max(1.0, command_rate), self._command_tick)
        self.get_logger().info(
            'wall_gap_node ready: initial_band=%.2f..%.2f wall_offset=+/-%.2f '
            'min_gap=%.2f square=%.2f control=%s'
            % (cfg.near_m, cfg.far_m, cfg.wall_line_offset_m,
               cfg.min_gap_m, cfg.square_size_m, self.enable_control))
        if not self.enable_control:
            self.get_logger().warn(
                'control disabled: square confirmation will latch '
                '/wall_gap/stop, but no /adas/target_ref will be published')

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
        pose = Pose2(msg.pose.position.x, msg.pose.position.y, yaw)
        self.latest_pose = pose
        self.latest_pose_s = self._clock_s()
        if self.controller.active:
            self.delta_tracker.update(pose)

    @staticmethod
    def _clustered_clearance(
        values: list[float],
        minimum_points: int,
        cluster_m: float,
    ) -> float | None:
        if len(values) < minimum_points:
            return None
        values_array = np.asarray(values, dtype=np.float64)
        candidate = float(np.percentile(values_array, 30.0))
        support = int(np.count_nonzero(
            np.abs(values_array - candidate) <= cluster_m))
        return candidate if support >= minimum_points else None

    def _on_rear_scan(self, msg: LaserScan) -> None:
        center = math.radians(float(
            self.get_parameter('rear_scan_center_deg').value))
        half = math.radians(float(
            self.get_parameter('rear_scan_half_width_deg').value))
        offset = float(self.get_parameter('rear_range_offset_m').value)
        values: list[float] = []
        for index, raw in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            angle_error = (angle - center + math.pi) % (2.0 * math.pi) - math.pi
            if abs(angle_error) > half:
                continue
            if not math.isfinite(raw) or raw < msg.range_min or raw > msg.range_max:
                continue
            corrected = float(raw) - offset
            if corrected > 0.0:
                values.append(corrected)
        self.latest_rear_clearance_m = self._clustered_clearance(
            values,
            int(self.get_parameter('rear_wall_min_points').value),
            float(self.get_parameter('rear_wall_cluster_m').value),
        )
        self.latest_rear_scan_s = self._clock_s()

    def _on_current_cloud(self, msg: PointCloud2) -> None:
        """Measure the rear wall from any LiDAR represented in fused cloud."""
        if msg.header.frame_id.lstrip('/') != self.base_frame.lstrip('/'):
            self.latest_fused_clearance_m = None
            self.latest_fused_cloud_s = self._clock_s()
            if not self.fused_frame_warned:
                self.fused_frame_warned = True
                self.get_logger().error(
                    'fused clearance cloud frame=%r rejected; expected %r'
                    % (msg.header.frame_id, self.base_frame))
            return
        pts = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y'), skip_nans=True)
        xy = (
            np.column_stack((pts['x'], pts['y'])) if pts.dtype.names
            else np.asarray(pts).reshape((-1, 2)))
        rear_x = float(self.get_parameter('rear_lidar_x_m').value)
        half = math.radians(float(
            self.get_parameter('rear_scan_half_width_deg').value))
        if len(xy):
            dx = xy[:, 0] - rear_x
            behind = dx < 0.0
            in_sector = np.abs(xy[:, 1]) <= (-dx) * math.tan(half)
            selected = xy[behind & in_sector]
            values = np.hypot(
                selected[:, 0] - rear_x, selected[:, 1]).tolist()
        else:
            values = []
        self.latest_fused_clearance_m = self._clustered_clearance(
            values,
            int(self.get_parameter('rear_wall_min_points').value),
            float(self.get_parameter('rear_wall_cluster_m').value),
        )
        self.latest_fused_cloud_s = self._clock_s()

    def _rear_clearance(self, now_s: float) -> float | None:
        candidates = []
        if (
            now_s - self.latest_rear_scan_s
            <= float(self.get_parameter('rear_scan_stale_timeout_s').value)
            and self.latest_rear_clearance_m is not None
        ):
            candidates.append(float(self.latest_rear_clearance_m))
        if (
            now_s - self.latest_fused_cloud_s
            <= float(self.get_parameter('fused_cloud_stale_timeout_s').value)
            and self.latest_fused_clearance_m is not None
        ):
            candidates.append(float(self.latest_fused_clearance_m))
        return min(candidates) if candidates else None

    def _publish_target(self, output: WallGapControlOutput) -> None:
        if not self.enable_control or output.reference_local is None:
            return
        reference = output.reference_local
        delta = self.delta_tracker.delta
        msg = TargetRef()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.ref_points = [RefPoint(
            x=float(reference.x),
            y=float(reference.y),
            yaw=float(reference.yaw),
            curvature=float(reference.curvature),
        )]
        msg.v_ref = float(output.v_ref_mps)
        msg.state = TargetRef.STATE_PARKING
        msg.dx = float(delta.dx)
        msg.dy = float(delta.dy)
        msg.dyaw = float(delta.dyaw)
        msg.update = int(delta.update)
        self.target_pub.publish(msg)

    def _command_tick(self) -> None:
        if not self.controller.active or self.latest_pose is None:
            self.stop_pub.publish(Bool(data=False))
            self.state_pub.publish(String(data=ControlState.IDLE.value))
            return
        # With control disabled, latch the confirmation stop for the manual
        # bench path.  In particular, never release T_Parking after one second
        # unless this node is also taking ownership of /adas/target_ref.
        if not self.enable_control:
            self.stop_pub.publish(Bool(data=True))
            self.state_pub.publish(String(data='confirmed_control_disabled'))
            return

        now_s = self._clock_s()
        output = self.controller.update(
            self.latest_pose,
            now_s,
            rear_clearance_m=self._rear_clearance(now_s),
        )
        pose_stale = (
            now_s - self.latest_pose_s
            > float(self.get_parameter('pose_stale_timeout_s').value))
        if pose_stale:
            output = WallGapControlOutput(
                output.state,
                output.reference_map,
                output.reference_local,
                0.0,
                'lidar_pose_stale_hold',
            )
        self.last_control_output = output
        self.stop_pub.publish(Bool(data=abs(output.v_ref_mps) < 1.0e-6))
        self.state_pub.publish(String(data=output.status))
        self._publish_target(output)
        if output.state != self.last_control_state:
            self.get_logger().info(
                'parking control: %s -> %s (%s, v_ref=%.2f)'
                % (self.last_control_state.value, output.state.value,
                   output.status, output.v_ref_mps))
            self.last_control_state = output.state

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
                now_s = self._clock_s()
                if not self.controller.start(self.reference_path, pose, now_s):
                    self.get_logger().error(
                        'reference path exists, but controller sampling failed')
                else:
                    self.delta_tracker.reset(pose)
                    # Send the first zero-speed frame in this same detection
                    # callback; do not wait up to one more 10ms command tick.
                    first_output = self.controller.update(
                        pose, now_s, rear_clearance_m=self._rear_clearance(now_s))
                    self.last_control_output = first_output
                    self.stop_pub.publish(Bool(data=True))
                    self.state_pub.publish(String(data=first_output.status))
                    self._publish_target(first_output)
                    self.get_logger().info(
                        'usable space confirmed: side=%s map=(%.2f,%.2f) '
                        'width=%.2fm — immediate stop, hold=%.1fs, '
                        'preview=%.1fm, v_forward=%.2f, v_reverse=%.2f, '
                        'rear stop<=%.2fm'
                        % (
                            cleared.side, cleared.map_x, cleared.map_y,
                            cleared.width_m, self.controller.config.hold_s,
                            self.controller.config.preview_distance_m,
                            self.controller.config.forward_speed_mps,
                            -self.controller.config.reverse_speed_mps,
                            self.controller.config.stop_clearance_m,
                        ))

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
                ('S', path.straight1_map[0], _rgba(0.2, 0.4, 1.0)),
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

        if (
            self.last_control_output is not None
            and self.last_control_output.reference_map is not None
        ):
            reference = self.last_control_output.reference_map
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'preview_point'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(reference.x)
            marker.pose.position.y = float(reference.y)
            marker.pose.position.z = 0.10
            marker.scale.x = marker.scale.y = marker.scale.z = 0.22
            marker.color = _rgba(0.9, 0.1, 1.0)
            array.markers.append(marker)

            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'control_state'
            text_marker.id = 0
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(reference.x)
            text_marker.pose.position.y = float(reference.y)
            text_marker.pose.position.z = 0.45
            text_marker.scale.z = 0.18
            text_marker.color = _rgba(1.0, 1.0, 1.0)
            text_marker.text = '%s  v=%.2f' % (
                self.last_control_output.status,
                self.last_control_output.v_ref_mps,
            )
            array.markers.append(text_marker)

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
