#!/usr/bin/env python3
"""Live left-wall parallel-parking detector, controller, and visualizer."""

from __future__ import annotations

from dataclasses import replace
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from .parallel_parking import (
    ParallelControlState,
    ParallelParkingConfig,
    ParallelParkingController,
    ParallelReferencePath,
    build_parallel_reference_path,
    candidate_rectangle_corners,
    rectangle_is_clear,
)
from .wall_gap_controller import PoseDeltaTracker, WallGapControlOutput
from .wall_gap_detector import SIDE_LEFT
from .wall_gap_node import WallGapNode, _rgba


class ParallelParkingNode(WallGapNode):
    """Reuse the proven fixed-wall SLAM inputs with parallel-path geometry."""

    def __init__(self) -> None:
        # Timers created by the base class bind these overridden callbacks.
        super().__init__('parallel_parking_node')

        self.declare_parameter('rectangle_wall_length_m', 1.5)
        self.declare_parameter('rectangle_inward_depth_m', 0.7)
        self.declare_parameter('parallel_turn_radius_m', 1.5)
        self.declare_parameter('parallel_end_straight_m', 1.5)
        self.declare_parameter('parallel_arc_angle_deg', 30.0)
        self.declare_parameter('parallel_arc_start_offset_m', 0.75)
        self.declare_parameter('parallel_arc_clockwise_offset_m', 0.5)
        # Entry line-arc-line straight (backing in) vs. the forward nudge
        # that follows it -- independently tunable (user directive,
        # 2026-09-04: entry stays 2m, nudge shortened to 1m).
        self.declare_parameter('parallel_entry_straight_m', 2.0)
        self.declare_parameter('parallel_opposite_straight_m', 1.0)

        self.rectangle_wall_length_m = float(
            self.get_parameter('rectangle_wall_length_m').value)
        self.rectangle_inward_depth_m = float(
            self.get_parameter('rectangle_inward_depth_m').value)
        self.parallel_turn_radius_m = float(
            self.get_parameter('parallel_turn_radius_m').value)
        self.parallel_end_straight_m = float(
            self.get_parameter('parallel_end_straight_m').value)
        self.parallel_arc_angle_deg = float(
            self.get_parameter('parallel_arc_angle_deg').value)
        self.parallel_arc_start_offset_m = float(
            self.get_parameter('parallel_arc_start_offset_m').value)
        self.parallel_arc_clockwise_offset_m = float(
            self.get_parameter('parallel_arc_clockwise_offset_m').value)
        self.parallel_entry_straight_m = float(
            self.get_parameter('parallel_entry_straight_m').value)
        self.parallel_opposite_straight_m = float(
            self.get_parameter('parallel_opposite_straight_m').value)

        # A 1.5m wall opening is required.  The common detector's 0.7m square
        # precheck is a necessary subset of our 1.5m x 0.7m rectangle; newly
        # clear candidates receive the full rectangle check in _tick().
        self.detector = type(self.detector)(replace(
            self.detector.config,
            min_gap_m=self.rectangle_wall_length_m,
            square_size_m=self.rectangle_inward_depth_m,
            search_sides=(SIDE_LEFT,),
        ))
        self.controller = ParallelParkingController(ParallelParkingConfig(
            direction_change_hold_s=float(
                self.get_parameter('direction_change_hold_s').value),
            preview_distance_m=float(
                self.get_parameter('preview_distance_m').value),
            forward_speed_mps=float(
                self.get_parameter('forward_speed_mps').value),
            reverse_speed_mps=float(
                self.get_parameter('reverse_speed_mps').value),
            sample_step_m=float(
                self.get_parameter('path_sample_step_m').value),
            entry_straight_m=self.parallel_entry_straight_m,
            opposite_straight_m=self.parallel_opposite_straight_m,
        ))
        self.delta_tracker = PoseDeltaTracker()
        self.reference_path: ParallelReferencePath | None = None
        self.confirmed_candidate = None
        self.seeded = False
        self.waiting_for_valid_point_pass = False
        self.path_failed = False
        self.last_control_output: WallGapControlOutput | None = None
        self.last_control_state = ParallelControlState.IDLE

        # Keep the CAN TargetRef topic shared, but isolate diagnostics and
        # markers so T-parking and parallel-parking logs cannot be confused.
        self.destroy_publisher(self.marker_pub)
        self.destroy_publisher(self.stop_pub)
        self.destroy_publisher(self.state_pub)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/parallel_parking/markers', 1)
        self.stop_pub = self.create_publisher(
            Bool, '/parallel_parking/stop', 1)
        self.state_pub = self.create_publisher(
            String, '/parallel_parking/state', 10)

        self.get_logger().info(
            'parallel parking ready: left wall, rectangle=%.2fm x %.2fm, '
            'symmetric R=%.2fm, arc origin=P0+%.2fm forward+%.2fm CW, '
            'arcs=%.1fdeg, '
            'end lines=%.2fm, entry straight=%.2fm, nudge straight=%.2fm, '
            'previous map accepted'
            % (
                self.rectangle_wall_length_m,
                self.rectangle_inward_depth_m,
                self.parallel_turn_radius_m,
                self.parallel_arc_start_offset_m,
                self.parallel_arc_clockwise_offset_m,
                self.parallel_arc_angle_deg,
                self.parallel_end_straight_m,
                self.parallel_entry_straight_m,
                self.parallel_opposite_straight_m,
            ))

    def _command_tick(self) -> None:
        if self.latest_pose is None:
            self._publish_zero_target()
            self.stop_pub.publish(Bool(data=self.enable_control))
            self.state_pub.publish(String(data=(
                'lidar_pose_missing_hold'
                if self.enable_control else ParallelControlState.IDLE.value)))
            return

        if not self.controller.active:
            # Continue straight while finding the rectangle and until the
            # vehicle has actually passed its valid midpoint P0.
            if self.enable_control and not self.path_failed:
                if not self.searching:
                    self.searching = True
                    self.get_logger().info(
                        'autonomous parallel test: straight search at %.2fm/s'
                        % self.search_speed_mps)
                state = (
                    'parallel_waiting_to_pass_valid_point'
                    if self.confirmed_candidate is not None
                    else 'parallel_searching_for_gap')
                blocked, reason = self._publish_search_target()
                self.stop_pub.publish(Bool(data=blocked))
                self.state_pub.publish(String(data=reason or state))
                return
            self.searching = False
            self.stop_pub.publish(Bool(data=True if self.path_failed else False))
            self.state_pub.publish(String(data=(
                'parallel_path_failed'
                if self.path_failed else ParallelControlState.IDLE.value)))
            return

        self.searching = False
        if not self.enable_control:
            self.stop_pub.publish(Bool(data=True))
            self.state_pub.publish(String(data='parallel_confirmed_control_disabled'))
            return

        now_s = self._clock_s()
        output = self.controller.update(self.latest_pose, now_s)
        pose_stale = (
            now_s - self.latest_pose_s
            > float(self.get_parameter('pose_stale_timeout_s').value))
        if pose_stale:
            output = WallGapControlOutput(
                output.state,
                output.reference_map,
                output.reference_local,
                0.0,
                'parallel_lidar_pose_stale_hold',
            )
        blocked, reason = self._safety_block()
        if blocked:
            output = WallGapControlOutput(
                output.state,
                output.reference_map,
                output.reference_local,
                0.0,
                reason,
            )
        self.last_control_output = output
        self.stop_pub.publish(Bool(data=abs(output.v_ref_mps) < 1.0e-6))
        self.state_pub.publish(String(data=output.status))
        self._publish_target(output)
        if output.state != self.last_control_state:
            self.get_logger().info(
                'parallel control: %s -> %s (%s, v_ref=%.2f)'
                % (
                    self.last_control_state.value,
                    output.state.value,
                    output.status,
                    output.v_ref_mps,
                ))
            self.last_control_state = output.state

    def _candidate_passed(self, pose) -> bool:
        candidate = self.confirmed_candidate
        if candidate is None:
            return False
        wall = self.detector.reference_walls.get(candidate.side)
        if wall is None:
            return False
        candidate_s = wall.project(np.asarray([
            [candidate.map_x, candidate.map_y]], dtype=np.float64))[0, 0]
        vehicle_s = wall.project(np.asarray([
            [pose.x, pose.y]], dtype=np.float64))[0, 0]
        return bool(vehicle_s >= candidate_s)

    def _tick(self) -> None:
        if self.latest_pose is None or len(self.latest_map) == 0:
            return
        pose = self.latest_pose

        if not self.seeded:
            # Unlike the T test, keep all map data accumulated before auto
            # mode.  Only the current pose is needed to orient the first fixed
            # left-wall reference in the vehicle's travel direction.
            self.seeded = True
            self.detector.set_seed(pose, side=SIDE_LEFT)
            self.get_logger().info(
                'parallel left-wall acquisition seeded without map reset at '
                '(%.2f,%.2f)' % (pose.x, pose.y))

        tested_before = {id(candidate) for candidate in self.detector.tracked
                         if candidate.tested}
        reference_sides_before = set(self.detector.reference_walls)
        self.detector.update(self.latest_map, pose)
        for side in set(self.detector.reference_walls) - reference_sides_before:
            wall = self.detector.reference_walls[side]
            self.get_logger().info(
                'parallel reference wall locked: side=%s yaw=%.1fdeg '
                'distance=%.2fm' % (
                    side, math.degrees(wall.yaw), wall.distance_from_seed_m))

        if self.confirmed_candidate is None:
            for candidate in self.detector.tracked:
                if (id(candidate) in tested_before or not candidate.tested
                        or candidate.clear is not True):
                    continue
                wall = self.detector.reference_walls.get(candidate.side)
                if wall is None:
                    continue
                candidate_s = wall.project(np.asarray([
                    [candidate.map_x, candidate.map_y]], dtype=np.float64))[0, 0]
                candidate.clear = rectangle_is_clear(
                    wall.project(self.latest_map),
                    float(candidate_s),
                    self.rectangle_wall_length_m,
                    self.rectangle_inward_depth_m,
                )
                if candidate.clear:
                    self.confirmed_candidate = candidate
                    self.waiting_for_valid_point_pass = True
                    self.get_logger().info(
                        'parallel rectangle confirmed: P0=(%.2f,%.2f), '
                        'wall=%.2fm, inward=%.2fm; continue straight past P0'
                        % (
                            candidate.map_x,
                            candidate.map_y,
                            self.rectangle_wall_length_m,
                            self.rectangle_inward_depth_m,
                        ))
                    break

        if (self.confirmed_candidate is not None
                and self.reference_path is None
                and not self.path_failed
                and self._candidate_passed(pose)):
            self.reference_path = build_parallel_reference_path(
                self.confirmed_candidate,
                pose,
                turn_radius_m=self.parallel_turn_radius_m,
                end_straight_m=self.parallel_end_straight_m,
                arc_angle_deg=self.parallel_arc_angle_deg,
                arc_start_offset_m=self.parallel_arc_start_offset_m,
                arc_clockwise_offset_m=self.parallel_arc_clockwise_offset_m,
            )
            if self.reference_path is None:
                self.path_failed = True
                self.get_logger().error(
                    'parallel reference-path parameters are degenerate')
            else:
                now_s = self._clock_s()
                if not self.controller.start(self.reference_path, pose, now_s):
                    self.path_failed = True
                    self.get_logger().error(
                        'parallel controller could not sample the reference path')
                else:
                    self.waiting_for_valid_point_pass = False
                    self.delta_tracker.reset(pose)
                    first_output = self.controller.update(pose, now_s)
                    self.last_control_output = first_output
                    blocked, reason = self._publish_target(first_output)
                    self.stop_pub.publish(Bool(data=(
                        blocked or abs(first_output.v_ref_mps) < 1.0e-6)))
                    self.state_pub.publish(String(data=(
                        reason or first_output.status)))
                    self.get_logger().info(
                        'parallel S path created after P0 pass: symmetric '
                        'R=%.2fm, origin=P0+%.2fm forward+%.2fm CW, '
                        'arc=%.1fdeg x2, '
                        'end straight=%.2fm x2, entry straight=%.2fm, '
                        'nudge straight=%.2fm, preview=%.2fm, '
                        'sequence=S-F/front-arc-R/rear-arc-F/S-R/S-F, '
                        'v_forward=%.2f, v_reverse=%.2f'
                        % (
                            self.parallel_turn_radius_m,
                            self.parallel_arc_start_offset_m,
                            self.parallel_arc_clockwise_offset_m,
                            self.parallel_arc_angle_deg,
                            self.parallel_end_straight_m,
                            self.parallel_entry_straight_m,
                            self.parallel_opposite_straight_m,
                            self.controller.config.preview_distance_m,
                            self.controller.config.forward_speed_mps,
                            -self.controller.config.reverse_speed_mps,
                        ))

        self._publish_markers()

    def _publish_markers(self) -> None:
        array = MarkerArray()
        delete = Marker()
        delete.header.frame_id = self.map_frame
        delete.header.stamp = self.get_clock().now().to_msg()
        delete.action = Marker.DELETEALL
        array.markers.append(delete)

        if self.latest_pose is not None:
            from .geometry import transform_points
            box_map = transform_points(self.vehicle_box_local, self.latest_pose)
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'vehicle_box'
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.04
            marker.color = _rgba(1.0, 1.0, 1.0)
            marker.points = [
                Point(x=float(x), y=float(y), z=0.02) for x, y in box_map]
            array.markers.append(marker)

        cfg = self.detector.config
        marker_id = 0
        for side, segments in self.detector.last_segments.items():
            for segment in segments:
                endpoints = self.detector.segment_map_points(side, segment)
                if endpoints is None:
                    continue
                array.markers.append(self._line_marker(
                    marker_id, endpoints[0], endpoints[1],
                    _rgba(0.2, 0.5, 1.0), 'parallel_walls', width=0.06))
                marker_id += 1

        for reference_id, (side, wall) in enumerate(
                self.detector.reference_walls.items()):
            segments = self.detector.last_segments.get(side, [])
            if segments:
                start_s = min(segment.start_s for segment in segments) - 1.0
                end_s = max(segment.end_s for segment in segments) + 1.0
            else:
                start_s, end_s = -1.0, 1.0
            center_line = wall.to_map(np.asarray([
                [start_s, 0.0], [end_s, 0.0]]))
            array.markers.append(self._line_marker(
                reference_id, center_line[0], center_line[1],
                _rgba(0.1, 1.0, 0.8, 0.9),
                'parallel_wall_reference', width=0.025))
            for offset_id, offset in enumerate((
                    -cfg.wall_line_offset_m, cfg.wall_line_offset_m)):
                boundary = wall.to_map(np.asarray([
                    [start_s, offset], [end_s, offset]]))
                array.markers.append(self._line_marker(
                    2 * reference_id + offset_id,
                    boundary[0], boundary[1],
                    _rgba(0.1, 1.0, 0.8, 0.35),
                    'parallel_wall_offset', width=0.015))

        for candidate_id, candidate in enumerate(self.detector.tracked):
            color = (
                _rgba(0.1, 0.9, 0.2) if candidate.clear is True
                else _rgba(0.9, 0.1, 0.1) if candidate.clear is False
                else _rgba(0.9, 0.9, 0.1))
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'parallel_candidates'
            marker.id = candidate_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = candidate.map_x
            marker.pose.position.y = candidate.map_y
            marker.scale.x = marker.scale.y = marker.scale.z = 0.18
            marker.color = color
            array.markers.append(marker)
            if candidate.tested:
                corners = candidate_rectangle_corners(
                    candidate,
                    self.rectangle_wall_length_m,
                    self.rectangle_inward_depth_m,
                )
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'parallel_rectangles'
                marker.id = candidate_id
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.05
                marker.color = color
                marker.points = [
                    Point(x=float(x), y=float(y), z=0.0) for x, y in corners]
                array.markers.append(marker)

        if self.reference_path is not None:
            path = self.reference_path
            segments = (
                ('parallel_path_rear_line', path.rear_line_map,
                 _rgba(0.2, 0.4, 1.0)),
                ('parallel_path_rear_arc', path.rear_arc_map,
                 _rgba(0.1, 0.9, 0.2)),
                ('parallel_path_front_arc', path.front_arc_map,
                 _rgba(1.0, 0.55, 0.0)),
                ('parallel_path_front_line', path.front_line_map,
                 _rgba(0.2, 0.4, 1.0)),
            )
            for ns, points, color in segments:
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
                    Point(x=float(x), y=float(y), z=0.03)
                    for x, y in points]
                array.markers.append(marker)

            extra_paths = (
                ('parallel_single_front_arc_path',
                 self.controller.single_arc_forward_path,
                 _rgba(0.1, 1.0, 0.3, 0.85)),
                ('parallel_single_rear_arc_path',
                 self.controller.opposite_arc_forward_path,
                 _rgba(1.0, 0.55, 0.0, 0.85)),
                ('parallel_reused_reference_path',
                 self.controller.reference_forward_path,
                 _rgba(0.6, 0.2, 1.0, 0.65)),
            )
            for ns, points, color in extra_paths:
                if not points:
                    continue
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = ns
                marker.id = 0
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.035
                marker.color = color
                marker.points = [
                    Point(x=float(point.x), y=float(point.y), z=0.06)
                    for point in points]
                array.markers.append(marker)

            for point_id, point in enumerate((
                path.rear_end_map,
                path.rear_tangent_map,
                path.p0_map,
                path.arc_origin_map,
                path.front_tangent_map,
                path.front_end_map,
            )):
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'parallel_path_points'
                marker.id = point_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = float(point[0])
                marker.pose.position.y = float(point[1])
                marker.pose.position.z = 0.04
                marker.scale.x = marker.scale.y = marker.scale.z = 0.14
                marker.color = (
                    _rgba(0.9, 0.2, 0.9) if point_id == 2
                    else (_rgba(0.1, 1.0, 0.3) if point_id == 3
                          else _rgba(1.0, 1.0, 1.0)))
                array.markers.append(marker)

        if (self.last_control_output is not None
                and self.last_control_output.reference_map is not None):
            reference = self.last_control_output.reference_map
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'parallel_preview_point'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(reference.x)
            marker.pose.position.y = float(reference.y)
            marker.pose.position.z = 0.10
            marker.scale.x = marker.scale.y = marker.scale.z = 0.22
            marker.color = _rgba(0.9, 0.1, 1.0)
            array.markers.append(marker)

            label = Marker()
            label.header = marker.header
            label.ns = 'parallel_control_state'
            label.id = 0
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(reference.x)
            label.pose.position.y = float(reference.y)
            label.pose.position.z = 0.45
            label.scale.z = 0.18
            label.color = _rgba(1.0, 1.0, 1.0)
            label.text = '%s  v=%.2f' % (
                self.last_control_output.status,
                self.last_control_output.v_ref_mps,
            )
            array.markers.append(label)

        self.marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = ParallelParkingNode()
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
