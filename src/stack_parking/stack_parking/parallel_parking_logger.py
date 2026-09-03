#!/usr/bin/env python3
"""Logger for the complete five-motion parallel-parking test.

ticks.csv adds (user directive, 2026-09-04): ``stop_count`` (increments
on every v_ref rising-edge-to-zero -- the moment the vehicle actually
stops, not a named *_HOLD status string) and the live vehicle pose
(``pose_x``/``pose_y``/``pose_yaw`` from /parking/slam_pose). Each stop
also dumps the surrounding /parking/local_map to its own
``map_snapshot_stop<N>.csv`` in the same run directory.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from fma_interfaces.msg import TargetRef, VehicleVector


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_LOG_ROOT = REPO_ROOT / 'log' / 'parallel_parking_test'
_CLEAR_GREEN = (0.1, 0.9, 0.2)
_COLOR_TOL = 0.02


class ParallelParkingLogger(Node):
    def __init__(self) -> None:
        super().__init__('parallel_parking_logger')
        self.declare_parameter('out_dir', '')
        self.declare_parameter('log_period_ms', 10.0)

        configured = str(self.get_parameter('out_dir').value).strip()
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.out_dir = (
            Path(configured).expanduser()
            if configured else DEFAULT_LOG_ROOT / stamp)
        self.out_dir.mkdir(parents=True, exist_ok=True)

        self.valid_point_written = False
        self.reference_path_written = False
        self.completion_received = False
        self.logging_finished = False
        self.latest_ref: TargetRef | None = None
        self.latest_vehicle: VehicleVector | None = None
        self.latest_state = 'idle'
        self.latest_pose: PoseStamped | None = None
        self.latest_map: PointCloud2 | None = None
        # Stop counter: incremented on every v_ref rising-edge-to-zero (the
        # moment the vehicle actually stops, not just a named *_HOLD state
        # string) -- user directive, 2026-09-04. Each edge also snapshots the
        # surrounding map to its own file.
        self.stop_count = 0
        self.was_moving = False

        self.ticks_path = self.out_dir / 'ticks.csv'
        self.ticks_file = open(self.ticks_path, 'w', newline='')
        self.ticks_writer = csv.writer(self.ticks_file)
        self.ticks_writer.writerow([
            'stamp_s', 'state', 'stop_count',
            'pose_x', 'pose_y', 'pose_yaw',
            'ref_x', 'ref_y', 'ref_yaw', 'ref_curvature',
            'target_str', 'act_str', 'v_ref', 'act_v',
            'dx', 'dy', 'dyaw',
        ])
        self.ticks_file.flush()
        self.tick_count = 0

        self.create_subscription(
            MarkerArray, '/parallel_parking/markers', self._on_markers, 5)
        self.create_subscription(
            TargetRef, '/adas/target_ref', self._on_target_ref, 5)
        self.create_subscription(
            VehicleVector, '/vehicle/vector', self._on_vehicle_vector,
            qos_profile_sensor_data)
        self.create_subscription(
            String, '/parallel_parking/state', self._on_state, 10)
        self.create_subscription(
            PoseStamped, '/parking/slam_pose', self._on_pose,
            qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/parking/local_map', self._on_map,
            qos_profile_sensor_data)
        period_s = max(
            1.0, float(self.get_parameter('log_period_ms').value)) / 1000.0
        self.log_timer = self.create_timer(period_s, self._on_log_tick)
        self.get_logger().info(
            'parallel parking logger writing to %s at %.0fms'
            % (self.out_dir, period_s * 1000.0))

    def _on_markers(self, msg: MarkerArray) -> None:
        if not self.valid_point_written:
            for marker in msg.markers:
                if marker.ns != 'parallel_candidates':
                    continue
                color = (marker.color.r, marker.color.g, marker.color.b)
                if all(abs(actual - expected) <= _COLOR_TOL
                       for actual, expected in zip(color, _CLEAR_GREEN)):
                    path = self.out_dir / 'valid_point.csv'
                    with open(path, 'w', newline='') as file:
                        writer = csv.writer(file)
                        writer.writerow(['map_x', 'map_y'])
                        writer.writerow([
                            marker.pose.position.x,
                            marker.pose.position.y,
                        ])
                    self.valid_point_written = True
                    self.get_logger().info('parallel valid point -> %s' % path)
                    break

        if not self.reference_path_written:
            segment_names = (
                'parallel_path_rear_line',
                'parallel_path_rear_arc',
                'parallel_path_front_arc',
                'parallel_path_front_line',
                'parallel_single_front_arc_path',
                'parallel_single_rear_arc_path',
                'parallel_reused_reference_path',
            )
            rows = []
            for marker in msg.markers:
                if marker.ns in segment_names:
                    for index, point in enumerate(marker.points):
                        rows.append((marker.ns, index, point.x, point.y))
                elif marker.ns == 'parallel_path_points':
                    label = {
                        0: 'rear_end',
                        1: 'rear_tangent',
                        2: 'P0',
                        3: 'arc_origin',
                        4: 'front_tangent',
                        5: 'front_end',
                    }.get(marker.id, str(marker.id))
                    rows.append((
                        'parallel_path_points:' + label,
                        0,
                        marker.pose.position.x,
                        marker.pose.position.y,
                    ))
            if rows:
                path = self.out_dir / 'reference_path.csv'
                with open(path, 'w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(['segment', 'index', 'map_x', 'map_y'])
                    writer.writerows(rows)
                self.reference_path_written = True
                self.get_logger().info(
                    'parallel reference path: %d points -> %s'
                    % (len(rows), path))

    def _on_target_ref(self, msg: TargetRef) -> None:
        self.latest_ref = msg

    def _on_vehicle_vector(self, msg: VehicleVector) -> None:
        self.latest_vehicle = msg

    def _on_pose(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def _on_map(self, msg: PointCloud2) -> None:
        self.latest_map = msg

    def _save_map_snapshot(self) -> None:
        """Dump the current /parking/local_map to its own CSV at a stop."""
        if self.latest_map is None:
            return
        points = point_cloud2.read_points_numpy(
            self.latest_map, field_names=('x', 'y'), skip_nans=True)
        path = self.out_dir / ('map_snapshot_stop%d.csv' % self.stop_count)
        with open(path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['map_x', 'map_y'])
            if points.dtype.names:
                writer.writerows(zip(points['x'].tolist(), points['y'].tolist()))
            else:
                writer.writerows(points.reshape((-1, 2)).tolist())
        self.get_logger().info(
            'stop #%d map snapshot: %d points -> %s'
            % (self.stop_count, len(points), path))

    def _on_state(self, msg: String) -> None:
        self.latest_state = msg.data
        if msg.data == 'parallel_parking_complete':
            # The controller publishes this only after the final one-second
            # stop has elapsed.  Finish on the next log tick so the last
            # zero-speed TargetRef can be recorded first.
            self.completion_received = True

    def _finish(self) -> None:
        if self.logging_finished:
            return
        self.logging_finished = True
        self.log_timer.cancel()
        if not self.ticks_file.closed:
            self.ticks_file.flush()
            self.ticks_file.close()
        self.get_logger().info(
            'parallel final hold complete — logging flushed and closed')
        if rclpy.ok():
            rclpy.shutdown()

    def _on_log_tick(self) -> None:
        if self.logging_finished:
            return
        vehicle = self.latest_vehicle
        reference_msg = self.latest_ref
        if vehicle is not None:
            point = (
                reference_msg.ref_points[0]
                if reference_msg is not None and reference_msg.ref_points
                else None)
            v_ref = reference_msg.v_ref if reference_msg is not None else None
            is_moving = v_ref is not None and abs(v_ref) > 1.0e-6
            if self.was_moving and not is_moving:
                # Rising edge of "stopped" -- the vehicle just came to a
                # stop, regardless of which named *_HOLD status string this
                # is (user directive, 2026-09-04).
                self.stop_count += 1
                self._save_map_snapshot()
            self.was_moving = is_moving

            pose = self.latest_pose
            if pose is not None:
                q = pose.pose.orientation
                pose_x = pose.pose.position.x
                pose_y = pose.pose.position.y
                pose_yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            else:
                pose_x = pose_y = pose_yaw = ''

            self.ticks_writer.writerow([
                self.get_clock().now().nanoseconds * 1.0e-9,
                self.latest_state,
                self.stop_count,
                pose_x, pose_y, pose_yaw,
                point.x if point else '',
                point.y if point else '',
                point.yaw if point else '',
                point.curvature if point else '',
                vehicle.str_ref,
                vehicle.str,
                v_ref if v_ref is not None else '',
                vehicle.v,
                reference_msg.dx if reference_msg is not None else '',
                reference_msg.dy if reference_msg is not None else '',
                reference_msg.dyaw if reference_msg is not None else '',
            ])
            self.tick_count += 1
            if self.tick_count % 500 == 0:
                self.ticks_file.flush()
                self.get_logger().info('%d parallel ticks logged' % self.tick_count)
        if self.completion_received:
            self._finish()

    def destroy_node(self) -> bool:
        if not self.ticks_file.closed:
            self.ticks_file.flush()
            self.ticks_file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ParallelParkingLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
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
