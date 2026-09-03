#!/usr/bin/env python3
"""Wall-gap reverse-parking test logger — records the fixed valid point,
the built reference path, and every commanded-vs-actual sample.

Run alongside wall_gap_test.launch.py (enable_control:=true). Reads only,
no ownership of any topic — safe to start/stop independently of the launch.

Written once per run (as soon as they appear on /wall_gap/markers):
  valid_point.csv     — side, map_x, map_y, width_m, near_distance_m
  reference_path.csv  — segment, index, map_x, map_y  (path_straight1,
                         path_arc, path_straight2, path_points rows)

Written continuously, one row every ``log_period_ms`` (default 10ms) on a
fixed ROS timer — not off either topic's own callback, so row spacing stays
uniform regardless of /adas/target_ref (100Hz) or /vehicle/vector arrival
jitter/drops. Each row holds the latest cached sample of both (same "hold
between updates" convention the CAN protocol itself uses, CLAUDE.md §5.8):
  ticks.csv — stamp_s, ref_x, ref_y, ref_yaw, ref_curvature, target_str,
              act_str, v_ref, act_v, dx, dy, dyaw

Files land in <repo_root>/log/wall_gap_reverse_test/<timestamp>/ (not
/tmp — the scratchpad is wiped on session restart, same reasoning as
bridge_dspace/tools/camera_traffic_ref_test.py).

Usage:
    python3 src/stack_parking/tools/wall_gap_logger.py
    python3 src/stack_parking/tools/wall_gap_logger.py --ros-args -p out_dir:=/custom/path
    python3 src/stack_parking/tools/wall_gap_logger.py --ros-args -p log_period_ms:=20
"""

from __future__ import annotations

import csv
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import MarkerArray

from fma_interfaces.msg import TargetRef, VehicleVector

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_LOG_ROOT = REPO_ROOT / "log" / "wall_gap_reverse_test"

# Matches wall_gap_node.py's candidate marker color for clear==True
# (0.1, 0.9, 0.2) — see _publish_markers's candidates loop.
_CLEAR_GREEN = (0.1, 0.9, 0.2)
_COLOR_TOL = 0.02


class WallGapLogger(Node):

    def __init__(self) -> None:
        super().__init__('wall_gap_logger')
        self.declare_parameter('out_dir', '')
        self.declare_parameter('log_period_ms', 10.0)

        out_dir_param = str(self.get_parameter('out_dir').value).strip()
        if out_dir_param:
            self.out_dir = Path(out_dir_param).expanduser()
        else:
            stamp = time.strftime('%Y%m%d_%H%M%S')
            self.out_dir = DEFAULT_LOG_ROOT / stamp
        self.out_dir.mkdir(parents=True, exist_ok=True)

        self.valid_point_written = False
        self.reference_path_written = False

        self.ticks_path = self.out_dir / 'ticks.csv'
        self.ticks_file = open(self.ticks_path, 'w', newline='')
        self.ticks_writer = csv.writer(self.ticks_file)
        self.ticks_writer.writerow([
            'stamp_s', 'ref_x', 'ref_y', 'ref_yaw', 'ref_curvature',
            'target_str', 'act_str', 'v_ref', 'act_v', 'dx', 'dy', 'dyaw',
        ])
        self.ticks_file.flush()
        self.tick_count = 0

        self.latest_ref: TargetRef | None = None
        self.latest_veh: VehicleVector | None = None

        self.create_subscription(
            MarkerArray, '/wall_gap/markers', self._on_markers, 5)
        self.create_subscription(
            TargetRef, '/adas/target_ref', self._on_target_ref, 5)
        self.create_subscription(
            VehicleVector, '/vehicle/vector', self._on_vehicle_vector,
            qos_profile_sensor_data)

        # Fixed-period timer rather than writing off the /vehicle/vector
        # callback: guarantees a uniform 10ms row spacing (user directive,
        # 2026-09-03) independent of that topic's actual arrival jitter or
        # any dropped frames -- each row just holds the latest cached sample
        # of each topic, same "hold between updates" convention the CAN
        # protocol itself uses (CLAUDE.md 5.8).
        period_s = max(1.0, float(self.get_parameter('log_period_ms').value)) / 1000.0
        self.create_timer(period_s, self._on_log_tick)

        self.get_logger().info(
            'wall_gap_logger writing to %s (period=%.0fms)'
            % (self.out_dir, period_s * 1000.0))

    def _on_markers(self, msg: MarkerArray) -> None:
        if not self.valid_point_written:
            for marker in msg.markers:
                if marker.ns != 'candidates':
                    continue
                color = (marker.color.r, marker.color.g, marker.color.b)
                if all(abs(a - b) <= _COLOR_TOL for a, b in zip(color, _CLEAR_GREEN)):
                    path = self.out_dir / 'valid_point.csv'
                    with open(path, 'w', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow(['map_x', 'map_y'])
                        writer.writerow([
                            marker.pose.position.x, marker.pose.position.y])
                    self.valid_point_written = True
                    self.get_logger().info(
                        'valid point logged: (%.3f, %.3f) -> %s'
                        % (marker.pose.position.x, marker.pose.position.y, path))
                    break

        if not self.reference_path_written:
            segment_ns = ('path_straight1', 'path_arc', 'path_straight2')
            rows = []
            for marker in msg.markers:
                if marker.ns in segment_ns:
                    for i, point in enumerate(marker.points):
                        rows.append((marker.ns, i, point.x, point.y))
                elif marker.ns == 'path_points':
                    label = {0: 'P0', 1: 'E', 2: 'S', 3: 'goal'}.get(
                        marker.id, str(marker.id))
                    rows.append((
                        'path_points:' + label, 0,
                        marker.pose.position.x, marker.pose.position.y))
            if rows:
                path = self.out_dir / 'reference_path.csv'
                with open(path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['segment', 'index', 'map_x', 'map_y'])
                    writer.writerows(rows)
                self.reference_path_written = True
                self.get_logger().info(
                    'reference path logged: %d points -> %s' % (len(rows), path))

    def _on_target_ref(self, msg: TargetRef) -> None:
        self.latest_ref = msg

    def _on_vehicle_vector(self, msg: VehicleVector) -> None:
        self.latest_veh = msg

    def _on_log_tick(self) -> None:
        veh = self.latest_veh
        if veh is None:
            return
        ref = self.latest_ref
        point = ref.ref_points[0] if ref is not None and ref.ref_points else None
        msg = veh
        self.ticks_writer.writerow([
            self.get_clock().now().nanoseconds * 1.0e-9,
            point.x if point else '',
            point.y if point else '',
            point.yaw if point else '',
            point.curvature if point else '',
            msg.str_ref,
            msg.str,
            ref.v_ref if ref is not None else '',
            msg.v,
            ref.dx if ref is not None else '',
            ref.dy if ref is not None else '',
            ref.dyaw if ref is not None else '',
        ])
        self.tick_count += 1
        if self.tick_count % 500 == 0:
            self.ticks_file.flush()
            self.get_logger().info('%d ticks logged' % self.tick_count)

    def destroy_node(self) -> bool:
        self.ticks_file.flush()
        self.ticks_file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WallGapLogger()
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
