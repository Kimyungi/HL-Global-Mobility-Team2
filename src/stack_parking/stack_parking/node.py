"""Four-LiDAR ICP parking pipeline ROS 2 wrapper.

Decision ownership remains in ``adas_mgm``. Until a stable, feasible parking
space exists this node publishes ``space_found=False`` and the existing lane or
GPS stack keeps the car moving straight. Once a plan exists, this node emits
one current-vehicle-frame preview point, matching ``ParkingStatus.msg``.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from fma_interfaces.msg import GpsPath, ParkingStatus, RefPoint, VehicleVector
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import Pose2, transform_points
from .icp_slam import IcpConfig, IcpSlam, voxel_downsample
from .mission import MissionConfig, MissionOutput, MissionState, ParkingMission
from .path_planner import MinimumRadiusParkingPlanner, PlannerConfig
from .space_detector import (
    MODE_PARALLEL,
    MODE_PERPENDICULAR,
    SIDE_LEFT,
    SIDE_RIGHT,
    ParkingSpaceDetector,
    SpaceDetectorConfig,
)


def _quaternion_z(yaw: float) -> tuple[float, float]:
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def _angle_distance(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


class StackParkingNode(Node):

    def __init__(self):
        super().__init__('stack_parking_node')
        self._declare_parameters()
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.slam = IcpSlam(self._icp_config())
        detector = ParkingSpaceDetector(self._detector_config())
        planner = MinimumRadiusParkingPlanner(self._planner_config())
        self.mission = ParkingMission(detector, planner, self._mission_config())

        self.latest_vehicle: Optional[VehicleVector] = None
        self.latest_rear_clearance_m: Optional[float] = None
        self.latest_rear_scan_s = -math.inf
        self.last_icp_result = None
        self.last_icp_accepted_s = -math.inf
        self.last_debug_publish_s = -math.inf
        self.latest_scan_map = np.empty((0, 2), dtype=np.float64)
        self.gps_zone_armed = True
        self.manual_gate_active = False
        self.last_output: Optional[MissionOutput] = None

        self.status_pub = self.create_publisher(
            ParkingStatus, '/perception/parking', 1)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/parking/slam_pose', 1)
        self.scan_pub = self.create_publisher(
            PointCloud2, '/parking/slam_scan', 1)
        self.map_pub = self.create_publisher(
            PointCloud2, '/parking/local_map', 1)
        self.path_pub = self.create_publisher(
            Path, '/parking/reference_path', 1)
        self.active_path_pub = self.create_publisher(
            Path, '/parking/active_path', 1)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/parking/debug_markers', 1)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, '/parking/diagnostics', 1)
        self.manual_gate_pub = None
        if bool(self.get_parameter('manual_test_publish_gps_gate').value):
            self.manual_gate_pub = self.create_publisher(
                GpsPath, '/perception/gps_path', 1)

        merged_topic = str(self.get_parameter('merged_cloud_topic').value)
        rear_topic = str(self.get_parameter('rear_scan_topic').value)
        self.cloud_sub = self.create_subscription(
            PointCloud2, merged_topic, self._on_cloud, qos_profile_sensor_data)
        self.rear_sub = self.create_subscription(
            LaserScan, rear_topic, self._on_rear_scan, qos_profile_sensor_data)
        self.vehicle_sub = self.create_subscription(
            VehicleVector, '/vehicle/vector', self._on_vehicle, qos_profile_sensor_data)
        self.gps_sub = self.create_subscription(
            GpsPath, '/perception/gps_path', self._on_gps_path, 1)
        self.gps_command_sub = self.create_subscription(
            String, '/parking/gps_command', self._on_command, 10)
        self.manual_command_sub = self.create_subscription(
            String, '/parking/manual_command', self._on_command, 10)

        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(1.0, publish_rate), self._tick)
        self.get_logger().info(
            'ICP parking ready: merged=%s rear=%s manual=/parking/manual_command '
            '(start perpendicular right | start parallel right | cancel)'
            % (merged_topic, rear_topic))

    def _declare_parameters(self) -> None:
        values = {
            'map_frame': 'parking_map',
            'base_frame': 'base_link',
            'merged_cloud_topic': '/lidar/merged_cloud',
            'rear_scan_topic': '/lidar/a2/scan',
            'publish_rate_hz': 20.0,
            'debug_publish_rate_hz': 5.0,
            'auto_trigger_gps_zone': True,
            'gps_default_mode': MODE_PERPENDICULAR,
            'gps_default_side': SIDE_RIGHT,
            # Test-only: synthesizes the existing MGM GPS gate after a manual
            # command. Never enable while the real stack_gps publisher runs.
            'manual_test_publish_gps_gate': False,
            'slam_stale_timeout_s': 0.6,
            'rear_scan_stale_timeout_s': 0.35,
            'rear_scan_center_deg': -90.0,
            'rear_scan_half_width_deg': 12.0,
            'rear_range_offset_m': 0.069,
            'rear_wall_min_points': 5,
            'rear_wall_cluster_m': 0.04,
            # Vehicle values mirror stack_avoid/config/params.yaml, the project
            # single source documented in stack_parking/MEASUREMENTS.md.
            'vehicle.width_m': 0.62,
            'vehicle.length_m': 0.85,
            'vehicle.front_m': 0.760,
            'vehicle.rear_m': 0.090,
            'vehicle.wheelbase_m': 0.595,
            'vehicle.min_turn_radius_m': 1.15,
            'lidar.rear_x_m': -0.055,
            'icp.scan_voxel_m': 0.06,
            'icp.max_scan_points': 900,
            'icp.map_voxel_m': 0.08,
            'icp.max_correspondence_m': 0.35,
            'icp.max_iterations': 18,
            'icp.min_correspondences': 24,
            'icp.max_rmse_m': 0.16,
            'icp.local_map_radius_m': 8.0,
            'space.boundary_near_m': 0.42,
            'space.boundary_far_m': 1.05,
            'space.parallel_min_length_m': 2.90,
            'space.perpendicular_min_width_m': 0.86,
            'space.perpendicular_min_depth_m': 1.35,
            'space.stable_frames': 3,
            'path.sample_step_m': 0.06,
            'path.static_clearance_m': 0.035,
            'preview_distance_m': 1.0,
            'speed.forward_mps': 0.60,
            'speed.reverse_turn_mps': 0.55,
            'speed.reverse_dock_mps': 0.15,
            'speed.exit_mps': 0.55,
            'dock_slow_distance_m': 0.55,
            'completion_clearance_m': 0.20,
            'completion_trigger_margin_m': 0.02,
            'parked_wait_s': 5.0,
            'require_vehicle_stop_feedback': True,
            'dynamic.confirm_frames': 2,
            'dynamic.static_match_m': 0.14,
            'dynamic.clearance_m': 0.12,
        }
        for name, value in values.items():
            self.declare_parameter(name, value)

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _icp_config(self) -> IcpConfig:
        return IcpConfig(
            scan_voxel_m=float(self._p('icp.scan_voxel_m')),
            max_scan_points=int(self._p('icp.max_scan_points')),
            map_voxel_m=float(self._p('icp.map_voxel_m')),
            max_correspondence_m=float(self._p('icp.max_correspondence_m')),
            max_iterations=int(self._p('icp.max_iterations')),
            min_correspondences=int(self._p('icp.min_correspondences')),
            max_rmse_m=float(self._p('icp.max_rmse_m')),
            local_map_radius_m=float(self._p('icp.local_map_radius_m')),
        )

    def _detector_config(self) -> SpaceDetectorConfig:
        return SpaceDetectorConfig(
            vehicle_width_m=float(self._p('vehicle.width_m')),
            vehicle_length_m=float(self._p('vehicle.length_m')),
            boundary_near_m=float(self._p('space.boundary_near_m')),
            boundary_far_m=float(self._p('space.boundary_far_m')),
            parallel_min_length_m=float(self._p('space.parallel_min_length_m')),
            perpendicular_min_width_m=float(self._p('space.perpendicular_min_width_m')),
            perpendicular_min_depth_m=float(self._p('space.perpendicular_min_depth_m')),
            rear_lidar_x_m=float(self._p('lidar.rear_x_m')),
            completion_clearance_m=float(self._p('completion_clearance_m')),
            completion_trigger_margin_m=float(self._p('completion_trigger_margin_m')),
            stable_frames=int(self._p('space.stable_frames')),
        )

    def _planner_config(self) -> PlannerConfig:
        return PlannerConfig(
            min_turn_radius_m=float(self._p('vehicle.min_turn_radius_m')),
            sample_step_m=float(self._p('path.sample_step_m')),
            vehicle_width_m=float(self._p('vehicle.width_m')),
            vehicle_front_m=float(self._p('vehicle.front_m')),
            vehicle_rear_m=float(self._p('vehicle.rear_m')),
            static_clearance_m=float(self._p('path.static_clearance_m')),
        )

    def _mission_config(self) -> MissionConfig:
        return MissionConfig(
            preview_distance_m=float(self._p('preview_distance_m')),
            forward_speed_mps=float(self._p('speed.forward_mps')),
            reverse_turn_speed_mps=float(self._p('speed.reverse_turn_mps')),
            reverse_dock_speed_mps=float(self._p('speed.reverse_dock_mps')),
            exit_speed_mps=float(self._p('speed.exit_mps')),
            dock_slow_distance_m=float(self._p('dock_slow_distance_m')),
            completion_clearance_m=float(self._p('completion_clearance_m')),
            parked_wait_s=float(self._p('parked_wait_s')),
            require_stationary_feedback=bool(self._p('require_vehicle_stop_feedback')),
            dynamic_confirm_frames=int(self._p('dynamic.confirm_frames')),
            dynamic_static_match_m=float(self._p('dynamic.static_match_m')),
            dynamic_clearance_m=float(self._p('dynamic.clearance_m')),
        )

    def _clock_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_vehicle(self, msg: VehicleVector) -> None:
        self.latest_vehicle = msg

    def _on_gps_path(self, msg: GpsPath) -> None:
        if not bool(self._p('auto_trigger_gps_zone')):
            return
        if not msg.parking_zone:
            self.gps_zone_armed = True
            return
        if self.gps_zone_armed:
            self.gps_zone_armed = False
            self._start_mission(
                str(self._p('gps_default_mode')),
                str(self._p('gps_default_side')),
                source=(
                    'GpsPath.parking_zone(default type; explicit type can use '
                    '/parking/gps_command)'),
            )

    def _parse_command(self, command: str) -> tuple[str, str] | None:
        normalized = command.lower().strip()
        for token in (':', ',', '/', '_', '-'):
            normalized = normalized.replace(token, ' ')
        words = normalized.split()
        if not words:
            return None
        if words[0] in ('cancel', 'reset', 'stop'):
            self.mission.cancel()
            self.manual_gate_active = False
            self.get_logger().warn('parking mission cancelled by command')
            return None
        if words[0] == 'start':
            words = words[1:]
        mode = next((word for word in words if word in (
            MODE_PARALLEL, MODE_PERPENDICULAR, 't', '1', 't자', '1자')), None)
        side = next((word for word in words if word in (
            SIDE_LEFT, SIDE_RIGHT, '좌측', '우측')), None)
        if mode in ('t', 't자'):
            mode = MODE_PERPENDICULAR
        elif mode in ('1', '1자'):
            mode = MODE_PARALLEL
        if side == '좌측':
            side = SIDE_LEFT
        elif side == '우측':
            side = SIDE_RIGHT
        if mode is None:
            return None
        return mode, side or str(self._p('gps_default_side'))

    def _on_command(self, msg: String) -> None:
        parsed = self._parse_command(msg.data)
        if parsed is None:
            if msg.data.lower().strip() not in ('cancel', 'reset', 'stop'):
                self.get_logger().error(
                    'invalid parking command: %r (use: start perpendicular right | '
                    'start parallel left | cancel)' % msg.data)
            return
        self._start_mission(parsed[0], parsed[1], source='command')

    def _start_mission(self, mode: str, side: str, source: str) -> None:
        if mode not in (MODE_PARALLEL, MODE_PERPENDICULAR) or side not in (
            SIDE_LEFT, SIDE_RIGHT
        ):
            self.get_logger().error('invalid parking type/side: %s %s' % (mode, side))
            return
        # A fresh local frame makes map coordinates deterministic and prevents
        # previous non-parking scenery from becoming a false static boundary.
        self.slam.reset(Pose2())
        accepted = self.mission.trigger(mode, side, Pose2())
        if accepted:
            self.last_icp_accepted_s = -math.inf
            if source == 'command' and bool(self._p('manual_test_publish_gps_gate')):
                self.manual_gate_active = True
                self.gps_zone_armed = False
            self.get_logger().info(
                'parking scan started: mode=%s side=%s source=%s' % (mode, side, source))
        else:
            self.get_logger().warn(
                'parking trigger ignored while state=%s' % self.mission.state.value)

    def _cloud_xy(self, msg: PointCloud2) -> np.ndarray:
        try:
            points = point_cloud2.read_points_numpy(
                msg, field_names=('x', 'y'), skip_nans=True)
            array = np.asarray(points)
            if array.dtype.names:
                return np.column_stack((array['x'], array['y'])).astype(np.float64)
            return np.asarray(array, dtype=np.float64).reshape((-1, 2))
        except (AttributeError, TypeError, ValueError):
            return np.asarray([
                (float(point[0]), float(point[1]))
                for point in point_cloud2.read_points(
                    msg, field_names=('x', 'y'), skip_nans=True)
            ], dtype=np.float64).reshape((-1, 2))

    def _on_cloud(self, msg: PointCloud2) -> None:
        points = self._cloud_xy(msg)
        odom = None
        if self.latest_vehicle is not None:
            odom = Pose2(
                float(self.latest_vehicle.x), float(self.latest_vehicle.y),
                float(self.latest_vehicle.yaw))
        result = self.slam.update(points, odom)
        self.last_icp_result = result
        now_s = self._clock_s()
        if result.accepted:
            self.last_icp_accepted_s = now_s
        prepared = voxel_downsample(points[np.isfinite(points).all(axis=1)], 0.05)
        self.latest_scan_map = transform_points(prepared, result.pose)
        map_points = self.slam.map_points()
        if self.mission.state == MissionState.SCANNING and self.slam.initialized:
            if self.mission.observe_map(map_points, result.pose):
                assert self.mission.plan is not None
                self.get_logger().info(
                    'parking plan accepted: mode=%s approach=%d reverse=%d confidence=%.2f'
                    % (
                        self.mission.plan.mode,
                        len(self.mission.plan.approach_path),
                        len(self.mission.plan.reverse_path),
                        self.mission.space.confidence if self.mission.space else 0.0,
                    ))
        self.mission.observe_dynamic(self.latest_scan_map)
        debug_period = 1.0 / max(0.1, float(self._p('debug_publish_rate_hz')))
        if now_s - self.last_debug_publish_s >= debug_period:
            self.last_debug_publish_s = now_s
            self._publish_slam_debug(msg.header.stamp)

    def _on_rear_scan(self, msg: LaserScan) -> None:
        center = math.radians(float(self._p('rear_scan_center_deg')))
        half = math.radians(float(self._p('rear_scan_half_width_deg')))
        offset = float(self._p('rear_range_offset_m'))
        values: list[float] = []
        for index, raw in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            if abs(_angle_distance(angle, center)) > half:
                continue
            if not math.isfinite(raw) or raw < msg.range_min or raw > msg.range_max:
                continue
            corrected = float(raw) - offset
            if corrected > 0.0:
                values.append(corrected)
        minimum_points = int(self._p('rear_wall_min_points'))
        if len(values) < minimum_points:
            self.latest_rear_clearance_m = None
        else:
            values_array = np.asarray(values)
            candidate = float(np.percentile(values_array, 30.0))
            support = int(np.count_nonzero(
                np.abs(values_array - candidate)
                <= float(self._p('rear_wall_cluster_m'))))
            self.latest_rear_clearance_m = candidate if support >= minimum_points else None
        self.latest_rear_scan_s = self._clock_s()

    def _rear_clearance(self, now_s: float) -> Optional[float]:
        if now_s - self.latest_rear_scan_s > float(self._p('rear_scan_stale_timeout_s')):
            return None
        return self.latest_rear_clearance_m

    def _localization_valid(self, now_s: float) -> bool:
        if not self.slam.initialized:
            return False
        if self.last_icp_result is not None and self.last_icp_result.accepted:
            return True
        return now_s - self.last_icp_accepted_s <= float(self._p('slam_stale_timeout_s'))

    def _tick(self) -> None:
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1.0e-9
        vehicle_speed = (
            float(self.latest_vehicle.v) if self.latest_vehicle is not None else None)
        output = self.mission.tick(
            self.slam.pose,
            now_s,
            rear_clearance_m=self._rear_clearance(now_s),
            vehicle_speed_mps=vehicle_speed,
            localization_valid=self._localization_valid(now_s),
        )
        self.last_output = output
        if output.state in (MissionState.COMPLETE, MissionState.IDLE):
            self.manual_gate_active = False
        self._publish_manual_gate(now.to_msg())
        msg = ParkingStatus()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.base_frame
        msg.space_found = output.space_found
        msg.path_blocked = output.path_blocked
        msg.done = output.done
        msg.v_suggest = float(output.v_suggest_mps)
        if output.reference_local is not None:
            point = RefPoint()
            point.x = float(output.reference_local.x)
            point.y = float(output.reference_local.y)
            point.yaw = float(output.reference_local.yaw)
            point.curvature = float(output.reference_local.curvature)
            msg.points.append(point)
        self.status_pub.publish(msg)
        self._publish_paths(now.to_msg())
        self._publish_markers(now.to_msg(), output)
        self._publish_diagnostics(now.to_msg(), output, now_s)

    def _publish_manual_gate(self, stamp) -> None:
        if self.manual_gate_pub is None:
            return
        msg = GpsPath()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        msg.parking_zone = self.manual_gate_active
        # A neutral one-metre point keeps the existing GpsPath freshness/data
        # contract valid before MGM changes into PARKING. It is never selected
        # as the parking path.
        point = RefPoint()
        point.x = 1.0
        point.y = 0.0
        point.yaw = 0.0
        point.curvature = 0.0
        msg.points.append(point)
        msg.fix_quality = 1
        msg.heading_source = GpsPath.HEADING_TANGENT
        self.manual_gate_pub.publish(msg)

    def _header(self, stamp) -> Header:
        header = Header()
        header.stamp = stamp
        header.frame_id = self.map_frame
        return header

    def _publish_slam_debug(self, stamp) -> None:
        header = self._header(stamp)
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = float(self.slam.pose.x)
        pose.pose.position.y = float(self.slam.pose.y)
        qz, qw = _quaternion_z(self.slam.pose.yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pose_pub.publish(pose)
        self.scan_pub.publish(point_cloud2.create_cloud_xyz32(
            header,
            [(float(x), float(y), 0.0) for x, y in self.latest_scan_map],
        ))
        map_points = self.slam.map_points()
        self.map_pub.publish(point_cloud2.create_cloud_xyz32(
            header,
            [(float(x), float(y), 0.0) for x, y in map_points],
        ))

    def _path_message(self, path_points, stamp) -> Path:
        msg = Path()
        msg.header = self._header(stamp)
        for point in path_points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(point.x)
            pose.pose.position.y = float(point.y)
            qz, qw = _quaternion_z(point.yaw)
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            msg.poses.append(pose)
        return msg

    def _publish_paths(self, stamp) -> None:
        self.path_pub.publish(self._path_message(self.mission.debug_path(), stamp))
        self.active_path_pub.publish(self._path_message(self.mission.current_path, stamp))

    def _publish_markers(self, stamp, output: MissionOutput) -> None:
        array = MarkerArray()
        delete = Marker()
        delete.header = self._header(stamp)
        delete.action = Marker.DELETEALL
        array.markers.append(delete)

        if self.mission.space is not None:
            space = self.mission.space
            side_sign = 1.0 if space.side == SIDE_LEFT else -1.0
            corners_lane = [
                (space.start_x_lane, 0.0),
                (space.end_x_lane, 0.0),
                (space.end_x_lane, side_sign * space.side_distance_m),
                (space.start_x_lane, side_sign * space.side_distance_m),
                (space.start_x_lane, 0.0),
            ]
            c = math.cos(space.lane_pose_map.yaw)
            s = math.sin(space.lane_pose_map.yaw)
            marker = Marker()
            marker.header = self._header(stamp)
            marker.ns = 'space'
            marker.id = 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.045
            marker.color.r = 0.1
            marker.color.g = 0.9
            marker.color.b = 0.2
            marker.color.a = 1.0
            for x, y in corners_lane:
                point = Point()
                point.x = space.lane_pose_map.x + c * x - s * y
                point.y = space.lane_pose_map.y + s * x + c * y
                marker.points.append(point)
            array.markers.append(marker)

        if self.mission.plan is not None:
            for marker_id, pose_value, color in (
                (2, self.mission.plan.stage_pose_map, (1.0, 0.65, 0.0)),
                (3, self.mission.plan.goal_pose_map, (0.0, 0.7, 1.0)),
            ):
                marker = Marker()
                marker.header = self._header(stamp)
                marker.ns = 'poses'
                marker.id = marker_id
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.pose.position.x = pose_value.x
                marker.pose.position.y = pose_value.y
                qz, qw = _quaternion_z(pose_value.yaw)
                marker.pose.orientation.z = qz
                marker.pose.orientation.w = qw
                marker.scale.x = 0.45
                marker.scale.y = 0.10
                marker.scale.z = 0.10
                marker.color.r, marker.color.g, marker.color.b = color
                marker.color.a = 1.0
                array.markers.append(marker)

        if output.reference_local is not None:
            ref = output.reference_local
            c = math.cos(self.slam.pose.yaw)
            s = math.sin(self.slam.pose.yaw)
            marker = Marker()
            marker.header = self._header(stamp)
            marker.ns = 'preview'
            marker.id = 4
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = self.slam.pose.x + c * ref.x - s * ref.y
            marker.pose.position.y = self.slam.pose.y + s * ref.x + c * ref.y
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.16
            marker.color.r = 1.0
            marker.color.g = 0.1
            marker.color.b = 0.8
            marker.color.a = 1.0
            array.markers.append(marker)

        text = Marker()
        text.header = self._header(stamp)
        text.ns = 'status'
        text.id = 5
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = self.slam.pose.x
        text.pose.position.y = self.slam.pose.y
        text.pose.position.z = 0.7
        text.scale.z = 0.22
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = '%s | %s | v=%.2f' % (
            output.state.value, output.status, output.v_suggest_mps)
        array.markers.append(text)
        self.marker_pub.publish(array)

    def _publish_diagnostics(self, stamp, output: MissionOutput, now_s: float) -> None:
        array = DiagnosticArray()
        array.header.stamp = stamp
        status = DiagnosticStatus()
        status.name = 'stack_parking/pipeline'
        status.hardware_id = 'multi_lidar_fusion'
        localization_ok = self._localization_valid(now_s)
        status.level = DiagnosticStatus.OK
        if not localization_ok and self.mission.state not in (
            MissionState.IDLE, MissionState.SCANNING
        ):
            status.level = DiagnosticStatus.ERROR
        elif output.path_blocked or self.mission.last_plan_error:
            status.level = DiagnosticStatus.WARN
        status.message = output.status
        icp = self.last_icp_result
        rear = self._rear_clearance(now_s)
        values = {
            'mission_state': output.state.value,
            'mode': self.mission.mode,
            'side': self.mission.side,
            'slam_pose': '%.3f %.3f %.3f' % (
                self.slam.pose.x, self.slam.pose.y, self.slam.pose.yaw),
            'slam_valid': str(localization_ok),
            'icp_accepted': str(bool(icp.accepted) if icp else False),
            'icp_rmse_m': ('%.4f' % icp.rmse_m) if icp and math.isfinite(icp.rmse_m) else 'inf',
            'icp_matches': str(icp.correspondences if icp else 0),
            'map_points': str(len(self.slam.map)),
            'rear_clearance_m': 'invalid' if rear is None else '%.3f' % rear,
            'path_blocked_dynamic_only': str(output.path_blocked),
            'progress': '%d/%d' % (output.progress_index, len(self.mission.current_path)),
            'preview_distance_m': str(float(self._p('preview_distance_m'))),
            'plan_error': self.mission.last_plan_error,
        }
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        array.status.append(status)
        self.diagnostic_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = StackParkingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
