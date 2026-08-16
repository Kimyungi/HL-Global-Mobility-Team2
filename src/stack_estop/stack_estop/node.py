"""Static and motion-confirmed dynamic emergency-stop path."""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from fma_interfaces.msg import EstopRequest
from .dynamic_motion_core import DYNAMIC_DEFAULTS, DynamicMotionCore


def combine_estop_levels(scan_timeout, static_estop, dynamic_estop):
    """Single official E-Stop combination policy."""
    return bool(scan_timeout or static_estop or dynamic_estop)


class DistanceEstopController:
    """Scan-driven E-Stop hysteresis independent of perception tracking."""

    def __init__(
        self,
        on_distance_m=0.70,
        off_distance_m=0.80,
        clear_confirm_scans=3,
    ):
        if not 0.0 < on_distance_m < off_distance_m:
            raise ValueError(
                'Require 0 < estop_on_distance_m < estop_off_distance_m'
            )
        if int(clear_confirm_scans) < 1:
            raise ValueError('estop_clear_confirm_scans must be >= 1')
        self.on_distance_m = float(on_distance_m)
        self.off_distance_m = float(off_distance_m)
        self.clear_confirm_scans = int(clear_confirm_scans)
        self.current_final_estop = True
        self.clear_count = 0

    def update_from_scan(self, nearest_corridor_x_m):
        """Update exactly once for each new, valid LaserScan."""
        danger_now = bool(
            nearest_corridor_x_m is not None
            and nearest_corridor_x_m <= self.on_distance_m
        )
        if danger_now:
            self.current_final_estop = True
            self.clear_count = 0
            return self.current_final_estop

        safe_now = bool(
            nearest_corridor_x_m is None
            or nearest_corridor_x_m >= self.off_distance_m
        )
        if safe_now:
            if self.current_final_estop:
                self.clear_count += 1
                if self.clear_count >= self.clear_confirm_scans:
                    self.current_final_estop = False
                    self.clear_count = 0
            else:
                self.clear_count = 0
        else:
            # Hysteresis band: retain the level and restart clear confirmation.
            self.clear_count = 0
        return self.current_final_estop

    def force_timeout(self):
        self.current_final_estop = True
        self.clear_count = 0


def nearest_corridor_cluster_x(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    *,
    min_range_m=0.15,
    max_range_m=5.0,
    corridor_min_x_m=0.15,
    corridor_max_x_m=1.50,
    corridor_half_width_m=0.30,
    cluster_min_points=3,
    max_index_gap=1,
    max_neighbor_distance_m=0.12,
    laser_yaw_in_base_rad=1.57079632679,
):
    """Return (valid_scan, nearest cluster min x) for compatibility."""
    result = analyze_corridor_scan(
        ranges,
        angle_min,
        angle_increment,
        range_min,
        range_max,
        min_range_m=min_range_m,
        max_range_m=max_range_m,
        corridor_min_x_m=corridor_min_x_m,
        corridor_max_x_m=corridor_max_x_m,
        corridor_half_width_m=corridor_half_width_m,
        cluster_min_points=cluster_min_points,
        max_index_gap=max_index_gap,
        max_neighbor_distance_m=max_neighbor_distance_m,
        laser_yaw_in_base_rad=laser_yaw_in_base_rad,
    )
    return result['valid_scan'], result['nearest_cluster_min_x']


def analyze_corridor_scan(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    *,
    min_range_m=0.15,
    max_range_m=5.0,
    corridor_min_x_m=0.15,
    corridor_max_x_m=1.50,
    corridor_half_width_m=0.30,
    cluster_min_points=3,
    max_index_gap=1,
    max_neighbor_distance_m=0.12,
    laser_yaw_in_base_rad=1.57079632679,
):
    """Return filtering and clustering diagnostics for one LaserScan."""
    result = {
        'valid_scan': False,
        'nearest_all_range': None,
        'nearest_roi_point': None,
        'roi_point_count': 0,
        'cluster_sizes': [],
        'cluster_count': 0,
        'nearest_cluster_min_x': None,
        'nearest_cluster_mean_x': None,
        'nearest_cluster_centroid_distance': None,
        'largest_cluster_points': 0,
        'reason': 'INVALID_SCAN',
        'base_points': [],
    }
    if (
        not ranges
        or not math.isfinite(angle_increment)
        or angle_increment == 0.0
        or not math.isfinite(angle_min)
        or not math.isfinite(laser_yaw_in_base_rad)
    ):
        return result
    minimum = max(float(range_min), float(min_range_m))
    maximum = min(float(range_max), float(max_range_m))
    if (
        not math.isfinite(minimum)
        or not math.isfinite(maximum)
        or maximum < minimum
    ):
        return result

    result['valid_scan'] = True
    valid_points = []
    points = []
    for index, value in enumerate(ranges):
        if (
            not math.isfinite(value)
            or value <= 0.0
            or not minimum <= value <= maximum
        ):
            continue
        laser_angle = angle_min + index * angle_increment
        base_angle = laser_angle + laser_yaw_in_base_rad
        x_value = float(value) * math.cos(base_angle)
        y_value = float(value) * math.sin(base_angle)
        point = (index, float(value), x_value, y_value,
                 laser_angle, base_angle)
        valid_points.append(point)
        result['base_points'].append((index, x_value, y_value, float(value)))
        if (
            corridor_min_x_m <= x_value <= corridor_max_x_m
            and abs(y_value) <= corridor_half_width_m
        ):
            points.append(point)

    result['nearest_all_range'] = min(
        (point[1] for point in valid_points), default=None)
    result['roi_point_count'] = len(points)
    if points:
        nearest_roi = min(points, key=lambda point: point[2])
        result['nearest_roi_point'] = {
            'x': nearest_roi[2],
            'y': nearest_roi[3],
            'laser_angle': nearest_roi[4],
            'base_angle': nearest_roi[5],
        }

    clusters = []
    for point in points:
        if not clusters:
            clusters.append([point])
            continue
        previous = clusters[-1][-1]
        index_close = point[0] - previous[0] <= max_index_gap + 1
        spatial_close = math.hypot(
            point[2] - previous[2], point[3] - previous[3]
        ) <= max_neighbor_distance_m
        if index_close and spatial_close:
            clusters[-1].append(point)
        else:
            clusters.append([point])

    result['cluster_sizes'] = [len(cluster) for cluster in clusters]
    result['cluster_count'] = len(clusters)
    result['largest_cluster_points'] = max(
        result['cluster_sizes'], default=0)
    accepted = [
        cluster for cluster in clusters
        if len(cluster) >= cluster_min_points
    ]
    if accepted:
        nearest_cluster = min(
            accepted, key=lambda cluster: min(point[2] for point in cluster))
        result['nearest_cluster_min_x'] = min(
            point[2] for point in nearest_cluster)
        mean_x = sum(point[2] for point in nearest_cluster) / len(nearest_cluster)
        mean_y = sum(point[3] for point in nearest_cluster) / len(nearest_cluster)
        result['nearest_cluster_mean_x'] = mean_x
        result['nearest_cluster_centroid_distance'] = math.hypot(mean_x, mean_y)
        result['reason'] = 'CLUSTER_ACCEPTED'
    elif not valid_points:
        result['reason'] = 'NO_POINTS'
    elif not points:
        result['reason'] = 'OUTSIDE_CORRIDOR'
    else:
        result['reason'] = 'TOO_FEW_CLUSTER_POINTS'
    return result


class StackEstopNode(Node):

    def __init__(self):
        super().__init__('stack_estop_node')
        # ── 정지거리 실측에서 역산한 값 (2026-08-17, run_0817_032728 × rec1_024).
        # 실측 정지 프로파일: v0 0.591 m/s → **반응지연 0.13s** → 감속 0.94 m/s²
        #   → 0.76s / 0.314m 에 정지. (0.59 0.57 0.54 0.51 0.45 0.36 0.26 0.13 0.01)
        # 여기에 감지 지연이 더 붙는다 — /scan 9.9Hz × 확정 3프레임 = 0.30s.
        #   필요 거리 = 0.303·v + 1.19·(0.13·v + v²/(2·0.94))
        #   (1.19 = 위 모델이 0.6m/s 실측보다 19% 낮게 나온 만큼의 보정)
        #     0.6 m/s → 0.49m   0.8 m/s → 0.77m   1.0 m/s → 1.09m
        # 종전 0.70/0.80 은 0.6 m/s 전용이었다(여유 0.21m). v_base 1.0 상향
        # (adas_mgm/config/params.yaml)에 맞춰 1.20/1.35 로 올린다 — 여유 0.11m.
        # ⚠ 이 둘은 **생성자에서 1회만** 읽어 DistanceEstopController 에 들어간다.
        #   `ros2 param set` 이 안 먹으므로 값을 바꾸려면 **재실행**해야 한다.
        # ⚠ 정지 트리거가 멀어지면 AVOID 진입과 겹친다 — 회피로 피할 것을 estop이
        #   먼저 세울 수 있다. TTC 임계와 함께 봐야 한다 (담당: 이기돈·박찬미).
        # 속도를 되돌리면 이 값도 함께 되돌릴 것.
        self.declare_parameter('estop_on_distance_m', 1.20)
        self.declare_parameter('estop_off_distance_m', 1.35)
        self.declare_parameter('estop_clear_confirm_scans', 3)
        self.declare_parameter('scan_timeout_sec', 0.25)
        self.declare_parameter('publish_period_sec', 0.05)
        self.declare_parameter('min_range_m', 0.15)
        self.declare_parameter('max_range_m', 5.0)
        self.declare_parameter('corridor_min_x_m', 0.15)
        self.declare_parameter('corridor_max_x_m', 1.50)
        self.declare_parameter('corridor_half_width_m', 0.30)
        self.declare_parameter('cluster_min_points', 3)
        self.declare_parameter('max_index_gap', 1)
        self.declare_parameter('max_neighbor_distance_m', 0.12)
        self.declare_parameter('laser_yaw_in_base_rad', 1.57079632679)
        # Must match stack_avoid's lidar_mount.forward_angle_deg=270.0;
        # update both when the shared LiDAR is remounted.
        self.declare_parameter('debug_log_period_sec', 0.20)
        self.declare_parameter('dynamic_enabled', True)
        self.declare_parameter('dynamic_stop_distance_m', 1.00)
        dynamic_parameters = {
            'dynamic_background_min_x_m': 'background_min_x_m',
            'dynamic_background_max_x_m': 'background_max_x_m',
            'dynamic_background_min_abs_y_m': 'background_min_abs_y_m',
            'dynamic_roi_min_x_m': 'roi_min_x_m',
            'dynamic_roi_max_x_m': 'roi_max_x_m',
            'dynamic_tracking_max_distance_m': 'tracking_max_x_m',
            'dynamic_roi_half_width_m': 'roi_half_width_m',
            'dynamic_cluster_min_points': 'cluster_min_points',
            'dynamic_cluster_max_index_gap': 'cluster_max_index_gap',
            'dynamic_cluster_gap_m': 'cluster_gap_m',
            'track_association_distance_m': 'association_distance_m',
            'track_lateral_span_difference_m':
                'association_max_lateral_span_diff_m',
            'track_point_ratio_min': 'association_point_ratio_min',
            'track_point_ratio_max': 'association_point_ratio_max',
            'dormant_reconnect_max_frames': 'dormant_reconnect_max_frames',
            'dormant_reconnect_max_time_sec':
                'dormant_reconnect_max_time_sec',
            'dormant_reconnect_distance_m': 'dormant_reconnect_distance_m',
            'dormant_reconnect_max_delta_x_m':
                'dormant_reconnect_max_delta_x_m',
            'dormant_reconnect_max_lateral_span_diff_m':
                'dormant_reconnect_max_lateral_span_diff_m',
            'dormant_reconnect_point_ratio_min':
                'dormant_reconnect_point_ratio_min',
            'dormant_reconnect_point_ratio_max':
                'dormant_reconnect_point_ratio_max',
            'dormant_reconnect_min_lateral_motion_m':
                'dormant_reconnect_min_lateral_motion_m',
            'dormant_reconnect_ambiguity_margin_m':
                'dormant_reconnect_ambiguity_margin_m',
            'icp_correspondence_distance_m':
                'icp_correspondence_distance_m',
            'icp_max_translation_m': 'icp_max_translation_m',
            'icp_max_rotation_deg': 'icp_max_rotation_deg',
            'icp_max_rmse_m': 'icp_max_rmse_m',
            'dynamic_frame_max_yaw_deg': 'dynamic_frame_max_yaw_deg',
            'dynamic_residual_vy_threshold_mps':
                'dynamic_residual_vy_threshold_mps',
            'corridor_approach_vy_threshold_mps':
                'corridor_approach_vy_threshold_mps',
            'dynamic_cumulative_lateral_displacement_m':
                'dynamic_cumulative_lateral_displacement_m',
            'motion_window_frames': 'motion_window_frames',
            'dynamic_confirm_frames': 'dynamic_confirm_frames',
            'dynamic_release_frames': 'dynamic_release_frames',
            'minimum_track_age_frames': 'minimum_track_age_frames',
            'prediction_horizon_sec': 'prediction_horizon_sec',
            'hazard_clear_frames': 'hazard_clear_frames',
        }
        for ros_name, core_name in dynamic_parameters.items():
            self.declare_parameter(ros_name, DYNAMIC_DEFAULTS[core_name])

        self.controller = DistanceEstopController(
            self.get_parameter('estop_on_distance_m').value,
            self.get_parameter('estop_off_distance_m').value,
            self.get_parameter('estop_clear_confirm_scans').value,
        )
        self.scan_timeout_sec = float(
            self.get_parameter('scan_timeout_sec').value
        )
        self.publish_period_sec = float(
            self.get_parameter('publish_period_sec').value
        )
        self.debug_log_period_sec = float(
            self.get_parameter('debug_log_period_sec').value
        )
        if not math.isfinite(self.scan_timeout_sec) or self.scan_timeout_sec <= 0:
            raise ValueError('scan_timeout_sec must be finite and > 0')
        if (
            not math.isfinite(self.publish_period_sec)
            or self.publish_period_sec <= 0
        ):
            raise ValueError('publish_period_sec must be finite and > 0')
        if (
            not math.isfinite(self.debug_log_period_sec)
            or self.debug_log_period_sec < 0.20
        ):
            raise ValueError('debug_log_period_sec must be finite and >= 0.20')

        self.geometry = {
            name: self.get_parameter(name).value
            for name in (
                'min_range_m',
                'max_range_m',
                'corridor_min_x_m',
                'corridor_max_x_m',
                'corridor_half_width_m',
                'cluster_min_points',
                'max_index_gap',
                'max_neighbor_distance_m',
                'laser_yaw_in_base_rad',
            )
        }
        self.dynamic_enabled = bool(
            self.get_parameter('dynamic_enabled').value)
        dynamic_values = {
            core_name: self.get_parameter(ros_name).value
            for ros_name, core_name in dynamic_parameters.items()
        }
        dynamic_values['dynamic_stop_distance_m'] = float(
            self.get_parameter('dynamic_stop_distance_m').value)
        # Static and dynamic clustering intentionally retain their separately
        # validated thresholds (3 and 4 points respectively).
        dynamic_values['corridor_half_width_m'] = float(
            self.geometry['corridor_half_width_m'])
        self.dynamic_core = DynamicMotionCore(dynamic_values)
        self.dynamic_status = {
            'icp_valid': False,
            'dynamic_track_count': 0,
            'hazard_latched': False,
            'dynamic_estop': False,
        }
        self.last_valid_scan_time = None
        self.scan_timeout_active = True
        self.last_processed_scan_stamp = None
        self.last_debug_log_time = None
        self.last_status_publish_time = None
        self.last_static_nearest_x = None

        self.get_logger().info(
            '[ESTOP CONFIG] '
            f"laser_yaw_in_base_rad="
            f"{float(self.geometry['laser_yaw_in_base_rad']):.11f}, "
            f'stop_distance={self.controller.on_distance_m:.2f}, '
            f'release_distance={self.controller.off_distance_m:.2f}, '
            f"corridor_width="
            f"{2.0 * float(self.geometry['corridor_half_width_m']):.2f}, "
            f"min_cluster_points={int(self.geometry['cluster_min_points'])}, "
            f"max_index_gap={int(self.geometry['max_index_gap'])}, "
            f"cluster_gap="
            f"{float(self.geometry['max_neighbor_distance_m']):.3f} m, "
            f'dynamic_enabled={str(self.dynamic_enabled).lower()}, '
            f"dynamic_stop_distance_m="
            f"{dynamic_values['dynamic_stop_distance_m']:.2f}, "
            f"dynamic_cluster_min_points="
            f"{int(dynamic_values['cluster_min_points'])}, "
            f"dynamic_tracking_max_distance_m="
            f"{float(dynamic_values['tracking_max_x_m']):.2f}, "
            f'scan_timeout={self.scan_timeout_sec:.3f} s'
        )

        self.pub = self.create_publisher(
            EstopRequest, '/perception/estop', 1
        )
        self.status_pub = self.create_publisher(
            String, '/perception/estop/status', 1)
        self.dynamic_detected_pub = self.create_publisher(
            Bool, '/perception/dynamic_obstacle_detected', 1)
        self.static_estop_pub = self.create_publisher(
            Bool, '/perception/static_estop', 1)
        self.dynamic_estop_pub = self.create_publisher(
            Bool, '/perception/dynamic_estop', 1)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
        self.timer = self.create_timer(
            self.publish_period_sec, self.publish_heartbeat
        )

    def scan_callback(self, msg):
        scan_stamp = (
            int(msg.header.stamp.sec),
            int(msg.header.stamp.nanosec),
        )
        if scan_stamp == self.last_processed_scan_stamp:
            return
        self.last_processed_scan_stamp = scan_stamp

        diagnostics = analyze_corridor_scan(
            msg.ranges,
            float(msg.angle_min),
            float(msg.angle_increment),
            float(msg.range_min),
            float(msg.range_max),
            min_range_m=float(self.geometry['min_range_m']),
            max_range_m=float(self.geometry['max_range_m']),
            corridor_min_x_m=float(self.geometry['corridor_min_x_m']),
            corridor_max_x_m=float(self.geometry['corridor_max_x_m']),
            corridor_half_width_m=float(
                self.geometry['corridor_half_width_m']
            ),
            cluster_min_points=int(self.geometry['cluster_min_points']),
            max_index_gap=int(self.geometry['max_index_gap']),
            max_neighbor_distance_m=float(
                self.geometry['max_neighbor_distance_m']
            ),
            laser_yaw_in_base_rad=float(
                self.geometry['laser_yaw_in_base_rad']
            ),
        )
        if not diagnostics['valid_scan']:
            self.log_debug(diagnostics, 'INVALID_SCAN')
            return

        base_points = diagnostics.pop('base_points')
        nearest_x = diagnostics['nearest_cluster_min_x']
        self.last_static_nearest_x = nearest_x
        previous_final = self.current_final_estop
        self.controller.update_from_scan(nearest_x)
        self.last_valid_scan_time = self.get_clock().now()
        self.scan_timeout_active = False
        if self.dynamic_enabled:
            try:
                timestamp = (
                    float(msg.header.stamp.sec)
                    + float(msg.header.stamp.nanosec) * 1e-9)
                self.dynamic_status = self.dynamic_core.process(
                    base_points, timestamp, scan_valid=True)
            except Exception as error:  # Keep the independent static path alive.
                self.get_logger().error(
                    f'[ESTOP DYNAMIC ERROR] {type(error).__name__}: {error}')
                self.dynamic_status['dynamic_estop'] = (
                    self.dynamic_core.hold_after_error())
                self.dynamic_status['icp_valid'] = False
        else:
            self.dynamic_status['dynamic_estop'] = False
        if self.current_final_estop and not previous_final:
            self.publish_current_level()
        if diagnostics['reason'] != 'CLUSTER_ACCEPTED':
            reason = diagnostics['reason']
        elif nearest_x <= self.controller.on_distance_m:
            reason = 'DISTANCE_AT_OR_BELOW_THRESHOLD'
        elif nearest_x < self.controller.off_distance_m:
            reason = 'HYSTERESIS_BAND'
        else:
            reason = 'DISTANCE_OVER_THRESHOLD'
        self.log_debug(diagnostics, reason)
        self.publish_diagnostics(nearest_x, scan_timeout=False)

    @property
    def current_final_estop(self):
        return combine_estop_levels(
            self.scan_timeout_active,
            self.controller.current_final_estop,
            self.dynamic_status.get('dynamic_estop', False))

    @staticmethod
    def publish_bool(publisher, value):
        message = Bool()
        message.data = bool(value)
        publisher.publish(message)

    def publish_diagnostics(self, static_nearest_x, scan_timeout):
        now = self.get_clock().now()
        if self.last_status_publish_time is not None:
            elapsed = (now - self.last_status_publish_time).nanoseconds * 1e-9
            if elapsed < self.debug_log_period_sec:
                return
        self.last_status_publish_time = now
        static_estop = bool(self.controller.current_final_estop)
        dynamic_estop = bool(self.dynamic_status.get('dynamic_estop', False))
        if scan_timeout:
            state = reason = 'SCAN_TIMEOUT'
        elif static_estop:
            state = reason = 'STATIC_ESTOP'
        elif dynamic_estop:
            state = reason = 'DYNAMIC_ESTOP'
        elif self.dynamic_status.get('dynamic_track_count', 0):
            state = reason = 'DYNAMIC_TRACKING'
        elif self.dynamic_enabled and not self.dynamic_status.get('icp_valid', False):
            state = reason = 'ICP_INVALID'
        else:
            state = reason = 'CLEAR'
        status = {
            'state': state,
            'reason': reason,
            'static_estop': static_estop,
            'static_nearest_cluster_min_x': static_nearest_x,
            'dynamic_enabled': self.dynamic_enabled,
            **{
                key: self.dynamic_status.get(key) for key in (
                    'icp_valid', 'icp_translation_x', 'icp_translation_y',
                    'icp_rotation_rad', 'icp_rmse_m',
                    'dynamic_track_count', 'selected_dynamic_track_id',
                    'dynamic_x', 'dynamic_y', 'residual_vx', 'residual_vy',
                    'cumulative_lateral_displacement',
                    'dynamic_confirm_count', 'side_entry_event',
                    'inside_appearance_event', 'hazard_latched',
                    'dynamic_stop_distance_m',
                    'dynamic_tracking_max_distance_m', 'dynamic_estop',
                    'candidate_track_id', 'candidate_x', 'candidate_y',
                    'candidate_outside_history',
                    'candidate_side_entry_confirmed', 'hazard_track_id',
                    'hazard_stopped', 'hazard_registration_type',
                    'latch_reason', 'dynamic_rejection_reason',
                    'hazard_clear_count', 'association_reason',
                    'reconnected_previous_track_id',
                    'association_predicted_x', 'association_predicted_y',
                    'association_distance_m', 'dormant_frame_count',
                    'association_ambiguous',
                    'dormant_reconnect_rejection_reason')
            },
            'scan_timeout': bool(scan_timeout),
            'final_estop': self.current_final_estop,
        }
        message = String()
        message.data = json.dumps(status, allow_nan=False)
        self.status_pub.publish(message)
        self.publish_bool(self.dynamic_detected_pub,
                          self.dynamic_status.get('dynamic_track_count', 0) > 0)
        self.publish_bool(self.static_estop_pub, static_estop)
        self.publish_bool(self.dynamic_estop_pub, dynamic_estop)

    @staticmethod
    def format_debug_value(value):
        return '-' if value is None else f'{value:.3f}'

    def log_debug(self, diagnostics, reason):
        now = self.get_clock().now()
        if self.last_debug_log_time is not None:
            elapsed = (now - self.last_debug_log_time).nanoseconds * 1e-9
            if elapsed < self.debug_log_period_sec:
                return
        self.last_debug_log_time = now
        roi = diagnostics.get('nearest_roi_point') or {}
        self.get_logger().info(
            '[ESTOP DEBUG] '
            f"nearest_all_range={self.format_debug_value(diagnostics.get('nearest_all_range'))}, "
            f"roi_x={self.format_debug_value(roi.get('x'))}, "
            f"roi_y={self.format_debug_value(roi.get('y'))}, "
            f"laser_angle={self.format_debug_value(roi.get('laser_angle'))}, "
            f"roi_points={diagnostics.get('roi_point_count', 0)}, "
            f"clusters={diagnostics.get('cluster_count', 0)}, "
            f"cluster_sizes={diagnostics.get('cluster_sizes', [])}, "
            f"nearest_cluster_min_x="
            f"{self.format_debug_value(diagnostics.get('nearest_cluster_min_x'))}, "
            f"nearest_cluster_mean_x="
            f"{self.format_debug_value(diagnostics.get('nearest_cluster_mean_x'))}, "
            f"centroid_distance="
            f"{self.format_debug_value(diagnostics.get('nearest_cluster_centroid_distance'))}, "
            f"required={int(self.geometry['cluster_min_points'])}, "
            f"estop={str(self.current_final_estop).lower()}, "
            f'reason={reason}'
        )

    def publish_current_level(self):
        msg = EstopRequest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.estop = self.current_final_estop
        self.pub.publish(msg)

    def publish_heartbeat(self):
        # No distance decision and no clear-count change are allowed here.
        now = self.get_clock().now()
        if self.last_valid_scan_time is None:
            self.controller.force_timeout()
            self.log_debug({}, 'SCAN_TIMEOUT')
            scan_timeout = True
        else:
            scan_age_sec = (
                now - self.last_valid_scan_time
            ).nanoseconds * 1e-9
            if scan_age_sec > self.scan_timeout_sec:
                self.controller.force_timeout()
                self.log_debug({}, 'SCAN_TIMEOUT')
                scan_timeout = True
            else:
                scan_timeout = False
        self.scan_timeout_active = scan_timeout
        self.publish_current_level()
        self.publish_diagnostics(self.last_static_nearest_x, scan_timeout)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StackEstopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
