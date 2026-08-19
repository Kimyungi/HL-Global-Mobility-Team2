#!/usr/bin/env python3
"""Integrated reverse-recovery decision and TargetRef safety gate."""

import copy
import json
import math
from enum import Enum

import rclpy
from fma_interfaces.msg import EstopRequest, TargetRef
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


REVERSE_CONFIRM_TOKEN = 'I_CONFIRM_REVERSE_RECOVERY_ACTUATION'
ABSOLUTE_REVERSE_SPEED_LIMIT_MPS = 0.30


class RecoveryState(str, Enum):
    NORMAL = 'NORMAL'
    WAIT_FRONT_5SEC = 'WAIT_FRONT_5SEC'
    WAIT_REAR_CLEAR = 'WAIT_REAR_CLEAR'
    REVERSE_READY = 'REVERSE_READY'
    REVERSE_ACTIVE = 'REVERSE_ACTIVE'
    STOP_AFTER_REVERSE = 'STOP_AFTER_REVERSE'
    FAULT_STOP = 'FAULT_STOP'


def validate_parameters(
        reverse_wait_sec, reverse_speed_mps, max_abs_reverse_speed_mps,
        reverse_max_duration_sec, post_reverse_stop_hold_sec,
        front_scan_timeout_sec, rear_scan_timeout_sec,
        status_stale_timeout_sec):
    if not math.isfinite(reverse_wait_sec) or reverse_wait_sec <= 0.0:
        raise ValueError('reverse_wait_sec must be finite and > 0')
    if not math.isfinite(reverse_speed_mps) or reverse_speed_mps >= 0.0:
        raise ValueError('reverse_speed_mps must be finite and < 0')
    if (
        not math.isfinite(max_abs_reverse_speed_mps)
        or max_abs_reverse_speed_mps <= 0.0
        or max_abs_reverse_speed_mps > ABSOLUTE_REVERSE_SPEED_LIMIT_MPS
    ):
        raise ValueError(
            'max_abs_reverse_speed_mps must be finite, > 0, and <= 0.30')
    if abs(reverse_speed_mps) > max_abs_reverse_speed_mps:
        raise ValueError(
            'abs(reverse_speed_mps) must be <= max_abs_reverse_speed_mps')
    if not math.isfinite(reverse_max_duration_sec):
        raise ValueError('reverse_max_duration_sec must be finite')
    for name, value in (
        ('post_reverse_stop_hold_sec', post_reverse_stop_hold_sec),
        ('front_scan_timeout_sec', front_scan_timeout_sec),
        ('rear_scan_timeout_sec', rear_scan_timeout_sec),
        ('status_stale_timeout_sec', status_stale_timeout_sec),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and > 0')


def analyze_rear_scan(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    *,
    rear_lidar_x_m=-0.055,
    rear_lidar_y_m=0.0,
    rear_lidar_yaw_rad=-1.51354952733,
    min_range_m=0.15,
    max_range_m=5.0,
    rear_roi_min_x_m=-0.80,
    rear_roi_max_x_m=-0.15,
    rear_roi_half_width_m=0.30,
    rear_cluster_min_points=3,
    max_index_gap=1,
    rear_cluster_gap_m=0.12,
):
    """Transform a rear LaserScan into base_link and inspect the rear ROI."""
    result = {
        'valid_scan': False,
        'rear_clear': False,
        'rear_nearest_distance': None,
        'rear_nearest_x': None,
        'rear_nearest_y': None,
        'rear_cluster_points': 0,
        'rear_point_count': 0,
        'rear_cluster_sizes': [],
        'rear_block_reason': 'INVALID_REAR_SCAN',
    }
    if (
        not ranges
        or not math.isfinite(angle_min)
        or not math.isfinite(angle_increment)
        or angle_increment == 0.0
        or not all(math.isfinite(float(value)) for value in (
            rear_lidar_x_m, rear_lidar_y_m, rear_lidar_yaw_rad))
    ):
        return result

    minimum = max(float(range_min), float(min_range_m))
    maximum = min(float(range_max), float(max_range_m))
    if not math.isfinite(minimum) or not math.isfinite(maximum) or maximum < minimum:
        return result

    cos_yaw = math.cos(float(rear_lidar_yaw_rad))
    sin_yaw = math.sin(float(rear_lidar_yaw_rad))
    points = []
    valid_range_count = 0
    for index, value in enumerate(ranges):
        if (
            not math.isfinite(value)
            or value <= 0.0
            or not minimum <= value <= maximum
        ):
            continue
        valid_range_count += 1
        angle = angle_min + index * angle_increment
        sensor_x = float(value) * math.cos(angle)
        sensor_y = float(value) * math.sin(angle)
        x_value = (
            float(rear_lidar_x_m)
            + cos_yaw * sensor_x - sin_yaw * sensor_y)
        y_value = (
            float(rear_lidar_y_m)
            + sin_yaw * sensor_x + cos_yaw * sensor_y)
        if (
            rear_roi_min_x_m <= x_value <= rear_roi_max_x_m
            and abs(y_value) <= rear_roi_half_width_m
        ):
            points.append((index, x_value, y_value))

    if valid_range_count == 0:
        return result
    result['valid_scan'] = True
    result['rear_point_count'] = len(points)

    clusters = []
    for point in points:
        if not clusters:
            clusters.append([point])
            continue
        previous = clusters[-1][-1]
        index_close = point[0] - previous[0] <= max_index_gap + 1
        spatial_close = math.hypot(
            point[1] - previous[1], point[2] - previous[2]
        ) <= rear_cluster_gap_m
        if index_close and spatial_close:
            clusters[-1].append(point)
        else:
            clusters.append([point])

    result['rear_cluster_sizes'] = [len(cluster) for cluster in clusters]
    accepted = [
        cluster for cluster in clusters
        if len(cluster) >= int(rear_cluster_min_points)
    ]
    if not accepted:
        result['rear_clear'] = True
        result['rear_block_reason'] = ''
        return result

    nearest_cluster = min(
        accepted,
        key=lambda cluster: min(
            math.hypot(point[1], point[2]) for point in cluster))
    nearest_point = min(
        nearest_cluster,
        key=lambda point: math.hypot(point[1], point[2]))
    result['rear_nearest_distance'] = math.hypot(
        nearest_point[1], nearest_point[2])
    result['rear_nearest_x'] = nearest_point[1]
    result['rear_nearest_y'] = nearest_point[2]
    result['rear_cluster_points'] = len(nearest_cluster)
    result['rear_clear'] = False
    result['rear_block_reason'] = 'REAR_BLOCKED'
    return result


def build_output_target_ref(source, output_override):
    """Deep-copy an MGM TargetRef and optionally override only v_ref."""
    output = copy.deepcopy(source)
    if output_override is not None:
        output.v_ref = float(output_override)
    return output


class ReverseRecoveryController:
    """Pure recovery state machine shared by the ROS node and unit tests."""

    def __init__(
        self,
        *,
        reverse_wait_sec=5.0,
        reverse_speed_mps=-0.05,
        max_abs_reverse_speed_mps=0.10,
        reverse_max_duration_sec=2.0,
        post_reverse_stop_hold_sec=0.5,
        actuation_authorized=False,
    ):
        validate_parameters(
            reverse_wait_sec, reverse_speed_mps, max_abs_reverse_speed_mps,
            reverse_max_duration_sec, post_reverse_stop_hold_sec,
            0.25, 0.25, 0.25)
        self.reverse_wait_sec = float(reverse_wait_sec)
        self.reverse_speed_mps = float(reverse_speed_mps)
        self.max_abs_reverse_speed_mps = float(max_abs_reverse_speed_mps)
        self.reverse_max_duration_sec = float(reverse_max_duration_sec)
        self.post_reverse_stop_hold_sec = float(post_reverse_stop_hold_sec)
        self.actuation_authorized = bool(actuation_authorized)
        self.state = RecoveryState.NORMAL
        self.front_wait_started_at = None
        self.front_wait_completed = False
        self.reverse_started_at = None
        self.stop_started_at = None
        self.last_stop_reason = ''
        self.fault_latched = False

    def _snapshot(self, now_sec, output_override):
        front_wait_elapsed = 0.0
        if self.front_wait_started_at is not None:
            front_wait_elapsed = max(
                0.0, float(now_sec) - self.front_wait_started_at)
        reverse_elapsed = 0.0
        if self.reverse_started_at is not None:
            reverse_elapsed = max(0.0, float(now_sec) - self.reverse_started_at)
            if self.reverse_max_duration_sec > 0.0:
                reverse_elapsed = min(
                    reverse_elapsed, self.reverse_max_duration_sec)
        return {
            'state': self.state.value,
            'output_override': output_override,
            'front_wait_elapsed_sec': front_wait_elapsed,
            'front_wait_completed': self.front_wait_completed,
            'waiting_for_rear_clear': (
                self.state == RecoveryState.WAIT_REAR_CLEAR),
            'reverse_ready': self.state in (
                RecoveryState.REVERSE_READY,
                RecoveryState.REVERSE_ACTIVE,
            ),
            'reverse_elapsed_sec': reverse_elapsed,
            'last_stop_reason': self.last_stop_reason,
            'fault_latched': self.fault_latched,
            'normal_forwarding': (
                self.state == RecoveryState.NORMAL
                and output_override is None),
        }

    def _enter_fault(self, reason):
        self.state = RecoveryState.FAULT_STOP
        self.fault_latched = True
        self.last_stop_reason = reason

    def _start_stop_hold(self, now_sec, reason):
        self.state = RecoveryState.STOP_AFTER_REVERSE
        self.stop_started_at = float(now_sec)
        self.last_stop_reason = reason

    def update(
        self,
        now_sec,
        *,
        estop_active,
        front_obstacle_present,
        front_scan_timeout,
        rear_scan_received,
        rear_scan_timeout,
        rear_clear,
        status_fresh,
    ):
        now_sec = float(now_sec)

        # Clearing E-Stop ends recovery regardless of rear occupancy.
        if not estop_active:
            if self.state == RecoveryState.NORMAL:
                return self._snapshot(now_sec, None)
            if self.state != RecoveryState.STOP_AFTER_REVERSE:
                self._start_stop_hold(now_sec, 'ESTOP_CLEARED')
            stop_elapsed = now_sec - self.stop_started_at
            if stop_elapsed >= self.post_reverse_stop_hold_sec:
                self.state = RecoveryState.NORMAL
                self.front_wait_started_at = None
                self.front_wait_completed = False
                self.reverse_started_at = None
                self.stop_started_at = None
                self.fault_latched = False
                return self._snapshot(now_sec, None)
            return self._snapshot(now_sec, 0.0)

        if self.state == RecoveryState.FAULT_STOP:
            return self._snapshot(now_sec, 0.0)
        if self.state == RecoveryState.STOP_AFTER_REVERSE:
            return self._snapshot(now_sec, 0.0)

        if not status_fresh:
            self._enter_fault('STATUS_STALE')
            return self._snapshot(now_sec, 0.0)
        if front_scan_timeout:
            self._enter_fault('FRONT_SCAN_TIMEOUT')
            return self._snapshot(now_sec, 0.0)
        if not front_obstacle_present:
            self._start_stop_hold(now_sec, 'NO_FRONT_OBSTACLE')
            return self._snapshot(now_sec, 0.0)
        if not rear_scan_received:
            self._enter_fault('REAR_SCAN_NOT_RECEIVED')
            return self._snapshot(now_sec, 0.0)
        if rear_scan_timeout:
            self._enter_fault('REAR_SCAN_TIMEOUT')
            return self._snapshot(now_sec, 0.0)

        if self.state == RecoveryState.NORMAL:
            self.state = RecoveryState.WAIT_FRONT_5SEC
            self.front_wait_started_at = now_sec
            self.front_wait_completed = False

        if self.state == RecoveryState.WAIT_FRONT_5SEC:
            elapsed = now_sec - self.front_wait_started_at
            if elapsed < self.reverse_wait_sec:
                self.last_stop_reason = 'FRONT_WAIT'
                return self._snapshot(now_sec, 0.0)
            self.front_wait_completed = True

        if not rear_clear:
            self.state = RecoveryState.WAIT_REAR_CLEAR
            self.last_stop_reason = 'REAR_BLOCKED'
            return self._snapshot(now_sec, 0.0)

        if self.state in (
            RecoveryState.WAIT_FRONT_5SEC,
            RecoveryState.WAIT_REAR_CLEAR,
            RecoveryState.REVERSE_READY,
        ):
            self.state = RecoveryState.REVERSE_READY
            if not self.actuation_authorized:
                self.last_stop_reason = 'ACTUATION_NOT_CONFIRMED'
                return self._snapshot(now_sec, 0.0)
            self.state = RecoveryState.REVERSE_ACTIVE
            self.reverse_started_at = now_sec
            self.last_stop_reason = ''

        if self.state == RecoveryState.REVERSE_ACTIVE:
            elapsed = now_sec - self.reverse_started_at
            if (
                self.reverse_max_duration_sec > 0.0
                and elapsed >= self.reverse_max_duration_sec
            ):
                self._start_stop_hold(now_sec, 'MAX_REVERSE_DURATION')
                return self._snapshot(now_sec, 0.0)
            return self._snapshot(now_sec, self.reverse_speed_mps)

        return self._snapshot(now_sec, 0.0)

    def force_internal_fault(self):
        self._enter_fault('INTERNAL_ERROR')


class ReverseRecoveryNode(Node):
    """Final TargetRef publisher for normal driving and reverse recovery."""

    def __init__(self):
        super().__init__('reverse_recovery_node')
        defaults = {
            'reverse_wait_sec': 5.0,
            'reverse_speed_mps': -0.30,
            'max_abs_reverse_speed_mps': 0.30,
            'reverse_max_duration_sec': 0.0,
            'post_reverse_stop_hold_sec': 0.5,
            'front_scan_timeout_sec': 0.25,
            'rear_scan_timeout_sec': 0.25,
            'status_stale_timeout_sec': 0.25,
            'reverse_actuation_enabled': False,
            'reverse_confirm_token': 'NOT_CONFIRMED',
            'rear_scan_topic': '/rear/scan',
            'rear_roi_min_x_m': -0.80,
            'rear_roi_max_x_m': -0.15,
            'rear_roi_half_width_m': 0.30,
            'rear_cluster_min_points': 3,
            'rear_cluster_gap_m': 0.12,
            'rear_lidar_x_m': -0.055,
            'rear_lidar_y_m': 0.0,
            'rear_lidar_z_m': 0.065,
            'rear_lidar_yaw_rad': -1.51354952733,
            'min_range_m': 0.15,
            'max_range_m': 5.0,
            'max_index_gap': 1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        value = lambda name: self.get_parameter(name).value
        self.reverse_wait_sec = float(value('reverse_wait_sec'))
        self.reverse_speed_mps = float(value('reverse_speed_mps'))
        self.max_abs_reverse_speed_mps = float(
            value('max_abs_reverse_speed_mps'))
        self.reverse_max_duration_sec = float(
            value('reverse_max_duration_sec'))
        self.post_reverse_stop_hold_sec = float(
            value('post_reverse_stop_hold_sec'))
        self.front_scan_timeout_sec = float(value('front_scan_timeout_sec'))
        self.rear_scan_timeout_sec = float(value('rear_scan_timeout_sec'))
        self.status_stale_timeout_sec = float(
            value('status_stale_timeout_sec'))
        token = str(value('reverse_confirm_token'))
        self.actuation_authorized = bool(
            value('reverse_actuation_enabled')
            and token == REVERSE_CONFIRM_TOKEN)
        validate_parameters(
            self.reverse_wait_sec, self.reverse_speed_mps,
            self.max_abs_reverse_speed_mps,
            self.reverse_max_duration_sec,
            self.post_reverse_stop_hold_sec,
            self.front_scan_timeout_sec,
            self.rear_scan_timeout_sec,
            self.status_stale_timeout_sec)

        self.scan_geometry = {
            name: value(name) for name in (
                'rear_roi_min_x_m', 'rear_roi_max_x_m',
                'rear_roi_half_width_m', 'rear_cluster_min_points',
                'rear_cluster_gap_m', 'rear_lidar_x_m', 'rear_lidar_y_m',
                'rear_lidar_yaw_rad', 'min_range_m', 'max_range_m',
                'max_index_gap',
            )
        }
        if not (
            float(self.scan_geometry['rear_roi_min_x_m'])
            < float(self.scan_geometry['rear_roi_max_x_m']) < 0.0
        ):
            raise ValueError('Require rear_roi_min_x_m < rear_roi_max_x_m < 0')
        if int(self.scan_geometry['rear_cluster_min_points']) < 2:
            raise ValueError('rear_cluster_min_points must be >= 2')

        self.controller = ReverseRecoveryController(
            reverse_wait_sec=self.reverse_wait_sec,
            reverse_speed_mps=self.reverse_speed_mps,
            max_abs_reverse_speed_mps=self.max_abs_reverse_speed_mps,
            reverse_max_duration_sec=self.reverse_max_duration_sec,
            post_reverse_stop_hold_sec=self.post_reverse_stop_hold_sec,
            actuation_authorized=self.actuation_authorized)

        self.estop_active = True
        self.front_obstacle_present = False
        self.status_scan_timeout = True
        self.latest_mgm_ref = None
        self.last_output_v_ref = None
        self.last_snapshot = None
        self.last_front_scan_time = None
        self.last_front_scan_stamp = None
        self.last_status_time = None
        self.last_rear_scan_time = None
        self.last_rear_scan_stamp = None
        self.rear_scan_valid = False
        self.rear_clear = False
        self.rear_diagnostics = {
            'rear_nearest_distance': None,
            'rear_nearest_x': None,
            'rear_nearest_y': None,
            'rear_cluster_points': 0,
            'rear_point_count': 0,
            'rear_cluster_sizes': [],
            'rear_block_reason': 'REAR_SCAN_NOT_RECEIVED',
        }
        self.internal_error_latched = False

        self.target_pub = self.create_publisher(TargetRef, '/adas/target_ref', 1)
        self.status_pub = self.create_publisher(
            String, '/perception/reverse_recovery/status', 1)
        self.create_subscription(
            TargetRef, '/adas/target_ref_mgm', self._mgm_callback, 1)
        self.create_subscription(
            EstopRequest, '/perception/estop', self._estop_callback, 1)
        self.create_subscription(
            String, '/perception/estop/status', self._status_callback, 1)
        self.create_subscription(
            LaserScan, '/scan', self._front_scan_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, str(value('rear_scan_topic')), self._rear_scan_callback,
            qos_profile_sensor_data)
        self.safety_timer = self.create_timer(0.02, self._safety_timer_callback)
        self.status_timer = self.create_timer(0.10, self._publish_status)

        self.get_logger().warning(
            'Reverse recovery initialized; '
            f'actuation authorized={self.actuation_authorized}')

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _age_sec(self, timestamp, now):
        if timestamp is None:
            return None
        return max(0.0, (now - timestamp).nanoseconds * 1e-9)

    def _current_inputs(self):
        now = self.get_clock().now()
        front_age = self._age_sec(self.last_front_scan_time, now)
        status_age = self._age_sec(self.last_status_time, now)
        rear_age = self._age_sec(self.last_rear_scan_time, now)
        rear_received = self.last_rear_scan_time is not None
        return {
            'estop_active': self.estop_active,
            'front_obstacle_present': self.front_obstacle_present,
            'front_scan_timeout': bool(
                self.status_scan_timeout
                or front_age is None
                or front_age > self.front_scan_timeout_sec),
            'rear_scan_received': bool(rear_received and self.rear_scan_valid),
            'rear_scan_timeout': bool(
                not rear_received
                or rear_age > self.rear_scan_timeout_sec),
            'rear_clear': bool(self.rear_scan_valid and self.rear_clear),
            'status_fresh': bool(
                status_age is not None
                and status_age <= self.status_stale_timeout_sec),
            'front_scan_age_sec': front_age,
            'rear_scan_age_sec': rear_age,
            'status_age_sec': status_age,
        }

    def _evaluate(self, publish_always=False):
        inputs = self._current_inputs()
        previous_state = None if self.last_snapshot is None else self.last_snapshot['state']
        previous_override = (
            None if self.last_snapshot is None
            else self.last_snapshot['output_override'])
        snapshot = self.controller.update(
            self._now_sec(),
            **{name: inputs[name] for name in (
                'estop_active', 'front_obstacle_present',
                'front_scan_timeout', 'rear_scan_received',
                'rear_scan_timeout', 'rear_clear', 'status_fresh')})
        self.last_snapshot = snapshot
        changed = bool(
            snapshot['state'] != previous_state
            or snapshot['output_override'] != previous_override)
        if self.latest_mgm_ref is not None and (publish_always or changed):
            output = build_output_target_ref(
                self.latest_mgm_ref, snapshot['output_override'])
            self.last_output_v_ref = float(output.v_ref)
            self.target_pub.publish(output)
        return snapshot, inputs

    def _evaluate_guarded(self, publish_always=False):
        if self.internal_error_latched:
            if publish_always and self.latest_mgm_ref is not None:
                output = build_output_target_ref(self.latest_mgm_ref, 0.0)
                self.last_output_v_ref = 0.0
                self.target_pub.publish(output)
            return self.last_snapshot, self._current_inputs()
        try:
            return self._evaluate(publish_always)
        except Exception as exception:
            self.internal_error_latched = True
            self.controller.force_internal_fault()
            self.get_logger().error(
                f'Internal error, forcing v_ref=0: {exception!r}')
            if self.latest_mgm_ref is not None:
                output = build_output_target_ref(self.latest_mgm_ref, 0.0)
                self.last_output_v_ref = 0.0
                self.target_pub.publish(output)
            self.last_snapshot = self.controller._snapshot(
                self._now_sec(), 0.0)
            return self.last_snapshot, self._current_inputs()

    def _mgm_callback(self, message):
        self.latest_mgm_ref = message
        self._evaluate_guarded(publish_always=True)

    def _estop_callback(self, message):
        self.estop_active = bool(message.estop)
        self._evaluate_guarded(publish_always=True)

    def _status_callback(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.status_scan_timeout = True
            self.front_obstacle_present = False
            self.last_status_time = None
            self._evaluate_guarded(publish_always=True)
            return
        self.status_scan_timeout = bool(status.get('scan_timeout', True))
        static_x = status.get('static_nearest_cluster_min_x')
        hazard_present = bool(status.get('hazard_latched', False))
        dynamic_present = bool(
            status.get('dynamic_estop', False)
            and status.get('dynamic_x') is not None)
        self.front_obstacle_present = bool(
            static_x is not None or hazard_present or dynamic_present)
        self.last_status_time = self.get_clock().now()
        self._evaluate_guarded(publish_always=True)

    def _front_scan_callback(self, message):
        stamp = (int(message.header.stamp.sec), int(message.header.stamp.nanosec))
        if stamp == self.last_front_scan_stamp:
            return
        self.last_front_scan_stamp = stamp
        self.last_front_scan_time = self.get_clock().now()
        self._evaluate_guarded()

    def _rear_scan_callback(self, message):
        stamp = (int(message.header.stamp.sec), int(message.header.stamp.nanosec))
        if stamp == self.last_rear_scan_stamp:
            return
        self.last_rear_scan_stamp = stamp
        result = analyze_rear_scan(
            message.ranges,
            float(message.angle_min),
            float(message.angle_increment),
            float(message.range_min),
            float(message.range_max),
            **self.scan_geometry)
        self.rear_scan_valid = bool(result['valid_scan'])
        self.rear_clear = bool(result['rear_clear'])
        self.rear_diagnostics = {
            name: result[name] for name in (
                'rear_nearest_distance', 'rear_nearest_x', 'rear_nearest_y',
                'rear_cluster_points', 'rear_point_count',
                'rear_cluster_sizes', 'rear_block_reason')
        }
        if self.rear_scan_valid:
            self.last_rear_scan_time = self.get_clock().now()
        self._evaluate_guarded()

    def _safety_timer_callback(self):
        self._evaluate_guarded()

    def _publish_status(self):
        snapshot, inputs = self._evaluate_guarded()
        message = String()
        message.data = json.dumps({
            'state': snapshot['state'],
            'reverse_actuation_enabled': self.actuation_authorized,
            'front_obstacle_present': inputs['front_obstacle_present'],
            'front_scan_timeout': inputs['front_scan_timeout'],
            'front_scan_age_sec': inputs['front_scan_age_sec'],
            'status_fresh': inputs['status_fresh'],
            'status_age_sec': inputs['status_age_sec'],
            'rear_clear': inputs['rear_clear'],
            'rear_scan_received': inputs['rear_scan_received'],
            'rear_scan_timeout': inputs['rear_scan_timeout'],
            'rear_scan_age_sec': inputs['rear_scan_age_sec'],
            **self.rear_diagnostics,
            'waiting_for_rear_clear': snapshot['waiting_for_rear_clear'],
            'front_wait_elapsed_sec': snapshot['front_wait_elapsed_sec'],
            'front_wait_completed': snapshot['front_wait_completed'],
            'reverse_ready': snapshot['reverse_ready'],
            'input_mgm_v_ref': (
                None if self.latest_mgm_ref is None
                else float(self.latest_mgm_ref.v_ref)),
            'output_v_ref': self.last_output_v_ref,
            'reverse_speed_mps': self.reverse_speed_mps,
            'max_abs_reverse_speed_mps': self.max_abs_reverse_speed_mps,
            'reverse_max_duration_sec': self.reverse_max_duration_sec,
            'duration_limit_enabled': self.reverse_max_duration_sec > 0.0,
            'post_reverse_stop_hold_sec': self.post_reverse_stop_hold_sec,
            'reverse_elapsed_sec': snapshot['reverse_elapsed_sec'],
            'last_stop_reason': snapshot['last_stop_reason'],
            'fault_latched': snapshot['fault_latched'],
            'normal_forwarding': snapshot['normal_forwarding'],
        }, allow_nan=False)
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ReverseRecoveryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
