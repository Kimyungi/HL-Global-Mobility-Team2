#!/usr/bin/env python3
"""OFFLINE TEST ONLY validator for the production recovery output chain."""

import json
import math
import time
from collections import deque

import rclpy
from fma_interfaces.msg import AvoidStatus, EstopRequest, TargetRef
from rclpy.node import Node
from std_msgs.msg import String


CHECK_NAMES = (
    'WAIT_REVERSE_DELAY',
    '10SEC_WAIT',
    'REVERSE_ACTIVE',
    'REVERSE_V_REF_-0.30',
    'FRONT_CLEAR_STOP',
    'STOP_HOLD_0.5SEC',
    'WAIT_AVOIDANCE',
    'FRESH_AVOID_STATUS',
    'FRESH_MGM_AVOID_TARGET',
    'AVOID_TARGET_PASSTHROUGH',
)


def target_signature(message):
    return (
        int(message.header.stamp.sec),
        int(message.header.stamp.nanosec),
        str(message.header.frame_id),
        int(message.state),
        round(float(message.v_ref), 6),
        tuple((
            round(float(point.x), 6),
            round(float(point.y), 6),
            round(float(point.yaw), 6),
            round(float(point.curvature), 6),
        ) for point in message.ref_points),
    )


class OfflineRecoveryValidator(Node):
    def __init__(self):
        super().__init__('offline_recovery_validator')
        self.started = time.monotonic()
        self.checks = {name: False for name in CHECK_NAMES}
        self.events = {}
        self.last_state = None
        self.stop_started = None
        self.wait_avoidance_started = None
        self.negative_seen = False
        self.estop_active = None
        self.recent_mgm = deque(maxlen=100)
        self.final_reported = False

        self.create_subscription(
            String, '/perception/reverse_recovery/status',
            self._status_callback, 10)
        self.create_subscription(
            EstopRequest, '/perception/estop', self._estop_callback, 10)
        self.create_subscription(
            AvoidStatus, '/perception/avoid', self._avoid_callback, 10)
        self.create_subscription(
            TargetRef, '/adas/target_ref_mgm', self._mgm_callback, 10)
        self.create_subscription(
            TargetRef, '/adas/target_ref', self._final_callback, 10)
        self.create_timer(1.0, self._maybe_report_pass)
        print('OFFLINE RECOVERY TEST', flush=True)

    def _elapsed(self):
        return time.monotonic() - self.started

    def _mark(self, name):
        if not self.checks[name]:
            self.checks[name] = True
            self.events[name] = self._elapsed()
            print(f'[EVENT] {name} t={self.events[name]:.3f}s', flush=True)

    def _estop_callback(self, message):
        previous = self.estop_active
        self.estop_active = bool(message.estop)
        if self.negative_seen and previous and not self.estop_active:
            self.events.setdefault('ESTOP_CLEAR', self._elapsed())

    def _avoid_callback(self, message):
        if (
            self.wait_avoidance_started is not None
            and message.obstacle_detected
            and message.avoidable
        ):
            self._mark('FRESH_AVOID_STATUS')

    def _mgm_callback(self, message):
        signature = target_signature(message)
        self.recent_mgm.append((self._elapsed(), signature))
        if (
            self.wait_avoidance_started is not None
            and message.state == TargetRef.STATE_AVOID
            and message.ref_points
            and all(math.isfinite(value) for point in message.ref_points
                    for value in (
                        point.x, point.y, point.yaw, point.curvature))
        ):
            self._mark('FRESH_MGM_AVOID_TARGET')

    def _final_callback(self, message):
        now = self._elapsed()
        if math.isclose(float(message.v_ref), -0.30, abs_tol=1e-4):
            self.negative_seen = True
            self._mark('REVERSE_V_REF_-0.30')
        elif self.negative_seen and self.estop_active is False:
            self._mark('FRONT_CLEAR_STOP')
        if (
            self.checks['FRESH_MGM_AVOID_TARGET']
            and self.last_state == 'NORMAL'
            and any(
                stamp >= self.wait_avoidance_started
                and signature == target_signature(message)
                for stamp, signature in self.recent_mgm)
        ):
            self._mark('AVOID_TARGET_PASSTHROUGH')

    def _status_callback(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        state = status.get('state')
        now = self._elapsed()
        if state != self.last_state:
            print(f'[STATE] {self.last_state} -> {state} t={now:.3f}s',
                  flush=True)
            self.last_state = state
        if state == 'WAIT_REVERSE_DELAY':
            self._mark('WAIT_REVERSE_DELAY')
        if status.get('front_wait_completed'):
            self._mark('10SEC_WAIT')
        if state == 'REVERSE_ACTIVE':
            self._mark('REVERSE_ACTIVE')
        if state == 'STOP_AFTER_REVERSE' and self.negative_seen:
            if self.stop_started is None:
                self.stop_started = now
        if state == 'WAIT_AVOIDANCE':
            self._mark('WAIT_AVOIDANCE')
            if self.wait_avoidance_started is None:
                self.wait_avoidance_started = now
            if (
                self.stop_started is not None
                and now - self.stop_started >= 0.45
            ):
                self._mark('STOP_HOLD_0.5SEC')
            if (
                status.get('avoid_status_fresh')
                and status.get('avoid_obstacle_detected')
                and status.get('avoid_avoidable')
            ):
                self._mark('FRESH_AVOID_STATUS')
            if (
                status.get('mgm_target_fresh')
                and status.get('mgm_state') == TargetRef.STATE_AVOID
                and status.get('mgm_ref_points_valid')
            ):
                self._mark('FRESH_MGM_AVOID_TARGET')

    def _maybe_report_pass(self):
        if all(self.checks.values()) and not self.final_reported:
            self.report()
            self.final_reported = True

    def report(self):
        print('\nOFFLINE RECOVERY TEST', flush=True)
        for name in CHECK_NAMES:
            result = 'PASS' if self.checks[name] else 'FAIL'
            stamp = self.events.get(name)
            suffix = '' if stamp is None else f' ({stamp:.3f}s)'
            print(f'{name}: {result}{suffix}', flush=True)
        final = 'PASS' if all(self.checks.values()) else 'FAIL'
        print(f'FINAL: {final}', flush=True)

    def destroy_node(self):
        if not self.final_reported:
            self.report()
            self.final_reported = True
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OfflineRecoveryValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
