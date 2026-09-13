#!/usr/bin/env python3
"""Direct-control test harness: forward the station+1m avoidance reference.

The planner supplies x/y/yaw/curvature, including the straight tail and GPS
return after obstacle detection clears. Preserve this geometry without ray
scaling. Every output still passes through the shared E-stop gate.
"""
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from fma_interfaces.msg import AvoidStatus, RefPoint, TargetRef

from stack_avoid.estop_gate import EstopGate
from stack_avoid.path_io import stamp_ns


class AvoidToRef(Node):
    def __init__(self):
        super().__init__('avoid_to_ref')
        self.target_speed = float(self.declare_parameter('target_speed_mps', 0.2).value)
        self.straight_x = float(self.declare_parameter('straight_x_m', 2.0).value)
        self.straight_when_clear = bool(self.declare_parameter('straight_when_clear', False).value)
        self.period = float(self.declare_parameter('period_ms', 10).value) / 1000.0
        self.reference_timeout = float(self.declare_parameter(
            'reference_timeout_s', 0.5, ParameterDescriptor(read_only=True)).value)
        if not math.isfinite(self.reference_timeout) or self.reference_timeout <= 0:
            raise ValueError('reference_timeout_s must be finite and positive')
        # ── estop 게이트 (박찬미 stack_estop) — 공용 모듈 ──
        # 기본 ON. 끄는 것은 스탠드 위 단독 디버깅 전용 — 실차 주행에서는 켜둘 것.
        self.gate = EstopGate(
            self,
            enabled=bool(self.declare_parameter('estop_gate', True).value),
            # 하트비트 50ms × 5 = 250ms (CLAUDE.md §5.7 MGM wrapper와 동일 기본값)
            stale_s=float(self.declare_parameter('estop_stale_s', 0.25).value))

        self.last = None
        self.active_episode = False
        self.pub = self.create_publisher(TargetRef, '/adas/target_ref', 1)
        self.sub = self.create_subscription(AvoidStatus, '/perception/avoid', self._on_avoid, 10)
        self.timer = self.create_timer(self.period, self.tick)
        self.add_on_set_parameters_callback(self._on_set_params)
        self.get_logger().warn(
            f'avoid_to_ref: station reference passthrough | v={self.target_speed}m/s | '
            f'{self.gate.banner()} | direct vehicle control test harness')

    def _on_avoid(self, msg):
        self.last = msg
        if msg.obstacle_detected or msg.points:
            self.active_episode = True
        if msg.maneuver_done and not msg.obstacle_detected:
            self.active_episode = False

    LIVE_PARAMS = {
        'target_speed_mps': ('target_speed', float),
        'straight_x_m': ('straight_x', float),
        'straight_when_clear': ('straight_when_clear', bool),
    }

    def _on_set_params(self, params):
        pending = []
        for p in params:
            if p.name == 'use_sim_time':
                continue
            if p.name not in self.LIVE_PARAMS:
                return SetParametersResult(successful=False, reason=f'{p.name}: unsupported live parameter')
            attr, cast = self.LIVE_PARAMS[p.name]
            value = cast(p.value)
            if cast is float and (not math.isfinite(value) or value < 0):
                return SetParametersResult(successful=False, reason=f'{p.name}: finite nonnegative value required')
            pending.append((attr, value))
        for attr, value in pending:
            setattr(self, attr, value)
        return SetParametersResult(successful=True)

    @staticmethod
    def _rp(x, y=0.0, yaw=0.0, curv=0.0):
        p = RefPoint()
        p.x, p.y, p.yaw, p.curvature = float(x), float(y), float(yaw), float(curv)
        return p

    def _fresh(self, stamp):
        ns = stamp_ns(stamp)
        age = (self.get_clock().now().nanoseconds-ns)*1e-9
        return ns > 0 and 0 <= age <= self.reference_timeout

    def tick(self):
        m = TargetRef()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        a = self.last
        m.state = TargetRef.STATE_AVOID
        m.v_ref = 0.0
        m.ref_points = [self._rp(self.straight_x)]
        reason = None
        if a is None or not a.scan_valid or not self._fresh(a.header.stamp):
            reason = 'avoid input unavailable/stale'
        elif a.points:
            p = a.points[0]
            if (len(a.points) != 1 or not self._fresh(a.reference_stamp)
                    or not all(math.isfinite(v) for v in (p.x, p.y, p.yaw, p.curvature))):
                reason = 'avoid reference invalid/stale'
            else:
                m.ref_points = [self._rp(p.x, p.y, p.yaw, p.curvature)]
                m.v_ref = float(a.v_suggest) if math.isfinite(a.v_suggest) and a.v_suggest > 1e-3 else self.target_speed
        elif a.obstacle_detected or self.active_episode:
            # A missing reference during an active/unknown episode is not clear.
            reason = 'avoid path unavailable'
        else:
            m.state = TargetRef.STATE_LANE
            m.v_ref = self.target_speed if self.straight_when_clear else 0.0
            if not self.straight_when_clear:
                reason = 'clear (straight disabled)'
        self._publish(m, reason)

    def _publish(self, m, reason):
        """모든 송신의 **유일한 출구** — estop 게이트가 마지막에 반드시 적용된다.

        송신 경로가 늘어나도 게이트를 빼먹을 수 없도록 publish 를 이 헬퍼 한 곳으로
        모은다. PR #27 리뷰 반영 — 예전 gps_style 분기의 조기 return 이 게이트를
        우회했던 차단 버그의 구조적 재발 방지. 그 분기 자체는 2026-08-11 에 제거됐지만,
        **출구를 하나로 유지하는 규칙은 그대로다.**
        tick() 안에서 self.pub.publish() 를 직접 부르지 말 것.

        게이트는 v_ref 만 0으로 만든다. ref_points 는 그대로 둔다 (§3 조향 직전 값
        유지·급조향 금지). 사유를 estop 으로 덮어써서, narrow_gap 과 estop 이 동시에
        성립해도 "안전 바닥이 실제로 걸렸다"가 로그에 남는다.
        """
        blocked, why = self.gate.block()
        if blocked:
            m.v_ref = 0.0
            reason = why if reason is None else f'{why} + {reason}'
        self.gate.log_reason(reason)
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = AvoidToRef()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
