#!/usr/bin/env python3
"""estop 게이트 — /adas/target_ref를 내는 테스트 하네스 공용 안전 바닥.

박찬미 stack_estop의 `/perception/estop`(EstopRequest)을 구독해, 하네스가 무엇을
판단했든 **최상위 우선권으로** v_ref=0을 강제한다. CLAUDE.md §4 "정지는 스테이트가
아니다 — 각 스테이트 우선권 표의 최상위 요구"와 같은 처리다.

★ 실차를 움직이는 하네스는 반드시 이 게이트를 통과시킬 것.
  (avoid_to_ref, step_injector가 이 모듈을 공유한다 — 안전 로직을 두 벌 두지 않는다.)

신선도 watchdog: EstopRequest 미수신(기동 직후)·stale(기본 250ms = 하트비트
50ms×5)이면 estop=true로 보정한다. CLAUDE.md §5.7 MGM wrapper와 같은 페일세이프이며,
판단이 아니라 입력 컨디셔닝 — 정지의 근거는 어디까지나 stack_estop의 요구다.

조향은 건드리지 않는다: v_ref만 0으로 하고 ref_points는 호출측이 유지한다
(§3 "정지 시 조향은 직전 값 유지, 급조향 금지").
"""
from fma_interfaces.msg import EstopRequest


class EstopGate:
    """하네스에 붙이는 estop 구독기 + 신선도 watchdog + 사유 로거."""

    def __init__(self, node, enabled=True, stale_s=0.25, topic='/perception/estop'):
        """node에 구독을 걸고 게이트를 초기화한다. enabled=False는 스탠드 디버깅 전용."""
        self._node = node
        self.enabled = bool(enabled)
        self.stale_s = float(stale_s)
        self.estop = False
        self.stamp = None
        self._stale_age = 0.0
        self._last_reason = None
        # 안전 신호 — 최신값만 의미 있으므로 depth 1.
        self._sub = node.create_subscription(EstopRequest, topic, self._on_estop, 1)
        if not self.enabled:
            node.get_logger().error(
                'estop_gate=false — stack_estop 안전 바닥이 무시된다. 실차 주행 금지.')

    def _on_estop(self, msg):
        self.estop = bool(msg.estop)
        self.stamp = self._node.get_clock().now()

    def block(self):
        """(정지시킬 것인가, 사유 문자열) 반환. 사유는 None이면 정상 주행."""
        if not self.enabled:
            return False, None
        if self.stamp is None:
            return True, 'ESTOP(미수신)'          # 기동 직후 — estop 노드가 뜨기 전
        age = (self._node.get_clock().now() - self.stamp).nanoseconds * 1e-9
        if age > self.stale_s:
            # 사유 문자열에 age를 넣지 말 것 — 매 틱 값이 바뀌어 변경-트리거 로그가
            # 100Hz로 도배된다. 경과시간은 전이 시점에 한 번만 찍는다.
            self._stale_age = age
            return True, 'ESTOP(stale)'           # 노드 사망·스캔 끊김
        if self.estop:
            return True, 'ESTOP'
        return False, None

    def log_reason(self, reason):
        """정지 사유가 바뀔 때만 로그 — 어느 기제가 세웠는지 사후 구분용."""
        if reason == self._last_reason:
            return
        if reason is None:
            self._node.get_logger().info('주행 재개')
        elif 'stale' in reason:
            self._node.get_logger().warn(
                f'v_ref=0 ← {reason} — estop 미갱신 {self._stale_age:.2f}s '
                f'(> {self.stale_s}s). stack_estop 사망/스캔 끊김 확인')
        else:
            self._node.get_logger().warn(f'v_ref=0 ← {reason}')
        self._last_reason = reason

    def banner(self):
        """기동 로그용 한 줄 요약."""
        return f"estop 게이트 {'ON' if self.enabled else '★OFF★'} (stale {self.stale_s}s)"
