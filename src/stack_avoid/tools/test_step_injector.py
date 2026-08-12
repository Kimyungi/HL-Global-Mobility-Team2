#!/usr/bin/env python3
"""step_injector 검증: 계단이 실제로 나오는가 + estop이 최상위인가."""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from fma_interfaces.msg import EstopRequest, TargetRef


class T(Node):
    def __init__(self):
        super().__init__('step_tester')
        self.estop_pub = self.create_publisher(EstopRequest, '/perception/estop', 1)
        self.create_subscription(TargetRef, '/adas/target_ref', self._ref, 10)
        self.create_subscription(String, '/test/event', self._ev, 10)
        self.ys, self.vs, self.events = [], [], []
        self.estop_val = False
        self.create_timer(0.05, self._pub)

    def _ref(self, m):
        if m.ref_points:
            self.ys.append(round(m.ref_points[0].y, 3))
        self.vs.append(round(m.v_ref, 3))

    def _ev(self, m):
        self.events.append(m.data)

    def _pub(self):
        if self.estop_val is None:
            return
        e = EstopRequest()
        e.header.stamp = self.get_clock().now().to_msg()
        e.estop = self.estop_val
        self.estop_pub.publish(e)


def settle(n, s):
    end = time.time() + s
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)


def main():
    rclpy.init()
    n = T()
    settle(n, 6.0)                       # settle 1s + step 1s + settle 1s + step 1s ...
    uniq = sorted(set(n.ys))
    moving = [v for v in n.vs if v > 1e-3]
    print(f'ref y 고유값: {uniq}')
    print(f'이벤트: {n.events[:8]}')
    ok1 = len(uniq) >= 3                 # 0 + 최소 2개 오프셋 → 계단 생성됨
    ok2 = len(moving) > 0                # estop=false 동안 주행 명령이 나옴
    print(f"{'✓' if ok1 else '✗'} 계단 생성 (고유 y {len(uniq)}종)")
    print(f"{'✓' if ok2 else '✗'} estop=false 시 v_ref>0 ({len(moving)}회)")

    n.estop_val = True
    settle(n, 0.5)          # 전이 유예 — estop 발행 주기(50ms)+전파만큼은 이전 명령이 남는다
    n.vs.clear()
    settle(n, 1.0)
    ok3 = len(n.vs) > 0 and all(v == 0.0 for v in n.vs)
    print(f"{'✓' if ok3 else '✗'} estop=true 시 v_ref 전부 0 ({len(n.vs)}샘플)")

    n.estop_val = None                   # 발행 중단 → stale
    settle(n, 0.5)
    n.vs.clear()
    settle(n, 1.0)
    ok4 = len(n.vs) > 0 and all(v == 0.0 for v in n.vs)
    print(f"{'✓' if ok4 else '✗'} stale 시 v_ref 전부 0 ({len(n.vs)}샘플)")

    n.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if all([ok1, ok2, ok3, ok4]) else 1)


if __name__ == '__main__':
    main()
