#!/usr/bin/env python3
"""avoid_to_ref estop 게이트 검증 (하드웨어 없음).  이기돈

ⓐⓑⓒ 실차 시험 전에 안전 게이트가 살아 있는지 확인하는 회귀 시험. LiDAR·CAN·dSPACE
없이 /perception/avoid 와 /perception/estop 을 합성 발행하고 /adas/target_ref 를 관찰한다.

  터미널 1: ros2 run stack_avoid avoid_to_ref --ros-args \
              -p target_speed_mps:=0.2 -p straight_when_clear:=true
  터미널 2: python3 src/stack_avoid/tools/test_estop_gate.py     # 전부 ✓ 여야 함

시나리오를 순서대로 밟으며 /adas/target_ref 의 v_ref 를 관찰한다:
  1) estop 미발행                     → v_ref=0 이어야 함 (미수신 페일세이프)
  2) estop=false 발행 + 장애물+목표점  → v_ref>0 (회피 주행)
  3) estop=true                       → v_ref=0 (안전 바닥)
  4) estop=false 복귀                 → v_ref>0 (재개)
  5) estop 발행 중단 (stale 0.25s)    → v_ref=0 (watchdog)
"""
import time

import rclpy
from rclpy.node import Node
from fma_interfaces.msg import AvoidStatus, EstopRequest, RefPoint, TargetRef


class Tester(Node):
    def __init__(self):
        super().__init__('estop_gate_tester')
        self.avoid_pub = self.create_publisher(AvoidStatus, '/perception/avoid', 10)
        self.estop_pub = self.create_publisher(EstopRequest, '/perception/estop', 1)
        self.create_subscription(TargetRef, '/adas/target_ref', self._on_ref, 1)
        self.last_v = None
        self.estop_val = None          # None = 발행 안 함
        self.create_timer(0.05, self._pub)

    def _on_ref(self, m):
        self.last_v = float(m.v_ref)

    def _pub(self):
        a = AvoidStatus()
        a.header.stamp = self.get_clock().now().to_msg()
        a.obstacle_detected = True
        p = RefPoint()
        p.x, p.y = 1.5, 0.46          # 전방 1.5m, 측방 0.46m 회피 목표점
        a.points = [p]
        a.ttc = 3.0
        self.avoid_pub.publish(a)
        if self.estop_val is not None:
            e = EstopRequest()
            e.header.stamp = self.get_clock().now().to_msg()
            e.estop = self.estop_val
            self.estop_pub.publish(e)


def settle(node, sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def main():
    rclpy.init()
    n = Tester()
    results = []

    def step(name, estop, wait, expect_moving):
        n.estop_val = estop
        settle(n, wait)
        v = n.last_v
        ok = (v is not None) and ((v > 1e-3) == expect_moving)
        results.append((name, v, expect_moving, ok))
        print(f"{'✓' if ok else '✗'} {name:34s} v_ref={v}  (기대: {'주행' if expect_moving else '정지'})")

    settle(n, 1.0)   # 노드 연결 대기
    step('1) estop 미수신', None, 1.0, False)
    step('2) estop=false', False, 1.0, True)
    step('3) estop=true', True, 1.0, False)
    step('4) estop=false 복귀', False, 1.0, True)
    step('5) estop 발행 중단 (stale)', None, 1.5, False)

    n.destroy_node()
    rclpy.shutdown()
    bad = [r for r in results if not r[3]]
    print('\n=== 결과: ' + ('전부 통과' if not bad else f'{len(bad)}건 실패') + ' ===')
    raise SystemExit(1 if bad else 0)


if __name__ == '__main__':
    main()
