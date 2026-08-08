#!/usr/bin/env python3
"""현장 세션을 흉내낸 합성 데이터 — 분석기의 ③·ⓐⓑⓒ 경로 검증용.

시나리오:
  0-4s   cone 3m 구간 — 안정 감지 (gap 3.0)
  4-8s   cone 1m 구간 — 불안정 감지 (절반만 감지)
  8-12s  ⓐ — 회피 목표점 있음, estop 미발동
  12-16s ⓑ — estop 발동 (정적)
  16-20s ⓒ — narrow_gap, estop 발동
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from fma_interfaces.msg import AvoidStatus, EstopRequest, RefPoint


class Synth(Node):
    def __init__(self):
        super().__init__('synth_session')
        self.avoid = self.create_publisher(AvoidStatus, '/perception/avoid', 10)
        self.estop = self.create_publisher(EstopRequest, '/perception/estop', 1)
        self.static_e = self.create_publisher(Bool, '/perception/static_estop', 1)
        self.dyn_e = self.create_publisher(Bool, '/perception/dynamic_estop', 1)
        self.event = self.create_publisher(String, '/test/event', 10)
        self.t0 = time.time()
        self.n = 0
        self.marked = set()
        self.create_timer(0.1, self.tick)

    def mark(self, label):
        if label in self.marked:
            return
        m = String()
        m.data = label
        self.event.publish(m)
        self.marked.add(label)
        print(f'  ▶ {label}')

    def tick(self):
        t = time.time() - self.t0
        self.n += 1
        a = AvoidStatus()
        a.header.stamp = self.get_clock().now().to_msg()
        a.ttc = 1e9
        est, st, dy = False, False, False

        if t < 4:
            self.mark('cone 3m 시작 (③ 감지 신뢰 거리)')
            a.obstacle_detected = True
            p = RefPoint()
            p.x, p.y = 3.76, 0.0          # gap 3.00 (라이다 0.76 오프셋)
            a.points = [p]
        elif t < 8:
            self.mark('cone 1m 시작 (③ 감지 신뢰 거리)')
            if self.n % 2 == 0:                       # 절반만 감지 = 불안정
                a.obstacle_detected = True
                p = RefPoint()
                p.x, p.y = 1.76, 0.0      # gap 1.00
                a.points = [p]
        elif t < 12:
            self.mark('ⓐ 3m 콘 회피 — estop 미발동 확인')
            a.obstacle_detected = True
            p = RefPoint()
            p.x, p.y = 2.5, 0.46
            a.points = [p]
        elif t < 16:
            self.mark('ⓑ 1m 급투입 — estop 발동 확인')
            a.obstacle_detected = True
            est, st = True, True
        else:
            self.mark('ⓒ 연석 접근 — avoid 미진입 + 정지 확인')
            a.obstacle_detected = True
            a.narrow_gap = True
            est, st = True, True

        self.avoid.publish(a)
        e = EstopRequest()
        e.header.stamp = self.get_clock().now().to_msg()
        e.estop = est
        self.estop.publish(e)
        self.static_e.publish(Bool(data=st))
        self.dyn_e.publish(Bool(data=dy))


def main():
    rclpy.init()
    n = Synth()
    end = time.time() + 20.5
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
