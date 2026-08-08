#!/usr/bin/env python3
"""조향 진단용: 다점(多點) 경로를 /adas/target_ref에 직접 발행 (MGM 우회).

2026-08-08 발견: GPS 성공 bag을 그대로 재생하면 조향이 반응하는데(±28°급),
완전히 똑같은 값을 매 프레임 반복하는 이 스크립트(정적)로는 state/v_ref/
n_points를 GPS와 동일하게 맞춰도 무반응이었음 — 유력한 차이는 "매 프레임
값이 조금이라도 실제로 변하는가"로 좁혀짐(고정값 = 죽은 센서로 판단해
무시하는 안전장치가 dSPACE 쪽에 있을 가능성). 기본으로 v_ref 램프업 +
y에 작은 흔들림(jitter)을 넣어 "살아있는" 신호처럼 보이게 한다.
`--jitter 0`으로 끄면 이전(완전 정적) 동작과 동일.

사용:
  python3 scripts/static_multi_point_publisher.py --n-points 20 --v-ref 0.1 --state 1
"""
import argparse
import math

import rclpy
from rclpy.node import Node

from fma_interfaces.msg import RefPoint, TargetRef


class StaticMultiPointPublisher(Node):

    def __init__(self, n_points: int, v_ref: float, state: int,
                 x_max: float, y_max: float, period_ms: int,
                 jitter_m: float, jitter_period_s: float, vref_ramp_s: float):
        super().__init__('static_multi_point_publisher')
        self.n_points = n_points
        self.v_ref = v_ref
        self.state = state
        self.x_max = x_max
        self.y_max = y_max
        self.jitter_m = jitter_m
        self.jitter_period_s = max(jitter_period_s, 1e-3)
        self.vref_ramp_s = max(vref_ramp_s, 1e-3)
        self.start_time = self.get_clock().now()

        self.pub = self.create_publisher(TargetRef, '/adas/target_ref', 1)
        self.timer = self.create_timer(period_ms / 1000.0, self.tick)
        self.get_logger().warn(
            f'다점 목표 발행 중 (테스트 전용, MGM 우회) — n_points={n_points} '
            f'v_ref={v_ref}(ramp {vref_ramp_s}s) state={state} x_max={x_max} '
            f'y_max={y_max} jitter=±{jitter_m}m/{jitter_period_s}s')

    def tick(self):
        elapsed_s = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        msg = TargetRef()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # v_ref: 0에서 목표까지 부드럽게 램프업 (실제 주행 가속 흉내)
        msg.v_ref = float(self.v_ref * min(1.0, elapsed_s / self.vref_ramp_s))
        msg.state = self.state

        # y_max 자체를 좌우로 오가게 — 고정 한쪽 편향이 아니라 실제 좌/우 전환도 되는지 확인
        y_max_signed = self.y_max * math.sin(2.0 * math.pi * elapsed_s / self.jitter_period_s)
        wobble = self.jitter_m * math.sin(2.0 * math.pi * elapsed_s / (self.jitter_period_s * 0.31))

        for i in range(self.n_points):
            t = (i + 1) / self.n_points  # 0 초과 ~ 1
            x = self.x_max * t
            y = y_max_signed * (t ** 1.3) + wobble  # 좌우로 오가는 곡선 + 잔물결
            dy_dx = (self.y_max * 1.3 * t ** 0.3) / self.x_max if self.x_max > 0 else 0.0
            yaw = math.atan(dy_dx)
            curvature = 0.15  # 대략적인 고정 곡률(참고용, dSPACE가 x,y로 자체 계산한다는 전제)
            p = RefPoint()
            p.x = float(x)
            p.y = float(y)
            p.yaw = float(yaw)
            p.curvature = float(curvature)
            msg.ref_points.append(p)

        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--n-points', type=int, default=10)
    parser.add_argument('--v-ref', type=float, default=0.1)
    parser.add_argument('--state', type=int, default=1)
    parser.add_argument('--x-max', type=float, default=3.0)
    parser.add_argument('--y-max', type=float, default=1.5)
    parser.add_argument('--period-ms', type=int, default=10)
    parser.add_argument('--jitter', type=float, default=0.08, help='y에 더할 흔들림 진폭[m], 0=이전(완전 정적) 동작')
    parser.add_argument('--jitter-period-s', type=float, default=2.0, help='흔들림 주기[s]')
    parser.add_argument('--vref-ramp-s', type=float, default=2.0, help='v_ref가 0->목표까지 램프업되는 시간[s]')
    args = parser.parse_args()

    rclpy.init()
    node = StaticMultiPointPublisher(
        args.n_points, args.v_ref, args.state, args.x_max, args.y_max, args.period_ms,
        args.jitter, args.jitter_period_s, args.vref_ramp_s)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
