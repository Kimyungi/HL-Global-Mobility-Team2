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

이어서 **대체 송신 경로가 되살아나지 않는지** 확인한다 (PR #27 리뷰 반영 —
gps_style 분기의 조기 return 이 게이트를 우회한 차단 버그의 회귀 방지):
  6) 제거된 스위치(ray_pull/gps_style/send_target_as_is/scale_match) set → 거부되어야 함
  7) self.pub.publish() 가 소스에 정확히 1곳(_publish) 에만 있어야 함

예전에는 경로가 4개라 경로마다 게이트를 확인했다. 2026-08-11 에 경로를 하나로 줄이면서
(MEASUREMENTS V절) "경로가 하나뿐"임을 강제하는 쪽으로 바꿨다 — 더 강한 보증이다.
"""
import time
from pathlib import Path

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

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

        self.param_cli = self.create_client(SetParameters, '/avoid_to_ref/set_parameters')

    def _on_ref(self, m):
        self.last_v = float(m.v_ref)

    def set_params(self, **kv):
        """avoid_to_ref 의 bool 파라미터를 동기 설정 → 전부 성공 여부."""
        req = SetParameters.Request()
        for name, val in kv.items():
            pp = Parameter()
            pp.name = name
            pp.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(val))
            req.parameters.append(pp)
        fut = self.param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        return bool(res) and all(r.successful for r in res.results)

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
        want = '주행' if expect_moving else '정지'
        print(f"{'✓' if ok else '✗'} {name:34s} v_ref={v}  (기대: {want})")

    settle(n, 1.0)   # 노드 연결 대기
    if not n.param_cli.wait_for_service(timeout_sec=5.0):
        print('✗ /avoid_to_ref 파라미터 서비스 없음 — 노드가 떠 있는지 확인')
        raise SystemExit(1)
    step('1) estop 미수신', None, 1.0, False)
    step('2) estop=false', False, 1.0, True)
    step('3) estop=true', True, 1.0, False)
    step('4) estop=false 복귀', False, 1.0, True)
    step('5) estop 발행 중단 (stale)', None, 1.5, False)

    # ── 제거된 대체 송신 경로가 되살아나지 않는지 ──
    # 예전에는 경로가 4개(ray_pull/gps_style/직송/역산)라 경로마다 estop 관통을
    # 확인해야 했다. 2026-08-11 에 경로를 하나로 줄이면서(팀장 리뷰 비차단 ①③,
    # MEASUREMENTS V절) 그 검사는 아래 두 가지로 대체한다 — 경로가 하나뿐임을
    # **강제**하는 편이 경로마다 게이트를 확인하는 것보다 강한 보증이다.
    #
    #   6) 제거된 스위치를 set 하면 거부되어야 한다. 조용히 무시되면 "껐다고 믿는데
    #      실제로는 기본 경로가 도는" §G 유형 사고가 된다.
    for name in ('ray_pull', 'gps_style', 'send_target_as_is', 'scale_match'):
        ok = not n.set_params(**{name: True})     # 거부(=False) 되어야 정상
        results.append((f'6) 제거된 파라미터 거부: {name}', None, True, ok))
        print(f"{'✓' if ok else '✗'} {'6) 거부: ' + name:34s} "
              f"{'거부됨' if ok else '★수용됨 — 경로가 되살아났다'}")

    #   7) 송신 출구가 하나인지 소스로 확인한다. tick() 이 직접 publish 하면 게이트를
    #      우회하게 된다 — I-9 차단 버그가 정확히 그 형태였다.
    src = Path(__file__).resolve().parents[1] / 'stack_avoid' / 'avoid_to_ref.py'
    body = src.read_text(encoding='utf-8')
    # 문장 시작이 호출인 줄만 센다 — docstring 의 "직접 부르지 말 것" 언급은 제외.
    calls = [ln for ln in body.splitlines() if ln.strip().startswith('self.pub.publish(')]
    ok = len(calls) == 1
    results.append(('7) publish 출구 단일', None, True, ok))
    print(f"{'✓' if ok else '✗'} {'7) publish 출구 단일':34s} "
          f"self.pub.publish() {len(calls)}곳 (기대: 1)")

    n.destroy_node()
    rclpy.shutdown()
    bad = [r for r in results if not r[3]]
    print('\n=== 결과: ' + ('전부 통과' if not bad else f'{len(bad)}건 실패') + ' ===')
    raise SystemExit(1 if bad else 0)


if __name__ == '__main__':
    main()
