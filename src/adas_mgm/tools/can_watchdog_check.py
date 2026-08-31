#!/usr/bin/env python3
"""§5.7 ⑥ CAN 헬스 watchdog 기능 시험 — **CAN 하드웨어 없이** 돈다.  이기돈

    ros2 run adas_mgm mgm_node --ros-args -p wait_go:=false   # 터미널 1
    python3 <this>                                            # 터미널 2

가짜 CanHealth·EstopRequest·GpsPath 를 먹여 놓고 /adas/target_ref 의 v_ref 만 본다.
브리지도 can0 도 필요 없다 — 검증 대상이 **MGM 의 반응**이기 때문이다.
버스·어댑터 쪽 실제 거동은 CAN_BRINGUP.md 8단계(실기)가 맡는다.

6단계로 v_ref 를 관찰한다:
  ① 건전            → 달린다 (v_ref > 0)
  ② 링크 다운 0.5s  → 즉시 정지, 래치 문턱(1.0s) 미만이므로 래치 안 걸림
  ③ 복구            → **스스로 재출발** (PROTOCOL.md:78 "래치 없음" 보존)
  ④ 링크 다운 2.0s  → 정지 + 래치
  ⑤ 복구            → 정지 유지 (자동 재출발 안 함)
  ⑥ /operator/go    → 재인가로 래치 해제 → 재출발

②③ 이 **PROTOCOL.md:78 "복구는 자동 · 래치 없음" 을 보존하는 구간**이고,
④⑤ 가 거기서 갈라지는 유일한 구간이다 (INTEGRATION_TEST_0826.md §6 ①).
문턱은 can_relatch_sec(기본 1.0s) — 0 으로 두면 ④⑤ 도 ②③ 처럼 자동 복귀한다.
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from fma_interfaces.msg import (CanHealth, EstopRequest, GpsPath, LanePath,
                                RefPoint, TargetRef)


class Harness(Node):
    def __init__(self):
        super().__init__('can_wd_harness')
        self.pub_can = self.create_publisher(CanHealth, '/bridge/can_health', 1)
        self.pub_estop = self.create_publisher(EstopRequest, '/perception/estop', 1)
        self.pub_lane = self.create_publisher(LanePath, '/perception/lane_path', 1)
        self.pub_gps = self.create_publisher(GpsPath, '/perception/gps_path', 1)
        self.pub_go = self.create_publisher(Bool, '/operator/go', 1)
        self.create_subscription(TargetRef, '/adas/target_ref', self.on_ref, 1)
        self.v_ref = None
        self.healthy = True
        self.create_timer(0.02, self.tick)

    def on_ref(self, m):
        self.v_ref = m.v_ref

    def tick(self):
        e = EstopRequest()
        e.header.stamp = self.get_clock().now().to_msg()
        e.estop = False
        self.pub_estop.publish(e)

        pts = []
        for i in range(20):
            p = RefPoint()
            p.x = 0.5 * (i + 1)
            pts.append(p)

        lp = LanePath()
        lp.header.stamp = e.header.stamp
        lp.confidence = 0.0        # 차선은 못 믿는 상태 → WAYPOINT 유지
        lp.points = list(pts)
        self.pub_lane.publish(lp)

        gp = GpsPath()             # WAYPOINT 가 쓸 유효 경로 (RTK FIXED)
        gp.header.stamp = e.header.stamp
        gp.points = list(pts)
        gp.fix_quality = 4
        gp.heading_source = GpsPath.HEADING_FUSED
        self.pub_gps.publish(gp)

        h = CanHealth()
        h.header.stamp = e.header.stamp
        h.link_up = self.healthy
        h.tx_ok = self.healthy
        h.consecutive_tx_fail = 0 if self.healthy else 50
        h.last_errno = 0 if self.healthy else 100
        h.down_duration_s = 0.0
        self.pub_can.publish(h)


def spin_for(node, sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def main():
    rclpy.init()
    n = Harness()
    out = []

    def mark(label):
        out.append((label, n.v_ref))
        print(f'{label:32s} v_ref = {n.v_ref}')

    def phase(label, healthy, sec):
        n.healthy = healthy
        spin_for(n, sec)
        mark(label)

    phase('① 건전 3s', True, 3.0)
    phase('② 링크다운 0.5s (래치 문턱 미만)', False, 0.5)
    phase('③ 복구 2s — 자동 재출발?', True, 2.0)
    phase('④ 링크다운 2.0s (래치 문턱 초과)', False, 2.0)
    phase('⑤ 복구 2s — 래치로 정지 유지?', True, 2.0)

    g = Bool()
    g.data = True
    for _ in range(5):
        n.pub_go.publish(g)
        spin_for(n, 0.1)
    phase('⑥ go 재인가 2s — 재출발?', True, 2.0)

    rclpy.shutdown()
    print('\n--- 판정 ---')
    if any(v is None for _, v in out):
        print('FAIL — /adas/target_ref 무수신. mgm_node 가 안 떠 있다:')
        print('  ros2 run adas_mgm mgm_node --ros-args -p wait_go:=false')
        raise SystemExit(1)
    ok = (out[0][1] > 0 and out[1][1] == 0.0 and out[2][1] > 0 and
          out[3][1] == 0.0 and out[4][1] == 0.0 and out[5][1] > 0)
    print('PASS' if ok else 'FAIL')


main()
