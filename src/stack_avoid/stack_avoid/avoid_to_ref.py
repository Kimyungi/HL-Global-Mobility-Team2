#!/usr/bin/env python3
"""테스트 하네스: /perception/avoid → /adas/target_ref (MGM 대체, 회피만).  ★실차 조향/구동★

stack_avoid의 회피 목표점을 dSPACE로 바로 보내, 라이다가 본 장애물에 따라 실제 조향/
구동이 회피 기동을 하는지 확인한다. 스테이트 머신/estop 게이팅 없음 — 테스트 전용.

판단(테스트용 최소):
  - 장애물 감지 + 목표점 있음 → state=AVOID, v_ref=target_speed, 목표점 송신 (회피 조향)
  - 장애물 감지 + 목표점 없음(narrow_gap) → v_ref=0 (통과 불가 → 정지)
  - 장애물 없음 → clear: straight_when_clear=true면 직진, 아니면 v_ref=0(정지)

★ 안전: 바퀴 들고(스탠드) 먼저, 물리 비상정지 준비, v_ref 낮게.
"""
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from fma_interfaces.msg import AvoidStatus, RefPoint, TargetRef


class AvoidToRef(Node):

    def __init__(self):
        super().__init__('avoid_to_ref')
        self.target_speed = float(self.declare_parameter('target_speed_mps', 0.2).value)
        self.straight_x = float(self.declare_parameter('straight_x_m', 2.0).value)
        self.straight_when_clear = bool(self.declare_parameter('straight_when_clear', False).value)
        # REF_POINT_00를 회피 arc 위 lookahead 지점으로 (짧을수록 강한 조향). GPS 최근접점 방식.
        self.lookahead = float(self.declare_parameter('lookahead_m', 0.4).value)
        self.curv_gain = float(self.declare_parameter('curvature_gain', 1.0).value)  # 조향 증폭 시험용
        self.period = float(self.declare_parameter('period_ms', 10).value) / 1000.0

        self.last = None
        self.pub = self.create_publisher(TargetRef, '/adas/target_ref', 1)
        self.sub = self.create_subscription(AvoidStatus, '/perception/avoid', self._on_avoid, 10)
        self.timer = self.create_timer(self.period, self.tick)
        # 라이브 튜닝: ros2 param set 으로 curv_gain·lookahead·속도 즉석 변경 (재시작 불필요)
        self.add_on_set_parameters_callback(self._on_set_params)
        self.get_logger().warn(
            f"avoid_to_ref (테스트 하네스): /perception/avoid → /adas/target_ref | "
            f"v={self.target_speed}m/s, clear시 {'직진' if self.straight_when_clear else '정지'}. "
            f"★실차 조향 — 안전 주의")

    def _on_avoid(self, msg):
        self.last = msg

    def _on_set_params(self, params):
        for p in params:
            if p.name == 'curvature_gain':
                self.curv_gain = float(p.value)
            elif p.name == 'lookahead_m':
                self.lookahead = float(p.value)
            elif p.name == 'target_speed_mps':
                self.target_speed = float(p.value)
            elif p.name == 'straight_x_m':
                self.straight_x = float(p.value)
        self.get_logger().info(
            f"param 변경 → v={self.target_speed} lookahead={self.lookahead} curv_gain={self.curv_gain}")
        return SetParametersResult(successful=True)

    @staticmethod
    def _rp(x, y=0.0, yaw=0.0, curv=0.0):
        p = RefPoint()
        p.x, p.y, p.yaw, p.curvature = float(x), float(y), float(yaw), float(curv)
        return p

    def _arc_points(self, tx, ty, n=20):
        """(0,0,헤딩0)→목표(tx,ty)를 잇는 등곡률 호를 n개 점으로 조밀화.
        GPS(path_engine)처럼 다수 점을 보내야 dSPACE가 제대로 추종(단일점은 미동작)."""
        d2 = tx * tx + ty * ty
        if d2 < 1e-6:
            return [self._rp(0.0)]
        kappa = 2.0 * ty / d2
        if abs(kappa) < 1e-4:                 # 거의 직진
            return [self._rp(tx * i / (n - 1)) for i in range(n)]
        theta = 2.0 * math.atan2(ty, tx)      # 목표에서의 헤딩
        s_total = theta / kappa               # 호 길이
        pts = []
        for i in range(n):
            th = kappa * (s_total * i / (n - 1))
            pts.append(self._rp(math.sin(th) / kappa,
                                (1.0 - math.cos(th)) / kappa, th, kappa))
        return pts

    def tick(self):
        m = TargetRef()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        a = self.last
        if a is not None and a.obstacle_detected and a.points:
            m.state = TargetRef.STATE_AVOID
            m.v_ref = float(a.v_suggest) if a.v_suggest > 1e-3 else self.target_speed
            # 회피점(x,y)로 가는 호(arc)의 곡률·헤딩을 채운다 → quintic 경계조건이
            # 명확해져 조향이 회피 방향으로 확실히 반응 (yaw=0/curv=0면 S자라 초기조향≈0).
            #   κ = 2y/(x²+y²),  yaw = 2·atan2(y,x)   (dummy_ref_publisher 규약과 동일)
            # dSPACE는 REF_POINT_00(첫 점)만 디코딩 → 그 점을 회피 arc 위 lookahead 지점으로.
            # (먼 목표점은 MPC 짧은 지평 밖이라 언더스티어 → 가까운 점이어야 강하게 조향, GPS 방식)
            # 헤딩(yaw) = 경로 접선 κ·s, curvature = 국소 arc 곡률 κ  (김윤기 path_engine과 동일 정의)
            p = a.points[0]
            d2 = p.x * p.x + p.y * p.y
            if d2 > 1e-6:
                kappa = 2.0 * p.y / d2                          # 회피 arc 곡률
                if abs(kappa) > 1e-4:
                    s_total = 2.0 * math.atan2(p.y, p.x) / kappa  # 목표까지 호길이
                    L = min(self.lookahead, abs(s_total))         # 목표를 넘지 않게 클램프
                    th = kappa * L                                # 접선 헤딩 (김윤기 방식)
                    lx, ly = math.sin(th) / kappa, (1.0 - math.cos(th)) / kappa
                else:                                             # 거의 직진
                    L = min(self.lookahead, math.hypot(p.x, p.y))
                    th, lx, ly = 0.0, L, 0.0
                m.ref_points = [self._rp(lx, ly, th, kappa * self.curv_gain)]
            else:
                m.ref_points = [self._rp(self.straight_x)]
        elif a is not None and a.obstacle_detected:      # narrow_gap: 통과 불가
            m.state = TargetRef.STATE_AVOID
            m.v_ref = 0.0
            m.ref_points = [self._rp(self.straight_x)]
        else:                                            # clear
            m.state = TargetRef.STATE_LANE
            m.v_ref = self.target_speed if self.straight_when_clear else 0.0
            m.ref_points = [self._rp(self.straight_x)]
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
