#!/usr/bin/env python3
"""테스트 하네스: /perception/avoid → /adas/target_ref (MGM 대체, 회피만).  ★실차 조향/구동★

stack_avoid의 회피 목표점을 dSPACE로 바로 보내, 라이다가 본 장애물에 따라 실제 조향/
구동이 회피 기동을 하는지 확인한다. 스테이트 머신 없음 — 테스트 전용.

판단(테스트용 최소):
  - 장애물 감지 + 목표점 있음 → state=AVOID, v_ref=target_speed, 목표점 송신 (회피 조향)
  - 장애물 감지 + 목표점 없음(narrow_gap) → v_ref=0 (통과 불가 → 정지)
  - 장애물 없음 → clear: straight_when_clear=true면 직진, 아니면 v_ref=0(정지)

★ estop 게이트 (박찬미 stack_estop): 위 판단 결과를 **최상위 우선권**으로 덮어쓴다 —
  estop=true면 v_ref=0. 상세·페일세이프 규칙은 `estop_gate.EstopGate` 참조.
  estop_gate:=false 로 끌 수 있으나 **실차에서는 켠 채로 쓸 것**.

★ 안전: 바퀴 들고(스탠드) 먼저, 물리 비상정지 준비, v_ref 낮게.
"""
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from fma_interfaces.msg import AvoidStatus, RefPoint, TargetRef

from stack_avoid.estop_gate import EstopGate

# ── dSPACE MPC 미리보기 창 (FMA_rev1.slx / MPC_Controller 차트에서 확인) ──
# Generate_Trajectory:  pointSpacing = sampleTime * abs(v_ref)
#                       requestedArc = (0:N_p) * pointSpacing
# 즉 MPC가 실제로 보는 궤적 길이 = N_p × Ts × v_ref 이고, v_ref=0.2 에서 겨우 4cm다.
# 그보다 멀리 찍은 ref 점은 지평 밖이라 조향에 반영되지 않는다(그 구간 곡률은
# quintic 시작 경계조건 = 현재 조향에 지배된다). 2026-08-07 실측에서 lookahead를
# 0.05~2.8m 로 흔들어도 조향이 무반응이었던 원인.
MPC_HORIZON_STEPS = 20      # N_p
MPC_SAMPLE_TIME_S = 0.01    # Ts (dSPACE 10ms 태스크)


class AvoidToRef(Node):

    def __init__(self):
        super().__init__('avoid_to_ref')
        self.target_speed = float(self.declare_parameter('target_speed_mps', 0.2).value)
        self.straight_x = float(self.declare_parameter('straight_x_m', 2.0).value)
        self.straight_when_clear = bool(self.declare_parameter('straight_when_clear', False).value)
        # REF_POINT_00를 회피 arc 위 lookahead 지점으로 (짧을수록 강한 조향). GPS 최근접점 방식.
        self.lookahead = float(self.declare_parameter('lookahead_m', 0.4).value)
        self.curv_gain = float(self.declare_parameter('curvature_gain', 1.0).value)  # 조향 증폭 시험용
        # lookahead를 MPC 미리보기 창 안으로 강제하는 비율. 1.0 = 창 끝, 0.5 = 창 절반.
        # 0 으로 두면 클램프 없이 lookahead_m 을 그대로 쓴다(예전 동작).
        self.preview_frac = float(self.declare_parameter('preview_frac', 0.5).value)
        # ★ lookahead 절대 상한 [m]. preview_frac 만 쓰면 속도에 비례해 lookahead 가
        #   늘어나는데, 4~6cm 를 넘으면 조향 부호가 뒤집힌다(2026-08-07 실측:
        #   v 0.2→0.6 에서 str −4.99 → −0.89 로 죽다가 반전).
        #   이유: dSPACE 는 궤적 시작 곡률을 curvature0 = tan(현재 조향)/wheelbase 로
        #   잡으므로 조향→궤적→조향이 폐루프다. 호가 미리보기보다 길면
        #   κ_sampled ≈ κ0 가 되어 어떤 조향값이든 평형점이 되고 응답이 눌러붙는다.
        #   호를 짧게 유지해 샘플이 목표점에서 클램프되면 κ_sampled = κ_target 이
        #   되어 폐루프 축퇴가 깨진다. 실측 안정 동작점 2cm 를 기본값으로.
        self.lookahead_max = float(self.declare_parameter('lookahead_max_m', 0.02).value)
        # ★ 당김 방식 파라미터 (기본 동작). 0 이하면 기존 호-lookahead 방식으로 돌아간다.
        #   목표점 y 는 유지하고 x 만 이만큼 당긴다 → κ=2y/(x²+y²) 가 커져 조향이 커지고,
        #   점이 장애물보다 앞에 놓여 더 일찍 꺾는다(충돌 여유 확보).
        self.pullback = float(self.declare_parameter('pullback_m', 1.2).value)
        # 당긴 뒤에도 이 값보다 가까이는 두지 않는다(너무 가까우면 quintic 이 무너진다).
        self.ref_x_min = float(self.declare_parameter('ref_x_min_m', 0.8).value)
        # 곡률 클램프 기준 — params.yaml 의 vehicle.min_turn_radius_m 과 같은 값.
        self.min_turn_radius = float(self.declare_parameter('min_turn_radius_m', 1.15).value)
        # 송신 ref 의 횡방향 성분(y·yaw·curvature) 부호. ★기본 +1 = 기하학적으로 올바름.
        #
        # 2026-08-07 실차에서 확인된 것:
        #   - 인지가 낸 회피 목표점 y 는 RViz 표시·실물과 일치한다(인지 프레임 정상).
        #   - 그런데 그대로 보내면 바퀴가 반대로 꺾인다.
        #   - vehicle_vector.str 은 우리 명령과 같은 부호로 돌아온다(κ +0.25 → str +5.77°).
        #     즉 텔레메트리까지는 맞고 실제 서보 구동만 반대 → 반전은 str 보고 지점보다 하류.
        #   - dSPACE MPC 소스에도 대응되는 대목이 있다:
        #       wheelSteering  = -deg2rad(Actuator_Cmd.target_angle)
        #       target_angle   = -rad2deg(outWheelSteering)
        #     MPC 내부 조향(REP-103, +가 좌)과 액추에이터 명령 사이에 부호 반전이 있다.
        #
        # -1 로 두면 바퀴는 맞게 돌지만 **기하학적으로 틀린 점**을 보내게 되어
        # CLAUDE.md §3(ref_points 는 vehicle frame)에 어긋나고, MGM 통합 시
        # 다른 스택(GPS·차선)과 규약이 어긋난다. 그래서 기본은 +1 로 둔다.
        # 서보 부호 규약이 정리되기 전까지는 실차에서 바퀴가 반대로 돌 수 있다.
        self.lat_sign = float(self.declare_parameter('lateral_sign', 1.0).value)
        self.period = float(self.declare_parameter('period_ms', 10).value) / 1000.0
        # ── estop 게이트 (박찬미 stack_estop) — 공용 모듈 ──
        # 기본 ON. 끄는 것은 스탠드 위 단독 디버깅 전용 — 실차 주행에서는 켜둘 것.
        self.gate = EstopGate(
            self,
            enabled=bool(self.declare_parameter('estop_gate', True).value),
            # 하트비트 50ms × 5 = 250ms (CLAUDE.md §5.7 MGM wrapper와 동일 기본값)
            stale_s=float(self.declare_parameter('estop_stale_s', 0.25).value))

        self.last = None
        self.pub = self.create_publisher(TargetRef, '/adas/target_ref', 1)
        self.sub = self.create_subscription(AvoidStatus, '/perception/avoid', self._on_avoid, 10)
        self.timer = self.create_timer(self.period, self.tick)
        # 라이브 튜닝: ros2 param set 으로 curv_gain·lookahead·속도 즉석 변경 (재시작 불필요)
        self.add_on_set_parameters_callback(self._on_set_params)
        self.get_logger().warn(
            f"avoid_to_ref (테스트 하네스): /perception/avoid → /adas/target_ref | "
            f"v={self.target_speed}m/s, clear시 {'직진' if self.straight_when_clear else '정지'} | "
            f"{self.gate.banner()}. ★실차 조향 — 안전 주의")

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
            elif p.name == 'lateral_sign':
                self.lat_sign = float(p.value)
            elif p.name == 'preview_frac':
                self.preview_frac = float(p.value)
            elif p.name == 'lookahead_max_m':
                self.lookahead_max = float(p.value)
            elif p.name == 'pullback_m':
                self.pullback = float(p.value)
            elif p.name == 'ref_x_min_m':
                self.ref_x_min = float(p.value)
            elif p.name == 'straight_x_m':
                self.straight_x = float(p.value)
        self.get_logger().info(
            f"param 변경 → v={self.target_speed} lookahead={self.lookahead} "
            f"curv_gain={self.curv_gain}")
        return SetParametersResult(successful=True)

    def _lookahead_in_preview(self, v_ref):
        """lookahead를 dSPACE MPC 미리보기 창 안으로 클램프한다.

        창 = N_p × Ts × v_ref (v_ref=0.2 → 4cm). 이보다 멀리 찍은 점은 MPC의
        샘플 구간 밖이라, 그 구간 곡률이 quintic 시작 경계조건(= 현재 조향)에
        지배되어 조향 명령이 사실상 "현재 유지"가 된다.
        preview_frac=0 이면 클램프하지 않는다(예전 동작, 비교용).
        """
        if self.preview_frac <= 0.0:
            return self.lookahead
        preview = MPC_HORIZON_STEPS * MPC_SAMPLE_TIME_S * abs(v_ref)
        if preview <= 1e-6:                 # v_ref=0 이면 창이 없다 — 원래 값 유지
            return self.lookahead
        # 절대 상한도 같이 건다 — 속도가 올라도 호가 길어지지 않게(폐루프 축퇴 방지).
        return min(self.lookahead, self.preview_frac * preview, self.lookahead_max)

    def _pullback_point(self, p):
        """★ 당김 방식 — 목표점의 측방(y)은 그대로 두고 전방거리(x)만 당긴다.

        κ = 2y/(x²+y²) 이므로 y 를 고정한 채 x 를 줄이면 곡률이 급격히 커진다.
        (호 위 lookahead 방식은 x·y 가 함께 줄어 κ 가 불변이라 조향이 안 커졌다.)

        효과 3가지 — 2026-08-07 모델 시뮬레이션으로 확인:
          ① 속도가 오를수록 조향이 커진다. dSPACE 미리보기(N_p·Ts·v)가 넓어질수록
             당겨진 점이 만드는 급한 quintic 을 더 많이 샘플하기 때문.
             x=1.0 기준 v 0.2/0.4/0.6 → 7.2° / 17.0° / 24.3°
          ② 크기 자체가 3~8배 커진다 (기존 방식 −2.9°/−0.9°/+0.9°)
          ③ 점이 장애물보다 앞(가까이)에 놓여 더 일찍 꺾으므로 충돌 여유가 커진다

        반환: (x, y, yaw, curvature) — 부호(lateral_sign)는 호출측에서 적용.
        """
        y = p.y
        x = max(self.ref_x_min, min(p.x, p.x - self.pullback))
        d2 = x * x + y * y
        kappa = 2.0 * y / d2
        # 물리적으로 낼 수 없는 곡률은 잘라낸다(최소회전반경). 넘겨봐야 MPC가 포화.
        k_max = 1.0 / self.min_turn_radius
        kappa = max(-k_max, min(k_max, kappa))
        yaw = 2.0 * math.atan2(y, x)
        return x, y, yaw, kappa

    def _arc_lookahead_point(self, p, v_ref):
        """기존 방식 — 목표점으로 가는 호 위 lookahead 지점. 비교용(pullback_m=0)."""
        kappa = 2.0 * p.y / (p.x * p.x + p.y * p.y)
        if abs(kappa) > 1e-4:
            s_total = 2.0 * math.atan2(p.y, p.x) / kappa
            length = min(self._lookahead_in_preview(v_ref), abs(s_total))
            th = kappa * length
            return math.sin(th) / kappa, (1.0 - math.cos(th)) / kappa, th, kappa
        length = min(self._lookahead_in_preview(v_ref), math.hypot(p.x, p.y))
        return length, 0.0, 0.0, kappa

    @staticmethod
    def _rp(x, y=0.0, yaw=0.0, curv=0.0):
        p = RefPoint()
        p.x, p.y, p.yaw, p.curvature = float(x), float(y), float(yaw), float(curv)
        return p

    def tick(self):
        m = TargetRef()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        a = self.last
        reason = None
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
                if self.pullback > 0.0:
                    lx, ly, th, kappa = self._pullback_point(p)
                else:
                    lx, ly, th, kappa = self._arc_lookahead_point(p, m.v_ref)
                # 횡방향 성분에만 부호 적용 — x(전방거리)는 그대로 둔다.
                s = self.lat_sign
                m.ref_points = [self._rp(lx, s * ly, s * th, s * kappa * self.curv_gain)]
            else:
                m.ref_points = [self._rp(self.straight_x)]
        elif a is not None and a.obstacle_detected:      # narrow_gap: 통과 불가
            m.state = TargetRef.STATE_AVOID
            m.v_ref = 0.0
            m.ref_points = [self._rp(self.straight_x)]
            reason = 'narrow_gap(하네스)'
        else:                                            # clear
            m.state = TargetRef.STATE_LANE
            m.v_ref = self.target_speed if self.straight_when_clear else 0.0
            m.ref_points = [self._rp(self.straight_x)]
            if not self.straight_when_clear:
                reason = 'clear(직진 비활성)'

        # ── estop 게이트: 위 판단을 덮어쓰는 최상위 우선권 ──
        # v_ref만 0으로. ref_points는 그대로 둔다 (§3 조향 직전 값 유지·급조향 금지).
        # 사유를 estop으로 덮어써서, ⓒ처럼 narrow_gap과 estop이 동시에 성립하는
        # 경우에도 "안전 바닥이 실제로 걸렸다"가 로그에 남게 한다.
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
