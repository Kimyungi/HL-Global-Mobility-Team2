#!/usr/bin/env python3
"""테스트 하네스: /perception/avoid → /adas/target_ref (MGM 대체, 회피만).  ★실차 조향/구동★

stack_avoid의 회피 목표점을 dSPACE로 바로 보내, 라이다가 본 장애물에 따라 실제 조향/
구동이 회피 기동을 하는지 확인한다. 스테이트 머신 없음 — 테스트 전용.

송신점(유일 경로 — 방향보존 당김): 인지 목표점(RViz 초록점) **방향은 그대로**, 원점→목표점 반직선
위에서 **거리만** `ref_lookahead_m`(1.39m = GPS 미션 실측 규약)으로 당겨 보낸다. dSPACE 는
REF_POINT_00 하나만 읽고 그 점의 방향각이 조향을 정하는데, 거리가 멀면 MPC 미리보기
(N_p×Ts×v_ref = v0.2 에서 4cm) 안이 거의 평평해 조향이 죽는다. 자세한 근거는 클래스 주석.

판단(테스트용 최소):
  - 장애물 감지 + 목표점 있음 → state=AVOID, v_ref=target_speed, 목표점 송신 (회피 조향)
  - 장애물 감지 + 목표점 없음(narrow_gap) → v_ref=0 (통과 불가 → 정지)
  - 장애물 없음 → clear: straight_when_clear=true면 직진, 아니면 v_ref=0(정지)

★ estop 게이트 (박찬미 stack_estop): 위 판단 결과를 **최상위 우선권**으로 덮어쓴다 —
  estop=true면 v_ref=0. 상세·페일세이프 규칙은 `estop_gate.EstopGate` 참조.
  estop_gate:=false 로 끌 수 있으나 **실차에서는 켠 채로 쓸 것**.

★ 안전: 바퀴 들고(스탠드) 먼저, 물리 비상정지 준비, v_ref 낮게.
"""
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from fma_interfaces.msg import AvoidStatus, RefPoint, TargetRef

from stack_avoid.estop_gate import EstopGate
from stack_avoid.ref_points import ray_points

# ── 왜 거리를 당기는가 (ref_lookahead_m 의 근거) ──
# dSPACE MPC 미리보기 창 (FMA_rev1.slx / MPC_Controller 차트):
#   Generate_Trajectory:  pointSpacing = sampleTime * abs(v_ref)
#                         requestedArc = (0:N_p) * pointSpacing
# 즉 MPC가 실제로 보는 궤적 길이 = N_p × Ts × v_ref (N_p=20, Ts=0.01) 이고,
# v_ref=0.2 에서 겨우 4cm다. 그보다 멀리 찍은 ref 점은 지평 밖이라 조향에 반영되지
# 않는다(그 구간 곡률은 quintic 시작 경계조건 = 현재 조향에 지배된다).
# 2026-08-07 실측에서 lookahead를 0.05~2.8m 로 흔들어도 조향이 무반응이었던 원인.
#
# ※ 이 상수들(N_p·Ts)을 코드가 직접 쓰던 곳은 "역산(scale_match)" 경로였고,
#   그 경로는 2026-08-11 에 제거됐다 (MEASUREMENTS V절). 지금은 근거 설명으로만 남긴다.


class AvoidToRef(Node):

    def __init__(self):
        super().__init__('avoid_to_ref')
        self.target_speed = float(self.declare_parameter('target_speed_mps', 0.2).value)
        self.straight_x = float(self.declare_parameter('straight_x_m', 2.0).value)
        self.straight_when_clear = bool(self.declare_parameter('straight_when_clear', False).value)
        self.curv_gain = float(self.declare_parameter('curvature_gain', 1.0).value)  # 조향 증폭 시험용
        # ★ 송신 yaw 배수. ★기본 0 = 목표점에서 원래 헤딩과 나란(GPS 방식).
        #   1.0 = 호 접선각(2·atan2(y,x)) — 측방 오프셋이 크면 80°가 넘는다.
        #   GPS 는 트랙 접선(≈0°)을 보내는데 우리는 60~85° 를 보낸다. dSPACE quintic 의
        #   끝점 접선이 거의 옆을 보게 되어 초기 구간이 완만해지는 것으로 의심된다.
        #   2026-08-07 GPS bag 분석: 실제 조향은 curvature 보다 atan2(y,x) 와 강상관
        #   (S자 −0.952 vs −0.394). 이 값으로 yaw 기여를 분리 측정한다.
        self.yaw_gain = float(self.declare_parameter('yaw_gain', 0.0).value)
        # ★ 진단용 — 송신 측방 y 배수. κ 도 축소된 점에 맞춰 재계산해 자기정합 유지.
        #   GPS 는 y 가 경로 이탈량(평균 0.15m)이라 작고, 우리는 측방 도약(0.7~0.9m)이다.
        #   dSPACE 응답이 y 에 어떻게 의존하는지 분리 측정하기 위한 값. 기본 1.0.
        self.y_scale = float(self.declare_parameter('y_scale', 1.0).value)
        # ★ 기본 동작 (2026-08-09) — 초록점 **방향은 보존**하고 **거리만** 팀 규약에 맞춘다.
        #   근거: dSPACE 는 REF_POINT_00 하나만 디코딩하고, 그 점의 방향각이 조향을
        #   지배한다(F-11 상관 −0.95). 그런데 같은 각도라도 거리가 조향 크기를 정한다:
        #       F-11  송신 x 0.80 · 12.4° → str 6.21°
        #       H     송신 x 3.37 · 10.8° → str 2.27°
        #   미리보기가 N_p×Ts×v_ref (v=0.2 → 4cm) 뿐이라 먼 점으로 가는 quintic 은
        #   앞 4cm 가 거의 평평하기 때문이다.
        #   GPS 미션은 첫 점을 x≈1.39m 에 두고 정상 주행한다 — MPC 는 그 lookahead 를
        #   전제로 튜닝돼 있다. 우리만 "장애물이 있는 자리"(3.4m)를 보내 규약을 벗어나 있었다.
        #   → 방향 atan2(y,x) 를 보존한 채 반직선 위에서 거리만 규약값으로 당긴다.
        #     MPC 튜닝이 아니라 **다른 스택과 같은 ref 규약으로 맞추는 것**이다.
        #   점은 매 스캔 갱신되므로, 접근할수록 같은 방향의 점이 계속 나와 측방 이동이
        #   누적된다(GPS·pure-pursuit 와 같은 거동). 한 번에 큰 y 를 주문하지 않는다.
        #
        #   ※ 예전의 `ray_pull` 파라미터(이 방식을 끄는 스위치)는 2026-08-11 에 제거했다.
        #     끄면 갈 곳이었던 대체 경로들이 전부 기각·삭제됐기 때문이다(MEASUREMENTS V절).
        # ★ 아래 3개는 GPS 미션 candump 를 직접 파싱해 얻은 실측 규약이다
        #   (~/gps_bags/run1_20260803_182800, run1_20260806_192400 — I 절).
        #   이전 세션 메모의 "첫 점 x≈1.39m" 은 **최댓값**을 규약으로 잘못 적은 것이었다.
        #   실측: 첫 점 거리 중앙 0.25m(8/3) · 0.97m(8/6), 5~95% 0.07~1.15m
        #        점 간격 중앙 0.314m(8/3) · 0.320m(8/6),  점 수 20
        self.ref_lookahead = float(self.declare_parameter('ref_lookahead_m', 0.90).value)
        # ★ 점 개수 기본 1 — J-1 통제 실험(2026-08-09)으로 확정: 기하 고정 상태에서
        #   1→20 개수만 바꿔도 str 은 소수점 둘째 자리까지 동일(−2.91°). dSPACE 는
        #   REF_POINT_00 만 쓴다(F-7 배선·I-3 상관과 일치). 20점은 효과 0 에 CAN 부하만
        #   7%→32% — PROTOCOL.md "모든 소스 1점" 합의대로 1 로 되돌림.
        #   ("1점이면 무반응" 통설은 GPS 트랙 종점 정지(v_ref→0)와의 교락이었다, I-2.)
        #   dSPACE 모델이 나중에 다점 지평을 쓰도록 바뀌면 이 파라미터로 즉시 복원.
        self.ray_n = int(self.declare_parameter('ray_n_points', 1).value)
        self.ray_spacing = float(self.declare_parameter('ray_spacing_m', 0.32).value)
        # 송신점을 이보다 가까이는 두지 않는다(너무 가까우면 quintic 이 무너진다).
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
        # 라이브 튜닝: ros2 param set 으로 lookahead·curv_gain·속도 즉석 변경
        self.add_on_set_parameters_callback(self._on_set_params)
        self.get_logger().warn(
            f"avoid_to_ref (테스트 하네스): /perception/avoid → /adas/target_ref | "
            f"v={self.target_speed}m/s, clear시 {'직진' if self.straight_when_clear else '정지'} | "
            f"송신점=방향보존 당김 {self.ref_lookahead:.2f}m × {self.ray_n}점 "
            f"@{self.ray_spacing:.2f}m | {self.gate.banner()}. ★실차 조향 — 안전 주의")

    def _on_avoid(self, msg):
        self.last = msg

    # 라이브 변경 지원 파라미터 → (속성명, 캐스터). ★여기 없는 자체 파라미터의 set 은
    # 거부한다 — "성공 응답이 오는데 실제로는 무효"였던 사고(MEASUREMENTS §G, 8/7 비교
    # 무효)의 재발 방지 (PR #27 리뷰 반영). estop_gate·estop_stale_s·min_turn_radius_m·
    # period_ms 는 초기화 시점에만 쓰이므로 의도적으로 제외 = 거부 대상.
    LIVE_PARAMS = {
        'ref_lookahead_m': ('ref_lookahead', float),
        'ray_n_points': ('ray_n', int),
        'ray_spacing_m': ('ray_spacing', float),
        'curvature_gain': ('curv_gain', float),
        'yaw_gain': ('yaw_gain', float),
        'y_scale': ('y_scale', float),
        'target_speed_mps': ('target_speed', float),
        'lateral_sign': ('lat_sign', float),
        'ref_x_min_m': ('ref_x_min', float),
        'straight_x_m': ('straight_x', float),
        'straight_when_clear': ('straight_when_clear', bool),
    }

    def _on_set_params(self, params):
        # ★ 검증을 **전부 끝낸 뒤에** 일괄 적용한다 (팀장 리뷰 2026-08-10 ③).
        #   예전엔 루프 안에서 즉시 setattr 하고 뒤쪽 미지원 값에서 successful=False 를
        #   반환했다. `ros2 param set` 으로 여러 개를 한 번에 넘기면 "거부 응답인데
        #   앞쪽 값은 이미 적용됨" 이 되어, bag 만 봐서는 왜 결과가 달라졌는지 알 수
        #   없다. §G("성공 응답인데 무효")의 정반대 사고다. ROS 파라미터 콜백은
        #   all-or-nothing 이어야 한다.
        pending = []
        for p in params:
            if p.name == 'use_sim_time':          # rclpy 자동 선언 — 통과
                continue
            if p.name not in self.LIVE_PARAMS:
                return SetParametersResult(
                    successful=False,
                    reason=f'{p.name}: 라이브 변경 미지원 — 노드 재시작 필요 '
                           f'(이번 요청의 다른 값도 적용하지 않았다)')
            attr, cast = self.LIVE_PARAMS[p.name]
            try:
                pending.append((attr, cast(p.value)))
            except (TypeError, ValueError) as e:
                return SetParametersResult(
                    successful=False,
                    reason=f'{p.name}: 값 변환 실패 ({e}) — 아무것도 적용하지 않았다')
        for attr, value in pending:               # 여기서부터는 실패하지 않는다
            setattr(self, attr, value)
        self.get_logger().info(
            f"param 변경 → v={self.target_speed} lookahead={self.ref_lookahead} "
            f"n={self.ray_n} spacing={self.ray_spacing} curv_gain={self.curv_gain} "
            f"yaw_gain={self.yaw_gain} y_scale={self.y_scale}")
        return SetParametersResult(successful=True)

    def _ray_points(self, p):
        """초록점 방향 보존 반직선 위 ref 점 — 계산은 공용 `ref_points` 모듈.

        ★ 2026-08-10: step_injector 와 계산이 갈라져 있던 것을 공용 모듈로 합쳤다
          (팀장 리뷰 ⑦). 여기서는 파라미터만 넘긴다. 반환값은 부호(lat_sign)와
          진단 게인(yaw_gain·curv_gain·y_scale)까지 **이미 적용된** 최종 튜플이다 —
          호출부에서 게인을 또 곱하지 말 것.
          실 bag 초록점 × 파라미터 10조합 6440건에서 기존 계산과 완전 일치 확인.
        """
        return ray_points(
            p.x, p.y,
            lookahead_m=self.ref_lookahead, n_points=self.ray_n,
            spacing_m=self.ray_spacing, min_turn_radius_m=self.min_turn_radius,
            ref_x_min_m=self.ref_x_min, lat_sign=self.lat_sign,
            yaw_gain=self.yaw_gain, curv_gain=self.curv_gain, y_scale=self.y_scale)

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
            # ★ 인지 회피 목표점의 **방향**을 dSPACE 로 보낸다.
            # 거리만 팀 ref 규약(ref_lookahead_m)으로 당기고, 그 점으로 가는 등곡률 호의
            # 헤딩·곡률을 채운다:  κ = 2y/(x²+y²),  yaw = 2·atan2(y,x) × yaw_gain
            #   (dummy_ref_publisher·path_engine 규약. yaw_gain 기본 0 = GPS 와 같은 경로 접선)
            # dSPACE 는 REF_POINT_00(첫 점) 하나만 디코딩한다.
            #
            # ★ 송신 경로는 **이것 하나뿐이다** (2026-08-11, 팀장 리뷰 비차단 ①③).
            #   예전에는 gps_style(20점 호 샘플)·send_target_as_is(초록점 직송)·
            #   역산(scale_match) 세 갈래가 더 있었다. G~J 절 실측으로 전부 기각됐고
            #   (직송·gps_style 은 측방 0.15m 로 회피 불성립, 역산은 방향이 변함),
            #   경로가 여러 개면 estop 게이트를 빼먹는 구조적 위험만 남았다(I-9).
            #   기각 근거와 수치는 MEASUREMENTS V절, 코드는 git 이력에 있다.
            p = a.points[0]
            d2 = p.x * p.x + p.y * p.y
            if d2 > 1e-6:
                # 공용 ref_points 모듈이 부호·게인까지 **적용해서** 돌려준다.
                # 여기서 게인을 또 곱하지 말 것.
                m.ref_points = [self._rp(*r) for r in self._ray_points(p)]
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
