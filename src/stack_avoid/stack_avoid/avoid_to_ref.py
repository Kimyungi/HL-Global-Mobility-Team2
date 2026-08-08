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
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from fma_interfaces.msg import AvoidStatus, RefPoint, TargetRef, VehicleVector

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
SOLVE_ITERS = 20            # 역산 이분법 반복 (x 해상도 ≈ 2μm)


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
        # 인지 회피 목표점을 그대로 송신할지. ★기본 false — 초록점 직송으로는
        #   물리적으로 회피가 불가능함이 실측으로 확인됐다(2026-08-07):
        #     초록점 직송  κ 0.169 → str 1.16° → 회전반경 29m → 측방 이동 0.15m
        #     당김+역산    κ 0.870 → str 6.89° → 회전반경 4.9m → 측방 이동 0.89m ✓
        #   dSPACE 가 명령의 20~25% 만 실행하므로, 기하가 요구하는 조향을 실제로
        #   내려면 그만큼 큰 곡률을 주문해야 한다. true 로 두면 초록점 자체를 보낸다.
        self.send_as_is = bool(self.declare_parameter('send_target_as_is', False).value)
        # ★ GPS 미션과 동일한 방식으로 ref 점 생성 (2026-08-07 지시).
        #   gps-bags-20260807 실측 규약: n_points 20 · 등간격 0.321m · 첫 점 x≈1.39m
        #   · yaw = 경로 접선(다음 점 방향각과 2.41° 이내) · curvature = 경로 국소 곡률.
        #   회피는 목표점까지의 등곡률 호를 "경로"로 삼아 그 위를 같은 규약으로 샘플한다.
        #   호 위 점이므로 yaw = κ·s, curvature = κ (일정) 가 곧 경로 접선·곡률이다.
        self.gps_style = bool(self.declare_parameter('gps_style', False).value)
        self.gps_lookahead = float(self.declare_parameter('gps_lookahead_m', 1.0).value)
        self.gps_spacing = float(self.declare_parameter('gps_spacing_m', 0.321).value)
        self.gps_n = int(self.declare_parameter('gps_n_points', 20).value)
        # ★ 세로 여유 (clear_before_m) — 회피 기동의 핵심 설계값. **앞범퍼 기준**.
        #   측방 이동이 끝나는 시점에 앞범퍼와 장애물 사이에 남길 세로 거리다.
        #
        #   주의: 회피 목표점 x 는 base_link(후축 중심) 기준이고 앞범퍼는 그보다
        #   front_offset_m(0.76) 앞에 있다. 그래서 후축 기준으로 잡으면 앞범퍼
        #   여유가 그만큼 깎여, 0.76 미만 값은 "앞범퍼가 이미 장애물을 지난 뒤에야
        #   측방 이동 완료" = 충돌을 뜻하게 된다. 여기서는 앞범퍼를 기준으로 삼아
        #   파라미터 값이 곧 실제 여유가 되게 한다.
        #
        #   요구 곡률 κ = 2y / (x_clear² + y²),  x_clear = x_obs − front_offset − clear_before
        #   장애물 2.96m·측방 0.93m 기준 요구 조향:
        #       0.00 m (앞범퍼가 장애물 옆에 닿는 순간) → 10.98°
        #       0.85 m (차 한 대 길이 먼저)            → 22.38°   ← 기본값
        #   조향 크기를 바꾸려면 이 값 하나만 만지면 된다.
        self.clear_before = float(self.declare_parameter('clear_before_m', 0.85).value)
        # 후축 중심 → 앞범퍼 [m]. params.yaml 의 wheelbase + front_overhang (= lidar_mount.x_m).
        self.front_offset = float(self.declare_parameter('front_offset_m', 0.76).value)
        # 조향 지연 보상 [s]. 조향이 다 서기까지 차는 v·τ 만큼 더 간다 → 그만큼 더
        # 먼저 비켜야 한다. 속도가 오를수록 요구 조향이 커지는 물리적 근거.
        # 8/6 실측 참고: dead 0.111s / 63% 0.330s / 95% 0.451s.
        self.steer_lag = float(self.declare_parameter('steer_lag_s', 0.35).value)
        # ★ 위에서 정한 "요구 곡률"을 dSPACE 가 실제로 실행하게 만들지 여부.
        #   MPC 는 궤적의 앞 (N_p × Ts × v_ref) m 만 본다(v=0.2 → 4cm). 요구 곡률을
        #   가진 점을 그냥 보내면 그 4cm 구간의 곡률은 아직 시작 경계조건(현재 조향)
        #   근처라 실행 조향이 훨씬 작아진다. true 면 "미리보기 지점에서 요구 곡률이
        #   나오도록" 송신점 x 를 역산한다(측방 y 는 항상 목표 그대로).
        #   false 면 요구 곡률 지점을 그대로 보낸다(기하는 정직하나 실행이 약함).
        self.scale_match = bool(self.declare_parameter('scale_match', True).value)
        # 송신점을 이보다 가까이는 두지 않는다(너무 가까우면 quintic 이 무너진다).
        self.ref_x_min = float(self.declare_parameter('ref_x_min_m', 0.8).value)
        # 곡률 클램프 기준 — params.yaml 의 vehicle.min_turn_radius_m 과 같은 값.
        self.min_turn_radius = float(self.declare_parameter('min_turn_radius_m', 1.15).value)
        self.wheelbase = float(self.declare_parameter('wheelbase_m', 0.595).value)
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
        # dSPACE 는 quintic 시작 곡률을 tan(현재 조향)/wheelbase 로 잡는다.
        # 역산에 그 값이 필요하므로 상태추정을 구독한다(미수신 시 0 = 직진 가정).
        self.cur_steer = 0.0
        self.vv_sub = self.create_subscription(
            VehicleVector, '/vehicle/vector', self._on_vv, qos_profile_sensor_data)
        self.pub = self.create_publisher(TargetRef, '/adas/target_ref', 1)
        self.sub = self.create_subscription(AvoidStatus, '/perception/avoid', self._on_avoid, 10)
        self.timer = self.create_timer(self.period, self.tick)
        # 라이브 튜닝: ros2 param set 으로 clear_before·curv_gain·속도 즉석 변경
        self.add_on_set_parameters_callback(self._on_set_params)
        self.get_logger().warn(
            f"avoid_to_ref (테스트 하네스): /perception/avoid → /adas/target_ref | "
            f"v={self.target_speed}m/s, clear시 {'직진' if self.straight_when_clear else '정지'} | "
            f"{self.gate.banner()}. ★실차 조향 — 안전 주의")

    def _on_avoid(self, msg):
        self.last = msg

    def _on_vv(self, msg):
        self.cur_steer = float(msg.str)

    def _on_set_params(self, params):
        for p in params:
            if p.name == 'gps_style':
                self.gps_style = bool(p.value)
            elif p.name == 'send_target_as_is':
                self.send_as_is = bool(p.value)
            elif p.name == 'scale_match':
                self.scale_match = bool(p.value)
            elif p.name == 'curvature_gain':
                self.curv_gain = float(p.value)
            elif p.name == 'yaw_gain':
                self.yaw_gain = float(p.value)
            elif p.name == 'y_scale':
                self.y_scale = float(p.value)
            elif p.name == 'gps_lookahead_m':
                self.gps_lookahead = float(p.value)
            elif p.name == 'gps_spacing_m':
                self.gps_spacing = float(p.value)
            elif p.name == 'target_speed_mps':
                self.target_speed = float(p.value)
            elif p.name == 'lateral_sign':
                self.lat_sign = float(p.value)
            elif p.name == 'clear_before_m':
                self.clear_before = float(p.value)
            elif p.name == 'front_offset_m':
                self.front_offset = float(p.value)
            elif p.name == 'steer_lag_s':
                self.steer_lag = float(p.value)
            elif p.name == 'ref_x_min_m':
                self.ref_x_min = float(p.value)
            elif p.name == 'straight_x_m':
                self.straight_x = float(p.value)
        self.get_logger().info(
            f"param 변경 → v={self.target_speed} clear_before={self.clear_before} "
            f"curv_gain={self.curv_gain} gps_style={self.gps_style} "
            f"as_is={self.send_as_is} scale_match={self.scale_match}")
        return SetParametersResult(successful=True)

    def _quintic_kappa(self, xt, yt, kt, k0, s):
        """dSPACE Generate_Trajectory 와 동일한 quintic 의 호길이 s 지점 곡률.

        경계조건: 시작 (0,0) 헤딩0 곡률 k0(=현재 조향), 끝 (xt,yt) 헤딩·곡률 kt.
        닫힌형 Hermite 계수를 쓰고 호길이는 chord·u 로 근사한다 — 완만한 곡선에서
        정확 모델 대비 오차 0.002 이내(2026-08-07 검증). O(1) 이라 100Hz 에서 안전.
        """
        chord = math.hypot(xt, yt)
        if chord < 1e-6:
            return k0
        yaw = 2.0 * math.atan2(yt, xt)
        # 위치·속도·가속도 경계값 (dSPACE 와 같은 스케일링: v=chord·tangent, a=chord²·κ·normal)
        v0 = (chord, 0.0)
        a0 = (0.0, chord * chord * k0)
        v1 = (chord * math.cos(yaw), chord * math.sin(yaw))
        a1 = (chord * chord * kt * -math.sin(yaw), chord * chord * kt * math.cos(yaw))
        c = []
        for i, (d, s0, s1, b0, b1) in enumerate((
                (xt, v0[0], v1[0], a0[0], a1[0]), (yt, v0[1], v1[1], a0[1], a1[1]))):
            c.append((0.0, s0, b0 / 2.0,
                      10 * d - 6 * s0 - 4 * s1 - 1.5 * b0 + 0.5 * b1,
                      -15 * d + 8 * s0 + 7 * s1 + 1.5 * b0 - b1,
                      6 * d - 3 * s0 - 3 * s1 - 0.5 * b0 + 0.5 * b1))
        u = min(1.0, max(0.0, s / chord))
        d1 = [cc[1] + 2 * cc[2] * u + 3 * cc[3] * u**2 + 4 * cc[4] * u**3 + 5 * cc[5] * u**4
              for cc in c]
        d2 = [2 * cc[2] + 6 * cc[3] * u + 12 * cc[4] * u**2 + 20 * cc[5] * u**3 for cc in c]
        sp2 = max(d1[0] * d1[0] + d1[1] * d1[1], 1e-12)
        return (d1[0] * d2[1] - d1[1] * d2[0]) / sp2 ** 1.5

    def _gps_style_points(self, p):
        """GPS 미션과 동일한 규약으로 ref 점 목록 생성.

        목표점까지의 등곡률 호를 경로로 삼아, lookahead 지점부터 등간격으로
        n 개 샘플한다. 각 점의 yaw = 경로 접선(κ·s), curvature = 경로 곡률(κ).
        GPS 실측 규약(간격 0.321m · 20점 · 첫 점 x≈1.39m)과 같은 형태.
        """
        k_max = 1.0 / self.min_turn_radius
        kappa = max(-k_max, min(k_max, 2.0 * p.y / (p.x * p.x + p.y * p.y)))
        sg = self.lat_sign
        if abs(kappa) < 1e-4:                       # 거의 직진
            return [self._rp(self.gps_lookahead + i * self.gps_spacing)
                    for i in range(self.gps_n)]
        # 목표점을 넘어서도 호를 연장한다 — GPS 는 트랙이 이어져 20점이 모두 다르다.
        # 클램프하면 뒷점들이 한 자리에 뭉쳐 경로가 "멈춘 것"처럼 보인다.
        pts = []
        for i in range(self.gps_n):
            s = self.gps_lookahead + i * self.gps_spacing
            th = kappa * s                          # 경로 접선각
            pts.append(self._rp(math.sin(th) / kappa,
                                sg * (1.0 - math.cos(th)) / kappa,
                                sg * th, sg * kappa))
        return pts

    def _required_curvature(self, p, v_ref):
        """회피 기하가 요구하는 곡률 — 앞범퍼가 장애물보다 clear_before 만큼 먼저 다 비키기.

        조향 지연(v·τ) 동안 차가 더 가므로 그만큼 더 먼저 비켜야 한다.
        """
        # 앞범퍼 기준: 후축 원점 좌표에서 front_offset 을 먼저 빼야 앞범퍼 위치가 된다.
        x_clear = max(self.ref_x_min,
                      p.x - self.front_offset - self.clear_before
                      - abs(v_ref) * self.steer_lag)
        k_max = 1.0 / self.min_turn_radius
        k = 2.0 * p.y / (x_clear * x_clear + p.y * p.y)
        return x_clear, max(-k_max, min(k_max, k))

    def _scale_matched_point(self, p, v_ref):
        """요구 곡률이 MPC 미리보기 지점에서 실제로 나오도록 송신점 x 를 역산.

        측방 y 는 목표 그대로 유지하고 x 만 움직인다. scale_match=false 면
        요구 곡률 지점을 그대로 보낸다.
        """
        yt = p.y
        x_clear, k_des = self._required_curvature(p, v_ref)
        k_max = 1.0 / self.min_turn_radius
        preview = MPC_HORIZON_STEPS * MPC_SAMPLE_TIME_S * abs(v_ref)
        if not self.scale_match or preview <= 1e-6:
            return x_clear, yt, 2.0 * math.atan2(yt, x_clear), k_des
        k0 = math.tan(self.cur_steer) / self.wheelbase

        def excess(x):
            kt = max(-k_max, min(k_max, 2.0 * yt / (x * x + yt * yt)))
            return self._quintic_kappa(x, yt, kt, k0, preview) - k_des

        lo, hi = self.ref_x_min, max(self.ref_x_min + 1e-3, p.x)
        if excess(hi) >= 0.0:
            x = hi
        elif excess(lo) <= 0.0:
            x = lo
        else:
            for _ in range(SOLVE_ITERS):     # x 감소 → 지평 곡률 증가 (단조)
                mid = 0.5 * (lo + hi)
                if excess(mid) > 0.0:
                    lo = mid
                else:
                    hi = mid
            x = 0.5 * (lo + hi)
        kappa = max(-k_max, min(k_max, 2.0 * yt / (x * x + yt * yt)))
        return x, yt, 2.0 * math.atan2(yt, x), kappa

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
            # ★ 인지 회피 목표점을 그대로 dSPACE 로 보낸다 (2026-08-07 지시).
            # 위치(x, y)는 손대지 않고, 그 점으로 가는 등곡률 호의 헤딩·곡률만 채운다:
            #   κ = 2y/(x²+y²),  yaw = 2·atan2(y,x)   (dummy_ref_publisher·path_engine 규약)
            # yaw/curvature 를 0으로 두면 quintic 이 S자가 되어 초기 조향이 ≈0 이 된다.
            # dSPACE 는 REF_POINT_00(첫 점) 하나만 디코딩한다.
            p = a.points[0]
            d2 = p.x * p.x + p.y * p.y
            if d2 > 1e-6:
                if self.gps_style:
                    m.ref_points = self._gps_style_points(p)
                    self.gate.log_reason(reason)
                    self.pub.publish(m)
                    return
                if self.send_as_is:
                    k_max = 1.0 / self.min_turn_radius
                    kappa = max(-k_max, min(k_max, 2.0 * p.y / d2))
                    lx, ly, th = p.x, p.y, 2.0 * math.atan2(p.y, p.x)
                else:
                    lx, ly, th, kappa = self._scale_matched_point(p, m.v_ref)
                # 횡방향 성분에만 부호 적용 — x(전방거리)는 그대로 둔다.
                s = self.lat_sign
                # y_scale 적용 시 κ 를 축소된 점 기준으로 재계산 (자기정합)
                ly2 = ly * self.y_scale
                if abs(self.y_scale - 1.0) > 1e-9:
                    d2b = lx * lx + ly2 * ly2
                    kappa = 2.0 * ly2 / d2b if d2b > 1e-9 else 0.0
                    kmax = 1.0 / self.min_turn_radius
                    kappa = max(-kmax, min(kmax, kappa))
                    th = 2.0 * math.atan2(ly2, lx)
                m.ref_points = [self._rp(lx, s * ly2, s * th * self.yaw_gain,
                                         s * kappa * self.curv_gain)]
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
