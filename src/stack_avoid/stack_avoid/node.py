"""stack_avoid — 장애물 인지, 회피 가능 판정 재료(TTC·측방), 회피 경로
담당: 이기돈

- 전방 2D LiDAR 스캔(`/scan`)을 구독해 전방 통로(corridor) 안 최근접 장애물 거리·TTC 산출.
- 장애물 감지 시 회피 목표점 1개를 follow-the-gap(양쪽 장애물 고려)으로 생성 → `points[]`.
  전방 FOV의 열림(gap)들 중 통과 가능·최소 편차 열림 중심으로 조준.
  (avoid n_points=1: dSPACE quintic이 현재 자세에서 이 목표점으로 궤적을 채움)
- 모든 차량/센서/튜닝 값은 `config/params.yaml`에서 로드(하드코딩 금지, CLAUDE.md §5).
- LiDAR 장착 오프셋으로 vehicle frame(base_link=후축 중심) 보정 + static TF 발행.
- narrow_gap: offset_max 안에 통과 가능한 열림이 없으면 True (감속 근거).
- ttc 자차속도: dSPACE VehicleVector.v(신선) 우선, 미수신/오래되면 target_speed 폴백.
- avoidable/maneuver_done/v_suggest: 2026-08-12 MGM 통합 시 구현 (팀장).
  ★ 이기돈 검증 필요 — 특히 maneuver_done 클리어런스 시간은 실차 회피 주행으로
  확인할 것. v_suggest는 감속 금지(조향 하한 0.5 m/s, 이기돈 실측) — on_scan 주석 참조.

설계 메모:
- 앞 LiDAR로 반응형 회피 (맵 생성 없음, REQUIREMENTS §계약).
- 스캔이 들어올 때마다(≈100ms) AvoidStatus 발행 → MGM은 최신 스냅샷을 pull.
- ttc는 장애물 없으면 반드시 큰 값(1e9). 정지/모드 결정은 이 스택 금지 → MGM 몫.
- 이 노드는 MGM 10ms 루프와 별도 프로세스 (CLAUDE.md §5.2).
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from fma_interfaces.msg import AvoidStatus, RefPoint, VehicleVector

TTC_INF = 1.0e9   # 장애물 없을 때 ttc (0 금지 — MGM이 즉시 정지 바닥을 밟음)
EPS_SPEED = 1e-3  # 이보다 느리면 정지 상태로 보고 ttc=INF
VV_FRESH_S = 0.2  # VehicleVector 신선도 [s] — 이보다 오래되면 목표속도로 폴백


def wrap_to_pi(a: float) -> float:
    """각도를 (-pi, pi]로 감싼다."""
    return math.atan2(math.sin(a), math.cos(a))


def euler_to_quat(roll: float, pitch: float, yaw: float):
    """ZYX 오일러(rad) → 쿼터니언 (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
        cr * cp * cy + sr * sp * sy,   # w
    )


class StackAvoidNode(Node):

    def __init__(self):
        super().__init__('stack_avoid_node')

        # ── 파라미터 로드 (config/params.yaml, 미지정 시 기본값) ──────────
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.target_speed = self.declare_parameter('target_speed_mps', 0.5).value

        # 차량 제원 (통로 폭 계산에 차폭, maneuver_done 클리어런스에 전장 사용)
        self.vehicle_width = self.declare_parameter('vehicle.width_m', 0.62).value
        self.vehicle_len = self.declare_parameter('vehicle.length_m', 0.85).value

        # LiDAR 장착 (원점=후축 중심 기준)
        self.lidar_x = self.declare_parameter('lidar_mount.x_m', 0.76).value
        self.lidar_y = self.declare_parameter('lidar_mount.y_m', 0.0).value
        self.lidar_z = self.declare_parameter('lidar_mount.z_m', 0.065).value
        self.lidar_yaw = self.declare_parameter('lidar_mount.yaw_deg', 0.0).value
        self.lidar_roll = self.declare_parameter('lidar_mount.roll_deg', 0.0).value
        self.lidar_pitch = self.declare_parameter('lidar_mount.pitch_deg', 0.0).value
        # 차량 전방을 가리키는 스캔 각도(드라이버 프레임 관례). 필드검증 확정 = 270°
        # (FOV = raw 180°~360°). ★코드 고정: read_only → 런타임(param set) 변경 불가.
        #  값을 바꾸려면 이 기본값 또는 params.yaml 수정 후 재빌드해야 한다.
        self.front_center = math.radians(
            self.declare_parameter(
                'lidar_mount.forward_angle_deg', 270.0,
                ParameterDescriptor(
                    read_only=True,
                    description='전방=raw270°(FOV raw180~360). 코드 고정, 런타임 변경 불가')).value)

        # 회피 판단 (튜닝 글로벌)
        self.roi_angle = self.declare_parameter('avoid.roi_angle_deg', 180.0).value
        self.ttc_stop = self.declare_parameter('avoid.ttc_stop_s', 1.5).value
        self.lateral_margin = self.declare_parameter('avoid.lateral_margin_m', 0.15).value
        self.detect_range = self.declare_parameter('avoid.detect_range_m', 3.0).value
        self.max_range = self.declare_parameter('avoid.max_range_m', 12.0).value
        # 회피 목표점 측방 오프셋 상한
        self.offset_max = self.declare_parameter('avoid.offset_max_m', 1.0).value
        # follow-the-gap: 장애물 전방거리 ±이 값 안의 blocker를 양쪽 고려
        self.depth_band = self.declare_parameter('avoid.depth_band_m', 0.6).value
        # ★ 목표 y 변화율 상한 [m/s]. 0 이면 제한 없음(구동작). 상세는 _rate_limit 주석.
        #   3.0 = 10Hz 스캔에서 프레임당 0.30m. 2개 bag 재생으로 정한 값 —
        #   1m 초과 점프를 24·32회 → 2·0회로 줄이면서 narrow_gap 은 늘지 않았다.
        self.target_rate_mps = float(
            self.declare_parameter('avoid.target_rate_limit_mps', 3.0).value)
        # dt 이상치일 때 대체용 스캔 주기 [Hz] (T-mini Plus 실측 10Hz)
        self.scan_rate_hz = float(self.declare_parameter('avoid.scan_rate_hz', 10.0).value)
        # maneuver_done 클리어런스 여유 [m] — 완료 판정 통과거리 = 마지막 감지거리
        # + 전장 + 이 값. 실차 튜닝: ros2 param set /stack_avoid_node avoid.clear_margin_m
        self.clear_margin = float(self.declare_parameter('avoid.clear_margin_m', 0.3).value)
        # 클리어런스 계산에 쓰는 "마지막 감지거리"의 상한 [m].
        # 2026-08-13에 1.0m 하드코딩으로 넣었다가 2026-08-14 실차에서 **측면 충돌**을
        # 유발했다: 감지는 3.0m에서 시작해 1.1초 뒤(≈2.45m 지점) 소실됐는데, 상한
        # 1.0m 때문에 통과거리를 2.15m로 잡아 4.3s 만에 done → GPS 복귀로 트랙에
        # 되돌아가는 순간 콘이 0.8m 앞에 재출현(ttc 0.1)했다. 필요한 통과거리는
        # 2.45+0.85+0.3 = 3.6m(7.2s)였으므로 1.45m 부족했다.
        # 감지거리는 본래 detect_range로 유계이므로 기본값을 그에 맞춘다.
        # 과대 대기로 인한 횡오차 누적은 아래 "통과 유지점"이 막는다.
        self.clear_gap_max = float(
            self.declare_parameter('avoid.clear_gap_max_m', 3.0).value)
        # obstacle_detected 해제 히스테리시스 [m] — 진입은 detect_range, 해제는
        # detect_range + 이 값. 경계에 걸친 물체(벽·연석)가 깜빡이면 그때마다
        # 클리어런스 타이머가 리셋돼 maneuver_done이 영원히 안 선다
        # (2026-08-15 run_0815_143039: 20초에 28회 토글, AVOID 15초+ 지속,
        #  그동안 직진 유지점만 나가 횡오차 5.3m까지 발산).
        self.detect_hysteresis = float(
            self.declare_parameter('avoid.detect_hysteresis_m', 0.4).value)
        self._detected_prev = False     # 히스테리시스 상태
        self._prev_center = None        # 직전 목표 y (rate limit 상태)
        self._prev_center_t = None

        # maneuver_done 내부 상태 (REQUIREMENTS §계약 — "스캔에 안 보임 = 완료" 금지.
        # 최근 장애물의 짧은 유지 같은 내부 상태는 허용된 정상 구현)
        self._maneuver_armed = False    # detected+avoidable을 낸 적 있음 → 완료 판정 대상
        self._clear_since = None        # 전방 통로 무감지가 시작된 시각 [monotonic]
        self._last_gap = None           # 마지막 감지 거리 [m] — 클리어런스 시간 계산용
        self._done_until = 0.0          # maneuver_done=True 펄스 만료 시각 [monotonic]

        self._recompute_derived()

        # ── I/O ──────────────────────────────────────────────────────────
        # LaserScan은 Best Effort(sensor data QoS) — 구독도 맞춰야 수신됨.
        self.sub = self.create_subscription(
            LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)
        # 자차속도(TTC 입력): dSPACE 상태추정 VehicleVector.v 구독. Best Effort QoS.
        self.ego_v = None          # 최근 수신 속도 [m/s] (None = 미수신)
        self.ego_v_stamp = None    # 최근 수신 시각 (신선도 판정)
        self.vv_sub = self.create_subscription(
            VehicleVector, '/vehicle/vector', self.on_vehicle_vector,
            qos_profile_sensor_data)
        self.pub = self.create_publisher(AvoidStatus, '/perception/avoid', 1)
        # 디버그: 전방 FOV만 남긴 스캔 (뒤쪽은 inf → RViz 미표기). forward_angle 검증용.
        self.front_scan_pub = self.create_publisher(
            LaserScan, '/scan_front', qos_profile_sensor_data)

        # base_link → laser_frame static TF (실측 오프셋 단일 소스 = params.yaml)
        self.tf_static = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        # ros2 param set 으로 값 바꾸면 즉시 반영 (실주행 튜닝)
        self.add_on_set_parameters_callback(self._on_set_params)

        self.get_logger().info(
            f"stack_avoid: '{self.scan_topic}' 구독 | 차폭 {self.vehicle_width}m, "
            f"LiDAR(x={self.lidar_x},y={self.lidar_y},z={self.lidar_z}) | "
            f"통로반폭 {self.corridor_half_width:.2f}m, 전방FOV {self.roi_angle}deg, "
            f"forward={math.degrees(self.front_center):.0f}deg, "
            f"detect<{self.detect_range}m, ttc_stop {self.ttc_stop}s, v={self.target_speed}m/s")

    def _recompute_derived(self):
        """실측/튜닝값에서 파생되는 내부값 갱신."""
        # 내 경로 위 장애물로 볼 좌우 반폭 = 차폭/2 + 측방 여유
        self.corridor_half_width = self.vehicle_width / 2.0 + self.lateral_margin
        # roi_angle_deg = 전방 FOV 전체 각도 (180 = 앞쪽 180°, ±90°)
        self.front_half_angle = math.radians(self.roi_angle / 2.0)

    def _publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'
        t.transform.translation.x = float(self.lidar_x)
        t.transform.translation.y = float(self.lidar_y)
        t.transform.translation.z = float(self.lidar_z)
        # laser_frame = 드라이버 프레임. 스캔각 0 이 향하는 방향이 차량 전방(+x)과
        # forward_angle 만큼 어긋나 있다(노드 로직: rel = raw - front_center). RViz가
        # 스캔을 로직과 같은 방향으로 그리려면 TF yaw = -front_center 로 그 오프셋을
        # 반영해야 한다(예전엔 물리 yaw=0 만 써서 스캔이 90° 틀어져 보였음).
        # lidar_yaw 는 그 위의 물리 미세보정으로 가산.
        yaw_rad = math.radians(self.lidar_yaw) - self.front_center
        qx, qy, qz, qw = euler_to_quat(
            math.radians(self.lidar_roll),
            math.radians(self.lidar_pitch),
            yaw_rad)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_static.sendTransform(t)

    def _on_set_params(self, params):
        """런타임 파라미터 변경 반영. scan_topic·장착값은 재시작 권장."""
        for p in params:
            if p.name == 'target_speed_mps':
                self.target_speed = p.value
            elif p.name == 'avoid.roi_angle_deg':
                self.roi_angle = p.value
            elif p.name == 'avoid.ttc_stop_s':
                self.ttc_stop = p.value
            elif p.name == 'avoid.lateral_margin_m':
                self.lateral_margin = p.value
            elif p.name == 'avoid.detect_range_m':
                self.detect_range = p.value
            elif p.name == 'avoid.max_range_m':
                self.max_range = p.value
            elif p.name == 'avoid.offset_max_m':
                self.offset_max = p.value
            elif p.name == 'avoid.depth_band_m':
                self.depth_band = p.value
            elif p.name == 'avoid.target_rate_limit_mps':
                self.target_rate_mps = float(p.value)
            elif p.name == 'avoid.clear_margin_m':
                self.clear_margin = float(p.value)
            elif p.name == 'avoid.clear_gap_max_m':
                self.clear_gap_max = float(p.value)
            elif p.name == 'vehicle.width_m':
                self.vehicle_width = p.value
            elif p.name == 'lidar_mount.forward_angle_deg':
                self.front_center = math.radians(p.value)
        self._recompute_derived()
        self._publish_static_tf()   # forward_angle 변경 시 TF도 갱신(스캔 방향 일치)
        self.get_logger().info(
            f"param 변경 → 통로반폭 {self.corridor_half_width:.2f}m, 전방FOV {self.roi_angle}deg, "
            f"forward={math.degrees(self.front_center):.0f}deg, v={self.target_speed}m/s")
        return SetParametersResult(successful=True)

    def on_vehicle_vector(self, vv: VehicleVector):
        """dSPACE 상태추정 수신 → 자차속도 갱신 (TTC 계산 입력). 후진도 대비해 절대값."""
        self.ego_v = abs(float(vv.v))
        self.ego_v_stamp = self.get_clock().now()

    def _ego_speed(self) -> float:
        """TTC용 자차속도 [m/s]. VehicleVector.v가 신선하면 그 값, 아니면 target_speed 폴백.

        dSPACE 미연결(단독 테스트)·통신 끊김 시에도 TTC가 죽지 않도록 목표속도로 근사.
        더 빠른 쪽을 쓰면 TTC가 작아져(보수적) 안전하나, 여기선 실측 우선·미가용 시 근사."""
        if self.ego_v is not None and self.ego_v_stamp is not None:
            age = (self.get_clock().now() - self.ego_v_stamp).nanoseconds * 1e-9
            if age <= VV_FRESH_S:
                return self.ego_v
        return self.target_speed

    def on_scan(self, scan: LaserScan):
        """전방 통로 안 최근접 장애물 → 거리·TTC + 회피 목표점 발행."""
        self.front_scan_pub.publish(self._front_only_scan(scan))  # 시각화용
        obs = self._nearest_front_obstacle(scan)   # (gap, y_veh) or None
        gap = obs[0] if obs is not None else None

        msg = AvoidStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'          # points[]는 vehicle frame(후축 원점)

        # 히스테리시스: 진입 detect_range, 해제 detect_range + detect_hysteresis.
        # 경계에 걸친 물체가 깜빡이면 클리어런스 타이머가 매번 리셋된다(위 주석).
        if gap is None:
            self._detected_prev = False
        else:
            thr = (self.detect_range + self.detect_hysteresis
                   if self._detected_prev else self.detect_range)
            self._detected_prev = gap < thr
        msg.obstacle_detected = self._detected_prev

        # TTC = 거리 ÷ 자차속도 (정적 장애물 가정, REQUIREMENTS). 정지 중이면 INF.
        # 자차속도는 dSPACE VehicleVector.v(신선) 우선, 미수신/오래되면 target_speed 폴백.
        speed = self._ego_speed()
        if gap is None or speed <= EPS_SPEED:
            msg.ttc = TTC_INF
        else:
            msg.ttc = float(gap / speed)

        # 회피 목표점 1개(follow-the-gap, 양쪽 고려), vehicle frame — 감지 시에만.
        # offset_max 안에 통과 가능한 열림이 없으면 None (여유 미달 지점을 억지로
        # 내지 않음 — 팀장 리뷰 반영). 이 경우 narrow_gap 으로 감속 근거만 제공.
        if msg.obstacle_detected:
            tgt = self._gap_target(scan, gap)
        else:
            tgt = None
            self._prev_center = None        # 장애물 없음 → rate limit 이력 리셋
        msg.points = [tgt] if tgt is not None else []

        # avoidable (stage2, 2026-08-12 MGM 통합): 측방 여유 확보(목표점 성립) AND
        # TTC 여유. 판정 "재료"만 — lane/waypoint→avoid 전이 결정은 MGM (§계약).
        msg.avoidable = tgt is not None and msg.ttc >= self.ttc_stop
        # 감지했으나 offset_max 안에 통과 가능한 열림이 없음 = 통로 좁음 → 감속 근거.
        msg.narrow_gap = msg.obstacle_detected and tgt is None

        # maneuver_done (stage4, 2026-08-12 MGM 통합) — "스캔에 안 보임 = 완료" 금지
        # (REQUIREMENTS: 지나치는 중엔 전방 시야에서 사라져도 아직 차 옆에 있다).
        # 완료 = [마지막 감지 거리 + 전장 + 여유]를 현재 속도로 통과하는 시간 동안
        # 전방 무감지가 유지된 시점. 속도는 _ego_speed()(VehicleVector 미연결 시
        # target_speed 폴백)라 시간 기반이 곧 거리 기반이다. 정지 중(속도≈0)엔
        # 지나가지 못하므로 완료가 서지 않는 것이 올바른 동작.
        # ★ 이기돈 검증 필요: 여유 0.3m·펄스 0.5s는 초기값 — 실차 회피로 확인.
        now_mono = time.monotonic()
        if msg.obstacle_detected:
            self._last_gap = gap
            self._clear_since = None
            self._done_until = 0.0          # 새 장애물 → 이전 완료 펄스 무효
            if msg.avoidable:
                self._maneuver_armed = True  # MGM이 AVOID로 들어갔을 수 있음
        elif self._maneuver_armed:
            if self._clear_since is None:
                self._clear_since = now_mono
            # 감지 소실은 대부분 회피 조향으로 장애물이 통로를 **측방** 이탈한
            # 것이지 통과한 것이 아니다 — 그 시점의 종방향 거리를 그대로 써야
            # "차 뒤로 보낼" 거리가 나온다. 상한을 1.0m로 조이면 조기 복귀로
            # 측면을 스친다 (2026-08-14 실차 2회, clear_gap_max_m 주석 참조).
            clear_dist = (min(self._last_gap or self.clear_gap_max, self.clear_gap_max)
                          + self.vehicle_len + self.clear_margin)
            if speed > EPS_SPEED and \
                    now_mono - self._clear_since >= clear_dist / speed:
                self._maneuver_armed = False
                self._done_until = now_mono + 0.5   # MGM(10ms 루프)이 소비할 펄스
            # 통과 유지점 — 대기 중 빈 경로를 내면 MGM 조립이 직전 목표를 감쇠
            # hold하다 원점 부근의 퇴화 ref가 되어 조향이 표류한다 (2026-08-13
            # run_003037 실측: ref가 (0.05,-0.2)까지 수축 → 복귀 전 이탈 1m).
            # 전방 직진점(현 헤딩 유지)으로 통과 구간을 안정화한다.
            if tgt is None:
                msg.points = [self._rp(1.5, 0.0)]
        msg.maneuver_done = (not msg.obstacle_detected) and now_mono < self._done_until

        # v_suggest (stage3, 2026-08-12 MGM 통합): 목표속도 그대로 — **회피 중 감속 금지**.
        # 근거 (이기돈 실측): dSPACE 조향은 v_ref ≥ 0.5 m/s 이상이어야 제대로 반응.
        # 기하 비례 감속(초기 구현, 계수 0.4)은 v_ref를 0.44까지 내려 조향 하한을 깨서
        # 회피 자체를 무너뜨렸다 (run_0812_234253 — 직진 후 estop). 감속하면 조향이
        # 죽는 구조라 회피 중 종방향 안전은 TTC 즉시정지 바닥(MGM)·estop이 담당하고,
        # narrow 시 v_narrow 상한도 MGM 우선권 표 몫.
        msg.v_suggest = float(self.target_speed)

        self.pub.publish(msg)

        # 튜닝 보조 로그(비권위적): ttc_stop 임계 확인용. 정지 결정은 MGM.
        if msg.obstacle_detected and msg.ttc < self.ttc_stop:
            self.get_logger().debug(
                f"ttc {msg.ttc:.2f}s < ttc_stop {self.ttc_stop}s (gap {gap:.2f}m)")

    # 목표가 표면보다 이만큼 이상 멀면 "뒤"로 본다 [m]. 측정 잡음·클러스터 두께 흡수.
    BEHIND_TOL_M = 0.10
    # 목표 방위각 주변 이 각도 안의 측정치를 본다 [deg]. 3m 에서 ±0.10m 에 해당.
    BEHIND_WIN_DEG = 2.0

    def _behind_surface(self, scan, tx, ty):
        """목표 (tx,ty) 가 **스캔된 표면보다 뒤**에 있는가 (vehicle frame 입력).

        ★ 왜 이 판정인가 (2026-08-10 실차 규명). `_gap_target` 은 장면을 "한 깊이
          슬래브에 늘어선 점들의 1차원 줄"로 모델링한다. 실제 장면은 2차원이라
          **대각선 벽은 어느 깊이로 잘라도 그 단면에 '끝'이 생기고**, 알고리즘은 그
          가짜 끝을 돌아갈 수 있는 모서리로 착각해 벽 반대편에 목표를 찍는다.
          실측(avoid_20260809_220343 t=9.89s): 왼쪽 벽이 x 1.5→4.0 에서 y +1.4→+0.2 로
          기우는 하나의 연속 벽인데 슬래브 3.02~4.22 에서는 y +0.38~+0.71 로만 보여,
          그 "끝" 바깥 +1.17 을 목표로 냈다. 15프레임 재현.

          스캔은 방위각의 함수 r(θ) 다. "장애물 뒤"의 정확한 정의는 **목표의 방위각에서
          목표까지의 거리가 측정 거리보다 먼 것** — 즉 스캔된 표면을 뚫고 들어간 것이다.
          위 프레임: 목표 거리 3.09m vs 그 방위각 측정 2.18m → 뒤 ✓

        ★ 처음에는 "경로 수직거리" 로 검사했는데 **너무 엄격해 실주행이 막혔다**
          (2026-08-10 실차: 230/260 프레임 기각, 23초 정지). 통로를 따라 정상 주행할 때
          벽은 항상 clear 언저리를 스치므로, 수직거리 기준은 "벽 옆을 지나감" 과
          "벽을 뚫고 감" 을 구분하지 못한다. 같은 프레임에서 이 표면 판정은
          목표 2.57m vs 측정 3.95m → 통과로 올바르게 판정한다.
        """
        xl, yl = tx - self.lidar_x, ty - self.lidar_y      # 라이다 프레임으로
        tr = math.hypot(xl, yl)
        if tr < 1e-6:
            return False
        tb = math.atan2(yl, xl)
        win = math.radians(self.BEHIND_WIN_DEG)
        nearest = None
        angle = scan.angle_min
        for r in scan.ranges:
            rel = wrap_to_pi(angle - self.front_center)
            angle += scan.angle_increment
            if not math.isfinite(r) or r < scan.range_min:
                continue
            if r > min(scan.range_max, self.max_range):
                continue
            if abs(wrap_to_pi(rel - tb)) > win:
                continue
            if nearest is None or r < nearest:
                nearest = r
        return nearest is not None and tr > nearest + self.BEHIND_TOL_M

    def _gap_target(self, scan: LaserScan, gap: float):
        """follow-the-gap (양쪽 고려) — 회피 목표점 1개, vehicle frame(후축 원점).

        장애물 전방거리(obs_x) 깊이 밴드 안의 blocker들(양쪽)을 모아, 통과 가능한
        열림(사이/바깥)들 중 직진에서 가장 덜 벗어나는 열림 중심으로 목표점을 낸다.
        - 사이가 통과 최소폭(차폭+2·여유) 이상이면 그 사이로,
        - 아니면 바깥으로 돌아감. 없거나 벗어나면 None.
        dSPACE quintic이 현재 자세→이 점으로 궤적 복원. (avoid n_points=1)"""
        obs_x = self.lidar_x + gap
        lo, hi = obs_x - self.depth_band, obs_x + self.depth_band
        pass_w = self.vehicle_width + 2.0 * self.lateral_margin   # 통과 최소폭
        clear = self.vehicle_width / 2.0 + self.lateral_margin    # 편측 여유

        # 깊이 밴드 내 blocker들의 측방 y (vehicle frame)
        ys = []
        angle = scan.angle_min
        for r in scan.ranges:
            rel = wrap_to_pi(angle - self.front_center)
            angle += scan.angle_increment
            if abs(rel) > self.front_half_angle:
                continue
            # ★ blocker 범위는 **거리(r)가 아니라 기하**로 자른다 (2026-08-09 실차 규명).
            #
            #   detect_range(3.0)로 자르면: 깊이 밴드가 obs_x±depth_band 라 4.4m 까지
            #   뻗는데 3.0m 초과 점이 통째로 빠져 **없는 빈틈이 생긴다.**
            #   실측(avoid_20260809_214356 t=11.24s): 밴드 안 44점 중 33점이 빠져
            #   y=+0.09 에 가짜 열림이 생겼고 목표점이 장애물 0.10m 앞에 찍혔다.
            #
            #   그렇다고 max_range(12m)로 풀면 반대로 망가진다: FOV 가 ±90° 라 거의
            #   옆(rel≈89°)의 먼 벽이 x_v 만 밴드에 걸려 y=+6.98 로 들어오고, 그것이
            #   ys[-1] 이 되어 좌측 바깥 후보를 +0.12 → +7.44 로 밀어낸다. 후보가
            #   전멸해 **출발부터 narrow_gap** 이 됐다(2026-08-09 지상 시험 2회 정지).
            #
            #   올바른 경계는 **차가 실제로 갈 수 있는 가로 범위**다:
            #       |y| <= offset_max + clear
            #   목표 중심은 |y| <= offset_max 까지만 갈 수 있고 차 반쪽이 clear 안에
            #   들어오므로, 이 밖의 점은 어떤 후보로도 부딪힐 수 없다(안전하게 무시 가능).
            #   x 범위는 깊이 밴드가, y 범위는 이 식이 건다. 거리 컷은 쓰지 않는다.
            if (not math.isfinite(r) or r < scan.range_min
                    or r > min(scan.range_max, self.max_range)):
                continue
            x_v = self.lidar_x + r * math.cos(rel)
            y_v = r * math.sin(rel) + self.lidar_y
            if abs(y_v) > self.offset_max + clear:
                continue
            if lo <= x_v <= hi:
                ys.append(y_v)
        if not ys:
            self._prev_center = None       # 목표 없음 → 이력 리셋
            return None
        ys.sort()

        # 후보 열림 중심 y: 좌 바깥 / blocker 사이(통과폭 충족) / 우 바깥
        cands = [ys[-1] + clear, ys[0] - clear]
        for a, b in zip(ys, ys[1:]):
            if (b - a) >= pass_w:
                cands.append((a + b) / 2.0)

        # offset_max 안에서 통과 가능한 열림만 실현 가능. 하나도 없으면 안전한
        # 목표가 없는 것 → None (예전엔 ±offset_max로 클램프해 여유 미달 지점을
        # 목표로 냈음. 팀장 리뷰 반영). narrow_gap 판정은 호출측(on_scan)이 담당.
        # ★ 후보가 **스캔된 표면 뒤**에 있으면 버린다. 열림 판정(pass_w)은 한 깊이
        #   슬래브 안의 1차원 문제라, 대각선 벽의 단면에 생기는 가짜 '끝'을 걸러내지
        #   못한다 (_behind_surface 주석).
        reach = [c for c in cands
                 if abs(c) <= self.offset_max
                 and not self._behind_surface(scan, obs_x, c)]
        if not reach:
            self._prev_center = None       # 목표 소실 → 이력 리셋
            return None
        center = min(reach, key=abs)   # 직진에서 가장 덜 벗어나는 열림 (이미 clamp 불필요)
        center = self._rate_limit(scan, obs_x, center, ys, clear)
        return self._rp(obs_x, center)

    def _rate_limit(self, scan, obs_x, center, ys=None, clear=None):
        """목표 y 의 프레임 간 변화를 제한한다 (급변 억제).

        ★ 왜 필요한가 (2026-08-10 실측). 열림이 바뀌면 목표 y 가 한 프레임에 1.5m 씩
          튄다. 조향은 63% 서는 데 0.33s 가 걸리는데 0.1s 마다 그런 명령이 오면 차는
          어느 쪽도 못 따라간다. 실측: offset_max 1.6 에서 1m 초과 점프가 24회,
          그 세션 4번 장애물에서 이격이 차폭 안(−0.04m)까지 들어갔다.

          급변의 81% 는 "이전에 고르던 쪽 후보가 소멸" 이라 히스테리시스(전환 여유)로는
          못 잡는다 — 없는 후보를 고를 수는 없기 때문이다(여유 0.5m 까지 시험, 무효).
          그래서 후보 선택이 아니라 **출력 변화율**을 제한한다.

        ★ 안전: 속도 제한된 중간값이 **표면 뒤면 쓰지 않고 그냥 점프**한다. 부드러움
          보다 정확성이 우선 — 중간값이 장애물을 가리키면 안 된다. 실측에서 이 경우는
          243프레임 중 3회였다.

        효과 (2개 bag 재생, offset_max 1.6):
            제한 없음 → 1m 초과 점프 24·32회
            0.30m/프레임 → 2·0 회, narrow_gap 은 그대로 0·19
        """
        prev = self._prev_center
        if prev is not None and self.target_rate_mps > 0.0:
            now = self.get_clock().now().nanoseconds * 1e-9
            dt = now - self._prev_center_t if self._prev_center_t else 0.0
            # dt 이상치(첫 프레임·스캔 유실) 는 1스캔 주기로 본다
            if not 0.0 < dt < 1.0:
                dt = 1.0 / max(1.0, self.scan_rate_hz)
            step = self.target_rate_mps * dt
            if abs(center - prev) > step:
                limited = prev + math.copysign(step, center - prev)
                # ★ 중간값이 blocker 를 **스치거나 관통**하면 쓰지 않고 그냥 점프한다
                #   (2026-08-16 실차 규명). _behind_surface 만으로는 못 막는다 —
                #   그건 표면 **뒤**로 들어간 목표를 거르는데, 목표가 표면 **위**에
                #   있으면 tr ≈ nearest 라 `tr > nearest + TOL` 이 성립하지 않아
                #   통과해버린다.
                #
                #   실측 run_0816_182715 두 번째 장애물: 후보는 −0.82 / +0.63 으로
                #   안정적인데, 선택이 좌↔우로 바뀌자 rate limiter 가 목표를
                #   0.30m/프레임(= target_rate_limit_mps 3.0 × 0.1s)으로 끌고 가며
                #   −1.56 → −0.55 → **+0.04** → +1.18 로 걸어갔다. blocker y 범위가
                #   [−0.36, +0.17] 이라 +0.04 는 콘 한복판을 조준한 것이고, 그 1초
                #   동안 차가 계속 접근해 estop 거리에 들어갔다.
                #
                #   판정은 열림 후보와 **같은 기준**(편측 여유 clear)을 쓴다. 후보는
                #   구성상 이미 이를 만족하므로(바깥 후보는 정확히 clear, 사이 후보는
                #   pass_w/2 = clear 이상) 정상 목표를 막지 않는다.
                near_ok = (not ys) or (clear is None) or all(
                    abs(limited - y) >= clear - 1e-6 for y in ys)
                if near_ok and not self._behind_surface(scan, obs_x, limited):
                    center = limited
        self._prev_center = center
        self._prev_center_t = self.get_clock().now().nanoseconds * 1e-9
        return center

    @staticmethod
    def _rp(x: float, y: float, yaw: float = 0.0, curvature: float = 0.0) -> RefPoint:
        p = RefPoint()
        p.x = float(x)
        p.y = float(y)
        p.yaw = float(yaw)
        p.curvature = float(curvature)
        return p

    def _front_only_scan(self, scan: LaserScan) -> LaserScan:
        """전방 FOV 밖(뒤쪽) 포인트를 inf로 만든 스캔 복사본 (RViz 시각화용)."""
        out = LaserScan()
        out.header = scan.header
        out.angle_min = scan.angle_min
        out.angle_max = scan.angle_max
        out.angle_increment = scan.angle_increment
        out.time_increment = scan.time_increment
        out.scan_time = scan.scan_time
        out.range_min = scan.range_min
        out.range_max = scan.range_max
        inf = float('inf')
        out.ranges = [
            r if abs(wrap_to_pi(scan.angle_min + i * scan.angle_increment
                                - self.front_center)) <= self.front_half_angle
            else inf
            for i, r in enumerate(scan.ranges)
        ]
        out.intensities = scan.intensities
        return out

    def _nearest_front_obstacle(self, scan: LaserScan):
        """전방 통로(±front_half_angle, |y_veh|<corridor) 안 최근접 장애물의
        (앞범퍼 기준 거리 gap [m], vehicle frame 측방 y_veh [m]). 없으면 None.

        LiDAR가 앞범퍼(x=lidar_x)에 있어 LiDAR 전방거리가 곧 앞범퍼~장애물 gap."""
        nearest_x = None
        nearest_y = 0.0
        angle = scan.angle_min
        for r in scan.ranges:
            rel = wrap_to_pi(angle - self.front_center)  # 차량 전방 기준 상대각
            angle += scan.angle_increment
            if (not math.isfinite(r) or r < scan.range_min
                    or r > min(scan.range_max, self.max_range)):
                continue
            if abs(rel) > self.front_half_angle:
                continue
            x_l = r * math.cos(rel)   # LiDAR 전방(+) = 앞범퍼 기준 gap
            y_l = r * math.sin(rel)   # LiDAR 좌측(+)
            y_veh = y_l + self.lidar_y
            if x_l <= 0.0 or abs(y_veh) > self.corridor_half_width:
                continue
            if nearest_x is None or x_l < nearest_x:
                nearest_x = x_l
                nearest_y = y_veh
        if nearest_x is None:
            return None
        return (nearest_x, nearest_y)


def main(args=None):
    rclpy.init(args=args)
    node = StackAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
