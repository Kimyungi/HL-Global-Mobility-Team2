"""stack_avoid — 장애물 인지, 회피 가능 판정 재료(TTC·측방), 회피 경로
담당: 이기돈

- 전방 2D LiDAR 스캔(`/scan`)을 구독해 전방 통로(corridor) 안 최근접 장애물 거리·TTC 산출.
- 장애물 감지 시 회피 목표점 1개를 follow-the-gap(양쪽 장애물 고려)으로 생성 → `points[]`.
  전방 FOV의 열림(gap)들 중 통과 가능·최소 편차 열림 중심으로 조준.
  (avoid n_points=1: dSPACE quintic이 현재 자세에서 이 목표점으로 궤적을 채움)
- 모든 차량/센서/튜닝 값은 `config/params.yaml`에서 로드(하드코딩 금지, CLAUDE.md §5).
- LiDAR 장착 오프셋으로 vehicle frame(base_link=후축 중심) 보정 + static TF 발행.
- avoidable/narrow_gap/maneuver_done/v_suggest 는 다음 단계(TODO).

설계 메모:
- 앞 LiDAR로 반응형 회피 (맵 생성 없음, REQUIREMENTS §계약).
- 스캔이 들어올 때마다(≈100ms) AvoidStatus 발행 → MGM은 최신 스냅샷을 pull.
- ttc는 장애물 없으면 반드시 큰 값(1e9). 정지/모드 결정은 이 스택 금지 → MGM 몫.
- 이 노드는 MGM 10ms 루프와 별도 프로세스 (CLAUDE.md §5.2).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from fma_interfaces.msg import AvoidStatus, RefPoint

TTC_INF = 1.0e9   # 장애물 없을 때 ttc (0 금지 — MGM이 즉시 정지 바닥을 밟음)
EPS_SPEED = 1e-3  # 이보다 느리면 정지 상태로 보고 ttc=INF


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

        # 차량 제원 (통로 폭 계산에 차폭 사용)
        self.vehicle_width = self.declare_parameter('vehicle.width_m', 0.62).value

        # LiDAR 장착 (원점=후축 중심 기준)
        self.lidar_x = self.declare_parameter('lidar_mount.x_m', 0.76).value
        self.lidar_y = self.declare_parameter('lidar_mount.y_m', 0.0).value
        self.lidar_z = self.declare_parameter('lidar_mount.z_m', 0.065).value
        self.lidar_yaw = self.declare_parameter('lidar_mount.yaw_deg', 0.0).value
        self.lidar_roll = self.declare_parameter('lidar_mount.roll_deg', 0.0).value
        self.lidar_pitch = self.declare_parameter('lidar_mount.pitch_deg', 0.0).value
        # 차량 전방을 가리키는 스캔 각도(드라이버 프레임 관례) — 필드검증 완료(270)
        self.front_center = math.radians(
            self.declare_parameter('lidar_mount.forward_angle_deg', 270.0).value)

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

        self._recompute_derived()

        # ── I/O ──────────────────────────────────────────────────────────
        # LaserScan은 Best Effort(sensor data QoS) — 구독도 맞춰야 수신됨.
        self.sub = self.create_subscription(
            LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)
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
        qx, qy, qz, qw = euler_to_quat(
            math.radians(self.lidar_roll),
            math.radians(self.lidar_pitch),
            math.radians(self.lidar_yaw))
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
            elif p.name == 'vehicle.width_m':
                self.vehicle_width = p.value
            elif p.name == 'lidar_mount.forward_angle_deg':
                self.front_center = math.radians(p.value)
        self._recompute_derived()
        self.get_logger().info(
            f"param 변경 → 통로반폭 {self.corridor_half_width:.2f}m, 전방FOV {self.roi_angle}deg, "
            f"forward={math.degrees(self.front_center):.0f}deg, v={self.target_speed}m/s")
        return SetParametersResult(successful=True)

    def on_scan(self, scan: LaserScan):
        """전방 통로 안 최근접 장애물 → 거리·TTC + 회피 목표점 발행."""
        self.front_scan_pub.publish(self._front_only_scan(scan))  # 시각화용
        obs = self._nearest_front_obstacle(scan)   # (gap, y_veh) or None
        gap = obs[0] if obs is not None else None

        msg = AvoidStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'          # points[]는 vehicle frame(후축 원점)

        msg.obstacle_detected = gap is not None and gap < self.detect_range

        # TTC = 거리 ÷ 자차속도 (정적 장애물 가정, REQUIREMENTS). 정지 중이면 INF.
        # NOTE(stage2): 자차속도는 VehicleVector.v 구독으로 교체 예정. 지금은 목표속도 근사.
        if gap is None or self.target_speed <= EPS_SPEED:
            msg.ttc = TTC_INF
        else:
            msg.ttc = float(gap / self.target_speed)

        # 회피 목표점 1개(follow-the-gap, 양쪽 고려), vehicle frame — 감지 시에만.
        if msg.obstacle_detected:
            tgt = self._gap_target(scan, gap)
            msg.points = [tgt] if tgt is not None else []
        else:
            msg.points = []

        # TODO(stage2): avoidable = (측방 여유 확보 AND ttc >= ttc_stop). 판정 재료만.
        msg.avoidable = False
        # TODO(stage2): 통과 최소폭(= width + 2*margin) 대비 여유 폭 좁음 판정.
        msg.narrow_gap = False
        # TODO(stage4): "스캔에 안 보임 = 완료" 금지. 측방 클리어런스/최근 위치 유지로 판정.
        msg.maneuver_done = False
        # TODO(stage3): 회피 기하 기반 권장 속도(v_suggest).
        msg.v_suggest = 0.0

        self.pub.publish(msg)

        # 튜닝 보조 로그(비권위적): ttc_stop 임계 확인용. 정지 결정은 MGM.
        if msg.obstacle_detected and msg.ttc < self.ttc_stop:
            self.get_logger().debug(
                f"ttc {msg.ttc:.2f}s < ttc_stop {self.ttc_stop}s (gap {gap:.2f}m)")

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
            if not math.isfinite(r) or r < scan.range_min or r > self.detect_range:
                continue
            x_v = self.lidar_x + r * math.cos(rel)
            if x_v < lo or x_v > hi:
                continue
            ys.append(r * math.sin(rel) + self.lidar_y)
        if not ys:
            return None
        ys.sort()

        # 후보 열림 중심 y: 좌 바깥 / blocker 사이(통과폭 충족) / 우 바깥
        cands = [ys[-1] + clear, ys[0] - clear]
        for a, b in zip(ys, ys[1:]):
            if (b - a) >= pass_w:
                cands.append((a + b) / 2.0)

        # offset_max 안에 실현 가능한 것 우선, 직진에서 가장 덜 벗어나는 중심 선택
        reach = [c for c in cands if abs(c) <= self.offset_max]
        center = min(reach if reach else cands, key=abs)
        target_y = max(-self.offset_max, min(self.offset_max, center))
        return self._rp(obs_x, target_y)

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
            if not math.isfinite(r) or r < scan.range_min or r > min(scan.range_max, self.max_range):
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
