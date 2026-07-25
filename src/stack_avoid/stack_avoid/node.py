"""stack_avoid — 장애물 인지, 회피 가능 판정 재료(TTC·측방), 회피 경로
담당: 이기돈

1단계 골격: 전방 2D LiDAR 스캔(`/scan`)을 구독해 전방 주행 통로(corridor) 안의
최근접 장애물까지의 거리를 실시간으로 뽑는다. TTC·회피 경로·복귀 판정은 아직 TODO.

설계 메모:
- 앞 LiDAR 1개만으로 반응형 회피 (맵 생성 없음, REQUIREMENTS §11).
- 스캔이 들어올 때마다(≈100ms) AvoidStatus를 발행 → MGM은 최신 스냅샷을 pull.
- 출력 계약은 REQUIREMENTS.md / AvoidStatus.msg 참조. ttc는 장애물 없으면 반드시 큰 값.
- 이 노드는 MGM 10ms 루프와 별도 프로세스 (CLAUDE.md §5.2).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from fma_interfaces.msg import AvoidStatus

TTC_INF = 1.0e9  # 장애물 없을 때 ttc 값 (0 금지 — MGM이 즉시 정지 바닥을 밟음)


def wrap_to_pi(a: float) -> float:
    """각도를 (-pi, pi]로 감싼다."""
    return math.atan2(math.sin(a), math.cos(a))


class StackAvoidNode(Node):

    def __init__(self):
        super().__init__('stack_avoid_node')

        # --- 파라미터 (앞 LiDAR 토픽 확정되면 scan_topic만 바꾸면 됨) ---
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        # 전방 시야각(FOV) [deg]. 180 → 전방 중심 기준 좌우 ±90 = 앞 180도. 나머지는 버린다.
        front_fov = self.declare_parameter('front_fov_deg', 180.0).value
        self.front_half_angle = math.radians(front_fov / 2.0)
        # 전방 중심 각도 [deg]. LiDAR 0도가 차량 정면이 아닐 때 보정. 180 → 반대편을 전방으로.
        self.front_center = math.radians(
            self.declare_parameter('front_offset_deg', 270.0).value)
        # 주행 통로 반폭 [m]. |y| 가 이 값 이내인 점만 "내 경로 위 장애물"로 본다.
        self.corridor_half_width = self.declare_parameter('corridor_half_width', 0.5).value
        # 이 거리 안에 장애물이 들어오면 obstacle_detected=True [m].
        self.detect_range = self.declare_parameter('detect_range', 3.0).value

        # LaserScan은 Best Effort(sensor data QoS)로 발행됨 — 구독도 맞춰야 수신됨.
        self.sub = self.create_subscription(
            LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(AvoidStatus, '/perception/avoid', 1)
        # 디버그: 전방 FOV만 남긴 스캔 (뒤 180도는 inf 처리 → RViz 미표기). 시각화용.
        self.front_scan_pub = self.create_publisher(
            LaserScan, '/scan_front', qos_profile_sensor_data)

        # 재시작 없이 실시간으로 파라미터 변경 반영 (예: front_offset_deg 90도씩 회전)
        self.add_on_set_parameters_callback(self._on_set_params)

        self.get_logger().info(
            f"stack_avoid: '{self.scan_topic}' 구독, front_fov={front_fov}deg, "
            f"front_offset={math.degrees(self.front_center):.0f}deg, "
            f"corridor±{self.corridor_half_width}m, detect<{self.detect_range}m")

    def _on_set_params(self, params):
        """ros2 param set 으로 값 바꾸면 즉시 반영. scan_topic은 재시작 필요."""
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'front_offset_deg':
                self.front_center = math.radians(p.value)
                self.get_logger().info(f"front_offset → {p.value:.0f}도")
            elif p.name == 'front_fov_deg':
                self.front_half_angle = math.radians(p.value / 2.0)
            elif p.name == 'corridor_half_width':
                self.corridor_half_width = p.value
            elif p.name == 'detect_range':
                self.detect_range = p.value
        return SetParametersResult(successful=True)

    def on_scan(self, scan: LaserScan):
        """전방 통로 안의 최근접 장애물 전방거리(x)를 구한다."""
        self.front_scan_pub.publish(self._front_only_scan(scan))  # 시각화용
        nearest_x = self._nearest_front_obstacle(scan)

        msg = AvoidStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'  # vehicle frame (앞 LiDAR 장착 오프셋은 TODO)

        msg.obstacle_detected = nearest_x is not None and nearest_x < self.detect_range

        # TODO(2단계): ttc = nearest_x / ego_speed (VehicleVector.v 구독). 정적 장애물 가정.
        msg.ttc = TTC_INF
        # TODO(2단계): 측방 여유로 회피 성립 판정 (판정 재료만 — 전이 결정은 MGM).
        msg.avoidable = False
        # TODO(2단계): 여유 폭 좁음 → avoid 스테이트 감속 근거.
        msg.narrow_gap = False
        # TODO(4단계): "스캔에 안 보임 = 완료" 금지. 측방 클리어런스/최근 위치 유지로 판정.
        msg.maneuver_done = False
        # TODO(3단계): 회피 기하 기반 권장 속도.
        msg.v_suggest = 0.0
        # TODO(3단계): 회피 경로 RefPoint[] (vehicle frame). 지금은 비움.
        msg.points = []

        self.pub.publish(msg)

    def _front_only_scan(self, scan: LaserScan) -> LaserScan:
        """전방 FOV 밖(뒤쪽) 포인트를 inf로 만든 스캔 복사본. RViz에서 뒤 180도는 안 보인다."""
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
        out.intensities = scan.intensities  # 원본 유지 (크기 맞으면 RViz 무시)
        return out

    def _nearest_front_obstacle(self, scan: LaserScan):
        """전방 통로(±front_half_angle, |y|<corridor) 안 최근접 장애물의 전방거리 x [m].
        없으면 None."""
        nearest_x = None
        angle = scan.angle_min
        for r in scan.ranges:
            # 전방 중심 기준 상대 각도 (front_offset 보정)
            rel = wrap_to_pi(angle - self.front_center)
            angle += scan.angle_increment
            # 무효값 / 측정범위 밖 스킵
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue
            if abs(rel) > self.front_half_angle:
                continue
            x = r * math.cos(rel)  # 전방 +
            y = r * math.sin(rel)  # 좌측 +
            if x <= 0.0 or abs(y) > self.corridor_half_width:
                continue
            if nearest_x is None or x < nearest_x:
                nearest_x = x
        return nearest_x


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
