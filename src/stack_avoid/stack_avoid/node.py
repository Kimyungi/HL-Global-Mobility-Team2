"""LiDAR surface avoidance with a persistent path and bounded station tracking.

Whole-contour gaps seed footprint-checked paths. Each fresh localized scan
returns one vehicle-frame point at station+1m, including tangent/curvature.
Unknown obstacle backs retain a straight tail; observed free space to the GPS
reference enables a return connector. Completion requires reaching that return.
TTC and avoidability are perception inputs; MGM owns speed and mode decisions.
Vehicle and sensor dimensions come from config/params.yaml.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from fma_interfaces.msg import AvoidStatus, RefPoint, VehicleVector
from stack_avoid.path_io import StationPathIO, stamp_ns
from stack_avoid.surfaces import (
    scan_surfaces, nearest_in_corridor,
    outside_intervals, surface_clearance, occluded,
)

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


class StackAvoidNode(StationPathIO, Node):

    def __init__(self):
        super().__init__('stack_avoid_node')

        # ── 파라미터 로드 (config/params.yaml, 미지정 시 기본값) ──────────
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.target_speed = self.declare_parameter('target_speed_mps', 0.5).value

        # 차량 제원 (경로 전체의 회전된 차체 footprint 검사에 사용)
        self.vehicle_width = self.declare_parameter('vehicle.width_m', 0.62).value
        self.vehicle_len = self.declare_parameter('vehicle.length_m', 0.85).value

        # LiDAR 장착 (원점=후축 중심 기준)
        self.lidar_x = self.declare_parameter('lidar_mount.x_m', 0.76).value
        self.lidar_y = self.declare_parameter('lidar_mount.y_m', 0.0).value
        self.lidar_z = self.declare_parameter('lidar_mount.z_m', 0.065).value
        self.lidar_yaw = self.declare_parameter('lidar_mount.yaw_deg', 0.0).value
        self.lidar_roll = self.declare_parameter('lidar_mount.roll_deg', 0.0).value
        self.lidar_pitch = self.declare_parameter('lidar_mount.pitch_deg', 0.0).value
        # 차량 전방을 가리키는 스캔 각도. 단일 /scan 기본값은 270°이며,
        # 4-LiDAR 통합 launch는 a1 보정값에서 계산한 87°를 시작 시 전달한다.
        # read_only: 실행 중 좌표 기준이 바뀌지 않도록 param set은 금지한다.
        self.front_center = math.radians(
            self.declare_parameter(
                'lidar_mount.forward_angle_deg', 270.0,
                ParameterDescriptor(
                    read_only=True,
                    description='원본 스캔의 차량 전방 각도; 시작 시 설정, 런타임 변경 불가')).value)

        # 회피 판단 (튜닝 글로벌)
        self.roi_angle = self.declare_parameter('avoid.roi_angle_deg', 180.0).value
        self.ttc_stop = self.declare_parameter('avoid.ttc_stop_s', 1.5).value
        self.lateral_margin = self.declare_parameter('avoid.lateral_margin_m', 0.15).value
        self.detect_range = self.declare_parameter('avoid.detect_range_m', 3.5).value
        # Detection area is independent of the vehicle collision clearance.
        self.detect_half_width = self.declare_parameter('avoid.detect_half_width_m', 0.5).value
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.detect_range, self.detect_half_width)):
            raise ValueError('detection range and half width must be finite and positive')
        self.max_range = self.declare_parameter('avoid.max_range_m', 12.0).value
        # 회피 목표점 측방 오프셋 상한
        self.offset_max = self.declare_parameter('avoid.offset_max_m', 2.5).value
        # 이 깊이 밴드에 걸친 연결 윤곽 전체를 양쪽 고려한다.
        self.depth_band = self.declare_parameter('avoid.depth_band_m', 0.6).value
        self.cluster_dist = float(self.declare_parameter('avoid.cluster_dist_m', 0.10).value)
        self.surface_link_scale = float(
            self.declare_parameter('avoid.surface_link_scale', 3.0).value)
        self.surface_max_link = float(
            self.declare_parameter('avoid.surface_max_link_m', 0.30).value)
        if not self._valid_surface_params(
                self.cluster_dist, self.surface_link_scale, self.surface_max_link):
            raise ValueError('surface link parameters must be finite and positive; max >= cluster_dist')
        self._surfaces = []
        # obstacle_detected 해제 히스테리시스 [m] — 진입은 detect_range, 해제는
        # detect_range + 이 값. 경계에 걸친 물체(벽·연석)가 깜빡이면 그때마다
        # 클리어런스 타이머가 리셋돼 maneuver_done이 영원히 안 선다
        # (2026-08-15 run_0815_143039: 20초에 28회 토글, AVOID 15초+ 지속,
        #  그동안 직진 유지점만 나가 횡오차 5.3m까지 발산).
        self.detect_hysteresis = float(
            self.declare_parameter('avoid.detect_hysteresis_m', 0.4).value)
        self._detected_prev = False     # 히스테리시스 상태
        self._recompute_derived()
        self._init_path_tracking()

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
            f"검출반폭 {self.detect_half_width:.2f}m, 통로반폭 {self.corridor_half_width:.2f}m, 전방FOV {self.roi_angle}deg, "
            f"forward={math.degrees(self.front_center):.0f}deg, "
            f"detect<{self.detect_range}m, ttc_stop {self.ttc_stop}s, v={self.target_speed}m/s")

    def _recompute_derived(self):
        """실측/튜닝값에서 파생되는 내부값 갱신."""
        # 차량 충돌 여유 반폭. 장애물 검출 영역은 detect_half_width로 별도 설정한다.
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
        proposed = {p.name: p.value for p in params}
        if not all(math.isfinite(v) and v > 0 for v in (
                proposed.get('avoid.detect_range_m', self.detect_range),
                proposed.get('avoid.detect_half_width_m', self.detect_half_width))):
            return SetParametersResult(successful=False, reason='invalid detection range or half width')
        width = proposed.get('vehicle.width_m', self.vehicle_width)
        margin = proposed.get('avoid.lateral_margin_m', self.lateral_margin)
        if not (math.isfinite(width) and width > 0 and math.isfinite(margin) and margin >= 0):
            return SetParametersResult(successful=False, reason='invalid vehicle width or lateral margin')
        if not self._valid_surface_params(
                proposed.get('avoid.cluster_dist_m', self.cluster_dist),
                proposed.get('avoid.surface_link_scale', self.surface_link_scale),
                proposed.get('avoid.surface_max_link_m', self.surface_max_link)):
            return SetParametersResult(successful=False, reason=(
                'surface link parameters must be finite and positive; max >= cluster_dist'))
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
            elif p.name == 'avoid.detect_half_width_m':
                self.detect_half_width = p.value
            elif p.name == 'avoid.max_range_m':
                self.max_range = p.value
            elif p.name == 'avoid.offset_max_m':
                self.offset_max = p.value
            elif p.name == 'avoid.depth_band_m':
                self.depth_band = p.value
            elif p.name == 'avoid.cluster_dist_m':
                self.cluster_dist = float(p.value)
            elif p.name == 'avoid.surface_link_scale':
                self.surface_link_scale = float(p.value)
            elif p.name == 'avoid.surface_max_link_m':
                self.surface_max_link = float(p.value)
            elif p.name == 'vehicle.width_m':
                self.vehicle_width = p.value
            elif p.name == 'lidar_mount.forward_angle_deg':
                self.front_center = math.radians(p.value)
        self._recompute_derived()
        self._publish_static_tf()   # forward_angle 변경 시 TF도 갱신(스캔 방향 일치)
        self.get_logger().info(
            f"param 변경 → 검출반폭 {self.detect_half_width:.2f}m, 통로반폭 {self.corridor_half_width:.2f}m, 전방FOV {self.roi_angle}deg, "
            f"forward={math.degrees(self.front_center):.0f}deg, v={self.target_speed}m/s")
        return SetParametersResult(successful=True)

    def on_vehicle_vector(self, vv: VehicleVector):
        """dSPACE 상태추정 수신 → 자차속도 갱신 (TTC 계산 입력). 후진도 대비해 절대값."""
        self._store_vehicle_pose(vv)
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
        """One fresh scan advances the persistent path station at most once."""
        stamp = stamp_ns(scan.header.stamp)
        if stamp > 0 and stamp <= self._path_last_scan:
            return
        msg = AvoidStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.ttc = TTC_INF
        msg.v_suggest = float(self.target_speed)
        msg.scan_valid = bool(
            scan.ranges and math.isfinite(scan.angle_min)
            and math.isfinite(scan.angle_increment) and scan.angle_increment != 0.
            and math.isfinite(scan.range_min) and math.isfinite(scan.range_max)
            and scan.range_max >= scan.range_min
            and self._fresh_stamp(stamp, self.path_pose_timeout)
            and any(r == math.inf or (math.isfinite(r) and scan.range_min <= r <= scan.range_max)
                    for r in scan.ranges))
        if not msg.scan_valid:
            # Invalid data cannot certify clearance, advance station or finish.
            msg.obstacle_detected = self._detected_prev
            self._planner.path = None
            self._publish_path(scan, None)
            self.pub.publish(msg)
            return
        self._path_last_scan = stamp
        self.front_scan_pub.publish(self._front_only_scan(scan))
        self._surfaces = self._scan_surfaces(scan)
        obs = self._nearest_front_obstacle(scan)
        gap = obs[0] if obs is not None else None
        threshold = self.detect_range+(self.detect_hysteresis if self._detected_prev else 0.)
        self._detected_prev = gap is not None and gap < threshold
        msg.obstacle_detected = self._detected_prev
        speed = self._ego_speed()
        if gap is not None and math.isfinite(speed) and speed > EPS_SPEED:
            msg.ttc = float(gap/speed)
        if msg.obstacle_detected and self._completed:
            self._planner.reset()
            self._completed = False
        if self._completed:
            self._planner.reset()
        point, done, generation = self._station_reference(scan, gap, msg.obstacle_detected)
        if point is not None:
            msg.points = [self._rp(*point)]
            msg.reference_stamp.sec = generation//1_000_000_000
            msg.reference_stamp.nanosec = generation%1_000_000_000
        self._completed = (self._completed or done) and not msg.obstacle_detected
        msg.maneuver_done = self._completed
        msg.avoidable = bool(msg.points) and msg.ttc >= self.ttc_stop
        msg.narrow_gap = msg.obstacle_detected and not msg.points
        self.pub.publish(msg)

    # 목표가 표면보다 이만큼 이상 멀면 "뒤"로 본다 [m]. 측정 잡음·클러스터 두께 흡수.
    BEHIND_TOL_M = 0.10
    # 목표 방위각 주변 이 각도 안의 측정치를 본다 [deg]. 3m 에서 ±0.10m 에 해당.
    BEHIND_WIN_DEG = 2.0

    @staticmethod
    def _valid_surface_params(distance, scale, maximum):
        return (all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0
                    for v in (distance, scale, maximum)) and maximum >= distance)

    def _scan_surfaces(self, scan):
        return scan_surfaces(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, min(scan.range_max, self.max_range),
            self.front_center, self.front_half_angle, self.lidar_x, self.lidar_y,
            self.cluster_dist, self.surface_link_scale, self.surface_max_link)

    def _behind_surface(self, scan, tx, ty):
        """Reject targets hidden by connected edges, plus the existing beam window guard."""
        if occluded((self.lidar_x, self.lidar_y), (tx, ty),
                    self._surfaces, self.BEHIND_TOL_M):
            return True
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

    def _target_clear(self, scan, x, y, intervals, clear):
        return (outside_intervals(y, intervals)
                and surface_clearance((x, y), self._surfaces) >= clear - 1e-6
                and not self._behind_surface(scan, x, y))

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
        """Surface/corridor intersection, including gaps between adjacent returns.

        `_surfaces` is rebuilt once per on_scan before detection/target selection.
        The LiDAR x is the front bumper; return (bumper distance, vehicle y).
        """
        return nearest_in_corridor(self._surfaces, self.lidar_x, self.detect_half_width)


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
