"""stack_gps 노드 — 웨이포인트 CSV + RTK 로버(GGA) → /perception/gps_path
담당: 김윤기 (팀장)

구성 (wrapper — 판단 로직 없음, CLAUDE.md §5.5):
  - GgaLink 스레드: 로버 시리얼에서 GGA 수신 + 베이스 RTCM 주입
  - PathEngine (ROS 무의존): CSV 웨이포인트 → vehicle frame ref points
  - 타이머(기본 10Hz): 최신 fix를 pull → 변환 → 발행

fix가 없거나 오래되면(stale_timeout) points를 비우고 fix_quality=0으로
발행한다 — GPS를 신뢰할지 판단은 MGM 스테이트 머신의 몫.

TODO(2단계): /vehicle/vector(dSPACE 상태 추정) 구독 dead-reckoning으로
GGA 사이(수백 ms)를 보간 — dSPACE 프레임과 ENU 정렬 방법 확정 후.

실행 예:
  ros2 run stack_gps stack_gps_node --ros-args \
      -p waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_track_A.csv \
      -p rtcm_host:=100.70.198.29
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from fma_interfaces.msg import GpsPath, RefPoint

from stack_gps.gga_link import GgaLink
from stack_gps.path_engine import PathEngine, load_waypoints_csv


def _pair_ranges(flat, name, logger):
    """[s0, e0, s1, e1, ...] → [(s0, e0), ...]. 홀수 길이는 마지막 값 무시."""
    if len(flat) % 2:
        logger.warn(f"{name} 길이가 홀수 — 마지막 값 무시: {list(flat)}")
    return [(int(flat[i]), int(flat[i + 1])) for i in range(0, len(flat) - 1, 2)]


class StackGpsNode(Node):

    def __init__(self):
        super().__init__('stack_gps_node')
        self.declare_parameter('waypoint_csv', '')
        self.declare_parameter('rtcm_host', '')       # 빈 값 = 주입 안 함
        self.declare_parameter('rtcm_port', 2101)
        self.declare_parameter('serial_port', '/dev/ttyRover')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('n_points', 30)
        self.declare_parameter('publish_period', 0.1)
        self.declare_parameter('stale_timeout', 1.5)  # [s] 이보다 오래된 fix는 무효
        self.declare_parameter('accel_zone_ranges', [0])    # [start,end,...] 인덱스 쌍
        self.declare_parameter('parking_zone_ranges', [0])  # 기본 [0] = 미설정(쌍 안 됨)

        p = self.get_parameter
        csv_path = p('waypoint_csv').value
        if not csv_path:
            raise RuntimeError(
                "waypoint_csv 파라미터가 필요합니다 — "
                "tools/waypoints/record_waypoints.py로 기록한 CSV 경로를 지정하세요.")

        accel = _pair_ranges(p('accel_zone_ranges').value or [],
                             'accel_zone_ranges', self.get_logger())
        parking = _pair_ranges(p('parking_zone_ranges').value or [],
                               'parking_zone_ranges', self.get_logger())

        pts = load_waypoints_csv(csv_path)
        self.engine = PathEngine(pts, n_points=int(p('n_points').value),
                                 accel_ranges=accel, parking_ranges=parking)
        self.get_logger().info(
            f"웨이포인트 {len(pts)}개 로드: {csv_path} "
            f"(accel {accel or '없음'}, parking {parking or '없음'})")

        self.link = GgaLink(
            serial_port=p('serial_port').value, baud=int(p('baud').value),
            rtcm_host=p('rtcm_host').value, rtcm_port=int(p('rtcm_port').value),
            log=lambda m: self.get_logger().info(f"[link] {m}"))
        self.link.start()

        self.stale_timeout = float(p('stale_timeout').value)
        self._last_snap = None
        self.pub = self.create_publisher(GpsPath, '/perception/gps_path', 1)
        self.timer = self.create_timer(float(p('publish_period').value), self.tick)
        self.status_timer = self.create_timer(2.0, self.report_status)

    def tick(self):
        msg = GpsPath()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        fix = self.link.latest_fix()
        if fix is None or fix[4] > self.stale_timeout or fix[3] == 0:
            msg.fix_quality = 0
            self.pub.publish(msg)
            return

        lat, lon, _, quality, _ = fix
        snap = self.engine.snapshot(lat, lon)
        for x, y, yaw, curv in snap['points']:
            rp = RefPoint()
            rp.x, rp.y, rp.yaw, rp.curvature = float(x), float(y), float(yaw), float(curv)
            msg.points.append(rp)
        msg.accel_zone = snap['accel_zone']
        msg.parking_zone = snap['parking_zone']
        msg.fix_quality = quality
        self.pub.publish(msg)
        self._last_snap = snap

    def report_status(self):
        fix = self.link.latest_fix()
        rtcm = self.link.rtcm_rate_and_reset() / 2.0
        if fix is None:
            self.get_logger().warn(f"fix 없음 (RTCM {rtcm:.0f}B/s)")
            return
        _, _, _, quality, age = fix
        qnames = {0: "NOFIX", 1: "GPS", 2: "DGPS", 4: "FIXED", 5: "FLOAT"}
        snap = self._last_snap
        detail = (f"  idx {snap['idx']}  횡오차 {snap['cross_track_m']:.2f}m"
                  + ("  [트랙 끝]" if snap['at_end'] else "")) if snap else ""
        self.get_logger().info(
            f"{qnames.get(quality, quality)}  age {age:.1f}s  RTCM {rtcm:.0f}B/s{detail}")


def main(args=None):
    rclpy.init(args=args)
    node = StackGpsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.link.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
