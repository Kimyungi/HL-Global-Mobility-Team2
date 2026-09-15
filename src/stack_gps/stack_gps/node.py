"""stack_gps 노드 — RTK/IMU → /perception/gps_path + relative /perception/imu
담당: 김윤기 (팀장)

구성 (wrapper — 판단 로직 없음, CLAUDE.md §5.5):
  - GgaLink 스레드: 로버 시리얼에서 GGA 수신 + 베이스 RTCM 주입
  - ImuLink 스레드: HandsFree IMU 오일러각·자이로 수신 (imu_port:=off로 비활성)
  - HeadingFusion (ROS 무의존): IMU yaw + COG → 정지 포함 유효한 ENU 헤딩
  - PathEngine (ROS 무의존): CSV 웨이포인트 → vehicle frame ref points
  - 타이머(기본 10Hz): 최신 fix를 pull → 변환 → 발행

fix가 없거나 오래되면(stale_timeout) points를 비우고 fix_quality=0으로
발행한다 — GPS를 신뢰할지 판단은 MGM 스테이트 머신의 몫.

TODO(2단계): /vehicle/vector(dSPACE 상태 추정) 구독 dead-reckoning으로
GGA 사이(수백 ms)를 보간 — dSPACE 프레임과 ENU 정렬 방법 확정 후.

실행 예:
  ros2 run stack_gps stack_gps_node --ros-args \
      -p waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_wonju_license_20260818_160511.csv \
      -p rtcm_host:=100.70.198.29
"""
import math
import os
import time
import uuid
from types import SimpleNamespace

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.duration import Duration

from fma_interfaces.msg import EstopRequest, GpsPath, GpsRoute, RefPoint, ZoneContext, MgmState, TargetRef
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, NavSatFix
from tf2_ros import TransformBroadcaster

from stack_gps.zones import ZoneMap, load_zone_definitions
from stack_gps.route_plan import RoutePlan, copy_geometry
from stack_gps.gga_link import GgaLink
from stack_gps.heading_fusion import HeadingFusion
from stack_gps.imu_link import ImuLink
from stack_gps.path_engine import PathEngine, PoseDeltaTracker, load_waypoints_csv, wrap_angle, avoidance_marker_range


def _pair_ranges(flat, name, logger):
    """[s0, e0, s1, e1, ...] → [(s0, e0), ...]. 홀수 길이는 마지막 값 무시."""
    if len(flat) % 2:
        logger.warn(f"{name} 길이가 홀수 — 마지막 값 무시: {list(flat)}")
    return [(int(flat[i]), int(flat[i + 1])) for i in range(0, len(flat) - 1, 2)]


def _parse_latlon_spec(spec, per, name, logger):
    """"lat,lon; lat,lon" → [(lat, lon), ...] (per=2) / 구간이면 per=4.

    지정 구간을 **위경도 문자열**로 받는 이유:
      · 웨이포인트 인덱스는 트랙을 다시 기록하면 전부 어긋나지만 위경도는 장소다.
      · launch 인자는 문자열밖에 못 넘긴다 — 실수 배열 파라미터는 launch
        치환에서 타입이 깨져(ParameterValue 리스트 미지원) 현장에서 못 고친다.
        문자열로 받아 노드가 파싱하면 `ros2 param` 으로도 그대로 다룰 수 있다.
    구분자: 항목 사이 ';', 숫자 사이 ','. 형식이 어긋난 항목은 버리고 경고.
    """
    out = []
    for chunk in (spec or "").split(';'):
        chunk = chunk.strip()
        if not chunk:
            continue
        try:
            vals = [float(v) for v in chunk.split(',')]
        except ValueError:
            logger.warn(f"{name} 항목 파싱 실패 — 무시: '{chunk}'")
            continue
        if len(vals) != per:
            logger.warn(f"{name} 항목은 숫자 {per}개여야 함 — 무시: '{chunk}'")
            continue
        out.append(tuple(vals))
    return out


def _load_zones_file(path, logger):
    """구간 YAML → 정지점, 회피/GPS 구간, 주차점 목록.

    파일이 없으면 조용히 빈 목록 — 구간을 안 쓰는 run 이 정상이기 때문이다.
    형식이 깨졌거나 회피 구간의 끝점이 없으면 **그 항목만 버리고 경고**한다:
    구간 하나 때문에 주행 전체를 막는 것도, 조용히 무시하는 것도 나쁘다.
    """
    empty = ([], [], [], [])
    if not path:
        return empty
    if not os.path.isfile(path):
        logger.info(f"구간 파일 없음 (지정 구간 미사용): {path}")
        return empty
    try:
        with open(path) as f:
            z = yaml.safe_load(f) or {}
    except Exception as e:                                    # noqa: BLE001
        logger.error(f"구간 파일을 읽을 수 없음 — 무시하고 계속: {path} ({e})")
        return empty

    stops = []
    for i, e in enumerate(z.get('stop_points') or []):
        try:
            stops.append((float(e['lat']), float(e['lon'])))
            note = e.get('note') or ''
            logger.info(f"  구간 파일 정지 지점 {i + 1}: {e['lat']:.7f},{e['lon']:.7f}"
                        + (f" ({note})" if note else ""))
        except (KeyError, TypeError, ValueError):
            logger.warn(f"  정지 지점 {i + 1} 형식 오류 — 무시: {e!r}")
    def _pairs(key, label):
        out = []
        for i, e in enumerate(z.get(key) or []):
            try:
                s_, t_ = e['start'], e['end']
                out.append((float(s_['lat']), float(s_['lon']),
                            float(t_['lat']), float(t_['lon'])))
            except (KeyError, TypeError, ValueError):
                logger.warn(f"  {label} {i + 1} 이 불완전(끝점 없음?) — 무시: {e!r}")
        return out

    avoid = _pairs('avoid_zones', '회피 구간')
    gps_only = _pairs('gps_only_zones', 'GPS 전용 구간')
    parking = []
    for i, e in enumerate(z.get('parking_points') or []):
        try:
            mode = str(e['mode']).strip().lower()
            if mode in ('t', 't_parking', 'perpendicular'):
                mode = 'perpendicular'
            elif mode in ('parallel', 'parallel_parking'):
                mode = 'parallel'
            else:
                raise ValueError(f'알 수 없는 mode={mode!r}')
            lat = float(e['lat'])
            lon = float(e['lon'])
            parking.append((lat, lon, mode))
            note = e.get('note') or ''
            logger.info(
                f"  구간 파일 주차 지점 {i + 1}: {lat:.7f},{lon:.7f} "
                f"({mode}{f', {note}' if note else ''})")
        except (KeyError, TypeError, ValueError) as error:
            logger.warn(f"  주차 지점 {i + 1} 형식 오류 — 무시: {e!r} ({error})")
    logger.info(f"구간 파일 로드: {path} (정지 {len(stops)} · 회피 {len(avoid)} · "
                f"GPS전용 {len(gps_only)} · 주차 {len(parking)})")
    return stops, avoid, gps_only, parking


class _RouteLog:
    """A skipped spatial definition would change Mission completion requirements."""
    def __init__(self, logger):
        self.logger = logger

    def __getattr__(self, name):
        return getattr(self.logger, name)

    def error(self, message):
        raise ValueError(message)


class StackGpsNode(Node):

    def __init__(self):
        super().__init__('stack_gps_node')
        self.declare_parameter('waypoint_csv', '')
        self.declare_parameter('route_sequence_file', '')
        self.declare_parameter('route_start_id', '')
        self.declare_parameter('route_end_id', '')
        self.declare_parameter('rtcm_host', '')       # 빈 값 = 주입 안 함
        self.declare_parameter('rtcm_port', 2101)
        self.declare_parameter('serial_port', '/dev/ttyRover')
        self.declare_parameter('link_mode', 'direct', ParameterDescriptor(read_only=True))
        self.declare_parameter('baud', 115200)
        self.declare_parameter('n_points', 1, ParameterDescriptor(read_only=True))
        # Fixed station +2.5m preview; publish_period is the station search sample_time.
        self.declare_parameter('publish_period', 0.1, ParameterDescriptor(read_only=True))
        self.station_sample_time = float(self.get_parameter('publish_period').value)
        if not math.isfinite(self.station_sample_time) or self.station_sample_time <= 0.0:
            raise ValueError('publish_period must be finite and positive')
        if self.get_parameter('n_points').value != 1:
            raise ValueError('v2 GPS returns exactly one station preview point (n_points=1)')
        self.declare_parameter('stale_timeout', 1.5)  # [s] 이보다 오래된 fix는 무효
        # v_base(MGM params.yaml)보다 낮아야 이동 중 COG가 잡힌다.
        # 0.25에서 COG 노이즈 ~7° 수준 — 융합 저역통과가 흡수 (저속 시험 대응)
        self.declare_parameter('cog_min_speed', 0.25)
        self.declare_parameter('cog_max_age', 1.0)    # [s] COG 신선도 한계
        self.declare_parameter('imu_port', '/dev/ttyUSB_IMU')  # 'off' = IMU 없이
        self.declare_parameter('imu_baud', 921600)
        # 헤딩 소스 = 자이로 적분 yaw (2026-08-04: 오일러 yaw는 지자기 오염으로
        # 폐기 — imu_link 참조). 자이로는 반시계+ 실측 → 기본 +1.0.
        # 기종 교체·재장착 시 tools/imu_sign_check.py로 재판정.
        self.declare_parameter('imu_yaw_sign', 1.0)
        self.declare_parameter('imu_topic', '/perception/imu')
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('imu_publish_stale_s', 0.25)
        self.declare_parameter('fusion_alpha', 0.1)   # offset 저역통과 이득
        self.declare_parameter('accel_zone_ranges', [0])    # [start,end,...] 인덱스 쌍
        # Legacy parking_zone_ranges now means T/perpendicular parking.
        # A separate range selects the parallel test case. The two sets must
        # not overlap because one GPS position must select exactly one mode.
        self.declare_parameter('parking_zone_ranges', [0])
        self.declare_parameter('parallel_parking_zone_ranges', [0])
        # ── 트랙 위 지정 구간 (2026-08-18). 위경도 문자열로 받아 시작 시
        # 웨이포인트 인덱스 구간으로 변환한다 (_parse_latlon_spec 주석 참조).
        #   stop_points_latlon : "lat,lon" — 그 지점에서 정지(정지 시간 판단은 MGM).
        #                        여러 개는 ';' 로. 번호는 나열 순서(1부터).
        #   avoid_zone_latlon  : "lat1,lon1,lat2,lon2" — 회피를 허용할 구간의
        #                        시작·끝 지점. MGM avoid_zone_only 와 짝.
        # 빈 문자열 = 없음. 지정 지점이 트랙에서 stop_zone_snap_max_m 보다 멀면
        # **다른 코스의 좌표**로 보고 버린다 (조용히 엉뚱한 곳에서 서지 않도록).
        # 구간 파일 (mark_zone 이 기록하는 YAML). 문자열 파라미터와 **합쳐진다** —
        # 파일이 상시 소스이고, 문자열 쪽은 일회성 수동 지정/시험용이다.
        self.declare_parameter('zones_file', '')
        self.declare_parameter('stop_points_latlon', '')
        self.declare_parameter('avoid_zone_latlon', '')
        self.declare_parameter('gps_only_zone_latlon', '')
        self.declare_parameter('stop_zone_span_m', 1.0)      # 정지 구간 폭 [m]
        # parking_points 한 점을 이 길이의 웨이포인트 구간으로 넓힌다.
        self.declare_parameter('parking_zone_span_m', 1.0)
        # 회피 허용 구간을 **앞쪽으로** 늘리는 길이 [m] (2026-08-18 실차에서 도출).
        # 사람은 콘이 있는 자리를 구간으로 찍지만, 회피 판단(avoidable)은 감지
        # 거리 3m 안에서 TTC 가 문턱(1.5s) 위일 때만 성립한다 — 구간이 콘에서
        # 시작하면 그 창이 이미 지나간 뒤에 게이트가 열린다. detect_range 3.0m +
        # 여유 2m 로 잡는다. path_engine.index_before() 주석에 실측 근거.
        self.declare_parameter('avoid_zone_lead_m', 5.0)
        self.declare_parameter('stop_zone_snap_max_m', 5.0)  # 트랙 스냅 허용 [m]
        self.declare_parameter('error_log_csv', '')  # 지정 시 매 틱 횡오차 CSV 기록

        p = self.get_parameter
        start_id, end_id = p('route_start_id').value, p('route_end_id').value
        if (start_id or end_id) and not p('route_sequence_file').value:
            raise ValueError('route_start_id/route_end_id require route_sequence_file')
        self._route_plan = RoutePlan(p('route_sequence_file').value, start_id, end_id) if p('route_sequence_file').value else None
        self._route_loading = self._route_plan is not None
        self._route_instance = (uuid.uuid4().int & ((1 << 64) - 1)) or 1
        csv_path = p('waypoint_csv').value
        if self._route_plan:
            first = str(self._route_plan.files[0].csv)
            if csv_path and os.path.realpath(csv_path) != first:
                raise ValueError('waypoint_csv must be empty or the first sequence CSV')
            csv_path = first
            for key in ('accel_zone_ranges', 'parking_zone_ranges', 'parallel_parking_zone_ranges'):
                if list(p(key).value) not in ([], [0]):
                    raise ValueError(f'{key}: sequence uses per-route Zone files')
            for key in ('stop_points_latlon', 'avoid_zone_latlon', 'gps_only_zone_latlon'):
                if p(key).value:
                    raise ValueError(f'{key}: sequence uses per-route Zone files')
        if not csv_path:
            raise RuntimeError(
                "waypoint_csv 파라미터가 필요합니다 — "
                "tools/waypoints/record_waypoints.py로 기록한 CSV 경로를 지정하세요.")

        accel = _pair_ranges(p('accel_zone_ranges').value or [],
                             'accel_zone_ranges', self.get_logger())
        parking = _pair_ranges(p('parking_zone_ranges').value or [],
                               'parking_zone_ranges', self.get_logger())
        parallel_parking = _pair_ranges(
            p('parallel_parking_zone_ranges').value or [],
            'parallel_parking_zone_ranges', self.get_logger())

        pts = load_waypoints_csv(csv_path, log=self.get_logger().warn)
        self.engine = PathEngine(pts, n_points=1, station_tracking=True,
                                 accel_ranges=accel, parking_ranges=parking,
                                 parallel_parking_ranges=parallel_parking)
        if self._route_plan:
            initial_engine = self.engine
            def factory(files):
                # Each route gets independent station history; source geometry stays unchanged.
                self.engine = copy_geometry(initial_engine, files.points)
                route_values = {'zones_file': str(files.zones), 'waypoint_csv': str(files.csv)}
                self._setup_zones(lambda key: SimpleNamespace(value=route_values[key]) if key in route_values else p(key))
                return self.engine, self.zone_map
            self._route_plan.bind(factory)
            self.engine = self._route_plan.engines[0]
            self.zone_map = self._route_plan.zone_maps[0]
            self.get_logger().info(f'Route sequence ready: {len(self._route_plan.files)} routes, id={self._route_plan.sequence_id}')
        else:
            self._setup_zones(p)
        self._route_loading = False
        self.get_logger().info(
            f"재합류 기하: lookahead {self.engine.lookahead_m:.2f}m "
            f"rate_damp {self.engine.rate_damp_s:.2f}s "
            f"full_cross {self.engine.full_cross_m:.2f}m "
            f"target_max {self.engine.target_max_m:.2f}m "
            f"target_min {self.engine.target_min_m:.2f}m "
            f"e_lpf {self.engine.e_lpf_s:.2f}s "
            f"곡선ff {self.engine.curve_ff:.2f} 곡선당김 {self.engine.curve_margin:.1f}")
        self.get_logger().info(
            f"웨이포인트 {len(pts)}개 로드: {csv_path} "
            f"(accel {accel or '없음'}, "
            f"T parking {self.engine.parking_ranges or '없음'}, "
            f"parallel parking {self.engine.parallel_parking_ranges or '없음'})")

        rtcm_host = p('rtcm_host').value
        if rtcm_host.lower() in ('off', 'none'):  # launch 인자는 빈 값 불가
            rtcm_host = ''
            self.get_logger().warn(
                "RTCM 주입 꺼짐 — 단독 GPS 모드 (RTK 없음, 오차 수 m 예상)")
        # NMEA 두절 시 USB 리셋 복구 (2026-08-18 실차 — usb_reset.py 주석).
        # 0 이하면 끔. 판정은 fix 품질이 아니라 **NMEA 무수신**으로만 한다.
        self.declare_parameter('usb_reset_after_s', 20.0)
        self.declare_parameter('usb_reset_cooldown_s', 60.0)
        mode = p('link_mode').value
        if mode not in ('direct', 'persistent'):
            raise ValueError('link_mode must be direct or persistent')
        if mode == 'persistent':
            from stack_gps.persistent_link import PersistentGgaLink
            link_type = PersistentGgaLink
        else:
            link_type = GgaLink
        self.link = link_type(
            serial_port=p('serial_port').value, baud=int(p('baud').value),
            rtcm_host=rtcm_host, rtcm_port=int(p('rtcm_port').value),
            log=lambda m: self.get_logger().info(f"[link] {m}"),
            usb_reset_after_s=float(p('usb_reset_after_s').value),
            usb_reset_cooldown_s=float(p('usb_reset_cooldown_s').value))
        self.link.start()

        self.stale_timeout = float(p('stale_timeout').value)
        self.cog_min_speed = float(p('cog_min_speed').value)
        self.cog_max_age = float(p('cog_max_age').value)

        imu_port = p('imu_port').value
        self.imu = None
        self.fusion = None
        if imu_port and imu_port.lower() not in ('off', 'none'):
            self.imu = ImuLink(imu_port, baud=int(p('imu_baud').value),
                               log=lambda m: self.get_logger().info(f"[imu] {m}"))
            self.imu.start()
            self.fusion = HeadingFusion(alpha=float(p('fusion_alpha').value),
                                        sign=float(p('imu_yaw_sign').value))
        else:
            self.get_logger().warn(
                "IMU 꺼짐 — 헤딩은 COG/접선만 사용 (정지 시 절대 헤딩 없음)")
        self._heading_src = '접선'
        self._pose_delta_tracker = PoseDeltaTracker()
        self._zone_telemetry_fix = None
        self._zone_telemetry_previous = None
        self._reference_fix_time = None
        self._reference_stamp = None
        self._imu_gen = 0
        self._cog_ok = False
        self._was_aligned = False
        self._last_snap = None
        self._err_log = None
        log_path = p('error_log_csv').value
        if log_path:
            log_dir = os.path.dirname(log_path)
            if log_dir:
                os.makedirs(log_dir, exist_ok=True)
            self._err_log = open(log_path, 'w', buffering=1)  # line-buffered
            self._err_log.write(
                "stamp_s,lat,lon,quality,idx,cross_track_m,at_end,fix_age_s,"
                "heading_deg,heading_src,imu_yaw_deg,offset_deg,go,route_index,route_id,sequence_id,route_connecting,"
                "station_m,station_window_low_m,station_window_high_m,station_v_ref,station_sample_time_s,"
                "preview_requested_station_m,preview_station_m,preview_index,preview_snapped\n")
            self.get_logger().info(f"횡오차 로그: {log_path}")
        self._last_fix_t = None  # 새 GGA 판별용 (gps_fix는 새 측정에만 발행)
        # 자율/수동 구분 로그용 — GO(estop 해제) 발행 중인지. 판단 아님, 기록만.
        # (2026-08-03: 종점 정지 후 조이스틱 이동이 자율 유턴으로 오독된 사례)
        self._go_t = None
        self.sub_estop = self.create_subscription(
            EstopRequest, '/perception/estop', self._on_estop, 1)
        self._station_v_ref = 0.0
        self._station_target_stamp = 0
        self._station_target_received = None
        self._station_target_age = 0.0
        self.sub_target = self.create_subscription(TargetRef, '/adas/target_ref', self._on_target_ref, 1)
        self.sub_session = self.create_subscription(Bool, '/operator/start_session', self._on_start_session, 1)
        self.pub = self.create_publisher(GpsPath, '/perception/gps_path', 1)
        self.sub_route = self.create_subscription(MgmState, '/adas/mgm_state', self._on_route_control, 1) if self._route_plan else None
        # Raw gyro-integrated yaw has an arbitrary zero.  Consumers must use
        # orientation differences, not treat it as an absolute ENU heading.
        self.pub_imu = self.create_publisher(
            Imu, str(p('imu_topic').value), 10)
        # 디버그·시각화용 (MGM 계약 아님): RViz Path + 전역 위치 + TF(map→base_link)
        self.pub_viz = self.create_publisher(Path, '/perception/gps_path_viz', 1)
        self.pub_fix = self.create_publisher(NavSatFix, '/perception/gps_fix', 1)
        self.tf_bc = TransformBroadcaster(self)

        # 기록 트랙 전체 — map(ENU, 트랙 첫 점 원점) 프레임, latched 1회 발행
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_track = self.create_publisher(Path, '/perception/gps_track_viz', latched)
        self.pub_route_geometry = self.create_publisher(GpsRoute, '/perception/gps_route', latched)
        self._publish_track()
        self.timer = self.create_timer(float(p('publish_period').value), self.tick)
        self.status_timer = self.create_timer(2.0, self.report_status)
        self.add_on_set_parameters_callback(self._on_param)

    def _publish_track(self):
        track = Path()
        track.header.frame_id = 'map'
        track.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(self.engine.e)):
            ps = PoseStamped()
            ps.header = track.header
            ps.pose.position.x = float(self.engine.e[i])
            ps.pose.position.y = float(self.engine.n[i])
            if self._route_plan:
                lat, lon = self._route_plan.active_points[i]
                ps.pose.position.x, ps.pose.position.y = self._route_plan.position(lat, lon)
            ps.pose.orientation.z = math.sin(self.engine.yaw[i] / 2.0)
            ps.pose.orientation.w = math.cos(self.engine.yaw[i] / 2.0)
            track.poses.append(ps)
        self.pub_track.publish(track)
        route = GpsRoute()
        route.header = track.header
        self._fill_route(route)
        route.route_id = (route.route.route_id if route.route.enabled else
                          os.path.basename(str(self.get_parameter('waypoint_csv').value)))
        route.points = [RefPoint(x=p.pose.position.x, y=p.pose.position.y,
                                yaw=float(self.engine.yaw[i])) for i, p in enumerate(track.poses)]
        self.pub_route_geometry.publish(route)

    def _on_target_ref(self, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
        if not math.isfinite(msg.v_ref) or stamp <= 0 or not 0 <= age <= self.stale_timeout:
            self._station_v_ref = 0.0
            self._station_target_received = None
            return
        if stamp <= self._station_target_stamp:
            return
        self._station_v_ref = float(msg.v_ref)
        self._station_target_stamp = stamp
        self._station_target_received = time.monotonic()
        self._station_target_age = age

    def _snapshot_at_station(self, lat, lon, heading, fix_t):
        v_ref = 0.0
        if self._station_target_received is not None:
            age = self._station_target_age + time.monotonic() - self._station_target_received
            if 0 <= age <= self.stale_timeout:
                v_ref = self._station_v_ref
        snap = self.engine.snapshot(lat, lon, heading, v_ref=v_ref,
                                    sample_time=self.station_sample_time, generation=fix_t)
        snap['station_v_ref'] = v_ref
        return snap

    def _on_start_session(self, msg):
        if msg.data:
            self.engine.reset_station()
            self._last_snap = None

    def _fill_station_reference(self, msg, snap):
        point, = snap['points']
        x, y, yaw, curvature = map(float, point)
        msg.points = [RefPoint(x=x, y=y, yaw=yaw, curvature=curvature)]
        msg.station_yaw_error_rad = float(snap.get('station_yaw_error_rad', 0.0))
        msg.station_error_valid = bool(snap.get('station_error_valid', False))
        msg.waypoint_station_m = float(snap['station_m'])
        msg.waypoint_stations = [float(s) for s in snap['waypoint_stations']]
        msg.waypoint_points = [RefPoint(x=float(x), y=float(y), yaw=float(yaw), curvature=float(k))
                               for x, y, yaw, k in snap['waypoint_points']]
        msg.waypoint_window_valid = len(msg.waypoint_points) >= 2

    def _on_route_control(self, msg):
        plan = self._route_plan
        if plan is None or not msg.route.enabled or msg.route.phase != msg.route.WAIT_ACK:
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
        if stamp <= 0 or not 0 <= age <= self.stale_timeout:
            return
        r = msg.route
        if not plan.apply(r.sequence_id, r.instance_id, self._route_instance, r.request_id, r.requested_index, r.requested_connecting):
            return
        self.engine = plan.active_engine
        self.zone_map = plan.active_zones
        self._zone_telemetry_fix = self._zone_telemetry_previous = None
        self._last_snap = None
        self._publish_track()
        self.get_logger().info(f'Route applied: index={plan.index}, id={plan.files[plan.index].id}, connecting={plan.connecting}, request={r.request_id}')

    def _fill_route(self, msg):
        plan = self._route_plan
        if plan is None:
            return
        r = msg.route
        r.enabled = True
        r.connecting, r.next_connecting = plan.connecting, plan.next_connecting
        r.sequence_id, r.instance_id = plan.sequence_id, self._route_instance
        r.index, r.count, r.acknowledged_request = plan.index, len(plan.files), plan.acknowledged_request
        r.required_missions = list(plan.required[plan.index])
        files = plan.files[plan.index]
        r.completion = files.completion
        r.route_id, r.waypoint_csv, r.zones_file = files.id, str(files.csv), str(files.zones)

    def _publish_imu(self, stamp) -> None:
        if self.imu is None:
            return
        yaw_sample = self.imu.latest_yaw_gyro()
        stale_s = float(self.get_parameter('imu_publish_stale_s').value)
        if yaw_sample is None or yaw_sample[1] > stale_s:
            return

        sign = float(self.get_parameter('imu_yaw_sign').value)
        yaw = sign * float(yaw_sample[0])
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = str(self.get_parameter('imu_frame_id').value)
        msg.orientation.z = math.sin(0.5 * yaw)
        msg.orientation.w = math.cos(0.5 * yaw)
        # Roll/pitch are intentionally not sourced from the magnetometer-based
        # AHRS solution.  A large covariance leaves only the relative yaw
        # useful to downstream 2-D localization.
        msg.orientation_covariance = [
            1.0e6, 0.0, 0.0,
            0.0, 1.0e6, 0.0,
            0.0, 0.0, math.radians(1.0) ** 2,
        ]
        gyro = self.imu.latest_gyro_z()
        if gyro is not None and gyro[1] <= stale_s:
            msg.angular_velocity.z = sign * float(gyro[0])
            msg.angular_velocity_covariance = [
                1.0e6, 0.0, 0.0,
                0.0, 1.0e6, 0.0,
                0.0, 0.0, math.radians(0.5) ** 2,
            ]
        else:
            msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        self.pub_imu.publish(msg)

    def tick(self):
        msg = GpsPath()
        self._fill_route(msg)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self._publish_imu(msg.header.stamp)
        msg.dx, msg.dy, msg.dyaw = self._pose_delta_tracker.delta
        msg.update = self._pose_delta_tracker.update

        fix = self.link.latest_fix()
        if fix is None or fix[4] > self.stale_timeout or fix[3] == 0:
            # Do not bridge an unknown outage with one large delta on recovery.
            # The last valid delta/update stays visible while the next valid fix
            # will establish a fresh zero-displacement baseline.
            self._pose_delta_tracker.invalidate()
            msg.fix_quality = 0
            msg.heading_source = GpsPath.HEADING_TANGENT   # fix 없음 → 헤딩도 신뢰 불가
            self.pub.publish(msg)
            return

        lat, lon, height, quality, age, fix_t = fix

        # 헤딩 선택: 융합(IMU+COG, 정지 포함 유효) > COG(이동 중) > 접선 폴백.
        # 융합은 이동 중 COG로 IMU→ENU 오프셋을 맞춘 뒤부터 유효 — 그 전에는
        # 기존 COG/접선 동작과 동일 (출발 전 정렬 필수 — path_engine 참조)
        now = time.monotonic()
        cog = self.link.latest_cog()
        # COG 유효 판정에 히스테리시스 — 속도가 문턱(0.5m/s)에 걸치면 10Hz로
        # COG↔접선이 깜빡이며 발행 프레임이 흔들린다 (2026-08-03 출발 구간 실사례)
        if cog is None or cog[2] > self.cog_max_age:
            self._cog_ok = False
        elif self._cog_ok:
            self._cog_ok = cog[0] >= 0.7 * self.cog_min_speed
        else:
            self._cog_ok = cog[0] >= self.cog_min_speed
        cog_valid = self._cog_ok
        if self.fusion is not None and self.imu is not None:
            gen = self.imu.generation()
            if gen != self._imu_gen:
                self._imu_gen = gen
                if self.fusion.aligned:
                    self.get_logger().warn(
                        "IMU 재연결 감지 — yaw 기준점 리셋 가능성, 헤딩 오프셋 폐기 "
                        "(다음 직진에서 자동 재정렬)")
                self.fusion.reset_alignment()
            yawg = self.imu.latest_yaw_gyro()
            if yawg is not None:
                gz = self.imu.latest_gyro_z()
                self.fusion.update_imu(yawg[0], now - yawg[1],
                                       gyro_z=gz[0] if gz else None)
            if cog_valid:
                self.fusion.update_cog(cog[1], now - cog[2], speed=cog[0])
        fused = self.fusion.heading(now) if self.fusion is not None else None

        if self.fusion is not None and self.fusion.aligned != self._was_aligned:
            self._was_aligned = self.fusion.aligned
            if self._was_aligned:
                self.get_logger().info(
                    f"융합 헤딩 정렬: offset {math.degrees(self.fusion.offset):+.1f}°")
            else:
                self.get_logger().warn("융합 정렬 해제 — 직진 주행으로 재정렬 대기")

        if fused is not None:
            heading, self._heading_src = fused, '융합'
        elif cog_valid:
            heading, self._heading_src = cog[1], 'COG'
        else:
            heading, self._heading_src = None, '접선'

        try:
            snap = self._snapshot_at_station(lat, lon, heading, fix_t)
        except ValueError as exc:
            self.get_logger().error(f'GPS station sample rejected: {exc}')
            msg.fix_quality = 0
            self.pub.publish(msg)
            return
        yaw = heading if heading is not None else self.engine.yaw[snap['idx']]
        msg.heading_source = (
            GpsPath.HEADING_FUSED if self._heading_src == '융합'
            else GpsPath.HEADING_COG if self._heading_src == 'COG'
            else GpsPath.HEADING_TANGENT)
        # The GGA link can return the same latest sample on more than one timer
        # tick.  Count and calculate motion only once per actual GNSS sample.
        local_position = self.engine.to_enu(lat, lon)
        east, north = self._route_plan.position(lat, lon) if self._route_plan else local_position
        msg.position_valid = True
        msg.position_x, msg.position_y = float(east), float(north)
        msg.track_index = int(snap['idx'])
        delta, update = self._pose_delta_tracker.consume(
            fix_t, (east, north, yaw), msg.heading_source)
        msg.dx, msg.dy, msg.dyaw = delta
        msg.update = update
        self._set_reference_stamp(msg, fix_t)
        self._fill_station_reference(msg, snap)
        msg.accel_zone = snap['accel_zone']
        msg.parking_zone = snap['parking_zone']
        msg.parking_mode = (
            GpsPath.PARKING_PERPENDICULAR
            if snap['parking_mode'] == 'perpendicular'
            else GpsPath.PARKING_PARALLEL
            if snap['parking_mode'] == 'parallel'
            else GpsPath.PARKING_NONE)
        msg.stop_zone = int(snap['stop_zone'])
        if self._route_plan and msg.stop_zone:
            msg.stop_zone += self._route_plan.stop_offsets[self._route_plan.index]
        msg.avoid_zone = snap['avoid_zone']
        msg.gps_only_zone = snap['gps_only_zone']
        self._fill_zone_context(msg, snap['idx'], snap.get('preview_index'))
        self._fill_zone_telemetry(msg, fix_t, east, north, yaw, heading is not None, local_position)
        msg.at_end = snap['at_end']
        msg.fix_quality = quality
        msg.cross_track_m = float(snap['cross_track_m'])
        # 헤딩 신뢰도를 계약에 실는다 (2026-08-16 신설, GpsPath.msg 주석 참조).
        # MGM이 "이 헤딩을 믿어도 되는가"를 알아야 역방향 가드·재합류를 안전하게
        # 판단할 수 있다 — 접선 폴백은 이탈 상태에서 가정이 깨지고, COG는 저속에서
        # 무작위가 된다.
        self.pub.publish(msg)
        self._last_snap = snap

        if self._err_log is not None:
            t = self.get_clock().now().nanoseconds * 1e-9
            yawg = self.imu.latest_yaw_gyro() if self.imu is not None else None
            imu_deg = (f"{math.degrees(wrap_angle(yawg[0])):.1f}"
                       if yawg else "")
            off = self.fusion.offset if self.fusion is not None else None
            off_deg = f"{math.degrees(off):.1f}" if off is not None else ""
            go = int(self._go_t is not None
                     and time.monotonic() - self._go_t < 0.25)
            self._err_log.write(
                f"{t:.3f},{lat:.7f},{lon:.7f},{quality},{snap['idx']},"
                f"{snap['cross_track_m']:.3f},{int(snap['at_end'])},{age:.3f},"
                f"{math.degrees(yaw):.1f},{self._heading_src},"
                f"{imu_deg},{off_deg},{go},{msg.route.index},{msg.route.route_id},{msg.route.sequence_id},{int(msg.route.connecting)},"
                f"{snap['station_m']:.6f},{snap['station_window_low_m']:.6f},{snap['station_window_high_m']:.6f},"
                f"{snap['station_v_ref']:.6f},{self.station_sample_time:.6f},"
                f"{snap['preview_requested_station_m']:.6f},{snap['preview_station_m']:.6f},"
                f"{snap['preview_index']},{int(snap['preview_snapped'])}\n")

        viz = Path()
        viz.header = msg.header
        for x, y, pt_yaw, _ in snap['points']:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.z = math.sin(pt_yaw / 2.0)
            ps.pose.orientation.w = math.cos(pt_yaw / 2.0)
            viz.poses.append(ps)
        self.pub_viz.publish(viz)

        # 새 GGA 측정일 때만 발행 — rosbag의 gps_fix 간격 = 실제 GPS 갱신 주기
        if fix_t != self._last_fix_t:
            self._last_fix_t = fix_t
            nsf = NavSatFix()
            nsf.header.stamp = msg.header.stamp
            nsf.header.frame_id = 'base_link'
            nsf.latitude, nsf.longitude, nsf.altitude = lat, lon, height
            nsf.status.status = 0 if quality > 0 else -1  # STATUS_FIX / NO_FIX
            self.pub_fix.publish(nsf)

        # TF map(ENU) → base_link: 위치 = fix, 헤딩 = 엔진과 동일 소스(COG/접선)
        ev, nv = east, north
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = ev, nv
        tf.transform.rotation.z = math.sin(yaw / 2.0)
        tf.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_bc.sendTransform(tf)

    def _on_estop(self, msg):
        # estop=false 하트비트(manual_go/stack_estop)가 살아 있는 동안만 GO
        self._go_t = time.monotonic() if not msg.estop else None

    def _set_reference_stamp(self, msg, fix_t):
        # Reuse the existing actual GNSS sample identity; timer ticks do not renew it.
        if fix_t != self._reference_fix_time:
            self._reference_fix_time = fix_t
            sample_age = time.monotonic() - fix_t
            self._reference_stamp = (
                (self.get_clock().now() - Duration(seconds=sample_age)).to_msg()
                if math.isfinite(sample_age) and sample_age >= 0.0 else None)
        if self._reference_stamp is not None:
            msg.reference_stamp = self._reference_stamp

    def _fill_zone_context(self, msg, current_position_index, preview_position_index=None):
        """Publish memberships; only GPS-only Zones may use preview OR station."""
        msg.zone_valid = True
        for definition, in_zone in self.zone_map.snapshot(
                current_position_index, preview_position_index):
            context = ZoneContext()
            context.zone_valid = True
            context.zone_id = definition.zone_id
            context.zone_type = int(definition.zone_type)
            context.mission_type = int(definition.mission_type)
            context.mission_id = definition.mission_id
            context.in_zone = in_zone
            context.raw_in_zone = in_zone
            msg.zones.append(context)

    def _fill_zone_telemetry(self, msg, fix_t, east, north, yaw, heading_valid, local_position=None):
        """Record existing localization observations, without altering nearest/path selection."""
        if fix_t != self._zone_telemetry_fix:
            previous = self._zone_telemetry_previous
            self._zone_telemetry_values = (
                previous[0] if previous else -1,
                math.hypot(east-previous[1], north-previous[2]) if previous else 0.0,
                yaw, heading_valid)
            self._zone_telemetry_previous = (msg.track_index, east, north)
            self._zone_telemetry_fix = fix_t
        (msg.previous_track_index, msg.position_step_m,
         msg.vehicle_heading_rad, msg.vehicle_heading_valid) = self._zone_telemetry_values
        definitions = {z.zone_id: z for z in self.zone_map.definitions}
        local_e, local_n = local_position if local_position is not None else (east, north)
        for context in msg.zones:
            zone = definitions[context.zone_id]
            context.boundary_distance_m = min(
                math.hypot(local_e-self.engine.e[i], local_n-self.engine.n[i])
                for i in (zone.start_index, zone.end_index))

    def _setup_zones(self, p):
        """위경도로 준 지정 구간 → 웨이포인트 인덱스 구간 (2026-08-18).

        여기서 하는 일은 **좌표 변환과 검증**뿐이다. "정지 지점에 오면 3초 선다",
        "이 구간에서만 회피한다" 같은 판단은 전부 MGM 스테이트 머신에 있다
        (CLAUDE.md §5.1) — stack_gps 는 "지금 몇 번 구간 안인가"만 싣는다.
        """
        log = _RouteLog(self.get_logger()) if getattr(self, '_route_loading', False) else self.get_logger()
        snap_max = float(p('stop_zone_snap_max_m').value)
        span = float(p('stop_zone_span_m').value)

        file_stops, file_avoid, file_gps_only, file_parking = _load_zones_file(
            p('zones_file').value, log)

        # 주차 지점은 위치만 YAML에 보존하고, 현재 트랙을 로드할 때 짧은 인덱스
        # 구간으로 변환한다. 트랙을 다시 기록해 인덱스가 바뀌어도 장소와 모드는
        # 유지된다. 무엇을 수행할지는 MGM/stack_parking이 결정한다.
        parking_span = float(p('parking_zone_span_m').value)
        perpendicular_ranges = list(self.engine.parking_ranges)
        parallel_ranges = list(self.engine.parallel_parking_ranges)

        def _overlaps(candidate, ranges):
            return any(max(candidate[0], other[0]) <= min(candidate[1], other[1])
                       for other in ranges)

        for k, (lat, lon, mode) in enumerate(file_parking):
            a, b, d = self.engine.range_from_latlon(lat, lon, parking_span)
            if d > snap_max:
                log.error(
                    f"주차 지점 {k + 1} ({lat:.7f},{lon:.7f})이 트랙에서 {d:.1f}m "
                    f"떨어져 있음 (한계 {snap_max:.1f}m) — 다른 코스의 좌표로 보고 **무시**한다")
                continue
            candidate = (a, b)
            own = perpendicular_ranges if mode == 'perpendicular' else parallel_ranges
            other = parallel_ranges if mode == 'perpendicular' else perpendicular_ranges
            if _overlaps(candidate, other):
                log.error(
                    f"주차 지점 {k + 1}의 {mode} 구간 {a}~{b}가 반대 주차 모드와 겹침 "
                    "— 모드가 모호하므로 **무시**한다")
                continue
            if candidate not in own:
                own.append(candidate)
            log.info(f"주차 지점 {k + 1}: {mode} idx {a}~{b} "
                     f"(트랙 스냅 {d:.2f}m, 폭 {parking_span:.1f}m)")

        self.engine.parking_ranges = perpendicular_ranges
        self.engine.parallel_parking_ranges = parallel_ranges

        stop_ranges = []
        for k, (lat, lon) in enumerate(
                file_stops +
                _parse_latlon_spec(p('stop_points_latlon').value, 2,
                                   'stop_points_latlon', log)):
            a, b, d = self.engine.range_from_latlon(lat, lon, span)
            if d > snap_max:
                log.error(
                    f"정지 지점 {k + 1} ({lat:.7f},{lon:.7f})이 트랙에서 {d:.1f}m "
                    f"떨어져 있음 (한계 {snap_max:.1f}m) — 다른 코스의 좌표로 보고 **무시**한다")
                continue
            stop_ranges.append((a, b))
            log.info(f"정지 지점 {len(stop_ranges)}: idx {a}~{b} "
                     f"(트랙 스냅 {d:.2f}m, 폭 {span:.1f}m)")
        # 지점이 서로 붙어 있으면 차가 몇 m 간격으로 두 번 선다 (번호가 다르면
        # MGM 이 각각 한 번씩 정차한다). mark_zone 이 1.5m 안쪽은 갱신으로 합치지만
        # 손으로 편집했거나 애매하게 떨어진 경우가 있으므로 여기서 한 번 더 알린다.
        for i in range(1, len(stop_ranges)):
            gap = stop_ranges[i][0] - stop_ranges[i - 1][1]
            if gap < 10:
                log.warn(f"정지 지점 {i}·{i + 1} 이 웨이포인트 {gap}칸 간격 — "
                         "연속으로 두 번 정차하게 된다 (의도한 것인지 확인)")
        self.engine.stop_ranges = stop_ranges

        # A CSV state=4 marker has no geographic exit: retain its indication
        # through this route and let MGM latch completion after waypoint return.
        avoid_ranges = avoidance_marker_range(p('waypoint_csv').value)
        if avoid_ranges:
            log.info(f"CSV state=4 회피 시작: idx {avoid_ranges[0][0]}; 종료는 MGM waypoint 복귀 판정")
        for lat1, lon1, lat2, lon2 in file_avoid + _parse_latlon_spec(
                p('avoid_zone_latlon').value, 4, 'avoid_zone_latlon', log):
            i1, d1 = self.engine.index_of(lat1, lon1)
            i2, d2 = self.engine.index_of(lat2, lon2)
            if max(d1, d2) > snap_max:
                log.error(
                    f"회피 구간 끝점이 트랙에서 {max(d1, d2):.1f}m 떨어져 있음 "
                    f"(한계 {snap_max:.1f}m) — **무시**한다")
                continue
            a, b = min(i1, i2), max(i1, i2)
            lead = float(p('avoid_zone_lead_m').value)
            a_lead = self.engine.index_before(a, lead)
            avoid_ranges.append((a_lead, b))
            log.info(f"회피 허용 구간 {len(avoid_ranges)}: idx {a_lead}~{b} "
                     f"(찍은 구간 {a}~{b} + 앞쪽 {lead:.1f}m 확장 — "
                     f"감지 거리 안에서 미리 무장해야 회피가 성립한다, 스냅 {d1:.2f}/{d2:.2f}m)")
        self.engine.avoid_ranges = avoid_ranges

        gps_only_ranges = []
        for lat1, lon1, lat2, lon2 in file_gps_only + _parse_latlon_spec(
                p('gps_only_zone_latlon').value, 4, 'gps_only_zone_latlon', log):
            i1, d1 = self.engine.index_of(lat1, lon1)
            i2, d2 = self.engine.index_of(lat2, lon2)
            if max(d1, d2) > snap_max:
                log.error(f"GPS 전용 구간 끝점이 트랙에서 {max(d1, d2):.1f}m 떨어져 있음 "
                          f"(한계 {snap_max:.1f}m) — **무시**한다")
                continue
            gps_only_ranges.append((min(i1, i2), max(i1, i2)))
            log.info(f"GPS 전용 구간 {len(gps_only_ranges)}: idx {min(i1, i2)}~{max(i1, i2)} "
                     f"(스냅 {d1:.2f}/{d2:.2f}m) — 이 구간에서는 차선 전이 없음")
        self.engine.gps_only_ranges = gps_only_ranges
        explicit_zones = load_zone_definitions(p('zones_file').value, self.engine, snap_max)
        self.zone_map = ZoneMap.from_engine(self.engine, explicit_zones)
        for zone in self.zone_map.definitions:
            log.info(f"Zone {zone.zone_id}: {zone.zone_type.name} "
                     f"idx {zone.start_index}~{zone.end_index}, "
                     f"mission={zone.mission_id}/{zone.mission_type.name}")

        if not stop_ranges:
            log.info("지정 정지 지점 없음")
        if not avoid_ranges:
            log.warn("회피 허용 구간 없음 — MGM avoid_zone_only 가 켜져 있으면 "
                     "AVOID 전이가 어디서도 일어나지 않는다 (장애물은 estop 정지)")

    def _on_param(self, params):
        for prm in params:
            if (getattr(self, '_route_plan', None) is not None or
                    prm.name in ('n_points', 'publish_period', 'route_sequence_file', 'route_start_id',
                                 'route_end_id', 'waypoint_csv', 'zones_file', 'ref_lookahead_m') or
                    prm.name.startswith('rejoin_')):
                return SetParametersResult(successful=False,
                    reason='station preview/route configuration is startup-only; legacy rejoin is disabled')
        return SetParametersResult(successful=True)

    def report_status(self):
        fix = self.link.latest_fix()
        rtcm = self.link.rtcm_rate_and_reset() / 2.0
        nmea = self.link.nmea_rate_and_reset() / 2.0
        if fix is None:
            # NMEA 를 함께 찍는다 — RTCM>0 인데 NMEA==0 이면 수신기가 걸린 것이고
            # (USB 리셋 대상), 둘 다 0 이면 시리얼 자체가 끊긴 것이다. 종전 로그는
            # RTCM 만 찍어서 두 고장이 구분되지 않았다 (2026-08-18).
            nrst = self.link.usb_reset_count()
            self.get_logger().warn(
                f"fix 없음 (RTCM {rtcm:.0f}B/s · NMEA {nmea:.0f}B/s"
                + (f" · USB 리셋 {nrst}회" if nrst else "")
                + (" ← 수신기 무응답, USB 리셋 대기" if rtcm > 0 and nmea == 0 else "") + ")")
            return
        _, _, _, quality, age, _ = fix
        qnames = {0: "NOFIX", 1: "GPS", 2: "DGPS", 4: "FIXED", 5: "FLOAT"}
        snap = self._last_snap
        detail = (f"  idx {snap['idx']}  횡오차 {snap['cross_track_m']:.2f}m"
                  + ("  [트랙 끝]" if snap['at_end'] else "")) if snap else ""
        imu_stat = ""
        if self.imu is not None:
            frames, crc_err = self.imu.stats_and_reset()
            if frames == 0:
                imu_stat = "  IMU:없음"
            else:
                off = self.fusion.offset
                align = (f"정렬 {math.degrees(off):+.0f}°" if off is not None
                         else "미정렬(직진 주행 필요)")
                imu_stat = (f"  IMU:{frames / 2.0:.0f}Hz {align}"
                            + (f" CRC오류 {crc_err}" if crc_err else "")
                            + (f" 잔차거부 {self.fusion.rejected}"
                               if self.fusion.rejected else "")
                            + (f" 재정렬 {self.fusion.reseeds}"
                               if self.fusion.reseeds else ""))
        sats, hdop = self.link.latest_sat_info()
        self.get_logger().info(
            f"{qnames.get(quality, quality)}  위성 {sats}개 HDOP {hdop:.1f}"
            f"  age {age:.1f}s  RTCM {rtcm:.0f}B/s"
            f"  헤딩:{self._heading_src}{imu_stat}{detail}")


def main(args=None):
    rclpy.init(args=args)
    node = StackGpsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.link.stop()
        if node.imu is not None:
            node.imu.stop()
        if node._err_log is not None:
            node._err_log.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
