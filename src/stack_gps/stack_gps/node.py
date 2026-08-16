"""stack_gps 노드 — 웨이포인트 CSV + RTK 로버(GGA) → /perception/gps_path
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
      -p waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_track_A.csv \
      -p rtcm_host:=100.70.198.29
"""
import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from fma_interfaces.msg import EstopRequest, GpsPath, RefPoint
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster

from stack_gps.gga_link import GgaLink
from stack_gps.heading_fusion import HeadingFusion
from stack_gps.imu_link import ImuLink
from stack_gps.path_engine import PathEngine, load_waypoints_csv, wrap_angle


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
        # dSPACE는 첫 ref점만 목표로 사용(stack_avoid 실측) — 최근접점(옆구리)을
        # 주면 도달 곡률 폭주로 풀조향 위빙. 전방 lookahead 점을 첫 점으로.
        # 짧을수록 강한 조향(이기돈 회피=0.4), 트랙 추종은 1.0 권장. 0 = 구동작.
        self.declare_parameter('ref_lookahead_m', 1.0)
        # ── 재합류 기하 3종 (2026-08-17에 상수 → 파라미터). 셋 다 **dSPACE 조향이
        # 느리던 시절의 보상값**이라, 조향 특성이 바뀌면 함께 재조정해야 한다.
        # 주행 중 `ros2 param set /stack_gps_node <이름> <값>` 으로 즉시 반영된다
        # (RUNBOOK_avoid_field_test.md §5 "GPS 구간만 오실레이션" 행 참조).
        #   rejoin_rate_damp_s : ψₑ 변화율 선행 보상 [s]. 조향 PI 도입으로 기본 0.
        #   rejoin_full_cross_m: 접근각 α 가 최대(25°)가 되는 횡오차 [m]. 작을수록
        #                        고이득 (0.5 = 50°/m). 올리면 이득이 반비례로 감소.
        #   rejoin_target_max_m: 목표점 거리 상한 [m]. 짧을수록 고이득(κ=2sinb/d).
        #                        ⚠ 이것만 올려도 안 는다 — 실제 거리는
        #                        min(트랙점거리, 상한) 이라 ref_lookahead_m 도 같이.
        #   rejoin_target_min_m: 목표점 거리 **하한** [m] (2026-08-17 신설). 실측상
        #                        거리는 82%가 하한에 붙어 있으므로 **실효 이득을
        #                        정하는 건 사실상 이 값**이다. 올리면 잡음 증폭이
        #                        반비례로 준다. 밴드 [1.27, 1.8] 밖은 미검증.
        #   rejoin_e_lpf_s     : 접근각이 쓰는 횡오차 e 의 저역통과 [s]. 0 = 끔.
        #                        ψₑ 에는 걸지 말 것 — 그쪽은 백색 성분이 0이라
        #                        필터가 실동작만 죽인다(path_engine 클래스 주석).
        self.declare_parameter('rejoin_rate_damp_s', PathEngine.REJOIN_RATE_DAMP_S)
        self.declare_parameter('rejoin_full_cross_m', PathEngine.REJOIN_FULL_CROSS_M)
        self.declare_parameter('rejoin_target_max_m', PathEngine.REJOIN_TARGET_MAX_M)
        self.declare_parameter('rejoin_target_min_m', PathEngine.REJOIN_TARGET_MIN_M)
        self.declare_parameter('rejoin_e_lpf_s', PathEngine.REJOIN_E_LPF_S)
        self.declare_parameter('publish_period', 0.1)
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
        self.declare_parameter('fusion_alpha', 0.1)   # offset 저역통과 이득
        self.declare_parameter('accel_zone_ranges', [0])    # [start,end,...] 인덱스 쌍
        self.declare_parameter('parking_zone_ranges', [0])  # 기본 [0] = 미설정(쌍 안 됨)
        self.declare_parameter('error_log_csv', '')  # 지정 시 매 틱 횡오차 CSV 기록

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

        pts = load_waypoints_csv(csv_path, log=self.get_logger().warn)
        self.engine = PathEngine(pts, n_points=int(p('n_points').value),
                                 accel_ranges=accel, parking_ranges=parking,
                                 lookahead_m=float(p('ref_lookahead_m').value),
                                 rate_damp_s=float(p('rejoin_rate_damp_s').value),
                                 full_cross_m=float(p('rejoin_full_cross_m').value),
                                 target_max_m=float(p('rejoin_target_max_m').value),
                                 target_min_m=float(p('rejoin_target_min_m').value),
                                 e_lpf_s=float(p('rejoin_e_lpf_s').value))
        self.add_on_set_parameters_callback(self._on_param)
        self.get_logger().info(
            f"재합류 기하: lookahead {self.engine.lookahead_m:.2f}m "
            f"rate_damp {self.engine.rate_damp_s:.2f}s "
            f"full_cross {self.engine.full_cross_m:.2f}m "
            f"target_max {self.engine.target_max_m:.2f}m "
            f"target_min {self.engine.target_min_m:.2f}m "
            f"e_lpf {self.engine.e_lpf_s:.2f}s")
        self.get_logger().info(
            f"웨이포인트 {len(pts)}개 로드: {csv_path} "
            f"(accel {accel or '없음'}, parking {parking or '없음'})")

        rtcm_host = p('rtcm_host').value
        if rtcm_host.lower() in ('off', 'none'):  # launch 인자는 빈 값 불가
            rtcm_host = ''
            self.get_logger().warn(
                "RTCM 주입 꺼짐 — 단독 GPS 모드 (RTK 없음, 오차 수 m 예상)")
        self.link = GgaLink(
            serial_port=p('serial_port').value, baud=int(p('baud').value),
            rtcm_host=rtcm_host, rtcm_port=int(p('rtcm_port').value),
            log=lambda m: self.get_logger().info(f"[link] {m}"))
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
                "heading_deg,heading_src,imu_yaw_deg,offset_deg,go\n")
            self.get_logger().info(f"횡오차 로그: {log_path}")
        self._last_fix_t = None  # 새 GGA 판별용 (gps_fix는 새 측정에만 발행)
        # 자율/수동 구분 로그용 — GO(estop 해제) 발행 중인지. 판단 아님, 기록만.
        # (2026-08-03: 종점 정지 후 조이스틱 이동이 자율 유턴으로 오독된 사례)
        self._go_t = None
        self.sub_estop = self.create_subscription(
            EstopRequest, '/perception/estop', self._on_estop, 1)
        self.pub = self.create_publisher(GpsPath, '/perception/gps_path', 1)
        # 디버그·시각화용 (MGM 계약 아님): RViz Path + 전역 위치 + TF(map→base_link)
        self.pub_viz = self.create_publisher(Path, '/perception/gps_path_viz', 1)
        self.pub_fix = self.create_publisher(NavSatFix, '/perception/gps_fix', 1)
        self.tf_bc = TransformBroadcaster(self)

        # 기록 트랙 전체 — map(ENU, 트랙 첫 점 원점) 프레임, latched 1회 발행
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_track = self.create_publisher(Path, '/perception/gps_track_viz', latched)
        track = Path()
        track.header.frame_id = 'map'
        track.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(self.engine.e)):
            ps = PoseStamped()
            ps.header = track.header
            ps.pose.position.x = float(self.engine.e[i])
            ps.pose.position.y = float(self.engine.n[i])
            ps.pose.orientation.z = math.sin(self.engine.yaw[i] / 2.0)
            ps.pose.orientation.w = math.cos(self.engine.yaw[i] / 2.0)
            track.poses.append(ps)
        self.pub_track.publish(track)
        self.timer = self.create_timer(float(p('publish_period').value), self.tick)
        self.status_timer = self.create_timer(2.0, self.report_status)

    def tick(self):
        msg = GpsPath()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        fix = self.link.latest_fix()
        if fix is None or fix[4] > self.stale_timeout or fix[3] == 0:
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

        snap = self.engine.snapshot(lat, lon, heading)
        yaw = heading if heading is not None else self.engine.yaw[snap['idx']]
        for x, y, pt_yaw, curv in snap['points']:
            rp = RefPoint()
            rp.x, rp.y, rp.yaw, rp.curvature = (float(x), float(y),
                                                float(pt_yaw), float(curv))
            msg.points.append(rp)
        msg.accel_zone = snap['accel_zone']
        msg.parking_zone = snap['parking_zone']
        msg.at_end = snap['at_end']
        msg.fix_quality = quality
        msg.cross_track_m = float(snap['cross_track_m'])
        # 헤딩 신뢰도를 계약에 실는다 (2026-08-16 신설, GpsPath.msg 주석 참조).
        # MGM이 "이 헤딩을 믿어도 되는가"를 알아야 역방향 가드·재합류를 안전하게
        # 판단할 수 있다 — 접선 폴백은 이탈 상태에서 가정이 깨지고, COG는 저속에서
        # 무작위가 된다.
        msg.heading_source = (
            GpsPath.HEADING_FUSED if self._heading_src == '융합'
            else GpsPath.HEADING_COG if self._heading_src == 'COG'
            else GpsPath.HEADING_TANGENT)
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
                f"{imu_deg},{off_deg},{go}\n")

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
        ev, nv = self.engine.to_enu(lat, lon)
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

    def _on_param(self, params):
        """재합류 기하 6종의 주행 중 조정 (2026-08-17).

        런북 §5 표의 stack_avoid 노브들과 같은 용법 — 재시작 없이 한 번에 하나씩
        바꿔 보고, 확정값은 launch 기본값에 반영해야 다음 세션에 살아남는다.
        판단이 아니라 기하 이득이라 노드에서 직접 반영한다(경로 소스는 그대로).
        """
        for prm in params:
            v = float(prm.value)
            if prm.name == 'ref_lookahead_m':
                if v < 0.0:
                    return SetParametersResult(
                        successful=False, reason='ref_lookahead_m 은 0 이상')
                self.engine.set_lookahead(v)
            elif prm.name == 'rejoin_rate_damp_s':
                if v < 0.0:
                    return SetParametersResult(
                        successful=False, reason='rejoin_rate_damp_s 는 0 이상')
                self.engine.rate_damp_s = v
            elif prm.name == 'rejoin_full_cross_m':
                if v <= 0.0:
                    return SetParametersResult(
                        successful=False, reason='rejoin_full_cross_m 은 양수')
                self.engine.full_cross_m = v
            elif prm.name == 'rejoin_target_max_m':
                # 하한(2·R_min·sinβ ≈ 1.27m)보다 낮게 잡아도 하한이 이기므로 무의미
                if v <= 0.0:
                    return SetParametersResult(
                        successful=False, reason='rejoin_target_max_m 은 양수')
                self.engine.target_max_m = v
            elif prm.name == 'rejoin_target_min_m':
                if v <= 0.0:
                    return SetParametersResult(
                        successful=False, reason='rejoin_target_min_m 은 양수')
                self.engine.target_min_m = v
            elif prm.name == 'rejoin_e_lpf_s':
                if v < 0.0:
                    return SetParametersResult(
                        successful=False, reason='rejoin_e_lpf_s 는 0 이상 (0=끔)')
                self.engine.e_lpf_s = v
            else:
                continue
            self.get_logger().warn(f"재합류 기하 변경: {prm.name} = {v:.3f}")
        return SetParametersResult(successful=True)

    def report_status(self):
        fix = self.link.latest_fix()
        rtcm = self.link.rtcm_rate_and_reset() / 2.0
        if fix is None:
            self.get_logger().warn(f"fix 없음 (RTCM {rtcm:.0f}B/s)")
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
