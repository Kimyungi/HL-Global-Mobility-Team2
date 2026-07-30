#!/usr/bin/env python3
"""LiDAR 장착 캘리브레이션 — front_center(=a*), psi_l, x_l 측정 및 검증.

측정 절차서 M7 항목의 4~5단계(직선 피팅)를 자동화한다. 정렬(1단계)과
D_axle 측정(2단계)은 줄자로 직접 해야 한다.

  preview  스캔을 각도별로 훑어 벽이 어느 구간에 있는지 확인
  yaw      벽에 직선을 피팅해 a*(=front_center)와 D_lidar 산출 (40스캔 후 종료)
  live     yaw 를 끊지 않고 반복하며 결과를 RViz 마커로 발행 (정렬 작업용)
  corner   직각 코너에서 변환 결과를 검증 (두 벽이 90°인지, 거리가 줄자와 맞는지)

사용 예:
  python3 calibrate_lidar.py preview
  python3 calibrate_lidar.py yaw --d-axle 1.82
  python3 calibrate_lidar.py live --center-deg -82 --d-axle 1.80
  python3 calibrate_lidar.py corner --front-center -25.3 --xl 0.30 --yl 0.02 \\
      --d-front 1.45 --d-left 0.92

주의: 이 도구는 판단을 하지 않는다. 숫자만 낸다. 합격 여부는 절차서 기준으로 사람이 본다.
"""
import argparse
import math
import sys

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


def wrap(a):
    """(-pi, pi] 로 정규화."""
    return np.arctan2(np.sin(a), np.cos(a))


def scan_to_xy(scan, rmin, rmax):
    """LaserScan → (angles, ranges, xy). 무효값·범위 밖 제거. 회전 보정 없음(센서 원본)."""
    n = len(scan.ranges)
    ang = scan.angle_min + np.arange(n) * scan.angle_increment
    rng = np.asarray(scan.ranges, dtype=float)
    ok = (np.isfinite(rng) & (rng >= max(scan.range_min, rmin))
          & (rng <= min(scan.range_max, rmax)))
    ang, rng = ang[ok], rng[ok]
    return ang, rng, np.stack([rng * np.cos(ang), rng * np.sin(ang)], 1)


def fit_line(pts):
    """전축 최소제곱(PCA) 직선 피팅.

    반환 (n, d, rms, cnt) — n은 원점→직선 방향 단위법선, d는 원점~직선 수직거리.
    일반 최소제곱이 아니라 PCA를 쓰는 이유: 벽이 센서 좌표에서 거의 수직이면
    y=ax+b 형태가 발산한다.
    """
    if len(pts) < 8:
        return None
    c = pts.mean(0)
    q = pts - c
    _, vec = np.linalg.eigh(q.T @ q)
    nrm = vec[:, 0]                       # 최소 고윳값 → 법선
    if nrm @ c < 0:
        nrm = -nrm                        # 원점에서 벽 쪽을 향하게
    return nrm, float(nrm @ c), float(np.sqrt(np.mean((q @ nrm) ** 2))), len(pts)


def select_wall(ang, xy, center, span, inlier, refine=2):
    """벽 구간 선택 → 피팅 → 인라이어 재선택을 반복.

    center 가 None 이면 최근접 점 방향을 초기 중심으로 쓴다.
    """
    if len(xy) < 8:
        return None
    if center is None:
        center = ang[int(np.argmin(np.hypot(xy[:, 0], xy[:, 1])))]
    sel = np.abs(wrap(ang - center)) <= span
    if sel.sum() < 8:
        return None
    res = fit_line(xy[sel])
    for _ in range(refine):
        if res is None:
            return None
        nrm, d = res[0], res[1]
        near = np.abs(xy @ nrm - d) <= inlier
        wide = np.abs(wrap(ang - center)) <= span * 1.6
        sel2 = near & wide
        if sel2.sum() < 8:
            break
        res = fit_line(xy[sel2])
        sel = sel2
    return res + (sel,)


class Collector(Node):
    """스캔을 n_scans 개 모으고 종료."""

    def __init__(self, n_scans):
        super().__init__('calibrate_lidar')
        self.want = n_scans
        self.scans = []
        self.create_subscription(LaserScan, '/scan', self._cb, qos_profile_sensor_data)
        self.get_logger().info(f'/scan 구독 — {n_scans}개 수집 대기')

    def _cb(self, msg):
        if len(self.scans) < self.want:
            self.scans.append(msg)
            if len(self.scans) % 10 == 0:
                print(f'  ... {len(self.scans)}/{self.want}', file=sys.stderr)


def collect(n_scans, timeout=30.0):
    rclpy.init()
    node = Collector(n_scans)
    t0 = node.get_clock().now()
    while rclpy.ok() and len(node.scans) < n_scans:
        rclpy.spin_once(node, timeout_sec=0.2)
        if (node.get_clock().now() - t0).nanoseconds * 1e-9 > timeout:
            print(f'\n[!] {timeout}s 안에 {n_scans}개를 못 받았습니다 '
                  f'({len(node.scans)}개 수신). LiDAR 드라이버가 떠 있는지 확인하세요.',
                  file=sys.stderr)
            break
    out = node.scans
    node.destroy_node()
    rclpy.try_shutdown()
    if not out:
        sys.exit('[!] 스캔을 하나도 받지 못했습니다.')
    return out


# ------------------------------------------------------------------ preview
def cmd_preview(a):
    scans = collect(a.scans)
    ang, rng, xy = scan_to_xy(scans[len(scans) // 2], a.rmin, a.rmax)
    print(f'\n스캔 1장: 유효 {len(ang)}점 / 전체 {len(scans[0].ranges)}점')
    print(f'angle_min {math.degrees(scans[0].angle_min):.2f}deg  '
          f'increment {math.degrees(scans[0].angle_increment):.3f}deg  '
          f'range {scans[0].range_min:.3f}~{scans[0].range_max:.2f}m\n')

    step = math.radians(a.bin_deg)
    edges = np.arange(-math.pi, math.pi + step, step)
    idx = np.digitize(ang, edges) - 1
    print(f'{"각도[deg]":>10} {"최소거리[m]":>11}  분포')
    for b in range(len(edges) - 1):
        m = idx == b
        if not m.any():
            continue
        rmin_b = rng[m].min()
        bar = '#' * max(1, int(round(28 * (1.0 - min(rmin_b, a.rmax) / a.rmax))))
        c = math.degrees((edges[b] + edges[b + 1]) / 2)
        print(f'{c:10.1f} {rmin_b:11.3f}  {bar}')

    k = int(np.argmin(rng))
    print(f'\n최근접: {math.degrees(ang[k]):+.2f}deg 에서 {rng[k]:.3f}m')
    print('→ 이 각도가 벽 방향이면 yaw 모드를 그대로 쓰고,')
    print('   아니면 --center-deg 로 벽 구간 중심을 직접 지정하세요.')


# ---------------------------------------------------------------------- yaw
def cmd_yaw(a):
    scans = collect(a.scans)
    center = None if a.center_deg is None else math.radians(a.center_deg)
    span = math.radians(a.span_deg)

    rows = []
    for s in scans:
        ang, rng, xy = scan_to_xy(s, a.rmin, a.rmax)
        r = select_wall(ang, xy, center, span, a.inlier)
        if r is None:
            continue
        nrm, d, rms, cnt, _ = r
        rows.append((math.degrees(math.atan2(nrm[1], nrm[0])), d, rms, cnt))
    if not rows:
        sys.exit('[!] 벽을 찾지 못했습니다. preview 로 확인 후 --center-deg 를 주세요.')

    arr = np.array(rows)
    a_star, d_lidar = arr[:, 0].mean(), arr[:, 1].mean()
    print(f'\n{"="*58}\n벽 직선 피팅 결과  ({len(rows)}/{len(scans)} 스캔 성공)\n{"="*58}')
    print(f'  a*        = {a_star:+9.4f} deg   (std {arr[:, 0].std():.4f}, '
          f'p-p {arr[:, 0].ptp():.4f})')
    print(f'  D_lidar   = {d_lidar:9.4f} m     (std {arr[:, 1].std():.5f})')
    print(f'  피팅 잔차   = {arr[:, 2].mean() * 1000:9.2f} mm rms')
    print(f'  사용 점 수  = {arr[:, 3].mean():9.1f} 개/스캔')

    print(f'\n  → front_center = {a_star:+.4f} deg')
    print(f'  → psi_l        = {-a_star:+.4f} deg')

    if a.d_axle is not None:
        xl = a.d_axle - d_lidar
        print(f'  → x_l          = {a.d_axle:.4f} - {d_lidar:.4f} = {xl:+.4f} m')

    print(f'\n{"-"*58}\n판정 (절차서 기준으로 직접 확인)')
    print(f'  · a* 표준편차 {arr[:, 0].std():.4f}deg  — 0.1deg 넘으면 벽 구간 선택이 나쁘거나')
    print( '                                    차가 흔들리고 있습니다')
    print(f'  · 잔차 {arr[:, 2].mean() * 1000:.2f}mm      — 20mm 넘으면 벽이 평평하지 않거나')
    print( '                                    다른 물체가 섞였습니다')
    if a.d_axle is not None:
        print('  · x_l 을 M4의 줄자값과 대조 — 5mm 넘게 차이나면 정렬부터 다시')
    else:
        print('  · --d-axle 을 주면 x_l 까지 계산합니다')


# --------------------------------------------------------------------- live
class LiveFitter(Node):
    """매 스캔 벽을 피팅해 터미널에 찍고 RViz 마커로 발행한다. 종료는 Ctrl+C."""

    def __init__(self, a):
        super().__init__('calibrate_lidar_live')
        self.a = a
        self.center = None if a.center_deg is None else math.radians(a.center_deg)
        self.span = math.radians(a.span_deg)
        self.hist = []
        self.n = 0
        self.pub = self.create_publisher(MarkerArray, '/calib/markers', 1)
        self.create_subscription(LaserScan, '/scan', self._cb, qos_profile_sensor_data)
        print('벽 피팅 중 — Ctrl+C 로 종료. RViz 에서 /calib/markers 를 켜세요.\n')
        print(f'{"#":>5} {"a*[deg]":>10} {"평균":>10} {"std":>7} '
              f'{"D_lidar":>9} {"잔차mm":>7} {"점":>5}')

    def _cb(self, scan):
        ang, rng, xy = scan_to_xy(scan, self.a.rmin, self.a.rmax)
        r = select_wall(ang, xy, self.center, self.span, self.a.inlier)
        if r is None:
            return
        nrm, d, rms, cnt, sel = r
        a_star = math.degrees(math.atan2(nrm[1], nrm[0]))
        self.hist.append(a_star)
        if len(self.hist) > 100:
            self.hist.pop(0)
        self.n += 1
        h = np.array(self.hist)
        if self.n % 5 == 0:
            line = (f'{self.n:5d} {a_star:10.4f} {h.mean():10.4f} {h.std():7.4f} '
                    f'{d:9.4f} {rms * 1000:7.2f} {cnt:5d}')
            if self.a.d_axle is not None:
                line += f'   x_l={self.a.d_axle - d:+.4f}'
            print(line)
        self.pub.publish(self._markers(scan, nrm, d, rms, cnt, a_star, h, xy[sel]))

    def _markers(self, scan, nrm, d, rms, cnt, a_star, h, pts):
        frame = scan.header.frame_id or 'laser_frame'
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        def base(i, typ, ns):
            m = Marker()
            m.header.frame_id, m.header.stamp = frame, now
            m.ns, m.id, m.type, m.action = ns, i, typ, Marker.ADD
            m.pose.orientation.w = 1.0
            m.lifetime.sec = 1
            return m

        foot = nrm * d                      # 원점에서 직선에 내린 발
        tang = np.array([-nrm[1], nrm[0]])  # 벽 방향

        # 1) 피팅된 벽 직선
        m = base(0, Marker.LINE_STRIP, 'wall')
        m.scale.x = 0.02
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.42, 0.07, 0.95
        for t in (-2.5, 2.5):
            p = Point(); p.x, p.y = float(foot[0] + t * tang[0]), float(foot[1] + t * tang[1])
            m.points.append(p)
        arr.markers.append(m)

        # 2) 법선 화살표 (= 차량 정면이라고 판정한 방향)
        m = base(1, Marker.ARROW, 'normal')
        m.scale.x, m.scale.y, m.scale.z = 0.02, 0.05, 0.06
        m.color.r, m.color.g, m.color.b, m.color.a = 0.15, 0.85, 0.55, 1.0
        p0 = Point(); p1 = Point()
        p1.x, p1.y = float(foot[0]), float(foot[1])
        m.points = [p0, p1]
        arr.markers.append(m)

        # 3) 피팅에 쓴 점들
        m = base(2, Marker.POINTS, 'inliers')
        m.scale.x = m.scale.y = 0.025
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.85, 0.25, 1.0
        for q in pts:
            p = Point(); p.x, p.y = float(q[0]), float(q[1])
            m.points.append(p)
        arr.markers.append(m)

        # 4) LiDAR 0도 기준선 — a* 를 눈으로 가늠하는 기준
        m = base(3, Marker.LINE_STRIP, 'zero')
        m.scale.x = 0.008
        m.color.r = m.color.g = m.color.b = 0.6
        m.color.a = 0.8
        for t in (0.0, 1.0):
            p = Point(); p.x = float(t)
            m.points.append(p)
        arr.markers.append(m)

        # 5) 수치 오버레이
        m = base(4, Marker.TEXT_VIEW_FACING, 'text')
        m.pose.position.x, m.pose.position.y, m.pose.position.z = 0.0, 0.0, 0.55
        m.scale.z = 0.13
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        txt = (f'a* = {a_star:+.3f} deg  (avg {h.mean():+.3f}, std {h.std():.3f})\n'
               f'front_center = {a_star:+.3f} | psi_l = {-a_star:+.3f}\n'
               f'D_lidar = {d:.4f} m   rms {rms * 1000:.1f} mm   n={cnt}')
        if self.a.d_axle is not None:
            txt += f'\nx_l = {self.a.d_axle:.4f} - {d:.4f} = {self.a.d_axle - d:+.4f} m'
        m.text = txt
        arr.markers.append(m)
        return arr


def cmd_live(a):
    rclpy.init()
    node = LiveFitter(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node.hist:
        h = np.array(node.hist)
        print(f'\n{"="*58}\n최근 {len(h)}개 평균\n{"="*58}')
        print(f'  a*           = {h.mean():+9.4f} deg   (std {h.std():.4f})')
        print(f'  front_center = {h.mean():+9.4f} deg')
        print(f'  psi_l        = {-h.mean():+9.4f} deg')
    node.destroy_node()
    rclpy.try_shutdown()


# ------------------------------------------------------------------- corner
def cmd_corner(a):
    scans = collect(a.scans)
    psi = math.radians(-a.front_center)          # psi_l = -front_center
    cs, sn = math.cos(psi), math.sin(psi)
    span = math.radians(a.span_deg)

    res = []
    for s in scans:
        ang, rng, xy = scan_to_xy(s, a.rmin, a.rmax)
        r1 = select_wall(ang, xy, None, span, a.inlier)
        if r1 is None:
            continue
        keep = ~r1[4]
        if keep.sum() < 20:
            continue
        r2 = select_wall(ang[keep], xy[keep], None, span, a.inlier)
        if r2 is None:
            continue
        # base_link 로 변환한 뒤 다시 피팅 (변환이 맞는지 보는 것이 목적)
        walls = []
        for r, sel in ((r1, r1[4]), (r2, None)):
            pts = xy[sel] if sel is not None else xy[keep][r2[4]]
            bx = a.xl + cs * pts[:, 0] - sn * pts[:, 1]
            by = a.yl + sn * pts[:, 0] + cs * pts[:, 1]
            walls.append(fit_line(np.stack([bx, by], 1)))
        n1, d1 = walls[0][0], walls[0][1]
        n2, d2 = walls[1][0], walls[1][1]
        ang_between = math.degrees(math.acos(min(1.0, abs(float(n1 @ n2)))))
        res.append((90.0 - ang_between, d1, d2,
                    math.degrees(math.atan2(n1[1], n1[0])),
                    math.degrees(math.atan2(n2[1], n2[0]))))
    if not res:
        sys.exit('[!] 두 벽을 분리하지 못했습니다. 코너가 시야에 다 들어오는지 확인하세요.')

    arr = np.array(res)
    print(f'\n{"="*58}\n코너 검증  ({len(res)}/{len(scans)} 스캔 성공)\n{"="*58}')
    print(f'  적용값: front_center={a.front_center:+.4f}deg  '
          f'x_l={a.xl:+.4f}  y_l={a.yl:+.4f}')
    print(f'\n  두 벽 사이 각    = {90 - arr[:, 0].mean():8.4f} deg  '
          f'(90 에서 {arr[:, 0].mean():+.4f})')
    # 법선 방향으로 어느 쪽이 정면/좌측인지 구분
    for i, (col, lab) in enumerate(((1, '벽 A'), (2, '벽 B'))):
        nd = arr[:, 3 + i].mean()
        side = '정면(x)' if abs(nd) < 45 else ('좌측(y)' if nd > 0 else '우측(-y)')
        print(f'  {lab} 수직거리     = {arr[:, col].mean():8.4f} m   '
              f'법선 {nd:+7.2f}deg → {side}')

    print(f'\n{"-"*58}\n판정')
    err = abs(arr[:, 0].mean())
    print(f'  · 직각 오차 {err:.4f}deg — 0.5deg 넘으면 front_center 가 틀렸습니다')
    if a.d_front is not None:
        print(f'  · 정면 벽: 줄자 {a.d_front:.4f} vs 측정 — 차이가 크면 x_l 이 틀렸습니다')
    if a.d_left is not None:
        print(f'  · 좌측 벽: 줄자 {a.d_left:.4f} vs 측정 — 차이가 크면 y_l 이 틀렸습니다')
    print('  · 차를 제자리에서 90도 돌려 반복해도 값이 유지되어야 x_l, y_l 까지 맞은 것입니다')


def main():
    p = argparse.ArgumentParser(
        description='LiDAR 장착 캘리브레이션 (측정 절차서 M7)',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = p.add_subparsers(dest='cmd', required=True)

    def common(q, n=40):
        q.add_argument('--scans', type=int, default=n, help=f'수집할 스캔 수 (기본 {n})')
        q.add_argument('--rmin', type=float, default=0.15, help='유효 최소거리 [m]')
        q.add_argument('--rmax', type=float, default=6.0, help='유효 최대거리 [m]')

    q = sub.add_parser('preview', help='벽이 어느 각도에 있는지 확인')
    common(q, 5)
    q.add_argument('--bin-deg', type=float, default=5.0, help='표시 각도 폭 [deg]')
    q.set_defaults(fn=cmd_preview)

    q = sub.add_parser('yaw', help='a*(front_center)·D_lidar 측정')
    common(q)
    q.add_argument('--center-deg', type=float, default=None,
                   help='벽 구간 중심 [deg]. 생략하면 최근접 방향')
    q.add_argument('--span-deg', type=float, default=25.0, help='벽 구간 반각 [deg]')
    q.add_argument('--inlier', type=float, default=0.03, help='인라이어 거리 [m]')
    q.add_argument('--d-axle', type=float, default=None,
                   help='뒷축 중심~벽 줄자값 [m]. 주면 x_l 까지 계산')
    q.set_defaults(fn=cmd_yaw)

    q = sub.add_parser('live', help='연속 측정 + RViz 마커 발행')
    common(q)
    q.add_argument('--center-deg', type=float, default=None,
                   help='벽 구간 중심 [deg]. 생략하면 매 스캔 최근접 방향')
    q.add_argument('--span-deg', type=float, default=30.0, help='벽 구간 반각 [deg]')
    q.add_argument('--inlier', type=float, default=0.03, help='인라이어 거리 [m]')
    q.add_argument('--d-axle', type=float, default=None,
                   help='뒷축 중심~벽 줄자값 [m]. 주면 x_l 도 실시간 표시')
    q.set_defaults(fn=cmd_live)

    q = sub.add_parser('corner', help='직각 코너에서 변환 검증')
    common(q)
    q.add_argument('--front-center', type=float, required=True, help='[deg]')
    q.add_argument('--xl', type=float, required=True, help='[m]')
    q.add_argument('--yl', type=float, required=True, help='[m]')
    q.add_argument('--span-deg', type=float, default=30.0)
    q.add_argument('--inlier', type=float, default=0.03)
    q.add_argument('--d-front', type=float, default=None, help='줄자로 잰 정면 벽 거리 [m]')
    q.add_argument('--d-left', type=float, default=None, help='줄자로 잰 좌측 벽 거리 [m]')
    q.set_defaults(fn=cmd_corner)

    a = p.parse_args()
    a.fn(a)


if __name__ == '__main__':
    main()
