#!/usr/bin/env python3
"""두 라이다가 같은 공간을 같은 공간으로 보는가 — 겹침 정합 측정.

주차 로컬리제이션은 여러 라이다의 스캔을 하나의 로컬맵으로 합쳐 쓴다. 그 전제가
**두 스캔이 공통 좌표계에서 겹쳤을 때 같은 물체가 한 겹으로 보이는 것**이다.
외부 파라미터(상대 위치·자세)가 틀리면 같은 벽이 두 겹으로 갈라진다(이중벽).

이 도구는 SLAM 이전 단계를 담당한다 — SLAM 을 돌려서 이중벽이 나오면 SLAM 탓인지
캘리브 탓인지 구분되지 않으므로, 캘리브부터 수치로 확정한다.

  residual  현재 TF 기준으로 두 스캔의 정합 잔차·겹침률을 측정
  icp       잔차를 최소화하는 보정 변환을 추정 (줄자값 검증 / 캘리브 산출)

사용 예:
  python3 lidar_overlap.py residual
  python3 lidar_overlap.py residual --topic-a /lidar_a/scan --topic-b /lidar_b/scan --frame bench
  python3 lidar_overlap.py icp --max-corr 0.30

주의: 이 도구는 판단을 하지 않는다. 숫자만 낸다. 합격 여부는 사람이 정한다.
"""
import argparse
import math
import sys

import numpy as np
from scipy.spatial import cKDTree

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import tf2_ros


def scan_xy(scan, rmin, rmax):
    """LaserScan → 센서 좌표계 (N,2) 점군. 무효값 제거."""
    n = len(scan.ranges)
    ang = scan.angle_min + np.arange(n) * scan.angle_increment
    r = np.asarray(scan.ranges, dtype=float)
    ok = (np.isfinite(r) & (r >= max(scan.range_min, rmin))
          & (r <= min(scan.range_max, rmax)))
    ang, r = ang[ok], r[ok]
    return np.stack([r * np.cos(ang), r * np.sin(ang)], 1)


def se2(tx, ty, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, tx], [s, c, ty], [0.0, 0.0, 1.0]])


def apply(T, pts):
    return (pts @ T[:2, :2].T) + T[:2, 2]


class Grabber(Node):
    """두 스캔과 그 사이 TF 를 한 벌 잡아온다."""

    def __init__(self, a):
        super().__init__('lidar_overlap')
        self.a = a
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.sa, self.sb = [], []
        self.create_subscription(LaserScan, a.topic_a,
                                 lambda m: self.sa.append(m), qos_profile_sensor_data)
        self.create_subscription(LaserScan, a.topic_b,
                                 lambda m: self.sb.append(m), qos_profile_sensor_data)

    def grab(self, n, timeout=25.0):
        t0 = self.get_clock().now()
        while rclpy.ok() and (len(self.sa) < n or len(self.sb) < n):
            rclpy.spin_once(self, timeout_sec=0.2)
            if (self.get_clock().now() - t0).nanoseconds * 1e-9 > timeout:
                break
        if not self.sa or not self.sb:
            sys.exit(f'[!] 스캔 수신 실패 (A {len(self.sa)}, B {len(self.sb)}). '
                     '두 드라이버가 다 떠 있는지 확인하세요.')
        return self.sa[:n], self.sb[:n]

    def tf_to(self, frame, child):
        """frame ← child 변환을 SE(2) 행렬로. 실패 시 None."""
        try:
            t = self.buf.lookup_transform(frame, child, rclpy.time.Time())
        except Exception as e:                                   # noqa: BLE001
            print(f'[!] TF {frame} ← {child} 조회 실패: {e}', file=sys.stderr)
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        return se2(t.transform.translation.x, t.transform.translation.y, yaw)


def stack_points(scans, T, rmin, rmax):
    """여러 스캔을 공통 좌표계로 옮겨 하나의 점군으로."""
    out = [apply(T, scan_xy(s, rmin, rmax)) for s in scans]
    return np.vstack([p for p in out if len(p)])


def residual_stats(pa, pb, max_corr):
    """A→B 최근접 이웃 거리 분포. 겹침률 = 대응점을 찾은 비율."""
    if len(pa) == 0 or len(pb) == 0:
        return None
    d, _ = cKDTree(pb).query(pa, k=1)
    inl = d <= max_corr
    return {
        'n': len(pa), 'overlap': float(inl.mean()),
        'med': float(np.median(d[inl])) if inl.any() else float('nan'),
        'p90': float(np.percentile(d[inl], 90)) if inl.any() else float('nan'),
        'rms': float(np.sqrt(np.mean(d[inl] ** 2))) if inl.any() else float('nan'),
    }


def icp(pa, pb, max_corr, iters=40, tol=1e-6):
    """점-점 ICP (SE2). pa 를 pb 에 맞추는 보정 변환을 반환."""
    tree = cKDTree(pb)
    T = np.eye(3)
    cur = pa.copy()
    prev = None
    for _ in range(iters):
        d, idx = tree.query(cur, k=1)
        m = d <= max_corr
        if m.sum() < 20:
            break
        src, dst = cur[m], pb[idx[m]]
        cs, cd = src.mean(0), dst.mean(0)
        H = (src - cs).T @ (dst - cd)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:                      # 반사 제거
            Vt[1] *= -1
            R = Vt.T @ U.T
        step = np.eye(3)
        step[:2, :2] = R
        step[:2, 2] = cd - R @ cs
        T = step @ T
        cur = apply(step, cur)
        err = float(np.sqrt(np.mean(d[m] ** 2)))
        if prev is not None and abs(prev - err) < tol:
            break
        prev = err
    return T, prev


def collect(a):
    rclpy.init()
    node = Grabber(a)
    scans_a, scans_b = node.grab(a.scans)
    Ta = node.tf_to(a.frame, scans_a[0].header.frame_id)
    Tb = node.tf_to(a.frame, scans_b[0].header.frame_id)
    node.destroy_node()
    rclpy.try_shutdown()
    if Ta is None or Tb is None:
        sys.exit('[!] TF 를 못 읽었습니다. static_transform_publisher 가 떠 있는지 확인하세요.')
    pa = stack_points(scans_a, Ta, a.rmin, a.rmax)
    pb = stack_points(scans_b, Tb, a.rmin, a.rmax)
    return scans_a, scans_b, pa, pb


def cmd_residual(a):
    sa, sb, pa, pb = collect(a)
    print(f'\n{"="*66}\n겹침 정합   frame={a.frame}   {a.scans}스캔씩 누적\n{"="*66}')
    print(f'  A {sa[0].header.frame_id}: {len(pa)}점   '
          f'B {sb[0].header.frame_id}: {len(pb)}점')

    for lab, x, y in (('A→B', pa, pb), ('B→A', pb, pa)):
        st = residual_stats(x, y, a.max_corr)
        print(f'\n  [{lab}]  대응 반경 {a.max_corr*100:.0f}cm')
        print(f'    겹침률   {st["overlap"]*100:6.2f} %   '
              f'({int(st["overlap"]*st["n"])}/{st["n"]}점)')
        print(f'    잔차     중앙 {st["med"]*1000:6.1f} mm   '
              f'p90 {st["p90"]*1000:6.1f} mm   rms {st["rms"]*1000:6.1f} mm')

    print(f'\n{"-"*66}\n판정 (사람이 확인)')
    print('  · 겹침률이 높고 잔차 중앙값이 센서 노이즈 수준(수 mm~1cm)이면 → 정합 양호.')
    print('  · 잔차가 수 cm 이상이거나 p90 이 크게 벌어지면 → 외부 파라미터가 틀렸다.')
    print('    RViz 에서 같은 벽이 두 겹으로 갈라져 보이는지 함께 확인할 것.')
    print('  · icp 모드로 보정 변환을 뽑아 줄자값과 대조하면 원인이 확정된다.')


def cmd_icp(a):
    sa, sb, pa, pb = collect(a)
    st0 = residual_stats(pa, pb, a.max_corr)
    T, err = icp(pa, pb, a.max_corr, a.iters)
    st1 = residual_stats(apply(T, pa), pb, a.max_corr)

    dx, dy = T[0, 2], T[1, 2]
    dyaw = math.degrees(math.atan2(T[1, 0], T[0, 0]))
    print(f'\n{"="*66}\nICP 보정 추정   frame={a.frame}\n{"="*66}')
    print('  A 점군에 적용해야 할 보정 (현재 TF 가 이만큼 틀렸다는 뜻):')
    print(f'    dx   = {dx*1000:+9.1f} mm')
    print(f'    dy   = {dy*1000:+9.1f} mm')
    print(f'    dyaw = {dyaw:+9.3f} deg')
    print(f'\n  잔차 rms  {st0["rms"]*1000:.1f} mm  →  {st1["rms"]*1000:.1f} mm')
    print(f'  겹침률    {st0["overlap"]*100:.2f} %  →  {st1["overlap"]*100:.2f} %')
    print(f'\n{"-"*66}\n판정 (사람이 확인)')
    print('  · 보정량이 mm 수준이면 현재 외부 파라미터가 맞다.')
    print('  · cm 수준이면 줄자값을 그만큼 고쳐야 한다 — 다만 ICP 는 대칭적인 장면'
          '(빈 복도 등)에서 미끄러지므로, 특징이 있는 장소에서 재확인할 것.')
    print('  · 이 값은 두 라이다의 상대 변환 보정이다. 차량 좌표 원점과는 별개.')


def main():
    p = argparse.ArgumentParser(
        description='두 라이다의 겹침 정합 측정 (SLAM 이전 캘리브 검증)',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = p.add_subparsers(dest='cmd', required=True)

    def common(q):
        q.add_argument('--topic-a', default='/lidar_a/scan')
        q.add_argument('--topic-b', default='/lidar_b/scan')
        q.add_argument('--frame', default='bench', help='공통 좌표계 (기본 bench)')
        q.add_argument('--scans', type=int, default=10, help='누적 스캔 수')
        q.add_argument('--rmin', type=float, default=0.20,
                       help='유효 최소거리 [m] — 상대 라이다 본체를 빼려면 크게')
        q.add_argument('--rmax', type=float, default=8.0, help='유효 최대거리 [m]')
        q.add_argument('--max-corr', type=float, default=0.20,
                       help='대응점으로 인정할 최대 거리 [m]')

    q = sub.add_parser('residual', help='현재 TF 기준 정합 잔차·겹침률')
    common(q)
    q.set_defaults(fn=cmd_residual)

    q = sub.add_parser('icp', help='잔차를 최소화하는 보정 변환 추정')
    common(q)
    q.add_argument('--iters', type=int, default=40)
    q.set_defaults(fn=cmd_icp)

    a = p.parse_args()
    a.fn(a)


if __name__ == '__main__':
    main()
