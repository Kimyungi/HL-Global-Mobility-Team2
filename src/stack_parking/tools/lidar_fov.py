#!/usr/bin/env python3
"""라이다 유효 시야각(FOV) 측정 — 차체 자기가림 구간을 찾아 사용 섹터를 확정한다.

RViz2에는 각도 눈금 표시 기능이 없다 (Grid 는 직교 격자뿐). 그래서 이 도구가
각도 눈금자·거리 링·라벨을 MarkerArray 로 그려 RViz 화면에 얹는다.

  protractor  각도 눈금자 + 근거리(차체 의심) 점 강조를 실시간 발행 — 사람이 눈으로 읽는 용도
  mask        여러 스캔의 통계로 자기가림 섹터를 자동 판정 — 숫자로 확정하는 용도

측정 원리(mask): **차체 반사는 차가 움직여도 각도별 거리가 변하지 않는다.** 환경 반사는 변한다.
따라서 각도별 최소거리의 (a) 짧음 (b) 스캔 간 변동 없음 (c) 높은 수신율 이 셋이 겹치면 자기가림이다.
차체에 너무 밀착해 아예 무효값이 되는 각도도 있어서 수신율이 바닥인 구간도 함께 가려낸다.

각도 표기는 **스캔 원본 각도**(laser frame, +x = 0deg, 반시계 +) 기준이며
0~360 과 -180~180 을 함께 찍는다. params.yaml 의 `forward_angle_deg`(예 270)는 0~360 쪽 값이다.

사용 예:
  python3 lidar_fov.py protractor --topic /scan_rear
  python3 lidar_fov.py protractor --topic /scan_rear --sector 90 270
  python3 lidar_fov.py mask --topic /scan_rear --scans 80

주의: 이 도구는 판단을 하지 않는다. 숫자만 낸다. 최종 섹터는 사람이 정한다.
"""
import argparse
import math
import sys
import warnings

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


def to360(deg):
    """(-180,180] → [0,360)."""
    return deg % 360.0


def scan_angles_ranges(scan, rmin, rmax):
    """LaserScan → (전체 각도배열, 유효마스크, 거리배열). 무효값은 마스크로만 걸러 인덱스를 보존한다."""
    n = len(scan.ranges)
    ang = scan.angle_min + np.arange(n) * scan.angle_increment
    rng = np.asarray(scan.ranges, dtype=float)
    ok = (np.isfinite(rng) & (rng >= max(scan.range_min, rmin))
          & (rng <= min(scan.range_max, rmax)))
    return ang, ok, rng


def bin_edges(bin_deg):
    step = math.radians(bin_deg)
    return np.arange(-math.pi, math.pi + step * 0.5, step)


def per_bin_min(scan, edges, rmin, rmax):
    """각도 bin 별 최소거리. 유효 반사가 없으면 nan."""
    ang, ok, rng = scan_angles_ranges(scan, rmin, rmax)
    out = np.full(len(edges) - 1, np.nan)
    if not ok.any():
        return out
    idx = np.clip(np.digitize(ang[ok], edges) - 1, 0, len(edges) - 2)
    for b, r in zip(idx, rng[ok]):
        if math.isnan(out[b]) or r < out[b]:
            out[b] = r
    return out


def collect(topic, n_scans, timeout=30.0):
    """스캔 n개 수집 후 반환. 드라이버가 없으면 종료."""
    rclpy.init()
    node = Node('lidar_fov_collect')
    got = []
    node.create_subscription(
        LaserScan, topic, lambda m: got.append(m) if len(got) < n_scans else None,
        qos_profile_sensor_data)
    node.get_logger().info(f'{topic} 구독 — {n_scans}개 수집 대기')
    t0 = node.get_clock().now()
    last = 0
    while rclpy.ok() and len(got) < n_scans:
        rclpy.spin_once(node, timeout_sec=0.2)
        if len(got) >= last + 20:
            last = len(got)
            print(f'  ... {last}/{n_scans}', file=sys.stderr)
        if (node.get_clock().now() - t0).nanoseconds * 1e-9 > timeout:
            print(f'\n[!] {timeout}s 안에 {n_scans}개를 못 받았습니다 ({len(got)}개 수신).',
                  file=sys.stderr)
            break
    node.destroy_node()
    rclpy.try_shutdown()
    if not got:
        sys.exit(f'[!] {topic} 에서 스캔을 하나도 받지 못했습니다. 드라이버가 떠 있는지 확인하세요.')
    return got


def contiguous(flags):
    """원형 배열에서 True 인 구간들을 (시작index, 끝index) 목록으로. 랩어라운드 처리."""
    n = len(flags)
    if flags.all():
        return [(0, n - 1)]
    if not flags.any():
        return []
    start = None
    for i in range(n):                      # False→True 경계를 시작점으로
        if flags[i] and not flags[(i - 1) % n]:
            start = i
            break
    out, i, cnt = [], start, 0
    while cnt < n:
        if flags[i]:
            j = i
            while flags[(j + 1) % n] and (j + 1) % n != start:
                j = (j + 1) % n
                cnt += 1
            out.append((i, j))
            i = (j + 1) % n
        else:
            i = (i + 1) % n
        cnt += 1
    return out


# ----------------------------------------------------------------- protractor
class Protractor(Node):
    """각도 눈금자 + 근거리 점 강조를 RViz 마커로 발행한다. 종료는 Ctrl+C."""

    def __init__(self, a):
        super().__init__('lidar_fov_protractor')
        self.a = a
        self.frame = None
        self.pub = self.create_publisher(MarkerArray, a.marker_topic, 1)
        self.create_subscription(LaserScan, a.topic, self._cb, qos_profile_sensor_data)
        self.n = 0
        print(f'{a.topic} 구독 — RViz 에서 {a.marker_topic} (MarkerArray) 를 켜세요.')
        print(f'눈금 {a.tick_deg}deg / 라벨 {a.label_deg}deg / 근거리 강조 {a.near_m}m 이내')
        print('Ctrl+C 로 종료.\n')

    def _cb(self, scan):
        self.frame = scan.header.frame_id or 'laser_frame'
        self.n += 1
        if self.n % max(1, self.a.every) != 0:
            return
        self.pub.publish(self._build(scan))

    # -- 마커 조립 ---------------------------------------------------------
    def _base(self, mid, typ, ns):
        m = Marker()
        m.header.frame_id, m.header.stamp = self.frame, self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, typ, Marker.ADD
        m.pose.orientation.w = 1.0
        m.lifetime.sec = 2
        return m

    def _build(self, scan):
        a = self.a
        R = a.radius
        arr = MarkerArray()

        # 1) 각도 눈금 — tick_deg 마다 방사선. label_deg 배수는 길고 밝게.
        spokes = self._base(0, Marker.LINE_LIST, 'ticks')
        spokes.scale.x = 0.004
        spokes.color.r, spokes.color.g, spokes.color.b, spokes.color.a = .45, .50, .55, .55
        major = self._base(1, Marker.LINE_LIST, 'ticks_major')
        major.scale.x = 0.010
        major.color.r, major.color.g, major.color.b, major.color.a = .75, .80, .85, .95
        d = a.tick_deg
        for k in range(int(round(360.0 / d))):
            th = math.radians(k * d)
            is_major = abs((k * d) % a.label_deg) < 1e-6
            r0 = 0.0 if is_major else R * 0.90
            p0, p1 = Point(), Point()
            p0.x, p0.y = r0 * math.cos(th), r0 * math.sin(th)
            p1.x, p1.y = R * math.cos(th), R * math.sin(th)
            (major if is_major else spokes).points.extend([p0, p1])
        arr.markers.append(spokes)
        arr.markers.append(major)

        # 2) 각도 라벨 — 0~360 (params 표기) 와 -180~180 을 같이 찍는다
        mid = 10
        for k in range(int(round(360.0 / a.label_deg))):
            deg = k * a.label_deg
            th = math.radians(deg)
            m = self._base(mid, Marker.TEXT_VIEW_FACING, 'labels')
            mid += 1
            m.pose.position.x = (R + a.label_pad) * math.cos(th)
            m.pose.position.y = (R + a.label_pad) * math.sin(th)
            m.scale.z = a.text_size
            signed = deg if deg <= 180 else deg - 360
            m.color.r, m.color.g, m.color.b, m.color.a = .95, .95, .95, 1.0
            m.text = f'{deg:.0f}° ({signed:+.0f})'
            arr.markers.append(m)

        # 3) 거리 링 — 반경을 눈으로 가늠
        mid = 40
        for rr in a.rings:
            if rr > R:
                continue
            m = self._base(mid, Marker.LINE_STRIP, 'rings')
            mid += 1
            m.scale.x = 0.005
            m.color.r, m.color.g, m.color.b, m.color.a = .35, .55, .45, .7
            for t in range(0, 361, 3):
                p = Point()
                p.x, p.y = rr * math.cos(math.radians(t)), rr * math.sin(math.radians(t))
                m.points.append(p)
            arr.markers.append(m)
            t = self._base(mid, Marker.TEXT_VIEW_FACING, 'rings')
            mid += 1
            t.pose.position.x, t.pose.position.y = rr * 0.7071, rr * 0.7071
            t.scale.z = a.text_size * 0.8
            t.color.r, t.color.g, t.color.b, t.color.a = .45, .70, .55, .9
            t.text = f'{rr:g}m'
            arr.markers.append(t)

        # 4) 근거리 점 강조 — 차체 자기가림 후보. 여기가 측정의 핵심 시각 단서다.
        ang, ok, rng = scan_angles_ranges(scan, 0.0, a.max_range)
        near = ok & (rng <= a.near_m)
        m = self._base(60, Marker.POINTS, 'near')
        m.scale.x = m.scale.y = 0.035
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, .25, .25, 1.0
        for th, r in zip(ang[near], rng[near]):
            p = Point()
            p.x, p.y = r * math.cos(th), r * math.sin(th)
            m.points.append(p)
        arr.markers.append(m)

        # 5) 후보 섹터 음영 — --sector 로 준 구간을 초록 부채꼴로
        if a.sector is not None:
            s0, s1 = a.sector
            m = self._base(70, Marker.TRIANGLE_LIST, 'sector')
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = .20, .90, .45, .18
            span = (s1 - s0) % 360.0 or 360.0
            steps = max(2, int(span / 2))
            for i in range(steps):
                t0 = math.radians(s0 + span * i / steps)
                t1 = math.radians(s0 + span * (i + 1) / steps)
                o, p1, p2 = Point(), Point(), Point()
                p1.x, p1.y = R * math.cos(t0), R * math.sin(t0)
                p2.x, p2.y = R * math.cos(t1), R * math.sin(t1)
                m.points.extend([o, p1, p2])
            arr.markers.append(m)

        # 6) 수치 오버레이
        m = self._base(80, Marker.TEXT_VIEW_FACING, 'info')
        m.pose.position.z = 0.4
        m.scale.z = a.text_size
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        n_near = int(near.sum())
        if ok.any():
            rmin_deg = to360(math.degrees(ang[ok][np.argmin(rng[ok])]))
            rmin_txt = f'{rng[ok].min():.3f}m @ {rmin_deg:.1f}°'
        else:
            rmin_txt = '-'
        m.text = (f'{a.topic}   frame={self.frame}\n'
                  f'유효 {int(ok.sum())}/{len(rng)}점   최근접 {rmin_txt}\n'
                  f'근거리(<{a.near_m}m, 차체 의심) {n_near}점')
        if a.sector is not None:
            m.text += f'\n후보 섹터 {a.sector[0]:.0f}° ~ {a.sector[1]:.0f}°'
        arr.markers.append(m)
        return arr


def cmd_protractor(a):
    rclpy.init()
    node = Protractor(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


# ----------------------------------------------------------------------- mask
def cmd_mask(a):
    print('\n[측정 조건] 환경 반사가 스캔마다 변해야 차체 반사와 구분됩니다.')
    print('  → 수집 중 차를 천천히 제자리 회전시키거나, 주변에서 사람이 움직여 주세요.')
    print('  → 사방이 트인 곳(반사 없음)은 피하세요. 벽이 둘러싼 곳이 좋습니다.\n')

    scans = collect(a.topic, a.scans)
    edges = bin_edges(a.bin_deg)
    nb = len(edges) - 1
    mat = np.stack([per_bin_min(s, edges, 0.0, a.max_range) for s in scans])

    # 반사가 하나도 없는 각도 bin 은 전부 nan — nanmedian/nanstd 의 경고는 정상 상황이므로 억제
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        hit = np.mean(np.isfinite(mat), axis=0)
        med = np.nanmedian(mat, axis=0)
        std = np.nanstd(mat, axis=0)
    med = np.nan_to_num(med, nan=np.inf)
    std = np.nan_to_num(std, nan=0.0)

    # 자기가림 판정 — 두 경로
    body = (hit >= a.min_hit) & (med <= a.self_max) & (std <= a.self_std)  # 차체 반사
    dead = hit < a.min_hit                                                 # 무반사(밀착/흡수)
    blocked = body | dead

    print(f'{"="*74}')
    print(f'자기가림 판정  ({len(scans)}스캔 / bin {a.bin_deg}deg / 차체판정 '
          f'거리<={a.self_max}m, std<={a.self_std}m, 수신율>={a.min_hit})')
    print(f'{"="*74}')
    print(f'{"각도[0~360]":>11} {"(부호)":>7} {"수신율":>6} {"중앙거리":>9} {"변동std":>8}  판정')
    for b in range(nb):
        c = math.degrees((edges[b] + edges[b + 1]) / 2)
        tag = '차체' if body[b] else ('무반사' if dead[b] else '')
        if a.only_blocked and not blocked[b]:
            continue
        d = '-' if not np.isfinite(med[b]) else f'{med[b]:9.3f}'
        print(f'{to360(c):11.1f} {c:+7.1f} {hit[b]:6.2f} {d} {std[b]:8.4f}  {tag}')

    usable = contiguous(~blocked)
    print(f'\n{"-"*74}\n사용 가능 섹터 (자기가림 제외)')
    if not usable:
        print('  없음 — 전 각도가 가림으로 판정됐습니다. 측정 조건을 다시 보세요.')
    for s, e in usable:
        a0 = to360(math.degrees(edges[s]))
        a1 = to360(math.degrees(edges[e + 1]))
        span = (a1 - a0) % 360.0 or 360.0
        print(f'  {a0:6.1f}° ~ {a1:6.1f}°   (폭 {span:.1f}°)')
    print(f'\n  → RViz 로 눈으로 확인: python3 lidar_fov.py protractor --topic {a.topic} '
          f'--sector {to360(math.degrees(edges[usable[0][0]])):.0f} '
          f'{to360(math.degrees(edges[usable[0][1] + 1])):.0f}' if usable else '')
    print('  → 경계는 여유를 두고 안쪽으로 몇 도 깎아 쓰는 것을 권합니다 (차체 진동·장착 오차).')


# --------------------------------------------------------------- record/compare
def cmd_record(a):
    """정지 장면을 각도 격자 기준으로 기록한다 (간섭 A/B 시험용).

    ⚠ 이 드라이버는 `fixed_resolution: true` 여도 스캔마다 포인트 수가 달라진다
    (실측 404~430). 따라서 광선 인덱스로는 비교할 수 없고 **고정 각도 격자**에 담아야 한다.
    bin 당 최소거리와 그 점의 강도를 기록한다 (가장 가까운 반사가 유령점 판별에 핵심).
    """
    scans = collect(a.topic, a.scans)
    edges = bin_edges(a.bin_deg)
    nb = len(edges) - 1

    rng = np.full((len(scans), nb), np.nan)
    inten = np.full((len(scans), nb), np.nan)
    for i, s in enumerate(scans):
        ang, ok, r = scan_angles_ranges(s, 0.0, a.max_range)
        if not ok.any():
            continue
        q = (np.asarray(s.intensities, dtype=float)
             if len(s.intensities) == len(r) else np.full(len(r), np.nan))
        idx = np.clip(np.digitize(ang[ok], edges) - 1, 0, nb - 1)
        for b, rr, qq in zip(idx, r[ok], q[ok]):
            if math.isnan(rng[i, b]) or rr < rng[i, b]:
                rng[i, b], inten[i, b] = rr, qq

    centers = (edges[:-1] + edges[1:]) / 2.0
    np.savez_compressed(a.out, ranges=rng, intensities=inten, angles=centers,
                        bin_deg=a.bin_deg, topic=a.topic, n_scans=len(scans))
    print(f'\n저장: {a.out}')
    print(f'  {len(scans)}스캔 x {nb}bin ({a.bin_deg}deg)   '
          f'유효율 {np.mean(np.isfinite(rng))*100:.1f}%   토픽 {a.topic}')


def cmd_compare(a):
    base = np.load(a.base, allow_pickle=True)
    test = np.load(a.test, allow_pickle=True)
    if base['ranges'].shape[1] != test['ranges'].shape[1]:
        sys.exit('[!] 각도 격자가 다릅니다 — 같은 --bin-deg 로 기록한 파일이어야 합니다.')

    ang = base['angles']
    n = len(ang)
    bv = np.mean(np.isfinite(base['ranges']), axis=0)   # bin 별 수신율
    tv = np.mean(np.isfinite(test['ranges']), axis=0)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        bm = np.nanmedian(base['ranges'], axis=0)
        tm = np.nanmedian(test['ranges'], axis=0)
        bi = np.nanmedian(base['intensities'], axis=0)
        ti = np.nanmedian(test['intensities'], axis=0)

    ghost = (bv <= a.quiet) & (tv >= a.active)          # 없던 곳에 새로 생김
    drop = (bv >= a.active) & (tv <= a.quiet)           # 있던 게 사라짐
    both = (bv >= a.active) & (tv >= a.active)
    shift = both & (np.abs(tm - bm) > a.shift_m)        # 거리값이 흔들림

    print(f'\n{"="*74}')
    print(f'간섭 비교   base={a.base}  test={a.test}')
    print(f'{"="*74}')
    print(f'  bin {n}개 ({float(base["bin_deg"]):g}deg)   '
          f'수신율 base {bv.mean()*100:.1f}%  →  test {tv.mean()*100:.1f}%')
    print(f'\n  유령 bin (없던 반사가 생김)  : {int(ghost.sum()):4d}개  ({ghost.sum()/n*100:.2f}%)')
    print(f'  소실 bin (있던 반사가 사라짐): {int(drop.sum()):4d}개  ({drop.sum()/n*100:.2f}%)')
    print(f'  거리 변동 (>{a.shift_m}m)         : {int(shift.sum()):4d}개  ({shift.sum()/n*100:.2f}%)')

    def report(mask, title, show_range=True):
        if not mask.any():
            return
        print(f'\n{"-"*74}\n{title}  ({int(mask.sum())}개)')
        idx = np.where(mask)[0]
        # 연속 구간으로 묶어서 출력 — 흩어진 단발인지 뭉친 덩어리인지 구분된다
        groups, start = [], idx[0]
        for k in range(1, len(idx) + 1):
            if k == len(idx) or idx[k] != idx[k - 1] + 1:
                groups.append((start, idx[k - 1]))
                if k < len(idx):
                    start = idx[k]
        print(f'  {"각도[0~360]":>16} {"bin수":>6} {"거리 base→test":>20} {"강도 base→test":>18}')
        for s, e in groups[:a.max_rows]:
            a0, a1 = to360(math.degrees(ang[s])), to360(math.degrees(ang[e]))
            seg = slice(s, e + 1)
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', RuntimeWarning)
                rb, rt = np.nanmedian(bm[seg]), np.nanmedian(tm[seg])
                ib, it = np.nanmedian(bi[seg]), np.nanmedian(ti[seg])
            rtxt = f'{rb:7.3f} → {rt:7.3f}' if show_range else ' ' * 17
            print(f'  {a0:7.1f}~{a1:6.1f} {e - s + 1:6d} {rtxt:>20} '
                  f'{ib:7.1f} → {it:7.1f}')
        if len(groups) > a.max_rows:
            print(f'  ... 외 {len(groups) - a.max_rows}개 구간')

    report(ghost, '유령 광선 구간 — 간섭의 직접 증거')
    report(drop, '소실 광선 구간')
    report(shift, '거리 변동 구간')

    print(f'\n{"-"*74}\n판정 (사람이 확인)')
    if ghost.sum() == 0 and drop.sum() == 0 and shift.sum() == 0:
        print('  · 차이 없음 — 이 배치에서는 간섭이 관측되지 않았다.')
    else:
        print('  · 유령/변동이 **상대 라이다 방향에만** 몰려 있으면 → 어차피 마스킹할 섹터라 무해.')
        print('  · **엉뚱한 각도에 흩어져** 있으면 → 위험. 없는 장애물을 만들어낸다.')
        print('  · 장면이 정말 정지 상태였는지 먼저 의심할 것 (사람·커튼·의자 흔들림).')
        print('  · 강도(base→test)가 크게 다른 유령은 간섭일 가능성이 높다.')


def main():
    p = argparse.ArgumentParser(
        description='라이다 유효 시야각(FOV) 측정 · 상호 간섭 시험',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = p.add_subparsers(dest='cmd', required=True)

    def common(q):
        q.add_argument('--topic', default='/scan', help='LaserScan 토픽 (기본 /scan)')
        q.add_argument('--max-range', type=float, default=12.0, help='유효 최대거리 [m]')

    q = sub.add_parser('protractor', help='각도 눈금자를 RViz 마커로 발행')
    common(q)
    q.add_argument('--marker-topic', default='/fov/markers')
    q.add_argument('--radius', type=float, default=2.0, help='눈금자 반경 [m]')
    q.add_argument('--tick-deg', type=float, default=10.0, help='눈금 간격 [deg]')
    q.add_argument('--label-deg', type=float, default=30.0, help='라벨 간격 [deg]')
    q.add_argument('--near-m', type=float, default=0.5, help='이 거리 안의 점을 빨강 강조 [m]')
    q.add_argument('--rings', type=float, nargs='*', default=[0.5, 1.0, 2.0])
    q.add_argument('--sector', type=float, nargs=2, default=None,
                   metavar=('START', 'END'), help='후보 섹터 음영 [deg, 0~360, 반시계]')
    q.add_argument('--text-size', type=float, default=0.09)
    q.add_argument('--label-pad', type=float, default=0.16)
    q.add_argument('--every', type=int, default=2, help='N스캔마다 1회 발행')
    q.set_defaults(fn=cmd_protractor)

    q = sub.add_parser('mask', help='통계로 자기가림 섹터 자동 판정')
    common(q)
    q.add_argument('--scans', type=int, default=80, help='수집 스캔 수 (기본 80 = 8초)')
    q.add_argument('--bin-deg', type=float, default=2.0, help='판정 각도 폭 [deg]')
    q.add_argument('--self-max', type=float, default=0.35,
                   help='이 거리 이하 + 변동 없으면 차체로 판정 [m]')
    q.add_argument('--self-std', type=float, default=0.01,
                   help='차체 판정 최대 변동 [m]')
    q.add_argument('--min-hit', type=float, default=0.5,
                   help='수신율이 이 값 미만이면 무반사 구간으로 판정')
    q.add_argument('--only-blocked', action='store_true', help='가림 구간만 출력')
    q.set_defaults(fn=cmd_mask)

    q = sub.add_parser('record', help='정지 장면을 광선 단위로 기록 (간섭 A/B 시험)')
    common(q)
    q.add_argument('--out', required=True, help='저장 경로 (.npz)')
    q.add_argument('--scans', type=int, default=60, help='수집 스캔 수 (기본 60 = 6초)')
    q.add_argument('--bin-deg', type=float, default=1.0,
                   help='각도 격자 [deg] (기본 1.0 — 각분해능 0.839deg 에 근접)')
    q.set_defaults(fn=cmd_record)

    q = sub.add_parser('compare', help='기록 2개를 비교해 간섭 흔적 추출')
    q.add_argument('--base', required=True, help='기준 기록 (상대 유닛 꺼짐)')
    q.add_argument('--test', required=True, help='시험 기록 (상대 유닛 켜짐)')
    q.add_argument('--quiet', type=float, default=0.1, help='이 수신율 이하 = 반사 없음')
    q.add_argument('--active', type=float, default=0.5, help='이 수신율 이상 = 반사 있음')
    q.add_argument('--shift-m', type=float, default=0.05, help='거리 변동 판정 [m]')
    q.add_argument('--max-rows', type=int, default=25, help='구간 출력 최대 줄 수')
    q.set_defaults(fn=cmd_compare)

    a = p.parse_args()
    a.fn(a)


if __name__ == '__main__':
    main()
