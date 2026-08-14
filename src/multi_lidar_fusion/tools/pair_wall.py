#!/usr/bin/env python3
"""두 라이다가 **같은 벽**을 보게 해서 좌표계 정합을 잰다 (겹침 영역 검증).

이 파일의 역할:
    `wall_calibrate.py` 는 라이다마다 자기 벽을 따로 재므로, 4대가 각자 완벽해도
    장착 x/y/yaw 가 틀리면 병합 화면은 어긋난다 — 서로를 비교할 근거가 데이터
    안에 없기 때문이다. 이 도구는 **하나의 벽을 두 대가 동시에** 보게 하고,
    각자 적합한 직선을 `base_link` 로 옮겨 비교한다.

        거리 차  ->  장착 x/y 오차 (또는 센서 거리 바이어스)
        각도 차  ->  장착 yaw 오차

    ★ 참값(줄자)이 필요 없다. "둘이 같은 것을 같게 말하는가"만 보기 때문이다.
      회피 로직은 merged_scan 하나만 보므로, 실제로 중요한 것도 이 상대 일치다.

    ★ 벽을 **겹침 영역**에 세워야 한다. 인접 센서끼리 50~60도 겹친다:
        앞 ∩ 오른쪽 : 차량 기준 -90 ~ -40 도   (우전방 대각선)
        앞 ∩ 왼쪽   : 차량 기준 +30 ~ +90 도
        뒤 ∩ 왼쪽   : 차량 기준 +110 ~ +160 도
        뒤 ∩ 오른쪽 : 차량 기준 -160 ~ -110 도

입력 topic : /lidar/<id>/scan  (LaserScan, 드라이버 원본) 2개
             + TF (base_link <- lidar_<id>_link) — 융합 노드가 발행하는 것을 쓴다.
             ★ TF 를 직접 계산하지 않고 조회하는 이유: 융합 파이프라인이 실제로
               쓰는 값과 **같은 것**을 봐야 검증이 성립한다.
출력       : 화면 실시간 비교표
frame      : 비교는 base_link 에서 한다.

파라미터:
    first / second   "앞","뒤","왼쪽","오른쪽" 또는 a1/a2/b1/b2
    base_frame       "base_link"
    r_min / r_max    0.25 / 3.00   이 거리대의 점만 벽 후보 [m]
                     (0.25 하한이 차체 반사를 걸러낸다 — RP 는 raw 0 근처 6~7cm,
                      YD 는 16~18cm 에 자기 차체가 잡힌다)
    min_points       10
    min_span_deg     8.0    이 각도보다 좁은 덩어리는 기준물로 보지 않는다(잡물 제거)
    window_scans     20     최근 몇 스캔을 평균낼 것인가
    tol_dist_m       0.02   합격선: 거리 차
    tol_ang_deg      2.0    합격선: 각도 차
    max_rms_m        0.012  이보다 면이 거칠면 확정하지 않는다(모서리를 보는 중일 수 있음)
    sanity_dist_m    0.30   두 값이 이보다 벌어지면 "서로 다른 물체"로 보고 확정 보류
    sanity_ang_deg   30.0   각도도 마찬가지
    stable_s         2.0    이 시간 동안 값이 안 흔들리면 **자동 확정하고 종료**
    report_period_s  0.5

실행:
    # TF 가 필요하므로 융합 노드를 함께 띄운다 (RViz 없이)
    ros2 launch multi_lidar_fusion multi_lidar_fusion.launch.py
    ros2 run multi_lidar_fusion pair_wall.py --ros-args -p first:=앞 -p second:=오른쪽

관계: 여기서 나온 차이를 `stack_parking/config/lidar_mounts.yaml` 의 x/y/yaw 에
      반영한다. 거리 단위계(wall_calibrate.py)를 먼저 맞춘 뒤에 할 것 —
      거리 바이어스가 남아 있으면 그것이 위치 오차처럼 보인다.
"""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformListener

POSITION_NAME = {'a1': '앞', 'a2': '뒤', 'b1': '왼쪽', 'b2': '오른쪽'}
# 쌍별 겹침 구간 (vehicle frame, deg) — lidar_mounts.yaml 의 시야각에서 계산된 값.
# 기준물은 이 방향에서만 찾는다. 겹침 밖의 물체는 애초에 두 대가 함께 볼 수 없다.
OVERLAP = {
    frozenset(('a1', 'b1')): (30.0, 90.0),      # 앞 ∩ 왼쪽
    frozenset(('a1', 'b2')): (-90.0, -40.0),    # 앞 ∩ 오른쪽
    frozenset(('a2', 'b1')): (110.0, 160.0),    # 뒤 ∩ 왼쪽
    frozenset(('a2', 'b2')): (-160.0, -110.0),  # 뒤 ∩ 오른쪽
}

SLOT_ALIAS = {
    'a1': 'a1', '앞': 'a1', 'front': 'a1',
    'a2': 'a2', '뒤': 'a2', 'rear': 'a2',
    'b1': 'b1', '왼쪽': 'b1', '좌': 'b1', 'left': 'b1',
    'b2': 'b2', '오른쪽': 'b2', '우': 'b2', 'right': 'b2',
}


def out(msg=''):
    print(msg, flush=True)


def wrap_deg(a):
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def fit_line(points):
    """전최소제곱(PCA) 직선 적합 → (원점까지 수직거리, 법선 단위벡터, 잔차 RMS, 중심점)."""
    n = len(points)
    if n < 2:
        return None
    cx = sum(p[0] for p in points) / n
    cy = sum(p[1] for p in points) / n
    sxx = syy = sxy = 0.0
    for x, y in points:
        dx, dy = x - cx, y - cy
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy
    sxx /= n
    syy /= n
    sxy /= n
    tr = sxx + syy
    disc = max(0.0, tr * tr / 4.0 - (sxx * syy - sxy * sxy))
    lam = tr / 2.0 - math.sqrt(disc)
    if abs(sxy) > 1e-12:
        nx, ny = lam - syy, sxy
    else:
        nx, ny = (1.0, 0.0) if sxx <= syy else (0.0, 1.0)
    norm = math.hypot(nx, ny)
    if norm < 1e-12:
        return None
    nx, ny = nx / norm, ny / norm
    if nx * cx + ny * cy < 0:       # 법선이 센서 바깥(벽 쪽)을 향하게
        nx, ny = -nx, -ny
    return abs(nx * cx + ny * cy), (nx, ny), math.sqrt(max(0.0, lam)), (cx, cy)


class PairWall(Node):

    def __init__(self):
        super().__init__('pair_wall')

        a = str(self.declare_parameter('first', '앞').value).strip()
        b = str(self.declare_parameter('second', '오른쪽').value).strip()
        self.base = self.declare_parameter('base_frame', 'base_link').value
        self.r_min = float(self.declare_parameter('r_min', 0.25).value)
        self.r_max = float(self.declare_parameter('r_max', 3.00).value)
        self.min_points = int(self.declare_parameter('min_points', 10).value)
        self.min_span = float(self.declare_parameter('min_span_deg', 8.0).value)
        self.win = int(self.declare_parameter('window_scans', 20).value)
        self.tol_d = float(self.declare_parameter('tol_dist_m', 0.02).value)
        self.tol_a = float(self.declare_parameter('tol_ang_deg', 2.0).value)
        self.max_rms = float(self.declare_parameter('max_rms_m', 0.012).value)
        self.sane_d = float(self.declare_parameter('sanity_dist_m', 0.30).value)
        self.sane_a = float(self.declare_parameter('sanity_ang_deg', 30.0).value)
        self.stable_s = float(self.declare_parameter('stable_s', 2.0).value)
        sec_lo = float(self.declare_parameter('sector_min_deg', 999.0).value)
        sec_hi = float(self.declare_parameter('sector_max_deg', 999.0).value)
        self.margin = float(self.declare_parameter('sector_margin_deg', 5.0).value)
        period = float(self.declare_parameter('report_period_s', 0.5).value)
        self.period = period
        self.recent_cmp = []
        self.topic_tmpl = self.declare_parameter('topic_tmpl', '/lidar/{}/scan').value

        self.slots = []
        for name in (a, b):
            s = SLOT_ALIAS.get(name.lower())
            if s is None:
                out(f'! "{name}" 를 못 알아듣겠습니다. 앞/뒤/왼쪽/오른쪽 중에서 주세요.')
                raise SystemExit(2)
            self.slots.append(s)
        if self.slots[0] == self.slots[1]:
            out('! 같은 라이다 두 개를 비교할 수는 없습니다.')
            raise SystemExit(2)

        # ★ 기준물을 **겹침 방향에서만** 찾는다. 방향 제한 없이 "가장 가까운 덩어리"를
        #   고르면 두 라이다가 서로 다른 물체를 집는다(2026-08-14 실측: 계속 "다른 물체"
        #   경고). 겹침 밖의 물체는 두 대가 함께 볼 수 없으므로 볼 이유도 없다.
        if sec_lo > 360.0 or sec_hi > 360.0:
            lo, hi = OVERLAP.get(frozenset(self.slots), (-180.0, 180.0))
        else:
            lo, hi = sec_lo, sec_hi
        self.sector = (wrap_deg(lo - self.margin), wrap_deg(hi + self.margin))

        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)

        self.hist = {s: [] for s in self.slots}     # (거리, 법선각deg, rms, 점수)
        self.seen = {s: 0 for s in self.slots}
        for s in self.slots:
            self.create_subscription(
                LaserScan, self.topic_tmpl.format(s),
                lambda m, key=s: self.on_scan(key, m), qos_profile_sensor_data)

        self.t0 = time.time()
        self.create_timer(period, self.report)

        n0, n1 = (POSITION_NAME.get(s, s) for s in self.slots)
        out()
        out('=' * 74)
        out(f'  좌표계 정합 — [{n0}] 와 [{n1}] 이 **같은 벽**을 보게 하세요')
        out('=' * 74)
        out()
        out(f'  기준물은 차량 기준 {self.sector[0]:+.0f} ~ {self.sector[1]:+.0f} 도 '
            '방향에서만 찾습니다 (두 대의 겹침 구간).')
        out(f'  거리는 {self.r_min * 100:.0f}~{self.r_max * 100:.0f} cm 안이어야 하고,')
        out('  넓고 평평할수록 좋습니다. 참값(줄자)은 필요 없습니다.')
        out()
        out(f'  합격선: 거리 차 < {self.tol_d * 100:.0f} cm,  각도 차 < {self.tol_a:.0f} 도')
        out(f'  두 값이 {self.stable_s:.0f}초간 안 흔들리면 **자동 확정하고 종료**합니다.')
        out('  중간에 끝내려면 Ctrl-C')
        out()
        out('-' * 74)

    def in_sector(self, bearing_deg):
        lo, hi = self.sector
        if lo > hi:                      # ±180 을 가로지르는 구간
            return bearing_deg >= lo or bearing_deg <= hi
        return lo <= bearing_deg <= hi

    def on_scan(self, key, msg):
        n = len(msg.ranges)
        if n == 0 or not msg.angle_increment:
            return
        self.seen[key] += 1

        # TF 는 융합 노드가 내는 것을 그대로 쓴다 (파이프라인과 같은 값을 봐야 검증이 성립).
        try:
            tf = self.buf.lookup_transform(self.base, msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, sn = math.cos(yaw), math.sin(yaw)
        tx, ty = tf.transform.translation.x, tf.transform.translation.y

        # 점을 base_link 로 옮긴 뒤 **겹침 방향**만 남긴다.
        pts = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < self.r_min or r > self.r_max:
                continue
            a = msg.angle_min + msg.angle_increment * i
            xs, ys = r * math.cos(a), r * math.sin(a)
            xb, yb = c * xs - sn * ys + tx, sn * xs + c * ys + ty
            bearing = math.degrees(math.atan2(yb, xb))
            if not self.in_sector(bearing):
                continue
            pts.append((bearing, xb, yb, math.hypot(xb, yb)))
        if len(pts) < self.min_points:
            return
        pts.sort(key=lambda p: p[0])

        groups, cur = [], []
        for p in pts:
            if cur and (abs(p[0] - cur[-1][0]) > 2.0 or abs(p[3] - cur[-1][3]) > 0.08):
                groups.append(cur)
                cur = []
            cur.append(p)
        if cur:
            groups.append(cur)
        cand = [g for g in groups
                if len(g) >= self.min_points and abs(g[-1][0] - g[0][0]) >= self.min_span]
        if not cand:
            return
        best = min(cand, key=lambda g: sum(p[3] for p in g) / len(g))

        fit = fit_line([(p[1], p[2]) for p in best])
        if fit is None:
            return
        d_base, (nbx, nby), rms, _ = fit
        ang = math.degrees(math.atan2(nby, nbx))
        bearing_mid = (best[0][0] + best[-1][0]) / 2.0

        self.hist[key].append((d_base, ang, rms, len(best), bearing_mid))
        if len(self.hist[key]) > self.win:
            self.hist[key].pop(0)

    def report(self):
        rows = {}
        for s in self.slots:
            h = self.hist[s]
            if not h:
                rows[s] = None
                continue
            n = len(h)
            d = sum(x[0] for x in h) / n
            # 각도는 원형 평균
            sx = sum(math.cos(math.radians(x[1])) for x in h) / n
            sy = sum(math.sin(math.radians(x[1])) for x in h) / n
            ang = math.degrees(math.atan2(sy, sx))
            rms = sum(x[2] for x in h) / n
            pts = sum(x[3] for x in h) / n
            brg = sum(x[4] for x in h) / n
            rows[s] = (d, ang, rms, pts, brg)

        out()
        for s in self.slots:
            name = POSITION_NAME.get(s, s)
            r = rows[s]
            if r is None:
                why = '수신 없음' if self.seen[s] == 0 else '벽 안 보임 (TF 없음이거나 거리 밖)'
                out(f'  {name:<5} {why}')
                continue
            d, ang, rms, pts, brg = r
            out(f'  {name:<5} 벽까지 {d * 100:6.2f} cm   법선 {ang:+7.2f} 도   '
                f'물체방향 {brg:+6.1f} 도   평탄도 {rms * 1000:4.1f} mm  {pts:4.0f}점')

        a, b = (rows[s] for s in self.slots)
        if not (a and b):
            self.recent_cmp.clear()
            return

        dd = a[0] - b[0]
        da = wrap_deg(a[1] - b[1])
        ok_d = abs(dd) < self.tol_d
        ok_a = abs(da) < self.tol_a
        out(f'   -> 거리 차 {dd * 100:+6.2f} cm [{"OK" if ok_d else "!"}]   '
            f'각도 차 {da:+6.2f} 도 [{"OK" if ok_a else "!"}]')

        # ── 확정해도 되는 상태인가 ────────────────────────────────────
        # 합격/불합격과 무관하게 **값이 믿을 만해지면** 확정한다. 어긋난 값도
        # 그대로 확정해야 무엇을 고칠지 알 수 있기 때문이다.
        if abs(dd) > self.sane_d or abs(da) > self.sane_a:
            out('      ! 둘이 서로 다른 물체를 보고 있는 것 같습니다 — '
                '박스를 두 라이다가 함께 보는 자리로 옮겨주세요')
            self.recent_cmp.clear()
            return
        if a[2] > self.max_rms or b[2] > self.max_rms:
            out('      ! 면이 거칩니다 — 한쪽이 모서리를 보고 있을 수 있습니다. '
                '박스를 조금 돌려주세요')
            self.recent_cmp.clear()
            return

        self.recent_cmp.append((a[0], b[0], a[1], b[1]))
        need = max(3, int(self.stable_s / self.period) + 1)
        if len(self.recent_cmp) > need:
            self.recent_cmp.pop(0)
        if len(self.recent_cmp) < need:
            out(f'      값 안정 확인 중... ({len(self.recent_cmp)}/{need})')
            return
        cols = list(zip(*self.recent_cmp))
        spans = [max(c) - min(c) for c in cols]
        if spans[0] > 0.006 or spans[1] > 0.006 or spans[2] > 0.6 or spans[3] > 0.6:
            out('      값이 아직 흔들립니다 — 박스를 고정해 주세요')
            return
        self.finish(rows, dd, da, ok_d, ok_a)

    def finish(self, rows, dd, da, ok_d, ok_a):
        n0, n1 = (POSITION_NAME.get(s, s) for s in self.slots)
        lines = ['', '=' * 74,
                 f'  확정 — [{n0}] ↔ [{n1}] 공유 기준물 비교 (base_link 기준)',
                 '=' * 74]
        for s in self.slots:
            d, ang, rms, pts, brg = rows[s]
            lines.append(f'  {POSITION_NAME.get(s, s):<5} 벽까지 {d * 100:7.2f} cm   '
                         f'법선 {ang:+7.2f} 도   물체방향 {brg:+6.1f} 도   '
                         f'평탄도 {rms * 1000:4.1f} mm  {pts:4.0f}점')
        lines.append('')
        lines.append(f'  거리 차 {dd * 100:+6.2f} cm   (합격선 {self.tol_d * 100:.0f} cm) '
                     f'{"OK" if ok_d else "-> 어긋남"}')
        lines.append(f'  각도 차 {da:+6.2f} 도    (합격선 {self.tol_a:.0f} 도) '
                     f'{"OK" if ok_a else "-> 어긋남"}')
        lines.append('')
        if ok_d and ok_a:
            lines.append('  두 라이다가 같은 것을 같게 말합니다. 이 쌍은 정합 완료.')
        else:
            if not ok_d:
                lines.append('  거리 차 → 장착 x/y 오차 또는 센서 거리 바이어스')
            if not ok_a:
                lines.append('  각도 차 → 장착 yaw 오차')
            lines.append('  ※ 거리 단위계(wall_calibrate)를 먼저 맞춰야 x/y 판단이 정확합니다.')
        lines.append('')
        text = '\n'.join(lines)
        out(text)
        path = os.path.expanduser(f'~/pair_wall_{self.slots[0]}_{self.slots[1]}.txt')
        try:
            with open(path, 'w', encoding='utf-8') as f:
                f.write(text + '\n')
            out(f'  저장: {path}')
        except OSError as e:
            out(f'  ! 저장 실패: {e}')
        out()
        out('  창을 닫으셔도 됩니다.')
        raise SystemExit(0)


def main():
    rclpy.init()
    node = PairWall()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit as e:
        node.destroy_node()
        rclpy.try_shutdown()
        try:
            input('\n  [Enter] 를 누르면 닫힙니다...')
        except (EOFError, KeyboardInterrupt):
            pass
        sys.exit(e.code if isinstance(e.code, int) else 0)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
