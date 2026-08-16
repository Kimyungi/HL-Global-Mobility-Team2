#!/usr/bin/env python3
"""겹침을 구속조건으로 걸어 장착값을 **역산**한다 (쌍별 외부 캘리브).

이 파일의 역할:
    `pair_wall.py` 는 두 라이다의 불일치를 **보여주기만** 한다. 이 도구는 그 불일치를
    0 으로 만드는 장착값을 직접 푼다 — 값을 손으로 밀어보는 대신.

    원리: 두 라이다가 **같은 평면**을 보면 base_link 에서 법선각과 수직거리가 같아야 한다.

        alpha_b = alpha_s + psi                                  (psi = 장착 yaw)
        d_b     = d_s + delta + (x*cos(alpha_b) + y*sin(alpha_b)) (delta = 거리 바이어스)

    캡처 1회당 방정식 2개(법선각·수직거리). 한쪽을 기준으로 고정하면 상대편의
    psi, x, y 3개가 미지수이므로 **3~4회 캡처하면 과결정**되어 최소제곱으로 풀린다.

    ★ 캡처마다 **법선 방향을 바꿔야** 한다. 같은 각도로만 놓으면 위치(n·t)와 거리
      바이어스(delta)가 수식상 구분되지 않아 답이 갈라지지 않는다. 박스를 겹침 구간
      안에서 좌우로 옮기고 각도도 틀어가며 놓을 것.

    ★ delta(거리 바이어스)는 **미지수로 두지 않는다.** wall_calibrate.py 로 이미 따로
      쟀고(YD +4.5cm / RP +0.4cm), 미지수로 함께 풀면 좁은 겹침각(50~60도) 안에서
      위치와 뒤섞여 둘 다 못 믿게 된다.

입력 topic : /lidar/<id>/scan 2개 + TF (base_link <- lidar_<id>_link)
출력       : 화면 — 캡처 진행, 최종 보정값(psi, x, y) 과 잔차(before/after)
             결과 파일 ~/pair_calib_<A>_<B>.txt
frame      : 적합은 **센서 좌표계**에서, 비교는 base_link 에서.

파라미터:
    first / second     기준 / 보정 대상.  기준은 손대지 않는다.
    samples            4      필요한 캡처 수
    delta_first_m      0.045  기준 센서의 거리 바이어스 [m] — **양수 = 길게 읽는다**
    delta_second_m     0.004  보정 대상의 거리 바이어스 [m]  (참값 = 측정 - delta)
    solve_xy           false  x/y 도 함께 풀지 여부. 겹침각이 좁으면 y 와 거리
                              바이어스가 거의 구분되지 않으므로 **기본은 yaw 만** 푼다.
    r_min / r_max      0.25 / 2.5
    min_points         12
    min_span_deg       8.0
    min_common_deg     16.0   두 라이다가 함께 보는 각도 폭의 하한(좁으면 법선이 흔들림)
    stable_s           1.5    이 시간 안정되면 한 캡처로 확정
    max_rms_m          0.012

실행:
    ros2 launch multi_lidar_fusion multi_lidar_fusion.launch.py     # TF 필요
    ros2 run multi_lidar_fusion pair_calibrate.py --ros-args \
        -p first:=앞 -p second:=오른쪽

관계: 나온 값을 `stack_parking/config/lidar_mounts.yaml` 의 해당 항목
      (yaw_deg, x, y) 에 반영한다. 기준 센서는 바뀌지 않는다.
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
SLOT_ALIAS = {
    'a1': 'a1', '앞': 'a1', 'front': 'a1',
    'a2': 'a2', '뒤': 'a2', 'rear': 'a2',
    'b1': 'b1', '왼쪽': 'b1', '좌': 'b1', 'left': 'b1',
    'b2': 'b2', '오른쪽': 'b2', '우': 'b2', 'right': 'b2',
}
OVERLAP = {
    frozenset(('a1', 'b1')): (30.0, 90.0),
    frozenset(('a1', 'b2')): (-90.0, -40.0),
    frozenset(('a2', 'b1')): (110.0, 160.0),
    frozenset(('a2', 'b2')): (-160.0, -110.0),
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
    """전최소제곱(PCA) → (수직거리, 법선각[deg], 잔차 RMS). 좌표계는 넣은 그대로."""
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
    nn = math.hypot(nx, ny)
    if nn < 1e-12:
        return None
    nx, ny = nx / nn, ny / nn
    if nx * cx + ny * cy < 0:
        nx, ny = -nx, -ny
    return abs(nx * cx + ny * cy), math.degrees(math.atan2(ny, nx)), math.sqrt(max(0.0, lam))


def solve2x2(a11, a12, a21, a22, b1, b2):
    det = a11 * a22 - a12 * a21
    if abs(det) < 1e-9:
        return None
    return ((b1 * a22 - a12 * b2) / det, (a11 * b2 - b1 * a21) / det)


class PairCalibrate(Node):

    def __init__(self):
        super().__init__('pair_calibrate')

        a = str(self.declare_parameter('first', '앞').value).strip()
        b = str(self.declare_parameter('second', '오른쪽').value).strip()
        self.base = self.declare_parameter('base_frame', 'base_link').value
        self.need = int(self.declare_parameter('samples', 4).value)
        self.delta = [float(self.declare_parameter('delta_first_m', 0.045).value),
                      float(self.declare_parameter('delta_second_m', 0.004).value)]
        self.r_min = float(self.declare_parameter('r_min', 0.25).value)
        self.r_max = float(self.declare_parameter('r_max', 2.5).value)
        self.min_points = int(self.declare_parameter('min_points', 12).value)
        self.min_span = float(self.declare_parameter('min_span_deg', 8.0).value)
        # 두 라이다가 **함께 보는** 각도 폭의 하한. 좁으면 법선이 흔들린다.
        self.min_common_deg = float(self.declare_parameter('min_common_deg', 16.0).value)
        self.stable_s = float(self.declare_parameter('stable_s', 1.5).value)
        # ★ 12mm 로 되돌림. 2026-08-14 3차 캡처에서 17.7mm 짜리 한 개가 섞였는데,
        #   그것만 빼니 상대 yaw 편차가 2.64도 -> 0.05도 로 떨어졌다. 거친 캡처 하나가
        #   결과를 통째로 흔든다.
        self.max_rms = float(self.declare_parameter('max_rms_m', 0.012).value)
        self.margin = float(self.declare_parameter('sector_margin_deg', 5.0).value)
        # ★ 기본은 yaw 만 푼다. 겹침 구간이 50~60도면 법선의 n_y 가 거의 일정해서
        #   y 와 거리 바이어스가 수치적으로 갈리지 않는다 — 억지로 풀면 y 가 남은 오차를
        #   전부 흡수해 14cm 씩 튄다(2026-08-14 실측). x/y 는 줄자 실측을 신뢰한다.
        self.solve_xy = bool(self.declare_parameter('solve_xy', False).value)
        self.topic_tmpl = self.declare_parameter('topic_tmpl', '/lidar/{}/scan').value

        self.slots = []
        for name in (a, b):
            s = SLOT_ALIAS.get(name.lower())
            if s is None:
                out(f'! "{name}" 를 못 알아듣겠습니다.')
                raise SystemExit(2)
            self.slots.append(s)
        lo, hi = OVERLAP.get(frozenset(self.slots), (-180.0, 180.0))
        self.sector = (wrap_deg(lo - self.margin), wrap_deg(hi + self.margin))

        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.tf_cache = {}
        # 센서별 최신 관측: (d_s, alpha_s, rms, npts)  ← **센서 좌표계** 기준
        self.last = {s: None for s in self.slots}
        self.hold = None
        self.recent = []
        self.win_ema = None       # 공통 각도 구간 평활 (경계 흔들림 억제)
        self.caps = []            # 확정된 캡처들
        self.seen = {s: 0 for s in self.slots}

        for s in self.slots:
            self.create_subscription(
                LaserScan, self.topic_tmpl.format(s),
                lambda m, key=s: self.on_scan(key, m), qos_profile_sensor_data)
        self.create_timer(0.4, self.tick)
        self.last_line = ''
        self.last_say = 0.0

        n0, n1 = (POSITION_NAME.get(s, s) for s in self.slots)
        out()
        out('=' * 76)
        out(f'  장착값 역산 — 기준 [{n0}] 고정,  [{n1}] 의 yaw / x / y 를 푼다')
        out('=' * 76)
        out()
        out(f'  박스(평평한 면)를 차량 기준 {self.sector[0]:+.0f}~{self.sector[1]:+.0f} 도 '
            '안에 두고,')
        out(f'  **위치와 각도를 바꿔가며 {self.need} 번** 놓아주세요. 안정되면 자동 저장됩니다.')
        out()
        out('  ★ 매번 각도를 바꾸는 것이 핵심입니다 — 같은 각도로만 놓으면')
        out('    위치(x,y)와 거리 바이어스가 수식상 구분되지 않아 답이 안 나옵니다.')
        out()
        out(f'  거리 바이어스는 실측값으로 고정: {n0} {self.delta[0] * 100:+.1f}cm, '
            f'{n1} {self.delta[1] * 100:+.1f}cm')
        out('  Ctrl-C 로 중단')
        out('-' * 76)

    def say(self, line):
        now = time.time()
        if line != self.last_line or now - self.last_say > 1.0:
            out(line)
            self.last_line = line
            self.last_say = now

    def tf_of(self, frame):
        if frame in self.tf_cache:
            return self.tf_cache[frame]
        try:
            tf = self.buf.lookup_transform(self.base, frame, rclpy.time.Time())
        except Exception:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = (tf.transform.translation.x, tf.transform.translation.y, yaw)
        self.tf_cache[frame] = v
        return v

    def on_scan(self, key, msg):
        if not msg.ranges or not msg.angle_increment:
            return
        self.seen[key] += 1
        tf = self.tf_of(msg.header.frame_id)
        if tf is None:
            return
        tx, ty, psi = tf
        c, sn = math.cos(psi), math.sin(psi)
        inc = abs(math.degrees(msg.angle_increment))

        pts = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < self.r_min or r > self.r_max:
                continue
            ang = msg.angle_min + msg.angle_increment * i
            xs, ys = r * math.cos(ang), r * math.sin(ang)
            xb, yb = c * xs - sn * ys + tx, sn * xs + c * ys + ty
            bearing = math.degrees(math.atan2(yb, xb))
            lo, hi = self.sector
            ok = (bearing >= lo or bearing <= hi) if lo > hi else (lo <= bearing <= hi)
            if ok:
                pts.append((math.degrees(ang), xs, ys, r, bearing))
        if len(pts) < self.min_points:
            self.last[key] = None
            return
        pts.sort(key=lambda p: p[0])
        groups, cur = [], []
        for p in pts:
            if cur and (abs(p[0] - cur[-1][0]) > 3.0 * inc or abs(p[3] - cur[-1][3]) > 0.08):
                groups.append(cur)
                cur = []
            cur.append(p)
        if cur:
            groups.append(cur)
        cand = [g for g in groups
                if len(g) >= self.min_points and abs(g[-1][0] - g[0][0]) >= self.min_span]
        if not cand:
            self.last[key] = None
            return
        best = min(cand, key=lambda g: sum(p[3] for p in g) / len(g))
        # ★ 여기서 바로 적합하지 않는다. 두 센서가 **공통으로 보는 각도 구간**만 써야
        #   같은 평면을 비교하게 된다 — 한쪽이 더 넓게 보면 박스 밖 다른 면까지 끌어들여
        #   적합이 무너진다(2026-08-14: 오른쪽 평탄도 40mm). 실제 적합은 tick() 에서 한다.
        #   점은 (base 방위, 센서 x, 센서 y) 로 들고 간다.
        self.last[key] = [(pb[4], pb[1], pb[2]) for pb in best]

    def tick(self):
        A, B = self.slots
        pa, pb = self.last[A], self.last[B]
        na, nb = POSITION_NAME.get(A, A), POSITION_NAME.get(B, B)
        if not pa or not pb:
            miss = [n for n, r in ((na, pa), (nb, pb)) if not r]
            self.say(f'  [{len(self.caps)}/{self.need}] 박스를 찾는 중... '
                     f'(못 보는 쪽: {", ".join(miss)})')
            self.hold = None
            self.recent = []
            return

        # ★ 공통 각도 구간(base 방위)만 남겨서 **같은 면 조각**을 비교한다.
        lo_r = max(min(q[0] for q in pa), min(q[0] for q in pb)) + 2.0
        hi_r = min(max(q[0] for q in pa), max(q[0] for q in pb)) - 2.0
        # 경계를 그대로 쓰면 프레임마다 1~2도씩 흔들려 적합값이 따라 흔들린다.
        if self.win_ema is None or abs(lo_r - self.win_ema[0]) > 10.0:
            self.win_ema = (lo_r, hi_r)
        else:
            self.win_ema = (0.7 * self.win_ema[0] + 0.3 * lo_r,
                            0.7 * self.win_ema[1] + 0.3 * hi_r)
        lo, hi = self.win_ema
        # ★ 공통 구간이 좁으면 법선 추정이 무너진다. 2026-08-14 왼쪽 쌍에서 11~13도
        #   구간으로 캡처했더니 캡처별 상대 yaw 가 -0.01/-8.75/-1.73 로 갈렸다
        #   (오른쪽 쌍은 18~26도 구간에서 편차 0.05도). 넓은 구간을 강제한다.
        if hi - lo < self.min_common_deg:
            self.say(f'  [{len(self.caps)}/{self.need}] 함께 보는 구간이 '
                     f'{max(0.0, hi - lo):.0f}도 뿐입니다 (최소 {self.min_common_deg:.0f}도) — '
                     '박스를 더 멀리 두거나 겹침 한가운데로 옮기세요')
            self.hold = None
            self.recent = []
            return
        cut_a = [(q[1], q[2]) for q in pa if lo <= q[0] <= hi]
        cut_b = [(q[1], q[2]) for q in pb if lo <= q[0] <= hi]
        if len(cut_a) < self.min_points or len(cut_b) < self.min_points:
            self.say(f'  [{len(self.caps)}/{self.need}] 공통 구간의 점이 부족합니다 '
                     f'({na} {len(cut_a)} / {nb} {len(cut_b)}점)')
            self.hold = None
            self.recent = []
            return
        fa, fb = fit_line(cut_a), fit_line(cut_b)
        if fa is None or fb is None:
            return
        ra = (fa[0], fa[1], fa[2], len(cut_a))
        rb = (fb[0], fb[1], fb[2], len(cut_b))
        if ra[2] > self.max_rms or rb[2] > self.max_rms:
            self.say(f'  [{len(self.caps)}/{self.need}] 면이 거칩니다 '
                     f'({na} {ra[2] * 1000:.0f}mm / {nb} {rb[2] * 1000:.0f}mm) — 박스를 조금 돌리세요')
            self.hold = None
            self.recent = []
            return

        self.recent.append((ra[0], ra[1], rb[0], rb[1]))
        if len(self.recent) > 6:
            self.recent.pop(0)
        if len(self.recent) < 4:
            self.say(f'  [{len(self.caps)}/{self.need}] 측정 중...')
            return
        cols = list(zip(*self.recent))
        if (max(cols[0]) - min(cols[0]) > 0.010 or max(cols[2]) - min(cols[2]) > 0.010 or
                max(cols[1]) - min(cols[1]) > 1.5 or max(cols[3]) - min(cols[3]) > 1.5):
            self.say(f'  [{len(self.caps)}/{self.need}] 박스를 고정해 주세요')
            self.hold = None
            return
        now = time.time()
        if self.hold is None:
            self.hold = now
            return
        if now - self.hold < self.stable_s:
            return

        # 이전 캡처와 법선이 너무 비슷하면 새 정보가 없다 — 각도를 바꾸라고 요구한다.
        tfa = self.tf_of(f'lidar_{A}_link')
        alpha_b = wrap_deg(ra[1] + math.degrees(tfa[2]))
        if any(abs(wrap_deg(alpha_b - c['alpha_b'])) < 8.0 for c in self.caps):
            self.say(f'  [{len(self.caps)}/{self.need}] 직전과 각도가 비슷합니다 '
                     f'(법선 {alpha_b:+.0f}도) — 박스 각도를 8도 이상 바꿔주세요')
            self.hold = None
            return

        self.caps.append({'dA': ra[0], 'aA': ra[1], 'dB': rb[0], 'aB': rb[1],
                          'alpha_b': alpha_b, 'nA': ra[3], 'nB': rb[3],
                          'rmsA': ra[2], 'rmsB': rb[2], 'win': (lo, hi)})
        out(f'  ✔ 캡처 {len(self.caps)}/{self.need}  법선 {alpha_b:+.1f}도  '
            f'({na} {ra[0] * 100:.1f}cm/{ra[3]}점, {nb} {rb[0] * 100:.1f}cm/{rb[3]}점)')
        self.hold = None
        self.recent = []
        self.win_ema = None
        self.last_line = ''
        if len(self.caps) >= self.need:
            self.solve()
        else:
            out('    → 박스를 다른 위치·각도로 옮겨주세요')

    def solve(self):
        A, B = self.slots
        tfa, tfb = self.tf_of(f'lidar_{A}_link'), self.tf_of(f'lidar_{B}_link')
        xa, ya, psiA = tfa
        xb0, yb0, psiB0 = tfb
        dA, dB = self.delta

        # ① yaw: alpha_sA + psiA = alpha_sB + psiB  →  psiB = mean(alpha_sA + psiA - alpha_sB)
        errs = [wrap_deg(c['aA'] + math.degrees(psiA) - c['aB']) for c in self.caps]
        sx = sum(math.cos(math.radians(e)) for e in errs) / len(errs)
        sy = sum(math.sin(math.radians(e)) for e in errs) / len(errs)
        psiB_new = math.degrees(math.atan2(sy, sx))
        spread = max(errs) - min(errs)

        # ② 위치: d_sA + dA + n·tA = d_sB + dB + n·tB   (n 은 확정된 alpha_b 방향)
        s11 = s12 = s22 = b1 = b2 = 0.0
        for c in self.caps:
            ab = math.radians(wrap_deg(c['aB'] + psiB_new))
            nx, ny = math.cos(ab), math.sin(ab)
            # ★ 바이어스는 **뺀다**. delta 는 "센서가 이만큼 길게 읽는다"이므로
            #   참값 = 측정 - delta 다. 더하면 편향이 두 배가 되고, 솔버가 그 오차를
            #   x/y 로 흡수해 버린다(2026-08-14: y 가 14.6cm 튀어 나옴).
            rhs = (c['dA'] - dA + nx * xa + ny * ya) - (c['dB'] - dB)
            s11 += nx * nx
            s12 += nx * ny
            s22 += ny * ny
            b1 += nx * rhs
            b2 += ny * rhs
        sol = solve2x2(s11, s12, s12, s22, b1, b2) if self.solve_xy else None

        lines = ['', '=' * 76,
                 f'  역산 결과 — 기준 [{POSITION_NAME.get(A, A)}] 고정, '
                 f'[{POSITION_NAME.get(B, B)}] 보정',
                 '=' * 76,
                 f'  캡처 {len(self.caps)}회, 법선 방향 '
                 f'{min(c["alpha_b"] for c in self.caps):+.0f} ~ '
                 f'{max(c["alpha_b"] for c in self.caps):+.0f} 도']
        lines.append('')
        lines.append(f'  yaw   {math.degrees(psiB0):+8.2f} 도  ->  {psiB_new:+8.2f} 도   '
                     f'(변화 {wrap_deg(psiB_new - math.degrees(psiB0)):+.2f}, '
                     f'캡처간 편차 {spread:.2f})')
        if sol is None:
            if not self.solve_xy:
                # yaw 만 풀었으므로, 남은 거리 불일치를 그대로 보고한다.
                res = []
                for c in self.caps:
                    ab = math.radians(wrap_deg(c['aB'] + psiB_new))
                    lhs = c['dA'] - dA + math.cos(ab) * xa + math.sin(ab) * ya
                    res.append(lhs - (c['dB'] - dB + math.cos(ab) * xb0 + math.sin(ab) * yb0))
                m = sum(res) / len(res)
                lines.append('  x/y   손대지 않음 (줄자 실측값 유지, solve_xy:=true 로 켤 수 있음)')
                lines.append('')
                lines.append(f'  남은 거리 불일치 평균 {m * 100:+6.2f} cm '
                             f'(범위 {min(res) * 100:+.2f} ~ {max(res) * 100:+.2f})')
                lines.append('    2cm 안이면 장착 위치는 문제 없다. 넘으면 x/y 또는')
                lines.append('    거리 바이어스(delta) 를 의심할 것.')
            else:
                lines.append('  x/y   풀 수 없음 — 법선 방향이 충분히 다르지 않습니다.')
                lines.append('        박스 각도를 더 크게 바꿔가며 다시 재세요.')
        else:
            xb, yb = sol
            lines.append(f'  x     {xb0:+8.3f} m  ->  {xb:+8.3f} m   (변화 {xb - xb0:+.3f})')
            lines.append(f'  y     {yb0:+8.3f} m  ->  {yb:+8.3f} m   (변화 {yb - yb0:+.3f})')
            # 잔차
            before = after = 0.0
            for c in self.caps:
                ab0 = math.radians(wrap_deg(c['aB'] + math.degrees(psiB0)))
                ab1 = math.radians(wrap_deg(c['aB'] + psiB_new))
                lhs = c['dA'] - dA + math.cos(ab0) * xa + math.sin(ab0) * ya
                before += abs(lhs - (c['dB'] - dB + math.cos(ab0) * xb0 + math.sin(ab0) * yb0))
                lhs1 = c['dA'] - dA + math.cos(ab1) * xa + math.sin(ab1) * ya
                after += abs(lhs1 - (c['dB'] - dB + math.cos(ab1) * xb + math.sin(ab1) * yb))
            n = len(self.caps)
            lines.append('')
            lines.append(f'  거리 잔차 평균  {before / n * 100:5.2f} cm  ->  {after / n * 100:5.2f} cm')
            lines.append('')
            lines.append('  stack_parking/config/lidar_mounts.yaml 에 반영:')
            key = {'a1': 'front', 'a2': 'rear', 'b1': 'left', 'b2': 'right'}[B]
            lines.append(f'    {key}:')
            lines.append(f'      x: {xb:.3f}')
            lines.append(f'      y: {yb:.3f}')
            lines.append(f'      yaw_deg: {psiB_new:.1f}')
        if spread > 4.0:
            lines.append('')
            lines.append(f'  ! yaw 추정이 캡처마다 {spread:.1f}도 흔들립니다 — 박스 면이 평평한지,')
            lines.append('    두 라이다가 같은 면을 보는지 확인하고 다시 재는 편이 좋습니다.')
        lines.append('')
        # ★ 캡처 원자료를 남긴다. 이게 없으면 delta 나 기준 센서 yaw 를 다르게
        #   가정해 볼 때마다 실물 캡처를 다시 해야 한다(2026-08-14에 그렇게 낭비했다).
        #   아래 값만 있으면 오프라인에서 어떤 가정이든 다시 풀 수 있다.
        lines.append('# ---- 캡처 원자료 (오프라인 재분석용) ----')
        lines.append(f'# ref={A} tgt={B}  '
                     f'tA=({xa:.4f},{ya:.4f}) psiA={math.degrees(psiA):.4f}  '
                     f'tB0=({xb0:.4f},{yb0:.4f}) psiB0={math.degrees(psiB0):.4f}  '
                     f'deltaA={dA:.4f} deltaB={dB:.4f}')
        for i, c in enumerate(self.caps):
            lines.append(f'#CAP {i} dA={c["dA"]:.5f} aA={c["aA"]:.4f} '
                         f'dB={c["dB"]:.5f} aB={c["aB"]:.4f} '
                         f'rmsA={c["rmsA"]:.5f} rmsB={c["rmsB"]:.5f} '
                         f'nA={c["nA"]} nB={c["nB"]} win={c["win"][0]:.1f},{c["win"][1]:.1f}')
        lines.append('')
        text = '\n'.join(lines)
        out(text)
        path = os.path.expanduser(f'~/pair_calib_{A}_{B}.txt')
        try:
            with open(path, 'w', encoding='utf-8') as f:
                f.write(text + '\n')
            out(f'  저장: {path}')
        except OSError as e:
            out(f'  ! 저장 실패: {e}')
        out()
        raise SystemExit(0)


def main():
    rclpy.init()
    node = PairCalibrate()
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
