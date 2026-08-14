#!/usr/bin/env python3
"""라이다 4대의 **거리 단위계**를 평평한 판 하나로 맞춘다 (안내형).

이 파일의 역할:
    장착 방향(yaw)과 시야(FOV)는 view_one_lidar 로 맞췄다. 남는 것은 "같은 거리를
    같은 숫자로 말하는가"다. 모델이 다르면 근거리 바이어스가 달라, 같은 벽이 병합
    화면에서 여러 겹으로 보인다.

    측정법: 각 라이다 **정면에 평평한 판을 알려진 거리**(기본 0.30m)로 세우고,
    그 방향 점들에 **직선을 적합해 수직거리**를 잰다.

    ★ 최소거리(min range) 하나만 읽으면 안 된다. 판이 몇 도만 기울어도 최소거리는
      수직거리와 달라지고, 그 차이가 그대로 "센서 바이어스"로 둔갑한다. 직선 적합은
      판의 기울기를 분리해서 함께 보고하므로, 큰 기울기는 센서가 아니라 판을 잘못
      세운 것으로 판정할 수 있다.

    ★ 사람이 표를 해석할 필요가 없게 만든 것이 이 버전의 요점이다:
      어느 라이다 앞에 판을 뒀는지 **자동으로 인식**하고, 자세가 좋아져 값이
      **안정되면 자동 저장**한 뒤 다음 라이다로 안내한다. 조작자는 판만 옮기면 된다.

입력 topic : /lidar/<id>/scan  (LaserScan)  ← **드라이버 원본**
             융합 출력(merged)이 아니다. 융합은 FOV·필터를 이미 거쳐 원인 분리가 안 된다.
출력       : 화면 안내 + 결과 요약, 그리고 결과 파일 (기본 ~/wall_calib_<거리>cm.txt)
frame      : 쓰지 않는다. 센서 좌표계의 각도·거리만 본다.

파라미터:
    ids            ["a1","a2","b1","b2"]
    front_deg      [-90.0, -90.0, 180.0, 180.0]
                   각 유닛의 **정면**이 센서 좌표계에서 몇 도인가 (ids 와 같은 순서).
                   YD T-mini Plus = -90 (raw 270), RPLiDAR C1M1 = 180. 2026-08-13 확정.
    target_m       0.30    판까지의 실제 거리 [m]
    window_deg     25.0    정면 기준 이 각도 안의 점만 판으로 본다
    r_min / r_max  0.05 / 1.00   이 거리 밖의 점은 판이 아니다 [m]
    min_points     8       이보다 적으면 "판 없음"
    max_tilt_deg   5.0     판 기울기 허용치 (넘으면 판을 돌리라고 안내)
    max_rms_m      0.010   판 평탄도 허용치 (넘으면 휘었거나 딴 물체가 섞인 것)
    stable_s       1.5     이 시간 동안 조건을 만족하고 값이 안 흔들리면 자동 확정
    stable_span_m  0.004   확정 판정에 쓰는 수직거리 변동 폭 [m]
    only           ""      **한 대만** 잰다. a1/a2/b1/b2 또는 앞/뒤/왼쪽/오른쪽.
                           비우면 4대를 한 창에서 (판을 옮겨가며) 잰다.
    out_dir        ""      결과 저장 폴더. 비우면 홈

실행:
    # 한 대씩 (창 하나 = 라이다 하나) — 권장
    ros2 run multi_lidar_fusion wall_calibrate.py --ros-args -p only:=왼쪽
    ros2 run multi_lidar_fusion wall_calibrate.py --ros-args -p only:=앞 -p target_m:=0.60

조작:
    판을 그 라이다 앞에 세우기만 하면 된다. 나머지는 화면이 안내한다.

★ 결과는 라이다별 파일(~/wall_calib_30cm_b1.txt)로 남고, 매 실행 끝에 **같은 거리의
  기존 결과를 모아 4대 비교표**를 다시 낸다. 창을 따로 써도 비교는 자동으로 된다.

관계: 여기서 나온 4대 간 편차가 1cm 를 넘으면 센서별 거리 보정값을 넣어야 한다
      (융합 노드 정규화 단계). 편차가 그 안이면 보정 없이 통과.
"""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan

# 사람이 읽는 위치 이름 — 판을 어디에 두라고 안내할 때 쓴다.
POSITION_NAME = {'a1': '앞', 'a2': '뒤', 'b1': '왼쪽', 'b2': '오른쪽'}
# only 파라미터로 받는 이름 → 슬롯. 한글/영문/슬롯명 아무거나 받는다.
SLOT_ALIAS = {
    'a1': 'a1', '앞': 'a1', 'front': 'a1', 'f': 'a1',
    'a2': 'a2', '뒤': 'a2', 'rear': 'a2', 'back': 'a2', 'r': 'a2',
    'b1': 'b1', '왼쪽': 'b1', '좌': 'b1', 'left': 'b1', 'l': 'b1',
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
    """점들에 직선을 적합해 (원점까지 수직거리, 법선 각도[deg], 잔차 RMS) 를 낸다.

    최소제곱(y = ax + b)이 아니라 **전최소제곱(PCA)** 이다. 판이 센서 정면에 서면
    점들이 거의 수직선을 이루는데, y=ax+b 형태는 그 경우 기울기가 발산한다.
    공분산의 최소 고유벡터 = 판의 법선이므로 방향에 무관하게 안정적이다.
    """
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
    det = sxx * syy - sxy * sxy
    disc = max(0.0, tr * tr / 4.0 - det)
    lam_min = tr / 2.0 - math.sqrt(disc)
    if abs(sxy) > 1e-12:
        nx, ny = lam_min - syy, sxy
    else:
        nx, ny = (1.0, 0.0) if sxx <= syy else (0.0, 1.0)
    norm = math.hypot(nx, ny)
    if norm < 1e-12:
        return None
    nx, ny = nx / norm, ny / norm
    d = abs(nx * cx + ny * cy)
    if nx * cx + ny * cy < 0:
        nx, ny = -nx, -ny
    rms = math.sqrt(max(0.0, lam_min))
    return d, math.degrees(math.atan2(ny, nx)), rms


class WallCalibrate(Node):

    def __init__(self):
        super().__init__('wall_calibrate')

        self.ids = self.declare_parameter('ids', ['a1', 'a2', 'b1', 'b2']).value
        front = [float(v) for v in self.declare_parameter(
            'front_deg', [-90.0, -90.0, 180.0, 180.0]).value]
        self.topic_tmpl = self.declare_parameter('topic_tmpl', '/lidar/{}/scan').value
        self.target = float(self.declare_parameter('target_m', 0.30).value)
        self.window = float(self.declare_parameter('window_deg', 25.0).value)
        self.r_min = float(self.declare_parameter('r_min', 0.05).value)
        self.r_max = float(self.declare_parameter('r_max', 1.00).value)
        self.min_points = int(self.declare_parameter('min_points', 8).value)
        self.max_tilt = float(self.declare_parameter('max_tilt_deg', 5.0).value)
        self.max_rms = float(self.declare_parameter('max_rms_m', 0.010).value)
        self.stable_s = float(self.declare_parameter('stable_s', 1.5).value)
        self.stable_span = float(self.declare_parameter('stable_span_m', 0.004).value)
        only = str(self.declare_parameter('only', '').value or '').strip()
        out_dir = self.declare_parameter('out_dir', '').value

        if len(front) != len(self.ids):
            out('! ids 와 front_deg 의 길이가 다르다')
            raise SystemExit(2)
        self.front_of = dict(zip(self.ids, front))

        # 한 대만 재는 모드 — 창 하나 = 라이다 하나.
        self.only = None
        if only:
            self.only = SLOT_ALIAS.get(only.lower())
            if self.only is None or self.only not in self.ids:
                out(f'! only:={only} 를 못 알아듣겠습니다. '
                    '앞 / 뒤 / 왼쪽 / 오른쪽 (또는 a1 a2 b1 b2) 중 하나로 주세요.')
                raise SystemExit(2)
            self.ids = [self.only]

        self.out_dir = os.path.expanduser(out_dir) if out_dir else os.path.expanduser('~')
        self.cm = round(self.target * 100)

        # 슬롯별 최근 적합 (d, tilt, rms, npts) — None 이면 이번 스캔에 판이 없다
        self.last = {k: None for k in self.ids}
        self.seen = {k: 0 for k in self.ids}
        self.recent = {k: [] for k in self.ids}     # 안정성 판정용 최근 수직거리
        self.hold_since = {k: None for k in self.ids}
        self.result = {}                            # 확정된 값

        for k in self.ids:
            self.create_subscription(
                LaserScan, self.topic_tmpl.format(k),
                lambda m, key=k: self.on_scan(key, m),
                qos_profile_sensor_data)

        self.last_line = ''
        self.last_say = 0.0
        self.t_start = time.time()
        self.create_timer(0.2, self.tick)

        out()
        out('=' * 70)
        if self.only:
            out(f'  [{POSITION_NAME.get(self.only, self.only)}] 라이다 거리 측정 '
                f'— 정면 {self.target * 100:.0f} cm 에 평평한 판')
        else:
            out(f'  거리 맞추기 — 라이다 정면 {self.target * 100:.0f} cm 에 평평한 판')
        out('=' * 70)
        out()
        if self.only:
            out(f'  이 창은 **{POSITION_NAME.get(self.only, self.only)} 라이다 한 대만** 봅니다.')
            out('  판을 그 앞에 세우기만 하세요. 값이 안정되면 자동으로 저장하고 끝납니다.')
        else:
            out('  판을 라이다 한 대 앞에 세우기만 하세요. 나머지는 화면이 안내합니다.')
            out('  값이 좋아지면 자동으로 저장하고 다음으로 넘어갑니다.')
        out()
        out('  준비: 판 폭 30cm 이상, 세로로 세우고, 라이다를 마주 보게')
        out('  끝내려면 Ctrl-C')
        out()
        out('-' * 70)

    # ── 수신 ────────────────────────────────────────────────────────────
    def on_scan(self, key, msg):
        """360도 전체에서 **판처럼 생긴 덩어리**를 찾는다.

        ★ 정면 창 안만 보면, 판을 다른 각도에 두었을 때 화면이 멈춘 것처럼 보이고
          "왜 반응이 없는지"를 알 길이 없다(2026-08-14에 실제로 겪음). 전체를 훑어
          가장 큰 덩어리를 찾고, 그것이 몇 도에 있는지 함께 알려준다 — 그러면 판이
          엉뚱한 곳에 있는지, 정면 각도 설정이 틀렸는지가 화면에서 바로 드러난다.
        """
        n = len(msg.ranges)
        if n == 0 or not msg.angle_increment:
            return
        self.seen[key] += 1
        inc_deg = abs(math.degrees(msg.angle_increment))
        front = self.front_of[key]

        # (각도, 거리) — 유효 거리대만
        pts = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < self.r_min or r > self.r_max:
                continue
            pts.append((wrap_deg(math.degrees(msg.angle_min + msg.angle_increment * i)), r))

        def biggest(points):
            """각도가 끊기거나 거리가 튀면 다른 물체로 보고, 가장 큰 덩어리를 고른다."""
            if len(points) < self.min_points:
                return None
            groups, cur = [], []
            for a, r in points:
                if cur and (abs(wrap_deg(a - cur[-1][0])) > 3.0 * inc_deg or
                            abs(r - cur[-1][1]) > 0.06):
                    groups.append(cur)
                    cur = []
                cur.append((a, r))
            if cur:
                groups.append(cur)
            groups = [g for g in groups if len(g) >= self.min_points]
            return max(groups, key=len) if groups else None

        # ★ 정면 창 안을 **먼저** 본다. 창 밖에서 가장 큰 덩어리를 집으면 차체 반사에
        #   붙잡힌다 — 좌/우 RPLiDAR 는 raw 0 근처에 차체가 6~7cm 로 크게 잡혀서,
        #   전체에서 최대 덩어리를 고르면 판 대신 차체를 재게 된다(2026-08-14 실측).
        #   창 안에 없을 때만 전체를 훑어 "판이 저기 있다"고 알려주는 용도로 쓴다.
        best = biggest([(a, r) for a, r in pts if abs(wrap_deg(a - front)) <= self.window])
        in_window = best is not None
        if best is None:
            best = biggest(pts)
        if best is None:
            self.last[key] = None
            self.recent[key].clear()
            self.hold_since[key] = None
            return
        xy = [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a, r in best]
        fit = fit_line(xy)
        if fit is None:
            self.last[key] = None
            return
        d, normal_deg, rms = fit
        center = wrap_deg(sum(wrap_deg(a - best[0][0]) for a, _ in best) / len(best) + best[0][0])
        # 기울기 = 판의 법선이 "센서가 그 판을 보는 방향"과 얼마나 어긋났나.
        # 0 이면 판이 시선에 정확히 수직이다.
        tilt = wrap_deg(normal_deg - center)
        span = abs(wrap_deg(best[-1][0] - best[0][0]))
        self.last[key] = (d, tilt, rms, len(best), center, span, in_window)
        self.recent[key].append(d)
        if len(self.recent[key]) > 40:
            self.recent[key].pop(0)

    # ── 안내 ────────────────────────────────────────────────────────────
    def say(self, line, force=False):
        """살아 있다는 게 보이도록 주기적으로 찍는다 (같은 말은 0.8초에 한 번)."""
        now = time.time()
        if force or line != self.last_line or now - self.last_say > 0.8:
            out(line)
            self.last_line = line
            self.last_say = now

    def active_slot(self):
        """판이 보이는 슬롯 중 점이 가장 많은 것 = 지금 재는 대상."""
        cand = [(v[3], k) for k, v in self.last.items() if v is not None]
        if not cand:
            return None
        return max(cand)[1]

    def tick(self):
        missing = [k for k in self.ids if self.seen[k] == 0]
        if missing and time.time() - self.t_start > 8.0:
            self.say(f'  ! 스캔이 안 들어오는 라이다: {", ".join(missing)}  '
                     '(드라이버가 떠 있는지 확인)')
            return

        slot = self.active_slot()
        if slot is None:
            left = ', '.join(POSITION_NAME.get(k, k) for k in self.ids if k not in self.result)
            self.say(f'  판을 찾는 중...  ({left} 라이다 앞 {self.target * 100:.0f} cm 에 '
                     f'세워주세요. {self.r_min * 100:.0f}~{self.r_max * 100:.0f} cm 안이어야 합니다)')
            return

        d, tilt, rms, npts, center, span_deg, in_window = self.last[slot]
        name = POSITION_NAME.get(slot, slot)
        front = self.front_of[slot]
        off = wrap_deg(center - front)
        where = f'raw {center % 360:.0f}도'

        # ★ 판이 그 라이다의 "정면"으로 알고 있는 방향에서 벗어나 있으면 그것부터 알린다.
        #   설정된 정면 각도가 틀렸거나, 판을 다른 라이다 앞에 둔 경우다.
        if not in_window:
            self.say(f'  [{name}] 판으로 보이는 것: {d * 100:.1f} cm @ {where} '
                     f'(폭 {span_deg:.0f}도, {npts}점)\n'
                     f'         → 설정된 정면(raw {front % 360:.0f}도)에서 {off:+.0f}도 벗어나 '
                     f'있습니다. 판 위치나 정면 설정을 확인하세요.')
            self.hold_since[slot] = None
            return

        # 자세가 안 맞으면 무엇을 고쳐야 하는지 말해준다.
        if abs(tilt) > self.max_tilt:
            way = '시계 반대' if tilt > 0 else '시계'
            self.hold_since[slot] = None
            self.say(f'  [{name}] {d * 100:.1f} cm @ {where} — 판이 {abs(tilt):.0f}도 기울었습니다. '
                     f'{way} 방향으로 조금 돌려주세요')
            return
        if rms > self.max_rms:
            self.hold_since[slot] = None
            self.say(f'  [{name}] {d * 100:.1f} cm @ {where} — 면이 고르지 않습니다 '
                     f'({rms * 1000:.0f} mm). 휜 판이거나 다른 물체가 같이 보입니다')
            return

        # 값이 흔들리지 않는지
        jitter = (max(self.recent[slot]) - min(self.recent[slot])) if self.recent[slot] else 9.9
        if jitter > self.stable_span:
            self.hold_since[slot] = None
            self.say(f'  [{name}] {d * 100:.1f} cm @ {where} — 자세 좋습니다. '
                     f'판을 고정해 주세요 (흔들림 {jitter * 1000:.0f} mm)')
            return

        now = time.time()
        if self.hold_since[slot] is None:
            self.hold_since[slot] = now
            self.say(f'  [{name}] {d * 100:.1f} cm — 측정 중...', force=True)
            return
        if now - self.hold_since[slot] < self.stable_s:
            return

        # 확정
        err = d - self.target
        self.result[slot] = dict(d=d, tilt=tilt, rms=rms, npts=npts, err=err)
        self.hold_since[slot] = None
        self.recent[slot].clear()
        mark = 'OK' if abs(err) <= 0.01 else '차이 있음'
        out(f'  ✔ [{name}] 확정  {d * 100:.2f} cm   '
            f'(기준 {self.target * 100:.0f} cm 대비 {err * 100:+.2f} cm, {mark})')
        self.last_line = ''
        remaining = [k for k in self.ids if k not in self.result]
        if remaining:
            nxt = ', '.join(POSITION_NAME.get(k, k) for k in remaining)
            out(f'    → 다음: {nxt} 라이다 앞으로 판을 옮기세요')
            out('-' * 70)
        else:
            self.finish()

    # ── 결과 ────────────────────────────────────────────────────────────
    def _file_of(self, slot):
        return os.path.join(self.out_dir, f'wall_calib_{self.cm}cm_{slot}.txt')

    def _save(self, slot, r):
        """라이다 1대의 결과를 파일로. 사람이 읽는 줄 + 기계가 읽는 #DATA 줄."""
        name = POSITION_NAME.get(slot, slot)
        text = (f'{name}({slot}) 라이다 — 기준 {self.target * 100:.0f} cm\n'
                f'  수직거리 {r["d"] * 100:.2f} cm   기준대비 {r["err"] * 100:+.2f} cm\n'
                f'  기울기 {r["tilt"]:+.1f} deg   평탄도 {r["rms"] * 1000:.1f} mm'
                f'   점 {r["npts"]}개\n'
                f'  측정 {time.strftime("%Y-%m-%d %H:%M:%S")}\n'
                f'#DATA slot={slot} target={self.target:.4f} d={r["d"]:.6f}'
                f' tilt={r["tilt"]:.2f} rms={r["rms"]:.5f} n={r["npts"]}\n')
        try:
            with open(self._file_of(slot), 'w', encoding='utf-8') as f:
                f.write(text)
            return self._file_of(slot)
        except OSError as e:
            out(f'  ! 결과 파일을 못 썼습니다: {e}')
            return None

    def _load_all(self):
        """같은 거리로 이미 잰 결과들을 모은다 — 창을 따로 써도 비교가 되게."""
        found = {}
        for slot in ('a1', 'a2', 'b1', 'b2'):
            path = self._file_of(slot)
            if not os.path.isfile(path):
                continue
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    for line in f:
                        if not line.startswith('#DATA'):
                            continue
                        kv = dict(p.split('=', 1) for p in line.split()[1:] if '=' in p)
                        found[slot] = dict(
                            d=float(kv['d']), tilt=float(kv['tilt']),
                            rms=float(kv['rms']), npts=int(kv['n']))
            except (OSError, ValueError, KeyError):
                continue
        return found

    def finish(self):
        for slot, r in self.result.items():
            path = self._save(slot, r)
            if path:
                out(f'  저장: {path}')

        allr = self._load_all()
        lines = ['', '=' * 70,
                 f'  지금까지 측정 — 기준 거리 {self.target * 100:.0f} cm',
                 '=' * 70,
                 f'  {"라이다":<8}{"측정":>10}{"기준대비":>10}{"기울기":>9}'
                 f'{"평탄도":>9}{"점수":>7}']
        for k in ('a1', 'a2', 'b1', 'b2'):
            name = POSITION_NAME.get(k, k)
            r = allr.get(k)
            if r is None:
                lines.append(f'  {name:<8}{"아직 안 잼":>12}')
                continue
            err = r['d'] - self.target
            lines.append(f'  {name:<8}{r["d"] * 100:>9.2f}cm{err * 100:>+9.2f}cm'
                         f'{r["tilt"]:>+9.1f}{r["rms"] * 1000:>8.1f}mm{r["npts"]:>7d}')

        ds = [v['d'] for v in allr.values()]
        lines.append('')
        if len(ds) >= 2:
            spread = max(ds) - min(ds)
            lines.append(f'  잰 것들 사이 최대 편차: {spread * 100:.2f} cm'
                         f'  ({len(ds)}/4 대)')
            if len(ds) == 4:
                if spread <= 0.01:
                    lines.append('  → 1 cm 이내. 거리 보정 없이 그대로 쓰면 됩니다.')
                else:
                    lines.append('  → 1 cm 초과. 센서별 거리 보정이 필요합니다.')
                    lines.append('     오프셋인지 배율인지 가리려면 60 cm 에서 한 번 더 재세요.')
        else:
            lines.append('  (비교하려면 나머지 라이다도 같은 거리로 재세요)')
        lines.append('')
        out('\n'.join(lines))
        out('  창을 닫으셔도 됩니다.')
        raise SystemExit(0)


def main():
    rclpy.init()
    node = WallCalibrate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.result:
            try:
                node.finish()
            except SystemExit:
                pass
    except SystemExit as e:
        code = e.code if isinstance(e.code, int) else 0
        node.destroy_node()
        rclpy.try_shutdown()
        # 결과를 읽을 시간을 준다 (창이 바로 닫히지 않게)
        try:
            input('\n  [Enter] 를 누르면 닫힙니다...')
        except (EOFError, KeyboardInterrupt):
            pass
        sys.exit(code)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
