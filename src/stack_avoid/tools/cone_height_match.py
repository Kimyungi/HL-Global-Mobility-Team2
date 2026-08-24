#!/usr/bin/env python3
"""콘으로 라이다 2대의 **스캔 평면 높이차**를 재는 도구.  이기돈

원리 — 콘은 위로 갈수록 가늘어진다. 같은 콘을 봐도 **낮게 스캔하는 쪽이 더 넓게** 본다.
두 라이다가 재는 **콘 폭(현 길이, m)** 이 같아지면 스캔 평면이 같은 높이다.

  python3 src/stack_avoid/tools/cone_height_match.py

  A(파랑) /scan_a   B(빨강) /scan_b   ← dual_lidar.launch.py 기준

★ 왜 각도가 아니라 폭(m)인가 — 두 라이다는 콘까지 거리가 서로 다르다. 각도폭은 거리에
  따라 변하지만 **실제 폭(m)은 거리와 무관**하다. 그래서 위치를 정확히 안 맞춰도 된다.

★ 왜 절대 높이를 안 재는가 — 데이터시트의 광학중심 오프셋을 몰라도, 두 값이 **같아질
  때까지** 심을 넣으면 되는 널(null) 측정이다. 바닥·윗면을 맞추는 건 틀린 방법이다
  (모델마다 base→스캔면 오프셋이 다르다).

콘 치수를 주면 높이차(mm)까지 환산해 준다 (선택):
  --cone-base-d 0.30 --cone-h 0.50     # 밑면 지름 0.30m, 높이 0.50m
      기울기 = (밑면지름/2)/높이 → Δ높이 = Δ폭 / (2·기울기)

사용 절차
  1) 콘을 두 라이다 **모두 보이는 곳**에 둔다 (0.3~3m).
  2) 이 도구를 켜고 두 폭이 안정적으로 읽히는지 확인한다.
  3) 낮은 쪽(=폭이 넓은 쪽) 라이다 밑에 심을 넣어 올린다.
  4) 두 폭이 같아지면 완료. 그 상태의 실제 높이를 자로 재서 TF z 에 기록한다.
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def _clusters(scan, rmin, rmax, gap):
    """스캔 → (거리, 각도) 점들을 인접 간격으로 묶은 클러스터 목록."""
    pts = []
    a = scan.angle_min
    for r in scan.ranges:
        ang = a
        a += scan.angle_increment
        if not math.isfinite(r) or not (rmin <= r <= rmax):
            pts.append(None)
            continue
        pts.append((r * math.cos(ang), r * math.sin(ang), r))
    out, cur = [], []
    for p in pts:
        if p is None:
            if len(cur) >= 2:
                out.append(cur)
            cur = []
            continue
        if cur and math.dist(p[:2], cur[-1][:2]) > gap:
            if len(cur) >= 2:
                out.append(cur)
            cur = []
        cur.append(p)
    if len(cur) >= 2:
        out.append(cur)
    return out


def _cone(scan, rmin, rmax, gap, minpts, amin=-180.0, amax=180.0):
    """가장 가까운 유효 클러스터 = 콘으로 본다. → (폭 m, 중심거리 m, 점수, 방위 deg).

    ★ 방위 창(amin~amax)으로 콘만 남길 것. 벤치에서는 **상대 라이다 하우징**이 콘보다
      가까울 수 있어, 창을 안 주면 그쪽을 콘으로 오인한다(실측에서 실제로 발생).
    """
    best = None
    for c in _clusters(scan, rmin, rmax, gap):
        if len(c) < minpts:
            continue
        cx = sum(p[0] for p in c) / len(c)
        cy = sum(p[1] for p in c) / len(c)
        bear = math.degrees(math.atan2(cy, cx))
        if not (amin <= bear <= amax):
            continue
        d = min(p[2] for p in c)
        if best is None or d < best[0]:
            best = (d, c, bear)
    if best is None:
        return None
    _, c, bear = best
    width = math.dist(c[0][:2], c[-1][:2])          # 현 길이 = 실제 폭
    rng = sum(p[2] for p in c) / len(c)
    return width, rng, len(c), bear


class Matcher(Node):

    def __init__(self, a):
        super().__init__('cone_height_match')
        self.a = a
        self.last = {}
        for topic, tag in (('/scan_a', 'A'), ('/scan_b', 'B')):
            self.create_subscription(
                LaserScan, topic,
                lambda m, t=tag: self.last.__setitem__(t, m),
                qos_profile_sensor_data)
        self.slope = None
        if a.cone_base_d and a.cone_h:
            self.slope = (a.cone_base_d / 2.0) / a.cone_h
        self.create_timer(0.5, self.tick)
        print('콘 폭 비교 — 두 값이 같아지면 스캔 평면 높이가 같다. Ctrl-C 종료\n')
        print(f"{'':6}{'A(파랑) T-mini':>22}{'B(빨강) C1':>22}   {'폭 차이':>10}")

    def tick(self):
        res = {}
        for tag in ('A', 'B'):
            m = self.last.get(tag)
            res[tag] = _cone(m, self.a.rmin, self.a.rmax,
                             self.a.gap, self.a.minpts) if m else None

        def fmt(r):
            if r is None:
                return f"{'— 콘 미검출 —':>22}"
            return f"{r[0] * 1000:8.0f}mm @{r[1]:5.2f}m {r[2]:3d}pt"
        line = f"{'':6}{fmt(res['A'])}{fmt(res['B'])}"
        if res['A'] and res['B']:
            dw = res['A'][0] - res['B'][0]
            line += f"   {dw * 1000:+8.0f}mm"
            if self.slope:
                dh = dw / (2.0 * self.slope)
                # 폭이 넓은 쪽이 더 낮다 → 그쪽을 올려야 한다
                who = 'A' if dw > 0 else 'B'
                line += f"  → 높이차 {abs(dh) * 1000:.0f}mm ({who} 를 올릴 것)"
        print(line, flush=True)


def main():
    ap = argparse.ArgumentParser(description='콘으로 두 라이다 스캔면 높이 맞추기')
    ap.add_argument('--rmin', type=float, default=0.20, help='콘 탐색 최소거리 [m]')
    ap.add_argument('--rmax', type=float, default=3.0, help='콘 탐색 최대거리 [m]')
    ap.add_argument('--gap', type=float, default=0.08, help='클러스터 분리 간격 [m]')
    ap.add_argument('--minpts', type=int, default=3, help='콘으로 인정할 최소 점수')
    ap.add_argument('--cone-base-d', type=float, default=0.0, help='콘 밑면 지름 [m]')
    ap.add_argument('--cone-h', type=float, default=0.0, help='콘 높이 [m]')
    a, _ = ap.parse_known_args()
    rclpy.init()
    n = Matcher(a)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
