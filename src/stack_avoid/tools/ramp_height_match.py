#!/usr/bin/env python3
"""경사판으로 라이다 2대의 **스캔 평면 높이차**를 재는 도구.  이기돈

  1단계  수직판을 세우고:   python3 ramp_height_match.py --baseline
  2단계  경사판으로 바꾸고: python3 ramp_height_match.py --slope 0.20

원리
  경사판은 스캔 높이에 따라 맞는 지점이 달라진다. 경사가 완만할수록 증폭된다.
      Δ거리 = Δ높이 / tanθ      (1:5 경사 → 높이차 1cm 이 거리차 5cm 로 보임)

  그런데 두 라이다는 **위치도 다르다**(크기가 달라 원점이 어긋남). 경사판 거리차에는
  높이차와 위치차가 섞인다. 그래서 **수직판**을 먼저 잰다 — 수직면은 스캔 높이가
  달라도 거리가 같으므로, 그 거리차가 곧 순수 위치차(Δx)다.

      수직판:  Δd_wall = Δx
      경사판:  Δd_ramp = Δx + Δh/tanθ
      →        Δh = tanθ × (Δd_ramp − Δd_wall)

  이 방식은 라이다를 옮기거나 위치를 실측할 필요가 없다.

주의
  · 두 판 모두 **측정 방향에 수직**으로, 같은 자리에 놓을 것 (경사판만 기울임).
  · 경사는 rise/run 으로 실측해 --slope 에 tanθ 로 준다 (10cm/50cm → 0.20).
  · 너무 완만하면(<1:8) 스침각이 커져 반사가 약해지고 값이 튄다. 1:3~1:5 권장.
  · 판이 두 라이다 **모두**에게 정면으로 보여야 한다.
"""
import argparse
import json
import math
import os
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

STATE = os.path.expanduser('~/.ramp_height_match.json')


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Ramp(Node):

    def __init__(self, a):
        super().__init__('ramp_height_match')
        self.a = a
        self.scan = {}
        self.yaw = {}
        self.buf = {'A': [], 'B': []}
        self.tf = Buffer()
        TransformListener(self.tf, self)
        for topic, tag in (('/scan_a', 'A'), ('/scan_b', 'B')):
            self.create_subscription(
                LaserScan, topic,
                lambda m, t=tag: self.scan.__setitem__(t, m),
                qos_profile_sensor_data)
        self.create_timer(0.1, self.collect)
        self.create_timer(1.0, self.report)
        self.t0 = time.time()
        mode = '수직판 기준 측정' if a.baseline else f'경사판 측정 (tanθ={a.slope})'
        print(f'=== {mode} ===')
        print(f'측정 방위 {a.bearing:+.0f}° ± {a.win:.0f}°  (base_link 기준, +x 전방)\n')
        print(f"{'A(파랑) T-mini':>20}{'B(빨강) C1':>20}{'거리차 A−B':>14}")

    def _range_at(self, tag):
        """지정 방위(base_link 기준) 창 안의 **중앙값 거리**. 라이다 자기 원점 기준."""
        m = self.scan.get(tag)
        if m is None:
            return None
        if tag not in self.yaw:
            try:
                tr = self.tf.lookup_transform('base_link', f'laser_{tag.lower()}',
                                              rclpy.time.Time())
                self.yaw[tag] = _yaw(tr.transform.rotation)
            except Exception:
                return None
        vals = []
        ang = m.angle_min
        for r in m.ranges:
            b = math.degrees(ang + self.yaw[tag])       # base_link 기준 방위
            ang += m.angle_increment
            b = (b + 180.0) % 360.0 - 180.0
            if not math.isfinite(r) or r < m.range_min or r > m.range_max:
                continue
            if abs(b - self.a.bearing) <= self.a.win:
                vals.append(r)
        return statistics.median(vals) if len(vals) >= 3 else None

    def collect(self):
        for tag in ('A', 'B'):
            v = self._range_at(tag)
            if v is not None:
                self.buf[tag].append(v)
                del self.buf[tag][:-self.a.window]

    def report(self):
        med, cells = {}, []
        for tag in ('A', 'B'):
            b = self.buf[tag]
            if len(b) < 5:
                med[tag] = None
                cells.append(f"{'— 미검출 —':>20}")
                continue
            med[tag] = statistics.median(b)
            spread = (max(b) - min(b)) * 1000
            cells.append(f"{med[tag]:12.4f}m ±{spread:3.0f}")
        line = ''.join(cells)
        if med['A'] is not None and med['B'] is not None:
            d = med['A'] - med['B']
            line += f"{d * 1000:+11.1f}mm"
            if not self.a.baseline:
                base = self._load_baseline()
                if base is None:
                    line += '   ⚠ 수직판 기준값 없음 — 먼저 --baseline 실행'
                else:
                    dh = self.a.slope * (d - base)
                    who = 'B' if dh > 0 else 'A'    # A 가 더 멀리 맞으면 A 가 더 높다
                    line += (f'  → 높이차 {abs(dh) * 1000:5.1f}mm'
                             + ('  ★ 일치' if abs(dh) < 0.002
                                else f'  ({who} 를 올릴 것)'))
        print(line, flush=True)
        if self.a.baseline and time.time() - self.t0 > self.a.seconds:
            if med['A'] is not None and med['B'] is not None:
                self._save_baseline(med['A'] - med['B'])
            raise SystemExit

    @staticmethod
    def _load_baseline():
        try:
            with open(STATE) as f:
                return json.load(f)['wall_delta_m']
        except Exception:
            return None

    @staticmethod
    def _save_baseline(v):
        with open(STATE, 'w') as f:
            json.dump({'wall_delta_m': v}, f)
        print(f'\n★ 수직판 기준값 저장: Δx = {v * 1000:+.1f}mm  →  {STATE}')
        print('   이제 경사판으로 바꾸고  --slope <tanθ>  로 다시 실행하세요.')


def main():
    p = argparse.ArgumentParser(description='경사판으로 두 라이다 스캔면 높이 맞추기')
    p.add_argument('--baseline', action='store_true',
                   help='수직판 단계: 위치차(Δx)를 재서 저장하고 종료')
    p.add_argument('--slope', type=float, default=0.20,
                   help='경사 tanθ = rise/run (10cm/50cm → 0.20)')
    p.add_argument('--bearing', type=float, default=0.0, help='측정 방위 [deg]')
    p.add_argument('--win', type=float, default=3.0, help='방위 창 반폭 [deg]')
    p.add_argument('--window', type=int, default=30, help='중앙값 프레임 수')
    p.add_argument('--seconds', type=float, default=6.0, help='--baseline 측정 시간')
    a, _ = p.parse_known_args()
    rclpy.init()
    n = Ramp(a)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
