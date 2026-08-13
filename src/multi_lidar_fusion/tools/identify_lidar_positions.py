#!/usr/bin/env python3
"""라이다 4대 중 어느 유닛이 앞/뒤/좌/우인지 손으로 가려서 확정하는 도구.

이 파일의 역할:
    시리얼 포트만 봐서는 어느 물리 유닛이 어느 장착 위치인지 알 수 없다
    (특히 YDLiDAR 2대는 시리얼이 둘 다 `0001` 이라 by-id 로도 구분되지 않는다).
    4대를 동시에 띄워 놓고 "앞 라이다를 손으로 가려주세요" 라고 물은 뒤,
    근접 반사가 급증한 유닛을 그 위치로 배정한다.

입력 topic : /probe/<key>/scan  (LaserScan)   ← identify_positions.launch.py 가 띄운 드라이버들
출력       : 화면에 위치↔포트 매핑 + 그대로 붙여넣을 YAML 조각
frame      : 사용 안 함 (거리값만 본다)

파라미터:
    keys        ["yd0","yd1","rp0","rp1"]  토픽 키 목록
    ports       위 키와 같은 순서의 시리얼 포트 경로 (출력에 그대로 찍는다)
    positions   ["front","rear","left","right"]  물어볼 순서
    close_m     이 거리보다 가까우면 "가려짐 후보" [m]
    rise        전체 빔 대비 근접 빔 비율이 기준선보다 이만큼 오르면 가려진 것으로 본다
    margin      1등이 2등보다 이 배수 이상 커야 확정 (옆 유닛이 같이 보는 것 방지)

관계: 여기서 나온 매핑을 config/lidar_extrinsics.yaml 의 sensors.<id>.* 와
      launch/multi_lidar_drivers.launch.py 의 DEFAULT_PORTS 에 반영한다.
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


def out(msg=''):
    print(msg, flush=True)


class Identifier(Node):

    def __init__(self):
        super().__init__('identify_lidar_positions')

        self.keys = self.declare_parameter(
            'keys', ['yd0', 'yd1', 'rp0', 'rp1']).value
        self.positions = self.declare_parameter(
            'positions', ['front', 'rear', 'left', 'right']).value
        self.close_m = self.declare_parameter('close_m', 0.30).value
        self.rise = self.declare_parameter('rise', 0.04).value
        self.margin = self.declare_parameter('margin', 1.5).value
        self.baseline_s = self.declare_parameter('baseline_s', 3.0).value
        self.hold_s = self.declare_parameter('hold_s', 0.6).value

        # 포트는 키마다 **개별 파라미터**로 받는다.
        # launch 에서 LaunchConfiguration 여러 개를 리스트로 넘기면 launch_ros 가
        # 하나의 문자열로 이어붙여 버려서(STRING_ARRAY 가 되지 않는다) 노드가 죽는다.
        self.port_of = {
            k: self.declare_parameter('port_' + k, '?').value for k in self.keys
        }

        # 키별 상태
        self.frac = {k: 0.0 for k in self.keys}      # 최근 근접 빔 비율
        self.seen = {k: 0 for k in self.keys}        # 수신 스캔 수
        self.base = {k: None for k in self.keys}     # 기준선
        self.base_acc = {k: [] for k in self.keys}

        for k in self.keys:
            topic = f'/probe/{k}/scan'
            self.create_subscription(
                LaserScan, topic,
                lambda m, key=k: self.on_scan(key, m),
                qos_profile_sensor_data)

        # 진행 상태
        self.phase = 'wait'      # wait -> baseline -> ask -> confirm -> done
        self.idx = 0             # 몇 번째 위치를 묻는 중인가
        self.assigned = {}       # position -> key
        self.used = set()
        self.hold_since = None
        self.hold_key = None
        self.t_phase = self.get_clock().now()

        self.create_timer(0.1, self.tick)

        out()
        out('=' * 68)
        out(' 라이다 위치 식별 — 손으로 가려서 앞/뒤/좌/우를 확정한다')
        out('=' * 68)
        for k in self.keys:
            out(f'   {k:<5} <- {self.port_of.get(k, "?")}')
        out()
        out(' 4대 모두 스캔이 들어오기를 기다리는 중...')

    # ── 수신 ────────────────────────────────────────────────────────────
    def on_scan(self, key, msg):
        n = len(msg.ranges)
        if n == 0:
            return
        rmin = max(msg.range_min, 1e-3)
        close = 0
        for r in msg.ranges:
            if math.isfinite(r) and rmin < r < self.close_m:
                close += 1
        # 전체 빔 대비 비율 — 가려서 무효가 된 빔이 있어도 분모가 흔들리지 않는다.
        f = close / float(n)
        # 가벼운 저역통과 (한 프레임 튐 방지)
        self.frac[key] = 0.6 * self.frac[key] + 0.4 * f
        self.seen[key] += 1
        if self.phase == 'baseline':
            self.base_acc[key].append(f)

    # ── 진행 ────────────────────────────────────────────────────────────
    def elapsed(self):
        return (self.get_clock().now() - self.t_phase).nanoseconds / 1e9

    def goto(self, phase):
        self.phase = phase
        self.t_phase = self.get_clock().now()

    def live_line(self):
        parts = []
        for k in self.keys:
            b = self.base[k] if self.base[k] is not None else 0.0
            d = max(self.frac[k] - b, 0.0)
            bar = '#' * min(int(d * 60), 20)
            mark = '*' if k in self.used else ' '
            parts.append(f'{k}{mark}{d * 100:5.1f}%|{bar:<20}|')
        return '  ' + ' '.join(parts)

    def tick(self):
        if self.phase == 'wait':
            missing = [k for k in self.keys if self.seen[k] == 0]
            if not missing:
                out(' 4대 모두 수신 확인.')
                out()
                out(f' 기준선 측정 중 ({self.baseline_s:.0f}초) — 라이다에서 손을 떼고 계세요.')
                self.goto('baseline')
            elif self.elapsed() > 15.0:
                out()
                out(f' ! 스캔이 안 들어오는 유닛: {", ".join(missing)}')
                out('   드라이버가 떴는지 / 포트가 맞는지 확인하세요. 종료합니다.')
                rclpy.shutdown()
            return

        if self.phase == 'baseline':
            if self.elapsed() >= self.baseline_s:
                for k in self.keys:
                    acc = self.base_acc[k]
                    self.base[k] = sum(acc) / len(acc) if acc else 0.0
                out('   기준선: ' + '  '.join(
                    f'{k}={self.base[k] * 100:.1f}%' for k in self.keys))
                self.ask()
            return

        if self.phase == 'ask':
            print('\r' + self.live_line(), end='', flush=True)
            cand = [(self.frac[k] - (self.base[k] or 0.0), k)
                    for k in self.keys if k not in self.used]
            cand.sort(reverse=True)
            if not cand:
                return
            top_d, top_k = cand[0]
            second = cand[1][0] if len(cand) > 1 else 0.0
            ok = top_d >= self.rise and top_d >= self.margin * max(second, 1e-6)
            if ok:
                if self.hold_key != top_k:
                    self.hold_key = top_k
                    self.hold_since = self.get_clock().now()
                held = (self.get_clock().now() - self.hold_since).nanoseconds / 1e9
                if held >= self.hold_s:
                    pos = self.positions[self.idx]
                    self.assigned[pos] = top_k
                    self.used.add(top_k)
                    print('\r' + ' ' * 100, end='')
                    out(f'\r   -> {pos} = {top_k}  ({self.port_of.get(top_k, "?")})')
                    self.hold_key = None
                    self.goto('release')
            else:
                self.hold_key = None
            return

        if self.phase == 'release':
            # 손을 뗄 때까지 기다린다 (다음 질문에 잔상이 섞이지 않게)
            last = self.assigned[self.positions[self.idx]]
            d = self.frac[last] - (self.base[last] or 0.0)
            if d < self.rise * 0.5:
                self.idx += 1
                if self.idx >= len(self.positions) or len(self.used) >= len(self.keys):
                    self.finish()
                else:
                    self.ask()
            elif self.elapsed() > 20.0:
                out('   (손을 떼주세요)')
                self.t_phase = self.get_clock().now()
            return

    def ask(self):
        pos = self.positions[self.idx]
        ko = {'front': '앞(전방)', 'rear': '뒤(후방)',
              'left': '왼쪽', 'right': '오른쪽'}.get(pos, pos)
        out()
        out(f' [{self.idx + 1}/{len(self.positions)}]  ** {ko} ** 라이다를 손으로 가려주세요'
            f'  (10cm 이내, {self.hold_s:.1f}초 유지)')
        self.goto('ask')

    def finish(self):
        # 아직 안 배정된 유닛이 하나 남았으면 자동 배정
        left_pos = [p for p in self.positions if p not in self.assigned]
        left_key = [k for k in self.keys if k not in self.used]
        if len(left_pos) == 1 and len(left_key) == 1:
            self.assigned[left_pos[0]] = left_key[0]
            out(f'   -> {left_pos[0]} = {left_key[0]}  (남은 하나라 자동 배정)')

        out()
        out('=' * 68)
        out(' 결과')
        out('=' * 68)
        slot = {'front': 'a1', 'rear': 'a2', 'left': 'b1', 'right': 'b2'}
        for pos in self.positions:
            k = self.assigned.get(pos)
            if k is None:
                out(f'   {pos:<6} : (미확정)')
                continue
            out(f'   {pos:<6} = slot {slot.get(pos, "?"):<3} <- {k:<5} '
                f'{self.port_of.get(k, "?")}')

        out()
        out(' multi_lidar_drivers.launch.py 의 DEFAULT_PORTS 에 붙여넣기:')
        out()
        out('DEFAULT_PORTS = {')
        for pos in self.positions:
            k = self.assigned.get(pos)
            if k is None:
                continue
            out(f"    '{slot.get(pos, '?')}': '{self.port_of.get(k, '?')}',"
                f"   # {pos}")
        out('}')
        out()
        rclpy.shutdown()


def main():
    rclpy.init(args=sys.argv)
    node = Identifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        out('\n 중단됨.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
