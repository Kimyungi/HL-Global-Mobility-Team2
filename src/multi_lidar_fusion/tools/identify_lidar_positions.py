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
    keys          ["yd0","yd1","rp0","rp1"]  토픽 키 목록
    port_<key>    각 키의 시리얼 포트 경로 (출력에 그대로 찍는다)
    positions     ["front","rear","left","right"]  물어볼 순서
    near_m        이 거리 안의 반사만 "손"으로 본다 [m]
    min_arc_deg   근접 반사가 연속으로 이 각도 이상이면 가려진 것으로 판정 [deg]
    margin        1등이 2등보다 이 배수 이상 커야 확정 (옆 유닛이 같이 보는 것 방지)

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
        # 손 판정: 이 거리보다 가까운 반사가 [연속된 각도]로 이만큼 나타나면 가려진 것.
        #   거리 기준이 핵심이다 — 좌/우 유닛은 31cm 떨어져 있어, 같은 손을 봐도
        #   가린 쪽만 near_m 안에 들어온다. 이것이 두 유닛을 갈라준다.
        self.near_m = self.declare_parameter('near_m', 0.25).value
        self.min_arc_deg = self.declare_parameter('min_arc_deg', 12.0).value
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
        #   base[k]  : 빔별 "평상시 가까움" 이진 지도 — 기준선 단계에서 만든다
        #   frac[k]  : 새로 나타난 근접 반사의 최대 연속 각도 [deg]
        self.frac = {k: 0.0 for k in self.keys}
        self.seen = {k: 0 for k in self.keys}
        self.base = {k: None for k in self.keys}
        self.base_acc = {k: [] for k in self.keys}   # 기준선 단계의 원본 스캔들

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
        self.t_live = self.get_clock().now()
        self.live_period = self.declare_parameter('live_period_s', 0.5).value

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
        self.seen[key] += 1

        rmin = max(msg.range_min, 1e-3)
        # 유효하지 않거나 최소거리 미만이면 None — "반사 없음"으로 통일해서 다룬다.
        # (손을 바짝 대면 최소거리 밑으로 들어가 무효가 되는데, 그것도 '변화'다.
        #  근접 반사만 세던 이전 방식은 바로 이 경우를 놓쳤다.)
        cur = [r if (math.isfinite(r) and r > rmin) else None for r in msg.ranges]

        if self.phase == 'baseline':
            self.base_acc[key].append(cur)
            return

        near_base = self.base.get(key)
        if near_base is None:
            return
        n = len(cur)
        inc = abs(msg.angle_increment) if msg.angle_increment else 0.0
        if inc <= 0.0:
            return

        # "원래부터 가까웠던" 빔은 제외한다(차체 자기반사). 인덱스 지터를 감안해
        # 앞뒤 ±3 빔까지 봐서, 그 근처가 원래 가까웠으면 새로운 것으로 치지 않는다.
        m = len(near_base)
        newly = [False] * n
        for i in range(n):
            r = cur[i]
            if r is None or r >= self.near_m:
                continue
            was_near = False
            for j in range(i - 3, i + 4):
                if 0 <= j < m and near_base[j]:
                    was_near = True
                    break
            newly[i] = not was_near

        # 가장 긴 연속 구간 (원형 스캔이므로 배열을 두 번 이어 붙여 경계를 넘긴다)
        best = 0
        run = 0
        for i in range(2 * n):
            if newly[i % n]:
                run += 1
                best = max(best, run)
            else:
                run = 0
        best = min(best, n)
        arc_deg = math.degrees(best * inc)

        # 가벼운 저역통과 (한 프레임 튐 방지)
        self.frac[key] = 0.6 * self.frac[key] + 0.4 * arc_deg

    def build_baseline(self, key):
        """기준선 단계의 스캔들에서 '평상시 가까운 빔' 지도를 만든다.

        절대 거리 프로파일이 아니라 near/far 이진 지도다. T-mini Plus 는 프레임마다
        빔 각도가 미세하게 흔들려서 빔별 거리를 1:1 비교하면 정지 상태에서도 15%가
        '변했다'로 나온다(실측). 이진 지도는 그 지터에 훨씬 둔감하다.
        """
        acc = self.base_acc[key]
        if not acc:
            return None
        n = min(len(s) for s in acc)
        near_map = []
        for i in range(n):
            hits = sum(1 for s in acc if s[i] is not None and s[i] < self.near_m * 1.2)
            near_map.append(hits * 3 > len(acc))    # 1/3 이상 가까웠으면 상시 근접
        return near_map

    # ── 진행 ────────────────────────────────────────────────────────────
    def elapsed(self):
        return (self.get_clock().now() - self.t_phase).nanoseconds / 1e9

    def goto(self, phase):
        self.phase = phase
        self.t_phase = self.get_clock().now()

    def live_line(self):
        parts = []
        for k in self.keys:
            d = self.frac[k]                      # 근접 연속 각도 [deg]
            bar = '#' * min(int(d / 4.0), 20)
            mark = '*' if k in self.used else ' '
            parts.append(f'{k}{mark}{d:5.1f}d|{bar:<20}|')
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
                bad = []
                for k in self.keys:
                    self.base[k] = self.build_baseline(k)
                    if not self.base[k]:
                        bad.append(k)
                    self.base_acc[k] = []
                if bad:
                    out(f'   ! 기준선을 못 만든 유닛: {", ".join(bad)} — 종료합니다.')
                    rclpy.shutdown()
                    return
                out('   기준선(상시 근접 빔): ' + '  '.join(
                    f'{k}={sum(self.base[k])}/{len(self.base[k])}' for k in self.keys))
                self.ask()
            return

        if self.phase == 'ask':
            # launch 가 stdout 을 파이프로 잡으면 '\r' 만으로 갱신하는 진행바는
            # 개행이 올 때까지 버퍼에 갇혀 화면에 안 나온다. 개행으로 찍되 throttle 한다.
            if (self.get_clock().now() - self.t_live).nanoseconds / 1e9 >= self.live_period:
                out(self.live_line())
                self.t_live = self.get_clock().now()
            cand = [(self.frac[k], k) for k in self.keys if k not in self.used]
            cand.sort(reverse=True)
            if not cand:
                return
            top_d, top_k = cand[0]
            second = cand[1][0] if len(cand) > 1 else 0.0
            ok = (top_d >= self.min_arc_deg and
                  top_d >= self.margin * max(second, 1e-6))
            # 오래 헤매면 무엇이 걸림돌인지 알려준다 (좌/우처럼 가까이 붙은 쌍에서 흔함)
            if not ok and self.elapsed() > 15.0 and self.elapsed() % 8 < 0.11:
                if top_d < self.min_arc_deg:
                    out(f'   (근접 반사가 약합니다: 1등 {top_k} {top_d:.1f}deg '
                        f'< 문턱 {self.min_arc_deg:.0f}deg — 손을 {self.near_m * 100:.0f}cm '
                        f'안으로, 스캔 평면에 손바닥을 세워 막아보세요)')
                else:
                    out(f'   (두 유닛이 같이 반응: {cand[0][1]} {top_d:.1f}deg vs '
                        f'{cand[1][1]} {second:.1f}deg — 한쪽에 바짝 붙여주세요)')
            if ok:
                if self.hold_key != top_k:
                    self.hold_key = top_k
                    self.hold_since = self.get_clock().now()
                held = (self.get_clock().now() - self.hold_since).nanoseconds / 1e9
                if held >= self.hold_s:
                    pos = self.positions[self.idx]
                    self.assigned[pos] = top_k
                    self.used.add(top_k)
                    out(f'   -> {pos} = {top_k}  ({self.port_of.get(top_k, "?")})')
                    self.hold_key = None
                    self.goto('release')
            else:
                self.hold_key = None
            return

        if self.phase == 'release':
            # 손을 뗄 때까지 기다린다 (다음 질문에 잔상이 섞이지 않게)
            last = self.assigned[self.positions[self.idx]]
            d = self.frac[last]
            if d < self.min_arc_deg * 0.5:
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
