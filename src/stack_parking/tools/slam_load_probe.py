#!/usr/bin/env python3
"""SLAM 부하 시험 계측 — 전 센서를 켠 채 ICP SLAM 이 버티는가를 한 번에 잰다.

왜 필요한가
  이 프로젝트의 센서 고장은 대부분 **단독으로는 안 나타나고 같이 켰을 때만**
  나온다. 이미 실측으로 확정된 것만 셋이다:
    · OAK-D 가 USB3 로 열거되면 GPS C/N0 가 최대 16.5dB 깎인다 (CLAUDE.md §6)
      — 위성 수·HDOP·RTCM 은 정상값 그대로라 상태줄로는 안 보인다.
    · 카메라를 라이다와 같은 허브에 **버스 전원**으로 물리면 RPLiDAR 가
      health OK 인 채 /scan 0Hz 가 된다 (HANDOVER §3.7).
    · stack_traffic 이 CPU 487% 를 먹던 시절 다른 노드의 주기가 밀렸다 (PR #55).
  그래서 "SLAM 이 도는가"를 조용한 방에서 재면 아무 의미가 없다. **부하를 걸고**
  라이다·ICP·GPS·카메라를 **같은 시간축에서** 봐야 한다.

무엇을 보는가
  라이다 4대 개별 Hz · merged Hz · ICP 수용률/RMSE/대응점/맵 크기 ·
  GPS fix 품질 · 카메라 2종 Hz · 프로세스별 CPU

  ICP 지표는 /parking/diagnostics (DiagnosticArray) 에서 읽는다 — stack_parking
  이 이미 발행하고 있으므로 이 도구는 **관찰만 하고 아무것도 발행하지 않는다.**

사용
  # 라이다·융합·주차·카메라·GPS 를 각자 터미널에 띄운 뒤
  python3 src/stack_parking/tools/slam_load_probe.py --seconds 120
  python3 src/stack_parking/tools/slam_load_probe.py --seconds 300 --tag full_load
"""

import argparse
import collections
import json
import math
import statistics
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2

try:
    import psutil
except ImportError:
    psutil = None

# 부하 시험 합격선 — 근거는 RUNBOOK_parking_field_test_20260827.md §3-1 실측값.
THRESH = {
    'lidar_hz': 9.5,      # 기준 10Hz. 이 아래면 드라이버가 프레임을 흘리고 있다.
    'merged_hz': 19.0,    # 기준 20Hz.
    'icp_accept': 0.90,   # ICP 수용률. 떨어지면 맵이 안 붙는다.
    'lane_hz': 8.0,       # camera_fps 10 기준.
}
# CPU 를 묶어서 볼 프로세스 — (표시이름, 명령줄에서 찾을 문자열)
CPU_GROUPS = [
    ('ydlidar', 'ydlidar_ros2_driver_node'),
    ('rplidar', 'rplidar_node'),
    ('fusion', 'multi_lidar_fusion_node'),
    ('parking', 'stack_parking_node'),
    ('lane', 'stack_lane'),
    ('traffic', 'stack_traffic'),
    ('gps', 'stack_gps'),
]


class RateTracker:
    """토픽 도착 시각으로 순간 Hz 를 낸다 (윈도 안 샘플 수 / 실제 경과)."""

    def __init__(self, window=40):
        self.stamps = collections.deque(maxlen=window)
        self.total = 0

    def tick(self):
        self.stamps.append(time.monotonic())
        self.total += 1

    def hz(self):
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        if span <= 0:
            return 0.0
        # 마지막 샘플이 오래됐으면 죽은 것으로 본다 — 옛 윈도로 Hz 를 내면 거짓말이다.
        if time.monotonic() - self.stamps[-1] > 1.0:
            return 0.0
        return (len(self.stamps) - 1) / span


class LoadProbe(Node):

    def __init__(self, args):
        super().__init__('slam_load_probe')
        self.args = args
        self.rates = collections.defaultdict(RateTracker)
        self.samples = collections.defaultdict(list)
        self.icp = {'accepted': 0, 'total': 0, 'rmse': [], 'matches': [],
                    'map_points': [], 'slam_valid': 0}
        self.gps_quality = collections.Counter()
        self.merged_points = []

        for sid in ('a1', 'a2', 'b1', 'b2'):
            self.create_subscription(
                LaserScan, '/lidar/%s/scan' % sid,
                lambda _m, s=sid: self.rates[s].tick(), qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/lidar/merged_scan', self._on_merged, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/lidar/merged_cloud',
            lambda _m: self.rates['merged_cloud'].tick(), qos_profile_sensor_data)
        self.create_subscription(
            DiagnosticArray, '/parking/diagnostics', self._on_diag, 10)

        # 있으면 재고 없으면 0 으로 남는다 — 부하원이 안 떠 있어도 도구는 돈다.
        self._sub_optional('fma_interfaces.msg', 'LanePath',
                           '/perception/lane_path', 'lane')
        self._sub_optional('fma_interfaces.msg', 'TrafficStop',
                           '/perception/traffic_stop', 'traffic')
        self._sub_optional('fma_interfaces.msg', 'GpsPath',
                           '/perception/gps_path', 'gps', self._on_gps)

        self.t0 = time.monotonic()
        self.done = False
        self.rows = []
        self.create_timer(args.interval, self._report)

    def _sub_optional(self, module, cls_name, topic, key, cb=None):
        try:
            mod = __import__(module, fromlist=[cls_name])
            msg_cls = getattr(mod, cls_name)
        except (ImportError, AttributeError):
            self.get_logger().warn('%s 미탑재 — %s 계측 생략' % (cls_name, topic))
            return
        handler = cb if cb else (lambda _m, k=key: self.rates[k].tick())
        self.create_subscription(msg_cls, topic, handler, 10)

    def _on_merged(self, msg):
        self.rates['merged'].tick()
        valid = sum(1 for r in msg.ranges if math.isfinite(r) and r > 0.0)
        self.merged_points.append(valid)

    def _on_gps(self, msg):
        self.rates['gps'].tick()
        self.gps_quality[int(msg.fix_quality)] += 1

    def _on_diag(self, msg):
        for status in msg.status:
            if status.name != 'stack_parking/pipeline':
                continue
            kv = {item.key: item.value for item in status.values}
            self.rates['parking_diag'].tick()
            self.icp['total'] += 1
            if kv.get('icp_accepted') == 'True':
                self.icp['accepted'] += 1
            if kv.get('slam_valid') == 'True':
                self.icp['slam_valid'] += 1
            try:
                rmse = float(kv.get('icp_rmse_m', 'inf'))
                if math.isfinite(rmse):
                    self.icp['rmse'].append(rmse)
            except ValueError:
                pass
            for key, field in (('matches', 'icp_matches'),
                               ('map_points', 'map_points')):
                try:
                    self.icp[key].append(int(kv.get(field, 0)))
                except ValueError:
                    pass

    def _cpu(self):
        if psutil is None:
            return {}
        out = collections.defaultdict(float)
        for proc in psutil.process_iter(['name', 'cmdline', 'cpu_percent']):
            try:
                cmd = ' '.join(proc.info['cmdline'] or [])
            except (psutil.NoSuchProcess, TypeError):
                continue
            for label, needle in CPU_GROUPS:
                if needle in cmd:
                    out[label] += proc.info['cpu_percent'] or 0.0
                    break
        return dict(out)

    def _report(self):
        elapsed = time.monotonic() - self.t0
        row = {'t': round(elapsed, 1)}
        for key in ('a1', 'a2', 'b1', 'b2', 'merged', 'merged_cloud',
                    'parking_diag', 'lane', 'traffic', 'gps'):
            hz = self.rates[key].hz()
            row[key] = round(hz, 2)
            self.samples[key].append(hz)
        row['cpu'] = self._cpu()
        self.rows.append(row)

        accept = (self.icp['accepted'] / self.icp['total']) if self.icp['total'] else 0.0
        rmse = self.icp['rmse'][-1] if self.icp['rmse'] else float('nan')
        mappt = self.icp['map_points'][-1] if self.icp['map_points'] else 0
        cpu_txt = ' '.join('%s=%.0f%%' % (k, v) for k, v in sorted(row['cpu'].items()))
        print('[%5.1fs] lidar %.1f/%.1f/%.1f/%.1f  merged %.1f  '
              'ICP acc %.0f%% rmse %.3f map %d  lane %.1f traffic %.1f gps %.1f | %s'
              % (elapsed, row['a1'], row['a2'], row['b1'], row['b2'], row['merged'],
                 accept * 100, rmse, mappt, row['lane'], row['traffic'], row['gps'],
                 cpu_txt), flush=True)

        if elapsed >= self.args.seconds:
            self.done = True

    def summarize(self):
        def stat(key):
            vals = [v for v in self.samples[key] if v > 0]
            if not vals:
                return (0.0, 0.0, 0.0)
            return (min(vals), statistics.median(vals), max(vals))

        print('\n' + '=' * 78)
        print('SLAM 부하 시험 요약  (%.0fs%s)'
              % (time.monotonic() - self.t0, '  tag=' + self.args.tag if self.args.tag else ''))
        print('=' * 78)
        print('%-14s %8s %8s %8s   %s' % ('항목', '최소', '중앙', '최대', '판정'))
        verdicts = []

        for sid in ('a1', 'a2', 'b1', 'b2'):
            lo, mid, hi = stat(sid)
            ok = lo >= THRESH['lidar_hz']
            verdicts.append(('lidar %s' % sid, ok))
            print('%-14s %8.2f %8.2f %8.2f   %s'
                  % ('lidar ' + sid, lo, mid, hi, '✅' if ok else '❌ 프레임 유실'))

        lo, mid, hi = stat('merged')
        ok = lo >= THRESH['merged_hz']
        verdicts.append(('merged', ok))
        print('%-14s %8.2f %8.2f %8.2f   %s'
              % ('merged_scan', lo, mid, hi, '✅' if ok else '❌ 융합 지연'))

        if self.merged_points:
            print('%-14s %8d %8d %8d   %s'
                  % ('merged 유효점', min(self.merged_points),
                     int(statistics.median(self.merged_points)),
                     max(self.merged_points), '(참고: 08-27 실측 ~725pt)'))

        total = self.icp['total']
        if total:
            accept = self.icp['accepted'] / total
            ok = accept >= THRESH['icp_accept']
            verdicts.append(('ICP 수용률', ok))
            print('\nICP  수용률 %.1f%% (%d/%d)   %s'
                  % (accept * 100, self.icp['accepted'], total,
                     '✅' if ok else '❌ 맵이 안 붙는다'))
            print('     slam_valid %.1f%%' % (self.icp['slam_valid'] / total * 100))
            if self.icp['rmse']:
                r = sorted(self.icp['rmse'])
                print('     RMSE  중앙 %.4f  p95 %.4f  최대 %.4f m'
                      % (r[len(r) // 2], r[int(len(r) * 0.95)], r[-1]))
            if self.icp['matches']:
                m = self.icp['matches']
                print('     대응점 중앙 %d  최소 %d' % (int(statistics.median(m)), min(m)))
            if self.icp['map_points']:
                print('     맵 점수 %d -> %d (증가 = 누적됨)'
                      % (self.icp['map_points'][0], self.icp['map_points'][-1]))
        else:
            print('\nICP  /parking/diagnostics 무수신 — stack_parking 이 안 떠 있다.')

        if self.gps_quality:
            names = {0: 'NOFIX', 1: 'GPS', 2: 'DGPS', 4: 'FIXED', 5: 'FLOAT'}
            gtot = sum(self.gps_quality.values())
            parts = ['%s %.0f%%' % (names.get(q, str(q)), c / gtot * 100)
                     for q, c in sorted(self.gps_quality.items())]
            # ⚠ parking_standalone 은 manual_test_publish_gps_gate 로 **합성
            #   GpsPath 를 스스로 발행**한다. stack_gps 가 없으면 이 값은 실제
            #   측위가 아니라 그 시험용 게이트다 — 판정에 넣으면 거짓말이 된다.
            real_gps = 'stack_gps_node' in self.get_node_names()
            if real_gps:
                fixed = self.gps_quality.get(4, 0) / gtot
                verdicts.append(('GPS FIXED', fixed >= 0.95))
                print('\nGPS  %s   %s' % (' / '.join(parts),
                                          '✅' if fixed >= 0.95 else '⚠ FIXED 미유지'))
                print('     ⚠ C/N0 는 이 도구로 안 보인다 — 간섭 판정은 rtk_probe.py 로.')
            else:
                print('\nGPS  %s' % ' / '.join(parts))
                print('     ↑ 실제 측위 아님 — stack_gps 가 없다. parking_standalone 의'
                      ' 합성 게이트이므로 판정에서 제외한다.')

        for label, key in (('lane', 'lane'), ('traffic', 'traffic')):
            lo, mid, hi = stat(key)
            if hi > 0:
                ok = lo >= THRESH['lane_hz'] if label == 'lane' else True
                print('\n%-6s %.2f / %.2f / %.2f Hz   %s'
                      % (label, lo, mid, hi, '✅' if ok else '⚠ 카메라 프레임 저하'))
            else:
                print('\n%-6s 무수신 — 부하원이 안 떠 있다.' % label)

        cpu_last = self.rows[-1]['cpu'] if self.rows else {}
        if cpu_last:
            print('\nCPU  %s  (합 %.0f%%)'
                  % (' '.join('%s=%.0f%%' % kv for kv in sorted(cpu_last.items())),
                     sum(cpu_last.values())))

        bad = [name for name, ok in verdicts if not ok]
        print('\n' + '-' * 78)
        if bad:
            print('판정: ❌ 실패 — %s' % ', '.join(bad))
        else:
            print('판정: ✅ 전 항목 합격 — 부하 아래서 SLAM 이 유지된다')
        print('-' * 78)

        if self.args.out:
            with open(self.args.out, 'w') as fp:
                json.dump({'rows': self.rows, 'icp': self.icp,
                           'gps_quality': dict(self.gps_quality),
                           'merged_points': self.merged_points}, fp)
            print('원자료: %s' % self.args.out)


def main():
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except AttributeError:
        pass
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=120.0)
    ap.add_argument('--interval', type=float, default=5.0)
    ap.add_argument('--tag', default='')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    if psutil is None:
        print('⚠ psutil 없음 — CPU 항목은 비워둔다', file=sys.stderr)
    rclpy.init()
    node = LoadProbe(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.summarize()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
