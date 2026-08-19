#!/usr/bin/env python3
"""mark_zone — 지금 차가 서 있는 자리를 "지정 구간"으로 찍는 현장 도구.

실차 launch가 **돌고 있는 상태**에서 새 터미널로 실행한다. 구독만 하므로
시리얼·CAN을 건드리지 않는다 (stack_gps_node가 잡고 있는 포트와 무관).

  주행 → 조이스틱 모드로 전환 → 원하는 자리에 정차 → 이 도구 실행

사용:
  ros2 run stack_gps mark_zone stop --note "언덕 오르막"
  ros2 run stack_gps mark_zone stop --note "내리막"
  ros2 run stack_gps mark_zone avoid_start        # 회피를 허용할 구간
  ros2 run stack_gps mark_zone avoid_end
  ros2 run stack_gps mark_zone gps_only_start    # 차선 없이 GPS 로만 갈 구간
  ros2 run stack_gps mark_zone gps_only_end

왜 위경도로 남기나: 웨이포인트 인덱스는 트랙을 다시 기록하는 순간 전부 어긋나지만
위경도는 **장소**를 가리킨다. 같은 코스를 다시 딴 CSV에서도 그대로 쓸 수 있고,
엉뚱한 코스에 쓰면 stack_gps가 스냅 거리로 걸러 낸다.

왜 트랙 CSV에 안 넣나: 트랙(측량)과 시나리오(여기서 정차·여기서만 회피)는 수명이
다르다. 게다가 CSV 로더는 `quality != 4` 행을 버리므로(FLOAT 오염 방지), CSV 안에
표식을 넣으면 그 행이 버려질 때 **에러 없이 조용히 사라진다**.

정확도: RTK FIXED(quality=4) 표본만 쓰고, 기본 30개(약 3초)의 **중앙값**을 취한다.
정차 중 RTK 위치 잡음(sd 약 1.3cm)을 평균해 cm급으로 떨어뜨리기 위함이다.
"""
import argparse
import math
import os
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import NavSatFix

from fma_interfaces.msg import GpsPath

M_PER_DEG_LAT = 111_320.0

# 이 거리 안에 같은 종류의 지점이 이미 있으면 **새로 추가하지 않고 갱신**한다.
# 같은 자리를 두 번 찍었을 때 지점이 둘로 늘면 차가 20cm 간격으로 두 번 서게 된다
# (구간 번호가 다르면 MGM이 각각 한 번씩 정차한다 — mgm_step.cpp 소진 규칙).
MERGE_RADIUS_M = 1.5

# 구간 종류: 'stop' 은 점 하나, 나머지는 start/end 짝. 짝 처리 코드는 공통이다.
KINDS = ('stop', 'avoid_start', 'avoid_end', 'gps_only_start', 'gps_only_end')
# 접두사 → 구간 파일의 키 · 사람이 읽을 이름
ZONE_KINDS = {
    'avoid': ('avoid_zones', '회피 허용 구간'),
    'gps_only': ('gps_only_zones', 'GPS 전용 구간'),
}


def _dist_m(a_lat, a_lon, b_lat, b_lon):
    return math.hypot((a_lat - b_lat) * M_PER_DEG_LAT,
                      (a_lon - b_lon) * M_PER_DEG_LAT * math.cos(math.radians(a_lat)))


class Marker(Node):
    """gps_fix(위치) + gps_path(fix 품질)를 함께 본다.

    NavSatFix의 status는 FIX/NO_FIX만 구분하고 **RTK FIXED인지는 안 알려준다**.
    품질은 GpsPath.fix_quality(4 = RTK FIXED)에 있으므로 두 토픽을 같이 구독한다.
    둘은 같은 콜백에서 연달아 발행되므로 시각 정합은 신경 쓸 필요가 없다.
    """

    def __init__(self):
        super().__init__('mark_zone')
        self.samples = []
        self.quality = None
        self.q_t = 0.0
        self.create_subscription(NavSatFix, '/perception/gps_fix', self._on_fix, 10)
        self.create_subscription(GpsPath, '/perception/gps_path', self._on_path, 10)

    def _on_path(self, m):
        self.quality = m.fix_quality
        self.q_t = time.monotonic()

    def _on_fix(self, m):
        if self.quality == 4 and time.monotonic() - self.q_t < 0.5:
            self.samples.append((m.latitude, m.longitude))

    def waypoint_csv(self, timeout=3.0):
        """돌고 있는 stack_gps_node에게 트랙 CSV 경로를 물어본다.

        구간 파일 이름을 트랙에서 유도하기 위함이다 — 사람이 매번 경로를 적으면
        엉뚱한 트랙의 구간 파일에 섞여 들어간다.
        """
        cli = self.create_client(GetParameters, '/stack_gps_node/get_parameters')
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        req = GetParameters.Request()
        req.names = ['waypoint_csv']
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.result() is None or not fut.result().values:
            return None
        return fut.result().values[0].string_value or None


def zones_path_for(waypoint_csv):
    """waypoints_<이름>.csv → 같은 폴더의 zones_<이름>.yaml."""
    d, base = os.path.split(waypoint_csv)
    stem = base[:-4] if base.endswith('.csv') else base
    if stem.startswith('waypoints_'):
        stem = stem[len('waypoints_'):]
    return os.path.join(d, f'zones_{stem}.yaml')


def load_zones(path):
    """구간 파일 읽기 — 없으면 빈 구조.

    기본 키는 **한 곳에서** 만든다. 종류를 늘릴 때 조기 반환 쪽을 빠뜨리면
    새 종류를 찍는 순간 KeyError 로 죽는다 (2026-08-18 시험에서 실제로 걸렸다).
    """
    z = {}
    if os.path.isfile(path):
        with open(path) as f:
            z = yaml.safe_load(f) or {}
    z.setdefault('stop_points', [])
    for key, _ in ZONE_KINDS.values():
        z.setdefault(key, [])
    return z


def save_zones(path, zones, track):
    zones['track'] = os.path.basename(track) if track else zones.get('track', '')
    header = (
        '# stack_gps 지정 구간 — `ros2 run stack_gps mark_zone` 이 자동 기록.\n'
        '# 위경도는 RTK FIXED 표본의 중앙값(기본 3초). 손으로 고쳐도 된다.\n'
        '#   stop_points : 그 지점에서 정지 (정차 시간은 MGM stop_zone_hold_cycles)\n'
        '#   avoid_zones : 이 구간 안에서만 회피 허용 (MGM avoid_zone_only 와 짝)\n'
        '#   gps_only_zones : 이 구간에서는 차선 전이 없이 GPS(WAYPOINT)로만 주행\n'
        '# launch 가 waypoint_csv 옆의 이 파일을 자동으로 읽는다.\n')
    with open(path, 'w') as f:
        f.write(header)
        yaml.safe_dump(zones, f, allow_unicode=True, sort_keys=False)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='지금 위치를 정지 지점 / 회피 구간 끝점으로 기록한다')
    ap.add_argument('kind', choices=KINDS)
    ap.add_argument('--note', default='', help='사람이 알아볼 메모 (예: "언덕 오르막")')
    ap.add_argument('--out', default='', help='구간 파일 경로 (기본: 트랙 CSV 옆 zones_*.yaml)')
    ap.add_argument('--samples', type=int, default=30, help='중앙값에 쓸 표본 수 (10Hz)')
    ap.add_argument('--timeout', type=float, default=15.0)
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = Marker()
    try:
        out = args.out
        track = None
        if not out:
            track = node.waypoint_csv()
            if not track:
                print('✖ 돌고 있는 stack_gps_node 를 못 찾았다 — 실차 launch(V2)가 떠 있어야 한다.\n'
                      '   (launch 없이 쓰려면 --out 으로 구간 파일 경로를 직접 지정)', file=sys.stderr)
                return 1
            out = zones_path_for(track)

        print(f'수집 중… RTK FIXED 표본 {args.samples}개 (약 {args.samples / 10:.0f}초). '
              '차를 움직이지 마세요.', flush=True)
        t_end = time.monotonic() + args.timeout
        while len(node.samples) < args.samples and time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.1)

        if len(node.samples) < args.samples:
            got, q = len(node.samples), node.quality
            why = ('gps_path 미수신 — stack_gps 확인' if q is None
                   else f'fix_quality={q} (4=RTK FIXED 아님)' if q != 4
                   else 'gps_fix 갱신 없음')
            print(f'✖ 표본 부족 ({got}/{args.samples}) — {why}\n'
                  '   FIXED 가 잡힌 뒤 다시 실행하세요 (기록된 지점 없음).', file=sys.stderr)
            return 1

        lat = sorted(s[0] for s in node.samples)[len(node.samples) // 2]
        lon = sorted(s[1] for s in node.samples)[len(node.samples) // 2]
        spread = max(_dist_m(lat, lon, *s) for s in node.samples)

        # 지금 찍는 자리가 정말 이 트랙 위인지 **여기서** 확인한다. stack_gps 도
        # 기동 시 같은 검사를 하지만, 그때는 이미 현장을 떠난 뒤라 손쓸 수 없다.
        if track and os.path.isfile(track):
            try:
                from stack_gps.path_engine import PathEngine, load_waypoints_csv
                eng = PathEngine(load_waypoints_csv(track))
                _, snap = eng.index_of(lat, lon)
                if snap > 5.0:
                    print(f'✖ 이 지점이 트랙에서 {snap:.1f}m 떨어져 있다 (한계 5m) — '
                          '기록하지 않는다.\n'
                          '   트랙을 벗어난 자리이거나 waypoint_csv 가 다른 코스다.',
                          file=sys.stderr)
                    return 1
                print(f'   트랙 스냅 {snap:.2f}m (웨이포인트 기준)')
            except Exception as e:                            # noqa: BLE001
                print(f'   (트랙 스냅 확인 생략: {e})')

        zones = load_zones(out)
        entry = {'lat': round(lat, 7), 'lon': round(lon, 7),
                 'note': args.note, 'marked': time.strftime('%Y-%m-%d %H:%M:%S')}

        if args.kind == 'stop':
            near = next((p for p in zones['stop_points']
                         if _dist_m(lat, lon, p['lat'], p['lon']) <= MERGE_RADIUS_M), None)
            if near:
                d = _dist_m(lat, lon, near['lat'], near['lon'])
                near.update(entry)
                if not args.note:
                    near['note'] = near.get('note', '')
                print(f'↻ {MERGE_RADIUS_M}m 안에 이미 있던 정지 지점을 갱신 (이동 {d:.2f}m) '
                      '— 같은 자리에 두 번 서는 것 방지')
            else:
                zones['stop_points'].append(entry)
            n = len(zones['stop_points'])
            print(f'✔ 정지 지점 {n}번: {lat:.7f}, {lon:.7f}  (표본 산포 {spread * 100:.1f}cm)')
        else:
            prefix, edge = args.kind.rsplit('_', 1)     # avoid_start → ('avoid','start')
            key, label = ZONE_KINDS[prefix]
            zl = zones[key]
            if edge == 'start':
                if zl and 'end' not in zl[-1]:
                    print('↻ 끝점을 안 찍은 시작점이 있어 그것을 갱신한다')
                    zl[-1]['start'] = entry
                else:
                    zl.append({'start': entry})
                print(f'✔ {label} {len(zl)}번 **시작**: {lat:.7f}, {lon:.7f} '
                      f'(표본 산포 {spread * 100:.1f}cm)\n'
                      f'   → 구간 끝에서 `ros2 run stack_gps mark_zone {prefix}_end`')
            else:
                if not zl or 'end' in zl[-1]:
                    print(f'✖ 짝이 될 시작점이 없다 — 먼저 {prefix}_start 를 찍어야 한다.',
                          file=sys.stderr)
                    return 1
                zl[-1]['end'] = entry
                s = zl[-1]['start']
                print(f'✔ {label} {len(zl)}번 **끝**: {lat:.7f}, {lon:.7f} '
                      f'(구간 길이 {_dist_m(lat, lon, s["lat"], s["lon"]):.1f}m)')

        save_zones(out, zones, track)
        print(f'→ {out}')
        print('   ⚠ 반영은 **V2 재기동 후**다 (구간은 stack_gps 기동 시 인덱스로 변환된다).')
        return 0
    except KeyboardInterrupt:
        print('\n중단 — 기록된 지점 없음.', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():        # 외부 SIGINT 로 이미 내려간 경우 이중 shutdown 방지
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
