#!/usr/bin/env python3
"""field_session bag → 회피 판정 체인 분석.  이기돈

한 회피 세션에서 확인해야 하는 것은 언제나 같은 3단 체인이다:

    인지가 본 것(/perception/avoid 초록점)
      → 우리가 보낸 것(/adas/target_ref 첫 점)
        → dSPACE 가 실행한 것(/vehicle/vector.str)

셋을 시간축에 나란히 놓아야 "안 비켰다"의 원인이 인지인지·송신 기하인지·실행 게인인지
갈린다. 매번 손으로 sqlite 를 열던 것을 고정한다.

    python3 analyze_session.py <bag_dir 또는 .db3>
    python3 analyze_session.py <bag> --from 3 --to 9      # 구간 한정
    python3 analyze_session.py <bag> --rates              # 토픽 주기만

★ 궤적(x·y·yaw)은 /vehicle/vector 값을 쓰지 않고 v·str 로 적분한다 — dSPACE 가
  pose 필드를 0 으로만 보내고 있기 때문(2026-08-09 확인, CLAUDE.md §3 RX 미준수).
  정차 구간 str 평균을 바이어스로 빼고 적분한다(실측 −3.1° 의 조향 영점 오차).
"""
import argparse
import glob
import json
import math
import os
import sqlite3
import statistics
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WHEELBASE_M = 0.595          # params.yaml vehicle.wheelbase_m
CORRIDOR_HALF_M = 0.46       # width/2 + lateral_margin — 합격선
VEHICLE_HALF_W = 0.31        # 차폭/2 — 이격에서 이걸 빼면 차 옆면~장애물 실제 틈
LIDAR_X_M = 0.76             # 후축 원점 → LiDAR(앞범퍼)
# ★ /scan·/scan_front 은 **드라이버 원각도 그대로**다 — 전방이 0 이 아니라 raw 270°.
#   node.py 가 매 점마다 wrap_to_pi(angle − front_center) 로 상대각을 만든다.
#   이걸 빼먹고 angle=0 을 전방으로 읽으면 옆벽을 "전방 장애물"로 집계해, 차가
#   6m 를 갔는데도 최근접이 3.7m 로 고정된 것처럼 보인다 (실제로 한 번 속았다).
FORWARD_ANGLE_DEG = 270.0    # params.yaml lidar_mount.forward_angle_deg


def resolve_db(path):
    if os.path.isdir(path):
        hits = sorted(glob.glob(os.path.join(path, '*.db3')))
        if not hits:
            sys.exit(f'✗ {path} 안에 .db3 가 없다')
        return hits[-1]
    return path


class Bag:
    def __init__(self, db):
        self.conn = sqlite3.connect(db)
        cur = self.conn.cursor()
        self.topics = {n: (i, t) for i, n, t in
                       cur.execute('SELECT id, name, type FROM topics')}
        self.t0 = cur.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0]

    def sec(self, ts):
        return (ts - self.t0) / 1e9

    def read(self, name):
        if name not in self.topics:
            return []
        tid, typ = self.topics[name]
        mt = get_message(typ)
        return [(self.sec(ts), deserialize_message(d, mt)) for ts, d in self.conn.execute(
            'SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp', (tid,))]


def at(series, t):
    """t 이하 마지막 샘플 (zero-order hold)."""
    lo, hi = 0, len(series) - 1
    if hi < 0:
        return None
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if series[mid][0] <= t:
            lo = mid
        else:
            hi = mid - 1
    return series[lo][1]


def print_rates(bag):
    print(f"{'topic':40} {'n':>5} {'Hz':>6} {'dt중앙':>8} {'dt최대':>8}")
    for name in sorted(bag.topics):
        tid, _ = bag.topics[name]
        ts = [r[0] for r in bag.conn.execute(
            'SELECT timestamp FROM messages WHERE topic_id=? ORDER BY timestamp', (tid,))]
        if len(ts) < 2:
            print(f'{name:40} {len(ts):>5}')
            continue
        d = [(b - a) / 1e6 for a, b in zip(ts, ts[1:])]
        print(f'{name:40} {len(ts):>5} {len(ts)/((ts[-1]-ts[0])/1e9):>6.1f} '
              f'{statistics.median(d):>7.1f}ms {max(d):>7.1f}ms')


def print_chain(bag, t_from, t_to):
    av, tr, vv = (bag.read(n) for n in
                  ('/perception/avoid', '/adas/target_ref', '/vehicle/vector'))
    if not av:
        sys.exit('✗ /perception/avoid 없음 — field_session bag 이 맞나')
    print('=== 인지 초록점 → 송신 ref → 실행 조향 ===')
    print(' t     | state | 목표점 x,y    | ttc  | avoidable | 송신 x,y,κ         '
          '| str[°] | v_ref | v')
    for t, m in av:
        if not t_from <= t <= t_to:
            continue
        rm, vm = at(tr, t), at(vv, t)
        gx, gy = (m.points[0].x, m.points[0].y) if m.points else (float('nan'),) * 2
        p = rm.ref_points[0] if (rm and rm.ref_points) else None
        state = {0: 'LANE', 1: 'WP', 2: 'AVOID', 3: 'PARK'}.get(rm.state if rm else -1, '?')
        sent = (f'{p.x:5.2f},{p.y:+.3f},{p.curvature:+.3f}' if p else '   (없음)      ')
        drive = (f'{math.degrees(vm.str):+6.2f} | {rm.v_ref:5.2f} | {vm.v:+.2f}'
                 if vm and rm else '  (없음)')
        print(f'{t:6.2f} | {state:5} | {gx:5.2f},{gy:+5.2f} | {min(m.ttc, 99):5.2f} | '
              f'{str(m.avoidable):9} | {sent} | {drive}')


def print_estop(bag, t_from, t_to):
    print('\n=== estop 전이 ===')
    for topic in ('/perception/estop', '/perception/static_estop', '/perception/dynamic_estop'):
        prev, out = None, []
        for t, m in bag.read(topic):
            val = m.estop if hasattr(m, 'estop') else m.data
            if val != prev:
                out.append(f'{t:.1f}s→{val}')
                prev = val
        print(f'{topic:34} {" ".join(out) if out else "(없음)"}')

    st = bag.read('/perception/estop/status')
    if st:
        print('\n=== estop status: 최근접 클러스터 (구간 내, 값 있는 것만) ===')
        for t, m in st:
            if not t_from <= t <= t_to:
                continue
            j = json.loads(m.data)
            near = j.get('static_nearest_cluster_min_x')
            if near is not None or j.get('final_estop'):
                print(f't={t:6.2f}s nearest={near} dyn_tracks={j.get("dynamic_track_count")} '
                      f'final={j.get("final_estop")} reason={j.get("reason")}')


def _nearest_ahead(scan, fan_deg=35.0, max_r=6.0, corridor_only=False):
    """전방 부채꼴 안 최근접 반사점 → (gap, y). gap 은 앞범퍼(=LiDAR) 기준."""
    fc = math.radians(FORWARD_ANGLE_DEG)
    fan = math.radians(fan_deg)
    best, a = None, scan.angle_min
    for r in scan.ranges:
        rel = math.atan2(math.sin(a - fc), math.cos(a - fc))
        a += scan.angle_increment
        if not (scan.range_min < r < max_r) or abs(rel) > fan:
            continue
        x, y = r * math.cos(rel), r * math.sin(rel)
        if x <= 0:
            continue
        if corridor_only and abs(y) >= CORRIDOR_HALF_M:
            continue
        if best is None or x < best[0]:
            best = (x, y)
    return best


def print_corridor(bag, t_from, t_to, step=0.5):
    """장애물 추적 — **통로 밖으로 나간 뒤에도 계속 따라간다**.

    ★ 2026-08-09 에 여기서 크게 틀렸다. 처음엔 `|y| < 0.46` 통로 안만 봤는데,
      **회피가 성공하는 순간 장애물은 정의상 통로를 벗어난다.** 그래서 도구가
      "장애물이 사라졌다 → 회피가 아니라 딴 이유"라고 보고했고, 실제로는 차가
      제대로 비켜 간 것이었다. 성공하면 눈이 머는 필터였다.
      → 통로 판정은 표시만 하고, 추적은 넓은 부채꼴(±35°)로 한다.
    """
    sf = bag.read('/scan_front') or bag.read('/scan')
    if not sf:
        return
    print(f'\n=== 장애물 추적 (전방 ±35°, gap=앞범퍼 기준) — {step}s 간격 ===')
    print('  통로 = |y|<0.46m. 통로를 벗어나면 그 물체는 더 이상 진로에 없다(= 비킴)')
    last = -1e9
    for t, m in sf:
        if not t_from <= t <= t_to or t - last < step:
            continue
        last = t
        best = _nearest_ahead(m)
        if not best:
            print(f't={t:6.2f}s  (6m 내 반사 없음)')
            continue
        gap, y = best
        mark = '통로 안' if abs(y) < CORRIDOR_HALF_M else '통로 밖 ✓비킴'
        print(f't={t:6.2f}s  gap={gap:5.2f}  y={y:+6.3f}  [{mark}]')


def print_execution(bag, t_from, t_to):
    """송신 곡률이 실제 조향으로 몇 % 실행됐나 — 이 세션들의 핵심 지표.

    기하가 요구하는 조향 = atan(κ · wheelbase). dSPACE 가 그 몇 %를 어느 부호로
    내는지가 회피 성패를 가른다. 정차 구간 str 바이어스는 빼고 본다.
    """
    tr, vv = bag.read('/adas/target_ref'), bag.read('/vehicle/vector')
    if not tr or not vv:
        return
    still = [m.str for _, m in vv if abs(m.v) < 0.02]
    bias = statistics.median(still) if still else 0.0
    rows = []
    for t, m in tr:
        if not t_from <= t <= t_to or not m.ref_points:
            continue
        k = m.ref_points[0].curvature
        if abs(k) < 0.05:                       # 직진 명령은 비율 계산에서 제외
            continue
        vm = at(vv, t)
        if vm is None or abs(vm.v) < 0.05:      # 정차 중 조향은 무의미
            continue
        want = math.degrees(math.atan(k * WHEELBASE_M))
        got = math.degrees(vm.str - bias)
        rows.append((want, got))
    print('\n=== 조향 실행률 (곡률 명령 구간) ===')
    if not rows:
        print('  곡률 명령 구간 없음')
        return
    ratios = [g / w for w, g in rows if abs(w) > 1e-6]
    same = sum(1 for w, g in rows if w * g > 0)
    print(f'  샘플 {len(rows)} | 요구 조향 중앙 {statistics.median(w for w, _ in rows):+.2f}° '
          f'| 실행 중앙 {statistics.median(g for _, g in rows):+.2f}°')
    print(f'  실행/요구 중앙 {statistics.median(ratios):+.2f} '
          f'(부호 일치 {same}/{len(rows)})')
    if same * 2 < len(rows):
        print('  ★ str 부호가 명령과 반대다. 다만 **바퀴가 반대로 돈다는 뜻은 아니다** —'
              '\n    아래 ICP 교차검증이 실제 회전 방향의 판정자다. 둘이 어긋나면 '
              '거짓말하는 쪽은\n    텔레메트리이고, 비율만 크기 지표로 쓴다 '
              '(2026-08-09 세션에서 실제로 그랬다).')


def print_gap_side_flips(bag, t_from, t_to):
    """회피 목표점의 좌/우 전환 횟수 — follow-the-gap 은 히스테리시스가 없다.

    좌우 열림이 비슷하면 스캔마다 반대편을 고를 수 있고, 그때마다 조향 명령의
    부호가 뒤집혀 실제 측방 이동이 상쇄된다.
    """
    av = [(t, m.points[0].y) for t, m in bag.read('/perception/avoid')
          if t_from <= t <= t_to and m.points]
    flips = [(t, y) for (_, p), (t, y) in zip(av, av[1:]) if p * y < 0]
    print(f'\n=== 회피 목표점 좌우 전환 {len(flips)}회 / 목표점 {len(av)}개 ===')
    for t, y in flips[:10]:
        print(f'  t={t:6.2f}s → y={y:+.2f}')


def print_trajectory(bag):
    """v·str 적분 궤적 — 측방 이동량이 회피 성패의 최종 지표다."""
    vv = bag.read('/vehicle/vector')
    if len(vv) < 2:
        return
    still = [m.str for t, m in vv if abs(m.v) < 0.02]
    bias = statistics.median(still) if still else 0.0
    pose_dead = all(abs(m.x) < 1e-6 and abs(m.y) < 1e-6 and abs(m.yaw) < 1e-6 for _, m in vv)
    print('\n=== 궤적 (v·str 적분, bicycle) ===')
    print(f'정차 구간 str 바이어스 {math.degrees(bias):+.2f}° (보정 적용) | '
          f'dSPACE pose 필드 {"전부 0 — 적분으로 대체" if pose_dead else "유효"}')
    # ★ str 텔레메트리는 부호가 실제와 반대다 (M-1, ICP·장애물 상대운동·GPS 대조로 확정).
    #   여기서 뒤집어 **+y = 좌** 로 출력한다. 뒤집지 않으면 좌우가 거울이 되어,
    #   "왼쪽으로 비켰다"가 "오른쪽으로 갔다"로 읽힌다 (2026-08-09 오판의 한 축).
    print('  부호 보정 적용: **+y = 좌**, +yaw = 좌 (str 텔레메트리 반전을 되돌림)')
    x = y = yaw = 0.0
    marks, nxt = [], vv[0][0]
    for (t0, m), (t1, _) in zip(vv, vv[1:]):
        dt = t1 - t0
        yaw += m.v * math.tan(-(m.str - bias)) / WHEELBASE_M * dt
        x += m.v * math.cos(yaw) * dt
        y += m.v * math.sin(yaw) * dt
        if t1 >= nxt:
            marks.append(f't={t1:4.1f}s x={x:5.2f} y={y:+6.3f} yaw={math.degrees(yaw):+6.1f}°')
            nxt += 2.0
    for i in range(0, len(marks), 3):
        print('  ' + ' | '.join(marks[i:i + 3]))
    print(f'  최종 측방 이동 {y:+.2f} m, 헤딩 변화 {math.degrees(yaw):+.1f}°')


def print_icp_check(bag, t_from, t_to):
    """스캔 정합(ICP)으로 본 자차 회전 방향 — dSPACE 텔레메트리와 독립인 유일한 증거.

    stack_estop 이 매 스캔 prev→cur SE(2) 변환을 풀어 status JSON 에 넣는다.
    그 변환은 **세상이 센서 안에서 어떻게 움직였나**이므로 자차 운동은 부호 반대다:
        전진  → icp_translation_x < 0
        좌회전 → icp_rotation_rad  < 0
    전진 부호가 맞는 것이 확인되면(차는 분명히 앞으로 갔다) 같은 변환에서 나온
    회전 부호도 신뢰할 수 있다. `/vehicle/vector.str` 이 거짓말을 하는지 여기서 갈린다.

    ★ status 는 스캔보다 느리게(약 절반) 발행되므로 **누적량은 과소평가**다.
      쓸 수 있는 것은 방향과 순간 속도뿐 — 거리·각도 총량으로 쓰지 말 것.
    """
    st = bag.read('/perception/estop/status')
    if not st:
        return
    fwd = dyaw_sum = 0.0
    rot = n = 0
    for t, m in st:
        if not t_from <= t <= t_to:
            continue
        j = json.loads(m.data)
        if not j.get('icp_valid'):
            continue
        tx, dy = j['icp_translation_x'], j['icp_rotation_rad']
        fwd += -tx
        dyaw_sum += -dy
        rot += 1 if -dy > 0 else (-1 if -dy < 0 else 0)
        n += 1
    if not n:
        print('\n=== ICP 교차검증: 유효 프레임 없음 ===')
        return
    scan_dt = _scan_period(bag)
    print('\n=== ICP 교차검증 (스캔 정합, dSPACE 무관) ===')
    print(f'  유효 프레임 {n} | 전진 부호 '
          f'{"정상(+) — 차는 앞으로 갔다" if fwd > 0 else "★역(-)"}')
    print(f'  자차 회전 {"좌(+)" if dyaw_sum > 0 else "우(−)"}, '
          f'프레임 다수결 {rot:+d}/{n}')
    if scan_dt is None:
        return
    # 프레임당 값 = 스캔 1주기 동안의 값. status 가 스캔보다 드물게 발행되므로
    # 누적이 아니라 **평균 rate** 로 환산해야 한다.
    v_icp = fwd / n / scan_dt
    yaw_rate = dyaw_sum / n / scan_dt                    # [rad/s]
    v_dspace = statistics.median(
        [m.v for t, m in bag.read('/vehicle/vector') if t_from <= t <= t_to] or [0.0])
    v_gap = _closing_speed(bag, t_from, t_to)
    print(f'  요레이트 {math.degrees(yaw_rate):+.2f} °/s (회전은 ICP 가 잘 푼다)')
    print(f'  전진속도  ICP {v_icp:.2f} | dSPACE {v_dspace:.2f} | '
          f'장애물 접근율 {"%.2f" % v_gap if v_gap else "-"} m/s')
    # ★ ICP 전진속도는 믿지 말 것 — 복도·벽처럼 진행방향으로 형상이 균일하면
    #   그 방향 이동이 관측되지 않는다(개구 문제). 실제로 이 세션에서 ICP 0.2 vs
    #   장애물 접근율 0.32 로 갈렸고, 정적 장애물까지의 거리 감소율이 맞다.
    #   회전은 형상 균일성과 무관하게 잘 구속되므로 요레이트만 ICP 를 쓴다.
    v = v_gap or v_dspace
    if abs(v) > 0.05:
        delta = math.degrees(math.atan(yaw_rate * WHEELBASE_M / v))
        print(f'  ★ 역산 실제 조향각 {delta:+.2f}° (요레이트 + 접근율 기준) — '
              f'텔레메트리 str 과 무관한 실측치')


def _closing_speed(bag, t_from, t_to):
    """정적 장애물까지의 거리 감소율 = 자차 전진속도. 회피 목표점 x 로 잰다.

    /perception/avoid 목표점 x = lidar_x + gap 이라 그 감소율이 곧 접근 속도다.
    ICP 개구 문제·dSPACE 엔코더 오차 어느 쪽도 타지 않는 독립 측정이다.
    """
    pts = [(t, m.points[0].x) for t, m in bag.read('/perception/avoid')
           if t_from <= t <= t_to and m.points]
    if len(pts) < 10:
        return None
    # 목표점이 좌우 열림을 오가면 x 가 튄다 → 양 끝 구간 중앙값으로 기울기.
    head = pts[:len(pts) // 4]
    tail = pts[-len(pts) // 4:]
    dt = statistics.median(t for t, _ in tail) - statistics.median(t for t, _ in head)
    dx = statistics.median(x for _, x in head) - statistics.median(x for _, x in tail)
    return dx / dt if dt > 0.5 else None


def _scan_period(bag):
    for topic in ('/scan_front', '/scan'):
        if topic not in bag.topics:
            continue
        tid, _ = bag.topics[topic]
        ts = [r[0] for r in bag.conn.execute(
            'SELECT timestamp FROM messages WHERE topic_id=? ORDER BY timestamp', (tid,))]
        if len(ts) > 1:
            return statistics.median((b - a) / 1e9 for a, b in zip(ts, ts[1:]))
    return None


def print_verdict(bag):
    """★ 회피 성패 판정 — 창을 어떻게 자르든 결론이 안 바뀌게 **전 구간**에서 본다.

    합격 = 장애물을 지나가는 순간의 측방 이격 ≥ 0.46m (차폭/2 + 여유).

    ★ 이 함수는 2026-08-09 의 오판 재발 방지용이다. 그때는 AVOID 스테이트 구간
      (4.0~8.1s)만 보고 "회피 실패"라고 적었는데, 실제 기동의 정점은 8.0~8.8s 로
      **스테이트가 빠진 뒤**였다. 스테이트 이탈은 실패가 아니라 오히려 성공 신호다 —
      장애물이 진로에서 빠졌으니 AVOID 를 유지할 이유가 없다.
      그래서 판정은 스테이트가 아니라 **기하(이격)**로만 한다.
    """
    sf = bag.read('/scan_front') or bag.read('/scan')
    av = bag.read('/perception/avoid')
    if not sf or not av:
        return
    detected = [t for t, m in av if m.obstacle_detected]
    print('\n' + '=' * 60)
    print('=== ★ 회피 성패 판정 (전 구간 기하 기준) ===')
    if not detected:
        print('  장애물 감지 구간 없음 — 회피 시험이 성립하지 않았다')
        return
    t_on, t_off = detected[0], detected[-1]
    print(f'  장애물 감지 {t_on:.2f}~{t_off:.2f}s ({t_off - t_on:.1f}s)')

    # 주행 중 정지(narrow_gap)가 있었나 — 완주 여부가 이격보다 먼저다
    tr = bag.read('/adas/target_ref')
    moving = [t for t, m in bag.read('/vehicle/vector') if m.v > 0.05]
    stalls, run = [], None
    for t, m in tr:
        if moving and t > moving[0] and m.v_ref < 0.01:
            run = run if run is not None else t
        elif run is not None:
            if t - run > 0.3:
                stalls.append((run, t))
            run = None
    # ★ 마지막까지 이어진 정지는 elif 를 못 만나 누락된다 — 반드시 flush.
    #   (213832 세션에서 21s 부터 끝까지 선 것을 "완주"로 찍었다.)
    if run is not None and tr[-1][0] - run > 0.3:
        stalls.append((run, tr[-1][0]))
    if stalls:
        print(f'  ★ 주행 중 정지 {len(stalls)}회: '
              + ', '.join(f'{a:.1f}~{b:.1f}s' for a, b in stalls[:4]) + '  → 완주 실패')
        # 정지 원인은 둘 중 하나다. 목표점이 있는데 섰다면 estop, 없으면 narrow_gap.
        for a, b in stalls[:4]:
            mid = (a + b) / 2
            am = at(bag.read('/perception/avoid'), mid)
            em = at(bag.read('/perception/estop'), mid)
            why = ('estop' if (em is not None and em.estop)
                   else 'narrow_gap(목표점 없음)' if (am is not None and not am.points)
                   else '불명')
            extra = ''
            if why == 'estop':
                sm = at(bag.read('/perception/estop/status'), mid)
                if sm is not None:
                    j = json.loads(sm.data)
                    extra = (f" [{j.get('reason')}] static={j.get('static_estop')} "
                             f"dyn={j.get('dynamic_estop')} 최근접={j.get('static_nearest_cluster_min_x')}")
            print(f'      {a:.1f}~{b:.1f}s 원인: {why}{extra}')
    else:
        print('  완주 ✓ (주행 중 v_ref=0 구간 없음)')

    # 장애물별 통과 — gap 의 국소 최솟값이 곧 "스쳐 지나가는 순간"이다.
    # ★ 전 구간 최솟값 하나만 보면 장애물이 여러 개일 때 나머지가 안 보인다.
    # ★ 차가 **움직이는 동안**만 본다. 정지 중에는 gap 이 일정하게 유지돼 국소
    #   최솟값이 계속 잡히고, 서 있는 장애물을 "통과했다"고 세어 버린다
    #   (212156 세션에서 정지 15초가 통과 9개로 집계됐다).
    vmov = [(t, m.v) for t, m in bag.read('/vehicle/vector')]
    track = [(t, ) + _nearest_ahead(m) for t, m in sf
             if t >= t_on and _nearest_ahead(m)
             and (at(vmov, t) or 0) > 0.05]
    if len(track) < 5:
        print('  추적 실패')
        return
    passes = []
    for i in range(2, len(track) - 2):
        t, g, y = track[i]
        win = [track[j][1] for j in range(i - 2, i + 3)]
        if g < 1.2 and g == min(win) and (not passes or t - passes[-1][0] > 1.5):
            passes.append((t, g, y))
    if not passes:
        t_min, gap_min, y_min = min(track, key=lambda r: r[1])
        passes = [(t_min, gap_min, y_min)]
        print('  (통과 지점 미검출 — 전 구간 최근접으로 대체)')
    print(f'  통과한 장애물 {len(passes)}개:')
    for n, (t, g, y) in enumerate(passes, 1):
        ok = abs(y) >= CORRIDOR_HALF_M
        mark = (f'★합격 (여유 +{abs(y) - CORRIDOR_HALF_M:.2f}m)' if ok
                else f'△{CORRIDOR_HALF_M - abs(y):.2f}m 부족')
        print(f'    {n}번  t={t:5.2f}s  gap={g:.2f}m  측방이격 {abs(y):.2f}m  {mark}'
              f'   차 옆면까지 {abs(y) - VEHICLE_HALF_W:.2f}m')
    # estop 이 기동을 방해했는지. ★기동 전 기동(SCAN_TIMEOUT 페일세이프)은 제외해야
    #   한다 — 감지 시작이 estop 해제보다 조금 빠르면 그 꼬리가 "기동 중 발동"으로
    #   잘못 집계된다(실제로 그랬다: 4샘플 = 0.2초, 전부 출발 전).
    est = [(t, m.estop) for t, m in bag.read('/perception/estop') if t_on <= t <= t_off]
    fired = [t for t, e in est if e]
    if not est:
        return
    if not fired:
        print('  기동 중 estop: 미발동 ✓')
        return
    # 차가 실제로 움직이기 시작한 시각 이후만 "기동 중"으로 본다
    moving = [t for t, m in bag.read('/vehicle/vector') if m.v > 0.05]
    t_move = moving[0] if moving else t_on
    during = [t for t in fired if t >= t_move]
    if during:
        print(f'  ★ 기동 중 estop 발동 {len(during)}회 (첫 발동 t={during[0]:.2f}s)')
    else:
        print(f'  기동 중 estop: 미발동 ✓ '
              f'(출발 전 페일세이프 {len(fired)}샘플은 제외, ~{max(fired):.2f}s)')


def main():
    ap = argparse.ArgumentParser(description='field_session bag → 회피 판정 체인 분석')
    ap.add_argument('bag', help='bag 디렉터리 또는 .db3')
    ap.add_argument('--from', dest='t_from', type=float, default=0.0)
    ap.add_argument('--to', dest='t_to', type=float, default=1e9)
    ap.add_argument('--rates', action='store_true', help='토픽 주기만 출력')
    a = ap.parse_args()

    db = resolve_db(a.bag)
    print(f'# {db}\n')
    bag = Bag(db)
    print_rates(bag)
    if a.rates:
        return 0
    print()
    print_chain(bag, a.t_from, a.t_to)
    print_estop(bag, a.t_from, a.t_to)
    print_corridor(bag, a.t_from, a.t_to)
    print_gap_side_flips(bag, a.t_from, a.t_to)
    print_execution(bag, a.t_from, a.t_to)
    print_icp_check(bag, a.t_from, a.t_to)
    print_trajectory(bag)
    print_verdict(bag)              # ★ 항상 전 구간 — --from/--to 의 영향을 받지 않는다
    # 사이드카(_params.yaml/_can.log)는 bag **디렉터리와 나란히** 놓인다:
    #   field_logs/avoid_<stamp>/          ← bag 디렉터리
    #   field_logs/avoid_<stamp>_params.yaml
    # 그래서 기준은 bag 디렉터리의 **부모** + 디렉터리 이름이다.
    bag_dir = os.path.dirname(os.path.abspath(db))
    stem = os.path.join(os.path.dirname(bag_dir), os.path.basename(bag_dir))
    print()
    for side in ('_params.yaml', '_can.log', '_note.txt'):
        cands = [stem + side,
                 (db[:-len('_0.db3')] if db.endswith('_0.db3') else db) + side]
        found = next((p for p in cands if os.path.exists(p)), None)
        if found:
            extra = ''
            if side == '_note.txt':
                with open(found) as fh:
                    extra = '  → ' + fh.read().strip()
            elif side == '_can.log':
                extra = f'  ({os.path.getsize(found) / 1e6:.1f} MB)'
            print(f'{side}: {found}{extra}')
        elif side != '_note.txt':
            print(f'{side}: 없음 — 이 세션은 설정/버스 기록이 빠졌다')
    return 0


if __name__ == '__main__':
    sys.exit(main())
