#!/usr/bin/env python3
"""실차 세션 bag → stage2 실측값.  이기돈

field_session.launch.py 로 딴 bag 하나를 넣으면 ①②③ⓐⓑⓒ 중 그 bag에 들어 있는
항목을 뽑아 출력한다. MEASUREMENTS.md 에 그대로 옮겨 적을 수 있는 형태.

  python3 src/stack_avoid/tools/analyze_field_bag.py ~/avoid_logs/field_step_.../bag

① 조향 응답   : ref y 스텝 → VehicleVector.str 의 dead time / 63% / 95% 도달
② 측방 이동   : 스텝 이후 |측방변위| 0.30m·0.46m 도달까지의 전진거리
③ 감지 신뢰   : /test/event 의 cone 구간별 감지율·gap 평균/표준편차
ⓐⓑⓒ          : 구간별 estop 발동 여부 (정적/동적 분리)

★ str이 통째로 고정이면 액추에이션 사망(조이스틱 전원 off) — 그 구간은 무효다.
  8/6 로그에서 실제로 있었던 일이라 자동으로 경고한다.
"""
import glob
import math
import os
import sqlite3
import statistics
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# 스텝 판정은 ref_points[0].curvature(κ)로 한다.
#   ref로 나가는 y는 "목표점"이 아니라 그 호 위의 lookahead 점(≈0.4m 앞)이라
#   목표 오프셋 0.46m가 y≈0.03m로만 나타난다. κ = 2y_target/(x²+y²)는
#   같은 입력에서 ≈0.37 1/m라 잡음과 확실히 구분된다.
STEP_MIN_DK = 0.05
# ② 에서 "복귀(정렬)" 구간을 제외할 κ 하한.
RETURN_MAX_DK = 0.10
# 응답 시작으로 볼 str 변화 비율 (dead time 판정).
DEAD_FRAC = 0.05
# 기준선·정착값을 낼 구간 길이 [s].
BASE_S = 0.5
# str이 이 진폭 미만이면 액추에이션 사망 의심 [rad].
STR_DEAD_AMP = 0.02
# 응답을 볼 수 있는 스텝 간격 [s]. 너무 짧으면 정착 전에 다음 스텝이 오고(측정 불가),
# 너무 길면 그 사이 무관한 조작(수동 조향 등)이 섞인다. 8/6 수동 튜닝 로그에서
# 38s짜리 허수 dead time이 나온 원인 — 통계에서 제외한다.
MIN_HOLD_S = 0.5
MAX_HOLD_S = 20.0


def load(dbfile, topic):
    """bag에서 한 토픽을 (시각[s], 메시지) 목록으로 읽는다."""
    con = sqlite3.connect(dbfile)
    rows = con.execute(
        'select t.type, m.timestamp, m.data from messages m '
        'join topics t on t.id = m.topic_id where t.name = ? order by m.timestamp',
        (topic,)).fetchall()
    con.close()
    if not rows:
        return []
    msgt = get_message(rows[0][0])
    return [(ts * 1e-9, deserialize_message(d, msgt)) for _, ts, d in rows]


def at(series, t):
    """시각 t 직전의 마지막 샘플 (없으면 None)."""
    prev = None
    for ts, m in series:
        if ts > t:
            break
        prev = m
    return prev


def window(series, t0, t1):
    """[t0, t1) 구간 샘플."""
    return [(ts, m) for ts, m in series if t0 <= ts < t1]


def find_steps(refs):
    """조향 명령이 계단처럼 바뀐 시점 → [(시각, 이전κ, 이후κ)].

    판정 신호는 curvature — 상단 STEP_MIN_DK 주석 참조.
    """
    steps = []
    prev_k = None
    for ts, m in refs:
        if not m.ref_points:
            continue
        k = m.ref_points[0].curvature
        if prev_k is not None and abs(k - prev_k) > STEP_MIN_DK:
            steps.append((ts, prev_k, k))
        prev_k = k
    return steps


def analyze_steering(refs, vv, out):
    """① 조향 응답 시간 — ref y 스텝에 대한 str의 dead time·상승 시간."""
    steps = find_steps(refs)
    if not steps or not vv:
        return
    strs = [m.str for _, m in vv]
    if max(strs) - min(strs) < STR_DEAD_AMP:
        out.append(f'⚠ str 진폭 {max(strs) - min(strs):.4f} rad — 액추에이션 사망 의심'
                   f'(조이스틱 전원 off?). 이 bag의 ①은 무효.')
        return

    out.append(f'\n① 조향 응답 — 스텝 {len(steps)}개')
    out.append(f'{"κ 스텝[1/m]":>20s} {"Δstr[rad]":>10s} {"dead[s]":>8s} '
               f'{"63%[s]":>8s} {"95%[s]":>8s}')
    deads, t63s, t95s = [], [], []
    skipped = 0
    for i, (ts, y0, y1) in enumerate(steps):
        nxt = steps[i + 1][0] if i + 1 < len(steps) else vv[-1][0]
        hold = nxt - ts
        if not (MIN_HOLD_S <= hold <= MAX_HOLD_S):
            skipped += 1                  # 정착 불가 / 무관 조작 혼입 구간
            continue
        base_w = window(vv, ts - BASE_S, ts)
        resp_w = window(vv, ts, nxt)
        if len(base_w) < 3 or len(resp_w) < 5:
            continue
        base = statistics.median([m.str for _, m in base_w])
        settle_w = window(vv, max(ts, nxt - BASE_S), nxt)
        if not settle_w:
            continue
        final = statistics.median([m.str for _, m in settle_w])
        delta = final - base
        if abs(delta) < STR_DEAD_AMP:
            continue                      # 명령은 바뀌었는데 조향이 안 움직임 — 별도 표시
        dead = t63 = t95 = None
        for t, m in resp_w:
            frac = (m.str - base) / delta
            if dead is None and frac >= DEAD_FRAC:
                dead = t - ts
            if t63 is None and frac >= 0.63:
                t63 = t - ts
            if t95 is None and frac >= 0.95:
                t95 = t - ts
                break
        out.append(f'{y0:+.3f}→{y1:+.3f} @{ts - refs[0][0]:7.1f}s {delta:+10.3f} '
                   f'{_f(dead):>8s} {_f(t63):>8s} {_f(t95):>8s}')
        if dead is not None:
            deads.append(dead)
        if t63 is not None:
            t63s.append(t63)
        if t95 is not None:
            t95s.append(t95)

    if skipped:
        out.append(f'  ({skipped}개 제외 — 스텝 간격이 {MIN_HOLD_S}~{MAX_HOLD_S}s 밖. '
                   f'정착 전 다음 스텝이 오거나 무관한 조작이 섞인 구간)')
    if deads:
        out.append(f'\n  중앙값: dead {statistics.median(deads):.3f}s · '
                   f'63% {statistics.median(t63s):.3f}s · '
                   f'95% {statistics.median(t95s):.3f}s   (n={len(deads)})')
        out.append(f'  최악값: dead {max(deads):.3f}s · 95% {max(t95s) if t95s else 0:.3f}s'
                   f'   ← avoidable 공식의 지연여유는 이 최악값 기준')
        if skipped > len(deads):
            out.append('  ⚠ 제외가 유효 스텝보다 많다 — 수동 조작이 섞인 로그일 가능성. '
                       'step_injector로 깨끗한 스텝을 다시 딸 것.')


def analyze_lateral(refs, vv, out):
    """② 측방 이동 곡선 — 스텝 후 |측방변위| 0.30·0.46m 도달까지의 전진거리."""
    steps = find_steps(refs)
    if not steps or not vv:
        return
    moving = [m.v for _, m in vv if abs(m.v) > 0.05]
    if not moving:
        out.append('\n② 측방 이동 — 차가 움직이지 않음(v≈0). 스탠드 bag이면 정상, '
                   '지상 시험이면 무효.')
        return

    out.append(f'\n② 측방 이동 곡선 — 평균 속도 {statistics.mean(moving):.2f} m/s')
    out.append('  ⚠ 지상 주행 bag에서만 유효. dSPACE 추측항법은 바퀴가 떠 있어도 v를 적분하므로'
               ' 스탠드 bag에서도 숫자가 나오지만 의미 없다.')
    out.append(f'{"κ 스텝[1/m]":>20s} {"→0.30m":>9s} {"→0.46m":>9s}   (전진거리 [m])')
    d30s, d46s = [], []
    for i, (ts, y0, y1) in enumerate(steps):
        if abs(y1) < RETURN_MAX_DK:       # κ≈0 = 직진 복귀(정렬) 구간 — 제외
            continue
        nxt = steps[i + 1][0] if i + 1 < len(steps) else vv[-1][0]
        seg = window(vv, ts, nxt)
        if len(seg) < 10:
            continue
        m0 = seg[0][1]
        x0, y0v, yaw0 = m0.x, m0.y, m0.yaw
        c, s = math.cos(yaw0), math.sin(yaw0)
        d30 = d46 = None
        for _, m in seg:
            dx, dy = m.x - x0, m.y - y0v
            fwd = dx * c + dy * s               # 스텝 시작 헤딩 기준 전진
            lat = -dx * s + dy * c              # 좌우 변위
            if d30 is None and abs(lat) >= 0.30:
                d30 = fwd
            if d46 is None and abs(lat) >= 0.46:
                d46 = fwd
                break
        out.append(f'{y0:+.3f}→{y1:+.3f} @{ts - refs[0][0]:7.1f}s '
                   f'{_f(d30):>9s} {_f(d46):>9s}')
        if d30 is not None:
            d30s.append(d30)
        if d46 is not None:
            d46s.append(d46)

    if d30s or d46s:
        out.append('\n  중앙값: '
                   + (f'0.30m→{statistics.median(d30s):.2f}m  ' if d30s else '')
                   + (f'0.46m→{statistics.median(d46s):.2f}m' if d46s else ''))
        out.append('  기하 이상치(조향지연 제외, R=1.15m): 0.30m→0.775m · 0.46m→0.920m')
        if d46s:
            out.append(f'  → 실측이 이상치보다 {statistics.median(d46s) - 0.920:+.2f}m '
                       f'(이 차이가 조향 지연분)')


def analyze_detection(events, avoid, out):
    """③ 감지 신뢰 거리 — cone 구간별 감지율·gap 통계."""
    segs = [(ts, m.data) for ts, m in events if 'cone' in m.data]
    if not segs or not avoid:
        return
    out.append('\n③ 감지 신뢰 거리')
    out.append(f'{"구간":>34s} {"감지율":>7s} {"gap평균":>8s} {"gap표준편차":>10s} {"샘플":>5s}')
    for i, (ts, label) in enumerate(segs):
        end = segs[i + 1][0] if i + 1 < len(segs) else avoid[-1][0]
        seg = window(avoid, ts, end)
        if not seg:
            continue
        det = [m for _, m in seg if m.obstacle_detected]
        rate = len(det) / len(seg)
        # ttc = gap / v 이므로 gap 을 직접 못 읽는다 — points[0].x 에서 라이다 오프셋을 뺀다.
        gaps = [m.points[0].x - 0.76 for m in det if m.points]
        gm = f'{statistics.mean(gaps):.3f}' if gaps else '-'
        gs = f'{statistics.pstdev(gaps):.3f}' if len(gaps) > 1 else '-'
        flag = '  ← 불안정' if rate < 0.95 else ''
        out.append(f'{label:>34s} {rate * 100:6.1f}% {gm:>8s} {gs:>10s} {len(seg):5d}{flag}')


def analyze_boundary(events, estop, static_e, dynamic_e, avoid, out):
    """ⓐⓑⓒ — 구간별 estop 발동 여부와 회피 목표점 유무."""
    segs = [(ts, m.data) for ts, m in events if m.data.startswith(('ⓐ', 'ⓑ', 'ⓒ'))]
    if not segs:
        return
    out.append('\nⓐⓑⓒ 경계 시험')
    for i, (ts, label) in enumerate(segs):
        end = segs[i + 1][0] if i + 1 < len(segs) else (estop[-1][0] if estop else ts + 60)
        fired = any(m.estop for _, m in window(estop, ts, end))
        st = any(m.data for _, m in window(static_e, ts, end)) if static_e else None
        dy = any(m.data for _, m in window(dynamic_e, ts, end)) if dynamic_e else None
        seg_a = window(avoid, ts, end)
        had_target = any(m.points for _, m in seg_a)
        narrow = any(m.narrow_gap for _, m in seg_a)
        src = []
        if st:
            src.append('정적')
        if dy:
            src.append('동적')
        out.append(f'  {label}')
        out.append(f'    estop {"발동" if fired else "미발동"}'
                   + (f' ({"+".join(src)})' if src else '')
                   + f' · 회피목표점 {"있었음" if had_target else "없었음"}'
                   + (' · narrow_gap' if narrow else ''))


def _f(v):
    return '-' if v is None else f'{v:.3f}'


def main():
    """bag 경로를 받아 항목별 분석 결과를 출력한다."""
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    bagdir = os.path.expanduser(sys.argv[1])
    files = glob.glob(os.path.join(bagdir, '**', '*.db3'), recursive=True)
    if not files:
        print(f'db3 없음: {bagdir}')
        return 1
    f = files[0]

    refs = load(f, '/adas/target_ref')
    vv = load(f, '/vehicle/vector')
    avoid = load(f, '/perception/avoid')
    events = load(f, '/test/event')
    estop = load(f, '/perception/estop')
    static_e = load(f, '/perception/static_estop')
    dynamic_e = load(f, '/perception/dynamic_estop')

    out = [f'=== {os.path.basename(os.path.dirname(f)) or bagdir} ===']
    dur = (vv[-1][0] - vv[0][0]) if vv else 0
    out.append(f'길이 {dur:.0f}s · ref {len(refs)} · vv {len(vv)} · avoid {len(avoid)} '
               f'· event {len(events)} · estop {len(estop)}')
    if not events:
        out.append('⚠ /test/event 없음 — 구간 라벨이 없어 ③ⓐⓑⓒ는 분석 불가. '
                   '다음엔 `ros2 run stack_avoid mark`를 같이 띄울 것.')

    analyze_steering(refs, vv, out)
    analyze_lateral(refs, vv, out)
    analyze_detection(events, avoid, out)
    analyze_boundary(events, estop, static_e, dynamic_e, avoid, out)

    print('\n'.join(out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
