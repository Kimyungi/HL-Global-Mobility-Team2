"""path_engine 단독 검증 — ROS·하드웨어 무의존.

실행: python3 test/test_path_engine.py  (또는 pytest)
합성 트랙(직선·원)으로 접선/곡률/vehicle frame 변환/구간 플래그를 확인한다.
"""
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from stack_gps.path_engine import (M_PER_DEG_LAT, PathEngine,
                                   load_waypoints_csv, wrap_angle)

LAT0, LON0 = 37.5, 127.0
M_PER_DEG_LON = M_PER_DEG_LAT * math.cos(math.radians(LAT0))


def en_to_latlon(e, n):
    return LAT0 + n / M_PER_DEG_LAT, LON0 + e / M_PER_DEG_LON


def make_track(en_pts):
    return [en_to_latlon(e, n) for e, n in en_pts]


def test_straight_east():
    """정동 직선: yaw=0, curvature=0, 차량이 트랙 위에 있으면 x 전진 y≈0."""
    track = make_track([(0.2 * i, 0.0) for i in range(100)])
    eng = PathEngine(track, n_points=10)
    snap = eng.snapshot(*en_to_latlon(4.0, 0.0))  # 20번째 점 위
    assert snap["idx"] == 20, snap["idx"]
    assert snap["cross_track_m"] < 1e-6
    xs = [p[0] for p in snap["points"]]
    for i, (x, y, yaw, k) in enumerate(snap["points"]):
        assert abs(x - 0.2 * i) < 1e-6, (i, x)
        assert abs(y) < 1e-6 and abs(yaw) < 1e-6 and abs(k) < 1e-6
    assert xs == sorted(xs)


def test_straight_north_offset():
    """정북 직선 + 차량이 경로 오른쪽 0.5m: y=+0.5 (경로가 차량 좌측)."""
    track = make_track([(0.0, 0.2 * i) for i in range(100)])
    eng = PathEngine(track, n_points=5)
    # 정북 주행 시 차량 오른쪽 = 동쪽 → 차량을 e=+0.5에 놓으면 경로는 좌측(y+)
    snap = eng.snapshot(*en_to_latlon(0.5, 2.0))
    assert abs(snap["cross_track_m"] - 0.5) < 1e-3
    for x, y, yaw, k in snap["points"]:
        assert abs(y - 0.5) < 1e-3, y
        assert abs(yaw) < 1e-6


def test_circle_ccw():
    """반지름 10m 반시계 원: curvature ≈ +0.1 (좌회전 양수)."""
    r = 10.0
    track = make_track([(r * math.cos(t), r * math.sin(t))
                        for t in [i * 0.02 for i in range(315)]])  # 점간격 0.2m
    eng = PathEngine(track, n_points=20)
    ks = eng.curvature[10:-10]
    for k in ks:
        assert abs(k - 1.0 / r) < 0.01, k
    # 원 위 한 점에서: 앞점들이 왼쪽으로 휘어야 함 (y가 점점 +)
    snap = eng.snapshot(*en_to_latlon(r, 0.0))
    assert snap["points"][-1][1] > 0.05
    # yaw도 진행에 따라 + 방향으로 증가
    yaws = [p[2] for p in snap["points"]]
    assert yaws[-1] > yaws[0]


def test_circle_cw_negative_curvature():
    """시계 방향 원: curvature ≈ -0.1."""
    r = 10.0
    track = make_track([(r * math.cos(-t), r * math.sin(-t))
                        for t in [i * 0.02 for i in range(315)]])
    eng = PathEngine(track, n_points=5)
    for k in eng.curvature[10:-10]:
        assert abs(k + 1.0 / r) < 0.01, k


def test_zones_and_end():
    track = make_track([(0.2 * i, 0.0) for i in range(50)])
    eng = PathEngine(track, n_points=10,
                     accel_ranges=[(10, 20)], parking_ranges=[(40, 49)])
    s = eng.snapshot(*en_to_latlon(3.0, 0.0))   # idx 15
    assert s["accel_zone"] and not s["parking_zone"]
    s = eng.snapshot(*en_to_latlon(9.0, 0.0))   # idx 45 — 끝 근처, 창 클램프
    assert s["parking_zone"] and not s["accel_zone"]
    assert len(s["points"]) == 5
    assert not s["at_end"]
    s = eng.snapshot(*en_to_latlon(9.8, 0.0))   # 마지막 점
    assert s["at_end"]


def test_csv_roundtrip():
    """record_waypoints 포맷 CSV 로드 (중복점 제거 포함)."""
    rows = ["idx,utc,lat,lon,height_m,east_m,north_m,quality"]
    track = make_track([(0.2 * i, 0.1 * i) for i in range(30)])
    for i, (lat, lon) in enumerate(track):
        rows.append(f"{i},000000.00,{lat:.7f},{lon:.7f},50.000,0,0,4")
    rows.append(rows[-1].replace("29,", "30,", 1))  # 같은 좌표 중복
    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as f:
        f.write("\n".join(rows) + "\n")
        path = f.name
    try:
        pts = load_waypoints_csv(path)
        assert len(pts) == 30, len(pts)
        eng = PathEngine(pts, n_points=3)
        s = eng.snapshot(*pts[0])
        assert len(s["points"]) == 3 and s["idx"] == 0
    finally:
        os.unlink(path)


def test_csv_drops_non_fixed_rows():
    """FLOAT(q5) 오염점은 로드 시 제외 — 2026-08-01 course_1 idx 0·84 사례."""
    rows = ["idx,utc,lat,lon,height_m,east_m,north_m,quality"]
    track = make_track([(0.3 * i, 0.0) for i in range(10)])
    for i, (lat, lon) in enumerate(track):
        q = 5 if i in (0, 4) else 4          # 시작점·중간점 오염
        rows.append(f"{i},000000.00,{lat:.7f},{lon:.7f},50.000,0,0,{q}")
    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as f:
        f.write("\n".join(rows) + "\n")
        path = f.name
    try:
        logs = []
        pts = load_waypoints_csv(path, log=logs.append)
        assert len(pts) == 8
        # 오염 시작점이 원점이 되지 않는다 (CSV는 소수 7자리 반올림 — 근사 비교)
        assert abs(pts[0][0] - track[1][0]) < 1e-6
        assert abs(pts[0][1] - track[1][1]) < 1e-6
        assert all(abs(la - track[4][0]) > 1e-9 or abs(lo - track[4][1]) > 1e-9
                   for la, lo in pts)
        assert logs and "2개" in logs[0]
    finally:
        os.unlink(path)


def test_csv_without_quality_column_loads_all():
    """quality 열이 없는 구형/수제 CSV는 전부 통과."""
    rows = ["lat,lon"]
    for lat, lon in make_track([(0.3 * i, 0.0) for i in range(5)]):
        rows.append(f"{lat:.7f},{lon:.7f}")
    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as f:
        f.write("\n".join(rows) + "\n")
        path = f.name
    try:
        assert len(load_waypoints_csv(path)) == 5
    finally:
        os.unlink(path)


def test_lookahead_shifts_first_ref_point():
    """dSPACE는 첫 점만 목표로 씀 — lookahead면 첫 점이 최근접점(옆구리)이
    아니라 전방 트랙 점이어야 한다 (2026-08-05 위빙 원인 수정)."""
    track = make_track([(0.3 * i, 0.0) for i in range(30)])
    base = PathEngine(track, n_points=5)
    la = PathEngine(track, n_points=5, lookahead_m=0.9)   # ≈3점
    # 차가 트랙 중간점 옆 0.3m에 있음
    from_lat, from_lon = en_to_latlon(3.0, 0.3)
    s0 = base.snapshot(from_lat, from_lon)
    s1 = la.snapshot(from_lat, from_lon)
    assert s1["idx"] == s0["idx"]                        # 최근접·at_end 판정은 동일
    assert abs(s1["cross_track_m"] - s0["cross_track_m"]) < 1e-9
    # 첫 점: 기본은 옆구리(x≈0), lookahead는 전방
    assert abs(s0["points"][0][0]) < 0.2
    x1, y1 = s1["points"][0][0], s1["points"][0][1]
    assert x1 >= 0.8, f"lookahead 점이 전방이 아님: x={x1:.2f}"
    assert math.hypot(x1, y1) >= 0.9 - 1e-6, "lookahead 거리 미달"
    assert abs(y1 + 0.3) < 0.05                      # 횡오차는 그대로 담김
    # 2026-08-14: 정확히 0.9m 지점을 요구하던 단언을 완화했다. 이제 도달 곡률
    # κ=2y/L² 이 1/R_min(1.5m)을 넘지 않는 점을 고르는데, 이 기하에서 0.9m 점은
    # 요구 반경이 정확히 1.5m라 한 점 더 나간다(요구 반경 2.55m). 의도된 변화 —
    # 가까운 점을 주면 조향이 포화돼 재합류가 진동한다.
    assert (x1 * x1 + y1 * y1) >= 2.0 * abs(y1) * PathEngine.MIN_TURN_RADIUS_M


def test_lookahead_target_stays_ahead_when_far_off_track():
    """회피 직후처럼 크게 이탈했을 때도 ref 첫 점은 **차량 앞**이어야 한다.

    구현이 "트랙 호길이로 1m 앞"이던 시절엔 이 기하에서 첫 점이 차량 뒤로 나와
    (x<0) 전진밖에 못 하는 차가 재합류할 수 없었다 — 2026-08-14 run_0814_195116
    실측: cross 2.7m·헤딩 30° 이탈에서 ref[0]=(-3.15,-0.10), cross 4.5m까지 발산.
    """
    # 동쪽으로 뻗은 직선 트랙
    track = make_track([(0.3 * i, 0.0) for i in range(60)])
    eng = PathEngine(track, n_points=20, lookahead_m=1.0)
    # 차는 트랙 중간쯤에서 좌로 2.7m 이탈, 헤딩은 트랙(+동) 대비 30° 틀어짐
    lat, lon = en_to_latlon(6.0, 2.7)
    snap = eng.snapshot(lat, lon, heading=math.radians(30.0))
    x0, y0 = snap["points"][0][0], snap["points"][0][1]
    assert x0 >= PathEngine.MIN_FORWARD_M, f"첫 점이 전방이 아님: x={x0:.2f}"
    assert math.hypot(x0, y0) >= 1.0 - 1e-6, "lookahead 거리 미달"
    assert abs(snap["cross_track_m"] - 2.7) < 0.05


def test_rejoin_target_when_heading_away_from_track():
    """트랙에서 멀어지는 방향을 보고 있을 때 — 트랙 점이 아니라 재합류 목표를 낸다.

    dSPACE는 ref 첫 점만 목표로 쓴다(2026-08-14 손상민 확인). 차가 트랙에서
    멀어지는 쪽을 보면 트랙은 전진하는 만큼 옆으로도 벌어져 **트랙 위 어떤
    점도** 조준 가능한 방위에 없다 — run_0814_201650 실측: 20점 전부 방위
    76~84°, 명령 2.3m인데 실제 24m 반경으로 사실상 직진, 횡오차 5m 발산.
    """
    track = make_track([(0.3 * i, 0.0) for i in range(200)])
    eng = PathEngine(track, n_points=20, lookahead_m=1.0)
    # 실측 기하: 트랙 좌측 4.4m, 트랙(+동) 대비 18° 틀어져 멀어지는 중
    snap = eng.snapshot(*en_to_latlon(20.0, 4.4), heading=math.radians(18.0))
    x, y = snap["points"][0][0], snap["points"][0][1]
    bearing = abs(math.atan2(y, x))
    assert bearing <= PathEngine.MAX_TARGET_BEARING_RAD + 1e-6, \
        f"방위 {math.degrees(bearing):.1f}° — 조준 불가"
    assert x > 0, "목표가 전방이 아님"
    # 트랙 쪽(y<0)으로 조준해야 재합류한다
    assert y < 0, "트랙 반대쪽으로 조준"
    # 도달 곡률이 최소 회전반경 안
    L2 = x * x + y * y
    assert L2 >= 2.0 * abs(y) * PathEngine.MIN_TURN_RADIUS_M


def test_rejoin_target_distance_does_not_grow_with_deviation():
    """이탈이 커져도 재합류 목표는 조향 응답 밴드 안에 머문다 (2026-08-15).

    dSPACE 조향 응답은 방위뿐 아니라 **목표 거리**에 강하게 의존한다 —
    0.8~1.6m에서 명령 곡률의 ~55%가 실현되는데 2.5m를 넘으면 10% 아래로
    붕괴한다. 거리 상한이 없던 구현은 이탈이 클수록 트랙 점이 멀어져
    목표도 멀어지는 양의 되먹임이었다: run_0815_144142에서 cross 1.88m→5.04m
    동안 명령 선회반경이 3.4m→6.3m로 되레 완만해지며 복귀에 실패했다.
    (같은 방위·같은 속도인데 d=1.28~1.62m였던 run_0815_142817은 복귀 성공)
    """
    track = make_track([(0.3 * i, 0.0) for i in range(200)])
    eng = PathEngine(track, n_points=20, lookahead_m=1.0)
    prev_r = None
    for cross in (1.0, 1.88, 2.7, 4.0, 5.04):
        snap = eng.snapshot(*en_to_latlon(20.0, cross), heading=math.radians(25.0))
        x, y = snap["points"][0][0], snap["points"][0][1]
        d = math.hypot(x, y)
        assert d <= PathEngine.REJOIN_TARGET_MAX_M + 1e-6, \
            f"cross {cross}m → 목표 {d:.2f}m — 응답 밴드 밖"
        # 도달 곡률 R = d/(2·sinβ) 이 R_min 이상 (풀조향 포화 방지)
        r = d / (2.0 * abs(math.sin(math.atan2(y, x))))
        assert r >= PathEngine.MIN_TURN_RADIUS_M - 1e-6
        # 이탈이 커진다고 선회가 완만해지면 안 된다 (양의 되먹임 차단)
        if prev_r is not None:
            assert r <= prev_r + 1e-6, f"cross {cross}m에서 R이 {prev_r:.2f}→{r:.2f}로 완만해짐"
        prev_r = r


def test_lookahead_target_when_track_is_behind():
    """차가 트랙을 완전히 등지면 전방 후보가 없다 — 폴백해도 예외 없이 동작.

    이 상황의 답은 재합류가 아니라 정지이고, MGM 역방향 가드가 담당한다.
    여기서는 엔진이 죽지 않고 유효한 점을 내는 것만 보장한다.
    """
    track = make_track([(0.3 * i, 0.0) for i in range(30)])
    eng = PathEngine(track, n_points=5, lookahead_m=1.0)
    lat, lon = en_to_latlon(8.0, 0.0)
    snap = eng.snapshot(lat, lon, heading=math.pi)   # 트랙 진행방향의 정반대
    assert len(snap["points"]) >= 1
    assert all(math.isfinite(v) for v in snap["points"][0])


def test_wrap_angle():
    assert abs(wrap_angle(3 * math.pi) + math.pi) < 1e-9
    assert abs(wrap_angle(-0.1) + 0.1) < 1e-9


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"OK  {fn.__name__}")
    print(f"\n{len(fns)}개 테스트 전부 통과")


def test_rejoin_approach_angle_stops_overturning():
    """이미 접근각을 넘겨 돌았으면 재합류 방위가 **반대로** 나와야 한다 (2026-08-16).

    종전엔 트랙 점 방위를 25°로 클램프만 해서, 차가 충분히 돌아섰는데도 횡오차가
    남아 있으면 계속 "더 꺾어라"를 냈다 — 적분기에 브레이크가 없는 꼴이라 트랙을
    직각으로 가로질렀다. run_0816_194948: 복귀는 성공(cross 2.87→0.15m)했으나
    t=59~68 10초 내내 방위 −25.5° 포화로 헤딩이 67° 과회전, 도달 시점 헤딩오차가
    +74.5° 라 그대로 통과해 반대편 1.7m 까지 이탈했다.
    """
    track = make_track([(0.3 * i, 0.0) for i in range(200)])
    eng = PathEngine(track, n_points=20, lookahead_m=1.0)
    # 트랙(+동)에서 좌측 2.5m, 트랙 쪽(우측)으로 이미 40° 꺾어 접근 중 —
    # 접근각 상한(25°)을 넘겼으므로 되돌려야 한다.
    snap = eng.snapshot(*en_to_latlon(20.0, 2.5), heading=math.radians(-40.0))
    x, y = snap["points"][0][0], snap["points"][0][1]
    bearing = math.degrees(math.atan2(y, x))
    assert bearing > 2.0, f"과회전인데 방위 {bearing:.1f}° — 되돌리지 않음"
    assert bearing <= math.degrees(PathEngine.MAX_TARGET_BEARING_RAD) + 1e-6

    # 아직 덜 돌았으면 계속 트랙 쪽으로 꺾어야 한다 (부호 반대)
    snap2 = eng.snapshot(*en_to_latlon(20.0, 2.5), heading=math.radians(0.0))
    x2, y2 = snap2["points"][0][0], snap2["points"][0][1]
    assert math.degrees(math.atan2(y2, x2)) < -2.0, "덜 돌았는데 트랙 쪽으로 안 꺾음"


def test_rejoin_approach_angle_scales_down_near_track():
    """트랙에 가까워지면 접근각이 줄어 **접선으로** 붙어야 한다 (직각 통과 방지)."""
    track = make_track([(0.3 * i, 0.0) for i in range(200)])
    eng = PathEngine(track, n_points=20, lookahead_m=1.0)
    prev = None
    for cross in (2.0, 1.0, 0.5, 0.2):
        snap = eng.snapshot(*en_to_latlon(20.0, cross), heading=math.radians(0.0))
        x, y = snap["points"][0][0], snap["points"][0][1]
        b = abs(math.degrees(math.atan2(y, x)))
        if prev is not None:
            assert b <= prev + 1e-6, f"cross {cross}m 에서 접근각이 되레 커짐"
        prev = b
