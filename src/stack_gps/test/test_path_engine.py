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


def test_wrap_angle():
    assert abs(wrap_angle(3 * math.pi) + math.pi) < 1e-9
    assert abs(wrap_angle(-0.1) + 0.1) < 1e-9


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"OK  {fn.__name__}")
    print(f"\n{len(fns)}개 테스트 전부 통과")
