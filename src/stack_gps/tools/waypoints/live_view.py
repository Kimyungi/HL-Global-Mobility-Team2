#!/usr/bin/env python3
"""라이브 뷰어 — 기록 트랙 위에서 지금 내가 어디에, 얼마나 벗어나 있는지 시각화.

stack_gps_node가 돌고 있는 상태에서 별도 터미널로 실행한다 (시리얼은 노드가
단독 점유 — 이 뷰어는 ROS 토픽만 구독하므로 충돌 없음).

  ros2 run stack_gps stack_gps_node --ros-args -p waypoint_csv:=... -p rtcm_host:=...
  python3 live_view.py                      # 최신 waypoints_*.csv 자동 선택
  python3 live_view.py --csv <경로>         # 트랙 지정

왼쪽: 전역(ENU) — 기록 트랙 전체 + 현재 위치·이동 궤적 + 횡오차
오른쪽: vehicle frame — 노드가 실제 발행 중인 /perception/gps_path 경로창
        (전방이 위, 좌측이 왼쪽. 내가 트랙 오른쪽에 서면 경로가 왼쪽(+y)에 보임)

구독: /perception/gps_fix (전역 위치), /perception/gps_path (경로창·품질)
"""
import argparse
import glob
import math
import os
import sys
import threading

import matplotlib
from matplotlib import font_manager

for _f in ("NanumGothic", "Noto Sans CJK KR", "Noto Sans KR",
           "Noto Sans CJK JP", "UnDotum"):  # CJK JP도 한글 글리프 포함
    if any(ft.name == _f for ft in font_manager.fontManager.ttflist):
        matplotlib.rcParams["font.family"] = _f
        break
matplotlib.rcParams["axes.unicode_minus"] = False

import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", ".."))
from stack_gps.path_engine import PathEngine, load_waypoints_csv  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from fma_interfaces.msg import GpsPath  # noqa: E402
from sensor_msgs.msg import NavSatFix  # noqa: E402

# Okabe-Ito 기반 — CVD 안전, 역할 고정: 트랙=회색, 나=파랑, 노드 경로창=주황
C_TRACK = "#8a8a8a"
C_ME = "#0072B2"
C_TRAIL = "#9ecbe8"
C_PATH = "#E69F00"
C_INK = "#333333"
QNAMES = {0: "NOFIX", 1: "GPS", 2: "DGPS", 4: "RTK FIXED", 5: "RTK FLOAT"}


class Listener(Node):
    def __init__(self):
        super().__init__("live_view")
        self.lock = threading.Lock()
        self.fix = None          # (lat, lon)
        self.path = None         # GpsPath
        self.create_subscription(NavSatFix, "/perception/gps_fix", self._on_fix, 1)
        self.create_subscription(GpsPath, "/perception/gps_path", self._on_path, 1)

    def _on_fix(self, m):
        with self.lock:
            self.fix = (m.latitude, m.longitude)

    def _on_path(self, m):
        with self.lock:
            self.path = m


def newest_csv(waypoints_dir):
    files = sorted(glob.glob(os.path.join(waypoints_dir, "waypoints_*.csv")))
    if not files:
        sys.exit(f"waypoints CSV가 없습니다: {waypoints_dir}")
    return files[-1]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    default_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "..", "waypoints")
    ap.add_argument("--csv", default="", help="트랙 CSV (기본: 최신 파일)")
    ap.add_argument("--save", default="", help="첫 수신 후 PNG 저장하고 종료 (테스트용)")
    args = ap.parse_args()

    csv_path = args.csv or newest_csv(default_dir)
    pts = load_waypoints_csv(csv_path)
    eng = PathEngine(pts)
    print(f"[view] 트랙: {os.path.basename(csv_path)} ({len(pts)}점)")

    rclpy.init()
    node = Listener()
    th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    th.start()

    fig, (axg, axv) = plt.subplots(1, 2, figsize=(12, 6),
                                   gridspec_kw={"width_ratios": [1.3, 1]})
    fig.canvas.manager.set_window_title("stack_gps live view") \
        if hasattr(fig.canvas.manager, "set_window_title") else None

    trail = []

    def draw():
        with node.lock:
            fix, path = node.fix, node.path

        axg.clear()
        axv.clear()

        # ── 왼쪽: 전역 ENU
        axg.plot(eng.e, eng.n, "-", color=C_TRACK, lw=2, label="기록 트랙")
        axg.plot(eng.e[0], eng.n[0], "^", color=C_TRACK, ms=9)
        axg.plot(eng.e[-1], eng.n[-1], "s", color=C_TRACK, ms=8)
        axg.annotate("시작", (eng.e[0], eng.n[0]), color=C_INK, fontsize=9,
                     xytext=(6, 6), textcoords="offset points")
        axg.annotate("끝", (eng.e[-1], eng.n[-1]), color=C_INK, fontsize=9,
                     xytext=(6, 6), textcoords="offset points")

        head = "위치 수신 대기 중… (노드가 FIXED인지 확인)"
        if fix is not None:
            ev, nv = eng.to_enu(*fix)
            trail.append((ev, nv))
            del trail[:-600]
            if len(trail) > 1:
                axg.plot(*zip(*trail), "-", color=C_TRAIL, lw=1.5, label="이동 궤적")
            axg.plot(ev, nv, "o", color=C_ME, ms=10, label="현재 위치")
            snap = eng.snapshot(*fix)
            i = snap["idx"]
            axg.plot([ev, eng.e[i]], [nv, eng.n[i]], "--", color=C_ME, lw=1)
            q = QNAMES.get(path.fix_quality, "?") if path else "?"
            head = (f"{q}   최근접 idx {i}/{len(pts)-1}   "
                    f"횡오차 {snap['cross_track_m']*100:.0f} cm"
                    + ("   [트랙 끝]" if snap["at_end"] else ""))

        axg.set_title("전역 (ENU) — 트랙 대비 내 위치", color=C_INK, fontsize=11)
        axg.set_xlabel("동쪽 [m]")
        axg.set_ylabel("북쪽 [m]")
        axg.set_aspect("equal", adjustable="datalim")
        axg.grid(alpha=0.25)
        axg.legend(loc="upper right", fontsize=9)

        # ── 오른쪽: vehicle frame (전방 ↑, 좌측 ←)
        if path is not None and path.points:
            px = [p.x for p in path.points]
            py = [p.y for p in path.points]
            axv.plot(py, px, "o-", color=C_PATH, ms=4, lw=1.5,
                     label="노드 출력 경로창")
        axv.plot(0, 0, marker=(3, 0, 0), color=C_ME, ms=16, label="차량(나)")
        axv.axhline(0, color=C_TRACK, lw=0.5, alpha=0.5)
        axv.axvline(0, color=C_TRACK, lw=0.5, alpha=0.5)
        axv.set_title("vehicle frame — /perception/gps_path", color=C_INK, fontsize=11)
        axv.set_xlabel("y 좌측 [m]")
        axv.set_ylabel("x 전방 [m]")
        axv.set_aspect("equal", adjustable="datalim")
        axv.grid(alpha=0.25)
        axv.legend(loc="upper right", fontsize=9)
        axv.invert_xaxis()  # 화면 왼쪽 = +y(좌측) — 운전자 시점과 일치

        fig.suptitle(head, color=C_INK, fontsize=13)

    if args.save:
        import time
        t0 = time.time()
        while time.time() - t0 < 15:
            with node.lock:
                ready = node.fix is not None and node.path is not None
            if ready:
                break
            time.sleep(0.2)
        draw()
        fig.savefig(args.save, dpi=110, bbox_inches="tight")
        print(f"[view] 저장: {args.save}")
    else:
        from matplotlib.animation import FuncAnimation
        anim = FuncAnimation(fig, lambda _: draw(), interval=500,
                             cache_frame_data=False)
        plt.show()

    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
