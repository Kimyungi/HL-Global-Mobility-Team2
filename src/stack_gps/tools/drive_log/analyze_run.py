#!/usr/bin/env python3
"""주행 로그 기하 분석 — 헤딩 가정 오차·발행 경로 방향·차의 경로 좌우 위치 판정.

8/1 첫 주행의 좌회전 발산을 진단한 도구를 범용화한 것. bag의 실제 궤적과
발행된 ref를 대조해 "우리 변환이 옳았는지 vs 하위(조향 부호 등) 문제인지"를
계층 분리한다.

사용 (워크스페이스 source 필요 — rosbag 역직렬화 때문):
  source ~/FMA_ws/install/setup.bash
  python3 analyze_run.py ~/FMA_ws/drive_logs/run1_20260801_182407
"""
import csv
import math
import os
import sqlite3
import sys

from rclpy.serialization import deserialize_message

from fma_interfaces.msg import GpsPath
from sensor_msgs.msg import NavSatFix

M = 111_320.0


def main():
    if len(sys.argv) < 2:
        runs = sorted(
            (d for d in os.listdir(os.path.expanduser("~/FMA_ws/drive_logs"))
             if os.path.isdir(os.path.expanduser(f"~/FMA_ws/drive_logs/{d}"))),
            reverse=True)
        run = os.path.expanduser(f"~/FMA_ws/drive_logs/{runs[0]}")
        print(f"[analyze] run 미지정 — 최신 사용: {run}")
    else:
        run = sys.argv[1].rstrip("/")

    bag_dir = os.path.join(run, "bag")
    db = [os.path.join(bag_dir, n) for n in os.listdir(bag_dir) if n.endswith(".db3")][0]
    trk_csv = [os.path.join(run, n) for n in os.listdir(run)
               if n.startswith("waypoints_") and n.endswith(".csv")][0]

    con = sqlite3.connect(db)
    topics = dict(con.execute("SELECT name, id FROM topics").fetchall())

    def msgs(topic, typ):
        for (ts, data) in con.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
                (topics[topic],)):
            yield ts * 1e-9, deserialize_message(bytes(data), typ)

    # 노드(load_waypoints_csv)와 동일하게 비-FIXED 점 제외 — FLOAT 오염 방지
    trk = [r for r in csv.DictReader(open(trk_csv))
           if r.get("quality") is None or int(r["quality"]) == 4]
    lat0, lon0 = float(trk[0]["lat"]), float(trk[0]["lon"])
    te = [(float(r["lon"]) - lon0) * M * math.cos(math.radians(lat0)) for r in trk]
    tn = [(float(r["lat"]) - lat0) * M for r in trk]

    fixes = [(t, (m.longitude - lon0) * M * math.cos(math.radians(lat0)),
              (m.latitude - lat0) * M) for t, m in msgs("/perception/gps_fix", NavSatFix)]
    paths = [(t, m) for t, m in msgs("/perception/gps_path", GpsPath) if m.points]
    print(f"[analyze] gps_fix {len(fixes)}개, gps_path {len(paths)}개, 트랙 {len(te)}점")
    if len(fixes) < 3:
        sys.exit("fix가 너무 적음 — 주행 데이터가 아닌 듯")

    def nearest(e, n):
        d2 = [(e - x) ** 2 + (n - y) ** 2 for x, y in zip(te, tn)]
        i = d2.index(min(d2))
        return i, math.sqrt(d2[i])

    print("\n t(s) | 이동방향(전역) 경로접선 | 헤딩차이 | 발행y0 | 경로의(좌/우) | 횡오차")
    t0 = fixes[0][0]
    step = max(1, len(fixes) // 20)
    for k in range(1, len(fixes), step):
        t, e, n = fixes[k]
        pe, pn = fixes[k - 1][1], fixes[k - 1][2]
        if math.hypot(e - pe, n - pn) < 0.05:
            continue
        h_true = math.atan2(n - pn, e - pe)
        i, dist = nearest(e, n)
        j = min(i + 1, len(te) - 1)
        psi = math.atan2(tn[j] - tn[i], te[j] - te[i])
        err = math.degrees(math.atan2(math.sin(h_true - psi), math.cos(h_true - psi)))
        side = (e - te[i]) * -math.sin(psi) + (n - tn[i]) * math.cos(psi)
        pm = min(paths, key=lambda p: abs(p[0] - t))
        y0 = pm[1].points[0].y
        print(f"{t-t0:5.1f} | {math.degrees(h_true):7.1f}°  {math.degrees(psi):7.1f}° | "
              f"{err:+7.1f}° | {y0:+6.2f} | {'좌' if side > 0 else '우'}({side:+5.1f}m) | {dist:5.2f}m")
    con.close()
    print("\n해석: |헤딩차이| 큰데 지속되면 헤딩 소스 문제 / y0 부호가 항상 경로 쪽을"
          "\n가리키는데도 차가 반대로 가면 하위(조향 부호) 문제.")


if __name__ == "__main__":
    main()
