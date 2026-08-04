#!/usr/bin/env python3
"""헤딩 진단 캡처 — 융합 내부 전 신호를 10Hz 연속 기록 (2분).

2026-08-04 절대 검증 2연속 불합격(회전 후 정지 σ 27~34°)의 원인 분리용:
yaw_gyro / offset / COG / heading 을 동시에 기록해 "정지 중 흔들림"이
어느 신호에서 오는지 확정한다.

사용:  python3 heading_debug_capture.py            # 베이스 있으면
       python3 heading_debug_capture.py --rtcm-host off
안내 문구에 맞춰 조이스틱 조작. 끝나면 CSV 경로가 출력된다.
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from stack_gps.gga_link import GgaLink              # noqa: E402
from stack_gps.heading_fusion import HeadingFusion  # noqa: E402
from stack_gps.imu_link import ImuLink              # noqa: E402
from stack_gps.path_engine import wrap_angle        # noqa: E402

STEPS = [
    (15, "가만히 정지"),
    (15, "제자리 좌회전 ~90° 후 정지"),
    (15, "가만히 정지"),
    (15, "제자리 우회전 ~90° (원위치) 후 정지"),
    (15, "가만히 정지"),
    (20, "전진 4~5m (부드럽게)"),
    (15, "가만히 정지"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial-port", default="/dev/ttyRover")
    ap.add_argument("--imu-port", default="/dev/ttyUSB_IMU")
    ap.add_argument("--rtcm-host", default="127.0.0.1")
    args = ap.parse_args()
    if args.rtcm_host.lower() in ("off", "none"):
        args.rtcm_host = ""

    gga = GgaLink(args.serial_port, rtcm_host=args.rtcm_host,
                  log=lambda m: print(f"  [gps] {m}"))
    imu = ImuLink(args.imu_port, log=lambda m: print(f"  [imu] {m}"))
    gga.start(); imu.start()
    fusion = HeadingFusion(sign=1.0)   # 노드와 동일

    out_path = os.path.expanduser(
        f"~/FMA_ws/drive_logs/heading_debug_{time.strftime('%m%d_%H%M')}.csv")
    f = open(out_path, "w", buffering=1)
    f.write("t,step,quality,cog_spd,cog_deg,cog_age,cog_ok,"
            "yaw_gyro_deg,gyro_z,offset_deg,heading_deg,aligned,rejected,reseeds\n")

    print("준비 3초...")
    time.sleep(3)
    t0 = time.monotonic()
    step_i, step_t = 0, 0.0
    cog_ok = False
    print(f"▶ 1/{len(STEPS)}: {STEPS[0][1]}", flush=True)
    while True:
        now = time.monotonic()
        t = now - t0
        # 단계 안내
        if step_i < len(STEPS) and t - step_t >= STEPS[step_i][0]:
            step_i += 1
            step_t = t
            if step_i >= len(STEPS):
                break
            print(f"▶ {step_i+1}/{len(STEPS)}: {STEPS[step_i][1]}", flush=True)

        # node.tick과 동일한 융합 배선
        yawg = imu.latest_yaw_gyro()
        g = imu.latest_gyro_z()
        if yawg is not None:
            fusion.update_imu(yawg[0], now - yawg[1],
                              gyro_z=g[0] if g else None)
        cog = gga.latest_cog()
        if cog is None or cog[2] > 1.0:
            cog_ok = False
        elif cog_ok:
            cog_ok = cog[0] >= 0.7 * 0.25
        else:
            cog_ok = cog[0] >= 0.25
        if cog_ok:
            fusion.update_cog(cog[1], now - cog[2], speed=cog[0])
        h = fusion.heading(now)

        fix = gga.latest_fix()
        f.write(",".join([
            f"{t:.2f}", str(step_i),
            str(fix[3]) if fix else "",
            f"{cog[0]:.3f}" if cog else "",
            f"{math.degrees(cog[1]):.1f}" if cog else "",
            f"{cog[2]:.2f}" if cog else "",
            str(int(cog_ok)),
            f"{math.degrees(yawg[0]):.2f}" if yawg else "",
            f"{g[0]:.4f}" if g else "",
            f"{math.degrees(fusion.offset):.2f}" if fusion.offset is not None else "",
            f"{math.degrees(h):.2f}" if h is not None else "",
            str(int(fusion.aligned)), str(fusion.rejected), str(fusion.reseeds),
        ]) + "\n")
        time.sleep(0.1)

    gga.stop(); imu.stop()
    print(f"완료 — {out_path}")


if __name__ == "__main__":
    main()
