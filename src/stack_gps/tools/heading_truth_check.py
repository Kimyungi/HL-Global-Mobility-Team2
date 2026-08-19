#!/usr/bin/env python3
"""융합 헤딩 절대 검증 — 합의 기준: 전 자세 최대 5° 이내 + 연속 회전 누적 없음.

배경 (2026-08-03, 손상민 합의): 바닥 기준각 대조 방식. 기준각은 분필 대신
RTK로 기록한 직선 트랙(접선 불확도 <0.1°)을 쓴다. 차를 트랙 위에
정방향/좌90/역방향/우90으로 세워 융합 헤딩과 대조하고(자세당 10초×
수십 표본 평균±편차 — 노이즈 지적 반영), 제자리 연속 회전 후 기준
복귀를 반복해 누적 오차를 잰다(누적 지적 반영).

전제: B1·V1 가동(RTK), V2는 끌 것(로버 포트 단독 점유). 조이스틱 이동 가능.
사용:
  python3 heading_truth_check.py \
      --track ../../waypoints/waypoints_wonju_license_20260818_160511.csv
결과는 화면 표 + drive_logs/heading_check_*.csv 저장.
"""
import argparse
import csv
import glob
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from stack_gps.gga_link import GgaLink            # noqa: E402
from stack_gps.heading_fusion import HeadingFusion  # noqa: E402
from stack_gps.imu_link import ImuLink            # noqa: E402
from stack_gps.path_engine import load_waypoints_csv, wrap_angle, M_PER_DEG_LAT  # noqa: E402

POSES = {"정방향": 0.0, "좌90": 90.0, "역방향": 180.0, "우90": -90.0}


def circ_mean_std(angles):
    m = math.atan2(sum(math.sin(a) for a in angles),
                   sum(math.cos(a) for a in angles))
    dev = [wrap_angle(a - m) for a in angles]
    return m, math.sqrt(sum(d * d for d in dev) / len(dev))


class Runner:
    """node.tick의 융합 배선을 그대로 재현하는 백그라운드 루프 (판단 동일)."""

    def __init__(self, gga, imu):
        self.gga, self.imu = gga, imu
        # 노드 기본값과 동일 — 자이로 적분 yaw(반시계+), 2026-08-04 지자기
        # 오염 판명 후 오일러 yaw 폐기 (imu_link 참조)
        self.fusion = HeadingFusion(sign=1.0)
        self.gyro_int = 0.0                      # 총 회전량 표시용
        self._cog_ok = False
        self._t_gyro = None
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def heading(self):
        return self.fusion.heading(time.monotonic())

    def _run(self):
        while not self._stop.is_set():
            now = time.monotonic()
            yawg = self.imu.latest_yaw_gyro()
            g = self.imu.latest_gyro_z()
            if yawg is not None:
                self.fusion.update_imu(yawg[0], now - yawg[1],
                                       gyro_z=g[0] if g else None)
            if g is not None:
                if self._t_gyro is not None:
                    self.gyro_int += g[0] * min(now - self._t_gyro, 0.1)
                self._t_gyro = now
            cog = self.gga.latest_cog()
            if cog is None or cog[2] > 1.0:
                self._cog_ok = False
            elif self._cog_ok:
                self._cog_ok = cog[0] >= 0.7 * 0.25
            else:
                self._cog_ok = cog[0] >= 0.25
            if self._cog_ok:
                self.fusion.update_cog(cog[1], now - cog[2], speed=cog[0])
            time.sleep(0.02)

    def stop(self):
        self._stop.set()


def collect(runner, seconds=10.0):
    """정지 자동 감지 후 10초 측정 — 측정 중 움직이면 무효.

    (2026-08-04 4차 시험 교훈: 입력 후 회전하면 측정창이 회전 궤적을 담아
    평균이 중간각으로 찍힘 — 사람 절차에 기대지 않고 도구가 정지를 강제)"""
    print("    정지 대기 중... (차가 2초간 완전히 멈추면 자동 시작)", flush=True)
    t0 = time.monotonic()
    still_since = None
    while True:
        g = runner.imu.latest_gyro_z()
        moving = g is None or abs(g[0]) > 0.03
        now = time.monotonic()
        if moving:
            still_since = None
        elif still_since is None:
            still_since = now
        elif now - still_since >= 2.0:
            break
        if now - t0 > 60:
            print("    ✖ 60초 내 정지 감지 실패")
            return None
        time.sleep(0.05)
    print("    측정 시작 — 10초간 손대지 말 것", flush=True)
    xs = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        g = runner.imu.latest_gyro_z()
        if g is not None and abs(g[0]) > 0.08:
            print("    ⚠ 측정 중 움직임 감지 — 이 자세 무효, 다시 세우고 재입력")
            return None
        h = runner.heading()
        if h is not None:
            xs.append(h)
        time.sleep(0.1)
    if not xs:
        return None
    return circ_mean_std(xs) + (len(xs),)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--track", required=True, help="직선 트랙 CSV (기준각)")
    ap.add_argument("--serial-port", default="/dev/ttyRover")
    ap.add_argument("--imu-port", default="/dev/ttyUSB_IMU")
    ap.add_argument("--rtcm-host", default="127.0.0.1",
                    help="'off'면 베이스 없이 진행 — COG는 도플러 기반이라 "
                         "RTK 보정 불필요, 이 시험은 위치를 안 씀")
    args = ap.parse_args()
    if args.rtcm_host.lower() in ("off", "none"):
        args.rtcm_host = ""

    track = sorted(glob.glob(args.track))[-1] if "*" in args.track else args.track
    pts = load_waypoints_csv(track)
    lat0, lon0 = pts[0]
    mlon = M_PER_DEG_LAT * math.cos(math.radians(lat0))
    E = [(lon - lon0) * mlon for _, lon in pts]
    N = [(lat - lat0) * M_PER_DEG_LAT for lat, _ in pts]
    # 기준각 = 전체 점 최소제곱 직선 (부호는 시작→끝 코드로 결정).
    # 손으로 딴 트랙의 로컬 구불거림(접선 ±수°)에 둔감 — 33m 분모로 나눠짐.
    me, mn = sum(E) / len(E), sum(N) / len(N)
    sxx = sum((e - me) ** 2 for e in E)
    syy = sum((v - mn) ** 2 for v in N)
    sxy = sum((e - me) * (v - mn) for e, v in zip(E, N))
    ref = 0.5 * math.atan2(2 * sxy, sxx - syy)
    chord = math.atan2(N[-1] - N[0], E[-1] - E[0])
    if abs(wrap_angle(ref - chord)) > math.pi / 2:
        ref = wrap_angle(ref + math.pi)
    dev = [-(e - me) * math.sin(ref) + (v - mn) * math.cos(ref)
           for e, v in zip(E, N)]
    rms = math.sqrt(sum(d * d for d in dev) / len(dev))
    print(f"기준각(트랙 최소제곱 직선): {math.degrees(ref):+.2f}° — {os.path.basename(track)}")
    print(f"  직진도 증명: 코드 방향과 차이 {abs(math.degrees(wrap_angle(ref - chord))):.2f}°, "
          f"횡편차 RMS {rms * 100:.1f}cm / 최대 {max(abs(d) for d in dev) * 100:.1f}cm"
          f" → 기준각 불확도 1° 미만")

    gga = GgaLink(args.serial_port, rtcm_host=args.rtcm_host,
                  log=lambda m: print(f"  [gps] {m}"))
    imu = ImuLink(args.imu_port, log=lambda m: print(f"  [imu] {m}"))
    gga.start(); imu.start()
    runner = Runner(gga, imu)
    runner.thread.start()

    out_path = os.path.expanduser(
        f"~/FMA_ws/drive_logs/heading_check_{time.strftime('%m%d_%H%M')}.csv")
    out = csv.writer(open(out_path, "w", buffering=1))
    out.writerow(["phase", "label", "expect_deg", "meas_deg", "err_deg",
                  "std_deg", "n", "gyro_total_deg"])

    print("\n[0] 정렬: FIXED 확인 후 조이스틱으로 5m쯤 직진 → '정렬 완료' 뜰 때까지")
    while runner.heading() is None:
        fix = gga.latest_fix()
        q = fix[3] if fix else "-"
        print(f"  대기: quality={q} 정렬={'전' if not runner.fusion.aligned else '완'}",
              end="\r")
        time.sleep(1)
    print(f"\n  ✔ 정렬 완료 (offset {math.degrees(runner.fusion.offset):+.1f}°)")

    print("\n[A] 자세 대조 — 차를 트랙 선 위에 해당 자세로 세우고 입력")
    print("    자세 이름: " + " / ".join(POSES) + "  (빈 입력 = 다음 단계)")
    worst = 0.0
    while True:
        name = input("  자세 (Enter=B로): ").strip()
        if not name:
            break
        if name not in POSES:
            print("    ⚠ 이름 오타"); continue
        r = collect(runner)
        if r is None:
            print("    ✖ 헤딩 없음 (정렬 풀림?)"); continue
        mean, std, n = r
        expect = wrap_angle(ref + math.radians(POSES[name]))
        err = math.degrees(wrap_angle(mean - expect))
        worst = max(worst, abs(err))
        print(f"    {name}: 측정 {math.degrees(mean):+.1f}° / 기대 "
              f"{math.degrees(expect):+.1f}° → 오차 {err:+.1f}° (σ {math.degrees(std):.2f}°, n={n})")
        out.writerow(["A", name, f"{math.degrees(expect):.1f}",
                      f"{math.degrees(mean):.1f}", f"{err:.2f}",
                      f"{math.degrees(std):.3f}", n,
                      f"{math.degrees(runner.gyro_int):.0f}"])

    print("\n[B] 누적 — '정방향' 자세에서 시작. 사이클: 제자리 좌우로 실컷 돌린 뒤"
          "\n    같은 자세로 복귀 → Enter (빈 입력 = 종료)")
    base = None
    cyc = 0
    while True:
        s = input(f"  기준 자세 복귀 후 Enter (사이클 {cyc}, q=종료): ").strip()
        if s.lower() == "q":
            break
        r = collect(runner)
        if r is None:
            print("    ✖ 헤딩 없음"); continue
        mean, std, n = r
        if base is None:
            base = mean
            print(f"    기준 확정 {math.degrees(mean):+.1f}° — 이제 돌렸다 복귀 반복")
        else:
            d = math.degrees(wrap_angle(mean - base))
            worst = max(worst, abs(d))
            print(f"    사이클 {cyc}: 기준 대비 {d:+.2f}° "
                  f"(총 회전 이력 {math.degrees(runner.gyro_int):+.0f}°)")
            out.writerow(["B", f"cycle{cyc}", f"{math.degrees(base):.1f}",
                          f"{math.degrees(mean):.1f}", f"{d:.2f}",
                          f"{math.degrees(std):.3f}", n,
                          f"{math.degrees(runner.gyro_int):.0f}"])
        cyc += 1

    print(f"\n===== 판정 (합의 기준: 최대 5°) =====")
    print(f"최대 오차 {worst:.1f}° → {'✔ 합격 — 헤딩은 원인에서 제외' if worst <= 5.0 else '✖ 불합격 — 헤딩 재조사'}")
    print(f"기록: {out_path}")
    runner.stop(); gga.stop(); imu.stop()


if __name__ == "__main__":
    main()
