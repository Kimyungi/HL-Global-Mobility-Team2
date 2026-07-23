#!/usr/bin/env python3
"""웨이포인트 기록 도구 — RTK FIXED 좌표를 걸으며/주행하며 CSV로 기록.

하나의 프로세스가 두 역할을 겸한다 (포트 공유 금지 원칙):
  ① 베이스 RTCM을 TCP로 받아 로버 시리얼에 주입 (rtcm_client_inject 역할)
  ② 로버의 UBX 고정밀 위치(NAV-HPPOSLLH, mm 해상도)를 읽어 웨이포인트 기록

⚠ 실행 전 rtcm_client_inject.py 는 반드시 종료할 것 — 같은 포트를 두 프로세스가
읽으면 데이터가 조각난다. 이 도구 하나만 돌리면 주입+기록이 모두 된다.

기록 규칙:
  - RTK FIXED(carrSoln=2)인 순간의 좌표만 기록. FLOAT/NONE 구간은 자동 일시정지.
  - 직전 기록점에서 --spacing (기본 0.2m) 이상 이동했을 때만 새 점 추가.
  - Ctrl-C 로 종료 → CSV 마감 + 요약 출력.

출력: src/stack_gps/waypoints/waypoints_[이름_]YYYYMMDD_HHMMSS.csv
  idx, utc, lat, lon, height_m, east_m, north_m, h_acc_mm
  (east/north 는 첫 점 기준 로컬 미터 좌표 — 시각화·검토용)

사용:
  python3 record_waypoints.py --host 172.20.10.2                  # 기본
  python3 record_waypoints.py --host 172.20.10.2 --name track_A --spacing 0.5
"""
import argparse
import csv
import math
import os
import socket
import sys
import time

import serial
from pyubx2 import UBX_PROTOCOL, UBXMessage, UBXReader

M_PER_DEG_LAT = 111_320.0


def enable_ubx(ser):
    """로버 USB 포트에 UBX 항법 출력(5Hz) 활성화 (RAM — 전원 차단 시 원복)."""
    msg = UBXMessage.config_set(1, 0, [
        ("CFG_USBOUTPROT_UBX", 1),
        ("CFG_MSGOUT_UBX_NAV_PVT_USB", 1),
        ("CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB", 1),
        ("CFG_RATE_MEAS", 200),   # 5Hz — 도보 1.4m/s 기준 점 간격 0.28m 해상도
        ("CFG_RATE_NAV", 1),
    ])
    ser.write(msg.serialize())


def connect_base(host, port):
    s = socket.create_connection((host, port), timeout=5)
    s.setblocking(False)
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", required=True, help="베이스 RTCM 서버 IP")
    ap.add_argument("--port", type=int, default=2101)
    ap.add_argument("--serial", default="/dev/ttyRover", help="로버 시리얼 포트")
    ap.add_argument("--baud", type=int, default=115200, help="USB CDC는 무의미, RS232면 38400")
    ap.add_argument("--spacing", type=float, default=0.2, help="기록 점 간격 [m]")
    ap.add_argument("--name", default="", help="트랙 이름 (파일명에 포함)")
    ap.add_argument("--outdir", default=os.path.join(os.path.dirname(__file__),
                    "..", "..", "waypoints"), help="CSV 저장 폴더")
    args = ap.parse_args()

    outdir = os.path.abspath(args.outdir)
    os.makedirs(outdir, exist_ok=True)
    tag = f"{args.name}_" if args.name else ""
    path = os.path.join(outdir, f"waypoints_{tag}{time.strftime('%Y%m%d_%H%M%S')}.csv")

    ser = serial.Serial(args.serial, args.baud, timeout=1)
    ubr = UBXReader(ser, protfilter=UBX_PROTOCOL)
    enable_ubx(ser)
    sock = connect_base(args.host, args.port)
    print(f"[record] 베이스 {args.host}:{args.port} → {args.serial} 주입 시작")
    print(f"[record] 기록 파일: {path}")
    print(f"[record] 점 간격 {args.spacing}m — RTK FIXED에서만 기록. Ctrl-C로 종료.")

    f = open(path, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(["idx", "utc", "lat", "lon", "height_m",
                     "east_m", "north_m", "h_acc_mm"])

    carr = 0            # 최신 carrSoln
    origin = None       # (lat0, lon0) — ENU 기준점
    last_en = None      # 마지막 기록점 (e, n)
    n_pts = 0
    total_len = 0.0
    rtcm_bytes = 0
    t_stat = time.time()
    names = {0: "NO-RTK", 1: "FLOAT", 2: "FIXED"}

    try:
        while True:
            # ① RTCM 펌프 (논블로킹 — 밀린 만큼 전부 로버로)
            try:
                while True:
                    d = sock.recv(4096)
                    if not d:
                        raise ConnectionError("서버 연결 종료")
                    ser.write(d)
                    rtcm_bytes += len(d)
            except (BlockingIOError, InterruptedError):
                pass
            except OSError as e:
                print(f"[record] ⚠ 베이스 연결 오류: {e} — 3초 후 재접속")
                time.sleep(3)
                try:
                    sock = connect_base(args.host, args.port)
                except OSError:
                    pass

            # ② 로버 위치 읽기
            raw, msg = ubr.read()
            if msg is None:
                continue
            if msg.identity == "NAV-PVT":
                carr = msg.carrSoln
            elif msg.identity == "NAV-HPPOSLLH" and carr == 2:
                lat, lon = msg.lat, msg.lon        # pyubx2가 HP 성분 합산 완료 (deg)
                h = msg.height * 1e-3              # mm → m (타원체고)
                if origin is None:
                    origin = (lat, lon)
                    print(f"[record] 기준점 고정: {lat:.9f}, {lon:.9f}")
                e = (lon - origin[1]) * M_PER_DEG_LAT * math.cos(math.radians(origin[0]))
                n = (lat - origin[0]) * M_PER_DEG_LAT
                if last_en is None:
                    dist = 0.0
                else:
                    dist = math.hypot(e - last_en[0], n - last_en[1])
                if last_en is None or dist >= args.spacing:
                    utc = time.strftime("%H:%M:%S", time.gmtime())
                    writer.writerow([n_pts, utc, f"{lat:.9f}", f"{lon:.9f}",
                                     f"{h:.4f}", f"{e:.3f}", f"{n:.3f}", msg.hAcc])
                    f.flush()
                    if last_en is not None:
                        total_len += dist
                    last_en = (e, n)
                    n_pts += 1

            # ③ 상태 표시 (2초마다)
            now = time.time()
            if now - t_stat >= 2:
                sys.stdout.write(
                    f"\r[record] {names.get(carr, '?'):6s}  점 {n_pts:4d}개  "
                    f"경로 {total_len:7.1f}m  RTCM {rtcm_bytes / (now - t_stat):5.0f}B/s   ")
                sys.stdout.flush()
                rtcm_bytes = 0
                t_stat = now
                if carr != 2 and n_pts > 0:
                    sys.stdout.write("⚠ FIX 풀림 — 기록 일시정지  ")
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
        try:
            sock.close()
        except OSError:
            pass
        ser.close()

    print(f"\n[record] 종료 — 점 {n_pts}개, 경로 길이 {total_len:.1f}m")
    print(f"[record] 저장됨: {path}")
    if n_pts < 2:
        print("[record] ⚠ 점이 거의 없음 — FIXED 상태와 이동 여부를 확인하세요.")


if __name__ == "__main__":
    main()
