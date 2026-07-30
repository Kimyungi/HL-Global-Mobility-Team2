#!/usr/bin/env python3
"""웨이포인트 기록 도구 — RTK FIXED 좌표를 걸으며/주행하며 CSV로 기록.

하나의 프로세스가 두 역할을 겸한다 (포트 공유 금지 원칙):
  ① 베이스 RTCM을 TCP로 받아 로버 시리얼에 주입 (rtcm_client_inject 역할)
  ② 로버의 NMEA GGA를 파싱해 웨이포인트 기록

NMEA 기반인 이유: FST-UEF9P의 USB는 출하 설정상 NMEA 출력 + RTCM 입력만
허용하고 UBX 설정 명령을 받지 않는다 (실측 확인). GGA 좌표는 분 단위 소수
5자리 = 약 1.85cm 양자화 → RTK 오차와 합산해 웨이포인트 유효 정밀도 2~3cm.

⚠ 실행 전 rtcm_client_inject.py 는 반드시 종료할 것 — 같은 포트를 두 프로세스가
읽으면 문장이 조각난다. 이 도구 하나만 돌리면 주입+기록이 모두 된다.

기록 규칙:
  - GGA quality=4 (RTK FIXED)인 문장만 기록. FLOAT/DGPS 구간은 자동 일시정지.
  - 직전 기록점에서 --spacing (기본 0.2m) 이상 이동했을 때만 새 점 추가.
  - Ctrl-C 로 종료 → CSV 마감 + 요약 출력.

출력: src/stack_gps/waypoints/waypoints_[이름_]YYYYMMDD_HHMMSS.csv
  idx, utc, lat, lon, height_m, east_m, north_m, quality
  (height_m = 타원체고 — GGA의 MSL고도 + 지오이드 분리값)

사용:
  python3 record_waypoints.py --host 100.70.198.29                  # 기본
  python3 record_waypoints.py --host 100.70.198.29 --name track_A --spacing 0.5
"""
import argparse
import csv
import math
import os
import socket
import sys
import time

import serial

M_PER_DEG_LAT = 111_320.0


def parse_gga(line):
    """GGA 문장 → (utc, lat, lon, h_ellip, quality) 또는 None.

    lat/lon: ddmm.mmmmm 형식을 십진도로 변환. 좌표 비면 None.
    """
    f = line.split(",")
    if len(f) < 12 or not f[2] or not f[4] or not f[6].isdigit():
        return None
    try:
        lat = int(f[2][:2]) + float(f[2][2:]) / 60.0
        if f[3] == "S":
            lat = -lat
        lon = int(f[4][:3]) + float(f[4][3:]) / 60.0
        if f[5] == "W":
            lon = -lon
        alt_msl = float(f[9]) if f[9] else 0.0
        geoid = float(f[11]) if f[11] else 0.0
        return f[1], lat, lon, alt_msl + geoid, int(f[6])
    except ValueError:
        return None


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
    ap.add_argument("--min-quality", type=int, default=4,
                    help="기록 최소 GGA quality (기본 4=RTK FIXED, 5 허용 시 주의)")
    ap.add_argument("--outdir", default=os.path.join(os.path.dirname(__file__),
                    "..", "..", "waypoints"), help="CSV 저장 폴더")
    args = ap.parse_args()

    outdir = os.path.abspath(args.outdir)
    os.makedirs(outdir, exist_ok=True)
    tag = f"{args.name}_" if args.name else ""
    path = os.path.join(outdir, f"waypoints_{tag}{time.strftime('%Y%m%d_%H%M%S')}.csv")

    ser = serial.Serial(args.serial, args.baud, timeout=0.2)
    sock = connect_base(args.host, args.port)
    print(f"[record] 베이스 {args.host}:{args.port} → {args.serial} 주입 시작")
    print(f"[record] 기록 파일: {path}")
    print(f"[record] 점 간격 {args.spacing}m — quality≥{args.min_quality}에서만 기록. "
          "Ctrl-C로 종료.")

    f = open(path, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(["idx", "utc", "lat", "lon", "height_m",
                     "east_m", "north_m", "quality"])

    quality = 0
    origin = None       # (lat0, lon0) — ENU 기준점
    last_en = None      # 마지막 기록점 (e, n)
    n_pts = 0
    total_len = 0.0
    rtcm_bytes = 0
    nmea_buf = b""
    t_stat = time.time()
    qnames = {0: "NOFIX", 1: "GPS", 2: "DGPS", 4: "FIXED", 5: "FLOAT"}

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
                print(f"\n[record] ⚠ 베이스 연결 오류: {e} — 3초 후 재접속")
                time.sleep(3)
                try:
                    sock = connect_base(args.host, args.port)
                except OSError:
                    pass

            # ② NMEA 수신·GGA 파싱
            nmea_buf += ser.read(ser.in_waiting or 1)
            while b"\n" in nmea_buf:
                raw, nmea_buf = nmea_buf.split(b"\n", 1)
                line = raw.decode(errors="ignore").strip()
                if not (line.startswith("$G") and "GGA" in line[:7]):
                    continue
                parsed = parse_gga(line)
                if parsed is None:
                    continue
                utc, lat, lon, h, quality = parsed
                if quality < args.min_quality:
                    continue
                if origin is None:
                    origin = (lat, lon)
                    print(f"[record] 기준점 고정: {lat:.7f}, {lon:.7f}")
                e = (lon - origin[1]) * M_PER_DEG_LAT * math.cos(math.radians(origin[0]))
                n = (lat - origin[0]) * M_PER_DEG_LAT
                dist = 0.0 if last_en is None else math.hypot(e - last_en[0],
                                                              n - last_en[1])
                if last_en is None or dist >= args.spacing:
                    writer.writerow([n_pts, utc, f"{lat:.7f}", f"{lon:.7f}",
                                     f"{h:.3f}", f"{e:.3f}", f"{n:.3f}", quality])
                    f.flush()
                    if last_en is not None:
                        total_len += dist
                    last_en = (e, n)
                    n_pts += 1

            # ③ 상태 표시 (2초마다)
            now = time.time()
            if now - t_stat >= 2:
                warn = "  ⚠ FIX 아님 — 기록 일시정지" if quality < args.min_quality else ""
                sys.stdout.write(
                    f"\r[record] {qnames.get(quality, quality):5}  점 {n_pts:4d}개  "
                    f"경로 {total_len:7.1f}m  RTCM {rtcm_bytes / (now - t_stat):5.0f}B/s{warn}   ")
                sys.stdout.flush()
                rtcm_bytes = 0
                t_stat = now
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
