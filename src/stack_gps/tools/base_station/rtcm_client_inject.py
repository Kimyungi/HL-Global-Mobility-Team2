#!/usr/bin/env python3
"""RTCM 주입 클라이언트 — 차량 PC용 (4단계, FST-UEF9P 대비).

베이스 PC의 rtcm_server.py에 TCP로 접속해 받은 RTCM3를 로버 수신기의 시리얼
포트에 그대로 써 넣는다. --monitor를 주면 같은 포트에서 나오는 NMEA GGA를
읽어 RTK 상태(quality 4=FIXED, 5=FLOAT)를 함께 표시한다.

대상 로버:
  - FST-UEF9P(차량): RS-232 38400bps — USB-RS232 어댑터의 /dev/ttyUSB* 사용
  - EVK-F9P를 임시 로버로 쓸 때: --serial /dev/ttyF9P_uart2 (베이스 검증용)

사용 (차량 PC):
  python3 rtcm_client_inject.py --host <베이스PC_IP> --monitor

ntrip_inject.py 와 동일 구조 — 소스만 NGII NTRIP → 자체 베이스 TCP로 교체한 것.
"""
import argparse
import socket
import time

import serial


def gga_quality(line):
    """$G?GGA 문장에서 quality 필드(6번째) 추출. GGA가 아니면 None."""
    if not line.startswith("$") or "GGA" not in line[:7]:
        return None
    parts = line.split(",")
    if len(parts) > 6 and parts[6].isdigit():
        return int(parts[6])
    return None


QUALITY_NAMES = {0: "NO FIX", 1: "GPS", 2: "DGPS", 4: "RTK FIXED", 5: "RTK FLOAT"}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", required=True, help="베이스 PC IP")
    ap.add_argument("--port", type=int, default=2101, help="rtcm_server.py TCP 포트")
    ap.add_argument("--serial", default="/dev/ttyUSB0", help="로버 시리얼 포트")
    ap.add_argument("--baud", type=int, default=38400, help="FST-UEF9P는 38400 고정")
    ap.add_argument("--monitor", action="store_true", help="NMEA GGA로 RTK 상태 표시")
    args = ap.parse_args()

    ser = serial.Serial(args.serial, args.baud, timeout=0.1)
    print(f"[client] 로버 포트 열림: {args.serial} @ {args.baud}")

    nmea_buf = b""
    quality = None
    t_stats = time.time()
    n_bytes = 0

    while True:  # TCP 재접속 루프
        try:
            sock = socket.create_connection((args.host, args.port), timeout=5)
            sock.settimeout(0.2)
            print(f"[client] 베이스 접속: {args.host}:{args.port}")
            while True:
                try:
                    data = sock.recv(2048)
                    if not data:
                        raise ConnectionError("서버가 연결을 닫음")
                    ser.write(data)
                    n_bytes += len(data)
                except socket.timeout:
                    pass

                if args.monitor and ser.in_waiting:
                    nmea_buf += ser.read(ser.in_waiting)
                    while b"\n" in nmea_buf:
                        line, nmea_buf = nmea_buf.split(b"\n", 1)
                        q = gga_quality(line.decode(errors="ignore").strip())
                        if q is not None:
                            quality = q

                now = time.time()
                if now - t_stats >= 5:
                    msg = f"[client] RTCM {n_bytes / (now - t_stats):6.0f} B/s"
                    if args.monitor:
                        msg += f"  RTK 상태: {QUALITY_NAMES.get(quality, quality)}"
                    print(msg)
                    n_bytes, t_stats = 0, now
        except (OSError, ConnectionError) as e:
            print(f"[client] 연결 오류: {e} — 3초 후 재접속")
            time.sleep(3)


if __name__ == "__main__":
    main()
