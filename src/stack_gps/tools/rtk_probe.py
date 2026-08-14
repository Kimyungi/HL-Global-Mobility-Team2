#!/usr/bin/env python3
"""RTK 진단 프로브 — record_waypoints와 동일한 경로(RTCM 주입 + GGA 수신)로
FIXED 여부를 보되, 간섭 판정용으로 **C/N0(GSV 신호세기)**를 추가로 뽑는다.

C/N0가 핵심: RF 간섭이면 위성 수·HDOP는 그대로여도 C/N0가 몇 dB 떨어진다.
(2026-08-14 로그 분석에서 위성 12개·HDOP 0.6이 정상 run과 동일했던 이유)

사용: python3 rtk_probe.py --seconds 60 [--no-rtcm] [--tag baseline]
"""
import argparse
import socket
import sys
import time

import serial

QNAME = {0: "NOFIX", 1: "GPS", 2: "DGPS", 4: "FIXED", 5: "FLOAT"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial", default="/dev/ttyRover")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2101)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--no-rtcm", action="store_true", help="RTCM 주입 없이 관찰만")
    ap.add_argument("--interval", type=float, default=2.0, help="출력 주기 [s]")
    ap.add_argument("--cn0-ttl", type=float, default=4.0,
                    help="이 시간 안에 안 보인 위성은 C/N0 집계에서 제외 [s]")
    ap.add_argument("--target", type=float, default=38.0,
                    help="합격 C/N0 [dB] — 넘으면 줄 앞에 OK 표시")
    ap.add_argument("--tag", default="")
    args = ap.parse_args()

    ser = serial.Serial(args.serial, args.baud, timeout=0.2)
    sock = None
    if not args.no_rtcm:
        sock = socket.create_connection((args.host, args.port), timeout=5)
        sock.setblocking(False)

    quality = sats = 0
    hdop = 0.0
    # prn -> (C/N0, 수신시각). 만료(--cn0-ttl)로 오래된 위성을 버린다 — 누적만 하면
    # 안테나를 옮겨도 옛 값이 남아 반응이 굼떠서 위치 탐색에 못 쓴다 (2026-08-14).
    cn0 = {}
    cn0_epoch = {}    # 수집 중인 에폭
    rtcm_bytes = 0
    buf = b""
    t_end = time.time() + args.seconds
    t_stat = time.time()
    print(f"[probe] {args.tag or 'run'} — {args.seconds:.0f}s, "
          f"RTCM {'꺼짐' if args.no_rtcm else args.host}, "
          f"{args.interval:.0f}초마다 출력, 목표 C/N0 {args.target:.0f}dB", flush=True)

    while time.time() < t_end:
        if sock is not None:
            try:
                while True:
                    d = sock.recv(4096)
                    if not d:
                        raise ConnectionError("RTCM 서버 종료")
                    ser.write(d)
                    rtcm_bytes += len(d)
            except (BlockingIOError, InterruptedError):
                pass
            except OSError as e:
                print(f"[probe] ⚠ RTCM 오류: {e}", flush=True)
                sock = None

        buf += ser.read(ser.in_waiting or 1)
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode(errors="ignore").strip()
            if not line.startswith("$G"):
                continue
            f = line.split(",")
            tag = line[3:6]
            if tag == "GGA" and len(f) >= 12 and f[6].isdigit():
                quality = int(f[6])
                sats = int(f[7]) if f[7].isdigit() else 0
                hdop = float(f[8]) if f[8] else 0.0
            elif tag == "GSV" and len(f) >= 4:
                try:
                    msg_n, msg_i = int(f[1]), int(f[2])
                except ValueError:
                    continue
                for k in range(4, len(f) - 3, 4):
                    try:
                        prn, snr = f[k], f[k + 3].split("*")[0]
                        if prn and snr:
                            cn0_epoch[f"{line[1:3]}{prn}"] = int(snr)
                    except (ValueError, IndexError):
                        pass
                if msg_i == msg_n:      # 에폭 끝 — 스냅샷 확정
                    now_m = time.monotonic()
                    for k, v in cn0_epoch.items():
                        cn0[k] = (v, now_m)
                    cn0_epoch = {}

        now = time.time()
        if now - t_stat >= args.interval:
            cutoff = time.monotonic() - args.cn0_ttl
            for k in [k for k, (_, t) in cn0.items() if t < cutoff]:
                del cn0[k]                      # 안 보이는 위성은 버린다
            vals = sorted((v for v, _ in cn0.values()), reverse=True)
            top = vals[:8]
            mean_top = sum(top) / len(top) if top else 0.0
            strong = sum(1 for v in vals if v >= 40)
            mark = "OK " if mean_top >= args.target else "-- "
            print(f"  {mark}{QNAME.get(quality, quality):5}  "
                  f"C/N0 {mean_top:4.1f}dB  40dB↑ {strong:2d}  "
                  f"위성 {len(vals):2d}  HDOP {hdop:4.1f}  "
                  f"RTCM {rtcm_bytes / (now - t_stat):5.0f}B/s", flush=True)
            rtcm_bytes = 0
            t_stat = now

    ser.close()
    if sock:
        sock.close()
    print(f"[probe] 끝 — 최종 {QNAME.get(quality, quality)}", flush=True)


if __name__ == "__main__":
    main()
