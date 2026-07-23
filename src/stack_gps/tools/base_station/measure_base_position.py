#!/usr/bin/env python3
"""베이스 안테나 절대좌표 측정기 (1단계).

EVK-F9P가 NGII VRS 보정을 받아 RTK FIXED인 상태에서 고정밀 위치(NAV-HPPOSLLH)를
장시간 수집·평균하여 베이스 좌표를 확정한다. 결과로 setup_base.py 실행 커맨드를
그대로 출력해준다.

전제 (자세한 절차: README.md):
  - EVK-F9P UART1 = --port (기본 /dev/ttyF9P @ 115200)
  - 다른 터미널에서 NGII 보정 주입 중이어야 함: python3 ntrip_inject.py
  - ublox_gps ROS 노드는 꺼둘 것 (UART1 포트 충돌)

사용:
  python3 measure_base_position.py                  # 기본 10분 수집
  python3 measure_base_position.py --duration 1800  # 30분 수집
"""
import argparse
import math
import os
import statistics
import sys
import threading
import time

import serial


def port_is_stale(ser, path):
    """USB 재열거 감지 — 노드가 재생성되면 열어둔 fd는 옛 장치를 가리킨다."""
    try:
        st_path = os.stat(path)
        st_fd = os.fstat(ser.fileno())
        return (st_path.st_rdev != st_fd.st_rdev
                or st_path.st_ino != st_fd.st_ino)
    except OSError:
        return True
from pyubx2 import UBX_PROTOCOL, UBXMessage, UBXReader

# RTK FIXED 좌표의 잔여 흔들림 대비 넉넉한 허용치. 이보다 크면 FIX가 아니거나 환경 불량.
MAX_STD_WARN_M = 0.05


def enable_messages(ser):
    """UART1에 UBX 출력 개방 + NAV-PVT / NAV-HPPOSLLH 활성화.

    CFG_UART1OUTPROT_UBX까지 켜는 이유: 이전 세션 잔재로 출력 프로토콜이
    통째로 꺼져 있으면 메시지 활성화만으로는 아무것도 안 나온다 (실제 겪음).
    """
    msg = UBXMessage.config_set(1, 0, [
        ("CFG_UART1OUTPROT_UBX", 1),
        ("CFG_MSGOUT_UBX_NAV_PVT_UART1", 1),
        ("CFG_MSGOUT_UBX_NAV_HPPOSLLH_UART1", 1),
    ])
    ser.write(msg.serialize())


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyF9P", help="F9P UART1 포트")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--duration", type=int, default=600, help="수집 시간 [s] (기본 600)")
    ap.add_argument("--allow-float", action="store_true",
                    help="RTK FLOAT 샘플도 수집 (비권장 — FIX가 안 뜰 때 진단용)")
    args = ap.parse_args()

    def open_receiver():
        s = serial.Serial(args.port, args.baud, timeout=1)
        enable_messages(s)
        return s, UBXReader(s, protfilter=UBX_PROTOCOL)

    ser, ubr = open_receiver()

    # 무수신 감시: UBX가 한동안 안 오면 파서가 조용히 대기만 하므로 원인 안내를 띄운다.
    last_rx = [time.time()]

    def watchdog():
        while True:
            time.sleep(5)
            silent = time.time() - last_rx[0]
            if silent > 15:
                # USB 재열거로 fd가 stale이면 읽기가 조용히 굶는다 —
                # 포트를 강제로 닫아 메인 루프의 재접속 경로를 발동시킨다.
                if port_is_stale(ser, args.port):
                    print("[measure] ⚠ USB 재열거 감지 — 포트 재접속 발동")
                    try:
                        ser.close()
                    except OSError:
                        pass
                    last_rx[0] = time.time()
                    continue
                print(f"[measure] ⚠ {int(silent)}초째 UBX 무수신 — 점검: "
                      "① baud(기본 115200) ② 다른 프로세스의 포트 점유 "
                      "③ 수신기 UART1 출력 설정 (README 트러블슈팅 참조)")

    threading.Thread(target=watchdog, daemon=True).start()

    lats, lons, heights = [], [], []
    carr_soln = 0          # 최신 NAV-PVT의 carrSoln (0=없음 1=FLOAT 2=FIXED)
    need = 2 if not args.allow_float else 1
    t0 = time.time()
    t_last_print = 0.0

    print(f"[measure] {args.port} @ {args.baud} — {args.duration}s 수집 시작 "
          f"(조건: RTK {'FLOAT 이상' if args.allow_float else 'FIXED'})")
    print("[measure] 다른 터미널에서 ntrip_inject.py 가 돌고 있어야 합니다.")

    try:
        while time.time() - t0 < args.duration:
            try:
                raw, msg = ubr.read()
            except (serial.SerialException, OSError):
                # USB 순단(재열거) — 수집분은 유지한 채 포트 재접속 후 계속
                print("[measure] ⚠ 시리얼 끊김 — 3초 후 재접속 (수집분 유지)")
                try:
                    ser.close()
                except OSError:
                    pass
                time.sleep(3)
                try:
                    ser, ubr = open_receiver()
                    carr_soln = 0  # 재접속 직후 상태는 다시 확인될 때까지 불신
                except (serial.SerialException, OSError) as e:
                    print(f"[measure] 재접속 실패({e}) — 3초 후 재시도")
                continue
            if msg is None:
                continue
            last_rx[0] = time.time()
            if msg.identity == "NAV-PVT":
                carr_soln = getattr(msg, "carrSoln", 0)
            elif msg.identity == "NAV-HPPOSLLH" and carr_soln >= need:
                # pyubx2가 고정밀(HP) 성분을 lat/lon/height에 이미 합산해준다.
                # lat/lon은 도 단위, height는 타원체고 mm 단위.
                lats.append(msg.lat)
                lons.append(msg.lon)
                heights.append(msg.height * 1e-3)

            now = time.time()
            if now - t_last_print >= 5:
                t_last_print = now
                soln = {0: "NO-RTK", 1: "FLOAT", 2: "FIXED"}.get(carr_soln, "?")
                print(f"[measure] {int(now - t0):4d}s  carrSoln={soln:6s}  "
                      f"수집 샘플={len(lats)}")
    except KeyboardInterrupt:
        print("\n[measure] 중단 — 지금까지 수집분으로 계산합니다.")

    if len(lats) < 30:
        print(f"[measure] 샘플 부족({len(lats)}개) — RTK FIX 여부와 보정 주입을 확인하세요.")
        sys.exit(1)

    lat_m, lon_m, h_m = (statistics.fmean(v) for v in (lats, lons, heights))
    # 표준편차를 m 단위로 환산 (위도 1도 ≈ 111.32km, 경도는 cos(lat) 보정)
    lat_std = statistics.pstdev(lats) * 111_320
    lon_std = statistics.pstdev(lons) * 111_320 * math.cos(math.radians(lat_m))
    h_std = statistics.pstdev(heights)

    print("\n===== 베이스 안테나 좌표 (WGS84, 타원체고) =====")
    print(f"  샘플 수 : {len(lats)}")
    print(f"  위도    : {lat_m:.9f} deg  (std {lat_std * 100:.1f} cm)")
    print(f"  경도    : {lon_m:.9f} deg  (std {lon_std * 100:.1f} cm)")
    print(f"  타원체고: {h_m:.4f} m       (std {h_std * 100:.1f} cm)")
    if max(lat_std, lon_std, h_std) > MAX_STD_WARN_M:
        print(f"  ⚠ 표준편차가 {MAX_STD_WARN_M * 100:.0f}cm를 넘습니다. "
              "FIX 불안정 — 더 길게 재측정을 권장합니다.")
    print("\n이 좌표는 앞으로 절대 바꾸지 않습니다. 아래 명령으로 2단계 진행:")
    print(f"\n  python3 setup_base.py --lat {lat_m:.9f} --lon {lon_m:.9f} "
          f"--height {h_m:.4f}\n")


if __name__ == "__main__":
    main()
