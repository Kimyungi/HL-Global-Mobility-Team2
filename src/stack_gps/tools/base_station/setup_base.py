#!/usr/bin/env python3
"""EVK-F9P(ZED-F9P)를 RTK 베이스 스테이션으로 설정한다 (2단계).

하는 일 (UBX CFG-VALSET, RAM+BBR+FLASH 저장 → 이후 전원만 넣으면 베이스로 동작):
  1. TMODE3 = FIXED (measure_base_position.py로 확정한 좌표) 또는 --svin
  2. 항법 주기 1Hz (베이스 표준)
  3. UART2 → RTCM3 전용 출력: 1005/1074/1084/1094/1124/1230
  4. 설정 검증: FIXED면 NAV-PVT fixType=5(TIME) 확인, SVIN이면 진행률 표시

사용:
  python3 setup_base.py --lat 37.303512345 --lon 127.906512345 --height 85.1234
  python3 setup_base.py --svin                # 좌표 없이 임시 운용 (survey-in, 비권장)
  python3 setup_base.py --disable             # 베이스 해제 (로버 실습용 원복)

주의: --height 는 해발고도(MSL)가 아니라 WGS84 타원체고 —
measure_base_position.py 출력값을 그대로 쓰면 된다.
"""
import argparse
import sys
import time

import serial
from pyubx2 import UBX_PROTOCOL, UBXMessage, UBXReader

LAYERS_RAM = 1
LAYERS_ALL = 7  # RAM | BBR | FLASH

RTCM_MSGS = ["1005", "1074", "1084", "1094", "1124", "1230"]


def split_hp(value, scale_main, scale_hp):
    """u-blox 고정밀 좌표 분해: value → (main, hp). 예) 위도: 1e-7도 + 1e-9도."""
    total = round(value / scale_hp)
    ratio = round(scale_main / scale_hp)
    main, hp = divmod(total, ratio)
    return int(main), int(hp)


def build_cfg(args):
    if args.disable:
        cfg = [("CFG_TMODE_MODE", 0)]
        cfg += [(f"CFG_MSGOUT_RTCM_3X_TYPE{m}_UART2", 0) for m in RTCM_MSGS]
        return cfg

    if args.svin:
        cfg = [
            ("CFG_TMODE_MODE", 1),  # SURVEY_IN
            ("CFG_TMODE_SVIN_MIN_DUR", args.svin_dur),
            ("CFG_TMODE_SVIN_ACC_LIMIT", round(args.svin_acc * 1e4)),  # 0.1mm
        ]
    else:
        lat, lat_hp = split_hp(args.lat, 1e-7, 1e-9)
        lon, lon_hp = split_hp(args.lon, 1e-7, 1e-9)
        h, h_hp = split_hp(args.height, 1e-2, 1e-4)  # cm + 0.1mm
        cfg = [
            ("CFG_TMODE_MODE", 2),       # FIXED
            ("CFG_TMODE_POS_TYPE", 1),   # LLH
            ("CFG_TMODE_LAT", lat), ("CFG_TMODE_LAT_HP", lat_hp),
            ("CFG_TMODE_LON", lon), ("CFG_TMODE_LON_HP", lon_hp),
            ("CFG_TMODE_HEIGHT", h), ("CFG_TMODE_HEIGHT_HP", h_hp),
            ("CFG_TMODE_FIXED_POS_ACC", round(args.acc * 1e4)),  # 0.1mm
        ]

    cfg += [
        # 베이스는 1Hz가 표준 (RTCM 대역폭·로버 처리 모두 이걸 기준으로 함)
        ("CFG_RATE_MEAS", 1000), ("CFG_RATE_NAV", 1),
        # UART2 = RTCM3 전용 출력 포트
        ("CFG_UART2_BAUDRATE", args.uart2_baud),
        ("CFG_UART2OUTPROT_UBX", 0),
        ("CFG_UART2OUTPROT_NMEA", 0),
        ("CFG_UART2OUTPROT_RTCM3X", 1),
    ]
    cfg += [(f"CFG_MSGOUT_RTCM_3X_TYPE{m}_UART2", 1) for m in RTCM_MSGS]
    return cfg


def send_valset(ser, ubr, cfg, layers):
    ser.write(UBXMessage.config_set(layers, 0, cfg).serialize())
    t0 = time.time()
    while time.time() - t0 < 3:
        raw, msg = ubr.read()
        if msg is None:
            continue
        if msg.identity == "ACK-ACK":
            return True
        if msg.identity == "ACK-NAK":
            return False
    raise TimeoutError("VALSET 응답 없음 — 포트/baud 확인")


def watch_fixed(ser, ubr, seconds=15):
    """FIXED 모드 검증: fixType=5(TIME ONLY)가 뜨면 베이스 정상 동작."""
    ser.write(UBXMessage.config_set(LAYERS_RAM, 0,
              [("CFG_MSGOUT_UBX_NAV_PVT_UART1", 1)]).serialize())
    t0 = time.time()
    while time.time() - t0 < seconds:
        raw, msg = ubr.read()
        if msg is not None and msg.identity == "NAV-PVT":
            ft = msg.fixType
            print(f"[verify] fixType={ft} ({'TIME — 베이스 정상' if ft == 5 else '대기 중'})")
            if ft == 5:
                return True
    return False


def watch_svin(ser, ubr):
    """survey-in 진행률 표시 (완료까지 대기, Ctrl-C로 중단 가능)."""
    ser.write(UBXMessage.config_set(LAYERS_RAM, 0,
              [("CFG_MSGOUT_UBX_NAV_SVIN_UART1", 1)]).serialize())
    while True:
        raw, msg = ubr.read()
        if msg is None or msg.identity != "NAV-SVIN":
            continue
        acc_m = msg.meanAcc * 1e-4  # 0.1mm → m
        print(f"[svin] {msg.dur:4d}s  meanAcc={acc_m:.3f}m  "
              f"active={msg.active} valid={msg.valid}")
        if msg.valid:
            print("[svin] 완료 — 베이스 송출 시작됨. "
                  "단, 이 좌표는 재부팅 시 다시 측량됩니다(웨이포인트 재현성 없음).")
            return


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyF9P", help="F9P UART1 포트")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--lat", type=float, help="베이스 위도 [deg]")
    ap.add_argument("--lon", type=float, help="베이스 경도 [deg]")
    ap.add_argument("--height", type=float, help="베이스 타원체고 [m]")
    ap.add_argument("--acc", type=float, default=0.02,
                    help="입력 좌표의 정확도 추정치 [m] (기본 0.02)")
    ap.add_argument("--svin", action="store_true", help="survey-in 모드 (좌표 불필요)")
    ap.add_argument("--svin-dur", type=int, default=300, help="survey-in 최소 시간 [s]")
    ap.add_argument("--svin-acc", type=float, default=2.0, help="survey-in 목표 정확도 [m]")
    ap.add_argument("--uart2-baud", type=int, default=38400,
                    help="RTCM 출력(UART2) baud — 기존 udev/서버 기본값과 일치")
    ap.add_argument("--disable", action="store_true", help="베이스 해제(TMODE off)")
    args = ap.parse_args()

    if not args.disable and not args.svin and None in (args.lat, args.lon, args.height):
        ap.error("--lat/--lon/--height 를 모두 주거나, --svin 또는 --disable 을 사용하세요")

    ser = serial.Serial(args.port, args.baud, timeout=1)
    ubr = UBXReader(ser, protfilter=UBX_PROTOCOL)

    cfg = build_cfg(args)
    print(f"[setup] {len(cfg)}개 설정 키 전송 (RAM+BBR+FLASH 저장)")
    if not send_valset(ser, ubr, cfg, LAYERS_ALL):
        print("[setup] NAK — 키/값 거부됨. 좌표 범위를 확인하세요.")
        sys.exit(1)
    print("[setup] ACK — 설정 완료 및 플래시 저장됨.")

    if args.disable:
        print("[setup] 베이스 해제 완료. 로버(TMODE off) 상태로 복귀했습니다.")
        return
    if args.svin:
        watch_svin(ser, ubr)
    elif watch_fixed(ser, ubr):
        print("[setup] 베이스 가동 확인. UART2에서 RTCM3가 송출됩니다 "
              f"(rtcm_server.py --baud {args.uart2_baud} 로 배포 시작).")
    else:
        print("[setup] fixType=5가 아직 안 뜸 — 안테나 하늘 시야를 확인하고 "
              "몇 분 뒤 rtcm_server.py 통계로 재확인하세요.")


if __name__ == "__main__":
    main()
