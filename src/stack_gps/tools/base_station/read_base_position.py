#!/usr/bin/env python3
"""EVK-F9P 에 **지금 저장돼 있는** 베이스 좌표를 읽어낸다 (덮어쓰기 전 백업용).

왜 필요한가 — 베이스 좌표의 사본은 딱 두 군데뿐이다.

  ① F9P 플래시 (setup_base.py 가 RAM+BBR+FLASH 에 쓴다) — **한 벌만 남는다**
  ② base_station/README.md 의 "지점별 좌표 기록" — 사람이 손으로 옮겨 적은 것

`measure_base_position.py` 는 결과를 **화면에만** 출력하고 파일로 남기지 않는다.
그래서 ②에 적어 두지 않은 채 새 지점을 측량해 `setup_base.py` 를 돌리면 옛 좌표는
복구할 데가 없어진다. 그 지점에서 기록한 코스 CSV 가 통째로 못 쓰게 된다 —
웨이포인트 절대좌표 = 베이스 좌표 + RTK 기선이라, 베이스 값이 달라지면 코스 전체가
그만큼 밀리기 때문이다.

**그러니 새 지점을 측량하기 전에 이걸 먼저 돌리고, 출력을 README 에 붙여 넣을 것.**

읽기 전용이다 — 수신기 설정을 바꾸지 않는다 (UBX-CFG-VALGET 폴링).

사용:
  python3 read_base_position.py                    # 기본 포트로 현재 좌표 읽기
  python3 read_base_position.py --port /dev/ttyACM0 --baud 38400
  python3 read_base_position.py --layer flash      # 플래시에 저장된 값만 (기본: RAM)
"""
import argparse
import sys
import time

import serial
from pyubx2 import UBX_PROTOCOL, UBXMessage, UBXReader

# CFG-VALGET layer 코드 (u-blox interface description)
LAYERS = {"ram": 0, "bbr": 1, "flash": 2, "default": 7}

KEYS = [
    "CFG_TMODE_MODE",
    "CFG_TMODE_POS_TYPE",
    "CFG_TMODE_LAT", "CFG_TMODE_LAT_HP",
    "CFG_TMODE_LON", "CFG_TMODE_LON_HP",
    "CFG_TMODE_HEIGHT", "CFG_TMODE_HEIGHT_HP",
    "CFG_TMODE_FIXED_POS_ACC",
]

MODE_NAME = {0: "DISABLED (베이스 아님)", 1: "SURVEY_IN", 2: "FIXED"}


def poll(ser, ubr, layer, timeout=3.0):
    """CFG-VALGET 으로 KEYS 를 읽어 dict 로 돌려준다."""
    ser.write(UBXMessage.config_poll(layer, 0, KEYS).serialize())
    got = {}
    t0 = time.time()
    while time.time() - t0 < timeout:
        raw, msg = ubr.read()
        if raw is None:
            continue
        if msg.identity == "ACK-NAK":
            raise RuntimeError(
                "NAK — 수신기가 CFG-VALGET 을 거부했습니다 "
                "(펌웨어/키 확인). 포트·baud 도 함께 확인하세요.")
        if msg.identity != "CFG-VALGET":
            continue
        for k in KEYS:
            if hasattr(msg, k):
                got[k] = getattr(msg, k)
        if len(got) == len(KEYS):
            return got
    if got:
        return got
    raise TimeoutError(
        "CFG-VALGET 응답 없음 — 포트/baud 를 확인하세요 "
        "(EVK-F9P USB 는 보통 /dev/ttyACM0).")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=38400)
    ap.add_argument("--layer", choices=sorted(LAYERS), default="ram",
                    help="읽을 설정 계층 (기본 ram = 현재 동작값)")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
    except serial.SerialException as e:
        print(f"[read] 포트를 열 수 없음: {e}", file=sys.stderr)
        return 2

    with ser:
        ubr = UBXReader(ser, protfilter=UBX_PROTOCOL)
        try:
            v = poll(ser, ubr, LAYERS[args.layer])
        except (RuntimeError, TimeoutError) as e:
            print(f"[read] {e}", file=sys.stderr)
            return 2

    mode = v.get("CFG_TMODE_MODE")
    print(f"\n[read] {args.port} — 계층 {args.layer}")
    print(f"  TMODE       : {MODE_NAME.get(mode, mode)}")

    if mode != 2:
        print("\n  ⚠ FIXED 가 아니라 저장된 확정 좌표가 없습니다.")
        if mode == 1:
            print("     SURVEY_IN 으로 돌고 있습니다 — 이 위치는 전원을 내리면 사라지고,")
            print("     이 상태로 기록한 코스는 재현되지 않습니다 (README 1단계 참조).")
        return 1

    # setup_base.py 의 split_hp 역변환 — main 은 1e-7도, hp 는 1e-9도 (고도는 cm + 0.1mm)
    lat = v["CFG_TMODE_LAT"] * 1e-7 + v["CFG_TMODE_LAT_HP"] * 1e-9
    lon = v["CFG_TMODE_LON"] * 1e-7 + v["CFG_TMODE_LON_HP"] * 1e-9
    height = v["CFG_TMODE_HEIGHT"] * 1e-2 + v["CFG_TMODE_HEIGHT_HP"] * 1e-4
    acc = v.get("CFG_TMODE_FIXED_POS_ACC", 0) * 1e-4

    print(f"  위도        : {lat:.9f} deg")
    print(f"  경도        : {lon:.9f} deg")
    print(f"  타원체고    : {height:.4f} m")
    print(f"  설정 정확도 : {acc:.4f} m")

    print("\n── base_station/README.md 의 \"지점별 좌표 기록\"에 붙여 넣을 줄 ──")
    print(f"> - <지점 이름> (<날짜>): "
          f"`lat {lat:.9f} / lon {lon:.9f} / 타원체고 {height:.4f} m`")

    print("\n── 이 지점으로 되돌릴 때 쓸 커맨드 ──")
    print(f"  python3 setup_base.py --lat {lat:.9f} --lon {lon:.9f} "
          f"--height {height:.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
