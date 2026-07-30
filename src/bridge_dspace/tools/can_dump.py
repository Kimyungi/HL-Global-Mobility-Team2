#!/usr/bin/env python3
"""candump + 프로토콜 해석 CAN 수신 모니터 (dSPACE 링크 디버그용).

PROTOCOL.md의 ID 맵을 알고 있어서 raw hex와 함께 공학 단위 해석을 출력한다.
(순수 raw는 can-utils의 `candump can0`으로도 충분)

사용:
    python3 can_dump.py                          # can0 수신
    python3 can_dump.py --iface vcan0 --changes  # 값이 바뀔 때만 출력
"""
import argparse
import socket
import struct
import time

ID_TARGET_HEADER = 0x100
ID_REF_POINT_BASE = 0x101   # 0x101..0x114
NUM_POINTS = 20
ID_VEH_POSE = 0x200
ID_VEH_VEL = 0x201
ID_VEH_COMMIT = 0x202

STATES = {0: "lane", 1: "waypoint", 2: "avoid", 3: "parking"}

# 양자화 스케일 — can_protocol.hpp와 일치할 것
POS_SCALE = 1e-3
YAW_SCALE = 1e-4
CURV_SCALE = 5e-4
VEL_SCALE = 1e-3


def decode(can_id: int, d: bytes) -> str:
    if can_id == ID_TARGET_HEADER:
        counter, state, n_points, v_ref, _ = struct.unpack("<HBBhH", d)
        return (f"HEADER counter={counter} state={STATES.get(state, state)} "
                f"n={n_points} v_ref={v_ref * VEL_SCALE:.3f}m/s")
    if ID_REF_POINT_BASE <= can_id < ID_REF_POINT_BASE + NUM_POINTS:
        x, y, yaw, curv = struct.unpack("<hhhh", d)
        return (f"PT[{can_id - ID_REF_POINT_BASE:02d}] x={x * POS_SCALE:.3f} "
                f"y={y * POS_SCALE:.3f} yaw={yaw * YAW_SCALE:.4f} "
                f"k={curv * CURV_SCALE:.4f}")
    if can_id == ID_VEH_POSE:
        x, y = struct.unpack("<ff", d)
        return f"POSE x={x:.3f} y={y:.3f}"
    if can_id == ID_VEH_VEL:
        yaw, v = struct.unpack("<ff", d)
        return f"VEL yaw={yaw:.4f} v={v:.3f}"
    if can_id == ID_VEH_COMMIT:
        s, counter, _ = struct.unpack("<fHH", d)
        return f"COMMIT str={s:.4f} counter={counter}"
    return "?"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--changes", action="store_true", help="페이로드가 바뀔 때만 출력")
    args = ap.parse_args()

    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((args.iface,))
    print(f"listening {args.iface}  (Ctrl-C 종료)")

    last = {}
    n = 0
    t_rate = time.monotonic()
    n_rate = 0
    try:
        while True:
            frame = s.recv(16)
            can_id, dlc = struct.unpack("<IB3x", frame[:8])
            can_id &= socket.CAN_EFF_MASK
            data = frame[8:8 + dlc]
            n += 1
            n_rate += 1
            now = time.monotonic()
            if now - t_rate >= 5.0:
                print(f"--- {n_rate / (now - t_rate):.0f} frame/s, 누적 {n} ---")
                t_rate, n_rate = now, 0
            if args.changes and last.get(can_id) == data:
                continue
            last[can_id] = data
            ts = time.strftime("%H:%M:%S") + f".{int((time.time() % 1) * 1000):03d}"
            print(f"{ts}  0x{can_id:03X}  [{data.hex(' ')}]  {decode(can_id, data)}")
    except KeyboardInterrupt:
        print(f"\n총 {n}프레임")


if __name__ == "__main__":
    main()
