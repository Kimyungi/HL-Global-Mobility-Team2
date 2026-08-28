#!/usr/bin/env python3
"""candump + 프로토콜 해석 CAN 수신 모니터 (dSPACE 링크 디버그용).

PROTOCOL.md의 ID 맵을 알고 있어서 raw hex와 함께 공학 단위 해석을 출력한다.
(순수 raw는 can-utils의 `candump can0`으로도 충분)

classic·CAN FD 를 **둘 다** 받는다 (CAN_RAW_FD_FRAMES). 각 줄에 어느 포맷으로
왔는지 표시하므로, dSPACE 가 FD 로 넘어왔는지 여기서 바로 확인할 수 있다.

사용:
    python3 can_dump.py                          # can0 수신
    python3 can_dump.py --iface vcan0 --changes  # 값이 바뀔 때만 출력
"""
import argparse
import socket
import struct
import time

ID_TARGET_HEADER = 0x100
ID_REF_POINT_BASE = 0x101   # v5 — 0x101 한 개
NUM_POINTS = 1
ID_VEH_FEEDBACK = 0x200     # v5 — 64B 단일 프레임
# v3 (8B ×3) 잔재 — dSPACE 가 되돌아가도 해석되게 남긴다
ID_VEH_POSE = 0x200
ID_VEH_VEL = 0x201
ID_VEH_COMMIT = 0x202

STATES = {0: "lane", 1: "waypoint", 2: "avoid", 3: "parking"}

CAN_MTU = 16          # classic can_frame
CANFD_MTU = 72        # canfd_frame
CANFD_BRS = 0x01      # 데이터 구간 비트레이트 전환 플래그

# 양자화 스케일 — can_protocol.hpp와 일치할 것
POS_SCALE = 1e-3
YAW_SCALE = 1e-4
CURV_SCALE = 5e-4
VEL_SCALE = 1e-3


def decode(can_id: int, d: bytes) -> str:
    # ── v5 (PR #52) 64바이트 페이로드. 길이가 계약을 가른다.
    if len(d) == 64:
        if can_id == ID_REF_POINT_BASE:
            x, y, yaw, k, dx, dy, dyaw, upd = struct.unpack("<7dQ", d)
            return (f"MPC_TARGET x={x:.3f} y={y:.3f} yaw={yaw:.4f} k={k:.4f} "
                    f"d=({dx:.3f},{dy:.3f},{dyaw:.4f}) upd={upd}")
        if can_id == ID_VEH_FEEDBACK:
            x, y, yaw, v, st, sr, cnt, rsv = struct.unpack("<6dQQ", d)
            return (f"VEH_FB x={x:.3f} y={y:.3f} yaw={yaw:.4f} v={v:.3f} "
                    f"str={st:.4f} str_ref={sr:.4f} counter={cnt}")
        return f"64B (미등록 ID)"

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
    # FD 수신 활성화 — 켜면 classic(16B)·FD(72B) 를 모두 받는다. 인터페이스가
    # classic 전용이면 실패하는데, 그건 정상이므로 조용히 classic 모드로 계속한다
    # (진단 도구라 "덜 보이는 것"보다 "안 뜨는 것"이 나쁘다).
    fd_rx = True
    try:
        s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
    except OSError:
        fd_rx = False
    print(f"listening {args.iface}  ({'classic+FD' if fd_rx else 'classic 전용'} 수신, "
          f"Ctrl-C 종료)")

    last = {}
    n = 0
    t_rate = time.monotonic()
    n_rate = 0
    try:
        while True:
            frame = s.recv(CANFD_MTU)
            # can_frame·canfd_frame 모두 앞 5바이트가 (can_id, len) 이다. 6번째
            # 바이트는 FD 에서만 flags 이고 classic 에선 패딩이라 is_fd 로 가린다.
            can_id, dlc, flags = struct.unpack("<IBB", frame[:6])
            can_id &= socket.CAN_EFF_MASK
            is_fd = len(frame) == CANFD_MTU
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
            fmt = ("FD/BRS" if (is_fd and flags & CANFD_BRS) else "FD  " if is_fd else "STD ")
            print(f"{ts}  {fmt}  0x{can_id:03X}  [{data.hex(' ')}]  "
                  f"{decode(can_id, data)}")
    except KeyboardInterrupt:
        print(f"\n총 {n}프레임")


if __name__ == "__main__":
    main()
