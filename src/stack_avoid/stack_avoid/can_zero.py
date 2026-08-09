#!/usr/bin/env python3
"""세션 종료 시 dSPACE 목표값을 0 으로 되돌리는 안전 가드.  이기돈

**왜 필요한가 — dSPACE 에 watchdog 이 없다 (2026-08-09 실측).**
PC 가 송신을 멈춰도 dSPACE 는 마지막 목표값을 무기한 유지한다. CLAUDE.md §3 의
"헤더 counter 30ms 미갱신 → v_ref=0" 이 동작하지 않는다. 실제로 8/9 스윕(v_ref 0.2)
이후 PC 가 아무것도 안 보내는 상태로 dSPACE 가 `v=0.20` 을 계속 들고 있었다.
즉 **launch 를 끄는 것만으로는 차가 정지 상태가 되지 않는다.**

동작 — 두 가지 방식:
  기본(가드 모드): 뜬 채로 대기하다가 SIGINT/SIGTERM 을 받으면 0 프레임을 쓰고 종료.
      launch 가 시작한 프로세스이므로 종료 시 launch 가 이 프로세스를 기다린다.
      브리지·노드가 이미 죽은 뒤여도 **SocketCAN 에 직접** 쓰므로 영향받지 않는다.
  `--once`: 지금 즉시 0 을 쓰고 종료 (수동 복구용).

송신 내용 (PROTOCOL.md):
  0x101 REF_POINT_0 = 전부 0
  0x100 TARGET_HEADER = counter++ · state 0 · n_points 1 · v_ref 0
  헤더가 커밋이므로 **점 프레임 → 헤더** 순서로 쓴다.

★ 조향은 건드리지 않는다 — v_ref 만 0. §3 "정지 시 조향은 직전 값 유지, 급조향 금지".
"""
import argparse
import signal
import socket
import struct
import sys
import time

CAN_FRAME_FMT = '<IB3x8s'
ID_REF_POINT_0 = 0x101
ID_TARGET_HEADER = 0x100
# 0 을 몇 번 보낼지. dSPACE 가 한 프레임을 놓쳐도 확실히 받도록 여유 있게.
REPEAT = 30
PERIOD_S = 0.01


def _send_zero(sock, counter):
    """0 세트 1회 송신 → 다음 counter."""
    point = bytes(8)                                   # x·y·yaw·curvature 전부 0
    header = struct.pack('<HBBhH', counter & 0xFFFF, 0, 1, 0, 0)
    for can_id, data in ((ID_REF_POINT_0, point), (ID_TARGET_HEADER, header)):
        sock.send(struct.pack(CAN_FRAME_FMT, can_id, len(data), data))
    return counter + 1


def zero_out(iface):
    """0 을 REPEAT 회 송신. 성공하면 True."""
    try:
        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        s.bind((iface,))
    except OSError as e:
        print(f'✗ can_zero: {iface} 열기 실패 — {e}  ★목표값이 0 이 아닐 수 있다★',
              file=sys.stderr, flush=True)
        return False
    counter = 0
    try:
        for _ in range(REPEAT):
            counter = _send_zero(s, counter)
            time.sleep(PERIOD_S)
    except OSError as e:
        print(f'✗ can_zero: 송신 실패 — {e}', file=sys.stderr, flush=True)
        return False
    finally:
        s.close()
    print(f'can_zero: {iface} 에 v_ref=0 · ref_point 0 을 {REPEAT}회 송신 완료 '
          f'(dSPACE watchdog 없음 대응)', flush=True)
    return True


def main():
    ap = argparse.ArgumentParser(description='종료 시 dSPACE 목표값 0 복귀')
    ap.add_argument('--iface', default='can0')
    ap.add_argument('--once', action='store_true',
                    help='대기 없이 지금 0 을 쓰고 종료 (수동 복구용)')
    # ros2 launch 가 붙이는 인자를 무시하기 위해 알 수 없는 인자는 버린다.
    a, _ = ap.parse_known_args()

    if a.once:
        return 0 if zero_out(a.iface) else 1

    stop = {'v': False}

    def on_signal(_sig, _frm):
        stop['v'] = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    print(f'can_zero: 가드 대기 중 — 종료 시 {a.iface} 목표값을 0 으로 되돌린다', flush=True)
    while not stop['v']:
        time.sleep(0.2)
    return 0 if zero_out(a.iface) else 1


if __name__ == '__main__':
    sys.exit(main())
