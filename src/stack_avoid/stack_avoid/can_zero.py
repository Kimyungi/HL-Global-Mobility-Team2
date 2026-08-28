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
  `--once`: 지금 즉시 0 을 쓰고 종료 (수동 복구용 / 브리지 종료 직후 실행용).

★ 순서 문제 (팀장 리뷰 2026-08-10 ⑤) — 가드 모드 **단독으로는 불충분**하다.
  `ros2 launch` 는 SIGINT 를 전 프로세스에 **동시** 전달한다. 파이썬 teardown 이
  수백 ms 걸리므로, 이 가드의 0 버스트(30×10ms=0.3s)가 끝난 **뒤에** can_bridge 가
  마지막 큐를 비우며 nonzero v_ref 를 한 프레임이라도 더 실으면 dSPACE 는 그 값을
  무기한 latch 한다 — 이 가드가 막으려던 바로 그 상황이다.
  → field_session.launch.py 가 `OnProcessExit(can_bridge_node)` 로 **브리지가 완전히
    죽은 뒤** `--once` 를 한 번 더 실행한다. 가드 모드는 그 폴백으로 남긴다
    (같은 0 을 쓰므로 두 번 겹쳐도 무해하고, 이벤트가 안 걸리는 경우를 대비한다).

송신 내용 (PROTOCOL.md):
  0x100 TARGET_HEADER = counter++ · state 0 · n_points 1 · v_ref 0
  0x101 REF_POINT_0   = 전부 0   ← `--keep-steer` 면 **보내지 않는다**
  헤더가 커밋이므로 점 프레임을 보낼 때는 **점 → 헤더** 순서로 쓴다.

★ CAN FD (2026-08-28) — 와이어 포맷은 **인터페이스 MTU 로 자동 판정**한다.
  72 면 FD 프레임(BRS), 16 이면 classic. 여기에 플래그를 요구하면 안 되는 이유:
  이건 launch teardown 이 부르는 안전 가드라 인자를 한 번 빠뜨리면 dSPACE 가
  마지막 v_ref 를 그대로 물고 있는다 — 이 스크립트가 막으려던 바로 그 상황이다.
  A/B 대조가 필요하면 `--classic` 으로 강제할 수 있다.

★ 조향 처리 — 기본값은 §3 와 어긋난다. 알고 쓸 것.
  §3 는 "정지 시 조향은 직전 값 유지, 급조향 금지" 인데, 기본 동작은 REF_POINT 를
  (0,0,0,0) 으로 **덮어쓴다**. 원점 점은 chord=0 이라 quintic 이 퇴화하므로 dSPACE
  출력이 무엇이 될지 보장이 없다. `--keep-steer` 는 헤더만 보내 직전 점을 그대로
  두므로 §3 에 부합하지만, **실차에서 dSPACE 가 점 프레임 없이 헤더만 받았을 때의
  동작이 아직 미검증**이라 기본값으로 올리지 않았다. 실차 확인 후 전환할 것.
"""
import argparse
import pathlib
import signal
import socket
import struct
import sys
import time

CAN_FRAME_FMT = '<IB3x8s'        # classic can_frame (16 B)
CANFD_FRAME_FMT = '<IBB2x64s'    # canfd_frame (72 B) — can_id, len, flags, pad, data
CANFD_MTU = 72
CANFD_BRS = 0x01                 # 데이터 구간 비트레이트 전환 (dSPACE 설정과 일치)
ID_REF_POINT_0 = 0x101
ID_TARGET_HEADER = 0x100
# 0 을 몇 번 보낼지. dSPACE 가 한 프레임을 놓쳐도 확실히 받도록 여유 있게.
REPEAT = 30
PERIOD_S = 0.01


def iface_tx_packets(iface):
    """커널이 **실제로 버스에 올린** 누적 프레임 수. 못 읽으면 None.

    write() 성공과 별개다 — 상대가 ACK 하지 않으면 write 는 성공해도 이 값은 안 는다.
    2026-08-28 실측: dSPACE 가 classic 인 버스에 FD 프레임 30회 → send() 30회 성공,
    tx_packets 0, bus-errors +16. 이 가드에서는 그 침묵이 곧 **목표값이 0 이 아닌 채로
    세션이 끝나는 것**이므로 반드시 확인한다.
    """
    try:
        return int(pathlib.Path(
            f'/sys/class/net/{iface}/statistics/tx_packets').read_text())
    except (OSError, ValueError):
        return None


def iface_is_fd(iface):
    """인터페이스가 CAN FD 로 올라와 있는가 (MTU 72). 못 읽으면 False = classic."""
    try:
        return int(pathlib.Path(f'/sys/class/net/{iface}/mtu').read_text()) >= CANFD_MTU
    except (OSError, ValueError):
        return False


def _pack(can_id, data, use_fd):
    """한 프레임을 와이어 포맷으로. use_fd 면 canfd_frame(72 B, BRS)."""
    if use_fd:
        return struct.pack(CANFD_FRAME_FMT, can_id, len(data), CANFD_BRS,
                           data.ljust(64, b'\x00'))
    return struct.pack(CAN_FRAME_FMT, can_id, len(data), data)


def _send_zero(sock, counter, keep_steer, use_fd):
    """0 세트 1회 송신 → 다음 counter.

    keep_steer=False (기본): 헤더 + REF_POINT_0(전부 0) 를 보낸다.
    keep_steer=True        : **헤더만** 보낸다. dSPACE 는 직전에 latch 한 점을
        그대로 들고 있으므로 조향은 유지되고 속도만 0 이 된다 (CLAUDE.md §3
        "정지 시 조향은 직전 값 유지, 급조향 금지" 에 부합).
    """
    header = struct.pack('<HBBhH', counter & 0xFFFF, 0, 1, 0, 0)
    frames = ([(ID_TARGET_HEADER, header)] if keep_steer
              else [(ID_REF_POINT_0, bytes(8)), (ID_TARGET_HEADER, header)])
    for can_id, data in frames:
        sock.send(_pack(can_id, data, use_fd))
    return counter + 1


def zero_out(iface, keep_steer=False, force_classic=False):
    """0 을 REPEAT 회 송신. 성공하면 True."""
    use_fd = (not force_classic) and iface_is_fd(iface)
    try:
        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        s.bind((iface,))
        if use_fd:
            s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
    except OSError as e:
        print(f'✗ can_zero: {iface} 열기 실패 — {e}  ★목표값이 0 이 아닐 수 있다★',
              file=sys.stderr, flush=True)
        return False
    counter = 0
    wire_before = iface_tx_packets(iface)
    try:
        for _ in range(REPEAT):
            counter = _send_zero(s, counter, keep_steer, use_fd)
            time.sleep(PERIOD_S)
    except OSError as e:
        print(f'✗ can_zero: 송신 실패 — {e}', file=sys.stderr, flush=True)
        return False
    finally:
        s.close()
    what = 'v_ref=0 (헤더만 — 조향 유지)' if keep_steer else 'v_ref=0 · ref_point 0'
    fmt = 'CAN FD' if use_fd else 'classic CAN'

    # ★ write() 성공만으로는 0 이 dSPACE 에 닿았다고 말할 수 없다 (위 docstring).
    wire_after = iface_tx_packets(iface)
    if wire_before is not None and wire_after is not None:
        sent = wire_after - wire_before
        if sent == 0:
            print(f'✗ can_zero: {iface} 에 {REPEAT}회 write 했지만 '
                  f'**버스에 나간 프레임이 0** ({fmt}) — dSPACE 가 ACK 하지 않는다.\n'
                  f'  ★목표값이 0 이 아닐 수 있다★  와이어 포맷 불일치를 의심할 것: '
                  f'상대가 classic 이면 --classic 으로 재시도.\n'
                  f'  확인:  ip -details -statistics link show {iface}',
                  file=sys.stderr, flush=True)
            return False
        if sent < REPEAT:
            print(f'⚠ can_zero: {REPEAT}회 중 {sent}회만 버스에 나갔다 ({fmt}) — '
                  f'비트레이트·배선 확인 필요', file=sys.stderr, flush=True)

    print(f'can_zero: {iface} 에 {what} 을 {REPEAT}회 송신 완료 '
          f'({fmt}, dSPACE watchdog 없음 대응)', flush=True)
    return True


def main():
    ap = argparse.ArgumentParser(description='종료 시 dSPACE 목표값 0 복귀')
    ap.add_argument('--iface', default='can0')
    ap.add_argument('--once', action='store_true',
                    help='대기 없이 지금 0 을 쓰고 종료 (수동 복구용 / 브리지 종료 후 실행)')
    ap.add_argument('--keep-steer', action='store_true',
                    help='헤더만 보내 조향을 직전 값으로 유지 (§3 권장). '
                         '기본은 REF_POINT 도 0 으로 덮는 기존 동작 — 실차 검증 대기')
    ap.add_argument('--classic', action='store_true',
                    help='FD 인터페이스에서도 classic 프레임으로 강제 (A/B 대조용). '
                         '기본은 인터페이스 MTU 로 자동 판정')
    # ros2 launch 가 붙이는 인자를 무시하기 위해 알 수 없는 인자는 버린다.
    a, _ = ap.parse_known_args()

    if a.once:
        return 0 if zero_out(a.iface, a.keep_steer, a.classic) else 1

    stop = {'v': False}

    def on_signal(_sig, _frm):
        stop['v'] = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    print(f'can_zero: 가드 대기 중 — 종료 시 {a.iface} 목표값을 0 으로 되돌린다', flush=True)
    while not stop['v']:
        time.sleep(0.2)
    return 0 if zero_out(a.iface, a.keep_steer, a.classic) else 1


if __name__ == '__main__':
    sys.exit(main())
