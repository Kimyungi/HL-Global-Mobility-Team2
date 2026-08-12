#!/usr/bin/env python3
"""raw CAN → candump 포맷 로그 파일.  이기돈

can-utils 의 `candump -l` 이 이 PC 에 없어서 같은 포맷으로 직접 남긴다.
GPS 팀 bag(`~/gps_bags/*/candump-*.log`)과 **동일한 형식**이라 같은 파서로 읽힌다:

    (1786011840.126547) can0 101#3C161CFE96FC1400

왜 필요한가 — rosbag 은 "우리가 무엇을 publish 했나"까지만 남긴다. 버스에 실제로
몇 프레임이 나갔는지, 드롭이 있었는지는 여기서만 보인다. ref 20점 송신은 주기당
2 → 21 프레임으로 늘어(부하 7%→32%) 이 확인이 특히 중요하다.

사용:
    python3 can_log.py --out /home/xanadu/avoid_logs/세션/can.log
    python3 can_log.py --iface can0 --out ... --duration 600
    python3 can_log.py --selftest          # 하드웨어 없이 포맷만 검증

Ctrl-C 로 종료하면 요약(ID별 프레임 수·주기당 평균)을 출력한다.
"""
import argparse
import collections
import os
import re
import socket
import struct
import sys
import time

CAN_FRAME_FMT = '<IB3x8s'
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
CAN_EFF_FLAG = 0x80000000
CAN_SFF_MASK = 0x000007FF
# candump -l 한 줄 형식 — 이 정규식으로 되읽을 수 있어야 한다.
LINE_RE = re.compile(r'^\((\d+\.\d+)\)\s+(\S+)\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)$')


def fmt_line(ts, iface, can_id, data):
    """한 프레임 → candump 한 줄."""
    return f'({ts:.6f}) {iface} {can_id:03X}#{data.hex().upper()}'


def selftest():
    """하드웨어 없이 포맷 왕복(생성→파싱)만 검증."""
    line = fmt_line(1786011840.126547, 'can0', 0x101,
                    bytes([0x3C, 0x16, 0x1C, 0xFE, 0x96, 0xFC, 0x14, 0x00]))
    m = LINE_RE.match(line)
    ok = (bool(m) and m.group(2) == 'can0' and int(m.group(3), 16) == 0x101
          and len(bytes.fromhex(m.group(4))) == 8
          and abs(float(m.group(1)) - 1786011840.126547) < 1e-6)
    print(line)
    print('selftest: ' + ('OK — GPS bag 과 같은 형식으로 되읽힌다' if ok else '✗ 형식 불일치'))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description='raw CAN → candump 포맷 로그')
    ap.add_argument('--iface', default='can0')
    ap.add_argument('--out', help='출력 파일 경로 (없으면 stdout)')
    ap.add_argument('--duration', type=float, default=0.0, help='초. 0 = 무한 (Ctrl-C 종료)')
    ap.add_argument('--selftest', action='store_true', help='하드웨어 없이 포맷만 검증')
    a = ap.parse_args()

    if a.selftest:
        return selftest()

    try:
        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        s.bind((a.iface,))
    except OSError as e:
        print(f'✗ {a.iface} 열기 실패: {e}\n'
              f'  can0 이 올라와 있는지 확인:  ip link show {a.iface}\n'
              f'  올리는 명령(sudo 필요):  '
              f'sudo ip link set {a.iface} up type can bitrate 1000000 restart-ms 100',
              file=sys.stderr)
        return 1
    s.settimeout(1.0)

    out = open(a.out, 'w', buffering=1) if a.out else sys.stdout
    if a.out:
        print(f'{a.iface} → {a.out}  (Ctrl-C 로 종료)')
    counts = collections.Counter()
    t0 = time.time()
    try:
        while True:
            if a.duration and time.time() - t0 >= a.duration:
                break
            try:
                frame = s.recv(CAN_FRAME_SIZE)
            except socket.timeout:
                continue
            can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, frame)
            if can_id & CAN_EFF_FLAG:      # 이 프로토콜은 표준 ID(11bit)만 쓴다
                continue
            can_id &= CAN_SFF_MASK
            counts[can_id] += 1
            out.write(fmt_line(time.time(), a.iface, can_id, data[:dlc]) + '\n')
    except KeyboardInterrupt:
        pass
    finally:
        if a.out:
            out.close()
        dt = max(time.time() - t0, 1e-6)
        total = sum(counts.values())
        print(f'\n{dt:.1f}s, 총 {total} 프레임 ({total/dt:.0f} f/s)')
        if counts:
            print(f'{"ID":>6} {"프레임":>8} {"주기당":>7}')
            for cid in sorted(counts):
                # 10ms 주기 기준 주기당 프레임 수
                print(f'  0x{cid:03X} {counts[cid]:>8} {counts[cid]/(dt*100):>7.2f}')
            tx = sum(v for k, v in counts.items() if 0x100 <= k <= 0x114)
            hdr = counts.get(0x100, 0)
            if hdr:
                print(f'  TX 프레임/헤더 = {tx/hdr:.2f}  '
                      f'(= 1 + n_points. 20점이면 21.00 이어야 한다)')
        if a.out and os.path.exists(a.out):
            print(f'저장: {a.out} ({os.path.getsize(a.out)/1e6:.1f} MB)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
