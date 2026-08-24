#!/usr/bin/env python3
"""dSPACE 로 TargetRef 를 송신한다 — PROTOCOL.md v2 계약 그대로.

  0x101+i  REF_POINT   x,y,yaw,κ  (i16: 1mm / 1mm / 1e-4rad / 5e-4 1/m, little-endian)
  0x100    TARGET_HEADER  counter,state,n_points,v_ref  ← 매 주기 마지막(커밋)

기본값은 v_ref 0 = 정지라 차가 움직이지 않는다. -v 를 주면 실제로 움직인다.
dSPACE 회신(0x200~0x202)이 오면 받아서 같이 보여준다.

  예)  python3 ~/FMA_ws/can_tx.py                # 1점, x=0.5m, v_ref 0, 무한
       python3 ~/FMA_ws/can_tx.py -t 10          # 10초만
       python3 ~/FMA_ws/can_tx.py --once         # 1주기만 (프레임 2장)
       python3 ~/FMA_ws/can_tx.py -n 20          # 20점 (실운용 포맷)
       python3 ~/FMA_ws/can_tx.py -v 0.3         # ⚠ 차가 움직인다
"""
import argparse, errno, socket, struct, sys, time

STATE_NAME = {0: 'lane', 1: 'waypoint', 2: 'avoid', 3: 'parking'}

p = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                            description=__doc__)
p.add_argument('-i', '--iface', default='can0',      help='CAN 인터페이스 (기본 can0)')
p.add_argument('-n', '--points', type=int, default=1, help='점 수 1~20 (기본 1 = PROTOCOL.md 계약)')
p.add_argument('-x', type=float, default=0.5,        help='목표점 x [m] (기본 0.5)')
p.add_argument('-y', type=float, default=0.0,        help='목표점 y [m] (기본 0 = 직진)')
p.add_argument('-v', '--vref', type=float, default=0.0, help='v_ref [m/s] (기본 0 = 정지). 음수 = 후진')
p.add_argument('-s', '--state', type=int, default=1, help='0=lane 1=waypoint 2=avoid 3=parking (기본 1)')
p.add_argument('-r', '--rate', type=float, default=100.0, help='송신 주파수 [Hz] (기본 100 = 10ms)')
p.add_argument('-t', '--sec', type=float, default=0.0, help='송신 시간 [s] (기본 0 = 무한, Ctrl-C 로 종료)')
p.add_argument('--once', action='store_true',        help='1주기만 보내고 끝')
p.add_argument('--yes', action='store_true',         help='v_ref≠0 경고 확인을 건너뛴다')
a = p.parse_args()

if not 1 <= a.points <= 20:
    sys.exit('점 수는 1~20 이어야 한다 (ID 0x101~0x114)')
if a.vref != 0.0 and not a.yes and sys.stdin.isatty():
    print(f"⚠ v_ref = {a.vref} m/s — 차가 실제로 움직인다. 주변 정리했나?")
    if input("  계속하려면 yes 입력: ").strip().lower() != 'yes':
        sys.exit('취소')

def q(val, lsb):
    return max(-32767, min(32767, int(round(val / lsb))))

def frame(cid, payload):
    return struct.pack('=IB3x8s', cid, 8, payload)

def open_sock(loopback_rx=False):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((a.iface,))
    return s

sock = open_sock()
rx = open_sock()
rx.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
              b''.join(struct.pack('=II', i, 0x7FF) for i in (0x200, 0x201, 0x202)))
rx.settimeout(0.0)

def send(f):
    """ENOBUFS 는 재시도, 인터페이스 소실은 재접속 (HANDOVER §3.7)."""
    global sock
    for _ in range(2000):
        try:
            sock.send(f); return 1
        except OSError as e:
            if e.errno == errno.ENOBUFS:
                time.sleep(0.0002); continue
            print(f"  ⚠ 인터페이스 오류({e.errno} {e.strerror}) — 재접속 시도", flush=True)
            try: sock.close()
            except Exception: pass
            sock = None
            while sock is None:
                time.sleep(1.0)
                try:
                    sock = open_sock(); print("  ✅ 재접속", flush=True)
                except OSError: pass
            return 0
    return 0

# 목표점을 등간격으로 나눠 n 점 생성 (n=1 이면 목표점 하나)
yaw = 0.0
pts = []
for i in range(a.points):
    t = (i + 1) / a.points
    pts.append(struct.pack('<hhhh', q(a.x * t, 1e-3), q(a.y * t, 1e-3), q(yaw, 1e-4), 0))

period = 1.0 / a.rate
print(f"[{a.iface}] {a.points}점 + 헤더 × {a.rate:g}Hz  "
      f"state={a.state}({STATE_NAME.get(a.state,'?')})  v_ref={a.vref:+.3f} m/s  "
      f"목표=({a.x:.3f}, {a.y:.3f})m")
print(f"  0x101~0x{0x100+a.points:X} REF_POINT / 0x100 TARGET_HEADER(커밋)  — Ctrl-C 로 종료", flush=True)

c = sent = rxn = 0
t0 = last = time.time()
try:
    while True:
        for i in range(a.points):
            sent += send(frame(0x101 + i, pts[i]))
        sent += send(frame(0x100, struct.pack('<HBBhH', c & 0xFFFF, a.state,
                                              a.points, q(a.vref, 1e-3), 0)))
        c += 1
        vec = {}
        while True:
            try: d = rx.recv(16)
            except BlockingIOError: break
            vec[struct.unpack('=I', d[:4])[0] & 0x7FF] = d[8:16]; rxn += 1
        if 0x202 in vec:
            x_, y_ = struct.unpack('<ff', vec.get(0x200, b'\0'*8))
            yw, vv = struct.unpack('<ff', vec.get(0x201, b'\0'*8))
            st, cn, _ = struct.unpack('<fHH', vec[0x202])
            print(f"  ◀ 회신 x={x_:+.3f} y={y_:+.3f} yaw={yw:+.4f} v={vv:+.3f} str={st:+.4f} cnt={cn}",
                  flush=True)
        if a.once:
            break
        if time.time() - last >= 5.0:
            print(f"  [{time.strftime('%H:%M:%S')}] counter={c-1} · TX {sent}프레임 · "
                  f"RX {rxn}프레임" + ("" if rxn else " ⚠ dSPACE 회신 없음"), flush=True)
            last = time.time()
        if a.sec and time.time() - t0 >= a.sec:
            break
        d = t0 + c * period - time.time()
        if d > 0: time.sleep(d)
except KeyboardInterrupt:
    print()
print(f"종료 — 주기 {c} · TX {sent}프레임 · RX {rxn}프레임 ({time.time()-t0:.2f}s)")
