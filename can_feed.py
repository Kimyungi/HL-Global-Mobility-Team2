#!/usr/bin/env python3
"""실운용과 동일한 20점 ref 를 계속 송신한다 (v_ref 0 — 차는 움직이지 않는다).

can0 이 USB 재열거로 사라져도 죽지 않고 다시 붙는다 (HANDOVER §3.7).
  사용: python3 ~/FMA_ws/can_feed.py [초]      기본 무제한
"""
import socket, struct, time, sys, errno, math

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 0   # 0 = 무제한
IFACE = 'can0'
sock = None

def open_sock():
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((IFACE,))
    return s

def q(v, lsb):
    return max(-32767, min(32767, int(round(v / lsb))))

def send(frame):
    """ENOBUFS 는 재시도, 인터페이스 소실은 재접속."""
    global sock
    for _ in range(400):
        try:
            sock.send(frame); return True
        except OSError as e:
            if e.errno == errno.ENOBUFS:
                time.sleep(0.0003); continue
            print(f"  ⚠ 인터페이스 오류({e.errno} {e.strerror}) — 재접속 시도", flush=True)
            try: sock.close()
            except Exception: pass
            sock = None
            while sock is None:
                time.sleep(1.0)
                try:
                    sock = open_sock()
                    print("  ✅ can0 재접속 성공", flush=True)
                except OSError:
                    pass
            return False
    return False

sock = open_sock()
print(f"송신 시작 — 20점 + 헤더 × 100Hz (v_ref 0). Ctrl-C 로 종료", flush=True)
cnt = sent = 0
t0 = last = time.time()
while DUR == 0 or time.time() - t0 < DUR:
    el = time.time() - t0
    tx, ty = 1.80, 0.10 + 0.02 * math.sin(el * 0.5)
    yaw = math.atan2(ty, tx)
    for i in range(20):
        t = (i + 1) / 20.0
        pl = struct.pack('<hhhh', q(tx * t, 1e-3), q(ty * t, 1e-3), q(yaw, 1e-4), 0)
        sent += send(struct.pack('=IB3x8s', 0x101 + i, 8, pl))
    sent += send(struct.pack('=IB3x8s', 0x100, 8,
                             struct.pack('<HBBhH', cnt & 0xFFFF, 1, 20, 0, 0)))
    cnt += 1
    if time.time() - last >= 10:
        print(f"  [{time.strftime('%H:%M:%S')}] 주기 {cnt} · 송신 {sent} 프레임", flush=True)
        last = time.time()
    time.sleep(0.01)
print(f"종료 — 주기 {cnt} · 송신 {sent}")
