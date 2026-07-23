#!/usr/bin/env python3
"""RTCM TCP 서버 — 베이스 PC용 (3단계).

베이스 F9P의 UART2에서 나오는 RTCM3 스트림을 읽어 접속한 모든 TCP 클라이언트
(차량 PC들)에 그대로 중계한다. 인터넷 불필요 — 로컬 WiFi면 충분.

사용 (베이스 PC):
  python3 rtcm_server.py                          # /dev/ttyF9P_uart2 @38400, :2101
  python3 rtcm_server.py --tcp-port 5000
"""
import argparse
import socket
import threading
import time

import serial

STATS_INTERVAL = 10  # s


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyF9P_uart2", help="F9P UART2 포트")
    ap.add_argument("--baud", type=int, default=38400)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--tcp-port", type=int, default=2101)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.bind, args.tcp_port))
    srv.listen(8)

    clients = {}  # sock -> addr
    lock = threading.Lock()

    def accept_loop():
        while True:
            sock, addr = srv.accept()
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(2.0)
            with lock:
                clients[sock] = addr
            print(f"[server] 클라이언트 접속: {addr[0]}:{addr[1]} (총 {len(clients)})")

    threading.Thread(target=accept_loop, daemon=True).start()
    print(f"[server] {args.port} @ {args.baud} → tcp://{args.bind}:{args.tcp_port}")

    n_bytes = 0
    t_stats = time.time()
    while True:
        data = ser.read(1024)
        if data:
            n_bytes += len(data)
            with lock:
                for sock in list(clients):
                    try:
                        sock.sendall(data)
                    except OSError:
                        addr = clients.pop(sock)
                        sock.close()
                        print(f"[server] 클라이언트 끊김: {addr[0]}:{addr[1]}")

        now = time.time()
        if now - t_stats >= STATS_INTERVAL:
            rate = n_bytes / (now - t_stats)
            with lock:
                n_cli = len(clients)
            status = f"{rate:6.0f} B/s"
            if rate == 0:
                status += "  ⚠ RTCM 없음 — setup_base.py 완료 여부/포트 확인"
            print(f"[server] RTCM {status}, 클라이언트 {n_cli}")
            n_bytes, t_stats = 0, now


if __name__ == "__main__":
    main()
