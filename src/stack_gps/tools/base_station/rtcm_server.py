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
    ap.add_argument("--radio", default="",
                    help="텔레메트리 라디오 시리얼 포트 (예: /dev/ttyUSB0) — "
                         "지정 시 TCP와 동시에 무선으로도 RTCM 송출")
    ap.add_argument("--radio-baud", type=int, default=38400)
    # 무음 감지 후 시리얼 재개방 — USB 엔드포인트 스톨 복구용 (2026-08-14 실측).
    # ftdi_sio가 `urb stopped: -32`(EPIPE)로 수신 URB를 영구히 멈추는 사례가 있는데,
    # 이때 fd·DTR/RTS·장치 노드는 전부 정상으로 보여서 진단이 어렵다. 커널이 자동
    # 복구하지 않으므로 **포트를 닫았다 다시 여는 것만이** 복구 수단이다.
    # (그날 세션에서 이것 때문에 RTCM이 끊겨 run이 통째로 날아갔다)
    ap.add_argument("--reopen-after", type=float, default=5.0,
                    help="이 시간[s] 동안 1바이트도 안 들어오면 포트 재개방 (0=끔)")
    args = ap.parse_args()

    def open_serial():
        return serial.Serial(args.port, args.baud, timeout=0.2)

    ser = open_serial()
    radio = None
    if args.radio:
        radio = serial.Serial(args.radio, args.radio_baud, timeout=0.2)
        print(f"[server] 라디오 송출: {args.radio} @ {args.radio_baud}")
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
    t_last_data = time.time()
    n_reopen = 0
    while True:
        try:
            data = ser.read(1024)
        except (serial.SerialException, OSError) as e:
            print(f"[server] ⚠ 시리얼 읽기 오류: {e} — 재개방 시도")
            data = b""
            try:
                ser.close()
            except OSError:
                pass
            time.sleep(0.5)
            try:
                ser = open_serial()
                n_reopen += 1
            except (serial.SerialException, OSError) as e2:
                print(f"[server] ⚠ 재개방 실패: {e2} — 2초 후 재시도")
                time.sleep(2.0)
                continue
            t_last_data = time.time()

        # 무음 감지 → 포트 재개방 (USB 엔드포인트 스톨 복구, --reopen-after 참조)
        if args.reopen_after > 0 and not data and \
                time.time() - t_last_data >= args.reopen_after:
            print(f"[server] ⚠ {args.reopen_after:.0f}s 무음 — 포트 재개방 "
                  f"(USB 스톨 의심, 누적 {n_reopen + 1}회)")
            try:
                ser.close()
            except OSError:
                pass
            try:
                ser = open_serial()
                n_reopen += 1
            except (serial.SerialException, OSError) as e:
                print(f"[server] ⚠ 재개방 실패: {e} — 2초 후 재시도")
                time.sleep(2.0)
            t_last_data = time.time()

        if data:
            t_last_data = time.time()
            n_bytes += len(data)
            if radio is not None:
                try:
                    radio.write(data)
                except (serial.SerialException, OSError):
                    print("[server] ⚠ 라디오 포트 오류 — 3초 후 재접속 시도")
                    try:
                        radio.close()
                    except OSError:
                        pass
                    time.sleep(3)
                    try:
                        radio = serial.Serial(args.radio, args.radio_baud, timeout=0.2)
                    except (serial.SerialException, OSError):
                        pass
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
            print(f"[server] RTCM {status}, 클라이언트 {n_cli}"
                  + (f", 포트 재개방 {n_reopen}회" if n_reopen else ""))
            n_bytes, t_stats = 0, now


if __name__ == "__main__":
    main()
