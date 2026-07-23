#!/usr/bin/env python3
"""NGII NTRIP → F9P UART2 보정 주입기 — 1단계(베이스 좌표 측량) 전용.

베이스 좌표를 확정하는 동안만 국토지리정보원 VRS 보정을 EVK-F9P에 넣어
RTK FIXED를 만들어주는 도구. 베이스 운용이 시작되면 더 이상 쓰지 않는다
(운용 중 보정 배포는 rtcm_server.py / rtcm_client_inject.py 담당).

동작:
  - UART1(/dev/ttyF9P): measure_base_position.py가 사용 (건드리지 않음)
  - UART2(/dev/ttyF9P_uart2): 여기로 NTRIP RTCM3를 기록 → 수신기가 내부 적용

사용:
  export NGII_USER=<아이디> NGII_PASS=<비밀번호>   # 또는 --user/--password
  python3 ntrip_inject.py
  python3 ntrip_inject.py --lat 37.30 --lon 127.90 # VRS 기준 위치 변경 시

계정은 팀 저장소에 커밋되지 않도록 환경변수로만 받는다.
"""
import argparse
import base64
import functools
import os
import socket
import stat
import time

import serial


def port_is_stale(ser, path):
    """USB 재열거 감지: 장치 노드가 재생성되면 같은 이름이라도 열어둔 fd는
    옛 장치를 가리켜 데이터가 허공으로 간다 (에러도 안 남 — 실제 겪음).
    경로의 rdev/inode와 fd의 것을 비교해 어긋나면 stale."""
    try:
        st_path = os.stat(path)
        st_fd = os.fstat(ser.fileno())
        return (st_path.st_rdev != st_fd.st_rdev
                or st_path.st_ino != st_fd.st_ino)
    except OSError:
        return True


def make_gga(lat, lon, alt=50.0):
    """유효 체크섬 GGA 문장 생성 (VRS에 개략 위치 보고용)."""
    latd = int(abs(lat)); latm = (abs(lat) - latd) * 60
    lond = int(abs(lon)); lonm = (abs(lon) - lond) * 60
    ns = 'N' if lat >= 0 else 'S'
    ew = 'E' if lon >= 0 else 'W'
    t = time.strftime("%H%M%S", time.gmtime())
    body = (f"GPGGA,{t}.00,{latd:02d}{latm:08.5f},{ns},"
            f"{lond:03d}{lonm:08.5f},{ew},1,12,0.8,{alt:.1f},M,20.0,M,,")
    cs = functools.reduce(lambda a, c: a ^ ord(c), body, 0)
    return f"${body}*{cs:02X}\r\n"


def connect_ntrip(args):
    auth = base64.b64encode(f"{args.user}:{args.password}".encode()).decode()
    req = (f"GET /{args.mount} HTTP/1.1\r\nHost: {args.host}:{args.ntrip_port}\r\n"
           f"Ntrip-Version: Ntrip/2.0\r\nUser-Agent: NTRIP fma-base-survey/1.0\r\n"
           f"Authorization: Basic {auth}\r\n\r\n")
    s = socket.create_connection((args.host, args.ntrip_port), timeout=10)
    s.sendall(req.encode())
    s.settimeout(10)
    hdr = b""
    while b"\r\n\r\n" not in hdr and len(hdr) < 600:
        hdr += s.recv(1)
    status = hdr.split(b"\r\n")[0].decode(errors="replace")
    if not ("200" in status or "ICY" in status):
        raise RuntimeError(f"NTRIP 인증 실패: {status}")
    s.sendall(make_gga(args.lat, args.lon).encode())
    return s, status


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyF9P_uart2", help="F9P UART2 포트")
    ap.add_argument("--baud", type=int, default=38400, help="UART2 baud")
    ap.add_argument("--host", default="RTS2.ngii.go.kr")
    ap.add_argument("--ntrip-port", type=int, default=2101)
    ap.add_argument("--mount", default="VRS-RTCM32", help="F9P는 RTCM3.x 필요")
    ap.add_argument("--user", default=os.environ.get("NGII_USER", ""),
                    help="NGII 계정 (기본: 환경변수 NGII_USER, 동시접속 1개 제한)")
    ap.add_argument("--password", default=os.environ.get("NGII_PASS", ""),
                    help="NGII 비밀번호 (기본: 환경변수 NGII_PASS)")
    ap.add_argument("--lat", type=float, default=37.3035, help="VRS용 개략 위도 (수 km 정확도면 충분)")
    ap.add_argument("--lon", type=float, default=127.9065)
    args = ap.parse_args()

    if not args.user or not args.password:
        ap.error("NGII 계정 필요 — 환경변수 NGII_USER/NGII_PASS 설정 또는 --user/--password 사용")

    def open_serial():
        s = serial.Serial(args.port, args.baud, timeout=1)
        print(f"[inject] UART2 열림: {args.port} @ {args.baud}bps")
        return s

    def connect_with_retry():
        """NGII는 동시접속 1개 — 비정상 종료 직후엔 서버에 이전 세션이 남아
        몇 분간 401이 나온다. 성공할 때까지 30초 간격 재시도."""
        while True:
            try:
                s, status = connect_ntrip(args)
                print(f"[inject] NTRIP 접속 성공: {status}")
                s.settimeout(1.0)
                return s
            except (OSError, RuntimeError) as e:
                print(f"[inject] NTRIP 접속 실패: {e} — 30초 후 재시도 "
                      "(401이면 이전 세션 정리 대기 중일 수 있음)")
                time.sleep(30)

    ser = open_serial()
    print(f"[inject] NTRIP 접속: {args.host}:{args.ntrip_port}/{args.mount} (user={args.user})")
    s = connect_with_retry()

    total = 0
    d3 = 0
    last_gga = time.time()
    last_stat = time.time()
    try:
        while True:
            try:
                # 10초마다 GGA 갱신 (VRS 세션 유지)
                if time.time() - last_gga > 10:
                    s.sendall(make_gga(args.lat, args.lon).encode())
                    last_gga = time.time()
                try:
                    data = s.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    raise ConnectionError("NTRIP 스트림 종료됨")
            except (OSError, ConnectionError) as e:
                # WiFi/LTE 순단 포함 모든 네트워크 오류에서 재접속 (원본은 여기서 죽었음)
                print(f"[inject] 네트워크 오류: {e}")
                s = connect_with_retry()
                continue

            try:
                if port_is_stale(ser, args.port):
                    raise serial.SerialException("USB 재열거 감지 (stale fd)")
                ser.write(data)      # RTCM을 F9P UART2로 직접 기록
            except (serial.SerialException, OSError):
                # USB 순단(재열거) — 포트가 되살아날 때까지 재시도 (원본은 여기서 죽었음)
                print("[inject] ⚠ 시리얼 끊김 — 3초 간격 재접속")
                try:
                    ser.close()
                except OSError:
                    pass
                while True:
                    time.sleep(3)
                    try:
                        ser = open_serial()
                        break
                    except (serial.SerialException, OSError):
                        pass
                continue
            total += len(data)
            d3 += data.count(0xD3)   # 0xD3 바이트 수 — 프레임 수의 근사치일 뿐
            if time.time() - last_stat > 5:
                print(f"[inject] 누적 {total}바이트 주입 중 (RTCM3 프레임 약 {d3}개)")
                last_stat = time.time()
    except KeyboardInterrupt:
        print("\n[inject] 종료.")
    finally:
        for closer in (s.close, ser.close):
            try:
                closer()
            except OSError:
                pass


if __name__ == "__main__":
    main()
