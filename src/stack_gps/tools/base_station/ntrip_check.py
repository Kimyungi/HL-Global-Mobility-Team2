#!/usr/bin/env python3
"""NGII NTRIP 계정만 확인한다 (F9P 연결 불필요, 보정데이터는 받지 않고 즉시 끊음).

  python3 ntrip_check.py            # ID 입력 → 비번은 화면에 안 보이게 입력
  python3 ntrip_check.py <ID>       # ID를 인자로

비번은 getpass로 받으므로 셸 히스토리에 남지 않는다.
"""
import base64
import getpass
import socket
import sys

HOST, PORT, MOUNT = "RTS1.ngii.go.kr", 2101, "VRS-RTCM31"


def try_login(user, pw):
    auth = base64.b64encode(f"{user}:{pw}".encode()).decode()
    req = (f"GET /{MOUNT} HTTP/1.0\r\n"
           f"User-Agent: NTRIP fma-check/1.0\r\n"
           f"Authorization: Basic {auth}\r\n\r\n")
    s = socket.create_connection((HOST, PORT), timeout=10)
    try:
        s.sendall(req.encode())
        s.settimeout(10)
        hdr = b""
        while b"\r\n\r\n" not in hdr and len(hdr) < 1000:
            try:
                c = s.recv(1)
            except socket.timeout:
                break
            if not c:
                break
            hdr += c
        return hdr.decode(errors="replace").splitlines()
    finally:
        s.close()


def main():
    user = sys.argv[1] if len(sys.argv) > 1 else input("NGII 아이디: ").strip()
    pw = getpass.getpass("NGII 비밀번호 (화면에 안 보임): ")
    print(f"\n{HOST}:{PORT}/{MOUNT} 로 접속 시도 (user={user}) ...\n")

    lines = try_login(user, pw)
    status = lines[0] if lines else "(응답 없음)"
    print(f"  응답: {status}")

    if "200" in status or "ICY" in status:
        print("\n✅ 인증 성공 — 이 계정이 맞다. 아래를 실행하면 된다:\n")
        print(f"    export NGII_USER={user} NGII_PASS='<방금 그 비번>'")
        print("    python3 ntrip_inject.py\n")
    elif "401" in status:
        print("\n❌ 401 — 이 ID/PW로는 실시간 보정정보 접속이 안 된다.")
        print("   RTS1까지 401이면 계정 쪽 문제다(RTS2는 원래 401이 나는 상태).")
        print("   geodesy.ngii.go.kr 마이페이지 → 통합회원 연계 에서 ID 확인.")
        print("   문의: 위치기준과 031-210-2656 (김대현)\n")
    else:
        print("\n⚠ 예상 밖 응답 — 전체 헤더:")
        for ln in lines[:8]:
            print("   ", ln)
        print()


if __name__ == "__main__":
    main()
