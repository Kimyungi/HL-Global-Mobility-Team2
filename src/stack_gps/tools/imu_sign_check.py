#!/usr/bin/env python3
"""IMU yaw 부호 판정 도구 — 벤치/현장에서 차를 손으로 돌려 확인 (ROS 불필요).

무엇을 판정하나:
  ① yaw 부호: 왼쪽(위에서 봐서 반시계) 회전 시 yaw가 증가하면 ENU와 동일(CCW+)
     → stack_gps 기본값 imu_yaw_sign=+1.0 그대로. 감소하면 -1.0 필요.
  ② gyro_z↔yaw 일관성: 융합의 선회 게이트(gyro_gate)는 |gyro_z|만 쓰므로
     부호가 달라도 동작하지만, 크게 어긋나면(상관 낮음) 센서 이상 신호.
  ③ 각도 스케일: 90° 돌렸는데 Δyaw가 그와 크게 다르면 자기장 교란 의심
     (실내 철제 구조물·모터 근처에서 흔함 — 실외 재시험).

사용:  python3 imu_sign_check.py            # 안내 따라 Enter → 회전
       python3 imu_sign_check.py --port /dev/ttyUSB_IMU
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from stack_gps.imu_link import ImuLink  # noqa: E402

TH = 0.06          # [rad/s] 회전 시작 감지 (~3.4°/s)
STOP_HOLD = 1.5    # [s] 이 시간 동안 조용하면 회전 종료로 판정
TIMEOUT = 30.0     # [s] 회전 시작 대기 한도


class YawTracker:
    """unwrap 누적 yaw + gyro 적분을 동시에 추적."""

    def __init__(self, link):
        self.link = link
        self.prev = None
        self.yaw = 0.0        # unwrapped [rad]
        self.gyro_int = 0.0   # gyro_z 적분 [rad]
        self._t_gyro = None

    def poll(self):
        e = self.link.latest_euler()
        g = self.link.latest_gyro_z()
        if e is not None:
            y = e[2]
            if self.prev is not None:
                self.yaw += (y - self.prev + math.pi) % (2 * math.pi) - math.pi
            self.prev = y
        if g is not None:
            now = time.monotonic()
            if self._t_gyro is not None:
                self.gyro_int += g[0] * min(now - self._t_gyro, 0.1)
            self._t_gyro = now
        return g[0] if g is not None else 0.0


def capture_rotation(tr, direction):
    input(f"\n▶ 준비되면 Enter → 차를 {direction}으로 60~90° 천천히(5초쯤) 돌리고 멈추세요: ")
    print("  회전 시작 대기 중...", flush=True)
    t0 = time.monotonic()
    while abs(tr.poll()) < TH:
        if time.monotonic() - t0 > TIMEOUT:
            print("  ✖ 30초 내 회전 미감지 — 이 단계 건너뜀")
            return None
        time.sleep(0.01)
    y0, g0 = tr.yaw, tr.gyro_int
    print("  회전 감지! 멈추면 자동 판정합니다...", flush=True)
    quiet = None
    while True:
        gz = tr.poll()
        if abs(gz) < TH:
            if quiet is None:
                quiet = time.monotonic()
            elif time.monotonic() - quiet > STOP_HOLD:
                break
        else:
            quiet = None
        time.sleep(0.01)
    dyaw = math.degrees(tr.yaw - y0)
    dgyro = math.degrees(tr.gyro_int - g0)
    print(f"  Δyaw = {dyaw:+.1f}°   gyro 적분 = {dgyro:+.1f}°")
    return dyaw, dgyro


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyUSB_IMU")
    args = ap.parse_args()

    link = ImuLink(args.port, log=lambda m: print(f"[imu] {m}"))
    link.start()
    time.sleep(1.0)
    if link.latest_euler() is None:
        sys.exit("✖ IMU 데이터 없음 — 포트·연결 확인")
    frames, crc = link.stats_and_reset()
    print(f"IMU 수신 OK ({frames:.0f}프레임/초기1초, CRC오류 {crc})")
    print("차를 위에서 내려다볼 때 기준입니다. 왼쪽 = 반시계.")

    tr = YawTracker(link)
    left = capture_rotation(tr, "왼쪽(반시계)")
    right = capture_rotation(tr, "오른쪽(시계)")
    link.stop()

    print("\n===== 판정 =====")
    checks = [(left, +1, "왼쪽"), (right, -1, "오른쪽")]
    votes = []
    for res, expect, name in checks:
        if res is None:
            continue
        dyaw, dgyro = res
        if abs(dyaw) < 15:
            print(f"{name}: Δyaw {dyaw:+.1f}° — 회전량이 작아 판정 제외 (60° 이상 돌려주세요)")
            continue
        ccw_positive = (dyaw > 0) == (expect > 0)
        votes.append(ccw_positive)
        agree = "일치" if (dyaw > 0) == (dgyro > 0) else "불일치 ⚠"
        print(f"{name}: Δyaw {dyaw:+.1f}° / gyro 적분 {dgyro:+.1f}° (부호 {agree})"
              + (f" / 스케일 비 {abs(dyaw/dgyro):.2f}" if abs(dgyro) > 5 else ""))
    if not votes:
        sys.exit("\n✖ 유효한 회전이 없어 판정 불가 — 다시 실행")
    if all(votes):
        print("\n✔ yaw는 반시계(+) — ENU와 동일. imu_yaw_sign=+1.0 (기본값 그대로)")
    elif not any(votes):
        print("\n✖ yaw는 시계(+) — ENU와 반대. 노드에 -p imu_yaw_sign:=-1.0 필요!")
    else:
        print("\n⚠ 좌우 판정이 엇갈림 — 자기장 교란 의심. 실외에서 재시험 필요")


if __name__ == "__main__":
    main()
