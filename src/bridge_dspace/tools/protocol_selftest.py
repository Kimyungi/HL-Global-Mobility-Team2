#!/usr/bin/env python3
"""PROTOCOL.md 왕복 셀프테스트 — ROS·CAN 하드웨어 없이 돌아간다.

can_bridge_node의 TX 패킹과 dspace_sim_node의 수신·latch·watchdog·차량 모델을
바이트 수준에서 파이썬으로 재현해 맞물려 돌린다. 검증 항목:
  1. 프레임 레이아웃/크기 (모든 페이로드 8B, ID 맵)
  2. 양자화 왕복 오차 (스케일별 ≤ 1 LSB)
  3. 커밋 순서 latch (헤더 도착 전 point는 스테이징만)
  4. 정상 주기 → 차량 속도 수렴, counter 갱신
  5. 송신 중단 → watchdog 타임아웃(30ms) → v_ref=0 감속 정지, 조향 유지

사용: python3 protocol_selftest.py   (전부 통과 시 exit 0, "ALL PASS")
"""
import math
import struct
import sys

# ── can_protocol.hpp와 1:1 (여기 바꾸면 그쪽도) ──────────────────────────
ID_TARGET_HEADER = 0x100
ID_REF_POINT_BASE = 0x101
NUM_POINTS_MAX = 20
ID_VEH_POSE = 0x200
ID_VEH_VEL = 0x201
ID_VEH_COMMIT = 0x202

POS_SCALE = 1e-3
YAW_SCALE = 1e-4
CURV_SCALE = 5e-4
VEL_SCALE = 1e-3

FMT_REF_POINT = "<hhhh"
FMT_HEADER = "<HBBhH"
FMT_POSE = "<ff"
FMT_VEL = "<ff"
FMT_COMMIT = "<fHH"


def quantize(v, scale):
    return max(-32767, min(32767, round(v / scale)))


# ── can_bridge_node.sendFrames 재현 ─────────────────────────────────────
class Bridge:
    def __init__(self):
        self.counter = 0

    def send_cycle(self, points, v_ref, state):
        """TargetRef 1회 수신 → (id, payload) 프레임 리스트 (point들 → 헤더 마지막)"""
        frames = []
        for i, (x, y, yaw, curv) in enumerate(points):
            frames.append((ID_REF_POINT_BASE + i, struct.pack(
                FMT_REF_POINT, quantize(x, POS_SCALE), quantize(y, POS_SCALE),
                quantize(yaw, YAW_SCALE), quantize(curv, CURV_SCALE))))
        self.counter = (self.counter + 1) & 0xFFFF
        frames.append((ID_TARGET_HEADER, struct.pack(
            FMT_HEADER, self.counter, state, len(points),
            quantize(v_ref, VEL_SCALE), 0)))
        return frames


# ── dspace_sim_node 재현 (rxLoop + step, 10ms 틱) ───────────────────────
class DspaceSim:
    WHEELBASE = 0.32
    TIMEOUT_MS = 30

    def __init__(self):
        self.staged = [(0, 0, 0, 0)] * NUM_POINTS_MAX
        self.latched = None          # 헤더 도착 시에만 채워짐
        self.last_counter = 0
        self.age_ms = 0              # last_rx 이후 경과
        self.v_ref = 0.0
        self.str_ref = 0.0
        self.x = self.y = self.yaw = self.v = self.str = 0.0
        self.tx_counter = 0

    def rx(self, can_id, payload):
        if ID_REF_POINT_BASE <= can_id < ID_REF_POINT_BASE + NUM_POINTS_MAX:
            self.staged[can_id - ID_REF_POINT_BASE] = struct.unpack(FMT_REF_POINT, payload)
        elif can_id == ID_TARGET_HEADER:
            counter, state, n_points, v_ref_q, _ = struct.unpack(FMT_HEADER, payload)
            if counter != self.last_counter:
                self.last_counter = counter
                self.age_ms = 0
            self.latched = (state, n_points, list(self.staged[:n_points]))
            self.v_ref = v_ref_q * VEL_SCALE
            self.str_ref = math.atan(self.WHEELBASE * self.staged[0][3] * CURV_SCALE)

    def step(self):
        """10ms 틱 → (id, payload) 회신 3프레임"""
        self.age_ms += 10
        timed_out = self.age_ms > self.TIMEOUT_MS
        v_cmd = 0.0 if timed_out else self.v_ref
        str_cmd = self.str_ref  # 조향 유지 (급조향 금지)

        dt = 0.01
        self.v += (v_cmd - self.v) * 0.2
        self.str += (str_cmd - self.str) * 0.3
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / self.WHEELBASE * math.tan(self.str) * dt

        self.tx_counter = (self.tx_counter + 1) & 0xFFFF
        return [
            (ID_VEH_POSE, struct.pack(FMT_POSE, self.x, self.y)),
            (ID_VEH_VEL, struct.pack(FMT_VEL, self.yaw, self.v)),
            (ID_VEH_COMMIT, struct.pack(FMT_COMMIT, self.str, self.tx_counter, 0)),
        ], timed_out


# ── can_bridge_node.rxLoop 재현 (커밋에서 퍼블리시) ──────────────────────
class BridgeRx:
    def __init__(self):
        self.pose = (0.0, 0.0)
        self.vel = (0.0, 0.0)
        self.published = []

    def rx(self, can_id, payload):
        if can_id == ID_VEH_POSE:
            self.pose = struct.unpack(FMT_POSE, payload)
        elif can_id == ID_VEH_VEL:
            self.vel = struct.unpack(FMT_VEL, payload)
        elif can_id == ID_VEH_COMMIT:
            s, counter, _ = struct.unpack(FMT_COMMIT, payload)
            self.published.append(
                {"x": self.pose[0], "y": self.pose[1], "yaw": self.vel[0],
                 "v": self.vel[1], "str": s, "counter": counter})


failures = []


def check(name, cond, detail=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))
    if not cond:
        failures.append(name)


def main():
    print("1. 프레임 레이아웃")
    for fmt in (FMT_REF_POINT, FMT_HEADER, FMT_POSE, FMT_VEL, FMT_COMMIT):
        check(f"payload {fmt} = 8B", struct.calcsize(fmt) == 8, f"{struct.calcsize(fmt)}B")
    check("REF_POINT ID 상한 = 0x114", ID_REF_POINT_BASE + NUM_POINTS_MAX - 1 == 0x114)

    print("2. 양자화 왕복")
    for val, scale, name in ((1.2345, POS_SCALE, "pos"), (-2.9, YAW_SCALE, "yaw"),
                             (2.86, CURV_SCALE, "curv"), (0.333, VEL_SCALE, "vel")):
        err = abs(quantize(val, scale) * scale - val)
        check(f"{name} 오차 ≤ 1 LSB", err <= scale, f"err={err:.2e}")
    check("클램프 +", quantize(100.0, POS_SCALE) == 32767)
    check("클램프 -", quantize(-100.0, POS_SCALE) == -32767)

    print("3. 커밋 latch 순서")
    sim = DspaceSim()
    pt = struct.pack(FMT_REF_POINT, 500, 0, 0, quantize(1.0, CURV_SCALE))
    sim.rx(ID_REF_POINT_BASE, pt)
    check("헤더 전: latch 없음", sim.latched is None and sim.v_ref == 0.0)
    sim.rx(ID_TARGET_HEADER, struct.pack(FMT_HEADER, 1, 0, 1, quantize(0.3, VEL_SCALE), 0))
    check("헤더 후: latch 완료", sim.latched is not None and abs(sim.v_ref - 0.3) < VEL_SCALE)
    check("조향 목표 = atan(L·k)", abs(sim.str_ref - math.atan(0.32 * 1.0)) < 1e-3)

    print("4. 정상 주기 왕복 (lane 1점, v_ref 0.3, 100틱 = 1s)")
    bridge, sim, brx = Bridge(), DspaceSim(), BridgeRx()
    points = [(0.5, 0.0, 0.0, 0.0)]
    for _ in range(100):
        for fid, payload in bridge.send_cycle(points, 0.3, state=0):
            sim.rx(fid, payload)
        replies, timed_out = sim.step()
        check_no_timeout = not timed_out
        for fid, payload in replies:
            brx.rx(fid, payload)
        if not check_no_timeout:
            break
    check("타임아웃 미발생", check_no_timeout)
    check("퍼블리시 100회 (커밋마다 1회)", len(brx.published) == 100,
          f"{len(brx.published)}")
    last = brx.published[-1]
    check("v 수렴 → 0.3", abs(last["v"] - 0.3) < 0.01, f"v={last['v']:.3f}")
    check("x 전진", last["x"] > 0.2, f"x={last['x']:.3f}")
    check("counter 단조 증가", last["counter"] == 100)

    print("5. watchdog (송신 중단)")
    stopped_v = None
    for i in range(60):  # 600ms 송신 없음
        replies, timed_out = sim.step()
        for fid, payload in replies:
            brx.rx(fid, payload)
        if i == 3:
            check("30ms 초과 시 타임아웃", timed_out)
    check("감속 정지 v→0", abs(brx.published[-1]["v"]) < 0.01,
          f"v={brx.published[-1]['v']:.4f}")
    check("조향 유지 (급조향 금지)", abs(brx.published[-1]["str"] - sim.str_ref) < 1e-6)
    print()
    if failures:
        print(f"FAIL: {len(failures)}건 — {failures}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
