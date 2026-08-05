"""HeadingFusion 단위 테스트 — ROS 없이 실행: pytest test/test_heading_fusion.py"""
import math

import pytest

from stack_gps.heading_fusion import HeadingFusion
from stack_gps.path_engine import wrap_angle


def test_before_alignment_returns_none():
    f = HeadingFusion(seed_n=1)
    assert f.heading(0.0) is None
    f.update_imu(1.0, t=0.0)
    assert f.heading(0.0) is None          # COG를 본 적 없음 → 미정렬
    assert not f.aligned


def test_first_cog_sets_offset_exactly():
    f = HeadingFusion(seed_n=1)
    f.update_imu(0.5, t=0.0)
    f.update_cog(1.2, t=0.0)
    assert f.aligned
    assert f.heading(0.0) == pytest.approx(1.2)


def test_offset_holds_at_standstill_and_tracks_imu():
    """정지(COG 없음)에서도 IMU가 도는 만큼 헤딩이 따라가야 한다."""
    f = HeadingFusion(seed_n=1)
    f.update_imu(0.0, t=0.0)
    f.update_cog(math.pi / 2, t=0.0)       # offset = +90°
    f.update_imu(0.3, t=1.0)               # 차가 0.3rad 회전
    assert f.heading(1.0) == pytest.approx(math.pi / 2 + 0.3)


def test_offset_lowpass_converges():
    f = HeadingFusion(alpha=0.5, seed_n=1)
    f.update_imu(0.0, t=0.0)
    f.update_cog(0.0, t=0.0)
    for i in range(20):                    # COG가 일관되게 +0.2 주장
        f.update_imu(0.0, t=float(i))
        f.update_cog(0.2, t=float(i))
    assert f.heading(19.0) == pytest.approx(0.2, abs=1e-3)


def test_wrap_across_pi_boundary():
    """COG 179°, IMU -179° 같은 ±π 경계에서 오프셋이 358°로 튀면 안 된다."""
    f = HeadingFusion(seed_n=1)
    f.update_imu(math.radians(-179.0), t=0.0)
    f.update_cog(math.radians(179.0), t=0.0)
    assert abs(f.offset) == pytest.approx(math.radians(2.0), abs=1e-9)
    # 경계 부근에서 계속 갱신해도 발산하지 않음
    for i in range(10):
        f.update_imu(math.radians(-179.0 + i * 0.1), t=float(i))
        f.update_cog(math.radians(179.0 + i * 0.1), t=float(i))
    h = f.heading(9.0)
    assert abs(wrap_angle(h - math.radians(179.9))) < math.radians(0.5)


def test_sign_flip():
    f = HeadingFusion(sign=-1.0, seed_n=1)
    f.update_imu(0.5, t=0.0)               # 내부적으로 -0.5로 취급
    f.update_cog(1.0, t=0.0)               # offset = 1.5
    f.update_imu(0.7, t=1.0)               # CW+ 장치가 +0.2 → 실제 -0.2
    assert f.heading(1.0) == pytest.approx(1.0 - 0.2)


def test_imu_staleness_gates_everything():
    f = HeadingFusion(imu_timeout=0.5, seed_n=1)
    f.update_imu(0.0, t=0.0)
    f.update_cog(1.0, t=1.0)               # IMU가 1초 낡음 → offset 갱신 거부
    assert not f.aligned
    f.update_cog(1.0, t=0.2)               # 신선 → 갱신
    assert f.aligned
    assert f.heading(0.2) == pytest.approx(1.0)
    assert f.heading(5.0) is None          # IMU 죽으면 융합도 죽는다


def test_arc_gate_blocks_pivot_antenna_cog():
    """제자리 선회 시 안테나(회전 중심 밖)가 원호를 그리며 COG가 '이동'으로
    보인다 — speed < R_min·|gyro_z| 면 소반경 원호로 판정하고 차단.
    (2026-08-04 진단 캡처: 선회 중 cog_ok 63/150, offset -14.7° 오염 실증)"""
    f = HeadingFusion(seed_n=1)
    f.update_imu(0.0, t=0.0, gyro_z=0.1)   # 선회율 0.1rad/s (gyro_gate는 통과)
    f.update_cog(1.0, t=0.0, speed=0.12)   # 0.12 < 3.0×0.1 → 원호 차단
    assert not f.aligned
    assert f.arc_blocked == 1
    f.update_imu(0.0, t=1.0, gyro_z=0.01)  # 직진 (반경 30m 상당)
    f.update_cog(1.0, t=1.0, speed=0.3)    # 0.3 > 3.0×0.01 → 통과
    assert f.aligned


def test_seed_ignores_brief_backward_roll():
    """출발 전 차가 뒤로 구르면 COG가 차머리 반대(180°)를 잠깐 보고한다 —
    합의 시드는 이를 무시하고 이후 전진 주행으로만 정렬해야 한다.
    (2026-08-03 저속 run 실사례: 첫 COG로 즉시 시드 → offset 180° 오염)"""
    f = HeadingFusion()          # 기본 seed_n=5, seed_width=1.0
    # 뒤로 구름: 반대 방향 표본 2개
    for t in (0.0, 0.3):
        f.update_imu(0.0, t=t)
        f.update_cog(math.pi - 0.05, t=t)
    assert not f.aligned         # 표본 부족 — 시드 금지
    # 전진 주행: 진짜 방향 0.5rad 표본이 쌓임 — 후진 표본(버퍼 12s)이
    # 만료될 때까지는 spread 게이트가 시드를 보류한다
    t = 1.0
    while t < 15.0:
        f.update_imu(0.0, t=t)
        f.update_cog(0.5, t=t)
        t += 0.3
    assert f.aligned
    assert f.heading(t) == pytest.approx(0.5, abs=1e-6)  # 오염 없이 정렬


def test_reseed_recovers_from_poisoned_alignment():
    """정렬이 어떤 이유로든 크게 틀어졌으면(유효 COG와 지속 모순),
    잔차 거부만 반복하며 잠기지 말고 스스로 재정렬해야 한다."""
    f = HeadingFusion(seed_n=1, reseed_after=5)
    f.update_imu(0.0, t=0.0)
    f.update_cog(math.pi, t=0.0)           # 오염된 정렬 (offset=π)
    assert f.aligned
    t = 1.0
    for _ in range(5):                     # 진짜 COG(0.0)가 계속 모순 → 재시드
        f.update_imu(0.0, t=t)
        f.update_cog(0.0, t=t)
        t += 0.1
    assert f.reseeds == 1
    f.update_imu(0.0, t=t)
    f.update_cog(0.0, t=t)                 # seed_n=1 — 즉시 재정렬
    assert f.heading(t) == pytest.approx(0.0)


def test_innovation_gate_rejects_reverse_cog():
    """후진 시 COG는 차머리 반대(≈180°) — offset이 끌려가면 안 된다.
    (2026-08-03 주행 말미 offset 124.7°→21.1° 오염 재발 방지)"""
    f = HeadingFusion(alpha=0.1, seed_n=1)
    f.update_imu(0.0, t=0.0)
    f.update_cog(1.0, t=0.0)               # 정렬: offset=1.0
    for i in range(1, 20):                 # 후진 주행: COG가 지속적으로 ~180° 반대
        f.update_imu(0.0, t=float(i))
        f.update_cog(1.0 + math.pi, t=float(i))
    assert f.offset == pytest.approx(1.0)  # 오염 없음
    assert f.rejected == 19
    # 정상 소잔차는 여전히 통과
    f.update_imu(0.0, t=20.0)
    f.update_cog(1.05, t=20.0)
    assert f.offset == pytest.approx(1.005)


def test_reset_alignment_requires_realignment():
    """IMU 재연결(전원 재인가) 시 offset 폐기 → 재정렬 전까지 None."""
    f = HeadingFusion(seed_n=1)
    f.update_imu(0.5, t=0.0)
    f.update_cog(1.0, t=0.0)
    assert f.heading(0.0) is not None
    f.reset_alignment()
    assert not f.aligned
    assert f.heading(0.0) is None          # 폴백(COG/접선)으로 안전
    f.update_imu(2.0, t=1.0)               # 재인가 후 새 yaw 기준
    f.update_cog(1.0, t=1.0)               # 다음 직진 COG로 재정렬
    assert f.heading(1.0) == pytest.approx(1.0)


def test_gyro_gate_blocks_update_while_turning():
    f = HeadingFusion(gyro_gate=0.15, seed_n=1)
    f.update_imu(0.0, t=0.0, gyro_z=0.5)   # 선회 중
    f.update_cog(1.0, t=0.0)
    assert not f.aligned                   # 선회 중 COG는 짝짓기 오차 → 거부
    f.update_imu(0.0, t=0.3, gyro_z=0.05)  # 직진 (선회 종료 0.3s)
    f.update_cog(1.0, t=0.3)
    assert not f.aligned                   # 진정 시간(0.5s) 내 표본 — 거부
    f.update_imu(0.0, t=0.9, gyro_z=0.05)
    f.update_cog(1.0, t=0.9)               # 진정 후 → 통과
    assert f.aligned


def test_stale_pivot_cog_not_reconsumed_at_standstill():
    """선회 직후 정지: 선회 중 찍힌 낡은 COG가 (자이로≈0인 지금 기준으론
    게이트를 다 통과해도) 진정 시간·dedupe에 걸려 offset을 못 끌고 간다.
    (2026-08-04 3차 시험 실증: 이 경로로 자세마다 offset 수십° 오염)"""
    f = HeadingFusion(seed_n=1)
    f.update_imu(0.0, t=0.0, gyro_z=0.0)
    f.update_cog(1.0, t=0.0)               # 정상 정렬 offset=1.0
    # 제자리 선회 (t=1~3), 마지막 COG는 t=2.9에 원호 접선(+90° 오염) 측정
    for i in range(20):
        f.update_imu(0.1 * i, t=1.0 + 0.1 * i, gyro_z=0.3)
    # 정지 (자이로 0) — 낡은 표본(t=2.9)이 50Hz 루프처럼 반복 소비 시도
    for k in range(50):
        f.update_imu(2.0, t=3.1 + 0.02 * k, gyro_z=0.0)
        f.update_cog(2.0 + 1.0 + math.pi / 2, t=2.9, speed=0.3)
    assert f.offset == pytest.approx(1.0)  # 오염 없음 (진정 시간 + dedupe)
