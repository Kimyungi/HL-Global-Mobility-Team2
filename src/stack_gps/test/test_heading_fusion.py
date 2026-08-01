"""HeadingFusion 단위 테스트 — ROS 없이 실행: pytest test/test_heading_fusion.py"""
import math

import pytest

from stack_gps.heading_fusion import HeadingFusion
from stack_gps.path_engine import wrap_angle


def test_before_alignment_returns_none():
    f = HeadingFusion()
    assert f.heading(0.0) is None
    f.update_imu(1.0, t=0.0)
    assert f.heading(0.0) is None          # COG를 본 적 없음 → 미정렬
    assert not f.aligned


def test_first_cog_sets_offset_exactly():
    f = HeadingFusion()
    f.update_imu(0.5, t=0.0)
    f.update_cog(1.2, t=0.0)
    assert f.aligned
    assert f.heading(0.0) == pytest.approx(1.2)


def test_offset_holds_at_standstill_and_tracks_imu():
    """정지(COG 없음)에서도 IMU가 도는 만큼 헤딩이 따라가야 한다."""
    f = HeadingFusion()
    f.update_imu(0.0, t=0.0)
    f.update_cog(math.pi / 2, t=0.0)       # offset = +90°
    f.update_imu(0.3, t=1.0)               # 차가 0.3rad 회전
    assert f.heading(1.0) == pytest.approx(math.pi / 2 + 0.3)


def test_offset_lowpass_converges():
    f = HeadingFusion(alpha=0.5)
    f.update_imu(0.0, t=0.0)
    f.update_cog(0.0, t=0.0)
    for i in range(20):                    # COG가 일관되게 +0.2 주장
        f.update_imu(0.0, t=float(i))
        f.update_cog(0.2, t=float(i))
    assert f.heading(19.0) == pytest.approx(0.2, abs=1e-3)


def test_wrap_across_pi_boundary():
    """COG 179°, IMU -179° 같은 ±π 경계에서 오프셋이 358°로 튀면 안 된다."""
    f = HeadingFusion()
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
    f = HeadingFusion(sign=-1.0)
    f.update_imu(0.5, t=0.0)               # 내부적으로 -0.5로 취급
    f.update_cog(1.0, t=0.0)               # offset = 1.5
    f.update_imu(0.7, t=1.0)               # CW+ 장치가 +0.2 → 실제 -0.2
    assert f.heading(1.0) == pytest.approx(1.0 - 0.2)


def test_imu_staleness_gates_everything():
    f = HeadingFusion(imu_timeout=0.5)
    f.update_imu(0.0, t=0.0)
    f.update_cog(1.0, t=1.0)               # IMU가 1초 낡음 → offset 갱신 거부
    assert not f.aligned
    f.update_cog(1.0, t=0.2)               # 신선 → 갱신
    assert f.aligned
    assert f.heading(0.2) == pytest.approx(1.0)
    assert f.heading(5.0) is None          # IMU 죽으면 융합도 죽는다


def test_gyro_gate_blocks_update_while_turning():
    f = HeadingFusion(gyro_gate=0.15)
    f.update_imu(0.0, t=0.0, gyro_z=0.5)   # 선회 중
    f.update_cog(1.0, t=0.0)
    assert not f.aligned                   # 선회 중 COG는 짝짓기 오차 → 거부
    f.update_imu(0.0, t=0.1, gyro_z=0.05)  # 직진
    f.update_cog(1.0, t=0.1)
    assert f.aligned
