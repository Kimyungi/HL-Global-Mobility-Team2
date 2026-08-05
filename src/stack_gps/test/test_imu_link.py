"""imu_link 파서 테스트 — 2026-08-01 실기(/dev/ttyUSB_IMU) 캡처 바이트 사용."""
import math

import pytest

from stack_gps.imu_link import crc16_modbus, parse_stream

# 실기 캡처: 오일러각 프레임(0x14) 2개 + 관성 프레임(0x2C) 1개
EULER_1 = bytes.fromhex(
    "aa551423a0030095ae700effbb0b4041409540c19780436b7c")
EULER_2 = bytes.fromhex(
    "aa551423a00300a6c8700e1de70b40e5249540239a8043152d")
INERTIAL = bytes.fromhex(
    "aa552c29a00300bcbb700e203185bb2b03893bc07033bd0bd3abbd83c43bbc"
    "e36f84bfde7167becd42b2be9d4c26bf41fa")


def test_euler_frame_decodes_to_degrees_seen_on_device():
    frames, rest, crc_err = parse_stream(EULER_1)
    assert crc_err == 0 and rest == b""
    (kind, roll, pitch, yaw), = frames
    assert kind == 'euler'
    assert math.degrees(roll) == pytest.approx(2.183, abs=0.001)
    assert math.degrees(pitch) == pytest.approx(4.664, abs=0.001)
    assert math.degrees(yaw) == pytest.approx(257.19, abs=0.01)


def test_inertial_frame_gyro_and_accel():
    frames, rest, crc_err = parse_stream(INERTIAL)
    assert crc_err == 0 and rest == b""
    (kind, gx, gy, gz, ax, ay, az), = frames
    assert kind == 'inertial'
    assert abs(gz) < 0.1                      # 정지 캡처 → 선회율 ≈ 0
    assert az == pytest.approx(-1.03, abs=0.01)  # 중력 ≈ 1g


def test_stream_reassembly_across_chunks():
    """프레임이 read() 경계에서 쪼개져 들어와도 복원돼야 한다."""
    stream = EULER_1 + INERTIAL + EULER_2
    got = []
    buf = b""
    for i in range(0, len(stream), 7):        # 7바이트씩 잘라 공급
        frames, buf, crc_err = parse_stream(buf + stream[i:i + 7])
        assert crc_err == 0
        got.extend(frames)
    assert [f[0] for f in got] == ['euler', 'inertial', 'euler']


def test_corrupted_frame_resyncs_to_next():
    bad = bytearray(EULER_1)
    bad[15] ^= 0xFF                           # 페이로드 오염 → CRC 불일치
    frames, rest, crc_err = parse_stream(bytes(bad) + EULER_2)
    assert crc_err == 1
    assert len(frames) == 1                   # 뒤 프레임은 살아남는다
    assert math.degrees(frames[0][3]) == pytest.approx(257.20, abs=0.01)


def test_garbage_between_frames_is_skipped():
    frames, rest, crc_err = parse_stream(
        b"\x00\xaaU" + EULER_1 + b"\xaa\x55\x99" + EULER_2 + b"\xaa")
    assert len(frames) == 2 and rest == b"\xaa"


def test_crc_reference():
    assert crc16_modbus(EULER_1[2:-2]) == int.from_bytes(EULER_1[-2:], "little")
