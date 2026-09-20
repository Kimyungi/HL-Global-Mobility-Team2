"""Measured steering bridges missing IMU rotation without inventing CAN samples."""
import math
from types import SimpleNamespace as NS

import pytest

from stack_gps.heading_fusion import HeadingFusion
from stack_gps.steering_heading import SteeringHeadingRecovery
from stack_gps.path_engine import wrap_angle


def prepared(v=1., steering=-.2, heading=.4):
    recovery = SteeringHeadingRecovery()
    fusion = HeadingFusion(steering_recovery=recovery)
    recovery.observe(v, steering, 1.)
    fusion.update_imu(.7, 1., generation=1)
    assert fusion.initialize_from_waypoint(heading, 1.)
    return fusion, recovery


@pytest.mark.parametrize('v,steering', [(1., -.2), (1., .2), (-1., -.2), (0., .4)])
def test_reconnect_restores_integrated_heading_and_then_tracks_new_imu(v, steering):
    fusion, recovery = prepared(v, steering)
    for i in range(1, 301):
        recovery.observe(v, steering, 1. + i * .01)
    assert fusion.heading(4.) is None  # The model is only used to restore the IMU datum.
    expected = wrap_angle(.4 + v * math.tan(-steering) / .595 * 3.)
    fusion.update_imu(-2., 4., generation=2)
    assert fusion.heading(4.) == pytest.approx(expected)
    assert fusion.steering_recoveries == 1
    fusion.update_imu(-1.9, 4.05, generation=2)
    assert fusion.heading(4.05) == pytest.approx(wrap_angle(expected + .1))


def test_short_reenumeration_detected_even_before_imu_stale_timeout():
    fusion, recovery = prepared()
    recovery.observe(1., -.2, 1.1)
    fusion.update_imu(-2., 1.1, generation=2)
    assert fusion.steering_recoveries == 1
    assert fusion.heading(1.1) == pytest.approx(.4 + math.tan(.2) / .595 * .1)


def test_stream_resumption_without_usb_reenumeration_also_recovers():
    fusion, recovery = prepared()
    for i in range(1, 101):
        recovery.observe(1., -.2, 1. + i * .01)
    fusion.update_imu(.7, 2., generation=1)
    assert fusion.steering_recoveries == 1
    assert fusion.heading(2.) == pytest.approx(.4 + math.tan(.2) / .595)


@pytest.mark.parametrize('bad', ['gap', 'nan', 'steering', 'speed', 'clock'])
def test_bad_feedback_cannot_be_repaired_by_later_valid_samples(bad):
    fusion, recovery = prepared()
    recovery.observe(1., -.2, 1.1)
    if bad == 'gap':
        recovery.observe(1., -.2, 1.5)
    elif bad == 'nan':
        recovery.observe(float('nan'), -.2, 1.2)
    elif bad == 'steering':
        recovery.observe(1., 1.6, 1.2)
    elif bad == 'speed':
        recovery.observe(100., -.2, 1.2)
    else:
        recovery.observe(1., -.2, .9)
    for i in range(51, 101):
        recovery.observe(1., -.2, 1. + i * .01)
    fusion.update_imu(-2., 2., generation=2)
    assert fusion.heading(2.) is None
    assert fusion.steering_recovery_failures == 1
    assert not fusion.initialize_from_waypoint(0., 2.)


def test_no_feedback_or_excessively_long_outage_does_not_restore():
    fusion, recovery = prepared()
    fusion.update_imu(-2., 2., generation=2)
    assert fusion.heading(2.) is None
    fusion, recovery = prepared()
    for i in range(1, 1101):
        recovery.observe(1., -.2, 1. + i * .01)
    fusion.update_imu(-2., 12., generation=2)
    assert fusion.heading(12.) is None


def test_no_initial_heading_cannot_be_invented_by_steering():
    recovery = SteeringHeadingRecovery()
    fusion = HeadingFusion(steering_recovery=recovery)
    fusion.update_imu(0., 1., generation=1)
    for i in range(101):
        recovery.observe(1., -.2, 1. + i * .01)
    fusion.update_imu(-2., 2., generation=2)
    assert fusion.heading(2.) is None


def test_imu_timestamp_behind_latest_feedback_and_duplicate_imu_poll():
    fusion, recovery = prepared()
    for i in range(1, 102):
        recovery.observe(1., -.2, 1. + i * .01)
    fusion.update_imu(-2., 2., generation=2)
    expected = .4 + math.tan(.2) / .595
    assert fusion.heading(2.) == pytest.approx(expected)
    fusion.update_imu(-2., 2., generation=2)
    assert fusion.steering_recoveries == 1
    assert recovery.project(2.01) == pytest.approx(expected + math.tan(.2) / .595 * .01)


def test_duplicate_feedback_does_not_extend_freshness():
    fusion, recovery = prepared()
    for _ in range(100):
        recovery.observe(1., -.2, 1.)
    fusion.update_imu(-2., 2., generation=2)
    assert fusion.heading(2.) is None


def test_parking_hold_and_old_cog_do_not_override_restored_heading():
    fusion, recovery = prepared(-1.)
    fusion.cog_hold = True
    for i in range(1, 101):
        recovery.observe(-1., -.2, 1. + i * .01)
    fusion.update_imu(-2., 2., generation=2)
    expected = fusion.heading(2.)
    fusion.update_cog(math.pi, 2., speed=1.)
    assert fusion.heading(2.) == expected
    fusion.cog_hold = False
    fusion.update_cog(expected + .2, 1.9, speed=1.)
    assert fusion.heading(2.) == expected


def test_node_feedback_uses_actual_steering_not_command_and_rejects_stale(monkeypatch):
    from stack_gps.node import StackGpsNode
    monkeypatch.setattr('stack_gps.node.time.monotonic', lambda: 10.)
    recovery = SteeringHeadingRecovery()
    node = NS(steering_recovery=recovery, _vehicle_feedback_stamp=0,
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000)))
    msg = NS(header=NS(stamp=NS(sec=9, nanosec=900_000_000)),
             v=1., str=-.2, str_ref=.4, counter=0)
    StackGpsNode._on_vehicle_feedback(node, msg)
    assert recovery.samples[-1] == pytest.approx((9.9, math.tan(.2) / .595))
    StackGpsNode._on_vehicle_feedback(node, msg)
    assert len(recovery.samples) == 1
    recovery.anchor(.3, 9.9)
    msg.header.stamp.nanosec = 100_000_000
    StackGpsNode._on_vehicle_feedback(node, msg)
    assert recovery.pose is None


def test_reopened_serial_clears_old_generation_before_any_new_frame(monkeypatch):
    from stack_gps.imu_link import ImuLink
    imu = ImuLink('/unused')
    imu._gen, imu._yaw_gyro, imu._yaw_gyro_t, imu._gyro_z = 2, .4, 1., (.1, 1.)
    imu._last_ts_us = 1234
    class Port:
        in_waiting = 0

        def read(self, _):
            assert imu.generation() == 3
            assert imu.heading_sample() is None
            assert imu.latest_gyro_z() is None
            assert imu._last_ts_us is None
            imu._stop.set()
            return b''

        def close(self):
            pass

    monkeypatch.setattr('stack_gps.imu_link.serial.Serial', lambda *a, **k: Port())
    imu._run()


def test_changing_steering_and_speed_integrates_each_interval_once():
    fusion, recovery = prepared(v=1., steering=-.1, heading=3.1)
    expected = 3.1
    previous_v, previous_str = 1., -.1
    for i in range(1, 21):
        expected += previous_v * math.tan(-previous_str) / .595 * .1
        v, steering = (1., -.1) if i < 10 else (-.5, .2)
        recovery.observe(v, steering, 1. + i * .1)
        previous_v, previous_str = v, steering
    fusion.update_imu(-.5, 3., generation=2)
    assert fusion.heading(3.) == pytest.approx(wrap_angle(expected))


def test_node_reconnection_uses_atomic_sample_and_reports_success():
    from stack_gps.node import StackGpsNode
    fusion, recovery = prepared()
    for i in range(1, 101):
        recovery.observe(1., -.2, 1. + i * .01)
    logs = []
    node = NS(imu=NS(heading_sample=lambda: (-2., 2., 0., 2)), fusion=fusion,
              get_logger=lambda: NS(info=logs.append, warn=logs.append))
    StackGpsNode._update_imu_heading(node, 2.01)
    assert fusion.heading(2.01) == pytest.approx(.4 + math.tan(.2) / .595)
    assert len(logs) == 1 and '헤딩 복구:' in logs[0]
    StackGpsNode._update_imu_heading(node, 2.02)
    assert len(logs) == 1


def test_healthy_imu_heading_matches_existing_fusion_despite_steering_feedback():
    original = HeadingFusion()
    recovery = SteeringHeadingRecovery()
    enabled = HeadingFusion(steering_recovery=recovery)
    for fusion in (original, enabled):
        fusion.update_imu(.7, 1., generation=1)
        assert fusion.initialize_from_waypoint(.4, 1.)
    for i in range(1, 101):
        t = 1. + i * .1
        # Even an unrelated measured steering angle must not change healthy IMU yaw.
        recovery.observe(1., .3 * math.sin(t), t)
        for fusion in (original, enabled):
            fusion.update_imu(.7 + i * .002, t, gyro_z=.02, generation=1)
            fusion.update_cog(.4 + i * .002 + .01, t, speed=1.)
        assert enabled.heading(t) == pytest.approx(original.heading(t), abs=1e-12)
        assert enabled.offset == pytest.approx(original.offset, abs=1e-12)
    assert enabled.steering_recoveries == enabled.steering_recovery_failures == 0
