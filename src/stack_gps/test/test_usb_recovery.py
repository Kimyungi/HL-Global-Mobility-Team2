"""NMEA 두절 → USB 리셋 복구 로직 (실제 장치 없이 검증).

실차에서만 재현되는 고장이라(수신기가 열거된 채 출력만 죽음) 회귀 시험이 없으면
다음에 손댈 때 조용히 깨진다. 판정 조건만 떼어 시험한다.
"""
import time

from stack_gps.gga_link import GgaLink
from stack_gps import usb_reset


class _FakeSerial:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


def _link(**kw):
    kw.setdefault('usb_reset_after_s', 1.0)
    kw.setdefault('usb_reset_cooldown_s', 10.0)
    return GgaLink(serial_port='/dev/null', log=lambda m: None, **kw)


def test_no_reset_before_timeout(monkeypatch):
    called = []
    monkeypatch.setattr(usb_reset, 'reset', lambda *a, **k: called.append(1) or True)
    link = _link(usb_reset_after_s=5.0)
    link._last_nmea_t = time.monotonic() - 1.0        # 1초 두절 (문턱 5초)
    assert link._maybe_usb_reset(_FakeSerial()) is False
    assert not called


def test_reset_after_timeout_closes_port(monkeypatch):
    called = []
    monkeypatch.setattr(usb_reset, 'reset', lambda *a, **k: called.append(1) or True)
    monkeypatch.setattr(usb_reset, 'wait_for_tty', lambda *a, **k: True)
    link = _link()
    link._last_nmea_t = time.monotonic() - 2.0        # 2초 두절 (문턱 1초)
    ser = _FakeSerial()
    assert link._maybe_usb_reset(ser) is True
    assert called, "리셋이 호출돼야 한다"
    assert ser.closed, "리셋 전에 시리얼을 닫아야 한다 (열린 핸들은 리셋 후 EIO)"
    assert link.usb_reset_count() == 1


def test_cooldown_blocks_reset_storm(monkeypatch):
    called = []
    monkeypatch.setattr(usb_reset, 'reset', lambda *a, **k: called.append(1) or True)
    monkeypatch.setattr(usb_reset, 'wait_for_tty', lambda *a, **k: True)
    link = _link()
    link._last_nmea_t = time.monotonic() - 2.0
    link._maybe_usb_reset(_FakeSerial())
    link._last_nmea_t = time.monotonic() - 2.0        # 여전히 두절
    assert link._maybe_usb_reset(_FakeSerial()) is False, "쿨다운 안에서는 다시 안 한다"
    assert len(called) == 1


def test_disabled_by_zero(monkeypatch):
    called = []
    monkeypatch.setattr(usb_reset, 'reset', lambda *a, **k: called.append(1) or True)
    link = _link(usb_reset_after_s=0.0)
    link._last_nmea_t = time.monotonic() - 100.0
    assert link._maybe_usb_reset(_FakeSerial()) is False
    assert not called


def test_first_call_only_sets_baseline(monkeypatch):
    """첫 연결 직후에는 기준점만 잡고 리셋하지 않는다 (기동 중 오작동 방지)."""
    called = []
    monkeypatch.setattr(usb_reset, 'reset', lambda *a, **k: called.append(1) or True)
    link = _link()
    assert link._last_nmea_t is None
    assert link._maybe_usb_reset(_FakeSerial()) is False
    assert link._last_nmea_t is not None
    assert not called
