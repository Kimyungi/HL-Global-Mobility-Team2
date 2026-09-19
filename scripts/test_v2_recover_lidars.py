"""Offline checks for identity-based recovery; never access a serial device."""
import importlib.util
from concurrent.futures import ThreadPoolExecutor
import fcntl
import os
from pathlib import Path
import pty
import select
import subprocess
import termios
import time

import pytest

spec = importlib.util.spec_from_file_location('recovery', Path(__file__).with_name('v2_recover_lidars.py'))
recovery = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recovery)


def packet(number):
    return recovery.INFO_HEADER + bytes([100, 1, 2, 1]) + bytes(map(int, number))


def records():
    # Deliberately unrelated tty numbers/order: position must come from serial.
    return [(Path(f'/dev/ttyUSB{i + 20}'), f'pci-0000:00:14.0-usb-0:4.{i+1}:1.0', number)
            for i, number in enumerate(reversed(recovery.POSITIONS))]


def test_device_info_is_incomplete_until_all_serial_digits_arrive():
    data = packet('2026000800070184')
    for length in range(len(data)):
        assert recovery.parse_info(data[:length]) is None
    assert recovery.parse_info(b'noise' + data) == '2026000800070184'


def test_other_response_and_invalid_digits_cannot_identify_a_lidar():
    data = packet('2026000800070184')
    assert recovery.parse_info(data[:6] + b'\x06' + data[7:]) is None
    with pytest.raises(RuntimeError, match='digits'):
        recovery.parse_info(data[:-1] + b'\xff')


def test_rules_follow_device_serial_when_tty_order_is_different():
    text = recovery.make_rules(records())
    for device, path, number in records():
        line = next(line for line in text.splitlines() if f'ID_PATH}}=="{path}"' in line)
        assert f'SYMLINK+="lidar_{recovery.POSITIONS[number]}"' in line
        assert device.name not in line
    assert 'SYMLINK+="ttyUSB_LIDAR"' in text
    assert 'SYMLINK+="ydlidar"' in text


@pytest.mark.parametrize('kind', ['missing', 'duplicate', 'unknown', 'duplicate_path', 'bad_path'])
def test_incomplete_or_ambiguous_identity_never_generates_rules(kind):
    values = records()
    device, path, number = values[0]
    if kind == 'missing':
        values.pop()
    elif kind == 'duplicate':
        values[0] = (device, path, values[1][2])
    elif kind == 'unknown':
        values[0] = (device, path, '0000000000000000')
    elif kind == 'duplicate_path':
        values[0] = (device, values[1][1], number)
    else:
        values[0] = (device, 'bad"path', number)
    with pytest.raises(RuntimeError):
        recovery.make_rules(values)


def test_topology_change_refuses_before_replacing_existing_rules(monkeypatch, tmp_path):
    rule = tmp_path / '99-fma-lidars.rules'
    rule.write_text('original')
    monkeypatch.setattr(recovery, 'RULES', rule)
    monkeypatch.setattr(recovery, 'require_stopped', lambda _: None)
    monkeypatch.setattr(recovery, 'properties', lambda _: {'ID_PATH': 'changed'})
    with pytest.raises(RuntimeError, match='topology changed'):
        recovery.apply_rules(records(), recovery.make_rules(records()))
    assert rule.read_text() == 'original'
    assert list(tmp_path.iterdir()) == [rule]


def test_running_vehicle_refuses_before_serial_query(monkeypatch):
    monkeypatch.setattr(recovery.os, 'geteuid', lambda: 0)
    monkeypatch.setattr(recovery.sys, 'argv', ['recovery', '--apply'])
    monkeypatch.setattr(recovery, 'candidates', lambda: records())
    def stopped(_):
        raise RuntimeError('Vehicle/lidar process is running')
    def query(_):
        pytest.fail('Must not touch serial ports while vehicle launch is running')
    monkeypatch.setattr(recovery, 'require_stopped', stopped)
    monkeypatch.setattr(recovery, 'read_serial', query)
    assert recovery.main() == 1


def test_tool_imports_without_user_or_system_site_packages():
    result = subprocess.run(['/usr/bin/python3', '-I', '-S', recovery.__file__, '--help'],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert '--apply' in result.stdout


@pytest.mark.parametrize('invalid', [False, True])
def test_query_only_serial_exchange_over_real_pty(invalid):
    master, slave = pty.openpty()
    try:
        with ThreadPoolExecutor(max_workers=1) as pool:
            result = pool.submit(recovery.read_serial, os.ttyname(slave), 1.0, 1)
            assert select.select([master], [], [], 1)[0]
            assert os.read(master, 256) == bytes.fromhex('a5 90')
            options = termios.tcgetattr(slave)
            assert options[4:6] == [termios.B230400, termios.B230400]
            assert not options[0] and not options[1] and not options[3]
            response = packet('2026000800070184')
            if invalid:
                response = response[:-1] + b'\xff'
            os.write(master, response[:5])
            time.sleep(0.01)
            assert not result.done()  # partial frame cannot identify the device
            os.write(master, response[5:])
            if invalid:
                with pytest.raises(RuntimeError, match='digits'):
                    result.result(timeout=2)
            else:
                assert result.result(timeout=2) == '2026000800070184'
        # Success and decode failure must both release the serial lock.
        fcntl.flock(slave, fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert not select.select([master], [], [], 0)[0]  # no scan/motor command
    finally:
        os.close(slave)
        os.close(master)


def test_no_response_times_out_and_releases_serial_lock():
    master, slave = pty.openpty()
    try:
        with pytest.raises(RuntimeError, match='no GET_DEVICE_INFO response'):
            recovery.read_serial(os.ttyname(slave), response_timeout=0.02, attempts=2)
        assert os.read(master, 256) == bytes.fromhex('a5 90 a5 90')
        fcntl.flock(slave, fcntl.LOCK_EX | fcntl.LOCK_NB)
    finally:
        os.close(slave)
        os.close(master)


def test_locked_serial_port_receives_no_commands():
    master, slave = pty.openpty()
    try:
        fcntl.flock(slave, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(BlockingIOError):
            recovery.read_serial(os.ttyname(slave))
        assert not select.select([master], [], [], 0)[0]
    finally:
        os.close(slave)
        os.close(master)
