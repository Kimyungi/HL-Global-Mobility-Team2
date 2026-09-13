#!/usr/bin/env python3
"""Identify the four installed lidars and optionally repair their udev links.

Only GET_DEVICE_INFO (A5 90) is sent, using the protocol in
src/ydlidar_sdk/core/common/ydlidar_protocol.h. No scan/motor/ROS/CAN start.
Serial-to-position records: src/multi_lidar_fusion/tools/99-fma-lidars.rules.
Use after stopping the vehicle launch, with physical E-stop engaged.
Uses only the Python standard library, including in sudo's clean environment.
"""
import argparse
from array import array
from datetime import datetime
import errno
import fcntl
import os
from pathlib import Path
import re
import select
import shutil
import stat
import subprocess
import sys
import tempfile
import termios
import time

POSITIONS = {
    '2026000800070184': 'front',
    '2026000800070152': 'rear',
    '2025110300090701': 'left',
    '2025110300090660': 'right',
}
INFO_HEADER = bytes.fromhex('a5 5a 14 00 00 00 04')
RULES = Path('/etc/udev/rules.d/99-fma-lidars.rules')


def properties(device):
    result = subprocess.run(['udevadm', 'info', '-q', 'property', '-n', str(device)],
                            check=True, text=True, capture_output=True)
    return dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)


def candidates():
    devices = []
    for device in sorted(Path('/dev').glob('ttyUSB[0-9]*')):
        props = properties(device)
        if (props.get('ID_VENDOR_ID') == '10c4' and props.get('ID_MODEL_ID') == 'ea60'
                and props.get('ID_MODEL') == 'CP2102_USB_to_UART_Bridge_Controller'):
            devices.append((device, props.get('ID_PATH', '')))
    if len(devices) != 4:
        raise RuntimeError(f'Expected four CP2102 lidar devices; found {len(devices)}. No rules changed.')
    return devices


def require_stopped(devices):
    numbers = {p.stat().st_rdev for p, _ in devices}
    for process in Path('/proc').glob('[0-9]*'):
        try:
            args = process.joinpath('cmdline').read_bytes().split(b'\0')
            names = {os.fsdecode(arg).rsplit('/', 1)[-1] for arg in args[:2]}
            if names & {'mgm_node', 'can_bridge_node', 'ydlidar_ros2_driver_node'}:
                raise RuntimeError('Vehicle/lidar process is running. Stop V2 launch with Ctrl-C first.')
            for fd in process.joinpath('fd').iterdir():
                try:
                    info = fd.stat()
                    if stat.S_ISCHR(info.st_mode) and info.st_rdev in numbers:
                        raise RuntimeError(f'Lidar serial port is already open by PID {process.name}.')
                except (FileNotFoundError, ProcessLookupError):
                    continue
        except (FileNotFoundError, ProcessLookupError):
            continue


def parse_info(data):
    index = data.find(INFO_HEADER)
    if index < 0 or len(data) < index + len(INFO_HEADER) + 20:
        return None
    payload = data[index + len(INFO_HEADER):index + len(INFO_HEADER) + 20]
    digits = payload[4:20]
    if any(value > 9 for value in digits):
        raise RuntimeError('Invalid device serial digits; refusing position assignment.')
    return ''.join(str(value) for value in digits)


def read_serial(device, response_timeout=2.0, attempts=3):
    # sudo does not inherit the user's pip packages. Configure Linux 230400 8N1
    # directly, with no software/hardware flow control, using stdlib only.
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        options = termios.tcgetattr(fd)
        options[:6] = [0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL,
                       0, termios.B230400, termios.B230400]
        options[6][termios.VMIN] = 0
        options[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, options)
        try:
            # Leave DTR/RTS low, as in the original query-only configuration.
            fcntl.ioctl(fd, termios.TIOCMBIC, array('i', [termios.TIOCM_DTR | termios.TIOCM_RTS]))
        except OSError as error:
            # PTYs (offline tests) have no modem control lines.
            if error.errno != errno.ENOTTY:
                raise
        for _ in range(attempts):
            termios.tcflush(fd, termios.TCIFLUSH)
            command = bytes.fromhex('a5 90')
            write_deadline = time.monotonic() + 1.0
            while command:
                remaining = write_deadline - time.monotonic()
                if remaining <= 0 or not select.select([], [fd], [], remaining)[1]:
                    raise RuntimeError(f'{device}: GET_DEVICE_INFO write timed out.')
                try:
                    sent = os.write(fd, command)
                except BlockingIOError:
                    continue
                if sent == 0:
                    raise RuntimeError(f'{device}: could not send GET_DEVICE_INFO.')
                command = command[sent:]
            data = b''
            deadline = time.monotonic() + response_timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not select.select([fd], [], [], remaining)[0]:
                    break
                try:
                    chunk = os.read(fd, 256)
                except BlockingIOError:
                    continue
                if not chunk:
                    raise RuntimeError(f'{device}: device disconnected during identification.')
                data = (data + chunk)[-4096:]
                number = parse_info(data)
                if number is not None:
                    return number
        raise RuntimeError(f'{device}: no GET_DEVICE_INFO response. No rules changed.')
    finally:
        os.close(fd)


def make_rules(records):
    if len(records) != 4 or {number for _, _, number in records} != set(POSITIONS):
        raise RuntimeError('Missing, duplicate, or unregistered lidar serial. No rules changed.')
    paths = [path for _, path, _ in records]
    if len(set(paths)) != 4 or any(not re.fullmatch(r'[A-Za-z0-9_:.-]+', path) for path in paths):
        raise RuntimeError('Invalid or duplicate USB ID_PATH. No rules changed.')
    lines = ['# Generated after hardware GET_DEVICE_INFO verification.',
             '# Device serials identify positions; ttyUSB numbering is not a position.',
             '# Re-run scripts/v2_recover_lidars.py if the USB topology changes.']
    for device, path, number in sorted(records, key=lambda r: POSITIONS[r[2]]):
        position = POSITIONS[number]
        lines.append(f'# {position}: serial {number}, observed {device}')
        line = (f'SUBSYSTEM=="tty", ENV{{ID_PATH}}=="{path}", '
                'ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", '
                'ATTRS{product}=="CP2102 USB to UART Bridge Controller", '
                f'MODE:="0666", GROUP:="dialout", SYMLINK+="lidar_{position}"')
        if position == 'front':
            line += ', SYMLINK+="ttyUSB_LIDAR", SYMLINK+="ydlidar"'
        lines.append(line)
    return '\n'.join(lines) + '\n'


def apply_rules(records, content):
    require_stopped([(device, path) for device, path, _ in records])
    for device, path, _ in records:
        if properties(device).get('ID_PATH') != path:
            raise RuntimeError('USB topology changed during identification. Retry; no rules changed.')
    if RULES.exists():
        backup = RULES.with_name(RULES.name + '.backup-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
        shutil.copy2(RULES, backup)
        print('Backup:', backup, flush=True)
    fd, name = tempfile.mkstemp(prefix='.fma-lidars-', dir=RULES.parent)
    try:
        with os.fdopen(fd, 'w') as output:
            output.write(content)
        os.chmod(name, 0o644)
        os.replace(name, RULES)
    finally:
        if os.path.exists(name):
            os.unlink(name)
    subprocess.run(['udevadm', 'control', '--reload-rules'], check=True)
    for device, _, _ in records:
        subprocess.run(['udevadm', 'trigger', '--action=add', '--subsystem-match=tty',
                        '--sysname-match=' + device.name], check=True)
    subprocess.run(['udevadm', 'settle', '--timeout=10'], check=True)
    for device, _, number in records:
        link = Path('/dev/lidar_' + POSITIONS[number])
        if not link.exists() or not link.samefile(device) or link.stat().st_mode & 0o006 != 0o006:
            raise RuntimeError(f'{link}: link/permission verification failed. Keep vehicle launch stopped.')
        print(f'OK: {link} -> {device} ({number})', flush=True)
    print('LIDAR_LINKS_READY: four identities, links and access permissions verified. '
          'Restart the integrated launch and verify all four scan topics before go.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='Back up and install verified udev rules.')
    args = parser.parse_args()
    if os.geteuid() != 0:
        parser.exit(1, 'Run with sudo /usr/bin/python3; serial ports and udev require root on this PC.\n')
    try:
        devices = candidates()
        require_stopped(devices)
        records = []
        for device, path in devices:
            print(f'Querying {device} (GET_DEVICE_INFO, 230400 baud)...', flush=True)
            number = read_serial(device)
            print(f'{device}: serial={number}, position={POSITIONS.get(number, "UNKNOWN")}, ID_PATH={path}', flush=True)
            records.append((device, path, number))
        content = make_rules(records)
        print(content, flush=True)
        if args.apply:
            apply_rules(records, content)
        else:
            print('Preview only. Re-run with --apply to install these verified links.')
    except (OSError, termios.error, RuntimeError, subprocess.SubprocessError) as error:
        print(f'RECOVERY_STOPPED: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
