"""Explicitly managed GPS/RTCM service that survives stop and launch Ctrl-C.

No driving/control topics are published. Only `gps-off` shuts this service down.
FIX quality always comes from the receiver, never a latched or fabricated 4.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid

import yaml

from stack_gps.gga_link import GgaLink
from stack_gps.persistent_link import request, runtime_dir


def configuration(path, **overrides):
    values = yaml.safe_load(Path(path).read_text())
    values.update({k: v for k, v in overrides.items() if v is not None})
    return values


def ensure_running(config, relay_script):
    """Reuse the same receiver; incompatible device settings require explicit stop."""
    directory = runtime_dir()
    directory.mkdir(mode=0o700, exist_ok=True)
    if directory.stat().st_uid != os.getuid():
        raise RuntimeError('GPS runtime directory is owned by another user')
    os.chmod(directory, 0o700)
    with (directory / 'start.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try:
            current = request(timeout=1.)
        except (OSError, ValueError):
            current = None
        if current:
            if current.get('stopping'):
                raise RuntimeError('GPS service is stopping; retry preparation after it exits')
            if current.get('config') != config:
                raise RuntimeError('GPS service uses different settings; inspect gps-status and use gps-off before reconfiguring')
            return current
        config_path = directory / 'config.json'
        config_path.write_text(json.dumps(config))
        with (directory / 'service.log').open('ab') as log:
            proc = subprocess.Popen(
                [sys.executable, '-u', '-m', 'stack_gps.persistent_service', 'serve', '--config', str(config_path),
                 '--relay-script', str(relay_script)],
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True, close_fds=True)
        deadline = time.monotonic()+5.
        while time.monotonic() < deadline:
            try:
                current = request(timeout=.2)
                if current.get('config') == config:
                    return current
            except (OSError, ValueError):
                pass
            if proc.poll() is not None:
                break
            time.sleep(.05)
        raise RuntimeError(f'GPS preparation failed; see {directory / "service.log"}')


class SnapshotSource:
    def __init__(self, link, config):
        self.link, self.config = link, config
        self.service_id = uuid.uuid4().hex
        self.rtcm_bytes = self.nmea_bytes = 0

    def snapshot(self):
        self.rtcm_bytes += self.link.rtcm_rate_and_reset()
        self.nmea_bytes += self.link.nmea_rate_and_reset()
        return dict(service_id=self.service_id, pid=os.getpid(), config=self.config,
                    observed_at=time.monotonic(), fix=self.link.latest_fix(),
                    cog=self.link.latest_cog(), satellites=self.link.latest_sat_info(),
                    rtcm_bytes=self.rtcm_bytes, nmea_bytes=self.nmea_bytes,
                    reset_count=self.link.usb_reset_count())


def serve(config, relay_script):
    directory = runtime_dir()
    with (directory / 'owner.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('GPS service already owns the receiver')
        link = GgaLink(**{k: config[k] for k in (
            'serial_port', 'baud', 'rtcm_host', 'rtcm_port',
            'usb_reset_after_s', 'usb_reset_cooldown_s')})
        source = SnapshotSource(link, config)
        stop = threading.Event()
        for sig in (signal.SIGTERM, signal.SIGINT):
            signal.signal(sig, lambda *_: stop.set())
        relay, retry_at = None, 0.
        endpoint = directory / 'link.sock'
        endpoint.unlink(missing_ok=True)
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
            server.bind(str(endpoint))
            server.listen(8)
            server.settimeout(.2)
            link.start()
            try:
                while not stop.is_set():
                    if config['start_relay'] and (relay is None or relay.poll() is not None) and time.monotonic() >= retry_at:
                        relay = subprocess.Popen([
                            sys.executable, '-u', str(relay_script), '--port', config['relay_device'],
                            '--baud', str(config['relay_baud']), '--bind', '127.0.0.1',
                            '--tcp-port', str(config['rtcm_port'])])
                        retry_at = time.monotonic()+3.
                    try:
                        client, _ = server.accept()
                    except socket.timeout:
                        continue
                    with client:
                        client.settimeout(.2)
                        try:
                            op = client.recv(64).strip()
                            if op == b'stop':
                                stop.set()
                            result = source.snapshot()
                            result['stopping'] = stop.is_set()
                            client.sendall((json.dumps(result)+'\n').encode())
                        except (OSError, ValueError):
                            pass
            finally:
                link.stop()
                link._thread.join(timeout=6.)
                if relay and relay.poll() is None:
                    relay.terminate()
                    try:
                        relay.wait(timeout=3.)
                    except subprocess.TimeoutExpired:
                        relay.kill()
                        relay.wait()
                endpoint.unlink(missing_ok=True)


def status_line(current, previous=None):
    fix = current.get('fix')
    quality = fix[3] if fix and 0 <= time.monotonic()-fix[5] <= 1.5 else 0
    label = {0: 'NO FIX', 1: 'GPS', 2: 'DGPS', 4: 'FIXED', 5: 'FLOAT'}.get(quality, 'OTHER')
    rate = '--'
    if previous and previous['service_id'] == current['service_id']:
        elapsed = current['observed_at'] - previous['observed_at']
        if elapsed > 0:
            rate = f'{max(0, current["rtcm_bytes"] - previous["rtcm_bytes"]) / elapsed:.0f}'
    return (f'RTCM {rate:>6} B/s | {label} (FIX={quality}) '
            f'| 누적 {current["rtcm_bytes"]} B | GPS PID={current["pid"]}')


def monitor(current):
    """Watch receiver observations without owning or stopping the GPS service."""
    print('GPS 수신 상태를 1초마다 표시합니다. Ctrl-C: 표시만 종료, GPS 연결 유지.', flush=True)
    print('이후 scripts/v2 prepare … 실행 / GPS 완전 종료: scripts/v2 gps-off', flush=True)
    previous = None
    try:
        while True:
            print(status_line(current, previous), flush=True)
            if current.get('stopping'):
                print('GPS 서비스 종료 중.', flush=True)
                return
            previous = current
            time.sleep(1.)
            try:
                current = request(timeout=1.)
            except (OSError, ValueError):
                print('GPS 서비스 연결 끊김 — 상태 표시를 종료합니다.', flush=True)
                return
    except KeyboardInterrupt:
        print('\n상태 표시 종료. GPS/RTCM 연결은 계속 유지됩니다.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('start', 'status', 'stop', 'serve'))
    parser.add_argument('--config')
    parser.add_argument('--relay-script')
    args = parser.parse_args()
    if args.action == 'serve':
        serve(configuration(args.config), args.relay_script)
        return
    if args.action == 'start':
        current = ensure_running(configuration(args.config), args.relay_script)
        print(f'Log: {runtime_dir() / "service.log"}', flush=True)
        monitor(current)
        return
    else:
        try:
            current = request('stop' if args.action == 'stop' else 'snapshot', timeout=1.)
        except OSError:
            print('GPS service is not running')
            return
    print(f'{status_line(current)} '
          f'| {"stopping" if current.get("stopping") else "persists across drive stop/exit"}')
    print(f'Log: {runtime_dir() / "service.log"}')
    if args.action == 'stop':
        deadline = time.monotonic()+10.
        while (runtime_dir() / 'link.sock').exists() and time.monotonic() < deadline:
            time.sleep(.1)
        if (runtime_dir() / 'link.sock').exists():
            raise RuntimeError('GPS shutdown still pending; inspect service.log')


if __name__ == '__main__':
    main()
