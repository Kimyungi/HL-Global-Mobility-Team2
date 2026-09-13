"""Local snapshot client; stopping a route node never stops the physical GPS link."""
import json
import os
from pathlib import Path
import socket
import time


def runtime_dir():
    return Path('/tmp') / f'fma-gps-{os.getuid()}'


def request(operation='snapshot', timeout=.2):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)
        sock.connect(str(runtime_dir() / 'link.sock'))
        sock.sendall((operation+'\n').encode())
        data = b''
        while b'\n' not in data:
            chunk = sock.recv(4096)
            if not chunk or len(data) > 65536:
                raise OSError('incomplete GPS service response')
            data += chunk
        return json.loads(data.split(b'\n', 1)[0])


class PersistentGgaLink:
    """GgaLink interface using original monotonic observation generations."""
    def __init__(self, **_kwargs):
        self._snapshot = {}
        self._read_at = -1.
        self._counters = {}

    def start(self):
        pass  # The preparation launcher owns the service, not this client.

    def stop(self):
        pass

    def _read(self):
        now = time.monotonic()
        if now-self._read_at >= .02:
            try:
                self._snapshot = request()
            except (OSError, ValueError):
                self._snapshot = {}
            self._read_at = now
        return self._snapshot

    def latest_fix(self):
        fix = self._read().get('fix')
        if not fix:
            return None
        lat, lon, height, quality, _age, observed = fix
        return lat, lon, height, quality, max(0., time.monotonic()-observed), observed

    def latest_cog(self):
        snap = self._read()
        cog = snap.get('cog')
        if not cog:
            return None
        return cog[0], cog[1], max(0., cog[2]+time.monotonic()-snap['observed_at'])

    def latest_sat_info(self):
        return tuple(self._read().get('satellites', (0, 0.)))

    def _count(self, key):
        snap = self._read()
        generation = snap.get('service_id')
        total = snap.get(key, 0)
        previous = self._counters.get(key, (None, total))
        self._counters[key] = (generation, total)
        return max(0, total-previous[1]) if previous[0] == generation else 0

    def rtcm_rate_and_reset(self):
        return self._count('rtcm_bytes')

    def nmea_rate_and_reset(self):
        return self._count('nmea_bytes')

    def usb_reset_count(self):
        return self._read().get('reset_count', 0)
