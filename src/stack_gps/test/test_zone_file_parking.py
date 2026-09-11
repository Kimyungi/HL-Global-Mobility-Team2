import tempfile
from pathlib import Path

from stack_gps.node import _load_zones_file


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(('info', message))

    def warn(self, message):
        self.messages.append(('warn', message))

    def error(self, message):
        self.messages.append(('error', message))


def test_zone_file_loads_t_and_parallel_parking_points():
    content = '''
stop_points: []
avoid_zones: []
gps_only_zones: []
parking_points:
- {mode: perpendicular, lat: 37.1, lon: 127.1}
- {mode: parallel, lat: 37.2, lon: 127.2}
'''
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'zones.yaml'
        path.write_text(content, encoding='utf-8')
        stops, avoid, gps_only, parking = _load_zones_file(str(path), _Logger())
    assert stops == []
    assert avoid == []
    assert gps_only == []
    assert parking == [
        (37.1, 127.1, 'perpendicular'),
        (37.2, 127.2, 'parallel'),
    ]


def test_zone_file_rejects_unknown_parking_mode():
    content = '''
parking_points:
- {mode: diagonal, lat: 37.1, lon: 127.1}
'''
    logger = _Logger()
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'zones.yaml'
        path.write_text(content, encoding='utf-8')
        *_, parking = _load_zones_file(str(path), logger)
    assert parking == []
    assert any(level == 'warn' and '주차 지점' in message
               for level, message in logger.messages)
