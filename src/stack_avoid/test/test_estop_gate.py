"""Regression tests for the direct-control E-stop fail-safe gate."""

from types import SimpleNamespace

from stack_avoid.estop_gate import EstopGate


class _Duration:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds


class _Time:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1.0e9)

    def __sub__(self, other):
        return _Duration(self.nanoseconds - other.nanoseconds)


class _Clock:
    def __init__(self):
        self.seconds = 0.0

    def now(self):
        return _Time(self.seconds)


class _Logger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass

    def error(self, _message):
        pass


class _Node:
    def __init__(self):
        self.clock = _Clock()

    def create_subscription(self, *_args):
        return object()

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return _Logger()


def test_missing_estop_heartbeat_blocks_motion():
    gate = EstopGate(_Node(), stale_s=0.25)
    assert gate.block() == (True, 'ESTOP(미수신)')


def test_fresh_clear_releases_and_stale_reblocks():
    node = _Node()
    gate = EstopGate(node, stale_s=0.25)
    gate._on_estop(SimpleNamespace(estop=False))
    assert gate.block() == (False, None)
    node.clock.seconds = 0.251
    assert gate.block() == (True, 'ESTOP(stale)')


def test_fresh_estop_blocks_until_fresh_clear():
    node = _Node()
    gate = EstopGate(node, stale_s=0.25)
    gate._on_estop(SimpleNamespace(estop=True))
    assert gate.block() == (True, 'ESTOP')
    node.clock.seconds = 0.1
    gate._on_estop(SimpleNamespace(estop=False))
    assert gate.block() == (False, None)
