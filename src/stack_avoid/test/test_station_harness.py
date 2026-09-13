"""The direct harness preserves station geometry and its final E-stop gate."""
from types import MethodType, SimpleNamespace as NS

import pytest
from fma_interfaces.msg import AvoidStatus, RefPoint
from rclpy.time import Time
from stack_avoid.avoid_to_ref import AvoidToRef


def harness():
    clock = NS(now=lambda: Time(seconds=100.))
    node = NS(get_clock=lambda: clock, reference_timeout=.5, target_speed=.2,
              straight_x=2., straight_when_clear=True, last=None, active_episode=False)
    node.messages = []
    node.pub = NS(publish=node.messages.append)
    node.gate = NS(block=lambda: (False, ''), log_reason=lambda reason: None)
    for name in ('_on_avoid', '_fresh', 'tick', '_publish'):
        setattr(node, name, MethodType(getattr(AvoidToRef, name), node))
    node._rp = AvoidToRef._rp
    return node


def status(points=True):
    msg = AvoidStatus()
    msg.header.stamp = msg.reference_stamp = Time(seconds=99.9).to_msg()
    msg.scan_valid = True
    msg.v_suggest = .2
    if points:
        msg.points = [RefPoint(x=.96, y=.24, yaw=.21, curvature=.12)]
    return msg


def test_straight_tail_and_return_reference_pass_through_when_detection_clears():
    n = harness()
    n._on_avoid(status())  # detection false, path still active
    n.tick()
    p = n.messages[-1].ref_points[0]
    assert (p.x, p.y, p.yaw, p.curvature) == pytest.approx((.96, .24, .21, .12))
    assert n.messages[-1].v_ref > 0
    n.gate.block = lambda: (True, 'ESTOP')
    n.tick()
    assert n.messages[-1].v_ref == 0
    assert n.messages[-1].ref_points[0] == p


def test_stale_reference_or_missing_path_during_episode_stops_harness():
    n = harness()
    msg = status()
    msg.reference_stamp = Time(seconds=98.).to_msg()
    n._on_avoid(msg)
    n.tick()
    assert n.messages[-1].v_ref == 0
    n._on_avoid(status(points=False))
    n.tick()
    assert n.messages[-1].v_ref == 0


def test_explicit_straight_clear_requires_fresh_valid_scan_and_no_active_episode():
    n = harness()
    n.tick()
    assert n.messages[-1].v_ref == 0
    msg = status(points=False)
    n._on_avoid(msg)
    n.tick()
    assert n.messages[-1].v_ref > 0
    msg.scan_valid = False
    n.tick()
    assert n.messages[-1].v_ref == 0
