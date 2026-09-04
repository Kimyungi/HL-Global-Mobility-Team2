"""Pytest registration for the seven hardware-free launch safety checks."""

from launch import LaunchContext
from launch.actions import Shutdown
from launch_ros.actions import Node

from stack_avoid.launch_parts import can_bridge_with_zero_guard, safety_node


def _context(**values):
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


def _nodes(actions):
    return [action for action in actions if isinstance(action, Node)]


def _parameters(node):
    result = {}
    for key, value in node._Node__parameters[0].items():
        name = ''.join(getattr(token, 'text', '') for token in key)
        result[name] = value
    return result


def test_estop_release_distance_is_derived():
    actions = safety_node(_context(
        estop_on_distance_m='0.70', estop_off_distance_m='', dynamic='true'))
    assert len(_nodes(actions)) == 1
    assert _parameters(_nodes(actions)[0])['estop_off_distance_m'] == 0.80


def test_nondefault_estop_on_still_derives_valid_release():
    actions = safety_node(_context(
        estop_on_distance_m='0.90', estop_off_distance_m='', dynamic='true'))
    assert _parameters(_nodes(actions)[0])['estop_off_distance_m'] == 1.0


def test_inverted_estop_hysteresis_stops_launch():
    actions = safety_node(_context(
        estop_on_distance_m='0.90', estop_off_distance_m='0.80',
        dynamic='true'))
    assert any(isinstance(action, Shutdown) for action in actions)
    assert not _nodes(actions)


def test_nonpositive_estop_distance_stops_launch():
    actions = safety_node(_context(
        estop_on_distance_m='0', estop_off_distance_m='', dynamic='true'))
    assert any(isinstance(action, Shutdown) for action in actions)
    assert not _nodes(actions)


def test_estop_node_exit_stops_session():
    node = _nodes(safety_node(_context(
        estop_on_distance_m='0.70', estop_off_distance_m='',
        dynamic='true')))[0]
    on_exit = next(
        (getattr(node, name) for name in dir(node) if name.endswith('__on_exit')),
        None,
    )
    assert isinstance(on_exit, Shutdown) or (
        isinstance(on_exit, (list, tuple))
        and any(isinstance(action, Shutdown) for action in on_exit)
    )


def test_can_guard_contains_bridge_zero_handler_and_fallback():
    actions = can_bridge_with_zero_guard()
    assert [type(action).__name__ for action in actions] == [
        'Node', 'RegisterEventHandler', 'Node']


def test_can_bridge_respawns_after_failure():
    bridge = can_bridge_with_zero_guard()[0]
    respawn = next(
        (getattr(bridge, name) for name in dir(bridge)
         if name.endswith('__respawn')),
        None,
    )
    delay = next(
        (getattr(bridge, name) for name in dir(bridge)
         if name.endswith('__respawn_delay')),
        None,
    )
    assert bool(respawn)
    assert float(delay) == 1.0
