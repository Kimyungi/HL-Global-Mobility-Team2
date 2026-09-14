from fma_interfaces.msg import MgmState
from rclpy.time import Time

from test_surface_node import node, scene_scan  # noqa: F401


def state(active, stamp=99.95):
    msg = MgmState(avoidance=1 if active else 0)
    msg.header.stamp = Time(seconds=stamp).to_msg()
    return msg


def tick(node, segments):
    scan = scene_scan(segments)
    scan.header.stamp = Time(nanoseconds=max(99_900_000_000, node._path_last_scan+1_000_000)).to_msg()
    node.on_scan(scan)


def test_zone_mode_detects_obstacles_but_waits_for_manager_before_planning(node):
    node.require_mgm_active = True
    segments = [((2., -.2), (2., .2))]
    tick(node, segments)
    assert node.messages[-1].obstacle_detected
    assert not node.messages[-1].points and not node._planner.episode_active
    node._on_mgm_state(state(True))
    tick(node, segments)
    assert node.messages[-1].points and node._planner.episode_active


def test_new_activation_discards_old_goal_and_done(node):
    tick(node, [((2., -.2), (2., .2))])
    assert node._planner.goal is not None
    node._completed = True
    node.require_mgm_active = True
    node._on_mgm_state(state(True))
    assert node._planner.goal is None and not node._completed
    tick(node, [])
    assert node.messages[-1].points and not node.messages[-1].maneuver_done
    assert not node._planner.episode_active


def test_stale_state_stops_reference_without_certifying_completion(node):
    node.require_mgm_active = True
    node._on_mgm_state(state(True))
    tick(node, [((2., -.2), (2., .2))])
    goal = node._planner.goal
    node._mgm_stamp = 99_000_000_000
    tick(node, [])
    assert not node.messages[-1].points and not node.messages[-1].maneuver_done
    assert node._planner.goal == goal
    node._on_mgm_state(state(True, 99.96))
    tick(node, [])
    assert node.messages[-1].points


def test_inactive_feedback_clears_episode_and_keeps_raw_detection(node):
    node.require_mgm_active = True
    node._on_mgm_state(state(True))
    segments = [((2., -.2), (2., .2))]
    tick(node, segments)
    node._on_mgm_state(state(False, 99.96))
    tick(node, segments)
    assert node.messages[-1].obstacle_detected and not node.messages[-1].points
    assert node._planner.goal is None and not node._completed
