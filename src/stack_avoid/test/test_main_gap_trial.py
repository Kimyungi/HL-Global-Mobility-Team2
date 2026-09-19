"""Regression against main's original output and v2 input-generation guards; no CAN."""
import copy
import math

import pytest
import rclpy
from sensor_msgs.msg import LaserScan
from stack_avoid.main_gap_original import StackAvoidNode
from stack_avoid.main_gap_trial import MainGapTrialNode


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(copy.deepcopy(msg))


@pytest.fixture
def nodes():
    rclpy.init(args=['--ros-args', '-p', 'lidar_mount.forward_angle_deg:=0.0',
                    '-p', 'avoid.target_rate_limit_mps:=0.0'])
    original, trial = StackAvoidNode(), MainGapTrialNode()
    original.pub, trial._output, trial.raw_pub, trial.path_pub = [Capture() for _ in range(4)]
    trial.require_active = False
    yield original, trial
    original.destroy_node()
    trial.destroy_node()
    rclpy.shutdown()


def scan(node, center=0.):
    msg = LaserScan(angle_min=-math.pi/2, angle_max=math.pi/2,
                    angle_increment=math.pi/180, range_min=.05, range_max=12.)
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.ranges = [1.9/math.cos(math.radians(i-90))
                  if abs(1.9*math.tan(math.radians(i-90))-center)<.2 else math.inf
                  for i in range(181)]
    return msg


@pytest.mark.parametrize('center', [-.35, 0., .35])
def test_original_gap_and_wire_geometry(nodes, center):
    original, trial = nodes
    msg = scan(trial, center)
    original.on_scan(msg)
    trial.on_scan(msg)
    expected, raw, output = original.pub.messages[-1], trial.raw_pub.messages[-1], trial._output.messages[-1]
    assert raw.points == expected.points
    assert raw.obstacle_detected == expected.obstacle_detected
    assert raw.avoidable == expected.avoidable
    assert len(output.points) == 1
    goal, wire = expected.points[0], output.points[0]
    assert wire.x == pytest.approx(goal.x/20.)
    assert wire.y == pytest.approx(goal.y/20.)
    assert wire.yaw == pytest.approx(math.atan2(goal.y, goal.x))
    assert wire.curvature == 0.
    assert output.reference_stamp == msg.header.stamp and output.scan_valid


def test_invalid_and_duplicate_scans_cannot_refresh_or_complete(nodes):
    _, trial = nodes
    msg = scan(trial)
    trial.on_scan(msg)
    count = len(trial._output.messages)
    trial.on_scan(msg)
    assert len(trial._output.messages) == count
    msg = scan(trial)
    msg.ranges = [math.nan]*181
    trial._clear_since = 1.
    trial.on_scan(msg)
    output = trial._output.messages[-1]
    assert not output.scan_valid and not output.points and not output.maneuver_done
    assert output.reference_stamp.sec == output.reference_stamp.nanosec == 0
    assert trial._clear_since is None


def test_inactive_zone_does_not_arm_or_publish_targets(nodes):
    _, trial = nodes
    trial.require_active = True
    trial.on_scan(scan(trial))
    output = trial._output.messages[-1]
    assert output.scan_valid and output.obstacle_detected
    assert not output.points and not trial._maneuver_armed
    assert not output.maneuver_done
