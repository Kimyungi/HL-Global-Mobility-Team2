"""Real parking wrapper lifecycle with clocks/publishers mocked, no devices/ROS node.

Detector, planner, SLAM/map reset, mission and PipelineController are production
objects. Source methods are bound without running Node.__init__ or opening sensors.
"""
from collections import deque
from types import MethodType, SimpleNamespace as NS
import math

import numpy as np
import pytest
from rclpy.time import Time
from fma_interfaces.msg import ParkingCommand, GpsPath
from std_msgs.msg import String

from stack_parking.geometry import Pose2
from stack_parking.icp_slam import IcpSlam, IcpConfig
from stack_parking.localization import (
    FrontRearCloudPairer, MotionPrior, MotionPriorConfig, PipelineController,
    PipelineStage, StampedCloud,
)
from stack_parking.mission import MissionState
from stack_parking.node import StackParkingNode
from stack_parking.simulation import build_mission, synthetic_scene
from stack_parking.space_detector import MODE_PERPENDICULAR, MODE_PARALLEL, SIDE_RIGHT
from stack_parking.wall_gap_controller import PoseDeltaTracker


class Pub:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def node():
    clock = NS(now=100.)
    params = {'reset_map_on_mission_start': False, 'manual_test_publish_gps_gate': False,
              'slam_stale_timeout_s': .6, 'prior.velocity_timeout_s': .2,
              'cloud.stale_timeout_s': .35}
    n = NS(
        mission=build_mission(), slam=IcpSlam(IcpConfig()),
        prior=MotionPrior(MotionPriorConfig()), pipeline=PipelineController(),
        cloud_pairer=FrontRearCloudPairer(10), pose_delta_tracker=PoseDeltaTracker(),
        search_request_id=0, search_mission_mode=0, search_running=False,
        execution_authorized=False, search_start_s=-math.inf,
        manual_gate_active=False, gps_zone_armed=True,
        latest_vehicle=None, latest_vehicle_s=-math.inf,
        latest_rear_clearance_m=None, latest_rear_scan_s=-math.inf,
        last_icp_result=None, last_icp_accepted_s=-math.inf,
        reference_input_stamp_s=-math.inf, slam_update_times=deque(),
        latest_merged_cloud=None, _merged_mode=True,
        _p=lambda key: params[key], _clock_s=lambda: clock.now,
        get_clock=lambda: NS(now=lambda: Time(seconds=clock.now)),
        get_logger=lambda: NS(info=lambda *a: None, warn=lambda *a: None, error=lambda *a: None),
        status_pub=Pub(), stage_pub=Pub(), base_frame='base_link',
        _publish_manual_gate=lambda *a: None, _publish_paths=lambda *a: None,
        _publish_markers=lambda *a: None, _publish_diagnostics=lambda *a: None,
        _rear_clearance=lambda *a: None,
        clock=clock,
    )
    for name in ('_on_mission_command', '_start_mission', '_cancel_search',
                 '_preparation_ready', '_localization_valid', '_tick',
                 '_set_reference_stamp', '_parse_command', '_on_command', '_process_slam'):
        setattr(n, name, MethodType(getattr(StackParkingNode, name), n))
    return n


def command(n, request=101, action=ParkingCommand.PREPARE, mode=1):
    n._on_mission_command(ParkingCommand(request_id=request, action=action, mission_mode=mode))


def plan(n, mode=MODE_PERPENDICULAR):
    scene = synthetic_scene(mode, SIDE_RIGHT)
    for _ in range(n.mission.detector.config.stable_frames + 2):
        n.mission.observe_map(scene, Pose2(-1.5, 0., 0.))
    assert n.mission.plan is not None
    assert n.mission.space is not None
    # Use the real stage transitions/confirmation counts, no new ready rules.
    for _ in range(n.pipeline.slam_confirm_scans):
        n.pipeline.observe_slam(True, len(scene))
    assert n.pipeline.plan_ready()
    n.slam.initialized = True
    n.last_icp_result = NS(accepted=True)
    n.last_icp_accepted_s = n.clock.now
    n.reference_input_stamp_s = n.clock.now


def localize(n):
    for _ in range(n.pipeline.localization_confirm_scans):
        n.pipeline.observe_slam(True, 100)


def test_new_request_forces_all_resets_even_when_legacy_map_reset_disabled(node):
    command(node)
    plan(node)
    localize(node)
    node.latest_merged_cloud = object()
    node.cloud_pairer.push('front', StampedCloud(99., 100., 'base_link', np.ones((5, 2))))
    node.slam.map.add(np.ones((5, 2)))
    command(node, request=102)
    assert node.mission.state == MissionState.SCANNING
    assert node.mission.space is None and node.mission.plan is None
    assert node.mission.current_path == () and node.mission.progress == 0
    assert len(node.slam.map) == 0 and node.pipeline.stage == PipelineStage.SLAM
    assert node.latest_merged_cloud is None and node.last_icp_result is None
    assert node.reference_input_stamp_s == -math.inf and not node.execution_authorized
    assert not node._preparation_ready(node.clock.now)
    node._tick()
    msg = node.status_pub.messages[-1]
    assert msg.request_id == 102 and msg.search_active and not msg.mission_active
    assert not msg.search_space_found and not msg.preparation_ready and not msg.points


def test_duplicate_and_old_commands_do_not_reset_or_cancel_current_plan(node):
    command(node)
    plan(node)
    original = node.mission.plan
    start = node.search_start_s
    node.clock.now += .1
    command(node)
    command(node, request=100, action=ParkingCommand.CANCEL)
    command(node, request=100)
    assert node.mission.plan is original and node.search_start_s == start
    assert node.search_request_id == 101 and node.search_running


@pytest.mark.parametrize('mode,kind', [(MODE_PERPENDICULAR,1), (MODE_PARALLEL,2)])
def test_plan_waits_for_localization_then_activation_before_advancing(node, mode, kind):
    command(node, mode=kind)
    plan(node, mode)
    node._tick()
    msg = node.status_pub.messages[-1]
    assert msg.search_space_found and not msg.preparation_ready and not msg.mission_active
    assert not msg.points and msg.v_suggest == 0
    localize(node)
    assert node._preparation_ready(node.clock.now)
    last = node.mission.current_path[-1]
    node.slam.pose = Pose2(last.x, last.y, last.yaw)
    state, progress = node.mission.state, node.mission.progress
    node._tick()
    msg = node.status_pub.messages[-1]
    assert msg.preparation_ready and msg.preparation_stamp.sec == 100
    assert node.mission.state == state and node.mission.progress == progress
    assert not msg.mission_active and not msg.done and not msg.points
    command(node, request=100, action=ParkingCommand.ACTIVATE, mode=kind)
    assert not node.execution_authorized
    command(node, action=ParkingCommand.ACTIVATE, mode=kind)
    assert node.execution_authorized
    node._tick()
    assert node.status_pub.messages[-1].mission_active
    assert node.mission.state != state or node.mission.progress != progress


def test_readiness_expires_with_localization_input_even_if_heartbeat_continues(node):
    command(node)
    plan(node)
    localize(node)
    assert node._preparation_ready(node.clock.now)
    node.clock.now += .7
    node._tick()
    msg = node.status_pub.messages[-1]
    assert msg.search_space_found and not msg.preparation_ready
    assert msg.preparation_stamp.sec == 0
    command(node, action=ParkingCommand.ACTIVATE)
    assert not node.execution_authorized


def test_cancel_tombstone_prevents_delayed_prepare_and_preserves_mode(node):
    command(node, action=ParkingCommand.CANCEL, mode=2)
    command(node, mode=2)
    assert not node.search_running and node.mission.state == MissionState.IDLE
    node._tick()
    msg = node.status_pub.messages[-1]
    assert msg.request_id == 101 and msg.mission_mode == 2 and not msg.search_active
    command(node, request=102, mode=2)
    assert node.search_running
    node._on_command(String(data='cancel'))
    node._tick()
    assert node.status_pub.messages[-1].request_id == 102
    assert node.status_pub.messages[-1].mission_mode == 2
    assert not node.status_pub.messages[-1].search_active


def test_pre_request_scan_arriving_after_reset_is_not_consumed(node):
    command(node)
    node.latest_merged_cloud = StampedCloud(99.9, 100., 'base_link', np.ones((20, 2)))
    node._process_slam()
    assert len(node.slam.map) == 0 and node.last_icp_result is None
    assert node.reference_input_stamp_s == -math.inf


def test_manual_start_cannot_overwrite_mgm_preparation(node):
    command(node)
    node._on_command(String(data='start parallel auto'))
    assert node.search_request_id == 101 and node.mission.mode == MODE_PERPENDICULAR
