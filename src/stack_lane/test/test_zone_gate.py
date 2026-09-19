"""Camera polling survives zone gating without loading a model or opening devices."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from fma_interfaces.msg import GpsPath, MgmState, ZoneContext
from stack_lane.node import StackLaneNode


def lane():
    node = object.__new__(StackLaneNode)
    node._camera_only = False
    node._mgm_blocks_lane = node._gps_blocks_lane = False
    node._lane_enabled = True
    node._prev_y = 1.
    node._prev_coeffs = [1., 2., 3.]
    node._held_estimate = object()
    node._reference_stamp = object()
    node._track_status = 'held'
    node._age_frames = 9
    return node


@pytest.mark.parametrize('field,value', [
    ('in_gps_only_zone', True), ('traffic_zone_active', True),
    ('mission', 1), ('avoidance', 1), ('avoidance', 3),
])
def test_special_zone_or_active_mission_disables_and_normal_reenables(field, value):
    node = lane()
    state = MgmState()
    setattr(state, field, value)
    node._on_mgm_state(state)
    assert not node._lane_enabled
    assert node._held_estimate is None and node._reference_stamp is None
    assert node._prev_coeffs is None and node._prev_y is None
    node._on_mgm_state(MgmState())
    assert node._lane_enabled and node._track_status == 'lost'
    assert node._age_frames == 0


def test_parking_and_actual_avoidance_gate_but_consumed_marker_does_not():
    node = lane()
    state = MgmState()
    state.zones = [ZoneContext(zone_id=1, zone_type=2, in_zone=True)]
    node._on_mgm_state(state)
    assert not node._lane_enabled
    node._on_gps_zone(GpsPath(zone_valid=True, avoid_zone=True))
    node._on_mgm_state(MgmState(avoidance=1))
    assert not node._lane_enabled
    node._on_mgm_state(MgmState())
    assert node._lane_enabled


@pytest.mark.parametrize("camera_only", [False, True])
def test_disabled_inference_keeps_latest_camera_frame_and_heartbeat(monkeypatch, camera_only):
    node = lane()
    node._camera_only = camera_only
    node._on_mgm_state(MgmState(in_gps_only_zone=not camera_only))
    image = np.ones((8, 8, 3), dtype=np.uint8)
    old, latest = Mock(), Mock()
    latest.getCvFrame.return_value = image
    node._queue = Mock()
    node._queue.tryGet.side_effect = [old, latest, None]
    node._frames_dropped = node._frames_seen = 0
    node.warmup_frames = 0
    node._capture_monotonic = Mock(return_value=123.)
    node._publish_camera_status = Mock()
    node.raw_image_pub = Mock()
    node.raw_image_pub.get_subscription_count.return_value = 1
    node.debug_pub = Mock()
    node.bridge = Mock()
    from rclpy.time import Time
    node.get_clock = Mock(return_value=SimpleNamespace(now=lambda: Time(seconds=200.)))
    monkeypatch.setattr('stack_lane.node.time.monotonic', lambda: 123.4)
    node.pub = Mock()
    monkeypatch.setattr('stack_lane.node.preprocess', lambda *a: pytest.fail('inference ran outside normal zone'))
    node.tick()
    node._publish_camera_status.assert_called_once_with(123.)
    assert node.bridge.cv2_to_imgmsg.call_args.args[0] is image
    node.debug_pub.publish.assert_called_once()
    node.pub.publish.assert_not_called()
    assert node._frames_dropped == 1
    raw = node.raw_image_pub.publish.call_args.args[0]
    assert bytes(raw.data) == image.tobytes()
    assert raw.encoding == 'bgr8'
    assert raw.header.stamp.sec == 199
    assert abs(raw.header.stamp.nanosec - 600_000_000) <= 1


def test_physical_waypoint_zone_disables_lane_without_traffic_zone():
    node = lane()
    node._on_gps_zone(GpsPath(zone_valid=True, gps_only_zone=True))
    node._on_mgm_state(MgmState())
    assert not node._lane_enabled
    node._on_gps_zone(GpsPath(zone_valid=True, gps_only_zone=False))
    assert node._lane_enabled


def test_camera_only_never_reenables_lane_in_normal_zone():
    node = lane()
    node._camera_only = True
    node._on_mgm_state(MgmState())
    node._on_gps_zone(GpsPath(zone_valid=True, gps_only_zone=False))
    assert not node._lane_enabled
    assert node._held_estimate is None


def test_camera_only_initialization_skips_model_and_homography(monkeypatch):
    import rclpy
    for name in ('resolve_device', 'load_model', 'load_homography'):
        monkeypatch.setattr('stack_lane.node.' + name,
                            lambda *a: pytest.fail('camera-only mode loaded lane resources'))
    monkeypatch.setattr(StackLaneNode, '_setup_camera', lambda *a: None)
    rclpy.init(args=['--ros-args', '-p', 'camera_only:=true'])
    node = None
    try:
        node = StackLaneNode()
        assert not node._lane_enabled
        assert not hasattr(node, 'model')
        assert node.raw_image_pub.topic_name == '/perception/lane_image_raw'
        assert node.camera_pub.topic_name == '/perception/lane_camera'
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
