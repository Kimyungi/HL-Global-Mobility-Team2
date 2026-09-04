import os
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
import rclpy

from stack_traffic.node import StackTrafficNode


class FakeYolo:
    names = {9: "traffic light"}

    def __init__(self, _model_path):
        self.predict_calls = 0

    def predict(self, **_kwargs):
        self.predict_calls += 1
        return []


class FakeStopLineYolo(FakeYolo):
    names = {0: "stop_line", 1: "crosswalk", 2: "other_road_marking"}


class FakeOakCamera:
    last_kwargs = None

    def __init__(self, **_kwargs):
        type(self).last_kwargs = _kwargs
        self.mxid = _kwargs.get("mxid") or "fake-auto-mxid"
        self.usb_speed = str(_kwargs.get("usb_speed", "super")).upper()
        self.last_read_status = "starting"
        self.depth_resized = False
        self.depth_native_shape = None
        self.frame = None

    def read(self):
        if self.frame is None:
            return False, None, None
        self.last_read_status = "ok"
        return True, self.frame, None

    def release(self):
        pass


class TestNodeInitialization(unittest.TestCase):
    def test_yolo_import_failure_preserves_original_error(self):
        os.environ["ROS_LOG_DIR"] = "/tmp/stack_traffic_test_ros_logs"
        original_error = RuntimeError(
            "operator torchvision::nms does not exist"
        )
        rclpy.init()
        try:
            with (
                patch("stack_traffic.node.YOLO", None),
                patch(
                    "stack_traffic.node.YOLO_IMPORT_ERROR",
                    original_error,
                ),
                self.assertRaisesRegex(
                    RuntimeError,
                    "stack_traffic_ml_preflight.*torchvision::nms",
                ) as raised,
            ):
                StackTrafficNode()
            self.assertIs(raised.exception.__cause__, original_error)
        finally:
            rclpy.shutdown()

    def test_oak_y_only_node_initializes_without_optional_models(self):
        os.environ["ROS_LOG_DIR"] = "/tmp/stack_traffic_test_ros_logs"
        rclpy.init(
            args=[
                "--ros-args",
                "-p",
                "camera_backend:=oak",
                "-p",
                "oak_depth_enabled:=false",
                "-p",
                "oak_mxid:=traffic-oak-mxid",
                "-p",
                "oak_usb_speed:=high",
                "-p",
                "stopline_detection_enabled:=true",
                "-p",
                "stopline_stop_y_ratio:=0.90",
            ]
        )
        node = None
        try:
            with (
                patch("stack_traffic.node.YOLO", FakeYolo),
                patch("stack_traffic.node.OakRgbdCamera", FakeOakCamera),
            ):
                node = StackTrafficNode()
            self.assertFalse(node.oak_depth_enabled)
            self.assertEqual(node.oak_mxid, "traffic-oak-mxid")
            self.assertEqual(node.oak_usb_speed, "high")
            self.assertEqual(
                FakeOakCamera.last_kwargs["mxid"],
                "traffic-oak-mxid",
            )
            self.assertEqual(FakeOakCamera.last_kwargs["usb_speed"], "high")
            self.assertIn(
                "mxid=traffic-oak-mxid",
                node._camera_description(),
            )
            self.assertIn("usb_actual=HIGH", node._camera_description())
            self.assertAlmostEqual(node.stopline_stop_y_ratio, 0.90)
            self.assertEqual(node.traffic_light_class_ids, [9])
            self.assertFalse(node.camera_fault_latched)
            self.assertTrue(node.startup_hold_latched)
            # 확정 초록은 TRAFFIC 상태를 즉시 해제한다.
            self.assertTrue(node.resume_on_green)
            self.assertFalse(node.resume_on_red_clear)
            self.assertFalse(node.show_debug)

            published_stops = []
            node._publish = (
                lambda stop, _distance, **_fields: published_stops.append(stop)
            )
            node.oak_camera.frame = np.zeros(
                (720, 1280, 3),
                dtype=np.uint8,
            )
            expected_frames = max(
                node.startup_minimum_frames,
                1
                + (node.vote_window - 1)
                * node.yolo_inference_interval,
            )
            for _ in range(expected_frames):
                node.tick()

            self.assertEqual(node.frame_index, expected_frames)
            self.assertEqual(node.model.predict_calls, node.vote_window)
            self.assertTrue(all(published_stops[:-1]))
            self.assertFalse(published_stops[-1])
            self.assertFalse(node.startup_hold_latched)

            node._tick_impl = Mock(side_effect=RuntimeError("boom"))
            node._publish = Mock()
            node.tick()
            self.assertTrue(node.camera_fault_latched)
            self.assertTrue(node.stop_required_latched)
            node._publish.assert_called_once_with(True, -1.0)
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()

    def test_yolo_seg_stopline_model_is_loaded_only_when_selected(self):
        os.environ["ROS_LOG_DIR"] = "/tmp/stack_traffic_test_ros_logs"
        weights = (
            Path(__file__).parents[1]
            / "models"
            / "stopline_yolov8s_seg.pt"
        )
        rclpy.init(
            args=[
                "--ros-args",
                "-p",
                "camera_backend:=oak",
                "-p",
                "oak_depth_enabled:=false",
                "-p",
                "stopline_detection_enabled:=true",
                "-p",
                "stopline_detector_type:=yolo_seg",
                "-p",
                f"stopline_model_path:={weights}",
            ]
        )
        node = None
        traffic_model = FakeYolo("traffic")
        stopline_model = FakeStopLineYolo("stopline")
        try:
            with (
                patch(
                    "stack_traffic.node.YOLO",
                    side_effect=[traffic_model, stopline_model],
                ),
                patch("stack_traffic.node.OakRgbdCamera", FakeOakCamera),
            ):
                node = StackTrafficNode()

            self.assertIs(node.model, traffic_model)
            self.assertIs(node.stopline_model, stopline_model)
            self.assertEqual(node.stopline_class_ids, [0])
            runtime = node._process_stopline(
                np.zeros((480, 640, 3), dtype=np.uint8),
                None,
            )
            self.assertFalse(runtime.detection.detected)
            self.assertEqual(stopline_model.predict_calls, 1)
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
