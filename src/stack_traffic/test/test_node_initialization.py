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
    def test_zone_gate_keeps_camera_live_and_resets_only_perception(self):
        from std_msgs.msg import Bool
        rclpy.init(args=['--ros-args', '-p', 'camera_backend:=oak',
                        '-p', 'traffic_zone_gated:=true'])
        node = None
        try:
            with patch('stack_traffic.node.YOLO', FakeYolo), \
                 patch('stack_traffic.node.OakRgbdCamera', FakeOakCamera):
                node = StackTrafficNode()
            camera = node.oak_camera
            camera.frame = np.zeros((360, 640, 3), dtype=np.uint8)
            node.publisher = Mock()
            node.camera_pub = Mock()
            node.raw_image_pub = Mock()
            node.raw_image_pub.get_subscription_count.return_value = 1
            node.debug_image_pub = Mock()
            node.debug_image_pub.get_subscription_count.return_value = 1
            import time
            camera.last_capture_monotonic = time.monotonic() - 0.4
            warmup_calls = node.model.predict_calls
            for _ in range(3):
                node.tick()
            self.assertEqual(node.model.predict_calls, warmup_calls)
            self.assertEqual(node.camera_pub.publish.call_count, 3)
            self.assertEqual(node.raw_image_pub.publish.call_count, 3)
            raw = node.raw_image_pub.publish.call_args.args[0]
            captured_ns = raw.header.stamp.sec * 1_000_000_000 + raw.header.stamp.nanosec
            self.assertGreaterEqual(node.get_clock().now().nanoseconds - captured_ns, 400_000_000)
            self.assertEqual(raw.encoding, 'bgr8')
            self.assertEqual(bytes(raw.data), camera.frame.tobytes())
            self.assertEqual(node.debug_image_pub.publish.call_count, 3)
            node.publisher.publish.assert_not_called()
            node._publish(True, -1.)  # Fault reports also stay gated outside.
            node.publisher.publish.assert_not_called()

            node._on_traffic_zone(Bool(data=True))
            node.tick()
            self.assertGreater(node.model.predict_calls, warmup_calls)
            self.assertTrue(node.publisher.publish.called)
            node.red_history.append(1)
            node.red_phase_latched = node.stop_required_latched = True
            node.tracked_bbox = (1, 2, 3, 4)
            node._on_traffic_zone(Bool(data=True))
            self.assertTrue(node.red_phase_latched)  # Repeated enable retains votes.
            node.camera_fault_latched = True
            node._on_traffic_zone(Bool(data=False))
            self.assertFalse(node.red_history)
            self.assertFalse(node.red_phase_latched)
            self.assertFalse(node.stop_required_latched)
            self.assertIsNone(node.tracked_bbox)
            self.assertTrue(node.camera_fault_latched)
            self.assertIs(node.oak_camera, camera)
            calls = node.model.predict_calls
            node.publisher.reset_mock()
            node.tick()
            self.assertEqual(node.model.predict_calls, calls)
            node.publisher.publish.assert_not_called()
            node._on_traffic_zone(Bool(data=True))
            self.assertEqual(node.frame_index, 0)
            self.assertEqual(node.startup_yolo_runs, 0)
            self.assertTrue(node.camera_fault_latched)
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()

    def test_halla_stopline_runs_without_red(self):
        for enabled in (False, True):
            rclpy.init(args=['--ros-args', '-p', 'camera_backend:=oak',
                            '-p', 'stopline_detection_enabled:=true',
                            '-p', f'halla_stopline_test_enabled:={str(enabled).lower()}'])
            node = None
            try:
                with patch('stack_traffic.node.YOLO',
                           side_effect=[FakeYolo('traffic'), FakeStopLineYolo('stopline')]), \
                     patch('stack_traffic.node.OakRgbdCamera', FakeOakCamera):
                    node = StackTrafficNode()
                node.oak_camera.frame = np.zeros((360, 640, 3), dtype=np.uint8)
                with patch.object(node, '_process_stopline',
                                  return_value=node._empty_stopline_runtime()) as process:
                    for _ in range(6):
                        node.tick()
                    self.assertEqual(process.called, enabled)
                    self.assertFalse(node.red_phase_latched)
            finally:
                if node is not None:
                    node.destroy_node()
                rclpy.shutdown()

    def test_stopline_tracking_threshold_and_miss_reset(self):
        from types import SimpleNamespace
        rclpy.init(args=['--ros-args', '-p', 'camera_backend:=oak',
                        '-p', 'stopline_detection_enabled:=true'])
        node = None
        try:
            with patch('stack_traffic.node.YOLO',
                       side_effect=[FakeYolo('traffic'), FakeStopLineYolo('stopline')]), \
                 patch('stack_traffic.node.OakRgbdCamera', FakeOakCamera):
                node = StackTrafficNode()
            frame = np.zeros((360, 640, 3), dtype=np.uint8)
            def result(score):
                return SimpleNamespace(
                    orig_shape=(360, 640), names={0: 'stop_line'},
                    boxes=SimpleNamespace(cls=[0], conf=[score]),
                    masks=SimpleNamespace(xy=[np.asarray(
                        [[100, 250], [500, 250], [500, 270], [100, 270]], dtype=np.float32)]))
            with patch.object(node.stopline_model, 'predict') as predict:
                predict.return_value = [result(.2)]
                self.assertFalse(node._process_stopline(frame, None).detection.detected)
                self.assertEqual(predict.call_args.kwargs['conf'], .30)
                predict.return_value = [result(.30)]
                self.assertTrue(node._process_stopline(frame, None).detection.detected)
                predict.return_value = [result(.2)]
                node._process_stopline(frame, None)
                runtime = node._process_stopline(frame, None)
                self.assertTrue(runtime.stable)
                self.assertEqual(predict.call_args.kwargs['conf'], .2)
                predict.return_value = []
                for _ in range(3):
                    node._process_stopline(frame, None)
                self.assertIsNone(node.stopline_tracked_bbox)
                predict.return_value = [result(.2)]
                self.assertFalse(node._process_stopline(frame, None).detection.detected)
                self.assertEqual(predict.call_args.kwargs['conf'], .30)
                predict.return_value = [result(.4)]
                node._process_stopline(frame, None)
                node._reset_zone_history()
                self.assertIsNone(node.stopline_tracked_bbox)
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()

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

    def test_oak_y_only_node_initializes_with_stopline_yolo(self):
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
                "-p",
                "resume_on_red_absence:=true",
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
            self.assertIs(node.stopline_model, stopline_model)
            self.assertFalse(node.oak_depth_enabled)
            self.assertEqual(node.oak_mxid, "traffic-oak-mxid")
            self.assertEqual(node.oak_usb_speed, "high")
            self.assertEqual(
                FakeOakCamera.last_kwargs["mxid"],
                "traffic-oak-mxid",
            )
            self.assertEqual(FakeOakCamera.last_kwargs["usb_speed"], "high")
            self.assertEqual(FakeOakCamera.last_kwargs["exposure_compensation"], -4)
            self.assertTrue(node.describe_parameter("oak_exposure_compensation").read_only)
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
            self.assertTrue(node.resume_on_red_absence)
            self.assertFalse(node.show_debug)
            self.assertEqual(node.red_phase_yolo_inference_interval, 3)
            # 카메라를 열기 전에 두 모델을 한 번씩 준비해 첫 주행 프레임의
            # cold-start 지연을 제거한다.
            self.assertEqual(traffic_model.predict_calls, 1)
            self.assertEqual(stopline_model.predict_calls, 1)

            published_stops = []
            node._publish = (
                lambda stop, _distance, **_fields: published_stops.append(stop)
            )
            node.oak_camera.frame = np.zeros(
                (720, 1280, 3),
                dtype=np.uint8,
            )
            node.camera_pub = Mock()
            node.debug_image_pub = Mock()
            node.debug_image_pub.get_subscription_count.return_value = 1
            expected_frames = max(
                node.startup_minimum_frames,
                1
                + (node.vote_window - 1)
                * node.yolo_inference_interval,
            )
            with patch('stack_traffic.node.cv2.imshow', side_effect=AssertionError('RViz must not open OpenCV windows')):
                for _ in range(expected_frames):
                    node.tick()

            image = node.debug_image_pub.publish.call_args.args[0]
            self.assertEqual((image.width, image.height, image.encoding), (1280, 720, 'bgr8'))
            self.assertGreater(np.count_nonzero(np.frombuffer(image.data, np.uint8)), 0)
            self.assertEqual(np.count_nonzero(node.oak_camera.frame), 0)

            self.assertEqual(node.camera_pub.publish.call_count, expected_frames)
            # Initial detection hold did not suppress successful physical frame evidence.
            self.assertGreater(node.camera_pub.publish.call_args.args[0].stamp.sec, 0)
            self.assertEqual(node.frame_index, expected_frames)
            self.assertEqual(
                node.model.predict_calls,
                node.vote_window + 1,
            )
            self.assertTrue(all(published_stops[:-1]))
            self.assertFalse(published_stops[-1])
            self.assertFalse(node.startup_hold_latched)

            # A remembered red/stop clears on a valid blank camera frame in
            # the v2 policy, without a green vote or resetting the node.
            node.red_phase_latched = node.stop_required_latched = True
            node.red_history.clear()
            node.red_history.extend([1] * node.vote_window)
            node.green_history.clear()
            signals = []
            node._publish = lambda stop, distance, **fields: signals.append((stop, fields))
            for _ in range(node.vote_window):
                node.tick()
            self.assertFalse(node.red_phase_latched)
            self.assertFalse(node.stop_required_latched)
            self.assertFalse(signals[-1][0])
            self.assertFalse(signals[-1][1]['red_active'])
            self.assertFalse(signals[-1][1]['green_active'])
            expected_frames += node.vote_window

            with patch.object(node, '_read_camera', return_value=(False, None, None)), \
                 patch('stack_traffic.node.camera_poll_timed_out', return_value=True):
                node.tick()
            self.assertEqual(node.camera_pub.publish.call_count, expected_frames,
                             'camera read failure must not renew heartbeat')

            # Failure publication must not turn a remembered red into no-red.
            node.red_phase_latched = True
            node.publisher = Mock()
            StackTrafficNode._publish(node, True, -1.0)
            failed = node.publisher.publish.call_args.args[0]
            self.assertTrue(failed.red_active)
            self.assertTrue(failed.fail_safe_stop)

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

    def test_stopline_model_is_loaded_when_detection_is_enabled(self):
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
            self.assertEqual(stopline_model.predict_calls, 2)
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
