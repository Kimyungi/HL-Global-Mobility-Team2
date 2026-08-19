from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

import numpy as np

from stack_traffic.oak_camera import (
    OakRgbdCamera,
    _build_oak_v3_pipeline,
    _open_oak_v2_session,
    _open_oak_v3_session,
    build_oak_pipeline,
    dai,
    depthai_api_major,
    estimate_oak_host_bandwidth_bytes_per_sec,
    normalize_oak_usb_speed,
    open_oak_device,
    validate_depthai_version,
    validate_oak_usb_bandwidth,
)


class FakeQueue:
    def __init__(self, message):
        self.message = message

    def tryGet(self):
        return self.message


class FailingQueue:
    def tryGet(self):
        raise RuntimeError("X_LINK_ERROR")


class FakeRgbMessage:
    def __init__(self, frame):
        self.frame = frame

    def getCvFrame(self):
        return self.frame


class FakeDepthMessage:
    def __init__(self, depth):
        self.depth = depth

    def getFrame(self):
        return self.depth


def make_camera(message):
    camera = OakRgbdCamera.__new__(OakRgbdCamera)
    camera.depth_enabled = True
    camera.queue = FakeQueue(message)
    camera.last_read_status = "starting"
    camera.depth_native_shape = None
    camera.depth_resized = False
    return camera


def make_device(mxid="traffic-oak", usb_speed="SUPER"):
    device = Mock()
    device.getMxId.return_value = mxid
    device.getUsbSpeed.return_value = f"UsbSpeed.{usb_speed}"
    return device


def make_device_info(mxid="traffic-oak"):
    device_info = Mock()
    device_info.getMxId.return_value = mxid
    return device_info


def make_pipeline_kwargs(
    *,
    width=1280,
    height=720,
    fps=10.0,
    depth_enabled=False,
):
    return {
        "width": width,
        "height": height,
        "fps": fps,
        "depth_confidence_threshold": 245,
        "depth_left_right_check": True,
        "depth_subpixel": False,
        "depth_median_filter_size": 3,
        "depth_decimation_factor": 2,
        "depth_speckle_filter": False,
        "depth_spatial_filter": False,
        "depth_temporal_filter": False,
        "minimum_depth_m": 0.3,
        "maximum_depth_m": 10.0,
        "depth_enabled": depth_enabled,
    }


class TestOakCameraRead(unittest.TestCase):
    def test_empty_poll_is_reported_without_frame_error(self):
        camera = make_camera(None)

        success, frame, depth = camera.read()

        self.assertFalse(success)
        self.assertIsNone(frame)
        self.assertIsNone(depth)
        self.assertEqual(camera.last_read_status, "empty")
        self.assertIsNone(camera.depth_native_shape)
        self.assertFalse(camera.depth_resized)

    def test_device_queue_error_is_reported_without_crashing(self):
        camera = make_camera(None)
        camera.queue = FailingQueue()

        success, frame, depth = camera.read()

        self.assertFalse(success)
        self.assertIsNone(frame)
        self.assertIsNone(depth)
        self.assertEqual(camera.last_read_status, "error")

    def test_missing_enabled_depth_is_rejected(self):
        frame = np.zeros((4, 6, 3), dtype=np.uint8)
        camera = make_camera(
            {
                "rgb": FakeRgbMessage(frame),
                "depth": FakeDepthMessage(None),
            }
        )

        success, returned_frame, returned_depth = camera.read()

        self.assertFalse(success)
        self.assertIsNone(returned_frame)
        self.assertIsNone(returned_depth)
        self.assertEqual(camera.last_read_status, "error")

    def test_same_aspect_depth_is_resized_with_nearest_neighbor(self):
        frame = np.zeros((4, 6, 3), dtype=np.uint8)
        native_depth = np.array(
            [
                [100, 200, 300],
                [400, 500, 600],
            ],
            dtype=np.uint16,
        )
        camera = make_camera(
            {
                "rgb": FakeRgbMessage(frame),
                "depth": FakeDepthMessage(native_depth),
            }
        )

        success, returned_frame, resized_depth = camera.read()

        expected_depth = np.repeat(
            np.repeat(native_depth, 2, axis=0),
            2,
            axis=1,
        )
        self.assertTrue(success)
        self.assertIs(returned_frame, frame)
        np.testing.assert_array_equal(resized_depth, expected_depth)
        self.assertEqual(resized_depth.dtype, native_depth.dtype)
        self.assertEqual(camera.last_read_status, "ok")
        self.assertEqual(camera.depth_native_shape, (2, 3))
        self.assertTrue(camera.depth_resized)

    def test_different_aspect_depth_is_rejected(self):
        frame = np.zeros((4, 8, 3), dtype=np.uint8)
        native_depth = np.ones((3, 4), dtype=np.uint16)
        camera = make_camera(
            {
                "rgb": FakeRgbMessage(frame),
                "depth": FakeDepthMessage(native_depth),
            }
        )

        success, returned_frame, returned_depth = camera.read()

        self.assertFalse(success)
        self.assertIsNone(returned_frame)
        self.assertIsNone(returned_depth)
        self.assertEqual(camera.last_read_status, "shape_error")
        self.assertEqual(camera.depth_native_shape, (3, 4))
        self.assertFalse(camera.depth_resized)


class TestOakDeviceSelection(unittest.TestCase):
    def test_camera_normalizes_mxid_once_before_opening_device(self):
        pipeline = object()
        expected_device = make_device()
        queue = object()
        session = SimpleNamespace(
            api_major=2,
            device=expected_device,
            pipeline=pipeline,
            queue=queue,
            mxid="traffic-oak",
            usb_speed="super",
        )

        with (
            patch("stack_traffic.oak_camera.dai", Mock()),
            patch(
                "stack_traffic.oak_camera.depthai_api_major",
                return_value=2,
            ),
            patch(
                "stack_traffic.oak_camera.validate_depthai_version",
                return_value=(2, (2, 30, 0, 0)),
            ),
            patch(
                "stack_traffic.oak_camera.normalize_oak_mxid",
                return_value="traffic-oak",
            ) as normalize,
            patch(
                "stack_traffic.oak_camera._open_oak_v2_session",
                return_value=session,
            ) as open_session,
        ):
            camera = OakRgbdCamera(
                width=1280,
                height=720,
                fps=20.0,
                depth_enabled=False,
                depth_confidence_threshold=245,
                depth_left_right_check=True,
                depth_subpixel=True,
                depth_median_filter_size=7,
                depth_decimation_factor=1,
                depth_speckle_filter=True,
                depth_spatial_filter=True,
                depth_temporal_filter=True,
                minimum_depth_m=0.3,
                maximum_depth_m=20.0,
                mxid="  traffic-oak\n",
            )

        normalize.assert_called_once_with("  traffic-oak\n")
        self.assertEqual(
            open_session.call_args.kwargs["requested_mxid"],
            "traffic-oak",
        )
        self.assertEqual(
            open_session.call_args.kwargs["requested_usb_speed"],
            "super",
        )
        self.assertEqual(camera.requested_mxid, "traffic-oak")

    def test_explicit_mxid_is_passed_to_depthai(self):
        pipeline = object()
        device_info = object()
        expected_device = make_device("14442C108144F1D000")
        fake_dai = Mock()
        fake_dai.DeviceInfo.return_value = device_info
        fake_dai.Device.return_value = expected_device

        with patch("stack_traffic.oak_camera.dai", fake_dai):
            device = open_oak_device(
                pipeline,
                "  14442C108144F1D000\n",
            )

        self.assertIs(device, expected_device)
        fake_dai.DeviceInfo.assert_called_once_with(
            "14442C108144F1D000"
        )
        fake_dai.Device.assert_called_once_with(
            pipeline,
            device_info,
            fake_dai.UsbSpeed.SUPER,
        )
        fake_dai.Device.getAllConnectedDevices.assert_not_called()

    def test_blank_mxid_opens_the_enumerated_single_device(self):
        pipeline = object()
        device_info = make_device_info()
        expected_device = make_device()
        fake_dai = Mock()
        fake_dai.Device.getAllConnectedDevices.return_value = [device_info]
        fake_dai.Device.return_value = expected_device

        with patch("stack_traffic.oak_camera.dai", fake_dai):
            device = open_oak_device(pipeline, "  ")

        self.assertIs(device, expected_device)
        fake_dai.DeviceInfo.assert_not_called()
        fake_dai.Device.assert_called_once_with(
            pipeline,
            device_info,
            fake_dai.UsbSpeed.SUPER,
        )

    def test_blank_mxid_with_no_device_retries_then_fails_fast(self):
        pipeline = object()
        fake_dai = Mock()
        fake_dai.Device.getAllConnectedDevices.return_value = []

        with (
            patch("stack_traffic.oak_camera.dai", fake_dai),
            patch("stack_traffic.oak_camera.time.sleep") as sleep,
            self.assertRaisesRegex(
                RuntimeError,
                "단일 OAK-D.*3회.*열거되지 않았습니다",
            ),
        ):
            open_oak_device(pipeline)

        fake_dai.DeviceInfo.assert_not_called()
        self.assertEqual(
            fake_dai.Device.getAllConnectedDevices.call_count,
            3,
        )
        fake_dai.Device.assert_not_called()
        self.assertEqual(
            sleep.call_args_list,
            [call(2.0), call(2.0)],
        )

    def test_blank_mxid_reenumerates_after_transient_open_failure(self):
        pipeline = object()
        stale_info = make_device_info()
        fresh_info = make_device_info()
        expected_device = make_device()
        fake_dai = Mock()
        fake_dai.Device.getAllConnectedDevices.side_effect = [
            [stale_info],
            [fresh_info],
        ]
        fake_dai.Device.side_effect = [
            RuntimeError("X_LINK_BOOTED"),
            expected_device,
        ]

        with (
            patch("stack_traffic.oak_camera.dai", fake_dai),
            patch("stack_traffic.oak_camera.time.sleep") as sleep,
        ):
            device = open_oak_device(pipeline)

        self.assertIs(device, expected_device)
        self.assertEqual(
            fake_dai.Device.call_args_list,
            [
                call(pipeline, stale_info, fake_dai.UsbSpeed.SUPER),
                call(pipeline, fresh_info, fake_dai.UsbSpeed.SUPER),
            ],
        )
        sleep.assert_called_once_with(2.0)

    def test_blank_mxid_accepts_device_appearing_during_retry(self):
        pipeline = object()
        device_info = make_device_info()
        expected_device = make_device()
        fake_dai = Mock()
        fake_dai.Device.getAllConnectedDevices.side_effect = [
            [],
            [device_info],
        ]
        fake_dai.Device.return_value = expected_device

        with (
            patch("stack_traffic.oak_camera.dai", fake_dai),
            patch("stack_traffic.oak_camera.time.sleep") as sleep,
        ):
            device = open_oak_device(pipeline)

        self.assertIs(device, expected_device)
        fake_dai.Device.assert_called_once_with(
            pipeline,
            device_info,
            fake_dai.UsbSpeed.SUPER,
        )
        sleep.assert_called_once_with(2.0)

    def test_blank_mxid_retries_enumeration_error(self):
        pipeline = object()
        device_info = make_device_info()
        expected_device = make_device()
        fake_dai = Mock()
        fake_dai.Device.getAllConnectedDevices.side_effect = [
            RuntimeError("X_LINK_ERROR"),
            [device_info],
        ]
        fake_dai.Device.return_value = expected_device

        with (
            patch("stack_traffic.oak_camera.dai", fake_dai),
            patch("stack_traffic.oak_camera.time.sleep") as sleep,
        ):
            device = open_oak_device(pipeline)

        self.assertIs(device, expected_device)
        fake_dai.Device.assert_called_once_with(
            pipeline,
            device_info,
            fake_dai.UsbSpeed.SUPER,
        )
        sleep.assert_called_once_with(2.0)

    def test_explicit_mxid_retries_transient_open_failure(self):
        pipeline = object()
        first_info = object()
        second_info = object()
        expected_device = make_device("14442C108144F1D000")
        fake_dai = Mock()
        fake_dai.DeviceInfo.side_effect = [first_info, second_info]
        fake_dai.Device.side_effect = [
            RuntimeError("X_LINK_BOOTED"),
            expected_device,
        ]

        with (
            patch("stack_traffic.oak_camera.dai", fake_dai),
            patch("stack_traffic.oak_camera.time.sleep") as sleep,
        ):
            device = open_oak_device(
                pipeline,
                "14442C108144F1D000",
            )

        self.assertIs(device, expected_device)
        self.assertEqual(
            fake_dai.DeviceInfo.call_args_list,
            [
                call("14442C108144F1D000"),
                call("14442C108144F1D000"),
            ],
        )
        self.assertEqual(
            fake_dai.Device.call_args_list,
            [
                call(pipeline, first_info, fake_dai.UsbSpeed.SUPER),
                call(pipeline, second_info, fake_dai.UsbSpeed.SUPER),
            ],
        )
        fake_dai.Device.getAllConnectedDevices.assert_not_called()
        sleep.assert_called_once_with(2.0)

    def test_explicit_mxid_reports_error_after_bounded_retries(self):
        pipeline = object()
        device_info = object()
        fake_dai = Mock()
        fake_dai.DeviceInfo.return_value = device_info
        fake_dai.Device.side_effect = RuntimeError("X_LINK_BOOTED")

        with (
            patch("stack_traffic.oak_camera.dai", fake_dai),
            patch("stack_traffic.oak_camera.time.sleep") as sleep,
            self.assertRaisesRegex(
                RuntimeError,
                "oak_mxid=traffic-oak.*3회.*X_LINK_BOOTED",
            ),
        ):
            open_oak_device(pipeline, "traffic-oak")

        self.assertEqual(fake_dai.DeviceInfo.call_count, 3)
        self.assertEqual(fake_dai.Device.call_count, 3)
        fake_dai.Device.getAllConnectedDevices.assert_not_called()
        self.assertEqual(
            sleep.call_args_list,
            [call(2.0), call(2.0)],
        )

    def test_blank_mxid_rejects_ambiguous_multiple_devices(self):
        pipeline = object()
        first = Mock()
        first.getMxId.return_value = "lane-oak"
        second = Mock()
        second.getMxId.return_value = "traffic-oak"
        fake_dai = Mock()
        fake_dai.Device.getAllConnectedDevices.return_value = [first, second]

        with (
            patch("stack_traffic.oak_camera.dai", fake_dai),
            patch("stack_traffic.oak_camera.time.sleep") as sleep,
            self.assertRaisesRegex(
                RuntimeError,
                "oak_mxid.*lane-oak.*traffic-oak",
            ),
        ):
            open_oak_device(pipeline)

        fake_dai.Device.assert_not_called()
        sleep.assert_not_called()

    def test_high_speed_is_passed_and_verified(self):
        pipeline = object()
        device_info = object()
        expected_device = Mock()
        expected_device.getMxId.return_value = "traffic-oak"
        expected_device.getUsbSpeed.return_value = "UsbSpeed.HIGH"
        fake_dai = Mock()
        fake_dai.DeviceInfo.return_value = device_info
        fake_dai.Device.return_value = expected_device

        with patch("stack_traffic.oak_camera.dai", fake_dai):
            device = open_oak_device(
                pipeline,
                "traffic-oak",
                "high",
            )

        self.assertIs(device, expected_device)
        fake_dai.Device.assert_called_once_with(
            pipeline,
            device_info,
            fake_dai.UsbSpeed.HIGH,
        )

    def test_high_speed_mismatch_closes_and_fails_without_retry(self):
        pipeline = object()
        expected_device = Mock()
        expected_device.getMxId.return_value = "traffic-oak"
        expected_device.getUsbSpeed.return_value = "UsbSpeed.SUPER"
        fake_dai = Mock()
        fake_dai.Device.return_value = expected_device

        with (
            patch("stack_traffic.oak_camera.dai", fake_dai),
            patch("stack_traffic.oak_camera.time.sleep") as sleep,
            self.assertRaisesRegex(RuntimeError, "실제 링크가 HIGH가 아닙니다"),
        ):
            open_oak_device(pipeline, "traffic-oak", "high")

        expected_device.close.assert_called_once_with()
        self.assertEqual(fake_dai.Device.call_count, 1)
        sleep.assert_not_called()

    def test_invalid_speed_fails_before_device_access(self):
        fake_dai = Mock()

        with (
            patch("stack_traffic.oak_camera.dai", fake_dai),
            self.assertRaisesRegex(ValueError, "high, super"),
        ):
            open_oak_device(object(), "traffic-oak", "usb2")

        fake_dai.DeviceInfo.assert_not_called()
        fake_dai.Device.assert_not_called()

    def test_unknown_actual_mxid_is_rejected_fail_closed(self):
        pipeline = object()
        device = Mock()
        device.getMxId.return_value = ""
        device.getDeviceId.return_value = ""
        fake_dai = Mock()
        fake_dai.Device.return_value = device

        with (
            patch("stack_traffic.oak_camera.dai", fake_dai),
            patch("stack_traffic.oak_camera.time.sleep") as sleep,
            self.assertRaisesRegex(RuntimeError, "실제 MxID를 확인할 수"),
        ):
            open_oak_device(pipeline, "traffic-oak")

        device.close.assert_called_once_with()
        sleep.assert_not_called()

    def test_blank_mxid_rejects_selected_and_actual_id_mismatch(self):
        pipeline = object()
        device_info = make_device_info("lane-oak")
        device = make_device("traffic-oak")
        fake_dai = Mock()
        fake_dai.Device.getAllConnectedDevices.return_value = [device_info]
        fake_dai.Device.return_value = device

        with (
            patch("stack_traffic.oak_camera.dai", fake_dai),
            patch("stack_traffic.oak_camera.time.sleep") as sleep,
            self.assertRaisesRegex(
                RuntimeError,
                "expected=lane-oak, connected=traffic-oak",
            ),
        ):
            open_oak_device(pipeline)

        device.close.assert_called_once_with()
        sleep.assert_not_called()

    def test_v2_queue_failure_rebuilds_full_session_before_retry(self):
        first_info = object()
        second_info = object()
        first_pipeline = object()
        second_pipeline = object()
        first_device = make_device("traffic-oak", "HIGH")
        second_device = make_device("traffic-oak", "HIGH")
        expected_queue = object()
        first_device.getOutputQueue.side_effect = RuntimeError(
            "X_LINK_QUEUE_ERROR"
        )
        second_device.getOutputQueue.return_value = expected_queue
        fake_dai = Mock()
        fake_dai.DeviceInfo.side_effect = [first_info, second_info]
        fake_dai.Device.side_effect = [first_device, second_device]

        with (
            patch("stack_traffic.oak_camera.dai", fake_dai),
            patch(
                "stack_traffic.oak_camera.build_oak_pipeline",
                side_effect=[first_pipeline, second_pipeline],
            ) as build_pipeline,
            patch("stack_traffic.oak_camera.time.sleep") as sleep,
        ):
            session = _open_oak_v2_session(
                requested_mxid="traffic-oak",
                requested_usb_speed="high",
                **make_pipeline_kwargs(),
            )

        self.assertIs(session.pipeline, second_pipeline)
        self.assertIs(session.device, second_device)
        self.assertIs(session.queue, expected_queue)
        self.assertEqual(build_pipeline.call_count, 2)
        first_device.close.assert_called_once_with()
        second_device.close.assert_not_called()
        sleep.assert_called_once_with(2.0)

    def test_actual_high_link_rechecks_super_profile_bandwidth(self):
        device = make_device("traffic-oak", "HIGH")
        fake_dai = Mock()
        fake_dai.Device.return_value = device

        with (
            patch("stack_traffic.oak_camera.dai", fake_dai),
            patch(
                "stack_traffic.oak_camera.build_oak_pipeline",
                return_value=object(),
            ),
            patch("stack_traffic.oak_camera.time.sleep") as sleep,
            self.assertRaisesRegex(RuntimeError, "82.9MB/s"),
        ):
            _open_oak_v2_session(
                requested_mxid="traffic-oak",
                requested_usb_speed="super",
                **make_pipeline_kwargs(fps=30.0),
            )

        device.getOutputQueue.assert_not_called()
        device.close.assert_called_once_with()
        sleep.assert_not_called()

    def test_v3_explicit_mxid_uses_device_then_pipeline(self):
        device_info = object()
        device = Mock()
        device.getMxId.return_value = "traffic-oak"
        device.getDeviceId.return_value = "traffic-oak"
        device.getUsbSpeed.return_value = "UsbSpeed.HIGH"
        pipeline = Mock()
        queue = object()
        fake_dai = Mock()
        fake_dai.DeviceInfo.return_value = device_info
        fake_dai.Device.return_value = device
        fake_dai.Pipeline.return_value = pipeline

        with (
            patch("stack_traffic.oak_camera.dai", fake_dai),
            patch(
                "stack_traffic.oak_camera._build_oak_v3_pipeline",
                return_value=queue,
            ) as build_v3,
        ):
            session = _open_oak_v3_session(
                requested_mxid="traffic-oak",
                requested_usb_speed="high",
                width=1280,
                height=720,
                fps=10.0,
                depth_confidence_threshold=245,
                depth_left_right_check=True,
                depth_subpixel=False,
                depth_median_filter_size=3,
                depth_decimation_factor=2,
                depth_speckle_filter=False,
                depth_spatial_filter=False,
                depth_temporal_filter=False,
                minimum_depth_m=0.3,
                maximum_depth_m=10.0,
                depth_enabled=False,
            )

        fake_dai.Device.assert_called_once_with(
            device_info,
            fake_dai.UsbSpeed.HIGH,
        )
        fake_dai.Pipeline.assert_called_once_with(device)
        build_v3.assert_called_once()
        pipeline.start.assert_called_once_with()
        self.assertEqual(session.api_major, 3)
        self.assertIs(session.queue, queue)

    def test_v3_start_failure_cleans_up_then_retries_fresh_session(self):
        first_info = object()
        second_info = object()
        first_device = Mock()
        second_device = Mock()
        for device in (first_device, second_device):
            device.getMxId.return_value = "traffic-oak"
            device.getUsbSpeed.return_value = "UsbSpeed.HIGH"
        first_pipeline = Mock()
        second_pipeline = Mock()
        first_pipeline.start.side_effect = RuntimeError("X_LINK_BOOTED")
        first_queue = Mock()
        second_queue = Mock()
        fake_dai = Mock()
        fake_dai.DeviceInfo.side_effect = [first_info, second_info]
        fake_dai.Device.side_effect = [first_device, second_device]
        fake_dai.Pipeline.side_effect = [first_pipeline, second_pipeline]

        with (
            patch("stack_traffic.oak_camera.dai", fake_dai),
            patch(
                "stack_traffic.oak_camera._build_oak_v3_pipeline",
                side_effect=[first_queue, second_queue],
            ),
            patch("stack_traffic.oak_camera.time.sleep") as sleep,
        ):
            session = _open_oak_v3_session(
                requested_mxid="traffic-oak",
                requested_usb_speed="high",
                width=1280,
                height=720,
                fps=10.0,
                depth_confidence_threshold=245,
                depth_left_right_check=True,
                depth_subpixel=False,
                depth_median_filter_size=3,
                depth_decimation_factor=2,
                depth_speckle_filter=False,
                depth_spatial_filter=False,
                depth_temporal_filter=False,
                minimum_depth_m=0.3,
                maximum_depth_m=10.0,
                depth_enabled=False,
            )

        self.assertIs(session.device, second_device)
        self.assertIs(session.queue, second_queue)
        first_queue.close.assert_called_once_with()
        first_pipeline.stop.assert_called_once_with()
        first_pipeline.wait.assert_called_once_with()
        first_device.close.assert_called_once_with()
        second_device.close.assert_not_called()
        sleep.assert_called_once_with(2.0)


class TestOakConfiguration(unittest.TestCase):
    def test_depthai_version_detection_does_not_construct_pipeline(self):
        class FakePipeline:
            def start(self):
                pass

        fake_dai = Mock()
        fake_dai.Pipeline = FakePipeline

        with patch("stack_traffic.oak_camera.dai", fake_dai):
            self.assertEqual(depthai_api_major(), 3)

    def test_supported_depthai_versions_are_enforced_per_api(self):
        class V2Pipeline:
            pass

        class V3Pipeline:
            def start(self):
                pass

        with patch(
            "stack_traffic.oak_camera.dai",
            SimpleNamespace(Pipeline=V2Pipeline, __version__="2.30.0"),
        ):
            self.assertEqual(validate_depthai_version()[0], 2)

        with patch(
            "stack_traffic.oak_camera.dai",
            SimpleNamespace(Pipeline=V3Pipeline, __version__="3.6.1"),
        ):
            self.assertEqual(validate_depthai_version()[0], 3)

        with (
            patch(
                "stack_traffic.oak_camera.dai",
                SimpleNamespace(Pipeline=V3Pipeline, __version__="3.5.0"),
            ),
            self.assertRaisesRegex(RuntimeError, "required>=3.6"),
        ):
            validate_depthai_version()

    def test_depthai_runtime_and_detected_api_major_must_match(self):
        class V2Pipeline:
            pass

        class V3Pipeline:
            def start(self):
                pass

        mismatches = (
            (V3Pipeline, "4.0.0", "API=3.x"),
            (V2Pipeline, "3.6.1", "API=2.x"),
            (V3Pipeline, "2.30.0", "API=3.x"),
        )
        for pipeline, version, expected_api in mismatches:
            with (
                self.subTest(version=version, expected_api=expected_api),
                patch(
                    "stack_traffic.oak_camera.dai",
                    SimpleNamespace(Pipeline=pipeline, __version__=version),
                ),
                self.assertRaisesRegex(
                    RuntimeError,
                    f"{expected_api}.*major 일치",
                ),
            ):
                validate_depthai_version()

    def test_usb_speed_parser_is_strict(self):
        self.assertEqual(normalize_oak_usb_speed(" HIGH\n"), "high")
        self.assertEqual(normalize_oak_usb_speed("super"), "super")
        with self.assertRaises(ValueError):
            normalize_oak_usb_speed("auto")

    def test_usb2_bandwidth_accepts_vehicle_rgb_profile(self):
        validate_oak_usb_bandwidth(
            width=1280,
            height=720,
            fps=10.0,
            depth_enabled=False,
            usb_speed="high",
        )
        self.assertAlmostEqual(
            estimate_oak_host_bandwidth_bytes_per_sec(
                1280,
                720,
                10.0,
                False,
            ),
            27_648_000.0,
        )

    def test_usb2_bandwidth_rejects_full_resolution_rgbd(self):
        with self.assertRaisesRegex(ValueError, "46.1MB/s"):
            validate_oak_usb_bandwidth(
                width=1280,
                height=720,
                fps=10.0,
                depth_enabled=True,
                usb_speed="high",
            )

    def test_v3_rgb_only_pipeline_uses_camera_output_queue(self):
        pipeline = Mock()
        camera_builder = Mock()
        camera = Mock()
        rgb_output = Mock()
        queue = object()
        camera_builder.build.return_value = camera
        camera.requestOutput.return_value = rgb_output
        rgb_output.createOutputQueue.return_value = queue
        pipeline.create.return_value = camera_builder
        fake_dai = Mock()

        with patch("stack_traffic.oak_camera.dai", fake_dai):
            result = _build_oak_v3_pipeline(
                pipeline,
                width=1280,
                height=720,
                fps=10.0,
                depth_confidence_threshold=245,
                depth_left_right_check=True,
                depth_subpixel=False,
                depth_median_filter_size=3,
                depth_decimation_factor=2,
                depth_speckle_filter=False,
                depth_spatial_filter=False,
                depth_temporal_filter=False,
                minimum_depth_m=0.3,
                maximum_depth_m=10.0,
                depth_enabled=False,
            )

        self.assertIs(result, queue)
        pipeline.create.assert_called_once_with(fake_dai.node.Camera)
        camera_builder.build.assert_called_once_with(
            fake_dai.CameraBoardSocket.CAM_A
        )
        camera.requestOutput.assert_called_once_with(
            size=(1280, 720),
            type=fake_dai.ImgFrame.Type.BGR888i,
            fps=10.0,
        )
        rgb_output.createOutputQueue.assert_called_once_with(
            maxSize=1,
            blocking=False,
        )

    def test_v3_rgbd_pipeline_links_aligned_depth_through_sync(self):
        pipeline = Mock()
        color_builder = Mock()
        left_builder = Mock()
        right_builder = Mock()
        color = Mock()
        left = Mock()
        right = Mock()
        rgb_output = Mock()
        left_output = Mock()
        right_output = Mock()
        stereo = Mock()
        sync = Mock()
        queue = object()
        color_builder.build.return_value = color
        left_builder.build.return_value = left
        right_builder.build.return_value = right
        color.requestOutput.return_value = rgb_output
        left.requestOutput.return_value = left_output
        right.requestOutput.return_value = right_output
        sync.inputs = {"rgb": Mock(), "depth": Mock()}
        sync.out.createOutputQueue.return_value = queue
        stereo_config = SimpleNamespace(
            postProcessing=SimpleNamespace(
                speckleFilter=SimpleNamespace(enable=False),
                spatialFilter=SimpleNamespace(
                    enable=False,
                    holeFillingRadius=0,
                    numIterations=0,
                ),
                temporalFilter=SimpleNamespace(enable=False),
                decimationFilter=SimpleNamespace(decimationFactor=1),
                thresholdFilter=SimpleNamespace(minRange=0, maxRange=0),
            )
        )
        stereo.initialConfig.get.return_value = stereo_config
        pipeline.create.side_effect = [
            color_builder,
            left_builder,
            right_builder,
            stereo,
            sync,
        ]
        fake_dai = Mock()

        with patch("stack_traffic.oak_camera.dai", fake_dai):
            result = _build_oak_v3_pipeline(
                pipeline,
                **make_pipeline_kwargs(
                    width=640,
                    height=360,
                    depth_enabled=True,
                ),
            )

        self.assertIs(result, queue)
        color_builder.build.assert_called_once_with(
            fake_dai.CameraBoardSocket.CAM_A
        )
        left_builder.build.assert_called_once_with(
            fake_dai.CameraBoardSocket.CAM_B
        )
        right_builder.build.assert_called_once_with(
            fake_dai.CameraBoardSocket.CAM_C
        )
        left_output.link.assert_called_once_with(stereo.left)
        right_output.link.assert_called_once_with(stereo.right)
        rgb_output.link.assert_any_call(stereo.inputAlignTo)
        rgb_output.link.assert_any_call(sync.inputs["rgb"])
        stereo.depth.link.assert_called_once_with(sync.inputs["depth"])
        stereo.setOutputSize.assert_called_once_with(640, 360)
        sync.out.createOutputQueue.assert_called_once_with(
            maxSize=1,
            blocking=False,
        )

    def test_release_is_idempotent_and_waits_for_v3_pipeline(self):
        camera = OakRgbdCamera.__new__(OakRgbdCamera)
        camera.api_major = 3
        camera.queue = Mock()
        camera.pipeline = Mock()
        camera.device = Mock()
        queue = camera.queue
        pipeline = camera.pipeline
        device = camera.device

        camera.release()
        camera.release()

        queue.close.assert_called_once_with()
        pipeline.stop.assert_called_once_with()
        pipeline.wait.assert_called_once_with()
        device.close.assert_called_once_with()


@unittest.skipIf(
    dai is None or depthai_api_major() != 2,
    "DepthAI 2.x serialization test only",
)
class TestOakPipeline(unittest.TestCase):
    def test_rgb_only_pipeline_omits_stereo_depth(self):
        pipeline = build_oak_pipeline(
            1280,
            720,
            20.0,
            depth_enabled=False,
        )
        serialized_nodes = [
            node
            for _, node in pipeline.serializeToJson()["pipeline"]["nodes"]
        ]
        node_names = [node["name"] for node in serialized_nodes]
        self.assertIn("ColorCamera", node_names)
        self.assertNotIn("MonoCamera", node_names)
        self.assertNotIn("StereoDepth", node_names)
        output = next(
            node for node in serialized_nodes if node["name"] == "XLinkOut"
        )
        self.assertEqual(output["properties"]["streamName"], "rgb")

    def test_full_resolution_raw_diagnostic_configuration(self):
        pipeline = build_oak_pipeline(
            1280,
            720,
            30.0,
            depth_confidence_threshold=245,
            depth_left_right_check=True,
            depth_median_filter_size=0,
            depth_speckle_filter=False,
            depth_spatial_filter=False,
            depth_temporal_filter=False,
            minimum_depth_m=0.3,
            maximum_depth_m=60.0,
        )
        stereo_properties = next(
            node["properties"]
            for _, node in pipeline.serializeToJson()["pipeline"]["nodes"]
            if node["name"] == "StereoDepth"
        )
        config = stereo_properties["initialConfig"]
        algorithm = config["algorithmControl"]
        post_processing = config["postProcessing"]

        self.assertTrue(algorithm["enableLeftRightCheck"])
        self.assertTrue(algorithm["enableSubpixel"])
        self.assertEqual(
            config["costMatching"]["confidenceThreshold"],
            245,
        )
        self.assertEqual(post_processing["median"], 0)
        self.assertFalse(post_processing["speckleFilter"]["enable"])
        self.assertFalse(post_processing["spatialFilter"]["enable"])
        self.assertFalse(post_processing["temporalFilter"]["enable"])
        self.assertEqual(
            post_processing["decimationFilter"]["decimationFactor"],
            1,
        )
        self.assertEqual(
            post_processing["thresholdFilter"]["maxRange"],
            60000,
        )
        self.assertEqual(stereo_properties["outWidth"], 1280)
        self.assertEqual(stereo_properties["outHeight"], 720)

    def test_light_depth_configuration_keeps_aligned_output_size(self):
        pipeline = build_oak_pipeline(
            1280,
            720,
            30.0,
            depth_left_right_check=True,
            depth_subpixel=False,
            depth_median_filter_size=3,
            depth_decimation_factor=2,
            depth_speckle_filter=False,
            depth_spatial_filter=False,
            depth_temporal_filter=False,
        )
        stereo_properties = next(
            node["properties"]
            for _, node in pipeline.serializeToJson()["pipeline"]["nodes"]
            if node["name"] == "StereoDepth"
        )
        config = stereo_properties["initialConfig"]
        self.assertFalse(config["algorithmControl"]["enableSubpixel"])
        self.assertEqual(
            config["postProcessing"]["decimationFilter"][
                "decimationFactor"
            ],
            2,
        )
        self.assertEqual(stereo_properties["outWidth"], 1280)
        self.assertEqual(stereo_properties["outHeight"], 720)


if __name__ == "__main__":
    unittest.main()
