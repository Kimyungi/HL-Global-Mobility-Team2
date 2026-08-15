from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import stack_lane.oak_camera as oak_camera
from stack_lane.oak_camera import (
    LaneOakConfigurationError,
    depthai_api_major,
    normalize_usb_speed,
    open_oak_session,
    open_oak_v2_device,
    open_oak_v3_device,
    validate_camera_profile,
    validate_depthai_version,
)


class TestDepthaiApiDetection(unittest.TestCase):
    def test_v2_detection_does_not_construct_pipeline(self):
        class Pipeline:
            def __init__(self):
                raise AssertionError("Pipeline must not be instantiated")

        self.assertEqual(
            depthai_api_major(SimpleNamespace(Pipeline=Pipeline)),
            2,
        )

    def test_v3_detection_does_not_construct_pipeline(self):
        class Pipeline:
            def __init__(self):
                raise AssertionError("Pipeline must not be instantiated")

            def start(self):
                pass

        self.assertEqual(
            depthai_api_major(SimpleNamespace(Pipeline=Pipeline)),
            3,
        )

    def test_supported_v2_and_v3_runtime_versions_pass(self):
        class V2Pipeline:
            pass

        class V3Pipeline:
            def start(self):
                pass

        v2_api, _ = validate_depthai_version(
            SimpleNamespace(Pipeline=V2Pipeline, __version__="2.30.0")
        )
        v3_api, _ = validate_depthai_version(
            SimpleNamespace(Pipeline=V3Pipeline, __version__="3.6.1")
        )

        self.assertEqual(v2_api, 2)
        self.assertEqual(v3_api, 3)

    def test_v4_unknown_and_api_runtime_major_mismatch_fail_closed(self):
        class V2Pipeline:
            pass

        class V3Pipeline:
            def start(self):
                pass

        unsupported = (
            (V3Pipeline, "4.0.0"),
            (V2Pipeline, "3.6.1"),
            (V3Pipeline, "2.30.0"),
            (V3Pipeline, ""),
        )
        for pipeline, version in unsupported:
            with (
                self.subTest(version=version),
                self.assertRaisesRegex(
                    LaneOakConfigurationError,
                    "runtime/API major must match",
                ),
            ):
                validate_depthai_version(
                    SimpleNamespace(
                        Pipeline=pipeline,
                        __version__=version,
                    )
                )


class TestLaneCameraConfiguration(unittest.TestCase):
    def test_usb_speed_parser_is_strict(self):
        self.assertEqual(normalize_usb_speed(" HIGH\n"), "high")
        self.assertEqual(normalize_usb_speed("super"), "super")
        with self.assertRaises(ValueError):
            normalize_usb_speed("auto")

    def test_usb2_accepts_production_profile_and_rejects_30fps(self):
        validate_camera_profile(
            width=1280,
            height=720,
            fps=10,
            usb_speed="high",
        )
        with self.assertRaisesRegex(ValueError, "82.9MB/s"):
            validate_camera_profile(
                width=1280,
                height=720,
                fps=30,
                usb_speed="high",
            )


class TestLaneCameraOpening(unittest.TestCase):
    @staticmethod
    def _fake_dai(*, actual_mxid="lane-oak", actual_speed="HIGH"):
        class UsbSpeed:
            HIGH = object()
            SUPER = object()

        device_info = object()
        device = Mock()
        device.getMxId.return_value = actual_mxid
        device.getUsbSpeed.return_value = f"UsbSpeed.{actual_speed}"
        module = SimpleNamespace(
            UsbSpeed=UsbSpeed,
            DeviceInfo=Mock(return_value=device_info),
            Device=Mock(return_value=device),
        )
        return module, device_info, device

    def test_v2_opens_explicit_mxid_and_high_speed(self):
        module, device_info, device = self._fake_dai()
        pipeline = object()

        opened = open_oak_v2_device(
            pipeline,
            mxid="lane-oak",
            usb_speed="high",
            dai_module=module,
            max_attempts=1,
        )

        self.assertIs(opened, device)
        module.DeviceInfo.assert_called_once_with("lane-oak")
        module.Device.assert_called_once_with(
            pipeline,
            device_info,
            module.UsbSpeed.HIGH,
        )

    def test_v3_opens_device_before_pipeline_with_mxid_and_high(self):
        module, device_info, device = self._fake_dai()

        opened = open_oak_v3_device(
            mxid="lane-oak",
            usb_speed="high",
            dai_module=module,
            max_attempts=1,
        )

        self.assertIs(opened, device)
        module.Device.assert_called_once_with(
            device_info,
            module.UsbSpeed.HIGH,
        )

    def test_high_speed_mismatch_closes_and_fails_without_retry(self):
        module, _device_info, device = self._fake_dai(
            actual_speed="SUPER",
        )
        sleep = Mock()

        with self.assertRaisesRegex(
            LaneOakConfigurationError,
            "actual link is not HIGH",
        ):
            open_oak_v2_device(
                object(),
                mxid="lane-oak",
                usb_speed="high",
                dai_module=module,
                sleep_fn=sleep,
            )

        device.close.assert_called_once_with()
        self.assertEqual(module.Device.call_count, 1)
        sleep.assert_not_called()

    def test_mxid_mismatch_closes_and_fails_without_retry(self):
        module, _device_info, device = self._fake_dai(
            actual_mxid="traffic-oak",
        )
        sleep = Mock()

        with self.assertRaisesRegex(
            LaneOakConfigurationError,
            "does not match camera_mxid",
        ):
            open_oak_v3_device(
                mxid="lane-oak",
                usb_speed="high",
                dai_module=module,
                sleep_fn=sleep,
            )

        device.close.assert_called_once_with()
        self.assertEqual(module.Device.call_count, 1)
        sleep.assert_not_called()

    def test_blank_mxid_rejects_multiple_connected_devices(self):
        module, _device_info, _device = self._fake_dai()
        first = Mock()
        first.getMxId.return_value = "lane-oak"
        second = Mock()
        second.getMxId.return_value = "traffic-oak"
        module.Device.getAllConnectedDevices.return_value = [first, second]

        with self.assertRaisesRegex(
            LaneOakConfigurationError,
            "camera_mxid is required",
        ):
            open_oak_v2_device(
                object(),
                mxid="",
                usb_speed="high",
                dai_module=module,
                max_attempts=1,
            )

        self.assertEqual(module.Device.call_count, 0)

    def test_blank_single_device_info_id_must_match_opened_device(self):
        module, _device_info, device = self._fake_dai(
            actual_mxid="traffic-oak",
        )
        selected_info = Mock()
        selected_info.getMxId.return_value = "lane-oak"
        module.Device.getAllConnectedDevices.return_value = [selected_info]

        with self.assertRaisesRegex(
            LaneOakConfigurationError,
            "requested=lane-oak, actual=traffic-oak",
        ):
            open_oak_v2_device(
                object(),
                mxid="",
                usb_speed="high",
                dai_module=module,
                max_attempts=1,
            )

        device.close.assert_called_once_with()

    def test_unknown_actual_speed_fails_closed(self):
        module, _device_info, device = self._fake_dai(
            actual_speed="UNKNOWN",
        )

        with self.assertRaisesRegex(
            LaneOakConfigurationError,
            "did not report its actual USB speed",
        ):
            open_oak_v3_device(
                mxid="lane-oak",
                usb_speed="super",
                dai_module=module,
                max_attempts=1,
            )

        device.close.assert_called_once_with()


class TestCompleteSessionRetry(unittest.TestCase):
    def test_v2_queue_failure_rebuilds_the_complete_session(self):
        class PipelineType:
            pass

        module = SimpleNamespace(
            Pipeline=PipelineType,
            __version__="2.30.0",
        )
        first_pipeline = object()
        second_pipeline = object()
        first_device = Mock()
        first_device.getOutputQueue.side_effect = RuntimeError(
            "queue creation failed"
        )
        second_device = Mock()
        second_queue = object()
        second_device.getOutputQueue.return_value = second_queue
        sleep = Mock()

        with (
            patch.object(
                oak_camera,
                "_build_v2_pipeline",
                side_effect=[first_pipeline, second_pipeline],
            ) as build_pipeline,
            patch.object(
                oak_camera,
                "open_oak_v2_device",
                side_effect=[first_device, second_device],
            ) as open_device,
        ):
            session = open_oak_session(
                width=1280,
                height=720,
                fps=10,
                mxid="lane-oak",
                usb_speed="high",
                dai_module=module,
                sleep_fn=sleep,
                max_attempts=2,
            )

        self.assertIs(session.pipeline, second_pipeline)
        self.assertIs(session.device, second_device)
        self.assertIs(session.queue, second_queue)
        self.assertEqual(build_pipeline.call_count, 2)
        self.assertEqual(open_device.call_count, 2)
        first_device.close.assert_called_once_with()
        sleep.assert_called_once_with(
            oak_camera.OPEN_RETRY_INTERVAL_SEC
        )

    def test_v3_start_failure_closes_queue_stops_waits_and_closes_device(
        self,
    ):
        queue = Mock()
        output = Mock()
        output.createOutputQueue.return_value = queue
        camera = Mock()
        camera.requestOutput.return_value = output
        builder = Mock()
        builder.build.return_value = camera

        class PipelineType:
            latest = None

            def __init__(self, _device):
                type(self).latest = self
                self.stop_mock = Mock()
                self.wait_mock = Mock()

            def create(self, _node_type):
                return builder

            def start(self):
                raise RuntimeError("pipeline start failed")

            def stop(self):
                self.stop_mock()

            def wait(self):
                self.wait_mock()

        class UsbSpeed:
            HIGH = object()
            SUPER = object()

        device = Mock()
        device.getMxId = None
        device.getDeviceId.return_value = "lane-oak"
        device.getUsbSpeed.return_value = "UsbSpeed.HIGH"
        module = SimpleNamespace(
            Pipeline=PipelineType,
            __version__="3.6.1",
            DeviceInfo=Mock(return_value=object()),
            Device=Mock(return_value=device),
            UsbSpeed=UsbSpeed,
            node=SimpleNamespace(Camera=object()),
            CameraBoardSocket=SimpleNamespace(CAM_A=object()),
            ImgFrame=SimpleNamespace(
                Type=SimpleNamespace(BGR888i=object())
            ),
        )

        with self.assertRaisesRegex(
            RuntimeError,
            "failed to create the complete OAK session",
        ):
            open_oak_session(
                width=1280,
                height=720,
                fps=10,
                mxid="lane-oak",
                usb_speed="high",
                dai_module=module,
                max_attempts=1,
            )

        queue.close.assert_called_once_with()
        PipelineType.latest.stop_mock.assert_called_once_with()
        PipelineType.latest.wait_mock.assert_called_once_with()
        device.close.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
