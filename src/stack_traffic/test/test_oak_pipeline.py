import unittest

import numpy as np

from stack_traffic.oak_camera import OakRgbdCamera, build_oak_pipeline, dai


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


@unittest.skipIf(dai is None, "depthai is not installed")
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
