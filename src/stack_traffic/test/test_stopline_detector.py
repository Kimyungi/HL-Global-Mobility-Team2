import math
import unittest
from collections import deque

import numpy as np

from stack_traffic.stopline_detector import (
    detect_stop_line_from_yolo_result,
    make_stopline_depth_bbox,
    stable_stopline_y,
    stopline_mask_in_frame,
)


class FakeBoxes:
    def __init__(self, classes, confidences):
        self.cls = np.asarray(classes, dtype=np.float32)
        self.conf = np.asarray(confidences, dtype=np.float32)


class FakeMasks:
    def __init__(self, polygons):
        self.xy = polygons


class FakeSegmentationResult:
    def __init__(
        self,
        classes,
        confidences,
        polygons,
        names=None,
        orig_shape=None,
    ):
        self.boxes = FakeBoxes(classes, confidences)
        self.masks = FakeMasks(polygons)
        self.names = names or {0: "stop_line", 1: "crosswalk"}
        if orig_shape is not None:
            self.orig_shape = orig_shape


class TestStopLineDetector(unittest.TestCase):
    def test_depth_bbox_uses_near_edge_and_inner_span(self):
        bbox = make_stopline_depth_bbox(
            (100, 300, 500, 330),
            (480, 640, 3),
            inner_width_ratio=0.50,
            band_height_px=12,
        )

        self.assertEqual(bbox, (200, 317, 400, 342))

    def test_stability_requires_repeated_nearby_rows(self):
        stable = stable_stopline_y(
            deque([math.nan, 390.0, 391.0, 389.0, math.nan], maxlen=5),
            minimum_samples=3,
            maximum_fit_error_px=4.0,
            maximum_forward_step_px=20.0,
            maximum_backward_step_px=2.0,
        )
        unstable = stable_stopline_y(
            [350.0, 390.0, 430.0],
            minimum_samples=3,
            maximum_fit_error_px=4.0,
            maximum_forward_step_px=20.0,
            maximum_backward_step_px=2.0,
        )

        self.assertEqual(stable, (390.0, 3, True))
        self.assertFalse(unstable[2])

    def test_stability_allows_smooth_forward_motion(self):
        median_y, count, stable = stable_stopline_y(
            [350.0, 365.0, 381.0, 398.0, 416.0],
            minimum_samples=3,
            maximum_fit_error_px=2.0,
            maximum_forward_step_px=20.0,
            maximum_backward_step_px=2.0,
        )

        self.assertEqual(count, 5)
        self.assertAlmostEqual(median_y, 381.0)
        self.assertTrue(stable)

    def test_converts_yolo_segmentation_mask_to_frame_coordinates(self):
        result = FakeSegmentationResult(
            classes=[0],
            confidences=[0.82],
            polygons=[
                np.asarray(
                    [[20, 100], [480, 105], [480, 125], [20, 120]],
                    dtype=np.float32,
                )
            ],
        )

        detection = detect_stop_line_from_yolo_result(
            result,
            frame_shape=(480, 640, 3),
            roi_bbox=(70, 220, 570, 470),
        )

        self.assertTrue(detection.detected)
        self.assertEqual(detection.bbox, (90, 320, 551, 346))
        self.assertAlmostEqual(detection.score, 0.82, places=2)
        self.assertGreater(detection.maximum_edge_y_px, 340)
        self.assertEqual(detection.white_mask.shape, (250, 500))
        self.assertGreater(np.count_nonzero(detection.white_mask), 0)

    def test_yolo_conversion_rejects_wrong_class_and_low_confidence(self):
        polygon = np.asarray(
            [[20, 100], [480, 100], [480, 120], [20, 120]],
            dtype=np.float32,
        )
        result = FakeSegmentationResult(
            classes=[1, 0],
            confidences=[0.99, 0.20],
            polygons=[polygon, polygon],
        )

        detection = detect_stop_line_from_yolo_result(
            result,
            frame_shape=(480, 640, 3),
            roi_bbox=(70, 220, 570, 470),
            confidence_threshold=0.35,
        )

        self.assertFalse(detection.detected)
        self.assertIsNone(detection.bbox)

    def test_yolo_full_frame_result_is_clipped_to_search_roi(self):
        result = FakeSegmentationResult(
            classes=[0, 0],
            confidences=[0.90, 0.80],
            polygons=[
                np.asarray(
                    [[20, 80], [620, 80], [620, 110], [20, 110]],
                    dtype=np.float32,
                ),
                np.asarray(
                    [[90, 320], [550, 325], [550, 345], [90, 340]],
                    dtype=np.float32,
                ),
            ],
            orig_shape=(480, 640),
        )

        detection = detect_stop_line_from_yolo_result(
            result,
            frame_shape=(480, 640, 3),
            roi_bbox=(70, 220, 570, 470),
        )

        self.assertTrue(detection.detected)
        self.assertEqual(detection.bbox, (90, 320, 551, 346))
        self.assertAlmostEqual(detection.score, 0.80, places=2)


if __name__ == "__main__":
    unittest.main()
