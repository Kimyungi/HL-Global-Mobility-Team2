import math
import unittest
from collections import deque

import cv2
import numpy as np

from stack_traffic.stopline_detector import (
    detect_stop_line,
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
    def setUp(self):
        self.frame = np.full((480, 640, 3), 70, dtype=np.uint8)

    def test_detects_long_horizontal_stop_bar(self):
        cv2.rectangle(self.frame, (45, 370), (595, 390), (235, 235, 235), -1)

        result = detect_stop_line(self.frame)

        self.assertTrue(result.detected)
        self.assertIsNotNone(result.bbox)
        self.assertGreater(result.width_ratio, 0.9)
        self.assertAlmostEqual(result.near_edge_y_px, 390, delta=2)
        self.assertAlmostEqual(result.maximum_edge_y_px, 390, delta=2)
        self.assertLess(abs(result.angle_deg), 2.0)

    def test_detects_dim_stop_bar_by_local_contrast_at_night(self):
        frame = np.full((480, 640, 3), 35, dtype=np.uint8)
        cv2.rectangle(frame, (45, 370), (595, 390), (90, 90, 90), -1)

        result = detect_stop_line(frame)

        self.assertTrue(result.detected)
        self.assertGreater(result.width_ratio, 0.9)
        self.assertAlmostEqual(result.maximum_edge_y_px, 390, delta=3)

    def test_rejects_uniform_dim_road_at_night(self):
        frame = np.full((480, 640, 3), 80, dtype=np.uint8)

        result = detect_stop_line(frame)

        self.assertFalse(result.detected)

    def test_detects_stop_bar_from_horizontal_edge_pair_only(self):
        frame = np.full((480, 640, 3), 55, dtype=np.uint8)
        cv2.rectangle(frame, (45, 370), (595, 390), (85, 85, 85), -1)

        result = detect_stop_line(
            frame,
            minimum_value=255,
            local_contrast_enabled=False,
            edge_pair_enabled=True,
            edge_pair_canny_low=10,
            edge_pair_canny_high=30,
        )

        self.assertTrue(result.detected)
        self.assertGreater(result.width_ratio, 0.9)

    def test_rejects_single_horizontal_edge(self):
        frame = np.full((480, 640, 3), 55, dtype=np.uint8)
        frame[380:, :] = 90

        result = detect_stop_line(
            frame,
            minimum_value=255,
            local_contrast_enabled=False,
            edge_pair_enabled=True,
            edge_pair_canny_low=10,
            edge_pair_canny_high=30,
        )

        self.assertFalse(result.detected)

    def test_rejects_separated_zebra_stripes(self):
        for x in range(90, 570, 95):
            polygon = np.array(
                [
                    [x, 260],
                    [x + 35, 260],
                    [x + 60, 410],
                    [x - 15, 410],
                ],
                dtype=np.int32,
            )
            cv2.fillConvexPoly(self.frame, polygon, (235, 235, 235))

        result = detect_stop_line(self.frame)

        self.assertFalse(result.detected)

    def test_selects_stop_bar_below_zebra_stripes(self):
        for x in range(100, 560, 100):
            cv2.rectangle(
                self.frame,
                (x, 260),
                (x + 42, 345),
                (235, 235, 235),
                -1,
            )
        cv2.rectangle(self.frame, (55, 380), (585, 396), (240, 240, 240), -1)

        result = detect_stop_line(self.frame)

        self.assertTrue(result.detected)
        self.assertAlmostEqual(result.near_edge_y_px, 396, delta=2)

    def test_rejects_steep_diagonal_marking(self):
        cv2.line(self.frame, (60, 410), (580, 260), (240, 240, 240), 14)

        result = detect_stop_line(self.frame)

        self.assertFalse(result.detected)

    def test_accepts_small_camera_roll(self):
        cv2.line(self.frame, (50, 375), (590, 420), (240, 240, 240), 14)

        result = detect_stop_line(self.frame)

        self.assertTrue(result.detected)
        self.assertLess(abs(result.angle_deg), 12.0)
        self.assertAlmostEqual(result.near_edge_y_px, 405, delta=4)
        self.assertAlmostEqual(result.maximum_edge_y_px, 427, delta=4)

    def test_uses_lowest_rotated_stopline_endpoint(self):
        cv2.line(self.frame, (50, 375), (590, 420), (240, 240, 240), 14)

        result = detect_stop_line(self.frame)

        self.assertTrue(result.detected)
        self.assertIsNotNone(result.bbox)
        self.assertAlmostEqual(
            result.maximum_edge_y_px,
            result.bbox[3] - 1,
            delta=2,
        )

    def test_rejects_two_long_segments_with_center_gap(self):
        cv2.line(self.frame, (45, 385), (270, 385), (240, 240, 240), 14)
        cv2.line(self.frame, (370, 385), (595, 385), (240, 240, 240), 14)

        result = detect_stop_line(self.frame)

        self.assertFalse(result.detected)

    def test_blank_frame_returns_diagnostic_mask(self):
        result = detect_stop_line(self.frame)

        self.assertFalse(result.detected)
        self.assertIsNone(result.bbox)
        self.assertTrue(math.isnan(result.near_edge_y_px))
        self.assertTrue(math.isnan(result.maximum_edge_y_px))
        x1, y1, x2, y2 = result.roi_bbox
        self.assertEqual(result.white_mask.shape, (y2 - y1, x2 - x1))

    def test_depth_bbox_uses_near_edge_and_inner_span(self):
        bbox = make_stopline_depth_bbox(
            (100, 300, 500, 330),
            (480, 640, 3),
            inner_width_ratio=0.50,
            band_height_px=12,
        )

        self.assertEqual(bbox, (200, 317, 400, 342))

    def test_roi_mask_is_mapped_back_to_full_frame(self):
        cv2.rectangle(self.frame, (45, 370), (595, 390), (235, 235, 235), -1)
        result = detect_stop_line(self.frame)

        mask = stopline_mask_in_frame(result, self.frame.shape)

        self.assertEqual(mask.shape, self.frame.shape[:2])
        self.assertGreater(np.count_nonzero(mask[365:395]), 0)
        self.assertEqual(np.count_nonzero(mask[:200]), 0)

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

    def test_depth_band_uses_rotated_line_center_edge(self):
        cv2.line(self.frame, (50, 375), (590, 420), (240, 240, 240), 14)
        result = detect_stop_line(self.frame)

        bbox = make_stopline_depth_bbox(
            result.bbox,
            self.frame.shape,
            inner_width_ratio=0.50,
            band_height_px=12,
            near_edge_y_px=result.near_edge_y_px,
        )

        self.assertAlmostEqual(
            0.5 * (bbox[1] + bbox[3] - 1),
            405,
            delta=4,
        )

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
