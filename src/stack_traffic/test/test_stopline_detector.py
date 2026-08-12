import math
import unittest
from collections import deque

import cv2
import numpy as np

from stack_traffic.stopline_detector import (
    detect_stop_line,
    make_stopline_depth_bbox,
    stable_stopline_y,
    stopline_mask_in_frame,
)


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


if __name__ == "__main__":
    unittest.main()
