import unittest

import cv2
import numpy as np

from stack_traffic.visual_tracker import (
    ShortTermTemplateTracker,
    scale_bbox,
    smooth_bbox,
)


class TestBboxHelpers(unittest.TestCase):
    frame_shape = (100, 160, 3)

    def test_scale_bbox_clips_to_frame(self):
        self.assertEqual(
            scale_bbox((0, 0, 20, 10), self.frame_shape, 2.0),
            (0, 0, 30, 15),
        )

    def test_smooth_bbox_reduces_detection_jitter(self):
        self.assertEqual(
            smooth_bbox(
                (40, 20, 80, 40),
                (44, 18, 84, 44),
                self.frame_shape,
                current_weight=0.5,
            ),
            (42, 19, 82, 42),
        )


class TestShortTermTemplateTracker(unittest.TestCase):
    @staticmethod
    def _make_frame(offset_x=0, offset_y=0):
        frame = np.full((120, 200, 3), 180, dtype=np.uint8)
        cv2.rectangle(
            frame,
            (70 + offset_x, 35 + offset_y),
            (130 + offset_x, 65 + offset_y),
            (15, 15, 15),
            -1,
        )
        cv2.circle(
            frame,
            (85 + offset_x, 50 + offset_y),
            8,
            (0, 0, 255),
            -1,
        )
        cv2.line(
            frame,
            (45 + offset_x, 35 + offset_y),
            (155 + offset_x, 35 + offset_y),
            (30, 30, 30),
            3,
        )
        return frame

    def test_tracks_small_shift_from_yolo_reference(self):
        tracker = ShortTermTemplateTracker(
            context_scale=1.8,
            search_scale=2.5,
            minimum_score=0.70,
            maximum_center_shift_ratio=0.75,
        )
        self.assertTrue(
            tracker.initialize(self._make_frame(), (70, 35, 130, 65))
        )
        result = tracker.track(self._make_frame(offset_x=4, offset_y=2))
        self.assertIsNotNone(result.bbox)
        self.assertGreaterEqual(result.score, 0.70)
        self.assertEqual(result.bbox, (74, 37, 134, 67))

    def test_rejects_unrelated_frame(self):
        tracker = ShortTermTemplateTracker(minimum_score=0.80)
        self.assertTrue(
            tracker.initialize(self._make_frame(), (70, 35, 130, 65))
        )
        unrelated = np.full((120, 200, 3), 180, dtype=np.uint8)
        result = tracker.track(unrelated)
        self.assertIsNone(result.bbox)

    def test_uniform_reference_is_not_initialized(self):
        tracker = ShortTermTemplateTracker()
        uniform = np.full((120, 200, 3), 128, dtype=np.uint8)
        self.assertFalse(tracker.initialize(uniform, (70, 35, 130, 65)))
        self.assertFalse(tracker.ready)


if __name__ == "__main__":
    unittest.main()
