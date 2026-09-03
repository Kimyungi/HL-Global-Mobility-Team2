import unittest

import numpy as np

from stack_traffic.yolo_stopline_detector import (
    detect_stop_line_yolo,
    stopline_class_ids,
)


class FakeTensor:
    def __init__(self, value):
        self._value = np.asarray(value)

    def cpu(self):
        return self

    def numpy(self):
        return self._value


class FakeBoxes:
    def __init__(self, confidences, classes):
        self.conf = FakeTensor(confidences)
        self.cls = FakeTensor(classes)


class FakeMasks:
    def __init__(self, masks):
        self.data = FakeTensor(masks)


class FakeResult:
    def __init__(self, masks, confidences, classes):
        self.masks = FakeMasks(masks) if masks is not None else None
        self.boxes = FakeBoxes(confidences, classes) if masks is not None else None


class FakeModel:
    def __init__(self, result):
        self.result = result
        self.kwargs = None

    def predict(self, **kwargs):
        self.kwargs = kwargs
        return [self.result]


class TestYoloStoplineDetector(unittest.TestCase):
    def test_finds_stopline_class_name(self):
        self.assertEqual(
            stopline_class_ids({0: "stop_line", 1: "crosswalk"}),
            [0],
        )

    def test_converts_mask_to_existing_detection_contract(self):
        frame = np.zeros((100, 200, 3), dtype=np.uint8)
        mask = np.zeros((100, 200), dtype=np.float32)
        mask[70:81, 25:176] = 1.0
        model = FakeModel(FakeResult([mask], [0.82], [0]))

        result = detect_stop_line_yolo(
            model,
            frame,
            class_ids=[0],
            roi_x_min=0.0,
            roi_y_min=0.5,
            roi_x_max=1.0,
            roi_y_max=1.0,
        )

        self.assertTrue(result.detected)
        self.assertEqual(result.bbox, (25, 70, 176, 81))
        self.assertAlmostEqual(result.near_edge_y_px, 80.0)
        self.assertAlmostEqual(result.maximum_edge_y_px, 80.0)
        self.assertAlmostEqual(result.score, 0.82, places=2)
        self.assertEqual(result.white_mask.shape, (50, 200))
        self.assertEqual(model.kwargs["classes"], [0])
        self.assertTrue(model.kwargs["retina_masks"])

    def test_rejects_mask_outside_stopline_roi(self):
        frame = np.zeros((100, 200, 3), dtype=np.uint8)
        mask = np.zeros((100, 200), dtype=np.float32)
        mask[10:20, 20:180] = 1.0
        model = FakeModel(FakeResult([mask], [0.95], [0]))

        result = detect_stop_line_yolo(
            model,
            frame,
            class_ids=[0],
            roi_y_min=0.5,
        )

        self.assertFalse(result.detected)
        self.assertIsNone(result.bbox)

    def test_prefers_closest_mask_over_higher_confidence_far_mask(self):
        frame = np.zeros((100, 200, 3), dtype=np.uint8)
        far = np.zeros((100, 200), dtype=np.float32)
        near = np.zeros((100, 200), dtype=np.float32)
        far[55:61, 20:180] = 1.0
        near[75:84, 30:170] = 1.0
        model = FakeModel(FakeResult([far, near], [0.95, 0.60], [0, 0]))

        result = detect_stop_line_yolo(model, frame, class_ids=[0])

        self.assertTrue(result.detected)
        self.assertAlmostEqual(result.maximum_edge_y_px, 83.0)
        self.assertAlmostEqual(result.score, 0.60, places=2)


if __name__ == "__main__":
    unittest.main()
