import unittest

import cv2
import numpy as np

from stack_traffic.node import classify_signal_color


def bgr_from_hsv(hue: int, saturation: int = 255, value: int = 255):
    hsv = np.array([[[hue, saturation, value]]], dtype=np.uint8)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]


class TestSignalColorDetection(unittest.TestCase):
    bbox = (0, 0, 24, 16)

    def _classify_solid_hue(self, hue: int):
        frame = np.empty((16, 24, 3), dtype=np.uint8)
        frame[:, :] = bgr_from_hsv(hue)
        return classify_signal_color(
            frame,
            self.bbox,
            minimum_red_ratio=0.004,
            minimum_green_ratio=0.004,
        )

    def test_oak_orange_shift_is_classified_as_red(self):
        red_raw, green_raw, red_ratio, *_ = self._classify_solid_hue(20)
        self.assertEqual(red_raw, 1)
        self.assertEqual(green_raw, 0)
        self.assertGreater(red_ratio, 0.9)

    def test_true_yellow_is_not_classified_as_red(self):
        red_raw, green_raw, *_ = self._classify_solid_hue(30)
        self.assertEqual(red_raw, 0)
        self.assertEqual(green_raw, 0)

    def test_green_remains_green(self):
        red_raw, green_raw, _, green_ratio, *_ = (
            self._classify_solid_hue(60)
        )
        self.assertEqual(red_raw, 0)
        self.assertEqual(green_raw, 1)
        self.assertGreater(green_ratio, 0.9)

    def test_overexposed_red_core_keeps_orange_ring(self):
        frame = np.zeros((16, 24, 3), dtype=np.uint8)
        orange = tuple(int(value) for value in bgr_from_hsv(20))
        cv2.circle(frame, (8, 8), 5, orange, -1)
        cv2.circle(frame, (8, 8), 2, (255, 255, 255), -1)

        red_raw, green_raw, red_ratio, *_ = classify_signal_color(
            frame,
            self.bbox,
            minimum_red_ratio=0.004,
            minimum_green_ratio=0.004,
        )

        self.assertEqual(red_raw, 1)
        self.assertEqual(green_raw, 0)
        self.assertGreater(red_ratio, 0.05)

    def test_two_by_two_distant_red_blob_is_preserved(self):
        frame = np.zeros((17, 44, 3), dtype=np.uint8)
        frame[7:9, 20:22] = bgr_from_hsv(20)

        red_raw, green_raw, red_ratio, *_ = classify_signal_color(
            frame,
            (0, 0, 44, 17),
            minimum_red_ratio=0.004,
            minimum_green_ratio=0.004,
        )

        self.assertEqual(red_raw, 1)
        self.assertEqual(green_raw, 0)
        self.assertAlmostEqual(red_ratio, 4.0 / (44 * 17))

    def test_single_red_noise_pixel_is_below_ratio_threshold(self):
        frame = np.zeros((17, 44, 3), dtype=np.uint8)
        frame[8, 21] = bgr_from_hsv(20)

        red_raw, green_raw, *_ = classify_signal_color(
            frame,
            (0, 0, 44, 17),
            minimum_red_ratio=0.004,
            minimum_green_ratio=0.004,
        )

        self.assertEqual(red_raw, 0)
        self.assertEqual(green_raw, 0)

    def test_hue_boundaries_keep_red_yellow_green_separate(self):
        expected = {
            25: (1, 0),
            26: (0, 0),
            34: (0, 0),
            35: (0, 1),
            164: (0, 0),
            165: (1, 0),
        }
        for hue, expected_result in expected.items():
            with self.subTest(hue=hue):
                red_raw, green_raw, *_ = self._classify_solid_hue(hue)
                self.assertEqual((red_raw, green_raw), expected_result)


if __name__ == "__main__":
    unittest.main()
