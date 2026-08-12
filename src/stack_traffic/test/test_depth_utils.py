import math
import unittest

import numpy as np

from stack_traffic.depth_utils import (
    measure_stopline_depth,
)


class TestStopLineDepth(unittest.TestCase):
    def test_inverse_depth_fit_estimates_near_edge_and_ignores_paint(self):
        height, width = 100, 120
        rows = np.arange(height, dtype=np.float32)
        expected_z = 1.0 / (0.12 + 0.002 * rows)
        depth = np.repeat(expected_z[:, None], width, axis=1) * 1000.0
        depth = depth.astype(np.float32)
        paint_mask = np.zeros((height, width), dtype=np.uint8)
        paint_mask[48:53, 10:110] = 255
        depth[48:53, 10:110] = 0.0
        depth[35, 30:60] = 18000.0

        result = measure_stopline_depth(
            depth_mm=depth,
            sample_bbox=(20, 30, 100, 70),
            target_y_px=52.0,
            exclusion_mask=paint_mask,
            minimum_depth_m=0.3,
            maximum_depth_m=10.0,
            minimum_valid_ratio=0.50,
            minimum_valid_pixels=100,
            minimum_valid_rows=8,
            maximum_row_depth_mad_m=0.20,
            coherence_absolute_tolerance_m=0.20,
            coherence_relative_tolerance=0.08,
            minimum_coherent_pixel_ratio=0.60,
            minimum_inverse_depth_slope_per_px=0.0001,
            maximum_inverse_depth_slope_per_px=0.02,
            maximum_fit_residual_m=0.25,
        )

        self.assertTrue(result.accepted)
        self.assertAlmostEqual(result.camera_z_m, expected_z[52], places=2)
        self.assertGreaterEqual(result.fit_inlier_row_count, 30)
        self.assertLess(result.fit_residual_m, 0.02)

    def test_sparse_depth_is_not_accepted(self):
        depth = np.zeros((40, 60), dtype=np.uint16)
        depth[20, 20:25] = 1500

        result = measure_stopline_depth(
            depth_mm=depth,
            sample_bbox=(10, 10, 50, 30),
            target_y_px=20.0,
            exclusion_mask=None,
            minimum_depth_m=0.3,
            maximum_depth_m=10.0,
            minimum_valid_ratio=0.10,
            minimum_valid_pixels=20,
            minimum_valid_rows=4,
            maximum_row_depth_mad_m=0.20,
            coherence_absolute_tolerance_m=0.20,
            coherence_relative_tolerance=0.08,
            minimum_coherent_pixel_ratio=0.60,
            minimum_inverse_depth_slope_per_px=0.0001,
            maximum_inverse_depth_slope_per_px=0.02,
            maximum_fit_residual_m=0.25,
        )

        self.assertFalse(result.accepted)
        self.assertEqual(result.valid_row_count, 1)

    def test_wrong_mask_shape_returns_invalid_measurement(self):
        result = measure_stopline_depth(
            depth_mm=np.full((20, 30), 1500, dtype=np.uint16),
            sample_bbox=(0, 0, 30, 20),
            target_y_px=10.0,
            exclusion_mask=np.zeros((10, 10), dtype=np.uint8),
            minimum_depth_m=0.3,
            maximum_depth_m=10.0,
            minimum_valid_ratio=0.10,
            minimum_valid_pixels=20,
            minimum_valid_rows=4,
            maximum_row_depth_mad_m=0.20,
            coherence_absolute_tolerance_m=0.20,
            coherence_relative_tolerance=0.08,
            minimum_coherent_pixel_ratio=0.60,
            minimum_inverse_depth_slope_per_px=0.0001,
            maximum_inverse_depth_slope_per_px=0.02,
            maximum_fit_residual_m=0.25,
        )

        self.assertFalse(result.accepted)
        self.assertTrue(math.isnan(result.camera_z_m))

    def test_constant_vertical_surface_is_rejected_as_non_ground(self):
        result = measure_stopline_depth(
            depth_mm=np.full((60, 80), 1500, dtype=np.uint16),
            sample_bbox=(10, 10, 70, 50),
            target_y_px=30.0,
            exclusion_mask=None,
            minimum_depth_m=0.3,
            maximum_depth_m=10.0,
            minimum_valid_ratio=0.50,
            minimum_valid_pixels=100,
            minimum_valid_rows=8,
            maximum_row_depth_mad_m=0.20,
            coherence_absolute_tolerance_m=0.20,
            coherence_relative_tolerance=0.08,
            minimum_coherent_pixel_ratio=0.60,
            minimum_inverse_depth_slope_per_px=0.0001,
            maximum_inverse_depth_slope_per_px=0.02,
            maximum_fit_residual_m=0.25,
        )

        self.assertFalse(result.accepted)
        self.assertAlmostEqual(result.inverse_depth_slope_per_px, 0.0)

    def test_inconsistent_depth_rows_are_rejected_by_fit_residual(self):
        depth = np.full((60, 80), 1000, dtype=np.uint16)
        depth[1::2] = 9000
        result = measure_stopline_depth(
            depth_mm=depth,
            sample_bbox=(10, 10, 70, 50),
            target_y_px=30.0,
            exclusion_mask=None,
            minimum_depth_m=0.3,
            maximum_depth_m=10.0,
            minimum_valid_ratio=0.50,
            minimum_valid_pixels=100,
            minimum_valid_rows=8,
            maximum_row_depth_mad_m=0.20,
            coherence_absolute_tolerance_m=0.20,
            coherence_relative_tolerance=0.08,
            minimum_coherent_pixel_ratio=0.60,
            minimum_inverse_depth_slope_per_px=0.0001,
            maximum_inverse_depth_slope_per_px=0.02,
            maximum_fit_residual_m=0.25,
        )

        self.assertFalse(result.accepted)
        self.assertGreater(result.fit_residual_m, 0.25)

    def test_random_spatial_depth_is_rejected_by_coherence(self):
        random_generator = np.random.default_rng(42)
        depth = random_generator.uniform(
            300.0,
            10000.0,
            size=(33, 250),
        ).astype(np.float32)
        result = measure_stopline_depth(
            depth_mm=depth,
            sample_bbox=(0, 0, 250, 33),
            target_y_px=16.0,
            exclusion_mask=None,
            minimum_depth_m=0.3,
            maximum_depth_m=10.0,
            minimum_valid_ratio=0.50,
            minimum_valid_pixels=100,
            minimum_valid_rows=8,
            maximum_row_depth_mad_m=0.20,
            coherence_absolute_tolerance_m=0.20,
            coherence_relative_tolerance=0.08,
            minimum_coherent_pixel_ratio=0.60,
            minimum_inverse_depth_slope_per_px=0.0001,
            maximum_inverse_depth_slope_per_px=0.02,
            maximum_fit_residual_m=0.25,
        )

        self.assertFalse(result.accepted)
        self.assertLess(result.coherent_pixel_ratio, 0.60)
        self.assertEqual(result.coherent_row_count, 0)


if __name__ == "__main__":
    unittest.main()
