import math
import unittest

from stack_estop.node import (
    DistanceEstopController,
    analyze_corridor_scan,
    validate_static_min_obstacle_extent,
)


YAW = 1.57079632679


def front_cluster(distance, count=3):
    increment = 0.01
    angle_min = -math.pi / 2 - increment * (count - 1) / 2
    return analyze_corridor_scan(
        [distance] * count,
        angle_min,
        increment,
        0.03,
        12.0,
        laser_yaw_in_base_rad=YAW,
    )


def front_cluster_with_extent(distance, extent, threshold):
    half_angle = math.asin(extent / (2.0 * distance))
    return analyze_corridor_scan(
        [distance, distance, distance],
        -math.pi / 2 - half_angle,
        half_angle,
        0.03,
        12.0,
        laser_yaw_in_base_rad=YAW,
        static_min_obstacle_extent_m=threshold,
    )


class DistanceEstopTest(unittest.TestCase):
    def test_front_060_and_069_stop(self):
        for distance in (0.60, 0.69):
            result = front_cluster(distance)
            controller = DistanceEstopController()
            controller.update_from_scan(result['nearest_cluster_min_x'])
            self.assertTrue(controller.current_final_estop)
            self.assertEqual(result['cluster_sizes'], [3])

    def test_hysteresis_depends_on_previous_level(self):
        distance = front_cluster(0.75)['nearest_cluster_min_x']
        clear = DistanceEstopController()
        for _ in range(3):
            clear.update_from_scan(None)
        clear.update_from_scan(distance)
        self.assertFalse(clear.current_final_estop)

        stopped = DistanceEstopController()
        stopped.update_from_scan(distance)
        self.assertTrue(stopped.current_final_estop)

    def test_three_safe_scans_release(self):
        controller = DistanceEstopController()
        safe_x = front_cluster(0.81)['nearest_cluster_min_x']
        values = []
        for _ in range(3):
            controller.update_from_scan(safe_x)
            values.append(controller.current_final_estop)
        self.assertEqual(values, [True, True, False])

    def test_side_point_not_front_obstacle(self):
        result = analyze_corridor_scan(
            [0.20], 0.0, 0.01, 0.03, 12.0,
            laser_yaw_in_base_rad=YAW)
        self.assertIsNone(result['nearest_cluster_min_x'])
        self.assertEqual(result['reason'], 'OUTSIDE_CORRIDOR')

    def test_single_noise_point_rejected(self):
        result = front_cluster(0.30, count=1)
        self.assertIsNone(result['nearest_cluster_min_x'])
        self.assertEqual(result['reason'], 'TOO_FEW_CLUSTER_POINTS')
        controller = DistanceEstopController()
        for _ in range(3):
            controller.update_from_scan(None)
        controller.update_from_scan(result['nearest_cluster_min_x'])
        self.assertFalse(controller.current_final_estop)

    def test_timeout_forces_stop(self):
        controller = DistanceEstopController()
        for _ in range(3):
            controller.update_from_scan(None)
        self.assertFalse(controller.current_final_estop)
        controller.force_timeout()
        self.assertTrue(controller.current_final_estop)

    def test_laser_minus_90_is_base_plus_x(self):
        roi = front_cluster(0.60)['nearest_roi_point']
        self.assertAlmostEqual(roi['x'], 0.60, places=3)
        self.assertAlmostEqual(roi['y'], 0.0, delta=0.01)

    def test_extent_zero_preserves_existing_behavior(self):
        result = front_cluster_with_extent(0.50, 0.03, 0.0)
        self.assertIsNotNone(result['nearest_cluster_min_x'])
        self.assertAlmostEqual(result['nearest_candidate_extent_m'], 0.03)

    def test_small_physical_extent_is_rejected(self):
        result = front_cluster_with_extent(0.50, 0.03, 0.05)
        self.assertIsNone(result['nearest_cluster_min_x'])
        self.assertEqual(result['reason'], 'SMALL_CLUSTER_EXTENT')
        self.assertEqual(result['small_cluster_rejected_count'], 1)

    def test_extent_at_threshold_is_accepted(self):
        result = front_cluster_with_extent(0.50, 0.05, 0.05)
        self.assertIsNotNone(result['nearest_cluster_min_x'])
        self.assertAlmostEqual(result['nearest_accepted_extent_m'], 0.05)

    def test_small_near_cluster_does_not_trigger_estop(self):
        result = front_cluster_with_extent(0.50, 0.03, 0.05)
        controller = DistanceEstopController()
        for _ in range(3):
            controller.update_from_scan(None)
        controller.update_from_scan(result['nearest_cluster_min_x'])
        self.assertFalse(controller.current_final_estop)

    def test_large_near_cluster_still_triggers_estop(self):
        result = front_cluster_with_extent(0.50, 0.08, 0.05)
        controller = DistanceEstopController()
        for _ in range(3):
            controller.update_from_scan(None)
        controller.update_from_scan(result['nearest_cluster_min_x'])
        self.assertTrue(controller.current_final_estop)

    def test_small_grass_rejected_while_larger_dummy_is_used(self):
        ranges = [0.50, 0.50, 0.50] + [math.inf] * 3 + [0.60] * 5
        result = analyze_corridor_scan(
            ranges,
            -math.pi / 2 - 0.125,
            0.025,
            0.03,
            12.0,
            laser_yaw_in_base_rad=YAW,
            static_min_obstacle_extent_m=0.05,
        )
        self.assertEqual(result['small_cluster_rejected_count'], 1)
        self.assertAlmostEqual(result['nearest_candidate_distance_m'], 0.496,
                               delta=0.01)
        self.assertAlmostEqual(result['nearest_cluster_min_x'], 0.598,
                               delta=0.01)
        self.assertGreaterEqual(result['nearest_accepted_extent_m'], 0.05)

    def test_far_sparse_cluster_uses_physical_extent(self):
        result = front_cluster_with_extent(1.20, 0.08, 0.05)
        self.assertEqual(result['cluster_sizes'], [3])
        self.assertIsNotNone(result['nearest_cluster_min_x'])
        self.assertAlmostEqual(result['nearest_accepted_extent_m'], 0.08)

    def test_negative_and_unreasonable_extent_rejected(self):
        for value in (-0.01, 0.51, math.inf, math.nan):
            with self.assertRaises(ValueError):
                validate_static_min_obstacle_extent(value)

    def test_current_seven_centimetre_boundary(self):
        cases = (
            (0.050, False),
            (0.069, False),
            (0.070, True),
            (0.080, True),
        )
        for extent, expected_accepted in cases:
            with self.subTest(extent=extent):
                result = front_cluster_with_extent(0.50, extent, 0.07)
                self.assertEqual(
                    result['nearest_cluster_min_x'] is not None,
                    expected_accepted)


if __name__ == '__main__':
    unittest.main()
