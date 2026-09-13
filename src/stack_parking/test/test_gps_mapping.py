"""Vehicle/GPS pose independence and nearby endpoint expiry regressions."""

import math
import unittest
from unittest.mock import patch

import numpy as np

from stack_parking.geometry import Pose2, transform_points
from stack_parking.icp_slam import IcpConfig, IcpSlam, VoxelPointMap
from stack_parking.localization import MotionPrior, MotionPriorConfig


class GpsMappingTest(unittest.TestCase):
    def make_slam(self, **kwargs):
        return IcpSlam(IcpConfig(
            map_correction_enabled=False, unobserved_delete_misses=5, **kwargs))

    def test_ground_returns_cannot_correct_vehicle_gps_pose(self):
        slam = self.make_slam()
        ground = np.array([[1.0, 0.0], [1.0, 0.5]])
        # Ground returns stay attached to the vehicle despite its motion.
        with patch.object(slam.map, 'nearest', side_effect=AssertionError('ICP ran')):
            for pose in (Pose2(1.0, 0.0, 0.2), Pose2(2.0, 0.0, -0.3)):
                result = slam.update(ground, pose)
                self.assertTrue(result.accepted)
                self.assertEqual(result.pose, pose)
                self.assertEqual(result.iterations, 0)

    def test_empty_scan_updates_pose_and_expires_nearby_map(self):
        slam = self.make_slam()
        slam.update(np.array([[1.0, 0.0], [4.2, 0.0]]), Pose2())
        for i in range(5):
            result = slam.update(np.empty((0, 2)), Pose2(0.1, 0.0, 0.0))
            self.assertTrue(result.accepted)
            self.assertEqual(len(slam.map), 2 if i < 4 else 1)
        np.testing.assert_allclose(slam.map_points(), [[4.2, 0.0]])

    def test_frozen_map_is_unchanged_while_pose_moves(self):
        slam = self.make_slam()
        slam.update(np.array([[1.0, 0.0]]), Pose2())
        before = {key: cell.copy() for key, cell in slam.map._cells.items()}
        for _ in range(5):
            result = slam.update(np.array([[2.0, 0.0]]), Pose2(1.0, 0.0, 0.4),
                                 update_map=False)
        self.assertEqual(result.pose, Pose2(1.0, 0.0, 0.4))
        self.assertEqual(before, slam.map._cells)

    def test_missing_or_invalid_prior_cannot_mutate_map(self):
        slam = self.make_slam()
        scan = np.array([[1.0, 0.0]])
        slam.update(scan, Pose2())
        for prior in (None, Pose2(math.nan, 0.0, 0.0)):
            result = slam.update(np.empty((0, 2)), prior)
            self.assertFalse(result.accepted)
            self.assertEqual(len(slam.map), 1)
            self.assertEqual(slam.pose, Pose2())

    def test_downsampling_does_not_count_as_missing_observations(self):
        slam = self.make_slam(max_scan_points=1)
        point = np.array([[1.0, 0.0]])
        slam.update(point, Pose2())
        # Polar subsampling keeps only the negative-bearing endpoint.
        scan = np.array([[1.0, -1.0], [1.0, 0.0], [1.0, 1.0]])
        for _ in range(5):
            slam.update(scan, Pose2())
        self.assertTrue(np.any(np.linalg.norm(slam.map_points() - point, axis=1) < 0.01))


class UnobservedPointTest(unittest.TestCase):
    def clear(self, point_map, scan, pose=Pose2(), misses=5):
        return point_map.clear_freespace(
            pose, np.asarray(scan).reshape((-1, 2)), 4.0,
            math.radians(1.0), 0.02, observation_match_radius_m=0.02,
            unobserved_delete_misses=misses)

    def test_empty_and_occluded_bins_expire_both_occupancy_states(self):
        for confirmed in (False, True):
            for scan in ([], [[0.5, 0.0]]):
                with self.subTest(confirmed=confirmed, scan=scan):
                    point_map = VoxelPointMap(0.08)
                    for _ in range(3 if confirmed else 1):
                        point_map.add(np.array([[1.0, 0.0]]))
                    for _ in range(4):
                        self.assertEqual(self.clear(point_map, scan), 0)
                    self.assertEqual(self.clear(point_map, scan), 1)

    def test_nearby_hit_resets_consecutive_misses_across_voxel_boundary(self):
        point_map = VoxelPointMap(0.08)
        point_map.add(np.array([[1.039, 0.0]]))
        for _ in range(4):
            self.clear(point_map, [])
        self.clear(point_map, [[1.041, 0.0]])
        for _ in range(4):
            self.assertEqual(self.clear(point_map, []), 0)
        self.assertEqual(self.clear(point_map, []), 1)

    def test_four_meter_boundary_is_relative_to_vehicle_position(self):
        pose = Pose2(5.0, -2.0, math.pi / 2)
        point_map = VoxelPointMap(0.01)
        points = transform_points(np.array([[4.0, 0.0], [4.02, 0.0]]), pose)
        point_map.add(points)
        self.assertEqual(self.clear(point_map, [], pose, misses=1), 1)
        np.testing.assert_allclose(point_map.points(), points[1:])

    def test_same_voxel_point_outside_match_radius_is_a_miss(self):
        point_map = VoxelPointMap(0.08)
        point_map.add(np.array([[1.041, 0.001]]))
        self.assertEqual(self.clear(point_map, [[1.119, 0.079]], misses=1), 1)


class GpsContinuityTest(unittest.TestCase):
    def test_vehicle_prediction_is_corrected_by_each_new_gps_delta_once(self):
        prior = MotionPrior(MotionPriorConfig(
            use_imu=False, use_steering=True, gps_position_gain=0.5))
        prior.update_vehicle(1.0, 0.0, 0.0)
        prior.predict(0.0)
        prior.update_gps(1, 0.0, 0.0, 0.0, 4, True)
        prior.update_gps(2, 0.2, 0.0, 0.0, 4, True)
        self.assertAlmostEqual(prior.predict(0.1).x, 0.15)
        self.assertTrue(prior.last_status.gps_corrected)
        self.assertFalse(prior.update_gps(2, 0.2, 0.0, 0.0, 4, True))
        self.assertAlmostEqual(prior.predict(0.2).x, 0.25)
        self.assertFalse(prior.last_status.gps_corrected)

    def test_quality_loss_or_counter_gap_reanchors_without_replaying_delta(self):
        for quality_loss in (False, True):
            with self.subTest(quality_loss=quality_loss):
                prior = MotionPrior(MotionPriorConfig(gps_position_gain=1.0))
                prior.predict(0.0)
                prior.update_gps(1, 0.0, 0.0, 0.0, 4, True)
                if quality_loss:
                    self.assertFalse(prior.update_gps(2, 0.5, 0.0, 0.0, 5, True))
                self.assertTrue(prior.update_gps(3, 1.0, 0.0, 0.0, 4, True))
                self.assertEqual(prior.predict(0.1), Pose2())
                prior.update_gps(4, 0.1, 0.0, 0.0, 4, True)
                self.assertAlmostEqual(prior.predict(0.2).x, 0.1)


if __name__ == '__main__':
    unittest.main()
