import math
import unittest

import numpy as np

from stack_parking.geometry import Pose2
from stack_parking.reference_path import build_reference_path
from stack_parking.wall_gap_detector import (
    SIDE_LEFT,
    WallGapConfig,
    WallGapDetector,
    candidate_square_corners,
)


class FixedReferenceWallTest(unittest.TestCase):

    @staticmethod
    def _scene(angle_deg=12.0, with_clutter=True):
        angle = math.radians(angle_deg)
        tangent = np.array([math.cos(angle), math.sin(angle)])
        normal = np.array([-math.sin(angle), math.cos(angle)])
        anchor = normal * 1.0
        first_s = np.linspace(-0.7, 0.8, 35)
        second_s = np.linspace(2.2, 4.0, 40)
        wall = np.vstack((
            anchor + np.outer(first_s, tangent),
            anchor + np.outer(second_s, tangent),
        ))
        # Parallel clutter outside the +/-12cm reference offset must not be
        # joined into either wall segment.
        clutter_s = np.linspace(0.8, 2.2, 30)
        clutter = anchor + np.outer(clutter_s, tangent) + 0.30 * normal
        points = np.vstack((wall, clutter)) if with_clutter else wall
        return points, anchor, tangent, normal

    @staticmethod
    def _detector():
        return WallGapDetector(WallGapConfig(
            search_sides=(SIDE_LEFT,),
            initial_wall_max_angle_deg=30.0,
        ))

    def test_first_wall_slope_and_offset_band_lock_in_map_frame(self):
        points, anchor, tangent, _ = self._scene()
        detector = self._detector()
        detector.set_seed(Pose2(), SIDE_LEFT)

        truth_coordinates = np.column_stack((
            (points - anchor) @ tangent,
            (points - anchor) @ np.array([-tangent[1], tangent[0]]),
        ))
        # Acquire from only the first wall beside the starting point. The
        # later segment and the gap do not need to exist yet.
        first_wall = points[
            (truth_coordinates[:, 0] <= 0.8 + 1.0e-9)
            & (np.abs(truth_coordinates[:, 1]) <= 0.12)]
        detector.update(first_wall, Pose2())
        reference_before = detector.reference_walls[SIDE_LEFT]
        detector.update(points, Pose2())
        endpoints_before = [
            detector.segment_map_points(SIDE_LEFT, segment).copy()
            for segment in detector.last_segments[SIDE_LEFT]
        ]
        self.assertEqual(len(endpoints_before), 2)
        self.assertAlmostEqual(reference_before.yaw, math.radians(12.0), places=6)
        np.testing.assert_allclose(
            [reference_before.anchor_x, reference_before.anchor_y],
            anchor,
            atol=1.0e-9,
        )

        # A large vehicle rotation used to rotate the wall markers because
        # every segment was reconstructed in the live vehicle frame.
        detector.update(points, Pose2(0.5, -0.2, math.radians(75.0)))
        self.assertEqual(detector.reference_walls[SIDE_LEFT], reference_before)
        endpoints_after = [
            detector.segment_map_points(SIDE_LEFT, segment)
            for segment in detector.last_segments[SIDE_LEFT]
        ]
        for before, after in zip(endpoints_before, endpoints_after):
            np.testing.assert_allclose(after, before, atol=1.0e-12)
        np.testing.assert_allclose(
            [reference_before.tangent_x, reference_before.tangent_y],
            tangent,
            atol=1.0e-9,
        )

    def test_candidate_and_square_do_not_follow_vehicle_yaw(self):
        points, anchor, tangent, normal = self._scene(with_clutter=False)
        detector = self._detector()
        detector.set_seed(Pose2(), SIDE_LEFT)
        detector.update(points, Pose2())
        self.assertEqual(len(detector.tracked), 1)
        candidate = detector.tracked[0]
        candidate_position = np.array([candidate.map_x, candidate.map_y])
        np.testing.assert_allclose(
            candidate_position, anchor + 1.5 * tangent, atol=1.0e-9)

        vehicle_position = 1.5 * tangent
        cleared = detector.update(points, Pose2(
            float(vehicle_position[0]), float(vehicle_position[1]),
            math.radians(80.0),
        ))
        self.assertIs(cleared, candidate)
        self.assertTrue(candidate.clear)

        corners_before = candidate_square_corners(candidate, detector.config)
        detector.update(points, Pose2(3.0, -2.0, math.radians(-55.0)))
        corners_after = candidate_square_corners(candidate, detector.config)
        np.testing.assert_allclose(corners_after, corners_before, atol=1.0e-12)
        np.testing.assert_allclose(
            corners_before[2] - corners_before[1], normal, atol=1.0e-9)

    def test_reference_path_stays_anchored_to_locked_wall(self):
        points, _, tangent, normal = self._scene(with_clutter=False)
        detector = self._detector()
        detector.set_seed(Pose2(), SIDE_LEFT)
        detector.update(points, Pose2())
        candidate = detector.tracked[0]
        vehicle_position = -1.0 * tangent
        vehicle_pose = Pose2(
            float(vehicle_position[0]), float(vehicle_position[1]),
            math.radians(65.0),
        )

        path = build_reference_path(
            candidate, vehicle_pose, min_turn_radius_m=1.15)
        self.assertIsNotNone(path)
        np.testing.assert_allclose(
            path.p0_map, [candidate.map_x, candidate.map_y], atol=1.0e-12)
        np.testing.assert_allclose(
            path.goal_map,
            np.array([candidate.map_x, candidate.map_y]) + 1.5 * normal,
            atol=1.0e-9,
        )
        self.assertAlmostEqual(
            float(np.linalg.norm(
                path.straight1_map[1] - path.straight1_map[0])),
            1.5,
            places=9,
        )
        self.assertAlmostEqual(
            float(np.linalg.norm(
                path.straight2_map[1] - path.straight2_map[0])),
            1.5,
            places=9,
        )
        # Both transitions are tangent-continuous: the outer line is wall
        # parallel and the inner line follows the inward wall normal.
        parallel_direction = (
            path.straight1_map[1] - path.straight1_map[0]) / 1.5
        inside_direction = (
            path.straight2_map[1] - path.straight2_map[0]) / 1.5
        self.assertAlmostEqual(abs(float(np.dot(parallel_direction, tangent))), 1.0)
        np.testing.assert_allclose(inside_direction, normal, atol=1.0e-9)
        np.testing.assert_allclose(path.arc_map[0], path.e_map, atol=1.0e-9)
        np.testing.assert_allclose(path.arc_map[-1], path.p0_map, atol=1.0e-9)
        radii = np.linalg.norm(
            path.arc_map - np.asarray(path.center_map), axis=1)
        np.testing.assert_allclose(radii, 1.15, atol=1.0e-9)
        outer_radius = np.asarray(path.e_map) - np.asarray(path.center_map)
        inner_radius = np.asarray(path.p0_map) - np.asarray(path.center_map)
        self.assertAlmostEqual(
            float(np.dot(outer_radius, parallel_direction)), 0.0, places=9)
        self.assertAlmostEqual(
            float(np.dot(inner_radius, inside_direction)), 0.0, places=9)
        self.assertAlmostEqual(
            float(np.dot(outer_radius, inner_radius)), 0.0, places=9)


if __name__ == '__main__':
    unittest.main()
