import math
import unittest

import numpy as np

from stack_parking.geometry import Pose2
from stack_parking.parallel_parking import (
    ParallelControlState,
    ParallelParkingConfig,
    ParallelParkingController,
    build_parallel_reference_path,
    candidate_rectangle_corners,
    parallel_controller_paths,
    parallel_opposite_single_arc_path,
    parallel_single_arc_paths,
    rectangle_is_clear,
)
from stack_parking.wall_gap_detector import TrackedCandidate


def _candidate() -> TrackedCandidate:
    return TrackedCandidate(
        side='left',
        map_x=0.0,
        map_y=0.0,
        width_m=1.5,
        near_distance=1.0,
        wall_tangent_x=1.0,
        wall_tangent_y=0.0,
        wall_normal_x=0.0,
        wall_normal_y=1.0,
        tested=True,
        clear=True,
    )


def _path():
    path = build_parallel_reference_path(
        _candidate(), Pose2(), turn_radius_m=2.0,
        end_straight_m=1.5, arc_angle_deg=50.0,
        arc_start_offset_m=0.5, arc_clockwise_offset_m=0.25)
    if path is None:
        raise AssertionError('parallel path was not built')
    return path


def _pose(point):
    return Pose2(point.x, point.y, point.yaw)


class ParallelParkingGeometryTest(unittest.TestCase):

    def test_validation_rectangle_is_1p5_by_0p7_at_gap_midpoint(self):
        candidate = _candidate()
        corners = candidate_rectangle_corners(candidate, 1.5, 0.7)
        np.testing.assert_allclose(corners, np.asarray([
            [-0.75, 0.0],
            [0.75, 0.0],
            [0.75, 0.7],
            [-0.75, 0.7],
            [-0.75, 0.0],
        ]))
        np.testing.assert_allclose(
            0.5 * (corners[0] + corners[1]),
            [candidate.map_x, candidate.map_y],
        )
        self.assertTrue(rectangle_is_clear(
            np.asarray([[0.8, 0.5], [0.0, 0.8]]), 0.0, 1.5, 0.7))
        # This point lies outside the 0.7m square precheck but inside the 1.5m
        # wall-parallel span, so the rectangle must reject it.
        self.assertFalse(rectangle_is_clear(
            np.asarray([[0.7, 0.5]]), 0.0, 1.5, 0.7))

    def test_radius_2p0_arcs_are_symmetric_about_shifted_origin(self):
        path = _path()
        p0 = np.asarray(path.p0_map)
        arc_origin = np.asarray(path.arc_origin_map)
        front_center = np.asarray(path.front_center_map)
        np.testing.assert_allclose(arc_origin, p0 + [0.5, -0.25])
        center_direction = (
            (front_center - arc_origin)
            / np.linalg.norm(front_center - arc_origin))
        self.assertAlmostEqual(
            center_direction[0], math.sin(math.radians(50.0)), places=9)
        self.assertAlmostEqual(
            center_direction[1], math.cos(math.radians(50.0)), places=9)

        front_radii = np.linalg.norm(
            path.front_arc_map - front_center, axis=1)
        rear_radii = np.linalg.norm(
            path.rear_arc_map - np.asarray(path.rear_center_map), axis=1)
        np.testing.assert_allclose(front_radii, 2.0, atol=1.0e-9)
        np.testing.assert_allclose(rear_radii, 2.0, atol=1.0e-9)
        np.testing.assert_allclose(
            path.rear_arc_map,
            2.0 * arc_origin - path.front_arc_map[::-1],
            atol=1.0e-9,
        )
        np.testing.assert_allclose(path.front_arc_map[0], arc_origin)
        np.testing.assert_allclose(path.rear_arc_map[-1], arc_origin)
        self.assertAlmostEqual(
            np.linalg.norm(path.front_line_map[1] - path.front_line_map[0]),
            1.5,
            places=9,
        )
        self.assertAlmostEqual(
            np.linalg.norm(path.rear_line_map[1] - path.rear_line_map[0]),
            1.5,
            places=9,
        )
        # At both outer arc ends the radius is normal to the wall, therefore
        # the tangent and attached 1.5-metre line are wall-parallel.
        front_radius = (
            np.asarray(path.front_tangent_map) - front_center)
        self.assertAlmostEqual(float(np.dot(front_radius, [1.0, 0.0])), 0.0)

        forward, reverse = parallel_controller_paths(path)
        self.assertTrue(all(point.gear == 1 for point in forward))
        self.assertTrue(all(point.gear == -1 for point in reverse))
        curve_signs = {math.copysign(1.0, point.curvature)
                       for point in forward if abs(point.curvature) > 1.0e-6}
        self.assertEqual(curve_signs, {-1.0, 1.0})

        single_forward, single_reverse = parallel_single_arc_paths(path)
        self.assertTrue(all(point.gear == 1 for point in single_forward))
        self.assertTrue(all(point.gear == -1 for point in single_reverse))
        self.assertAlmostEqual(math.hypot(
            single_forward[-1].x - path.front_tangent_map[0],
            single_forward[-1].y - path.front_tangent_map[1],
        ), 2.0, places=9)
        np.testing.assert_allclose(
            [single_reverse[-1].x, single_reverse[-1].y],
            [single_forward[0].x, single_forward[0].y],
        )
        self.assertAlmostEqual(math.hypot(
            single_forward[0].x - path.arc_origin_map[0],
            single_forward[0].y - path.arc_origin_map[1],
        ), 2.0, places=9)
        single_curve_signs = {
            math.copysign(1.0, point.curvature)
            for point in single_forward if abs(point.curvature) > 1.0e-6}
        self.assertEqual(len(single_curve_signs), 1)

        opposite_forward = parallel_opposite_single_arc_path(path)
        self.assertTrue(opposite_forward)
        origin = np.asarray(path.arc_origin_map)
        source_points = np.asarray([
            [point.x, point.y] for point in single_reverse])
        opposite_points = np.asarray([
            [point.x, point.y] for point in opposite_forward])
        np.testing.assert_allclose(
            opposite_points,
            2.0 * origin - source_points,
            atol=1.0e-9,
        )
        opposite_arc_points = np.asarray([
            [point.x, point.y] for point in opposite_forward
            if abs(point.curvature) > 1.0e-6])
        np.testing.assert_allclose(
            opposite_arc_points, path.rear_arc_map[:-1], atol=1.0e-9)
        opposite_curve_signs = {
            math.copysign(1.0, point.curvature)
            for point in opposite_forward if abs(point.curvature) > 1.0e-6}
        self.assertEqual(len(opposite_curve_signs), 1)
        self.assertNotEqual(single_curve_signs, opposite_curve_signs)
        self.assertAlmostEqual(
            sum(math.hypot(current.x - previous.x,
                           current.y - previous.y)
                for previous, current in zip(
                    opposite_forward, opposite_forward[1:])),
            4.0 + 2.0 * math.radians(50.0),
            places=3,
        )

    def test_arc_origin_uses_wall_slope_and_vehicle_only_selects_direction(self):
        angle = math.radians(6.0)
        wall_tangent = np.asarray([math.cos(angle), math.sin(angle)])
        clockwise = np.asarray([wall_tangent[1], -wall_tangent[0]])
        candidate = TrackedCandidate(
            side='left',
            map_x=2.0,
            map_y=3.0,
            width_m=1.5,
            near_distance=1.0,
            wall_tangent_x=float(wall_tangent[0]),
            wall_tangent_y=float(wall_tangent[1]),
            wall_normal_x=float(-wall_tangent[1]),
            wall_normal_y=float(wall_tangent[0]),
            tested=True,
            clear=True,
        )
        # Vehicle yaw differs from the wall angle. It must select only the
        # sign, while the 0.5m offset retains the fitted wall slope.
        path = build_parallel_reference_path(
            candidate, Pose2(yaw=math.radians(18.0)))
        self.assertIsNotNone(path)
        np.testing.assert_allclose(
            np.asarray(path.arc_origin_map) - np.asarray(path.p0_map),
            0.5 * wall_tangent + 0.25 * clockwise,
            atol=1.0e-9,
        )

        reverse_direction_path = build_parallel_reference_path(
            candidate, Pose2(yaw=math.radians(186.0)))
        self.assertIsNotNone(reverse_direction_path)
        np.testing.assert_allclose(
            np.asarray(reverse_direction_path.arc_origin_map)
            - np.asarray(reverse_direction_path.p0_map),
            -0.5 * wall_tangent - 0.25 * clockwise,
            atol=1.0e-9,
        )


class ParallelParkingControllerTest(unittest.TestCase):

    def test_inner_entry_extension_and_phase4_straight_are_shortened(self):
        controller = ParallelParkingController(ParallelParkingConfig(
            entry_straight_m=2.0,
            entry_inner_straight_m=1.0,
            reference_reverse_end_trim_m=1.0,
        ))
        path = _path()
        full_forward, full_reverse = parallel_controller_paths(path)
        self.assertTrue(controller.start(
            path, _pose(full_forward[0]), now_s=0.0))

        entry_start = controller.single_arc_reverse_path[0]
        entry_end = controller.single_arc_reverse_path[-1]
        self.assertAlmostEqual(math.hypot(
            entry_start.x - path.front_tangent_map[0],
            entry_start.y - path.front_tangent_map[1]), 2.0, places=9)
        self.assertAlmostEqual(math.hypot(
            entry_end.x - path.arc_origin_map[0],
            entry_end.y - path.arc_origin_map[1]), 1.0, places=9)

        full_length = sum(math.hypot(
            current.x - previous.x, current.y - previous.y)
            for previous, current in zip(full_reverse, full_reverse[1:]))
        phase4_length = controller.reference_reverse_lengths[-1]
        self.assertAlmostEqual(full_length - phase4_length, 1.0, places=9)

    def test_all_motion_arcs_use_the_same_symmetric_radius(self):
        controller = ParallelParkingController()
        path = _path()
        full_forward, _ = parallel_controller_paths(path)
        self.assertTrue(controller.start(
            path, _pose(full_forward[0]), now_s=0.0))

        expected = {round(1.0 / path.radius_m, 9)}
        for phase_path in (
                controller.initial_reference_forward_path,
                controller.single_arc_reverse_path,
                controller.opposite_arc_forward_path,
                controller.reference_reverse_path,
                controller.reference_forward_path):
            curvatures = {
                round(abs(point.curvature), 9)
                for point in phase_path
                if abs(point.curvature) > 1.0e-6
            }
            self.assertEqual(curvatures, expected)

    def test_five_motion_sequence_has_five_one_second_holds(self):
        controller = ParallelParkingController(ParallelParkingConfig(
            direction_change_hold_s=1.0,
            preview_distance_m=1.0,
            forward_speed_mps=0.75,
            reverse_speed_mps=0.75,
        ))
        path = _path()
        start = min(
            controller_paths_for_test(path),
            key=lambda point: math.hypot(
                point.x - path.p0_map[0], point.y - path.p0_map[1]),
        )
        self.assertTrue(controller.start(path, _pose(start), now_s=0.0))
        np.testing.assert_allclose(
            [[point.x, point.y]
             for point in controller.initial_reference_forward_path],
            [[point.x, point.y]
             for point in controller.reference_forward_path],
            atol=1.0e-9,
        )

        output = controller.update(_pose(start), 0.0)
        self.assertEqual(
            output.state, ParallelControlState.INITIAL_FORWARD)
        self.assertEqual(output.v_ref_mps, 0.75)

        front_trigger = controller.initial_reference_forward_path[-1]
        output = controller.update(_pose(front_trigger), 1.0)
        self.assertEqual(
            output.state, ParallelControlState.INITIAL_FORWARD_HOLD)
        self.assertEqual(output.v_ref_mps, 0.0)
        self.assertAlmostEqual(
            output.reference_map.x,
            controller.initial_reference_forward_path[-1].x,
            places=6,
        )

        output = controller.update(_pose(front_trigger), 2.0)
        self.assertEqual(
            output.state, ParallelControlState.SINGLE_ARC_REVERSE)
        self.assertEqual(output.v_ref_mps, -0.75)

        single_reverse_trigger = controller.single_arc_reverse_path[-1]
        output = controller.update(_pose(single_reverse_trigger), 3.0)
        self.assertEqual(
            output.state, ParallelControlState.SINGLE_ARC_REVERSE_HOLD)
        self.assertEqual(output.v_ref_mps, 0.0)

        output = controller.update(_pose(single_reverse_trigger), 4.0)
        self.assertEqual(
            output.state, ParallelControlState.OPPOSITE_ARC_FORWARD)
        self.assertEqual(output.v_ref_mps, 0.75)

        single_forward_trigger = controller.opposite_arc_forward_path[-1]
        output = controller.update(_pose(single_forward_trigger), 5.0)
        self.assertEqual(
            output.state, ParallelControlState.OPPOSITE_ARC_FORWARD_HOLD)
        self.assertEqual(output.v_ref_mps, 0.0)

        output = controller.update(_pose(single_forward_trigger), 6.0)
        self.assertEqual(
            output.state, ParallelControlState.REFERENCE_REVERSE)
        self.assertEqual(output.v_ref_mps, -0.75)

        reference_reverse_trigger = controller.reference_reverse_path[-1]
        output = controller.update(_pose(reference_reverse_trigger), 7.0)
        self.assertEqual(
            output.state, ParallelControlState.REFERENCE_REVERSE_HOLD)
        self.assertEqual(output.v_ref_mps, 0.0)

        output = controller.update(_pose(reference_reverse_trigger), 8.0)
        self.assertEqual(
            output.state, ParallelControlState.REFERENCE_FORWARD)
        self.assertEqual(output.v_ref_mps, 0.75)

        final_forward_trigger = controller.reference_forward_path[-1]
        output = controller.update(_pose(final_forward_trigger), 9.0)
        self.assertEqual(output.state, ParallelControlState.FINAL_HOLD)
        self.assertEqual(output.v_ref_mps, 0.0)
        output = controller.update(_pose(final_forward_trigger), 9.99)
        self.assertEqual(output.state, ParallelControlState.FINAL_HOLD)
        output = controller.update(_pose(final_forward_trigger), 10.0)
        self.assertEqual(output.state, ParallelControlState.STOPPED)
        self.assertEqual(output.v_ref_mps, 0.0)
        self.assertEqual(output.status, 'parallel_parking_complete')


def controller_paths_for_test(path):
    return parallel_controller_paths(path)[0]


if __name__ == '__main__':
    unittest.main()
