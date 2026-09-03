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
        _candidate(), Pose2(), turn_radius_m=1.12,
        end_straight_m=2.0, arc_angle_deg=45.0,
        arc_start_offset_m=0.25)
    if path is None:
        raise AssertionError('parallel path was not built')
    return path


def _pose(point):
    return Pose2(point.x, point.y, point.yaw)


def _preview_end_trigger(path, cumulative, preview_m):
    for index, length in enumerate(cumulative):
        if cumulative[-1] - length <= preview_m + 1.0e-9:
            return path[index]
    return path[-1]


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

    def test_radius_1p12_arcs_are_symmetric_about_shifted_origin(self):
        path = _path()
        p0 = np.asarray(path.p0_map)
        arc_origin = np.asarray(path.arc_origin_map)
        front_center = np.asarray(path.front_center_map)
        np.testing.assert_allclose(arc_origin, p0 + [0.25, 0.0])
        center_direction = (
            (front_center - arc_origin)
            / np.linalg.norm(front_center - arc_origin))
        self.assertAlmostEqual(center_direction[0], math.sqrt(0.5), places=9)
        self.assertAlmostEqual(center_direction[1], math.sqrt(0.5), places=9)

        front_radii = np.linalg.norm(
            path.front_arc_map - front_center, axis=1)
        rear_radii = np.linalg.norm(
            path.rear_arc_map - np.asarray(path.rear_center_map), axis=1)
        np.testing.assert_allclose(front_radii, 1.12, atol=1.0e-9)
        np.testing.assert_allclose(rear_radii, 1.12, atol=1.0e-9)
        np.testing.assert_allclose(
            path.rear_arc_map,
            2.0 * arc_origin - path.front_arc_map[::-1],
            atol=1.0e-9,
        )
        np.testing.assert_allclose(path.front_arc_map[0], arc_origin)
        np.testing.assert_allclose(path.rear_arc_map[-1], arc_origin)
        self.assertAlmostEqual(
            np.linalg.norm(path.front_line_map[1] - path.front_line_map[0]),
            2.0,
            places=9,
        )
        self.assertAlmostEqual(
            np.linalg.norm(path.rear_line_map[1] - path.rear_line_map[0]),
            2.0,
            places=9,
        )
        # At both outer arc ends the radius is normal to the wall, therefore
        # the tangent and attached two-metre line are wall-parallel.
        front_radius = (
            np.asarray(path.front_tangent_map) - front_center)
        self.assertAlmostEqual(float(np.dot(front_radius, [1.0, 0.0])), 0.0)

        forward, reverse = parallel_controller_paths(path)
        self.assertTrue(all(point.gear == 1 for point in forward))
        self.assertTrue(all(point.gear == -1 for point in reverse))
        curve_signs = {math.copysign(1.0, point.curvature)
                       for point in forward if abs(point.curvature) > 1.0e-6}
        self.assertEqual(curve_signs, {-1.0, 1.0})

    def test_arc_origin_uses_wall_slope_and_vehicle_only_selects_direction(self):
        angle = math.radians(6.0)
        wall_tangent = np.asarray([math.cos(angle), math.sin(angle)])
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
        # sign, while the 0.25m offset retains the fitted wall slope.
        path = build_parallel_reference_path(
            candidate, Pose2(yaw=math.radians(18.0)))
        self.assertIsNotNone(path)
        np.testing.assert_allclose(
            np.asarray(path.arc_origin_map) - np.asarray(path.p0_map),
            0.25 * wall_tangent,
            atol=1.0e-9,
        )

        reverse_direction_path = build_parallel_reference_path(
            candidate, Pose2(yaw=math.radians(186.0)))
        self.assertIsNotNone(reverse_direction_path)
        np.testing.assert_allclose(
            np.asarray(reverse_direction_path.arc_origin_map)
            - np.asarray(reverse_direction_path.p0_map),
            -0.25 * wall_tangent,
            atol=1.0e-9,
        )


class ParallelParkingControllerTest(unittest.TestCase):

    def test_forward_reverse_same_path_forward_three_one_second_holds(self):
        controller = ParallelParkingController(ParallelParkingConfig(
            direction_change_hold_s=1.0,
            preview_distance_m=1.5,
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

        output = controller.update(_pose(start), 0.0)
        self.assertEqual(output.state, ParallelControlState.FORWARD)
        self.assertEqual(output.v_ref_mps, 0.75)

        front_trigger = _preview_end_trigger(
            controller.forward_path,
            controller.forward_lengths,
            controller.config.preview_distance_m,
        )
        output = controller.update(_pose(front_trigger), 1.0)
        self.assertEqual(output.state, ParallelControlState.FRONT_HOLD)
        self.assertEqual(output.v_ref_mps, 0.0)
        self.assertAlmostEqual(
            output.reference_map.x, controller.forward_path[-1].x, places=6)

        output = controller.update(_pose(front_trigger), 2.0)
        self.assertEqual(output.state, ParallelControlState.REVERSE)
        self.assertEqual(output.v_ref_mps, -0.75)

        rear_trigger = _preview_end_trigger(
            controller.reverse_path,
            controller.reverse_lengths,
            controller.config.preview_distance_m,
        )
        output = controller.update(_pose(rear_trigger), 3.0)
        self.assertEqual(output.state, ParallelControlState.REAR_HOLD)
        self.assertEqual(output.v_ref_mps, 0.0)

        output = controller.update(_pose(rear_trigger), 4.0)
        self.assertEqual(output.state, ParallelControlState.FORWARD_RETURN)
        self.assertEqual(output.v_ref_mps, 0.75)

        output = controller.update(_pose(front_trigger), 5.0)
        self.assertEqual(output.state, ParallelControlState.FINAL_HOLD)
        self.assertEqual(output.v_ref_mps, 0.0)
        output = controller.update(_pose(front_trigger), 5.99)
        self.assertEqual(output.state, ParallelControlState.FINAL_HOLD)
        output = controller.update(_pose(front_trigger), 6.0)
        self.assertEqual(output.state, ParallelControlState.STOPPED)
        self.assertEqual(output.v_ref_mps, 0.0)
        self.assertEqual(output.status, 'parallel_parking_complete')


def controller_paths_for_test(path):
    return parallel_controller_paths(path)[0]


if __name__ == '__main__':
    unittest.main()
