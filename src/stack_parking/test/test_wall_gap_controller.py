import math
import unittest

import numpy as np

from stack_parking.geometry import Pose2
from stack_parking.reference_path import ReferencePath
from stack_parking.wall_gap_controller import (
    ControlState,
    PoseDeltaTracker,
    WallGapControlConfig,
    WallGapController,
    controller_paths,
    mirrored_forward_exit_path,
)


def _left_path() -> ReferencePath:
    center = np.array([1.0, 0.0])
    angles = np.linspace(-math.pi / 2.0, -math.pi, 25)
    arc = np.column_stack((
        center[0] + np.cos(angles),
        center[1] + np.sin(angles),
    ))
    return ReferencePath(
        side='left',
        radius_m=1.0,
        p0_map=(0.0, 0.0),
        center_map=(1.0, 0.0),
        e_map=(1.0, -1.0),
        goal_map=(0.0, 2.0),
        straight1_map=np.asarray([[4.0, -1.0], [1.0, -1.0]]),
        arc_map=arc,
        straight2_map=np.asarray([[0.0, 0.0], [0.0, 2.0]]),
    )


def _right_path() -> ReferencePath:
    center = np.array([1.0, 0.0])
    angles = np.linspace(math.pi / 2.0, math.pi, 25)
    arc = np.column_stack((
        center[0] + np.cos(angles),
        center[1] + np.sin(angles),
    ))
    return ReferencePath(
        side='right',
        radius_m=1.0,
        p0_map=(0.0, 0.0),
        center_map=(1.0, 0.0),
        e_map=(1.0, 1.0),
        goal_map=(0.0, -2.0),
        straight1_map=np.asarray([[4.0, 1.0], [1.0, 1.0]]),
        arc_map=arc,
        straight2_map=np.asarray([[0.0, 0.0], [0.0, -2.0]]),
    )


class WallGapControllerTest(unittest.TestCase):

    def test_path_has_forward_and_reverse_vehicle_pose_conventions(self):
        forward, reverse = controller_paths(_left_path(), step_m=0.05)
        self.assertAlmostEqual(forward[0].x, 1.0)
        self.assertAlmostEqual(forward[-1].x, 4.0)
        self.assertTrue(all(point.gear == 1 for point in forward))
        self.assertAlmostEqual(reverse[0].x, 4.0)
        self.assertAlmostEqual(reverse[-1].y, 2.0)
        self.assertTrue(all(point.gear == -1 for point in reverse))
        curved = [point for point in reverse if abs(point.curvature) > 0.5]
        self.assertTrue(curved)
        self.assertTrue(all(point.curvature > 0.0 for point in curved))
        self.assertAlmostEqual(curved[-1].yaw, -math.pi / 2.0, places=6)

        _, right_reverse = controller_paths(_right_path(), step_m=0.05)
        right_curve = [
            point for point in right_reverse if abs(point.curvature) > 0.5]
        self.assertTrue(all(point.curvature < 0.0 for point in right_curve))
        self.assertAlmostEqual(right_curve[-1].yaw, math.pi / 2.0, places=6)

    def test_no_detection_hold_then_two_direction_change_holds(self):
        controller = WallGapController(WallGapControlConfig(
            direction_change_hold_s=1.0,
            preview_distance_m=1.0,
            forward_speed_mps=0.3,
            reverse_speed_mps=0.3,
        ))
        initial = Pose2(1.0, -1.0, 0.0)
        self.assertTrue(controller.start(_left_path(), initial, now_s=10.0))

        # Detection starts the forward alignment immediately: no old
        # square-confirmation hold remains.
        output = controller.update(initial, 10.0)
        self.assertEqual(output.state, ControlState.FORWARD)
        self.assertAlmostEqual(output.v_ref_mps, 0.3)
        self.assertAlmostEqual(output.reference_local.x, 1.0, places=6)

        # At x=3 the one-metre preview is clamped to S=(4,-1), which is the
        # explicitly requested transition trigger. The reverse preview then
        # lies one metre behind the vehicle on the same wall-parallel line.
        align_pose = Pose2(3.0, -1.0, 0.0)
        output = controller.update(align_pose, 11.0)
        self.assertEqual(output.state, ControlState.ALIGN_HOLD)
        self.assertEqual(output.v_ref_mps, 0.0)

        output = controller.update(align_pose, 11.99)
        self.assertEqual(output.state, ControlState.ALIGN_HOLD)
        self.assertEqual(output.v_ref_mps, 0.0)

        output = controller.update(align_pose, 12.0)
        self.assertEqual(output.state, ControlState.REVERSE)
        self.assertAlmostEqual(output.v_ref_mps, -0.3)
        self.assertAlmostEqual(output.reference_local.x, -1.0, places=6)
        self.assertAlmostEqual(output.reference_local.y, 0.0, places=6)

        # Still mid-path — keeps reversing, nothing holds or stops it early.
        output = controller.update(Pose2(2.5, -1.0, 0.0), 12.1)
        self.assertEqual(output.state, ControlState.REVERSE)
        self.assertAlmostEqual(output.v_ref_mps, -0.3)

        # Reaching the reverse path's end draws the mirrored exit path and
        # holds before the forward pull-out.
        parked_pose = Pose2(0.0, 2.0, -math.pi / 2.0)
        output = controller.update(parked_pose, 13.0)
        self.assertEqual(output.state, ControlState.PARKED_HOLD)
        self.assertEqual(output.v_ref_mps, 0.0)
        self.assertIsNotNone(controller.exit_reference_path)
        self.assertTrue(controller.exit_path)

        output = controller.update(parked_pose, 13.99)
        self.assertEqual(output.state, ControlState.PARKED_HOLD)
        self.assertEqual(output.v_ref_mps, 0.0)

        output = controller.update(parked_pose, 14.0)
        self.assertEqual(output.state, ControlState.FORWARD_EXIT)
        self.assertEqual(output.v_ref_mps, 0.3)

        # The vehicle is still one preview distance short of the path end.
        # Stopping is triggered by the preview reaching end, not by the
        # vehicle pose itself arriving there.
        exit_end = controller.exit_path[-1]
        output = controller.update(
            Pose2(-3.0, -1.0, exit_end.yaw), 15.0)
        self.assertEqual(output.state, ControlState.STOPPED)
        self.assertEqual(output.v_ref_mps, 0.0)
        self.assertEqual(output.status, 'exit_path_end_stop')
        self.assertAlmostEqual(output.reference_map.x, exit_end.x, places=6)
        self.assertAlmostEqual(output.reference_map.y, exit_end.y, places=6)

    def test_forward_exit_is_mirrored_about_inside_straight(self):
        mirrored, exit_path = mirrored_forward_exit_path(
            _left_path(), step_m=0.05)
        self.assertIsNotNone(mirrored)
        np.testing.assert_allclose(
            mirrored.straight1_map,
            np.asarray([[-4.0, -1.0], [-1.0, -1.0]]),
            atol=1.0e-9,
        )
        np.testing.assert_allclose(mirrored.goal_map, [0.0, 2.0])
        self.assertAlmostEqual(exit_path[0].x, 0.0)
        self.assertAlmostEqual(exit_path[0].y, 2.0)
        self.assertAlmostEqual(exit_path[-1].x, -4.0)
        self.assertAlmostEqual(exit_path[-1].y, -1.0)
        self.assertTrue(all(point.gear == 1 for point in exit_path))

    def test_lidar_pose_delta_is_previous_vehicle_frame_and_held(self):
        tracker = PoseDeltaTracker()
        first = tracker.reset(Pose2(2.0, 3.0, math.pi / 2.0))
        self.assertEqual(first.update, 1)
        self.assertEqual((first.dx, first.dy, first.dyaw), (0.0, 0.0, 0.0))

        # Map +y is previous vehicle +x at yaw=+90deg.
        second = tracker.update(Pose2(2.0, 3.2, math.pi / 2.0 + 0.1))
        self.assertEqual(second.update, 2)
        self.assertAlmostEqual(second.dx, 0.2, places=9)
        self.assertAlmostEqual(second.dy, 0.0, places=9)
        self.assertAlmostEqual(second.dyaw, 0.1, places=9)
        self.assertIs(tracker.delta, second)


if __name__ == '__main__':
    unittest.main()
