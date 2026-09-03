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

    def test_hold_forward_preview_end_reverse_and_clearance_stop(self):
        controller = WallGapController(WallGapControlConfig(
            hold_s=1.0,
            preview_distance_m=1.0,
            forward_speed_mps=0.3,
            reverse_speed_mps=0.3,
            stop_clearance_m=0.2,
            require_rear_clearance=True,
        ))
        initial = Pose2(1.0, -1.0, 0.0)
        self.assertTrue(controller.start(_left_path(), initial, now_s=10.0))

        output = controller.update(initial, 10.5, rear_clearance_m=2.0)
        self.assertEqual(output.state, ControlState.HOLD)
        self.assertEqual(output.v_ref_mps, 0.0)

        output = controller.update(initial, 11.0, rear_clearance_m=2.0)
        self.assertEqual(output.state, ControlState.FORWARD)
        self.assertAlmostEqual(output.v_ref_mps, 0.3)
        self.assertAlmostEqual(output.reference_local.x, 1.0, places=6)

        # At x=3 the one-metre preview is clamped to S=(4,-1), which is the
        # explicitly requested transition trigger. The reverse preview then
        # lies one metre behind the vehicle on the same wall-parallel line.
        output = controller.update(
            Pose2(3.0, -1.0, 0.0), 12.0, rear_clearance_m=2.0)
        self.assertEqual(output.state, ControlState.REVERSE)
        self.assertAlmostEqual(output.v_ref_mps, -0.3)
        self.assertAlmostEqual(output.reference_local.x, -1.0, places=6)
        self.assertAlmostEqual(output.reference_local.y, 0.0, places=6)

        output = controller.update(
            Pose2(2.5, -1.0, 0.0), 12.1, rear_clearance_m=None)
        self.assertEqual(output.state, ControlState.REVERSE)
        self.assertEqual(output.v_ref_mps, 0.0)
        self.assertEqual(output.status, 'rear_lidar_invalid_hold')

        output = controller.update(
            Pose2(2.5, -1.0, 0.0), 12.2, rear_clearance_m=0.20)
        self.assertEqual(output.state, ControlState.STOPPED)
        self.assertEqual(output.v_ref_mps, 0.0)

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
