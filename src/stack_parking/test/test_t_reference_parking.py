"""No ROS, hardware, pytest, or CAN required: python -m unittest discover ..."""
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from stack_parking.geometry import PathPoint, Pose2
from stack_parking.t_reference_parking import (
    Candidate, Config, Scan, TwoReferenceParking, inspect_candidate,
    load_reverse_csv, rear_observation, scan_from_ranges,
)


def path(name, side):
    t = np.linspace(0, 4, 41)
    y = side * .8 * (1 - np.cos(np.pi * np.minimum(t, 2) / 2)) / 2
    yaw = -np.arctan(np.gradient(y, t))
    s = np.r_[0., np.cumsum(np.hypot(np.diff(t), np.diff(y)))]
    k = -np.gradient(yaw, s)
    return Candidate(name, tuple(PathPoint(-x, z, a, b, -1)
                                for x, z, a, b in zip(t, y, yaw, k)), s)


def left(stamp, obstacle=None):
    # Synthetic full-circle ray fixture tests geometry, NOT the real left FOV.
    angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    pts = np.c_[12 * np.cos(angles), .21 + 12 * np.sin(angles)]
    if obstacle is not None:
        pts = np.vstack((pts, obstacle))
    return Scan(stamp, pts, (0., .21))


def rear(stamp, distance=2.):
    return Scan(stamp, np.c_[np.full(21, -.11-distance), np.linspace(-.35, .35, 21)], (-.11, 0.))


class ReferenceParkingTests(unittest.TestCase):
    def setUp(self):
        # Synthetic S curves intentionally use a small radius to exercise FSM.
        # Production Config reports the repository's 1.15m reference threshold.
        self.cfg = Config(min_radius=.1)
        self.paths = (path('ref1', 1), path('ref2', -1))
        self.core = TwoReferenceParking(self.paths, self.cfg)
        self.core.trigger()

    def tick(self, now, **changes):
        args = dict(pose=Pose2(0, 0, self.paths[0].path[0].yaw),
                    pose_stamp=now, speed=0., speed_stamp=now,
                    left=left(now), rear=rear(now), parking_owned=True)
        args.update(changes)
        return self.core.tick(now, **args)

    def select(self, obstacle=None):
        for t in (0., .6, .7, .8):
            output = self.tick(t, left=left(t, obstacle))
        return output

    def near_end(self):
        self.select()
        self.core.index = len(self.paths[0].path) - 4
        p = self.paths[0].path[self.core.index]
        return Pose2(p.x, p.y, p.yaw)

    def test_no_motion_before_actual_stop(self):
        for t in (0., 1., 2.):
            out = self.tick(t, speed=.2)
            self.assertEqual(out.phase, 'STOPPING')
            self.assertEqual(out.v_suggest, 0)

    def test_owner_ack_required(self):
        self.assertEqual(self.tick(1, parking_owned=False).phase, 'STOPPING')

    def test_blocked_first_selects_second(self):
        out = self.select([[-3, .8]])
        self.assertEqual(out.selected, 'ref2')
        self.assertEqual(out.phase, 'REVERSE')

    def test_blocked_second_selects_first(self):
        self.assertEqual(self.select([[-3, -.8]]).selected, 'ref1')

    def test_both_observed_clear_tie_is_deterministic(self):
        self.assertEqual(self.select().selected, 'ref1')

    def test_both_blocked_stay_stopped(self):
        out = self.select([[-3, .8], [-3, -.8]])
        self.assertIsNone(out.selected)
        self.assertEqual(out.v_suggest, 0)

    def test_common_start_obstacle_blocks_both(self):
        self.assertIsNone(self.select([[-.1, 0]]).selected)

    def test_empty_scan_does_not_mean_free(self):
        out = self.tick(1, left=Scan(1, np.empty((0, 2)), (0, .21)))
        self.assertIsNone(out.selected)

    def test_same_scan_cannot_cast_multiple_votes(self):
        self.tick(0)
        for t in (.6, .61, .62, .63):
            out = self.tick(t, left=left(.6))
        self.assertIsNone(out.selected)
        self.assertEqual(self.core.votes, 1)

    def test_start_alignment_is_required(self):
        for t in (0, .6, .7, .8):
            out = self.tick(t, pose=Pose2(1, 0, 0))
        self.assertIsNone(out.selected)

    def test_curvature_rejection_only_when_explicitly_enabled(self):
        self.core = TwoReferenceParking(
            self.paths, Config(min_radius=100, enforce_min_radius=True))
        self.core.trigger()
        out = self.select()
        self.assertEqual(out.reason, 'csv_curvature_exceeds_vehicle_limit')

    def test_curvature_warning_does_not_block_default_selection(self):
        self.core = TwoReferenceParking(self.paths, Config(min_radius=100))
        self.core.trigger()
        out = self.select()
        self.assertEqual(out.phase, 'REVERSE')
        self.assertEqual(out.selected, 'ref1')
        self.assertEqual(len(out.warnings), 2)
        self.assertIn('tracking_requires_validation', out.warnings[0])
        moving = self.tick(.9)
        self.assertLess(moving.v_suggest, 0)
        self.assertEqual(moving.warnings, out.warnings)

    def test_negative_speed_and_body_yaw(self):
        self.select()
        out = self.tick(.9)
        self.assertLess(out.v_suggest, 0)
        self.assertFalse(out.request_stop)
        self.assertLess(out.reference.x, 0)

    def test_selected_path_remains_locked_despite_new_obstruction(self):
        self.select()
        out = self.tick(.9, left=left(.9, [[-.7, .25]]))
        self.assertEqual(out.selected, 'ref1')
        self.assertEqual(out.phase, 'REVERSE')
        self.assertLess(out.v_suggest, 0)

    def test_stale_sensor_does_not_latch_motion_fault(self):
        self.select()
        out = self.tick(1.5, rear=rear(.8))
        self.assertEqual(out.phase, 'REVERSE')
        self.assertLess(out.v_suggest, 0)

    def test_feedback_age_does_not_stop_selected_reverse(self):
        self.select()
        self.assertLess(self.tick(1., speed_stamp=2.).v_suggest, 0)

    def test_empty_rear_sector_keeps_selected_reverse_path(self):
        self.select()
        bad = Scan(.9, np.c_[np.ones(21), np.linspace(-.3, .3, 21)], (-.11, 0))
        out = self.tick(.9, rear=bad)
        self.assertEqual(out.phase, 'REVERSE')
        self.assertEqual(out.selected, 'ref1')
        self.assertLess(out.v_suggest, 0)
        self.assertIsNotNone(out.reference)
        self.assertFalse(out.parking_success)

    def test_wall_50cm_stops_then_confirms_stationary_success(self):
        pose = self.near_end()
        out = self.tick(.9, pose=pose, rear=rear(.9, .49), speed=-.15)
        self.assertEqual(out.phase, 'WALL_STOP')
        self.assertEqual(out.v_suggest, 0)
        for t in (1., 1.1, 1.2, 1.6):
            out = self.tick(t, pose=pose, rear=rear(t, .49))
        self.assertTrue(out.parking_success)
        self.assertEqual(out.v_suggest, 0)
        self.assertTrue(out.request_stop)
        self.core.trigger()
        self.assertEqual(self.tick(2.).phase, 'SUCCESS')

    def test_50cm_wall_before_docking_does_not_finish_or_stop(self):
        self.select()
        out = self.tick(.9, rear=rear(.9, .49))
        self.assertEqual(out.phase, 'REVERSE')
        self.assertLess(out.v_suggest, 0)
        self.assertFalse(out.parking_success)

    def test_single_near_return_is_not_wall_success_or_local_stop(self):
        pose = self.near_end()
        far = rear(.9)
        cone = Scan(.9, np.vstack((far.points, [-.5, 0.])), far.origin)
        out = self.tick(.9, pose=pose, rear=cone)
        self.assertEqual(out.phase, 'REVERSE')
        self.assertLess(out.v_suggest, 0)
        self.assertFalse(out.parking_success)

    def test_no_wall_at_csv_end_is_not_success(self):
        pose = self.near_end()
        p = self.paths[0].path[-1]
        self.core.index = len(self.paths[0].path)-1
        out = self.tick(.9, pose=Pose2(p.x, p.y, p.yaw))
        self.assertEqual(out.reason, 'path_end_without_rear_wall')

    def test_estop_holds_without_changing_selected_path(self):
        self.select()
        out = self.tick(.9, estop=True)
        self.assertEqual(out.phase, 'REVERSE')
        self.assertEqual(out.v_suggest, 0)
        self.assertLess(self.tick(1.).v_suggest, 0)

    def test_position_error_alone_keeps_selected_reverse_path(self):
        self.select()
        out = self.tick(.9, pose=Pose2(2, 2, 0))
        self.assertEqual(out.phase, 'REVERSE')
        self.assertLess(out.v_suggest, 0)
        self.assertEqual(out.selected, 'ref1')

    def test_wall_confirmation_needs_new_frames(self):
        pose = self.near_end()
        self.tick(.9, pose=pose, rear=rear(.9, .49))
        for t in (1., 1.1, 1.2):
            out = self.tick(t, pose=pose, rear=rear(.9, .49))
        self.assertFalse(out.parking_success)
        self.assertEqual(self.core.wall_votes, 1)

    def test_wall_does_not_succeed_while_vehicle_is_moving(self):
        pose = self.near_end()
        for t in (.9, 1., 1.2, 1.5, 1.8):
            out = self.tick(t, pose=pose, rear=rear(t, .49), speed=-.1)
        self.assertFalse(out.parking_success)
        self.assertEqual(out.v_suggest, 0)

    def test_occluded_rays_do_not_supply_clearance(self):
        angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
        scan = Scan(0, np.c_[.1*np.cos(angles), .21+.1*np.sin(angles)], (0., .21))
        _, observed = inspect_candidate(self.paths[0], Pose2(), scan, self.cfg, 20)
        self.assertEqual(observed, 0)


class SensorAndCsvTests(unittest.TestCase):
    def test_malformed_candidate_rejected(self):
        with self.assertRaises(ValueError):
            Candidate('bad', (PathPoint(0, 0, 0, 0, -1),), np.array([0.]))

    def test_reverse_yaw_and_curvature_sign(self):
        with tempfile.TemporaryDirectory() as folder:
            file = Path(folder) / 'path.csv'
            # Test fixture, not a rewrite of user waypoints.
            file.write_text('east_m,north_m,yaw_rad\n0,0,0\n1,0,0.1\n2,0,0.2\n', encoding='utf-8')
            p = load_reverse_csv(file)
        self.assertAlmostEqual(abs(p.path[0].yaw), math.pi)
        self.assertAlmostEqual(p.path[0].curvature, -.1)
        self.assertEqual(p.path[0].gear, -1)
        self.assertLessEqual(max(np.diff(p.s)), .0500001)

    def test_scan_extrinsic_offset_and_invalid_returns(self):
        scan = scan_from_ranges(1, [1.069, math.inf, math.nan, 50], 0, .1,
                                .15, 12, Pose2(.2, .3, math.pi/2), .069, 0, 1)
        np.testing.assert_allclose(scan.points, [[.2, 1.3]])
        self.assertEqual(scan.origin, (.2, .3))

    def test_wall_distance_is_sensor_relative(self):
        near, wall = rear_observation(rear(0, .5), Config())
        self.assertAlmostEqual(near, .5)
        self.assertAlmostEqual(wall, .5)

    def test_narrow_cluster_is_not_wall(self):
        scan = Scan(0, np.c_[np.full(20, -.6), np.linspace(-.03, .03, 20)], (-.1, 0))
        self.assertIsNone(rear_observation(scan, Config())[1])


if __name__ == '__main__':
    unittest.main()
