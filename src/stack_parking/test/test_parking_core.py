import math
import unittest

import numpy as np

from stack_parking.geometry import Pose2, between, compose, transform_points
from stack_parking.icp_slam import IcpConfig, IcpSlam
from stack_parking.mission import MissionState
from stack_parking.simulation import build_mission, simulate_once, synthetic_scene
from stack_parking.space_detector import (
    MODE_PARALLEL,
    MODE_PERPENDICULAR,
    SIDE_LEFT,
    SIDE_RIGHT,
)


class GeometryTest(unittest.TestCase):

    def test_pose_round_trip(self):
        frame = Pose2(1.2, -0.4, 0.7)
        local = Pose2(-0.3, 2.0, -0.5)
        self.assertAlmostEqual(between(frame, compose(frame, local)).x, local.x)
        self.assertAlmostEqual(between(frame, compose(frame, local)).y, local.y)
        self.assertAlmostEqual(between(frame, compose(frame, local)).yaw, local.yaw)


class IcpSlamTest(unittest.TestCase):

    def _world(self):
        return np.vstack((
            synthetic_scene(MODE_PERPENDICULAR, SIDE_RIGHT),
            np.column_stack((
                np.linspace(-2.0, 4.0, 100),
                np.linspace(-1.0, 1.3, 100),
            )),
        ))

    def test_icp_tracks_noisy_motion_with_odometry_prior(self):
        rng = np.random.default_rng(10)
        world = self._world()
        slam = IcpSlam(IcpConfig(
            min_correspondences=18,
            max_correspondence_m=0.40,
            max_rmse_m=0.18,
        ))
        accepted = 0
        truth = Pose2()
        for index in range(18):
            truth = Pose2(0.07 * index, 0.025 * math.sin(index * 0.25), 0.006 * index)
            local = transform_points(world, Pose2(
                -math.cos(truth.yaw) * truth.x - math.sin(truth.yaw) * truth.y,
                math.sin(truth.yaw) * truth.x - math.cos(truth.yaw) * truth.y,
                -truth.yaw,
            ))
            local += rng.normal(0.0, 0.004, local.shape)
            odom = Pose2(
                truth.x + 0.002 * index,
                truth.y - 0.001 * index,
                truth.yaw + 0.0005 * index,
            )
            result = slam.update(local, odom)
            accepted += int(result.accepted)
        self.assertGreaterEqual(accepted, 15)
        self.assertLess(math.hypot(slam.pose.x - truth.x, slam.pose.y - truth.y), 0.08)
        self.assertLess(abs(slam.pose.yaw - truth.yaw), math.radians(3.0))

    def test_icp_tracks_without_odometry(self):
        world = self._world()
        slam = IcpSlam(IcpConfig(
            min_correspondences=18,
            max_correspondence_m=0.40,
            max_rmse_m=0.18,
        ))
        truth = Pose2()
        accepted = 0
        for index in range(12):
            truth = Pose2(0.035 * index, 0.0, 0.003 * index)
            c = math.cos(truth.yaw)
            s = math.sin(truth.yaw)
            inverse_truth = Pose2(
                -c * truth.x - s * truth.y,
                s * truth.x - c * truth.y,
                -truth.yaw,
            )
            result = slam.update(transform_points(world, inverse_truth))
            accepted += int(result.accepted)
        self.assertGreaterEqual(accepted, 10)
        self.assertLess(math.hypot(slam.pose.x - truth.x, slam.pose.y - truth.y), 0.08)
        self.assertLess(abs(slam.pose.yaw - truth.yaw), math.radians(3.0))


class MissionSimulationTest(unittest.TestCase):

    def _planned_mission(self, mode=MODE_PERPENDICULAR, side=SIDE_RIGHT):
        mission = build_mission(stable_frames=3, wait_s=5.0)
        pose = Pose2(-1.5, 0.0, 0.0)
        mission.trigger(mode, side, pose)
        points = synthetic_scene(mode, side)
        self.assertFalse(mission.observe_map(points, pose))
        self.assertFalse(mission.observe_map(points, pose))
        self.assertTrue(mission.observe_map(points, pose))
        return mission, pose

    def test_all_parking_geometries_complete(self):
        for mode in (MODE_PARALLEL, MODE_PERPENDICULAR):
            for side in (SIDE_LEFT, SIDE_RIGHT):
                with self.subTest(mode=mode, side=side):
                    result = simulate_once(mode, side, seed=5)
                    self.assertTrue(result.success, result.reason)
                    self.assertEqual(result.final_state, MissionState.COMPLETE.value)
                    self.assertLessEqual(result.max_curvature, 1.0 / 1.15 + 1.0e-6)
                    # One 6cm path discretization step is the allowed error
                    # around the configured 1.0m preview.
                    self.assertLessEqual(result.max_preview_distance_m, 1.07)

    def test_dynamic_intrusion_stops_then_recovers(self):
        result = simulate_once(
            MODE_PERPENDICULAR, SIDE_RIGHT, seed=7, inject_dynamic=True)
        self.assertTrue(result.success, result.reason)
        self.assertTrue(result.dynamic_stop_seen)

    def test_noise_and_dropout(self):
        result = simulate_once(
            MODE_PARALLEL,
            SIDE_LEFT,
            seed=12,
            map_noise_std_m=0.012,
            dropout=0.08,
        )
        self.assertTrue(result.success, result.reason)

    def test_no_rear_wall_never_auto_completes(self):
        mission, pose = self._planned_mission()
        for _ in range(500):
            output = mission.tick(
                pose, 0.0, rear_clearance_m=None,
                vehicle_speed_mps=0.0, localization_valid=True)
            if output.v_suggest_mps != 0.0 and mission.current_path:
                target = mission.current_path[min(
                    mission.progress + 1, len(mission.current_path) - 1)]
                pose = target.pose
        self.assertEqual(mission.state, MissionState.REVERSE)
        self.assertFalse(output.done)
        self.assertEqual(output.v_suggest_mps, 0.0)
        self.assertEqual(output.status, 'planned_end_waiting_for_rear_wall')

    def test_wait_timer_requires_vehicle_stop_feedback(self):
        mission, pose = self._planned_mission()
        # Move to the analytical end and present the <=20cm wall condition.
        pose = mission.plan.reverse_path[-1].pose
        mission.state = MissionState.REVERSE
        mission._set_path(mission.plan.reverse_path)
        mission.progress = len(mission.current_path) - 1
        output = mission.tick(
            pose, 0.0, rear_clearance_m=0.19,
            vehicle_speed_mps=None, localization_valid=True)
        self.assertEqual(output.state, MissionState.PARKED_WAIT)
        for now in (1.0, 6.0, 20.0):
            output = mission.tick(
                pose, now, rear_clearance_m=0.19,
                vehicle_speed_mps=None, localization_valid=True)
        self.assertEqual(output.state, MissionState.PARKED_WAIT)
        output = mission.tick(
            pose, 21.0, rear_clearance_m=0.19,
            vehicle_speed_mps=0.0, localization_valid=True)
        self.assertEqual(output.state, MissionState.PARKED_WAIT)
        output = mission.tick(
            pose, 26.1, rear_clearance_m=0.19,
            vehicle_speed_mps=0.0, localization_valid=True)
        self.assertEqual(output.state, MissionState.EXIT)


if __name__ == '__main__':
    unittest.main()
