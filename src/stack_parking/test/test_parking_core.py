import math
import unittest

import numpy as np

from stack_parking.geometry import Pose2, between, compose, transform_points
from stack_parking.icp_slam import IcpConfig, IcpSlam
from stack_parking.localization import (
    FrontRearCloudPairer,
    MotionPrior,
    MotionPriorConfig,
    PipelineController,
    PipelineStage,
    StampedCloud,
)
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

    def test_localization_mode_does_not_grow_map(self):
        world = self._world()
        slam = IcpSlam(IcpConfig(
            min_correspondences=18,
            max_correspondence_m=0.40,
            max_rmse_m=0.18,
        ))
        self.assertTrue(slam.update(world, Pose2()).accepted)
        cells = len(slam.map)
        result = slam.update(world, Pose2(), update_map=False)
        self.assertTrue(result.accepted, result.reason)
        self.assertEqual(len(slam.map), cells)


class FrontRearLocalizationTest(unittest.TestCase):

    @staticmethod
    def _cloud(stamp, receipt, value):
        return StampedCloud(
            stamp_s=stamp,
            receipt_s=receipt,
            frame_id='base_link',
            points=np.asarray([[value, 0.0]], dtype=np.float64),
        )

    def test_cloud_pair_is_consumed_once(self):
        pairer = FrontRearCloudPairer(max_queue=4)
        pairer.push('front', self._cloud(1.00, 1.01, 1.0))
        pairer.push('rear', self._cloud(1.03, 1.04, -1.0))
        pair = pairer.pop(1.05, sync_tolerance_s=0.05,
                          stale_timeout_s=0.30)
        self.assertIsNotNone(pair)
        self.assertAlmostEqual(pair.skew_s, 0.03)
        np.testing.assert_allclose(pair.points[:, 0], [1.0, -1.0])
        self.assertIsNone(pairer.pop(
            1.06, sync_tolerance_s=0.05, stale_timeout_s=0.30))

    def test_cloud_pair_drops_unmatchable_old_sample(self):
        pairer = FrontRearCloudPairer(max_queue=4)
        pairer.push('front', self._cloud(1.00, 1.00, 1.0))
        pairer.push('rear', self._cloud(1.20, 1.20, -1.0))
        self.assertIsNone(pairer.pop(
            1.21, sync_tolerance_s=0.05, stale_timeout_s=0.30))
        self.assertEqual(pairer.sync_drops, 1)
        pairer.push('front', self._cloud(1.21, 1.21, 2.0))
        self.assertIsNotNone(pairer.pop(
            1.22, sync_tolerance_s=0.05, stale_timeout_s=0.30))

    def test_velocity_and_imu_integrate_in_parking_map(self):
        prior = MotionPrior(MotionPriorConfig(
            velocity_timeout_s=2.0,
            imu_timeout_s=2.0,
            max_dt_s=2.0,
            max_imu_rate_rad_s=10.0,
            imu_jump_margin_rad=0.0,
        ))
        prior.update_velocity(1.0, 0.0)
        prior.update_imu(0.0, 0.0)
        prior.predict(0.0)
        prior.update_velocity(1.0, 1.0)
        prior.update_imu(math.pi / 2.0, 1.0)
        pose = prior.predict(1.0)
        self.assertAlmostEqual(pose.x, 2.0 / math.pi, places=6)
        self.assertAlmostEqual(pose.y, 2.0 / math.pi, places=6)
        self.assertAlmostEqual(pose.yaw, math.pi / 2.0, places=6)

        prior.update_velocity(-1.0, 2.0)
        prior.update_imu(math.pi / 2.0, 2.0)
        pose = prior.predict(2.0)
        self.assertAlmostEqual(pose.x, 2.0 / math.pi, places=6)
        self.assertAlmostEqual(pose.y, 2.0 / math.pi - 1.0, places=6)

    def test_vehicle_rx_steering_integrates_bicycle_yaw_without_imu(self):
        prior = MotionPrior(MotionPriorConfig(
            velocity_timeout_s=2.0,
            steering_timeout_s=2.0,
            max_dt_s=2.0,
            use_imu=False,
            use_steering=True,
            wheelbase_m=1.0,
            steering_sign=-1.0,
            steering_deadband_rad=0.0,
            max_steering_rad=math.radians(60.0),
        ))
        # dSPACE reports the opposite sign: str=-45deg is ROS left steering.
        prior.update_vehicle(1.0, -math.pi / 4.0, 0.0)
        prior.predict(0.0)
        prior.update_vehicle(1.0, -math.pi / 4.0, 1.0)
        pose = prior.predict(1.0)
        self.assertAlmostEqual(pose.x, math.sin(1.0), places=6)
        self.assertAlmostEqual(pose.y, 1.0 - math.cos(1.0), places=6)
        self.assertAlmostEqual(pose.yaw, 1.0, places=6)
        self.assertEqual(prior.last_status.source, 'vehicle_bicycle')
        self.assertTrue(prior.last_status.velocity_fresh)
        self.assertTrue(prior.last_status.steering_fresh)
        self.assertFalse(prior.last_status.imu_fresh)

    def test_vehicle_rx_reverse_changes_bicycle_yaw_direction(self):
        prior = MotionPrior(MotionPriorConfig(
            velocity_timeout_s=2.0,
            steering_timeout_s=2.0,
            max_dt_s=2.0,
            use_imu=False,
            use_steering=True,
            wheelbase_m=1.0,
            steering_sign=-1.0,
            steering_deadband_rad=0.0,
        ))
        prior.update_vehicle(-1.0, -math.pi / 4.0, 0.0)
        prior.predict(0.0)
        prior.update_vehicle(-1.0, -math.pi / 4.0, 1.0)
        pose = prior.predict(1.0)
        self.assertAlmostEqual(pose.yaw, -1.0, places=6)

    def test_vehicle_rx_steering_deadband_suppresses_center_noise(self):
        prior = MotionPrior(MotionPriorConfig(
            velocity_timeout_s=2.0,
            steering_timeout_s=2.0,
            max_dt_s=2.0,
            use_imu=False,
            use_steering=True,
            steering_deadband_rad=math.radians(0.3),
        ))
        prior.update_vehicle(1.0, math.radians(0.2), 0.0)
        prior.predict(0.0)
        prior.update_vehicle(1.0, math.radians(0.2), 1.0)
        pose = prior.predict(1.0)
        self.assertAlmostEqual(pose.x, 1.0, places=6)
        self.assertAlmostEqual(pose.y, 0.0, places=6)
        self.assertAlmostEqual(pose.yaw, 0.0, places=6)

    def test_gps_delta_is_composed_in_previous_vehicle_frame(self):
        prior = MotionPrior(MotionPriorConfig(
            max_dt_s=1.0,
            gps_position_gain=1.0,
            gps_innovation_gate_m=10.0,
            gps_max_correction_m=10.0,
        ))
        prior.predict(0.0)
        self.assertTrue(prior.update_gps(1, 0.0, 0.0, 0.0, 4, True))
        self.assertTrue(prior.update_gps(
            2, 1.0, 0.0, math.pi / 2.0, 4, True))
        pose = prior.predict(0.1)
        self.assertAlmostEqual(pose.x, 1.0, places=6)
        self.assertAlmostEqual(pose.y, 0.0, places=6)
        self.assertAlmostEqual(pose.yaw, math.pi / 2.0, places=6)

        self.assertTrue(prior.update_gps(3, 1.0, 0.0, 0.0, 4, True))
        pose = prior.predict(0.2)
        self.assertAlmostEqual(pose.x, 1.0, places=6)
        self.assertAlmostEqual(pose.y, 1.0, places=6)
        self.assertAlmostEqual(pose.yaw, math.pi / 2.0, places=6)

    def test_only_rtk_fixed_is_accepted(self):
        prior = MotionPrior()
        self.assertFalse(prior.update_gps(1, 0.0, 0.0, 0.0, 5, True))
        self.assertTrue(prior.update_gps(1, 0.0, 0.0, 0.0, 4, True))

    def test_pipeline_freezes_map_before_parking(self):
        pipeline = PipelineController(
            slam_confirm_scans=2,
            localization_confirm_scans=2,
            minimum_map_points=5,
        )
        self.assertEqual(pipeline.stage, PipelineStage.SLAM)
        self.assertFalse(pipeline.observe_slam(True, 5))
        self.assertTrue(pipeline.observe_slam(True, 5))
        self.assertEqual(pipeline.stage, PipelineStage.MAPPING)
        self.assertTrue(pipeline.mapping_enabled)
        self.assertTrue(pipeline.plan_ready())
        self.assertEqual(pipeline.stage, PipelineStage.LOCALIZATION)
        self.assertFalse(pipeline.mapping_enabled)
        self.assertFalse(pipeline.observe_slam(True, 5))
        self.assertTrue(pipeline.observe_slam(True, 5))
        self.assertEqual(pipeline.stage, PipelineStage.PARKING)
        self.assertTrue(pipeline.parking_enabled)


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
