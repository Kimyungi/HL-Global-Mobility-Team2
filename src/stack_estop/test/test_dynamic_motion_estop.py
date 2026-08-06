import math
import unittest

import numpy as np

from stack_estop.dynamic_motion_core import DynamicMotionCore
from stack_estop.node import (
    DistanceEstopController, analyze_corridor_scan, combine_estop_levels)


def environment(frame, object_x=None, object_y=None, second_object=None):
    """Base-link points with static walls and optional compact obstacles."""
    ego_advance = 0.025 * frame
    points = []
    index = 0
    for x_value in np.linspace(0.4, 5.2, 145):
        ripple = 0.10 * math.sin(1.7 * x_value)
        for y_value in (1.05 + ripple, -1.10 - 0.07 * math.cos(2.1 * x_value)):
            x_base = x_value - ego_advance
            points.append((index, x_base, y_value,
                           math.hypot(x_base, y_value)))
            index += 1
    objects = []
    if object_x is not None and object_y is not None:
        objects.append((object_x, object_y))
    if second_object is not None:
        objects.append(second_object)
    for center_x, center_y in objects:
        for y_value in np.linspace(center_y - 0.08, center_y + 0.08, 9):
            points.append((index, center_x, y_value,
                           math.hypot(center_x, y_value)))
            index += 1
    return points


def arm_core(core, start_frame=0):
    for frame in range(start_frame, start_frame + 5):
        core.process(environment(frame), frame * 0.1)


def lateral_entry(core, x_value):
    results = []
    for offset, y_value in enumerate(
            (0.70, 0.62, 0.54, 0.46, 0.38, 0.24, 0.16), start=5):
        results.append(core.process(
            environment(offset, x_value, y_value), offset * 0.1))
    return results


def far_entry_core():
    return DynamicMotionCore({
        'tracking_max_x_m': 3.00,
        'dynamic_stop_distance_m': 1.20,
    })


def far_lateral_entry(core, x_value=2.50):
    results = []
    for offset, y_value in enumerate(
            (0.60, 0.60, 0.52, 0.44, 0.36, 0.28, 0.20), start=5):
        results.append(core.process(
            environment(offset, x_value, y_value), offset * 0.1))
    return results


def dormant_boundary_entry(core):
    """Build two motion-evidence frames, lose the track, then reappear."""
    arm_core(core)
    result = None
    for frame, y_value in enumerate((0.82, 0.74, 0.66, 0.58, 0.50), 5):
        result = core.process(
            environment(frame, 2.50, y_value), frame * 0.1)
    track_id = result['candidate_track_id']
    for frame in (10, 11):
        core.process(environment(frame), frame * 0.1)
    result = core.process(environment(12, 2.50, 0.20), 1.2)
    return track_id, result


class DynamicMotionEstopTest(unittest.TestCase):

    def test_dormant_outside_track_reconnects_inside_with_history(self):
        core = far_entry_core()
        old_track_id, result = dormant_boundary_entry(core)
        self.assertEqual(result['association_reason'], 'DORMANT_RECONNECTED')
        self.assertEqual(result['reconnected_previous_track_id'], old_track_id)
        self.assertEqual(result['hazard_track_id'], old_track_id)
        self.assertEqual(result['dormant_frame_count'], 2)
        self.assertFalse(result['association_ambiguous'])
        self.assertTrue(result['candidate_outside_history'])
        self.assertTrue(result['side_entry_event'])
        self.assertTrue(result['hazard_latched'])

    def test_reconnected_hazard_stops_but_latch_holds(self):
        core = far_entry_core()
        hazard_id, result = dormant_boundary_entry(core)
        for frame in range(13, 18):
            result = core.process(environment(frame, 2.50, 0.20), frame * 0.1)
        self.assertEqual(result['hazard_track_id'], hazard_id)
        self.assertTrue(result['hazard_latched'])
        self.assertTrue(result['hazard_stopped'])
        self.assertFalse(result['dynamic_estop'])

    def test_reconnected_stopped_hazard_approach_triggers_stop(self):
        core = far_entry_core()
        _, result = dormant_boundary_entry(core)
        for frame, x_value in enumerate(
                (2.30, 2.10, 1.90, 1.70, 1.50, 1.30, 1.19), 13):
            result = core.process(
                environment(frame, x_value, 0.20), frame * 0.1)
        self.assertTrue(result['hazard_latched'])
        self.assertTrue(result['dynamic_estop'])
        self.assertTrue(combine_estop_levels(False, False, True))

    def test_ambiguous_dormant_match_does_not_merge_two_obstacles(self):
        core = far_entry_core()
        arm_core(core)
        for frame, y_value in enumerate((0.82, 0.74, 0.66, 0.58, 0.50), 5):
            result = core.process(
                environment(frame, 2.50, y_value), frame * 0.1)
        old_track_id = result['candidate_track_id']
        core.process(environment(10), 1.0)
        result = core.process(
            environment(11, 2.35, 0.20, second_object=(2.65, 0.20)), 1.1)
        self.assertTrue(result['association_ambiguous'])
        self.assertEqual(result['association_reason'], 'DORMANT_MATCH_AMBIGUOUS')
        self.assertIsNone(result['reconnected_previous_track_id'])
        self.assertFalse(result['hazard_latched'])
        self.assertGreaterEqual(len(result['tracks']), 3)
        self.assertIn(old_track_id, [track['track_id'] for track in result['tracks']])

    def test_static_inside_birth_never_receives_dormant_history(self):
        core = far_entry_core()
        arm_core(core)
        for frame in range(5, 13):
            result = core.process(environment(frame, 2.50, 0.20), frame * 0.1)
        self.assertIsNone(result['reconnected_previous_track_id'])
        self.assertFalse(result['candidate_outside_history'])
        self.assertFalse(result['hazard_latched'])

    def test_dormant_position_mismatch_creates_new_track(self):
        core = far_entry_core()
        arm_core(core)
        for frame, y_value in enumerate((0.82, 0.74, 0.66, 0.58, 0.50), 5):
            result = core.process(
                environment(frame, 2.50, y_value), frame * 0.1)
        old_track_id = result['candidate_track_id']
        core.process(environment(10), 1.0)
        result = core.process(environment(11, 1.80, 0.20), 1.1)
        self.assertIsNone(result['reconnected_previous_track_id'])
        self.assertFalse(result['hazard_latched'])
        visible_ids = [
            track['track_id'] for track in result['tracks']
            if track['missing_frames'] == 0]
        self.assertNotIn(old_track_id, visible_ids)

    def test_far_250_lateral_entry_registers_hazard(self):
        core = far_entry_core()
        arm_core(core)
        results = far_lateral_entry(core)
        self.assertTrue(any(item['side_entry_event'] for item in results))
        self.assertTrue(results[-1]['hazard_latched'])
        self.assertEqual(results[-1]['hazard_registration_type'], 'SIDE_ENTRY')
        self.assertGreater(results[-1]['dynamic_x'], 1.20)
        self.assertFalse(results[-1]['dynamic_estop'])
        self.assertEqual(results[-1]['dynamic_tracking_max_distance_m'], 3.00)

    def test_far_registered_hazard_stops_moving_but_latch_holds(self):
        core = far_entry_core()
        arm_core(core)
        far_lateral_entry(core)
        result = None
        for frame in range(12, 17):
            result = core.process(environment(frame, 2.50, 0.20), frame * 0.1)
        self.assertEqual(result['dynamic_track_count'], 0)
        self.assertTrue(result['hazard_latched'])
        self.assertTrue(result['hazard_stopped'])
        self.assertEqual(result['latch_reason'], 'REGISTERED_HAZARD_STOPPED')
        self.assertFalse(result['dynamic_estop'])

    def test_vehicle_approaches_stopped_far_hazard_at_119(self):
        core = far_entry_core()
        arm_core(core)
        far_lateral_entry(core)
        frame = 12
        result = None
        for x_value in (2.30, 2.10, 1.90, 1.70, 1.50, 1.30, 1.19):
            result = core.process(
                environment(frame, x_value, 0.20), frame * 0.1)
            frame += 1
        self.assertTrue(result['hazard_latched'])
        self.assertLessEqual(result['dynamic_x'], 1.20)
        self.assertTrue(result['dynamic_estop'])
        self.assertTrue(combine_estop_levels(
            False, False, result['dynamic_estop']))

    def test_far_250_static_inside_never_registers_dynamic_hazard(self):
        core = far_entry_core()
        arm_core(core)
        results = []
        for frame in range(5, 20):
            results.append(core.process(
                environment(frame, 2.50, 0.20), frame * 0.1))
        self.assertFalse(any(item['hazard_latched'] for item in results))
        self.assertFalse(any(item['dynamic_estop'] for item in results))
        self.assertFalse(any(item['side_entry_event'] for item in results))

    def test_far_hazard_exits_and_clear_confirmation_releases(self):
        core = far_entry_core()
        arm_core(core)
        far_lateral_entry(core)
        self.assertTrue(core.hazard_latched)
        values = []
        for frame in range(12, 17):
            result = core.process(environment(frame, 2.50, 0.60), frame * 0.1)
            values.append(result['hazard_latched'])
        self.assertEqual(values, [True, True, True, True, False])
        self.assertFalse(result['dynamic_estop'])
        self.assertIsNone(result['hazard_track_id'])

    def test_registered_hazard_track_survives_association_dropout(self):
        core = far_entry_core()
        arm_core(core)
        result = far_lateral_entry(core)[-1]
        hazard_id = result['hazard_track_id']
        for frame in range(12, 16):
            result = core.process(environment(frame), frame * 0.1)
        self.assertTrue(result['hazard_latched'])
        self.assertEqual(result['hazard_track_id'], hazard_id)
        self.assertIn(hazard_id, core.tracks)
        self.assertEqual(result['latch_reason'], 'HAZARD_TRACK_MISSING_HOLD')

    def test_static_ego_motion_at_090_is_not_dynamic(self):
        core = DynamicMotionCore()
        results = []
        for frame in range(20):
            results.append(core.process(
                environment(frame, 0.90, 0.10), frame * 0.1))
        self.assertFalse(any(item['dynamic_track_count'] for item in results))
        self.assertFalse(any(item['dynamic_estop'] for item in results))

    def test_lateral_entry_at_090_stops(self):
        core = DynamicMotionCore()
        arm_core(core)
        results = lateral_entry(core, 0.90)
        self.assertTrue(any(item['side_entry_event'] for item in results))
        self.assertTrue(any(item['hazard_latched'] for item in results))
        self.assertTrue(any(item['dynamic_estop'] for item in results))

    def test_lateral_entry_over_100_tracks_without_stop(self):
        core = DynamicMotionCore()
        arm_core(core)
        results = lateral_entry(core, 1.10)
        self.assertTrue(any(item['dynamic_track_count'] for item in results))
        self.assertTrue(any(item['hazard_latched'] for item in results))
        self.assertFalse(any(item['dynamic_estop'] for item in results))

    def test_latched_hazard_crosses_from_110_to_095(self):
        core = DynamicMotionCore()
        arm_core(core)
        lateral_entry(core, 1.10)
        result = core.process(environment(12, 0.95, 0.12), 1.2)
        self.assertTrue(result['hazard_latched'])
        self.assertTrue(result['dynamic_estop'])

    def test_single_residual_spike_and_short_motion_do_not_confirm(self):
        core = DynamicMotionCore()
        track = core._new_track({
            'centroid': np.array([0.9, 0.5]), 'centroid_x': 0.9,
            'centroid_y': 0.5, 'nearest_x': 0.9, 'min_y': 0.45,
            'max_y': 0.55, 'lateral_span': 0.1, 'point_count': 9,
        }, 0.0)
        core._update_motion_evidence(track, True, 0.0, 0.5)
        core._update_motion_evidence(track, True, -0.8, 0.4)
        self.assertFalse(track['dynamic_confirmed'])
        core._update_motion_evidence(track, True, -0.8, 0.3)
        self.assertFalse(track['dynamic_confirmed'])

    def test_moving_away_does_not_register_hazard(self):
        core = DynamicMotionCore()
        arm_core(core)
        results = []
        for offset, y_value in enumerate(
                (0.16, 0.24, 0.38, 0.46, 0.54, 0.62, 0.70), start=5):
            results.append(core.process(
                environment(offset, 0.90, y_value), offset * 0.1))
        self.assertFalse(any(item['hazard_latched'] for item in results))
        self.assertFalse(any(item['dynamic_estop'] for item in results))

    def test_icp_invalid_does_not_confirm_dynamic(self):
        core = DynamicMotionCore()
        results = []
        for frame, y_value in enumerate((0.70, 0.55, 0.40, 0.24, 0.16)):
            obstacle_only = [
                (index, 0.90, y, math.hypot(0.90, y))
                for index, y in enumerate(np.linspace(y_value - 0.08,
                                                       y_value + 0.08, 9))]
            results.append(core.process(obstacle_only, frame * 0.1))
        self.assertFalse(any(item['icp_valid'] for item in results))
        self.assertFalse(any(item['dynamic_estop'] for item in results))

    def test_two_clusters_remain_separate_tracks(self):
        core = DynamicMotionCore()
        arm_core(core)
        for frame in range(5, 9):
            result = core.process(environment(
                frame, 0.90, 0.60 - 0.05 * (frame - 5),
                second_object=(1.40, -0.60)), frame * 0.1)
        centroids = sorted(
            track['centroid_y'] for track in result['tracks'])
        self.assertGreaterEqual(len(centroids), 2)
        self.assertLess(centroids[0], -0.4)
        self.assertGreater(centroids[-1], 0.3)

    def test_one_missing_frame_holds_then_clear_releases(self):
        core = DynamicMotionCore()
        arm_core(core)
        self.assertTrue(lateral_entry(core, 0.90)[-1]['dynamic_estop'])
        one_missing = core.process(environment(12), 1.2)
        self.assertTrue(one_missing['dynamic_estop'])
        result = one_missing
        for frame in range(13, 18):
            result = core.process(environment(frame), frame * 0.1)
        self.assertFalse(result['hazard_latched'])
        self.assertFalse(result['dynamic_estop'])

    def test_dynamic_release_confirmation_is_two_frames(self):
        core = DynamicMotionCore()
        track = core._new_track({
            'centroid': np.array([0.9, 0.5]), 'centroid_x': 0.9,
            'centroid_y': 0.5, 'nearest_x': 0.9, 'min_y': 0.45,
            'max_y': 0.55, 'lateral_span': 0.1, 'point_count': 9,
        }, 0.0)
        track['dynamic_confirmed'] = True
        core._update_motion_evidence(track, False, 0.0, 0.5)
        self.assertTrue(track['dynamic_confirmed'])
        core._update_motion_evidence(track, False, 0.0, 0.5)
        self.assertFalse(track['dynamic_confirmed'])

    def test_static_and_dynamic_final_or_policy(self):
        static = DistanceEstopController()
        for _ in range(3):
            static.update_from_scan(None)
        self.assertFalse(static.current_final_estop)
        self.assertTrue(combine_estop_levels(False, False, True))
        static.update_from_scan(0.69)
        self.assertTrue(combine_estop_levels(
            False, static.current_final_estop, False))
        self.assertTrue(combine_estop_levels(True, False, False))

    def test_static_hard_stop_survives_icp_failure(self):
        static = DistanceEstopController()
        static.update_from_scan(0.69)
        core = DynamicMotionCore()
        result = core.process([], 0.0)
        self.assertFalse(result['icp_valid'])
        self.assertTrue(static.current_final_estop)

    def test_minus_90_degree_yaw_applied_once(self):
        yaw = 1.57079632679
        result = analyze_corridor_scan(
            [0.90] * 4, -math.pi / 2 - 0.015, 0.01,
            0.03, 12.0, laser_yaw_in_base_rad=yaw)
        points = result['base_points']
        self.assertAlmostEqual(points[1][1], 0.90, places=3)
        self.assertAlmostEqual(points[1][2], -0.005 * 0.90, places=3)
        core = DynamicMotionCore()
        core.process(points, 0.0)
        # Core receives base coordinates and performs no second yaw transform.
        self.assertAlmostEqual(points[1][1], 0.90, places=3)


if __name__ == '__main__':
    unittest.main()
