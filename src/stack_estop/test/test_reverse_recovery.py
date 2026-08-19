import math
import unittest
from pathlib import Path

from fma_interfaces.msg import RefPoint, TargetRef

from stack_estop.reverse_recovery import (
    REVERSE_CONFIRM_TOKEN,
    RecoveryState,
    ReverseRecoveryController,
    age_exceeds_timeout,
    analyze_rear_scan,
    build_output_target_ref,
    target_ref_has_valid_points,
    validate_parameters,
)


REAR_YAW = -1.51354952733


def inputs(**overrides):
    values = {
        'estop_active': True,
        'front_obstacle_present': True,
        'front_scan_timeout': False,
        'rear_scan_received': True,
        'rear_scan_timeout': False,
        'rear_clear': True,
        'status_fresh': True,
        'mgm_state': TargetRef.STATE_LANE,
        'avoid_status_fresh': False,
        'avoid_obstacle_detected': False,
        'avoid_avoidable': False,
        'mgm_target_fresh': False,
        'mgm_ref_points_valid': False,
    }
    values.update(overrides)
    return values


def controller(*, authorized=True, duration=0.0):
    return ReverseRecoveryController(
        reverse_wait_sec=10.0,
        reverse_speed_mps=-0.30,
        max_abs_reverse_speed_mps=0.30,
        reverse_max_duration_sec=duration,
        post_reverse_stop_hold_sec=0.5,
        actuation_authorized=authorized)


def start_reverse(instance):
    instance.update(0.0, **inputs())
    return instance.update(10.0, **inputs())


def rear_scan(distance=None, count=3):
    increment = 0.01
    sensor_angle = math.atan2(
        math.sin(math.pi - REAR_YAW), math.cos(math.pi - REAR_YAW))
    angle_min = sensor_angle - increment * (count - 1) / 2
    ranges = [] if distance is None else [distance] * count
    return analyze_rear_scan(
        ranges, angle_min, increment, 0.03, 12.0,
        rear_lidar_yaw_rad=REAR_YAW)


class ReverseRecoveryTest(unittest.TestCase):

    def test_status_stale_threshold_is_half_second(self):
        for age in (0.24, 0.30, 0.49, 0.50):
            self.assertFalse(age_exceeds_timeout(age, 0.50))
        self.assertTrue(age_exceeds_timeout(0.501, 0.50))

    def test_scan_timeout_threshold_remains_quarter_second(self):
        self.assertFalse(age_exceeds_timeout(0.25, 0.25))
        self.assertTrue(age_exceeds_timeout(0.251, 0.25))
        self.assertTrue(age_exceeds_timeout(None, 0.25))

    def test_final_actuation_confirmation_token(self):
        self.assertEqual(
            REVERSE_CONFIRM_TOKEN,
            'I_CONFIRM_REVERSE_RECOVERY_ACTUATION')

    def test_normal_mgm_passthrough(self):
        result = controller().update(0.0, **inputs(estop_active=False))
        self.assertEqual(result['state'], RecoveryState.NORMAL.value)
        self.assertIsNone(result['output_override'])
        source = TargetRef()
        source.v_ref = 0.42
        self.assertAlmostEqual(
            build_output_target_ref(source, result['output_override']).v_ref,
            0.42)

    def test_front_block_does_not_reverse_at_nine_point_nine_seconds(self):
        instance = controller()
        first = instance.update(0.0, **inputs())
        before = instance.update(9.9, **inputs())
        self.assertEqual(first['state'], RecoveryState.WAIT_REVERSE_DELAY.value)
        self.assertEqual(before['output_override'], 0.0)
        self.assertFalse(before['front_wait_completed'])

    def test_front_blocked_rear_clear_starts_reverse_at_ten_seconds(self):
        result = start_reverse(controller())
        self.assertEqual(result['state'], RecoveryState.REVERSE_ACTIVE.value)
        self.assertEqual(result['output_override'], -0.30)

    def test_front_blocked_rear_blocked_waits_for_rear(self):
        instance = controller()
        instance.update(0.0, **inputs(rear_clear=False))
        result = instance.update(10.0, **inputs(rear_clear=False))
        self.assertEqual(result['state'], RecoveryState.WAIT_REAR_CLEAR.value)
        self.assertEqual(result['output_override'], 0.0)
        self.assertTrue(result['front_wait_completed'])

    def test_rear_clear_resumes_without_another_ten_seconds(self):
        instance = controller()
        instance.update(0.0, **inputs(rear_clear=False))
        instance.update(10.0, **inputs(rear_clear=False))
        result = instance.update(10.1, **inputs(rear_clear=True))
        self.assertEqual(result['state'], RecoveryState.REVERSE_ACTIVE.value)
        self.assertEqual(result['output_override'], -0.30)

    def test_reverse_rear_block_stops_immediately(self):
        instance = controller()
        start_reverse(instance)
        result = instance.update(10.1, **inputs(rear_clear=False))
        self.assertEqual(result['state'], RecoveryState.WAIT_REAR_CLEAR.value)
        self.assertEqual(result['output_override'], 0.0)

    def test_reverse_resumes_after_rear_clears_again(self):
        instance = controller()
        start_reverse(instance)
        instance.update(10.1, **inputs(rear_clear=False))
        result = instance.update(10.2, **inputs(rear_clear=True))
        self.assertEqual(result['state'], RecoveryState.REVERSE_ACTIVE.value)
        self.assertEqual(result['output_override'], -0.30)

    def test_multiple_rear_pause_cycles_are_allowed(self):
        instance = controller()
        start_reverse(instance)
        for index in range(3):
            paused = instance.update(11.0 + index, **inputs(rear_clear=False))
            resumed = instance.update(11.1 + index, **inputs(rear_clear=True))
            self.assertEqual(paused['output_override'], 0.0)
            self.assertEqual(resumed['output_override'], -0.30)

    def test_front_clear_stops_reverse_immediately(self):
        instance = controller()
        start_reverse(instance)
        result = instance.update(10.1, **inputs(estop_active=False))
        self.assertEqual(result['state'], RecoveryState.STOP_AFTER_REVERSE.value)
        self.assertEqual(result['output_override'], 0.0)

    def test_reverse_clear_holds_stop_then_waits_for_avoidance(self):
        instance = controller()
        start_reverse(instance)
        instance.update(10.1, **inputs(estop_active=False, rear_clear=False))
        held = instance.update(10.59, **inputs(estop_active=False, rear_clear=False))
        waiting = instance.update(10.61, **inputs(estop_active=False, rear_clear=False))
        normal = instance.update(
            10.62, **inputs(
                estop_active=False, rear_clear=False,
                mgm_state=TargetRef.STATE_AVOID,
                avoid_status_fresh=True,
                avoid_obstacle_detected=True,
                avoid_avoidable=True,
                mgm_target_fresh=True,
                mgm_ref_points_valid=True))
        self.assertEqual(held['state'], RecoveryState.STOP_AFTER_REVERSE.value)
        self.assertEqual(waiting['state'], RecoveryState.WAIT_AVOIDANCE.value)
        self.assertEqual(waiting['output_override'], 0.0)
        self.assertEqual(normal['state'], RecoveryState.NORMAL.value)
        self.assertIsNone(normal['output_override'])

    def test_front_clear_ignores_rear_block_for_forward_return(self):
        instance = controller()
        instance.update(0.0, **inputs(rear_clear=False))
        instance.update(10.0, **inputs(rear_clear=False))
        instance.update(10.1, **inputs(estop_active=False, rear_clear=False))
        normal = instance.update(10.61, **inputs(estop_active=False, rear_clear=False))
        self.assertEqual(normal['state'], RecoveryState.NORMAL.value)
        self.assertTrue(normal['normal_forwarding'])

    def test_rear_pause_then_front_clear_hands_off_avoidance(self):
        instance = controller()
        start_reverse(instance)
        instance.update(10.1, **inputs(rear_clear=False))
        stopped = instance.update(
            10.2, **inputs(estop_active=False, rear_clear=False))
        waiting = instance.update(
            10.71, **inputs(estop_active=False, rear_clear=False))
        self.assertEqual(stopped['state'], RecoveryState.STOP_AFTER_REVERSE.value)
        self.assertEqual(waiting['state'], RecoveryState.WAIT_AVOIDANCE.value)
        self.assertEqual(waiting['output_override'], 0.0)

    def test_front_status_clears_before_estop_still_hands_off_avoidance(self):
        instance = controller()
        start_reverse(instance)
        stopped = instance.update(
            10.1, **inputs(front_obstacle_present=False))
        waiting = instance.update(
            10.61,
            **inputs(estop_active=False, front_obstacle_present=False))
        self.assertEqual(stopped['state'], RecoveryState.STOP_AFTER_REVERSE.value)
        self.assertEqual(waiting['state'], RecoveryState.WAIT_AVOIDANCE.value)
        self.assertEqual(waiting['output_override'], 0.0)

    def test_wait_avoidance_blocks_lane_and_waypoint(self):
        for mgm_state in (TargetRef.STATE_LANE, TargetRef.STATE_WAYPOINT):
            instance = controller()
            start_reverse(instance)
            instance.update(10.1, **inputs(estop_active=False))
            instance.update(10.61, **inputs(estop_active=False))
            result = instance.update(
                11.0, **inputs(
                    estop_active=False, mgm_state=mgm_state))
            self.assertEqual(
                result['state'], RecoveryState.WAIT_AVOIDANCE.value)
            self.assertEqual(result['output_override'], 0.0)

    def test_wait_avoidance_releases_only_for_fresh_valid_avoidance(self):
        instance = controller()
        start_reverse(instance)
        instance.update(10.1, **inputs(estop_active=False))
        instance.update(10.61, **inputs(estop_active=False))
        result = instance.update(
            10.7, **inputs(
                estop_active=False,
                mgm_state=TargetRef.STATE_AVOID,
                avoid_status_fresh=True,
                avoid_obstacle_detected=True,
                avoid_avoidable=True,
                mgm_target_fresh=True,
                mgm_ref_points_valid=True))
        self.assertEqual(result['state'], RecoveryState.NORMAL.value)
        self.assertIsNone(result['output_override'])
        source = TargetRef()
        source.state = TargetRef.STATE_AVOID
        source.v_ref = 0.47
        point = RefPoint()
        point.x = 1.25
        source.ref_points = [point]
        output = build_output_target_ref(source, result['output_override'])
        self.assertEqual(output, source)

    def test_preexisting_avoid_state_does_not_release_wait(self):
        instance = controller()
        start_reverse(instance)
        instance.update(10.1, **inputs(estop_active=False))
        instance.update(10.61, **inputs(estop_active=False))
        result = instance.update(
            10.7, **inputs(
                estop_active=False,
                mgm_state=TargetRef.STATE_AVOID,
                mgm_ref_points_valid=True))
        self.assertEqual(result['state'], RecoveryState.WAIT_AVOIDANCE.value)
        self.assertEqual(result['output_override'], 0.0)

    def test_fresh_avoid_without_fresh_mgm_does_not_release(self):
        instance = controller()
        start_reverse(instance)
        instance.update(10.1, **inputs(estop_active=False))
        instance.update(10.61, **inputs(estop_active=False))
        result = instance.update(
            10.7, **inputs(
                estop_active=False,
                avoid_status_fresh=True,
                avoid_obstacle_detected=True,
                avoid_avoidable=True,
                mgm_state=TargetRef.STATE_AVOID,
                mgm_ref_points_valid=True))
        self.assertEqual(result['state'], RecoveryState.WAIT_AVOIDANCE.value)

    def test_fresh_mgm_without_fresh_avoid_does_not_release(self):
        instance = controller()
        start_reverse(instance)
        instance.update(10.1, **inputs(estop_active=False))
        instance.update(10.61, **inputs(estop_active=False))
        result = instance.update(
            10.7, **inputs(
                estop_active=False,
                mgm_state=TargetRef.STATE_AVOID,
                mgm_target_fresh=True,
                mgm_ref_points_valid=True))
        self.assertEqual(result['state'], RecoveryState.WAIT_AVOIDANCE.value)

    def test_fresh_avoid_must_be_detected_and_avoidable(self):
        for detected, avoidable in ((True, False), (False, True)):
            instance = controller()
            start_reverse(instance)
            instance.update(10.1, **inputs(estop_active=False))
            instance.update(10.61, **inputs(estop_active=False))
            result = instance.update(
                10.7, **inputs(
                    estop_active=False,
                    avoid_status_fresh=True,
                    avoid_obstacle_detected=detected,
                    avoid_avoidable=avoidable,
                    mgm_state=TargetRef.STATE_AVOID,
                    mgm_target_fresh=True,
                    mgm_ref_points_valid=True))
            self.assertEqual(
                result['state'], RecoveryState.WAIT_AVOIDANCE.value)

    def test_empty_mgm_avoid_path_does_not_release(self):
        instance = controller()
        start_reverse(instance)
        instance.update(10.1, **inputs(estop_active=False))
        instance.update(10.61, **inputs(estop_active=False))
        result = instance.update(
            10.7, **inputs(
                estop_active=False,
                avoid_status_fresh=True,
                avoid_obstacle_detected=True,
                avoid_avoidable=True,
                mgm_state=TargetRef.STATE_AVOID,
                mgm_target_fresh=True,
                mgm_ref_points_valid=False))
        self.assertEqual(result['state'], RecoveryState.WAIT_AVOIDANCE.value)

    def test_target_ref_valid_points_require_finite_values(self):
        target = TargetRef()
        self.assertFalse(target_ref_has_valid_points(target))
        point = RefPoint()
        point.x = 1.0
        target.ref_points = [point]
        self.assertTrue(target_ref_has_valid_points(target))
        target.ref_points[0].yaw = math.nan
        self.assertFalse(target_ref_has_valid_points(target))

    def test_estop_reappears_while_waiting_avoidance_restarts_delay(self):
        instance = controller()
        start_reverse(instance)
        instance.update(10.1, **inputs(estop_active=False))
        instance.update(10.61, **inputs(estop_active=False))
        result = instance.update(10.7, **inputs(estop_active=True))
        self.assertEqual(result['state'], RecoveryState.WAIT_REVERSE_DELAY.value)
        self.assertEqual(result['output_override'], 0.0)
        self.assertFalse(result['front_wait_completed'])

    def test_clear_before_delay_returns_normal_without_avoidance(self):
        instance = controller()
        instance.update(0.0, **inputs())
        stopped = instance.update(9.9, **inputs(estop_active=False))
        normal = instance.update(10.41, **inputs(estop_active=False))
        self.assertEqual(stopped['state'], RecoveryState.STOP_AFTER_REVERSE.value)
        self.assertEqual(normal['state'], RecoveryState.NORMAL.value)
        self.assertFalse(normal['waiting_for_avoidance'])

    def test_front_timeout_enters_fault_and_does_not_auto_resume(self):
        instance = controller()
        start_reverse(instance)
        fault = instance.update(5.1, **inputs(front_scan_timeout=True))
        recovered = instance.update(6.0, **inputs())
        self.assertEqual(fault['state'], RecoveryState.FAULT_STOP.value)
        self.assertEqual(recovered['state'], RecoveryState.FAULT_STOP.value)

    def test_rear_timeout_enters_fault_and_does_not_auto_resume(self):
        instance = controller()
        start_reverse(instance)
        fault = instance.update(5.1, **inputs(rear_scan_timeout=True))
        recovered = instance.update(6.0, **inputs())
        self.assertEqual(fault['state'], RecoveryState.FAULT_STOP.value)
        self.assertEqual(recovered['state'], RecoveryState.FAULT_STOP.value)

    def test_missing_rear_scan_enters_fault(self):
        result = controller().update(
            0.0, **inputs(rear_scan_received=False))
        self.assertEqual(result['state'], RecoveryState.FAULT_STOP.value)
        self.assertEqual(result['output_override'], 0.0)

    def test_stale_status_enters_fault(self):
        result = controller().update(0.0, **inputs(status_fresh=False))
        self.assertEqual(result['state'], RecoveryState.FAULT_STOP.value)
        self.assertEqual(result['last_stop_reason'], 'STATUS_STALE')

    def test_new_episode_is_allowed_after_estop_clear(self):
        instance = controller()
        start_reverse(instance)
        instance.update(10.1, **inputs(front_scan_timeout=True))
        instance.update(11.0, **inputs(estop_active=False))
        instance.update(11.51, **inputs(estop_active=False))
        instance.update(20.0, **inputs())
        result = instance.update(30.0, **inputs())
        self.assertEqual(result['state'], RecoveryState.REVERSE_ACTIVE.value)

    def test_sensor_fault_clear_never_hands_off_avoidance(self):
        instance = controller()
        start_reverse(instance)
        instance.update(10.1, **inputs(front_scan_timeout=True))
        instance.update(10.2, **inputs(estop_active=False))
        result = instance.update(10.71, **inputs(estop_active=False))
        self.assertEqual(result['state'], RecoveryState.NORMAL.value)
        self.assertFalse(result['waiting_for_avoidance'])

    def test_sensor_fault_during_stop_hold_cancels_avoidance_handoff(self):
        instance = controller()
        start_reverse(instance)
        instance.update(10.1, **inputs(front_obstacle_present=False))
        fault = instance.update(10.2, **inputs(front_scan_timeout=True))
        instance.update(10.3, **inputs(estop_active=False))
        result = instance.update(10.81, **inputs(estop_active=False))
        self.assertEqual(fault['state'], RecoveryState.FAULT_STOP.value)
        self.assertEqual(result['state'], RecoveryState.NORMAL.value)
        self.assertFalse(result['waiting_for_avoidance'])

    def test_unauthorized_never_outputs_negative_vref(self):
        instance = controller(authorized=False)
        instance.update(0.0, **inputs())
        result = instance.update(10.0, **inputs())
        self.assertEqual(result['state'], RecoveryState.REVERSE_READY.value)
        self.assertEqual(result['output_override'], 0.0)

    def test_reverse_preserves_target_fields_and_changes_only_vref(self):
        source = TargetRef()
        source.header.frame_id = 'base_link'
        source.state = TargetRef.STATE_AVOID
        source.v_ref = 0.5
        point = RefPoint()
        point.x = 1.2
        point.y = -0.1
        point.yaw = 0.2
        point.curvature = 0.03
        source.ref_points = [point]
        output = build_output_target_ref(source, -0.30)
        self.assertIsNot(output, source)
        self.assertEqual(output.header.frame_id, source.header.frame_id)
        self.assertEqual(output.state, source.state)
        self.assertEqual(output.ref_points, source.ref_points)
        self.assertAlmostEqual(output.v_ref, -0.30)

    def test_output_builder_never_overrides_mgm_state(self):
        source = TargetRef()
        source.state = TargetRef.STATE_LANE
        source.v_ref = 0.5
        point = RefPoint()
        point.x = 1.0
        source.ref_points = [point]
        output = build_output_target_ref(source, 0.0)
        self.assertEqual(output.state, TargetRef.STATE_LANE)
        self.assertEqual(output.v_ref, 0.0)
        self.assertEqual(output.ref_points, source.ref_points)

    def test_duration_zero_never_stops_reverse_by_time(self):
        instance = controller(duration=0.0)
        start_reverse(instance)
        for now_sec in (10.3, 11.0, 20.0):
            result = instance.update(now_sec, **inputs())
            self.assertEqual(result['state'], RecoveryState.REVERSE_ACTIVE.value)
            self.assertEqual(result['output_override'], -0.30)

    def test_positive_duration_keeps_backward_compatible_limit(self):
        instance = controller(duration=2.0)
        start_reverse(instance)
        result = instance.update(12.0, **inputs())
        self.assertEqual(result['state'], RecoveryState.STOP_AFTER_REVERSE.value)
        self.assertEqual(result['last_stop_reason'], 'MAX_REVERSE_DURATION')

    def test_reverse_speed_above_limit_is_rejected(self):
        with self.assertRaises(ValueError):
            ReverseRecoveryController(
                reverse_speed_mps=-0.31,
                max_abs_reverse_speed_mps=0.30)

    def test_default_limit_rejects_point_three(self):
        with self.assertRaises(ValueError):
            ReverseRecoveryController(reverse_speed_mps=-0.30)

    def test_nonnegative_reverse_speed_is_rejected(self):
        with self.assertRaises(ValueError):
            ReverseRecoveryController(reverse_speed_mps=0.30)

    def test_parameter_validation_rejects_nonfinite_duration(self):
        with self.assertRaises(ValueError):
            validate_parameters(5.0, -0.05, 0.10, math.inf, 0.5, 0.25, 0.25, 0.25)

    def test_rear_cluster_blocks_but_single_noise_does_not(self):
        blocked = rear_scan(0.40, count=3)
        noise = rear_scan(0.40, count=1)
        self.assertTrue(blocked['valid_scan'])
        self.assertFalse(blocked['rear_clear'])
        self.assertEqual(blocked['rear_cluster_points'], 3)
        self.assertTrue(noise['rear_clear'])

    def test_invalid_rear_scan_is_not_clear(self):
        result = rear_scan(None)
        self.assertFalse(result['valid_scan'])
        self.assertFalse(result['rear_clear'])

    def test_measured_rear_yaw_maps_center_to_negative_base_x(self):
        result = rear_scan(0.50, count=3)
        self.assertLess(result['rear_nearest_x'], 0.0)
        self.assertAlmostEqual(result['rear_nearest_y'], 0.0, places=2)

    def test_internal_error_forces_fault(self):
        instance = controller()
        instance.force_internal_fault()
        result = instance.update(1.0, **inputs())
        self.assertEqual(result['state'], RecoveryState.FAULT_STOP.value)
        self.assertEqual(result['output_override'], 0.0)

    def test_launch_has_one_final_target_ref_publisher_path(self):
        launch_path = Path(__file__).parents[1] / 'launch' / (
            'REAL_VEHICLE_stack_estop_mgm_can_recovery.launch.py')
        text = launch_path.read_text(encoding='utf-8')
        self.assertIn("('/adas/target_ref', '/adas/target_ref_mgm')", text)
        self.assertEqual(text.count("executable='mgm_node'"), 1)
        self.assertEqual(text.count("executable='reverse_recovery_node'"), 1)
        self.assertEqual(text.count("executable='stack_avoid_node'"), 1)
        self.assertNotIn('reverse_recovery_request_node', text)
        self.assertNotIn('recovery_target_ref_gate', text)
        self.assertEqual(text.count("executable='can_bridge_node'"), 1)
        self.assertIn(
            "DeclareLaunchArgument('reverse_wait_sec', default_value='10.0')",
            text)


if __name__ == '__main__':
    unittest.main()
