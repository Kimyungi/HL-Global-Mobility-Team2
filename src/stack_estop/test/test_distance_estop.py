import math
import unittest
from types import SimpleNamespace

from stack_estop.node import (DistanceEstopController, analyze_corridor_scan,
                              rear_clear_level, rear_corridor_clear,
                              rear_scan_healthy)


YAW = 1.57079632679


def rear_scan(distance=math.inf, *, frame='base_link'):
    ranges = [math.inf] * 360
    ranges[0] = distance  # -pi: 차량 바로 뒤
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame), ranges=ranges,
        angle_min=-math.pi, angle_increment=math.pi / 180,
        range_min=0.15, range_max=12.0)


class RearClearTest(unittest.TestCase):
    def test_rear_corridor(self):
        self.assertTrue(rear_corridor_clear(rear_scan(1.0)))
        self.assertFalse(rear_corridor_clear(rear_scan(0.35)))
        self.assertFalse(rear_corridor_clear(rear_scan(1.0, frame='laser')))
        scan = rear_scan()
        scan.ranges[90] = 0.25  # 옆쪽은 후진 통로 밖
        self.assertTrue(rear_corridor_clear(scan))

    def test_rear_sensor_health(self):
        self.assertTrue(rear_scan_healthy(rear_scan(1.0)))
        self.assertFalse(rear_scan_healthy(rear_scan()))

    def test_rear_clear_requires_fresh_inputs(self):
        args = [True, True, True, True, 0.1, 0.1, 0.35, 0.25]
        self.assertTrue(rear_clear_level(*args))
        for index, bad in ((0, False), (1, False), (2, False),
                           (3, False), (4, None), (5, 0.3)):
            changed = args.copy()
            changed[index] = bad
            self.assertFalse(rear_clear_level(*changed))


def front_cluster(distance, count=3):
    increment = 0.01
    angle_min = -math.pi / 2 - increment * (count - 1) / 2
    return analyze_corridor_scan(
        [distance] * count,
        angle_min,
        increment,
        0.03,
        12.0,
        laser_yaw_in_base_rad=YAW,
    )


class DistanceEstopTest(unittest.TestCase):
    def test_front_060_and_069_stop(self):
        for distance in (0.60, 0.69):
            result = front_cluster(distance)
            controller = DistanceEstopController()
            controller.update_from_scan(result['nearest_cluster_min_x'])
            self.assertTrue(controller.current_final_estop)
            self.assertEqual(result['cluster_sizes'], [3])

    def test_hysteresis_depends_on_previous_level(self):
        distance = front_cluster(0.75)['nearest_cluster_min_x']
        clear = DistanceEstopController()
        for _ in range(3):
            clear.update_from_scan(None)
        clear.update_from_scan(distance)
        self.assertFalse(clear.current_final_estop)

        stopped = DistanceEstopController()
        stopped.update_from_scan(distance)
        self.assertTrue(stopped.current_final_estop)

    def test_three_safe_scans_release(self):
        controller = DistanceEstopController()
        safe_x = front_cluster(0.81)['nearest_cluster_min_x']
        values = []
        for _ in range(3):
            controller.update_from_scan(safe_x)
            values.append(controller.current_final_estop)
        self.assertEqual(values, [True, True, False])

    def test_side_point_not_front_obstacle(self):
        result = analyze_corridor_scan(
            [0.20], 0.0, 0.01, 0.03, 12.0,
            laser_yaw_in_base_rad=YAW)
        self.assertIsNone(result['nearest_cluster_min_x'])
        self.assertEqual(result['reason'], 'OUTSIDE_CORRIDOR')

    def test_single_noise_point_rejected(self):
        result = front_cluster(0.30, count=1)
        self.assertIsNone(result['nearest_cluster_min_x'])
        self.assertEqual(result['reason'], 'TOO_FEW_CLUSTER_POINTS')
        controller = DistanceEstopController()
        for _ in range(3):
            controller.update_from_scan(None)
        controller.update_from_scan(result['nearest_cluster_min_x'])
        self.assertFalse(controller.current_final_estop)

    def test_timeout_forces_stop(self):
        controller = DistanceEstopController()
        for _ in range(3):
            controller.update_from_scan(None)
        self.assertFalse(controller.current_final_estop)
        controller.force_timeout()
        self.assertTrue(controller.current_final_estop)

    def test_laser_minus_90_is_base_plus_x(self):
        roi = front_cluster(0.60)['nearest_roi_point']
        self.assertAlmostEqual(roi['x'], 0.60, places=3)
        self.assertAlmostEqual(roi['y'], 0.0, delta=0.01)


if __name__ == '__main__':
    unittest.main()
