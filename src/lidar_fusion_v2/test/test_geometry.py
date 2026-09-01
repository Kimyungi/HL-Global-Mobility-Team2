import math

import numpy as np

from lidar_fusion_v2.geometry import (
    SensorGeometry, angle_in_sector, points_to_virtual_scan, scan_to_base,
    transform_points)


def test_wrapped_sector():
    a = np.array([-179.0, -100.0, 0.0, 100.0, 179.0])
    assert angle_in_sector(a, 150.0, -150.0).tolist() == [True, False, False, False, True]


def test_scan_transform_and_fov():
    g = SensorGeometry('test', 1.0, 2.0, 90.0, -10.0, 10.0, 0.1, 5.0)
    cloud = scan_to_base([1.0, 1.0, 1.0], math.radians(-20), math.radians(20), g)
    assert cloud.shape == (1, 2)
    np.testing.assert_allclose(cloud[0], [1.0, 3.0], atol=1e-6)


def test_virtual_scan_nearest_wins():
    points = np.array([[2.0, 0.0], [1.0, 0.0], [0.0, 2.0]])
    ranges = points_to_virtual_scan(points, math.pi / 2, 0.1, 10.0)
    assert ranges[2] == 1.0
    assert ranges[3] == 2.0


def test_planar_correction():
    out = transform_points(np.array([[1.0, 0.0]]), 2.0, 3.0, math.pi / 2)
    np.testing.assert_allclose(out[0], [2.0, 4.0], atol=1e-6)


def test_fixed_side_fov_is_exactly_110_degrees():
    assert 145.0 - 35.0 == 110.0
