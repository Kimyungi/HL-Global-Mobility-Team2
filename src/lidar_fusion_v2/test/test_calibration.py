import math

import numpy as np

from lidar_fusion_v2.calibration import fit_line_ransac, solve_sensor_pose


def test_ransac_rejects_outliers():
    x = np.linspace(-2.0, 2.0, 80)
    wall = np.column_stack((x, 1.2 + 0.002 * np.sin(x)))
    outliers = np.array(((3.0, -2.0), (-4.0, 3.0), (2.2, 2.4)))
    normal, offset, mask = fit_line_ransac(np.vstack((wall, outliers)), 0.02)
    assert mask.sum() == 80
    assert np.max(np.abs(wall @ normal + offset)) < 0.01


def test_multiple_wall_angles_recover_side_pose():
    true = np.array((0.31, 0.155, math.radians(1.7)))
    c, s = math.cos(true[2]), math.sin(true[2])
    rotation = np.array(((c, -s), (s, c)))
    observations = []
    for normal, offset in [([0.0, 1.0], -1.0), ([1.0, 0.0], -1.5),
                           ([2 ** -0.5, 2 ** -0.5], -1.3)]:
        normal = np.asarray(normal)
        tangent = np.array((-normal[1], normal[0]))
        base = -offset * normal + np.linspace(-0.6, 0.6, 60)[:, None] * tangent
        local = (base - true[:2]) @ rotation
        observations.append({'normal': normal, 'offset': offset,
                             'target_local': local})
    pose, rms, result = solve_sensor_pose(
        observations, (0.28, 0.18, math.radians(0.0)))
    assert result.success
    assert np.allclose(pose, true, atol=1e-5)
    assert rms < 1e-6
