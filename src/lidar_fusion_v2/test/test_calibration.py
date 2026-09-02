import math

import numpy as np

from lidar_fusion_v2.calibration import fit_line_ransac
from lidar_fusion_v2.calibration import solve_four_lidar_poses
from lidar_fusion_v2.calibration import solve_sensor_pose


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


def test_joint_solver_recovers_rear_and_both_side_poses():
    initial = {
        'a1': np.array((0.76, 0.0, math.radians(-87.0))),
        'a2': np.array((-0.055, 0.0, math.radians(90.0))),
        'b1': np.array((0.29, 0.17, math.radians(2.8))),
        'b2': np.array((0.30, -0.22, math.radians(-179.7))),
    }
    true = {
        'a1': initial['a1'],
        'a2': np.array((-0.11, 0.003, math.radians(93.4))),
        'b1': np.array((0.215, 0.212, math.radians(1.7))),
        'b2': np.array((0.246, -0.225, math.radians(-177.8))),
    }

    def rotation(yaw):
        return np.array(((math.cos(yaw), -math.sin(yaw)),
                         (math.sin(yaw), math.cos(yaw))))

    observations = []
    walls = ((np.array((0.0, 1.0)), -1.0),
             (np.array((2 ** -0.5, 2 ** -0.5)), -1.4),
             (np.array((1.0, 0.0)), -1.6))
    for target in ('b1', 'b2'):
        for anchor in ('a1', 'a2'):
            for normal, offset in walls:
                tangent = np.array((-normal[1], normal[0]))
                base = (-offset * normal
                        + np.linspace(-0.7, 0.7, 70)[:, None] * tangent)
                target_local = ((base - true[target][:2])
                                @ rotation(true[target][2]))

                if anchor == 'a1':
                    recorded_normal, recorded_offset = normal, offset
                else:
                    old, actual = initial[anchor], true[anchor]
                    delta = rotation(old[2]) @ rotation(actual[2]).T
                    translation = old[:2] - delta @ actual[:2]
                    recorded_normal = delta @ normal
                    recorded_offset = (offset
                                       - recorded_normal @ translation)
                observations.append({
                    'anchor': anchor, 'target': target,
                    'normal': recorded_normal,
                    'offset': recorded_offset,
                    'target_local': target_local,
                })

    poses, rms, result = solve_four_lidar_poses(observations, initial)
    assert result.success
    for sensor_id in true:
        assert np.allclose(poses[sensor_id], true[sensor_id], atol=1e-5)
    assert rms < 1e-6


def test_joint_solver_requires_front_and_rear_bridge_observations():
    initial = {sensor_id: (0.0, 0.0, 0.0)
               for sensor_id in ('a1', 'a2', 'b1', 'b2')}
    observation = {
        'anchor': 'a1', 'target': 'b1', 'normal': (0.0, 1.0),
        'offset': -1.0, 'target_local': ((0.0, 1.0),),
    }
    try:
        solve_four_lidar_poses((observation,), initial)
    except ValueError as error:
        assert 'both a1 and a2' in str(error)
        return
    raise AssertionError('joint calibration must reject a missing bridge')
