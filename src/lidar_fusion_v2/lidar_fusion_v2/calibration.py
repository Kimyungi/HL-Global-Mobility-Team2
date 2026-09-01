"""Robust planar-wall calibration, independent from the fusion runtime."""

import math
from typing import Iterable, Mapping, Sequence, Tuple

import numpy as np
from scipy.optimize import least_squares


def fit_line_ransac(points: np.ndarray, threshold: float = 0.03,
                    iterations: int = 300) -> Tuple[np.ndarray, float, np.ndarray]:
    """Return unit normal, line offset and inlier mask for a dominant 2D line."""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 8:
        raise ValueError('at least 8 XY points are required')
    rng = np.random.default_rng(42)
    best = np.zeros(len(points), dtype=bool)
    for _ in range(iterations):
        i, j = rng.choice(len(points), 2, replace=False)
        direction = points[j] - points[i]
        length = np.linalg.norm(direction)
        if length < 0.05:
            continue
        normal = np.array((-direction[1], direction[0])) / length
        offset = -float(normal @ points[i])
        mask = np.abs(points @ normal + offset) <= threshold
        if mask.sum() > best.sum():
            best = mask
    if best.sum() < 8:
        raise ValueError('no stable wall line found')
    selected = points[best]
    center = selected.mean(axis=0)
    _, _, vh = np.linalg.svd(selected - center, full_matrices=False)
    direction = vh[0]
    normal = np.array((-direction[1], direction[0]))
    normal /= np.linalg.norm(normal)
    offset = -float(normal @ center)
    residual = np.abs(points @ normal + offset)
    best = residual <= threshold
    return normal, offset, best


def solve_sensor_pose(observations: Sequence[Mapping], initial: Iterable[float],
                      xy_bound: float = 0.30,
                      yaw_bound_deg: float = 20.0):
    """Fit absolute x/y/yaw from target-local points to anchor wall lines."""
    initial = np.asarray(tuple(initial), dtype=float)
    if len(observations) < 2:
        raise ValueError('capture at least two different wall angles')

    def residual(pose):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        rotation = np.array(((c, -s), (s, c)))
        chunks = []
        for observation in observations:
            local = np.asarray(observation['target_local'], dtype=float)
            normal = np.asarray(observation['normal'], dtype=float)
            offset = float(observation['offset'])
            base = local @ rotation.T
            base[:, 0] += x
            base[:, 1] += y
            chunks.append(base @ normal + offset)
        return np.concatenate(chunks)

    yaw_bound = math.radians(yaw_bound_deg)
    lower = initial - np.array((xy_bound, xy_bound, yaw_bound))
    upper = initial + np.array((xy_bound, xy_bound, yaw_bound))
    result = least_squares(
        residual, initial, bounds=(lower, upper), loss='soft_l1', f_scale=0.025)
    rms = float(np.sqrt(np.mean(np.square(residual(result.x)))))
    return result.x, rms, result
