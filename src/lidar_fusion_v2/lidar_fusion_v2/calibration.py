"""Robust planar-wall calibration, independent from the fusion runtime."""

import math
from typing import Dict, Iterable, Mapping, Sequence, Tuple

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


def solve_four_lidar_poses(
        observations: Sequence[Mapping],
        initial: Mapping[str, Iterable[float]],
        fixed_anchor: str = 'a1', moving_anchor: str = 'a2',
        targets: Sequence[str] = ('b1', 'b2'),
        xy_bound: float = 0.15, yaw_bound_deg: float = 10.0):
    """Jointly fit rear/left/right poses while keeping the front gauge fixed.

    A rear-anchor wall line was recorded in the base frame produced by the
    *initial* rear pose.  Moving that anchor therefore has to move its line as
    well.  Solving all bridge observations together avoids forcing a rear yaw
    error into both side LiDAR poses.
    """
    adjustable = (moving_anchor,) + tuple(targets)
    initial_pose: Dict[str, np.ndarray] = {
        sensor_id: np.asarray(tuple(initial[sensor_id]), dtype=float)
        for sensor_id in (fixed_anchor,) + adjustable
    }
    if any(pose.shape != (3,) for pose in initial_pose.values()):
        raise ValueError('each initial pose must contain x, y and yaw')

    observations = tuple(observations)
    for target in targets:
        anchors = {
            item.get('anchor') for item in observations
            if item.get('target') == target
        }
        if fixed_anchor not in anchors or moving_anchor not in anchors:
            raise ValueError(
                f'{target} requires observations from both {fixed_anchor} '
                f'and {moving_anchor}')

    def rotation(yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array(((c, -s), (s, c)))

    def unpack(values, sensor_id):
        start = adjustable.index(sensor_id) * 3
        return values[start:start + 3]

    def line_in_candidate_frame(observation, values):
        normal = np.asarray(observation['normal'], dtype=float)
        offset = float(observation['offset'])
        anchor = observation.get('anchor')
        if anchor == fixed_anchor:
            return normal, offset
        if anchor != moving_anchor:
            raise ValueError(f'unsupported anchor: {anchor}')

        old = np.asarray(
            observation.get('anchor_pose', initial_pose[moving_anchor]),
            dtype=float)
        if old.shape != (3,):
            raise ValueError('anchor_pose must contain x, y and yaw')
        new = unpack(values, moving_anchor)
        delta_rotation = rotation(new[2]) @ rotation(old[2]).T
        translation = new[:2] - delta_rotation @ old[:2]
        transformed_normal = delta_rotation @ normal
        transformed_offset = offset - transformed_normal @ translation
        return transformed_normal, transformed_offset

    def residual(values):
        chunks = []
        for observation in observations:
            target = observation.get('target')
            if target not in targets:
                continue
            pose = unpack(values, target)
            local = np.asarray(observation['target_local'], dtype=float)
            base = local @ rotation(pose[2]).T + pose[:2]
            normal, offset = line_in_candidate_frame(observation, values)
            chunks.append(base @ normal + offset)
        if not chunks:
            raise ValueError('no supported wall observations')
        return np.concatenate(chunks)

    x0 = np.concatenate([initial_pose[sensor_id] for sensor_id in adjustable])
    bounds = np.tile((xy_bound, xy_bound, math.radians(yaw_bound_deg)),
                     len(adjustable))
    result = least_squares(
        residual, x0, bounds=(x0 - bounds, x0 + bounds),
        loss='soft_l1', f_scale=0.025)
    rms = float(np.sqrt(np.mean(np.square(residual(result.x)))))
    poses = {fixed_anchor: initial_pose[fixed_anchor].copy()}
    poses.update({sensor_id: unpack(result.x, sensor_id).copy()
                  for sensor_id in adjustable})
    return poses, rms, result
