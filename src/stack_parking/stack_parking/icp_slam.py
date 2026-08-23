"""Small 2-D ICP SLAM core for the fused four-LiDAR endpoint cloud.

The fused cloud no longer carries a per-point sensor origin.  This module
therefore treats it as an endpoint scan for scan-to-local-map ICP; it does not
invent free-space rays from ``base_link``.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np

from .geometry import Pose2, between, compose, transform_points, wrap_angle


@dataclass
class IcpConfig:
    scan_voxel_m: float = 0.06
    max_scan_points: int = 900
    map_voxel_m: float = 0.08
    max_correspondence_m: float = 0.35
    max_iterations: int = 18
    min_correspondences: int = 24
    trim_fraction: float = 0.82
    translation_epsilon_m: float = 0.002
    yaw_epsilon_rad: float = math.radians(0.12)
    max_correction_m: float = 0.55
    max_correction_yaw_rad: float = math.radians(22.0)
    max_rmse_m: float = 0.16
    min_scan_range_m: float = 0.12
    max_scan_range_m: float = 8.0
    local_map_radius_m: float = 8.0
    max_map_cells: int = 40000


@dataclass(frozen=True)
class IcpResult:
    pose: Pose2
    accepted: bool
    initialized: bool
    correspondences: int
    rmse_m: float
    iterations: int
    used_odometry_prior: bool
    reason: str


class VoxelPointMap:
    """Centroid voxel map with a hash-grid nearest-neighbour query."""

    def __init__(self, voxel_m: float, max_cells: int = 40000):
        self.voxel_m = max(float(voxel_m), 1.0e-3)
        self.max_cells = max(int(max_cells), 100)
        # key -> [mean_x, mean_y, count, last_update]
        self._cells: dict[tuple[int, int], list[float]] = {}
        self._update_counter = 0

    def __len__(self) -> int:
        return len(self._cells)

    def clear(self) -> None:
        self._cells.clear()
        self._update_counter = 0

    def _key(self, point: np.ndarray) -> tuple[int, int]:
        return (
            int(math.floor(float(point[0]) / self.voxel_m)),
            int(math.floor(float(point[1]) / self.voxel_m)),
        )

    def add(self, points: np.ndarray) -> None:
        self._update_counter += 1
        for point in np.asarray(points, dtype=np.float64):
            key = self._key(point)
            cell = self._cells.get(key)
            if cell is None:
                self._cells[key] = [float(point[0]), float(point[1]), 1.0,
                                    float(self._update_counter)]
                continue
            # A capped running mean prevents one repeatedly observed wall from
            # becoming numerically immutable.
            count = min(cell[2], 30.0)
            new_count = count + 1.0
            cell[0] = (cell[0] * count + float(point[0])) / new_count
            cell[1] = (cell[1] * count + float(point[1])) / new_count
            cell[2] = new_count
            cell[3] = float(self._update_counter)
        self._prune_if_needed()

    def _prune_if_needed(self) -> None:
        excess = len(self._cells) - self.max_cells
        if excess <= 0:
            return
        oldest = sorted(self._cells.items(), key=lambda item: item[1][3])
        for key, _ in oldest[:excess]:
            del self._cells[key]

    def points(
        self,
        center: Optional[Pose2] = None,
        radius_m: Optional[float] = None,
    ) -> np.ndarray:
        if not self._cells:
            return np.empty((0, 2), dtype=np.float64)
        values = np.asarray([[cell[0], cell[1]] for cell in self._cells.values()],
                            dtype=np.float64)
        if center is None or radius_m is None or radius_m <= 0.0:
            return values
        delta = values - np.asarray([center.x, center.y])
        return values[np.einsum('ij,ij->i', delta, delta) <= radius_m * radius_m]

    def nearest(
        self,
        query: np.ndarray,
        max_distance_m: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return matching query indices, map points and squared distances."""
        query = np.asarray(query, dtype=np.float64)
        if query.size == 0 or not self._cells:
            return (
                np.empty(0, dtype=np.int64),
                np.empty((0, 2), dtype=np.float64),
                np.empty(0, dtype=np.float64),
            )
        cell_radius = max(1, int(math.ceil(max_distance_m / self.voxel_m)))
        max_d2 = max_distance_m * max_distance_m
        indices: list[int] = []
        targets: list[tuple[float, float]] = []
        distances: list[float] = []
        for index, point in enumerate(query):
            base = self._key(point)
            best_d2 = max_d2
            best: Optional[list[float]] = None
            for dx in range(-cell_radius, cell_radius + 1):
                for dy in range(-cell_radius, cell_radius + 1):
                    cell = self._cells.get((base[0] + dx, base[1] + dy))
                    if cell is None:
                        continue
                    d2 = (cell[0] - point[0]) ** 2 + (cell[1] - point[1]) ** 2
                    if d2 <= best_d2:
                        best_d2 = d2
                        best = cell
            if best is not None:
                indices.append(index)
                targets.append((best[0], best[1]))
                distances.append(best_d2)
        return (
            np.asarray(indices, dtype=np.int64),
            np.asarray(targets, dtype=np.float64).reshape((-1, 2)),
            np.asarray(distances, dtype=np.float64),
        )


def voxel_downsample(points: np.ndarray, voxel_m: float) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    if points.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    keys = np.floor(points[:, :2] / max(voxel_m, 1.0e-4)).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(first), :2]


def best_fit_transform(source: np.ndarray, target: np.ndarray) -> Pose2:
    """Least-squares rigid transform that maps source points to target."""
    source_center = np.mean(source, axis=0)
    target_center = np.mean(target, axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    return Pose2(
        float(translation[0]),
        float(translation[1]),
        math.atan2(float(rotation[1, 0]), float(rotation[0, 0])),
    )


class IcpSlam:
    """Incremental scan-to-voxel-map ICP with an optional odometry prior."""

    def __init__(self, config: Optional[IcpConfig] = None):
        self.config = config or IcpConfig()
        self.map = VoxelPointMap(self.config.map_voxel_m, self.config.max_map_cells)
        self.pose = Pose2()
        self.last_odom: Optional[Pose2] = None
        self.initialized = False

    def reset(self, pose: Pose2 = Pose2()) -> None:
        self.map.clear()
        self.pose = pose
        self.last_odom = None
        self.initialized = False

    def _prepare_scan(self, points: np.ndarray) -> np.ndarray:
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] < 2:
            return np.empty((0, 2), dtype=np.float64)
        points = points[:, :2]
        points = points[np.isfinite(points).all(axis=1)]
        radius2 = np.einsum('ij,ij->i', points, points)
        valid = (
            radius2 >= self.config.min_scan_range_m ** 2
        ) & (
            radius2 <= self.config.max_scan_range_m ** 2
        )
        points = voxel_downsample(points[valid], self.config.scan_voxel_m)
        if len(points) > self.config.max_scan_points > 0:
            # The merged cloud is concatenated sensor-by-sensor. Polar sorting
            # before deterministic subsampling preserves all four directions
            # instead of preferentially retaining the first LiDAR.
            order = np.argsort(np.arctan2(points[:, 1], points[:, 0]))
            selected = np.linspace(
                0, len(order) - 1, self.config.max_scan_points,
                dtype=np.int64)
            points = points[order[selected]]
        return points

    def update(self, scan_points: np.ndarray, odom_pose: Optional[Pose2] = None) -> IcpResult:
        scan = self._prepare_scan(scan_points)
        if len(scan) < self.config.min_correspondences:
            return IcpResult(
                self.pose, False, self.initialized, 0, math.inf, 0,
                odom_pose is not None, 'too_few_scan_points')

        if not self.initialized:
            self.map.add(transform_points(scan, self.pose))
            self.initialized = True
            self.last_odom = odom_pose
            return IcpResult(
                self.pose, True, True, len(scan), 0.0, 0,
                odom_pose is not None, 'initialized')

        used_prior = odom_pose is not None and self.last_odom is not None
        guess = self.pose
        if used_prior:
            guess = compose(self.pose, between(self.last_odom, odom_pose))
        self.last_odom = odom_pose if odom_pose is not None else self.last_odom

        estimate = guess
        rmse = math.inf
        match_count = 0
        reason = 'not_converged'
        iterations = 0

        for iteration in range(self.config.max_iterations):
            iterations = iteration + 1
            transformed = transform_points(scan, estimate)
            source_indices, targets, distances2 = self.map.nearest(
                transformed, self.config.max_correspondence_m)
            match_count = len(source_indices)
            if match_count < self.config.min_correspondences:
                reason = 'too_few_correspondences'
                break

            keep_count = max(
                self.config.min_correspondences,
                int(match_count * min(max(self.config.trim_fraction, 0.2), 1.0)),
            )
            if keep_count < match_count:
                keep = np.argpartition(distances2, keep_count - 1)[:keep_count]
                source_indices = source_indices[keep]
                targets = targets[keep]
                distances2 = distances2[keep]
                match_count = keep_count

            delta = best_fit_transform(transformed[source_indices], targets)
            estimate = compose(delta, estimate)
            rmse = math.sqrt(float(np.mean(distances2)))
            if (
                math.hypot(delta.x, delta.y) <= self.config.translation_epsilon_m
                and abs(delta.yaw) <= self.config.yaw_epsilon_rad
            ):
                reason = 'converged'
                break

        correction = between(guess, estimate)
        correction_m = math.hypot(correction.x, correction.y)
        accepted = (
            match_count >= self.config.min_correspondences
            and math.isfinite(rmse)
            and rmse <= self.config.max_rmse_m
            and correction_m <= self.config.max_correction_m
            and abs(wrap_angle(correction.yaw)) <= self.config.max_correction_yaw_rad
        )
        if accepted:
            self.pose = estimate
            self.map.add(transform_points(scan, self.pose))
        else:
            # Odometry is only a prediction.  A rejected ICP result must not
            # contaminate the point map; pose can follow a valid prior so the
            # matcher has a chance to recover on the next distinctive scan.
            if used_prior:
                self.pose = guess
                reason = 'rejected_using_odometry_prediction'
            elif reason == 'converged':
                reason = 'rejected_quality_gate'

        return IcpResult(
            self.pose, accepted, True, match_count, rmse, iterations,
            used_prior, reason)

    def map_points(self, local_only: bool = False) -> np.ndarray:
        if local_only:
            return self.map.points(self.pose, self.config.local_map_radius_m)
        return self.map.points()
