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
    # Free-space clearing: an old map point is dropped when the *current*
    # accepted-pose scan looks straight through where it used to be and
    # sees a farther return in the same direction. Empty endpoint-cloud bins
    # are unknown, not confirmed free space. Nearby nearer returns are spread
    # across a small angular padding so thin occluders protect points behind
    # them even when scan/map samples land on opposite bin boundaries.
    # Keep freespace_clear_radius_m within the upstream sensor cutoff
    # (lidar_fusion_v2 max_range/scan.range_max, currently 4.0m), so clearing
    # is limited to the sensor's useful mapped region.
    freespace_clear_enabled: bool = True
    freespace_clear_radius_m: float = 4.0
    freespace_bin_width_rad: float = math.radians(1.0)
    freespace_margin_m: float = 0.10
    freespace_occlusion_padding_rad: float = math.radians(1.5)
    # Point-to-point ICP under-constrains yaw when the matched points only
    # span a narrow bearing arc (e.g. one nearby wall segment while turning):
    # translation along that wall is still well fit, but rotation can drift
    # to a nearby wrong local minimum with an equally low RMSE — the map
    # then accumulates the same wall as several near-duplicate copies at
    # slightly different headings. Below this matched-point angular span,
    # the yaw correction is discarded (kept at the odometry prior's yaw)
    # while the translation correction is still applied.
    min_yaw_observable_span_rad: float = math.radians(50.0)
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

    def clear_freespace(
        self,
        pose: Pose2,
        local_scan: np.ndarray,
        clear_radius_m: float,
        bin_width_rad: float,
        margin_m: float,
        occlusion_padding_rad: float = 0.0,
    ) -> int:
        """Drop map points the current scan shows are no longer there.

        ``local_scan`` is the accepted-pose scan in *vehicle frame* (the same
        points ``update()`` just matched), already range-limited upstream —
        that limit is what bounds how far this may look, via
        ``clear_radius_m`` (must not exceed it, see ``IcpConfig`` note).
        Candidate map points are binned into vehicle-frame bearing bins and
        compared against the nearest current-scan return in that bin. An old
        point closer than that return by more than ``margin_m`` is stale.
        Empty bins are left untouched because an endpoint-only PointCloud2
        cannot distinguish an actual no-return ray from missing coverage.
        A point at or beyond the current return (same surface, or occluded
        by something nearer now) is left untouched. ``occlusion_padding_rad``
        extends nearer returns to neighboring bins to cover thin-object and
        voxel/bin-boundary sampling differences.
        """
        if not self._cells or clear_radius_m <= 0.0:
            return 0
        keys = list(self._cells.keys())
        values = np.asarray([[c[0], c[1]] for c in self._cells.values()],
                            dtype=np.float64)
        delta = values - np.asarray([pose.x, pose.y])
        within = np.einsum('ij,ij->i', delta, delta) <= clear_radius_m * clear_radius_m
        candidates = np.nonzero(within)[0]
        if candidates.size == 0:
            return 0

        c, s = math.cos(-pose.yaw), math.sin(-pose.yaw)
        dx = delta[candidates, 0]
        dy = delta[candidates, 1]
        local_x = c * dx - s * dy
        local_y = s * dx + c * dy
        local_range = np.hypot(local_x, local_y)
        local_bearing = np.arctan2(local_y, local_x)

        nbins = max(1, int(math.ceil(2.0 * math.pi / bin_width_rad)))

        def bin_of(angles: np.ndarray) -> np.ndarray:
            return np.clip(
                np.floor((angles + math.pi) / bin_width_rad).astype(np.int64),
                0, nbins - 1)

        current_range = np.full(nbins, np.inf, dtype=np.float64)
        scan = np.asarray(local_scan, dtype=np.float64)
        if scan.size:
            scan_range = np.hypot(scan[:, 0], scan[:, 1])
            np.minimum.at(current_range, bin_of(np.arctan2(scan[:, 1], scan[:, 0])),
                         scan_range)

        padding_bins = max(0, int(math.ceil(
            occlusion_padding_rad / bin_width_rad)))
        if padding_bins:
            raw_range = current_range.copy()
            for offset in range(1, padding_bins + 1):
                current_range = np.minimum(current_range, np.roll(raw_range, offset))
                current_range = np.minimum(current_range, np.roll(raw_range, -offset))

        observed = current_range[bin_of(local_bearing)]
        stale = np.isfinite(observed) & (local_range < (observed - margin_m))
        removed = 0
        for pos in candidates[stale]:
            del self._cells[keys[pos]]
            removed += 1
        return removed


def voxel_downsample(points: np.ndarray, voxel_m: float) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    if points.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    keys = np.floor(points[:, :2] / max(voxel_m, 1.0e-4)).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(first), :2]


def angular_span(bearings: np.ndarray) -> float:
    """Angle subtended by *bearings* (rad), robust to the -pi/pi wrap.

    Defined as ``2*pi`` minus the largest circular gap between consecutive
    sorted bearings — i.e. the width of the smallest arc containing them all.
    """
    bearings = np.asarray(bearings, dtype=np.float64)
    if bearings.size < 2:
        return 0.0
    sorted_b = np.sort(bearings)
    gaps = np.diff(sorted_b)
    wrap_gap = (sorted_b[0] + 2.0 * math.pi) - sorted_b[-1]
    return float(2.0 * math.pi - max(np.max(gaps), wrap_gap))


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

    def update(
        self,
        scan_points: np.ndarray,
        odom_pose: Optional[Pose2] = None,
        update_map: bool = True,
    ) -> IcpResult:
        """Register one scan and optionally add accepted endpoints to the map.

        ``update_map=False`` is localization-only mode.  It updates the pose
        against the frozen map but never lets parked vehicles or other dynamic
        objects contaminate the static mapping snapshot.
        """
        scan = self._prepare_scan(scan_points)
        if len(scan) < self.config.min_correspondences:
            return IcpResult(
                self.pose, False, self.initialized, 0, math.inf, 0,
                odom_pose is not None, 'too_few_scan_points')

        if not self.initialized and not update_map:
            return IcpResult(
                self.pose, False, False, 0, math.inf, 0,
                odom_pose is not None, 'mapping_disabled_uninitialized')

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

        yaw_observable = False
        if match_count >= self.config.min_correspondences:
            bearings = np.arctan2(scan[source_indices, 1], scan[source_indices, 0])
            yaw_observable = angular_span(bearings) >= self.config.min_yaw_observable_span_rad
        if not yaw_observable:
            estimate = Pose2(estimate.x, estimate.y, guess.yaw)

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
            if update_map:
                if self.config.freespace_clear_enabled:
                    self.map.clear_freespace(
                        self.pose, scan,
                        self.config.freespace_clear_radius_m,
                        self.config.freespace_bin_width_rad,
                        self.config.freespace_margin_m,
                        self.config.freespace_occlusion_padding_rad,
                    )
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
        if accepted and not yaw_observable:
            reason = 'accepted_yaw_locked_to_prior'

        return IcpResult(
            self.pose, accepted, True, match_count, rmse, iterations,
            used_prior, reason)

    def map_points(self, local_only: bool = False) -> np.ndarray:
        if local_only:
            return self.map.points(self.pose, self.config.local_map_radius_m)
        return self.map.points()
