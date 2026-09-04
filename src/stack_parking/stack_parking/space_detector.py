"""Parking-gap detection on the accumulated ICP endpoint map."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np

from .geometry import Pose2, compose, points_in_frame


MODE_PARALLEL = 'parallel'
MODE_PERPENDICULAR = 'perpendicular'
SIDE_LEFT = 'left'
SIDE_RIGHT = 'right'
# Search both sides and take whichever gap is encountered first (smallest
# start_x_lane) — the caller doesn't have to know which side the space is on.
SIDE_AUTO = 'auto'


@dataclass
class SpaceDetectorConfig:
    vehicle_width_m: float = 0.62
    vehicle_length_m: float = 0.85
    boundary_near_m: float = 0.42
    boundary_far_m: float = 1.05
    cluster_join_gap_m: float = 0.24
    cluster_min_span_m: float = 0.20
    cluster_min_points: int = 5
    perpendicular_min_width_m: float = 0.86
    parallel_min_length_m: float = 2.90
    perpendicular_min_depth_m: float = 1.35
    parallel_offset_min_m: float = 0.68
    parallel_offset_max_m: float = 1.00
    rear_lidar_x_m: float = -0.110354
    completion_clearance_m: float = 0.20
    # Plan a little past the threshold so quantization/noise can actually
    # satisfy rear_clearance <= completion_clearance at the path end.
    completion_trigger_margin_m: float = 0.02
    back_wall_bin_m: float = 0.10
    back_wall_min_points: int = 5
    candidate_max_ahead_m: float = 3.0
    candidate_max_behind_m: float = 2.5
    stable_frames: int = 3
    stable_edge_tolerance_m: float = 0.18
    # A ㄷ/U-shaped bay is one *connected* wall (both arms + back wall meet
    # at the corners) — boundary-band points get grouped into 2D-connected
    # blobs first (real x/y adjacency, not just x) so an unrelated object
    # sitting at a similar lateral distance elsewhere doesn't fuse into the
    # same "wall" the way pure x-axis banding would. Each blob's near-mouth
    # slice (points within this depth of boundary_near_m) is then clustered
    # by x alone — that still splits the two arm bases apart even though
    # the whole ㄷ is one blob, because the mouth itself has no points.
    mouth_slice_depth_m: float = 0.4


@dataclass(frozen=True)
class ParkingSpace:
    mode: str
    side: str
    start_x_lane: float
    end_x_lane: float
    side_distance_m: float
    back_wall_distance_m: Optional[float]
    goal_pose_map: Pose2
    lane_pose_map: Pose2
    confidence: float

    @property
    def size_along_lane_m(self) -> float:
        return self.end_x_lane - self.start_x_lane


@dataclass(frozen=True)
class _Candidate:
    start_x: float
    end_x: float
    side_distance: float
    back_wall_distance: Optional[float]
    confidence: float


class ParkingSpaceDetector:
    """Find a stable gap bracketed by two mapped obstacle clusters.

    Only endpoint occupancy is used.  The fused cloud has no per-point ray
    origin, so this detector deliberately does not ray-cast fictitious free
    space from the rear axle.
    """

    def __init__(self, config: Optional[SpaceDetectorConfig] = None):
        self.config = config or SpaceDetectorConfig()
        self._last_candidate: Optional[_Candidate] = None
        self._last_side: Optional[str] = None
        self._stable_count = 0

    def reset(self) -> None:
        self._last_candidate = None
        self._last_side = None
        self._stable_count = 0

    def _group_by_x(self, x_values: np.ndarray) -> list[list[float]]:
        """1D grouping by ``cluster_join_gap_m``, before any size filter."""
        if len(x_values) == 0:
            return []
        # Quantization prevents a dense wall from dominating while preserving
        # its physical span and the gap between two parked vehicles.
        quantized = np.unique(np.round(x_values / 0.04).astype(np.int64)) * 0.04
        groups: list[list[float]] = [[float(quantized[0])]]
        for value in quantized[1:]:
            if float(value) - groups[-1][-1] <= self.config.cluster_join_gap_m:
                groups[-1].append(float(value))
            else:
                groups.append([float(value)])
        return groups

    def _clusters(self, x_values: np.ndarray) -> list[tuple[float, float, int]]:
        if len(x_values) == 0:
            return []
        result: list[tuple[float, float, int]] = []
        for group in self._group_by_x(x_values):
            span = group[-1] - group[0]
            raw_count = int(np.count_nonzero(
                (x_values >= group[0] - 0.03) & (x_values <= group[-1] + 0.03)))
            if (
                span >= self.config.cluster_min_span_m
                and raw_count >= self.config.cluster_min_points
            ):
                result.append((group[0], group[-1], raw_count))
        return result

    def _mouth_clusters(
        self, boundary: np.ndarray, side_distance: np.ndarray,
    ) -> list[tuple[float, float, int]]:
        """Wall segments at the gap's mouth, 2D-blob-aware.

        A ㄷ/U-shaped bay is *one* connected wall — both arms and the back
        wall meet at the corners. Plain x-axis clustering (``_clusters``)
        would either merge that whole connected wall with an unrelated
        object sitting at a similar lateral distance elsewhere (nothing
        stops x-only banding from bridging across y), or, if it doesn't,
        still can't split "one connected wall" into the two arms needed to
        find the gap between them.

        Fix: cluster boundary points into real 2D-connected blobs first
        (union-find over a grid, cell size = cluster_join_gap_m — a
        distant clutter blob at a similar lateral distance no longer
        merges with the arm just because x-banding ignored the y gap
        between them). *Then*, within each blob, take only the points
        within ``mouth_slice_depth_m`` of ``boundary_near_m`` (the slice
        right at the opening) and cluster *that* by x — even a single
        connected ㄷ blob still splits into two segments here, because the
        open mouth itself has no points at all at that depth.
        """
        if len(boundary) == 0:
            return []
        coords = np.column_stack((boundary[:, 0], side_distance))
        gap = self.config.cluster_join_gap_m
        gap2 = gap * gap
        # Bucket by a gap-sized grid for candidate lookup, but still verify
        # the *actual* Euclidean distance before unioning — two points in
        # diagonally adjacent cells can be up to ~2*gap*sqrt(2) apart, well
        # past the intended join distance, and this data has real near-misses
        # at almost exactly that scale (an unrelated object's edge sitting
        # close to but not touching an arm's base).
        cell = max(gap, 1.0e-3)
        keys = np.floor(coords / cell).astype(np.int64)
        cell_to_indices: dict[tuple[int, int], list[int]] = {}
        for i, key in enumerate(map(tuple, keys)):
            cell_to_indices.setdefault(key, []).append(i)

        parent = list(range(len(boundary)))

        def find(a: int) -> int:
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a

        def union(a: int, b: int) -> None:
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[ra] = rb

        for (cx, cy), idxs in cell_to_indices.items():
            for i in idxs[1:]:
                union(idxs[0], i)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    neighbor = cell_to_indices.get((cx + dx, cy + dy))
                    if not neighbor:
                        continue
                    for i in idxs:
                        if find(i) in {find(j) for j in neighbor}:
                            continue
                        for j in neighbor:
                            d2 = float(np.sum((coords[i] - coords[j]) ** 2))
                            if d2 <= gap2:
                                union(i, j)
                                break

        blobs: dict[int, list[int]] = {}
        for i in range(len(boundary)):
            blobs.setdefault(find(i), []).append(i)

        near_limit = self.config.boundary_near_m + self.config.mouth_slice_depth_m
        result: list[tuple[float, float, int]] = []
        for idxs in blobs.values():
            member = np.asarray(idxs, dtype=np.int64)
            mouth = member[side_distance[member] <= near_limit]
            if len(mouth) < self.config.cluster_min_points:
                continue
            mouth_x = boundary[mouth, 0]
            # A whole ㄷ (both arms + back wall) is one blob — group its
            # mouth-slice x-values by cluster_join_gap_m *before* deciding
            # validity, so the two arm bases (far apart in x) are judged
            # separately rather than as a single min..max span across both.
            for group in self._group_by_x(mouth_x):
                span = group[-1] - group[0]
                raw_count = int(np.count_nonzero(
                    (mouth_x >= group[0] - 0.03) & (mouth_x <= group[-1] + 0.03)))
                if raw_count < self.config.cluster_min_points:
                    continue
                if span >= self.config.cluster_min_span_m:
                    result.append((group[0], group[-1], raw_count))
                    continue
                # Narrower than cluster_min_span_m in x — expected for a ㄷ
                # arm, which runs *along* the depth axis (near-constant x,
                # large y), not along the lane. The blob's own depth (not
                # this mouth slice) is the right "is this actually a wall"
                # signal for that case.
                blob_depth = float(
                    side_distance[member].max() - side_distance[member].min())
                if blob_depth >= self.config.cluster_min_span_m:
                    result.append((group[0], group[-1], raw_count))
        result.sort(key=lambda item: item[0])
        return result

    def _back_wall_distance(
        self,
        lane_points: np.ndarray,
        side_sign: float,
        gap_start: float,
        gap_end: float,
    ) -> Optional[float]:
        margin = min(0.18, max(0.05, 0.15 * (gap_end - gap_start)))
        x = lane_points[:, 0]
        side_distance = side_sign * lane_points[:, 1]
        relevant = side_distance[
            (x >= gap_start + margin)
            & (x <= gap_end - margin)
            & (side_distance > self.config.boundary_far_m)
        ]
        if len(relevant) < self.config.back_wall_min_points:
            return None
        bins = np.round(relevant / self.config.back_wall_bin_m).astype(np.int64)
        unique, counts = np.unique(bins, return_counts=True)
        valid = counts >= self.config.back_wall_min_points
        if not np.any(valid):
            return None
        # The farthest supported row is the physical end wall; isolated far
        # returns cannot pass the point-count gate.
        selected = int(np.max(unique[valid]))
        members = relevant[bins == selected]
        return float(np.median(members))

    def _find_candidate(
        self,
        lane_points: np.ndarray,
        current_x_lane: float,
        mode: str,
        side: str,
    ) -> Optional[_Candidate]:
        if len(lane_points) < self.config.cluster_min_points * 2:
            return None
        side_sign = 1.0 if side == SIDE_LEFT else -1.0
        side_distance = side_sign * lane_points[:, 1]
        boundary_mask = (
            (side_distance >= self.config.boundary_near_m)
            & (side_distance <= self.config.boundary_far_m)
        )
        boundary = lane_points[boundary_mask]
        clusters = self._mouth_clusters(boundary, side_distance[boundary_mask])
        minimum = (
            self.config.parallel_min_length_m
            if mode == MODE_PARALLEL
            else self.config.perpendicular_min_width_m
        )
        candidates: list[_Candidate] = []
        for left, right in zip(clusters, clusters[1:]):
            start = left[1]
            end = right[0]
            gap = end - start
            if gap < minimum:
                continue
            center = 0.5 * (start + end)
            if center > current_x_lane + self.config.candidate_max_ahead_m:
                continue
            if center < current_x_lane - self.config.candidate_max_behind_m:
                continue

            flank_mask = (
                ((boundary[:, 0] >= left[0]) & (boundary[:, 0] <= left[1]))
                | ((boundary[:, 0] >= right[0]) & (boundary[:, 0] <= right[1]))
            )
            flank_distance = side_sign * boundary[flank_mask, 1]
            if len(flank_distance) == 0:
                continue
            near_face = float(np.median(flank_distance))
            back_wall = self._back_wall_distance(
                lane_points, side_sign, start, end)

            if mode == MODE_PERPENDICULAR:
                if back_wall is None or back_wall < self.config.perpendicular_min_depth_m:
                    continue
                target_clearance = max(
                    0.05,
                    self.config.completion_clearance_m
                    - self.config.completion_trigger_margin_m,
                )
                goal_side_distance = back_wall - (
                    abs(self.config.rear_lidar_x_m) + target_clearance)
            else:
                # The mapped near face is the side of the adjacent parked car.
                # Its centreline is approximately half a vehicle width deeper.
                goal_side_distance = min(
                    self.config.parallel_offset_max_m,
                    max(self.config.parallel_offset_min_m,
                        near_face + 0.5 * self.config.vehicle_width_m),
                )

            size_margin = min(1.0, max(0.0, (gap - minimum) / max(minimum, 0.1)))
            support = min(1.0, (left[2] + right[2]) / 30.0)
            back_score = 1.0 if (mode == MODE_PARALLEL or back_wall is not None) else 0.0
            confidence = 0.45 * support + 0.35 * size_margin + 0.20 * back_score
            candidates.append(_Candidate(
                start, end, goal_side_distance, back_wall, confidence))

        if not candidates:
            return None
        # Prefer the first usable space encountered in the driving direction.
        return min(candidates, key=lambda item: item.start_x)

    def update(
        self,
        map_points: np.ndarray,
        current_pose_map: Pose2,
        lane_pose_map: Pose2,
        mode: str,
        side: str,
    ) -> Optional[ParkingSpace]:
        if mode not in (MODE_PARALLEL, MODE_PERPENDICULAR):
            raise ValueError(f'unsupported parking mode: {mode}')
        if side not in (SIDE_LEFT, SIDE_RIGHT, SIDE_AUTO):
            raise ValueError(f'unsupported parking side: {side}')
        sides_to_search = (SIDE_LEFT, SIDE_RIGHT) if side == SIDE_AUTO else (side,)

        lane_points = points_in_frame(map_points, lane_pose_map)
        current_lane = points_in_frame(
            np.asarray([[current_pose_map.x, current_pose_map.y]]), lane_pose_map)[0]
        per_side = [
            (s, self._find_candidate(lane_points, float(current_lane[0]), mode, s))
            for s in sides_to_search
        ]
        found = [(s, c) for s, c in per_side if c is not None]
        if not found:
            self._last_candidate = None
            self._last_side = None
            self._stable_count = 0
            return None
        # Whichever gap is encountered first in the driving direction wins,
        # even if the other side also has a (farther) candidate.
        side, candidate = min(found, key=lambda item: item[1].start_x)

        if (
            self._last_candidate is not None
            and self._last_side == side
            and abs(candidate.start_x - self._last_candidate.start_x)
            <= self.config.stable_edge_tolerance_m
            and abs(candidate.end_x - self._last_candidate.end_x)
            <= self.config.stable_edge_tolerance_m
        ):
            self._stable_count += 1
        else:
            self._stable_count = 1
        self._last_candidate = candidate
        self._last_side = side
        if self._stable_count < self.config.stable_frames:
            return None

        side_sign = 1.0 if side == SIDE_LEFT else -1.0
        if mode == MODE_PARALLEL:
            target_clearance = max(
                0.05,
                self.config.completion_clearance_m
                - self.config.completion_trigger_margin_m,
            )
            goal_x = candidate.start_x + (
                abs(self.config.rear_lidar_x_m)
                + target_clearance
            )
            goal_yaw = 0.0
        else:
            goal_x = 0.5 * (candidate.start_x + candidate.end_x)
            # The vehicle faces the road after reversing into a perpendicular
            # bay: right bay -> +90 deg, left bay -> -90 deg.
            goal_yaw = -side_sign * math.pi / 2.0

        goal_lane = Pose2(
            goal_x,
            side_sign * candidate.side_distance,
            goal_yaw,
        )
        goal_map = compose(lane_pose_map, goal_lane)
        stable_bonus = min(0.15, 0.03 * (self._stable_count - self.config.stable_frames))
        return ParkingSpace(
            mode=mode,
            side=side,
            start_x_lane=candidate.start_x,
            end_x_lane=candidate.end_x,
            side_distance_m=candidate.side_distance,
            back_wall_distance_m=candidate.back_wall_distance,
            goal_pose_map=goal_map,
            lane_pose_map=lane_pose_map,
            confidence=min(1.0, candidate.confidence + stable_bonus),
        )
