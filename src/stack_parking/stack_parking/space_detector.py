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
        self._stable_count = 0

    def reset(self) -> None:
        self._last_candidate = None
        self._stable_count = 0

    def _clusters(self, x_values: np.ndarray) -> list[tuple[float, float, int]]:
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
        result: list[tuple[float, float, int]] = []
        for group in groups:
            span = group[-1] - group[0]
            raw_count = int(np.count_nonzero(
                (x_values >= group[0] - 0.03) & (x_values <= group[-1] + 0.03)))
            if (
                span >= self.config.cluster_min_span_m
                and raw_count >= self.config.cluster_min_points
            ):
                result.append((group[0], group[-1], raw_count))
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
        boundary = lane_points[
            (side_distance >= self.config.boundary_near_m)
            & (side_distance <= self.config.boundary_far_m)
        ]
        clusters = self._clusters(boundary[:, 0])
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
        if side not in (SIDE_LEFT, SIDE_RIGHT):
            raise ValueError(f'unsupported parking side: {side}')

        lane_points = points_in_frame(map_points, lane_pose_map)
        current_lane = points_in_frame(
            np.asarray([[current_pose_map.x, current_pose_map.y]]), lane_pose_map)[0]
        candidate = self._find_candidate(
            lane_points, float(current_lane[0]), mode, side)
        if candidate is None:
            self._last_candidate = None
            self._stable_count = 0
            return None

        if self._last_candidate is not None and (
            abs(candidate.start_x - self._last_candidate.start_x)
            <= self.config.stable_edge_tolerance_m
            and abs(candidate.end_x - self._last_candidate.end_x)
            <= self.config.stable_edge_tolerance_m
        ):
            self._stable_count += 1
        else:
            self._stable_count = 1
        self._last_candidate = candidate
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
