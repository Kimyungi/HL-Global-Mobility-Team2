"""Minimum-turn-radius analytical parking paths and footprint checks."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence

import numpy as np

from .geometry import (
    PathPoint,
    Pose2,
    between,
    deduplicate_path,
    path_from_frame,
    sample_motion,
    sample_pose_line,
)
from .space_detector import (
    MODE_PARALLEL,
    MODE_PERPENDICULAR,
    ParkingSpace,
    SIDE_LEFT,
)


@dataclass
class PlannerConfig:
    min_turn_radius_m: float = 1.15
    sample_step_m: float = 0.06
    vehicle_width_m: float = 0.62
    vehicle_front_m: float = 0.760
    vehicle_rear_m: float = 0.090
    static_clearance_m: float = 0.035
    lane_lateral_tolerance_m: float = 0.35
    lane_yaw_tolerance_rad: float = math.radians(25.0)
    stage_position_tolerance_m: float = 0.08


@dataclass(frozen=True)
class Collision:
    path_index: int
    point_x: float
    point_y: float


@dataclass(frozen=True)
class ParkingPlan:
    approach_path: tuple[PathPoint, ...]
    reverse_path: tuple[PathPoint, ...]
    stage_pose_map: Pose2
    goal_pose_map: Pose2
    minimum_turn_radius_m: float
    mode: str
    side: str

    @property
    def full_entry_path(self) -> tuple[PathPoint, ...]:
        return self.approach_path + self.reverse_path


def first_footprint_collision(
    path: Sequence[PathPoint],
    obstacle_points: np.ndarray,
    vehicle_front_m: float,
    vehicle_rear_m: float,
    vehicle_width_m: float,
    margin_m: float,
) -> Optional[Collision]:
    points = np.asarray(obstacle_points, dtype=np.float64)
    if len(path) == 0 or points.size == 0:
        return None
    front = vehicle_front_m + margin_m
    rear = vehicle_rear_m + margin_m
    half_width = 0.5 * vehicle_width_m + margin_m
    radius2 = (max(front, rear) ** 2 + half_width ** 2)
    for index, pose in enumerate(path):
        dx = points[:, 0] - pose.x
        dy = points[:, 1] - pose.y
        nearby = dx * dx + dy * dy <= radius2
        if not np.any(nearby):
            continue
        c = math.cos(pose.yaw)
        s = math.sin(pose.yaw)
        local_x = c * dx[nearby] + s * dy[nearby]
        local_y = -s * dx[nearby] + c * dy[nearby]
        inside = (
            (local_x >= -rear)
            & (local_x <= front)
            & (np.abs(local_y) <= half_width)
        )
        if np.any(inside):
            hit_index = np.flatnonzero(nearby)[np.flatnonzero(inside)[0]]
            return Collision(index, float(points[hit_index, 0]), float(points[hit_index, 1]))
    return None


class MinimumRadiusParkingPlanner:
    def __init__(self, config: Optional[PlannerConfig] = None):
        self.config = config or PlannerConfig()
        self.last_error = ''

    def _perpendicular_reverse_lane(
        self,
        stage: Pose2,
        goal: Pose2,
        side_sign: float,
    ) -> list[PathPoint]:
        radius = self.config.min_turn_radius_m
        curvature = side_sign / radius
        arc = sample_motion(
            stage,
            signed_distance=-radius * math.pi / 2.0,
            curvature=curvature,
            step_m=self.config.sample_step_m,
            gear=-1,
        )
        path = list(arc)
        arc_end = arc[-1].pose
        remaining = abs(goal.y - arc_end.y)
        if remaining > self.config.sample_step_m * 0.5:
            path.extend(sample_motion(
                arc_end,
                signed_distance=-remaining,
                curvature=0.0,
                step_m=self.config.sample_step_m,
                gear=-1,
            ))
        return path

    def _parallel_reverse_lane(
        self,
        stage: Pose2,
        goal: Pose2,
        side_sign: float,
    ) -> Optional[list[PathPoint]]:
        radius = self.config.min_turn_radius_m
        lateral = abs(goal.y)
        if lateral <= 0.02 or lateral >= 2.0 * radius:
            self.last_error = 'parallel_lateral_offset_outside_two_arc_geometry'
            return None
        theta = math.acos(max(-1.0, min(1.0, 1.0 - lateral / (2.0 * radius))))
        first_curvature = side_sign / radius
        first = sample_motion(
            stage,
            signed_distance=-radius * theta,
            curvature=first_curvature,
            step_m=self.config.sample_step_m,
            gear=-1,
        )
        second = sample_motion(
            first[-1].pose,
            signed_distance=-radius * theta,
            curvature=-first_curvature,
            step_m=self.config.sample_step_m,
            gear=-1,
        )
        return first + second

    def plan(
        self,
        current_pose_map: Pose2,
        space: ParkingSpace,
        obstacle_points_map: np.ndarray,
    ) -> Optional[ParkingPlan]:
        self.last_error = ''
        lane_pose = space.lane_pose_map
        current_lane = between(lane_pose, current_pose_map)
        goal_lane = between(lane_pose, space.goal_pose_map)
        side_sign = 1.0 if space.side == SIDE_LEFT else -1.0
        radius = self.config.min_turn_radius_m

        if abs(current_lane.y) > self.config.lane_lateral_tolerance_m:
            self.last_error = 'vehicle_too_far_from_scan_lane'
            return None
        if abs(current_lane.yaw) > self.config.lane_yaw_tolerance_rad:
            self.last_error = 'vehicle_not_aligned_with_scan_lane'
            return None

        if space.mode == MODE_PERPENDICULAR:
            if abs(goal_lane.y) < radius - 0.02:
                self.last_error = 'perpendicular_bay_too_shallow_for_minimum_radius'
                return None
            stage_lane = Pose2(goal_lane.x + radius, 0.0, 0.0)
            reverse_lane = self._perpendicular_reverse_lane(
                stage_lane, goal_lane, side_sign)
        elif space.mode == MODE_PARALLEL:
            lateral = abs(goal_lane.y)
            if lateral >= 2.0 * radius:
                self.last_error = 'parallel_offset_exceeds_two_turn_radii'
                return None
            theta = math.acos(max(-1.0, min(1.0, 1.0 - lateral / (2.0 * radius))))
            stage_lane = Pose2(
                goal_lane.x + 2.0 * radius * math.sin(theta), 0.0, 0.0)
            reverse_lane = self._parallel_reverse_lane(
                stage_lane, goal_lane, side_sign)
            if reverse_lane is None:
                return None
        else:
            self.last_error = f'unsupported_mode:{space.mode}'
            return None

        approach_lane: list[PathPoint] = []
        stage_delta = stage_lane.x - current_lane.x
        if stage_delta > self.config.stage_position_tolerance_m:
            approach_lane = sample_pose_line(
                current_lane, stage_lane, self.config.sample_step_m, gear=1)
        elif stage_delta < -self.config.stage_position_tolerance_m:
            # The vehicle has already passed the analytical reverse start.
            # Reversing straight along the scan lane reaches the same start
            # without adding a forward detour.
            lead = sample_pose_line(
                current_lane, stage_lane, self.config.sample_step_m, gear=-1)
            reverse_lane = lead + reverse_lane
        elif math.hypot(current_lane.x - stage_lane.x, current_lane.y - stage_lane.y) > 0.01:
            reverse_lane = sample_pose_line(
                current_lane, stage_lane, self.config.sample_step_m, gear=-1) + reverse_lane

        approach_map = deduplicate_path(path_from_frame(approach_lane, lane_pose))
        reverse_map = deduplicate_path(path_from_frame(reverse_lane, lane_pose))
        if not reverse_map:
            self.last_error = 'empty_reverse_path'
            return None

        obstacles = np.asarray(obstacle_points_map, dtype=np.float64)
        approach_collision = first_footprint_collision(
            approach_map, obstacles,
            self.config.vehicle_front_m,
            self.config.vehicle_rear_m,
            self.config.vehicle_width_m,
            self.config.static_clearance_m,
        )
        if approach_collision is not None:
            self.last_error = (
                f'approach_collision@{approach_collision.point_x:.2f},'
                f'{approach_collision.point_y:.2f}')
            return None
        reverse_collision = first_footprint_collision(
            reverse_map, obstacles,
            self.config.vehicle_front_m,
            self.config.vehicle_rear_m,
            self.config.vehicle_width_m,
            self.config.static_clearance_m,
        )
        if reverse_collision is not None:
            self.last_error = (
                f'reverse_collision@{reverse_collision.point_x:.2f},'
                f'{reverse_collision.point_y:.2f}')
            return None

        max_curvature = max(abs(point.curvature) for point in reverse_map)
        if max_curvature > 1.0 / radius + 1.0e-6:
            self.last_error = 'curvature_limit_violation'
            return None

        return ParkingPlan(
            approach_path=tuple(approach_map),
            reverse_path=tuple(reverse_map),
            stage_pose_map=path_from_frame([
                PathPoint(stage_lane.x, stage_lane.y, stage_lane.yaw, 0.0, 1)
            ], lane_pose)[0].pose,
            goal_pose_map=space.goal_pose_map,
            minimum_turn_radius_m=radius,
            mode=space.mode,
            side=space.side,
        )
