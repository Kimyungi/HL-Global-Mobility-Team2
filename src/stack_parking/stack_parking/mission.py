"""Parking mission state machine, independent of ROS."""

from __future__ import annotations

from dataclasses import dataclass
import enum
import math
from typing import Optional, Sequence

import numpy as np

from .geometry import (
    PathPoint,
    Pose2,
    closest_path_index,
    cumulative_lengths,
    deduplicate_path,
    local_reference,
    preview_index,
    preview_path_point,
)
from .path_planner import (
    MinimumRadiusParkingPlanner,
    ParkingPlan,
    first_footprint_collision,
)
from .space_detector import ParkingSpace, ParkingSpaceDetector


class MissionState(str, enum.Enum):
    IDLE = 'idle'
    SCANNING = 'scanning'
    APPROACH = 'approach'
    REVERSE = 'reverse'
    PARKED_WAIT = 'parked_wait'
    EXIT = 'exit'
    COMPLETE = 'complete'


@dataclass
class MissionConfig:
    preview_distance_m: float = 1.0
    forward_speed_mps: float = 0.60
    reverse_turn_speed_mps: float = 0.55
    reverse_dock_speed_mps: float = 0.15
    dock_slow_distance_m: float = 0.55
    dock_curvature_threshold: float = 0.05
    exit_speed_mps: float = 0.55
    path_end_tolerance_m: float = 0.14
    completion_clearance_m: float = 0.20
    parked_wait_s: float = 5.0
    stationary_speed_mps: float = 0.035
    require_stationary_feedback: bool = True
    complete_latch_s: float = 1.0
    dynamic_static_match_m: float = 0.14
    dynamic_clearance_m: float = 0.12
    dynamic_confirm_frames: int = 2
    dynamic_path_horizon_m: float = 1.5


@dataclass(frozen=True)
class MissionOutput:
    state: MissionState
    space_found: bool
    path_blocked: bool
    done: bool
    reference_local: Optional[PathPoint]
    v_suggest_mps: float
    progress_index: int
    preview_index: int
    status: str


def _novel_points(
    observed: np.ndarray,
    static_map: np.ndarray,
    threshold_m: float,
) -> np.ndarray:
    observed = np.asarray(observed, dtype=np.float64)
    static_map = np.asarray(static_map, dtype=np.float64)
    if observed.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    if static_map.size == 0:
        return observed[:, :2]
    threshold2 = threshold_m * threshold_m
    novel = np.ones(len(observed), dtype=bool)
    # Chunking bounds peak memory for a several-thousand-point local map.
    for first in range(0, len(observed), 128):
        chunk = observed[first:first + 128, :2]
        best = np.full(len(chunk), math.inf)
        for map_first in range(0, len(static_map), 1024):
            target = static_map[map_first:map_first + 1024, :2]
            delta = chunk[:, None, :] - target[None, :, :]
            best = np.minimum(best, np.min(np.einsum('ijk,ijk->ij', delta, delta), axis=1))
        novel[first:first + len(chunk)] = best > threshold2
    return observed[novel, :2]


class ParkingMission:
    def __init__(
        self,
        detector: ParkingSpaceDetector,
        planner: MinimumRadiusParkingPlanner,
        config: Optional[MissionConfig] = None,
    ):
        self.detector = detector
        self.planner = planner
        self.config = config or MissionConfig()
        self.state = MissionState.IDLE
        self.mode = ''
        self.side = ''
        self.lane_pose_map = Pose2()
        self.space: Optional[ParkingSpace] = None
        self.plan: Optional[ParkingPlan] = None
        self.static_map = np.empty((0, 2), dtype=np.float64)
        self.current_path: tuple[PathPoint, ...] = ()
        self.current_lengths: list[float] = []
        self.progress = 0
        self._dynamic_count = 0
        self.path_blocked = False
        self._wait_started_at: Optional[float] = None
        self._completed_at: Optional[float] = None
        self._exit_from_index = 0
        self.last_plan_error = ''

    def reset(self) -> None:
        self.detector.reset()
        self.state = MissionState.IDLE
        self.mode = ''
        self.side = ''
        self.space = None
        self.plan = None
        self.static_map = np.empty((0, 2), dtype=np.float64)
        self.current_path = ()
        self.current_lengths = []
        self.progress = 0
        self._dynamic_count = 0
        self.path_blocked = False
        self._wait_started_at = None
        self._completed_at = None
        self._exit_from_index = 0
        self.last_plan_error = ''

    def trigger(self, mode: str, side: str, current_pose_map: Pose2) -> bool:
        if self.state not in (MissionState.IDLE, MissionState.COMPLETE):
            return False
        self.reset()
        self.mode = mode
        self.side = side
        self.lane_pose_map = current_pose_map
        self.state = MissionState.SCANNING
        return True

    def cancel(self) -> None:
        self.reset()

    def observe_map(self, map_points: np.ndarray, current_pose_map: Pose2) -> bool:
        """Update gap detection and create a plan; return True on plan creation."""
        if self.state != MissionState.SCANNING:
            return False
        space = self.detector.update(
            map_points,
            current_pose_map,
            self.lane_pose_map,
            self.mode,
            self.side,
        )
        if space is None:
            return False
        plan = self.planner.plan(current_pose_map, space, map_points)
        if plan is None:
            self.last_plan_error = self.planner.last_error
            return False

        self.space = space
        self.plan = plan
        self.last_plan_error = ''
        self.static_map = np.asarray(map_points, dtype=np.float64).copy()
        if plan.approach_path:
            self._set_path(plan.approach_path)
            self.state = MissionState.APPROACH
        else:
            self._set_path(plan.reverse_path)
            self.state = MissionState.REVERSE
        return True

    def _set_path(self, path: Sequence[PathPoint]) -> None:
        self.current_path = tuple(path)
        self.current_lengths = cumulative_lengths(self.current_path)
        self.progress = 0

    def observe_dynamic(self, observed_scan_map: np.ndarray) -> bool:
        if self.state not in (
            MissionState.APPROACH, MissionState.REVERSE, MissionState.EXIT
        ) or not self.current_path:
            self._dynamic_count = 0
            self.path_blocked = False
            return False
        novel = _novel_points(
            observed_scan_map,
            self.static_map,
            self.config.dynamic_static_match_m,
        )
        if len(novel) == 0:
            self._dynamic_count = 0
            self.path_blocked = False
            return False

        end = self.progress
        distance = 0.0
        while end + 1 < len(self.current_path) and distance < self.config.dynamic_path_horizon_m:
            distance += math.hypot(
                self.current_path[end + 1].x - self.current_path[end].x,
                self.current_path[end + 1].y - self.current_path[end].y,
            )
            end += 1
        collision = first_footprint_collision(
            self.current_path[self.progress:end + 1],
            novel,
            self.planner.config.vehicle_front_m,
            self.planner.config.vehicle_rear_m,
            self.planner.config.vehicle_width_m,
            self.config.dynamic_clearance_m,
        )
        if collision is None:
            self._dynamic_count = 0
        else:
            self._dynamic_count += 1
        self.path_blocked = self._dynamic_count >= self.config.dynamic_confirm_frames
        return self.path_blocked

    def _at_path_end(self, pose: Pose2) -> bool:
        if not self.current_path:
            return True
        end = self.current_path[-1]
        return (
            self.progress >= len(self.current_path) - 2
            and math.hypot(end.x - pose.x, end.y - pose.y)
            <= self.config.path_end_tolerance_m
        )

    def _at_docking_end(self, pose: Pose2) -> bool:
        """Tight end check: do not stop one sample before the 20cm trigger."""
        if not self.current_path:
            return True
        end = self.current_path[-1]
        return (
            self.progress >= len(self.current_path) - 1
            and math.hypot(end.x - pose.x, end.y - pose.y) <= 0.03
        )

    def _update_progress(self, pose: Pose2) -> tuple[int, int]:
        if not self.current_path:
            return 0, 0
        self.progress = closest_path_index(
            self.current_path, pose, self.progress)
        preview = preview_index(
            self.current_path,
            self.current_lengths,
            self.progress,
            self.config.preview_distance_m,
        )
        return self.progress, preview

    def _remaining_path_m(self) -> float:
        if not self.current_lengths:
            return 0.0
        return self.current_lengths[-1] - self.current_lengths[min(
            self.progress, len(self.current_lengths) - 1)]

    def _local_preview(self, pose: Pose2) -> tuple[PathPoint, int]:
        preview = preview_index(
            self.current_path,
            self.current_lengths,
            self.progress,
            self.config.preview_distance_m,
        )
        point = preview_path_point(
            self.current_path,
            self.current_lengths,
            self.progress,
            self.config.preview_distance_m,
        )
        return local_reference(point, pose), preview

    def _make_exit_path(self) -> tuple[PathPoint, ...]:
        if self.plan is None or not self.plan.reverse_path:
            return ()
        last = min(self._exit_from_index, len(self.plan.reverse_path) - 1)
        travelled = self.plan.reverse_path[:last + 1]
        reversed_points = [
            PathPoint(point.x, point.y, point.yaw, point.curvature, 1)
            for point in reversed(travelled)
        ]
        return tuple(deduplicate_path(reversed_points))

    def tick(
        self,
        current_pose_map: Pose2,
        now_s: float,
        rear_clearance_m: Optional[float] = None,
        vehicle_speed_mps: Optional[float] = None,
        localization_valid: bool = True,
    ) -> MissionOutput:
        if self.state == MissionState.IDLE:
            return MissionOutput(
                self.state, False, False, False, None, 0.0, 0, 0, 'idle')
        if self.state == MissionState.SCANNING:
            status = 'scanning_map'
            if self.last_plan_error:
                status = f'plan_rejected:{self.last_plan_error}'
            return MissionOutput(
                self.state, False, False, False, None, 0.0, 0, 0, status)

        if not localization_valid:
            return MissionOutput(
                self.state, True, self.path_blocked, False, None, 0.0,
                self.progress, self.progress, 'localization_invalid_stop')

        if self.state == MissionState.APPROACH:
            _, preview = self._update_progress(current_pose_map)
            if self._at_path_end(current_pose_map):
                assert self.plan is not None
                self._set_path(self.plan.reverse_path)
                self.state = MissionState.REVERSE
                _, preview = self._update_progress(current_pose_map)
            reference, preview = self._local_preview(current_pose_map)
            return MissionOutput(
                self.state, True, self.path_blocked, False,
                reference,
                0.0 if self.path_blocked else self.config.forward_speed_mps,
                self.progress, preview, 'approach_reverse_start')

        if self.state == MissionState.REVERSE:
            _, preview = self._update_progress(current_pose_map)
            if (
                rear_clearance_m is not None
                and rear_clearance_m <= self.config.completion_clearance_m + 1.0e-6
            ):
                self._exit_from_index = self.progress
                self.state = MissionState.PARKED_WAIT
                self._wait_started_at = None
                return MissionOutput(
                    self.state, True, False, False, None, 0.0,
                    self.progress, preview, 'rear_wall_reached_stop')

            reference, preview = self._local_preview(current_pose_map)
            if self._at_docking_end(current_pose_map):
                speed = 0.0
                status = 'planned_end_waiting_for_rear_wall'
            elif (
                self._remaining_path_m() <= self.config.dock_slow_distance_m
                and abs(reference.curvature) <= self.config.dock_curvature_threshold
            ):
                speed = -self.config.reverse_dock_speed_mps
                status = 'reverse_docking'
            else:
                speed = -self.config.reverse_turn_speed_mps
                status = 'reverse_maneuver'
            if self.path_blocked:
                speed = 0.0
                status = 'dynamic_path_blocked'
            return MissionOutput(
                self.state, True, self.path_blocked, False,
                reference, speed,
                self.progress, preview, status)

        if self.state == MissionState.PARKED_WAIT:
            stationary = (
                (vehicle_speed_mps is not None
                 and abs(vehicle_speed_mps) <= self.config.stationary_speed_mps)
                or (vehicle_speed_mps is None
                    and not self.config.require_stationary_feedback)
            )
            if stationary and self._wait_started_at is None:
                self._wait_started_at = now_s
            if not stationary:
                self._wait_started_at = None
            elapsed = 0.0 if self._wait_started_at is None else now_s - self._wait_started_at
            if elapsed >= self.config.parked_wait_s:
                exit_path = self._make_exit_path()
                if exit_path:
                    self._set_path(exit_path)
                    self.state = MissionState.EXIT
                    _, preview = self._update_progress(current_pose_map)
                    reference, preview = self._local_preview(current_pose_map)
                    return MissionOutput(
                        self.state, True, False, False,
                        reference,
                        self.config.exit_speed_mps,
                        self.progress, preview, 'pull_out_replay')
            return MissionOutput(
                self.state, True, False, False, None, 0.0,
                self.progress, self.progress,
                f'parked_wait:{elapsed:.1f}/{self.config.parked_wait_s:.1f}s')

        if self.state == MissionState.EXIT:
            _, preview = self._update_progress(current_pose_map)
            if self._at_path_end(current_pose_map):
                self.state = MissionState.COMPLETE
                self._completed_at = now_s
                return MissionOutput(
                    self.state, False, False, True, None, 0.0,
                    self.progress, preview, 'parking_complete_line_mode')
            reference, preview = self._local_preview(current_pose_map)
            speed = 0.0 if self.path_blocked else self.config.exit_speed_mps
            return MissionOutput(
                self.state, True, self.path_blocked, False,
                reference, speed,
                self.progress, preview,
                'dynamic_path_blocked' if self.path_blocked else 'pull_out_replay')

        if self.state == MissionState.COMPLETE:
            if (
                self._completed_at is not None
                and now_s - self._completed_at >= self.config.complete_latch_s
            ):
                self.reset()
                return MissionOutput(
                    self.state, False, False, False, None, 0.0, 0, 0, 'idle')
            return MissionOutput(
                self.state, False, False, True, None, 0.0,
                self.progress, self.progress, 'parking_complete_line_mode')

        raise RuntimeError(f'unhandled mission state: {self.state}')

    def debug_path(self) -> tuple[PathPoint, ...]:
        if self.plan is None:
            return ()
        if self.state == MissionState.EXIT:
            return self.current_path
        return self.plan.full_entry_path
