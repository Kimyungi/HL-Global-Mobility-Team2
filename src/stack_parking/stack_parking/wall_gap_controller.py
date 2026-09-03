"""ROS-independent controller for the fixed-wall gap parking experiment.

The controller owns only the motion that starts after a square has been
confirmed.  It immediately drives toward the outer end of the wall-parallel
segment, holds there, follows the fixed path in reverse into the bay, holds
again, then drives forward on a path mirrored about the bay's inward straight.
All reference points are converted from ``parking_map`` to the live
LiDAR-localized vehicle frame on every update.
"""

from __future__ import annotations

from dataclasses import dataclass
import enum
import math
from typing import Optional, Sequence

import numpy as np

from .geometry import (
    PathPoint,
    Pose2,
    between,
    closest_path_index,
    cumulative_lengths,
    local_reference,
    preview_path_point,
    wrap_angle,
)
from .reference_path import (
    ReferencePath,
    mirror_reference_path_about_inside_straight,
)


class ControlState(str, enum.Enum):
    IDLE = 'idle'
    FORWARD = 'forward_align'
    ALIGN_HOLD = 'forward_path_end_hold'
    REVERSE = 'reverse_park'
    PARKED_HOLD = 'reverse_path_end_hold'
    FORWARD_EXIT = 'forward_exit'
    STOPPED = 'stopped'


@dataclass(frozen=True)
class WallGapControlConfig:
    direction_change_hold_s: float = 1.0
    preview_distance_m: float = 1.5
    forward_speed_mps: float = 0.3
    reverse_speed_mps: float = 0.3
    sample_step_m: float = 0.05


@dataclass(frozen=True)
class WallGapControlOutput:
    state: ControlState
    reference_map: Optional[PathPoint]
    reference_local: Optional[PathPoint]
    v_ref_mps: float
    status: str


@dataclass(frozen=True)
class PoseDelta:
    dx: float
    dy: float
    dyaw: float
    update: int


class PoseDeltaTracker:
    """Track successive LiDAR poses using the TargetRef delta convention."""

    def __init__(self) -> None:
        self._previous: Optional[Pose2] = None
        self._delta = PoseDelta(0.0, 0.0, 0.0, 0)

    @property
    def delta(self) -> PoseDelta:
        return self._delta

    def reset(self, pose: Pose2) -> PoseDelta:
        self._previous = pose
        # The first valid localization sample is update=1 with a zero delta.
        self._delta = PoseDelta(0.0, 0.0, 0.0, 1)
        return self._delta

    def update(self, pose: Pose2) -> PoseDelta:
        if self._previous is None:
            return self.reset(pose)
        movement = between(self._previous, pose)
        self._previous = pose
        self._delta = PoseDelta(
            movement.x,
            movement.y,
            wrap_angle(movement.yaw),
            (self._delta.update + 1) & ((1 << 64) - 1),
        )
        return self._delta


def _can_local_reference(point: PathPoint, vehicle_pose: Pose2) -> PathPoint:
    """``local_reference`` plus the sign flip the CAN-bound vehicle frame
    needs for yaw/curvature (found on the vehicle, 2026-09-03: target_yaw
    came out negative where it needed to be positive). x/y stay as
    ``local_reference`` gives them — only yaw and curvature, which are the
    same handedness-dependent quantity (curvature = dyaw/ds), are negated.
    """
    local = local_reference(point, vehicle_pose)
    return PathPoint(
        local.x, local.y,
        wrap_angle(-local.yaw), -local.curvature, local.gear)


def _sample_line(
    start: np.ndarray,
    end: np.ndarray,
    yaw: float,
    curvature: float,
    gear: int,
    step_m: float,
) -> list[PathPoint]:
    distance = float(np.linalg.norm(end - start))
    count = max(1, int(math.ceil(distance / max(step_m, 1.0e-3))))
    return [PathPoint(
        float(start[0] + (end[0] - start[0]) * index / count),
        float(start[1] + (end[1] - start[1]) * index / count),
        wrap_angle(yaw),
        float(curvature),
        gear,
    ) for index in range(count + 1)]


def _append_without_duplicate(
    target: list[PathPoint],
    source: Sequence[PathPoint],
) -> None:
    for point in source:
        if target and math.hypot(
            point.x - target[-1].x, point.y - target[-1].y
        ) < 1.0e-6:
            continue
        target.append(point)


def controller_paths(
    reference: ReferencePath,
    step_m: float = 0.05,
) -> tuple[tuple[PathPoint, ...], tuple[PathPoint, ...]]:
    """Build forward E->S and reverse S->E->P0->G vehicle-pose paths."""
    start = np.asarray(reference.straight1_map[0], dtype=np.float64)
    tangent_point = np.asarray(reference.e_map, dtype=np.float64)
    goal = np.asarray(reference.goal_map, dtype=np.float64)
    p0 = np.asarray(reference.p0_map, dtype=np.float64)
    center = np.asarray(reference.center_map, dtype=np.float64)

    body_direction = start - tangent_point
    norm = float(np.linalg.norm(body_direction))
    if norm <= 1.0e-9:
        return (), ()
    body_direction /= norm
    wall_yaw = math.atan2(body_direction[1], body_direction[0])

    forward = _sample_line(
        tangent_point, start, wall_yaw, 0.0, 1, step_m)

    reverse: list[PathPoint] = []
    _append_without_duplicate(reverse, _sample_line(
        start, tangent_point, wall_yaw, 0.0, -1, step_m))

    arc_xy = np.asarray(reference.arc_map, dtype=np.float64)
    if len(arc_xy) < 2:
        return tuple(forward), ()
    radius_angles = np.arctan2(arc_xy[:, 1] - center[1],
                               arc_xy[:, 0] - center[0])
    angle_progress = [0.0]
    for previous, current in zip(radius_angles, radius_angles[1:]):
        angle_progress.append(
            angle_progress[-1] + wrap_angle(float(current - previous)))
    sweep = angle_progress[-1]
    arc_distance = abs(sweep) * reference.radius_m
    if arc_distance <= 1.0e-9:
        return tuple(forward), ()
    # yaw_rate = velocity * curvature.  The arc is traversed with negative
    # velocity, while vehicle body yaw changes by the geometric circle sweep.
    reverse_curvature = -sweep / arc_distance
    arc_points = [PathPoint(
        float(point[0]),
        float(point[1]),
        wrap_angle(wall_yaw + progress),
        float(reverse_curvature),
        -1,
    ) for point, progress in zip(arc_xy, angle_progress)]
    _append_without_duplicate(reverse, arc_points)

    final_yaw = wrap_angle(wall_yaw + sweep)
    _append_without_duplicate(reverse, _sample_line(
        p0, goal, final_yaw, 0.0, -1, step_m))
    return tuple(forward), tuple(reverse)


def mirrored_forward_exit_path(
    reference: ReferencePath,
    step_m: float = 0.05,
) -> tuple[Optional[ReferencePath], tuple[PathPoint, ...]]:
    """Build G->P0->mirrored arc->mirrored S for forward pull-out.

    Mirroring uses the straight that enters the bay as its axis.  Reversing
    the mirrored parking traversal makes the new path start where the reverse
    parking leg ended.  Vehicle yaw and curvature remain valid when the gear
    changes from reverse to forward because traversal direction is reversed
    at the same time.
    """
    mirrored = mirror_reference_path_about_inside_straight(reference)
    if mirrored is None:
        return None, ()
    _, mirrored_reverse = controller_paths(mirrored, step_m)
    forward_exit = tuple(PathPoint(
        point.x,
        point.y,
        point.yaw,
        point.curvature,
        1,
    ) for point in reversed(mirrored_reverse))
    return mirrored, forward_exit


class WallGapController:
    def __init__(self, config: Optional[WallGapControlConfig] = None):
        self.config = config or WallGapControlConfig()
        self.state = ControlState.IDLE
        self.forward_path: tuple[PathPoint, ...] = ()
        self.reverse_path: tuple[PathPoint, ...] = ()
        self.exit_path: tuple[PathPoint, ...] = ()
        self.forward_lengths: list[float] = []
        self.reverse_lengths: list[float] = []
        self.exit_lengths: list[float] = []
        self.progress = 0
        self.hold_started_at_s = 0.0
        self.entry_reference_path: Optional[ReferencePath] = None
        self.exit_reference_path: Optional[ReferencePath] = None
        self._last_reference_map: Optional[PathPoint] = None
        self.stop_reason = ''

    @property
    def active(self) -> bool:
        return self.state != ControlState.IDLE

    def start(
        self,
        path: ReferencePath,
        current_pose_map: Pose2,
        now_s: float,
    ) -> bool:
        forward, reverse = controller_paths(path, self.config.sample_step_m)
        if not forward or not reverse:
            return False
        self.forward_path = forward
        self.reverse_path = reverse
        self.exit_path = ()
        self.forward_lengths = cumulative_lengths(forward)
        self.reverse_lengths = cumulative_lengths(reverse)
        self.exit_lengths = []
        self.progress = closest_path_index(forward, current_pose_map)
        self.hold_started_at_s = 0.0
        self.entry_reference_path = path
        self.exit_reference_path = None
        # Space confirmation itself no longer inserts a stop.  The first
        # command produced after start() is the forward alignment command.
        self.state = ControlState.FORWARD
        self.stop_reason = ''
        self._last_reference_map = self._preview(
            forward, self.forward_lengths, current_pose_map)
        return True

    def _preview(
        self,
        path: Sequence[PathPoint],
        lengths: Sequence[float],
        pose: Pose2,
    ) -> PathPoint:
        self.progress = closest_path_index(path, pose, self.progress)
        return preview_path_point(
            path, lengths, self.progress, self.config.preview_distance_m)

    def _output(
        self,
        pose: Pose2,
        reference: Optional[PathPoint],
        speed: float,
        status: str,
    ) -> WallGapControlOutput:
        if reference is not None:
            self._last_reference_map = reference
        chosen = reference if reference is not None else self._last_reference_map
        return WallGapControlOutput(
            self.state,
            chosen,
            _can_local_reference(chosen, pose) if chosen is not None else None,
            float(speed),
            status,
        )

    def update(
        self,
        current_pose_map: Pose2,
        now_s: float,
    ) -> WallGapControlOutput:
        if self.state == ControlState.IDLE:
            return self._output(current_pose_map, None, 0.0, 'idle')

        if self.state == ControlState.FORWARD:
            reference = self._preview(
                self.forward_path, self.forward_lengths, current_pose_map)
            end = self.forward_path[-1]
            # The requested switch condition is the preview point reaching
            # the outer end S, rather than the vehicle itself reaching S.
            preview_at_end = math.hypot(
                reference.x - end.x, reference.y - end.y) <= 1.0e-6
            if not preview_at_end:
                return self._output(
                    current_pose_map, reference,
                    self.config.forward_speed_mps, 'forward_alignment')

            self.state = ControlState.ALIGN_HOLD
            self.hold_started_at_s = float(now_s)
            return self._output(
                current_pose_map, reference, 0.0,
                'forward_path_end_hold:0.00/%.2fs'
                % self.config.direction_change_hold_s)

        if self.state == ControlState.ALIGN_HOLD:
            reference = self.forward_path[-1]
            elapsed = float(now_s) - self.hold_started_at_s
            if elapsed < self.config.direction_change_hold_s:
                return self._output(
                    current_pose_map, reference, 0.0,
                    'forward_path_end_hold:%.2f/%.2fs' % (
                        max(0.0, elapsed),
                        self.config.direction_change_hold_s,
                    ))
            self.state = ControlState.REVERSE
            self.progress = closest_path_index(
                self.reverse_path, current_pose_map)

        if self.state == ControlState.REVERSE:
            reference = self._preview(
                self.reverse_path, self.reverse_lengths, current_pose_map)
            end = self.reverse_path[-1]
            # Same stop rule as the forward leg's own state switch: the
            # preview point reaching the last straight segment's end,
            # not the vehicle itself reaching it (user directive,
            # 2026-09-03). Never continue reversing beyond the finite 2m
            # path even if the expected wall return is missing or
            # miscalibrated.
            preview_at_end = math.hypot(
                reference.x - end.x, reference.y - end.y) <= 1.0e-6
            if preview_at_end:
                if self.entry_reference_path is None:
                    self.state = ControlState.STOPPED
                    self.stop_reason = 'missing_entry_reference_path'
                    return self._output(
                        current_pose_map, reference, 0.0,
                        self.stop_reason)
                mirrored, exit_path = mirrored_forward_exit_path(
                    self.entry_reference_path, self.config.sample_step_m)
                if mirrored is None or not exit_path:
                    self.state = ControlState.STOPPED
                    self.stop_reason = 'exit_reference_path_failed'
                    return self._output(
                        current_pose_map, reference, 0.0,
                        self.stop_reason)
                self.exit_reference_path = mirrored
                self.exit_path = exit_path
                self.exit_lengths = cumulative_lengths(exit_path)
                self.progress = closest_path_index(
                    self.exit_path, current_pose_map)
                self.state = ControlState.PARKED_HOLD
                self.hold_started_at_s = float(now_s)
                reference = self._preview(
                    self.exit_path, self.exit_lengths, current_pose_map)
                return self._output(
                    current_pose_map, reference, 0.0,
                    'reverse_path_end_hold:0.00/%.2fs'
                    % self.config.direction_change_hold_s)
            return self._output(
                current_pose_map, reference,
                -self.config.reverse_speed_mps, 'reverse_parking')

        if self.state == ControlState.PARKED_HOLD:
            reference = self._preview(
                self.exit_path, self.exit_lengths, current_pose_map)
            elapsed = float(now_s) - self.hold_started_at_s
            if elapsed < self.config.direction_change_hold_s:
                return self._output(
                    current_pose_map, reference, 0.0,
                    'reverse_path_end_hold:%.2f/%.2fs' % (
                        max(0.0, elapsed),
                        self.config.direction_change_hold_s,
                    ))
            self.state = ControlState.FORWARD_EXIT

        if self.state == ControlState.FORWARD_EXIT:
            reference = self._preview(
                self.exit_path, self.exit_lengths, current_pose_map)
            end = self.exit_path[-1]
            preview_at_end = math.hypot(
                reference.x - end.x, reference.y - end.y) <= 1.0e-6
            if preview_at_end:
                self.state = ControlState.STOPPED
                self.stop_reason = 'exit_path_end'
                return self._output(
                    current_pose_map, reference, 0.0,
                    'exit_path_end_stop')
            return self._output(
                current_pose_map, reference,
                self.config.forward_speed_mps, 'forward_exit')

        return self._output(
            current_pose_map, self._last_reference_map, 0.0,
            self.stop_reason or 'stopped')
