"""ROS-independent geometry and control for the parallel-parking test.

The valid point P0 is the midpoint between the two wall faces and the midpoint
of the wall-side edge of a 1.5m x 0.7m validation rectangle.  The S-curve
origin is shifted 0.25m from P0 along the wall tangent oriented toward the
vehicle's travel direction; the vehicle yaw itself is not used as its slope. A
45-degree arc of radius 1.12m is constructed there, then rotated 180 degrees
about the shifted origin.  Two-metre wall-parallel lines extend both ends.
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
    closest_path_index,
    cumulative_lengths,
    preview_path_point,
    wrap_angle,
)
from .wall_gap_controller import WallGapControlOutput, _can_local_reference
from .wall_gap_detector import TrackedCandidate


@dataclass(frozen=True)
class ParallelReferencePath:
    side: str
    radius_m: float
    p0_map: tuple[float, float]
    arc_origin_map: tuple[float, float]
    front_center_map: tuple[float, float]
    rear_center_map: tuple[float, float]
    front_tangent_map: tuple[float, float]
    rear_tangent_map: tuple[float, float]
    front_end_map: tuple[float, float]
    rear_end_map: tuple[float, float]
    rear_line_map: np.ndarray
    rear_arc_map: np.ndarray
    front_arc_map: np.ndarray
    front_line_map: np.ndarray


@dataclass(frozen=True)
class ParallelParkingConfig:
    direction_change_hold_s: float = 1.0
    preview_distance_m: float = 1.5
    forward_speed_mps: float = 0.75
    reverse_speed_mps: float = 0.75
    sample_step_m: float = 0.05


class ParallelControlState(str, enum.Enum):
    IDLE = 'idle'
    FORWARD = 'parallel_forward_lead'
    FRONT_HOLD = 'parallel_front_end_hold'
    REVERSE = 'parallel_reverse_park'
    REAR_HOLD = 'parallel_reverse_end_hold'
    FORWARD_RETURN = 'parallel_forward_return'
    FINAL_HOLD = 'parallel_final_end_hold'
    STOPPED = 'parallel_stopped'


def rectangle_is_clear(
    points_wall: np.ndarray,
    center_s: float,
    wall_length_m: float = 1.5,
    inward_depth_m: float = 0.7,
) -> bool:
    """Check a wall-aligned rectangle extending inward from the wall line."""
    half_length = 0.5 * float(wall_length_m)
    depth = float(inward_depth_m)
    if half_length <= 0.0 or depth <= 0.0:
        return False
    points = np.asarray(points_wall, dtype=np.float64)
    inside = (
        (points[:, 0] >= center_s - half_length)
        & (points[:, 0] <= center_s + half_length)
        & (points[:, 1] >= 0.0)
        & (points[:, 1] <= depth)
    )
    return not bool(np.any(inside))


def candidate_rectangle_corners(
    candidate: TrackedCandidate,
    wall_length_m: float = 1.5,
    inward_depth_m: float = 0.7,
) -> np.ndarray:
    """Return the validation rectangle whose wall-side midpoint is P0."""
    half_length = 0.5 * float(wall_length_m)
    center = np.array([candidate.map_x, candidate.map_y], dtype=np.float64)
    tangent = np.array([
        candidate.wall_tangent_x, candidate.wall_tangent_y], dtype=np.float64)
    inward = np.array([
        candidate.wall_normal_x, candidate.wall_normal_y], dtype=np.float64)
    return np.asarray([
        center - half_length * tangent,
        center + half_length * tangent,
        center + half_length * tangent + inward_depth_m * inward,
        center - half_length * tangent + inward_depth_m * inward,
        center - half_length * tangent,
    ])


def _rotate(vector: np.ndarray, angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([
        c * vector[0] - s * vector[1],
        s * vector[0] + c * vector[1],
    ])


def build_parallel_reference_path(
    candidate: TrackedCandidate,
    vehicle_pose: Pose2,
    turn_radius_m: float = 1.12,
    end_straight_m: float = 2.0,
    arc_angle_deg: float = 45.0,
    arc_start_offset_m: float = 0.25,
    arc_points: int = 24,
) -> Optional[ParallelReferencePath]:
    """Construct the point-symmetric S path requested for parallel parking."""
    radius = float(turn_radius_m)
    line_length = float(end_straight_m)
    arc_start_offset = float(arc_start_offset_m)
    arc_angle = math.radians(float(arc_angle_deg))
    if (radius <= 0.0 or line_length <= 0.0 or arc_start_offset < 0.0
            or not 0.0 < arc_angle < math.pi / 2.0):
        return None

    p0 = np.array([candidate.map_x, candidate.map_y], dtype=np.float64)
    wall_tangent = np.array([
        candidate.wall_tangent_x, candidate.wall_tangent_y], dtype=np.float64)
    inward = np.array([
        candidate.wall_normal_x, candidate.wall_normal_y], dtype=np.float64)
    tangent_norm = float(np.linalg.norm(wall_tangent))
    inward_norm = float(np.linalg.norm(inward))
    if tangent_norm <= 1.0e-9 or inward_norm <= 1.0e-9:
        return None
    wall_tangent /= tangent_norm
    inward /= inward_norm

    vehicle_forward = np.array([
        math.cos(vehicle_pose.yaw), math.sin(vehicle_pose.yaw)])
    # Preserve the fitted wall's slope. Vehicle yaw only selects which of the
    # two wall-tangent directions is the forward-travel direction.
    direction = 1.0 if float(np.dot(vehicle_forward, wall_tangent)) >= 0.0 else -1.0
    tangent = direction * wall_tangent
    arc_origin = p0 + arc_start_offset * tangent

    # For 45 degrees, arc_origin->centre is radius/sqrt(2) along both the
    # forward wall tangent and inward normal.  The formulation below also
    # keeps the angle parameter explicit for geometry validation.
    front_center = arc_origin + radius * (
        math.cos(arc_angle) * tangent + math.sin(arc_angle) * inward)
    front_tangent = front_center - radius * inward
    radius_at_origin = arc_origin - front_center
    radius_at_end = front_tangent - front_center
    sweep = math.atan2(
        radius_at_origin[0] * radius_at_end[1]
        - radius_at_origin[1] * radius_at_end[0],
        float(np.dot(radius_at_origin, radius_at_end)),
    )
    angles = np.linspace(0.0, sweep, max(2, int(arc_points)))
    front_arc = np.asarray([
        front_center + _rotate(radius_at_origin, float(angle))
        for angle in angles
    ])

    # The second arc is an exact 180-degree rotation of the first about the
    # shifted origin. Reverse samples so the complete path is rear->front.
    rear_center = 2.0 * arc_origin - front_center
    rear_arc_from_origin = 2.0 * arc_origin - front_arc
    rear_arc = rear_arc_from_origin[::-1].copy()
    rear_tangent = rear_arc[0]
    front_end = front_tangent + line_length * tangent
    rear_end = rear_tangent - line_length * tangent

    return ParallelReferencePath(
        side=candidate.side,
        radius_m=radius,
        p0_map=(float(p0[0]), float(p0[1])),
        arc_origin_map=(float(arc_origin[0]), float(arc_origin[1])),
        front_center_map=(float(front_center[0]), float(front_center[1])),
        rear_center_map=(float(rear_center[0]), float(rear_center[1])),
        front_tangent_map=(float(front_tangent[0]), float(front_tangent[1])),
        rear_tangent_map=(float(rear_tangent[0]), float(rear_tangent[1])),
        front_end_map=(float(front_end[0]), float(front_end[1])),
        rear_end_map=(float(rear_end[0]), float(rear_end[1])),
        rear_line_map=np.asarray([rear_end, rear_tangent]),
        rear_arc_map=rear_arc,
        front_arc_map=front_arc,
        front_line_map=np.asarray([front_tangent, front_end]),
    )


def _sample_line(
    start: np.ndarray,
    end: np.ndarray,
    yaw: float,
    gear: int,
    step_m: float,
) -> list[PathPoint]:
    distance = float(np.linalg.norm(end - start))
    count = max(1, int(math.ceil(distance / max(step_m, 1.0e-3))))
    return [PathPoint(
        float(start[0] + (end[0] - start[0]) * index / count),
        float(start[1] + (end[1] - start[1]) * index / count),
        wrap_angle(yaw),
        0.0,
        gear,
    ) for index in range(count + 1)]


def _arc_path_points(
    points: np.ndarray,
    center: np.ndarray,
    radius_m: float,
    gear: int,
) -> list[PathPoint]:
    radius_angles = np.arctan2(
        points[:, 1] - center[1], points[:, 0] - center[0])
    progress = [0.0]
    for previous, current in zip(radius_angles, radius_angles[1:]):
        progress.append(progress[-1] + wrap_angle(float(current - previous)))
    sweep = progress[-1]
    if abs(sweep) <= 1.0e-9:
        return []
    turn_sign = 1.0 if sweep > 0.0 else -1.0
    curvature = turn_sign / radius_m
    return [PathPoint(
        float(point[0]),
        float(point[1]),
        wrap_angle(float(angle) + turn_sign * math.pi / 2.0),
        curvature,
        gear,
    ) for point, angle in zip(points, radius_angles)]


def _append_without_duplicate(
    target: list[PathPoint], source: Sequence[PathPoint],
) -> None:
    for point in source:
        if target and math.hypot(
            point.x - target[-1].x, point.y - target[-1].y,
        ) < 1.0e-6:
            continue
        target.append(point)


def parallel_controller_paths(
    reference: ParallelReferencePath,
    step_m: float = 0.05,
) -> tuple[tuple[PathPoint, ...], tuple[PathPoint, ...]]:
    """Return forward rear->front and reverse front->rear path samples."""
    rear_end = np.asarray(reference.rear_end_map, dtype=np.float64)
    rear_tangent = np.asarray(reference.rear_tangent_map, dtype=np.float64)
    front_tangent = np.asarray(reference.front_tangent_map, dtype=np.float64)
    front_end = np.asarray(reference.front_end_map, dtype=np.float64)
    direction = front_end - front_tangent
    direction_norm = float(np.linalg.norm(direction))
    if direction_norm <= 1.0e-9:
        return (), ()
    direction /= direction_norm
    wall_yaw = math.atan2(direction[1], direction[0])

    forward: list[PathPoint] = []
    _append_without_duplicate(forward, _sample_line(
        rear_end, rear_tangent, wall_yaw, 1, step_m))
    _append_without_duplicate(forward, _arc_path_points(
        reference.rear_arc_map,
        np.asarray(reference.rear_center_map),
        reference.radius_m,
        1,
    ))
    _append_without_duplicate(forward, _arc_path_points(
        reference.front_arc_map,
        np.asarray(reference.front_center_map),
        reference.radius_m,
        1,
    ))
    _append_without_duplicate(forward, _sample_line(
        front_tangent, front_end, wall_yaw, 1, step_m))
    reverse = tuple(PathPoint(
        point.x, point.y, point.yaw, point.curvature, -1)
        for point in reversed(forward))
    return tuple(forward), reverse


class ParallelParkingController:
    def __init__(self, config: Optional[ParallelParkingConfig] = None):
        self.config = config or ParallelParkingConfig()
        self.state = ParallelControlState.IDLE
        self.forward_path: tuple[PathPoint, ...] = ()
        self.reverse_path: tuple[PathPoint, ...] = ()
        self.forward_lengths: list[float] = []
        self.reverse_lengths: list[float] = []
        self.progress = 0
        self.hold_started_at_s = 0.0
        self._last_reference_map: Optional[PathPoint] = None
        self.stop_reason = ''

    @property
    def active(self) -> bool:
        return self.state != ParallelControlState.IDLE

    def start(
        self,
        path: ParallelReferencePath,
        current_pose_map: Pose2,
        now_s: float,
    ) -> bool:
        forward, reverse = parallel_controller_paths(
            path, self.config.sample_step_m)
        if not forward or not reverse:
            return False
        self.forward_path = forward
        self.reverse_path = reverse
        self.forward_lengths = cumulative_lengths(forward)
        self.reverse_lengths = cumulative_lengths(reverse)
        self.progress = closest_path_index(forward, current_pose_map)
        self.state = ParallelControlState.FORWARD
        self.hold_started_at_s = float(now_s)
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

    @staticmethod
    def _at_end(reference: PathPoint, path: Sequence[PathPoint]) -> bool:
        end = path[-1]
        return math.hypot(
            reference.x - end.x, reference.y - end.y) <= 1.0e-6

    def update(
        self,
        current_pose_map: Pose2,
        now_s: float,
    ) -> WallGapControlOutput:
        if self.state == ParallelControlState.IDLE:
            return self._output(current_pose_map, None, 0.0, 'idle')

        if self.state == ParallelControlState.FORWARD:
            reference = self._preview(
                self.forward_path, self.forward_lengths, current_pose_map)
            if not self._at_end(reference, self.forward_path):
                return self._output(
                    current_pose_map, reference,
                    self.config.forward_speed_mps, 'parallel_forward_lead')
            self.state = ParallelControlState.FRONT_HOLD
            self.hold_started_at_s = float(now_s)
            return self._output(
                current_pose_map, reference, 0.0,
                'parallel_front_end_hold:0.00/%.2fs'
                % self.config.direction_change_hold_s)

        if self.state == ParallelControlState.FRONT_HOLD:
            reference = self.forward_path[-1]
            elapsed = float(now_s) - self.hold_started_at_s
            if elapsed < self.config.direction_change_hold_s:
                return self._output(
                    current_pose_map, reference, 0.0,
                    'parallel_front_end_hold:%.2f/%.2fs' % (
                        max(0.0, elapsed),
                        self.config.direction_change_hold_s,
                    ))
            self.state = ParallelControlState.REVERSE
            self.progress = closest_path_index(
                self.reverse_path, current_pose_map)

        if self.state == ParallelControlState.REVERSE:
            reference = self._preview(
                self.reverse_path, self.reverse_lengths, current_pose_map)
            if not self._at_end(reference, self.reverse_path):
                return self._output(
                    current_pose_map, reference,
                    -self.config.reverse_speed_mps, 'parallel_reverse_park')
            self.state = ParallelControlState.REAR_HOLD
            self.hold_started_at_s = float(now_s)
            return self._output(
                current_pose_map, reference, 0.0,
                'parallel_reverse_end_hold:0.00/%.2fs'
                % self.config.direction_change_hold_s)

        if self.state == ParallelControlState.REAR_HOLD:
            reference = self.reverse_path[-1]
            elapsed = float(now_s) - self.hold_started_at_s
            if elapsed < self.config.direction_change_hold_s:
                return self._output(
                    current_pose_map, reference, 0.0,
                    'parallel_reverse_end_hold:%.2f/%.2fs' % (
                        max(0.0, elapsed),
                        self.config.direction_change_hold_s,
                    ))
            self.state = ParallelControlState.FORWARD_RETURN
            self.progress = closest_path_index(
                self.forward_path, current_pose_map)

        if self.state == ParallelControlState.FORWARD_RETURN:
            reference = self._preview(
                self.forward_path, self.forward_lengths, current_pose_map)
            if not self._at_end(reference, self.forward_path):
                return self._output(
                    current_pose_map, reference,
                    self.config.forward_speed_mps, 'parallel_forward_return')
            self.state = ParallelControlState.FINAL_HOLD
            self.hold_started_at_s = float(now_s)
            return self._output(
                current_pose_map, reference, 0.0,
                'parallel_final_end_hold:0.00/%.2fs'
                % self.config.direction_change_hold_s)

        if self.state == ParallelControlState.FINAL_HOLD:
            reference = self.forward_path[-1]
            elapsed = float(now_s) - self.hold_started_at_s
            if elapsed < self.config.direction_change_hold_s:
                return self._output(
                    current_pose_map, reference, 0.0,
                    'parallel_final_end_hold:%.2f/%.2fs' % (
                        max(0.0, elapsed),
                        self.config.direction_change_hold_s,
                    ))
            self.state = ParallelControlState.STOPPED
            self.stop_reason = 'parallel_parking_complete'
            return self._output(
                current_pose_map, reference, 0.0, self.stop_reason)

        return self._output(
            current_pose_map, self._last_reference_map, 0.0,
            self.stop_reason or 'parallel_stopped')
