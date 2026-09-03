"""ROS-independent geometry and control for the parallel-parking test.

The valid point P0 is the midpoint between the two wall faces and the midpoint
of the wall-side edge of a 1.5m x 0.7m validation rectangle.  The S-curve
origin is shifted 0.5m from P0 along the wall tangent oriented toward the
vehicle's travel direction; the vehicle yaw itself is not used as its slope. A
45-degree arc of radius 1.12m is constructed there, then rotated 180 degrees
about the shifted origin.  A 1.5-metre wall-parallel line extends from each end
of the S.  The five-motion controller additionally isolates one arc into a
line-arc-line path with two-metre straight sections and obtains the other
line-arc-line by rotating the complete first shape 180 degrees about the shared
arc origin.  The final reverse and forward phases reuse the original,
unmodified S reference path.

The front (entry) arc used for that isolated line-arc-line -- the one the
vehicle actually backs into the bay on -- can take its own radius via
``entry_radius_m``, independent of the rear arc's ``turn_radius_m`` used only
by the full-S passes (user directive, 2026-09-04). The isolated line-arc-line
also has two independently tunable straight lengths: ``entry_straight_m`` for
backing in (kept at 2m) and ``opposite_straight_m`` for the forward nudge
that follows (shortened to 1m).
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
    # front = the arc used to enter the bay (SINGLE_ARC_REVERSE); rear = the
    # arc used only in the full S (INITIAL_FORWARD/REFERENCE_REVERSE/
    # REFERENCE_FORWARD). Independent so the entry radius can be tuned
    # without touching the inner one (user directive, 2026-09-04).
    front_radius_m: float
    rear_radius_m: float
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
    # Straight length of the entry line-arc-line (SINGLE_ARC_REVERSE, backing
    # into the bay) versus the following forward nudge (OPPOSITE_ARC_FORWARD)
    # -- independently tunable (user directive, 2026-09-04). Both default to
    # the original symmetric 2m; parallel_parking_node.py's own parameter
    # default shortens the nudge to 1m -- kept symmetric here so a bare
    # ParallelParkingConfig() (e.g. in tests) keeps its old behaviour.
    entry_straight_m: float = 2.0
    opposite_straight_m: float = 2.0


class ParallelControlState(str, enum.Enum):
    IDLE = 'idle'
    INITIAL_FORWARD = 'parallel_initial_reference_forward'
    INITIAL_FORWARD_HOLD = 'parallel_initial_forward_hold'
    SINGLE_ARC_REVERSE = 'parallel_single_arc_reverse'
    SINGLE_ARC_REVERSE_HOLD = 'parallel_single_arc_reverse_hold'
    OPPOSITE_ARC_FORWARD = 'parallel_opposite_arc_forward'
    OPPOSITE_ARC_FORWARD_HOLD = 'parallel_opposite_arc_forward_hold'
    REFERENCE_REVERSE = 'parallel_reference_reverse'
    REFERENCE_REVERSE_HOLD = 'parallel_reference_reverse_hold'
    REFERENCE_FORWARD = 'parallel_reference_forward'
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
    end_straight_m: float = 1.5,
    arc_angle_deg: float = 45.0,
    arc_start_offset_m: float = 0.5,
    arc_points: int = 24,
    entry_radius_m: Optional[float] = None,
) -> Optional[ParallelReferencePath]:
    """Construct the point-symmetric S path requested for parallel parking.

    ``entry_radius_m`` overrides the radius of only the front/entry arc
    (the one SINGLE_ARC_REVERSE backs the vehicle in on) -- e.g. doubling it
    for a gentler entry sweep while ``turn_radius_m`` keeps sizing the rear
    arc used by the full-S passes. Defaults to ``turn_radius_m`` (the
    original single-radius S) when not given.
    """
    radius = float(turn_radius_m)
    front_radius = float(entry_radius_m) if entry_radius_m is not None else radius
    line_length = float(end_straight_m)
    arc_start_offset = float(arc_start_offset_m)
    arc_angle = math.radians(float(arc_angle_deg))
    if (radius <= 0.0 or front_radius <= 0.0 or line_length <= 0.0
            or arc_start_offset < 0.0
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

    # For 45 degrees, arc_origin->centre is front_radius/sqrt(2) along both
    # the forward wall tangent and inward normal.  The formulation below
    # also keeps the angle parameter explicit for geometry validation.
    front_direction = math.cos(arc_angle) * tangent + math.sin(arc_angle) * inward
    front_center = arc_origin + front_radius * front_direction
    front_tangent = front_center - front_radius * inward
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

    # The rear arc is the same 180-degree-about-arc_origin construction, but
    # built from its own radius (independent of front_radius) rather than by
    # reflecting front_arc's points -- a point reflection only stays on a
    # radius-rear circle when the two radii match. The sweep angle itself
    # (a pure angle between unit directions) doesn't depend on either
    # radius, so the same ``angles`` apply. Reverse samples so the complete
    # path is rear->front.
    rear_center = arc_origin - radius * front_direction
    rear_tangent = rear_center + radius * inward
    rear_radius_at_origin = arc_origin - rear_center
    rear_arc_from_origin = np.asarray([
        rear_center + _rotate(rear_radius_at_origin, float(angle))
        for angle in angles
    ])
    rear_arc = rear_arc_from_origin[::-1].copy()
    front_end = front_tangent + line_length * tangent
    rear_end = rear_tangent - line_length * tangent

    return ParallelReferencePath(
        side=candidate.side,
        front_radius_m=front_radius,
        rear_radius_m=radius,
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
        reference.rear_radius_m,
        1,
    ))
    _append_without_duplicate(forward, _arc_path_points(
        reference.front_arc_map,
        np.asarray(reference.front_center_map),
        reference.front_radius_m,
        1,
    ))
    _append_without_duplicate(forward, _sample_line(
        front_tangent, front_end, wall_yaw, 1, step_m))
    reverse = tuple(PathPoint(
        point.x, point.y, point.yaw, point.curvature, -1)
        for point in reversed(forward))
    return tuple(forward), reverse


def parallel_single_arc_paths(
    reference: ParallelReferencePath,
    step_m: float = 0.05,
    straight_m: float = 2.0,
) -> tuple[tuple[PathPoint, ...], tuple[PathPoint, ...]]:
    """Return the front line-arc-line path in both travel directions.

    Both straight sections retain the phase-specific two-metre length.  The
    reverse phase traverses outer-line -> arc -> inner-line; the following
    forward phase traverses the same corridor in the opposite direction.
    """
    arc_origin = np.asarray(reference.arc_origin_map, dtype=np.float64)
    front_tangent = np.asarray(
        reference.front_tangent_map, dtype=np.float64)
    front_end = np.asarray(reference.front_end_map, dtype=np.float64)
    line_length = float(straight_m)
    wall_direction = front_end - front_tangent
    wall_direction_norm = float(np.linalg.norm(wall_direction))
    if line_length <= 1.0e-9 or wall_direction_norm <= 1.0e-9:
        return (), ()
    wall_direction /= wall_direction_norm
    outer_end = front_tangent + line_length * wall_direction

    arc = _arc_path_points(
        reference.front_arc_map,
        np.asarray(reference.front_center_map),
        reference.front_radius_m,
        1,
    )
    if not arc:
        return (), ()
    inner_yaw = arc[0].yaw
    inner_direction = np.asarray([
        math.cos(inner_yaw), math.sin(inner_yaw)], dtype=np.float64)
    inner_start = arc_origin - line_length * inner_direction
    wall_yaw = math.atan2(wall_direction[1], wall_direction[0])

    forward: list[PathPoint] = []
    _append_without_duplicate(forward, _sample_line(
        inner_start, arc_origin, inner_yaw, 1, step_m))
    _append_without_duplicate(forward, arc)
    _append_without_duplicate(forward, _sample_line(
        front_tangent, outer_end, wall_yaw, 1, step_m))
    reverse = tuple(PathPoint(
        point.x, point.y, point.yaw, point.curvature, -1)
        for point in reversed(forward))
    return tuple(forward), reverse


def parallel_opposite_single_arc_path(
    reference: ParallelReferencePath,
    step_m: float = 0.05,
    straight_m: float = 2.0,
) -> tuple[PathPoint, ...]:
    """Rotate the first line-arc-line 180 degrees about the arc origin.

    Positions are point-reflected exactly.  Because this result is driven
    forward while the source path is driven in reverse, vehicle yaw is kept
    and curvature is negated.  ``straight_m`` is independent of the entry
    line-arc-line's own straight length (user directive, 2026-09-04: the
    entry stays 2m, this "nudge forward" leg can be shorter, e.g. 1m).
    """
    _, first_reverse = parallel_single_arc_paths(reference, step_m, straight_m)
    if not first_reverse:
        return ()
    origin_x, origin_y = reference.arc_origin_map
    return tuple(PathPoint(
        2.0 * origin_x - point.x,
        2.0 * origin_y - point.y,
        point.yaw,
        -point.curvature,
        1,
    ) for point in first_reverse)


class ParallelParkingController:
    def __init__(self, config: Optional[ParallelParkingConfig] = None):
        self.config = config or ParallelParkingConfig()
        self.state = ParallelControlState.IDLE
        self.reference_forward_path: tuple[PathPoint, ...] = ()
        self.reference_reverse_path: tuple[PathPoint, ...] = ()
        self.initial_reference_forward_path: tuple[PathPoint, ...] = ()
        self.single_arc_forward_path: tuple[PathPoint, ...] = ()
        self.single_arc_reverse_path: tuple[PathPoint, ...] = ()
        self.opposite_arc_forward_path: tuple[PathPoint, ...] = ()
        self.reference_forward_lengths: list[float] = []
        self.reference_reverse_lengths: list[float] = []
        self.initial_reference_forward_lengths: list[float] = []
        self.single_arc_forward_lengths: list[float] = []
        self.single_arc_reverse_lengths: list[float] = []
        self.opposite_arc_forward_lengths: list[float] = []
        # Compatibility aliases used by older analysis tools.
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
        single_forward, single_reverse = parallel_single_arc_paths(
            path, self.config.sample_step_m, self.config.entry_straight_m)
        if (not forward or not reverse
                or not single_forward or not single_reverse):
            return False
        opposite_forward = parallel_opposite_single_arc_path(
            path, self.config.sample_step_m, self.config.opposite_straight_m)
        if not opposite_forward:
            return False
        self.initial_reference_forward_path = forward
        self.reference_forward_path = forward
        self.reference_reverse_path = reverse
        self.single_arc_forward_path = single_forward
        self.single_arc_reverse_path = single_reverse
        self.opposite_arc_forward_path = opposite_forward
        self.initial_reference_forward_lengths = cumulative_lengths(forward)
        self.reference_forward_lengths = cumulative_lengths(forward)
        self.reference_reverse_lengths = cumulative_lengths(reverse)
        self.single_arc_forward_lengths = cumulative_lengths(single_forward)
        self.single_arc_reverse_lengths = cumulative_lengths(single_reverse)
        self.opposite_arc_forward_lengths = cumulative_lengths(
            opposite_forward)
        self.forward_path = self.reference_forward_path
        self.reverse_path = self.reference_reverse_path
        self.forward_lengths = self.reference_forward_lengths
        self.reverse_lengths = self.reference_reverse_lengths
        self.progress = closest_path_index(forward, current_pose_map)
        self.state = ParallelControlState.INITIAL_FORWARD
        self.hold_started_at_s = float(now_s)
        self.stop_reason = ''
        self._last_reference_map = self._preview(
            forward, self.initial_reference_forward_lengths, current_pose_map)
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

    def _activate(
        self,
        state: ParallelControlState,
        path: Sequence[PathPoint],
        current_pose_map: Pose2,
    ) -> None:
        self.state = state
        self.progress = closest_path_index(path, current_pose_map)

    def _holding(
        self,
        current_pose_map: Pose2,
        reference: PathPoint,
        now_s: float,
        status: str,
    ) -> Optional[WallGapControlOutput]:
        elapsed = float(now_s) - self.hold_started_at_s
        if elapsed + 1.0e-9 >= self.config.direction_change_hold_s:
            return None
        return self._output(
            current_pose_map, reference, 0.0,
            '%s:%.2f/%.2fs' % (
                status,
                max(0.0, elapsed),
                self.config.direction_change_hold_s,
            ),
        )

    def _start_hold(
        self,
        current_pose_map: Pose2,
        reference: PathPoint,
        now_s: float,
        state: ParallelControlState,
        status: str,
    ) -> WallGapControlOutput:
        self.state = state
        self.hold_started_at_s = float(now_s)
        return self._output(
            current_pose_map, reference, 0.0,
            '%s:0.00/%.2fs' % (
                status, self.config.direction_change_hold_s),
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

        if self.state == ParallelControlState.INITIAL_FORWARD:
            reference = self._preview(
                self.initial_reference_forward_path,
                self.initial_reference_forward_lengths,
                current_pose_map,
            )
            if not self._at_end(
                    reference, self.initial_reference_forward_path):
                return self._output(
                    current_pose_map, reference,
                    self.config.forward_speed_mps,
                    ParallelControlState.INITIAL_FORWARD.value,
                )
            return self._start_hold(
                current_pose_map, reference, now_s,
                ParallelControlState.INITIAL_FORWARD_HOLD,
                ParallelControlState.INITIAL_FORWARD_HOLD.value,
            )

        if self.state == ParallelControlState.INITIAL_FORWARD_HOLD:
            reference = self.initial_reference_forward_path[-1]
            output = self._holding(
                current_pose_map, reference, now_s,
                ParallelControlState.INITIAL_FORWARD_HOLD.value,
            )
            if output is not None:
                return output
            self._activate(
                ParallelControlState.SINGLE_ARC_REVERSE,
                self.single_arc_reverse_path,
                current_pose_map,
            )

        if self.state == ParallelControlState.SINGLE_ARC_REVERSE:
            reference = self._preview(
                self.single_arc_reverse_path,
                self.single_arc_reverse_lengths,
                current_pose_map,
            )
            if not self._at_end(reference, self.single_arc_reverse_path):
                return self._output(
                    current_pose_map, reference,
                    -self.config.reverse_speed_mps,
                    ParallelControlState.SINGLE_ARC_REVERSE.value,
                )
            return self._start_hold(
                current_pose_map, reference, now_s,
                ParallelControlState.SINGLE_ARC_REVERSE_HOLD,
                ParallelControlState.SINGLE_ARC_REVERSE_HOLD.value,
            )

        if self.state == ParallelControlState.SINGLE_ARC_REVERSE_HOLD:
            reference = self.single_arc_reverse_path[-1]
            output = self._holding(
                current_pose_map, reference, now_s,
                ParallelControlState.SINGLE_ARC_REVERSE_HOLD.value,
            )
            if output is not None:
                return output
            self._activate(
                ParallelControlState.OPPOSITE_ARC_FORWARD,
                self.opposite_arc_forward_path,
                current_pose_map,
            )

        if self.state == ParallelControlState.OPPOSITE_ARC_FORWARD:
            reference = self._preview(
                self.opposite_arc_forward_path,
                self.opposite_arc_forward_lengths,
                current_pose_map,
            )
            if not self._at_end(reference, self.opposite_arc_forward_path):
                return self._output(
                    current_pose_map, reference,
                    self.config.forward_speed_mps,
                    ParallelControlState.OPPOSITE_ARC_FORWARD.value,
                )
            return self._start_hold(
                current_pose_map, reference, now_s,
                ParallelControlState.OPPOSITE_ARC_FORWARD_HOLD,
                ParallelControlState.OPPOSITE_ARC_FORWARD_HOLD.value,
            )

        if self.state == ParallelControlState.OPPOSITE_ARC_FORWARD_HOLD:
            reference = self.opposite_arc_forward_path[-1]
            output = self._holding(
                current_pose_map, reference, now_s,
                ParallelControlState.OPPOSITE_ARC_FORWARD_HOLD.value,
            )
            if output is not None:
                return output
            self._activate(
                ParallelControlState.REFERENCE_REVERSE,
                self.reference_reverse_path,
                current_pose_map,
            )

        if self.state == ParallelControlState.REFERENCE_REVERSE:
            reference = self._preview(
                self.reference_reverse_path,
                self.reference_reverse_lengths,
                current_pose_map,
            )
            if not self._at_end(reference, self.reference_reverse_path):
                return self._output(
                    current_pose_map, reference,
                    -self.config.reverse_speed_mps,
                    ParallelControlState.REFERENCE_REVERSE.value,
                )
            return self._start_hold(
                current_pose_map, reference, now_s,
                ParallelControlState.REFERENCE_REVERSE_HOLD,
                ParallelControlState.REFERENCE_REVERSE_HOLD.value,
            )

        if self.state == ParallelControlState.REFERENCE_REVERSE_HOLD:
            reference = self.reference_reverse_path[-1]
            output = self._holding(
                current_pose_map, reference, now_s,
                ParallelControlState.REFERENCE_REVERSE_HOLD.value,
            )
            if output is not None:
                return output
            self._activate(
                ParallelControlState.REFERENCE_FORWARD,
                self.reference_forward_path,
                current_pose_map,
            )

        if self.state == ParallelControlState.REFERENCE_FORWARD:
            reference = self._preview(
                self.reference_forward_path,
                self.reference_forward_lengths,
                current_pose_map,
            )
            if not self._at_end(reference, self.reference_forward_path):
                return self._output(
                    current_pose_map, reference,
                    self.config.forward_speed_mps,
                    ParallelControlState.REFERENCE_FORWARD.value,
                )
            return self._start_hold(
                current_pose_map, reference, now_s,
                ParallelControlState.FINAL_HOLD,
                ParallelControlState.FINAL_HOLD.value,
            )

        if self.state == ParallelControlState.FINAL_HOLD:
            reference = self.reference_forward_path[-1]
            output = self._holding(
                current_pose_map, reference, now_s,
                ParallelControlState.FINAL_HOLD.value,
            )
            if output is not None:
                return output
            self.state = ParallelControlState.STOPPED
            self.stop_reason = 'parallel_parking_complete'
            return self._output(
                current_pose_map, reference, 0.0, self.stop_reason)

        return self._output(
            current_pose_map, self._last_reference_map, 0.0,
            self.stop_reason or 'parallel_stopped')
