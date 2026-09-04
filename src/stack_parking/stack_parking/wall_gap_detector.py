"""Fixed-reference wall-gap parking-space search.

The first sufficiently long wall seen beside the vehicle is fitted once in
the map frame. Its anchor and direction then remain immutable until reset.
Later map points count as the same wall only when they fall inside a narrow
offset band around the infinitely extended reference line. Consequently a
change in vehicle yaw changes only the vehicle's progress along the wall; it
cannot rotate or translate the detected wall.

No ROS imports (matches geometry.py / space_detector.py).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np

from .geometry import Pose2, points_in_frame

SIDE_LEFT = 'left'
SIDE_RIGHT = 'right'


@dataclass
class WallGapConfig:
    # Initial search band, measured from the vehicle pose captured by
    # set_seed(). It is used only to acquire the first reference wall.
    near_m: float = 0.3
    far_m: float = 1.6
    # Half-width of the offset band around the locked reference line. After
    # acquisition this replaces the vehicle-relative near/far band.
    wall_line_offset_m: float = 0.12
    initial_wall_min_points: int = 6
    initial_wall_min_length_m: float = 0.5
    initial_wall_max_angle_deg: float = 45.0
    join_gap_m: float = 0.3
    min_segment_points: int = 3
    min_gap_m: float = 1.2
    square_size_m: float = 1.0
    candidate_max_ahead_m: float = 5.0
    candidate_max_behind_m: float = 1.0
    reach_tolerance_m: float = 0.3
    dedup_tolerance_m: float = 0.5
    search_sides: tuple[str, ...] = (SIDE_LEFT, SIDE_RIGHT)


@dataclass(frozen=True)
class ReferenceWall:
    """A map-fixed wall line.

    ``tangent`` is aligned with the vehicle's forward direction at detector
    startup. ``normal`` points from the startup lane toward the searched
    side, which is also the direction in which the parking square extends.
    The anchor is the perpendicular foot from the startup vehicle position,
    so along-wall coordinate s=0 corresponds to the starting location.
    """

    side: str
    anchor_x: float
    anchor_y: float
    tangent_x: float
    tangent_y: float
    normal_x: float
    normal_y: float
    distance_from_seed_m: float

    @property
    def yaw(self) -> float:
        return math.atan2(self.tangent_y, self.tangent_x)

    def project(self, map_points: np.ndarray) -> np.ndarray:
        points = np.asarray(map_points, dtype=np.float64)
        if points.size == 0:
            return np.empty((0, 2), dtype=np.float64)
        delta = points[:, :2] - np.array([self.anchor_x, self.anchor_y])
        return np.column_stack((
            delta[:, 0] * self.tangent_x + delta[:, 1] * self.tangent_y,
            delta[:, 0] * self.normal_x + delta[:, 1] * self.normal_y,
        ))

    def to_map(self, wall_points: np.ndarray) -> np.ndarray:
        points = np.asarray(wall_points, dtype=np.float64)
        if points.size == 0:
            return np.empty((0, 2), dtype=np.float64)
        return np.column_stack((
            self.anchor_x
            + points[:, 0] * self.tangent_x
            + points[:, 1] * self.normal_x,
            self.anchor_y
            + points[:, 0] * self.tangent_y
            + points[:, 1] * self.normal_y,
        ))


@dataclass(frozen=True)
class WallSegment:
    start_s: float
    end_s: float
    count: int


@dataclass(frozen=True)
class GapCandidate:
    side: str
    start_s: float
    end_s: float
    center_s: float

    @property
    def width_m(self) -> float:
        return self.end_s - self.start_s


@dataclass
class TrackedCandidate:
    side: str
    map_x: float
    map_y: float
    width_m: float
    # Kept for compatibility with the existing logging/path code. Unlike the
    # old live measurement, this is the fixed startup-to-wall distance.
    near_distance: float
    wall_tangent_x: float
    wall_tangent_y: float
    wall_normal_x: float
    wall_normal_y: float
    tested: bool = False
    clear: Optional[bool] = None


def _angle_difference_rad(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


def _fit_reference_wall(
    map_points: np.ndarray,
    seed_pose: Pose2,
    side: str,
    cfg: WallGapConfig,
) -> Optional[ReferenceWall]:
    """Fit the first side wall with a deterministic, bounded RANSAC pass."""
    local = points_in_frame(map_points, seed_pose)
    side_sign = 1.0 if side == SIDE_LEFT else -1.0
    side_distance = side_sign * local[:, 1]
    band = local[
        (side_distance >= cfg.near_m) & (side_distance <= cfg.far_m)]
    if len(band) < cfg.initial_wall_min_points:
        return None

    # A bounded subset keeps first acquisition cheap even after a large map
    # has accumulated. Even spacing preserves the full longitudinal extent.
    order = np.argsort(band[:, 0])
    ordered = band[order]
    sample_count = min(80, len(ordered))
    sample_indices = np.linspace(
        0, len(ordered) - 1, sample_count, dtype=np.int64)
    sample = ordered[sample_indices]
    tolerance = max(0.02, cfg.wall_line_offset_m)
    max_angle = math.radians(max(0.0, cfg.initial_wall_max_angle_deg))

    best_mask: Optional[np.ndarray] = None
    best_key: Optional[tuple[float, ...]] = None
    seed_forward = np.array([1.0, 0.0])
    for i in range(len(sample) - 1):
        delta = sample[i + 1:] - sample[i]
        lengths = np.linalg.norm(delta, axis=1)
        for rel_j in np.flatnonzero(lengths >= cfg.initial_wall_min_length_m):
            vector = delta[rel_j] / lengths[rel_j]
            if float(np.dot(vector, seed_forward)) < 0.0:
                vector = -vector
            angle = math.atan2(float(vector[1]), float(vector[0]))
            if _angle_difference_rad(angle, 0.0) > max_angle:
                continue
            normal_left = np.array([-vector[1], vector[0]])
            normal = side_sign * normal_left
            seed_distance = float(np.dot(sample[i], normal))
            if not (cfg.near_m - tolerance <= seed_distance
                    <= cfg.far_m + tolerance):
                continue
            distances = np.abs((band - sample[i]) @ normal)
            mask = distances <= tolerance
            count = int(np.count_nonzero(mask))
            if count < cfg.initial_wall_min_points:
                continue
            along = band[mask] @ vector
            span = float(np.ptp(along))
            if span < cfg.initial_wall_min_length_m:
                continue
            # Prefer support and span. Distance breaks close ties in favour
            # of the first/nearest wall beside the starting position.
            key = (float(count), span, -seed_distance)
            if best_key is None or key > best_key:
                best_key = key
                best_mask = mask

    if best_mask is None:
        return None

    # Orthogonal least-squares refinement over the selected inliers.
    inliers = band[best_mask]
    centroid = np.mean(inliers, axis=0)
    covariance = (inliers - centroid).T @ (inliers - centroid)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    tangent_local = eigenvectors[:, int(np.argmax(eigenvalues))]
    if float(tangent_local[0]) < 0.0:
        tangent_local = -tangent_local
    angle_local = math.atan2(float(tangent_local[1]), float(tangent_local[0]))
    if _angle_difference_rad(angle_local, 0.0) > max_angle:
        return None

    normal_local = side_sign * np.array(
        [-tangent_local[1], tangent_local[0]])
    distance = float(np.dot(centroid, normal_local))
    if not (cfg.near_m - tolerance <= distance <= cfg.far_m + tolerance):
        return None

    c = math.cos(seed_pose.yaw)
    s = math.sin(seed_pose.yaw)
    tangent_map = np.array([
        c * tangent_local[0] - s * tangent_local[1],
        s * tangent_local[0] + c * tangent_local[1],
    ])
    normal_map = np.array([
        c * normal_local[0] - s * normal_local[1],
        s * normal_local[0] + c * normal_local[1],
    ])
    anchor = np.array([seed_pose.x, seed_pose.y]) + distance * normal_map
    return ReferenceWall(
        side=side,
        anchor_x=float(anchor[0]),
        anchor_y=float(anchor[1]),
        tangent_x=float(tangent_map[0]),
        tangent_y=float(tangent_map[1]),
        normal_x=float(normal_map[0]),
        normal_y=float(normal_map[1]),
        distance_from_seed_m=distance,
    )


def wall_segments(
    points_wall: np.ndarray, cfg: WallGapConfig,
) -> list[WallSegment]:
    """Group points inside the fixed wall-line offset band by along-wall s."""
    if len(points_wall) == 0:
        return []
    band_s = points_wall[
        np.abs(points_wall[:, 1]) <= cfg.wall_line_offset_m, 0]
    if len(band_s) == 0:
        return []
    xs = np.sort(band_s)
    groups: list[list[float]] = [[float(xs[0])]]
    for value in xs[1:]:
        x = float(value)
        if x - groups[-1][-1] <= cfg.join_gap_m:
            groups[-1].append(x)
        else:
            groups.append([x])
    return [
        WallSegment(group[0], group[-1], len(group))
        for group in groups if len(group) >= cfg.min_segment_points
    ]


def find_gap_candidates(
    segments: list[WallSegment], side: str, vehicle_s: float,
    cfg: WallGapConfig,
) -> list[GapCandidate]:
    """Find openings between adjacent fixed-reference wall segments."""
    candidates = []
    for first, second in zip(segments, segments[1:]):
        gap = second.start_s - first.end_s
        if gap < cfg.min_gap_m:
            continue
        center = 0.5 * (first.end_s + second.start_s)
        relative_center = center - vehicle_s
        if (relative_center > cfg.candidate_max_ahead_m
                or relative_center < -cfg.candidate_max_behind_m):
            continue
        candidates.append(GapCandidate(
            side, first.end_s, second.start_s, center))
    return candidates


def square_is_clear(
    points_wall: np.ndarray, center_s: float, cfg: WallGapConfig,
) -> bool:
    """Check the square extending inward from the fixed reference wall."""
    half = 0.5 * cfg.square_size_m
    inside = (
        (points_wall[:, 0] >= center_s - half)
        & (points_wall[:, 0] <= center_s + half)
        & (points_wall[:, 1] >= 0.0)
        & (points_wall[:, 1] <= cfg.square_size_m)
    )
    return not bool(np.any(inside))


def candidate_square_corners(
    candidate: TrackedCandidate, cfg: WallGapConfig,
) -> np.ndarray:
    """Return the candidate square in map coordinates, independent of yaw."""
    half = 0.5 * cfg.square_size_m
    center = np.array([candidate.map_x, candidate.map_y])
    tangent = np.array([
        candidate.wall_tangent_x, candidate.wall_tangent_y])
    normal = np.array([
        candidate.wall_normal_x, candidate.wall_normal_y])
    return np.asarray([
        center - half * tangent,
        center + half * tangent,
        center + half * tangent + cfg.square_size_m * normal,
        center - half * tangent + cfg.square_size_m * normal,
        center - half * tangent,
    ])


class WallGapDetector:
    """Track wall gaps against map-fixed reference lines."""

    def __init__(self, config: Optional[WallGapConfig] = None):
        self.config = config or WallGapConfig()
        self.tracked: list[TrackedCandidate] = []
        self.last_segments: dict[str, list[WallSegment]] = {
            SIDE_LEFT: [], SIDE_RIGHT: []}
        self.last_live_candidates: list[GapCandidate] = []
        self.seed_pose: Optional[Pose2] = None
        self.seed_side: Optional[str] = None
        self.reference_walls: dict[str, ReferenceWall] = {}

    def reset(self) -> None:
        self.tracked.clear()
        self.last_segments = {SIDE_LEFT: [], SIDE_RIGHT: []}
        self.last_live_candidates = []
        self.seed_pose = None
        self.seed_side = None
        self.reference_walls.clear()

    def set_seed(self, vehicle_pose: Pose2, side: str = SIDE_LEFT) -> None:
        # A new seed defines a new run. The reference is never silently moved
        # within a run; callers must reset/set_seed explicitly to reacquire.
        self.seed_pose = vehicle_pose
        self.seed_side = side

    def segment_map_points(
        self, side: str, segment: WallSegment,
    ) -> Optional[np.ndarray]:
        wall = self.reference_walls.get(side)
        if wall is None:
            return None
        return wall.to_map(np.array([
            [segment.start_s, 0.0], [segment.end_s, 0.0]]))

    def update(
        self, map_points: np.ndarray, vehicle_pose: Pose2,
    ) -> Optional[TrackedCandidate]:
        """Run one tick and return a newly confirmed clear candidate."""
        cfg = self.config
        if self.seed_pose is None:
            first_side = cfg.search_sides[0] if cfg.search_sides else SIDE_LEFT
            self.set_seed(vehicle_pose, first_side)

        live_candidates: list[GapCandidate] = []
        for side in (SIDE_LEFT, SIDE_RIGHT):
            if side not in cfg.search_sides:
                self.last_segments[side] = []
                continue

            wall = self.reference_walls.get(side)
            if wall is None:
                wall = _fit_reference_wall(
                    map_points, self.seed_pose, side, cfg)
                if wall is None:
                    self.last_segments[side] = []
                    continue
                self.reference_walls[side] = wall

            points_wall = wall.project(map_points)
            vehicle_wall = wall.project(np.array([
                [vehicle_pose.x, vehicle_pose.y]]))[0]
            segments = wall_segments(points_wall, cfg)
            self.last_segments[side] = segments
            candidates = find_gap_candidates(
                segments, side, float(vehicle_wall[0]), cfg)
            live_candidates.extend(candidates)

            for candidate in candidates:
                map_point = wall.to_map(np.array([
                    [candidate.center_s, 0.0]]))[0]
                matched = next(
                    (tracked for tracked in self.tracked
                     if tracked.side == side and not tracked.tested
                     and math.hypot(
                         tracked.map_x - map_point[0],
                         tracked.map_y - map_point[1],
                     ) <= cfg.dedup_tolerance_m),
                    None,
                )
                if matched is not None:
                    # Refresh only the gap's along-wall position/width. The
                    # locked wall anchor, slope and normal never change.
                    matched.map_x = float(map_point[0])
                    matched.map_y = float(map_point[1])
                    matched.width_m = candidate.width_m
                    continue
                if any(
                    tracked.side == side and tracked.tested
                    and math.hypot(
                        tracked.map_x - map_point[0],
                        tracked.map_y - map_point[1],
                    ) <= cfg.dedup_tolerance_m
                    for tracked in self.tracked
                ):
                    continue
                self.tracked.append(TrackedCandidate(
                    side=side,
                    map_x=float(map_point[0]),
                    map_y=float(map_point[1]),
                    width_m=candidate.width_m,
                    near_distance=wall.distance_from_seed_m,
                    wall_tangent_x=wall.tangent_x,
                    wall_tangent_y=wall.tangent_y,
                    wall_normal_x=wall.normal_x,
                    wall_normal_y=wall.normal_y,
                ))
        self.last_live_candidates = live_candidates

        newly_cleared: Optional[TrackedCandidate] = None
        for tracked in self.tracked:
            if tracked.tested:
                continue
            wall = self.reference_walls.get(tracked.side)
            if wall is None:
                continue
            candidate_wall = wall.project(np.array([
                [tracked.map_x, tracked.map_y]]))[0]
            vehicle_wall = wall.project(np.array([
                [vehicle_pose.x, vehicle_pose.y]]))[0]
            if abs(candidate_wall[0] - vehicle_wall[0]) > cfg.reach_tolerance_m:
                continue
            tracked.tested = True
            tracked.clear = square_is_clear(
                wall.project(map_points), float(candidate_wall[0]), cfg)
            if tracked.clear:
                newly_cleared = tracked
        return newly_cleared
