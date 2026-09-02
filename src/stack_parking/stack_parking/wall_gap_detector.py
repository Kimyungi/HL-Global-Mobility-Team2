"""Wall-gap + inscribed-square parking space search (user spec, 2026-09-02).

No ROS imports (matches geometry.py / space_detector.py). Deliberately
independent of space_detector.py's clustering — a fresh, simpler algorithm:

  1. Classify current map points into left-wall / right-wall bands relative
     to the vehicle's current pose (a fixed lateral distance range).
  2. As the vehicle drives, group each side's wall points along the driving
     direction into segments; a gap between two adjacent segments >=
     ``min_gap_m`` (~1.2m) is a *candidate* — an opening in the wall.
  3. Once the vehicle is alongside a candidate's gap center, test whether a
     ``square_size_m`` x ``square_size_m`` square placed just inside the
     wall line there is free of mapped points. Clear -> usable space.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np

from .geometry import Pose2, points_in_frame, transform_points

SIDE_LEFT = 'left'
SIDE_RIGHT = 'right'


@dataclass
class WallGapConfig:
    near_m: float = 0.3
    far_m: float = 1.6
    join_gap_m: float = 0.3
    min_segment_points: int = 3
    min_gap_m: float = 1.2
    square_size_m: float = 1.0
    candidate_max_ahead_m: float = 5.0
    candidate_max_behind_m: float = 1.0
    reach_tolerance_m: float = 0.3
    dedup_tolerance_m: float = 0.5
    # Which side(s) to search. (SIDE_LEFT,) only, per user directive
    # 2026-09-02, is the common case — restricting search avoids spurious
    # candidates on a side that was never meant to be searched.
    search_sides: tuple[str, ...] = (SIDE_LEFT, SIDE_RIGHT)


@dataclass(frozen=True)
class WallSegment:
    start_x: float
    end_x: float
    count: int
    # Actual nearest point's lateral distance within this segment (user
    # directive, 2026-09-02) — the wall is drawn/placed at wherever the
    # real nearest point cluster is, not at the configured near_m band edge
    # (which is only a search cutoff, not the wall's real position).
    near_distance: float


@dataclass(frozen=True)
class GapCandidate:
    side: str
    start_x: float
    end_x: float
    center_x: float
    near_distance: float

    @property
    def width_m(self) -> float:
        return self.end_x - self.start_x


@dataclass
class TrackedCandidate:
    side: str
    map_x: float
    map_y: float
    width_m: float
    near_distance: float
    tested: bool = False
    clear: Optional[bool] = None


def wall_segments(
    points_vehicle: np.ndarray, side_sign: float, cfg: WallGapConfig,
) -> list[WallSegment]:
    """Group one side's band points into wall segments, sorted by x."""
    if len(points_vehicle) == 0:
        return []
    side_distance = side_sign * points_vehicle[:, 1]
    band_mask = (side_distance >= cfg.near_m) & (side_distance <= cfg.far_m)
    band_x = points_vehicle[band_mask, 0]
    band_dist = side_distance[band_mask]
    if len(band_x) == 0:
        return []
    order = np.argsort(band_x)
    xs = band_x[order]
    dists = band_dist[order]
    groups_x: list[list[float]] = [[float(xs[0])]]
    groups_d: list[list[float]] = [[float(dists[0])]]
    for x, d in zip(xs[1:], dists[1:]):
        if float(x) - groups_x[-1][-1] <= cfg.join_gap_m:
            groups_x[-1].append(float(x))
            groups_d[-1].append(float(d))
        else:
            groups_x.append([float(x)])
            groups_d.append([float(d)])
    return [
        WallSegment(gx[0], gx[-1], len(gx), min(gd))
        for gx, gd in zip(groups_x, groups_d) if len(gx) >= cfg.min_segment_points
    ]


def find_gap_candidates(
    segments: list[WallSegment], side: str, cfg: WallGapConfig,
) -> list[GapCandidate]:
    """Openings >= min_gap_m between adjacent wall segments."""
    candidates = []
    for a, b in zip(segments, segments[1:]):
        gap = b.start_x - a.end_x
        if gap < cfg.min_gap_m:
            continue
        center = 0.5 * (a.end_x + b.start_x)
        if center > cfg.candidate_max_ahead_m or center < -cfg.candidate_max_behind_m:
            continue
        near_distance = 0.5 * (a.near_distance + b.near_distance)
        candidates.append(GapCandidate(side, a.end_x, b.start_x, center, near_distance))
    return candidates


def square_is_clear(
    points_vehicle: np.ndarray, side_sign: float, center_x: float,
    near_distance: float, cfg: WallGapConfig,
) -> bool:
    """Is a square_size_m square, placed just inside the wall line at
    center_x, free of mapped points? The wall line is ``near_distance``
    (the real nearest-point-cluster distance for this candidate), not the
    configured near_m search-band edge."""
    half = 0.5 * cfg.square_size_m
    side_distance = side_sign * points_vehicle[:, 1]
    inside = (
        (points_vehicle[:, 0] >= center_x - half)
        & (points_vehicle[:, 0] <= center_x + half)
        & (side_distance >= near_distance)
        & (side_distance <= near_distance + cfg.square_size_m)
    )
    return not bool(np.any(inside))


class WallGapDetector:
    """Stateful driver: tracks candidates across ticks in map frame."""

    def __init__(self, config: Optional[WallGapConfig] = None):
        self.config = config or WallGapConfig()
        self.tracked: list[TrackedCandidate] = []
        self.last_segments: dict[str, list[WallSegment]] = {
            SIDE_LEFT: [], SIDE_RIGHT: []}
        self.last_live_candidates: list[GapCandidate] = []
        # Seed wall (user directive, 2026-09-02): a virtual wall segment
        # planted at the vehicle's pose when the detector starts/resets, so
        # the very first real segment found ahead has something to pair
        # against for a gap — without it, "gap between two wall segments"
        # has no meaning until a *second* real segment shows up.
        self.seed_pose: Optional[Pose2] = None
        self.seed_side: Optional[str] = None

    def reset(self) -> None:
        self.tracked.clear()
        self.last_segments = {SIDE_LEFT: [], SIDE_RIGHT: []}
        self.last_live_candidates = []
        self.seed_pose = None
        self.seed_side = None

    def set_seed(self, vehicle_pose: Pose2, side: str = SIDE_LEFT) -> None:
        self.seed_pose = vehicle_pose
        self.seed_side = side

    def update(
        self, map_points: np.ndarray, vehicle_pose: Pose2,
    ) -> Optional[TrackedCandidate]:
        """Run one tick. Returns the candidate that just got tested clear,
        if any (i.e. a usable space was just confirmed this tick)."""
        cfg = self.config
        points_vehicle = points_in_frame(map_points, vehicle_pose)

        live_candidates: list[GapCandidate] = []
        for side in (SIDE_LEFT, SIDE_RIGHT):
            if side not in cfg.search_sides:
                self.last_segments[side] = []
                continue
            sign = 1.0 if side == SIDE_LEFT else -1.0
            segments = wall_segments(points_vehicle, sign, cfg)
            if side == self.seed_side and self.seed_pose is not None:
                seed_local = points_in_frame(
                    np.array([[self.seed_pose.x, self.seed_pose.y]]), vehicle_pose)[0]
                seed_x = float(seed_local[0])
                seed_seg = WallSegment(
                    seed_x, seed_x + 0.01, cfg.min_segment_points, cfg.near_m)
                segments = sorted(segments + [seed_seg], key=lambda s: s.start_x)
            self.last_segments[side] = segments
            candidates = find_gap_candidates(segments, side, cfg)
            live_candidates.extend(candidates)
            for cand in candidates:
                # Valid point pinned to the wall's own near_distance (not the
                # square's mid-depth) so it plots exactly ON the wall line —
                # "_ . _" (2026-09-02 user spec): the dot sits on the same
                # line as the wall segments either side of the gap.
                map_pt = transform_points(
                    np.array([[cand.center_x, sign * cand.near_distance]]),
                    vehicle_pose)[0]
                matched = next(
                    (t for t in self.tracked
                     if t.side == side and not t.tested
                     and math.hypot(t.map_x - map_pt[0], t.map_y - map_pt[1])
                     <= cfg.dedup_tolerance_m),
                    None,
                )
                if matched is not None:
                    # Live re-measurement supersedes the old one — refresh
                    # rather than freeze at first detection, so the valid
                    # point keeps tracking the wall's real current surface as
                    # mapping accumulates more/better points.
                    matched.map_x = float(map_pt[0])
                    matched.map_y = float(map_pt[1])
                    matched.width_m = cand.width_m
                    matched.near_distance = cand.near_distance
                    continue
                if any(
                    t.side == side and t.tested
                    and math.hypot(t.map_x - map_pt[0], t.map_y - map_pt[1])
                    <= cfg.dedup_tolerance_m
                    for t in self.tracked
                ):
                    continue
                self.tracked.append(TrackedCandidate(
                    side=side, map_x=float(map_pt[0]), map_y=float(map_pt[1]),
                    width_m=cand.width_m, near_distance=cand.near_distance))
        self.last_live_candidates = live_candidates

        newly_cleared: Optional[TrackedCandidate] = None
        for tracked in self.tracked:
            if tracked.tested:
                continue
            local = points_in_frame(
                np.array([[tracked.map_x, tracked.map_y]]), vehicle_pose)[0]
            if abs(local[0]) > cfg.reach_tolerance_m:
                continue
            sign = 1.0 if tracked.side == SIDE_LEFT else -1.0
            tracked.tested = True
            tracked.clear = square_is_clear(
                points_vehicle, sign, local[0], tracked.near_distance, cfg)
            if tracked.clear:
                newly_cleared = tracked
        return newly_cleared
