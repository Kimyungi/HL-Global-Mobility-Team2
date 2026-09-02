"""Straight -> minimum-radius arc -> straight entry path into a parking gap.

No ROS imports (matches geometry.py / space_detector.py), independent of
path_planner.py's MinimumRadiusParkingPlanner — this is a separate, simpler
construction the user asked to see plotted before deciding whether to wire
it in anywhere.

Construction (all in the vehicle's current frame, +x forward, +y left):
  1. mouth center M = midpoint of the two wall segments' facing (inner) ends.
  2. goal B = M pushed further into the bay by ``depth``, i.e. straight past
     the wall segments along the same lateral direction.
  3. The vehicle reaches B heading straight into the bay (perpendicular to
     its start heading) via a single ``min_turn_radius`` arc, entered and
     left tangentially so both transitions are straight lines:
       - straight 1: vehicle start -> arc start A (A is on the y=0 lane line,
         directly `min_turn_radius` short of B in x, since that's where a
         quarter-circle of that radius must begin to end up facing +y/-y
         exactly at B's x)
       - arc: A -> E, centered at C = A + (0, side*min_turn_radius), a 90°
         sweep toward whichever side B is on
       - straight 2: E -> B (E already faces into the bay, so this leg is
         purely lateral)
  This only closes exactly when B lies far enough ahead/aside for both
  straight legs to be forward-going (non-negative) — see ``ArcEntryPath``'s
  ``straight1_len_m`` / ``straight2_len_m``, which go negative otherwise
  (the construction is still returned so the caller/plot can show why).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from .geometry import Pose2, transform_points


@dataclass(frozen=True)
class ArcEntryPath:
    mouth_center: tuple[float, float]
    goal: tuple[float, float]
    side: str  # 'left' or 'right'
    arc_start: tuple[float, float]
    arc_center: tuple[float, float]
    arc_end: tuple[float, float]
    radius_m: float
    straight1_len_m: float
    straight2_len_m: float

    def straight1_points(self, vehicle_pose: Pose2) -> np.ndarray:
        local = np.array([[0.0, 0.0], list(self.arc_start)])
        return transform_points(local, vehicle_pose)

    def arc_points(self, vehicle_pose: Pose2, n: int = 24) -> np.ndarray:
        side_sign = 1.0 if self.side == 'left' else -1.0
        cx, cy = self.arc_center
        ax, ay = self.arc_start
        alpha0 = math.atan2(ay - cy, ax - cx)
        sweep = side_sign * (math.pi / 2.0)
        angles = alpha0 + np.linspace(0.0, sweep, n)
        local = np.column_stack((
            cx + self.radius_m * np.cos(angles),
            cy + self.radius_m * np.sin(angles),
        ))
        return transform_points(local, vehicle_pose)

    def straight2_points(self, vehicle_pose: Pose2) -> np.ndarray:
        local = np.array([list(self.arc_end), list(self.goal)])
        return transform_points(local, vehicle_pose)


def build_entry_path(
    wall_segment_a: Sequence[Sequence[float]],
    wall_segment_b: Sequence[Sequence[float]],
    depth_m: float,
    min_turn_radius_m: float,
) -> ArcEntryPath:
    """Build the straight-arc-straight path, all in the vehicle's frame.

    ``wall_segment_a``/``wall_segment_b`` are each ``[[x1,y1],[x2,y2]]`` in
    the vehicle's current frame (+x forward, +y left) — the two walls
    bounding the gap. The segment endpoints *nearer to each other* are taken
    as the gap's facing edges.
    """
    a = np.asarray(wall_segment_a, dtype=np.float64).reshape(2, 2)
    b = np.asarray(wall_segment_b, dtype=np.float64).reshape(2, 2)

    best = None
    best_d2 = math.inf
    for pa in a:
        for pb in b:
            d2 = float(np.sum((pa - pb) ** 2))
            if d2 < best_d2:
                best_d2 = d2
                best = (pa, pb)
    facing_a, facing_b = best
    mouth = 0.5 * (facing_a + facing_b)

    # Lateral direction into the bay: perpendicular to the gap line, pointing
    # away from the vehicle (x=0) side — i.e. away from the lane.
    lateral_sign = 1.0 if mouth[1] >= 0.0 else -1.0
    goal = np.array([mouth[0], mouth[1] + lateral_sign * depth_m])
    side = 'left' if lateral_sign > 0.0 else 'right'

    r = float(min_turn_radius_m)
    arc_start = (goal[0] - r, 0.0)
    arc_center = (goal[0] - r, lateral_sign * r)
    arc_end = (goal[0], lateral_sign * r)

    return ArcEntryPath(
        mouth_center=(float(mouth[0]), float(mouth[1])),
        goal=(float(goal[0]), float(goal[1])),
        side=side,
        arc_start=arc_start,
        arc_center=arc_center,
        arc_end=arc_end,
        radius_m=r,
        straight1_len_m=arc_start[0],
        # Signed length *along the direction of travel into the bay*
        # (lateral_sign), not the raw y-difference — for a right-side bay
        # (lateral_sign=-1) both goal[1] and arc_end[1] are negative, so the
        # plain difference flips sign relative to actual forward progress.
        straight2_len_m=float(lateral_sign * goal[1] - r),
    )
