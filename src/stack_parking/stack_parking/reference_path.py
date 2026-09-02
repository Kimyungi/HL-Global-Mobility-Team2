"""Reference-path construction for a wall_gap_detector-confirmed space.

2026-09-02 user spec, refined through two plotted iterations (both bugs are
noted here so they don't get reintroduced):
  P0 = centre of the confirmed square's near-wall edge (map position of the
       candidate, pinned to the wall line depth near_m).
  C  = P0 shifted by min_turn_radius_m horizontally *toward the vehicle*
       (not away from it — the first version anchored C on the goal side by
       reusing simple_entry_path.py's construction, which the user flagged
       as backwards).
  E  = the point where the tangent line *from the vehicle's actual position*
       touches the circle (centre C, radius r) — not a fixed 90-degree
       sweep from P0 (tried first: left a ~30 degree kink between the
       straight approach and the arc, since a fixed-angle sweep from P0
       generally doesn't land on the vehicle's tangent line).
  goal = P0 pushed further into the bay by half the square size (the
         square's own centre) — the final parking position.
Path = straight(vehicle -> E) + arc(E -> P0) + straight(P0 -> goal).
All returned points are in *map frame*, computed once at the vehicle pose
passed in (call again if the vehicle moves and a fresh path is needed).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np

from .geometry import Pose2, points_in_frame, transform_points, wrap_angle
from .wall_gap_detector import SIDE_LEFT, TrackedCandidate, WallGapConfig


@dataclass(frozen=True)
class ReferencePath:
    side: str
    radius_m: float
    p0_map: tuple[float, float]
    center_map: tuple[float, float]
    e_map: tuple[float, float]
    goal_map: tuple[float, float]
    straight1_map: np.ndarray
    arc_map: np.ndarray
    straight2_map: np.ndarray


def build_reference_path(
    candidate: TrackedCandidate,
    vehicle_pose: Pose2,
    cfg: WallGapConfig,
    min_turn_radius_m: float,
    arc_points: int = 24,
) -> Optional[ReferencePath]:
    side_sign = 1.0 if candidate.side == SIDE_LEFT else -1.0
    p0_local = points_in_frame(
        np.array([[candidate.map_x, candidate.map_y]]), vehicle_pose)[0]
    # Pinned to the candidate's actual nearest-point-cluster distance (user
    # directive, 2026-09-02), not the configured near_m search-band edge.
    p0_local = np.array([p0_local[0], side_sign * candidate.near_distance])

    r = float(min_turn_radius_m)
    direction = 1.0 if p0_local[0] < 0.0 else -1.0
    center_local = np.array([p0_local[0] + direction * r, p0_local[1]])

    to_vehicle = -center_local
    d = float(np.linalg.norm(to_vehicle))
    if d <= r:
        return None  # vehicle is inside the turning circle — degenerate
    angle_c = math.acos(min(1.0, r / d))
    u = to_vehicle / d

    def _rot(vec: np.ndarray, theta: float) -> np.ndarray:
        c, s = math.cos(theta), math.sin(theta)
        return np.array([c * vec[0] - s * vec[1], s * vec[0] + c * vec[1]])

    tangent_candidates = [
        center_local + r * _rot(u, angle_c),
        center_local + r * _rot(u, -angle_c),
    ]
    # The forward (+x) tangent point is the usable approach; the other one
    # sits behind the vehicle.
    e_local = max(tangent_candidates, key=lambda p: p[0])

    a0 = math.atan2(p0_local[1] - center_local[1], p0_local[0] - center_local[0])
    theta_end = math.atan2(e_local[1] - center_local[1], e_local[0] - center_local[0])
    sweep = wrap_angle(theta_end - a0)
    angles = a0 + np.linspace(0.0, sweep, arc_points)
    arc_local = np.column_stack((
        center_local[0] + r * np.cos(angles), center_local[1] + r * np.sin(angles)))

    goal_local = p0_local + np.array([0.0, side_sign * 0.5 * cfg.square_size_m])
    straight1_local = np.array([[0.0, 0.0], list(e_local)])
    straight2_local = np.array([list(p0_local), list(goal_local)])

    def to_map(pts):
        return transform_points(np.asarray(pts), vehicle_pose)

    p0_map = to_map([list(p0_local)])[0]
    center_map = to_map([list(center_local)])[0]
    e_map = to_map([list(e_local)])[0]
    goal_map = to_map([list(goal_local)])[0]

    return ReferencePath(
        side=candidate.side,
        radius_m=r,
        p0_map=(float(p0_map[0]), float(p0_map[1])),
        center_map=(float(center_map[0]), float(center_map[1])),
        e_map=(float(e_map[0]), float(e_map[1])),
        goal_map=(float(goal_map[0]), float(goal_map[1])),
        straight1_map=to_map(straight1_local),
        arc_map=to_map(arc_local),
        straight2_map=to_map(straight2_local),
    )
