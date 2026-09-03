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
The confirmed candidate carries the map-fixed tangent and inward normal of
the initially acquired wall. P0, the circle and the goal are constructed
directly in that frame, so turning the vehicle after detection cannot rotate
the space or move it away from the locked wall.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np

from .geometry import Pose2, wrap_angle
from .wall_gap_detector import TrackedCandidate, WallGapConfig


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
    p0_map = np.array([candidate.map_x, candidate.map_y], dtype=np.float64)
    tangent = np.array([
        candidate.wall_tangent_x, candidate.wall_tangent_y], dtype=np.float64)
    inward = np.array([
        candidate.wall_normal_x, candidate.wall_normal_y], dtype=np.float64)
    vehicle_map = np.array([vehicle_pose.x, vehicle_pose.y], dtype=np.float64)
    r = float(min_turn_radius_m)
    vehicle_along = float(np.dot(vehicle_map - p0_map, tangent))
    direction = 1.0 if vehicle_along >= 0.0 else -1.0
    center_map = p0_map + direction * r * tangent

    to_vehicle = vehicle_map - center_map
    d = float(np.linalg.norm(to_vehicle))
    if d <= r:
        return None  # vehicle is inside the turning circle — degenerate
    angle_c = math.acos(min(1.0, r / d))
    u = to_vehicle / d

    def _rot(vec: np.ndarray, theta: float) -> np.ndarray:
        c, s = math.cos(theta), math.sin(theta)
        return np.array([c * vec[0] - s * vec[1], s * vec[0] + c * vec[1]])

    tangent_candidates = [
        center_map + r * _rot(u, angle_c),
        center_map + r * _rot(u, -angle_c),
    ]
    vehicle_forward = np.array([
        math.cos(vehicle_pose.yaw), math.sin(vehicle_pose.yaw)])
    e_map = max(
        tangent_candidates,
        key=lambda point: float(np.dot(point - vehicle_map, vehicle_forward)),
    )

    a0 = math.atan2(p0_map[1] - center_map[1], p0_map[0] - center_map[0])
    theta_end = math.atan2(e_map[1] - center_map[1], e_map[0] - center_map[0])
    sweep = wrap_angle(theta_end - a0)
    angles = a0 + np.linspace(0.0, sweep, arc_points)
    arc_map = np.column_stack((
        center_map[0] + r * np.cos(angles),
        center_map[1] + r * np.sin(angles),
    ))

    goal_map = p0_map + 0.5 * cfg.square_size_m * inward
    straight1_map = np.array([vehicle_map, e_map])
    straight2_map = np.array([p0_map, goal_map])

    return ReferencePath(
        side=candidate.side,
        radius_m=r,
        p0_map=(float(p0_map[0]), float(p0_map[1])),
        center_map=(float(center_map[0]), float(center_map[1])),
        e_map=(float(e_map[0]), float(e_map[1])),
        goal_map=(float(goal_map[0]), float(goal_map[1])),
        straight1_map=straight1_map,
        arc_map=arc_map,
        straight2_map=straight2_map,
    )
