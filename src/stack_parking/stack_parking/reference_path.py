"""Map-fixed reference path for a wall-gap candidate.

Every coordinate is derived from P0, the midpoint of the confirmed square's
wall-side edge:

  S ---- 2m wall-parallel straight ---- E
                                          ) 90-degree arc, radius R_min
                                       P0
                                        |
                                        | 2m wall-normal straight into bay
                                        G

The geometric parking traversal is S -> E -> P0 -> G. At E the arc tangent
is parallel to the wall; at P0 it is perpendicular to the wall and exactly
collinear with P0 -> G. The vehicle yaw is used only to select which mirrored
side of P0 contains S/E. It does not translate or rotate the map-fixed path.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np

from .geometry import Pose2, wrap_angle
from .wall_gap_detector import TrackedCandidate


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
    min_turn_radius_m: float,
    inside_straight_m: float = 2.0,
    parallel_straight_m: float = 2.0,
    arc_points: int = 24,
) -> Optional[ReferencePath]:
    p0_map = np.array([candidate.map_x, candidate.map_y], dtype=np.float64)
    tangent = np.array([
        candidate.wall_tangent_x, candidate.wall_tangent_y], dtype=np.float64)
    inward = np.array([
        candidate.wall_normal_x, candidate.wall_normal_y], dtype=np.float64)
    r = float(min_turn_radius_m)
    inside_length = float(inside_straight_m)
    parallel_length = float(parallel_straight_m)
    if r <= 0.0 or inside_length <= 0.0 or parallel_length <= 0.0:
        return None

    # Candidate tangent is aligned with the startup vehicle direction. Use
    # the live vehicle yaw only to select the forward mirrored construction.
    vehicle_forward = np.array([
        math.cos(vehicle_pose.yaw), math.sin(vehicle_pose.yaw)])
    direction = 1.0 if float(np.dot(vehicle_forward, tangent)) >= 0.0 else -1.0
    center_map = p0_map + direction * r * tangent
    e_map = center_map - r * inward
    start_map = e_map + direction * parallel_length * tangent

    theta_start = math.atan2(
        e_map[1] - center_map[1], e_map[0] - center_map[0])
    theta_end = math.atan2(
        p0_map[1] - center_map[1], p0_map[0] - center_map[0])
    sweep = wrap_angle(theta_end - theta_start)
    angles = theta_start + np.linspace(0.0, sweep, max(2, int(arc_points)))
    arc_map = np.column_stack((
        center_map[0] + r * np.cos(angles),
        center_map[1] + r * np.sin(angles),
    ))

    goal_map = p0_map + inside_length * inward
    straight1_map = np.array([start_map, e_map])
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


def mirror_reference_path_about_inside_straight(
    reference: ReferencePath,
) -> Optional[ReferencePath]:
    """Mirror a path about its P0-to-goal parking-bay centreline.

    The wall-normal straight remains fixed.  The wall-parallel straight and
    quarter-circle move to the opposite side, giving the vehicle a distinct
    forward pull-out path after it has reversed into the bay.
    """
    p0_map = np.asarray(reference.p0_map, dtype=np.float64)
    goal_map = np.asarray(reference.goal_map, dtype=np.float64)
    axis = goal_map - p0_map
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm <= 1.0e-9:
        return None
    axis /= axis_norm
    reflection = 2.0 * np.outer(axis, axis) - np.eye(2)

    def reflect_points(points) -> np.ndarray:
        values = np.asarray(points, dtype=np.float64)
        return p0_map + (values - p0_map) @ reflection.T

    center_map = reflect_points(reference.center_map)
    e_map = reflect_points(reference.e_map)
    mirrored_goal = reflect_points(reference.goal_map)
    mirrored_side = {
        'left': 'right',
        'right': 'left',
    }.get(reference.side, reference.side)
    return ReferencePath(
        side=mirrored_side,
        radius_m=reference.radius_m,
        p0_map=reference.p0_map,
        center_map=(float(center_map[0]), float(center_map[1])),
        e_map=(float(e_map[0]), float(e_map[1])),
        goal_map=(float(mirrored_goal[0]), float(mirrored_goal[1])),
        straight1_map=reflect_points(reference.straight1_map),
        arc_map=reflect_points(reference.arc_map),
        straight2_map=reflect_points(reference.straight2_map),
    )
