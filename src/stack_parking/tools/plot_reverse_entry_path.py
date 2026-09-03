#!/usr/bin/env python3
"""Plot the fixed-wall detector -> square -> reference-path pipeline."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, __file__.rsplit('/tools/', 1)[0])

from stack_parking.geometry import Pose2  # noqa: E402
from stack_parking.reference_path import build_reference_path  # noqa: E402
from stack_parking.wall_gap_detector import (  # noqa: E402
    SIDE_LEFT,
    WallGapConfig,
    WallGapDetector,
    candidate_square_corners,
)


def build_scene() -> tuple[np.ndarray, np.ndarray]:
    rng = np.random.default_rng(0)
    angle = math.radians(10.0)
    tangent = np.array([math.cos(angle), math.sin(angle)])
    normal = np.array([-math.sin(angle), math.cos(angle)])
    anchor = normal

    def segment(start_s: float, end_s: float, count: int) -> np.ndarray:
        along = np.linspace(start_s, end_s, count)
        points = anchor + np.outer(along, tangent)
        return points + rng.normal(0.0, 0.008, points.shape)

    return np.vstack((segment(-0.8, 0.8, 36),
                      segment(2.2, 4.0, 42))), tangent


def main() -> None:
    scene, tangent = build_scene()
    cfg = WallGapConfig(
        search_sides=(SIDE_LEFT,), initial_wall_max_angle_deg=30.0)
    detector = WallGapDetector(cfg)
    detector.set_seed(Pose2(), SIDE_LEFT)

    confirmed = None
    confirm_s = None
    for along in np.arange(0.0, 3.0, 0.1):
        vehicle_xy = along * tangent
        result = detector.update(scene, Pose2(
            float(vehicle_xy[0]), float(vehicle_xy[1]),
            math.radians(35.0) * math.sin(float(along))))
        if result is not None:
            confirmed = result
            confirm_s = float(along)
            break
    if confirmed is None or confirm_s is None:
        raise SystemExit('no candidate confirmed')

    advanced_xy = (confirm_s + 0.5) * tangent
    advanced_pose = Pose2(
        float(advanced_xy[0]), float(advanced_xy[1]), math.radians(30.0))
    path = build_reference_path(
        confirmed, advanced_pose, cfg, min_turn_radius_m=1.15)
    if path is None:
        raise SystemExit('reference path is degenerate')

    wall = detector.reference_walls[SIDE_LEFT]
    square = candidate_square_corners(confirmed, cfg)
    reference = wall.to_map(np.array([[-1.5, 0.0], [4.8, 0.0]]))
    offset_a = wall.to_map(np.array([
        [-1.5, -cfg.wall_line_offset_m], [4.8, -cfg.wall_line_offset_m]]))
    offset_b = wall.to_map(np.array([
        [-1.5, cfg.wall_line_offset_m], [4.8, cfg.wall_line_offset_m]]))

    fig, ax = plt.subplots(figsize=(9, 7))
    ax.scatter(scene[:, 0], scene[:, 1], s=8, c='k', alpha=0.55,
               label='map points')
    ax.plot(reference[:, 0], reference[:, 1], color='cyan', lw=1.5,
            label='locked reference wall')
    ax.plot(offset_a[:, 0], offset_a[:, 1], 'c--', lw=0.8,
            label='wall offset band')
    ax.plot(offset_b[:, 0], offset_b[:, 1], 'c--', lw=0.8)
    ax.plot(square[:, 0], square[:, 1], 'y--', lw=1.5,
            label='confirmed square')
    ax.plot(path.straight1_map[:, 0], path.straight1_map[:, 1], 'b-', lw=2,
            label='straight 1')
    ax.plot(path.arc_map[:, 0], path.arc_map[:, 1], 'g-', lw=2,
            label='arc')
    ax.plot(path.straight2_map[:, 0], path.straight2_map[:, 1], 'r-', lw=2,
            label='straight 2')
    ax.plot(advanced_pose.x, advanced_pose.y, 'r^', ms=12,
            label='vehicle after 0.5m advance')
    ax.plot(*path.p0_map, 'mx', ms=10, mew=2, label='P0')
    ax.plot(*path.goal_map, 'r*', ms=14, label='goal')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('parking_map x [m]')
    ax.set_ylabel('parking_map y [m]')
    ax.set_title('fixed reference wall and parking path')
    ax.legend(loc='best', fontsize=8)
    fig.tight_layout()
    output = Path.cwd() / 'reverse_entry_path.png'
    fig.savefig(output, dpi=150)
    print('saved', output)


if __name__ == '__main__':
    main()
