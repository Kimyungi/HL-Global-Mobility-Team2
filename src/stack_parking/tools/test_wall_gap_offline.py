#!/usr/bin/env python3
"""Offline check for fixed-reference wall-gap detection.

The fixture contains a sloped left wall with a 1.4m opening and a parallel
row of clutter outside the configured wall offset. The simulated vehicle
changes yaw while passing the opening; the acquired reference line must stay
bit-for-bit unchanged and the opening must still be confirmed.
"""

from __future__ import annotations

import math

import numpy as np

from stack_parking.geometry import Pose2
from stack_parking.wall_gap_detector import (
    SIDE_LEFT,
    WallGapConfig,
    WallGapDetector,
)


def build_scene() -> tuple[np.ndarray, np.ndarray]:
    rng = np.random.default_rng(0)
    angle = math.radians(10.0)
    tangent = np.array([math.cos(angle), math.sin(angle)])
    normal = np.array([-math.sin(angle), math.cos(angle)])
    anchor = 1.0 * normal

    def segment(start_s: float, end_s: float, count: int) -> np.ndarray:
        along = np.linspace(start_s, end_s, count)
        points = anchor + np.outer(along, tangent)
        return points + rng.normal(0.0, 0.008, points.shape)

    wall = np.vstack((segment(-0.8, 0.8, 36), segment(2.2, 4.0, 42)))
    # This row is parallel but 30cm toward the lane, outside +/-12cm.
    clutter_s = np.linspace(0.5, 2.5, 34)
    clutter = (
        anchor + np.outer(clutter_s, tangent) - 0.30 * normal
        + rng.normal(0.0, 0.006, (len(clutter_s), 2)))
    return np.vstack((wall, clutter)), tangent


def main() -> None:
    scene, tangent = build_scene()
    cfg = WallGapConfig(
        search_sides=(SIDE_LEFT,),
        wall_line_offset_m=0.12,
        initial_wall_max_angle_deg=30.0,
    )
    detector = WallGapDetector(cfg)
    detector.set_seed(Pose2(), SIDE_LEFT)

    confirmed = None
    locked_reference = None
    for along in np.arange(0.0, 3.0, 0.1):
        # Deliberately rotate the vehicle by up to 50deg. Detection uses only
        # its projected progress; the reference wall remains in parking_map.
        yaw = math.radians(50.0) * math.sin(float(along))
        vehicle_xy = along * tangent
        result = detector.update(scene, Pose2(
            float(vehicle_xy[0]), float(vehicle_xy[1]), yaw))
        reference = detector.reference_walls.get(SIDE_LEFT)
        if reference is not None and locked_reference is None:
            locked_reference = reference
            print('LOCKED yaw=%.2fdeg distance=%.3fm offset=+/-%.2fm' % (
                math.degrees(reference.yaw),
                reference.distance_from_seed_m,
                cfg.wall_line_offset_m,
            ))
        elif reference is not None and reference != locked_reference:
            raise AssertionError('reference wall moved after acquisition')
        if result is not None:
            confirmed = result
            print('CONFIRMED along=%.2fm map=(%.2f,%.2f) width=%.2fm' % (
                along, result.map_x, result.map_y, result.width_m))
            break

    if locked_reference is None:
        raise SystemExit('failed to acquire the initial left wall')
    if confirmed is None:
        raise SystemExit('failed to confirm the wall opening')


if __name__ == '__main__':
    main()
