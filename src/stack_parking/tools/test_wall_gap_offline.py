#!/usr/bin/env python3
"""Offline test of wall_gap_detector against the real room geometry
(reconstructed from the 2026-09-02 live-map analysis — no recording existed
at the time, see record_map.py for capturing one going forward).

Room, in vehicle/lane frame at the moment of analysis:
  - left arm:     x in [1.60, 1.93], y from ~1.32 to ~3.77 (dense)
  - right arm:    x in [3.12, 3.24], y from ~1.6  to ~2.7
  - back wall:    y ~= 2.65-2.75,   x from ~1.8 to ~3.2 (connects the arms)
  - clutter row:  y ~= 1.1-1.65,    x from -3.5 to +2.0 (continuous, NOT
                  part of the bay — this is exactly what broke plain x-axis
                  clustering in space_detector.py before the 2D-blob fix)
  - far clutter:  x in [-3.16,-2.20], y ~= 1.0-1.05
  - opposite wall: y ~= -1.0..-1.1, x from -4 to +4 (right side, unrelated)

Runs the detector through simulated forward motion and reports whether/when
it finds and confirms the bay, with both wall_gap_detector's *default*
near/far band (0.3-1.6m, tuned for "gap between two parked cars") and a
band tuned to this room's actual scale (1.7-2.3m) — the point is to show
this new algorithm has the *same* scene-scale dependency the old one did,
not a magic fix for it.
"""

from __future__ import annotations

import numpy as np

from stack_parking.geometry import Pose2
from stack_parking.wall_gap_detector import WallGapConfig, WallGapDetector


def build_room() -> np.ndarray:
    rng = np.random.default_rng(0)

    def line(x0, x1, y0, y1, n, jitter=0.01):
        t = np.linspace(0.0, 1.0, n)
        x = x0 + (x1 - x0) * t + rng.normal(0, jitter, n)
        y = y0 + (y1 - y0) * t + rng.normal(0, jitter, n)
        return np.column_stack((x, y))

    # Back wall at near_m + 1.4m depth (2026-09-02: real deployment will be
    # laid out at ~1.4m depth, not this room's ~0.98m — arms extended to
    # match so they still reach the back wall).
    left_arm = line(1.75, 1.80, 1.32, 4.20, 44)
    right_arm = line(3.18, 3.20, 1.6, 3.15, 26)
    back_wall = line(1.85, 3.15, 3.12, 3.10, 26)
    clutter_row = line(-3.5, 2.0, 1.1, 1.65, 90)
    far_clutter = line(-3.16, -2.20, 1.0, 1.05, 15)
    opposite_wall = line(-4.0, 4.0, -1.05, -1.0, 100)

    return np.vstack([
        left_arm, right_arm, back_wall, clutter_row, far_clutter, opposite_wall,
    ])


def run(label: str, cfg: WallGapConfig, room: np.ndarray) -> None:
    print('--- %s (near=%.2f far=%.2f) ---' % (label, cfg.near_m, cfg.far_m))
    det = WallGapDetector(cfg)
    # Simulate the vehicle driving from x=-1.0 to x=4.0 in 0.2m steps —
    # each step is one _tick equivalent (the real node runs this every map
    # update, here it's just "one detector.update() call per position").
    found = None
    for x in np.arange(-1.0, 4.0, 0.2):
        result = det.update(room, Pose2(float(x), 0.0, 0.0))
        if result is not None and found is None:
            found = (x, result)
    if found:
        x, cand = found
        print('  CONFIRMED at vehicle x=%.2f: side=%s map=(%.2f,%.2f) width=%.2fm'
              % (x, cand.side, cand.map_x, cand.map_y, cand.width_m))
    else:
        print('  not confirmed. tracked candidates:')
        for c in det.tracked:
            print('    side=%s map=(%.2f,%.2f) width=%.2f tested=%s clear=%s'
                  % (c.side, c.map_x, c.map_y, c.width_m, c.tested, c.clear))
        if not det.tracked:
            print('    (none — no gap >= min_gap_m ever seen in this band)')


def main() -> None:
    room = build_room()
    print('room fixture: %d points' % len(room))

    run('default band', WallGapConfig(), room)
    run('room-tuned band', WallGapConfig(near_m=1.7, far_m=2.3), room)


if __name__ == '__main__':
    main()
