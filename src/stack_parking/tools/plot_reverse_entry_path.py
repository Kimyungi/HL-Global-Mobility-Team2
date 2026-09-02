#!/usr/bin/env python3
"""Plot the wall_gap_detector -> reference-path pipeline end to end:
room (offline fixture) -> confirmed square -> 50cm advance -> reference path.

P0 = centre of the square's near-wall edge (2026-09-02 spec) is fed straight
into simple_entry_path.build_entry_path() as the mouth point (depth =
0.5 * square_size_m, reaching the square's centre) — that function already
builds a tangent-continuous straight -> arc -> straight path (verified
earlier: A sits on the vehicle's y=0 heading line so straight 1 has no kink,
and the arc's radius vector is vertical at A / horizontal at E so both
transitions are tangent). An earlier version of this script re-derived the
arc by hand anchored differently and got a non-tangent, wrong-direction
straight 1 — fixed by reusing the validated construction instead.
"""

from __future__ import annotations

import math
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, __file__.rsplit('/tools/', 1)[0])

from stack_parking.geometry import Pose2, points_in_frame, transform_points, wrap_angle  # noqa: E402
from stack_parking.wall_gap_detector import SIDE_LEFT, WallGapConfig, WallGapDetector  # noqa: E402


def build_room() -> np.ndarray:
    rng = np.random.default_rng(0)

    def line(x0, x1, y0, y1, n, jitter=0.01):
        t = np.linspace(0.0, 1.0, n)
        x = x0 + (x1 - x0) * t + rng.normal(0, jitter, n)
        y = y0 + (y1 - y0) * t + rng.normal(0, jitter, n)
        return np.column_stack((x, y))

    left_arm = line(1.75, 1.80, 1.32, 4.20, 44)
    right_arm = line(3.18, 3.20, 1.6, 3.15, 26)
    back_wall = line(1.85, 3.15, 3.12, 3.10, 26)
    clutter_row = line(-3.5, 2.0, 1.1, 1.65, 90)
    far_clutter = line(-3.16, -2.20, 1.0, 1.05, 15)
    opposite_wall = line(-4.0, 4.0, -1.05, -1.0, 100)
    return np.vstack([
        left_arm, right_arm, back_wall, clutter_row, far_clutter, opposite_wall,
    ])


def main() -> None:
    room = build_room()
    cfg = WallGapConfig(near_m=1.7, far_m=2.3)
    det = WallGapDetector(cfg)

    confirmed = None
    confirm_x = None
    for x in np.arange(-1.0, 4.0, 0.2):
        result = det.update(room, Pose2(float(x), 0.0, 0.0))
        if result is not None:
            confirmed = result
            confirm_x = x
            break
    if confirmed is None:
        print('no candidate confirmed — nothing to plot')
        return
    print('confirmed at vehicle x=%.2f: %s' % (confirm_x, confirmed))

    # Advance the vehicle 50cm further, as specified.
    advanced_pose = Pose2(confirm_x + 0.5, 0.0, 0.0)

    side_sign = 1.0 if confirmed.side == SIDE_LEFT else -1.0
    p0_local = points_in_frame(
        np.array([[confirmed.map_x, confirmed.map_y]]), advanced_pose)[0]
    # confirmed.map_x/map_y were stored at the *gap centre, mid-depth* (see
    # WallGapDetector.update) — pin P0 to the wall line itself (near_m), not
    # that mid-depth point, since P0 is defined as the square's near-wall
    # edge centre.
    p0_local = np.array([p0_local[0], side_sign * cfg.near_m])

    r = 1.15  # vehicle.min_turn_radius_m (CLAUDE.md single source)
    # Corrected per user feedback: the centre must be on the vehicle side of
    # P0 (P0 + r horizontally *toward* the vehicle), not on the far/goal
    # side — the earlier build_entry_path reuse put it on the wrong side.
    direction = 1.0 if p0_local[0] < 0.0 else -1.0
    center_local = np.array([p0_local[0] + direction * r, p0_local[1]])

    # E is where the tangent line *from the vehicle itself* touches this
    # circle — that's what actually guarantees straight1 has no kink (a
    # fixed 90-degree sweep from P0, tried earlier, put E off that tangent
    # line by ~30 degrees; the tangent-from-vehicle construction is exact,
    # at the cost of the sweep not landing on exactly 90 degrees).
    to_vehicle = -center_local  # vehicle is at local (0,0)
    d = float(np.linalg.norm(to_vehicle))
    angle_c = math.acos(min(1.0, r / d))
    u = to_vehicle / d

    def _rot(vec, theta):
        c, s = math.cos(theta), math.sin(theta)
        return np.array([c * vec[0] - s * vec[1], s * vec[0] + c * vec[1]])

    candidates = [center_local + r * _rot(u, angle_c),
                 center_local + r * _rot(u, -angle_c)]
    # Pick whichever tangent point is on the vehicle's forward (+x) side —
    # the other one sits behind the vehicle, which isn't a usable approach.
    e_local = max(candidates, key=lambda p: p[0])

    a0 = math.atan2(p0_local[1] - center_local[1], p0_local[0] - center_local[0])
    theta_end = math.atan2(e_local[1] - center_local[1], e_local[0] - center_local[0])
    sweep = wrap_angle(theta_end - a0)
    goal_local = p0_local + np.array([0.0, side_sign * 0.5 * cfg.square_size_m])

    n = 24
    angles = a0 + np.linspace(0.0, sweep, n)
    arc_local = np.column_stack((
        center_local[0] + r * np.cos(angles), center_local[1] + r * np.sin(angles)))
    straight1_local = np.array([[0.0, 0.0], list(e_local)])
    straight2_local = np.array([list(p0_local), list(goal_local)])

    # Tangency check at E: heading of straight1 vs. the arc's tangent there.
    straight1_heading = math.atan2(e_local[1] - 0.0, e_local[0] - 0.0)
    # Tangent direction at a point on a circle, angle theta from centre,
    # travelling in the direction of increasing angle (CCW) is
    # (-sin theta, cos theta); flip it if the sweep runs the other way (CW).
    ccw = np.sign(sweep) if sweep != 0 else 1.0
    tangent_at_e = math.atan2(ccw * math.cos(theta_end), -ccw * math.sin(theta_end))
    print('straight1 heading=%.1fdeg, arc tangent at E=%.1fdeg (should match for no kink)'
          % (math.degrees(straight1_heading), math.degrees(tangent_at_e)))

    to_map = lambda pts: transform_points(np.asarray(pts), advanced_pose)  # noqa: E731

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.scatter(room[:, 0], room[:, 1], s=4, c='k', alpha=0.5, label='map points')
    veh_map = advanced_pose.x, advanced_pose.y
    ax.plot(veh_map[0], veh_map[1], 'r^', ms=14, label='vehicle (after 50cm advance)')

    s1 = to_map(straight1_local)
    arc = to_map(arc_local)
    s2 = to_map(straight2_local)
    ax.plot(s1[:, 0], s1[:, 1], 'b-', lw=2, label='straight 1 (vehicle -> E)')
    ax.plot(arc[:, 0], arc[:, 1], 'g-', lw=2, label='arc (E -> P0, r=%.2fm)' % r)
    ax.plot(s2[:, 0], s2[:, 1], 'r-', lw=2, label='straight 2 (P0 -> goal)')

    p0_map = to_map([list(p0_local)])[0]
    c_map = to_map([list(center_local)])[0]
    e_map = to_map([list(e_local)])[0]
    goal_map = to_map([list(goal_local)])[0]
    ax.plot(*p0_map, 'mx', ms=10, mew=2, label='P0 (square near-wall edge centre)')
    ax.plot(*c_map, 'g+', ms=12, mew=2, label='arc centre C (= P0 + r toward vehicle)')
    ax.plot(*e_map, 'k+', ms=12, mew=2, label='E (arc end, 90deg from P0)')
    ax.plot(*goal_map, 'r*', ms=16, label='goal (square centre)')

    # square outline for context
    half = 0.5 * cfg.square_size_m
    sq_local = np.array([
        [p0_local[0] - half, side_sign * cfg.near_m],
        [p0_local[0] + half, side_sign * cfg.near_m],
        [p0_local[0] + half, side_sign * (cfg.near_m + cfg.square_size_m)],
        [p0_local[0] - half, side_sign * (cfg.near_m + cfg.square_size_m)],
        [p0_local[0] - half, side_sign * cfg.near_m],
    ])
    sq_map = to_map(sq_local)
    ax.plot(sq_map[:, 0], sq_map[:, 1], 'y--', lw=1.5, label='confirmed square')

    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('map x [m]')
    ax.set_ylabel('map y [m]')
    ax.set_title('reference path: side=%s r=%.2fm' % (confirmed.side, r))
    ax.legend(loc='best', fontsize=7)
    fig.tight_layout()
    out = '/tmp/reverse_entry_path.png'
    fig.savefig(out, dpi=150)
    print('saved', out)


if __name__ == '__main__':
    main()
