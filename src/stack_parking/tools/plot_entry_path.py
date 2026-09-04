#!/usr/bin/env python3
"""Plot the straight -> min-radius-arc -> straight entry path
(simple_entry_path.build_entry_path) against a synthetic 1m x 1m+ gap.

No ROS needed — pure geometry + matplotlib, for visually checking the
construction before deciding whether to wire it into the mission FSM.

Usage:
  python3 plot_entry_path.py                       # synthetic example
  python3 plot_entry_path.py --gap 1.5 --depth 2.0 --mouth-x 2.5 --mouth-y 0.6
  python3 plot_entry_path.py --side right
  python3 plot_entry_path.py --out /tmp/entry_path.png
"""

from __future__ import annotations

import argparse
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, __file__.rsplit('/tools/', 1)[0])

from stack_parking.geometry import Pose2  # noqa: E402
from stack_parking.simple_entry_path import build_entry_path  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gap', type=float, default=1.5, help='gap width [m], >=1.0')
    parser.add_argument('--depth', type=float, default=1.5, help='bay depth [m], >=1.0')
    parser.add_argument('--mouth-x', type=float, default=2.75,
                        help='gap center x ahead of vehicle [m]')
    parser.add_argument('--side', choices=('left', 'right'), default='left')
    parser.add_argument('--min-radius', type=float, default=1.15,
                        help='vehicle.min_turn_radius_m')
    parser.add_argument('--out', default='/tmp/entry_path.png')
    args = parser.parse_args()

    if args.gap < 1.0 or args.depth < 1.0:
        print('warning: gap/depth below the 1m x 1m recognition minimum '
              '— this is just a geometry demo, not a detector check.')

    lateral = 0.6  # where the wall segments sit (boundary_near..far band)
    sign = 1.0 if args.side == 'left' else -1.0
    wall_a = [
        [args.mouth_x - 0.5 * args.gap - 1.5, sign * lateral],
        [args.mouth_x - 0.5 * args.gap, sign * lateral],
    ]
    wall_b = [
        [args.mouth_x + 0.5 * args.gap, sign * lateral],
        [args.mouth_x + 0.5 * args.gap + 1.5, sign * lateral],
    ]

    path = build_entry_path(wall_a, wall_b, args.depth, args.min_radius)
    vehicle_pose = Pose2(0.0, 0.0, 0.0)

    s1 = path.straight1_points(vehicle_pose)
    arc = path.arc_points(vehicle_pose)
    s2 = path.straight2_points(vehicle_pose)

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(*zip(*wall_a), color='k', lw=4, label='wall A')
    ax.plot(*zip(*wall_b), color='dimgray', lw=4, label='wall B')
    ax.plot(s1[:, 0], s1[:, 1], 'b-', lw=2, label='straight 1')
    ax.plot(arc[:, 0], arc[:, 1], 'g-', lw=2, label='arc (r=%.2fm)' % path.radius_m)
    ax.plot(s2[:, 0], s2[:, 1], 'r-', lw=2, label='straight 2')

    ax.plot(0, 0, 'k^', ms=12, label='vehicle start')
    ax.plot(*path.mouth_center, 'mx', ms=10, mew=2, label='mouth center M')
    ax.plot(*path.goal, 'r*', ms=16, label='goal B')
    ax.plot(*path.arc_center, 'g+', ms=12, mew=2, label='arc center C')

    neg = []
    if path.straight1_len_m < 0:
        neg.append('straight1_len_m=%.2f (< 0: gap closer than min radius)'
                   % path.straight1_len_m)
    if path.straight2_len_m < 0:
        neg.append('straight2_len_m=%.2f (< 0: depth shorter than min radius)'
                   % path.straight2_len_m)
    title = 'ArcEntryPath — side=%s r=%.2fm' % (path.side, path.radius_m)
    if neg:
        title += '\nINVALID: ' + '; '.join(neg)
    ax.set_title(title)
    ax.set_xlabel('x (vehicle forward) [m]')
    ax.set_ylabel('y (vehicle left) [m]')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=8)
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print('saved', args.out)
    print('mouth_center=%s goal=%s arc_start=%s arc_center=%s arc_end=%s'
          % (path.mouth_center, path.goal, path.arc_start, path.arc_center, path.arc_end))
    print('straight1_len_m=%.3f straight2_len_m=%.3f' %
          (path.straight1_len_m, path.straight2_len_m))


if __name__ == '__main__':
    main()
