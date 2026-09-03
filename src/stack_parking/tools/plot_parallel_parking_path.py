#!/usr/bin/env python3
"""Offline simulation plot for the dedicated parallel-parking test."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from stack_parking.geometry import Pose2  # noqa: E402
from stack_parking.parallel_parking import (  # noqa: E402
    ParallelControlState,
    ParallelParkingConfig,
    ParallelParkingController,
    build_parallel_reference_path,
    candidate_rectangle_corners,
    parallel_controller_paths,
    rectangle_is_clear,
)
from stack_parking.wall_gap_detector import (  # noqa: E402
    SIDE_LEFT,
    WallGapConfig,
    WallGapDetector,
)


def build_scene() -> np.ndarray:
    rng = np.random.default_rng(4)
    wall_angle = math.radians(6.0)
    tangent = np.asarray([math.cos(wall_angle), math.sin(wall_angle)])
    normal = np.asarray([-math.sin(wall_angle), math.cos(wall_angle)])
    anchor = normal

    def segment(start: float, end: float, count: int) -> np.ndarray:
        along = np.linspace(start, end, count)
        return (anchor + np.outer(along, tangent)
                + rng.normal(0.0, 0.006, (count, 2)))

    return np.vstack((segment(-4.0, -0.82, 65), segment(0.82, 4.0, 65)))


def pose(point) -> Pose2:
    return Pose2(point.x, point.y, point.yaw)


def run_simulation(path):
    controller = ParallelParkingController(ParallelParkingConfig(
        direction_change_hold_s=1.0,
        preview_distance_m=1.5,
        forward_speed_mps=0.75,
        reverse_speed_mps=0.75,
        sample_step_m=0.05,
    ))
    forward_samples = []
    reverse_samples = []
    return_samples = []
    full_forward, _ = parallel_controller_paths(path)
    p0_index = min(range(len(full_forward)), key=lambda index: math.hypot(
        full_forward[index].x - path.p0_map[0],
        full_forward[index].y - path.p0_map[1],
    ))
    start_pose = pose(full_forward[p0_index])
    if not controller.start(path, start_pose, 0.0):
        raise RuntimeError('parallel controller rejected the S path')

    now_s = 0.0
    for point in controller.forward_path[controller.progress:]:
        output = controller.update(pose(point), now_s)
        forward_samples.append((point.x, point.y))
        now_s += 0.05
        if output.state == ParallelControlState.FRONT_HOLD:
            front_stop = pose(point)
            break
    else:
        raise RuntimeError('forward preview did not reach the front end')

    now_s += 1.01
    output = controller.update(front_stop, now_s)
    if output.state != ParallelControlState.REVERSE:
        raise RuntimeError('controller did not leave the first 1s hold')
    for point in controller.reverse_path[controller.progress:]:
        output = controller.update(pose(point), now_s)
        reverse_samples.append((point.x, point.y))
        now_s += 0.05
        if output.state == ParallelControlState.REAR_HOLD:
            rear_stop = pose(point)
            break
    else:
        raise RuntimeError('reverse preview did not reach the rear end')

    now_s += 1.01
    output = controller.update(rear_stop, now_s)
    if output.state != ParallelControlState.FORWARD_RETURN:
        raise RuntimeError('controller did not leave the reverse-end hold')
    for point in controller.forward_path[controller.progress:]:
        update_time_s = now_s
        output = controller.update(pose(point), update_time_s)
        return_samples.append((point.x, point.y))
        now_s += 0.05
        if output.state == ParallelControlState.FINAL_HOLD:
            final_stop = pose(point)
            final_hold_started_s = update_time_s
            break
    else:
        raise RuntimeError('return preview did not reach the front end')

    output = controller.update(final_stop, final_hold_started_s + 0.99)
    if output.state != ParallelControlState.FINAL_HOLD:
        raise RuntimeError('final hold ended before one second')
    output = controller.update(final_stop, final_hold_started_s + 1.0)
    if (output.state != ParallelControlState.STOPPED
            or output.status != 'parallel_parking_complete'):
        raise RuntimeError('final hold did not complete logging state')

    return {
        'controller': controller,
        'forward': np.asarray(forward_samples),
        'reverse': np.asarray(reverse_samples),
        'return': np.asarray(return_samples),
        'front_stop': front_stop,
        'rear_stop': rear_stop,
        'final_stop': final_stop,
        'final_status': output.status,
    }


def arrowed(ax, points, color, label):
    ax.plot(points[:, 0], points[:, 1], color=color, lw=3.0, label=label)
    if len(points) > 1:
        for index in np.linspace(0, len(points) - 2, 4, dtype=int):
            ax.annotate('', xy=points[index + 1], xytext=points[index],
                        arrowprops={'arrowstyle': '-|>', 'color': color,
                                    'lw': 1.8, 'mutation_scale': 12})


def context(ax, scene, rectangle, path):
    ax.scatter(scene[:, 0], scene[:, 1], s=7, c='0.25', alpha=0.55,
               label='left-wall LiDAR points')
    ax.plot(rectangle[:, 0], rectangle[:, 1], color='gold', ls='--', lw=2,
            label='1.5m x 0.7m clear rectangle')
    ax.plot(path.rear_line_map[:, 0], path.rear_line_map[:, 1],
            color='0.65', ls=':', lw=1.5)
    ax.plot(path.rear_arc_map[:, 0], path.rear_arc_map[:, 1],
            color='0.65', ls=':', lw=1.5)
    ax.plot(path.front_arc_map[:, 0], path.front_arc_map[:, 1],
            color='0.65', ls=':', lw=1.5)
    ax.plot(path.front_line_map[:, 0], path.front_line_map[:, 1],
            color='0.65', ls=':', lw=1.5, label='R1.12m S reference path')
    ax.plot(path.p0_map[0], path.p0_map[1], 'mx', ms=10, mew=2,
            label='P0: wall-edge midpoint')
    ax.plot(path.arc_origin_map[0], path.arc_origin_map[1], 'g+', ms=12,
            mew=2, label='arc origin: P0 + 0.25m wall-parallel forward')
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.25)
    ax.set_xlabel('parking_map x [m]')


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', type=Path, default=Path.cwd())
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    scene = build_scene()
    detector = WallGapDetector(WallGapConfig(
        min_gap_m=1.5,
        square_size_m=0.7,
        search_sides=(SIDE_LEFT,),
        initial_wall_max_angle_deg=20.0,
    ))
    detector.set_seed(Pose2(), SIDE_LEFT)
    candidate = None
    pending_candidate = None
    for vehicle_x in np.arange(-1.0, 0.31, 0.1):
        newly_clear = detector.update(scene, Pose2(float(vehicle_x), 0.0, 0.0))
        wall = detector.reference_walls.get(SIDE_LEFT)
        if newly_clear is not None and wall is not None:
            center_s = wall.project(np.asarray([
                [newly_clear.map_x, newly_clear.map_y]]))[0, 0]
            newly_clear.clear = rectangle_is_clear(
                wall.project(scene), float(center_s), 1.5, 0.7)
            if newly_clear.clear:
                pending_candidate = newly_clear
        if pending_candidate is not None and wall is not None:
            center_s = wall.project(np.asarray([[
                pending_candidate.map_x, pending_candidate.map_y]]))[0, 0]
            vehicle_s = wall.project(np.asarray([[vehicle_x, 0.0]]))[0, 0]
            if vehicle_s >= center_s:
                candidate = pending_candidate
                break
    if candidate is None:
        raise RuntimeError('parallel validation rectangle was not confirmed')

    path = build_parallel_reference_path(
        candidate,
        Pose2(0.1, 0.0, 0.0),
        turn_radius_m=1.12,
        end_straight_m=2.0,
        arc_angle_deg=45.0,
        arc_start_offset_m=0.25,
    )
    if path is None:
        raise RuntimeError('parallel reference path was not created')
    simulation = run_simulation(path)
    rectangle = candidate_rectangle_corners(candidate, 1.5, 0.7)

    fig, axes = plt.subplots(3, 1, figsize=(12, 12), sharex=True, sharey=True)
    phases = (
        ('1. Forward after passing P0\npreview at front end -> hold 1s',
         simulation['forward'], 'royalblue', simulation['front_stop']),
        ('2. Reverse on the same S path\npreview at rear end -> hold 1s',
         simulation['reverse'], 'crimson', simulation['rear_stop']),
        ('3. Forward on the same path\npreview at end -> hold 1s -> log stop',
         simulation['return'], 'darkorange', simulation['final_stop']),
    )
    for axis, (title, points, color, stop_pose) in zip(axes, phases):
        context(axis, scene, rectangle, path)
        arrowed(axis, points, color, title.split('\n')[0])
        axis.plot(stop_pose.x, stop_pose.y, marker='s', color='navy', ms=8,
                  label='vehicle stop (preview at end)')
        axis.set_title(title)
        axis.legend(loc='best', fontsize=7)
    for axis in axes:
        axis.set_ylabel('parking_map y [m]')
    fig.suptitle(
        'Parallel parking test: 1.5m x 0.7m slot, R=1.12m, '
        'arc origin P0+0.25m wall-parallel forward\n'
        'two 45deg arcs, 2m end lines',
        fontsize=15,
    )
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    output = args.output_dir / 'parallel_parking_simulation.png'
    fig.savefig(output, dpi=180)
    plt.close(fig)

    print('state_sequence=forward>hold1s>reverse>hold1s>'
          'same_path_forward>hold1s>parallel_parking_complete')
    print('rectangle_m=1.500x0.700')
    print('turn_radius_m=1.120')
    print('arc_start_offset_m=0.250')
    print('arc_angle_deg=45.000x2')
    print('end_straight_m=2.000x2')
    print('preview_distance_m=1.500')
    print('v_forward_mps=0.750')
    print('v_reverse_mps=-0.750')
    print('final_status=%s' % simulation['final_status'])
    print('saved=%s' % output)


if __name__ == '__main__':
    main()
