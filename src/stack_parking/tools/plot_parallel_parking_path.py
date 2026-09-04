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
        preview_distance_m=1.0,
        forward_speed_mps=0.75,
        reverse_speed_mps=0.75,
        sample_step_m=0.05,
        entry_straight_m=2.0,
        entry_inner_straight_m=1.0,
        opposite_straight_m=1.0,
        reference_reverse_end_trim_m=1.0,
    ))
    full_forward, _ = parallel_controller_paths(path)
    p0_index = min(range(len(full_forward)), key=lambda index: math.hypot(
        full_forward[index].x - path.p0_map[0],
        full_forward[index].y - path.p0_map[1],
    ))
    start_pose = pose(full_forward[p0_index])
    if not controller.start(path, start_pose, 0.0):
        raise RuntimeError('parallel controller rejected the S path')

    def drive_to_hold(reference_points, hold_state, now_s):
        samples = []
        for point in reference_points[controller.progress:]:
            update_time_s = now_s
            output = controller.update(pose(point), update_time_s)
            samples.append((point.x, point.y))
            now_s += 0.05
            if output.state == hold_state:
                return (
                    np.asarray(samples), pose(point), update_time_s, now_s)
        raise RuntimeError('%s was not reached' % hold_state.value)

    def release_hold(stop_pose, hold_started_s, next_state):
        output = controller.update(stop_pose, hold_started_s + 0.99)
        if output.state == next_state:
            raise RuntimeError('hold ended before one second')
        output = controller.update(stop_pose, hold_started_s + 1.0)
        if output.state != next_state:
            raise RuntimeError(
                'controller entered %s instead of %s'
                % (output.state.value, next_state.value))
        return hold_started_s + 1.0

    phases = []
    now_s = 0.0
    samples, stop_pose, hold_started_s, now_s = drive_to_hold(
        controller.initial_reference_forward_path,
        ParallelControlState.INITIAL_FORWARD_HOLD,
        now_s,
    )
    phases.append(('initial_forward', samples, stop_pose))
    now_s = release_hold(
        stop_pose, hold_started_s,
        ParallelControlState.SINGLE_ARC_REVERSE)

    samples, stop_pose, hold_started_s, now_s = drive_to_hold(
        controller.single_arc_reverse_path,
        ParallelControlState.SINGLE_ARC_REVERSE_HOLD,
        now_s,
    )
    phases.append(('single_arc_reverse', samples, stop_pose))
    now_s = release_hold(
        stop_pose, hold_started_s,
        ParallelControlState.OPPOSITE_ARC_FORWARD)

    samples, stop_pose, hold_started_s, now_s = drive_to_hold(
        controller.opposite_arc_forward_path,
        ParallelControlState.OPPOSITE_ARC_FORWARD_HOLD,
        now_s,
    )
    phases.append(('opposite_arc_forward', samples, stop_pose))
    now_s = release_hold(
        stop_pose, hold_started_s,
        ParallelControlState.REFERENCE_REVERSE)

    samples, stop_pose, hold_started_s, now_s = drive_to_hold(
        controller.reference_reverse_path,
        ParallelControlState.REFERENCE_REVERSE_HOLD,
        now_s,
    )
    phases.append(('reference_reverse', samples, stop_pose))
    now_s = release_hold(
        stop_pose, hold_started_s,
        ParallelControlState.REFERENCE_FORWARD)

    samples, final_stop, final_hold_started_s, now_s = drive_to_hold(
        controller.reference_forward_path,
        ParallelControlState.FINAL_HOLD,
        now_s,
    )
    phases.append(('reference_forward', samples, final_stop))

    output = controller.update(final_stop, final_hold_started_s + 0.99)
    if output.state != ParallelControlState.FINAL_HOLD:
        raise RuntimeError('final hold ended before one second')
    output = controller.update(final_stop, final_hold_started_s + 1.0)
    if (output.state != ParallelControlState.STOPPED
            or output.status != 'parallel_parking_complete'):
        raise RuntimeError('final hold did not complete logging state')

    return {
        'controller': controller,
        'phases': phases,
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


def context(ax, scene, rectangle, path, active_reference):
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
            color='0.65', ls=':', lw=1.5, label='R2.0m S reference path')
    ax.plot(path.p0_map[0], path.p0_map[1], 'mx', ms=10, mew=2,
            label='P0: wall-edge midpoint')
    ax.plot(path.arc_origin_map[0], path.arc_origin_map[1], 'g+', ms=12,
            mew=2, label='arc origin: P0 + 0.5m forward + 0.25m CW')
    active_xy = np.asarray([
        [point.x, point.y] for point in active_reference])
    ax.plot(active_xy[:, 0], active_xy[:, 1], color='seagreen', ls='--',
            lw=1.5, label='active reference for this phase')
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
        turn_radius_m=2.0,
        end_straight_m=1.5,
        arc_angle_deg=50.0,
        arc_start_offset_m=0.5,
        arc_clockwise_offset_m=0.25,
    )
    if path is None:
        raise RuntimeError('parallel reference path was not created')
    simulation = run_simulation(path)
    rectangle = candidate_rectangle_corners(candidate, 1.5, 0.7)

    controller = simulation['controller']
    fig, axes = plt.subplots(5, 1, figsize=(13, 20), sharex=True, sharey=True)
    plot_phases = (
        ('1. Initial full-S forward\npreview at end -> hold 1s',
         simulation['phases'][0], 'royalblue',
         controller.initial_reference_forward_path),
        ('2. Reverse: 2.0m outer - one arc - 1.0m inner\n'
         'preview at end -> hold 1s',
         simulation['phases'][1], 'crimson',
         controller.single_arc_reverse_path),
        ('3. Forward: distinct opposite arc\n'
         '1.0m line - one arc - 1.0m line -> hold 1s',
         simulation['phases'][2], 'darkorange',
         controller.opposite_arc_forward_path),
        ('4. Reference-S reverse\nfinal straight shortened 1.0m -> hold 1s',
         simulation['phases'][3], 'purple',
         controller.reference_reverse_path),
        ('5. Full reference-S forward\n'
         'preview at end -> hold 1s -> log stop',
         simulation['phases'][4], 'teal',
         controller.reference_forward_path),
    )
    for axis, (title, phase, color, active_path) in zip(axes, plot_phases):
        _, points, stop_pose = phase
        context(axis, scene, rectangle, path, active_path)
        arrowed(axis, points, color, title.split('\n')[0])
        axis.plot(stop_pose.x, stop_pose.y, marker='s', color='navy', ms=8,
                  label='vehicle stop (preview at end)')
        axis.set_title(title)
        axis.legend(loc='best', fontsize=7)
    for axis in axes:
        axis.set_ylabel('parking_map y [m]')
    fig.suptitle(
        'Parallel parking test: 1.5m x 0.7m slot, R=2.0m, 50deg arcs, '
        'origin P0+0.5m forward+0.25m clockwise\n'
        'full-S forward -> two distinct single arcs -> '
        'full-S reverse/forward',
        fontsize=15,
    )
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    output = args.output_dir / 'parallel_parking_simulation.png'
    fig.savefig(output, dpi=180)
    plt.close(fig)

    print('state_sequence=full_s_forward>hold1s>'
          'single_arc_reverse>hold1s>opposite_arc_forward>hold1s>'
          'full_s_reverse>hold1s>full_s_forward>hold1s>'
          'parallel_parking_complete')
    print('rectangle_m=1.500x0.700')
    print('turn_radius_m=2.000')
    print('arc_start_offset_m=0.500')
    print('arc_clockwise_offset_m=0.250')
    print('arc_angle_deg=50.000x2')
    print('s_end_straight_m=1.500x2')
    print('entry_outer_straight_m=2.000')
    print('entry_inner_straight_m=1.000')
    print('opposite_straight_m=1.000x2')
    print('phase4_end_trim_m=1.000')
    print('preview_distance_m=1.000')
    print('v_forward_mps=0.750')
    print('v_reverse_mps=-0.750')
    print('final_status=%s' % simulation['final_status'])
    print('saved=%s' % output)


if __name__ == '__main__':
    main()
