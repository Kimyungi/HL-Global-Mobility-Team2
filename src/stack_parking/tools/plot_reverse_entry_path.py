#!/usr/bin/env python3
"""Simulate and plot the two-stage wall-gap parking reference paths."""

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

from stack_parking.geometry import PathPoint, Pose2  # noqa: E402
from stack_parking.reference_path import build_reference_path  # noqa: E402
from stack_parking.wall_gap_controller import (  # noqa: E402
    ControlState,
    WallGapControlConfig,
    WallGapController,
)
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


def _pose(point: PathPoint) -> Pose2:
    return Pose2(point.x, point.y, point.yaw)


def simulate_controller(path, start_pose: Pose2):
    controller = WallGapController(WallGapControlConfig(
        direction_change_hold_s=1.0,
        preview_distance_m=1.0,
        forward_speed_mps=0.3,
        reverse_speed_mps=0.3,
        sample_step_m=0.05,
    ))
    if not controller.start(path, start_pose, now_s=0.0):
        raise RuntimeError('controller rejected reference path')

    time_s = 0.0
    forward_trace = []
    reverse_trace = []
    exit_trace = []

    for point in controller.forward_path[controller.progress:]:
        output = controller.update(_pose(point), time_s)
        forward_trace.append((point.x, point.y))
        time_s += 0.05
        if output.state == ControlState.ALIGN_HOLD:
            align_stop = _pose(point)
            break
    else:
        raise RuntimeError('simulation never reached forward-path hold')

    # Complete the one-second direction-change hold without moving the pose.
    time_s += controller.config.direction_change_hold_s + 0.01
    output = controller.update(align_stop, time_s)
    if output.state != ControlState.REVERSE:
        raise RuntimeError('simulation did not enter reverse parking')

    for point in controller.reverse_path[controller.progress:]:
        output = controller.update(_pose(point), time_s)
        reverse_trace.append((point.x, point.y))
        time_s += 0.05
        if output.state == ControlState.PARKED_HOLD:
            parked_stop = _pose(point)
            break
    else:
        raise RuntimeError('simulation never reached reverse-path hold')

    if controller.exit_reference_path is None or not controller.exit_path:
        raise RuntimeError('mirrored forward exit path was not generated')

    # Complete the second one-second hold, then follow the newly mirrored path.
    time_s += controller.config.direction_change_hold_s + 0.01
    output = controller.update(parked_stop, time_s)
    if output.state != ControlState.FORWARD_EXIT:
        raise RuntimeError('simulation did not enter forward exit')

    for point in controller.exit_path[controller.progress:]:
        output = controller.update(_pose(point), time_s)
        exit_trace.append((point.x, point.y))
        time_s += 0.05
        if output.state == ControlState.STOPPED:
            break
    if output.state != ControlState.STOPPED:
        raise RuntimeError('simulation never reached final stop')

    return {
        'controller': controller,
        'forward_trace': np.asarray(forward_trace),
        'reverse_trace': np.asarray(reverse_trace),
        'exit_trace': np.asarray(exit_trace),
        'align_stop': align_stop,
        'parked_stop': parked_stop,
        'elapsed_s': time_s,
    }


def _add_direction_arrows(ax, points: np.ndarray, color: str, count: int = 3):
    if len(points) < 2:
        return
    choices = np.linspace(0, len(points) - 2, count, dtype=int)
    for index in choices:
        p0 = points[index]
        p1 = points[index + 1]
        ax.annotate('', xy=p1, xytext=p0, arrowprops={
            'arrowstyle': '-|>', 'color': color, 'lw': 1.6,
            'mutation_scale': 12,
        })


def _draw_context(ax, scene, wall, square, wall_cfg):
    reference = wall.to_map(np.array([[-1.5, 0.0], [4.8, 0.0]]))
    offset_a = wall.to_map(np.array([
        [-1.5, -wall_cfg.wall_line_offset_m],
        [4.8, -wall_cfg.wall_line_offset_m],
    ]))
    offset_b = wall.to_map(np.array([
        [-1.5, wall_cfg.wall_line_offset_m],
        [4.8, wall_cfg.wall_line_offset_m],
    ]))
    ax.scatter(scene[:, 0], scene[:, 1], s=8, c='0.2', alpha=0.45,
               label='LiDAR map points')
    ax.plot(reference[:, 0], reference[:, 1], color='teal', lw=1.5,
            label='locked wall')
    ax.plot(offset_a[:, 0], offset_a[:, 1], color='teal', ls='--', lw=0.8)
    ax.plot(offset_b[:, 0], offset_b[:, 1], color='teal', ls='--', lw=0.8)
    ax.plot(square[:, 0], square[:, 1], color='gold', ls='--', lw=1.5,
            label='confirmed 1m square')


def _finish_axes(ax, title: str):
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.25)
    ax.set_xlabel('parking_map x [m]')
    ax.set_ylabel('parking_map y [m]')
    ax.set_title(title)
    ax.legend(loc='best', fontsize=8)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', type=Path, default=Path.cwd())
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    scene, tangent = build_scene()
    wall_cfg = WallGapConfig(
        search_sides=(SIDE_LEFT,), initial_wall_max_angle_deg=30.0)
    detector = WallGapDetector(wall_cfg)
    detector.set_seed(Pose2(), SIDE_LEFT)

    confirmed = None
    confirm_pose = None
    for along in np.arange(0.0, 3.0, 0.1):
        vehicle_xy = along * tangent
        pose = Pose2(
            float(vehicle_xy[0]), float(vehicle_xy[1]),
            math.radians(35.0) * math.sin(float(along)))
        result = detector.update(scene, pose)
        if result is not None:
            confirmed = result
            confirm_pose = pose
            break
    if confirmed is None or confirm_pose is None:
        raise SystemExit('no candidate confirmed')

    path = build_reference_path(
        confirmed,
        confirm_pose,
        min_turn_radius_m=1.15,
        inside_straight_m=1.5,
        parallel_straight_m=1.5,
    )
    if path is None:
        raise SystemExit('reference path is degenerate')

    # Start on E so this deterministic offline simulation focuses on the
    # post-detection controller sequence rather than approach-to-path capture.
    start = np.asarray(path.straight1_map[0])
    e_map = np.asarray(path.e_map)
    wall_yaw = math.atan2(start[1] - e_map[1], start[0] - e_map[0])
    simulation = simulate_controller(
        path, Pose2(float(e_map[0]), float(e_map[1]), wall_yaw))
    controller = simulation['controller']
    exit_reference = controller.exit_reference_path
    entry_geometry = np.vstack((
        path.straight1_map,
        path.arc_map[1:],
        path.straight2_map[1:],
    ))
    exit_geometry = np.asarray([[point.x, point.y]
                                for point in controller.exit_path])
    wall = detector.reference_walls[SIDE_LEFT]
    square = candidate_square_corners(confirmed, wall_cfg)

    fig, ax = plt.subplots(figsize=(9, 7))
    _draw_context(ax, scene, wall, square, wall_cfg)
    ax.plot(controller.forward_path[0].x, controller.forward_path[0].y,
            'ko', ms=6, label='E: alignment start')
    ax.plot(simulation['forward_trace'][:, 0],
            simulation['forward_trace'][:, 1],
            color='royalblue', lw=2.8, label='forward alignment')
    _add_direction_arrows(
        ax, simulation['forward_trace'], 'royalblue', count=2)
    ax.plot(simulation['reverse_trace'][:, 0],
            simulation['reverse_trace'][:, 1],
            color='crimson', lw=2.8, label='reverse parking')
    _add_direction_arrows(ax, simulation['reverse_trace'], 'crimson', count=4)
    ax.plot(simulation['align_stop'].x, simulation['align_stop'].y,
            marker='s', color='navy', ms=8, label='1s stop: preview at S')
    ax.plot(simulation['parked_stop'].x, simulation['parked_stop'].y,
            marker='s', color='darkred', ms=8, label='1s stop: reverse end')
    ax.plot(path.p0_map[0], path.p0_map[1], 'mx', ms=10, mew=2, label='P0')
    ax.plot(path.goal_map[0], path.goal_map[1], 'r*', ms=13,
            label='parking-path end')
    _finish_axes(
        ax,
        'Reference path 1: immediate alignment, 1s hold, reverse parking\n'
        'wall-parallel and inward straights = 1.5m',
    )
    fig.tight_layout()
    entry_output = args.output_dir / 'parking_reference_path_1.png'
    fig.savefig(entry_output, dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 7))
    _draw_context(ax, scene, wall, square, wall_cfg)
    ax.plot(entry_geometry[:, 0], entry_geometry[:, 1], color='0.65',
            ls='--', lw=1.6, label='reference path 1 (entry)')
    ax.plot(exit_geometry[:, 0], exit_geometry[:, 1], color='darkorange',
            lw=3.0, label='reference path 2 (mirrored forward exit)')
    _add_direction_arrows(ax, exit_geometry, 'darkorange', count=5)
    axis = np.asarray([path.p0_map, path.goal_map])
    axis_direction = axis[1] - axis[0]
    axis_direction /= np.linalg.norm(axis_direction)
    axis_line = np.vstack((
        axis[0] - 0.6 * axis_direction,
        axis[1] + 0.6 * axis_direction,
    ))
    ax.plot(axis_line[:, 0], axis_line[:, 1], color='purple', ls=':', lw=2,
            label='mirror axis: inward straight')
    ax.plot(simulation['parked_stop'].x, simulation['parked_stop'].y,
            marker='s', color='darkred', ms=8,
            label='start forward after 1s hold')
    ax.plot(simulation['exit_trace'][-1, 0],
            simulation['exit_trace'][-1, 1],
            marker='s', color='navy', ms=8,
            label='final stop: preview at exit end')
    ax.plot(exit_reference.straight1_map[0, 0],
            exit_reference.straight1_map[0, 1],
            marker='*', color='darkorange', ms=14, label='exit-path end')
    ax.plot(path.p0_map[0], path.p0_map[1], 'mx', ms=10, mew=2, label='P0')
    _finish_axes(
        ax,
        'Reference path 2: mirror about inward straight, then forward exit',
    )
    fig.tight_layout()
    exit_output = args.output_dir / 'parking_reference_path_2_mirrored_exit.png'
    fig.savefig(exit_output, dpi=180)
    plt.close(fig)

    parallel_length = float(np.linalg.norm(
        path.straight1_map[1] - path.straight1_map[0]))
    print('state_sequence=forward_align>forward_path_end_hold>reverse_park>'
          'reverse_path_end_hold>forward_exit>stopped')
    print('parallel_straight_m=%.3f' % parallel_length)
    print('direction_change_hold_s=%.3f'
          % controller.config.direction_change_hold_s)
    print('exit_path_points=%d' % len(controller.exit_path))
    print('saved=%s' % entry_output)
    print('saved=%s' % exit_output)


if __name__ == '__main__':
    main()
