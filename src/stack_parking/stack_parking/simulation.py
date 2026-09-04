"""ROS-free repeatable parking simulations for CI and parameter regression."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from typing import Iterable

import numpy as np

from .geometry import Pose2, between, compose
from .mission import MissionConfig, MissionState, ParkingMission
from .path_planner import MinimumRadiusParkingPlanner, PlannerConfig
from .space_detector import (
    MODE_PARALLEL,
    MODE_PERPENDICULAR,
    SIDE_LEFT,
    SIDE_RIGHT,
    ParkingSpaceDetector,
    SpaceDetectorConfig,
)


def _line(a: tuple[float, float], b: tuple[float, float], step: float = 0.04) -> np.ndarray:
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    count = max(2, int(math.ceil(length / step)) + 1)
    return np.column_stack((
        np.linspace(a[0], b[0], count),
        np.linspace(a[1], b[1], count),
    ))


def _rectangle(x0: float, x1: float, y0: float, y1: float) -> np.ndarray:
    return np.vstack((
        _line((x0, y0), (x1, y0)),
        _line((x1, y0), (x1, y1)),
        _line((x1, y1), (x0, y1)),
        _line((x0, y1), (x0, y0)),
    ))


def synthetic_scene(mode: str, side: str) -> np.ndarray:
    """Return endpoint geometry for a bracketed scale-vehicle parking bay."""
    if mode == MODE_PARALLEL:
        points = np.vstack((
            _rectangle(-1.45, -0.20, -1.10, -0.55),
            _rectangle(3.00, 4.25, -1.10, -0.55),
            _line((-2.5, 1.8), (5.0, 1.8), 0.10),
        ))
    elif mode == MODE_PERPENDICULAR:
        points = np.vstack((
            _rectangle(-0.90, -0.20, -2.00, -0.55),
            _rectangle(1.20, 1.90, -2.00, -0.55),
            _line((-0.20, -2.00), (1.20, -2.00)),
            _line((-2.5, 1.8), (4.0, 1.8), 0.10),
        ))
    else:
        raise ValueError(mode)
    if side == SIDE_LEFT:
        points[:, 1] *= -1.0
    return points


@dataclass(frozen=True)
class SimulationResult:
    success: bool
    mode: str
    side: str
    reason: str
    ticks: int
    plan_points: int
    approach_points: int
    reverse_points: int
    max_curvature: float
    max_preview_distance_m: float
    dynamic_stop_seen: bool
    final_state: str


def build_mission(stable_frames: int = 3, wait_s: float = 5.0) -> ParkingMission:
    detector = ParkingSpaceDetector(SpaceDetectorConfig(stable_frames=stable_frames))
    planner = MinimumRadiusParkingPlanner(PlannerConfig())
    return ParkingMission(
        detector,
        planner,
        MissionConfig(parked_wait_s=wait_s, complete_latch_s=0.4),
    )


def simulate_once(
    mode: str,
    side: str,
    seed: int = 1,
    map_noise_std_m: float = 0.0,
    dropout: float = 0.0,
    inject_dynamic: bool = False,
) -> SimulationResult:
    rng = np.random.default_rng(seed)
    points = synthetic_scene(mode, side)
    if dropout > 0.0:
        points = points[rng.random(len(points)) >= dropout]
    if map_noise_std_m > 0.0:
        points = points + rng.normal(0.0, map_noise_std_m, points.shape)

    mission = build_mission()
    pose = Pose2(-1.50, 0.0, 0.0)
    mission.trigger(mode, side, pose)
    for _ in range(mission.detector.config.stable_frames + 2):
        mission.observe_map(points, pose)
        if mission.plan is not None:
            break
    if mission.plan is None:
        return SimulationResult(
            False, mode, side, 'no_plan:' + mission.last_plan_error, 0, 0, 0, 0,
            0.0, 0.0, False, mission.state.value)

    max_curvature = max(abs(point.curvature) for point in mission.plan.reverse_path)
    max_preview = 0.0
    dynamic_stop_seen = False
    dynamic_injected = False
    now_s = 0.0
    rear_clearance = None

    for tick in range(1, 2000):
        if (
            inject_dynamic
            and not dynamic_injected
            and mission.state == MissionState.REVERSE
            and mission.progress >= min(8, len(mission.current_path) // 3)
        ):
            blocker = mission.current_path[min(
                mission.progress + 1, len(mission.current_path) - 1)]
            observed = np.asarray([[blocker.x, blocker.y]])
            mission.observe_dynamic(observed)
            mission.observe_dynamic(observed)
            dynamic_injected = True

        output = mission.tick(
            pose, now_s, rear_clearance_m=rear_clearance,
            vehicle_speed_mps=0.0, localization_valid=True)
        if output.reference_local is not None:
            max_preview = max(max_preview, math.hypot(
                output.reference_local.x, output.reference_local.y))
        if output.path_blocked:
            dynamic_stop_seen = dynamic_stop_seen or abs(output.v_suggest_mps) < 1.0e-9
            # A moving object leaves after one control tick.
            mission.observe_dynamic(np.empty((0, 2)))

        if output.done:
            return SimulationResult(
                True, mode, side, 'complete', tick,
                len(mission.plan.full_entry_path),
                len(mission.plan.approach_path),
                len(mission.plan.reverse_path),
                max_curvature, max_preview, dynamic_stop_seen,
                output.state.value)

        if output.v_suggest_mps != 0.0 and mission.current_path:
            next_index = min(mission.progress + 1, len(mission.current_path) - 1)
            target = mission.current_path[next_index]
            pose = Pose2(target.x, target.y, target.yaw)

        if mission.state == MissionState.REVERSE and mission.space is not None:
            rear_sensor_map = compose(pose, Pose2(-0.110354, 0.002473, 0.0))
            sensor_lane = between(mission.space.lane_pose_map, rear_sensor_map)
            if mode == MODE_PARALLEL:
                geometric_clearance = sensor_lane.x - mission.space.start_x_lane
            else:
                side_sign = 1.0 if side == SIDE_LEFT else -1.0
                geometric_clearance = (
                    mission.space.back_wall_distance_m
                    - side_sign * sensor_lane.y
                )
            rear_clearance = (
                geometric_clearance if 0.0 < geometric_clearance < 0.60 else None)
        else:
            rear_clearance = None
        now_s += 0.1

    return SimulationResult(
        False, mode, side, 'timeout', 2000,
        len(mission.plan.full_entry_path),
        len(mission.plan.approach_path),
        len(mission.plan.reverse_path),
        max_curvature, max_preview, dynamic_stop_seen,
        mission.state.value)


def run_batch(runs: int, seed: int, noise: float, dropout: float) -> dict:
    results: list[SimulationResult] = []
    scenarios: Iterable[tuple[str, str]] = (
        (MODE_PARALLEL, SIDE_RIGHT),
        (MODE_PARALLEL, SIDE_LEFT),
        (MODE_PERPENDICULAR, SIDE_RIGHT),
        (MODE_PERPENDICULAR, SIDE_LEFT),
    )
    for run in range(runs):
        for scenario_index, (mode, side) in enumerate(scenarios):
            results.append(simulate_once(
                mode,
                side,
                seed=seed + run * 17 + scenario_index,
                map_noise_std_m=noise,
                dropout=dropout,
                inject_dynamic=(run == 0),
            ))
    successes = sum(result.success for result in results)
    return {
        'runs_per_scenario': runs,
        'scenario_count': len(results),
        'successes': successes,
        'failures': len(results) - successes,
        'success_rate': successes / max(1, len(results)),
        'noise_std_m': noise,
        'dropout': dropout,
        'max_curvature': max((result.max_curvature for result in results), default=0.0),
        'curvature_limit': 1.0 / PlannerConfig().min_turn_radius_m,
        'max_preview_distance_m': max(
            (result.max_preview_distance_m for result in results), default=0.0),
        'dynamic_stop_checks': sum(result.dynamic_stop_seen for result in results),
        'failures_detail': [asdict(result) for result in results if not result.success],
    }


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=int, default=25, help='runs per mode/side scenario')
    parser.add_argument('--seed', type=int, default=20260823)
    parser.add_argument('--noise', type=float, default=0.012, help='map point stddev [m]')
    parser.add_argument('--dropout', type=float, default=0.08, help='random endpoint dropout 0..1')
    options = parser.parse_args(args)
    report = run_batch(options.runs, options.seed, options.noise, options.dropout)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['failures'] == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
