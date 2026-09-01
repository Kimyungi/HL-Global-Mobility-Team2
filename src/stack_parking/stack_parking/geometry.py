"""ROS-independent SE(2) and parking-path geometry helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

import numpy as np


def wrap_angle(angle: float) -> float:
    """Return *angle* in [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Pose2:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class PathPoint:
    """A vehicle pose and the steering curvature used to reach it.

    ``gear`` is +1 for forward and -1 for reverse.  Curvature is the
    kinematic-bicycle steering curvature (yaw_rate = velocity * curvature),
    so the sign remains valid when an entry path is replayed in reverse order
    during pull-out.
    """

    x: float
    y: float
    yaw: float
    curvature: float
    gear: int

    @property
    def pose(self) -> Pose2:
        return Pose2(self.x, self.y, self.yaw)


def compose(a: Pose2, b: Pose2) -> Pose2:
    """Compose map<-a and a<-b transforms."""
    c = math.cos(a.yaw)
    s = math.sin(a.yaw)
    return Pose2(
        a.x + c * b.x - s * b.y,
        a.y + s * b.x + c * b.y,
        wrap_angle(a.yaw + b.yaw),
    )


def inverse(pose: Pose2) -> Pose2:
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    return Pose2(
        -c * pose.x - s * pose.y,
        s * pose.x - c * pose.y,
        wrap_angle(-pose.yaw),
    )


def between(a: Pose2, b: Pose2) -> Pose2:
    """Return the pose of *b* expressed in frame *a*."""
    return compose(inverse(a), b)


def transform_points(points: np.ndarray, pose: Pose2) -> np.ndarray:
    """Transform an Nx2 array by *pose*."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    out = np.empty_like(pts[:, :2], dtype=np.float64)
    out[:, 0] = c * pts[:, 0] - s * pts[:, 1] + pose.x
    out[:, 1] = s * pts[:, 0] + c * pts[:, 1] + pose.y
    return out


def points_in_frame(points: np.ndarray, frame_pose: Pose2) -> np.ndarray:
    return transform_points(points, inverse(frame_pose))


def pose_in_frame(pose: Pose2, frame_pose: Pose2) -> Pose2:
    return between(frame_pose, pose)


def path_to_frame(path: Sequence[PathPoint], frame_pose: Pose2) -> list[PathPoint]:
    result: list[PathPoint] = []
    for point in path:
        local = between(frame_pose, point.pose)
        result.append(PathPoint(
            local.x, local.y, local.yaw, point.curvature, point.gear))
    return result


def path_from_frame(path: Sequence[PathPoint], frame_pose: Pose2) -> list[PathPoint]:
    result: list[PathPoint] = []
    for point in path:
        world = compose(frame_pose, point.pose)
        result.append(PathPoint(
            world.x, world.y, world.yaw, point.curvature, point.gear))
    return result


def advance(pose: Pose2, signed_distance: float, curvature: float) -> Pose2:
    """Integrate a constant-curvature bicycle pose over signed distance."""
    if abs(curvature) < 1.0e-9:
        return Pose2(
            pose.x + signed_distance * math.cos(pose.yaw),
            pose.y + signed_distance * math.sin(pose.yaw),
            pose.yaw,
        )
    end_yaw = pose.yaw + signed_distance * curvature
    return Pose2(
        pose.x + (math.sin(end_yaw) - math.sin(pose.yaw)) / curvature,
        pose.y + (-math.cos(end_yaw) + math.cos(pose.yaw)) / curvature,
        wrap_angle(end_yaw),
    )


def sample_motion(
    start: Pose2,
    signed_distance: float,
    curvature: float,
    step_m: float,
    gear: int,
) -> list[PathPoint]:
    """Sample one constant-curvature motion, including its end pose."""
    count = max(1, int(math.ceil(abs(signed_distance) / max(step_m, 1.0e-3))))
    result: list[PathPoint] = []
    for index in range(1, count + 1):
        pose = advance(start, signed_distance * index / count, curvature)
        result.append(PathPoint(pose.x, pose.y, pose.yaw, curvature, gear))
    return result


def sample_pose_line(
    start: Pose2,
    end: Pose2,
    step_m: float,
    gear: int,
) -> list[PathPoint]:
    """Sample a short near-straight connection used on the scan lane."""
    distance = math.hypot(end.x - start.x, end.y - start.y)
    count = max(1, int(math.ceil(distance / max(step_m, 1.0e-3))))
    yaw_delta = wrap_angle(end.yaw - start.yaw)
    result: list[PathPoint] = []
    for index in range(1, count + 1):
        t = index / count
        result.append(PathPoint(
            start.x + t * (end.x - start.x),
            start.y + t * (end.y - start.y),
            wrap_angle(start.yaw + t * yaw_delta),
            0.0,
            gear,
        ))
    return result


def cumulative_lengths(path: Sequence[PathPoint]) -> list[float]:
    if not path:
        return []
    lengths = [0.0]
    for previous, current in zip(path, path[1:]):
        lengths.append(lengths[-1] + math.hypot(
            current.x - previous.x, current.y - previous.y))
    return lengths


def closest_path_index(
    path: Sequence[PathPoint],
    pose: Pose2,
    previous_index: int = 0,
    backward_window: int = 3,
) -> int:
    if not path:
        return 0
    first = max(0, int(previous_index) - backward_window)
    # Progress is monotonic; looking behind the last accepted point is useful
    # for noisy ICP, but accepting a lower index is not.
    distances = [
        (path[i].x - pose.x) ** 2 + (path[i].y - pose.y) ** 2
        for i in range(first, len(path))
    ]
    candidate = first + int(np.argmin(distances))
    return max(int(previous_index), candidate)


def preview_index(
    path: Sequence[PathPoint],
    cumulative: Sequence[float],
    progress_index: int,
    preview_distance_m: float,
) -> int:
    if not path:
        return 0
    target_length = cumulative[min(progress_index, len(cumulative) - 1)] + max(
        0.0, preview_distance_m)
    index = int(np.searchsorted(np.asarray(cumulative), target_length, side='left'))
    return min(index, len(path) - 1)


def preview_path_point(
    path: Sequence[PathPoint],
    cumulative: Sequence[float],
    progress_index: int,
    preview_distance_m: float,
) -> PathPoint:
    """Interpolate a preview at the requested arc distance (not next sample)."""
    if not path:
        raise ValueError('preview requested for an empty path')
    progress_index = min(max(0, progress_index), len(path) - 1)
    target = min(
        cumulative[-1],
        cumulative[progress_index] + max(0.0, preview_distance_m),
    )
    upper = int(np.searchsorted(np.asarray(cumulative), target, side='left'))
    upper = min(max(progress_index, upper), len(path) - 1)
    if upper == 0 or cumulative[upper] <= cumulative[upper - 1] + 1.0e-9:
        return path[upper]
    lower = upper - 1
    alpha = (target - cumulative[lower]) / (cumulative[upper] - cumulative[lower])
    yaw_delta = wrap_angle(path[upper].yaw - path[lower].yaw)
    return PathPoint(
        path[lower].x + alpha * (path[upper].x - path[lower].x),
        path[lower].y + alpha * (path[upper].y - path[lower].y),
        wrap_angle(path[lower].yaw + alpha * yaw_delta),
        path[lower].curvature + alpha * (
            path[upper].curvature - path[lower].curvature),
        path[upper].gear,
    )


def local_reference(point: PathPoint, vehicle_pose: Pose2) -> PathPoint:
    """Convert a map-frame path point to the current vehicle frame."""
    local = between(vehicle_pose, point.pose)
    return PathPoint(local.x, local.y, local.yaw, point.curvature, point.gear)


def deduplicate_path(path: Iterable[PathPoint], tolerance_m: float = 1.0e-4) -> list[PathPoint]:
    result: list[PathPoint] = []
    for point in path:
        if result and math.hypot(point.x - result[-1].x, point.y - result[-1].y) < tolerance_m:
            result[-1] = point
        else:
            result.append(point)
    return result
