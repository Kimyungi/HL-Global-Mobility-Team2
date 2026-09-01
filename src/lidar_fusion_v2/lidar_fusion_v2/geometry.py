"""Pure geometry for the v2 unifier. No dependency on the legacy fusion package."""

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


def wrap_deg(angle: float) -> float:
    """Normalize degrees to [-180, 180)."""
    return (float(angle) + 180.0) % 360.0 - 180.0


def angle_in_sector(angle_deg: np.ndarray, minimum_deg: float,
                    maximum_deg: float) -> np.ndarray:
    """Inclusive angular mask; minimum > maximum denotes a wrapped sector."""
    a = (np.asarray(angle_deg, dtype=float) + 180.0) % 360.0 - 180.0
    lo = wrap_deg(minimum_deg)
    hi = wrap_deg(maximum_deg)
    eps = 1e-7
    if lo <= hi:
        return (a >= lo - eps) & (a <= hi + eps)
    return (a >= lo - eps) | (a <= hi + eps)


@dataclass(frozen=True)
class SensorGeometry:
    sensor_id: str
    x: float
    y: float
    yaw_deg: float
    fov_min_deg: float
    fov_max_deg: float
    min_range: float = 0.15
    max_range: float = 12.0
    range_offset: float = 0.0


def scan_to_base(ranges: Sequence[float], angle_min: float,
                 angle_increment: float, geometry: SensorGeometry) -> np.ndarray:
    """Convert one LaserScan range array to an N×2 cloud in base_link."""
    r = np.asarray(ranges, dtype=float) - geometry.range_offset
    angles = angle_min + np.arange(r.size, dtype=float) * angle_increment
    angles_deg = np.rad2deg(angles)
    valid = np.isfinite(r)
    valid &= r >= geometry.min_range
    valid &= r <= geometry.max_range
    valid &= angle_in_sector(angles_deg, geometry.fov_min_deg, geometry.fov_max_deg)
    if not np.any(valid):
        return np.empty((0, 2), dtype=np.float32)

    local = np.column_stack((r[valid] * np.cos(angles[valid]),
                             r[valid] * np.sin(angles[valid])))
    yaw = math.radians(geometry.yaw_deg)
    rot = np.array(((math.cos(yaw), -math.sin(yaw)),
                    (math.sin(yaw), math.cos(yaw))))
    base = local @ rot.T
    base[:, 0] += geometry.x
    base[:, 1] += geometry.y
    return base.astype(np.float32, copy=False)


def scan_to_local(ranges: Sequence[float], angle_min: float,
                  angle_increment: float, geometry: SensorGeometry) -> np.ndarray:
    """Convert a LaserScan to valid sensor-frame points using its fixed FOV."""
    local_geometry = SensorGeometry(
        sensor_id=geometry.sensor_id,
        x=0.0, y=0.0, yaw_deg=0.0,
        fov_min_deg=geometry.fov_min_deg,
        fov_max_deg=geometry.fov_max_deg,
        min_range=geometry.min_range,
        max_range=geometry.max_range,
        range_offset=geometry.range_offset,
    )
    return scan_to_base(ranges, angle_min, angle_increment, local_geometry)


def local_to_base(points: np.ndarray, geometry: SensorGeometry) -> np.ndarray:
    """Transform already-filtered sensor-frame XY points into base_link."""
    return transform_points(
        points, geometry.x, geometry.y, math.radians(geometry.yaw_deg))


def points_to_virtual_scan(points: np.ndarray, angle_increment: float,
                           range_min: float, range_max: float,
                           angle_min: float = -math.pi,
                           angle_max: float = math.pi) -> np.ndarray:
    """Nearest-return rasterization of a base-frame cloud into one 2D scan."""
    count = int(math.ceil((angle_max - angle_min) / angle_increment))
    output = np.full(count, np.inf, dtype=np.float32)
    if points.size == 0:
        return output
    xy = np.asarray(points, dtype=float)[:, :2]
    distance = np.hypot(xy[:, 0], xy[:, 1])
    angle = np.arctan2(xy[:, 1], xy[:, 0])
    valid = np.isfinite(distance) & (distance >= range_min) & (distance <= range_max)
    index = np.floor((angle[valid] - angle_min) / angle_increment).astype(int)
    distance = distance[valid]
    inside = (index >= 0) & (index < count)
    np.minimum.at(output, index[inside], distance[inside].astype(np.float32))
    return output


def transform_points(points: np.ndarray, dx: float, dy: float,
                     dyaw: float) -> np.ndarray:
    """Apply a planar correction, used by wall calibration and its tests."""
    p = np.asarray(points, dtype=float)
    c, s = math.cos(dyaw), math.sin(dyaw)
    out = p @ np.array(((c, -s), (s, c))).T
    out[:, 0] += dx
    out[:, 1] += dy
    return out
