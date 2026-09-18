"""Offline-testable two-reference reverse parking; no ROS/CAN side effects.

All paths and poses must share a metric frame. Scans are calibrated endpoints
in current base_link; their origin is the actual LiDAR mount, NOT base_link.
The one-free-candidate guarantee permits elimination only after an obstruction
on the other candidate is observed repeatedly. Missing data is never evidence.
"""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .geometry import PathPoint, Pose2, local_reference, wrap_angle


@dataclass(frozen=True)
class Config:
    selection_after_turn: float = 1.5
    front: float = 0.760
    rear: float = 0.090
    width: float = 0.620
    margin: float = 0.08
    min_radius: float = 1.15
    # CSV is a reference for downstream quintic/MPC control, not its executed
    # trajectory. Report this geometric limit; enforce only when opted in.
    enforce_min_radius: bool = False
    stop_speed: float = 0.035
    stop_hold: float = 0.5
    sensor_timeout: float = 0.35
    feedback_timeout: float = 0.25
    confirm_frames: int = 3
    min_scan_points: int = 12
    observed_fraction: float = 0.7
    ray_tolerance: float = math.radians(1.0)
    rear_stop: float = 0.50  # distance from rear LiDAR, not rear bumper
    wall_min_points: int = 6
    wall_min_width: float = 0.25
    wall_residual: float = 0.025
    reverse_speed: float = 1.0
    dock_speed: float = 1.0
    forward_speed: float = 1.0
    preview: float = 1.2
    dock_remaining: float = 2.0
    start_tolerance: float = 0.30
    yaw_tolerance: float = math.radians(20)
    tracking_error: float = 0.30
    tracking_yaw_error: float = math.radians(45)


@dataclass(frozen=True)
class Candidate:
    name: str
    path: tuple[PathPoint, ...]
    s: np.ndarray
    turn_end_s: float | None = None

    def __post_init__(self):
        if (len(self.path) < 2 or len(self.path) != len(self.s)
            or not np.isfinite(self.s).all() or abs(self.s[0]) > 1e-9
            or np.any(np.diff(self.s) <= 0)
            or any(p.gear != -1 or not all(math.isfinite(v) for v in
                   (p.x, p.y, p.yaw, p.curvature)) for p in self.path)):
            raise ValueError('Candidate must be finite, reverse-only and ordered by increasing distance')

    @property
    def minimum_radius(self) -> float:
        k = max(abs(p.curvature) for p in self.path)
        return math.inf if k < 1e-9 else 1.0 / k


def csv_turn_end(rows, station):
    """First straight sample following the CSV's final curved segment."""
    curved = [i for i, r in enumerate(rows)
              if r.get('segment_type', 'STRAIGHT') != 'STRAIGHT']
    return float(station[min(curved[-1]+1, len(rows)-1)]) if curved else None


def selection_path(candidate, cfg):
    """Inspection prefix only; keep the full candidate for actual driving."""
    if candidate.turn_end_s is None:
        return candidate.path
    end = min(candidate.s[-1], candidate.turn_end_s + cfg.selection_after_turn)
    count = int(np.searchsorted(candidate.s, end, side='right'))
    points = candidate.path[:count]
    if count < len(candidate.path) and end > candidate.s[count-1]:
        a, b = candidate.path[count-1:count+1]
        t = (end-candidate.s[count-1])/(candidate.s[count]-candidate.s[count-1])
        points += (PathPoint(a.x+t*(b.x-a.x), a.y+t*(b.y-a.y),
                            wrap_angle(a.yaw+t*wrap_angle(b.yaw-a.yaw)),
                            a.curvature+t*(b.curvature-a.curvature), -1),)
    return points


def load_reverse_csv(filename: str | Path) -> Candidate:
    """CSV yaw denotes travel tangent. Reverse BODY yaw is tangent + pi.

    Steering curvature is -d(tangent)/ds because signed velocity is negative.
    Densification is for footprint evaluation, not a change to the CSV route.
    """
    with open(filename, encoding='utf-8-sig', newline='') as stream:
        rows = list(csv.DictReader(stream))
    xy = np.array([[float(r['east_m']), float(r['north_m'])] for r in rows])
    tangent = np.unwrap([float(r['yaw_rad']) for r in rows])
    if len(rows) < 3 or not np.isfinite(xy).all() or not np.isfinite(tangent).all():
        raise ValueError('Need at least three finite waypoint poses')
    s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    if np.any(np.diff(s) <= 1e-6):
        raise ValueError('Duplicate or zero-length waypoint segment')
    curvature = -np.gradient(tangent, s)
    dense_s = np.unique(np.r_[s, np.arange(0., s[-1], 0.05)])
    x, y = (np.interp(dense_s, s, xy[:, i]) for i in (0, 1))
    yaw = np.interp(dense_s, s, tangent) + math.pi
    k = np.interp(dense_s, s, curvature)
    path = tuple(PathPoint(float(a), float(b), wrap_angle(c), float(d), -1)
                 for a, b, c, d in zip(x, y, yaw, k))
    return Candidate(Path(filename).stem, path, dense_s, csv_turn_end(rows, s))


@dataclass(frozen=True)
class Scan:
    stamp: float  # acquisition time on same clock as tick; do not use receive time
    points: np.ndarray  # finite, corrected, FOV-filtered endpoints in base_link
    origin: tuple[float, float]  # actual sensor origin in base_link


def scan_from_ranges(stamp, ranges, angle_min, angle_increment, range_min,
                     range_max, mount: Pose2, range_offset, fov_min, fov_max) -> Scan:
    """Angles/FOV in radians, driver scan frame; correction applied once only.

    Pass fixed_geometry.yaml values converted to radians. NaN/Inf/out-of-range
    returns are unknown, not artificial free rays. The caller must self-filter
    body returns if its driver/FOV does not already exclude them.
    """
    raw = np.asarray(ranges, dtype=float)
    angle = angle_min + np.arange(len(raw)) * angle_increment
    valid = (np.isfinite(raw) & (raw >= range_min) & (raw <= range_max)
             & (angle >= fov_min) & (angle <= fov_max) & (raw > range_offset))
    radius = raw[valid] - range_offset
    theta = angle[valid] + mount.yaw
    pts = np.column_stack((mount.x + radius * np.cos(theta),
                           mount.y + radius * np.sin(theta)))
    return Scan(stamp, pts, (mount.x, mount.y))


def fresh(stamp: float, now: float, timeout: float) -> bool:
    return math.isfinite(stamp) and 0 <= now - stamp <= timeout


def healthy(scan: Scan | None, now: float, cfg: Config) -> bool:
    return (scan is not None and fresh(scan.stamp, now, cfg.sensor_timeout)
            and scan.points.ndim == 2 and scan.points.shape[1] == 2
            and len(scan.points) >= cfg.min_scan_points
            and np.isfinite(scan.points).all() and np.isfinite(scan.origin).all())


def footprint_hits(point, pose, scan, cfg):
    local = local_reference(point, pose)
    c, s = math.cos(local.yaw), math.sin(local.yaw)
    delta = scan.points - [local.x, local.y]
    x = c * delta[:, 0] + s * delta[:, 1]
    y = -s * delta[:, 0] + c * delta[:, 1]
    return ((x >= -cfg.rear - cfg.margin)
            & (x <= cfg.front + cfg.margin)
            & (abs(y) <= cfg.width / 2 + cfg.margin))


def inspect_candidate(candidate: Candidate, pose: Pose2, scan: Scan,
                      cfg: Config, first=0, last=None) -> tuple[bool, float]:
    """Return measured footprint collision and ray-observed sample fraction.

    No parking-bay corners needed. Use oriented vehicle rectangles along path,
    not distance to its centreline. An endpoint farther along a measured ray
    supplies positive clearance evidence; absent angular bins supply none.
    """
    hits = False
    targets = []
    for p in selection_path(candidate, cfg)[first:last]:
        local = local_reference(p, pose)
        c, s = math.cos(local.yaw), math.sin(local.yaw)
        hits |= bool(np.any(footprint_hits(p, pose, scan, cfg)))
        for longitudinal in (-cfg.rear, 0., cfg.front):
            for lateral in (-cfg.width / 2, 0., cfg.width / 2):
                targets.append((local.x + c * longitudinal - s * lateral,
                                local.y + s * longitudinal + c * lateral))
    if not targets:
        return hits, 0.
    rays = scan.points - scan.origin
    ray_angle = np.arctan2(rays[:, 1], rays[:, 0])
    ray_distance = np.linalg.norm(rays, axis=1)
    observed = 0
    for target in np.asarray(targets) - scan.origin:
        angle = math.atan2(target[1], target[0])
        diff = abs((ray_angle - angle + math.pi) % (2 * math.pi) - math.pi)
        nearest = int(np.argmin(diff))
        observed += (diff[nearest] <= cfg.ray_tolerance
                     and ray_distance[nearest] >= np.linalg.norm(target) + cfg.margin)
    return hits, observed / len(targets)


def rear_observation(scan: Scan, cfg: Config) -> tuple[float, float | None]:
    """Nearest rear obstruction + supported transverse wall clearance.

    Both measured longitudinally from rear LiDAR. A cone/isolated return can
    request stop but cannot declare success. Fit x=a*y+b to rear-facing support.
    """
    pts = scan.points - scan.origin
    pts = pts[(pts[:, 0] < 0) & (abs(pts[:, 1]) <= cfg.width / 2 + cfg.margin)]
    if not len(pts):
        return math.inf, None
    nearest = float(np.min(-pts[:, 0]))
    seed = float(np.percentile(-pts[:, 0], 30))
    wall = pts[abs(-pts[:, 0] - seed) <= 0.08]
    if len(wall) < cfg.wall_min_points or np.ptp(wall[:, 1]) < cfg.wall_min_width:
        return nearest, None
    a, b = np.linalg.lstsq(np.c_[wall[:, 1], np.ones(len(wall))], wall[:, 0], rcond=None)[0]
    rms = np.sqrt(np.mean((wall[:, 0] - a * wall[:, 1] - b) ** 2))
    if abs(a) > math.tan(math.radians(20)) or rms > cfg.wall_residual or b >= 0:
        return nearest, None
    return nearest, float(-b)


@dataclass(frozen=True)
class Output:
    phase: str
    request_stop: bool
    selected: str | None
    v_suggest: float
    reference: PathPoint | None
    parking_success: bool
    reason: str
    warnings: tuple[str, ...] = ()
    # Intentionally NOT ParkingStatus.done: current MGM would resume driving.


class TwoReferenceParking:
    def __init__(self, candidates: tuple[Candidate, Candidate], cfg=Config()):
        if len(candidates) != 2 or candidates[0].name == candidates[1].name:
            raise ValueError('Exactly two uniquely named candidates required')
        self.candidates, self.cfg = candidates, cfg
        self.warnings = tuple(
            f'{p.name}: reference_radius={p.minimum_radius:.3f}m '
            f'< configured_radius={cfg.min_radius:.3f}m; tracking_requires_validation'
            for p in candidates if p.minimum_radius < cfg.min_radius)
        self.phase = 'IDLE'
        self.selected = None
        self.index = 0
        self.stopped_since = None
        self.vote = None
        self.votes = 0
        self.last_left = self.last_rear = -math.inf
        self.wall_votes = 0
        self.reason = 'idle'

    def trigger(self):
        # No automatic rearm after success or failure. New object = explicit reset.
        if self.phase == 'IDLE':
            self.phase, self.reason = 'STOPPING', 't_parking_requested'

    def _out(self, speed=0., reference=None):
        return Output(self.phase, self.phase != 'IDLE' and speed == 0.,
                      None if self.selected is None else self.candidates[self.selected].name,
                      speed, reference, self.phase == 'SUCCESS', self.reason, self.warnings)

    def tick(self, now: float, pose: Pose2, pose_stamp: float, speed: float,
             speed_stamp: float, left: Scan | None, rear: Scan | None,
             *, parking_owned: bool, estop=False) -> Output:
        """parking_owned is MGM acknowledgment, never guessed from trigger.

        request_stop must be consumed by MGM BEFORE space_found exists. This
        core does not publish to /adas/target_ref and is not wired to the car.
        """
        cfg = self.cfg
        if self.phase in ('IDLE', 'SUCCESS', 'FAULT'):
            return self._out()
        if estop:
            return self._out()
        if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw, speed)):
            return self._out()
        if self.selected is None and (left is None or not len(left.points)):
            return self._out()
        if abs(speed) <= cfg.stop_speed:
            if self.stopped_since is None:
                self.stopped_since = now
        else:
            self.stopped_since = None
        stationary = self.stopped_since is not None and now - self.stopped_since >= cfg.stop_hold
        if self.phase == 'END_STOP':
            if not fresh(speed_stamp, now, cfg.feedback_timeout):
                self.stopped_since = None
            elif stationary:
                self.phase, self.reason = 'SUCCESS', 'path_end_and_stationary'
            return self._out()
        if self.phase == 'STOPPING':
            if not stationary:
                return self._out()
            self.phase = 'SCANNING'
        if self.phase == 'SCANNING':
            if not stationary:
                self.phase, self.votes = 'STOPPING', 0
                return self._out()
            if left.stamp <= self.last_left:
                return self._out()
            self.last_left = left.stamp
            findings = [inspect_candidate(p, pose, left, cfg) for p in self.candidates]
            feasible = [not cfg.enforce_min_radius or p.minimum_radius >= cfg.min_radius
                        for p in self.candidates]
            allowed = []
            for i, candidate in enumerate(self.candidates):
                p0 = candidate.path[0]
                aligned = (math.hypot(pose.x - p0.x, pose.y - p0.y) <= cfg.start_tolerance
                           and abs(wrap_angle(pose.yaw - p0.yaw)) <= cfg.yaw_tolerance)
                blocked, observed = findings[i]
                # Explicit user assumption: at least one candidate is unobstructed.
                evidence = findings[1-i][0] or observed >= cfg.observed_fraction
                if feasible[i] and aligned and not blocked and evidence:
                    allowed.append(i)
            choice = max(allowed, key=lambda i: (findings[i][1], -i)) if allowed else None
            self.reason = ('candidate_unresolved' if any(feasible) else 'csv_curvature_exceeds_vehicle_limit')
            self.votes = self.votes + 1 if choice is not None and choice == self.vote else int(choice is not None)
            self.vote = choice
            if choice is not None and self.votes >= cfg.confirm_frames:
                self.selected = choice
                self.phase, self.reason = 'REVERSE', 'candidate_latched'
                self.stopped_since = None
            return self._out()  # deliberate zero-speed handoff tick
        candidate = self.candidates[self.selected]
        # Bounded monotonic progress avoids jumping to a spatially nearby later leg.
        bound = int(np.searchsorted(candidate.s, candidate.s[self.index] + 1.5, side='right'))
        search = candidate.path[self.index:max(self.index + 1, bound)]
        self.index += int(np.argmin([(p.x-pose.x)**2 + (p.y-pose.y)**2 for p in search]))
        remaining = candidate.s[-1] - candidate.s[self.index]
        nearest, wall = rear_observation(rear, cfg) if rear is not None and len(rear.points) else (math.inf, None)
        docking = remaining <= cfg.dock_remaining and abs(candidate.path[self.index].curvature) <= 0.05
        if self.phase == 'WALL_STOP':
            if rear is not None and rear.stamp > self.last_rear:
                self.last_rear = rear.stamp
                self.wall_votes = self.wall_votes + 1 if wall is not None and wall <= cfg.rear_stop else 0
            if stationary and self.wall_votes >= cfg.confirm_frames:
                self.phase, self.reason = 'SUCCESS', 'rear_wall_0_50m_and_stationary'
            return self._out()
        if docking and wall is not None and wall <= cfg.rear_stop:
            self.phase, self.reason = 'WALL_STOP', 'rear_wall_stop'
            self.last_rear, self.wall_votes = rear.stamp, 1
            self.stopped_since = None
            return self._out()
        if remaining <= 0.01:
            self.phase, self.reason = 'END_STOP', 'path_end_stop'
            self.stopped_since = None
            return self._out()
        preview = min(int(np.searchsorted(candidate.s, candidate.s[self.index] + cfg.preview)), len(candidate.path)-1)
        self.reason = 'reverse_docking' if docking else 'reverse_tracking'
        # Reverse references use a virtual origin 0.5 m behind the measured
        # vehicle, along its own x axis. Shift in the shared metric frame first,
        # then rotate/translate the reference into that virtual local frame.
        reference_pose = Pose2(pose.x - 0.5 * math.cos(pose.yaw),
                               pose.y - 0.5 * math.sin(pose.yaw), pose.yaw)
        return self._out(-cfg.dock_speed if docking else -cfg.reverse_speed,
                         local_reference(candidate.path[preview], reference_pose))
