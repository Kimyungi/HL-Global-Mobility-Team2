"""Front/rear scan synchronization and parking-local motion prediction.

This module deliberately has no ROS imports so its frame and timing contracts
can be regression-tested without hardware.  ``parking_map`` starts at the
vehicle pose used by :meth:`MotionPrior.reset`; every incremental translation
is expressed in the previous ``base_link`` frame and composed in SE(2).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import enum
import math
from typing import Optional

import numpy as np

from .geometry import Pose2, compose, wrap_angle


@dataclass(frozen=True)
class StampedCloud:
    stamp_s: float
    receipt_s: float
    frame_id: str
    points: np.ndarray


@dataclass(frozen=True)
class CloudPair:
    front: StampedCloud
    rear: StampedCloud
    stamp_s: float
    skew_s: float

    @property
    def points(self) -> np.ndarray:
        return np.vstack((self.front.points, self.rear.points))


class FrontRearCloudPairer:
    """Consume each front/rear cloud at most once using closest timestamps."""

    def __init__(self, max_queue: int = 5):
        self.max_queue = max(2, int(max_queue))
        self._front: deque[StampedCloud] = deque()
        self._rear: deque[StampedCloud] = deque()
        self.pairs = 0
        self.stale_drops = 0
        self.sync_drops = 0

    def clear(self) -> None:
        self._front.clear()
        self._rear.clear()

    def push(self, sensor: str, cloud: StampedCloud) -> None:
        queue = self._front if sensor == 'front' else self._rear
        if sensor not in ('front', 'rear'):
            raise ValueError(f'unknown cloud sensor: {sensor}')
        queue.append(cloud)
        while len(queue) > self.max_queue:
            queue.popleft()
            self.sync_drops += 1

    def _drop_stale(self, queue: deque[StampedCloud], now_s: float,
                    stale_timeout_s: float) -> None:
        while queue and now_s - queue[0].receipt_s > stale_timeout_s:
            queue.popleft()
            self.stale_drops += 1

    def pop(self, now_s: float, sync_tolerance_s: float,
            stale_timeout_s: float) -> Optional[CloudPair]:
        self._drop_stale(self._front, now_s, stale_timeout_s)
        self._drop_stale(self._rear, now_s, stale_timeout_s)
        if not self._front or not self._rear:
            return None

        front = list(self._front)
        rear = list(self._rear)
        best_i = best_j = 0
        best_skew = math.inf
        for i, first in enumerate(front):
            for j, second in enumerate(rear):
                skew = abs(first.stamp_s - second.stamp_s)
                if skew < best_skew:
                    best_i, best_j, best_skew = i, j, skew

        if best_skew > max(0.0, sync_tolerance_s):
            # No queued pair can match.  Discard the chronologically oldest
            # endpoint so a future sample can form a pair with the newer side.
            if self._front[0].stamp_s <= self._rear[0].stamp_s:
                self._front.popleft()
            else:
                self._rear.popleft()
            self.sync_drops += 1
            return None

        for _ in range(best_i + 1):
            selected_front = self._front.popleft()
        for _ in range(best_j + 1):
            selected_rear = self._rear.popleft()
        self.sync_drops += best_i + best_j
        self.pairs += 1
        return CloudPair(
            selected_front,
            selected_rear,
            max(selected_front.stamp_s, selected_rear.stamp_s),
            best_skew,
        )


@dataclass
class MotionPriorConfig:
    velocity_timeout_s: float = 0.25
    imu_timeout_s: float = 0.25
    max_dt_s: float = 0.30
    max_speed_mps: float = 3.0
    max_imu_rate_rad_s: float = math.radians(220.0)
    imu_jump_margin_rad: float = math.radians(3.0)
    gps_fix_quality: int = 4
    gps_position_gain: float = 0.15
    gps_innovation_gate_m: float = 1.50
    gps_max_correction_m: float = 0.20


@dataclass(frozen=True)
class MotionPriorStatus:
    source: str = 'uninitialized'
    velocity_fresh: bool = False
    imu_fresh: bool = False
    gps_corrected: bool = False
    gps_innovation_m: float = math.inf


class MotionPrior:
    """Predict a local pose from actual speed, IMU yaw and gated GNSS delta.

    IMU yaw may have an arbitrary zero; only differences are used.  GPS deltas
    are accumulated with :func:`compose` because ``dx``/``dy`` are expressed
    in the *previous* vehicle frame.  GPS yaw is only a fallback when the
    direct IMU sample is stale.
    """

    def __init__(self, config: Optional[MotionPriorConfig] = None):
        self.config = config or MotionPriorConfig()
        self.reset()

    def reset(self, pose: Pose2 = Pose2()) -> None:
        self.pose = pose
        self._last_predict_s: Optional[float] = None
        self._velocity_mps = 0.0
        self._velocity_stamp_s = -math.inf
        self._imu_yaw_rad: Optional[float] = None
        self._imu_stamp_s = -math.inf
        self._last_used_imu_yaw: Optional[float] = None
        self._gps_pose: Optional[Pose2] = None
        self._last_gps_update: Optional[int] = None
        self._pending_gps_yaw = 0.0
        self._gps_position_pending = False
        self.last_status = MotionPriorStatus()

    def update_velocity(self, velocity_mps: float, stamp_s: float) -> None:
        if not math.isfinite(velocity_mps) or not math.isfinite(stamp_s):
            return
        limit = max(0.0, self.config.max_speed_mps)
        self._velocity_mps = max(-limit, min(limit, float(velocity_mps)))
        self._velocity_stamp_s = float(stamp_s)

    def update_imu(self, yaw_rad: float, stamp_s: float) -> None:
        if not math.isfinite(yaw_rad) or not math.isfinite(stamp_s):
            return
        self._imu_yaw_rad = float(yaw_rad)
        self._imu_stamp_s = float(stamp_s)

    def update_gps(
        self,
        update: int,
        dx: float,
        dy: float,
        dyaw: float,
        fix_quality: int,
        heading_reliable: bool,
    ) -> bool:
        """Consume one new RTK delta; return whether it was accepted."""
        counter = int(update)
        if self._last_gps_update == counter:
            return False
        if int(fix_quality) != int(self.config.gps_fix_quality):
            return False
        if not all(math.isfinite(value) for value in (dx, dy, dyaw)):
            return False

        self._last_gps_update = counter
        if self._gps_pose is None:
            # The first post-reset sample establishes the same local origin;
            # its delta may predate reset and must not be replayed.
            self._gps_pose = self.pose
            return True

        delta = Pose2(float(dx), float(dy), wrap_angle(float(dyaw)))
        self._gps_pose = compose(self._gps_pose, delta)
        self._gps_position_pending = True
        if heading_reliable:
            self._pending_gps_yaw = wrap_angle(
                self._pending_gps_yaw + delta.yaw)
        return True

    @staticmethod
    def _fresh(sample_s: float, target_s: float, timeout_s: float) -> bool:
        # Independent 10 Hz sensor timers can be almost one full phase apart.
        # The newest sample on either side of the paired scan is accepted only
        # inside the configured bound; larger clock jumps remain invalid.
        return abs(target_s - sample_s) <= max(0.0, timeout_s)

    @staticmethod
    def _integrate_body_motion(pose: Pose2, distance_m: float,
                               yaw_delta: float) -> Pose2:
        if abs(yaw_delta) < 1.0e-7:
            dx_body, dy_body = distance_m, 0.0
        else:
            radius = distance_m / yaw_delta
            dx_body = radius * math.sin(yaw_delta)
            dy_body = radius * (1.0 - math.cos(yaw_delta))
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        return Pose2(
            pose.x + c * dx_body - s * dy_body,
            pose.y + s * dx_body + c * dy_body,
            wrap_angle(pose.yaw + yaw_delta),
        )

    def predict(self, stamp_s: float) -> Pose2:
        stamp_s = float(stamp_s)
        imu_fresh = (
            self._imu_yaw_rad is not None
            and self._fresh(self._imu_stamp_s, stamp_s,
                            self.config.imu_timeout_s)
        )
        velocity_fresh = self._fresh(
            self._velocity_stamp_s, stamp_s, self.config.velocity_timeout_s)

        if self._last_predict_s is None:
            self._last_predict_s = stamp_s
            self._last_used_imu_yaw = self._imu_yaw_rad if imu_fresh else None
            self.last_status = MotionPriorStatus(
                'baseline', velocity_fresh, imu_fresh, False, math.inf)
            return self.pose

        dt = stamp_s - self._last_predict_s
        self._last_predict_s = stamp_s
        if dt <= 0.0 or dt > self.config.max_dt_s:
            self._last_used_imu_yaw = self._imu_yaw_rad if imu_fresh else None
            self._pending_gps_yaw = 0.0
            self.last_status = MotionPriorStatus(
                'time_reset', velocity_fresh, imu_fresh, False, math.inf)
            return self.pose

        yaw_delta = 0.0
        source = 'yaw_held'
        if imu_fresh:
            source = 'imu'
            if self._last_used_imu_yaw is not None:
                candidate = wrap_angle(
                    self._imu_yaw_rad - self._last_used_imu_yaw)
                maximum = (
                    max(0.0, self.config.max_imu_rate_rad_s) * dt
                    + max(0.0, self.config.imu_jump_margin_rad)
                )
                if abs(candidate) <= maximum:
                    yaw_delta = candidate
                else:
                    source = 'imu_jump_rejected'
            self._last_used_imu_yaw = self._imu_yaw_rad
            # Never replay GPS yaw accumulated while direct IMU was healthy.
            self._pending_gps_yaw = 0.0
        else:
            self._last_used_imu_yaw = None
            if abs(self._pending_gps_yaw) > 0.0:
                yaw_delta = self._pending_gps_yaw
                self._pending_gps_yaw = 0.0
                source = 'gps_dyaw_fallback'

        distance = self._velocity_mps * dt if velocity_fresh else 0.0
        self.pose = self._integrate_body_motion(self.pose, distance, yaw_delta)

        corrected = False
        innovation_m = math.inf
        if self._gps_position_pending and self._gps_pose is not None:
            error_x = self._gps_pose.x - self.pose.x
            error_y = self._gps_pose.y - self.pose.y
            innovation_m = math.hypot(error_x, error_y)
            if innovation_m <= max(0.0, self.config.gps_innovation_gate_m):
                gain = min(max(self.config.gps_position_gain, 0.0), 1.0)
                step_x, step_y = gain * error_x, gain * error_y
                step_m = math.hypot(step_x, step_y)
                limit = max(0.0, self.config.gps_max_correction_m)
                if limit > 0.0 and step_m > limit:
                    scale = limit / step_m
                    step_x, step_y = step_x * scale, step_y * scale
                self.pose = Pose2(
                    self.pose.x + step_x,
                    self.pose.y + step_y,
                    self.pose.yaw,
                )
                corrected = gain > 0.0
            self._gps_position_pending = False

        self.last_status = MotionPriorStatus(
            source, velocity_fresh, imu_fresh, corrected, innovation_m)
        return self.pose


class PipelineStage(str, enum.Enum):
    SLAM = 'slam'
    MAPPING = 'mapping'
    LOCALIZATION = 'localization'
    PARKING = 'parking'


class PipelineController:
    """Own the four parking perception stages independently of mission detail."""

    def __init__(self, slam_confirm_scans: int = 10,
                 localization_confirm_scans: int = 3,
                 minimum_map_points: int = 80):
        self.slam_confirm_scans = max(1, int(slam_confirm_scans))
        self.localization_confirm_scans = max(
            1, int(localization_confirm_scans))
        self.minimum_map_points = max(1, int(minimum_map_points))
        self.reset()

    def reset(self) -> None:
        self.stage = PipelineStage.SLAM
        self.slam_accepted = 0
        self.localization_accepted = 0

    @property
    def mapping_enabled(self) -> bool:
        return self.stage in (PipelineStage.SLAM, PipelineStage.MAPPING)

    @property
    def parking_enabled(self) -> bool:
        return self.stage == PipelineStage.PARKING

    def observe_slam(self, accepted: bool, map_points: int) -> bool:
        """Update confirmation counts; return True when the stage changes."""
        previous = self.stage
        if self.stage == PipelineStage.SLAM:
            self.slam_accepted = self.slam_accepted + 1 if accepted else 0
            if (
                self.slam_accepted >= self.slam_confirm_scans
                and int(map_points) >= self.minimum_map_points
            ):
                self.stage = PipelineStage.MAPPING
        elif self.stage == PipelineStage.LOCALIZATION:
            self.localization_accepted = (
                self.localization_accepted + 1 if accepted else 0)
            if self.localization_accepted >= self.localization_confirm_scans:
                self.stage = PipelineStage.PARKING
        return self.stage != previous

    def plan_ready(self) -> bool:
        if self.stage != PipelineStage.MAPPING:
            return False
        self.stage = PipelineStage.LOCALIZATION
        self.localization_accepted = 0
        return True

    def return_to_mapping(self, map_initialized: bool) -> None:
        self.stage = (
            PipelineStage.MAPPING if map_initialized else PipelineStage.SLAM)
        self.localization_accepted = 0
