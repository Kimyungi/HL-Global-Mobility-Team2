"""Acquire a left reference wall from five independent local scan windows."""
from dataclasses import dataclass
import math
import numpy as np

from .geometry import Pose2, transform_points
from .wall_gap_detector import ReferenceWall, WallGapConfig


@dataclass(frozen=True)
class LateralWallConfig:
    # b1 translation from lidar_fusion_v2 fixed geometry; yaw is deliberately
    # not used: the search ray is vehicle +y, not the sensor's optical axis.
    origin_x: float
    origin_y: float
    angular_step_deg: float = 1.0
    neighbors_each_side: int = 5
    confirm_frames: int = 5
    stale_s: float = .35


class LateralWallAcquisition:
    def __init__(self, config: LateralWallConfig, wall_config=None):
        if config.angular_step_deg <= 0 or config.confirm_frames <= 0 or config.neighbors_each_side < 1:
            raise ValueError('Invalid wall acquisition parameters')
        self.config = config
        self.wall_config = wall_config or WallGapConfig()
        self.reset()

    def reset(self):
        self.generation = None
        self.frames = []
        self.selected_base = np.empty((0,2))
        self.support_map = np.empty((0,2))
        self.wall = None
        self.candidate = None
        self.reason = 'WAITING_FOR_SCAN'
        self.first_pose = None

    @property
    def count(self):
        return len(self.frames)

    def invalidate(self, reason):
        self.reason = reason
        self.selected_base = np.empty((0,2))
        if self.wall is None:
            self.frames.clear()
            self.candidate = None
            self.support_map = np.empty((0,2))
            self.first_pose = None

    def select(self, points):
        """One nearest return per angular bin, centre + five bins either side.

        Input is the fused base_link cloud, with no filtering by sensor ID.
        Missing bins are not replaced by points farther outside the window.
        """
        pts = np.asarray(points,dtype=float).reshape((-1,2))
        pts = pts[np.isfinite(pts).all(axis=1)]
        delta = pts - [self.config.origin_x,self.config.origin_y]
        distance = np.linalg.norm(delta,axis=1)
        offset = np.degrees(np.arctan2(delta[:,1],delta[:,0])) - 90.
        bins = np.floor(offset/self.config.angular_step_deg+.5).astype(int)
        mask = (delta[:,1]>0) & (np.abs(bins)<=self.config.neighbors_each_side)
        # Select the nearest endpoint before range validation: an occluding
        # close obstacle must not be replaced by a farther wall behind it.
        chosen = []
        centre = False
        for b in range(-self.config.neighbors_each_side,self.config.neighbors_each_side+1):
            ids = np.flatnonzero(mask & (bins==b))
            if not len(ids): continue
            i = ids[np.argmin(distance[ids])]
            if not self.wall_config.near_m <= distance[i] <= self.wall_config.far_m: continue
            chosen.append(pts[i]); centre |= b==0
        return np.asarray(chosen,dtype=float).reshape((-1,2)), centre

    def _fit(self, points, pose):
        centroid = points.mean(axis=0)
        delta = points-centroid
        values,vectors = np.linalg.eigh(delta.T@delta)
        if values[-1] <= np.finfo(float).eps: return None
        tangent = vectors[:,-1]
        forward = np.array([math.cos(pose.yaw),math.sin(pose.yaw)])
        if tangent@forward<0: tangent=-tangent
        if abs(math.atan2(forward[0]*tangent[1]-forward[1]*tangent[0],forward@tangent)) > math.radians(self.wall_config.initial_wall_max_angle_deg):
            return None
        normal = np.array([-tangent[1],tangent[0]])
        if np.max(np.abs(delta@normal)) > self.wall_config.wall_line_offset_m: return None
        mount = transform_points(np.array([[self.config.origin_x,self.config.origin_y]]),pose)[0]
        distance = float((centroid-mount)@normal)
        if distance <= 0: return None
        anchor = mount+distance*normal
        return ReferenceWall('left',float(anchor[0]),float(anchor[1]),float(tangent[0]),float(tangent[1]),
                             float(normal[0]),float(normal[1]),distance)

    def update(self, points, pose: Pose2, generation_s: float, now_s: float):
        if not math.isfinite(generation_s) or not 0 < generation_s <= now_s or now_s-generation_s>self.config.stale_s:
            self.invalidate('STALE_SCAN'); return self.wall
        if self.generation is not None:
            if generation_s < self.generation:
                self.invalidate('REGRESSING_SCAN'); return self.wall
            if generation_s == self.generation: return self.wall
            if generation_s-self.generation>self.config.stale_s: self.invalidate('SCAN_GAP')
        self.generation = generation_s
        if not all(math.isfinite(x) for x in (pose.x,pose.y,pose.yaw)):
            self.invalidate('INVALID_POSE'); return self.wall
        selected,centre = self.select(points)
        if not centre or len(selected)<self.wall_config.initial_wall_min_points:
            self.invalidate('NO_CENTRE_RETURN' if not centre else 'INSUFFICIENT_POINTS'); return self.wall
        self.selected_base = selected
        if self.wall is not None:
            self.reason = 'LOCKED'; return self.wall
        mapped = transform_points(selected,pose)
        candidate = self._fit(mapped,pose)
        if candidate is None:
            self.invalidate('NOT_A_WALL_LINE'); return None
        if not self.frames: self.first_pose=pose
        pooled = np.vstack(self.frames+[mapped])
        candidate = self._fit(pooled,self.first_pose)
        if candidate is None:
            # An inconsistent observation breaks the consecutive run and
            # becomes the first sample of a new run, never the fifth one.
            self.frames=[]; self.first_pose=pose; pooled=mapped
            candidate=self._fit(mapped,pose)
        self.frames.append(mapped)
        self.support_map=pooled; self.candidate=candidate
        self.reason='CONFIRMING'
        if self.count>=self.config.confirm_frames:
            self.wall=candidate; self.reason='LOCKED'
        return self.wall
