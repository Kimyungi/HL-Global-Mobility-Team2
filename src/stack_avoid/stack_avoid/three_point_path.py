"""A=(0,0), gap-side B, C=(obstacle_x+2.7,0), frozen in detection pose."""
import math

from stack_avoid.gps_cubic_path import GpsCubicPlanner
from stack_avoid.station_path import StationPath, to_local, to_world, wrap


class ThreePointPlanner(GpsCubicPlanner):
    def reset(self):
        super().reset()
        self.anchors = None
        self.frame = None
        self.last_pose_stamp = None
        self.finished = False

    def create(self, goal, pose, surfaces, generation, offset_max):
        """Search only outward offsets of main's chosen gap; keep C fixed.

        main's minimum-clearance endpoint does not imply a safe swept curve.
        B may move up to 0.4m outward, within the configured lateral limit.
        Every candidate checks both curves with the existing footprint/radius math.
        """
        if self.path is not None:
            return True
        if not all(math.isfinite(v) for v in (*goal, *pose)) or goal[0] <= self.spacing:
            self.reason = 'invalid detection anchors'
            return False
        start, end = (0., 0., 0., 0.), (goal[0]+self.return_distance, 0., 0., 0.)
        for extra in (0., .1, .2, .3, .4):
            side = (goal[0], goal[1]+math.copysign(extra, goal[1]), 0., 0.)
            if abs(side[1]) > offset_max:
                continue
            first = self._curve(start, side, surfaces)
            if first is None:
                continue
            second = self._curve(side, end, surfaces)
            if second is None:
                continue
            self.frame = tuple(pose)
            self.anchors = (start, side, end)
            self.path = StationPath([to_world(p, pose) for p in first+second[1:]])
            self.path.generation = generation
            self.last_pose_stamp = generation
            self.episode_active = True
            self.reason = f'fixed A-B-C path; B extra={extra:.1f}m'
            return True
        self.reason = 'no safe A-B-C curve within gap side and turning radius'
        return False

    def track(self, pose, stamp, speed, surfaces, detected, lookahead=1.):
        if self.path is None or self.finished:
            return None, self.finished, []
        if pose is None:
            self.reason = 'waiting for fresh vehicle pose'
            return None, False, []
        if self.last_pose_stamp is None or stamp > self.last_pose_stamp:
            dt = (stamp-self.last_pose_stamp)*1e-9 if self.last_pose_stamp else .1
            self.path.update(pose[:2], speed, min(.5, max(.001, dt)), stamp)
            self.last_pose_stamp = stamp
        remaining = [to_local(p, pose) for p in self.path.remaining(self.spacing)]
        # Once accepted, retain the path through scan changes and tracking error.
        # Independent LiDAR E-stop remains responsible for immediate obstacles.
        # Initial creation still checks both full curves and the turning radius.
        end = self.path.points[-1]
        reached = (not detected and self.path.station >= self.path.s[-1]-.1
                   and math.dist(end[:2], pose[:2]) <= .1
                   and abs(wrap(end[2]-pose[2])) <= math.radians(20.))
        if reached:
            self.finished = True
            self.reason = 'reached detection-frame C; requesting GPS return'
            return None, True, remaining
        target = to_local(self.path.at(self.path.station+lookahead), pose)
        # Do not issue a point behind the vehicle while waiting for end alignment.
        if target[0] <= 1e-3:
            self.reason = 'fixed path endpoint is not ahead of vehicle'
            return None, False, []
        self.reason = 'tracking fixed A-B-C path'
        return target, False, remaining
