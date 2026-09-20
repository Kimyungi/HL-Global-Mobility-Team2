"""ROS-independent route-3 -> reverse park -> 3 s -> forward exit controller.

MGM owns activation, final arbitration and the route-4 switch. This module
never publishes CAN, guesses a gear acknowledgement, or synthesizes a pose.
"""
from dataclasses import dataclass, replace
import csv
import math
from pathlib import Path

import numpy as np

from .geometry import PathPoint, Pose2, local_reference, wrap_angle
from .t_reference_parking import (
    Candidate, Config, TwoReferenceParking, inspect_candidate, csv_turn_end,
)


@dataclass(frozen=True)
class SequenceOutput:
    phase: str
    speed: float = 0.
    reference: PathPoint | None = None
    done: bool = False
    selected: int | None = None
    reason: str = ''


def csv_rows(path):
    with open(path, encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def metric_path(rows, origin, reverse=False):
    """Use exactly stack_gps' equirectangular datum, NOT CSV east_m/north_m.

    RoutePlan uses its FIRST SELECTED CSV first lat/lon as the common origin.
    This also supports sessions starting at 02 or 03, not just at 01.
    """
    lat0, lon0 = origin
    xy = np.array([[(float(r['lon'])-lon0)*111320.*math.cos(math.radians(lat0)),
                    (float(r['lat'])-lat0)*111320.] for r in rows])
    tangent = np.unwrap([float(r['yaw_rad']) for r in rows])
    if len(rows) < 2 or not np.isfinite(xy).all() or not np.isfinite(tangent).all():
        raise ValueError('Invalid reference coordinates')
    s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    if np.any(np.diff(s) <= 1e-6):
        raise ValueError('Duplicate waypoint')
    dense = np.unique(np.r_[s, np.arange(0., s[-1], .05)])
    k = np.gradient(tangent, s) * (-1 if reverse else 1)
    path = tuple(PathPoint(float(x), float(y), wrap_angle(float(yaw)), float(curv),
                           -1 if reverse else 1)
                 for x, y, yaw, curv in zip(np.interp(dense,s,xy[:,0]),
                     np.interp(dense,s,xy[:,1]),
                     np.interp(dense,s,tangent)+(math.pi if reverse else 0),
                     np.interp(dense,s,k)))
    return path, dense


def load_course(origin_csv, route_csv, parking_csvs, *, route_id=3, mission_state=1):
    first = csv_rows(origin_csv)[0]
    origin = float(first['lat']), float(first['lon'])
    route = csv_rows(route_csv)
    trigger = [i for i,r in enumerate(route) if int(r['state']) == mission_state]
    if len(trigger) != 1 or any(int(r['path_id']) != route_id for r in route):
        raise ValueError(f'Expected route {route_id:02} with one state={mission_state}')
    # Stable zone confirmation can arrive a few samples after the marker.
    # Zone span/stability can stop slightly before the exact state marker.
    # Keep the preceding CSV points as well; no invented connector segment.
    approach, station = metric_path(route[max(0,trigger[0]-8):], origin)
    candidates = []
    for p in parking_csvs:
        rows = csv_rows(p)
        path, candidate_station = metric_path(rows, origin, True)
        raw_xy = np.array([[(float(r['lon'])-origin[1])*111320.*math.cos(math.radians(origin[0])),
                            (float(r['lat'])-origin[0])*111320.] for r in rows])
        raw_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(raw_xy, axis=0), axis=1))]
        candidates.append(Candidate(Path(p).stem, path, candidate_station, csv_turn_end(rows, raw_s)))
    candidates = tuple(candidates)
    for c in candidates:
        # halla_0919 straightening shifts the endpoint by 8 cm. Accept only
        # gaps within the existing 14 cm forward endpoint arrival tolerance.
        if math.hypot(c.path[0].x-approach[-1].x, c.path[0].y-approach[-1].y) > .14:
            raise ValueError(f'Route {route_id:02} end and parking start do not match')
    return candidates, approach, station


class TParkingSequence:
    WAIT_SECONDS = 3.
    SELECTION_TIMEOUT_SECONDS = 3.

    def __init__(self, candidates, approach, station, cfg=Config(), *, exits=None):
        self.candidates, self.approach, self.station, self.cfg = candidates, approach, station, cfg
        self.exits = exits
        self.phase = 'STOP_SELECT'
        self.reason = 'await_stop_and_left_scan'
        self.selected = None
        self.stopped_since = self.wait_since = None
        self.selection_since = None
        self.last_now = self.last_left = -math.inf
        self.vote, self.votes = None, 0
        self.index = None
        self.reverse = None
        self.exit_path = self.exit_station = None

    def out(self, speed=0., reference=None):
        return SequenceOutput(self.phase, speed, reference, self.phase == 'DONE',
                              self.selected, self.reason)

    def fault(self, reason):
        self.phase, self.reason = 'FAULT', reason
        return self.out()

    def _track(self, path, station, pose):
        if self.index is None:
            self.index = int(np.argmin([(p.x-pose.x)**2+(p.y-pose.y)**2 for p in path]))
        bound = int(np.searchsorted(station, station[self.index]+1.5, side='right'))
        search = path[self.index:max(self.index+1,bound)]
        self.index += int(np.argmin([(p.x-pose.x)**2+(p.y-pose.y)**2 for p in search]))
        end = path[-1]
        arrived = (station[-1]-station[self.index] <= .14
                   and math.hypot(end.x-pose.x,end.y-pose.y) <= .14)
        preview = min(int(np.searchsorted(station,station[self.index]+self.cfg.preview)),len(path)-1)
        return arrived, local_reference(path[preview],pose)

    def tick(self, now, pose, pose_stamp, speed, speed_stamp, left, rear, front,
             *, owned, route_at_end=False, motion_allowed=True):
        cfg = self.cfg
        if self.phase == 'FAULT':
            return self.out()
        if not math.isfinite(now) or not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw, speed)):
            return self.out()
        if now < self.last_now:
            self.stopped_since = self.wait_since = None
            self.selection_since = None
        self.last_now = now
        if abs(speed) <= cfg.stop_speed:
            if self.stopped_since is None:
                self.stopped_since = now
        else:
            self.stopped_since = None
        stationary = self.stopped_since is not None and now-self.stopped_since >= cfg.stop_hold

        if self.phase == 'STOP_SELECT':
            if not stationary:
                self.selection_since = None
                return self.out()
            if self.selection_since is None:
                self.selection_since = now
            if now-self.selection_since >= self.SELECTION_TIMEOUT_SECONDS:
                # Catalog order: T = 01/02, parallel = 03/04. The user-defined
                # timeout choice is independent of missing/ambiguous scan data.
                self.selected = 0
                self.phase, self.reason = 'ADVANCE_3', 'candidate_timeout_default'
                self.index = None
                self.stopped_since = None
                return self.out()  # zero-speed handoff; adapter awaits ACTIVATE
            if left is None or not len(left.points) or left.stamp <= self.last_left:
                return self.out()
            self.last_left = left.stamp
            findings = [inspect_candidate(c,pose,left,cfg) for c in self.candidates]
            allowed = [i for i,c in enumerate(self.candidates)
                       if not findings[i][0]
                       and (findings[1-i][0] or findings[i][1] >= cfg.observed_fraction)
                       and (not cfg.enforce_min_radius or c.minimum_radius >= cfg.min_radius)]
            choice = max(allowed,key=lambda i:(findings[i][1],-i)) if allowed else None
            self.votes = self.votes+1 if choice is not None and choice == self.vote else int(choice is not None)
            self.vote = choice
            if choice is not None and self.votes >= cfg.confirm_frames:
                self.selected = choice
                self.phase, self.reason = 'ADVANCE_3', 'candidate_locked_finish_route3'
                self.index = None
                self.stopped_since = None
            return self.out()

        if self.phase in ('ADVANCE_3','EXIT'):
            path, station = (self.approach,self.station) if self.phase == 'ADVANCE_3' else (self.exit_path,self.exit_station)
            tracked = self._track(path,station,pose)
            arrived, reference = tracked
            if arrived:
                # GPS endpoint and metric junction must both agree before reverse.
                if self.phase == 'ADVANCE_3' and not route_at_end:
                    self.reason = 'await_gps_route3_endpoint'
                    return self.out()
                if self.phase == 'EXIT':
                    self.phase, self.reason = 'DONE', 'exit_complete_continue_next_route'
                    return self.out(cfg.forward_speed, reference)
                self.phase = 'STOP_REVERSE'
                self.stopped_since = None
                return self.out()
            self.reason = 'forward_reference_tracking'
            return self.out(cfg.forward_speed,reference)

        if self.phase == 'STOP_REVERSE':
            if not stationary:
                return self.out()
            self.reverse = TwoReferenceParking(self.candidates,cfg)
            self.reverse.selected = self.selected
            self.reverse.phase = 'REVERSE'
            self.phase = 'REVERSE'
            self.stopped_since = None
            return self.out()

        if self.phase == 'REVERSE':
            result = self.reverse.tick(now,pose,pose_stamp,speed,speed_stamp,left,rear,
                                       parking_owned=True)
            self.reason = result.reason
            if result.phase == 'FAULT':
                return self.fault(result.reason)
            if result.parking_success:
                # Exit precisely the driven prefix, not an unvisited CSV tail.
                candidate = self.candidates[self.selected]
                if self.exits is not None:
                    self.exit_path, self.exit_station = self.exits[self.selected]
                else:
                    prefix = candidate.path[:self.reverse.index+1]
                    self.exit_path = tuple(replace(p,gear=1) for p in reversed(prefix))
                    distances = candidate.s[:self.reverse.index+1]
                    self.exit_station = distances[-1]-distances[::-1]
                self.phase, self.wait_since = 'WAIT_3', now
                return self.out()
            return self.out(result.v_suggest,result.reference)

        if self.phase == 'WAIT_3':
            if not stationary:
                self.wait_since = None
                return self.out()
            if self.wait_since is None:
                self.wait_since = now
            if now-self.wait_since >= self.WAIT_SECONDS:
                self.phase, self.index, self.reason = 'EXIT', 0, 'three_seconds_complete'
            return self.out()  # zero-speed boundary before changing velocity sign

        if self.phase == 'DONE':
            # Keep publishing the final forward reference until MGM acknowledges
            # completion. A zero-speed completion message would brake the car
            # before the next route's normal navigation takes ownership.
            return self.out(cfg.forward_speed, local_reference(self.exit_path[-1], pose))
        return self.out()
