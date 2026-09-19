"""Historical persistent planner for regression replay.

Production StationPathIO now uses gps_cubic_path.GpsCubicPlanner.
"""
import math

from stack_avoid.station_path import (
    StationPath, connector, straight, to_world, to_local, transform_surfaces,
    footprint_clear, observed_free,
)


class AvoidPathPlanner:
    def __init__(self, *, width, length, front, margin, min_radius, spacing=.05,
                 preview=1., tail_length=3., return_tolerance=.25):
        self.width, self.length, self.front = width, length, front
        self.rear, self.margin = max(0., length-front), margin
        self.min_radius, self.spacing = min_radius, spacing
        self.preview, self.tail_length = preview, tail_length
        self.return_tolerance = return_tolerance
        self.reset()

    def reset(self):
        self.path = None
        self.episode_active = False
        self.replan_count = 0
        self.last_replan_distance = 0.
        self.initial_surfaces = []
        self.last_surfaces = []
        self.mode = 'idle'
        self.return_station = None
        self.reason = 'no obstacle episode'

    def _safe(self, points, surfaces):
        if not points or any(abs(p[3]) > 1/self.min_radius+1e-6 for p in points):
            return False
        # Conservative travel/rotation allowance for finite path sampling.
        radius = math.hypot(max(self.front, self.rear), self.width/2)
        pad = self.spacing/2 * (1+radius/self.min_radius)
        return footprint_clear(points, surfaces, self.width, self.front,
                               self.rear, self.margin+pad)

    def _return_visible(self, points, scan, lidar):
        return observed_free(points, scan, lidar, self.width/2+self.margin,
                             self.front, self.spacing/2)

    def _set_tail(self, local_tail, pose):
        world_tail = [to_world(p, pose) for p in local_tail]
        if self.path is None:
            self.path = StationPath(world_tail)
        else:
            self.path.splice(world_tail)

    def step(self, *, pose, surfaces, scan, lidar, goals, detected, gps_goal,
             v_ref, sample_time, generation):
        """Return (vehicle-frame preview or None, maneuver_done).

        Pose is latched to a stable localization source by the ROS wrapper.
        Normal replanning preserves the traveled prefix. An off-path vehicle
        starts a new path at its current pose, retaining episode obstacle memory.
        """
        self.episode_active = self.episode_active or detected
        if self.path is None and not self.episode_active:
            self.reason = 'no obstacle episode'
            return None, False
        if pose is None:
            self.reason = 'localization unavailable'
            return None, False
        observed_world = transform_surfaces(surfaces, pose)
        if detected:
            if not self.initial_surfaces:
                self.initial_surfaces = observed_world
            self.last_surfaces = observed_world
        remembered = transform_surfaces(self.initial_surfaces+self.last_surfaces, pose, inverse=True)
        obstacles = surfaces+remembered
        if self.path is not None:
            self.path.update(pose[:2], v_ref, sample_time, generation)
            start = to_local(self.path.at(self.path.station), pose)
            distance = math.hypot(start[0], start[1])
            if distance > .75:
                # A mission interruption or reverse can leave the old path.
                # Its station is no longer a usable start for a connector.
                # Keep the localization frame and measured obstacles; certify
                # a new path from the actual vehicle pose, even on later scans
                # if the first attempt fails or detection subsequently clears.
                self.episode_active = True
                self.path = None
                self.mode, self.return_station = 'replan', None
                self.replan_count += 1
                self.last_replan_distance = distance
                start = (0., 0., 0., 0.)
        else:
            start = (0., 0., 0., 0.)

        # A return is latched, so the moving GPS preview cannot move its finish
        # station forever. Recheck the actual remaining route each scan.
        remaining = ([to_local(p, pose) for p in self.path.remaining(self.spacing)]
                     if self.path is not None else [])
        route_safe = self._safe(remaining, obstacles)
        if self.mode == 'return':
            to_goal = [p for s, p in zip(self.path.s, self.path.points)
                       if self.path.station <= s <= self.return_station]
            visible = self._return_visible([to_local(p, pose) for p in to_goal], scan, lidar)
            if gps_goal is None or not route_safe or not visible:
                self.mode, self.return_station = 'straight', None
                # An uncertified return must not remain the active suffix.
                remaining, route_safe = [], False
            elif self.path.station >= self.return_station-self.return_tolerance:
                finish = connector(start, gps_goal, self.spacing)
                if self._safe(finish, obstacles) and self._return_visible(finish, scan, lidar):
                    self.reason = 'GPS return reached and onward connector clear'
                    return to_local(self.path.at(self.path.station+self.preview), pose), True

        if self.mode != 'return' and gps_goal is not None:
            returning = connector(start, gps_goal, self.spacing)
            if (self._safe(returning, obstacles)
                    and self._return_visible(returning, scan, lidar)):
                end_distance = StationPath(returning).s[-1]
                tail = straight(returning[-1], self.preview+self.tail_length, self.spacing)
                # Unknown beyond the GPS goal is not required to certify that
                # goal, but known obstacles must still be excluded from preview.
                combined = returning+tail[1:]
                if self._safe(combined, obstacles):
                    self._set_tail(combined, pose)
                    self.return_station = self.path.station+end_distance
                    self.mode = 'return'
                    remaining, route_safe = combined, True

        if not route_safe and self.mode != 'return':
            planned = None
            if detected:
                for goal in goals:
                    for lead in (self.front, self.front/2, 0.):
                        turn_end = (goal[0]-lead, goal[1], 0., 0.)
                        entry = connector(start, turn_end, self.spacing)
                        if not entry:
                            continue
                        length = max(self.tail_length, goal[0]-turn_end[0]+self.length+self.preview)
                        tail = straight(entry[-1], length, self.spacing)
                        candidate = entry+tail[1:]
                        if self._safe(candidate, obstacles):
                            planned = candidate
                            break
                    if planned:
                        break
            else:
                # Rear/occluded obstacle extent unknown: extend the current
                # path tangent rather than guessing an early turn toward GPS.
                candidate = straight(start, self.tail_length+self.preview, self.spacing)
                if self._safe(candidate, obstacles):
                    planned = candidate
            if planned is None:
                self.reason = 'no collision-free, feasible path'
                return None, False
            self._set_tail(planned, pose)
            self.mode = 'bypass' if detected else 'straight'

        if self.path.s[-1]-self.path.station < self.preview+self.spacing:
            local_end = to_local(self.path.points[-1], pose)
            extension = straight(local_end, self.tail_length, self.spacing)
            if not self._safe(extension, obstacles):
                self.reason = 'straight path extension blocked'
                return None, False
            self.path.points.extend(to_world(p, pose) for p in extension[1:])
            self.path._lengths()
        self.reason = 'returning to GPS' if self.mode == 'return' else 'following avoidance path'
        return to_local(self.path.at(self.path.station+self.preview), pose), False
