"""At most two obstacle-owned ENU goals, consumed in GPS station order."""
from dataclasses import dataclass
import itertools
import math

from stack_avoid.gps_cubic_path import GpsCubicPlanner
from stack_avoid.station_path import to_local, to_world, transform_surfaces


@dataclass(frozen=True)
class GoalGroup:
    low: float
    high: float
    goals: tuple  # vehicle-frame (x,y) candidates belonging to this station band


@dataclass(frozen=True)
class FixedGoal:
    point: tuple  # ENU, never translated with the moving vehicle
    station: float
    low: float
    high: float


def obstacle_bands(surfaces, pose, waypoints, half_width, merge_gap=.3):
    """Group overlapping station extents; never connect separated scan edges.

    Contours at the same depth may be two visible faces of one obstacle or
    objects beside each other. Treat them as one planning band in either case.
    """
    bands = []
    for surface in surfaces:
        stations, offsets = [], []
        for point in surface.points:
            world = to_world((*point, 0., 0.), pose)
            station = waypoints.project(world)
            if station is None:
                continue
            foot = waypoints.at(station)
            stations.append(station)
            offsets.append(-math.sin(foot[2])*(world[0]-foot[0]) +
                           math.cos(foot[2])*(world[1]-foot[1]))
        if not stations or max(stations) <= waypoints.station:
            continue
        # Include the GPS corridor and the vehicle's current forward corridor.
        ys = [p[1] for p in surface.points]
        relevant = min(offsets) <= half_width and max(offsets) >= -half_width
        relevant |= min(ys) <= half_width and max(ys) >= -half_width
        if relevant and max(p[0] for p in surface.points) > 0:
            bands.append((min(stations), max(stations), [surface]))
    merged = []
    for low, high, contours in sorted(bands, key=lambda b: b[0]):
        if merged and low <= merged[-1][1] + merge_gap:
            old_low, old_high, old_contours = merged[-1]
            merged[-1] = (old_low, max(old_high, high), old_contours + contours)
        else:
            merged.append((low, high, contours))
    return merged


class FixedObstaclePlanner(GpsCubicPlanner):
    """Recheck/rebuild cubics, but do not reselect emitted obstacle goals."""
    def reset(self):
        super().reset()
        self.fixed_goals = ()
        self.passed_goals = 0
        self.return_world = None
        self._entry_goal = None
        self._entry_world = ()

    def wants_group(self, low, high):
        if len(self.fixed_goals) >= 2:
            return False
        # Reobserving the first contour cannot replace its goal or become G2.
        return not self.fixed_goals or low > self.fixed_goals[-1].high + .3

    def step(self, *, pose, waypoints, surfaces, goals=(), detected=False, goal_groups=()):
        if not self.episode_active and not detected and not self.fixed_goals:
            return super().step(pose=pose, waypoints=waypoints, surfaces=surfaces,
                                goals=goals, detected=False)
        self.path = self.control_target = None
        self.anchor_fallback = self.side_switched = False
        self.replan_count += 1
        self.episode_active |= detected
        if pose is None or waypoints is None:
            self.reason = 'fresh GPS pose and waypoint station window required'
            return None, False
        self.gps_station = waypoints.station
        self.anchor_station = waypoints.station + self.anchor_distance
        anchor_world = waypoints.at(self.anchor_station)
        if anchor_world is None:
            self.reason = 'waypoint station+1m is outside route window'
            return None, False
        anchor = to_local(anchor_world, pose)
        observed = transform_surfaces(surfaces, pose)
        if detected or goal_groups:
            if not self.initial_surfaces:
                self.initial_surfaces = observed
            self.last_surfaces = observed
        obstacles = surfaces + transform_surfaces(self.initial_surfaces + self.last_surfaces,
                                                  pose, inverse=True)
        # A passed goal stays passed even if the car subsequently reverses.
        while (self.passed_goals < len(self.fixed_goals) and
               waypoints.station >= self.fixed_goals[self.passed_goals].station):
            self.passed_goals += 1

        groups = [g for g in goal_groups if self.wants_group(g.low, g.high)]
        groups = sorted(groups, key=lambda g: g.low)[:2-len(self.fixed_goals)]
        options = []
        for group in groups:
            candidates = []
            for x, y in group.goals:
                world = to_world((x, y, 0., 0.), pose)
                station = waypoints.project(world, waypoints.station)
                if station is None or (self.fixed_goals and station <= self.fixed_goals[-1].station):
                    continue
                tangent = waypoints.at(station)
                candidates.append(FixedGoal((world[0], world[1], tangent[2], 0.),
                                            station, group.low, group.high))
            if not candidates:
                self.reason = 'observed obstacle has no free side candidate; fixed goals retained'
                return None, False
            options.append(candidates)
        if not self.fixed_goals and not options:
            self.reason = 'waiting for obstacle-owned candidates; no published goal to hold'
            return None, False

        # Cache identical connectors only within this observation, so current
        # surfaces are always checked. This avoids repeating G2->return for G1s.
        curves = {}
        def curve(start, end):
            key = (tuple(start), tuple(end))
            if key not in curves:
                curves[key] = self._curve(start, end, obstacles)
            return curves[key]

        for suffix in itertools.product(*options):
            chain = self.fixed_goals + tuple(suffix)
            if any(b.station <= a.station for a, b in zip(chain, chain[1:])):
                continue
            return_station = chain[-1].station + self.return_distance
            finish_world = (waypoints.at(return_station) if suffix else self.return_world)
            if finish_world is None:
                self.reason = 'last obstacle return waypoint is outside route window'
                continue
            finish = to_local(finish_world, pose)
            pending = chain[self.passed_goals:]
            if not pending:
                foot = to_local(waypoints.at(waypoints.station), pose)
                done = (not detected and waypoints.station >= return_station-.25 and
                        math.hypot(*foot[:2]) <= .25 and abs(foot[2]) <= math.radians(20))
                target = anchor if done else finish
                local = curve((0., 0., 0., 0.), target)
                if local and self._publishable(local, pose):
                    self.mode, self.reason = ('gps', 'fixed obstacle chain returned to GPS') if done else (
                        'return', 'fixed obstacle goals passed; returning to final waypoint')
                    self.control_target = target
                    return target, done
                continue
            ends = [to_local(g.point, pose) for g in pending] + [finish]
            tail = []
            for start, end in zip(ends, ends[1:]):
                segment = curve(start, end)
                if segment is None:
                    break
                tail += segment if not tail else segment[1:]
            else:
                start = anchor if self.anchor_station < pending[0].station and anchor[0] > 0 else (0., 0., 0., 0.)
                entry = curve(start, ends[0])
                fallback = start == (0., 0., 0., 0.)
                if entry is None and not fallback:
                    entry = curve((0., 0., 0., 0.), ends[0])
                    fallback = True
                if (entry is None and not suffix and self._entry_goal == pending[0].point and
                        math.hypot(*ends[0][:2]) < self.spacing):
                    # The connector rejects chords shorter than one sample.
                    # Revalidate the already-generated cubic's terminal part
                    # instead of stopping just before the station threshold.
                    retained = [to_local(p, pose) for p in self._entry_world]
                    if retained and self._safe(retained, obstacles):
                        entry = retained
                if entry and self._publishable(entry + tail[1:], pose):
                    # Commit only a fully feasible chain that will be emitted.
                    self.fixed_goals = chain
                    self.return_world, self.return_station = finish_world, return_station
                    self.goal, self.goal_station = pending[0].point, pending[0].station
                    self.goal_side = self._goal_side(self.goal, self.goal_station, waypoints)
                    self.anchor_fallback = fallback
                    self.mode, self.reason = 'bypass', 'fixed obstacle goals; cubic chain revalidated'
                    self.control_target = entry[-1]
                    self._entry_goal = pending[0].point
                    self._entry_world = tuple(to_world(p, pose) for p in entry)
                    return self.control_target, False
        self.reason = ('fixed obstacle chain blocked; goals retained' if self.fixed_goals else
                       'no collision-free cubic obstacle chain')
        return None, False
