"""GPS station -> free obstacle-side target -> GPS station+2.7m, rebuilt per scan."""
from bisect import bisect_right
import math

from stack_avoid.station_path import StationPath, footprint_clear, to_local, to_world, transform_surfaces, wrap


class WaypointWindow:
    """Waypoint geometry in one fixed ENU frame, with GPS-owned CSV stations."""
    def __init__(self, stations, points, station):
        self.s, self.points, self.station = list(stations), list(points), float(station)
        if (len(self.s) < 2 or len(self.s) != len(self.points)
                or not all(math.isfinite(s) for s in self.s)
                or not all(math.isfinite(v) for p in self.points for v in p)
                or any(b <= a for a, b in zip(self.s, self.s[1:]))
                or any(math.dist(a[:2], b[:2]) < 1e-9 for a, b in zip(self.points, self.points[1:]))
                or not self.s[0] <= self.station <= self.s[-1]):
            raise ValueError('invalid GPS waypoint station window')

    def at(self, station):
        if not math.isfinite(station) or not self.s[0] <= station <= self.s[-1]:
            return None  # Never clamp an unavailable configured return to an earlier point.
        i = min(len(self.s)-2, max(0, bisect_right(self.s, station)-1))
        t = (station-self.s[i])/(self.s[i+1]-self.s[i])
        a, b = self.points[i:i+2]
        return (a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]),
                wrap(a[2]+t*wrap(b[2]-a[2])), a[3]+t*(b[3]-a[3]))

    def project(self, point, low=None, high=None):
        low = self.s[0] if low is None else max(low, self.s[0])
        high = self.s[-1] if high is None else min(high, self.s[-1])
        best = None
        for i, (a, b) in enumerate(zip(self.points, self.points[1:])):
            if self.s[i+1] < low or self.s[i] > high:
                continue
            length = self.s[i+1]-self.s[i]
            dx, dy = b[0]-a[0], b[1]-a[1]
            norm = dx*dx+dy*dy
            if norm < 1e-12:
                continue
            t = max((low-self.s[i])/length, 0.,
                    min((high-self.s[i])/length, 1.,
                        ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/norm))
            candidate = (math.hypot(point[0]-a[0]-t*dx, point[1]-a[1]-t*dy),
                         self.s[i]+t*length)
            if best is None or candidate < best:
                best = candidate
        return None if best is None else best[1]


def cubic_connector(start, goal, spacing=.05, scale=1.):
    """Parametric cubic Hermite: exact endpoints and endpoint tangent headings.

    Both x(t) and y(t) are degree three. Tangent length candidates allow a
    feasible turning radius without changing either endpoint or its heading.
    """
    distance = math.dist(start[:2], goal[:2])
    chord = (goal[0]-start[0], goal[1]-start[1])
    if (distance < spacing or any(chord[0]*math.cos(yaw)+chord[1]*math.sin(yaw) <= 0
                                  for yaw in (start[2], goal[2]))):
        return []
    tangent = distance*scale
    m0 = (tangent*math.cos(start[2]), tangent*math.sin(start[2]))
    m1 = (tangent*math.cos(goal[2]), tangent*math.sin(goal[2]))
    # Conservative sampling in parameter t; validate actual chord spacing too.
    count = max(2, math.ceil(distance*(2+2*scale)/spacing))
    out = []
    for j in range(count+1):
        t = j/count
        h = (2*t**3-3*t*t+1, t**3-2*t*t+t, -2*t**3+3*t*t, t**3-t*t)
        d = (6*t*t-6*t, 3*t*t-4*t+1, -6*t*t+6*t, 3*t*t-2*t)
        dd = (12*t-6, 6*t-4, -12*t+6, 6*t-2)
        coords = [(start[k], m0[k], goal[k], m1[k]) for k in (0, 1)]
        x, y = [sum(a*b for a, b in zip(h, c)) for c in coords]
        dx, dy = [sum(a*b for a, b in zip(d, c)) for c in coords]
        ddx, ddy = [sum(a*b for a, b in zip(dd, c)) for c in coords]
        speed = math.hypot(dx, dy)
        if speed < 1e-8:
            return []
        out.append((x, y, math.atan2(dy, dx), (dx*ddy-dy*ddx)/speed**3))
    return out


class GpsCubicPlanner:
    def __init__(self, *, width, length, front, margin, min_radius, spacing=.05,
                 anchor_distance=1., return_distance=2.7, connector=None, collision_check=None):
        self.width, self.front = width, front
        self.rear, self.margin = max(0., length-front), margin
        self.min_radius, self.spacing = min_radius, spacing
        self.anchor_distance, self.return_distance = anchor_distance, return_distance
        self._connector = connector or cubic_connector
        self._collision_check = collision_check or footprint_clear
        self.reset()

    def reset(self):
        self.path = None
        self.episode_active = False
        self.goal = None
        self.goal_side = None
        self.side_switched = False
        self.goal_station = self.return_station = None
        self.initial_surfaces, self.last_surfaces = [], []
        self.reason, self.mode = 'GPS waypoints unavailable', 'idle'
        self.replan_count = 0
        self.gps_station = self.anchor_station = None
        self.control_target = None
        self.anchor_fallback = False

    def _safe(self, points, obstacles):
        if not points or any(abs(p[3]) > 1/self.min_radius+1e-6 for p in points):
            return False
        radius = math.hypot(max(self.front, self.rear), self.width/2)
        pad = self.spacing/2*(1+radius/self.min_radius)
        return self._collision_check(points, obstacles, self.width, self.front, self.rear, self.margin+pad)

    @staticmethod
    def _goal_side(world, station, waypoints):
        # Classify against the GPS route, not vehicle Y: turning/moving the
        # vehicle must not reverse the meaning of the chosen passing side.
        tangent = waypoints.at(station)
        lateral = (-math.sin(tangent[2])*(world[0]-tangent[0])
                   + math.cos(tangent[2])*(world[1]-tangent[1]))
        return 0 if abs(lateral) <= .05 else (1 if lateral > 0 else -1)

    def _ordered_goals(self, candidates, waypoints):
        if self.goal is None or self.goal_side is None:
            return candidates
        # Reorder fresh candidates only. Never reuse/interpolate an unchecked
        # old endpoint. Every candidate still needs both safe cubic connectors.
        def preference(candidate):
            world, station = candidate
            same_side = self._goal_side(world, station, waypoints) == self.goal_side
            return (0, math.dist(world[:2], self.goal[:2])) if same_side else (1, 0.)
        return sorted(candidates, key=preference)

    def _curve(self, start, goal, obstacles):
        feasible = []
        for scale in (1., 1.25, 1.5, 1.75):
            points = self._connector(start, goal, self.spacing, scale)
            if self._safe(points, obstacles):
                feasible.append(points)
        return min(feasible, key=lambda p: max(abs(q[3]) for q in p)) if feasible else None

    def _publishable(self, local, pose):
        """Remove traveled prefix; never append old path points on an update."""
        if len(local) < 2:
            return False
        path = StationPath(local)
        window = WaypointWindow(path.s, path.points, 0.)
        station = window.project((0., 0.))
        remaining = [window.at(station)] + [p for s, p in zip(path.s, local) if s > station+1e-8]
        if len(remaining) < 2:
            return False
        self.path = StationPath([to_world(p, pose) for p in remaining])
        return True

    def step(self, *, pose, waypoints, surfaces, goals, detected):
        self.path, self.control_target = None, None  # No stale route on failure.
        self.anchor_fallback = False
        self.side_switched = False
        self.replan_count += 1
        self.episode_active = self.episode_active or detected
        if pose is None or waypoints is None:
            self.reason = 'fresh GPS pose and waypoint station window required'
            return None, False
        self.gps_station = waypoints.station
        self.anchor_station = waypoints.station+self.anchor_distance
        anchor_world = waypoints.at(self.anchor_station)
        if anchor_world is None:
            self.reason = 'waypoint station+1m is outside route window'
            return None, False
        anchor = to_local(anchor_world, pose)
        observed = transform_surfaces(surfaces, pose)
        if detected:
            if not self.initial_surfaces:
                self.initial_surfaces = observed
            self.last_surfaces = observed
        obstacles = surfaces+transform_surfaces(self.initial_surfaces+self.last_surfaces, pose, inverse=True)
        if not self.episode_active:
            local = [to_local(p, pose) for s, p in zip(waypoints.s, waypoints.points)
                     if s >= waypoints.station]
            current = waypoints.at(waypoints.station)
            local = [to_local(current, pose)]+[p for p in local if math.dist(p[:2], to_local(current, pose)[:2]) > 1e-8]
            if not self._publishable(local, pose):
                self.reason = 'empty forward waypoint route'
                return None, False
            self.mode, self.reason = 'gps', 'following GPS station+1m'
            self.control_target = anchor
            return anchor, False

        # Every observed scan can choose a new free-space point. Between
        # detections retain only its ENU location/CSV station, never an old path.
        candidates = []
        if detected:
            for x, y in goals:
                world = to_world((x, y, 0., 0.), pose)
                station = waypoints.project(world, waypoints.station)
                if station is None:
                    continue
                tangent = waypoints.at(station)
                candidates.append(((world[0], world[1], tangent[2], 0.), station))
            candidates = self._ordered_goals(candidates, waypoints)
        elif self.goal is not None:
            candidates = [(self.goal, self.goal_station)]
        else:
            # Detection disappeared before any feasible target existed. Try a
            # direct cubic GPS return from current pose, rather than deadlock.
            curve = self._curve((0., 0., 0., 0.), anchor, obstacles)
            if curve and self._publishable(curve, pose):
                self.mode, self.reason = 'return', 'obstacle cleared; cubic GPS return'
                self.control_target = anchor
                foot = to_local(waypoints.at(waypoints.station), pose)
                done = math.hypot(foot[0], foot[1]) <= .25 and abs(foot[2]) <= math.radians(20)
                return anchor, done

        for goal_world, goal_station in candidates:
            return_station = goal_station+self.return_distance
            return_world = waypoints.at(return_station)
            if return_world is None:
                continue
            goal, finish = to_local(goal_world, pose), to_local(return_world, pose)
            returning = not detected and (waypoints.station >= goal_station or goal[0] <= .05)
            if returning:
                foot = to_local(waypoints.at(waypoints.station), pose)
                if (waypoints.station >= return_station-.25 and math.hypot(*foot[:2]) <= .25
                        and abs(foot[2]) <= math.radians(20)):
                    onward = self._curve((0., 0., 0., 0.), anchor, obstacles)
                    if onward and self._publishable(onward, pose):
                        self.mode, self.reason = 'gps', 'GPS return station reached'
                        self.control_target = anchor
                        return anchor, True
                    continue
                local = self._curve((0., 0., 0., 0.), finish, obstacles)
                target = finish
            else:
                # Once the moving +1m anchor has passed the side point, only
                # the untraveled connector from the actual pose remains.
                start = anchor if self.anchor_station < goal_station and anchor[0] > 0 else (0., 0., 0., 0.)
                exit_curve = self._curve(goal, finish, obstacles)
                if exit_curve is None:
                    continue
                entry = self._curve(start, goal, obstacles)
                fallback = start == (0., 0., 0., 0.)
                if entry is None and not fallback:
                    entry = self._curve((0., 0., 0., 0.), goal, obstacles)
                    fallback = True
                if entry is None:
                    continue
                self.anchor_fallback = fallback
                local = entry+exit_curve[1:]
                # Both cubics share the waypoint tangent at the side point.
                target = entry[-1]
            if local and self._publishable(local, pose):
                if detected:
                    side = self._goal_side(goal_world, goal_station, waypoints)
                    self.side_switched = self.goal_side is not None and side != self.goal_side
                    self.goal_side = side
                self.goal, self.goal_station = goal_world, goal_station
                self.return_station = return_station
                self.control_target = target
                self.mode = 'return' if returning else 'bypass'
                self.reason = 'cubic GPS return' if returning else 'cubic waypoint anchor -> side point -> waypoint return'
                return target, False
        self.reason = f'no collision-free cubic path with waypoint return at +{self.return_distance:g}m'
        return None, False
