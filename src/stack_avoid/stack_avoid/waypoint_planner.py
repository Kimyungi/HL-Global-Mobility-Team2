"""Fixed ENU cubic paths around LiDAR obstacles; no ROS dependencies.

Station/projection/ENU conversion come from stack_gps.PathEngine. Only the
preview point is transformed to the moving vehicle frame. No random sampling.
"""
from dataclasses import dataclass, replace
import math

import numpy as np

from stack_gps.path_engine import wrap_angle


@dataclass(frozen=True)
class Config:
    wall_offset: float = 2.0
    avoid_offset: float = 1.0
    approach: float = 3.0
    hold: float = 0.7
    departure: float = 3.0
    preview: float = 1.0
    range_limit: float = 3.0
    obstacle_offsets: tuple = (1.0,)
    lateral_tolerance: float = 0.25
    cluster_distance: float = 0.18
    min_points: int = 3
    max_cluster_span: float = 1.0
    association_distance: float = 0.6
    confirmation_frames: int = 2
    vehicle_width: float = 0.62
    vehicle_front: float = 0.76
    vehicle_rear: float = 0.09
    margin: float = 0.10
    sample_step: float = 0.025
    min_turn_radius: float = 1.15
    enforce_turn_radius: bool = True

    def __post_init__(self):
        positive = ('wall_offset', 'avoid_offset', 'approach', 'hold', 'departure',
                    'preview', 'range_limit', 'lateral_tolerance', 'cluster_distance',
                    'max_cluster_span', 'association_distance', 'vehicle_width',
                    'vehicle_front', 'vehicle_rear', 'sample_step', 'min_turn_radius')
        if any(not math.isfinite(getattr(self, k)) or getattr(self, k) <= 0 for k in positive):
            raise ValueError('geometry/range parameters must be finite and positive')
        if self.min_points < 1 or self.confirmation_frames < 1 or self.margin < 0:
            raise ValueError('invalid detection or margin configuration')
        if not self.obstacle_offsets or any(not math.isfinite(d) or d <= 0 for d in self.obstacle_offsets):
            raise ValueError('obstacle_offsets must be positive lateral distances')
        if self.avoid_offset + self.vehicle_width / 2 + self.margin >= self.wall_offset:
            raise ValueError('avoidance offset leaves insufficient wall clearance')


@dataclass(frozen=True)
class ControlPoint:
    station: float
    x: float
    y: float
    yaw: float
    lateral: float


@dataclass(frozen=True)
class Obstacle:
    station: float
    lateral: float
    x: float
    y: float
    points: tuple = ()


@dataclass(frozen=True)
class Cubic:
    start: ControlPoint
    end: ControlPoint

    def evaluate(self, u):
        """Cubic Hermite x(s), y(s); endpoint d(position)/ds = unit yaw.

        Using the same derivative at shared control points makes the path C1
        in station. Curvature is deliberately not claimed to be continuous.
        """
        p, q = self.start, self.end
        length = q.station - p.station
        if length <= 0:
            raise ValueError('control stations must increase')
        a = np.array([p.x, p.y])
        b = np.array([q.x, q.y])
        m = length * np.array([math.cos(p.yaw), math.sin(p.yaw)])
        n = length * np.array([math.cos(q.yaw), math.sin(q.yaw)])
        pos = (2*u**3 - 3*u**2 + 1)*a + (u**3 - 2*u**2 + u)*m
        pos += (-2*u**3 + 3*u**2)*b + (u**3 - u**2)*n
        vel = (6*u*u - 6*u)*a + (3*u*u - 4*u + 1)*m
        vel += (-6*u*u + 6*u)*b + (3*u*u - 2*u)*n
        acc = (12*u - 6)*a + (6*u - 4)*m + (-12*u + 6)*b + (6*u - 2)*n
        norm = float(np.linalg.norm(vel))
        if norm < 1e-9:
            raise ValueError('cubic contains a zero-speed cusp')
        return (float(pos[0]), float(pos[1]), math.atan2(vel[1], vel[0]),
                float((vel[0]*acc[1] - vel[1]*acc[0]) / norm**3),
                p.station + u*length)

    def samples(self, step):
        distance = max(self.end.station - self.start.station,
                       math.hypot(self.end.x-self.start.x, self.end.y-self.start.y))
        count = max(20, math.ceil(2 * distance / step))
        return tuple(self.evaluate(float(u)) for u in np.linspace(0, 1, count + 1))


@dataclass(frozen=True)
class Maneuver:
    obstacle: Obstacle
    points: tuple

    @property
    def segments(self):
        return tuple(Cubic(a, b) for a, b in zip(self.points, self.points[1:]))


def to_global(points, pose):
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return p @ np.array([[c, s], [-s, c]]) + np.asarray(pose[:2])


def to_vehicle(point, pose):
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return c*dx + s*dy, -s*dx + c*dy, wrap_angle(point[2] - pose[2]), point[3]


def filter_cloud(points, config):
    """3 m radial crop after four-sensor fusion, in rear-axle base_link."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    finite = np.all(np.isfinite(p), axis=1)
    within = np.sum(p*p, axis=1) <= config.range_limit**2
    self_hit = ((p[:, 0] >= -config.vehicle_rear - .02) &
                (p[:, 0] <= config.vehicle_front + .02) &
                (np.abs(p[:, 1]) <= config.vehicle_width/2 + .02))
    return p[finite & within & ~self_hit]


def connected_clusters(points, distance):
    """Euclidean connected components using a spatial hash (unordered clouds)."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    cells = {}
    for i, xy in enumerate(p):
        cell = tuple(np.floor(xy / distance).astype(int))
        cells.setdefault(cell, []).append(i)
    unseen = set(range(len(p)))
    clusters = []
    while unseen:
        seed = min(unseen)
        unseen.remove(seed)
        pending, component = [seed], []
        while pending:
            i = pending.pop()
            component.append(i)
            cx, cy = np.floor(p[i] / distance).astype(int)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for j in cells.get((cx+dx, cy+dy), ()):
                        if j in unseen and np.sum((p[i] - p[j])**2) <= distance**2:
                            unseen.remove(j)
                            pending.append(j)
        clusters.append(p[component])
    return clusters


class Detector:
    def __init__(self, route, config):
        self.route, self.config = route, config
        self.previous = []

    def observe(self, map_points):
        """Stable lateral-band objects. Empty scans reset *candidate* confirmation.

        Accepted path/obstacle geometry is stored separately by FixedPlanner.
        Missing returns never mean that an existing maneuver is complete.
        """
        cfg = self.config
        current = []
        used = set()
        for cluster in connected_clusters(map_points, cfg.cluster_distance):
            if len(cluster) < cfg.min_points:
                continue
            if np.linalg.norm(np.ptp(cluster, axis=0)) > cfg.max_cluster_span:
                continue
            x, y = np.mean(cluster, axis=0)
            station, lateral, _, _ = self.route.project_station(x, y)
            if min(abs(abs(lateral)-d) for d in cfg.obstacle_offsets) > cfg.lateral_tolerance:
                continue
            obstacle = Obstacle(station, lateral, float(x), float(y),
                                tuple(map(tuple, cluster.tolist())))
            matches = [(math.hypot(x-o.x, y-o.y), i, count)
                       for i, (o, count) in enumerate(self.previous) if i not in used]
            nearest = min(matches, default=None)
            count = 1
            if nearest and nearest[0] < cfg.association_distance:
                used.add(nearest[1])
                count = nearest[2] + 1
            current.append((obstacle, count))
        self.previous = current
        return sorted((o for o, n in current if n >= cfg.confirmation_frames),
                      key=lambda o: o.station)


class FixedPlanner:
    def __init__(self, route, config=Config()):
        self.route, self.config = route, config
        self.maneuvers = []
        self.segments = ()
        self.samples = ()
        self.completed = []
        self.active_index = 0
        self.revision = 0
        self.last_reason = ''
        self.progress = 0

    def point(self, station, lateral=0.0):
        x, y, yaw, _ = self.route.at_station(station)
        return ControlPoint(station, x - lateral*math.sin(yaw),
                            y + lateral*math.cos(yaw), yaw, lateral)

    def make_maneuver(self, obstacle, first=None):
        c, s = self.config, obstacle.station
        d = -math.copysign(c.avoid_offset, obstacle.lateral)
        points = (first or self.point(s-c.approach), self.point(s, d),
                  self.point(s+c.hold, d), self.point(s+c.hold+c.departure))
        if any(b.station <= a.station for a, b in zip(points, points[1:])):
            raise ValueError('next obstacle is before the frozen third control point')
        return Maneuver(obstacle, points)

    def _sample(self, segments):
        rows = []
        for i, segment in enumerate(segments):
            samples = segment.samples(self.config.sample_step)
            rows.extend(samples if i == 0 else samples[1:])
        return tuple(rows)

    def known(self, obstacle):
        others = self.completed + [m.obstacle for m in self.maneuvers]
        return any(math.hypot(obstacle.x-o.x, obstacle.y-o.y) < self.config.association_distance
                   for o in others)

    def accept(self, obstacle, ego_station):
        """Latch once. A subsequent obstacle may replace only the return suffix."""
        if self.known(obstacle) or obstacle.station <= ego_station:
            return False
        try:
            if not self.maneuvers:
                maneuver = self.make_maneuver(obstacle)
                if ego_station > maneuver.points[0].station:
                    raise ValueError('obstacle detected after the required approach start')
                segments = maneuver.segments
                maneuvers = [maneuver]
            else:
                previous = self.maneuvers[-1]
                # The user explicitly permits replacing P4, including while
                # driving P3->P4. P1->P2->P3 stays bit-for-bit fixed; if P3 was
                # already passed, advance() immediately selects the next plan.
                # The new preview must still be reachable; otherwise the node
                # reports blocked while retaining the new fixed geometry.
                maneuver = self.make_maneuver(obstacle, previous.points[2])
                segments = self.segments[:-1] + maneuver.segments
                changed = replace(previous, points=previous.points[:3] + (maneuver.points[1],))
                maneuvers = self.maneuvers[:-1] + [changed, maneuver]
            samples = self._sample(segments)
        except ValueError as error:
            self.last_reason = str(error)
            return False
        self.maneuvers, self.segments, self.samples = maneuvers, segments, samples
        while self.active_index + 1 < len(maneuvers) and ego_station > maneuvers[self.active_index].points[2].station:
            self.active_index += 1
        self.revision += 1
        self.last_reason = ''
        return True

    def advance(self, pose):
        """(3) becomes next (1); only crossing final (4) clears the fixed path."""
        if not self.maneuvers:
            return False
        station = self.route.project_station(pose[0], pose[1])[0]
        while self.active_index + 1 < len(self.maneuvers):
            p3 = self.maneuvers[self.active_index].points[2]
            if station <= p3.station:
                break
            self.active_index += 1
        last = self.maneuvers[-1].points[3]
        along = (pose[0]-last.x)*math.cos(last.yaw) + (pose[1]-last.y)*math.sin(last.yaw)
        cross = -(pose[0]-last.x)*math.sin(last.yaw) + (pose[1]-last.y)*math.cos(last.yaw)
        if station > last.station and along > 0 and abs(cross) < .5:
            self.completed.extend(m.obstacle for m in self.maneuvers)
            self.maneuvers, self.segments, self.samples = [], (), ()
            self.active_index, self.progress = 0, 0
            self.revision += 1
            return True
        return False

    def preview(self, pose):
        """GPS-style forward Euclidean lookahead, exactly 1 m where intersectable.

        The preview is on the fixed curve. Before/after its endpoints, the
        unchanged waypoint centerline supplies the approach/exit continuation.
        An unreachable circle is reported rather than moving the global path.
        """
        if not self.samples:
            return None
        cfg = self.config
        ego_station = self.route.project_station(*pose[:2])[0]
        rows = []
        start, end = self.samples[0][4], self.samples[-1][4]
        if ego_station < start:
            for s in np.arange(max(0, ego_station-.2), start, cfg.sample_step):
                rows.append((*self.route.at_station(float(s)), float(s)))
        rows.extend(self.samples)
        for s in np.arange(end+cfg.sample_step,
                           min(end+2*cfg.preview, self.route.station[-1]), cfg.sample_step):
            rows.append((*self.route.at_station(float(s)), float(s)))
        array = np.asarray(rows)
        candidates = np.where(array[:, 4] >= ego_station-.5)[0]
        if not len(candidates):
            return None
        nearest = int(candidates[np.argmin(np.sum((array[candidates, :2]-pose[:2])**2, axis=1))])
        c, s = math.cos(pose[2]), math.sin(pose[2])
        for i in range(nearest, len(rows)-1):
            a, b = array[i], array[i+1]
            v, w = b[:2]-a[:2], a[:2]-pose[:2]
            aa, bb = float(v@v), float(2*w@v)
            cc = float(w@w-cfg.preview**2)
            disc = bb*bb - 4*aa*cc
            if aa < 1e-15 or disc < 0:
                continue
            for t in sorted(((-bb-math.sqrt(disc))/(2*aa), (-bb+math.sqrt(disc))/(2*aa))):
                if not 0 <= t <= 1:
                    continue
                xy = a[:2] + t*v
                if (xy[0]-pose[0])*c + (xy[1]-pose[1])*s <= 0:
                    continue
                yaw = wrap_angle(a[2] + t*wrap_angle(b[2]-a[2]))
                return (float(xy[0]), float(xy[1]), yaw,
                        float(a[3] + t*(b[3]-a[3])))
        return None

    def geometry_report(self):
        """Validate fixed geometry without changing user-specified control points."""
        if not self.samples:
            return {'valid': False, 'reason': 'no path'}
        cfg = self.config
        peak = max(abs(p[3]) for p in self.samples)
        radius = 1/peak if peak > 1e-9 else float('inf')
        wall_clearance = float('inf')
        for x, y, yaw, _, _ in self.samples[::4]:
            c, s = math.cos(yaw), math.sin(yaw)
            for longitudinal in (-cfg.vehicle_rear-cfg.margin, cfg.vehicle_front+cfg.margin):
                for lateral in (-cfg.vehicle_width/2-cfg.margin, cfg.vehicle_width/2+cfg.margin):
                    corner = (x+c*longitudinal-s*lateral, y+s*longitudinal+c*lateral)
                    d = self.route.project_station(*corner)[1]
                    wall_clearance = min(wall_clearance, cfg.wall_offset-abs(d))
        reason = ''
        if wall_clearance < 0:
            reason = 'vehicle footprint crosses the virtual wall'
        elif cfg.enforce_turn_radius and radius < cfg.min_turn_radius:
            reason = 'fixed cubic exceeds configured steering curvature'
        return {'valid': not reason, 'reason': reason,
                'min_radius': radius, 'wall_clearance': wall_clearance,
                'peak_curvature': peak}

    def path_blocked(self, map_points, ego_station):
        """Check swept vehicle rectangles against observed points, not centroids."""
        points = np.asarray(map_points, dtype=float).reshape(-1, 2)
        if not len(points):
            return False
        cfg = self.config
        for x, y, yaw, _, station in self.samples[::2]:
            if station < ego_station - cfg.vehicle_rear:
                continue
            delta = points - (x, y)
            c, s = math.cos(yaw), math.sin(yaw)
            longitudinal = delta[:, 0]*c + delta[:, 1]*s
            lateral = -delta[:, 0]*s + delta[:, 1]*c
            hit = ((longitudinal >= -cfg.vehicle_rear-cfg.margin) &
                   (longitudinal <= cfg.vehicle_front+cfg.margin) &
                   (np.abs(lateral) <= cfg.vehicle_width/2+cfg.margin))
            if np.any(hit):
                return True
        return False
