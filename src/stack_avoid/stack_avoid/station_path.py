"""Persistent avoidance path, bounded station search and smooth local connectors."""
from bisect import bisect_right
import math
import numpy as np

from stack_avoid.surfaces import Surface


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def to_world(point, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (pose[0] + c * point[0] - s * point[1],
            pose[1] + s * point[0] + c * point[1],
            wrap(pose[2] + point[2]), point[3])


def to_local(point, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    x, y = point[0] - pose[0], point[1] - pose[1]
    return (c*x + s*y, -s*x + c*y, wrap(point[2]-pose[2]), point[3])


def transform_surfaces(surfaces, pose, inverse=False):
    transform = to_local if inverse else to_world
    return [Surface(tuple(transform((*p, 0., 0.), pose)[:2] for p in surf.points))
            for surf in surfaces]


class StationPath:
    """Path indexes remain stable when only the untraveled suffix is replaced."""
    def __init__(self, points):
        self.points = list(points)
        self._lengths()
        self.station = 0.0
        self.index = 0
        self.generation = None
        self.window = (0., 0.)

    def _lengths(self):
        if len(self.points) < 2 or not all(math.isfinite(v) for p in self.points for v in p):
            raise ValueError('path requires finite geometry and at least two points')
        self.s = [0.]
        for a, b in zip(self.points, self.points[1:]):
            distance = math.hypot(b[0]-a[0], b[1]-a[1])
            if distance < 1e-9:
                raise ValueError('path contains duplicate points')
            self.s.append(self.s[-1]+distance)

    def at(self, station):
        station = max(0., min(self.s[-1], station))
        i = min(len(self.points)-2, max(0, bisect_right(self.s, station)-1))
        t = (station-self.s[i])/(self.s[i+1]-self.s[i])
        a, b = self.points[i:i+2]
        return (a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]),
                wrap(a[2]+t*wrap(b[2]-a[2])), a[3]+t*(b[3]-a[3]))

    def update(self, position, v_ref, sample_time, generation):
        if self.generation is not None and generation <= self.generation:
            return
        if not all(math.isfinite(v) for v in (*position, sample_time)) or sample_time <= 0:
            raise ValueError('invalid station sample')
        speed = abs(v_ref) if math.isfinite(v_ref) else 0.
        radius = speed*sample_time + .5
        low, high = max(0., self.station-radius), min(self.s[-1], self.station+radius)
        best = None
        first = max(0, bisect_right(self.s, low)-1)
        for i in range(first, len(self.points)-1):
            if self.s[i] > high:
                break
            length = self.s[i+1]-self.s[i]
            t0, t1 = max(0., (low-self.s[i])/length), min(1., (high-self.s[i])/length)
            a, b = self.points[i:i+2]
            dx, dy = b[0]-a[0], b[1]-a[1]
            t = min(t1, max(t0, ((position[0]-a[0])*dx+(position[1]-a[1])*dy)/(length*length)))
            station = self.s[i]+t*length
            distance = math.hypot(position[0]-a[0]-t*dx, position[1]-a[1]-t*dy)
            candidate = (distance, abs(station-self.station), station)
            if best is None or candidate < best:
                best = candidate
        self.station = best[2]
        self.index = min(len(self.points)-1, bisect_right(self.s, self.station)-1)
        self.window, self.generation = (low, high), generation

    def splice(self, tail):
        """Keep the traveled prefix and its station origin on replanning."""
        start = self.at(self.station)
        if math.dist(start[:2], tail[0][:2]) > 1e-6:
            raise ValueError('replacement must start at current station')
        prefix = [p for s, p in zip(self.s, self.points) if s < self.station-1e-8]
        self.points = prefix + list(tail)
        self._lengths()
        self.index = max(0, bisect_right(self.s, self.station)-1)

    def remaining(self, spacing):
        count = max(1, math.ceil((self.s[-1]-self.station)/spacing))
        return [self.at(self.station+(self.s[-1]-self.station)*i/count) for i in range(count+1)]


def connector(start, goal, spacing):
    """Quintic y(x), preserving both end headings with zero end second derivatives."""
    length = goal[0]-start[0]
    if length < spacing or abs(start[2]) >= 1.3 or abs(goal[2]) >= 1.3:
        return []
    delta = goal[1]-start[1]
    m0, m1 = math.tan(start[2])*length, math.tan(goal[2])*length
    c3, c4, c5 = 10*delta-6*m0-4*m1, -15*delta+8*m0+7*m1, 6*delta-3*m0-3*m1
    count = max(2, math.ceil((length+abs(delta))*2/spacing))
    points = []
    for i in range(count+1):
        t = i/count
        y = start[1]+m0*t+c3*t**3+c4*t**4+c5*t**5
        slope = (m0+3*c3*t*t+4*c4*t**3+5*c5*t**4)/length
        second = (6*c3*t+12*c4*t*t+20*c5*t**3)/length**2
        points.append((start[0]+t*length, y, math.atan(slope), second/(1+slope*slope)**1.5))
    return points


def straight(start, length, spacing):
    count = max(1, math.ceil(length/spacing))
    return [(start[0]+length*i/count*math.cos(start[2]),
             start[1]+length*i/count*math.sin(start[2]), start[2], 0.) for i in range(count+1)]


def footprint_clear(points, surfaces, width, front, rear, margin):
    """Check continuous measured edges against each sampled oriented vehicle box.

    Spacing/2 is included in margin by the caller to cover between-sample travel.
    """
    edges = [edge for s in surfaces for edge in s.segments()]
    if not edges or not points:
        return True
    edges = np.asarray(edges, dtype=float)
    # Chunk the path to bound memory even with dense full-resolution scans.
    for offset in range(0, len(points), 32):
        poses = np.asarray(points[offset:offset+32], dtype=float)
        relative = edges[None, :, :, :] - poses[:, None, None, :2]
        c, s = np.cos(poses[:, 2])[:, None, None], np.sin(poses[:, 2])[:, None, None]
        local = (c*relative[..., 0]+s*relative[..., 1],
                 -s*relative[..., 0]+c*relative[..., 1])
        near = np.zeros((len(poses), len(edges)))
        far = np.ones_like(near)
        possible = np.ones_like(near, dtype=bool)
        for coords, low, high in zip(local, (-rear-margin, -width/2-margin),
                                    (front+margin, width/2+margin)):
            a, delta = coords[..., 0], coords[..., 1]-coords[..., 0]
            parallel = np.abs(delta) < 1e-12
            possible &= ~parallel | ((a >= low) & (a <= high))
            denominator = np.where(parallel, 1., delta)
            t0, t1 = (low-a)/denominator, (high-a)/denominator
            near = np.maximum(near, np.where(parallel, -np.inf, np.minimum(t0, t1)))
            far = np.minimum(far, np.where(parallel, np.inf, np.maximum(t0, t1)))
        if np.any(possible & (near <= far)):
            return False
    return True


def observed_free(points, scan, lidar, half_width, front, tolerance):
    """A return connector needs observed free space, not just absent point hits.

    Unknown/invalid beams cannot certify a return. +inf means no hit up to the
    declared scan range. Existing vehicle footprint behind the sensor is exempt.
    """
    for p in points:
        for side in (-half_width, 0., half_width):
            q = to_world((front, side, 0., 0.), p)
            dx, dy = q[0]-lidar[0], q[1]-lidar[1]
            if dx <= 0.05:
                continue
            distance, bearing = math.hypot(dx, dy), math.atan2(dy, dx)
            if abs(bearing) > scan['half_angle'] or distance > scan['range_max']:
                return False
            raw = bearing+scan['front_center']
            # Equivalent angles handle both signed and 0..2pi raw scans.
            indices = [(raw+k*math.tau-scan['angle_min'])/scan['increment'] for k in (-1, 0, 1)]
            index = next((i for i in indices if 0 <= i <= len(scan['ranges'])-1), None)
            if index is None:
                return False
            for i in {math.floor(index), math.ceil(index)}:
                r = scan['ranges'][i]
                if math.isnan(r) or r < scan['range_min'] or r == -math.inf:
                    return False
                if math.isfinite(r) and (r > scan['range_max'] or distance > r-tolerance):
                    return False
    return True
