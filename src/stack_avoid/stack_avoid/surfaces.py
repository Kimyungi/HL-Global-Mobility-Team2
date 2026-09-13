"""Scan-local 2D obstacle contours and continuous geometry (no ROS dependency).

Only adjacent, valid, nearby returns are connected. A contour is not closed:
the unseen back of an obstacle cannot be reconstructed from one scan.
"""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Surface:
    points: tuple

    def segments(self):
        if len(self.points) == 1:
            yield self.points[0], self.points[0]
        else:
            yield from zip(self.points, self.points[1:])


def scan_surfaces(ranges, angle_min, angle_increment, range_min, range_max,
                  front_center, half_angle, lidar_x, lidar_y,
                  link_distance, link_scale, max_link):
    """Connect neighbors using bounded angular-resolution-aware spacing.

    Invalid/FOV-excluded rays always break a contour; no missing-ray bridging.
    Singletons and two-return obstacles remain occupied, without a size filter.
    """
    if (not ranges or not all(math.isfinite(v) for v in
                             (angle_min, angle_increment, range_min, range_max))
            or angle_increment == 0 or range_max < range_min):
        return []

    def linked(a, b):
        ra = math.hypot(a[0] - lidar_x, a[1] - lidar_y)
        rb = math.hypot(b[0] - lidar_x, b[1] - lidar_y)
        spacing = 2.0 * min(ra, rb) * abs(math.sin(angle_increment / 2.0))
        threshold = min(max_link, max(link_distance, link_scale * spacing))
        return math.dist(a, b) <= threshold

    groups = []
    current = []
    first_valid = last_valid = False
    for i, r in enumerate(ranges):
        angle = angle_min + i * angle_increment - front_center
        rel = math.atan2(math.sin(angle), math.cos(angle))
        valid = (math.isfinite(r) and range_min <= r <= range_max
                 and abs(rel) <= half_angle)
        if i == 0:
            first_valid = valid
        last_valid = valid
        if not valid:
            if current:
                groups.append(current)
                current = []
            continue
        point = (lidar_x + r * math.cos(rel), lidar_y + r * math.sin(rel))
        if current and not linked(current[-1], point):
            groups.append(current)
            current = []
        current.append(point)
    if current:
        groups.append(current)

    # A 360-degree scan may split the front contour at array index zero.
    full_circle = abs(abs(angle_increment) * len(ranges) - math.tau) <= (
        1.5 * abs(angle_increment))
    if (full_circle and first_valid and last_valid and len(groups) > 1
            and linked(groups[-1][-1], groups[0][0])):
        groups[0] = groups.pop() + groups[0]
    return [Surface(tuple(points)) for points in groups]


def clip_segment(a, b, axis, lo, hi):
    """Clip a segment to a closed coordinate slab; retain intersection points."""
    delta = b[axis] - a[axis]
    if abs(delta) < 1e-12:
        return (a, b) if lo <= a[axis] <= hi else None
    t0, t1 = sorted(((lo - a[axis]) / delta, (hi - a[axis]) / delta))
    t0, t1 = max(0.0, t0), min(1.0, t1)
    if t0 > t1:
        return None
    return tuple(tuple(a[j] + t * (b[j] - a[j]) for j in (0, 1))
                 for t in (t0, t1))


def nearest_in_corridor(surfaces, lidar_x, half_width):
    """Closest forward surface/corridor intersection, measured from bumper."""
    nearest = None
    for surface in surfaces:
        for a, b in surface.segments():
            clipped = clip_segment(a, b, 1, -half_width, half_width)
            if clipped is None:
                continue
            clipped = clip_segment(*clipped, 0, lidar_x, math.inf)
            if clipped is None:
                continue
            point = min(clipped, key=lambda p: p[0])
            gap = max(0.0, point[0] - lidar_x)
            if nearest is None or gap < nearest[0]:
                nearest = (gap, point[1])
    return nearest


def blocked_intervals(surfaces, lo_x, hi_x, clearance):
    """Project whole connected contours touching the depth band, then inflate.

    Never clip a contour's y extent to the planning window. Union overlapping
    occupied intervals so points on the same wall cannot form artificial gaps.
    """
    intervals = []
    for surface in surfaces:
        xs, ys = zip(*surface.points)
        if max(xs) >= lo_x and min(xs) <= hi_x:
            intervals.append((min(ys) - clearance, max(ys) + clearance))
    merged = []
    for lo, hi in sorted(intervals):
        # Touching intervals retain their exact-clearance passage candidate.
        if merged and lo < merged[-1][1] - 1e-9:
            merged[-1] = (merged[-1][0], max(merged[-1][1], hi))
        else:
            merged.append((lo, hi))
    return merged


def gap_centers(intervals, offset_max):
    # Ignore wholly unreachable occupied intervals, without shortening contours
    # that cross the window boundary (which would invent an obstacle endpoint).
    intervals = [(lo, hi) for lo, hi in intervals
                 if hi >= -offset_max and lo <= offset_max]
    if not intervals:
        return []
    candidates = [intervals[0][0], intervals[-1][1]]
    candidates.extend((a[1] + b[0]) / 2.0 for a, b in zip(intervals, intervals[1:]))
    return [y for y in candidates if abs(y) <= offset_max]


def outside_intervals(y, intervals):
    return all(y <= lo + 1e-9 or y >= hi - 1e-9 for lo, hi in intervals)


def point_segment_distance(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = (max(0.0, min(1.0, ((point[0] - a[0]) * dx
                           + (point[1] - a[1]) * dy) / length2))
         if length2 else 0.0)
    return math.hypot(point[0] - a[0] - t * dx, point[1] - a[1] - t * dy)


def surface_clearance(point, surfaces):
    return min((point_segment_distance(point, a, b)
                for surface in surfaces for a, b in surface.segments()),
               default=math.inf)


def occluded(origin, target, surfaces, tolerance):
    """Does the sight line hit a continuous contour before reaching target?"""
    dx, dy = target[0] - origin[0], target[1] - origin[1]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return False
    for surface in surfaces:
        for a, b in surface.segments():
            ex, ey = b[0] - a[0], b[1] - a[1]
            ax, ay = a[0] - origin[0], a[1] - origin[1]
            cross = dx * ey - dy * ex
            if abs(cross) < 1e-12:
                # Collinear edges and isolated returns still obstruct sight.
                if abs(ax * dy - ay * dx) > 1e-9 * length:
                    continue
                depths = sorted(((p[0] - origin[0]) * dx / length
                                 + (p[1] - origin[1]) * dy / length for p in (a, b)))
                if depths[1] >= 0 and max(0.0, depths[0]) < length - tolerance:
                    return True
                continue
            t = (ax * ey - ay * ex) / cross
            u = (ax * dy - ay * dx) / cross
            if 0.0 <= t and t * length < length - tolerance and 0.0 <= u <= 1.0:
                return True
    return False
