"""Continuous station tracking on ordered CSV segments (no ROS dependency)."""
from bisect import bisect_left, bisect_right
import math


PREVIEW_M = 2.5
SNAP_WEIGHT = 0.9


def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class StationPath:
    def __init__(self, east, north, yaw, curvature):
        self.e, self.n, self.yaw, self.curvature = map(tuple, (east, north, yaw, curvature))
        if len(self.e) < 2 or any(len(a) != len(self.e) for a in (self.n, self.yaw, self.curvature)):
            raise ValueError('station path requires matching arrays and at least two points')
        if not all(math.isfinite(v) for a in (self.e, self.n, self.yaw, self.curvature) for v in a):
            raise ValueError('station path requires finite geometry')
        self.s = [0.0]
        for i in range(len(self.e) - 1):
            length = math.hypot(self.e[i+1] - self.e[i], self.n[i+1] - self.n[i])
            if length <= 0.0:
                raise ValueError('station path contains consecutive duplicate points')
            self.s.append(self.s[-1] + length)
        self.reset()

    def reset(self):
        self.station = self.index = self.generation = None
        self.window_low = self.window_high = 0.0

    def _project(self, east, north, segments, low, high):
        best = None
        for i in segments:
            length = self.s[i+1] - self.s[i]
            t_min = max(0.0, (low - self.s[i]) / length)
            t_max = min(1.0, (high - self.s[i]) / length)
            if t_min > t_max:
                continue
            de, dn = self.e[i+1] - self.e[i], self.n[i+1] - self.n[i]
            t = ((east - self.e[i])*de + (north - self.n[i])*dn) / (length*length)
            t = min(t_max, max(t_min, t))
            pe, pn = self.e[i] + t*de, self.n[i] + t*dn
            station = self.s[i] + t*length
            distance = math.hypot(east - pe, north - pn)
            movement = abs(station - self.station) if self.station is not None else 0.0
            candidate = ((distance, movement, station), i, pe, pn)
            if best is None or candidate[0] < best[0]:
                best = candidate
        if best is None:
            raise ValueError('empty station window')
        return best

    def update(self, east, north, *, v_ref, sample_time, generation):
        if generation is None or not all(math.isfinite(v) for v in (east, north, sample_time, generation)) or sample_time <= 0:
            raise ValueError('invalid station sample')
        if self.generation is not None and generation <= self.generation:
            return  # A timer publication cannot move the search window a second time.
        last_segment = len(self.e) - 2
        if self.station is None:
            nearest = min(range(len(self.e)), key=lambda i: math.hypot(self.e[i]-east, self.n[i]-north))
            segments = range(max(0, nearest-1), min(last_segment, nearest)+1)
            low, high = 0.0, self.s[-1]
        else:
            radius = abs(v_ref) * sample_time * 2.0 if math.isfinite(v_ref) else 0.0
            if not math.isfinite(radius):
                radius = 0.0
            low, high = max(0.0, self.station-radius), min(self.s[-1], self.station+radius)
            first = max(0, bisect_right(self.s, low)-1)
            last = min(last_segment, bisect_left(self.s, high))
            segments = range(min(first, last_segment), last+1)
        best, segment, pe, pn = self._project(east, north, segments, low, high)
        self.station = best[2]
        # Closest CSV vertex to the bounded station; even the index stays put at zero speed.
        self.index = min((segment, segment+1),
                         key=lambda i: (math.hypot(self.e[i]-pe, self.n[i]-pn), i))
        self.generation = generation
        self.window_low, self.window_high = low, high

    def position(self):
        if self.station is None:
            raise ValueError('station is not initialized')
        i = min(len(self.e)-2, max(0, bisect_right(self.s, self.station)-1))
        t = (self.station-self.s[i]) / (self.s[i+1]-self.s[i])
        return self.e[i] + t*(self.e[i+1]-self.e[i]), self.n[i] + t*(self.n[i+1]-self.n[i])

    def preview(self):
        if self.station is None:
            raise ValueError('station is not initialized')
        requested = min(self.s[-1], self.station + PREVIEW_M)
        i = min(len(self.e)-2, max(0, bisect_right(self.s, requested)-1))
        t = (requested-self.s[i]) / (self.s[i+1]-self.s[i])
        # Tiny tolerance only resolves floating-point representation of exactly 90%.
        snapped = t <= 1.0-SNAP_WEIGHT+1e-12 or t >= SNAP_WEIGHT-1e-12
        if t <= 1.0-SNAP_WEIGHT+1e-12:
            t = 0.0
        elif t >= SNAP_WEIGHT-1e-12:
            t = 1.0
        point = (self.e[i] + t*(self.e[i+1]-self.e[i]),
                 self.n[i] + t*(self.n[i+1]-self.n[i]),
                 _wrap(self.yaw[i] + t*_wrap(self.yaw[i+1]-self.yaw[i])),
                 self.curvature[i] + t*(self.curvature[i+1]-self.curvature[i]))
        return point, {'station_m': self.station, 'preview_requested_station_m': requested,
                       'preview_station_m': self.s[i] + t*(self.s[i+1]-self.s[i]),
                       'preview_snapped': snapped, 'preview_right_weight': t,
                       'station_window_low_m': self.window_low,
                       'station_window_high_m': self.window_high}
