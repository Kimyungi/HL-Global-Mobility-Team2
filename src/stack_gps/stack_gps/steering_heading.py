"""Bridge an IMU outage with measured speed/road-wheel steering, never commands.

ENU yaw rate = v * tan(sign * steering) / wheelbase.  History permits an IMU
sample to anchor at its acquisition time rather than the slower GPS timer time.
Missing/invalid feedback breaks continuity; later feedback cannot fill that gap.
"""
from collections import deque
import math

from stack_gps.path_engine import wrap_angle


class SteeringHeadingRecovery:
    def __init__(self, wheelbase=.595, steering_sign=-1., timeout=.2,
                 max_steering=math.radians(30.), max_speed=3.):
        if (not all(math.isfinite(x) for x in
                    (wheelbase, steering_sign, timeout, max_steering, max_speed))
                or wheelbase <= 0 or steering_sign not in (-1., 1.)
                or timeout <= 0 or not 0 < max_steering < math.pi / 2
                or max_speed <= 0):
            raise ValueError('Invalid steering heading recovery configuration')
        self.wheelbase, self.steering_sign = wheelbase, steering_sign
        self.timeout, self.max_steering, self.max_speed = timeout, max_steering, max_speed
        self.samples = deque()
        self.pose = None  # (absolute yaw, monotonic sample time)

    def invalidate(self):
        self.pose = None
        self.samples.clear()

    def anchor(self, yaw, t):
        self.pose = (wrap_angle(yaw), t) if math.isfinite(yaw) and math.isfinite(t) else None

    def observe(self, v, steering, t):
        if (not all(math.isfinite(x) for x in (v, steering, t))
                or abs(v) > self.max_speed or abs(steering) > self.max_steering):
            self.invalidate()
            return
        if self.samples and t <= self.samples[-1][0]:
            if t < self.samples[-1][0]:
                self.invalidate()
            return
        rate = v * math.tan(self.steering_sign * steering) / self.wheelbase
        self.samples.append((t, rate))
        if self.pose is not None and t >= self.pose[1]:
            yaw = self.project(t)
            self.pose = (yaw, t) if yaw is not None else None
        # Keep enough past samples for anchoring from the last fresh IMU sample.
        while len(self.samples) > 2 and self.samples[1][0] < t - 2.:
            self.samples.popleft()

    def project(self, t):
        if self.pose is None or not math.isfinite(t):
            return None
        yaw, start = self.pose
        if t < start:
            # Reconnection's IMU sample may precede the latest CAN callback.
            sign, lo, hi = -1., t, start
        else:
            sign, lo, hi = 1., start, t
        previous = None
        integral = 0.
        cursor = lo
        for stamp, rate in self.samples:
            if stamp <= lo:
                previous = (stamp, rate)
                continue
            if stamp > hi:
                break
            if previous is None or stamp - previous[0] > self.timeout + 1e-9:
                return None
            integral += previous[1] * (stamp - cursor)
            cursor, previous = stamp, (stamp, rate)
        if previous is None or hi - previous[0] > self.timeout + 1e-9:
            return None
        integral += previous[1] * (hi - cursor)
        return wrap_angle(yaw + sign * integral)
