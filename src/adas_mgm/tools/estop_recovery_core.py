"""PR108 recovery sequence, driven by measured inputs; independent of ROS."""
import math


def fresh(stamp, now, timeout=.25):
    return math.isfinite(stamp) and 0 <= now-stamp <= timeout


class Recovery:
    HOLD_SECONDS = 6.
    REVERSE_SECONDS = 5.
    DISTANCE = 1.
    SPEED = -.3

    def __init__(self):
        self.armed = False
        self._seen_inactive = False
        self.request = 0
        self.phase = 'IDLE'
        self.distance = 0.
        self.hold_since = None
        self.reverse_since = None
        self.last_now = None
        self.last_speed_stamp = None
        self.last_speed = 0.
        self.reason = ''

    def fault(self, reason):
        self.phase, self.reason = 'FAULT', reason
        return 0., False

    def step(self, now, stamp_now, request, active, authorized,
             speed, speed_stamp, rear_clear, rear_stamp, *, state_valid=True):
        speed_ok = math.isfinite(speed) and fresh(speed_stamp, stamp_now)
        rear_ok = rear_clear and fresh(rear_stamp, stamp_now, .35)
        if not active:
            if state_valid:
                self._seen_inactive = True
            self.request, self.phase = 0, 'IDLE'
            self.last_now = now
            return 0., False
        if request <= 0:
            return 0., False
        if request != self.request:
            # A newly observed ESTOP entry arms recovery even before any motion.
            # Starting/restarting inside an existing episode must not repeat
            # a partly completed reverse with its travelled distance forgotten.
            self.armed = self._seen_inactive
            self.request, self.phase = request, 'HOLD'
            self.distance, self.hold_since, self.reverse_since = 0., None, None
            self.last_speed_stamp = None
            self.reason = ''
        if self.last_now is not None and now < self.last_now:
            return self.fault('clock moved backwards')
        self.last_now = now
        if self.phase == 'FAULT':
            return 0., False
        if not authorized or not speed_ok:
            if self.phase in ('REVERSE','SETTLE','DONE'):
                return self.fault('authority or measured speed lost')
            self.hold_since = None
            self.reason = 'waiting for authority and measured stop'
            return 0., False
        if self.phase == 'HOLD':
            if not self.armed or abs(speed) > .02:
                self.hold_since = None
                self.reason = 'not armed or not stationary'
                return 0., False
            if self.hold_since is None:
                self.hold_since = now
            if now-self.hold_since < self.HOLD_SECONDS or not rear_ok:
                self.reason = 'holding 6 seconds / waiting for observed rear clearance'
                return 0., False
            self.phase, self.reverse_since = 'REVERSE', now
            self.last_speed_stamp, self.last_speed = speed_stamp, speed
            self.reason = ''
            return self.SPEED, False
        if not rear_ok:
            return self.fault('rear corridor blocked or unknown')
        if self.phase == 'REVERSE':
            if speed > .02:
                return self.fault('motion opposite to reverse request')
            if speed_stamp < self.last_speed_stamp:
                return self.fault('speed timestamp moved backwards')
            if speed_stamp > self.last_speed_stamp:
                dt = speed_stamp-self.last_speed_stamp
                if dt > .25:
                    return self.fault('speed feedback gap')
                # New measured samples only. Repeated publishes cannot add distance.
                self.distance += (max(0.,-speed)+max(0.,-self.last_speed))*.5*dt
                self.last_speed_stamp, self.last_speed = speed_stamp,speed
            if self.distance >= self.DISTANCE:
                self.phase = 'SETTLE'
                return 0., False
            if now-self.reverse_since >= self.REVERSE_SECONDS:
                return self.fault('five second limit before one metre')
            return self.SPEED, False
        if self.phase == 'SETTLE':
            if abs(speed) <= .02 and speed_stamp > self.last_speed_stamp:
                self.phase = 'DONE'
        return 0., self.phase == 'DONE'


def rear_corridor_clear(ranges, angle_min, increment, range_min, range_max, mount,
                        body_rear=.09, half_width=.31, distance=1.25, margin=.10):
    """Conservative full reverse corridor, calibrated a2 rays in body frame.

    Unknown/NaN/inf does not certify clearance. Require measured coverage at
    five bearings across the far edge, and no valid return within the corridor.
    """
    if (len(mount) != 8 or not ranges or
            not all(math.isfinite(v) for v in (*mount, angle_min, increment, range_min, range_max)) or
            increment <= 0):
        return False
    x0,y0,yaw,fov_min,fov_max,offset,minimum,maximum = mount
    yaw = math.radians(yaw)
    width = half_width+margin
    observations = []
    for i, raw in enumerate(ranges):
        angle = angle_min+i*increment
        degrees = math.degrees(math.remainder(angle,2*math.pi))
        if degrees < fov_min:
            degrees += 360.
        if not fov_min <= degrees <= fov_max:
            continue
        if not math.isfinite(raw) or not max(range_min,minimum) <= raw <= min(range_max,maximum):
            continue
        r = raw+offset
        x,y = x0+r*math.cos(angle+yaw),y0+r*math.sin(angle+yaw)
        if -body_rear-distance <= x < -body_rear and abs(y) <= width:
            return False
        observations.append((angle,r))
    for y in (-width,-width/2,0.,width/2,width):
        x = -body_rear-distance
        target = math.atan2(y-y0,x-x0)-yaw
        required = math.hypot(x-x0,y-y0)
        if not any(abs(math.remainder(a-target,2*math.pi)) <= increment*.51 and r >= required
                   for a,r in observations):
            return False
    return True
