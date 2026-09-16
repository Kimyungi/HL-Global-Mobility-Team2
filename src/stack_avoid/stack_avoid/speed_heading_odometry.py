"""Short-encounter odometry from signed speed and yaw; RX x/y are not used."""
import math
from stack_avoid.station_path import wrap


class SpeedHeadingOdometry:
    def __init__(self):
        self.reset()

    def reset(self):
        self.pose = None
        self.stamp = None
        self.speed = 0.
        self.lost = False

    def update(self, stamp, speed, yaw):
        if self.lost or stamp <= 0 or not all(math.isfinite(v) for v in (speed, yaw)):
            return None
        if self.stamp is None:
            self.pose = (0., 0., wrap(yaw))
        elif stamp <= self.stamp:
            return None
        else:
            dt = (stamp-self.stamp)*1e-9
            if dt > .5:
                # Motion across an unobserved interval cannot be certified.
                # Keep path geometry, require a new episode to establish its pose.
                self.lost = True
                return None
            heading = self.pose[2]+wrap(yaw-self.pose[2])/2.
            distance = (self.speed+speed)*.5*dt
            self.pose = (self.pose[0]+distance*math.cos(heading),
                         self.pose[1]+distance*math.sin(heading), wrap(yaw))
        self.stamp, self.speed = stamp, speed
        return self.pose, stamp, speed
