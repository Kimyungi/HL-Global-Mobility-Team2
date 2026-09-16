"""main 34fbfc0 gap planner, adapted to v2 metadata and zone ownership.

main_gap_original.py and params_main_gap.yaml are unchanged snapshots from that
commit. main's assembler interpolates 20 points, but the unchanged v5 CAN bridge
transmits only the first: (goal.x/20, goal.y/20, atan2(y,x), 0).
"""
import math
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from fma_interfaces.msg import AvoidStatus, GpsPath, MgmState

from stack_avoid.main_gap_original import StackAvoidNode as MainNode
from stack_avoid.gps_cubic_path import WaypointWindow


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class _Publisher:
    def __init__(self, owner):
        self.owner = owner

    def publish(self, msg):
        self.owner._publish_adapted(msg)


class MainGapTrialNode(MainNode):
    def __init__(self):
        super().__init__()
        self.require_active = self.declare_parameter(
            'avoid.require_mgm_active', True, ParameterDescriptor(read_only=True)).value
        self._active = False
        self._active_stamp = self._scan_stamp = self._last_scan = 0
        self._gps = None
        self._route_key = None
        self._encountered = False
        self._output = self.pub
        self.pub = _Publisher(self)
        self.raw_pub = self.create_publisher(AvoidStatus, '/perception/avoid_main_goal', 1)
        self.path_pub = self.create_publisher(Path, '/perception/avoid_path', 1)
        self.gps_sub = self.create_subscription(GpsPath, '/perception/gps_path', self._on_gps, 1)
        self.mgm_sub = self.create_subscription(MgmState, '/adas/mgm_state', self._on_mgm, 1)
        self.session_sub = self.create_subscription(Bool, '/operator/start_session', self._on_session, 1)
        if getattr(self, 'MODE', 'main_gap') == 'main_gap':
            self.get_logger().info('avoid planner mode: main_gap; source=34fbfc0; wire=goal/20, atan2, curvature=0')

    def _fresh(self, stamp):
        return stamp > 0 and 0 <= (self.get_clock().now().nanoseconds-stamp)*1e-9 <= .5

    def _reset_episode(self):
        self._maneuver_armed = self._encountered = self._detected_prev = False
        self._clear_since = self._last_gap = None
        self._prev_center = self._prev_center_t = None
        self._done_until = 0.

    def _on_session(self, msg):
        if msg.data:
            self._reset_episode()
            self._gps = None

    def _on_mgm(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        if stamp <= self._active_stamp or not self._fresh(stamp):
            return
        active = msg.avoidance == 1
        if self.require_active and active != self._active:
            self._reset_episode()
        self._active, self._active_stamp = active, stamp

    def _on_gps(self, msg):
        key = (msg.route.sequence_id, msg.route.instance_id, msg.route.index,
               msg.route.connecting, msg.route.acknowledged_request, msg.route.waypoint_csv)
        if key != self._route_key:
            self._reset_episode()
            self._route_key = key
        self._gps = msg

    def _waiting_point(self):
        gps = self._gps
        if not (gps and self._fresh(stamp_ns(gps.reference_stamp)) and gps.fix_quality > 0
                and gps.position_valid and gps.vehicle_heading_valid
                and gps.heading_source != GpsPath.HEADING_TANGENT and gps.waypoint_window_valid):
            return None
        try:
            window = WaypointWindow(gps.waypoint_stations,
                [(p.x, p.y, p.yaw, p.curvature) for p in gps.waypoint_points], gps.waypoint_station_m)
            point = window.at(window.station+1.)
            return self._rp(*point) if point else None
        except ValueError:
            return None

    def on_scan(self, scan):
        stamp = stamp_ns(scan.header.stamp)
        if stamp > 0 and stamp <= self._last_scan:
            return
        valid = bool(scan.ranges and self._fresh(stamp)
            and math.isfinite(scan.angle_min) and math.isfinite(scan.angle_increment)
            and scan.angle_increment != 0 and math.isfinite(scan.range_min)
            and math.isfinite(scan.range_max) and scan.range_max >= scan.range_min
            and any(r == math.inf or (math.isfinite(r) and scan.range_min <= r <= scan.range_max)
                    for r in scan.ranges))
        if not valid:
            self._clear_since = None  # Unknown scans cannot count toward main's clearance timer.
            msg = AvoidStatus(obstacle_detected=self._detected_prev, ttc=1e9,
                              v_suggest=float(self.target_speed))
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            self._output.publish(msg)
            self._emit_path(Path(header=msg.header))
            return
        if self._last_scan and stamp-self._last_scan > 500_000_000:
            self._clear_since = None
        self._last_scan = self._scan_stamp = stamp
        started = time.perf_counter()
        super().on_scan(scan)
        self.get_logger().info(
            f'avoid {getattr(self, "MODE", "main_gap")}: planning_ms={(time.perf_counter()-started)*1000:.3f}; '
            f'armed={self._maneuver_armed}; encountered={self._encountered}', throttle_duration_sec=5.)

    def _emit_path(self, path):
        self.path_pub.publish(path)

    def _publish_adapted(self, msg):
        active = not self.require_active or (self._active and self._fresh(self._active_stamp))
        msg.scan_valid = True
        if not active:
            self._reset_episode()
            msg.points = []
            msg.maneuver_done = False
            msg.avoidable = False
        else:
            self._encountered |= msg.obstacle_detected
        self.raw_pub.publish(msg)
        path = Path(header=msg.header)
        for p in msg.points:
            origin, target = PoseStamped(header=msg.header), PoseStamped(header=msg.header)
            origin.pose.orientation.w = target.pose.orientation.w = 1.
            target.pose.position.x, target.pose.position.y = p.x, p.y
            path.poses = [origin, target]
        if msg.points:
            goal = msg.points[0]
            msg.points = [self._rp(goal.x/20., goal.y/20., math.atan2(goal.y, goal.x), 0.)]
            msg.reference_stamp.sec, msg.reference_stamp.nanosec = divmod(self._scan_stamp, 1_000_000_000)
        elif active and not self._encountered:
            point = self._waiting_point()
            if point:
                msg.points = [point]  # Zone entry before an obstacle uses v2 GPS station+1m.
                generation = min(self._scan_stamp, stamp_ns(self._gps.reference_stamp))
                msg.reference_stamp.sec, msg.reference_stamp.nanosec = divmod(generation, 1_000_000_000)
        self._emit_path(path)
        self._output.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MainGapTrialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
