"""ROS inputs and diagnostic outputs for the persistent avoidance planner."""
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from fma_interfaces.msg import GpsPath, TargetRef

from stack_avoid.path_planner import AvoidPathPlanner
from stack_avoid.station_path import to_world, to_local
from stack_avoid.surfaces import blocked_intervals, gap_centers


def stamp_ns(stamp):
    return stamp.sec*1_000_000_000+stamp.nanosec


class StationPathIO:
    def _init_path_tracking(self):
        readonly = ParameterDescriptor(read_only=True)
        def parameter(name, default):
            return float(self.declare_parameter(name, default, readonly).value)
        self.path_sample_time = parameter('avoid.path_sample_time_s', .1)
        self.path_spacing = parameter('avoid.path_spacing_m', .05)
        self.path_preview = parameter('avoid.path_preview_m', 1.)
        self.path_tail = parameter('avoid.path_tail_m', 3.)
        self.path_pose_timeout = parameter('avoid.path_pose_timeout_s', .5)
        self.path_command_timeout = parameter('avoid.path_command_timeout_s', .5)
        self.path_gps_timeout = parameter('avoid.path_gps_timeout_s', .5)
        self.path_min_radius = parameter('vehicle.min_turn_radius_m', 1.15)
        self.path_front = (parameter('vehicle.wheelbase_m', .595)
                           + parameter('vehicle.front_overhang_m', .165))
        values = (self.path_sample_time, self.path_spacing, self.path_preview,
                  self.path_tail, self.path_pose_timeout, self.path_command_timeout,
                  self.path_gps_timeout, self.path_min_radius,
                  self.vehicle_width, self.vehicle_len)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('path parameters must be finite and positive')
        if not math.isfinite(self.lateral_margin) or self.lateral_margin < 0:
            raise ValueError('lateral margin must be finite and nonnegative')
        if (self.path_spacing > .1 or not math.isfinite(self.path_front)
                or not 0 < self.path_front <= self.vehicle_len):
            raise ValueError('path spacing must be <=0.1m and front offset <= vehicle length')
        self._planner = AvoidPathPlanner(
            width=self.vehicle_width, length=self.vehicle_len, front=self.path_front,
            margin=self.lateral_margin, min_radius=self.path_min_radius,
            spacing=self.path_spacing, preview=self.path_preview, tail_length=self.path_tail)
        self._poses = {}
        self._pose_source = None
        self._gps_goals = {}
        self._gps_stamp = 0
        self._command_stamp, self._command_v = 0, 0.
        self._path_last_scan = 0
        self._completed = False
        self.gps_sub = self.create_subscription(GpsPath, '/perception/gps_path', self._on_gps_path, 1)
        self.target_sub = self.create_subscription(TargetRef, '/adas/target_ref', self._on_path_command, 1)
        self.session_sub = self.create_subscription(Bool, '/operator/start_session', self._on_path_session, 1)
        self.path_pub = self.create_publisher(Path, '/perception/avoid_path', 1)
        self.station_pub = self.create_publisher(Marker, '/perception/avoid_station', 1)

    def _fresh_stamp(self, stamp, timeout):
        age = (self.get_clock().now().nanoseconds-stamp)*1e-9
        return stamp > 0 and 0 <= age <= timeout

    def _on_path_session(self, msg):
        if msg.data:
            self._planner.reset()
            self._pose_source = None
            self._completed = self._detected_prev = False
            self._path_last_scan = 0
            self._gps_goals.clear()

    def _on_path_command(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        if not math.isfinite(msg.v_ref) or not self._fresh_stamp(stamp, self.path_command_timeout):
            self._command_v, self._command_stamp = 0., 0
            return
        if stamp > self._command_stamp:
            self._command_v, self._command_stamp = float(msg.v_ref), stamp

    def _store_vehicle_pose(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        pose = (float(msg.x), float(msg.y), float(msg.yaw))
        if not all(math.isfinite(v) for v in pose) or not self._fresh_stamp(stamp, self.path_pose_timeout):
            self._poses.pop('vehicle', None)
            return
        if stamp > self._poses.get('vehicle', (None, 0))[1]:
            self._poses['vehicle'] = (pose, stamp)

    def _on_gps_path(self, msg):
        stamp = stamp_ns(msg.reference_stamp)
        good = (msg.fix_quality > 0 and msg.position_valid and msg.vehicle_heading_valid
                and self._fresh_stamp(stamp, self.path_gps_timeout)
                and all(math.isfinite(v) for v in
                        (msg.position_x, msg.position_y, msg.vehicle_heading_rad)))
        if not good:
            self._poses.pop('gps', None)
            self._gps_goals.clear()
            return
        if stamp <= self._gps_stamp:
            return
        pose = (msg.position_x, msg.position_y, msg.vehicle_heading_rad)
        self._poses['gps'] = (pose, stamp)
        self._gps_stamp = stamp
        self._gps_goals.clear()
        if len(msg.points) != 1:
            return
        p = msg.points[0]
        point = (p.x, p.y, p.yaw, p.curvature)
        if not all(math.isfinite(v) for v in point) or p.x <= 0:
            return
        self._gps_goals['gps'] = (to_world(point, pose), stamp)
        # Both inputs are vehicle-frame observations. Keep the goal anchored to
        # the pose at receipt; do not reinterpret an old point after ego motion.
        vehicle = self._poses.get('vehicle')
        if vehicle and abs(vehicle[1]-stamp)*1e-9 <= .2:
            self._gps_goals['vehicle'] = (to_world(point, vehicle[0]), min(stamp, vehicle[1]))

    def _path_pose(self):
        if self._pose_source is None or self._planner.path is None:
            source = next((name for name in ('gps', 'vehicle')
                                      if name in self._poses and self._fresh_stamp(
                                          self._poses[name][1], self.path_pose_timeout)), None)
            if source != self._pose_source:
                self._planner.reset()
            self._pose_source = source
        observation = self._poses.get(self._pose_source)
        if observation and self._fresh_stamp(observation[1], self.path_pose_timeout):
            return observation
        return None, 0

    def _path_goals(self, scan, gap):
        x = self.lidar_x+gap
        clear = self.vehicle_width/2+self.lateral_margin
        intervals = blocked_intervals(self._surfaces, x-self.depth_band, x+self.depth_band, clear)
        goals = []
        for y in gap_centers(intervals, self.offset_max):
            # The footprint checker also covers finite sample spacing/rotation;
            # allow a little more room than a tangent endpoint if space permits.
            for extra in (0., self.path_spacing*2, self.path_spacing*4):
                candidate = y+math.copysign(extra, y)
                if (abs(candidate) <= self.offset_max
                        and self._target_clear(scan, x, candidate, intervals, clear)):
                    goals.append((x, candidate))
        return sorted(goals, key=lambda p: abs(p[1]))

    def _station_reference(self, scan, gap, detected):
        pose, pose_stamp = self._path_pose()
        gps = self._gps_goals.get(self._pose_source)
        gps_goal = (to_local(gps[0], pose) if pose is not None and gps
                    and self._fresh_stamp(gps[1], self.path_gps_timeout) else None)
        command = self._command_v if self._fresh_stamp(self._command_stamp, self.path_command_timeout) else 0.
        observation = dict(ranges=scan.ranges, angle_min=scan.angle_min,
                           increment=scan.angle_increment, range_min=scan.range_min,
                           range_max=min(scan.range_max, self.max_range),
                           front_center=self.front_center, half_angle=self.front_half_angle)
        self._planner.width, self._planner.margin = self.vehicle_width, self.lateral_margin
        point, done = self._planner.step(
            pose=pose, surfaces=self._surfaces, scan=observation,
            lidar=(self.lidar_x, self.lidar_y),
            goals=self._path_goals(scan, gap) if detected else [], detected=detected,
            gps_goal=gps_goal, v_ref=command, sample_time=self.path_sample_time,
            generation=pose_stamp)
        generation = min(stamp_ns(scan.header.stamp), pose_stamp)
        if self._planner.mode == 'return' and gps:
            generation = min(generation, gps[1])
        self._publish_path(scan, pose)
        return point, done, generation

    def _publish_path(self, scan, pose):
        path = Path()
        path.header.stamp = scan.header.stamp
        path.header.frame_id = 'base_link'
        marker = Marker()
        marker.header = path.header
        marker.ns, marker.id = 'avoid_station', 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.orientation.w = 1.
        marker.scale.z = .15
        marker.color.g = marker.color.a = 1.
        marker.action = Marker.DELETE
        if self._planner.path is not None and pose is not None:
            route = self._planner.path
            for point in route.points:
                x, y, yaw, _ = to_local(point, pose)
                p = PoseStamped()
                p.header = path.header
                p.pose.position.x, p.pose.position.y = x, y
                p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                path.poses.append(p)
            marker.action = Marker.ADD
            p = to_local(route.at(route.station), pose)
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = p[0], p[1], .4
            marker.text = (f'{self._planner.mode} | station {route.station:.2f}m | index {route.index}\n'
                           f'window [{route.window[0]:.2f}, {route.window[1]:.2f}] | +{self.path_preview:.2f}m')
        self.path_pub.publish(path)
        self.station_pub.publish(marker)
