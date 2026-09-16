"""ROS inputs and diagnostic outputs for the persistent avoidance planner."""
import math
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from fma_interfaces.msg import GpsPath, MgmState

from stack_avoid.gps_cubic_path import WaypointWindow
from stack_avoid.station_path import to_world, to_local
from stack_avoid.surfaces import gap_centers, surface_goal_sections, surface_intervals
from stack_avoid.compute_backend import compute_functions
from stack_avoid.fixed_obstacle_path import FixedObstaclePlanner, GoalGroup, obstacle_bands


def stamp_ns(stamp):
    return stamp.sec*1_000_000_000+stamp.nanosec


class StationPathIO:
    def _init_path_tracking(self):
        readonly = ParameterDescriptor(read_only=True)
        def parameter(name, default):
            return float(self.declare_parameter(name, default, readonly).value)
        self.path_spacing = parameter('avoid.path_spacing_m', .05)
        self.path_anchor = parameter('avoid.waypoint_anchor_m', 1.)
        self.path_return = parameter('avoid.waypoint_return_m', 2.7)
        self.path_pose_timeout = parameter('avoid.path_pose_timeout_s', .5)
        self.path_gps_timeout = parameter('avoid.path_gps_timeout_s', .5)
        self.require_mgm_active = bool(self.declare_parameter(
            'avoid.require_mgm_active', False, readonly).value)
        self.compute_backend = self.declare_parameter('avoid.compute_backend', 'python', readonly).value
        connector, collision_check = compute_functions(self.compute_backend)
        self.get_logger().info(f'avoid compute backend: {self.compute_backend}')
        self._mgm_active, self._mgm_stamp = False, 0
        self.path_min_radius = parameter('vehicle.min_turn_radius_m', 1.15)
        self.path_front = (parameter('vehicle.wheelbase_m', .595)
                           + parameter('vehicle.front_overhang_m', .165))
        values = (self.path_spacing,
                  self.path_anchor, self.path_return, self.path_pose_timeout,
                  self.path_gps_timeout, self.path_min_radius,
                  self.vehicle_width, self.vehicle_len)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('path parameters must be finite and positive')
        if not math.isfinite(self.lateral_margin) or self.lateral_margin < 0:
            raise ValueError('lateral margin must be finite and nonnegative')
        if (self.path_spacing > .1 or not math.isfinite(self.path_front)
                or not 0 < self.path_front <= self.vehicle_len):
            raise ValueError('path spacing must be <=0.1m and front offset <= vehicle length')
        self._planner = FixedObstaclePlanner(
            width=self.vehicle_width, length=self.vehicle_len, front=self.path_front,
            margin=self.lateral_margin, min_radius=self.path_min_radius,
            spacing=self.path_spacing, anchor_distance=self.path_anchor, return_distance=self.path_return,
            connector=connector, collision_check=collision_check)
        self._poses = {}
        self._pose_source = None
        self._gps_waypoints = None
        self._gps_route_key = None
        self._gps_stamp = 0
        self._path_last_scan = 0
        self._completed = False
        self.gps_sub = self.create_subscription(GpsPath, '/perception/gps_path', self._on_gps_path, 1)
        self.mgm_sub = self.create_subscription(MgmState, '/adas/mgm_state', self._on_mgm_state, 1)
        self.session_sub = self.create_subscription(Bool, '/operator/start_session', self._on_path_session, 1)
        self.path_pub = self.create_publisher(Path, '/perception/avoid_path', 1)
        self.station_pub = self.create_publisher(Marker, '/perception/avoid_station', 1)

    def _fresh_stamp(self, stamp, timeout):
        age = (self.get_clock().now().nanoseconds-stamp)*1e-9
        return stamp > 0 and 0 <= age <= timeout

    def _on_mgm_state(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        if stamp <= self._mgm_stamp or not self._fresh_stamp(stamp, self.path_pose_timeout):
            return
        active = msg.avoidance == 1  # AVOID_ACTIVE; GPS_RETURN is owned by GPS.
        if self.require_mgm_active and active != self._mgm_active:
            # A new marker must not inherit a path/done from an earlier encounter.
            self._planner.reset()
            self._completed = False
        self._mgm_active, self._mgm_stamp = active, stamp

    def _on_path_session(self, msg):
        if msg.data:
            self._planner.reset()
            self._pose_source = None
            self._completed = self._detected_prev = False
            self._path_last_scan = 0
            self._gps_waypoints = None

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
                and msg.heading_source != GpsPath.HEADING_TANGENT
                and self._fresh_stamp(stamp, self.path_gps_timeout)
                and all(math.isfinite(v) for v in
                        (msg.position_x, msg.position_y, msg.vehicle_heading_rad)))
        if not good:
            self._poses.pop('gps', None)
            self._gps_waypoints = None
            return
        route = msg.route
        key = (route.sequence_id, route.instance_id, route.index,
               route.connecting, route.acknowledged_request, route.waypoint_csv)
        if key != self._gps_route_key:
            self._planner.reset()
            self._completed = False
            self._gps_route_key = key
            self._gps_stamp = 0
        if stamp <= self._gps_stamp:
            return
        pose = (msg.position_x, msg.position_y, msg.vehicle_heading_rad)
        self._poses['gps'] = (pose, stamp)
        self._pose_source = 'gps'
        self._gps_stamp = stamp
        self._gps_waypoints = None
        if not msg.waypoint_window_valid:
            return
        try:
            points = [to_world((p.x, p.y, p.yaw, p.curvature), pose)
                      for p in msg.waypoint_points]
            self._gps_waypoints = WaypointWindow(msg.waypoint_stations, points,
                                                 msg.waypoint_station_m)
        except ValueError:
            pass  # Invalid window cannot be replaced with a navigation preview.

    def _path_pose(self):
        # GPS and its waypoint station window define the path's only frame.
        # CAN x/y/yaw are not a fallback for a missing GPS sample.
        self._pose_source = 'gps'
        observation = self._poses.get('gps')
        if observation and self._fresh_stamp(observation[1], self.path_pose_timeout):
            return observation
        return None, 0

    def _path_goals(self, scan, source_surfaces=None):
        clear = self.vehicle_width/2+self.lateral_margin
        sections = surface_goal_sections(
            self._surfaces if source_surfaces is None else source_surfaces, self.lidar_x,
            (self.lidar_x+self.detect_range+self.detect_hysteresis
             if source_surfaces is None else self.lidar_x+self.max_range),
            self.offset_max+clear, clear)
        goals = []
        for x in sections:
            intervals = surface_intervals(self._surfaces, x, clear)
            # Both sides and real openings between all faces at this section.
            stage = []
            for y in gap_centers(intervals, self.offset_max):
                for extra in (0., .05, .1, .15, .2, .3):
                    candidate = y+math.copysign(extra, y)
                    if any(abs(x-gx) < 1e-8 and abs(candidate-gy) < 1e-8
                           for gx, gy in stage):
                        continue
                    if (abs(candidate) <= self.offset_max
                            and self._target_clear(scan, x, candidate, intervals, clear)):
                        stage.append((x, candidate))
            goals.extend(sorted(stage, key=lambda p: abs(p[1])))
        return goals

    def _path_goal_groups(self, scan, pose):
        if pose is None or self._gps_waypoints is None or len(self._planner.fixed_goals) >= 2:
            return []
        groups = []
        for low, high, contours in obstacle_bands(
                self._surfaces, pose, self._gps_waypoints, self.detect_half_width):
            if not self._planner.wants_group(low, high):
                continue
            groups.append(GoalGroup(low, high, tuple(self._path_goals(scan, contours))))
            if len(groups) >= 2-len(self._planner.fixed_goals):
                break
        return groups

    def _station_reference(self, scan, gap, detected):
        pose, pose_stamp = self._path_pose()
        if self.require_mgm_active and (not self._mgm_active or
                not self._fresh_stamp(self._mgm_stamp, self.path_pose_timeout)):
            self._planner.path = self._planner.control_target = None
            self._planner.reason = 'waiting for fresh MGM AVOID_ACTIVE'
            self._publish_path(scan, None)
            return None, False, 0
        self._planner.width, self._planner.margin = self.vehicle_width, self.lateral_margin
        started = time.perf_counter()
        fixed = isinstance(self._planner, FixedObstaclePlanner)
        groups = self._path_goal_groups(scan, pose) if fixed and (detected or self._planner.episode_active) else []
        point, done = self._planner.step(
            pose=pose, waypoints=self._gps_waypoints if pose is not None else None,
            surfaces=self._surfaces,
            goals=self._path_goals(scan) if detected and not fixed else [], detected=detected,
            **({'goal_groups': groups} if fixed else {}))
        elapsed_ms = (time.perf_counter() - started) * 1000
        if started - getattr(self, '_compute_log_time', -math.inf) >= 5.0:
            self.get_logger().info(
                f'avoid compute: backend={getattr(self, "compute_backend", "python")}; '
                f'planning_ms={elapsed_ms:.3f}; target={point is not None}; '
                f'updates={self._planner.replan_count}; '
                f'fixed_goals={len(getattr(self._planner, "fixed_goals", ()))}; '
                f'passed_goals={getattr(self._planner, "passed_goals", 0)}')
            self._compute_log_time = started
        generation = min(stamp_ns(scan.header.stamp), pose_stamp)
        diagnostic = (self._planner.reason, point is not None, self._planner.anchor_fallback,
                      self._planner.goal_side, self._planner.side_switched)
        if diagnostic != getattr(self, '_path_diagnostic', None):
            self.get_logger().info(
                f'avoid planner: {self._planner.reason}; target={point is not None}; '
                f'pose=gps; GPS_station={self._planner.gps_station}; '
                f'anchor={self._planner.anchor_station}; current_pose_fallback={self._planner.anchor_fallback}; '
                f'side={self._planner.goal_station}; '
                f'return={self._planner.return_station}; updates={self._planner.replan_count}; '
                f'goal_side={self._planner.goal_side}; side_switched={self._planner.side_switched}')
            self._path_diagnostic = diagnostic
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
            marker.text = (f'{self._planner.mode} | GPS s={self._planner.gps_station:.2f}m\n'
                           f'anchor={self._planner.anchor_station:.2f} | '
                           f'side={self._planner.goal_station} | return={self._planner.return_station}')
            if isinstance(self._planner, FixedObstaclePlanner):
                marker.text += '\nfixed stations=' + ','.join(
                    f'{g.station:.2f}' for g in self._planner.fixed_goals)
                marker.text += f' | passed={self._planner.passed_goals}'
        self.path_pub.publish(path)
        self.station_pub.publish(marker)
