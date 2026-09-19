"""main gap selection -> two cubic connectors -> persistent path preview."""
import math

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from fma_interfaces.msg import GpsPath

from stack_avoid.main_gap_trial import MainGapTrialNode, stamp_ns
from stack_avoid.compute_backend import compute_functions
from stack_avoid.three_point_path import ThreePointPlanner
from stack_avoid.surfaces import scan_surfaces


class MainGapPathTrialNode(MainGapTrialNode):
    MODE = 'main_gap_path'

    def __init__(self):
        super().__init__()
        readonly = ParameterDescriptor(read_only=True)
        def value(name, default):
            return self.declare_parameter(name, default, readonly).value
        self.backend = value('avoid.compute_backend', 'native')
        connector, checker = compute_functions(self.backend)
        self.preview = float(value('avoid.path_preview_m', 1.))
        spacing = float(value('avoid.path_spacing_m', .05))
        distance = float(value('avoid.path_return_m', 2.7))
        radius = float(value('vehicle.min_turn_radius_m', 1.15))
        front = float(value('vehicle.wheelbase_m', .595))+float(value('vehicle.front_overhang_m', .165))
        if not all(math.isfinite(v) and v > 0 for v in (self.preview, spacing, distance, radius)) or spacing > .1:
            raise ValueError('invalid A-B-C path parameters')
        self._curve_planner = ThreePointPlanner(
            width=self.vehicle_width, length=self.vehicle_len, front=front,
            margin=self.lateral_margin, min_radius=radius, spacing=spacing,
            return_distance=distance, connector=connector, collision_check=checker)
        self._current_scan = None
        self._last_reason = None
        self.get_logger().info(
            f'avoid planner mode: main_gap_path; backend={self.backend}; '
            f'A=(0,0); C=(obstacle_x+{distance},0); preview={self.preview}m; '
            'pose=gps_map_enu; path_frame=map; path_hold=until_C')

    def _reset_episode(self):
        super()._reset_episode()
        if hasattr(self, '_curve_planner'):
            self._curve_planner.reset()

    def _gps_observation(self):
        gps = self._gps
        if not (gps and self._fresh(stamp_ns(gps.reference_stamp)) and gps.fix_quality > 0
                and gps.position_valid and gps.vehicle_heading_valid
                and gps.heading_source != GpsPath.HEADING_TANGENT
                and all(math.isfinite(v) for v in
                        (gps.position_x, gps.position_y, gps.vehicle_heading_rad))):
            return None
        # Same ENU origin and heading as GPS waypoint visualization and map TF.
        return ((gps.position_x, gps.position_y, gps.vehicle_heading_rad),
                stamp_ns(gps.reference_stamp), self._ego_speed())

    def _emit_path(self, path):
        # Preserve every original sample in map, including traveled points. Only
        # the separate control preview is transformed to current base_link.
        fixed = Path()
        fixed.header.stamp = path.header.stamp
        fixed.header.frame_id = 'map'
        planner = getattr(self, '_curve_planner', None)
        if planner is not None and planner.path is not None:
            for x, y, yaw, _ in planner.path.points:
                p = PoseStamped(header=fixed.header)
                p.pose.position.x, p.pose.position.y = x, y
                p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                fixed.poses.append(p)
        self.path_pub.publish(fixed)

    def on_scan(self, scan):
        self._current_scan = scan
        super().on_scan(scan)

    def _gap_target(self, scan, gap):
        if self._curve_planner.path is not None:
            return None  # Do not reselect the fixed B on each scan.
        return super()._gap_target(scan, gap)

    def _publish_adapted(self, msg):
        active = not self.require_active or (self._active and self._fresh(self._active_stamp))
        if not active:
            # Pre-supply the GPS waiting point while there is no obstacle so
            # the zone edge need not reset the speed ramp for an empty provider.
            # A stale active-state heartbeat never releases a frozen episode.
            msg.scan_valid = True
            msg.points = []
            msg.maneuver_done = msg.avoidable = False
            if not self._active and not msg.obstacle_detected:
                point = self._waiting_point()
                if point:
                    msg.points = [point]
                    generation = min(self._scan_stamp, stamp_ns(self._gps.reference_stamp))
                    msg.reference_stamp.sec, msg.reference_stamp.nanosec = divmod(generation, 1_000_000_000)
            self._maneuver_armed = False
            self._clear_since = self._last_gap = None
            self._done_until = 0.
            self.raw_pub.publish(msg)
            self._emit_path(Path(header=msg.header))
            self._output.publish(msg)
            return
        planner = self._curve_planner
        self._encountered |= msg.obstacle_detected
        self.raw_pub.publish(msg)
        goal = msg.points[0] if msg.obstacle_detected and msg.points else None
        msg.scan_valid = True
        msg.points = []
        msg.maneuver_done = False  # Only reaching C completes this path, never main's timer.
        local_path = []
        observation = self._gps_observation()
        scan = self._current_scan
        surfaces = [] if planner.path is not None else scan_surfaces(scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, min(scan.range_max, self.max_range), self.front_center,
            self.front_half_angle, self.lidar_x, self.lidar_y, .1, 3., .3)
        planner.width, planner.margin = self.vehicle_width, self.lateral_margin
        if not self._encountered:
            point = self._waiting_point()
            if point:
                msg.points = [point]
                generation = min(self._scan_stamp, stamp_ns(self._gps.reference_stamp))
                msg.reference_stamp.sec, msg.reference_stamp.nanosec = divmod(generation, 1_000_000_000)
        elif observation:
            pose, pose_stamp, speed = observation
            if planner.path is None and goal:
                planner.create((goal.x, goal.y), pose, surfaces, pose_stamp, self.offset_max)
            point, done, local_path = planner.track(
                pose, pose_stamp, speed, surfaces, msg.obstacle_detected, self.preview)
            msg.maneuver_done = done
            if point:
                msg.points = [self._rp(*point)]
            if point or done:
                generation = min(self._scan_stamp, pose_stamp)
                msg.reference_stamp.sec, msg.reference_stamp.nanosec = divmod(generation, 1_000_000_000)
        else:
            planner.reason = 'waiting for fresh GPS map position and measured heading'
        msg.avoidable = bool(msg.points) and msg.ttc >= self.ttc_stop
        msg.narrow_gap = msg.obstacle_detected and not msg.points
        self._emit_path(Path(header=msg.header))
        self._output.publish(msg)
        detail = (planner.reason, planner.anchors)
        if detail != self._last_reason:
            self._last_reason = detail
            self.get_logger().info(f'A-B-C planner: {planner.reason}; anchors={planner.anchors}; map_points={len(planner.path.points) if planner.path else 0}')


def main(args=None):
    rclpy.init(args=args)
    node = MainGapPathTrialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
