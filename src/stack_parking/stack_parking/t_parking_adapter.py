"""Single-publisher adapter for the existing MGM PREPARE/ACTIVATE protocol."""
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import qos_profile_sensor_data
from fma_interfaces.msg import GpsPath, MgmState, ParkingCommand, ParkingStatus, RefPoint, VehicleVector
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from .geometry import Pose2
from .t_parking_sequence import TParkingSequence, load_course
from .t_reference_parking import fresh, scan_from_ranges


def stamp_s(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class ReferenceParkingAdapter:
    def __init__(self, node):
        self.node = node
        share = Path(get_package_share_directory('stack_parking'))
        self.mission_mode = 1
        self.route_id = '03'
        catalog = node._p('parking_course_catalog')
        self.courses = {}
        if catalog:
            from .parking_courses import load_catalog
            self.courses = load_catalog(catalog, node._p('t_reference_origin_csv'))
            initial = self.courses[1]
            self.course, self.route_csv = initial.course, initial.route_csv
        else:
            self.route_csv = Path(node._p('t_reference_route_csv')).resolve()
            self.course = load_course(node._p('t_reference_origin_csv'), self.route_csv,
                                      [Path(node._p(f't_reference_reverse_{i}_csv'))
                                       if node._p(f't_reference_reverse_{i}_csv')
                                       else share/'config'/f'parking_ref_{i:02d}.csv'
                                       for i in (1,2)])
        geometry = Path(get_package_share_directory('lidar_fusion_v2'))/'config/fixed_geometry.yaml'
        self.sensors = yaml.safe_load(geometry.read_text(encoding='utf-8'))['/**']['ros__parameters']['sensors']
        self.gps = self.vehicle = self.mgm = None
        self.scans = {k: None for k in ('a1','a2','b1')}
        self.core = None
        self.request_id = 0
        self.active = self.authorized = False
        self.started = -math.inf
        self.route_identity = None
        self.last_phase = None
        self.estop_pause_since = None
        self.subs = [node.create_subscription(GpsPath,'/perception/gps_path',self.on_gps,1),
                     node.create_subscription(VehicleVector,str(node._p('vehicle_topic')),self.on_vehicle,qos_profile_sensor_data),
                     node.create_subscription(MgmState,'/adas/mgm_state',self.on_mgm,1)]
        for key in self.scans:
            self.subs.append(node.create_subscription(LaserScan,self.sensors[key]['topic'],
                              lambda msg,k=key:self.on_scan(k,msg),qos_profile_sensor_data))
        self.phase_pub = node.create_publisher(String,'/parking/t_reference_phase',1)
        self.selected_path_pub = node.create_publisher(RosPath, '/parking/selected_path_map', 1)
        from visualization_msgs.msg import MarkerArray
        self.selection_view_pub = node.create_publisher(MarkerArray, '/parking/selection_markers', 1)

    def publish_selection_view(self, pose):
        from .parking_selection_view import selection_markers
        self.selection_view_pub.publish(selection_markers(
            self.core.candidates, self.core.cfg, pose, self.scans['b1'], self.core.selected))

    def publish_selected_path(self, points=()):
        msg = RosPath()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        for point in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = float(point.x), float(point.y)
            pose.pose.orientation.z = math.sin(point.yaw/2)
            pose.pose.orientation.w = math.cos(point.yaw/2)
            msg.poses.append(pose)
        self.selected_path_pub.publish(msg)

    def on_gps(self,msg):
        self.gps = msg

    def on_vehicle(self,msg):
        self.vehicle = msg

    def on_mgm(self,msg):
        self.mgm = msg

    def on_scan(self,key,msg):
        if msg.header.frame_id.lstrip('/') != f'lidar_{key}_link':
            self.scans[key] = None
            return
        sensor = self.sensors[key]
        self.scans[key] = scan_from_ranges(
            stamp_s(msg.header.stamp),msg.ranges,msg.angle_min,msg.angle_increment,
            max(msg.range_min,sensor['min_range']),min(msg.range_max,sensor['max_range']),
            Pose2(sensor['x'],sensor['y'],math.radians(sensor['yaw_deg'])),
            sensor['range_offset_m'],math.radians(sensor['fov_min_deg']),math.radians(sensor['fov_max_deg']))

    def command(self,msg):
        """Return True if handled; parallel mode still uses the existing pipeline.

        Share the parent request watermark so delayed T/parallel/cancel traffic
        cannot steal the current session. CANCEL tombstones even before PREPARE.
        """
        n = self.node
        request = int(msg.request_id)
        if request <= 0 or request < n.search_request_id:
            return True
        if msg.action == ParkingCommand.CANCEL:
            self.active = self.authorized = False
            self.publish_selected_path()
            n.search_request_id = request
            n.search_mission_mode = int(msg.mission_mode)
            n._cancel_search()
            status = ParkingStatus()
            status.header.stamp = n.get_clock().now().to_msg()
            status.request_id, status.mission_mode = request, int(msg.mission_mode)
            n.status_pub.publish(status)
            return True
        mode = int(msg.mission_mode)
        courses = getattr(self, 'courses', {})
        if mode not in (courses if courses else (1,)):
            if request > n.search_request_id and msg.action == ParkingCommand.PREPARE:
                self.active = self.authorized = False
                self.publish_selected_path()
            return False
        if msg.action == ParkingCommand.PREPARE:
            if request == n.search_request_id:
                return True
            n._cancel_search()
            n.search_request_id, n.search_mission_mode = request, mode
            self.mission_mode = mode
            self.request_id = request
            exits = None
            if courses:
                selected_course = courses[mode]
                self.course = selected_course.course
                self.route_csv, self.route_id = selected_course.route_csv, selected_course.route_id
                exits = selected_course.exits
            self.core = TParkingSequence(*self.course, exits=exits)
            self.publish_selected_path()
            self.active, self.authorized = True, False
            self.started = n._clock_s()
            self.route_identity = None
            self.last_phase = None
        elif (msg.action == ParkingCommand.ACTIVATE and self.active
              and request == self.request_id and mode == self.mission_mode and self.core.selected is not None
              and self.core.phase != 'FAULT'):
            self.authorized = True
        return True

    def tick(self):
        if not self.active:
            return False
        n, core = self.node, self.core
        now = n._clock_s()
        gps, vehicle, mgm = self.gps,self.vehicle,self.mgm
        if mgm and getattr(mgm, 'estop_active', False):
            if self.estop_pause_since is None:
                self.estop_pause_since = now
            return True  # upper ESTOP owns motion; freeze parking progression
        if self.estop_pause_since is not None:
            if core.wait_since is not None:
                core.wait_since += now-self.estop_pause_since
            core.stopped_since = None
            self.estop_pause_since = None
        if (core.phase == 'DONE' and mgm and fresh(stamp_s(mgm.header.stamp),now,.25)
            and mgm.mission_request_id == self.request_id and mgm.mission_completed
            and not mgm.mission_request_active):
            self.active = self.authorized = False
            return False  # MGM acknowledged completion; release the sole publisher
        cfg = core.cfg
        pose_stamp = stamp_s(gps.reference_stamp) if gps else -math.inf
        speed_stamp = stamp_s(vehicle.header.stamp) if vehicle else -math.inf
        pose = Pose2(gps.position_x,gps.position_y,gps.vehicle_heading_rad) if gps else Pose2()
        route_ok = bool(gps and gps.route.enabled and not gps.route.connecting
                        and gps.route.route_id == getattr(self, 'route_id', '03')
                        and Path(gps.route.waypoint_csv).resolve() == self.route_csv)
        identity = ((gps.route.sequence_id,gps.route.instance_id,gps.route.index)
                    if route_ok else None)
        if self.route_identity is None and route_ok:
            self.route_identity = identity
        # Route identity is checked for initial selection. A locked candidate
        # stays attached to its original course until completion or CANCEL.
        route_ok = core.selected is not None or (route_ok and identity == self.route_identity)
        pose_ok = bool(route_ok and gps and gps.position_valid and gps.vehicle_heading_valid
                       and gps.heading_source == GpsPath.HEADING_FUSED)
        owner = self.authorized
        allowed = True  # MGM arbitrates operator stop, CAN and ESTOP.
        sensors = [self.scans[k] for k in ('b1','a2','a1')]
        inputs = bool(pose_ok and vehicle and
                      all(math.isfinite(v) for v in (pose.x,pose.y,pose.yaw,float(vehicle.v))))
        if not pose_ok:
            pose_stamp = -math.inf
        if core.selected is None or self.authorized or not inputs or not allowed:
            result = core.tick(now,pose,pose_stamp,float(vehicle.v) if vehicle else math.nan,
                               speed_stamp,*sensors,owned=owner,route_at_end=bool(gps and gps.at_end),
                               motion_allowed=allowed and inputs)
        else:
            result = core.out()  # selection is locked; wait for actual ACTIVATE
        if core.selected is not None:
            self.publish_selected_path(core.candidates[core.selected].path)
        if pose_ok:
            self.publish_selection_view(pose)
        ready = inputs and allowed and core.selected is not None and core.phase != 'FAULT'
        status = ParkingStatus()
        status.header.stamp = n.get_clock().now().to_msg()
        status.header.frame_id = 'base_link'
        status.request_id, status.mission_mode = self.request_id, self.mission_mode
        status.search_active = True
        status.search_space_found = core.selected is not None
        status.wall_acquisition_complete = bool(ready)
        status.wall_acquisition_frames = min(core.votes,255) if ready else 0
        status.preparation_ready = bool(ready)
        if ready:
            # RC test contract: reference generation is this planning tick,
            # computed using the latest received pose; not an acquisition stamp.
            status.preparation_stamp = status.header.stamp
            status.reference_stamp = status.header.stamp
        status.mission_active = self.authorized
        status.space_found = core.selected is not None
        status.path_blocked = core.phase == 'FAULT'
        status.v_suggest = result.speed if self.authorized and ready else 0.
        status.done = bool(self.authorized and ready and result.done)
        if self.authorized and ready and result.reference is not None:
            p = result.reference
            status.points = [RefPoint(x=p.x,y=p.y,yaw=p.yaw,curvature=p.curvature)]
        if gps and inputs:
            status.dx,status.dy,status.dyaw,status.update = gps.dx,gps.dy,gps.dyaw,gps.update
        n.status_pub.publish(status)
        selected = None if result.selected is None else result.selected+1
        self.phase_pub.publish(String(data=f'{result.phase}: ref={selected} {result.reason}'))
        if result.phase != self.last_phase:
            n.get_logger().info(f'T reference parking {result.phase}: {result.reason}')
            self.last_phase = result.phase
        return True
