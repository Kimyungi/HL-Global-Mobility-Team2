#!/usr/bin/python3
"""Read-only, vehicle-centred RViz presentation. Never publishes control inputs."""
from copy import deepcopy
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from builtin_interfaces.msg import Duration
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, Header
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from fma_interfaces.msg import GpsPath, LanePath, MgmState, ParkingStatus, ParkingWallStatus, AvoidStatus, TargetRef, TrafficStop, VehicleVector

FRAME = 'v2_vehicle_view'
PREFIX = '/integration_v2/view'
ORANGE = (1., .62, .08)
CYAN = (.05, .85, 1.)
PINK = (1., .18, .7)
GREEN = (.2, 1., .35)
WHITE = (.9, .93, 1.)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def local_xy(points, pose):
    """Map coordinates -> current vehicle coordinates; GPS and SLAM stay independent."""
    xy = np.asarray(points, dtype=float).reshape((-1, 2))
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (xy - np.asarray(pose[:2])) @ np.array([[c, -s], [s, c]])


def pose_xy_yaw(pose):
    q = pose.orientation
    values = (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        return None
    norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    if norm < 1e-12:
        return None
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), norm - 2*(q.y*q.y + q.z*q.z))
    return (pose.position.x, pose.position.y, yaw)


def stop_line_x(state, bumper):
    if state is None or not state.traffic_distance_known or not math.isfinite(state.traffic_remaining_m):
        return None
    return bumper + state.traffic_remaining_m  # signed: a passed line remains behind the bumper


def image_bgr(msg):
    channels = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}.get(msg.encoding)
    if not channels or msg.step < msg.width*channels or len(msg.data) < msg.height*msg.step:
        return None
    rows = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    pixels = rows[:, :msg.width*channels].reshape(msg.height, msg.width, channels)
    if channels == 1:
        return cv2.cvtColor(pixels, cv2.COLOR_GRAY2BGR)
    return cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else pixels


class IntegrationView(Node):
    def __init__(self):
        super().__init__('integration_v2_view')
        self.front = float(self.declare_parameter('vehicle_front_m', .760).value)
        self.rear = float(self.declare_parameter('vehicle_rear_m', .090).value)
        self.width = float(self.declare_parameter('vehicle_width_m', .62).value)
        self.samples = {}
        self.decoded_cloud = None
        self.route_key = None
        self.track_since = 0.
        self.parking_since = 0.
        self.request_id = 0
        self.subs = []
        specs = [
            ('gps', GpsPath, '/perception/gps_path'), ('lane', LanePath, '/perception/lane_path'),
            ('mgm', MgmState, '/adas/mgm_state'), ('target', TargetRef, '/adas/target_ref'),
            ('traffic', TrafficStop, '/perception/traffic_stop'), ('vehicle', VehicleVector, '/vehicle/vector'),
            ('avoid', AvoidStatus, '/perception/avoid'),
            ('parking', ParkingStatus, '/perception/parking'), ('pose', PoseStamped, '/parking/slam_pose'),
            ('map', PointCloud2, '/parking/local_map'), ('plan', Path, '/parking/reference_path'),
            ('active', Path, '/parking/active_path'), ('walls', MarkerArray, '/parking/debug_markers'),
            ('left_wall', MarkerArray, '/parking/left_wall/markers'),
            ('wall_status', ParkingWallStatus, '/parking/left_wall/status'),
            ('diagnostics', DiagnosticArray, '/parking/diagnostics'),
            ('lane_image', Image, '/perception/lane_debug_image'),
            ('traffic_image', Image, '/perception/traffic_debug_image')]
        specs += [(sid, PointCloud2, f'/unified_lidar/raw/{sid}') for sid in ('a1', 'a2', 'b1', 'b2')]
        for key, kind, topic in specs:
            self.subs.append(self.create_subscription(kind, topic, lambda msg, k=key: self.receive(k, msg),
                                                     qos_profile_sensor_data))
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subs.append(self.create_subscription(Path, '/perception/gps_track_viz',
                                                 lambda msg: self.receive('track', msg), latched))
        self.markers_pub = self.create_publisher(MarkerArray, PREFIX+'/markers', 1)
        self.hud_pub = self.create_publisher(Image, PREFIX+'/dashboard', qos_profile_sensor_data)
        self.cloud_pubs = {key: self.create_publisher(PointCloud2, PREFIX+'/'+key, qos_profile_sensor_data)
                           for key in ('map', 'a1', 'a2', 'b1', 'b2')}
        self.frame_broadcaster = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = FRAME
        tf.transform.rotation.w = 1.
        self.frame_broadcaster.sendTransform(tf)
        self.create_timer(.1, self.render)

    def receive(self, key, msg):
        if key == 'gps':
            route = (msg.route.sequence_id, msg.route.index, msg.route.connecting)
            previous = self.samples.get('gps')
            if self.route_key is not None and route != self.route_key and previous:
                self.track_since = seconds(previous[0].header.stamp)
            self.route_key = route
        if key == 'mgm' and msg.mission_request_active and msg.mission_request_id != self.request_id:
            self.request_id = msg.mission_request_id
            self.parking_since = seconds(msg.zone_entry_observation.stamp)
        self.samples[key] = (msg, time.monotonic())
        if key == 'map':
            try:
                xy = point_cloud2.read_points_numpy(msg, field_names=('x', 'y'), skip_nans=True)
                self.decoded_cloud = xy.reshape((-1, 2))
                self.decoded_cloud = self.decoded_cloud[np.isfinite(self.decoded_cloud).all(axis=1)]
            except (ValueError, AssertionError, TypeError):
                self.decoded_cloud = None

    def get(self, key, age=.6, generation=False):
        entry = self.samples.get(key)
        if entry is None or time.monotonic()-entry[1] > age:
            return None
        msg = entry[0]
        stamp = getattr(msg, 'reference_stamp', None) if generation else getattr(getattr(msg, 'header', None), 'stamp', None)
        if stamp is not None:
            elapsed = self.get_clock().now().nanoseconds*1e-9 - seconds(stamp)
            if seconds(stamp) <= 0 or not 0 <= elapsed <= age:
                return None
            if key in ('pose', 'map', 'plan', 'active', 'diagnostics') and seconds(stamp) < self.parking_since:
                return None
        return msg

    def marker(self, name, kind, color, scale=.08):
        msg = Marker(header=self.header, ns=name, id=0, type=kind, action=Marker.ADD)
        msg.pose.orientation.w = 1.
        msg.color = ColorRGBA(r=float(color[0]), g=float(color[1]), b=float(color[2]), a=1.)
        msg.scale.x = msg.scale.y = msg.scale.z = scale
        msg.lifetime = Duration(nanosec=400_000_000)
        self.markers.append(msg)
        return msg

    def line(self, name, xy, color, width=.07, z=.04):
        xy = np.asarray(xy, dtype=float).reshape((-1, 2))
        if not len(xy) or not np.isfinite(xy).all():
            return
        msg = self.marker(name, Marker.LINE_STRIP, color, width)
        msg.points = [Point(x=float(x), y=float(y), z=z) for x, y in xy]

    def text(self, name, text, x, y, color=WHITE, size=.24):
        msg = self.marker(name, Marker.TEXT_VIEW_FACING, color, size)
        msg.pose.position = Point(x=float(x), y=float(y), z=.55)
        msg.text = text

    def preview(self, name, msg, color, z):
        points = getattr(msg, 'points', getattr(msg, 'ref_points', [])) if msg is not None else []
        if not points or msg.header.frame_id.lstrip('/') != 'base_link':
            return
        p = points[0]
        if not all(math.isfinite(v) for v in (p.x, p.y, p.yaw, p.curvature)):
            return
        arrow = self.marker(name, Marker.ARROW, color)
        arrow.pose.position = Point(x=p.x, y=p.y, z=z)
        arrow.pose.orientation.z, arrow.pose.orientation.w = math.sin(p.yaw/2), math.cos(p.yaw/2)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = .5, .12, .16
        dot = self.marker(name+'_point', Marker.SPHERE, color, .18)
        dot.pose.position = arrow.pose.position
        self.text(name+'_label', name, p.x, p.y+.28, color, .18)

    def cloud(self, xy):
        xy = np.asarray(xy, dtype=np.float32).reshape((-1, 2))
        xyz = np.zeros((len(xy), 3), dtype='<f4')
        xyz[:, :2] = xy
        return PointCloud2(header=self.header, height=1, width=len(xyz), is_dense=True,
                           fields=[PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                                   for i, n in enumerate(('x', 'y', 'z'))],
                           point_step=12, row_step=12*len(xyz), data=xyz.tobytes())

    def render(self):
        self.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=FRAME)
        self.markers = [Marker(header=self.header, action=Marker.DELETEALL)]
        self.line('vehicle', [(-self.rear,-self.width/2),(self.front,-self.width/2),
                              (self.front,self.width/2),(-self.rear,self.width/2),(-self.rear,-self.width/2)], WHITE)
        self.line('vehicle_heading', [(0,0),(self.front,0),(self.front-.2,.15),
                                     (self.front,0),(self.front-.2,-.15)], WHITE)
        gps, lane, state = self.get('gps', .5, True), self.get('lane', 1., True), self.get('mgm', .3)
        traffic, parking = self.get('traffic', .5), self.get('parking', .5)
        target = self.get('target', .3)
        if gps is not None and gps.position_valid:
            self.preview('GPS', gps, ORANGE, .10)
            track = self.samples.get('track', (None,))[0]
            pose = (gps.position_x, gps.position_y, gps.vehicle_heading_rad)
            if track is not None and track.header.frame_id == 'map' and seconds(track.header.stamp) >= self.track_since and all(map(math.isfinite, pose)):
                xy = [(p.pose.position.x, p.pose.position.y) for p in track.poses]
                self.line('gps_route', local_xy(xy, pose), ORANGE, .07)
        if lane is not None:
            self.preview('CAMERA', lane, CYAN, .20)
        if target is not None:
            self.preview('MGM', target, PINK, .30)
        avoid = self.get('avoid', .5, True)
        if avoid is not None and avoid.scan_valid:
            self.preview('AVOID_TARGET', avoid, (1., 1., .05), .40)
        if parking is not None and (not self.request_id or parking.request_id == self.request_id):
            self.preview('PARKING', self.get('parking', .5, True), GREEN, .25)
        dist_x = stop_line_x(state, self.front)
        if dist_x is not None:
            self.line('stop_line_dist_estimate', [(dist_x,-1.1),(dist_x,1.1)], (1., .3, .16), .10)
            self.text('stop_line_label', f'STOP LINE dist={state.traffic_remaining_m:.2f}m (estimate)', dist_x, 1.7)
        lamp = self.marker('traffic_light_indicator', Marker.SPHERE,
                           (1., .12, .12) if traffic is not None and traffic.red_active else
                           (.2, 1., .2) if traffic is not None and traffic.green_active else (.35,.35,.35), .32)
        lamp.pose.position = Point(x=4., y=-3., z=.3)
        self.text('light_label', 'LIGHT STATUS (icon)', 4., -3.9, size=.20)

        pose_msg = self.get('pose')
        slam = pose_xy_yaw(pose_msg.pose) if pose_msg is not None else None
        map_msg = self.get('map')
        mapped = []
        if slam is not None:
            if map_msg is not None and map_msg.header.frame_id == pose_msg.header.frame_id and self.decoded_cloud is not None:
                mapped = local_xy(self.decoded_cloud, slam)
            for key, color in [('plan', (.15,.55,.2)), ('active', GREEN)]:
                path = self.get(key)
                if path is not None and path.header.frame_id == pose_msg.header.frame_id:
                    self.line('parking_'+key, local_xy([(p.pose.position.x,p.pose.position.y) for p in path.poses], slam), color, .07 if key=='plan' else .13, .12)
            wall_arrays = [self.get('walls'), self.get('left_wall')]
            for walls in wall_arrays:
                if walls is None: continue
                for original in walls.markers:
                    if original.action != Marker.ADD or original.type == Marker.TEXT_VIEW_FACING or original.header.frame_id != pose_msg.header.frame_id or seconds(original.header.stamp) < self.parking_since:
                        continue
                    marker = deepcopy(original)
                    if marker.pose.orientation == type(marker.pose.orientation)():
                        marker.pose.orientation.w = 1.  # point-list markers may use default identity
                    world = pose_xy_yaw(marker.pose)
                    if world is None:
                        continue
                    x, y = local_xy([world[:2]], slam)[0]
                    marker.pose.position.x, marker.pose.position.y = float(x), float(y)
                    marker.pose.orientation.x = marker.pose.orientation.y = 0.
                    marker.pose.orientation.z, marker.pose.orientation.w = math.sin((world[2]-slam[2])/2), math.cos((world[2]-slam[2])/2)
                    marker.header, marker.ns = self.header, 'parking_'+marker.ns
                    marker.lifetime = Duration(nanosec=400_000_000)
                    self.markers.append(marker)
        self.cloud_pubs['map'].publish(self.cloud(mapped))
        for sid in ('a1', 'a2', 'b1', 'b2'):
            raw = self.get(sid, .35)
            if raw is None or raw.header.frame_id.lstrip('/') != 'base_link':
                raw = self.cloud([])
            else:
                raw = deepcopy(raw)
                raw.header.frame_id = FRAME
            self.cloud_pubs[sid].publish(raw)
        self.markers_pub.publish(MarkerArray(markers=self.markers))
        self.dashboard(gps, lane, state, traffic, parking, target, slam is not None)

    def dashboard(self, gps, lane, state, traffic, parking, target, slam_valid):
        frame = np.full((1160,720,3), (28,25,23), np.uint8)
        def text(value, y, color=(230,230,230), scale=.61):
            cv2.putText(frame, value[:88], (16,y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
        preparation = ('PREP: NO MGM DATA' if state is None else
                       f'PREP: {"READY" if state.start_ready else "WAIT"} | '
                       f'CAM {"ON" if state.camera_available else "--"} | '
                       f'GPS {"FIXED" if state.gps_fixed_ready else "--"} | '
                       f'GO {"YES" if state.go_authorized else "NO"}')
        text(preparation, 30, (130,245,130) if state is not None and state.start_ready
             else (255,230,120), .62)
        light = 'NO DATA / STALE' if traffic is None else 'RED + GREEN' if traffic.red_active and traffic.green_active else 'RED' if traffic.red_active else 'GREEN' if traffic.green_active else 'UNKNOWN'
        text('LIGHT: '+light, 61, (90,90,255) if traffic is not None and traffic.red_active else
             (130,245,130) if traffic is not None and traffic.green_active else (160,160,160))
        text('GPS: NO FRESH FIX' if gps is None else f'GPS: fix={gps.fix_quality} route={gps.route.route_id or "single"} idx={gps.track_index} heading={"measured" if gps.vehicle_heading_valid else "TANGENT estimate"}', 89)
        vehicle = self.get('vehicle', .3)
        text(f'v_ref: {target.v_ref:.2f} m/s' if target is not None else 'v_ref: --', 117, (200,120,255))
        text(f'actual: {vehicle.v:.2f} m/s' if vehicle is not None else 'actual: --', 145)
        if state is None:
            mgm_text = 'MGM: NO DATA / STALE'
        else:
            top = {0:'WAIT GO',1:'DRIVE',2:'FINISH'}.get(state.top,'UNKNOWN')
            source = {0:'CAMERA',1:'GPS',2:'AVOID',3:'PARKING',4:'REVERSE'}.get(state.reference_source,'UNKNOWN')
            mission = 'SEARCH' if state.mission_request_active and not state.parking_ready else 'PARKING' if state.mission_request_active else 'NAVIGATION'
            mgm_text = f'MGM: {top} | {source} | {mission}'
            if state.active_safe_stop_reasons:
                mgm_text += f' | STOP 0x{state.active_safe_stop_reasons:X}'
        text(mgm_text, 173)
        text('STOP LINE dist: UNKNOWN' if stop_line_x(state,self.front) is None else f'STOP LINE dist: {state.traffic_remaining_m:.2f}m from front bumper (estimate)', 201)
        text('Camera optical-Z: --' if traffic is None or not math.isfinite(traffic.stop_distance) or traffic.stop_distance<0 else f'Camera optical-Z: {traffic.stop_distance:.2f}m (different distance)', 229)
        diagnostics = self.get('diagnostics')
        values = {v.key:v.value for d in diagnostics.status for v in d.values} if diagnostics is not None else {}
        text(f'SLAM: {"LIVE" if slam_valid else "NO DATA / STALE"} | stage={values.get("pipeline_stage","--")} map={values.get("map_points","--")}', 257)
        text('PARKING: NO DATA' if parking is None else f'PARKING: search={int(parking.search_active)} space={int(parking.search_space_found)} ready={int(parking.preparation_ready)} active={int(parking.mission_active)}', 285)
        text(f'Rear wall={values.get("rear_clearance_m","--")}m | {values.get("mission_state","--")} {values.get("plan_error","")}', 313)
        wall = self.get('wall_status', .5)
        text('LEFT SCANS: WAITING FOR DATA' if wall is None else
             f'LEFT SCANS {wall.frame_count}/5 | {wall.phase} | v={wall.actual_speed:.2f}'
             + ('' if wall.vehicle_speed_valid else ' INVALID'), 344, (140,200,230), .50)
        for key, title, top in [('lane_image','LANE CAMERA',360), ('traffic_image','TRAFFIC CAMERA',760)]:
            text(title, top+26, (200,230,255))
            msg = self.get(key, 1.)
            pixels = image_bgr(msg) if msg is not None else None
            if pixels is None:
                text('WAITING FOR CAMERA / STALE', top+190, (160,160,160))
                continue
            factor = min(720/pixels.shape[1], 360/pixels.shape[0])
            pixels = cv2.resize(pixels, (max(1,int(pixels.shape[1]*factor)), max(1,int(pixels.shape[0]*factor))))
            x, y = (720-pixels.shape[1])//2, top+35+(360-pixels.shape[0])//2
            frame[y:y+pixels.shape[0],x:x+pixels.shape[1]] = pixels
        self.hud_pub.publish(Image(header=self.header, height=1160, width=720, encoding='bgr8', step=2160, data=frame.tobytes()))


def main():
    rclpy.init()
    node = IntegrationView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
