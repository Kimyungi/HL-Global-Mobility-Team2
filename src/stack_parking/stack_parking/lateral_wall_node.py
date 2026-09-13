"""Integrated left wall perception and markers; no vehicle command publisher."""
import math
from pathlib import Path

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from fma_interfaces.msg import ParkingCommand, MgmState, VehicleVector, ParkingWallStatus
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import Pose2, transform_points
from .lateral_wall import LateralWallAcquisition, LateralWallConfig


class LateralWallNode(Node):
    def __init__(self):
        super().__init__('parking_lateral_wall')
        geometry=Path(get_package_share_directory('lidar_fusion_v2'))/'config/fixed_geometry.yaml'
        config=yaml.safe_load(geometry.read_text())['/**']['ros__parameters']
        mount=config['sensors']['b1']
        self.tracker=LateralWallAcquisition(LateralWallConfig(
            origin_x=float(mount['x']),origin_y=float(mount['y']),
            angular_step_deg=float(config['scan']['angle_increment_deg'])))
        self.pose=None; self.pose_stamp=0.; self.request_id=0
        self.mission_active=False; self.mgm_stamp=0.; self.entry_stamp=0.
        self.mgm_request_id=0; self.collected_frames=0; self.collection_generation=None
        self.vehicle_speed=math.nan; self.vehicle_stamp=0.; self.stopped_since=None
        self.slam_valid=False; self.slam_stamp=0.
        self.markers=self.create_publisher(MarkerArray,'/parking/left_wall/markers',1)
        self.diagnostics=self.create_publisher(DiagnosticArray,'/parking/left_wall/diagnostics',1)
        self.status_pub=self.create_publisher(ParkingWallStatus,'/parking/left_wall/status',1)
        self.create_subscription(PointCloud2,'/unified_lidar/cloud',self.on_cloud,qos_profile_sensor_data)
        self.create_subscription(PoseStamped,'/parking/slam_pose',self.on_pose,qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray,'/parking/diagnostics',self.on_diagnostic,qos_profile_sensor_data)
        self.create_subscription(ParkingCommand,'/parking/mission_command',self.on_command,10)
        self.create_subscription(MgmState,'/adas/mgm_state',self.on_mgm,qos_profile_sensor_data)
        self.create_subscription(VehicleVector,'/vehicle/vector',self.on_vehicle,qos_profile_sensor_data)
        self.create_subscription(Bool,'/parking/left_wall/reset',self.on_reset,1)
        self.create_timer(.1,self.publish)
        self.get_logger().info('Left wall: vehicle +90deg from b1 mount; centre + 5 each side; 5 fresh frames; +/-0.12m')

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def now_s(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_command(self,msg):
        if msg.action==ParkingCommand.PREPARE and msg.request_id>self.request_id:
            self.request_id=msg.request_id; self.tracker.reset()
            self.pose=None; self.slam_valid=False
            self.collected_frames=0; self.collection_generation=None; self.stopped_since=None

    def on_mgm(self,msg):
        active=msg.mission==1 and msg.mission_request_active
        if active and (not self.mission_active or msg.mission_request_id!=self.mgm_request_id):
            self.request_id=msg.mission_request_id; self.tracker.reset()
            self.stopped_since=None; self.entry_stamp=self.stamp(msg)
            self.collected_frames=0; self.collection_generation=None
        elif not active and self.mission_active:
            self.tracker.reset(); self.stopped_since=None
            self.collected_frames=0; self.collection_generation=None
        self.mgm_request_id=msg.mission_request_id
        self.mission_active=active; self.mgm_stamp=self.stamp(msg)

    def on_vehicle(self,msg):
        stamp=self.stamp(msg)
        if stamp < self.vehicle_stamp:
            self.vehicle_speed=math.nan
            return
        self.vehicle_speed=float(msg.v); self.vehicle_stamp=stamp

    def acquisition_gate(self,now):
        if not self.mission_active or not 0<=now-self.mgm_stamp<=.3:
            self.tracker.invalidate('WAITING_FOR_PARKING'); self.stopped_since=None
            if self.collected_frames<5:
                self.collected_frames=0; self.collection_generation=None
            return False
        if self.collected_frames>=5:
            return True  # request-scoped completion remains latched during resumed driving
        valid=(math.isfinite(self.vehicle_speed) and self.vehicle_stamp>=self.entry_stamp
               and 0<=now-self.vehicle_stamp<=.2)
        if not valid or abs(self.vehicle_speed)>float(np.float32(.1)):
            self.tracker.invalidate('WAITING_FOR_CAN_SPEED' if not valid else 'WAITING_FOR_STOP')
            self.stopped_since=None
            self.collected_frames=0; self.collection_generation=None
            return False
        if self.stopped_since is None:
            self.stopped_since=max(self.vehicle_stamp,self.entry_stamp)
        return True

    def on_reset(self,msg):
        if msg.data:
            self.tracker.reset(); self.stopped_since=None
            self.collected_frames=0; self.collection_generation=None

    def on_pose(self,msg):
        if msg.header.frame_id!='parking_map': return
        q=msg.pose.orientation
        self.pose=Pose2(msg.pose.position.x,msg.pose.position.y,
            math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        self.pose_stamp=self.stamp(msg)

    def on_diagnostic(self,msg):
        for status in msg.status:
            if status.name=='stack_parking/pipeline':
                values={v.key:v.value for v in status.values}
                self.slam_valid=values.get('slam_valid')=='True'
                self.slam_stamp=self.stamp(msg)

    def on_cloud(self,msg):
        now=self.now_s()
        if msg.header.frame_id!='base_link': return
        if not self.acquisition_gate(now): return
        if self.collected_frames<5 and self.stamp(msg)<self.stopped_since:
            return  # queued scans from before measured stopping never count
        if (self.pose is None or not self.slam_valid or self.pose_stamp<self.entry_stamp
                or self.slam_stamp<self.entry_stamp or not 0<=now-self.pose_stamp<=.6
                or not 0<=now-self.slam_stamp<=.6):
            self.tracker.invalidate('WAITING_FOR_VALID_SLAM'); return
        points=np.asarray([(float(p[0]),float(p[1])) for p in
            point_cloud2.read_points(msg,field_names=('x','y'),skip_nans=True)],dtype=float).reshape((-1,2))
        generation=self.stamp(msg)
        if not 0<generation<=now or now-generation>self.tracker.config.stale_s or not len(points):
            return
        if self.collected_frames<5:
            previous=self.collection_generation
            if previous is not None and generation<=previous:
                return
            if previous is not None and generation-previous>self.tracker.config.stale_s:
                self.collected_frames=0
            self.collection_generation=generation
            self.collected_frames+=1
        self.tracker.update(points,self.pose,generation,now)

    def publish(self):
        now=self.now_s(); stamp=self.get_clock().now().to_msg(); tracker=self.tracker
        gate=self.acquisition_gate(now)
        if gate and (tracker.generation is None or now-tracker.generation>tracker.config.stale_s):
            tracker.invalidate('WAITING_FOR_SCAN' if tracker.generation is None else 'STALE_SCAN')
        array=MarkerArray(); delete=Marker(); delete.action=Marker.DELETEALL; array.markers.append(delete)
        def marker(name,kind,points,color,width):
            m=Marker(); m.header.frame_id='parking_map'; m.header.stamp=stamp
            m.ns='left_wall_'+name; m.id=len(array.markers); m.type=kind; m.action=Marker.ADD
            m.pose.orientation.w=1.; m.scale.x=m.scale.y=m.scale.z=float(width)
            m.color.r,m.color.g,m.color.b,m.color.a=(*color,1.)
            m.points=[Point(x=float(x),y=float(y),z=.4) for x,y in points]
            array.markers.append(m); return m
        # Historical locked lines remain visible, but grey indicates missing
        # live scan/SLAM evidence. No historical line grants parking readiness.
        live=tracker.reason in ('LOCKED','CONFIRMING')
        wall=tracker.wall or tracker.candidate
        if self.pose is not None and len(tracker.selected_base):
            marker('selected_returns',Marker.POINTS,transform_points(tracker.selected_base,self.pose),(0.,1.,1.),.10)
        if wall is not None:
            support=tracker.support_map
            along=wall.project(support)[:,0]
            # Extend only the displayed reference line for readability.
            lo,hi=min(-1.5,float(along.min())),max(1.5,float(along.max()))
            for offset,name,color in [(0.,'reference',(0.,1.,.25)),(-.12,'minus_12cm',(1.,.65,.1)),(.12,'plus_12cm',(1.,.65,.1))]:
                marker(name,Marker.LINE_STRIP,wall.to_map(np.array([[lo,offset],[hi,offset]])),color if live else (.5,.5,.5),.045)
            marker('five_frame_support',Marker.POINTS,support,(.2,.8,1.),.06)
        if self.pose is not None:
            origin=transform_points(np.array([[tracker.config.origin_x,tracker.config.origin_y],
                                             [tracker.config.origin_x,tracker.config.origin_y+3.]]),self.pose)
            marker('vehicle_left_ray',Marker.LINE_STRIP,origin,(.7,.7,1.),.025)
            m=marker('status',Marker.TEXT_VIEW_FACING,[],(1.,1.,1.),.22)
            m.pose.position=Point(x=float(origin[0,0]),y=float(origin[0,1])+.6,z=.6)
            m.text=f'SCANS {self.collected_frames}/5 | WALL {tracker.count}/5 {tracker.reason} | +/-12cm'
        self.markers.publish(array)
        diagnostic=DiagnosticArray(); diagnostic.header.stamp=stamp
        status=DiagnosticStatus(); status.name='stack_parking/left_wall'; status.message=tracker.reason
        status.level=DiagnosticStatus.OK if live else DiagnosticStatus.WARN
        values={'collected_frames':self.collected_frames,'frames':tracker.count,'required_frames':5,'selected_points':len(tracker.selected_base),
                'locked':tracker.wall is not None,'fresh':live,'half_width_m':.12,
                'origin_x':tracker.config.origin_x,'origin_y':tracker.config.origin_y,
                'centre_direction_vehicle_deg':90,'source':'fused_all_sensors'}
        status.values=[KeyValue(key=k,value=str(v)) for k,v in values.items()]
        diagnostic.status=[status]; self.diagnostics.publish(diagnostic)
        msg=ParkingWallStatus(); msg.header.stamp=stamp; msg.request_id=self.request_id
        msg.mission_active=self.mission_active and 0<=now-self.mgm_stamp<=.3
        msg.vehicle_speed_valid=math.isfinite(self.vehicle_speed) and 0<=now-self.vehicle_stamp<=.2
        msg.stopped=msg.vehicle_speed_valid and abs(self.vehicle_speed)<=float(np.float32(.1))
        msg.actual_speed=self.vehicle_speed if math.isfinite(self.vehicle_speed) else 0.
        msg.frame_count=self.collected_frames
        msg.complete=msg.mission_active and self.collected_frames>=5
        msg.phase='COMPLETE_GPS_RESUME' if msg.complete else tracker.reason
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args); node=LateralWallNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__=='__main__': main()
