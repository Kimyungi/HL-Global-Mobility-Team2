#!/usr/bin/python3
"""Read-only replay of live parking maps. Publishes visualization only.

Uses the running node's parameters and original detector/planner. Planner trace
captures rejected geometry without changing collision tests or control outputs.
MGM PREPARE resets SLAM and the scan-lane pose to Pose2(); manual missions are
not supported. Replay stability counts are independent of the control node.
"""
import inspect
import json
import math
import re
import sys
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from diagnostic_msgs.msg import DiagnosticArray
from fma_interfaces.msg import GpsPath, ParkingStatus
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from stack_parking.node import StackParkingNode
from stack_parking.geometry import Pose2, transform_points
from stack_parking.space_detector import ParkingSpaceDetector
from stack_parking.path_planner import MinimumRadiusParkingPlanner

OUT = Path('/dev/shm/fma-parking-integrated/perception_debug')


class Debug(Node):
    def __init__(self):
        super().__init__('parking_perception_debug')
        OUT.mkdir(exist_ok=True)
        methods = (StackParkingNode._detector_config, StackParkingNode._planner_config)
        names = sorted(set(re.findall(r"_p\('([^']+)'\)", '\n'.join(inspect.getsource(m) for m in methods))))
        client = self.create_client(GetParameters, '/stack_parking_node/get_parameters')
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError('Running parking node is required')
        request = GetParameters.Request(names=names)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.)
        if not future.done() or future.result() is None:
            raise RuntimeError('Cannot read live parking parameters')
        self.params = {k: parameter_value_to_python(v) for k,v in zip(names, future.result().values)}
        self.detector = ParkingSpaceDetector(StackParkingNode._detector_config(self))
        self.planner = MinimumRadiusParkingPlanner(StackParkingNode._planner_config(self))
        (OUT/'parameters.json').write_text(json.dumps(self.params, indent=2))
        self.samples = {}; self.generation = None; self.request_id = 0
        self.last_plot = 0.; self.scene = []; self.report_data = {}
        self.pub = self.create_publisher(MarkerArray, '/parking_inspection/debug', 1)
        specs = [('map',PointCloud2,'/parking/local_map'),('pose',PoseStamped,'/parking/slam_pose'),
                 ('gps',GpsPath,'/perception/gps_path'),('status',ParkingStatus,'/perception/parking'),
                 ('diagnostics',DiagnosticArray,'/parking/diagnostics')]
        self.subs = [self.create_subscription(typ,topic,lambda m,k=key:self.receive(k,m),qos_profile_sensor_data)
                     for key,typ,topic in specs]
        self.create_timer(.2,self.render)

    def _p(self, key):
        return self.params[key]

    def receive(self,key,msg):
        self.samples[key]=(time.monotonic(),msg)

    def add(self,name,points,color,kind='line',size=.1):
        points=np.asarray(points,dtype=float).reshape((-1,2))
        if len(points): self.scene.append((name,points,color,kind,size))

    def evaluate(self,cloud,pose,status):
        if status.request_id != self.request_id:
            self.detector.reset(); self.request_id=status.request_id
        pts=np.asarray([(float(p[0]),float(p[1])) for p in point_cloud2.read_points(
            cloud,field_names=('x','y'),skip_nans=True)],dtype=float).reshape((-1,2))
        mode={1:'perpendicular',2:'parallel'}.get(status.mission_mode)
        # During PREPARE mission_mode may be zero; typed request mode is exposed
        # in the actual pipeline diagnostic message.
        values={v.key:v.value for s in self.samples['diagnostics'][1].status for v in s.values}
        mode=values.get('mode',mode)
        if mode not in ('perpendicular','parallel') or not self.request_id:
            return
        side=values.get('side','auto')
        captured={}; calls=[]; cfg=self.detector.config
        def trace(frame,event,arg):
            if event=='return':
                if frame.f_code is self.planner.plan.__func__.__code__:
                    captured.update(frame.f_locals.copy())
                elif frame.f_code is self.detector._find_candidate.__func__.__code__:
                    row=frame.f_locals.copy(); row['result']=arg; calls.append(row)
            return trace
        old_trace=sys.gettrace()
        try:
            sys.settrace(trace)
            space=self.detector.update(pts,pose,Pose2(),mode,side)
            plan=self.planner.plan(pose,space,pts) if space is not None else None
        finally:
            sys.settrace(old_trace)
        self.scene=[]
        self.add('map',pts,(.55,.58,.62),'points',.08)
        report={'kind':'read-only replay; independent stability history','request_id':self.request_id,
                'mode':mode,'side':side,'actual_node_plan_error':values.get('plan_error',''),
                'replay_plan_error':self.planner.last_error if space is not None else 'no stable candidate',
                'space':asdict(space) if space else None,'accepted':plan is not None,
                'map_points':len(pts),'walls':{},'stable_count':self.detector._stable_count}
        for row in calls:
            which=row['side']; sign=1. if which=='left' else -1.
            distance=sign*pts[:,1]
            boundary=pts[(distance>=cfg.boundary_near_m)&(distance<=cfg.boundary_far_m)]
            self.add(which+'_boundary_input',boundary,(.1,.6,.9),'points',.11)
            clusters=row.get('clusters',[])
            report['walls'][which]={'boundary_points':len(boundary),'mouth_clusters':clusters,
                                    'candidate':asdict(row['result']) if row['result'] else None}
            for i,(lo,hi,count) in enumerate(clusters):
                support=boundary[(boundary[:,0]>=lo-.03)&(boundary[:,0]<=hi+.03)&
                    (sign*boundary[:,1]<=cfg.boundary_near_m+cfg.mouth_slice_depth_m)]
                self.add(which+'_wall_support_'+str(i),support,(0.,1.,1.),'points',.19)
                self.add(which+'_mouth_cluster_'+str(i),[(lo,sign*cfg.boundary_near_m),(hi,sign*cfg.boundary_near_m)],(0.,1.,1.),size=.14)
            candidate=row['result']
            if candidate:
                a,b=candidate.start_x,candidate.end_x
                depth=candidate.back_wall_distance or candidate.side_distance
                self.add(which+'_candidate',[(a,0),(a,sign*depth),(b,sign*depth),(b,0),(a,0)],(.1,1.,.2),size=.10)
                if candidate.back_wall_distance is not None:
                    margin=min(.18,max(.05,.15*(b-a)))
                    mask=(pts[:,0]>=a+margin)&(pts[:,0]<=b-margin)&(distance>=cfg.perpendicular_min_depth_m)
                    ids=np.flatnonzero(mask)
                    if len(ids):
                        bins=np.round(distance[ids]/cfg.back_wall_bin_m).astype(int)
                        unique,counts=np.unique(bins,return_counts=True)
                        valid=unique[counts>=cfg.back_wall_min_points]
                        if len(valid): self.add(which+'_back_wall_support',pts[ids[bins==max(valid)]],(1.,.9,.1),'points',.20)
        if space:
            self.add('parking_goal',[(space.goal_pose_map.x,space.goal_pose_map.y)],(.15,1.,.2),'points',.35)
        paths={}
        for key,color in [('approach_map',(1.,.6,.05)),('reverse_map',(1.,.1,.8))]:
            path=captured.get(key,[]); paths[key]=[asdict(p) for p in path]
            self.add(key,[(p.x,p.y) for p in path],color,size=.11)
        stage=captured.get('stage_lane')
        if stage: self.add('reverse_start',[(stage.x,stage.y)],(1.,.6,.05),'points',.35)
        for key,pathkey in [('approach_collision','approach_map'),('reverse_collision','reverse_map')]:
            hit=captured.get(key)
            if hit:
                report['collision']={'phase':key,**asdict(hit)}
                self.add('collision',[(hit.point_x,hit.point_y)],(1.,.05,.05),'points',.38)
                at=captured[pathkey][hit.path_index]
                pc=self.planner.config; f=pc.vehicle_front_m+pc.static_clearance_m
                r=pc.vehicle_rear_m+pc.static_clearance_m; w=pc.vehicle_width_m/2+pc.static_clearance_m
                corners=np.array([[-r,-w],[f,-w],[f,w],[-r,w],[-r,-w]])
                self.add('collision_footprint',transform_points(corners,at.pose),(1.,.1,.1),size=.1)
        report['paths']=paths; self.report_data=report
        (OUT/'latest.json').write_text(json.dumps(report,indent=2))
        if time.monotonic()-self.last_plot>4.:
            self.plot(pose); self.last_plot=time.monotonic()

    def plot(self,pose):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,ax=plt.subplots(figsize=(10,8),layout='constrained')
        for name,xy,color,kind,size in self.scene:
            if kind=='points': ax.scatter(xy[:,0],xy[:,1],s=12 if 'map'==name else 32,c=[color],label=name)
            else: ax.plot(xy[:,0],xy[:,1],color=color,lw=2,label=name)
        ax.scatter([pose.x],[pose.y],marker='^',s=130,c='black',label='current vehicle')
        ax.set_aspect('equal'); ax.grid(alpha=.3)
        ax.set_xlabel('Scan-lane forward x (m)'); ax.set_ylabel('Left +y (m)')
        d=self.report_data
        ax.set_title('LIVE MAP REPLAY — perception and rejected path\nActual node: '+d.get('actual_node_plan_error','')+'\nReplay: '+d.get('replay_plan_error',''))
        ax.legend(loc='upper left',bbox_to_anchor=(1.01,1.),fontsize=7)
        fig.savefig(OUT/'latest.png',dpi=125); plt.close(fig)

    def render(self):
        now=time.monotonic()
        required=('map','pose','gps','status','diagnostics')
        if any(k not in self.samples or now-self.samples[k][0]>.7 for k in required): return
        cloud=self.samples['map'][1]; pose_msg=self.samples['pose'][1]; gps=self.samples['gps'][1]
        if cloud.header.frame_id!=pose_msg.header.frame_id or not gps.position_valid or not gps.vehicle_heading_valid: return
        q=pose_msg.pose.orientation
        pose=Pose2(pose_msg.pose.position.x,pose_msg.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        stamp=(cloud.header.stamp.sec,cloud.header.stamp.nanosec)
        if stamp!=self.generation:
            self.generation=stamp; self.evaluate(cloud,pose,self.samples['status'][1])
        c,s=math.cos(pose.yaw),math.sin(pose.yaw)
        cg,sg=math.cos(gps.vehicle_heading_rad),math.sin(gps.vehicle_heading_rad)
        result=[]
        for i,(name,xy,color,kind,size) in enumerate(self.scene):
            if name=='map': continue
            local=(xy-np.array([pose.x,pose.y]))@np.array([[c,-s],[s,c]])
            world=local@np.array([[cg,sg],[-sg,cg]])+np.array([gps.position_x,gps.position_y])
            m=Marker(); m.header.frame_id='parking_inspection_map'; m.header.stamp=self.get_clock().now().to_msg()
            m.ns='perception_replay_'+name; m.id=i; m.action=Marker.ADD
            m.type=Marker.POINTS if kind=='points' else Marker.LINE_STRIP
            m.pose.orientation.w=1.; m.scale.x=m.scale.y=m.scale.z=size
            m.color.r,m.color.g,m.color.b,m.color.a=(*color,1.)
            m.points=[Point(x=float(x),y=float(y),z=.35) for x,y in world]
            result.append(m)
        text=Marker(); text.header.frame_id='parking_inspection_map'; text.ns='perception_replay_status'; text.id=0
        text.type=Marker.TEXT_VIEW_FACING; text.pose.orientation.w=1.
        text.pose.position=Point(x=gps.position_x+4.,y=gps.position_y+5.,z=.6)
        text.scale.z=.45; text.color.r=text.color.g=text.color.b=text.color.a=1.
        text.text='PERCEPTION REPLAY (not control)\n'+self.report_data.get('replay_plan_error','waiting')
        result.append(text); self.pub.publish(MarkerArray(markers=result))


def main():
    rclpy.init(); node=Debug()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__': main()
