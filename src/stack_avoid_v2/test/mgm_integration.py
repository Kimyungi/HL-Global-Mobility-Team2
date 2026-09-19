#!/usr/bin/env python3
"""Production planner + MGM with synthetic scans; no drivers, CAN or go publisher."""
from collections import Counter
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import yaml
if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
    raise SystemExit('Use an isolated ROS domain and ROS_LOCALHOST_ONLY=1')
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from fma_interfaces.msg import (AvoidPlan, AvoidStatus, GpsPath, GpsRoute, RefPoint,
    VehicleVector, LanePath, CanHealth, EstopRequest, TrafficStop, MgmState, TargetRef)


def main():
    install=Path(sys.argv[1]);rclpy.init();node=rclpy.create_node('wall_mgm_test');latest={};reasons=Counter()
    def receive(m,key):
        latest[key]=m
        if key=='plan':reasons[m.reason]+=1
    subs=[node.create_subscription(kind,topic,lambda m,k=key:receive(m,k),10)
          for kind,topic,key in [(AvoidPlan,'/avoid_v2/plan','plan'),(MgmState,'/adas/mgm_state','mgm'),(TargetRef,'/adas/target_ref','ref')]]
    gps=GpsPath(fix_quality=4,position_valid=True,vehicle_heading_valid=True,
        heading_source=GpsPath.HEADING_FUSED,position_x=1.,points=[RefPoint(x=2.5)],
        zone_valid=True,station_error_valid=True)
    route=GpsRoute(route_id='test');route.header.frame_id='map'
    route.points=[RefPoint(x=i*.25) for i in range(161)]
    route_pub=node.create_publisher(GpsRoute,'/perception/gps_route',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    from sensor_msgs.msg import LaserScan
    messages={'/perception/gps_path':gps,'/vehicle/vector':VehicleVector(x=1.,v=.6),
        '/perception/lane_path':LanePath(confidence=.9,points=[RefPoint(x=2.5)]),
        '/bridge/can_health':CanHealth(link_up=True,tx_ok=True),
        '/perception/estop':EstopRequest(scan_valid=True),'/perception/traffic_stop':TrafficStop()}
    pubs={topic:node.create_publisher(type(m),topic,1) for topic,m in messages.items()}
    scan_pubs={sid:node.create_publisher(LaserScan,f'/lidar/{sid}/scan',qos_profile_sensor_data) for sid in ['a1','b1','b2']}
    # Full-angle synthetic room feeds isolate integration from the measured hardware FOV.
    angles=[-math.pi+i*2*math.pi/1440 for i in range(1441)];ranges=[]
    for a in angles:
        dx,dy=math.cos(a),math.sin(a);dist=[]
        if abs(dx)>1e-9:dist.append(7./dx if dx>0 else -3./dx)
        if abs(dy)>1e-9:dist.append(4./abs(dy))
        ranges.append(min(dist))
    params={'control_enabled':True,'sensor_ids':['a1','b1','b2'],'trigger_sensor':'a1'}
    for sid in params['sensor_ids']:
        params.update({f'sensors.{sid}.frame':f'lidar_{sid}_link',f'sensors.{sid}.fov_min_deg':-180.,f'sensors.{sid}.fov_max_deg':180.})
    last_gps=last_scan=0.;enabled_scans=True
    def stamp(t):return rclpy.time.Time(nanoseconds=int(t*1e9)).to_msg()
    def spin(duration):
        nonlocal last_gps,last_scan
        end=time.monotonic()+duration
        while time.monotonic()<end:
            now=node.get_clock().now().nanoseconds*1e-9
            for topic,msg in messages.items():
                if topic=='/perception/gps_path' and time.monotonic()-last_gps<.1:continue
                msg.header.stamp=stamp(now-.01)
                if hasattr(msg,'reference_stamp'):msg.reference_stamp=stamp(now-.04 if topic=='/perception/gps_path' else now-.01)
                pubs[topic].publish(msg)
                if topic=='/perception/gps_path':last_gps=time.monotonic()
            if enabled_scans and time.monotonic()-last_scan>=.1:
                for sid in ['b1','b2','a1']:
                    scan=LaserScan(angle_min=-math.pi,angle_max=math.pi,angle_increment=2*math.pi/1440,
                        time_increment=.05/1440,scan_time=.1,range_min=.05,range_max=12.,ranges=ranges)
                    scan.header.frame_id=f'lidar_{sid}_link';scan.header.stamp=stamp(now-.12)
                    scan_pubs[sid].publish(scan)
                last_scan=time.monotonic()
            rclpy.spin_once(node,timeout_sec=.008)
    def expect(predicate,why,timeout=8.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            spin(.04)
            if {'plan','mgm','ref'}<=latest.keys() and predicate():print('PASS:',why,flush=True);return
        print('reasons:',reasons);print('plan:',latest.get('plan'));print('mgm:',latest.get('mgm'))
        raise AssertionError(why)
    with tempfile.TemporaryDirectory(prefix='wall_mgm_') as tmp:
        config=Path(tmp)/'params.yaml';config.write_text(yaml.safe_dump({'avoid_v2_node':{'ros__parameters':params}}))
        with open(Path(tmp)/'log','w+') as log:
            planner=subprocess.Popen([str(install/'stack_avoid_v2/lib/stack_avoid_v2/avoid_v2_node'),'--ros-args','--params-file',str(config)],stdout=log,stderr=log)
            mgm=subprocess.Popen([str(install/'adas_mgm/lib/adas_mgm/mgm_node'),'--ros-args',
                '-p','wait_go:=false','-p','avoid_v2_enabled:=true','-p','avoid_zone_only:=true',
                '-p','zone_enter_confirm_samples:=5','-p','zone_exit_confirm_samples:=5',
                '-p','safe_stop_all_sensors_only:=true','-p','escape_after_cycles:=0',
                '-p','required_lidar_topics:=[/lidar/a1/scan,/lidar/b1/scan,/lidar/b2/scan]'],stdout=log,stderr=log)
            try:
                spin(.3);route.header.stamp=node.get_clock().now().to_msg();route_pub.publish(route)
                expect(lambda:latest['mgm'].avoidance==0 and latest['ref'].v_ref>0,'outside zone retains navigation with inactive wall planner')
                gps.avoid_zone=True
                expect(lambda:latest['mgm'].avoidance==1 and latest['mgm'].reference_source==2 and latest['plan'].plan_valid and latest['ref'].v_ref>0,
                    'five GPS observations activate actual wall planner and MGM')
                episode=latest['plan'].episode_id
                route.header.stamp=node.get_clock().now().to_msg();route_pub.publish(route)
                spin(.3)
                assert latest['plan'].episode_id==episode
                expect(lambda:latest['ref'].v_ref>0,'identical route repeat preserves the automatic workspace session')
                # Freeze producer so the exact plan remains available for comparison before expiry.
                planner.send_signal(signal.SIGSTOP)
                spin(.012)
                p=latest['plan'];ref=latest['ref']
                assert p.control_enabled and p.reference.points
                assert abs(ref.ref_points[0].x-p.reference.points[0].x)<1e-5
                assert abs(ref.ref_points[0].y-p.reference.points[0].y)<1e-5
                assert abs(ref.v_ref-p.speed_limit_mps)<1e-5
                print('PASS: target geometry and adaptive speed reach MGM without blending',flush=True)
                expect(lambda:latest['ref'].v_ref==0 and latest['mgm'].reference_motion_blocked and latest['mgm'].avoidance==1,
                    'frozen planner expires to zero speed while AVOID stays active',1.)
                planner.send_signal(signal.SIGCONT)
                expect(lambda:latest['ref'].v_ref>0,'new valid plan resumes avoidance')
                messages['/vehicle/vector'].v=-.2
                expect(lambda:latest['ref'].v_ref==0 and latest['mgm'].avoidance==1,
                    'negative measured velocity cannot request forward avoidance')
                messages['/vehicle/vector'].v=.6
                expect(lambda:latest['ref'].v_ref>0,'forward motion after recovery needs no session reset')
                gps.avoid_zone=False
                spin(.7)
                assert latest['mgm'].avoidance==1
                print('PASS: geographic zone exit does not truncate the episode',flush=True)
                enabled_scans=False
                expect(lambda:latest['ref'].v_ref==0 and latest['mgm'].avoidance==1,'missing scans HOLD without switching to healthy navigation')
                enabled_scans=True
                expect(lambda:latest['ref'].v_ref>0,'three fresh inputs recover without a rear lidar')
            finally:
                planner.send_signal(signal.SIGCONT)
                for proc in [planner,mgm]:
                    if proc.poll() is None:proc.send_signal(signal.SIGINT)
                for proc in [planner,mgm]:proc.wait(timeout=10)
                log.seek(0)
                print(log.read()[-2500:])
    node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
