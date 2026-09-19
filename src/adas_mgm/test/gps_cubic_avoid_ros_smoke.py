"""Mock GPS/scans -> production avoidance -> production MGM, isolated/no CAN."""
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
    raise SystemExit('Set ROS_LOCALHOST_ONLY=1 and a separate ROS_DOMAIN_ID')

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path as RosPath
from fma_interfaces.msg import (GpsPath, AvoidStatus, RefPoint, LanePath,
    VehicleVector, EstopRequest, CanHealth, TrafficStop, TargetRef, MgmState)


def main():
    root = Path(sys.argv[1]).resolve()
    zone_entry = '--zone-entry' in sys.argv[2:]
    compute_backend = 'native' if '--native' in sys.argv[2:] else 'python'
    two_obstacles = '--two-obstacles' in sys.argv[2:]
    main_gap = '--main-gap' in sys.argv[2:]
    main_gap_path = '--main-gap-path' in sys.argv[2:]
    rclpy.init()
    node = rclpy.create_node('gps_cubic_synthetic_inputs')
    gps = GpsPath(fix_quality=4, position_valid=True, vehicle_heading_valid=True,
                  heading_source=GpsPath.HEADING_FUSED, waypoint_window_valid=True,
                  waypoint_station_m=0., waypoint_stations=list(map(float, range(13))),
                  waypoint_points=[RefPoint(x=float(i)) for i in range(13)],
                  points=[RefPoint(x=2.5)], zone_valid=True,
                  station_error_valid=True)
    scan = LaserScan(angle_min=-math.pi/2, angle_max=math.pi/2,
                     angle_increment=math.pi/180, range_min=.05, range_max=12.,
                     ranges=[math.inf]*181)
    messages = {
        '/perception/gps_path': gps, '/scan': scan,
        '/perception/lane_path': LanePath(confidence=.9, points=[RefPoint(x=2.5)]),
        '/vehicle/vector': VehicleVector(v=.6),
        '/perception/estop': EstopRequest(scan_valid=True),
        '/bridge/can_health': CanHealth(link_up=True, tx_ok=True),
        '/perception/traffic_stop': TrafficStop(),
    }
    if main_gap_path:
        messages['/vehicle/vector'].v = 0.
    pubs = {topic: node.create_publisher(type(msg), topic,
              qos_profile_sensor_data if topic=='/scan' else 1) for topic, msg in messages.items()}
    latest = {}
    subs = [node.create_subscription(kind, topic, lambda m,k=key: latest.update({k:m}), 10)
            for kind, topic, key in [(AvoidStatus, '/perception/avoid', 'avoid'),
                                     (RosPath, '/perception/avoid_path', 'path'),
                                     (MgmState, '/adas/mgm_state', 'state'),
                                     (TargetRef, '/adas/target_ref', 'ref')]]
    with tempfile.TemporaryFile(mode='w+') as log:
        common = ['--ros-args']
        procs = []
        try:
            procs.append(subprocess.Popen([str(root/'install_v2/stack_avoid/lib/stack_avoid'/('main_gap_path_trial' if main_gap_path else 'main_gap_trial' if main_gap else 'stack_avoid_node'))]+
                common+['--params-file', str(root/'src/stack_avoid/config'/('params_main_gap.yaml' if main_gap or main_gap_path else 'params.yaml')),
                        '-p', 'lidar_mount.forward_angle_deg:=0.0',
                        '-p', f'avoid.compute_backend:={compute_backend}',
                        '-p', f'avoid.require_mgm_active:={str(zone_entry).lower()}']+
                        (['-p','target_speed_mps:=2.0'] if main_gap_path else []),stdout=log,stderr=log))
            procs.append(subprocess.Popen([str(root/'install_v2/adas_mgm/lib/adas_mgm/mgm_node')]+
                common+['-p','wait_go:=false','-p','base_state_machine_enabled:=true',
                        '-p','avoidance_enabled:=true','-p','avoid_max_cycles:=0',
                        '-p',f'avoid_zone_only:={str(zone_entry).lower()}',
                        '-p','zone_enter_confirm_samples:=5','-p','zone_exit_confirm_samples:=5',
                        '-p','escape_after_cycles:=0','-p','blend_cycles:=0']+
                        (['-p','v_base:=2.0','-p','v_avoid:=1.0'] if main_gap_path else []),stdout=log,stderr=log))
            update_inputs = lambda: None
            def expect(condition, text):
                end = time.monotonic()+8.
                while time.monotonic()<end:
                    update_inputs()
                    stamp=node.get_clock().now().to_msg()
                    for topic,msg in messages.items():
                        msg.header.stamp=stamp
                        if hasattr(msg,'reference_stamp'):msg.reference_stamp=stamp
                        pubs[topic].publish(msg)
                    rclpy.spin_once(node,timeout_sec=.03)
                    if len(latest)==4 and condition(latest):
                        print('PASS:',text,flush=True);return
                log.seek(0);print(log.read()[-6000:])
                print({key: (str(value)[:2500]) for key,value in latest.items() if key != 'path'})
                raise AssertionError(text)
            expect(lambda m: m['ref'].v_ref>0 and len(m['avoid'].points)==(0 if zone_entry and not main_gap_path else 1),
                   'clear scene permits normal navigation; zone mode waits for MGM activation')
            if main_gap_path:
                expect(lambda m: abs(m['ref'].v_ref-2.)<.01, 'normal driving commands 2m/s')
            if zone_entry:
                gps.avoid_zone = True
                expect(lambda m: m['state'].avoidance==1 and m['state'].reference_source==2 and
                       len(m['avoid'].points)==1 and not m['avoid'].obstacle_detected,
                       'marker enters AVOID with no obstacle and activates the real planner')
            if main_gap_path:
                expect(lambda m: abs(m['ref'].v_ref-1.)<.01, 'zone waiting commands 1m/s')
            scan.ranges=[1.9/math.cos(math.radians(i-90))
                         if abs(1.9*math.tan(math.radians(i-90)))<=.2 else math.inf
                         for i in range(181)]
            if main_gap_path:
                expect(lambda m: m['state'].reference_source==2 and m['ref'].v_ref>0 and
                       len(m['path'].poses)>50 and len(m['avoid'].points)==1 and
                       .7<m['ref'].ref_points[0].x<1.1 and
                       abs(m['ref'].ref_points[0].curvature)>.01,
                       'A-B-C path preview and actual curvature reach MGM')
                fixed = [(p.pose.position.x, p.pose.position.y,
                          2*math.atan2(p.pose.orientation.z, p.pose.orientation.w))
                         for p in latest['path'].poses]
                assert abs(fixed[-1][0]-5.36)<.02 and abs(fixed[-1][1])<.001
                gps.position_valid=False
                gps.waypoint_window_valid=False
                scan.ranges=[math.inf]*181
                expect(lambda m: not m['avoid'].points and m['ref'].v_ref==0 and
                       m['path'].header.frame_id=='map' and len(m['path'].poses)>50 and not m['avoid'].maneuver_done,
                       'GPS position loss stops control while retaining absolute map path')
                gps.position_valid=True
                expect(lambda m: bool(m['avoid'].points) and m['ref'].v_ref>0,
                       'restored GPS pose resumes stored map path without a new waypoint window')
                vv = messages['/vehicle/vector']
                vv.x=vv.y=9999.;vv.yaw=-2.
                gps.vehicle_heading_valid=False
                expect(lambda m: not m['avoid'].points and m['ref'].v_ref==0 and len(m['path'].poses)>50,
                       'unknown GPS heading stops control and retains map path')
                gps.vehicle_heading_valid=True
                gps.waypoint_window_valid=True
                expect(lambda m: bool(m['avoid'].points) and m['ref'].v_ref>0,
                       'measured GPS heading resumes path independently of CAN x/y/yaw')
                gps.cross_track_m=.5
                # A newly observed wall must not erase the accepted reference.
                scan.ranges = [.8/math.cos(math.radians(i-90))
                               if 10 < i < 170 else math.inf for i in range(181)]
                messages['/perception/estop'].estop = True
                expect(lambda m: m['ref'].v_ref==0 and bool(m['avoid'].points) and
                       len(m['path'].poses)>50,
                       'independent E-stop stops motion while frozen path survives a new wall')
                messages['/perception/estop'].estop = False
                scan.ranges=[math.inf]*181
                expect(lambda m: m['ref'].v_ref>0 and bool(m['avoid'].points),
                       'E-stop release reuses frozen reference')
                from stack_avoid.station_path import StationPath
                route = StationPath([(*p, 0.) for p in fixed])
                started = time.monotonic()
                def follow_curve():
                    station = min(route.s[-1], time.monotonic()-started)
                    gps.position_x,gps.position_y,gps.vehicle_heading_rad = route.at(station)[:3]
                    vv.v = 1. if station < route.s[-1] else 0.
                    vv.x = vv.y = 1000.  # RX x/y must not move the GPS map path.
                    if station < route.s[-1]-.2:
                        assert [(p.pose.position.x,p.pose.position.y) for p in latest['path'].poses] == [(p[0],p[1]) for p in fixed]
                update_inputs = follow_curve
                expect(lambda m: m['state'].avoidance==3 and m['state'].reference_source==1 and
                       abs(m['ref'].v_ref-1.)<.01,
                       'GPS vehicle pose reaches fixed map C; every published path point stays unchanged')
                update_inputs = lambda: None
                gps.cross_track_m=0.
                expect(lambda m: m['state'].avoidance==0 and abs(m['ref'].v_ref-2.)<.01,
                       'GPS alignment completes A-B-C zone episode and restores 2m/s')
                return
            if main_gap:
                expect(lambda m: m['state'].reference_source==2 and m['ref'].v_ref>0 and
                       abs(m['ref'].ref_points[0].x-2.66/20.)<.001 and
                       abs(m['ref'].ref_points[0].y)>.02 and
                       abs(m['ref'].ref_points[0].yaw)>.1 and
                       m['ref'].ref_points[0].curvature==0.,
                       'main gap goal/20 and bearing reach production MGM')
                gps.waypoint_window_valid=False
                expect(lambda m: m['ref'].v_ref>0 and len(m['avoid'].points)==1,
                       'main obstacle steering uses LiDAR without GPS waypoint window')
                held_scan = messages.pop('/scan')
                expect(lambda m: m['ref'].v_ref==0 and m['state'].reference_source==2,
                       'missing scan makes main reference stale and stops MGM')
                messages['/scan'] = held_scan
                expect(lambda m: m['ref'].v_ref>0,
                       'fresh gap reference releases stop')
                gps.waypoint_window_valid=True
                gps.cross_track_m=.5
                scan.ranges=[math.inf]*181
                expect(lambda m: not m['avoid'].obstacle_detected and
                       len(m['avoid'].points)==1 and
                       abs(m['avoid'].points[0].x-1.5/20.)<.001 and
                       m['state'].avoidance==1,
                       'main clearance interval follows scaled straight hold point')
                expect(lambda m: m['state'].avoidance==3 and m['state'].reference_source==1,
                       'main clearance completion requests GPS return')
                gps.cross_track_m=0.
                expect(lambda m: m['state'].avoidance==0 and m['ref'].v_ref>0,
                       'GPS alignment completes the zone episode')
                return
            if two_obstacles:
                # Rear obstacle is partially visible above the front contour.
                scan.ranges = [min(r, 4./math.cos(math.radians(i-90)))
                               if .15 <= 4.*math.tan(math.radians(i-90)) <= .55 else r
                               for i, r in enumerate(scan.ranges)]
            expect(lambda m: m['state'].reference_source==2 and m['ref'].v_ref>0 and
                   abs(m['ref'].ref_points[0].x-2.66)<.01 and
                   abs(m['ref'].ref_points[0].y)>.5 and len(m['path'].poses)>10,
                   'side point from cubic path reaches MGM as the actual control target')
            first_y = latest['ref'].ref_points[0].y
            gps.waypoint_window_valid=False
            expect(lambda m: m['state'].reference_source==2 and m['ref'].v_ref==0 and
                   not m['avoid'].points and not m['path'].poses,
                   'missing GPS station window clears path and stops despite healthy LINE')
            gps.waypoint_window_valid=True
            expect(lambda m: m['state'].reference_source==2 and m['ref'].v_ref>0 and
                   len(m['avoid'].points)==1 and len(m['path'].poses)>10,
                   'restored GPS window rebuilds path and resumes avoidance')
            if zone_entry:
                scan.ranges = [math.inf]*181
                cleared = time.monotonic()
                expect(lambda m: time.monotonic()-cleared>.2 and not m['avoid'].obstacle_detected,
                       'clear scan reaches planner before advancing synthetic GPS pose')
                if two_obstacles:
                    gps.position_x = gps.waypoint_station_m = 2.8
                    gps.position_y = first_y
                    gps.waypoint_points = [RefPoint(x=float(i)-2.8, y=-first_y) for i in range(13)]
                    expect(lambda m: m['state'].avoidance==1 and m['ref'].v_ref>0 and
                           abs(m['ref'].ref_points[0].x-(4.76-2.8))<.05,
                           'passing G1 selects fixed G2 without early GPS return')
                final_station = 7.4 if two_obstacles else 5.3
                gps.position_x = gps.waypoint_station_m = final_station
                gps.position_y = 0.
                gps.waypoint_points = [RefPoint(x=float(i)-final_station) for i in range(13)]
                expect(lambda m: m['state'].avoidance==0 and m['ref'].v_ref>0 and not m['avoid'].points,
                       'actual cubic waypoint return releases AVOID and deactivates planner')
                start = time.monotonic()
                expect(lambda m: time.monotonic()-start>1. and m['state'].avoidance==0 and not m['avoid'].points,
                       'consumed marker stays inactive while GPS avoid_zone remains true')
        finally:
            for proc in procs:
                if proc.poll() is None: proc.send_signal(signal.SIGINT)
            for proc in procs: proc.wait(timeout=10)
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
