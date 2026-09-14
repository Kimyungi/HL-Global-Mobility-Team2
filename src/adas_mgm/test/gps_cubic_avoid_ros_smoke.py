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
            procs.append(subprocess.Popen([str(root/'install_v2/stack_avoid/lib/stack_avoid/stack_avoid_node')]+
                common+['--params-file', str(root/'src/stack_avoid/config/params.yaml'),
                        '-p', 'lidar_mount.forward_angle_deg:=0.0',
                        '-p', f'avoid.require_mgm_active:={str(zone_entry).lower()}'],stdout=log,stderr=log))
            procs.append(subprocess.Popen([str(root/'install_v2/adas_mgm/lib/adas_mgm/mgm_node')]+
                common+['-p','wait_go:=false','-p','base_state_machine_enabled:=true',
                        '-p','avoidance_enabled:=true','-p','avoid_max_cycles:=0',
                        '-p',f'avoid_zone_only:={str(zone_entry).lower()}',
                        '-p','zone_enter_confirm_samples:=5','-p','zone_exit_confirm_samples:=5',
                        '-p','escape_after_cycles:=0','-p','blend_cycles:=0'],stdout=log,stderr=log))
            def expect(condition, text):
                end = time.monotonic()+8.
                while time.monotonic()<end:
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
            expect(lambda m: m['ref'].v_ref>0 and len(m['avoid'].points)==(0 if zone_entry else 1),
                   'clear scene permits normal navigation; zone mode waits for MGM activation')
            if zone_entry:
                gps.avoid_zone = True
                expect(lambda m: m['state'].avoidance==1 and m['state'].reference_source==2 and
                       len(m['avoid'].points)==1 and not m['avoid'].obstacle_detected,
                       'marker enters AVOID with no obstacle and activates the real planner')
            scan.ranges=[1.9/math.cos(math.radians(i-90))
                         if abs(1.9*math.tan(math.radians(i-90)))<=.2 else math.inf
                         for i in range(181)]
            expect(lambda m: m['state'].reference_source==2 and m['ref'].v_ref>0 and
                   abs(m['ref'].ref_points[0].x-2.66)<.01 and
                   abs(m['ref'].ref_points[0].y)>.5 and len(m['path'].poses)>10,
                   'side point from cubic path reaches MGM as the actual control target')
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
                gps.position_x = gps.waypoint_station_m = 5.3
                gps.waypoint_points = [RefPoint(x=float(i)-5.3) for i in range(13)]
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
