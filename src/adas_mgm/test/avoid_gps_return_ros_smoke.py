"""Isolated MGM wiring test; no sensor drivers, CAN bridge or vehicle commands."""
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
    raise SystemExit('Use ROS_LOCALHOST_ONLY=1 and a separate ROS_DOMAIN_ID')

import rclpy
from fma_interfaces.msg import (
    AvoidStatus, CanHealth, EstopRequest, GpsPath, LanePath, MgmState,
    RefPoint, TargetRef, TrafficStop, VehicleVector,
)


def main():
    rclpy.init()
    node = rclpy.create_node('avoid_return_synthetic_inputs')
    gps = GpsPath(fix_quality=4, points=[RefPoint(x=2.5, y=.2)], zone_valid=True,
                  heading_source=GpsPath.HEADING_FUSED, vehicle_heading_valid=True,
                  cross_track_m=.5, station_yaw_error_rad=.5, station_error_valid=True)
    avoid = AvoidStatus(scan_valid=True, ttc=100., v_suggest=.6,
                        points=[RefPoint(x=3., y=.5)])
    estop = EstopRequest(scan_valid=True)
    messages = {
        '/perception/gps_path': gps, '/perception/avoid': avoid, '/perception/estop': estop,
        '/perception/lane_path': LanePath(confidence=.9, points=[RefPoint(x=2.5, y=.1)]),
        '/perception/traffic_stop': TrafficStop(), '/vehicle/vector': VehicleVector(),
        '/bridge/can_health': CanHealth(link_up=True, tx_ok=True),
    }
    pubs = {topic: node.create_publisher(type(msg), topic, 1) for topic, msg in messages.items()}
    latest = {}
    subs = [node.create_subscription(kind, topic, lambda msg,k=key: latest.update({k: msg}), 10)
            for kind, topic, key in [(MgmState, '/adas/mgm_state', 'state'),
                                     (TargetRef, '/adas/target_ref', 'ref')]]
    paused = set()
    with tempfile.TemporaryFile(mode='w+') as log:
        proc = subprocess.Popen([sys.argv[1], '--ros-args', '-p', 'wait_go:=false',
            '-p', 'base_state_machine_enabled:=true', '-p', 'avoidance_enabled:=true',
            '-p', 'avoid_max_cycles:=0', '-p', 'escape_after_cycles:=10',
            '-p', 'escape_max_cycles:=30', '-p', 'escape_require_rear_clear:=false',
            '-p', 'v_escape:=-0.8', '-p', 'v_base:=1.0'], stdout=log, stderr=subprocess.STDOUT)

        def spin(duration):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                for topic, msg in messages.items():
                    if topic in paused:
                        continue
                    msg.header.stamp = node.get_clock().now().to_msg()
                    if hasattr(msg, 'reference_stamp'):
                        msg.reference_stamp = msg.header.stamp
                    pubs[topic].publish(msg)
                rclpy.spin_once(node, timeout_sec=.01)
                if proc.poll() is not None:
                    log.seek(0)
                    raise AssertionError(log.read())

        def expect(check, description):
            end = time.monotonic() + 3
            while time.monotonic() < end:
                spin(.02)
                if 'state' in latest and 'ref' in latest and check(latest['state'], latest['ref']):
                    print('PASS:', description)
                    return
            s = latest.get('state'); r = latest.get('ref')
            raise AssertionError(f'{description}: avoid={getattr(s,"avoidance",None)} '
                f'source={getattr(s,"reference_source",None)} v={getattr(r,"v_ref",None)}')

        try:
            expect(lambda s,r: s.top==1 and r.v_ref>0, 'mock LINE driving')
            assert node.count_publishers('/adas/target_ref') == 1
            assert node.count_subscribers('/adas/target_ref') == 1
            avoid.obstacle_detected = avoid.avoidable = True
            expect(lambda s,r: s.avoidance==1 and s.reference_source==2, 'obstacle maneuver')
            avoid.obstacle_detected = False; avoid.maneuver_done = True
            expect(lambda s,r: s.avoidance==3 and s.reference_source==1 and r.state==2 and
                   s.avoid_return_error_valid and abs(s.avoid_return_cross_track_m-.5)<1e-5,
                   'GPS return keeps AVOID and carries station error through ROS')
            gps.cross_track_m = .05; gps.station_yaw_error_rad = math.radians(21)
            spin(.2); assert latest['state'].avoidance == 3
            gps.station_yaw_error_rad = 0.; gps.station_error_valid = False
            spin(.2); assert latest['state'].avoidance == 3
            paused.add('/perception/gps_path')
            expect(lambda s,r: s.avoidance==3 and s.reference_source==1 and r.v_ref==0,
                   'GPS timeout holds return state and stops')
            paused.clear(); gps.station_error_valid = True; gps.station_yaw_error_rad = math.radians(-20)
            expect(lambda s,r: s.avoidance==0 and r.v_ref>0, 'both station thresholds release return')
            avoid.maneuver_done = False; avoid.avoidable = False
            estop.estop = True
            expect(lambda s,r: s.avoidance==1 and s.reference_source==4 and r.v_ref<0,
                   'E-stop reverse immediately enters AVOID')
            estop.estop = False
            expect(lambda s,r: s.avoidance==1 and s.reference_source==2 and r.v_ref>=0,
                   'reverse exit preserves obstacle maneuver')
            print('ROS avoidance return wiring passed')
        finally:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=10)
            node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
