"""Revised v2 wiring with synthetic inputs, never CAN/camera/GPS hardware."""
import os
import subprocess
import sys
import tempfile
import time
import yaml
from pathlib import Path
if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
    raise SystemExit('isolated localhost ROS domain required')
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Header
from sensor_msgs.msg import LaserScan
from fma_interfaces.msg import (CanHealth, EstopRequest, EstopRecovery, GpsPath,
    LanePath, MgmState, RefPoint, TargetRef, VehicleVector, ZoneContext, TrafficStop)


def main():
    rclpy.init()
    node = rclpy.create_node('revised_v2_synthetic')
    scans = ['/lidar/a1/scan', '/lidar/a2/scan', '/lidar/b1/scan', '/lidar/b2/scan']
    kinds = {'/perception/gps_path': GpsPath, '/perception/lane_path': LanePath,
        '/perception/lane_camera': Header, '/bridge/can_health': CanHealth,
        '/vehicle/vector': VehicleVector, '/perception/estop': EstopRequest,
        '/perception/traffic_stop': TrafficStop}
    kinds.update({s: LaserScan for s in scans})
    messages = {name: kind() for name, kind in kinds.items()}
    pubs = {name: node.create_publisher(kind,name,qos_profile_sensor_data if kind is LaserScan else 1)
            for name,kind in kinds.items()}
    for name in scans:
        scan = messages[name]; scan.angle_min=0.; scan.angle_increment=.1
        scan.range_min=.1; scan.range_max=12.; scan.ranges=[float('inf')]*10
    gps, lane = messages['/perception/gps_path'], messages['/perception/lane_path']
    gps.fix_quality=5; gps.points=[RefPoint(x=2.5,y=.2)]
    gps.position_valid=True; gps.zone_valid=True
    lane.confidence=.9; lane.points=[RefPoint(x=2.5,y=.1)]
    messages['/bridge/can_health'].link_up=True
    messages['/perception/estop'].estop=True  # must be ignored
    states, refs = [], []
    node.create_subscription(MgmState,'/adas/mgm_state',states.append,10)
    node.create_subscription(TargetRef,'/adas/target_ref',refs.append,10)
    go=node.create_publisher(Bool,'/operator/go',1)
    stop=node.create_publisher(Bool,'/operator/stop',1)
    recovery=node.create_publisher(EstopRecovery,'/planning/estop_recovery',1)
    params = dict(revised_v2_enabled=True, wait_go=True, lidar_estop_enabled=False,
        escape_after_cycles=0, v_base=2., required_lidar_topics=scans,
        zone_enter_confirm_samples=5,zone_exit_confirm_samples=5, avoid_zone_only=True)
    # Test-only aligned mounts, explicit frame; production launch reads calibration YAML.
    for direction in ['front','left','right']:
        params['estop_mount.'+direction]=[.76,0.,0.,-180.,180.,0.,.1,12.]
    with tempfile.TemporaryDirectory() as tmp:
        file=Path(tmp)/'params.yaml'; file.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
        log=tempfile.TemporaryFile(mode='w+')
        proc=subprocess.Popen([sys.argv[1],'--ros-args','--params-file',str(file)],stdout=log,stderr=subprocess.STDOUT)
        muted=set()
        def pump(duration=.15):
            end=time.monotonic()+duration
            while time.monotonic()<end:
                stamp=node.get_clock().now().to_msg()
                for topic,msg in messages.items():
                    if topic in muted: continue
                    if isinstance(msg,Header): msg.stamp=stamp
                    else: msg.header.stamp=stamp
                    if hasattr(msg,'reference_stamp'): msg.reference_stamp=stamp
                    pubs[topic].publish(msg)
                rclpy.spin_once(node,timeout_sec=.025)
                if proc.poll() is not None:
                    log.seek(0); raise AssertionError(log.read())
        def expect(condition,label):
            end=time.monotonic()+4.
            while time.monotonic()<end:
                pump()
                if states and refs and condition(states[-1],refs[-1]):
                    print('PASS:',label); return
            raise AssertionError(label+f' state={states[-1:]} target={refs[-1:]}')
        try:
            muted.add(scans[1]); pump(1.)
            expect(lambda s,r: s.revised_v2 and not s.start_ready,'rear required for departure')
            muted.clear(); pump(1.)
            expect(lambda s,r: not s.start_ready,'camera with FLOAT cannot authorize departure')
            gps.fix_quality=4
            expect(lambda s,r: s.start_ready,'all four raw sensors plus FIXED GPS ready')
            go.publish(Bool(data=True))
            expect(lambda s,r: s.go_authorized and r.v_ref>0 and s.navigation==1,'GPS drives despite good lane')
            gps.fix_quality=5
            expect(lambda s,r:r.v_ref==0 and s.navigation==1,'FLOAT stops without lane fallback')
            lane.confidence=.9
            pump(1.)
            expect(lambda s,r:r.v_ref==0 and not s.line_ref_valid,'high lane confidence cannot resume driving')
            zone=ZoneContext(zone_id=3,zone_type=1,zone_valid=True,in_zone=True)
            gps.zones=[zone]
            expect(lambda s,r:s.traffic_zone_active and r.v_ref==0,'FLOAT enters turn zone but cannot drive')
            gps.fix_quality=4
            expect(lambda s,r:s.navigation==2 and r.v_ref>0,'FIXED resumes zone GPS')
            zone.in_zone=False
            expect(lambda s,r:not s.traffic_zone_active,'confirmed zone exit switches detector off')
            muted.add(scans[1])
            expect(lambda s,r:not s.lidar_ready and r.v_ref>0,'rear loss alone does not stop runtime')
            muted.clear()
            messages['/vehicle/vector'].v=.03; pump(2.2)
            messages['/vehicle/vector'].v=0.
            messages[scans[0]].ranges=[.2]*10
            expect(lambda s,r:s.estop_active and s.safety==4 and r.v_ref==0,'raw front scans enter upper ESTOP; no executor holds zero')
            episode=states[-1].estop_request_id
            done=EstopRecovery(request_id=episode,done=True)
            done.header.stamp=done.reference_stamp=node.get_clock().now().to_msg()
            recovery.publish(done)
            expect(lambda s,r:not s.estop_active and r.v_ref>0,'matching completion returns to navigation')
            pump(.4); assert not states[-1].estop_active, 'same uncleared front must not retrigger'
            stop.publish(Bool(data=True))
            expect(lambda s,r:not s.go_authorized and r.v_ref==0,'operator stop overrides and revokes go')
        finally:
            proc.terminate(); proc.wait(timeout=5); log.close()
            node.destroy_node(); rclpy.try_shutdown()

if __name__=='__main__': main()
