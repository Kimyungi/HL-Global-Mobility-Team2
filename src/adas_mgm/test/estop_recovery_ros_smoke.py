"""Stop-only ESTOP station wiring with isolated synthetic inputs; no hardware."""
import os
import math
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
        zone_enter_confirm_samples=3,zone_exit_confirm_samples=3, avoid_zone_only=True,
        estop_station_zone_id=9)
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
            front=messages[scans[0]]
            front.angle_min=-math.pi;front.angle_increment=math.pi/720
            front.ranges=[float('inf')]*1441
            gps.fix_quality=4
            messages['/bridge/can_health'].tx_ok=True
            zone=ZoneContext(zone_id=9,zone_type=0,zone_valid=True,in_zone=False)
            gps.zones=[zone]
            expect(lambda s,r:s.start_ready,'synthetic inputs ready')
            go.publish(Bool(data=True))
            expect(lambda s,r:s.go_authorized and r.v_ref>0,'GPS driving outside station')
            def obstacle(lo=-.12,hi=.12):
                values=[float('inf')]*1441
                for i in range(1441):
                    a=front.angle_min+i*front.angle_increment
                    if math.cos(a)<=0:continue
                    distance=1.0/math.cos(a)
                    if lo<=distance*math.sin(a)<=hi:values[i]=distance
                front.ranges=values
            obstacle();pump(.5)
            assert not states[-1].estop_active, 'outside station must ignore obstacle'
            zone.in_zone=True
            expect(lambda s,r:s.estop_active and r.state==5 and r.v_ref==0,'station front width enters ESTOP')
            episode=states[-1].estop_request_id
            old=EstopRecovery(request_id=episode,done=True,v_suggest=-.3)
            old.header.stamp=old.reference_stamp=node.get_clock().now().to_msg()
            recovery.publish(old);pump(.4)
            assert states[-1].estop_active and refs[-1].v_ref==0,'old reverse/done ignored'
            muted.add(scans[0]);pump(.6)
            assert states[-1].estop_active and refs[-1].v_ref==0,'front sensor loss cannot release'
            muted.clear();front.ranges=[float('nan')]*1441;pump(.4)
            assert states[-1].estop_active,'invalid scan cannot release'
            obstacle(-.05,.05);pump(.4)
            assert states[-1].estop_active,'narrow remnant keeps stop'
            front.ranges=[float('inf')]*1441
            expect(lambda s,r:not s.estop_active and r.v_ref>0,'new clear scans restore GPS')
            obstacle();pump(.5)
            assert not states[-1].estop_active,'same station cannot retrigger'
            zone.in_zone=False;pump(.5)
            zone.in_zone=True
            expect(lambda s,r:s.estop_active and r.v_ref==0,'confirmed station exit rearms')
            assert all(ref.v_ref>=0 for ref in refs),'stop-only ESTOP never reverses'
            print('PASS: stop-only station ESTOP ROS wiring')
        finally:
            proc.terminate(); proc.wait(timeout=5); log.close()
            node.destroy_node(); rclpy.try_shutdown()

if __name__=='__main__': main()
