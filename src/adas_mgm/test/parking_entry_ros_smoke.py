"""Exercise the installed MGM command/status handoff with mock inputs only."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '178':
    raise SystemExit('Run in isolated localhost ROS_DOMAIN_ID=178')

import rclpy
from std_msgs.msg import Bool
from fma_interfaces.msg import (EstopRequest, GpsPath, LanePath, MgmState,
                               ParkingCommand, ParkingStatus, RefPoint, TargetRef,
                               VehicleVector, ZoneContext)


def main():
    rclpy.init()
    node = rclpy.create_node('parking_entry_mock_inputs')
    msgs = {
        '/perception/gps_path': GpsPath(), '/perception/lane_path': LanePath(),
        '/perception/parking': ParkingStatus(), '/perception/estop': EstopRequest(),
        '/vehicle/vector': VehicleVector(),
    }
    pubs = {topic: node.create_publisher(type(msg), topic, 10) for topic, msg in msgs.items()}
    gps, lane, parking = (msgs[t] for t in ('/perception/gps_path','/perception/lane_path','/perception/parking'))
    gps.points=[RefPoint(x=2.5)];gps.fix_quality=4;gps.zone_valid=True;gps.position_valid=True
    gps.route.enabled=True;gps.route.sequence_id=11;gps.route.instance_id=22
    gps.route.count=2;gps.route.required_missions=[0]
    zone=ZoneContext(zone_valid=True,zone_id=10,zone_type=2,mission_type=1,mission_id=0,in_zone=False)
    gps.zones=[zone]
    lane.points=[RefPoint(x=2.5)];lane.confidence=.9
    msgs['/perception/estop'].scan_valid=True
    states, refs, commands = [], [], []
    subscriptions = [
        node.create_subscription(MgmState,'/adas/mgm_state',states.append,100),
        node.create_subscription(TargetRef,'/adas/target_ref',refs.append,100),
        node.create_subscription(ParkingCommand,'/parking/mission_command',commands.append,100),
    ]
    go=node.create_publisher(Bool,'/operator/go',10)
    with tempfile.TemporaryDirectory(prefix='parking-entry-ros-') as folder:
        log_path=Path(folder)/'mgm.log'
        with log_path.open('w+') as log:
            proc=subprocess.Popen([sys.argv[1],'--ros-args','-p','wait_go:=true',
                '-p','route_sequence_enabled:=true','-p','avoidance_enabled:=false',
                '-p','zone_enter_confirm_samples:=5','-p','zone_exit_confirm_samples:=5',
                '-p','parking_zone_entry_active:=true','-p','parking_search_zone_only:=false'],
                stdout=log,stderr=subprocess.STDOUT)
            def spin(seconds=.15):
                deadline=time.monotonic()+seconds
                while time.monotonic()<deadline:
                    assert proc.poll() is None, 'MGM exited'
                    stamp=node.get_clock().now().to_msg()
                    for topic,msg in msgs.items():
                        msg.header.stamp=stamp
                        if hasattr(msg,'reference_stamp'):msg.reference_stamp=stamp
                        if isinstance(msg,ParkingStatus) and msg.preparation_ready:msg.preparation_stamp=stamp
                        pubs[topic].publish(msg)
                    until=time.monotonic()+.04
                    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
            def wait_for(predicate, label):
                deadline=time.monotonic()+4
                while time.monotonic()<deadline:
                    spin(.05)
                    if predicate():print('PASS:',label,flush=True);return
                raise AssertionError(label)
            try:
                wait_for(lambda:states and go.get_subscription_count()>0,'connected mock inputs')
                spin(.5);go.publish(Bool(data=True))
                wait_for(lambda:states[-1].top==1 and refs[-1].v_ref>0,'normal navigation before entry')
                zone.in_zone=True;gps.zones=[zone]
                wait_for(lambda:states[-1].mission==1 and commands,'immediate ACTIVE with PREPARE command')
                assert states[-1].parking_zone_entry_active
                assert commands[-1].action==ParkingCommand.PREPARE and refs[-1].state==3 and refs[-1].v_ref==0
                request=commands[-1].request_id
                parking.request_id=request;parking.mission_mode=1;parking.search_active=True
                zone.in_zone=False;gps.zones=[zone];spin(.6)
                assert states[-1].mission==1 and not states[-1].mission_failed and refs[-1].v_ref==0
                assert not any(c.action==ParkingCommand.CANCEL for c in commands)
                print('PASS: confirmed Zone exit keeps Parking stopped',flush=True)
                parking.preparation_ready=parking.search_space_found=True
                wait_for(lambda:any(c.action==ParkingCommand.ACTIVATE for c in commands),'fresh ready sends ACTIVATE')
                assert refs[-1].v_ref==0
                parking.mission_active=True;parking.points=[RefPoint(x=-1.,y=.1)];parking.v_suggest=-.3
                wait_for(lambda:refs[-1].v_ref<0 and refs[-1].state==3,'execution ack releases one-point parking motion')
                parking.done=True;parking.mission_active=False
                wait_for(lambda:states[-1].mission==0 and states[-1].mission_completed,'done returns navigation')
                wait_for(lambda:refs[-1].v_ref>0,'navigation resumes after done')
                # A second mission ends at this CSV endpoint before readiness.
                zone.mission_id=1;zone.mission_type=2;zone.zone_id=20;zone.in_zone=True;gps.zones=[zone]
                wait_for(lambda:states[-1].mission==1 and states[-1].request_mission_id==1,'second mission takes Parking immediately')
                second=states[-1].mission_request_id
                parking.request_id=second;parking.mission_mode=2;parking.mission_active=False
                parking.done=parking.preparation_ready=False;parking.points=[];parking.v_suggest=0.
                spin(.2);gps.at_end=True
                wait_for(lambda:states[-1].mission==0 and states[-1].mission_cancel_reason==10,'endpoint cancels unfinished Parking')
                assert states[-1].mission_failed and not states[-1].mission_completed
                wait_for(lambda:any(c.request_id==second and c.action==ParkingCommand.CANCEL for c in commands),
                         'typed CANCEL reaches module at endpoint')
                wait_for(lambda:states[-1].route.phase==4,'actual stop permits next CSV request')
                assert refs[-1].v_ref==0
                print('parking_entry_ros_smoke: PASS',flush=True)
            except Exception:
                log.flush();print(log_path.read_text()[-6000:],file=sys.stderr);raise
            finally:
                proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:proc.kill();proc.wait()
    for sub in subscriptions:node.destroy_subscription(sub)
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
