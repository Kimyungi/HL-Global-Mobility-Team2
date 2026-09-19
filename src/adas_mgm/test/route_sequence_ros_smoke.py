"""Production GPS wrapper + MGM, fake GNSS/IMU/perception; no hardware or CAN node.

Run under a dedicated localhost ROS_DOMAIN_ID with the v2 overlay sourced.
"""
import argparse
import csv
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
    raise SystemExit('Require isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool
from fma_interfaces.msg import (AvoidStatus, CanHealth, EstopRequest, LanePath, MgmState,
                               ParkingCommand, ParkingStatus, RefPoint, TargetRef, TrafficStop, VehicleVector)
sys.path.insert(0, str(Path(__file__).parents[2]/'stack_gps'))
import stack_gps.node as gps_module


class FakeLink:
    def __init__(self, **kwargs): self.sample = None; self.heading = 0.
    def start(self): pass
    def stop(self): pass
    def latest_fix(self): return self.sample
    def latest_cog(self): return (1., self.heading, 0.)
    def rtcm_rate_and_reset(self): return 100
    def nmea_rate_and_reset(self): return 100
    def latest_sat_info(self): return (20, .5)
    def usb_reset_count(self): return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mgm')
    parser.add_argument('--start', choices=['01','02'], required=True)
    parser.add_argument('--end', choices=['06','07'], required=True)
    parser.add_argument('--fail-search', action='store_true', help='Mock both searches failing at their Zone exit')
    args = parser.parse_args()
    expected_ids = [args.start, '03', '04', '05', args.end]
    manifest = Path(__file__).parents[2]/'stack_gps/waypoints'/'halla_route_sequence.yaml'
    # Replace hardware factories before constructing the production wrapper.
    gps_module.GgaLink = FakeLink
    def forbidden_imu(*a, **k): raise AssertionError('Hardware IMU must never start')
    gps_module.ImuLink = forbidden_imu
    rclpy.init(args=['--ros-args', '-p', f'route_sequence_file:={manifest}', '-p', "imu_port:='off'",
                    '-p', f"route_start_id:='{args.start}'", '-p', f"route_end_id:='{args.end}'", '-p', 'n_points:=1', '-p', 'publish_period:=0.02'])
    gps = gps_module.StackGpsNode()
    assert [r.id for r in gps._route_plan.files] == expected_ids
    assert gps._route_plan.connections == [None]*5
    node = rclpy.create_node('route_sequence_test_inputs')
    executor = SingleThreadedExecutor()
    executor.add_node(gps); executor.add_node(node)
    topics = [('/perception/lane_path',LanePath),('/perception/avoid',AvoidStatus),
              ('/perception/estop',EstopRequest),('/perception/traffic_stop',TrafficStop),
              ('/perception/parking',ParkingStatus),('/vehicle/vector',VehicleVector),('/bridge/can_health',CanHealth)]
    pubs = {t:node.create_publisher(kind,t,1) for t,kind in topics}
    msgs = {t:kind() for t,kind in topics}
    msgs['/perception/lane_path'].points = [RefPoint(x=1.)]
    msgs['/perception/lane_path'].confidence = 0.0
    msgs['/perception/avoid'].scan_valid = msgs['/perception/estop'].scan_valid = True
    msgs['/perception/avoid'].ttc = 100.
    msgs['/perception/avoid'].points = [RefPoint(x=1.)]
    msgs['/bridge/can_health'].link_up = True
    status, refs, commands = [], [], []
    subs = [node.create_subscription(MgmState,'/adas/mgm_state',status.append,100),
            node.create_subscription(TargetRef,'/adas/target_ref',refs.append,100),
            node.create_subscription(ParkingCommand,'/parking/mission_command',commands.append,10)]
    go = node.create_publisher(Bool,'/operator/go',1)
    log = tempfile.TemporaryFile(mode='w+')
    artifacts = tempfile.TemporaryDirectory(prefix='mgm-route-replay-')
    dump_path = Path(artifacts.name) / 'snapshots.bin'
    replay_path = Path(artifacts.name) / 'replay.csv'
    proc = subprocess.Popen([args.mgm,'--ros-args','-p','wait_go:=true','-p','route_sequence_enabled:=true',
        '-p','zone_enter_confirm_samples:=5','-p','zone_exit_confirm_samples:=5',
        '-p',f'snapshot_dump_path:={dump_path}','-p','parking_search_zone_only:=true',
        '-p','parking_zone_entry_active:=false'],stdout=log,stderr=subprocess.STDOUT)
    position = [0]
    current = [(0,False)]
    last_position = [None]
    parking_position = [None]
    def spin(seconds=.05):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            plan = gps._route_plan
            # Preserve the actual mock position at every CSV handoff. The 04->05
            # offset is traversed while Parking is ACTIVE, before its done reply.
            stage = (plan.index,plan.connecting)
            if stage != current[0]:
                # A failed 04 search continues to the 04 endpoint. The next CSV
                # must assemble its first reference from that same current fix,
                # even though its post-parking start is elsewhere.
                current[0]=stage; position[0]=0; parking_position[0]=last_position[0]
            lat,lon = parking_position[0] or plan.active_points[position[0]]
            last_position[0] = (lat,lon)
            gps.link.sample = (lat,lon,0.,4,0.,time.monotonic())
            gps.link.heading = gps.engine.yaw[position[0]]
            for topic,pub in pubs.items():
                msg=msgs[topic]; msg.header.stamp=node.get_clock().now().to_msg()
                if hasattr(msg,'reference_stamp'): msg.reference_stamp=msg.header.stamp
                if isinstance(msg,ParkingStatus) and msg.preparation_ready: msg.preparation_stamp=msg.header.stamp
                pub.publish(msg)
            executor.spin_once(timeout_sec=.005)
            if proc.poll() is not None:
                log.seek(0); raise AssertionError(log.read())
    def teleport(index):
        # This test isolates route/mission handoffs using discontinuous fixes.
        # Reset only the mock GPS station at each scripted teleport; continuous
        # station tracking is covered by waypoint and parking-search tests.
        position[0] = index
        gps.engine.reset_station()

    def expect(predicate, description, timeout=4.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            spin()
            if status and refs and predicate(status[-1],refs[-1]):
                print('PASS:',description,flush=True); return
        raise AssertionError(description+f' status={status[-1:]} ref={refs[-1:]}')
    try:
        spin(1.)
        assert node.count_publishers('/adas/target_ref')==1
        go.publish(Bool(data=True))
        completed=[]; failed=[]
        for index,files in enumerate(gps._route_plan.files):
            expect(lambda s,r:s.route.index==index and not s.route.connecting and s.route.phase==1 and s.route.seen_nonterminal and r.v_ref>0,
                   f'route {files.id} entered with valid new reference')
            parking_position[0]=None  # scripted driving to the CSV start after handoff
            spin(.05)
            definitions=[z for z in gps.zone_map.definitions if int(z.zone_type)==2]
            for zone in definitions:
                teleport(zone.start_index)
                expect(lambda s,r:s.mission==2, f'Mission {zone.mission_id} PREPARE')
                request=status[-1].mission_request_id
                parking=msgs['/perception/parking']
                parking.request_id=request; parking.mission_mode=int(zone.mission_type)
                if args.fail_search:
                    parking.search_active=True
                    expect(lambda s,r:s.parking_search_acknowledged, 'Search ack with no numeric limits')
                    assert status[-1].parking_search_zone_only and status[-1].parking_calibration_state==3
                    teleport(zone.end_index+1)
                    expect(lambda s,r:s.mission==0 and s.mission_failed and s.mission_cancel_reason==9,
                           f'{files.id} Zone exit fails search')
                    expect(lambda s,r:s.route.index==index and s.route.phase==1 and r.v_ref>0,
                           'Failure resumes current CSV without advancing its index')
                    assert not status[-1].mission_completed
                    assert any(c.request_id==request and c.action==ParkingCommand.CANCEL for c in commands)
                    failed.append(zone.mission_id)
                    # Late readiness/execution from cancelled request cannot take authority.
                    parking.preparation_ready=parking.mission_active=True
                    spin(.1)
                    assert status[-1].mission==0 and status[-1].route.index==index
                    msgs['/perception/parking']=ParkingStatus()
                    continue
                parking.search_active=parking.search_space_found=parking.preparation_ready=True
                expect(lambda s,r:s.mission==1, f'Mission {zone.mission_id} ACTIVE')
                parking.mission_active=True
                parking.points=[RefPoint(x=-1.,y=.2)]
                parking.v_suggest=-.3
                expect(lambda s,r:s.mission==1 and r.v_ref<0, 'Parking execution acknowledgement')
                if files.id == '04':
                    # Test-only Parking movement, not a claim about real tracking.
                    teleport(len(files.points)-2)  # endpoint region with a nonzero GPS preview
                    expect(lambda s,r:s.route.end_reached and s.mission==1,
                           '04 endpoint remembered while Parking retains authority')
                    parking_position[0] = gps._route_plan.files[index+1].points[0]
                    spin(.1)
                    assert status[-1].route.index == index and status[-1].mission == 1
                parking.done=True
                expect(lambda s,r:s.mission==0 and s.mission_completed, 'current Mission completion')
                completed.append(zone.mission_id)
                msgs['/perception/parking']=ParkingStatus()
            if files.id != '04' or args.fail_search:
                teleport(len(files.points)-2)  # endpoint region with a nonzero GPS preview
            if index+1<len(gps._route_plan.files):
                expect(lambda s,r:s.route.index==index+1, f'{files.id} endpoint automatically hands off')
            else:
                expect(lambda s,r:s.top==2 and s.route.phase==5 and r.v_ref==0, 'last route FINISH')
        outcomes = failed if args.fail_search else completed
        assert len(outcomes)==2 and len(set(outcomes))==2
        assert {s.route.index for s in status if s.top==2}=={4}
        assert len({s.route.request_id for s in status if s.route.phase==4})==4
        assert not any(s.route.connecting for s in status)
        print(f'PASS: {expected_ids}, 5 routes, {len(completed)} successes/{len(failed)} failures, 4 handoffs, one final FINISH; no connectors',flush=True)
    finally:
        proc.send_signal(signal.SIGINT)
        try: proc.wait(timeout=5)
        except subprocess.TimeoutExpired: proc.kill(); proc.wait()
        executor.remove_node(gps); executor.remove_node(node); executor.shutdown()
        gps.destroy_node(); node.destroy_node(); rclpy.shutdown(); log.close()
    try:
        replay = Path(args.mgm).with_name('core_replay')
        subprocess.run([str(replay), str(dump_path), str(replay_path)], check=True, capture_output=True, text=True)
        with replay_path.open() as stream:
            rows = list(csv.DictReader(stream))
        order = []
        for row in rows:
            if row['route_count'] == '5' and (not order or order[-1] != int(row['route_index'])):
                order.append(int(row['route_index']))
        assert order == list(range(5)), order
        assert {r['route_index'] for r in rows if r['top']=='2'} == {'4'}
        assert not any(r['route_connecting']=='1' for r in rows)
        assert rows[-1]['route_phase'] == '5' and float(rows[-1]['v_ref']) == 0.
        assert any(r['mission_failed']=='1' and r['mission_cancel_reason']=='9' for r in rows) == args.fail_search
        print('PASS: v15 raw dump replays identical route order, failure outcome and final FINISH', flush=True)
    finally:
        artifacts.cleanup()


if __name__=='__main__': main()
