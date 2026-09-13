"""Synthetic ROS wiring test. Only launch mgm_node; never start a CAN bridge.

Usage (source the test overlay first):
  ROS_DOMAIN_ID=171 ROS_LOCALHOST_ONLY=1 python3 manager_ros_smoke.py /path/to/mgm_node
All sensors and parking feedback below are synthetic. Domain isolation is required.
"""
import csv
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
    raise SystemExit('Run with a separate ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')

import rclpy
from fma_interfaces.msg import (
    AvoidStatus, CanHealth, EstopRequest, GpsPath, LanePath, MgmState,
    ParkingCommand, ParkingStatus, RefPoint, TargetRef, TrafficStop, VehicleVector,
)
from std_msgs.msg import Bool
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter

# Exercise the actual GPS Zone publication method with a synthetic PathEngine.
# Constructing StackGpsNode would open vehicle serial devices, so use its pure
# serialization method with only the zone_map dependency supplied.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'stack_gps'))
from stack_gps.path_engine import PathEngine
from stack_gps.zones import ZoneMap
from stack_gps.node import StackGpsNode


def main():
    rclpy.init()
    node = rclpy.create_node('manager_synthetic_inputs')
    publishers = {}
    messages = {}
    for topic, kind in [
        ('/perception/lane_path', LanePath), ('/perception/gps_path', GpsPath),
        ('/perception/avoid', AvoidStatus), ('/perception/estop', EstopRequest),
        ('/perception/parking', ParkingStatus), ('/perception/traffic_stop', TrafficStop),
        ('/vehicle/vector', VehicleVector), ('/bridge/can_health', CanHealth),
    ]:
        publishers[topic] = node.create_publisher(kind, topic, 1)
        messages[topic] = kind()
    lane = messages['/perception/lane_path']
    gps = messages['/perception/gps_path']
    avoid = messages['/perception/avoid']
    estop = messages['/perception/estop']
    parking = messages['/perception/parking']
    traffic = messages['/perception/traffic_stop']
    vehicle = messages['/vehicle/vector']
    can = messages['/bridge/can_health']
    lane.confidence = .9
    lane.points = [RefPoint(x=1., y=.1)]
    gps.fix_quality = 4
    gps.points = [RefPoint(x=1., y=.2)]
    avoid.scan_valid = estop.scan_valid = True
    avoid.ttc = 100.
    avoid.v_suggest = .6
    avoid.points = [RefPoint(x=1., y=.3)]
    can.link_up = True
    status = []
    refs = []
    commands = []
    subscriptions = [
        node.create_subscription(MgmState, '/adas/mgm_state', status.append, 10),
        node.create_subscription(TargetRef, '/adas/target_ref', refs.append, 10),
        node.create_subscription(ParkingCommand, '/parking/mission_command', commands.append, 10),
    ]
    go = node.create_publisher(Bool, '/operator/go', 1)
    session = node.create_publisher(Bool, '/operator/start_session', 1)
    cancel_mission = node.create_publisher(Bool, '/operator/cancel_mission', 1)
    artifacts = tempfile.TemporaryDirectory(prefix='mgm-preparation-ros-')
    event_csv = str(Path(artifacts.name) / 'mission_events.csv')
    zone_csv = str(Path(artifacts.name) / 'zone_observations.csv')
    log = tempfile.TemporaryFile(mode='w+')
    proc = subprocess.Popen([
        sys.argv[1], '--ros-args', '-p', 'wait_go:=true', '-p', 'v_base:=1.0',
        '-p', 'parking_search_zone_only:=false',  # historical lifetime policy regression
        '-p', 'parking_zone_entry_active:=false',
        '-p', 'a_up:=100.0', '-p', 'a_down:=100.0',
        '-p', 'zone_enter_confirm_samples:=3', '-p', 'zone_exit_confirm_samples:=3', '-p', 'parking_search_timeout:=30.0', '-p', 'max_parking_search_distance:=30.0',
        '-p', 'mission_events_csv_path:=' + event_csv,
        '-p', 'zone_observations_csv_path:=' + zone_csv,
    ], stdout=log, stderr=subprocess.STDOUT)
    paused = set()
    frozen_generation = set()
    track = [(37.5, 127.0 + i * 0.00001) for i in range(100)]
    engine = PathEngine(track, gps_only_ranges=[(10, 40)], parking_ranges=[(30, 35)])
    gps_adapter = SimpleNamespace(zone_map=ZoneMap.from_engine(engine))
    position_index = 0

    def spin(seconds=.12):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            gps.zones = []
            gps.zone_valid = False
            if gps.fix_quality:
                snap = engine.snapshot(*track[position_index], heading=0.)
                StackGpsNode._fill_zone_context(gps_adapter, gps, snap['idx'])
                gps.position_valid = True
                gps.position_x, gps.position_y = engine.to_enu(*track[position_index])
                gps.track_index = snap['idx']
            for topic, pub in publishers.items():
                if topic not in paused:
                    message = messages[topic]
                    message.header.stamp = node.get_clock().now().to_msg()
                    if hasattr(message, 'reference_stamp') and topic not in frozen_generation:
                        message.reference_stamp = message.header.stamp
                    if isinstance(message, ParkingStatus) and message.preparation_ready:
                        message.preparation_stamp = message.header.stamp
                    pub.publish(message)
            rclpy.spin_once(node, timeout_sec=.01)
            if proc.poll() is not None:
                log.seek(0)
                raise AssertionError('MGM exited: ' + log.read())

    def expect(predicate, description, timeout=3.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            spin(.05)
            if status and refs and predicate(status[-1], refs[-1]):
                print('PASS:', description)
                return
        raise AssertionError(description + f' (status={status[-1:]}, ref={refs[-1:]})')

    try:
        spin(1.)
        assert node.count_publishers('/adas/target_ref') == 1, 'exactly one output publisher'
        assert node.count_subscribers('/adas/target_ref') == 1, 'only this test may consume commands'
        expect(lambda s, r: s.top == 0 and r.v_ref == 0, 'operator enable gates startup')
        go.publish(Bool(data=True))
        expect(lambda s, r: s.navigation == 0 and r.v_ref > 0, 'LINE drives after go')
        paused.add('/perception/lane_path')
        expect(lambda s, r: s.navigation == 1 and r.v_ref > 0, 'real 0.5s lane timeout selects GPS')
        paused.clear()
        expect(lambda s, r: s.navigation == 0, 'valid LINE recovers after confidence hold')
        position_index = 15
        frozen_generation.add('/perception/gps_path')
        # Stamp stays fixed while header publications keep arriving at ~100Hz.
        gps.reference_stamp = node.get_clock().now().to_msg()
        spin(.12)
        context = next(z for z in status[-1].zones if z.zone_type == 1)
        assert context.raw_in_zone and not context.stable_in_zone and context.enter_count == 1
        gps.reference_stamp = node.get_clock().now().to_msg()
        spin(.12)
        context = next(z for z in status[-1].zones if z.zone_type == 1)
        assert not context.stable_in_zone and context.enter_count == 2
        frozen_generation.remove('/perception/gps_path')
        expect(lambda s, r: s.navigation == 2, 'GPS-only enters after three independent generations; duplicate fixes do not count')
        gps.fix_quality = 0
        expect(lambda s, r: s.navigation == 2 and s.safety == 3 and r.v_ref == 0,
               'invalid GPS cannot fabricate zone exit')
        gps.fix_quality = 4
        position_index = 15
        expect(lambda s, r: s.navigation == 2 and s.safety == 0, 'GPS recovery retains zone')
        position_index = 0
        expect(lambda s, r: s.navigation == 0 and r.v_ref > 0, 'valid zone exit reselects LINE')
        frozen_generation.update(('/perception/lane_path', '/perception/gps_path'))
        avoid.points = []
        expect(lambda s, r: not s.selected_reference_valid and r.v_ref == 0 and
               bool(s.active_safe_stop_reasons & 4),
               'live publications with frozen generation become stale and stop')
        frozen_generation.clear()
        avoid.points = [RefPoint(x=1.5, y=.3)]
        expect(lambda s, r: s.selected_reference_valid and r.v_ref > 0,
               'new actual generation recovers existing Navigation')
        estop.estop = True
        estop.scan_valid = False
        avoid.scan_valid = False
        expect(lambda s, r: s.safety == 0 and r.v_ref > 0, 'LiDAR timeout alone permits navigation')
        estop.scan_valid = avoid.scan_valid = True
        estop.estop = False
        position_index = 15
        traffic.red_active = traffic.stopline_detected = True
        avoid.obstacle_detected = avoid.avoidable = True
        expect(lambda s, r: s.navigation == 2 and s.reference_source == 2 and s.signal == 2 and s.speed_owner == 2,
               'GPS-only context + avoidance reference + traffic speed coexist over ROS')
        traffic.stopline_detected = False
        vehicle.v = 1.
        expect(lambda s, r: s.reference_source == 2 and r.v_ref == 0,
               'traffic stops while avoidance owns reference')
        vehicle.v = 0.
        traffic.red_active = False
        expect(lambda s, r: s.signal == 3 and r.v_ref == 0, 'lost red does not release stop')
        traffic.green_active = True
        estop.estop = True
        expect(lambda s, r: s.signal == 0 and s.safety == 1 and r.v_ref == 0,
               'green releases signal only, LiDAR brake remains')
        position_index = 32
        expect(lambda s, r: s.mission == 2 and s.reference_source == 2 and s.avoidance == 1,
               'Mission Zone enters PREPARE while ordinary avoidance and LiDAR brake remain')
        spin(.2)
        request_id = status[-1].mission_request_id
        assert status[-1].zone.zone_type == 2 and status[-1].in_gps_only_zone
        assert commands and all(c.action == ParkingCommand.PREPARE for c in commands)
        assert not (status[-1].active_safe_stop_reasons & 4), 'missing preparation ref is normal'
        position_index = 37  # outside narrow Mission Zone, still in GPS-only Zone
        parking.request_id = request_id - 1
        parking.mission_mode = 1
        parking.search_active = parking.search_space_found = parking.preparation_ready = True
        spin(.2)
        assert status[-1].mission == 2 and status[-1].mission_request_active
        print('PASS: Zone exit retains request and stale space/ready cannot activate it')
        parking.request_id = request_id
        expect(lambda s, r: s.mission == 1 and s.reference_source == 3 and s.avoidance == 0,
               'late current-request readiness outside Mission Zone hands control to parking')
        assert status[-1].active_safe_stop_reasons & 4 and refs[-1].v_ref == 0
        spin(.1)
        assert any(c.action == ParkingCommand.ACTIVATE and c.request_id == request_id for c in commands)
        parking.mission_active = True
        parking.points = [RefPoint(x=-1., y=.4)]
        parking.v_suggest = -.3
        expect(lambda s, r: s.mission == 1 and s.safety == 0 and r.v_ref < 0,
               'parking acknowledgement permits mission speed and masks LiDAR brake')
        frozen_generation.add('/perception/parking')
        expect(lambda s, r: s.mission == 1 and s.reference_source == 3 and
               not s.selected_reference_fresh and r.v_ref == 0,
               'Mission heartbeat cannot refresh frozen parking reference')
        frozen_generation.clear()
        expect(lambda s, r: s.mission_ref_valid and r.v_ref < 0,
               'new Mission generation restores its own reverse command')
        can.link_up = False
        expect(lambda s, r: s.safety == 3 and r.v_ref == 0, 'CAN failure stops parking too')
        can.link_up = True
        go.publish(Bool(data=True))
        estop.estop = False
        avoid.obstacle_detected = False
        avoid.avoidable = False
        parking.done = True
        parking.mission_active = False
        expect(lambda s, r: s.mission == 0 and s.navigation == 2, 'parking done returns to overlapping GPS-only Zone')
        command_count = len(commands)
        position_index = 32
        spin(.2)
        assert status[-1].mission == 0 and len(commands) == command_count, 'completed mission cannot restart inside Zone'
        position_index = 15
        spin(.15)
        position_index = 32
        spin(.15)
        assert status[-1].mission == 0 and len(commands) == command_count, 'completed mission cannot restart on Zone reentry'
        gps.at_end = True
        expect(lambda s, r: s.top == 2 and r.v_ref == 0, 'finish latch over ROS')
        gps.at_end = False
        estop.estop = True
        position_index = 32
        go.publish(Bool(data=True))
        spin(.3)
        assert status[-1].top == 2 and refs[-1].v_ref == 0, 'go/green/estop/mission cannot clear finish'
        estop.estop = False
        position_index = 0
        session.publish(Bool(data=True))
        expect(lambda s, r: s.top == 1 and r.v_ref > 0, 'explicit new session permits driving again')
        position_index = 32
        expect(lambda s, r: s.mission == 2 and s.mission_request_active,
               'new session reentry prepares a fresh request despite old module result')
        fresh_id = status[-1].mission_request_id
        assert fresh_id > request_id
        cancel_mission.publish(Bool(data=True))
        expect(lambda s, r: s.mission == 0 and not s.mission_request_active and
               s.mission_cancel_reason == 3 and not s.mission_completed,
               'explicit Mission cancel clears pending request without marking completion')
        spin(.1)
        assert any(c.action == ParkingCommand.CANCEL and c.request_id == fresh_id for c in commands)
        with open(event_csv) as stream:
            events = list(csv.DictReader(stream))
        first = {e['event']: e for e in events if int(e['request_id']) == request_id}
        assert set(first) == {'zone_entry', 'search_start', 'space_found', 'ready', 'handoff', 'done'}
        assert first['zone_entry']['track_index'] == '32'
        assert first['ready']['track_index'] == '37'
        assert first['zone_entry']['position_valid'] == '1'
        assert int(first['ready']['time_ns']) >= int(first['zone_entry']['time_ns'])
        assert all(e['speed_valid'] == '1' for e in first.values())
        print('PASS: mission calibration CSV records request/position/index/actual-speed milestones')
        assert not status[-1].recovery_configured and status[-1].rear_corridor_state == 0
        assert not status[-1].recovery_eligible
        print('PASS: disabled Recovery and UNKNOWN rear corridor are explicit on ROS diagnostics')
        client = node.create_client(SetParameters, '/mgm_node/set_parameters')
        assert client.wait_for_service(timeout_sec=3.)
        future = client.call_async(SetParameters.Request(parameters=[
            Parameter('parking_search_timeout', value=-1.0).to_parameter_msg()]))
        deadline = time.monotonic()+3
        while not future.done() and time.monotonic()<deadline:
            spin(.03)
        assert future.done() and all(r.successful for r in future.result().results)
        expect(lambda s, r: s.parking_calibration_state == 0, 'runtime parking calibration changes reach the core')
        position_index = 0; spin(.15); position_index = 32
        expect(lambda s, r: s.mission_cancel_reason == 6 and not s.mission_request_active and
               s.reference_source != 3, 'uncalibrated ROS request cancels without parking authority')
        with open(zone_csv) as stream:
            observations = list(csv.DictReader(stream))
        assert any(e['raw_in']=='1' and e['stable_in']=='0' and e['enter_count']=='1' for e in observations)
        assert all('boundary_endpoint_distance_m' in e for e in observations)
        report_file = Path(artifacts.name) / 'calibration.json'
        subprocess.run([sys.executable, str(Path(__file__).parents[1] / 'tools/analyze_mission_calibration.py'),
                        event_csv, '--json', str(report_file)], check=True)
        import json
        report = json.loads(report_file.read_text())
        assert not report['operating_parameters_generated']
        assert report['groups']['T_PARKING']['outcomes']['success'] == 1
        print('PASS: real mock-session event CSV analysis and raw/stable zone CSV checks')
        print('ROS synthetic wiring checks passed; no CAN bridge or hardware launched.')
    finally:
        proc.terminate()
        proc.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        artifacts.cleanup()


if __name__ == '__main__':
    main()
