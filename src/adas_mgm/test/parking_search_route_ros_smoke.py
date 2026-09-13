"""Real Halla CSV/Zone + GPS wrapper + MGM: no-space search automatically exits to 04.

GNSS positions and vehicle feedback are synthetic. No sensor, CAN or parking node
is started. The test advances its position only while MGM requests forward motion.
"""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '179':
    raise SystemExit('Run in isolated localhost ROS_DOMAIN_ID=179')

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool
from fma_interfaces.msg import (EstopRequest, GpsPath, LanePath, MgmState,
                               ParkingCommand, ParkingStatus, RefPoint, TargetRef, VehicleVector)
sys.path.insert(0, str(Path(__file__).parents[2] / 'stack_gps'))
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
    manifest = Path(__file__).parents[2] / 'stack_gps/waypoints/halla_route_sequence.yaml'
    gps_module.GgaLink = FakeLink
    def forbidden_imu(*args, **kwargs):
        raise AssertionError('Hardware IMU must never start')
    gps_module.ImuLink = forbidden_imu
    rclpy.init(args=['--ros-args', '-p', f'route_sequence_file:={manifest}',
                    '-p', "imu_port:='off'", '-p', "route_start_id:='01'",
                    '-p', "route_end_id:='07'", '-p', 'publish_period:=0.04'])
    gps = gps_module.StackGpsNode()
    plan = gps._route_plan
    assert [route.id for route in plan.files] == ['01', '03', '04', '05', '07']
    assert plan.files[0].points[-1] == plan.files[1].points[0]
    assert plan.files[1].points[-1] == plan.files[2].points[0]
    node = rclpy.create_node('parking_search_route_inputs')
    executor = SingleThreadedExecutor()
    executor.add_node(gps); executor.add_node(node)
    msgs = {'/perception/lane_path': LanePath(points=[RefPoint(x=2.5)], confidence=.9),
            '/perception/estop': EstopRequest(scan_valid=True),
            '/perception/parking': ParkingStatus(), '/vehicle/vector': VehicleVector()}
    pubs = {topic: node.create_publisher(type(msg), topic, 10) for topic, msg in msgs.items()}
    states, refs, fixes, commands = [], [], [], []
    subs = [node.create_subscription(kind, topic, callback, 100) for kind, topic, callback in (
        (MgmState, '/adas/mgm_state', states.append), (TargetRef, '/adas/target_ref', refs.append),
        (GpsPath, '/perception/gps_path', fixes.append),
        (ParkingCommand, '/parking/mission_command', commands.append))]
    go = node.create_publisher(Bool, '/operator/go', 10)
    # Start on the end portion of 01; traverse 03 using its unmodified 0.25m samples.
    cursor, stage = len(plan.files[0].points)-12, 0
    position = plan.files[0].points[cursor]
    with tempfile.TemporaryFile(mode='w+') as log:
        proc = subprocess.Popen([sys.argv[1], '--ros-args', '-p', 'wait_go:=true',
            '-p', 'route_sequence_enabled:=true', '-p', 'avoidance_enabled:=false',
            '-p', 'zone_enter_confirm_samples:=5', '-p', 'zone_exit_confirm_samples:=5',
            '-p', 'parking_zone_entry_active:=true'], stdout=log, stderr=subprocess.STDOUT)
        def spin():
            nonlocal cursor, stage, position
            assert proc.poll() is None, 'MGM exited'
            if plan.index != stage:
                cursor, stage = -1, plan.index
                # Preserve the last position; the existing endpoint detector can
                # fire on the penultimate sample. Move to the new start only on go.
            moving = bool(refs and refs[-1].v_ref > 0 and states and
                          states[-1].route.index == stage and states[-1].route.phase == 1)
            if moving and stage < 2:
                cursor = min(cursor+1, len(plan.files[stage].points)-1)
                position = plan.files[stage].points[cursor]
            gps.link.sample = (*position, 0., 4, 0., time.monotonic())
            gps.link.heading = gps.engine.yaw[max(0, cursor)]
            parking = msgs['/perception/parking']
            if commands and commands[-1].action == ParkingCommand.PREPARE:
                parking.request_id = commands[-1].request_id
                parking.mission_mode = commands[-1].mission_mode
                parking.search_active = True
            # Deliberately never report space_found, ready, done or any Parking point.
            msgs['/vehicle/vector'].v = 1. if moving else 0.
            stamp = node.get_clock().now().to_msg()
            for topic, msg in msgs.items():
                msg.header.stamp = stamp
                if hasattr(msg, 'reference_stamp'): msg.reference_stamp = stamp
                pubs[topic].publish(msg)
            deadline = time.monotonic()+.05
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.005)
        def wait_for(predicate, label, timeout=5.):
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                spin()
                if predicate():
                    print('PASS:', label, flush=True)
                    return
            raise AssertionError(f'{label}; route={stage}, position_index={cursor}, '
                                 f'state={states[-1:]}, target={refs[-1:]}')
        try:
            wait_for(lambda: states and refs and go.get_subscription_count(), 'mock GPS/MGM connected')
            go.publish(Bool(data=True))
            wait_for(lambda: plan.index == 1 and states[-1].route.index == 1 and refs[-1].v_ref > 0,
                     '01 endpoint automatically enters uploaded 03 CSV')
            wait_for(lambda: states[-1].mission == 1 and states[-1].reference_source == 1 and refs[-1].v_ref > 0,
                     '03 uploaded T-Parking Zone starts GPS-following search', timeout=15.)
            request = states[-1].mission_request_id
            wait_for(lambda: plan.index == 2 and states[-1].route.index == 2 and
                     states[-1].route.phase == 1 and refs[-1].v_ref > 0,
                     '03 no-space endpoint automatically applies 04 and resumes driving', timeout=10.)
            assert any(f.route.route_id == '03' and f.at_end for f in fixes)
            assert fixes[-1].route.route_id == '04'
            assert Path(fixes[-1].route.waypoint_csv).name == 'waypoints_halla_reference_path_04.csv'
            assert any(s.mission_request_id == request and s.mission_cancel_reason == 10 and
                       s.mission_failed and not s.mission_completed for s in states)
            assert any(c.request_id == request and c.action == ParkingCommand.CANCEL for c in commands)
            assert not any(c.request_id == request and c.action == ParkingCommand.ACTIVATE for c in commands)
            assert all(s.reference_source == 1 for s in states if s.mission == 1)
            assert states[-1].top == 1 and states[-1].reference_source == 1
            assert len(refs[-1].ref_points) == 1
            print('PASS: failed Mission recorded, no manual CSV command or second go; real GPS wrapper applied 04', flush=True)
        except Exception:
            log.seek(0); print(log.read()[-6000:], file=sys.stderr)
            raise
        finally:
            proc.send_signal(signal.SIGINT)
            try: proc.wait(timeout=5)
            except subprocess.TimeoutExpired: proc.kill(); proc.wait()
            executor.shutdown()
            for sub in subs: node.destroy_subscription(sub)
            gps.destroy_node(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
