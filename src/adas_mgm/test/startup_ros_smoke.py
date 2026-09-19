"""Actual MGM/CLI startup, stop and fallback wiring with synthetic inputs only."""
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
    raise SystemExit('An isolated localhost ROS_DOMAIN_ID is required')

import rclpy
from std_msgs.msg import Bool, Header
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from fma_interfaces.msg import (AvoidStatus, CanHealth, EstopRequest, GpsPath, LanePath,
                                MgmState, RefPoint, TargetRef, VehicleVector)


def main():
    rclpy.init()
    node = rclpy.create_node('preparation_synthetic_inputs')
    kinds = {'/perception/gps_path': GpsPath, '/perception/lane_path': LanePath,
             '/perception/lane_camera': Header, '/perception/avoid': AvoidStatus,
             '/perception/estop': EstopRequest, '/bridge/can_health': CanHealth,
             '/vehicle/vector': VehicleVector}
    scans = ['/lidar/a1/scan','/lidar/a2/scan','/lidar/b1/scan','/lidar/b2/scan']
    kinds.update({topic:LaserScan for topic in scans})
    pubs = {topic: node.create_publisher(kind, topic, qos_profile_sensor_data if kind is LaserScan else 1) for topic, kind in kinds.items()}
    messages = {topic: kind() for topic, kind in kinds.items()}
    for topic in scans:
        scan=messages[topic]; scan.angle_increment=.1; scan.range_min=.1; scan.range_max=12.; scan.ranges=[float('inf')]*10
    gps, lane = messages['/perception/gps_path'], messages['/perception/lane_path']
    gps.fix_quality = 5
    gps.points = [RefPoint(x=2.5, y=.2)]
    lane.confidence = .9
    messages['/perception/avoid'].scan_valid = messages['/perception/estop'].scan_valid = True
    messages['/perception/avoid'].ttc = 100.
    messages['/bridge/can_health'].link_up = True
    state, refs = [], []
    subs = [node.create_subscription(MgmState, '/adas/mgm_state', state.append, 10),
            node.create_subscription(TargetRef, '/adas/target_ref', refs.append, 10)]
    go = node.create_publisher(Bool, '/operator/go', 1)
    stop = node.create_publisher(Bool, '/operator/stop', 1)
    muted = {'/perception/lane_camera', '/perception/lane_path'}
    log = tempfile.TemporaryFile(mode='w+')
    proc = subprocess.Popen([sys.argv[1], '--ros-args', '-p', 'wait_go:=true',
                             '-p', 'escape_after_cycles:=0', '-p', 'v_base:=1.0',
                             '-p', 'a_up:=100.0', '-p', 'a_down:=100.0',
                             '-p', 'required_lidar_topics:=['+','.join(scans)+']'],
                            stdout=log, stderr=subprocess.STDOUT)
    def pump(duration=.1):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            stamp = node.get_clock().now().to_msg()
            for topic, msg in messages.items():
                if topic in muted: continue
                if isinstance(msg, Header): msg.stamp = stamp
                else: msg.header.stamp = stamp
                if hasattr(msg, 'reference_stamp'): msg.reference_stamp = stamp
                pubs[topic].publish(msg)
            rclpy.spin_once(node, timeout_sec=.02)
            if proc.poll() is not None:
                log.seek(0)
                raise AssertionError('MGM exited: '+log.read())
    def expect(predicate, label):
        end = time.monotonic()+3.
        while time.monotonic() < end:
            pump()
            if state and refs and predicate(state[-1], refs[-1]):
                print('PASS:', label)
                return
        raise AssertionError(label+f' state={state[-1:]} ref={refs[-1:]}')
    def cli_go():
        cli = subprocess.Popen([sys.executable, str(Path(__file__).parents[1] / 'tools/go'), '--timeout', '2'],
                               stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic()+5.
        while cli.poll() is None and time.monotonic() < deadline:
            pump()
        if cli.poll() is None:
            cli.terminate(); cli.wait(timeout=2.)
            raise AssertionError('go CLI hung')
        assert cli.returncode == 0, 'go CLI did not receive MGM acknowledgment'
    try:
        pump(1.)
        go.publish(Bool(data=True)); pump(.4)
        assert not state[-1].go_authorized and not state[-1].start_ready and refs[-1].v_ref == 0
        print('PASS: FLOAT without camera rejects bare go')
        gps.fix_quality = 4
        expect(lambda s,r: s.start_ready and s.gps_fixed_ready and not s.camera_available, 'GPS-only preparation')
        muted.add(scans[-1])
        expect(lambda s,r: not s.lidar_ready and not s.start_ready, 'one missing LiDAR prevents start')
        go.publish(Bool(data=True)); pump(.4)
        assert not state[-1].go_authorized and refs[-1].v_ref==0
        muted.remove(scans[-1])
        expect(lambda s,r:s.lidar_ready and s.start_ready,'all four fresh scans restore readiness')
        cli_go()
        expect(lambda s,r: s.go_authorized and s.top==1 and s.navigation==1 and r.v_ref>0, 'GPS-only go drives')
        muted.add(scans[1])
        expect(lambda s,r: not s.lidar_ready and (s.active_safe_stop_reasons & 2048) and r.v_ref==0,
               'rear scan timeout stops running vehicle despite fresh front perception')
        muted.remove(scans[1])
        messages[scans[1]].ranges=[]
        pump(.5)
        assert not state[-1].lidar_ready and refs[-1].v_ref==0
        messages[scans[1]].ranges=[float('inf')]*10
        expect(lambda s,r:s.lidar_ready and r.v_ref>0,'valid clear scan restores LiDAR health')
        stop.publish(Bool(data=True))
        expect(lambda s,r: not s.go_authorized and r.v_ref==0 and s.gps_fixed_ready, 'stop keeps fresh GPS FIXED')
        gps.fix_quality = 5
        muted.remove('/perception/lane_camera')
        expect(lambda s,r: s.start_ready and s.camera_available and not s.gps_fixed_ready, 'camera frame alone enables start')
        cli_go()
        expect(lambda s,r: s.go_authorized and s.navigation==1 and r.v_ref>0,
               'camera-on without lane detection starts on the existing GPS reference')
        gps.fix_quality = 0; gps.points = []
        lane.points = [RefPoint(x=2.5, y=.1)]
        muted.remove('/perception/lane_path')
        expect(lambda s,r: s.navigation==0 and r.v_ref>0, 'GPS loss selects camera navigation')
        gps.fix_quality = 4; gps.points = [RefPoint(x=2.5,y=.2)]
        muted.update(('/perception/lane_camera', '/perception/lane_path'))
        expect(lambda s,r: not s.camera_available and s.navigation==1 and r.v_ref>0, 'camera unplug falls back to GPS')
        stop.publish(Bool(data=True))
        expect(lambda s,r: not s.go_authorized and r.v_ref==0, 'stop revokes authorization')
        cli_go()
        expect(lambda s,r: s.go_authorized and r.v_ref>0, 'go clears operator stop without new GPS session')
    finally:
        proc.terminate()
        proc.wait(timeout=5.)
        node.destroy_node()
        rclpy.try_shutdown()
        log.close()


if __name__ == '__main__':
    main()
