"""Real MGM + GPS wrapper and optional real YOLO; fake sensors, no CAN/hardware."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import yaml

if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
    raise SystemExit('Require isolated localhost ROS domain')

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Header
from sensor_msgs.msg import Image, LaserScan
from fma_interfaces.msg import CanHealth, ExitDetection, LanePath, MgmState, RefPoint, TargetRef, VehicleVector
from route_sequence_ros_smoke import FakeLink
import stack_gps.node as gps_module


def main():
    root = Path(__file__).resolve().parents[3]
    mgm = root / 'install_v2/adas_mgm/lib/adas_mgm/mgm_node'
    detector = root / 'install_v2/stack_exit_decision/lib/stack_exit_decision/exit_detector'
    waypoints = root / 'src/stack_gps/waypoints'
    gps_module.GgaLink = FakeLink
    gps_module.ImuLink = lambda *a, **k: (_ for _ in ()).throw(AssertionError('No real IMU'))
    with tempfile.TemporaryDirectory(prefix='last_mission_ros_') as tmp:
        tmp = Path(tmp)
        data = yaml.safe_load((waypoints / 'halla_route_sequence.yaml').read_text())
        routes = [r for r in data['routes'] if r['id'] in ('05', '06', '07')]
        for route in routes:
            route['file'] = str(waypoints / route['file'])
            route['zones_file'] = str(waypoints / route['zones_file'])
            if route['id'] == '05':
                zone = {'track': Path(route['file']).name, 'zones': [
                    {'zone_id': 20, 'zone_type': 'LAST_MISSION_ZONE', 'index_range': [10, 20]}]}
                zone_file = tmp / 'synthetic_zone.yaml'
                zone_file.write_text(yaml.safe_dump(zone))
                route['zones_file'] = str(zone_file)
        manifest = tmp / 'plan.yaml'
        manifest.write_text(yaml.safe_dump({'routes': routes, 'exit_branches':
                                          {'source': '05', 'left': '06', 'right': '07'}}))
        scans = ['/lidar/a1/scan', '/lidar/a2/scan', '/lidar/b1/scan', '/lidar/b2/scan']
        params = dict(revised_v2_enabled=True, wait_go=True, lidar_estop_enabled=False,
                      route_sequence_enabled=True, required_lidar_topics=scans, escape_after_cycles=0,
                      snapshot_dump_path=str(tmp / 'last_mission.bin'),
                      zone_enter_confirm_samples=2, zone_exit_confirm_samples=2,
                      avoid_zone_only=True, v_base=1.)
        for direction in ('front', 'left', 'right'):
            params['estop_mount.'+direction] = [.76,0.,0.,-180.,180.,0.,.1,12.]
        config = tmp / 'mgm.yaml'
        config.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
        for cls, expected in ((0, '07'), (1, '06'), (-1, '06')):
            rclpy.init(args=['--ros-args', '-p', f'route_sequence_file:={manifest}',
                            '-p', "imu_port:='off'", '-p', 'turn_zone_policy:=true',
                            '-p', 'publish_period:=0.02', '-p', 'n_points:=1'])
            gps = gps_module.StackGpsNode()
            node = rclpy.create_node('last_mission_synthetic')
            executor = SingleThreadedExecutor(); executor.add_node(gps); executor.add_node(node)
            states, refs, observations = [], [], []
            node.create_subscription(MgmState, '/adas/mgm_state', states.append, 10)
            node.create_subscription(TargetRef, '/adas/target_ref', refs.append, 10)
            node.create_subscription(ExitDetection, '/perception/exit_detection', observations.append, 10)
            messages = {'/perception/lane_path': LanePath(confidence=.9, points=[RefPoint(x=2.5)]),
                        '/perception/lane_camera': Header(), '/vehicle/vector': VehicleVector(),
                        '/bridge/can_health': CanHealth(link_up=True)}
            for topic in scans:
                messages[topic] = LaserScan(angle_min=0., angle_increment=.1, range_min=.1,
                                            range_max=12., ranges=[float('inf')]*10)
            pubs = {topic: node.create_publisher(type(msg), topic,
                    qos_profile_sensor_data if isinstance(msg, LaserScan) else 1)
                    for topic, msg in messages.items()}
            go = node.create_publisher(Bool, '/operator/go', 1)
            exit_pub = node.create_publisher(ExitDetection, '/perception/exit_detection', 10)
            image_pub = node.create_publisher(Image, '/test/exit_image', qos_profile_sensor_data)
            log = tempfile.TemporaryFile(mode='w+')
            proc = subprocess.Popen([str(mgm), '--ros-args', '--params-file', str(config)], stdout=log, stderr=log)
            detection_proc = None
            if cls == -1:
                detection_proc = subprocess.Popen([str(detector), '--ros-args', '-p',
                                                   'image_topic:=/test/exit_image'], stdout=log, stderr=log)
            position, previous_route = 0, 0
            first_judging = None

            def pump(duration=.1):
                nonlocal position, previous_route, first_judging
                end = time.monotonic() + duration
                while time.monotonic() < end:
                    if gps._route_plan.index != previous_route:
                        previous_route = gps._route_plan.index
                        position = 5
                        gps.engine.reset_station()
                    lat, lon = gps._route_plan.active_points[position]
                    gps.link.sample = (lat, lon, 0., 4, 0., time.monotonic())
                    gps.link.heading = gps.engine.yaw[position]
                    stamp = node.get_clock().now().to_msg()
                    for topic, msg in messages.items():
                        if isinstance(msg, Header): msg.stamp = stamp
                        else: msg.header.stamp = stamp
                        if hasattr(msg, 'reference_stamp'): msg.reference_stamp = stamp
                        pubs[topic].publish(msg)
                    if states and states[-1].last_mission_phase == MgmState.LAST_JUDGING:
                        if first_judging is None: first_judging = time.monotonic()
                        if cls >= 0:
                            exit_pub.publish(ExitDetection(reference_stamp=stamp,
                                request_id=states[-1].last_mission_request_id, class_id=cls, confidence=.9))
                        else:
                            image = Image(height=480, width=640, encoding='bgr8', step=640*3,
                                          data=bytes(480*640*3))
                            image.header.stamp = stamp
                            image_pub.publish(image)
                    executor.spin_once(timeout_sec=.01)
                    if proc.poll() is not None:
                        log.seek(0); raise AssertionError(log.read())

            def expect(predicate, label, timeout=5.):
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    pump()
                    if states and refs and predicate(states[-1], refs[-1]):
                        print('PASS:', label, flush=True)
                        return
                log.seek(0)
                raise AssertionError(label + '\n' + log.read()[-5000:])

            try:
                expect(lambda s,r: s.start_ready, 'synthetic sensors ready')
                go.publish(Bool(data=True))
                expect(lambda s,r: s.go_authorized and r.v_ref > 0, 'normal navigation before exit zone')
                position = 15; gps.engine.reset_station()
                expect(lambda s,r: s.last_mission_phase == s.LAST_JUDGING and r.v_ref == 0,
                       'production GPS zone enters MGM stationary judgment')
                pump(8.)
                assert states[-1].last_mission_phase == MgmState.LAST_JUDGING
                assert refs[-1].v_ref == 0, 'must not depart before ten seconds'
                expect(lambda s,r: s.last_mission_phase == s.LAST_DONE and r.v_ref > 0,
                       f'class {cls}: acknowledged route {expected} resumes navigation', timeout=6.)
                assert time.monotonic() - first_judging >= 9.8
                assert gps._route_plan.files[gps._route_plan.index].id == expected
                assert states[-1].last_mission_route_id == int(expected)
                assert states[-1].last_mission_fallback == (cls == -1)
                if cls == -1:
                    assert observations and all(o.class_id == -1 for o in observations), 'real YOLO blank-frame path required'
                position = len(gps._route_plan.active_points) - 2; gps.engine.reset_station()
                expect(lambda s,r: s.top == 2 and r.v_ref == 0, 'selected exit endpoint enters FINISH')
            finally:
                for child in (detection_proc, proc):
                    if child is not None:
                        child.terminate(); child.wait(timeout=8)
                executor.remove_node(gps); executor.remove_node(node)
                gps.destroy_node(); node.destroy_node(); executor.shutdown(); rclpy.shutdown(); log.close()
            replay = root / 'install_v2/adas_mgm/lib/adas_mgm/core_replay'
            outputs = [tmp / 'replay_a.csv', tmp / 'replay_b.csv']
            for output in outputs:
                subprocess.run([str(replay), str(tmp / 'last_mission.bin'), str(output)], check=True,
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            assert outputs[0].read_bytes() == outputs[1].read_bytes(), 'last mission replay must be deterministic'
            import csv
            with outputs[0].open() as stream:
                rows = list(csv.DictReader(stream))
            assert any(row['last_mission_phase'] == '2' for row in rows)
            assert any(row['last_mission_phase'] == '5' and int(row['last_mission_route_id']) == int(expected)
                       for row in rows)
            print('PASS: v36 last mission snapshot replay is deterministic', flush=True)


if __name__ == '__main__':
    main()
