#!/usr/bin/env python3
"""Isolated ROS input/deadline test. No drivers, MGM, CAN or go publisher."""
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
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from fma_interfaces.msg import AvoidCourse, AvoidPlan, GpsPath, GpsRoute, RefPoint, VehicleVector
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def main():
    rclpy.init()
    node = rclpy.create_node('avoid_v2_synthetic_inputs')
    latest = {}
    subs = [node.create_subscription(AvoidPlan, '/avoid_v2/plan',
                                    lambda m: latest.update(plan=m), 1),
            node.create_subscription(Bool, '/avoid_v2/watchdog_stop',
                                     lambda m: latest.update(stop=m.data), 1)]
    course_pub = node.create_publisher(AvoidCourse, '/avoid_v2/course',
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    gps_pub = node.create_publisher(GpsPath, '/perception/gps_path', 1)
    route_pub = node.create_publisher(GpsRoute, '/perception/gps_route',
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    vv_pub = node.create_publisher(VehicleVector, '/vehicle/vector', qos_profile_sensor_data)
    scan_pub = node.create_publisher(LaserScan, '/avoid_v2/synthetic_scan', qos_profile_sensor_data)
    course = AvoidCourse(route_id='synthetic', entry_station_m=1., exit_station_m=12.)
    course.header.frame_id = 'map'
    route = GpsRoute(route_id='synthetic')
    route.header.frame_id = 'map'
    route.route.enabled = True
    route.route.route_id = 'synthetic'
    route.points = [RefPoint(x=i*.25) for i in range(81)]
    course.boundary = [RefPoint(x=x, y=y) for x, y in [(-2., -4.), (22., -4.), (22., 4.), (-2., 4.)]]
    angles = [-math.pi+i*2*math.pi/1440 for i in range(1441)]
    # Analytic first return from room walls, from a sensor at the rear axle (1,0).
    ranges = []
    for a in angles:
        dx, dy = math.cos(a), math.sin(a)
        candidates = []
        if abs(dx) > 1e-9:
            candidates.append((8.-1.)/dx if dx > 0 else (-2.-1.)/dx)
        if abs(dy) > 1e-9:
            candidates.append(4./abs(dy))
        ranges.append(min(candidates))
    paused = True
    in_zone = False
    zone_valid = True
    invalid_frame = False
    zero_timing = False
    bad_heading = False
    last_scan = 0.
    last_gps = 0.

    with tempfile.TemporaryDirectory(prefix='avoid_v2_smoke_') as tmp:
        params = Path(tmp)/'params.yaml'
        params.write_text('''avoid_v2_node:
  ros__parameters:
    sensor_ids: [sim]
    trigger_sensor: sim
    sensors.sim.frame: sim_laser
    sensors.sim.topic: /avoid_v2/synthetic_scan
    sensors.sim.fov_min_deg: -180.0
    sensors.sim.fov_max_deg: 180.0
    sensors.sim.max_range: 12.0
''')
        log = open(Path(tmp)/'node.log', 'w+')
        proc = subprocess.Popen([sys.argv[1], '--ros-args', '--params-file', str(params)], stdout=log, stderr=log)
        watchdog = subprocess.Popen([sys.executable, str(Path(__file__).parents[1]/'tools/watchdog.py')], stdout=log, stderr=log)

        def stamp_at(t):
            return rclpy.time.Time(nanoseconds=int(t*1e9)).to_msg()

        def spin(duration):
            nonlocal last_scan, last_gps
            end = time.monotonic()+duration
            while time.monotonic() < end:
                now = node.get_clock().now().nanoseconds*1e-9
                vv = VehicleVector(x=1., v=0.)
                vv.header.stamp = stamp_at(now-.01)
                vv_pub.publish(vv)
                if time.monotonic()-last_gps >= .10:
                    gps = GpsPath(fix_quality=4, position_valid=True, vehicle_heading_valid=True,
                                  position_x=1., heading_source=0 if bad_heading else GpsPath.HEADING_FUSED,
                                  zone_valid=zone_valid, avoid_zone=in_zone, points=[RefPoint(x=2.5)])
                    gps.reference_stamp = stamp_at(now-.04)
                    gps.route.enabled = True
                    gps.route.route_id = 'synthetic'
                    gps_pub.publish(gps)
                    last_gps = time.monotonic()
                if not paused and time.monotonic()-last_scan >= .10:
                    scan = LaserScan(angle_min=-math.pi, angle_max=math.pi,
                                     angle_increment=2*math.pi/1440, time_increment=0. if zero_timing else .05/1440,
                                     scan_time=.1, range_min=.05, range_max=12., ranges=ranges)
                    scan.header.frame_id = 'wrong' if invalid_frame else 'sim_laser'
                    scan.header.stamp = stamp_at(now-.12)
                    scan_pub.publish(scan)
                    last_scan = time.monotonic()
                rclpy.spin_once(node, timeout_sec=.008)
                if proc.poll() is not None or watchdog.poll() is not None:
                    log.flush();log.seek(0)
                    raise AssertionError(log.read())

        def expect(predicate, why, timeout=4.):
            until = time.monotonic()+timeout
            while time.monotonic() < until:
                spin(.02)
                if predicate(latest):
                    print('PASS:', why)
                    return
            p = latest.get('plan')
            raise AssertionError(f'{why}; last plan reason={p.reason if p else None}, latest={latest}')

        try:
            expect(lambda x: 'plan' in x and not x['plan'].plan_valid, 'missing course stays HOLD')
            course.header.stamp = node.get_clock().now().to_msg()
            course_pub.publish(course)
            expect(lambda x: not x['plan'].plan_valid, 'boundary alone cannot invent a GPS route')
            route.header.stamp = node.get_clock().now().to_msg()
            route_pub.publish(route)
            expect(lambda x: x['plan'].phase == AvoidPlan.READY and x['plan'].gps_follow
                   and not x['plan'].perception_active and not x['plan'].plan_valid,
                   'outside zone uses GPS ownership with no LiDAR input')
            expect(lambda x: x.get('stop') is False, 'outside zone watchdog accepts fresh GPS ownership')
            zone_valid = False
            expect(lambda x: x['plan'].phase == AvoidPlan.HOLD and x.get('stop') is True,
                   'invalid membership is not equivalent to outside zone')
            zone_valid = True
            in_zone = True
            expect(lambda x: not x['plan'].plan_valid and x['plan'].perception_active,
                   'zone entry waits for new scans')
            paused = False
            expect(lambda x: 'plan' in x and x['plan'].plan_valid, 'raw timestamped scan produces shadow plan')
            p = latest['plan']
            assert p.phase == AvoidPlan.WALL_FOLLOW and not p.gps_follow and not p.maneuver_active and p.expanded_nodes > 0
            assert len(p.reference.points) == 1 and len(p.path) > 10
            assert len(p.path_speed_mps) == len(p.path)
            assert 0 < p.speed_limit_mps <= .6 and abs(p.reference.v_suggest-p.speed_limit_mps) < 1e-6
            assert p.processing_ms <= 35 and p.planner_ms <= 20
            assert p.reference.points[0].x > .8 and not p.complete
            print('PASS: one-point vehicle reference and bounded processing')
            episode = p.episode_id
            for _ in range(4):
                course.header.stamp = node.get_clock().now().to_msg()
                route.header.stamp = node.get_clock().now().to_msg()
                course_pub.publish(course)
                route_pub.publish(route)
                spin(.15)
            assert latest['plan'].episode_id == episode
            expect(lambda x: x['plan'].plan_valid, 'identical course republish preserves session and observations')
            course.exit_station_m = 12.5
            course_pub.publish(course)
            expect(lambda x: x['plan'].episode_id > episode, 'changed course resets session')
            expect(lambda x: x['plan'].plan_valid, 'changed course reacquires localization and scans')
            zero_timing = True
            expect(lambda x: not x['plan'].plan_valid and 'scan' in x['plan'].reason,
                   'zero per-beam timing is rejected')
            zero_timing = False
            expect(lambda x: x['plan'].plan_valid, 'fresh scans restore plan')
            invalid_frame = True
            expect(lambda x: not x['plan'].plan_valid, 'wrong sensor frame invalidates plan')
            invalid_frame = False
            expect(lambda x: x['plan'].plan_valid, 'calibrated frame restores plan')
            bad_heading = True
            expect(lambda x: not x['plan'].plan_valid, 'tangent heading cannot authorize geometry')
            bad_heading = False
            expect(lambda x: x['plan'].plan_valid, 'measured heading restores plan')
            paused = True
            expect(lambda x: x.get('stop') is True and not x['plan'].plan_valid,
                   'scan silence expires reference and watchdog')
            paused = False
            expect(lambda x: x['plan'].plan_valid, 'new scan recovers shadow calculation')
            expect(lambda x: x.get('stop') is False, 'watchdog observes a fresh valid plan before freeze')
            proc.send_signal(signal.SIGSTOP)
            expect(lambda x: x.get('stop') is True, 'independent watchdog detects frozen planner')
            proc.send_signal(signal.SIGCONT)
            print('ROS smoke complete; no MGM/CAN command publisher was started')
        finally:
            if proc.poll() is None:
                proc.send_signal(signal.SIGCONT)
            for child in (proc, watchdog):
                if child.poll() is None:
                    child.terminate()
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        child.kill();child.wait()
            log.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
