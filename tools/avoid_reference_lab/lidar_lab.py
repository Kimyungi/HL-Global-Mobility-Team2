#!/usr/bin/env python3
"""Stationary live-LiDAR reference lab: production planner, private ROS outputs."""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from fma_interfaces.msg import AvoidStatus, GpsPath, MgmState, RefPoint, VehicleVector
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
import yaml

ROOT = Path(__file__).resolve().parents[2]
PREFIX = '/avoid_lab'


class Inputs(Node):
    def __init__(self, args):
        super().__init__('avoid_lab_inputs')
        self.args, self.last_scan, self.last_log = args, None, 0.
        self.last_status = None
        self.state = self.create_publisher(MgmState, PREFIX+'/mgm_state', 1)
        self.session = self.create_publisher(Bool, PREFIX+'/start_session', 1)
        self.vehicle = self.create_publisher(VehicleVector, PREFIX+'/vehicle', 1)
        self.gps = self.create_publisher(GpsPath, PREFIX+'/gps', 1) if not args.gps_topic else None
        self.markers = self.create_publisher(MarkerArray, PREFIX+'/guide', 1)
        self.scan_pub = self.create_publisher(LaserScan, PREFIX+'/input_scan', qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(LaserScan, args.scan_topic, self.scan, qos_profile_sensor_data)
        self.status_sub = self.create_subscription(AvoidStatus, PREFIX+'/status', self.status, 1)
        self.reset_service = self.create_service(Trigger, PREFIX+'/reset', self.reset)
        self.timer = self.create_timer(.1, self.tick)

    def scan(self, msg):
        self.last_scan = time.monotonic()
        # Raw a1 coordinates are unchanged; use the planner-owned calibrated TF.
        # Preserve the original sensor timestamp for freshness validation.
        msg.header.frame_id = 'laser_frame'
        self.scan_pub.publish(msg)

    def status(self, msg):
        self.last_status = msg

    def reset(self, request, response):
        self.session.publish(Bool(data=True))
        response.success = True
        response.message = '회피 목표 초기화 요청: 다음 유효 스캔에서 새 장면 계산'
        return response

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        state = MgmState(avoidance=1)
        state.header.stamp = stamp
        self.state.publish(state)
        # Zero is the real test assumption: parked vehicle, no TTC motion.
        vehicle = VehicleVector(v=0.)
        vehicle.header.stamp = stamp
        self.vehicle.publish(vehicle)
        if self.gps:
            gps = GpsPath(fix_quality=4, position_valid=True, vehicle_heading_valid=True,
                heading_source=GpsPath.HEADING_FUSED, waypoint_window_valid=True,
                waypoint_stations=list(map(float, range(21))),
                waypoint_points=[RefPoint(x=float(i)) for i in range(21)])
            gps.header.stamp = gps.reference_stamp = stamp
            gps.header.frame_id = 'base_link'
            self.gps.publish(gps)
        fresh = self.last_scan is not None and time.monotonic()-self.last_scan < .5
        text = Marker()
        text.header.stamp, text.header.frame_id = stamp, 'base_link'
        text.ns, text.id = 'lab', 0
        text.type, text.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.orientation.w = 1.
        text.pose.position.x, text.pose.position.y = 1., 2.5
        text.scale.z = .16
        text.color.r, text.color.g, text.color.b, text.color.a = 1., 1. if fresh else .3, .4, 1.
        route = 'LIVE GPS' if self.args.gps_topic else 'LOCAL STRAIGHT / PARKED VEHICLE'
        text.text = f'AVOID_ACTIVE test | {route}\n'+('LIVE LIDAR' if fresh else 'WAITING FOR LIVE LIDAR')
        if self.last_status and fresh:
            m = self.last_status
            target = f'({m.points[0].x:.2f}, {m.points[0].y:+.2f})' if m.points else 'NONE'
            text.text += f'\nscan_valid={m.scan_valid} | obstacle={m.obstacle_detected} | target={target}'
        markers = [text]
        goal = Marker()
        goal.header = text.header
        goal.ns, goal.id = 'lab', 2
        goal.type = Marker.SPHERE
        goal.pose.orientation.w = 1.
        goal.scale.x = goal.scale.y = goal.scale.z = .18
        goal.color.g, goal.color.a = 1., 1.
        goal.action = Marker.DELETE
        if fresh and self.last_status and self.last_status.points:
            goal.action = Marker.ADD
            goal.pose.position.x = self.last_status.points[0].x
            goal.pose.position.y = self.last_status.points[0].y
        markers.append(goal)
        if self.gps:
            line = Marker()
            line.header = text.header
            line.ns, line.id = 'lab', 1
            line.type, line.action = Marker.LINE_STRIP, Marker.ADD
            line.pose.orientation.w = 1.
            line.scale.x = .025
            line.color.b, line.color.g, line.color.a = 1., .55, .8
            line.points = [Point(x=float(i)) for i in range(11)]
            markers.append(line)
        self.markers.publish(MarkerArray(markers=markers))
        if time.monotonic()-self.last_log > 3:
            self.last_log = time.monotonic()
            self.get_logger().info(text.text.replace('\n', ' | '))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='Start the verified a1 driver on this device; otherwise subscribe only')
    parser.add_argument('--scan-topic', default='/lidar/a1/scan')
    parser.add_argument('--gps-topic', help='Use an existing GpsPath instead of the stationary local straight route')
    parser.add_argument('--forward-angle', type=float, help='Raw scan forward angle; default from a1 calibration')
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--duration', type=float, default=0., help='Auto-exit after N seconds; 0 runs until Ctrl-C')
    args = parser.parse_args()
    if args.port and not os.access(args.port, os.R_OK | os.W_OK):
        parser.error(f'Cannot read/write lidar device: {args.port}')
    params = ROOT/'src/stack_avoid/config/params.yaml'
    geometry = yaml.safe_load((ROOT/'src/lidar_fusion_v2/config/fixed_geometry.yaml').read_text())['/**']['ros__parameters']
    forward = args.forward_angle if args.forward_angle is not None else (-geometry['sensors']['a1']['yaw_deg']) % 360
    if not math.isfinite(forward) or args.duration < 0:
        parser.error('invalid angle or duration')
    if args.scan_topic in (PREFIX+'/input_scan', PREFIX+'/scan_front'):
        parser.error('scan-topic must be an external input, not the lab output')
    # Every planner output and simulated input is separate from the vehicle stack.
    remaps = {'/perception/avoid': PREFIX+'/status', '/perception/avoid_path': PREFIX+'/path',
        '/perception/avoid_station': PREFIX+'/station', '/scan_front': PREFIX+'/scan_front',
        '/perception/gps_path': args.gps_topic or PREFIX+'/gps', '/adas/mgm_state': PREFIX+'/mgm_state',
        '/operator/start_session': PREFIX+'/start_session', '/vehicle/vector': PREFIX+'/vehicle',
        '/tf_static': PREFIX+'/tf_static'}
    procs = []
    rclpy.init(args=[])
    node = Inputs(args)
    with tempfile.TemporaryDirectory(prefix='avoid-lidar-lab-') as tmp:
        def start(cmd):
            procs.append(subprocess.Popen(cmd))
        try:
            if args.port:
                from lidar_fusion_v2.driver_profiles import parameters
                driver = parameters('a1', args.port)
                driver['frame_id'] = 'laser_frame'
                driver_params = Path(tmp)/'driver.yaml'
                driver_params.write_text(yaml.safe_dump({'/**': {'ros__parameters': driver}}))
                start([str(ROOT/'install_v2/ydlidar_ros2_driver/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node'),
                    '--ros-args', '--params-file', str(driver_params), '-r', '__node:=avoid_lab_lidar',
                    '-r', '/scan:='+args.scan_topic])
            command = [str(ROOT/'install_v2/stack_avoid/lib/stack_avoid/stack_avoid_node'),
                '--ros-args', '--params-file', str(params), '-r', '__node:=avoid_lab_planner',
                '-p', 'avoid.require_mgm_active:=true', '-p', 'avoid.compute_backend:=native',
                '-p', 'scan_topic:='+PREFIX+'/input_scan', '-p', f'lidar_mount.forward_angle_deg:={forward}']
            for source, target in remaps.items():
                command += ['-r', source+':='+target]
            start(command)
            if not args.no_rviz:
                start(['rviz2', '-d', str(Path(__file__).with_name('lidar_lab.rviz')),
                    '--ros-args', '-r', '__node:=avoid_lab_rviz', '-r', '/tf_static:='+PREFIX+'/tf_static'])
            print('새 장애물 배치마다: ros2 service call /avoid_lab/reset std_srvs/srv/Trigger "{}"', flush=True)
            deadline = time.monotonic()+args.duration if args.duration else math.inf
            while rclpy.ok() and time.monotonic() < deadline:
                if any(p.poll() is not None for p in procs):
                    raise RuntimeError('A lab process exited; see its output above')
                rclpy.spin_once(node, timeout_sec=.1)
        except KeyboardInterrupt:
            pass
        finally:
            for proc in procs:
                if proc.poll() is None:
                    proc.send_signal(signal.SIGINT)
            for proc in procs:
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.terminate()
                    proc.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
