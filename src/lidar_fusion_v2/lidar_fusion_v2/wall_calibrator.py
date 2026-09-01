"""Capture walls from anchor/side overlaps and solve side LiDAR extrinsics."""

import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import yaml

from .calibration import fit_line_ransac, solve_sensor_pose
from .geometry import SensorGeometry, local_to_base, scan_to_local


PAIRS = {
    # base_link overlap sectors; a1/a2 remain immutable anchors.
    'front_left': ('a1', 'b1', 35.0, 90.0),
    'rear_left': ('a2', 'b1', 110.0, 145.0),
    'front_right': ('a1', 'b2', -90.0, -35.0),
    'rear_right': ('a2', 'b2', -145.0, -110.0),
}


def _load_geometry(path):
    raw = yaml.safe_load(Path(path).read_text())['/**']['ros__parameters']
    output = {}
    for sensor_id, value in raw['sensors'].items():
        output[sensor_id] = SensorGeometry(
            sensor_id=sensor_id, x=float(value['x']), y=float(value['y']),
            yaw_deg=float(value['yaw_deg']),
            fov_min_deg=float(value['fov_min_deg']),
            fov_max_deg=float(value['fov_max_deg']),
            min_range=float(value['min_range']), max_range=float(value['max_range']),
            range_offset=float(value.get('range_offset_m', 0.0)))
    topics = {key: value['topic'] for key, value in raw['sensors'].items()}
    return output, topics


def _sector(points, minimum, maximum):
    angle = np.rad2deg(np.arctan2(points[:, 1], points[:, 0]))
    return (angle >= minimum) & (angle <= maximum)


class CaptureNode(Node):
    def __init__(self, anchor, target, topics):
        super().__init__('lidar_wall_capture')
        self.messages = {anchor: [], target: []}
        self._scan_subscriptions = []
        for sensor_id in self.messages:
            self._scan_subscriptions.append(self.create_subscription(
                LaserScan, topics[sensor_id],
                lambda msg, sid=sensor_id: self.messages[sid].append(msg),
                qos_profile_sensor_data))


def _capture(args):
    geometry, topics = _load_geometry(args.config)
    anchor, target, sector_min, sector_max = PAIRS[args.pair]
    rclpy.init(args=[])
    node = CaptureNode(anchor, target, topics)
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    messages = node.messages
    node.destroy_node()
    rclpy.shutdown()
    if len(messages[anchor]) < 3 or len(messages[target]) < 3:
        raise RuntimeError(
            f'insufficient scans: {anchor}={len(messages[anchor])}, '
            f'{target}={len(messages[target])}')

    anchor_points = []
    target_local = []
    for msg in messages[anchor][-args.frames:]:
        local = scan_to_local(msg.ranges, msg.angle_min, msg.angle_increment,
                              geometry[anchor])
        base = local_to_base(local, geometry[anchor])
        anchor_points.append(base[_sector(base, sector_min, sector_max)])
    for msg in messages[target][-args.frames:]:
        local = scan_to_local(msg.ranges, msg.angle_min, msg.angle_increment,
                              geometry[target])
        base = local_to_base(local, geometry[target])
        target_local.append(local[_sector(base, sector_min, sector_max)])
    anchor_points = np.concatenate(anchor_points)
    target_local = np.concatenate(target_local)
    normal, offset, inliers = fit_line_ransac(
        anchor_points, threshold=args.line_threshold)

    nominal_target = local_to_base(target_local, geometry[target])
    same_wall = np.abs(nominal_target @ normal + offset) <= args.match_threshold
    target_local = target_local[same_wall]
    nominal_target = nominal_target[same_wall]
    if len(target_local) < args.minimum_points:
        raise RuntimeError(
            f'target found only {len(target_local)} matching wall points; '
            'move a flat wall into the selected overlap')
    target_normal, _, target_inliers = fit_line_ransac(
        nominal_target, threshold=args.line_threshold)
    alignment = abs(float(normal @ target_normal))
    angle_error = math.degrees(math.acos(np.clip(alignment, -1.0, 1.0)))
    target_local = target_local[target_inliers]
    nominal_target = nominal_target[target_inliers]
    anchor_tangent = np.array((-normal[1], normal[0]))
    wall_span = float(np.ptp(nominal_target @ anchor_tangent))
    if angle_error > args.maximum_angle_error:
        raise RuntimeError(
            f'target line differs from anchor by {angle_error:.1f}deg; '
            'both sensors must see the same flat wall')
    if len(target_local) < args.minimum_points or wall_span < args.minimum_span:
        raise RuntimeError(
            f'wall evidence too small: points={len(target_local)}, '
            f'span={wall_span:.3f}m')
    if len(target_local) > 500:
        indices = np.linspace(0, len(target_local) - 1, 500).astype(int)
        target_local = target_local[indices]

    record = {
        'pair': args.pair, 'anchor': anchor, 'target': target,
        'sector_deg': [sector_min, sector_max],
        'normal': normal.tolist(), 'offset': offset,
        'anchor_inliers': int(inliers.sum()),
        'anchor_rms_m': float(np.sqrt(np.mean(np.square(
            anchor_points[inliers] @ normal + offset)))),
        'target_nominal_angle_error_deg': angle_error,
        'target_span_m': wall_span,
        'target_local': target_local.tolist(), 'created_unix': time.time(),
    }
    dataset = Path(args.dataset)
    content = json.loads(dataset.read_text()) if dataset.exists() else {'captures': []}
    content['captures'].append(record)
    dataset.parent.mkdir(parents=True, exist_ok=True)
    dataset.write_text(json.dumps(content, indent=2) + '\n')
    print(f"captured {args.pair}: anchor={record['anchor_inliers']} "
          f"target={len(target_local)} anchor_rms={record['anchor_rms_m']:.4f}m")
    print(f'dataset={dataset}')


def _solve(args):
    geometry, _ = _load_geometry(args.config)
    sensor_id = 'b1' if args.side == 'left' else 'b2'
    content = json.loads(Path(args.dataset).read_text())
    captures = [item for item in content['captures'] if item['target'] == sensor_id]
    initial = (geometry[sensor_id].x, geometry[sensor_id].y,
               math.radians(geometry[sensor_id].yaw_deg))
    pose, rms, result = solve_sensor_pose(captures, initial)
    if not result.success:
        raise RuntimeError(result.message)
    delta = pose - np.asarray(initial)
    if np.linalg.norm(delta[:2]) > 0.15 or abs(math.degrees(delta[2])) > 10.0:
        raise RuntimeError(
            'solution exceeds the safe correction limit '
            f'(translation={np.linalg.norm(delta[:2]):.3f}m, '
            f'yaw={math.degrees(delta[2]):.2f}deg); recapture the walls')
    print(f'{sensor_id} captures={len(captures)} RMS={rms:.4f}m')
    print(f'x: {pose[0]:.6f}')
    print(f'y: {pose[1]:.6f}')
    print(f'yaw_deg: {math.degrees(pose[2]):.6f}')
    print('front/rear values and every FOV value remain unchanged')


def _parser():
    default_config = str(Path(get_package_share_directory('lidar_fusion_v2')) /
                         'config/fixed_geometry.yaml')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=default_config)
    parser.add_argument('--dataset', default='/tmp/lidar_wall_calibration.json')
    sub = parser.add_subparsers(dest='command', required=True)
    capture = sub.add_parser('capture')
    capture.add_argument('--pair', required=True, choices=PAIRS)
    capture.add_argument('--seconds', type=float, default=2.0)
    capture.add_argument('--frames', type=int, default=8)
    capture.add_argument('--line-threshold', type=float, default=0.03)
    capture.add_argument('--match-threshold', type=float, default=0.20)
    capture.add_argument('--minimum-points', type=int, default=40)
    capture.add_argument('--minimum-span', type=float, default=0.30)
    capture.add_argument('--maximum-angle-error', type=float, default=12.0)
    solve = sub.add_parser('solve')
    solve.add_argument('--side', required=True, choices=('left', 'right'))
    return parser


def main():
    args = _parser().parse_args(sys.argv[1:])
    if args.command == 'capture':
        _capture(args)
    else:
        _solve(args)
