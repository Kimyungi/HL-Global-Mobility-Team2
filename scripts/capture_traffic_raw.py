#!/usr/bin/env python3
"""Record unannotated traffic camera frames; never open a camera or start control."""
import argparse
import csv
import json
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--count', type=int, default=200)
    parser.add_argument('--fps', type=float, default=10.0, help='Maximum save rate; no duplicate frames')
    parser.add_argument('--timeout', type=float, default=180.0)
    parser.add_argument('--output', type=Path, default=Path('captures') / datetime.now().strftime('traffic_raw_%Y%m%d_%H%M%S'))
    args = parser.parse_args()
    if args.count <= 0 or args.fps <= 0 or args.timeout <= 0:
        parser.error('count, fps and timeout must be positive')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('traffic_raw_recorder')
    times = []
    started = time.monotonic()
    dimensions = None
    with (args.output / 'timestamps.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['file', 'receive_monotonic_s', 'ros_stamp_ns'])

        def receive(msg):
            nonlocal dimensions
            now = time.monotonic()
            if len(times) >= args.count or (times and now - times[-1] < 0.95 / args.fps):
                return
            if msg.encoding not in ('bgr8', 'rgb8'):
                raise ValueError(f'Unsupported image encoding: {msg.encoding}')
            pixels = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            pixels = pixels[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
            name = f'{len(times) + 1:04d}.png'
            if not cv2.imwrite(str(args.output / name), pixels):
                raise OSError(f'Cannot write {name}')
            writer.writerow([name, now, msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec])
            stream.flush()
            times.append(now)
            dimensions = [msg.width, msg.height]
            if len(times) % 25 == 0:
                print(f'Saved {len(times)}/{args.count}', flush=True)

        node.create_subscription(Image, '/perception/traffic_image_raw', receive, qos_profile_sensor_data)
        print(f'Recording raw frames to {args.output.resolve()}', flush=True)
        try:
            while len(times) < args.count and time.monotonic() - started < args.timeout:
                rclpy.spin_once(node, timeout_sec=0.5)
        except KeyboardInterrupt:
            pass
        finally:
            duration = times[-1] - times[0] if len(times) > 1 else 0.0
            summary = dict(count=len(times), requested_count=args.count, requested_fps=args.fps,
                           actual_fps=(len(times) - 1) / duration if duration else 0.0,
                           duration_s=duration, dimensions=dimensions, overlay=False,
                           topic='/perception/traffic_image_raw')
            (args.output / 'capture_info.json').write_text(json.dumps(summary, indent=2) + '\n')
            print(json.dumps(summary), flush=True)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0 if len(times) == args.count else 1


if __name__ == '__main__':
    raise SystemExit(main())
