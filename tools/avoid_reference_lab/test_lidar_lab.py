"""ROS integration check of the live-input lab using synthetic sensor packets.

source /opt/ros/humble/setup.bash; source install_v2/local_setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=186 python3 tools/avoid_reference_lab/test_lidar_lab.py
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from fma_interfaces.msg import AvoidStatus
from nav_msgs.msg import Path as RosPath
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

ROOT = Path(__file__).resolve().parents[2]


def main():
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1' and os.environ.get('ROS_DOMAIN_ID') == '186'
    rclpy.init()
    node = rclpy.create_node('avoid_lab_check')
    publisher = node.create_publisher(LaserScan, '/avoid_lab/test_scan', qos_profile_sensor_data)
    latest = {}
    subs = [node.create_subscription(kind, topic, lambda m, k=key: latest.update({k: m}), 1)
            for kind, topic, key in [(AvoidStatus, '/avoid_lab/status', 'status'),
                                    (RosPath, '/avoid_lab/path', 'path')]]
    client = node.create_client(Trigger, '/avoid_lab/reset')
    log_path = Path('/tmp/avoid-lidar-lab-check.log')
    with log_path.open('w') as log:
        proc = subprocess.Popen([str(ROOT/'scripts/avoid-lidar-lab'), '--no-rviz',
            '--scan-topic', '/avoid_lab/test_scan', '--forward-angle', '0'], stdout=log, stderr=log)
        def send(obstacle):
            scan = LaserScan(angle_min=-math.pi/2, angle_max=math.pi/2,
                angle_increment=math.pi/180, range_min=.05, range_max=12., ranges=[math.inf]*181)
            if obstacle:
                scan.ranges = [3.2/math.cos(math.radians(i-90))
                    if 0 < i < 180 and abs(3.2*math.tan(math.radians(i-90))) <= .2 else math.inf
                    for i in range(181)]
            scan.header.stamp = node.get_clock().now().to_msg()
            scan.header.frame_id = 'external_lidar_frame'
            publisher.publish(scan)
        def expect(predicate, label, obstacle=True, timeout=8.):
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                assert proc.poll() is None, log_path.read_text()[-3000:]
                if obstacle is not None:
                    send(obstacle)
                rclpy.spin_once(node, timeout_sec=.03)
                if predicate():
                    print('PASS:', label)
                    return
            raise AssertionError(label+'\n'+log_path.read_text()[-3000:])
        try:
            expect(lambda: 'status' in latest and latest['status'].obstacle_detected and
                latest['status'].points and 'path' in latest and len(latest['path'].poses)>10,
                'no zone input: AVOID_ACTIVE heartbeat enables production fixed-goal reference')
            goal = latest['status'].points[0]
            assert abs(goal.x-3.96)<.03 and abs(goal.y)>.5
            time.sleep(.15)
            expect(lambda: not latest['status'].obstacle_detected and latest['status'].points and
                abs(latest['status'].points[0].y-goal.y)<1e-5,
                'clear scan retains the emitted obstacle goal', obstacle=False)
            assert client.wait_for_service(timeout_sec=2.)
            future = client.call_async(Trigger.Request())
            expect(lambda: future.done() and future.result().success and latest['status'].points and
                abs(latest['status'].points[0].y)<1e-5 and abs(latest['status'].points[0].x-1.)<1e-5,
                'reset clears fixed goals and returns to local straight reference', obstacle=False)
            names = {name for name, types in node.get_topic_names_and_types()}
            assert '/adas/target_ref' not in names and '/perception/avoid' not in names
            print('PASS: private outputs; no vehicle target or production avoidance topic')
        finally:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=15)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
