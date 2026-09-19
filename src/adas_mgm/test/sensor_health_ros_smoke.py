"""Real MGM, mock seven-sensor inputs; isolated ROS domain and no CAN driver."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
    raise SystemExit('Use localhost and an isolated ROS_DOMAIN_ID')
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from fma_interfaces.msg import GpsPath, LanePath, RefPoint, MgmState, TargetRef, TrafficStop


def main():
    rclpy.init()
    node = rclpy.create_node('seven_sensor_health_test')
    health_topics = ['/perception/lane_camera', '/perception/traffic_camera',
                     '/lidar/a1/scan', '/lidar/a2/scan', '/lidar/b1/scan', '/lidar/b2/scan',
                     '/perception/gps_path']
    messages = {t: Header() for t in health_topics[:2]}
    messages.update({t: LaserScan(angle_min=0., angle_max=.1, angle_increment=.1,
                                 range_min=.05, range_max=10., ranges=[float('inf')]*2)
                     for t in health_topics[2:6]})
    messages[health_topics[6]] = GpsPath(fix_quality=5, position_valid=True, points=[RefPoint(x=1.)])
    # Healthy control path without a camera heartbeat must not fabricate camera health.
    messages['/perception/lane_path'] = LanePath(confidence=.9, points=[RefPoint(x=1.)])
    # Healthy traffic results likewise must not fabricate the second camera heartbeat.
    messages['/perception/traffic_stop'] = TrafficStop()
    pubs = {t: node.create_publisher(type(m),t,qos_profile_sensor_data if isinstance(m,LaserScan) else 1)
            for t,m in messages.items()}
    latest={}
    subs=[node.create_subscription(k,t,lambda m,key=key:latest.update({key:m}),10)
          for k,t,key in [(MgmState,'/adas/mgm_state','s'),(TargetRef,'/adas/target_ref','r')]]
    active=set(); frozen=False
    with tempfile.TemporaryFile(mode='w+') as log:
        proc=subprocess.Popen([sys.argv[1],'--ros-args','-p','wait_go:=false',
            '-p','safe_stop_all_sensors_only:=true','-p','escape_after_cycles:=0',
            '-p','required_lidar_topics:=['+','.join(health_topics[2:6])+']'],stdout=log,stderr=log)
        def expect(mask,label):
            deadline=time.monotonic()+4
            while time.monotonic()<deadline:
                now=node.get_clock().now().to_msg()
                for topic,msg in messages.items():
                    if topic in health_topics and topic not in active:continue
                    if not frozen or topic not in health_topics:
                        if isinstance(msg,Header):msg.stamp=now
                        else:msg.header.stamp=now
                        if hasattr(msg,'reference_stamp'):msg.reference_stamp=now
                    pubs[topic].publish(msg)
                rclpy.spin_once(node,timeout_sec=.025)
                s=latest.get('s');r=latest.get('r')
                if s and r and s.sensor_alive_mask==mask and s.safe_stop_all_sensors_only:
                    assert (s.safety==3)==(mask==0),s
                    assert s.active_safe_stop_reasons==(2 if mask==0 else 0),s
                    if mask==0 and r.v_ref!=0:continue  # state and command arrive separately
                    print('PASS:',label,flush=True);return
            log.seek(0); print(log.read()[-3000:]);raise AssertionError((label,latest.get('s')))
        try:
            expect(0,'path/traffic result messages do not imply physical sensors alive')
            for bit,topic in enumerate(health_topics):
                active={topic};frozen=False
                expect(1<<bit,'only '+topic+' alive releases SAFE_STOP')
                frozen=True
                expect(0,'republication of old timestamp expires '+topic)
            active=set(health_topics);frozen=False
            expect(127,'all seven independently visible; GPS FLOAT accepted')
            messages[health_topics[6]].fix_quality=0
            expect(63,'GPS NO FIX unavailable despite continuing messages')
            messages[health_topics[6]].fix_quality=4
            expect(127,'GPS FIX recovery')
        finally:
            proc.terminate()
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
            node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
