#!/usr/bin/env python3
"""Measured recovery executor. MGM alone decides ESTOP entry/exit and CAN output."""
import math
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from fma_interfaces.msg import MgmState, RouteControl, VehicleVector, CanHealth, EstopRecovery, RefPoint
from estop_recovery_core import Recovery, rear_corridor_clear, fresh


def stamp_s(stamp):
    return stamp.sec+stamp.nanosec*1.e-9


class EstopRecoveryNode(Node):
    def __init__(self):
        super().__init__('estop_recovery')
        geometry=Path(get_package_share_directory('lidar_fusion_v2'))/'config/fixed_geometry.yaml'
        cfg=yaml.safe_load(geometry.read_text())['/**']['ros__parameters']['sensors']['a2']
        self.mount=[cfg[k] for k in ('x','y','yaw_deg','fov_min_deg','fov_max_deg','range_offset_m','min_range','max_range')]
        self.core=Recovery()
        self.mgm=self.can=self.vehicle=self.rear=None
        self.rear_clear=False
        self.vehicle_counter=None
        self.create_subscription(MgmState,'/adas/mgm_state',self.on_mgm,1)
        self.create_subscription(CanHealth,'/bridge/can_health',self.on_can,1)
        self.create_subscription(VehicleVector,'/vehicle/vector',self.on_vehicle,qos_profile_sensor_data)
        self.create_subscription(LaserScan,cfg['topic'],self.on_rear,qos_profile_sensor_data)
        self.pub=self.create_publisher(EstopRecovery,'/planning/estop_recovery',1)
        self.status=self.create_publisher(String,'/planning/estop_recovery_status',1)
        self.create_timer(.02,self.tick)

    def on_mgm(self,msg):
        self.mgm=msg

    def on_can(self,msg):
        self.can=msg

    def on_vehicle(self,msg):
        if self.vehicle is not None and (msg.counter==self.vehicle_counter or
                stamp_s(msg.header.stamp)<=stamp_s(self.vehicle.header.stamp)):
            return
        self.vehicle,self.vehicle_counter=msg,msg.counter

    def on_rear(self,msg):
        if self.rear is not None and stamp_s(msg.header.stamp)<=stamp_s(self.rear.header.stamp):
            return
        self.rear=msg
        self.rear_clear=(msg.header.frame_id.lstrip('/')=='lidar_a2_link' and
            rear_corridor_clear(msg.ranges,msg.angle_min,msg.angle_increment,
                                msg.range_min,msg.range_max,self.mount))

    def tick(self):
        now=self.get_clock().now()
        seconds=now.nanoseconds*1.e-9
        m=self.mgm
        m_fresh=m is not None and fresh(stamp_s(m.header.stamp),seconds)
        can_ok=self.can is not None and fresh(stamp_s(self.can.header.stamp),seconds) and self.can.link_up and self.can.tx_ok and self.can.consecutive_tx_fail==0
        authorized=bool(m_fresh and can_ok and m.go_authorized and m.top==1 and m.route.phase not in (RouteControl.FINISHED,RouteControl.FAULT))
        speed=self.vehicle.v if self.vehicle else math.nan
        speed_stamp=stamp_s(self.vehicle.header.stamp) if self.vehicle else -math.inf
        rear_stamp=stamp_s(self.rear.header.stamp) if self.rear else -math.inf
        # A stale MGM message is authority loss, never proof that an episode ended.
        active=bool(m.estop_active) if m is not None else self.core.request!=0
        request=m.estop_request_id if m is not None else self.core.request
        velocity,done=self.core.step(time.monotonic(),seconds,request,active,authorized,
                                    speed,speed_stamp,self.rear_clear,rear_stamp)
        if not active or not request:
            return
        msg=EstopRecovery(request_id=request,v_suggest=velocity,done=done)
        msg.header.stamp=now.to_msg()
        if m_fresh and self.vehicle is not None and self.rear is not None:
            msg.reference_stamp=min((m.header.stamp,self.vehicle.header.stamp,self.rear.header.stamp),key=stamp_s)
        if velocity < 0 and authorized:
            msg.points=[RefPoint(x=-1.,y=0.,yaw=0.,curvature=0.)]
        self.pub.publish(msg)
        self.status.publish(String(data=f'{request} {self.core.phase} distance={self.core.distance:.3f}m {self.core.reason}'))


def main():
    rclpy.init()
    node=EstopRecoveryNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
