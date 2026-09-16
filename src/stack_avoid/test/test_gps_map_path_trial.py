"""Production callbacks: GPS waypoint ENU anchoring, frame invariance and loss."""
import copy
import math
import pytest
import rclpy
from fma_interfaces.msg import GpsPath, VehicleVector
from sensor_msgs.msg import LaserScan
from stack_avoid.main_gap_path_trial import MainGapPathTrialNode


class Capture:
    def __init__(self): self.messages=[]
    def publish(self,msg): self.messages.append(copy.deepcopy(msg))


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args','-p','lidar_mount.forward_angle_deg:=0.0',
                    '-p','avoid.compute_backend:=native'])
    n=MainGapPathTrialNode();n.require_active=False
    n._output,n.path_pub,n.raw_pub=Capture(),Capture(),Capture()
    yield n
    n.destroy_node();rclpy.shutdown()


def gps(n,x=100.,y=200.,yaw=math.pi/2,valid=True,source=GpsPath.HEADING_FUSED):
    m=GpsPath(position_valid=valid,fix_quality=4,position_x=x,position_y=y,
              vehicle_heading_valid=True,vehicle_heading_rad=yaw,heading_source=source)
    m.header.stamp=m.reference_stamp=n.get_clock().now().to_msg()
    n._on_gps(m)
    return m


def scan(n,invalid=False):
    s=LaserScan(angle_min=-math.pi/2,angle_max=math.pi/2,angle_increment=math.pi/180,
                range_min=.05,range_max=12.)
    s.header.stamp=n.get_clock().now().to_msg()
    s.ranges=[1.9/math.cos(math.radians(i-90)) if abs(1.9*math.tan(math.radians(i-90)))<.2
              else math.inf for i in range(181)]
    if invalid:s.ranges=[math.nan]*181
    n.on_scan(s)


def geometry(msg):
    return [(p.pose.position.x,p.pose.position.y,p.pose.orientation.z,p.pose.orientation.w)
            for p in msg.poses]


def test_map_path_stays_absolute_while_gps_vehicle_moves(node):
    gps(node);scan(node)
    original=node.path_pub.messages[-1];frozen=geometry(original)
    assert original.header.frame_id=='map' and len(frozen)>50
    assert all(p.header.frame_id=='map' for p in original.poses)
    assert frozen[0][:2]==pytest.approx((100.,200.))
    assert frozen[-1][:2]==pytest.approx((100.,205.36),abs=.02)
    first=copy.deepcopy(node._output.messages[-1].points[0])
    for x,y,yaw in [(100.,200.2,math.pi/2),(99.95,200.4,1.6),(100.,200.2,math.pi/2)]:
        gps(node,x,y,yaw)
        vv=VehicleVector(x=9999.,y=-9999.,yaw=-2.,v=.5)
        vv.header.stamp=node.get_clock().now().to_msg();node.on_vehicle_vector(vv)
        scan(node)
        assert geometry(node.path_pub.messages[-1])==frozen
        assert node._output.messages[-1].points
    assert node._output.messages[-1].points[0]!=first


def test_gps_loss_and_invalid_scan_keep_map_geometry(node):
    gps(node);scan(node);frozen=geometry(node.path_pub.messages[-1])
    gps(node,valid=False);scan(node)
    assert not node._output.messages[-1].points
    assert geometry(node.path_pub.messages[-1])==frozen
    gps(node,y=200.1);scan(node)
    assert node._output.messages[-1].points
    assert geometry(node.path_pub.messages[-1])==frozen
    scan(node,invalid=True)
    assert not node._output.messages[-1].scan_valid
    assert geometry(node.path_pub.messages[-1])==frozen


def test_tangent_heading_cannot_anchor_absolute_path(node):
    gps(node,source=GpsPath.HEADING_TANGENT);scan(node)
    assert not node._output.messages[-1].points
    assert not node.path_pub.messages[-1].poses
    gps(node);scan(node)
    assert node._output.messages[-1].points
