"""Real callbacks with fake clocks/clouds: no hardware or ROS node startup."""
from types import SimpleNamespace as NS, MethodType
import math
import numpy as np
from rclpy.time import Time
from std_msgs.msg import Header
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from fma_interfaces.msg import ParkingCommand, MgmState, VehicleVector
from stack_parking.geometry import Pose2
from stack_parking.lateral_wall import LateralWallAcquisition, LateralWallConfig
from stack_parking.lateral_wall_node import LateralWallNode


def header(t, frame='base_link'):
    return Header(stamp=Time(seconds=t).to_msg(), frame_id=frame)


def node():
    n=NS(tracker=LateralWallAcquisition(LateralWallConfig(.215329,.211549)),
         request_id=0, mgm_request_id=0, mission_active=False, mgm_stamp=0., entry_stamp=0.,
         vehicle_speed=math.nan, vehicle_stamp=0., stopped_since=None,
         collected_frames=0, collection_generation=None,
         pose=None, pose_stamp=0., slam_valid=False, slam_stamp=0., now=100.)
    n.stamp=LateralWallNode.stamp
    n.now_s=lambda:n.now
    for method in ('on_mgm','on_command','on_vehicle','acquisition_gate','on_cloud','on_reset'):
        setattr(n,method,MethodType(getattr(LateralWallNode,method),n))
    enter(n,101)
    return n


def enter(n, request):
    n.on_command(ParkingCommand(action=ParkingCommand.PREPARE, request_id=request))
    n.on_mgm(MgmState(header=header(n.now),mission=1,mission_request_active=True,
                     mission_request_id=request))


def sample(n,t,speed=.1,scan_time=None):
    n.now=t
    n.on_mgm(MgmState(header=header(t),mission=1,mission_request_active=True,
                     mission_request_id=n.request_id))
    n.on_vehicle(VehicleVector(header=header(t),v=float(np.float32(speed))))
    n.pose=Pose2(); n.pose_stamp=t; n.slam_valid=True; n.slam_stamp=t
    # Real nonempty scan with no left wall. Collection completion must not
    # depend on detecting a parking space or fitting a wall successfully.
    cloud=create_cloud_xyz32(header(t if scan_time is None else scan_time),[(1.,0.,0.)])
    n.on_cloud(cloud)


def test_stop_then_five_new_frames_resume_even_without_wall():
    n=node()
    sample(n,100.,.11)
    assert n.collected_frames==0 and n.tracker.reason=='WAITING_FOR_STOP'
    sample(n,100.1,.1,100.)  # queued before actual stopping
    assert n.collected_frames==0
    for i in range(4):
        sample(n,100.2+i*.1)
        assert n.collected_frames==i+1
        sample(n,100.2+i*.1)  # duplicate publication
        assert n.collected_frames==i+1
    sample(n,100.6)
    assert n.collected_frames==5 and n.tracker.wall is None
    sample(n,100.7,1.)
    assert n.collected_frames==5 and n.acquisition_gate(n.now)


def test_moving_or_missing_can_before_completion_restarts_collection():
    n=node(); sample(n,100.,0.); sample(n,100.1,0.)
    sample(n,100.2,-.11)
    assert n.collected_frames==0
    sample(n,100.3,-.1)
    assert n.collected_frames==1
    n.mgm_stamp=100.6
    assert not n.acquisition_gate(100.6)
    assert n.collected_frames==0 and n.tracker.reason=='WAITING_FOR_CAN_SPEED'


def test_new_request_after_prepare_first_requires_new_stop_and_frames():
    n=node()
    for i in range(5): sample(n,100.+i*.1)
    n.now=101.; enter(n,102)
    assert n.entry_stamp==101. and n.collected_frames==0
    assert not n.acquisition_gate(101.)  # prior request's speed is not evidence
    sample(n,101.,0.,100.9)
    assert n.collected_frames==0
    sample(n,101.1,0.)
    assert n.collected_frames==1


def test_missing_slam_and_stale_scan_do_not_count():
    n=node(); n.on_vehicle(VehicleVector(header=header(100.),v=0.))
    cloud=create_cloud_xyz32(header(100.),[(1.,0.,0.)])
    n.on_cloud(cloud)
    assert n.collected_frames==0
    sample(n,100.5,0.,100.)
    assert n.collected_frames==0
