"""Production callbacks with real ROS messages; no sensor/CAN nodes are launched."""
import math
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from fma_interfaces.msg import GpsPath, MgmState
from stack_avoid.waypoint_node import WaypointAvoidNode
from stack_gps.path_engine import load_waypoints_csv, M_PER_DEG_LAT

ROOT = Path(__file__).parents[2] / 'stack_gps/waypoints'

@pytest.fixture
def adapter(monkeypatch):
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    route = ROOT/'halla_0919_path_04.csv'
    origin = ROOT/'halla_0919_path_01.csv'
    rclpy.init(args=['--ros-args','-p',f'waypoint_csv:={route}', '-p',f'route_origin_csv:={origin}'], domain_id=197)
    node = WaypointAvoidNode()
    node.destroy_timer(node._timers[-1])
    messages=[]
    node.pub=NS(publish=messages.append)
    node._test_messages=messages
    def supply(station=10., cloud=None, active=True, zone=True):
        stamp=node.get_clock().now().to_msg()
        gps=GpsPath()
        gps.header.stamp=gps.reference_stamp=stamp
        gps.fix_quality=4;gps.heading_source=GpsPath.HEADING_FUSED
        gps.position_valid=gps.vehicle_heading_valid=True;gps.avoid_zone=zone
        gps.route.enabled=True;gps.route.route_id='04';gps.route.index=2
        gps.route.sequence_id=gps.route.instance_id=1;gps.route.waypoint_csv=str(route)
        node.on_gps(gps)
        mgm=MgmState();mgm.header.stamp=stamp;mgm.go_authorized=True;mgm.top=1
        mgm.avoidance=1 if active else 0
        node.on_mgm(mgm)
        x,y,yaw,_=node.route.at_station(station)
        transform=TransformStamped();transform.header.stamp=stamp
        transform.transform.translation.x=x;transform.transform.translation.y=y
        transform.transform.rotation.z=math.sin(yaw/2);transform.transform.rotation.w=math.cos(yaw/2)
        node.tf_buffer=NS(lookup_transform=lambda *a:transform)
        points=cloud if cloud is not None else [(2.,-1.8,0.)]
        node.on_cloud(point_cloud2.create_cloud_xyz32(Header(stamp=stamp,frame_id='base_link'),points))
        node.tick()
        return messages[-1]
    node._supply=supply
    try:yield node
    finally:node.destroy_node();rclpy.shutdown()


def test_preview_is_one_metre_and_keeps_acquisition_stamp(adapter):
    msg=adapter._supply()
    assert msg.avoidable and msg.scan_valid and len(msg.points)==1
    assert math.hypot(msg.points[0].x,msg.points[0].y)==pytest.approx(1.,abs=1e-5)
    assert msg.reference_stamp==adapter.cloud_stamp
    adapter.tick()
    assert adapter._test_messages[-1].reference_stamp==msg.reference_stamp


def test_no_upper_authority_or_float_withholds_reference(adapter):
    assert not adapter._supply(active=False).points
    assert adapter._supply().points
    adapter.gps.fix_quality=5
    adapter.tick()
    assert not adapter._test_messages[-1].points
    assert adapter._test_messages[-1].v_suggest==0.


def test_invalid_geometry_never_reaches_mgm_as_a_drivable_point(adapter):
    adapter._supply()
    # An invalid fixed curve remains visible diagnostically, but never provides
    # a control point (revised MGM deliberately does not use legacy TTC stops).
    adapter.planner.samples=[(0,0,0,2.,0)]
    adapter.planner.advance=lambda pose:False
    adapter.planner.preview=lambda pose:(pose[0]+math.cos(pose[2]),pose[1]+math.sin(pose[2]),pose[2],2.)
    adapter.planner.path_blocked=lambda *a:False
    adapter.publish_geometry=lambda:None
    adapter.report={'valid':False,'reason':'fixed cubic exceeds configured steering curvature'}
    adapter.tick()
    msg=adapter._test_messages[-1]
    assert not msg.points and not msg.avoidable and msg.v_suggest==0.


def test_route_changes_reset_planner_and_preserve_session_origin(adapter):
    adapter._supply()
    lat0,lon0=load_waypoints_csv(ROOT/'halla_0919_path_01.csv')[0]
    lat,lon=load_waypoints_csv(ROOT/'halla_0919_path_04.csv')[0]
    assert adapter.route.e[0]==pytest.approx((lon-lon0)*M_PER_DEG_LAT*math.cos(math.radians(lat0)))
    assert adapter.route.n[0]==pytest.approx((lat-lat0)*M_PER_DEG_LAT)
    gps=adapter.gps;gps.route.waypoint_csv=str(ROOT/'halla_0919_path_05.csv')
    gps.route.index=3;gps.reference_stamp=adapter.get_clock().now().to_msg()
    adapter.on_gps(gps)
    assert not adapter.session.active and not adapter.planner.samples
    assert adapter.cloud_stamp is None
    assert (adapter.route._lat0,adapter.route._lon0)==(lat0,lon0)


def test_zone_exit_discards_unfinished_path_and_reentry_rearms(adapter):
    assert adapter._supply().obstacle_detected
    old_planner, old_detector = adapter.planner, adapter.detector
    adapter.planner.samples=[(0,0,0,2.,0)]
    msg=adapter._supply(zone=False)  # MGM can still carry previous AVOID authority
    assert not msg.obstacle_detected and not adapter.session.active
    assert not msg.points and not adapter.planner.samples
    assert adapter.planner is not old_planner and adapter.detector is not old_detector
    assert adapter._supply().obstacle_detected
    assert adapter.session.active


def test_delayed_mgm_exit_cannot_consume_new_zone_entry(adapter):
    adapter._supply()
    assert not adapter._supply(active=False).points
    assert adapter.session.active
    assert adapter._supply().avoidable


def test_invalid_gps_does_not_clear_active_geometry(adapter):
    adapter._supply()
    planner = adapter.planner
    adapter.gps.position_valid = False
    adapter.gps.avoid_zone = False
    adapter.tick()
    assert adapter.session.active and adapter.planner is planner
    assert not adapter._test_messages[-1].points


def test_finished_path_reports_done_before_alignment(adapter):
    adapter._supply()
    adapter.session.passed_path()
    adapter.planner.samples=[]
    adapter.planner.rejoined=lambda pose:False
    msg=adapter._supply()
    assert msg.maneuver_done and msg.obstacle_detected
    adapter.session.accepted()
    msg=adapter._supply()
    assert not msg.maneuver_done and msg.obstacle_detected


def test_safety_pause_preserves_producer_episode(adapter):
    adapter._supply()
    mgm=MgmState();mgm.header.stamp=adapter.get_clock().now().to_msg()
    mgm.go_authorized=True;mgm.top=1;mgm.avoidance=0;mgm.estop_active=True
    adapter.on_mgm(mgm)
    assert adapter.session.active and not adapter.session.done
