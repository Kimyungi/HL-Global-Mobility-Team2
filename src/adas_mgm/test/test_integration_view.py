"""Visualization frame/distance/freshness tests; ROS uses an isolated test domain."""
import importlib.util
import math
from pathlib import Path as FilePath
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from fma_interfaces.msg import GpsPath, LanePath, MgmState, RefPoint, TrafficStop

spec = importlib.util.spec_from_file_location('integration_view', FilePath(__file__).parents[1]/'tools/integration_view.py')
view = importlib.util.module_from_spec(spec)
spec.loader.exec_module(view)


def test_independent_gps_and_slam_maps_share_vehicle_origin():
    gps = view.local_xy([(100.,202.),(99.,200.)], (100.,200.,math.pi/2))
    slam = view.local_xy([(5.,6.),(6.,8.)], (5.,8.,-math.pi/2))
    np.testing.assert_allclose(gps, [(2.,0.),(0.,1.)], atol=1e-12)
    np.testing.assert_allclose(slam, gps, atol=1e-12)


def test_stopline_uses_bumper_distance_and_preserves_passed_line():
    assert view.stop_line_x(MgmState(traffic_distance_known=True,traffic_remaining_m=1.5),.76) == pytest.approx(2.26)
    assert view.stop_line_x(MgmState(traffic_distance_known=True,traffic_remaining_m=-1.),.76) == pytest.approx(-.24)
    assert view.stop_line_x(MgmState(),.76) is None
    assert view.stop_line_x(MgmState(traffic_distance_known=True,traffic_remaining_m=float('nan')),.76) is None


def test_camera_rgb_with_padded_rows():
    msg=Image(height=1,width=2,encoding='rgb8',step=8,data=bytes([255,0,0,0,255,0,0,0]))
    assert view.image_bgr(msg).tolist()==[[[0,0,255],[0,255,0]]]


@pytest.fixture
def display():
    rclpy.init(args=[])
    node=view.IntegrationView()
    captured={}
    node.markers_pub=SimpleNamespace(publish=lambda msg:captured.update(markers=msg))
    node.hud_pub=SimpleNamespace(publish=lambda msg:captured.update(hud=msg))
    node.cloud_pubs={key:SimpleNamespace(publish=lambda msg,k=key:captured.update({k:msg}))
                     for key in ('map','a1','a2','b1','b2')}
    yield node,captured
    node.destroy_node()
    rclpy.shutdown()


def test_one_view_transforms_track_and_slam_and_displays_each_preview(display):
    node,out=display
    stamp=node.get_clock().now().to_msg()
    gps=GpsPath(position_valid=True,position_x=100.,position_y=200.,vehicle_heading_rad=math.pi/2,
                points=[RefPoint(x=2.5,y=.2)],reference_stamp=stamp)
    gps.header.stamp=stamp;gps.header.frame_id='base_link'
    node.receive('gps',gps)
    track=Path();track.header.stamp=stamp;track.header.frame_id='map'
    p=PoseStamped();p.pose.position.x=100.;p.pose.position.y=202.;track.poses=[p]
    node.receive('track',track)
    lane=LanePath(points=[RefPoint(x=2.4,y=-.4)],confidence=.9,reference_stamp=stamp)
    lane.header=gps.header;node.receive('lane',lane)
    pose=PoseStamped();pose.header.stamp=stamp;pose.header.frame_id='parking_map'
    pose.pose.position.x=5.;pose.pose.position.y=8.
    pose.pose.orientation.z=math.sin(-math.pi/4);pose.pose.orientation.w=math.cos(-math.pi/4)
    node.receive('pose',pose)
    node.header=pose.header
    node.receive('map',node.cloud([(5.,6.)]))
    node.render()
    markers={m.ns:m for m in out['markers'].markers}
    assert markers['gps_route'].points[0].x == pytest.approx(100.)
    assert markers['gps_route'].points[0].y == pytest.approx(202.)
    assert markers['GPS'].pose.position.x == 2.5
    assert markers['CAMERA'].pose.position.y == pytest.approx(-.4)
    points=view.point_cloud2.read_points_numpy(out['map'],field_names=('x','y'))
    np.testing.assert_allclose(points,[(2.,0.)],atol=1e-6)
    assert out['hud'].height==1160 and out['hud'].width==720
    assert all(m.header.frame_id==('map' if m.ns=='gps_route' else view.FRAME)
               for m in out['markers'].markers)


def test_stale_inputs_are_removed_and_optical_z_is_not_a_bumper_distance(display):
    node,out=display
    old=node.get_clock().now().to_msg();old.sec-=10
    gps=GpsPath(position_valid=True,points=[RefPoint(x=2.5)],reference_stamp=old)
    gps.header.stamp=node.get_clock().now().to_msg();gps.header.frame_id='base_link'
    node.receive('gps',gps)
    traffic=TrafficStop(stop_distance=7.,red_active=True)
    traffic.header.stamp=node.get_clock().now().to_msg();node.receive('traffic',traffic)
    node.render()
    names={m.ns for m in out['markers'].markers}
    assert 'GPS' not in names and 'stop_line_dist_estimate' not in names
    assert out['map'].width==0 and out['a1'].width==0


def test_new_request_cannot_display_old_parking_map(display):
    node,out=display
    pose=PoseStamped();pose.header.stamp=node.get_clock().now().to_msg()
    pose.header.stamp.sec-=1;pose.header.frame_id='parking_map';pose.pose.orientation.w=1.
    node.receive('pose',pose)
    state=MgmState(mission_request_active=True,mission_request_id=14)
    state.header.stamp=node.get_clock().now().to_msg();state.zone_entry_observation.stamp=state.header.stamp
    node.receive('mgm',state)
    assert node.parking_since>0 and node.get('pose',age=2.) is None
    node.render()
    assert out['map'].width==0


def test_absolute_avoid_path_marker_does_not_move_with_vehicle(display):
    node,out=display
    stamp=node.get_clock().now().to_msg()
    path=Path(); path.header.stamp=stamp; path.header.frame_id='map'
    for x,y in [(100.,200.),(99.,203.),(100.,206.)]:
        p=PoseStamped(); p.pose.position.x=x; p.pose.position.y=y
        p.pose.orientation.w=1.; path.poses.append(p)
    node.receive('avoid_path',path)
    for east,north,yaw in [(100.,200.,1.57),(101.,201.,.7)]:
        gps=GpsPath(position_valid=True,position_x=east,position_y=north,
                    vehicle_heading_rad=yaw,reference_stamp=stamp)
        gps.header.stamp=stamp;node.receive('gps',gps)
        node.render()
        marker=next(m for m in out['markers'].markers if m.ns=='AVOID_PATH')
        assert marker.header.frame_id=='map'
        assert [(p.x,p.y) for p in marker.points]==[(100.,200.),(99.,203.),(100.,206.)]
