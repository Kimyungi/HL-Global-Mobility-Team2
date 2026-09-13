import math
import numpy as np

from stack_parking.geometry import Pose2, points_in_frame
from stack_parking.lateral_wall import LateralWallAcquisition, LateralWallConfig


def tracker():
    return LateralWallAcquisition(LateralWallConfig(.215329,.211549))


def wall_scan(t, pose=Pose2(), wall_y=1.5):
    # Intersect the 11 vehicle-left rays with a fixed horizontal map wall.
    cfg=t.config; result=[]
    c,s=math.cos(pose.yaw),math.sin(pose.yaw)
    origin=np.array([pose.x+c*cfg.origin_x-s*cfg.origin_y,
                     pose.y+s*cfg.origin_x+c*cfg.origin_y])
    for offset in range(-5,6):
        angle=pose.yaw+math.radians(90+offset)
        direction=np.array([math.cos(angle),math.sin(angle)])
        result.append(origin+direction*((wall_y-origin[1])/direction[1]))
    return points_in_frame(np.asarray(result),pose)


def test_mount_origin_vehicle_normal_and_eleven_points():
    t=tracker(); expected=wall_scan(t)
    clutter=np.array([[2.,-2.],[-2.,-1.],[10.,1.5],[0.,-1.]])
    selected,centre=t.select(np.vstack((expected,clutter)))
    assert centre and len(selected)==11
    assert np.allclose(selected,expected)
    assert np.isclose(selected[5,0],t.config.origin_x)


def test_fifth_unique_frame_locks_and_republication_does_not_count():
    t=tracker(); pts=wall_scan(t)
    for i in range(4):
        stamp=10+i*.1
        assert t.update(pts,Pose2(),stamp,stamp) is None
        for _ in range(10): t.update(pts,Pose2(),stamp,stamp)
        assert t.count==i+1
    wall=t.update(pts,Pose2(),10.4,10.4)
    assert wall is not None and t.count==5
    assert abs(wall.anchor_y-1.5)<1e-9
    assert abs(wall.distance_from_seed_m-(1.5-t.config.origin_y))<1e-9
    assert np.allclose(wall.to_map(np.array([[0,-.12],[0,.12]]))[:,1],[1.38,1.62])


def test_missing_central_return_breaks_confirmation():
    t=tracker(); pts=wall_scan(t)
    t.update(pts,Pose2(),10,10)
    t.update(np.delete(pts,5,axis=0),Pose2(),10.1,10.1)
    assert t.count==0 and t.reason=='NO_CENTRE_RETURN'


def test_motion_compensation_preserves_map_wall():
    t=tracker()
    for i in range(5):
        pose=Pose2(.03*i,.01*i,.01*i)
        t.update(wall_scan(t,pose),pose,10+i*.1,10+i*.1)
    assert t.wall is not None
    assert abs(t.wall.anchor_y-1.5)<1e-9
    assert abs(t.wall.tangent_y)<1e-9


def test_farther_wall_does_not_replace_occluded_centre():
    t=tracker(); pts=wall_scan(t)
    occluder=np.array([[t.config.origin_x,t.config.origin_y+.1]])
    _,centre=t.select(np.vstack((pts,occluder)))
    assert not centre


def test_inconsistent_frames_start_new_run():
    t=tracker()
    for i in range(4): t.update(wall_scan(t),Pose2(),10+i*.1,10+i*.1)
    t.update(wall_scan(t,wall_y=2.4),Pose2(),10.4,10.4)
    assert t.wall is None and t.count==1


def test_invalid_clock_and_gap_cannot_complete_five_frames():
    t=tracker(); pts=wall_scan(t)
    t.update(pts,Pose2(),10,10)
    t.update(pts,Pose2(),11,10.1)
    assert t.count==0
    t.update(pts,Pose2(),10.2,10.2)
    t.update(pts,Pose2(),10.1,10.3)
    assert t.count==0
    t.update(pts,Pose2(),11,11)
    assert t.count==1


def test_locked_reference_is_fixed_until_reset():
    t=tracker()
    for i in range(5): t.update(wall_scan(t),Pose2(),10+i*.1,10+i*.1)
    wall=t.wall
    t.update(wall_scan(t,wall_y=2.),Pose2(),10.5,10.5)
    assert t.wall is wall
    t.reset()
    assert t.wall is None and t.count==0
