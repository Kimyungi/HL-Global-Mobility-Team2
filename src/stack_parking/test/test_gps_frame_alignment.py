"""ENU-to-parking frame regressions using actual waypoint directions."""
import csv
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from stack_parking.geometry import Pose2, compose, inverse
from stack_parking.localization import MotionPrior, MotionPriorConfig


def prior_at(pose=Pose2()):
    prior = MotionPrior(MotionPriorConfig(
        use_imu=False, gps_position_gain=1.,
        gps_max_correction_m=10., gps_innovation_gate_m=10.))
    prior.reset(pose)
    prior.predict(100.)
    return prior


def fix(prior, pose, stamp=100., update=1, quality=4, **kwargs):
    return prior.update_gps_pose(update, pose, stamp, stamp, quality, True, **kwargs)


def test_csv_southwest_motion_is_forward_in_initial_vehicle_frame():
    csv_path = (Path(__file__).resolve().parents[2] / 'stack_gps/waypoints/'
                'waypoints_halla_reference_path_01.csv')
    with csv_path.open() as f:
        rows = list(csv.DictReader(f))
    # Same projection as PathEngine, recomputed from lat/lon rather than
    # trusting a recording-session origin in the east_m/north_m columns.
    lat0, lon0 = float(rows[0]['lat']), float(rows[0]['lon'])
    east_scale = 111320. * math.cos(math.radians(lat0))
    xy = [((float(r['lon'])-lon0)*east_scale,
           (float(r['lat'])-lat0)*111320.) for r in rows[:6]]
    yaw = math.atan2(xy[1][1], xy[1][0])
    assert math.degrees(yaw) == pytest.approx(-135., abs=1.)
    assert float(rows[0]['yaw_rad']) == pytest.approx(yaw, abs=.01)
    prior = prior_at()
    fix(prior, Pose2(*xy[0], yaw))
    for i, (east, north) in enumerate(xy[1:], 1):
        fix(prior, Pose2(east, north, yaw), 100.+i*.1, i+1)
        pose = prior.predict(100.+i*.1)
        assert pose.x == pytest.approx(math.hypot(east, north), abs=.002)
        assert pose.y == pytest.approx(0., abs=.002)
        assert pose.yaw == pytest.approx(0.)


@pytest.mark.parametrize('heading', [0., math.pi/2, -math.pi/2, -3*math.pi/4])
def test_turn_and_reverse_with_nonzero_parking_origin(heading):
    local0 = Pose2(3., -2., .3)
    world0 = Pose2(100., 200., heading)
    transform = compose(local0, inverse(world0))
    prior = prior_at(local0)
    fix(prior, world0)
    # Forward, left turn, then reverse in the new body heading.
    world = world0
    for i, delta in enumerate([Pose2(.2, 0., 0.), Pose2(.1, .1, math.pi/2),
                               Pose2(-.2, 0., 0.)], 1):
        world = compose(world, delta)
        fix(prior, world, 100.+i*.1, i+1)
        pose = prior.predict(100.+i*.1)
        expected = compose(transform, world)
        np.testing.assert_allclose((pose.x, pose.y, pose.yaw),
                                   (expected.x, expected.y, expected.yaw), atol=1e-12)


def test_dropped_fixes_and_quality_loss_preserve_alignment_and_correct_drift():
    prior = prior_at()
    fix(prior, Pose2(10., 20., math.pi/2))
    alignment = prior.gps_map_transform
    assert not fix(prior, Pose2(10., 21., math.pi/2), 100.1, 2, quality=5)
    prior.pose = Pose2(.7, .1, 0.)  # vehicle odometry accumulated during outage
    fix(prior, Pose2(10., 21., math.pi/2), 100.2, 5)
    pose = prior.predict(100.2)
    assert prior.gps_map_transform == alignment
    np.testing.assert_allclose((pose.x, pose.y), (1., 0.), atol=1e-12)


def test_duplicate_and_old_fix_cannot_reapply_correction():
    prior = prior_at()
    fix(prior, Pose2())
    fix(prior, Pose2(.2, 0., 0.), 100.1, 2)
    prior.predict(100.1)
    assert not fix(prior, Pose2(1., 0., 0.), 100.1, 2)
    assert not fix(prior, Pose2(1., 0., 0.), 100.05, 1)
    assert prior.predict(100.2).x == pytest.approx(.2)
    assert not prior.last_status.gps_corrected


def test_new_enu_origin_reanchors_but_keeps_existing_map_pose():
    prior = prior_at(Pose2(3., -2., .4))
    fix(prior, Pose2(10., 20., .4), frame_key=('sequence', 1))
    old_pose = prior.pose
    fix(prior, Pose2(-100., 200., .4), 100.1, 2, frame_key=('sequence', 2))
    aligned = prior.predict(100.1)
    np.testing.assert_allclose((aligned.x, aligned.y, aligned.yaw),
                               (old_pose.x, old_pose.y, old_pose.yaw), atol=1e-12)
    fix(prior, Pose2(-99.8, 200., .4), 100.2, 3, frame_key=('sequence', 2))
    assert prior.predict(100.2).x == pytest.approx(3.2)
    prior.reset()
    assert prior.gps_map_transform is None


def test_outlier_is_gated_without_rotating_or_reanchoring_the_map():
    prior = prior_at()
    prior.config.gps_innovation_gate_m = 1.5
    fix(prior, Pose2())
    alignment = prior.gps_map_transform
    fix(prior, Pose2(100., 0., 0.), 100.1, 2)
    assert prior.predict(100.1) == Pose2()
    assert not prior.last_status.gps_corrected
    assert prior.gps_map_transform == alignment


def test_expired_observation_does_not_apply_position_or_yaw():
    prior = prior_at()
    fix(prior, Pose2())
    fix(prior, Pose2(.2, 0., .1), 100.1, 2)
    prior.predict(100.5)  # rejected dt; must not later use this pending position
    prior.predict(100.7)
    assert prior.pose == Pose2()
    assert not prior.last_status.gps_corrected


def test_node_uses_enu_position_not_relative_points_or_body_deltas():
    from fma_interfaces.msg import GpsPath
    from stack_parking.node import StackParkingNode
    params = {'gps.use_yaw_fallback': True, 'auto_trigger_gps_zone': False}
    n = NS(prior=prior_at(), _clock_s=lambda:100.2, _p=lambda k:params[k])
    msg = GpsPath(position_valid=True, vehicle_heading_valid=True,
                  heading_source=GpsPath.HEADING_FUSED, fix_quality=4,
                  position_x=10., position_y=20., vehicle_heading_rad=math.pi/2,
                  dx=999., dy=999., dyaw=2., update=1)
    msg.header.frame_id='base_link'  # applies to points, not position_x/y
    msg.reference_stamp.sec=100
    msg.route.enabled=True;msg.route.sequence_id=10;msg.route.index=0
    StackParkingNode._on_gps_path(n,msg)
    msg.position_y=20.2;msg.reference_stamp.nanosec=100_000_000;msg.update=4
    msg.route.index=1;msg.route.waypoint_csv='next_csv_has_a_different_origin.csv'
    StackParkingNode._on_gps_path(n,msg)
    assert n.prior.predict(100.1).x == pytest.approx(.2)
    assert n.prior.pose.y == pytest.approx(0.,abs=1e-12)


@pytest.mark.parametrize('field,value', [('heading_source',0),('vehicle_heading_valid',False),
    ('position_valid',False),('fix_quality',5),('position_x',math.nan)])
def test_node_rejects_untrusted_pose(field,value):
    from fma_interfaces.msg import GpsPath
    from stack_parking.node import StackParkingNode
    params = {'gps.use_yaw_fallback': True, 'auto_trigger_gps_zone': False}
    n = NS(prior=prior_at(), _clock_s=lambda:100.1, _p=lambda k:params[k])
    msg = GpsPath(position_valid=True,vehicle_heading_valid=True,
                  heading_source=GpsPath.HEADING_FUSED,fix_quality=4)
    msg.reference_stamp.sec=100
    setattr(msg,field,value)
    StackParkingNode._on_gps_path(n,msg)
    assert n.prior.gps_map_transform is None


@pytest.mark.parametrize('stamp', [0., 99., 101.])
def test_missing_stale_and_future_stamps_are_rejected(stamp):
    prior = prior_at()
    assert not prior.update_gps_pose(1, Pose2(), stamp, 100., 4, True)
    assert prior.gps_map_transform is None
