import math
import pytest
from stack_avoid.speed_heading_odometry import SpeedHeadingOdometry


def test_straight_distance_and_signed_reverse():
    odom = SpeedHeadingOdometry()
    odom.update(1_000_000_000, 2., 0.)
    pose, _, _ = odom.update(1_100_000_000, 2., 0.)
    assert pose == pytest.approx((.2, 0., 0.))
    odom.update(1_200_000_000, -2., 0.)
    pose, _, _ = odom.update(1_300_000_000, -2., 0.)
    assert pose == pytest.approx((0., 0., 0.))


def test_heading_frame_and_wrap():
    odom = SpeedHeadingOdometry()
    odom.update(1_000_000_000, 1., math.pi/2)
    assert odom.update(1_100_000_000, 1., math.pi/2)[0] == pytest.approx((0., .1, math.pi/2))
    odom.reset()
    odom.update(1_000_000_000, 1., math.radians(179))
    pose, _, _ = odom.update(1_100_000_000, 1., math.radians(-179))
    assert pose[:2] == pytest.approx((-.1, 0.))


def test_duplicates_invalid_samples_and_gap_do_not_invent_motion():
    odom = SpeedHeadingOdometry()
    odom.update(1_000_000_000, 1., 0.)
    assert odom.update(1_000_000_000, 1., 0.) is None
    assert odom.update(1_100_000_000, math.nan, 0.) is None
    assert odom.update(1_200_000_000, 1., 0.)[0][0] == pytest.approx(.2)
    assert odom.update(2_000_000_000, 1., 0.) is None
    assert odom.lost and odom.update(2_100_000_000, 1., 0.) is None
    odom.reset()
    assert odom.update(2_100_000_000, 1., 0.)[0] == (0., 0., 0.)


def test_field_heading_does_not_imply_diagonal_xy():
    odom = SpeedHeadingOdometry()
    odom.update(1_000_000_000, 1.9248, .3252)
    pose, _, _ = odom.update(1_480_000_000, 1.6305, .3597)
    distance = (1.9248+1.6305)*.5*.48
    assert math.hypot(*pose[:2]) == pytest.approx(distance)
    assert math.atan2(pose[1], pose[0]) == pytest.approx((.3252+.3597)/2)
    # The raw field log had dx~=dy; that input is deliberately not consumed.
    assert pose[0] > 2*pose[1]
