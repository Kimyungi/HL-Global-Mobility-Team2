"""Reverse references use a rear-shifted origin; forward tracking does not."""
import math
import numpy as np
import pytest

from stack_parking.geometry import PathPoint, Pose2
from stack_parking.t_reference_parking import Candidate, Config, TwoReferenceParking
from stack_parking.t_parking_sequence import TParkingSequence


@pytest.mark.parametrize('yaw', [0., math.pi/2, math.pi, -math.pi/2])
def test_reverse_origin_is_shifted_on_vehicle_x_before_local_conversion(yaw):
    station = np.arange(9) * .5
    c, s = math.cos(yaw), math.sin(yaw)
    pose = Pose2(10., 20., yaw)
    points = tuple(PathPoint(10.-d*c-.4*s, 20.-d*s+.4*c, yaw+.1, .02, -1)
                   for d in station)
    candidates = (Candidate('one', points, station), Candidate('two', points, station))
    core = TwoReferenceParking(candidates, Config(preview=1.8))
    core.phase, core.selected = 'REVERSE', 0
    out = core.tick(1., pose, 1., -.5, 1., None, None, parking_owned=True)
    assert out.v_suggest < 0
    # The selected point is 2 m behind the measured vehicle, but 1.5 m
    # behind the virtual vehicle origin. Lateral position/heading are unchanged.
    assert out.reference.x == pytest.approx(-1.5)
    assert out.reference.y == pytest.approx(.4)
    assert out.reference.yaw == pytest.approx(.1)
    assert out.reference.curvature == .02
    assert out.reference.gear == -1
    assert core.index == 0


@pytest.mark.parametrize('yaw', [0., math.pi/2])
def test_forward_reference_keeps_measured_vehicle_origin(yaw):
    station = np.arange(9) * .5
    points = tuple(PathPoint(10.+d*math.cos(yaw), 20.+d*math.sin(yaw), yaw, 0., 1)
                   for d in station)
    core = TParkingSequence((), points, station, Config(preview=1.8))
    arrived, ref = core._track(points, station, Pose2(10., 20., yaw))
    assert not arrived
    assert ref.x == pytest.approx(2.)
    assert ref.y == pytest.approx(0.)
