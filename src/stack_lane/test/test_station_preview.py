"""Offline geometry tests: no camera, inference model, ROS processes or CAN."""
import math

import numpy as np
import pytest

from stack_lane import lane_path
from stack_lane.bev import BevGrid
from stack_lane.lane_fit import LaneFitResult, SideFit
from stack_lane.station_preview import station_preview_x


def measured_arc(coeffs, start_x, end_x):
    # Independent numerical integration, rather than the production antiderivative.
    xs = np.linspace(start_x, end_x, 10001)
    return np.trapz(np.hypot(1.0, 2.0 * coeffs[0] * xs + coeffs[1]), xs)


@pytest.mark.parametrize('coeffs', [
    [0., 0., 0.], [0., 0., .8], [0., .4, .6],
    [.08, -.2, .3], [-.08, .2, -.3], [1e-12, .4, .6],
    [.15, -1.5, 3.], [.1, 0., -6.],
])
def test_projection_and_forward_station(coeffs):
    origin, target = station_preview_x(np.array(coeffs))
    a, b, c = coeffs
    y0 = (a*origin + b)*origin + c
    assert origin + y0 * (2*a*origin + b) == pytest.approx(0., abs=1e-8)
    # Check the chosen projection against other positions, including behind the vehicle.
    xs = np.linspace(-15., 15., 30001)
    assert math.hypot(origin, y0) <= np.hypot(xs, a*xs*xs + b*xs + c).min() + 1e-8
    assert target > origin
    assert measured_arc(coeffs, origin, target) == pytest.approx(2.5, abs=1e-8)


def test_station_is_not_fixed_x_or_radial_distance():
    coeffs = np.array([0., .4, .6])
    origin, target = station_preview_x(coeffs)
    assert origin == pytest.approx(-.4 * .6 / 1.16)
    assert target == pytest.approx(origin + 2.5 / math.sqrt(1.16))
    assert target != pytest.approx(2.5)
    assert math.hypot(target, .4 * target + .6) != pytest.approx(2.5)


@pytest.mark.parametrize('coeffs', [[math.nan, 0, 0], [0, math.inf, 0], [0, 0], [0, 0, 0, 0]])
def test_bad_fit_is_rejected(coeffs):
    with pytest.raises(ValueError):
        station_preview_x(np.array(coeffs))


def fit_result(coeffs, hit_ratio=.8):
    coeffs = np.array(coeffs, dtype=float)
    side = SideFit(coeffs, np.array([2.5, 4., 6.]), np.zeros(3), hit_ratio, .01)
    return LaneFitResult('left_only', coeffs, side, None, None)


def estimate(monkeypatch, result, **kwargs):
    monkeypatch.setattr(lane_path, 'warp_to_bev', lambda mask, *_: mask)
    monkeypatch.setattr(lane_path, 'fit_lane', lambda *_args, **_kw: result)
    return lane_path.estimate_lane_path(np.zeros((8, 8), np.uint8), np.eye(3), BevGrid(), **kwargs)


@pytest.mark.parametrize('samples', [1, 20, 30])
@pytest.mark.parametrize('hit_ratio', [.1, .8])
def test_internal_sample_count_and_confidence_never_change_one_preview(monkeypatch, samples, hit_ratio):
    coeffs = np.array([.06, -.1, .2])
    actual, debug = estimate(monkeypatch, fit_result(coeffs, hit_ratio), n_points=samples)
    point, = actual.points
    assert actual.mode == 'left_only'
    assert measured_arc(coeffs, debug['preview_projection_x'], point.x) == pytest.approx(2.5, abs=1e-8)
    assert point.y == pytest.approx(np.polyval(coeffs, point.x))
    dy = 2 * coeffs[0] * point.x + coeffs[1]
    assert point.yaw == pytest.approx(math.atan(dy))
    assert point.curvature == pytest.approx(2 * coeffs[0] / (1 + dy*dy)**1.5)
    assert (actual.x, actual.y, actual.yaw, actual.curvature) == (point.x, point.y, point.yaw, point.curvature)


def test_smoothing_precedes_station_and_derivatives(monkeypatch):
    raw = np.array([.04, .1, .2])
    old = np.array([-.03, .2, .3])
    actual, debug = estimate(monkeypatch, fit_result(raw), prev_coeffs=old, coeff_smoothing_alpha=.3)
    point, = actual.points
    smoothed = .3 * raw + .7 * old
    assert measured_arc(smoothed, debug['preview_projection_x'], point.x) == pytest.approx(2.5, abs=1e-8)
    assert point.y == pytest.approx(np.polyval(smoothed, point.x))
    dy = 2 * smoothed[0] * point.x + smoothed[1]
    assert point.yaw == pytest.approx(math.atan(dy))
    assert point.curvature == pytest.approx(2 * smoothed[0] / (1 + dy*dy)**1.5)


def test_internal_distant_geometry_still_rejects_bad_curve(monkeypatch):
    actual, _ = estimate(monkeypatch, fit_result([.2, 0., 0.]), n_points=20)
    assert actual.mode == 'none' and actual.confidence == 0.
    assert actual.reject_reason == 'implausible'  # y at x=6 exceeds the existing limit
    assert len(actual.points) == 1


def test_discontinuous_raw_fit_is_rejected_before_smoothing(monkeypatch):
    actual, _ = estimate(monkeypatch, fit_result([0., 0., 2.]), prev_y=0.,
                         prev_coeffs=np.zeros(3), coeff_smoothing_alpha=.01)
    assert actual.mode == 'none' and actual.confidence == 0.
    assert actual.reject_reason == 'discontinuous'


def test_missing_fit_is_invalid_not_a_fresh_preview(monkeypatch):
    actual, _ = estimate(monkeypatch, LaneFitResult('none', None, None, None, None))
    assert actual.mode == 'none' and actual.confidence == 0.
    assert actual.reject_reason == 'no_fit'
    assert len(actual.points) == 1
