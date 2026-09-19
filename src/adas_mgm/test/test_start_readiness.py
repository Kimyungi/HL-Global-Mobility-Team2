"""Authorization requires LiDAR AND (camera frames OR GPS)."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
spec = importlib.util.spec_from_file_location('start_readiness', TOOLS / 'start_readiness.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def state(camera=False, gps=False, stamp=100., lidar=True):
    return NS(header=NS(stamp=NS(sec=int(stamp), nanosec=int((stamp-int(stamp))*1e9))),
              camera_available=camera, gps_fixed_ready=gps, lidar_ready=lidar)


@pytest.mark.parametrize('camera,gps,expected', [(True,False,True),(False,True,True),(True,True,True),(False,False,False)])
def test_either_navigation_sensor_allows_start(camera, gps, expected):
    assert module.readiness({'state': state(camera, gps)}, 100_100_000_000)[0] == expected


def test_old_ready_message_does_not_authorize_start():
    assert not module.readiness({'state': state(True,True)}, 101_000_000_000)[0]
    assert not module.readiness({}, 100_100_000_000)[0]


def test_excluding_a_source_does_not_bypass_both_missing():
    assert not module.readiness({'state': state(True,False)}, 100_100_000_000, skip_camera=True)[0]
    assert not module.readiness({'state': state(False,True)}, 100_100_000_000, skip_gps=True)[0]


def test_traffic_is_optional_but_explicit_requirement_is_honored():
    inputs = {'state': state(True)}
    assert module.readiness(inputs, 100_100_000_000)[0]
    assert not module.readiness(inputs, 100_100_000_000, require_traffic=True)[0]
    inputs['traffic'] = state()
    assert module.readiness(inputs, 100_100_000_000, require_traffic=True)[0]


@pytest.mark.parametrize('camera,gps', [(True,False),(False,True),(True,True)])
def test_no_lidar_rejects_even_with_navigation_ready(camera,gps):
    assert not module.readiness({'state': state(camera,gps,lidar=False)},100_100_000_000)[0]


def test_revised_zone_off_does_not_require_traffic_heartbeat():
    msg = state(True, True)
    msg.revised_v2 = True
    msg.traffic_zone_active = False
    assert module.readiness({'state': msg}, 100_100_000_000, require_traffic=True)[0]


@pytest.mark.parametrize('camera,gps', [(True,False),(False,True),(True,True),(False,False)])
def test_v2_requires_gps_even_with_camera(camera, gps):
    msg = state(camera, gps)
    msg.revised_v2 = True
    assert module.readiness({'state': msg}, 100_100_000_000)[0] == gps
    assert not module.readiness({'state': msg}, 100_100_000_000, skip_gps=True)[0]
