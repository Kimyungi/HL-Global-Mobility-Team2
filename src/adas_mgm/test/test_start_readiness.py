"""Authorization uses camera frames OR GPS, never line confidence or all sensors."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
spec = importlib.util.spec_from_file_location('start_readiness', TOOLS / 'start_readiness.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def state(camera=False, gps=False, stamp=100.):
    return NS(header=NS(stamp=NS(sec=int(stamp), nanosec=int((stamp-int(stamp))*1e9))),
              camera_available=camera, gps_fixed_ready=gps)


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
