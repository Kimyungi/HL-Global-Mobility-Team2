from lidar_fusion_v2.driver_profiles import DEFAULT_PORTS
from lidar_fusion_v2.driver_profiles import SENSOR_IDS
from lidar_fusion_v2.driver_profiles import parameters


def test_four_unique_fixed_ports():
    assert tuple(DEFAULT_PORTS) == SENSOR_IDS
    assert len(set(DEFAULT_PORTS.values())) == 4


def test_all_units_share_direction_contract():
    for sensor_id in SENSOR_IDS:
        profile = parameters(sensor_id, DEFAULT_PORTS[sensor_id])
        assert profile['lidar_type'] == 1
        assert profile['reversion'] is True
        assert profile['inverted'] is False
        assert profile['frame_id'] == f'lidar_{sensor_id}_link'


def test_field_verified_stream_profiles():
    for sensor_id in SENSOR_IDS:
        stable = parameters(sensor_id, DEFAULT_PORTS[sensor_id])
        assert (stable['sample_rate'], stable['fixed_resolution'],
                stable['intensity_bit']) == (4, False, 8)
        assert stable['invalid_range_is_inf'] is False


def test_unknown_sensor_is_rejected():
    try:
        parameters('unknown', '/dev/null')
    except ValueError:
        return
    raise AssertionError('unknown sensor id must be rejected')
