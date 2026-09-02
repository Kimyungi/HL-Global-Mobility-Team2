"""Fixed, field-verified YDLiDAR driver profiles for the four positions."""


SENSOR_IDS = ('a1', 'a2', 'b1', 'b2')

DEFAULT_PORTS = {
    'a1': '/dev/lidar_front',
    'a2': '/dev/lidar_rear',
    'b1': '/dev/lidar_left',
    'b2': '/dev/lidar_right',
}


def parameters(sensor_id, port):
    """Return the field-verified ROS parameters for one unit."""
    if sensor_id not in SENSOR_IDS:
        raise ValueError(f'unknown sensor_id: {sensor_id}')

    # a2/b1/b2 report 4K samples with 8-bit intensity. Using the a1
    # 9K/16-bit profile on a2 caused checksum errors and stream failure.
    stable_4k = sensor_id != 'a1'
    return {
        'port': port,
        'frame_id': f'lidar_{sensor_id}_link',
        'baudrate': 230400,
        'lidar_type': 1,
        'device_type': 0,
        'sample_rate': 4 if stable_4k else 9,
        'abnormal_check_count': 4,
        'fixed_resolution': not stable_4k,
        'reversion': True,
        'inverted': False,
        'auto_reconnect': True,
        'isSingleChannel': False,
        'intensity': True,
        'intensity_bit': 8 if stable_4k else 16,
        'support_motor_dtr': False,
        'frequency': 10.0,
        'angle_max': 180.0,
        'angle_min': -180.0,
        'range_max': 12.0,
        'range_min': 0.03,
        'invalid_range_is_inf': not stable_4k,
        'ignore_array': '',
        'debug': False,
    }
