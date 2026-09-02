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

    # All four installed units report the stable 4K/8-bit stream.  The former
    # a1-only 9K/16-bit profile stopped publishing on the fixed-mount retest.
    return {
        'port': port,
        'frame_id': f'lidar_{sensor_id}_link',
        'baudrate': 230400,
        'lidar_type': 1,
        'device_type': 0,
        'sample_rate': 4,
        'abnormal_check_count': 4,
        'fixed_resolution': False,
        'reversion': True,
        'inverted': False,
        'auto_reconnect': True,
        'isSingleChannel': False,
        'intensity': True,
        'intensity_bit': 8,
        'support_motor_dtr': False,
        'frequency': 10.0,
        'angle_max': 180.0,
        'angle_min': -180.0,
        'range_max': 12.0,
        'range_min': 0.03,
        'invalid_range_is_inf': False,
        'ignore_array': '',
        'debug': False,
    }
