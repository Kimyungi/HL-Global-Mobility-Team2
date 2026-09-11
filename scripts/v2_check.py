"""Read-only install/contract check. Does not initialise ROS or open hardware."""
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from fma_interfaces.msg import EstopRequest, GpsPath, MgmState, ParkingCommand, ParkingStatus


def main():
    root = Path(os.environ['FMA_V2_WORKSPACE']).resolve()
    packages = (
        'adas_mgm', 'fma_interfaces', 'stack_gps', 'stack_lane', 'stack_avoid',
        'stack_estop', 'stack_parking', 'lidar_fusion_v2', 'stack_traffic',
        'bridge_dspace', 'ydlidar_ros2_driver', 'multi_lidar_fusion',
    )
    for package in packages:
        prefix = Path(get_package_prefix(package)).resolve()
        if not prefix.is_relative_to(root / 'install_v2'):
            raise RuntimeError(f'{package} resolves outside v2: {prefix}')
    contracts = (
        (GpsPath(), 'reference_stamp'), (GpsPath(), 'zones'),
        (ParkingCommand(), 'request_id'), (ParkingStatus(), 'preparation_stamp'),
        (EstopRequest(), 'rear_corridor_state'), (MgmState(), 'parking_calibration_state'),
    )
    for message, field in contracts:
        if not hasattr(message, field):
            raise RuntimeError(f'Old interface loaded: {type(message).__name__}.{field}')
    params = yaml.safe_load((Path(get_package_share_directory('adas_mgm')) /
                             'config/params.yaml').read_text())['mgm_node']['ros__parameters']
    if params.get('backend') != 'core' or not params.get('base_state_machine_enabled', True):
        raise RuntimeError('v2 requires backend=core and base_state_machine_enabled=true')
    print(f'V2_INSTALL_READY: {len(packages)} package prefixes and v2 message contract; {root}')
    print('Calibration: zone enter/exit =', params['zone_enter_confirm_samples'],
          params['zone_exit_confirm_samples'], '; parking seconds/metres =',
          params['parking_search_timeout'], params['max_parking_search_distance'],
          '; recovery delay =', params['escape_after_cycles'])


if __name__ == '__main__':
    main()
