"""Read-only install/contract check. Does not initialise ROS or open hardware."""
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from fma_interfaces.msg import AvoidPlan, GpsRoute, EstopRequest, EstopRecovery, GpsPath, MgmState, ParkingCommand, ParkingStatus


def main():
    root = Path(os.environ['FMA_V2_WORKSPACE']).resolve()
    packages = (
        'adas_mgm', 'fma_interfaces', 'stack_gps', 'stack_lane', 'stack_avoid', 'stack_avoid_v2',
        'stack_estop', 'stack_parking', 'lidar_fusion_v2', 'stack_traffic',
        'bridge_dspace', 'ydlidar_ros2_driver', 'multi_lidar_fusion',
    )
    for package in packages:
        prefix = Path(get_package_prefix(package)).resolve()
        if not prefix.is_relative_to(root / 'install_v2'):
            raise RuntimeError(f'{package} resolves outside v2: {prefix}')
    contracts = (
        (AvoidPlan(), 'control_enabled'), (AvoidPlan(), 'valid_until'), (GpsRoute(), 'points'),
        (GpsPath(), 'reference_stamp'), (GpsPath(), 'waypoint_stations'), (GpsPath(), 'zones'),
        (GpsPath(), 'route'), (MgmState(), 'route'),
        (GpsPath().route, 'connecting'), (MgmState().route, 'requested_connecting'),
        (ParkingCommand(), 'request_id'), (ParkingStatus(), 'preparation_stamp'),
        (MgmState(), 'mission_failed'), (MgmState(), 'parking_search_zone_only'),
        (MgmState(), 'parking_zone_entry_active'),
        (MgmState(), 'revised_v2'), (MgmState(), 'estop_active'),
        (MgmState(), 'estop_request_id'), (MgmState(), 'traffic_zone_active'),
        (EstopRecovery(), 'request_id'), (EstopRecovery(), 'reference_stamp'),
        (MgmState(), 'sensor_alive_mask'), (MgmState(), 'lidar_ready'), (MgmState(), 'camera_available'), (MgmState(), 'start_ready'),
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
    print('Parking policy:', 'Yongin T/parallel CSV parking holds route endpoint through forward exit'
          if params['parking_zone_entry_active'] else
          'source Zone only; no time/distance limit' if params['parking_search_zone_only'] else 'legacy time/distance limits')
    print('Zone enter/exit =', params['zone_enter_confirm_samples'], params['zone_exit_confirm_samples'],
          '; prepare/drive: upper ESTOP, legacy timed reverse excluded')
    print('Revised v2: GPS drive FIXED only; four raw LiDAR start gate; turn_zones controls GPS-only + traffic')
    print('Avoidance: Waypoint_Avoid_PR103; current CSV zone [5] only; one fresh outside fix cancels path; legacy New_Avoid_v2 disabled')
    print('State v09.17; Yongin: default course=yongin_no_parking (saved CSV); course=yongin is explicit opt-in; shared traffic/GPS zone [3]; dump v45. ESTOP: CSV zone [6] station; release after 7s measured standstill; no reverse')
    if not params['parking_zone_entry_active'] and not params['parking_search_zone_only']:
        print('Legacy parking limits seconds/metres =', params['parking_search_timeout'],
              params['max_parking_search_distance'])



if __name__ == '__main__':
    main()
