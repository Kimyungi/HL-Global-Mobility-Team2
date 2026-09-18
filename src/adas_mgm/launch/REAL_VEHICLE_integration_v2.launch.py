"""Integration v2 vehicle entry; use scripts/v2 vehicle from its own workspace."""
import importlib.util
import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def build_launch_description(*, lidar_estop_enabled=True):
    workspace = os.environ.get('FMA_V2_WORKSPACE')
    if not workspace:
        raise RuntimeError('Use the integration/v2_main workspace scripts/v2 vehicle entry.')
    root = Path(workspace).resolve()
    share = Path(get_package_share_directory('adas_mgm'))
    if not share.resolve().is_relative_to(root / 'install_v2'):
        raise RuntimeError('adas_mgm must come from this workspace install_v2.')
    path = share / 'launch' / 'REAL_VEHICLE_lane_gps_can.launch.py'
    spec = importlib.util.spec_from_file_location('integration_v2_vehicle_stack', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # Reuse the full stack and existing CAN token / wait_go / shutdown guards.
    # Only defaults for workspace assets and logs differ at this entry point.
    return module.build_launch_description(
        log_dir=str(root / 'drive_logs' / datetime.now().strftime(
            ('v2_' if lidar_estop_enabled else 'v2_no_estop_') + '%Y%m%d_%H%M%S_%f')),
        default_homography=str(root / 'src/stack_lane/config/homography.json'),
        default_lane_weights=str(root / 'src/stack_lane/models/yolopv2.pt'),
        lidar_estop_enabled=lidar_estop_enabled)


def generate_launch_description():
    raise RuntimeError('Retired v2 launcher excluded; use scripts/v2 prepare and RUN_BOOK_HALLA_FINAL.md')
