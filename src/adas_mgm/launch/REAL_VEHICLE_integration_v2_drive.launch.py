"""Halla full-stack + vehicle RTCM relay + unified RViz; explicit go stays separate."""
import importlib.util
import os
import stat
from datetime import datetime
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def selected_manifest(catalog_path, start, end):
    """Resolve original CSV/Zone pairs and keep the requested suffix of the course."""
    if start not in ('01', '02', '03', '04', '05', '06', '07'):
        raise ValueError('start_waypoint must be 01..07')
    if end not in ('06', '07') or (start in ('06', '07') and start != end):
        raise ValueError('end_waypoint must be 06/07 and agree with a terminal start')
    from stack_gps.route_plan import RoutePlan
    plan = RoutePlan(catalog_path, start if start in ('01', '02') else '01', end)
    ids = [route.id for route in plan.files]
    chosen = plan.files[ids.index(start):]
    return {'routes': [dict(id=route.id, file=str(route.csv),
                            zones_file=str(route.zones),
                            completion=('missions_complete' if route.completion
                                        else 'endpoint_and_missions'))
                       for route in chosen]}


def check_lidar_devices():
    """Catch missing USB links before starting CAN; scan readiness is checked in MGM."""
    missing = []
    for side in ('front', 'rear', 'left', 'right'):
        path = Path('/dev/lidar_' + side)
        if not (path.exists() and stat.S_ISCHR(path.stat().st_mode)
                and os.access(path, os.R_OK | os.W_OK)):
            missing.append(str(path))
    if missing:
        raise RuntimeError('라이다 장치/접근 권한 확인 실패: ' + ', '.join(missing)
                           + '. scripts/v2_recover_lidars.py --apply로 식별/링크를 복구하세요.')


def start_stack(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    if value('REAL_VEHICLE_CONFIRM') != 'I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX':
        raise RuntimeError('REAL_VEHICLE_CONFIRM token required before starting hardware')
    # No legacy fallback: reject retired switches before opening any hardware.
    if value('waypoint_avoid') != 'true' or value('avoid_v2_enabled') != 'false':
        raise RuntimeError('Legacy avoidance excluded: waypoint_avoid=true and avoid_v2_enabled=false required')
    for retired in ('avoid_planner_mode', 'avoid_compute_backend'):
        if context.launch_configurations.get(retired, ''):
            raise RuntimeError(f'Legacy avoidance option excluded: {retired}')
    if value('parking_enabled') != 'true' or value('avoid_zone_only') != 'true':
        raise RuntimeError('PR103 requires four-LiDAR bringup and the agreed zone entry policy')
    mode, backend = 'Waypoint_Avoid_PR103', 'stack_avoid.waypoint_planner'
    root = Path(os.environ['FMA_V2_WORKSPACE']).resolve()
    share = Path(get_package_share_directory('adas_mgm'))
    if not share.resolve().is_relative_to(root / 'install_v2'):
        raise RuntimeError('Use this workspace scripts/v2 drive and install_v2')
    manifest = selected_manifest(root / 'src/stack_gps/waypoints/halla_route_sequence.yaml',
                                 value('start_waypoint'), value('end_waypoint'))
    print('[v2 drive] selected start CSV: ' + manifest['routes'][0]['file'], flush=True)
    context.launch_configurations['t_reference_origin_csv'] = manifest['routes'][0]['file']
    context.launch_configurations['t_reference_route_csv'] = str(
        root / 'src/stack_gps/waypoints/waypoints_halla_20260916_path_03.csv')
    context.launch_configurations['avoid_waypoint_csv'] = manifest['routes'][0]['file']
    context.launch_configurations['avoid_route_origin_csv'] = manifest['routes'][0]['file']
    # This entry owns route selection; avoid ambiguous overrides from the base launch.
    for name in ('route_sequence_file', 'route_start_id', 'route_end_id',
                 'waypoint_csv', 'zones_file'):
        if context.launch_configurations.get(name, ''):
            raise RuntimeError(f'Use start_waypoint/end_waypoint instead of {name}')
    check_lidar_devices()
    # Hardware receiver ownership outlives this route/CAN/RViz launch.
    from stack_gps.persistent_service import configuration, ensure_running
    config = configuration(root / 'src/stack_gps/config/persistent_gps.yaml',
                           start_relay=value('start_rtcm') == 'true',
                           relay_device=value('rtcm_device'),
                           rtcm_host=value('rtcm_host'))
    ensure_running(config, root / 'src/stack_gps/tools/base_station/rtcm_server.py')
    context.launch_configurations['gps_link_mode'] = 'persistent'
    run = (Path(value('run_log_dir')).expanduser().resolve() if value('run_log_dir')
           else root / 'drive_logs' / datetime.now().strftime('v2_%Y%m%d_%H%M%S_%f'))
    run.mkdir(parents=True, exist_ok=False)
    route_file = run / 'route_selected.yaml'
    route_file.write_text(yaml.safe_dump(manifest, sort_keys=False))
    (run / 'avoid_compute_backend.txt').write_text(backend + '\n')
    (run / 'avoid_planner_mode.txt').write_text(mode + '\n')
    context.launch_configurations['route_sequence_file'] = str(route_file)
    spec = importlib.util.spec_from_file_location(
        'v2_full_drive_stack', share / 'launch/REAL_VEHICLE_lane_gps_can.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    stack = module.build_launch_description(
        log_dir=str(run), default_homography=str(root / 'src/stack_lane/config/homography.json'),
        default_lane_weights=str(root / 'src/stack_lane/models/yolopv2.pt'),
        lidar_estop_enabled=False, revised_v2_enabled=True,
        required_lidar_topics=['/lidar/a1/scan', '/lidar/a2/scan',
                               '/lidar/b1/scan', '/lidar/b2/scan'])
    actions = list(stack.entities)
    if value('rviz') == 'true':
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(share / 'launch/integration_v2_view.launch.py'))))
    print('[v2 drive] route: ' + ' -> '.join(r['id'] for r in manifest['routes']))
    print(f'[v2 drive] avoid planner: {mode}; backend={backend}; zone_only={value("avoid_zone_only")}')
    print(f'[v2 drive] logs: {run}; waiting for explicit go')
    return actions


def generate_launch_description():
    # Match the integrated field session; normal safety/arbitration remains in the core.
    profile = dict(
        parking_enabled='true', t_reference_enabled='true', t_parking_zone_ranges='[0]', parallel_parking_zone_ranges='[0]',
        zone_enter_confirm_samples='5', zone_exit_confirm_samples='5',
        parking_zone_entry_active='true', escape_after_cycles='0',
        avoidance_enabled='true', avoid_zone_only='true', avoid_v2_enabled='false', waypoint_avoid='true', avoid_target_speed_mps='1.0', usb_speed='high', camera_fps='10',
        lane_debug='true', traffic_show_debug='false', traffic_enabled='true',
        traffic_depth_enabled='false', traffic_yolo_image_size='640',
        traffic_yolo_inference_interval='2', traffic_red_phase_yolo_inference_interval='3',
        traffic_stopline_yolo_image_size='320', traffic_require_stop_gate='false',
        traffic_stop_y_ratio='0.0', traffic_exposure_compensation='-2', v_base='2.0', v_avoid='1.0', record='false')
    return LaunchDescription([
        DeclareLaunchArgument('REAL_VEHICLE_CONFIRM', default_value='NOT_CONFIRMED'),
        DeclareLaunchArgument('start_waypoint',
                              description='Required each session; scripts/v2 prompts when omitted',
                              choices=['01', '02', '03', '04', '05', '06', '07']),
        DeclareLaunchArgument('end_waypoint', default_value='07', choices=['06', '07']),
        DeclareLaunchArgument('run_log_dir', default_value='',
                              description='New session directory; empty uses workspace drive_logs'),
        DeclareLaunchArgument('start_rtcm', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rtcm_device', default_value='/dev/ttyRadio'),
        DeclareLaunchArgument('rtcm_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        *[DeclareLaunchArgument(k, default_value=v) for k, v in profile.items()],
        OpaqueFunction(function=start_stack),
    ])
