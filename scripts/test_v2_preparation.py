"""Preparation launch structure; never starts GPS, cameras, CAN or ROS processes."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from stack_gps import persistent_service

ROOT = Path(__file__).resolve().parents[1]


def module():
    path = ROOT / 'src/adas_mgm/launch/REAL_VEHICLE_integration_v2_drive.launch.py'
    spec = importlib.util.spec_from_file_location('prepare_drive', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def context(mod, tmp_path):
    ctx = LaunchContext()
    ctx.launch_configurations['start_waypoint'] = '03'
    ctx.launch_configurations['v_base'] = '2.0'
    for item in mod.generate_launch_description().entities:
        if isinstance(item, DeclareLaunchArgument):
            item.execute(ctx)
    ctx.launch_configurations['run_log_dir'] = str(tmp_path / 'run')
    ctx.launch_configurations['rviz'] = 'false'
    return ctx


def test_start_csv_is_required_without_wrapper_prompt():
    ctx = LaunchContext()
    with pytest.raises(RuntimeError, match='start_waypoint'):
        for item in module().generate_launch_description().entities:
            if isinstance(item, DeclareLaunchArgument):
                item.execute(ctx)


@pytest.mark.parametrize('start', ['01', '02', '03', '04', '05', '06', '07'])
def test_each_selected_start_resolves_to_its_registered_csv(start):
    end = '06' if start == '06' else '07'
    selected = module().selected_manifest(
        ROOT / 'src/stack_gps/waypoints/halla_route_sequence.yaml', start, end)
    routes = selected['routes']
    assert routes[0]['id'] == start
    assert routes[0]['file'].endswith(f'halla_0919_path_{start}.csv')
    assert routes[-1]['id'] == end


def test_show_args_and_invalid_token_do_not_start_receiver(monkeypatch, tmp_path):
    monkeypatch.setattr(persistent_service, 'ensure_running', lambda *a, **kw: pytest.fail('hardware touched'))
    mod = module()
    ctx = context(mod, tmp_path)
    with pytest.raises(RuntimeError, match='token required'):
        mod.start_stack(ctx)
    assert not (tmp_path / 'run').exists()


def test_drive_reuses_persistent_link_and_does_not_own_rtcm_child(monkeypatch, tmp_path):
    mod = module()
    monkeypatch.setenv('FMA_V2_WORKSPACE', str(ROOT))
    # Build a minimal isolated install share for the launch import; only return actions.
    share = ROOT / 'install_v2/adas_mgm/share/adas_mgm'
    monkeypatch.setattr(mod, 'get_package_share_directory', lambda _: str(share))
    monkeypatch.setattr(mod, 'check_lidar_devices', lambda: None)
    calls = []
    monkeypatch.setattr(persistent_service, 'ensure_running', lambda cfg, script: calls.append((cfg, script)))
    ctx = context(mod, tmp_path)
    ctx.launch_configurations['REAL_VEHICLE_CONFIRM'] = 'I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX'
    actions = mod.start_stack(ctx)
    assert len(calls) == 1 and calls[0][0]['start_relay'] is True
    assert ctx.launch_configurations['gps_link_mode'] == 'persistent'
    assert not any('rtcm_server.py' in str(getattr(a, 'cmd', '')) for a in actions if isinstance(a, ExecuteProcess))


def test_stop_command_cannot_shut_down_gps_and_prepare_is_explicit():
    script = (ROOT / 'scripts/v2').read_text()
    stop = script.split('  stop)', 1)[1].split(';;', 1)[0]
    assert '/operator/stop' in stop and 'persistent_service' not in stop
    assert 'gps-off) V2_GPS_ACTION=stop' in script
    assert 'gps-start) V2_GPS_ACTION=start' in script


def test_prepare_runs_full_wait_go_launcher():
    script = (ROOT / 'scripts/v2').read_text()
    assert '  prepare|drive)' in script
    assert 'REAL_VEHICLE_integration_v2_drive.launch.py' in script


def test_missing_lidar_aborts_before_hardware(monkeypatch,tmp_path):
    mod=module()
    monkeypatch.setenv('FMA_V2_WORKSPACE',str(ROOT))
    monkeypatch.setattr(mod,'get_package_share_directory',lambda _: str(ROOT/'install_v2/adas_mgm/share/adas_mgm'))
    monkeypatch.setattr(mod.os,'access',lambda *a:False)
    monkeypatch.setattr(persistent_service,'ensure_running',lambda *a,**kw:pytest.fail('receiver touched'))
    ctx=context(mod,tmp_path)
    ctx.launch_configurations['REAL_VEHICLE_CONFIRM']='I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX'
    with pytest.raises(RuntimeError,match='라이다 장치'):
        mod.start_stack(ctx)
    assert not (tmp_path/'run').exists()






@pytest.mark.parametrize('session_speed', ['0.7','2.0'])
def test_runbook_prepare_selects_waypoint_provider_without_legacy_backend(monkeypatch, tmp_path, session_speed):
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters
    from stack_avoid import compute_backend
    mod = module()
    ctx = context(mod, tmp_path)
    ctx.launch_configurations.update(
        REAL_VEHICLE_CONFIRM='I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX',
        start_waypoint='03', end_waypoint='07', parking_enabled='true',
        parking_zone_entry_active='true', avoidance_enabled='true', avoid_zone_only='true',
        zone_enter_confirm_samples='5',
        zone_exit_confirm_samples='5', traffic_enabled='true', v_base=session_speed)
    monkeypatch.setenv('FMA_V2_WORKSPACE', str(ROOT))
    monkeypatch.setattr(mod, 'check_lidar_devices', lambda: None)
    monkeypatch.setattr(persistent_service, 'ensure_running', lambda *a: None)
    monkeypatch.setattr(compute_backend, 'compute_functions',
                        lambda *a: pytest.fail('waypoint provider loaded legacy cubic backend'))
    actions = mod.start_stack(ctx)
    for action in actions:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(ctx)
    ctx.launch_configurations['route_sequence_enabled_resolved'] = 'true'
    nodes = {}
    for action in actions:
        if isinstance(action, Node):
            nodes.setdefault(action.node_package, action)  # stack_avoid also owns can_zero.
    assert 'stack_avoid_v2' not in nodes
    assert nodes['stack_avoid'].condition.evaluate(ctx)
    waypoint = evaluate_parameters(ctx, nodes['stack_avoid']._Node__parameters)[1]
    assert waypoint['waypoint_csv'].endswith('halla_0919_path_03.csv')
    assert waypoint['route_origin_csv'] == waypoint['waypoint_csv']
    assert waypoint['target_speed_mps'] == 1.0
    mgm = evaluate_parameters(ctx, nodes['adas_mgm']._Node__parameters)[1]
    assert mgm['avoid_v2_enabled'] is False and mgm['wait_go'] is True
    assert mgm['avoid_zone_only'] is True and mgm['avoidance_enabled'] is True
    assert mgm['zone_enter_confirm_samples'] == 5
    assert mgm['v_base'] == float(session_speed) and mgm['v_avoid'] == 1.0
    assert (tmp_path / 'run' / 'avoid_planner_mode.txt').read_text().strip() == 'Waypoint_Avoid_PR103'
    assert (tmp_path / 'run' / 'avoid_compute_backend.txt').read_text().strip() == 'stack_avoid.waypoint_planner'






@pytest.mark.parametrize('retired,value', [
    ('waypoint_avoid','false'), ('avoid_v2_enabled','true'),
    ('avoid_planner_mode','fixed_goals'), ('avoid_planner_mode','main_gap'),
    ('avoid_planner_mode','main_gap_path'), ('avoid_compute_backend','native'),
    ('avoid_compute_backend','python')])
def test_retired_selection_fails_before_hardware(monkeypatch,tmp_path,retired,value):
    mod=module();ctx=context(mod,tmp_path)
    ctx.launch_configurations['REAL_VEHICLE_CONFIRM']='I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX'
    ctx.launch_configurations[retired]=value
    monkeypatch.setattr(mod,'check_lidar_devices',lambda:pytest.fail('hardware touched'))
    monkeypatch.setattr(persistent_service,'ensure_running',lambda *a:pytest.fail('GPS touched'))
    with pytest.raises(RuntimeError,match='Legacy avoidance'):
        mod.start_stack(ctx)

@pytest.mark.parametrize('start', ['01', '04', '05', '06', '07'])
def test_exit_zone_preserves_branch_contract_for_partial_start(tmp_path, start):
    import yaml
    from stack_gps.route_plan import RoutePlan
    folder = ROOT / 'src/stack_gps/waypoints'
    data = yaml.safe_load((folder / 'halla_route_sequence.yaml').read_text())
    for route in data['routes']:
        route['file'] = str(folder / route['file'])
        route['zones_file'] = str(folder / route['zones_file'])
        if route['id'] == '05':
            zone = yaml.safe_load(Path(route['zones_file']).read_text())
            zone['zones'] = [{'zone_id': 20, 'zone_type': 'LAST_MISSION_ZONE', 'index_range': [10, 20]}]
            zone_file = tmp_path / 'synthetic_exit.yaml'
            zone_file.write_text(yaml.safe_dump(zone))
            route['zones_file'] = str(zone_file)
    catalog = tmp_path / 'catalog.yaml'
    catalog.write_text(yaml.safe_dump(data))
    selected = module().selected_manifest(catalog, start, start if start in ('06','07') else '07')
    selected_file = tmp_path / 'selected.yaml'
    selected_file.write_text(yaml.safe_dump(selected))
    plan = RoutePlan(selected_file)
    assert plan.files[0].id == start
    if start in ('06','07'):
        assert len(plan.files) == 1 and plan.exit_source == -1
    else:
        assert [r.id for r in plan.files][-3:] == ['05','06','07']
        assert plan.exit_branches == {'source':'05','left':'06','right':'07'}


def test_pr116_runtime_manifest_and_reverse_candidates(monkeypatch, tmp_path):
    from types import SimpleNamespace as NS
    import math
    from stack_gps.node import StackGpsNode
    from stack_gps.path_engine import PathEngine
    from stack_gps.route_plan import RoutePlan
    from stack_parking.t_parking_sequence import load_course
    mod = module()
    ctx = context(mod, tmp_path)
    ctx.launch_configurations['REAL_VEHICLE_CONFIRM'] = 'I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX'
    monkeypatch.setenv('FMA_V2_WORKSPACE', str(ROOT))
    monkeypatch.setattr(mod, 'check_lidar_devices', lambda: None)
    monkeypatch.setattr(persistent_service, 'ensure_running', lambda *a: None)
    ctx.launch_configurations['end_waypoint'] = '03'
    mod.start_stack(ctx, route_profile='parking')
    plan = RoutePlan(tmp_path / 'run/route_selected.yaml')
    assert [r.id for r in plan.files] == ['03']
    assert plan.files[0].csv.name == 'parking_waypoint.csv'
    assert len(plan.files[0].points) == 49
    assert plan.exit_branches is None
    config = ctx.launch_configurations
    reverse = [config[f't_reference_reverse_{i}_csv'] for i in (1, 2)]
    assert [Path(p).name for p in reverse] == ['parking_waypoint_rev1.csv', 'parking_waypoint_rev2.csv']
    candidates, approach, _ = load_course(config['t_reference_origin_csv'],
                                         config['t_reference_route_csv'], reverse)
    assert len(candidates) == 2
    for c in candidates:
        assert math.hypot(c.path[0].x-approach[-1].x, c.path[0].y-approach[-1].y) < .03

    def factory(route):
        values = dict(waypoint_csv=str(route.csv), zones_file=str(route.zones),
                      stop_zone_snap_max_m=3., stop_zone_span_m=1., parking_zone_span_m=1.,
                      stop_points_latlon='', avoid_zone_latlon='', gps_only_zone_latlon='',
                      avoid_zone_lead_m=5.)
        log = NS(info=lambda msg: None, warn=lambda msg: None, error=lambda msg: pytest.fail(msg))
        node = NS(engine=PathEngine(route.points), turn_zone_policy=True, get_logger=lambda: log)
        StackGpsNode._setup_zones(node, lambda key: NS(value=values[key]))
        return node.engine, node.zone_map

    plan.bind(factory)
    assert len(plan.required[0]) == 1
    first, last = plan.engines[0].parking_ranges[0]
    assert first <= 35 <= last
    assert not plan.next_connecting
    assert not plan.apply(plan.sequence_id, 1, 1, 1, 1)


@pytest.mark.parametrize('start,end', [('01', '07'), ('03', '07'), ('04', '03')])
def test_pr116_rejects_old_route_selection(start, end):
    with pytest.raises(ValueError, match='PR #116 requires'):
        module().parking_test_manifest(ROOT, start, end)


def test_pr117_obstacle_runtime_binding(monkeypatch, tmp_path):
    from types import SimpleNamespace as NS
    from stack_gps.node import StackGpsNode
    from stack_gps.path_engine import PathEngine
    from stack_gps.route_plan import RoutePlan
    mod = module()
    ctx = LaunchContext()
    for item in mod.generate_launch_description(route_profile='obstacle').entities:
        if isinstance(item, DeclareLaunchArgument):
            item.execute(ctx)
    ctx.launch_configurations.update(REAL_VEHICLE_CONFIRM='I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX',
                                     run_log_dir=str(tmp_path/'run'), rviz='false')
    monkeypatch.setenv('FMA_V2_WORKSPACE', str(ROOT))
    monkeypatch.setattr(mod, 'check_lidar_devices', lambda: None)
    monkeypatch.setattr(persistent_service, 'ensure_running', lambda *a: None)
    mod.start_stack(ctx, route_profile='obstacle')
    plan = RoutePlan(tmp_path/'run/route_selected.yaml')
    assert [r.id for r in plan.files] == ['01']
    route = plan.files[0]
    assert route.csv.name == 'obstacle_waypoint.csv'
    assert len(route.points) == 116
    assert ctx.launch_configurations['avoid_waypoint_csv'] == str(route.csv)
    assert ctx.launch_configurations['avoid_route_origin_csv'] == str(route.csv)
    assert ctx.launch_configurations['t_reference_enabled'] == 'false'

    def factory(route):
        values = dict(waypoint_csv=str(route.csv), zones_file=str(route.zones),
                      stop_zone_snap_max_m=3., stop_zone_span_m=1., parking_zone_span_m=1.,
                      stop_points_latlon='', avoid_zone_latlon='', gps_only_zone_latlon='',
                      avoid_zone_lead_m=5.)
        log = NS(info=lambda msg: None, warn=lambda msg: None, error=lambda msg: pytest.fail(msg))
        node = NS(engine=PathEngine(route.points), turn_zone_policy=True, get_logger=lambda: log)
        StackGpsNode._setup_zones(node, lambda key: NS(value=values[key]))
        return node.engine, node.zone_map
    plan.bind(factory)
    assert plan.engines[0].avoid_ranges == [(20, 115)]
    assert not plan.engines[0].parking_ranges
    assert not plan.engines[0].parallel_parking_ranges
    assert plan.required == [()]
    assert not plan.next_connecting
    with pytest.raises(ValueError, match='PR #117'):
        mod.obstacle_test_manifest(ROOT, '03', '03')


def test_halla0919_default_runtime_and_parking_join(monkeypatch, tmp_path):
    from stack_parking.t_parking_sequence import load_course
    import math
    mod = module()
    ctx = context(mod, tmp_path)
    ctx.launch_configurations.update(REAL_VEHICLE_CONFIRM='I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX',
                                     start_waypoint='01', end_waypoint='07')
    monkeypatch.setenv('FMA_V2_WORKSPACE', str(ROOT))
    monkeypatch.setattr(mod, 'check_lidar_devices', lambda: None)
    monkeypatch.setattr(persistent_service, 'ensure_running', lambda *a: None)
    mod.start_stack(ctx)
    import yaml
    data = yaml.safe_load((tmp_path/'run/route_selected.yaml').read_text())
    assert [r['id'] for r in data['routes']] == ['01','03','04','05','07']
    assert all(Path(r['file']).name == f"halla_0919_path_{r['id']}.csv" for r in data['routes'])
    cfg = ctx.launch_configurations
    assert cfg['avoid_route_origin_csv'] == data['routes'][0]['file']
    candidates, approach, _ = load_course(cfg['t_reference_origin_csv'],cfg['t_reference_route_csv'],
        [cfg[f't_reference_reverse_{i}_csv'] for i in (1,2)])
    assert all(.07 < math.hypot(c.path[0].x-approach[-1].x,c.path[0].y-approach[-1].y) < .09 for c in candidates)


def test_direct_halla_launch_requires_explicit_session_speed():
    ctx=LaunchContext()
    ctx.launch_configurations['start_waypoint']='01'
    with pytest.raises(RuntimeError, match='v_base'):
        for item in module().generate_launch_description().entities:
            if isinstance(item,DeclareLaunchArgument): item.execute(ctx)


@pytest.mark.parametrize('speed', ['0','-1','nan','inf'])
def test_invalid_speed_rejected_before_hardware(monkeypatch,tmp_path,speed):
    mod=module();ctx=context(mod,tmp_path)
    ctx.launch_configurations.update(REAL_VEHICLE_CONFIRM='I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX',v_base=speed)
    monkeypatch.setattr(mod,'check_lidar_devices',lambda:pytest.fail('hardware checked before speed validation'))
    with pytest.raises(ValueError,match='v_base'):mod.start_stack(ctx)
