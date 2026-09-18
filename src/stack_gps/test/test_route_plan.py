from pathlib import Path
import yaml
import pytest
from stack_gps.route_plan import RoutePlan
from stack_gps.path_engine import PathEngine
from stack_gps.zones import ZoneMap

WAYPOINTS = Path(__file__).parents[1] / 'waypoints'


@pytest.mark.parametrize('start', ['01', '02'])
@pytest.mark.parametrize('end', ['06', '07'])
def test_selected_five_routes_meet_at_uploaded_endpoints(start, end):
    plan = RoutePlan(WAYPOINTS / 'halla_route_sequence.yaml', start, end)
    assert [r.id for r in plan.files] == [start, '03', '04', '05', '06', '07']
    gaps = [(a.id,b.id) for a,b in zip(plan.files,plan.files[1:]) if (a.id, b.id) != ('06', '07') and a.points[-1] != b.points[0]]
    assert gaps == []  # 2026-09-16 map: route 4 is trimmed at the route-5 junction
    assert all(r.completion == 0 and not r.entry_connection for r in plan.files)
    plan.bind(lambda files: (PathEngine(files.points), ZoneMap([], len(files.points))))
    assert plan.connections == [None]*6


@pytest.mark.parametrize('start,end', [('', ''), ('01', ''), ('', '07'), ('03','07'), ('01','05'), ('1','7')])
def test_branch_choices_are_required_and_never_guessed(start, end):
    with pytest.raises(ValueError, match='Explicit route_start_id'):
        RoutePlan(WAYPOINTS / 'halla_route_sequence.yaml', start, end)


def test_each_selected_branch_has_a_distinct_identity():
    identities = {RoutePlan(WAYPOINTS / 'halla_route_sequence.yaml', s, e).sequence_id
                  for s in ('01','02') for e in ('06','07')}
    assert len(identities) == 2


def copy_manifest(tmp_path):
    data = yaml.safe_load((WAYPOINTS / 'halla_route_sequence.yaml').read_text())
    data.pop('sequence')  # generic fixed manifests remain supported
    for r in data['routes']:
        r['file'] = str((WAYPOINTS / r['file']).resolve())
        r['zones_file'] = str((WAYPOINTS / r['zones_file']).resolve())
        if r['id'] == '05':
            zone = yaml.safe_load(Path(r['zones_file']).read_text())
            zone['zones'] = []  # This fixture tests generic fixed manifests.
            zone_path = tmp_path / 'fixed_05.yaml'
            zone_path.write_text(yaml.safe_dump(zone))
            r['zones_file'] = str(zone_path)
    path = tmp_path/'plan.yaml'
    return path, data


@pytest.mark.parametrize('failure', ['missing', 'id', 'track', 'mode', 'condition', 'empty'])
def test_invalid_manifest_fails_before_any_engine_or_hardware(tmp_path, failure):
    path, data = copy_manifest(tmp_path)
    if failure == 'missing': del data['routes'][0]['zones_file']
    elif failure == 'id': data['routes'][1]['id'] = data['routes'][0]['id']
    elif failure == 'condition': data['routes'][0]['completion'] = 'skip_on_cancel'
    elif failure == 'empty': data['routes'] = []
    else:
        zone = tmp_path/'zones.yaml'
        fields = {'track': Path(data['routes'][0]['file']).name}
        if failure == 'track': fields['track'] = 'wrong.csv'
        else: fields['parking_points'] = [{'lat':37., 'lon':127., 'mode':'unsupported'}]
        zone.write_text(yaml.safe_dump(fields))
        data['routes'][0]['zones_file'] = str(zone)
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError): RoutePlan(path)


def test_global_ids_idempotent_commands_and_common_position(tmp_path):
    path, data = copy_manifest(tmp_path)
    path.write_text(yaml.safe_dump(data))
    plan = RoutePlan(path)
    def factory(files):
        # Test-only same local Mission/Zone/stop IDs on every route.
        engine = PathEngine(files.points, parking_ranges=[(2,4)], stop_ranges=[(6,8)])
        return engine, ZoneMap.from_engine(engine)
    plan.bind(factory)
    assert plan.required == [(i,) for i in range(7)]
    assert [m.definitions[0].zone_id for m in plan.zone_maps] == list(range(1,8))
    assert plan.stop_offsets == list(range(7))
    position = plan.position(*plan.files[0].points[-1])
    assert not plan.apply(plan.sequence_id, 99, 22, 1, 1)
    assert not plan.apply(plan.sequence_id, 22, 22, 1, 2)
    assert plan.apply(plan.sequence_id, 22, 22, 1, 1)
    assert position == plan.position(*plan.files[0].points[-1])  # common frame survives switch
    assert not plan.apply(plan.sequence_id, 22, 22, 1, 1)
    assert plan.apply(plan.sequence_id, 22, 22, 2, 0)
    assert not plan.apply(plan.sequence_id, 22, 22, 1, 1)


def test_configuration_hash_tracks_file_content(tmp_path):
    path, data = copy_manifest(tmp_path)
    path.write_text(yaml.safe_dump(data))
    first = RoutePlan(path).sequence_id
    data['routes'][0]['id'] = 'different-label'
    path.write_text(yaml.safe_dump(data))
    assert RoutePlan(path).sequence_id != first


def test_mission_only_condition_requires_a_real_mission(tmp_path):
    path, data = copy_manifest(tmp_path)
    data['routes'] = data['routes'][:1]
    data['routes'][0]['completion'] = 'missions_complete'
    path.write_text(yaml.safe_dump(data))
    plan = RoutePlan(path)
    assert plan.files[0].completion == 1
    def empty(files):
        engine = PathEngine(files.points)
        return engine, ZoneMap.from_engine(engine)
    with pytest.raises(ValueError, match='at least one Mission'):
        plan.bind(empty)
    plan = RoutePlan(path)
    def mission(files):
        engine = PathEngine(files.points, parking_ranges=[(2,4)])
        return engine, ZoneMap.from_engine(engine)
    plan.bind(mission)
    assert plan.required == [(0,)]


@pytest.mark.parametrize('index', [1,6])
def test_connection_reaches_next_csv_start_without_global_nearest_alias(tmp_path, index):
    path, data = copy_manifest(tmp_path)
    data['routes'][index]['entry_connection'] = 'straight'
    path.write_text(yaml.safe_dump(data))
    plan = RoutePlan(path)
    def factory(files):
        engine = PathEngine(files.points, parking_ranges=[(2,4)])
        return engine, ZoneMap.from_engine(engine)
    plan.bind(factory)
    connector = plan.connections[index]
    points = plan.connection_points[index]
    assert points[0] == plan.files[index-1].points[-1]
    assert points[-1] == plan.files[index].points[0]
    assert connector.snapshot(*points[0], heading=connector.yaw[0])['idx'] == 0
    assert not connector.snapshot(*points[0], heading=connector.yaw[0])['at_end']
    assert connector.snapshot(*points[-1], heading=connector.yaw[-1])['at_end']
    # At the connector's normal arrival cell, the next CSV selects its start,
    # not the terminal cell shared with the preceding CSV.
    destination = plan.engines[index]
    assert not destination.snapshot(*points[-2], heading=destination.yaw[0])['at_end']
    assert destination.snapshot(*points[-1], heading=destination.yaw[0])['idx'] == 0
    assert all(not z for z in (connector.parking_ranges, connector.gps_only_ranges, connector.stop_ranges))
    plan.index = index-1
    assert plan.next_connecting
    assert not plan.apply(plan.sequence_id,22,22,10,index,False)  # cannot bypass entry
    assert plan.apply(plan.sequence_id,22,22,10,index,True)
    assert plan.connecting and not plan.active_zones.definitions
    assert not plan.apply(plan.sequence_id,22,22,11,index+1,False)  # cannot skip destination CSV
    assert not plan.apply(plan.sequence_id,22,22,10,index,False)  # duplicate ID cannot change stage
    assert plan.apply(plan.sequence_id,22,22,11,index,False)
    assert plan.active_engine is destination and not plan.connecting
    assert plan.required[index]  # destination Missions survived the connector


@pytest.mark.parametrize('value', ['straight', 'guess'])
def test_invalid_first_connection_is_rejected(tmp_path, value):
    path, data = copy_manifest(tmp_path)
    data['routes'][0]['entry_connection'] = value
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError): RoutePlan(path)


def test_coincident_endpoints_do_not_need_an_extra_connection(tmp_path):
    path,data = copy_manifest(tmp_path)
    data['routes'] = data['routes'][1:3]
    data['routes'][0]['entry_connection'] = 'none'
    data['routes'][1]['entry_connection'] = 'straight'
    path.write_text(yaml.safe_dump(data))
    plan = RoutePlan(path)
    def factory(files):
        engine = PathEngine(files.points)
        return engine, ZoneMap.from_engine(engine)
    plan.bind(factory)
    assert plan.connections == [None,None]
    assert not plan.next_connecting
    assert plan.apply(plan.sequence_id,22,22,1,1)


@pytest.mark.parametrize('bad', ['unknown', 'duplicate', 'repeat', 'empty'])
def test_invalid_selection_schema_is_rejected(tmp_path, bad):
    path, data = copy_manifest(tmp_path)
    data['sequence'] = {'start':['01','02'], 'via':['03','04','05'], 'end':['06','07']}
    if bad == 'unknown': data['sequence']['via'] = ['08']
    elif bad == 'duplicate': data['sequence']['start'] = ['01','01']
    elif bad == 'repeat': data['sequence']['via'] = ['01']
    else: data['sequence']['end'] = []
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError): RoutePlan(path, '01', '07')
