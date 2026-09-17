from pathlib import Path
import yaml
import pytest

from stack_gps.path_engine import PathEngine
from stack_gps.route_plan import RoutePlan
from stack_gps.zones import ZoneMap, ZoneType, load_zone_definitions, turn_zone_map

WAYPOINTS = Path(__file__).parents[1] / 'waypoints'


def manifest(tmp_path, *, zone=True):
    data = yaml.safe_load((WAYPOINTS / 'halla_route_sequence.yaml').read_text())
    for route in data['routes']:
        route['file'] = str(WAYPOINTS / route['file'])
        route['zones_file'] = str(WAYPOINTS / route['zones_file'])
        if route['id'] == '05' and zone:
            definitions = yaml.safe_load(Path(route['zones_file']).read_text())
            # Synthetic test-only zone; the checked-in Halla map stays unchanged.
            definitions['zones'] = [{'zone_id': 20, 'zone_type': 'LAST_MISSION_ZONE', 'index_range': [10, 20]}]
            zone_path = tmp_path / 'exit_zone.yaml'
            zone_path.write_text(yaml.safe_dump(definitions))
            route['zones_file'] = str(zone_path)
    path = tmp_path / 'plan.yaml'
    path.write_text(yaml.safe_dump(data))
    return path, data


def bind(plan):
    def factory(files):
        engine = PathEngine(files.points)
        zones = ZoneMap.from_engine(engine, load_zone_definitions(files.zones, engine, 3.))
        zones = turn_zone_map(files.zones, engine, 3., zones)
        return engine, zones
    plan.bind(factory)


@pytest.mark.parametrize('end', ['06', '07'])
def test_zone_preloads_both_branches_and_ids_are_stable(tmp_path, end):
    path, _ = manifest(tmp_path)
    plan = RoutePlan(path, '01', end)
    assert [f.id for f in plan.files] == ['01', '03', '04', '05', '06', '07']
    assert (plan.exit_source, plan.exit_left, plan.exit_right) == (3, 4, 5)
    assert plan.sequence_id == RoutePlan(path, '01', '07' if end == '06' else '06').sequence_id
    bind(plan)
    zone_map = plan.zone_maps[3]
    assert any(z.zone_type == ZoneType.LAST_MISSION_ZONE and inside
               for z, inside in zone_map.snapshot(10))
    assert not any(z.zone_type == ZoneType.LAST_MISSION_ZONE and inside
                   for z, inside in zone_map.snapshot(9, 10))


@pytest.mark.parametrize('selected', [4, 5])
def test_acknowledged_branch_is_terminal_and_other_exit_cannot_follow(tmp_path, selected):
    path, _ = manifest(tmp_path)
    plan = RoutePlan(path, '01', '07')
    bind(plan)
    for index in range(1, 4):
        assert plan.apply(plan.sequence_id, 11, 11, index, index)
    assert not plan.apply(plan.sequence_id + 1, 11, 11, 4, selected)
    assert not plan.apply(plan.sequence_id, 12, 11, 4, selected)
    assert plan.apply(plan.sequence_id, 11, 11, 4, selected)
    assert not plan.apply(plan.sequence_id, 11, 11, 4, selected)
    assert not plan.apply(plan.sequence_id, 11, 11, 5, 5 if selected == 4 else 4)
    assert not plan.next_connecting
    assert plan.apply(plan.sequence_id, 11, 11, 6, 0)


def test_missing_zone_keeps_fixed_07_and_rejects_unbacked_branch_contract(tmp_path):
    path, data = manifest(tmp_path, zone=False)
    plan = RoutePlan(path, '01', '07')
    assert [f.id for f in plan.files][-1] == '07' and plan.exit_source == -1
    data.pop('sequence')
    data['routes'] = [r for r in data['routes'] if r['id'] in ('05', '06', '07')]
    data['exit_branches'] = {'source': '05', 'left': '06', 'right': '07'}
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='LAST_MISSION_ZONE'):
        RoutePlan(path)


def test_fixed_manifest_preserves_explicit_branch_contract(tmp_path):
    path, data = manifest(tmp_path)
    data.pop('sequence')
    data['routes'] = [r for r in data['routes'] if r['id'] in ('05', '06', '07')]
    data['exit_branches'] = {'source': '05', 'left': '06', 'right': '07'}
    path.write_text(yaml.safe_dump(data))
    plan = RoutePlan(path)
    assert plan.exit_source == 0
    bind(plan)
    data['exit_branches']['left'] = '07'
    data['exit_branches']['right'] = '06'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='directions'):
        RoutePlan(path)
