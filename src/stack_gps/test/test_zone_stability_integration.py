"""Real PathEngine raw results through the production C++ Zone filter (test-only N=3)."""
import ast
import math
from pathlib import Path
import subprocess
from types import SimpleNamespace as NS
import pytest
from stack_gps.path_engine import PathEngine
from stack_gps.zones import ZoneMap
from test_path_engine import en_to_latlon, make_track

ROOT = Path(__file__).parents[3]


@pytest.fixture(scope='module')
def zone_filter(tmp_path_factory):
    directory = tmp_path_factory.mktemp('zone_filter_cpp')
    source = directory / 'filter.cpp'
    source.write_text('''#include "core/zone_step.hpp"
#include <iostream>
using namespace adas_mgm;
int main() { ZoneState state{}; ZoneSnapshot input{}; input.zone_valid=true; input.count=1;
input.observations[0]={1,ZoneType::GPS_ONLY_ZONE,MissionType::NONE,0,false,-1};
int inside; while(std::cin>>inside) { ++input.generation; input.observations[0].in_zone=inside;
for(int tick=0;tick<10;++tick) {zone_step(input,true,state,3,3);}
std::cout << state.in_gps_only_zone << "\\n"; }}''')
    executable = directory / 'filter'
    subprocess.run(['g++', '-std=c++17', '-I', str(ROOT / 'src/adas_mgm'), str(source),
                    str(ROOT / 'src/adas_mgm/core/zone_step.cpp'), '-o', str(executable)], check=True)

    def apply(levels):
        result = subprocess.run([str(executable)], input='\n'.join(str(int(x)) for x in levels),
                                text=True, capture_output=True, check=True)
        return [bool(int(x)) for x in result.stdout.split()]
    return apply


def count_edges(levels):
    return sum(a != b for a, b in zip([False] + levels, levels))


@pytest.mark.parametrize('name,track,span,positions', [
    ('boundary', [(i*.2, 0) for i in range(50)], (10,20),
     [(1.9 + (.01 if i%2==0 else -.01), 0) for i in range(100)]),
    ('parallel', [(x,0) for x in (-2,-1,0,1,2)] + [(x,.2) for x in (2,1,0,-1,-2)], (0,4),
     [(0,.1+(-.01 if i%2==0 else .01)) for i in range(100)]),
    ('crossing', [(i*.02,0) for i in range(-10,11)] + [(0,i*.02) for i in range(-10,11)], (0,20),
     [(.011,0) if i%2==0 else (0,.011) for i in range(100)]),
])
def test_z7_z8_real_nearest_index_jumps_are_filtered_but_not_relocalized(zone_filter, name, track, span, positions):
    engine = PathEngine(make_track(track), gps_only_ranges=[span])
    zones = ZoneMap.from_engine(engine)
    indices = [engine.snapshot(*en_to_latlon(x,y), heading=0)['idx'] for x,y in positions]
    raw = [zones.snapshot(i)[0][1] for i in indices]
    stable = zone_filter(raw)
    print(f'{name}: raw_edges={count_edges(raw)}, stable_edges={count_edges(stable)}, '
          f'nearest_index_changes={sum(a!=b for a,b in zip(indices,indices[1:]))}')
    assert count_edges(raw) == 100
    assert count_edges(stable) == 0
    # Heading=0 did not constrain nearest segment. A persistent wrong membership
    # still passes N=3; this is an explicit unresolved localization limitation.
    assert zone_filter(raw + [True]*3)[-1]


def test_low_speed_boundary_progression_is_delayed_by_independent_samples(zone_filter):
    track = [(i*.2,0) for i in range(50)]
    engine = PathEngine(make_track(track), gps_only_ranges=[(10,20)])
    zones = ZoneMap.from_engine(engine)
    raw = [zones.snapshot(engine.snapshot(*en_to_latlon(1.8+i*.01,0), heading=0)['idx'])[0][1]
           for i in range(40)]  # synthetic 0.1 m/s at 10Hz
    stable = zone_filter(raw)
    assert stable.index(True) == raw.index(True) + 2
    assert count_edges(stable) == 1


def test_zone_calibration_telemetry_keeps_independent_fix_identity():
    # Invoke production wrapper method without starting ROS/sensors.
    tree = ast.parse((ROOT / 'src/stack_gps/stack_gps/node.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'StackGpsNode')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_fill_zone_telemetry')
    module = ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[]))
    namespace = {'math': math}; exec(compile(module, '<production telemetry>', 'exec'), namespace)
    engine = PathEngine(make_track([(i*.2,0) for i in range(50)]), gps_only_ranges=[(10,20)])
    instance = NS(_zone_telemetry_fix=None, _zone_telemetry_previous=None, engine=engine,
                  zone_map=ZoneMap.from_engine(engine))
    definition = instance.zone_map.definitions[0]
    msg = NS(track_index=10, zones=[NS(zone_id=definition.zone_id)])
    method = namespace['_fill_zone_telemetry']
    method(instance,msg,1,engine.e[10],engine.n[10],.2,True)
    assert msg.previous_track_index == -1
    assert msg.zones[0].boundary_distance_m == 0
    msg.track_index=11
    method(instance,msg,2,engine.e[11],engine.n[11],.3,True)
    assert msg.previous_track_index == 10
    assert msg.position_step_m == pytest.approx(.2, abs=.001)
    method(instance,msg,2,engine.e[11],engine.n[11],.7,False)
    assert msg.previous_track_index == 10
    assert msg.vehicle_heading_rad == .3 and msg.vehicle_heading_valid
