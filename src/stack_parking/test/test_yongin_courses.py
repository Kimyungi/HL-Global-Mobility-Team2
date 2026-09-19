"""Verify real Yongin entry/exit pairing and runtime exit selection."""
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from stack_parking.parking_courses import load_catalog
from stack_parking.t_parking_sequence import TParkingSequence
from stack_parking.geometry import Pose2

ROOT=Path(__file__).parents[3]


def courses(start='01'):
    return load_catalog(ROOT/'src/stack_parking/config/yongin_parking_courses.yaml',
                        ROOT/f'src/stack_gps/waypoints/yongin_reference_path_{start}.csv')


def test_pairings_and_common_origin_for_each_start():
    for start in ['01','02','03','04','05','06','07']:
        catalog=courses(start)
        assert catalog[1].route_id=='03' and catalog[2].route_id=='04'
        for mode, names in [(1,['01','02']),(2,['03','04'])]:
            c=catalog[mode]
            assert [p.name for p in c.course[0]]==['yongin_parking_ref_'+n for n in names]
            assert (c.exits is not None)==(mode==1)
            for candidate in c.course[0]:
                assert np.hypot(candidate.path[0].x-c.course[1][-1].x,candidate.path[0].y-c.course[1][-1].y)<.14


def test_t_uses_paired_exit_parallel_retraces_driven_prefix():
    for mode,c in courses().items():
        for selected in (0,1):
            core=TParkingSequence(*c.course,exits=c.exits)
            candidate=c.course[0][selected]
            core.phase='REVERSE';core.selected=selected
            index=len(candidate.path)-1
            core.reverse=NS(index=index,tick=lambda *a,**kw:NS(reason='done',phase='SUCCESS',parking_success=True))
            point=candidate.path[index]
            result=core.tick(1.,Pose2(point.x,point.y,point.yaw),1.,0.,1.,None,None,None,owned=True)
            assert result.phase=='WAIT_3'
            if mode==1:
                assert core.exit_path is c.exits[selected][0]
                assert all(p.gear==1 for p in core.exit_path)
            else:
                assert core.exit_path[-1].x==candidate.path[0].x
                assert core.exit_path[-1].y==candidate.path[0].y
                assert core.exit_path[0].x==candidate.path[index].x
