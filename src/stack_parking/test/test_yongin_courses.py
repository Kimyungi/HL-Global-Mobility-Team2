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


def test_selection_timeout_defaults_to_01_and_03(monkeypatch):
    from stack_parking.t_reference_parking import Scan
    import stack_parking.t_parking_sequence as sequence
    monkeypatch.setattr(sequence, 'inspect_candidate', lambda *args: (False, 0.))
    for mode, course in courses().items():
        for scan_kind in ('missing', 'empty', 'duplicate', 'unresolved'):
            core = TParkingSequence(*course.course, exits=course.exits)
            point = course.course[1][0]
            pose = Pose2(point.x, point.y, point.yaw)

            def tick(now, speed=0.):
                scan = None if scan_kind == 'missing' else Scan(
                    0. if scan_kind == 'duplicate' else now,
                    np.empty((0, 2)) if scan_kind == 'empty' else np.array([[10., 10.]]),
                    (0., 0.))
                return core.tick(now, pose, now, speed, now, scan, None, None, owned=False)

            assert tick(0.).selected is None
            assert tick(.5).selected is None  # stopped: selection timer starts
            assert tick(3.499).selected is None
            result = tick(3.5)
            assert result.phase == 'ADVANCE_3' and result.speed == 0.
            assert result.reason == 'candidate_timeout_default'
            assert core.candidates[result.selected].name == (
                'yongin_parking_ref_01' if mode == 1 else 'yongin_parking_ref_03')
            assert tick(4.).selected == 0


def test_selection_timeout_restarts_after_motion_or_clock_rollback():
    course = courses()[1]
    for interruption in ('motion', 'rollback'):
        core = TParkingSequence(*course.course, exits=course.exits)
        point = course.course[1][0]
        pose = Pose2(point.x, point.y, point.yaw)

        def tick(now, speed=0.):
            return core.tick(now, pose, now, speed, now, None, None, None, owned=False)

        tick(0.); tick(.5); tick(3.)
        if interruption == 'motion':
            assert tick(3.1, .2).selected is None
            tick(4.)
            start = 4.5
        else:
            assert tick(1.).selected is None
            start = 1.5
        assert tick(start).selected is None
        assert tick(start+2.999).selected is None
        assert tick(start+3.).selected == 0
