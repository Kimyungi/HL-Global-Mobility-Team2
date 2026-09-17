import math
import numpy as np
from stack_parking.geometry import Pose2
from stack_parking.t_reference_parking import Config, Scan, footprint_hits, inspect_candidate
from stack_parking.parking_selection_view import selection_markers
from test_t_reference_parking import path
from pathlib import Path
from stack_parking.t_parking_sequence import load_course
from stack_parking.t_reference_parking import selection_path


def test_real_csv_inspects_only_one_point_five_metres_after_turn():
    root = Path(__file__).resolve().parents[2]/'stack_gps'/'waypoints'
    candidates, _, _ = load_course(root/'parking_waypoint.csv', root/'parking_waypoint.csv',
                                   [root/f'parking_waypoint_rev{i}.csv' for i in (1,2)])
    cfg = Config()
    for candidate in candidates:
        prefix = selection_path(candidate, cfg)
        length = sum(math.hypot(b.x-a.x,b.y-a.y) for a,b in zip(prefix,prefix[1:]))
        assert math.isclose(length, candidate.turn_end_s+1.5, abs_tol=1e-6)
        assert len(prefix) < len(candidate.path)
        near, deep = prefix[-1], candidate.path[-1]
        scan = Scan(1., np.array([[near.x,near.y],[deep.x,deep.y]]), (0.,0.))
        mask = np.zeros(2, dtype=bool)
        for p in prefix:
            mask |= footprint_hits(p, Pose2(), scan, cfg)
        assert list(mask) == [True,False]
        deep_only = Scan(1.,scan.points[1:],(0.,0.))
        assert not inspect_candidate(candidate,Pose2(),deep_only,cfg)[0]
    markers = selection_markers(candidates,cfg,Pose2(),scan)
    for i,candidate in enumerate(candidates):
        assert len(markers.markers[i*3].points) == 6*len(selection_path(candidate,cfg))


def test_display_matches_selection_hits_in_rotated_map_frame():
    candidates = (path('a', 1), path('b', -1))
    cfg = Config()
    pose = Pose2(2., -1., math.pi/2)
    p = candidates[0].path[len(candidates[0].path)//2]
    dx, dy = p.x-pose.x, p.y-pose.y
    scan = Scan(1., np.array([[dy, -dx], [50., 50.]]), (0., 0.))
    markers = selection_markers(candidates, cfg, pose, scan)
    for i, candidate in enumerate(candidates):
        area, hits, label = markers.markers[i*3:i*3+3]
        mask = np.zeros(len(scan.points), dtype=bool)
        for point in candidate.path:
            mask |= footprint_hits(point, pose, scan, cfg)
        assert len(hits.points) == int(mask.sum())
        assert ('BLOCKED' in label.text) == inspect_candidate(candidate, pose, scan, cfg)[0]
        assert len(area.points) == len(candidate.path)*6
        assert area.header.frame_id == 'map'
    hit = markers.markers[1].points[0]
    assert math.isclose(hit.x, p.x, abs_tol=1e-9)
    assert math.isclose(hit.y, p.y, abs_tol=1e-9)


def test_missing_scan_is_not_displayed_as_clear():
    markers = selection_markers((path('a',1),path('b',-1)), Config(), Pose2(), None)
    assert all('NO SCAN' in markers.markers[i].text for i in (2,5))
    assert not markers.markers[1].points
