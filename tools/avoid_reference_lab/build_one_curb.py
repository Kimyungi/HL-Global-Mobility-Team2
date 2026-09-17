"""Render offline synthetic scans through production fixed-goal planning.

Run: source /opt/ros/humble/setup.bash; source install_v2/local_setup.bash
     python3 tools/avoid_reference_lab/build_one_curb.py
No ROS nodes or vehicle commands are started.
"""
import hashlib
import json
import math
from pathlib import Path
import sys
from types import MethodType, SimpleNamespace as NS

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/stack_avoid'))
# Keep the native extension from the built package available to source imports.
import stack_avoid
stack_avoid.__path__.append(str(ROOT / 'build_v2/stack_avoid/stack_avoid'))
from stack_avoid.compute_backend import compute_functions
from stack_avoid.fixed_obstacle_path import FixedObstaclePlanner
from stack_avoid.gps_cubic_path import WaypointWindow
from stack_avoid.node import StackAvoidNode
from stack_avoid.path_io import StationPathIO
from stack_avoid.surfaces import Surface, nearest_in_corridor, scan_surfaces

PARAMS = yaml.safe_load((ROOT / 'src/stack_avoid/config/params.yaml').read_text())['/**']['ros__parameters']
V, A, L = PARAMS['vehicle'], PARAMS['avoid'], PARAMS['lidar_mount']
CONNECTOR, CHECKER = compute_functions('native')


def planner():
    return FixedObstaclePlanner(width=V['width_m'], length=V['length_m'],
        front=V['wheelbase_m'] + V['front_overhang_m'], margin=A['lateral_margin_m'],
        min_radius=V['min_turn_radius_m'], spacing=A['path_spacing_m'],
        anchor_distance=A['waypoint_anchor_m'], return_distance=A['waypoint_return_m'],
        connector=CONNECTOR, collision_check=CHECKER)


def scan_scene(segments):
    # Synthetic front scan, normalized to vehicle-forward angle zero.
    step = math.radians(.5)
    ranges = []
    for i in range(361):
        dx, dy = math.cos(-math.pi/2+i*step), math.sin(-math.pi/2+i*step)
        hits = []
        for a, b in segments:
            ax, ay = a[0]-L['x_m'], a[1]-L['y_m']
            sx, sy = b[0]-a[0], b[1]-a[1]
            det = dx*sy-dy*sx
            if abs(det) < 1e-10:
                continue
            r, u = (ax*sy-ay*sx)/det, (ax*dy-ay*dx)/det
            if .05 <= r <= A['max_range_m'] and 0 <= u <= 1:
                hits.append(r)
        ranges.append(min(hits, default=math.inf))
    return NS(ranges=ranges, angle_min=-math.pi/2, angle_increment=step,
              range_min=.05, range_max=A['max_range_m'])


def observe(p, visible, left, right):
    box = [(3.95, -.2), (4.35, -.2), (4.35, .2), (3.95, .2)]
    edges = list(zip(box, box[1:]+box[:1]))
    curbs = {'left': [(-1., left), (10., left)], 'right': [(-1., right), (10., right)]}
    scan = scan_scene(edges + [curbs[k] for k in visible])
    surfaces = scan_surfaces(scan.ranges, scan.angle_min, scan.angle_increment,
        scan.range_min, scan.range_max, 0., math.pi/2, L['x_m'], L['y_m'],
        A['cluster_dist_m'], A['surface_link_scale'], A['surface_max_link_m'])
    waypoints = WaypointWindow(list(range(16)), [(float(i), 0., 0., 0.) for i in range(16)], 0.)
    n = NS(lidar_x=L['x_m'], lidar_y=L['y_m'], front_center=0.,
        max_range=A['max_range_m'], vehicle_width=V['width_m'], lateral_margin=A['lateral_margin_m'],
        offset_max=A['offset_max_m'], detect_range=A['detect_range_m'], detect_half_width=A['detect_half_width_m'],
        detect_hysteresis=.4, _surfaces=surfaces, _gps_waypoints=waypoints, _planner=p,
        BEHIND_TOL_M=StackAvoidNode.BEHIND_TOL_M, BEHIND_WIN_DEG=StackAvoidNode.BEHIND_WIN_DEG)
    for name in ('_behind_surface', '_target_clear'):
        setattr(n, name, MethodType(getattr(StackAvoidNode, name), n))
    for name in ('_path_goals', '_path_goal_groups'):
        setattr(n, name, MethodType(getattr(StationPathIO, name), n))
    groups = n._path_goal_groups(scan, (0., 0., 0.))
    nearest = nearest_in_corridor(surfaces, L['x_m'], A['detect_half_width_m'])
    detected = nearest is not None and nearest[0] < A['detect_range_m']
    before = [g.point for g in p.fixed_goals]
    target, done = p.step(pose=(0., 0., 0.), waypoints=waypoints, surfaces=surfaces,
                          detected=detected, goal_groups=groups)
    path = p.path.points if p.path else []
    # Isolate curb loss: add true curb geometry to the SAME observed obstacle.
    # Reconstructing unseen box backs would confound this comparison.
    truth = surfaces + [Surface(tuple(e)) for e in curbs.values()]
    observed_safe = bool(path) and p._safe(path, surfaces)
    truth_safe = bool(path) and p._safe(path, truth)
    assert not path or observed_safe
    if before:
        assert before == [g.point for g in p.fixed_goals]
    ttc = nearest[0]/PARAMS['target_speed_mps'] if nearest else 1e9
    return dict(visible=visible, curbs=curbs, box=box, surfaces=[s.points for s in surfaces],
        memory=[s.points for s in p.initial_surfaces+p.last_surfaces],
        candidates=[g for group in groups for g in group.goals], path=path,
        target=target, anchor=[A['waypoint_anchor_m'], 0.], finish=p.return_world,
        fixed=[g.point for g in p.fixed_goals], before=before, detected=detected,
        gap=nearest[0] if nearest else None, ttc=ttc,
        avoidable=bool(target) and ttc >= A['ttc_stop_s'], done=done,
        observed_safe=observed_safe, truth_safe=truth_safe, reason=p.reason,
        fallback=p.anchor_fallback)


def build():
    cases = []
    specs = [
        ('both', '양쪽 연석 검출', ['left', 'right'], 1.6, -1.6,
         '비교 기준입니다. 양쪽 연석과 장애물의 감지 면을 모두 입력합니다.'),
        ('left', '왼쪽 연석만 검출', ['left'], 1.6, -1.6,
         '오른쪽 연석은 실제 장면에 있지만 라이다 입력에서 제외했습니다. 도로 폭을 추정해 복원하지 않습니다.'),
        ('right', '오른쪽 연석만 검출', ['right'], 1.6, -1.6,
         '왼쪽 연석을 입력에서 제외했습니다. 미검출 쪽을 반드시 선택하는 규칙은 없으며 후보 순서와 경로 검사로 결정됩니다.'),
        ('hidden', '미검출 연석이 가까운 경우', ['left'], 1.6, -.85,
         '오른쪽 연석을 실제로 y = −0.85 m에 두고 감지만 제거했습니다. 감지 기준 검사와 미검출 연석 추가 검사의 차이를 확인하세요.'),
        ('tight', '같은 좁은 도로 · 양쪽 검출', ['left', 'right'], 1.6, -.85,
         '바로 앞 시나리오와 동일한 실제 형상입니다. 오른쪽 연석을 입력에 포함했을 때 선택한 목표와 경로를 비교하세요.'),
    ]
    for key, title, visible, left, right, note in specs:
        result = observe(planner(), visible, left, right)
        cases.append(dict(id=key, title=title, note=note, **result))
    p = planner()
    observe(p, ['left', 'right'], 1.6, -1.6)
    cases.append(dict(id='lost', title='목표 확정 후 오른쪽 검출 소실',
        note='동일한 차량 위치에서 두 스캔을 연속 입력했습니다. 첫 스캔은 양쪽 검출, 두 번째는 왼쪽만 검출입니다. 주행 시뮬레이션이 아닌 검출 소실 비교이며, 확정 목표와 최초 감지 면의 유지를 확인합니다.',
        **observe(p, ['left'], 1.6, -1.6)))
    assert all(c['target'] is not None for c in cases), [(c['id'], c['reason']) for c in cases]
    assert all(c['truth_safe'] for c in cases[:3])
    assert cases[1]['target'][1] < 0 < cases[2]['target'][1]
    assert cases[1]['target'] == cases[3]['target']
    assert not cases[3]['truth_safe'] and cases[3]['observed_safe']
    assert cases[4]['truth_safe']
    assert cases[5]['before'] == cases[5]['fixed'] and not cases[5]['candidates']
    sources = ['node.py', 'path_io.py', 'surfaces.py', 'fixed_obstacle_path.py',
               'gps_cubic_path.py', 'fast_cubic.py', 'compute_backend.py', 'station_path.py']
    provenance = []
    for path in [ROOT/'src/stack_avoid/stack_avoid'/s for s in sources] + [ROOT/'src/stack_avoid/config/params.yaml']:
        provenance.append(dict(path=str(path.relative_to(ROOT)), hash=hashlib.sha256(path.read_bytes()).hexdigest()[:12]))
    data = dict(cases=cases, vehicle=V, avoid=A, lidar=L, speed=PARAMS['target_speed_mps'], sources=provenance)
    target = ROOT/'AVOID_ONE_CURB_REFERENCE.html'
    template = (Path(__file__).with_name('one_curb.html')).read_text()
    target.write_text(template.replace('__SCENARIO_DATA__', json.dumps(data, ensure_ascii=False, allow_nan=False).replace('</', '<\\/')))
    (Path(__file__).with_name('one_curb_results.json')).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False))
    print(target)
    for c in cases:
        print(c['id'], 'target=', c['target'], 'observed_safe=', c['observed_safe'], 'truth_safe=', c['truth_safe'], 'candidates=', len(c['candidates']))


if __name__ == '__main__':
    build()
