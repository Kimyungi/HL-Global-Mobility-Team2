"""Reproducible straight/curved LiDAR perception and fixed-path plots.

This is a geometry/perception simulation, not a vehicle/dSPACE dynamics test.
Four ideal horizontal LiDAR views are ray-cast against static circular objects.
Both obstacles are detected from the synthetic 3 m LiDAR cloud. A separate
first-detection audit records entry timing on the waypoint centerline.
"""
import argparse
import csv
from dataclasses import replace
import json
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC/'stack_gps'), str(SRC/'stack_avoid')]
from stack_gps.path_engine import M_PER_DEG_LAT, PathEngine
from stack_avoid.waypoint_planner import (Config, Detector, FixedPlanner, Obstacle,
                                          filter_cloud, to_global, to_vehicle)


def make_route(curved):
    s = np.linspace(0, 28, 561)
    radius = 15.0
    x = radius*np.sin(s/radius) if curved else s
    y = radius*(1-np.cos(s/radius)) if curved else np.zeros(len(s))
    yaw = s/radius if curved else np.zeros(len(s))
    lat0, lon0 = 37.3, 127.9
    pts = [(lat0+float(n)/M_PER_DEG_LAT,
            lon0+float(e)/(M_PER_DEG_LAT*math.cos(math.radians(lat0)))) for e, n in zip(x, y)]
    return PathEngine(pts, waypoint_yaws=yaw.tolist())


def synthetic_fused_cloud(pose, objects, config, rng):
    # Pose/FOV are idealised already-calibrated base_link sensor views.
    sensors = ((.76, 0, 0, 90), (-.11, 0, 180, 70),
               (.215, .21, 90, 55), (.246, -.225, -90, 55))
    points = []
    for x, y, yaw_deg, half_deg in sensors:
        origin = to_global([(x, y)], pose)[0]
        for angle in np.deg2rad(np.arange(yaw_deg-half_deg, yaw_deg+half_deg+.1, 1.0)):
            direction = np.array([math.cos(angle+pose[2]), math.sin(angle+pose[2])])
            closest = float('inf')
            for o in objects:
                relative = origin - np.array([o.x, o.y])
                along = float(relative@direction)
                disc = along*along - float(relative@relative) + .13**2
                if disc >= 0:
                    distance = -along-math.sqrt(disc)
                    if 0 < distance < closest:
                        closest = distance
            if closest < config.range_limit+.8:
                hit = origin + (closest+rng.normal(0, .002))*direction
                point = to_vehicle((*hit, 0, 0), pose)
                points.append(point[:2])
    return filter_cloud(points, config)


def object_at(planner, station, lateral):
    p = planner.point(station, lateral)
    return Obstacle(station, lateral, p.x, p.y)


def path_pose(planner, station):
    for segment in planner.segments:
        if segment.start.station <= station <= segment.end.station:
            return segment.evaluate((station-segment.start.station)/
                                    (segment.end.station-segment.start.station))[:3]
    return planner.route.at_station(station)[:3]


def simulate(curved, same_side=False, half_offset=False):
    route = make_route(curved)
    cfg = replace(Config(), obstacle_offsets=(.5, 1.0) if half_offset else (1.0,))
    planner, detector = FixedPlanner(route, cfg), Detector(route, cfg)
    lateral = .5 if half_offset else 1.0
    second_s = 11.0
    objects = [object_at(planner, 8, lateral),
               object_at(planner, second_s, lateral if same_side else -lateral)]
    rng = np.random.default_rng(16)
    snapshots, history, seen, preview_errors = [], [], [], []
    first_prefix = None
    final_report = None
    last_path = None
    finished = False
    # Audit real first detection on the waypoint centerline, without preloading.
    live_planner, live_detector = FixedPlanner(route, cfg), Detector(route, cfg)
    audit_rng = np.random.default_rng(16)
    perception_audit = None
    for ego_s in np.arange(4.0, 8.0, .04):
        pose = route.at_station(float(ego_s))[:3]
        cloud = to_global(synthetic_fused_cloud(pose, objects[:1], cfg, audit_rng), pose)
        detections = live_detector.observe(cloud)
        if detections:
            o = detections[0]
            accepted = live_planner.accept(o, route.project_station(*pose[:2])[0])
            perception_audit = {'ego_station': float(ego_s),
                                'required_entry_station': o.station-cfg.approach,
                                'accepted': accepted, 'reason': live_planner.last_reason}
            break
    assert perception_audit is not None, 'first obstacle was never detected'
    for station in np.arange(4.0, second_s+cfg.hold+cfg.departure+.5, .04):
        pose = path_pose(planner, float(station))
        local_cloud = synthetic_fused_cloud(pose, objects, cfg, rng)
        map_cloud = to_global(local_cloud, pose)
        for o in detector.observe(map_cloud):
            before = planner.revision
            planner.accept(o, route.project_station(*pose[:2])[0])
            if before != planner.revision:
                snapshots.append((float(station), planner.samples, tuple(planner.maneuvers)))
                seen.append({'ego_station': float(station), 'obstacle_station': o.station,
                             'obstacle_lateral': o.lateral, 'revision': planner.revision})
                if len(snapshots) == 1:
                    first_prefix = planner.segments[:2]
                else:
                    assert first_prefix == planner.segments[:2], 'frozen prefix moved'
                final_report = planner.geometry_report()
                last_path = planner.samples
        if planner.samples:
            before = planner.samples
            target = planner.preview(pose)
            assert target is not None, 'missing 1 m preview'
            preview_errors.append(abs(math.hypot(target[0]-pose[0], target[1]-pose[1])-1))
            assert planner.samples == before, 'preview altered fixed path'
        history.append((*pose, float(station), planner.revision))
        if planner.advance(pose):
            finished = True
            break
    assert len(snapshots) == 2, f'expected two latched objects; got {len(snapshots)}: {planner.last_reason}'
    assert finished, 'final P4 was not passed'
    return {'route': route, 'cfg': cfg, 'objects': objects, 'snapshots': snapshots,
            'history': np.array(history), 'report': final_report, 'last_path': last_path,
            'events': seen, 'preview_error': max(preview_errors), 'finished': finished,
            'perception_audit': perception_audit}


def draw_scene(ax, result, title):
    route, cfg = result['route'], result['cfg']
    stations = np.linspace(3.5, 15.5, 420)
    xy = np.array([route.at_station(float(s))[:3] for s in stations])
    normal = np.column_stack((-np.sin(xy[:, 2]), np.cos(xy[:, 2])))
    ax.plot(xy[:, 0], xy[:, 1], color='#64748b', lw=1.3, ls='--', label='Waypoint centerline')
    for sign in (-1, 1):
        wall = xy[:, :2]+sign*cfg.wall_offset*normal
        ax.plot(wall[:, 0], wall[:, 1], color='#334155', lw=1.5,
                label='Virtual walls ±2 m' if sign == 1 else None)
    initial = np.array(result['snapshots'][0][1])
    joined = np.array(result['last_path'])
    ax.plot(initial[:, 0], initial[:, 1], color='#f59e0b', ls=':', lw=2.4, label='Initial return (replaced)')
    ax.plot(joined[:, 0], joined[:, 1], color='#2563eb', lw=2.3, label='Fixed chained cubic')
    for i, o in enumerate(result['objects']):
        ax.add_patch(Circle((o.x, o.y), .13, color='#dc2626'))
        ax.annotate(f'Obstacle {i+1}', (o.x, o.y), xytext=(7, 10), textcoords='offset points', fontsize=9)
    plans = result['snapshots'][-1][2]
    controls = [(p, name) for p, name in zip(plans[0].points[:3], ('A1', 'A2', 'A3 = B1'))]
    controls += [(p, name) for p, name in zip(plans[1].points[1:], ('A4 = B2', 'B3', 'B4'))]
    for p, name in controls:
        ax.scatter(p.x, p.y, color='#0f172a', s=25, zorder=5)
        ax.arrow(p.x, p.y, .4*math.cos(p.yaw), .4*math.sin(p.yaw),
                 head_width=.09, color='#059669', length_includes_head=True, zorder=4)
        offset = {'A2': (-8, -17), 'A3 = B1': (12, -31),
                  'A4 = B2': (-12, 17), 'B3': (12, -20)}.get(name, (4, -17))
        ax.annotate(name, (p.x, p.y), xytext=offset, textcoords='offset points', fontsize=8,
                    ha='center' if name in ('A3 = B1', 'A4 = B2') else 'left',
                    bbox={'facecolor': 'white', 'edgecolor': 'none', 'alpha': .8, 'pad': .4})
    # Circle and preview along the retained curve, after the first P2.
    planner = FixedPlanner(route, cfg)
    planner.maneuvers = list(plans)
    planner.segments = plans[0].segments[:2]+plans[1].segments
    planner.samples = result['last_path']
    pose = path_pose(planner, plans[0].points[1].station+.25)
    target = planner.preview(pose)
    ax.add_patch(Circle(pose[:2], 1, fill=False, color='#7c3aed', lw=1, alpha=.5))
    ax.plot([pose[0], target[0]], [pose[1], target[1]], color='#7c3aed', lw=1.5)
    ax.scatter(*target[:2], s=80, marker='*', color='#7c3aed', label='1 m preview', zorder=6)
    ax.set_title(title, loc='left', fontsize=12, weight='bold')
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('GPS ENU east [m]')
    ax.set_ylabel('GPS ENU north [m]')
    ax.grid(alpha=.15)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cases = [('straight_opposite', False, False, False), ('curve_opposite', True, False, False),
             ('straight_same_side', False, True, False), ('curve_same_side', True, True, False),
             ('straight_half_offset', False, False, True), ('curve_half_offset', True, False, True)]
    results, summaries = {}, {}
    for name, curved, same, half in cases:
        r = simulate(curved, same, half)
        results[name] = r
        summaries[name] = {'mode': 'synthetic 3 m LiDAR and ideal path poses; no dynamics',
                           'live_first_detection': r['perception_audit'],
                           'events': r['events'], 'final_passed': r['finished'],
                           'preview_max_error_m': r['preview_error'], **r['report']}
        with (args.output/(name+'.csv')).open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['east_m', 'north_m', 'yaw_rad', 'curvature_1_m', 'waypoint_station_m'])
            writer.writerows(r['last_path'])
    fig, axes = plt.subplots(2, 2, figsize=(15, 10), constrained_layout=True)
    for ax, name, title in zip(axes.flat, list(results)[:4],
                              ['Straight · opposite sides', 'Curve · opposite sides',
                               'Straight · same side', 'Curve · same side']):
        draw_scene(ax, results[name], title)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='outside lower center', ncol=5, frameon=False, fontsize=9)
    fig.suptitle('Offset 1 m | entry/exit 2.5 m | synthetic 3 m LiDAR (no dynamics)', fontsize=16, weight='bold')
    fig.savefig(args.output/'straight_curve_paths.png', dpi=180)
    plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(14, 5), constrained_layout=True)
    for ax, name, title in zip(axes, list(results)[4:], ['Straight · optional ±0.5 m', 'Curve · optional ±0.5 m']):
        draw_scene(ax, results[name], title)
    fig.savefig(args.output/'half_offset_paths.png', dpi=180)
    plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), constrained_layout=True)
    for name in ('straight_opposite', 'curve_opposite'):
        p = np.array(results[name]['last_path'])
        axes[0].plot(p[:, 4], p[:, 3], label=name)
        axes[1].plot(p[:, 4], np.rad2deg(p[:, 2]), label=name)
    axes[0].axhline(1/1.15, color='#dc2626', ls='--', label='Configured steering limit ±1/1.15')
    axes[0].axhline(-1/1.15, color='#dc2626', ls='--')
    axes[0].set_ylabel('Curvature [1/m]')
    axes[1].set_ylabel('Path yaw [deg]')
    axes[1].set_xlabel('Waypoint station [m]')
    for ax in axes:
        ax.grid(alpha=.2)
        ax.legend(frameon=False, fontsize=9)
    fig.suptitle('Geometry check: requested cubic exceeds the configured turn limit', fontsize=13)
    fig.savefig(args.output/'curvature_and_yaw.png', dpi=180)
    plt.close(fig)
    (args.output/'simulation_results.json').write_text(json.dumps(summaries, indent=2), encoding='utf-8')
    print(json.dumps(summaries, indent=2))


if __name__ == '__main__':
    main()
