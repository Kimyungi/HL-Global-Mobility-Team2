"""ROS-free geometry/state tests: python -m unittest discover -s ... -p ..."""
import csv
import math
from pathlib import Path
import sys
import tempfile
import unittest
from dataclasses import replace

import numpy as np

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / 'stack_gps'), str(SRC / 'stack_avoid')]
from stack_gps.path_engine import PathEngine, M_PER_DEG_LAT, load_waypoints_csv, wrap_angle
from stack_avoid.waypoint_planner import (Config, Detector, FixedPlanner, Obstacle,
                                          filter_cloud, to_global, to_vehicle)


def route(curved=False, heading=0.0, radius=15.0):
    s = np.linspace(0, 30, 601)
    if curved:
        x, y, yaw = radius*np.sin(s/radius), radius*(1-np.cos(s/radius)), s/radius
    else:
        x, y, yaw = s, np.zeros(len(s)), np.zeros(len(s))
    c, sn = math.cos(heading), math.sin(heading)
    e, n = x*c-y*sn, x*sn+y*c
    lat, lon = 37.3, 127.9
    pts = [(lat+float(b)/M_PER_DEG_LAT,
            lon+float(a)/(M_PER_DEG_LAT*math.cos(math.radians(lat)))) for a, b in zip(e, n)]
    return PathEngine(pts, waypoint_yaws=(yaw+heading).tolist())


def obstacle(planner, station, lateral):
    p = planner.point(station, lateral)
    return Obstacle(station, lateral, p.x, p.y)


def cloud_for(o, radius=.10):
    return np.array([(o.x+radius*math.cos(t), o.y+radius*math.sin(t))
                     for t in np.linspace(0, 2*math.pi, 18, endpoint=False)])


class WaypointTests(unittest.TestCase):
    def test_csv_yaw_survives_filtering_and_duplicate_removal(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/'route.csv'
            p.write_text('lat,lon,quality,yaw_rad\n37,127,4,1.2\n37,127,4,2.1\n'
                         '37,127.001,5,.4\n37,127.002,4,1.3\n', encoding='utf-8')
            points, yaws = load_waypoints_csv(p, include_yaw=True)
            self.assertEqual(yaws, [1.2, 1.3])
            engine = PathEngine(points, waypoint_yaws=yaws)
            self.assertAlmostEqual(engine.yaw[0], 1.2)

    def test_station_matches_gps_segment_foot_on_curve(self):
        r = route(True)
        for station in (1.1, 8.7, 20.3):
            x, y, yaw, _ = r.at_station(station)
            x, y = x-math.sin(yaw)*.8, y+math.cos(yaw)*.8
            projected, lateral, idx, distance = r.project_station(x, y)
            fe, fn, gps_distance = r._foot_on_track(idx, x, y)
            actual = r.at_station(projected)
            self.assertAlmostEqual(actual[0], fe)
            self.assertAlmostEqual(actual[1], fn)
            self.assertAlmostEqual(distance, gps_distance)
            self.assertAlmostEqual(lateral, .8, places=4)

    def test_control_points_follow_each_station_normal(self):
        for curved in (False, True):
            for side in (-1, 1):
                with self.subTest(curved=curved, side=side):
                    p = FixedPlanner(route(curved, heading=math.pi/2))
                    m = p.make_maneuver(obstacle(p, 8, side))
                    for cp, station, d in zip(m.points, (5.5, 8, 8.7, 11.2),
                                              (0, -side*1.0, -side*1.0, 0)):
                        x, y, yaw, _ = p.route.at_station(station)
                        self.assertAlmostEqual(cp.station, station)
                        self.assertAlmostEqual(cp.x-x, -d*math.sin(yaw))
                        self.assertAlmostEqual(cp.y-y, d*math.cos(yaw))
                        self.assertAlmostEqual(wrap_angle(cp.yaw-yaw), 0)

    def test_cubics_pass_through_points_with_specified_yaw(self):
        p = FixedPlanner(route(True, heading=2.4))
        m = p.make_maneuver(obstacle(p, 8, 1))
        for segment in m.segments:
            for u, cp in ((0, segment.start), (1, segment.end)):
                x, y, yaw, _, station = segment.evaluate(u)
                self.assertAlmostEqual(x, cp.x)
                self.assertAlmostEqual(y, cp.y)
                self.assertAlmostEqual(wrap_angle(yaw-cp.yaw), 0)
                self.assertAlmostEqual(station, cp.station)

    def test_latched_geometry_ignores_jitter_and_scan_dropout(self):
        p = FixedPlanner(route())
        original = obstacle(p, 8, 1)
        self.assertTrue(p.accept(original, 5))
        frozen = p.samples
        for shift in (-.03, .02, .04):
            self.assertFalse(p.accept(replace(original, x=original.x+shift), 5.2))
            self.assertFalse(p.advance((7, -1, 0)))
            self.assertEqual(p.samples, frozen)

    def test_second_obstacle_changes_only_return_suffix_both_sides(self):
        for curved in (False, True):
            for side in (-1, 1):
                with self.subTest(curved=curved, side=side):
                    p = FixedPlanner(route(curved))
                    p.accept(obstacle(p, 8, 1), 5)
                    prefix, first = p.segments[:2], p.maneuvers[0].points[:3]
                    self.assertTrue(p.accept(obstacle(p, 11, side), 8.2))
                    self.assertEqual(p.segments[:2], prefix)
                    self.assertEqual(p.maneuvers[0].points[:3], first)
                    self.assertEqual(p.maneuvers[0].points[3], p.maneuvers[1].points[1])
                    self.assertEqual(p.maneuvers[1].points[0], first[2])
                    xy = p.point(8.8, -1.0)
                    self.assertFalse(p.advance((xy.x, xy.y, xy.yaw)))
                    self.assertEqual(p.active_index, 1)
                    # Passing the original return station must not clear the chain.
                    xy = p.point(11.3)
                    self.assertFalse(p.advance((xy.x, xy.y, xy.yaw)))
                    xy = p.point(14.3)
                    self.assertTrue(p.advance((xy.x, xy.y, xy.yaw)))
                    self.assertFalse(p.samples)

    def test_preview_distance_is_one_metre_in_rotated_and_curved_frames(self):
        for curved in (False, True):
            p = FixedPlanner(route(curved, heading=math.pi/2))
            p.accept(obstacle(p, 8, 1), 5)
            frozen = p.samples
            for sample in p.samples[::35]:
                pose = sample[:3]
                target = p.preview(pose)
                self.assertIsNotNone(target)
                ref = to_vehicle(target, pose)
                self.assertAlmostEqual(math.hypot(ref[0], ref[1]), 1, places=8)
                self.assertGreater(ref[0], 0)
                self.assertEqual(p.samples, frozen)

    def test_cloud_transform_crop_and_self_filter(self):
        cfg = Config()
        points = filter_cloud([(0, 0), (1, 1), (3.1, 0), (float('nan'), 1), (-1, 1)], cfg)
        self.assertEqual(len(points), 2)
        transformed = to_global(points, (10, 20, math.pi/2))
        np.testing.assert_allclose(transformed, [(9, 21), (9, 19)])

    def test_detector_confirmation_and_optional_half_metre(self):
        r = route(True)
        p = FixedPlanner(r)
        objects = [obstacle(p, 8, 1), obstacle(p, 10, -.5), obstacle(p, 12, 2)]
        cloud = np.concatenate([cloud_for(o) for o in objects])
        d = Detector(r, Config())
        self.assertEqual(d.observe(cloud), [])
        self.assertEqual(len(d.observe(cloud)), 1)
        d = Detector(r, replace(Config(), obstacle_offsets=(.5, 1.0)))
        d.observe(cloud)
        self.assertEqual(len(d.observe(cloud)), 2)
        self.assertEqual(d.observe([]), [])
        self.assertEqual(d.observe(cloud), [])

    def test_two_point_five_metre_entry_and_close_chain_steering_limit(self):
        p = FixedPlanner(route())
        p.accept(obstacle(p, 8, 1), 5)
        report = p.geometry_report()
        self.assertFalse(report['valid'])
        self.assertAlmostEqual(report['min_radius'], 2.5**2/(6*1.0), places=6)
        self.assertTrue(p.accept(obstacle(p, 11, -1), 8.2))
        chained = p.geometry_report()
        self.assertFalse(chained['valid'])
        self.assertAlmostEqual(chained['peak_curvature'], 12/2.3**2, places=6)
        self.assertGreater(report['wall_clearance'], 0)

    def test_collision_checks_obstacle_extent_and_preserves_path(self):
        p = FixedPlanner(route(), replace(Config(), enforce_turn_radius=False))
        p.accept(obstacle(p, 8, 1), 5)
        frozen = p.samples
        self.assertFalse(p.path_blocked(cloud_for(obstacle(p, 8, 1)), 5))
        self.assertTrue(p.path_blocked(cloud_for(obstacle(p, 8, -1.0)), 5))
        self.assertEqual(p.samples, frozen)

    def test_late_chaining_changes_only_permitted_fourth_point_suffix(self):
        p = FixedPlanner(route())
        p.accept(obstacle(p, 8, 1), 5)
        frozen = p.segments[:2]
        self.assertTrue(p.accept(obstacle(p, 11, 1), 9))
        self.assertEqual(p.segments[:2], frozen)
        self.assertEqual(p.active_index, 1)

    def test_backward_chaining_does_not_change_path(self):
        p = FixedPlanner(route())
        p.accept(obstacle(p, 8, 1), 5)
        frozen = p.samples
        self.assertFalse(p.accept(obstacle(p, 8.5, -1), 8.1))
        self.assertEqual(p.samples, frozen)

    def test_three_metre_lidar_detection_before_entry_is_accepted(self):
        p = FixedPlanner(route())
        d = Detector(p.route, p.config)
        o = obstacle(p, 8, 1)
        pose = (5.25, 0, 0)
        local = [to_vehicle((x, y, 0, 0), pose)[:2] for x, y in cloud_for(o)]
        cloud = to_global(filter_cloud(local, p.config), pose)
        d.observe(cloud)
        detected = d.observe(cloud)
        self.assertEqual(len(detected), 1)
        self.assertTrue(p.accept(detected[0], pose[0]))
        self.assertTrue(p.samples)
        late = FixedPlanner(route())
        self.assertFalse(late.accept(detected[0], 6.0))
        self.assertIn('after the required approach start', late.last_reason)

    def test_route_end_does_not_silently_clip_control_points(self):
        p = FixedPlanner(route())
        self.assertFalse(p.accept(obstacle(p, 29, 1), 26))
        self.assertFalse(p.samples)


if __name__ == '__main__':
    unittest.main(verbosity=2)
