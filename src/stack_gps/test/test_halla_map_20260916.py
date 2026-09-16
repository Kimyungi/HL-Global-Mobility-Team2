"""Map-only regression checks using the actual route/stop classifiers; no ROS."""
import csv
import hashlib
import math
from pathlib import Path
import unittest

import yaml

from stack_gps.path_engine import PathEngine, avoidance_marker_range
from stack_gps.route_plan import RoutePlan
from stack_gps.zones import ZoneMap

DATA = Path(__file__).parents[1] / 'waypoints'
SOURCE = DATA / 'reference_paths_01_to_07_split_parking_20260916.csv'


def read_csv(file):
    with file.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


class HallaMap20260916Tests(unittest.TestCase):
    def setUp(self):
        self.source = read_csv(SOURCE)
        self.groups = {i: [r for r in self.source if int(r['path_id']) == i] for i in range(1,8)}

    def test_user_upload_is_preserved_byte_for_byte(self):
        self.assertEqual(hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
                         '6bb043e60f6402bba59e3ac7c80298b9c9f03dadb88c3be0dcc52144ffcfc3d4')

    def test_split_csvs_preserve_every_column_and_row(self):
        self.assertEqual(len(self.source), 901)
        for i, rows in self.groups.items():
            self.assertEqual(read_csv(DATA / f'waypoints_halla_20260916_path_{i:02d}.csv'), rows)
            self.assertEqual([int(r['idx']) for r in rows], list(range(len(rows))))

    def test_boundary_3_4_preserved_and_4_5_tail_trimmed(self):
        xy = lambda r: (float(r['east_m']),float(r['north_m']))
        self.assertEqual(xy(self.groups[3][-1]), (-31.372,-32.9603))
        self.assertEqual(xy(self.groups[3][-1]), xy(self.groups[4][0]))
        self.assertEqual(len(self.groups[4]), 206)
        self.assertEqual(xy(self.groups[4][-1]), (-64.335,-70.648))
        self.assertEqual(xy(self.groups[4][-1]), xy(self.groups[5][0]))

    def test_state5_is_the_only_new_stop_and_retains_user_coordinates(self):
        marked = [r for r in self.source if int(r['state']) == 5]
        self.assertEqual(len(marked), 1)
        self.assertEqual((marked[0]['path_id'],marked[0]['idx']), ('5','70'))
        self.assertEqual((float(marked[0]['east_m']),float(marked[0]['north_m'])),
                         (-51.91973,-60.044973))
        for i in range(1,8):
            config = yaml.safe_load((DATA / f'zones_halla_20260916_path_{i:02d}.yaml').read_text(encoding='utf-8'))
            self.assertEqual(len(config['stop_points']), int(i == 5))
            if i == 5:
                stop = config['stop_points'][0]
                self.assertEqual((stop['lat'],stop['lon']), (37.30363327,127.9069472))

    def test_stop_is_active_in_actual_path_engine(self):
        plan = RoutePlan(DATA / 'halla_route_sequence.yaml', '01', '07')
        route = next(r for r in plan.files if r.id == '05')
        engine = PathEngine(route.points)
        stop = yaml.safe_load(route.zones.read_text(encoding='utf-8'))['stop_points'][0]
        first,last,distance = engine.range_from_latlon(stop['lat'],stop['lon'],1.0)
        self.assertLess(distance, 1e-6)
        self.assertLessEqual(first,70)
        self.assertGreaterEqual(last,70)
        engine.stop_ranges=[(first,last)]
        # Same positional classification published as GpsPath.stop_zone by node.py.
        self.assertEqual(engine._zone_id(70,engine.stop_ranges),1)
        self.assertEqual(engine._zone_id(0,engine.stop_ranges),0)

    def test_all_four_branch_choices_load_without_teleport_connectors(self):
        identities=set()
        for start in ('01','02'):
            for end in ('06','07'):
                plan=RoutePlan(DATA / 'halla_route_sequence.yaml',start,end)
                self.assertEqual([r.id for r in plan.files],[start,'03','04','05',end])
                self.assertTrue(all('20260916' in r.csv.name and '20260916' in r.zones.name for r in plan.files))
                self.assertTrue(all(not r.entry_connection and r.completion == 0 for r in plan.files))
                for previous,current in zip(plan.files,plan.files[1:]):
                    a,b=previous.points[-1],current.points[0]
                    self.assertLess(math.hypot((a[0]-b[0])*111000,(a[1]-b[1])*88500),.02)
                identities.add(plan.sequence_id)
        self.assertEqual(len(identities),4)

    def test_parking_and_avoidance_events_survive_new_indices(self):
        expected={3:('perpendicular',145),4:('parallel',185)}
        for i,(mode,index) in expected.items():
            config=yaml.safe_load((DATA / f'zones_halla_20260916_path_{i:02d}.yaml').read_text(encoding='utf-8'))
            point=config['parking_points'][0]
            row=self.groups[i][index]
            self.assertEqual(point['mode'],mode)
            self.assertEqual((point['lat'],point['lon']),(float(row['lat']),float(row['lon'])))
        self.assertEqual(avoidance_marker_range(DATA / 'waypoints_halla_20260916_path_04.csv'),[(77,205)])

    def test_zone_endpoints_use_retained_coordinates(self):
        for i,rows in self.groups.items():
            available={(float(r['lat']),float(r['lon'])) for r in rows}
            zones=yaml.safe_load((DATA / f'zones_halla_20260916_path_{i:02d}.yaml').read_text(encoding='utf-8'))
            self.assertEqual(zones['track'],f'waypoints_halla_20260916_path_{i:02d}.csv')
            for key in ('gps_only_zones','avoid_zones'):
                for interval in zones[key]:
                    for end in ('start','end'):
                        self.assertIn((interval[end]['lat'],interval[end]['lon']),available)


if __name__ == '__main__':
    unittest.main()
