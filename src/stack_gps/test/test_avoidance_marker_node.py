from pathlib import Path
from types import SimpleNamespace as NS

from stack_gps.node import StackGpsNode
from stack_gps.path_engine import PathEngine, load_waypoints_csv


def test_production_zone_setup_uses_each_routes_own_marker():
    root = Path(__file__).parents[1] / 'waypoints'
    log = NS(info=lambda msg: None, warn=lambda msg: None, error=lambda msg: None)
    for route in ('01', '03', '04', '05', '07'):
        csv_path = root / f'waypoints_halla_reference_path_{route}.csv'
        values = dict(waypoint_csv=str(csv_path),
                      zones_file=str(root / f'zones_halla_reference_path_{route}.yaml'),
                      stop_zone_snap_max_m=3., stop_zone_span_m=1., parking_zone_span_m=1.,
                      stop_points_latlon='', avoid_zone_latlon='', gps_only_zone_latlon='',
                      avoid_zone_lead_m=5.)
        node = NS(engine=PathEngine(load_waypoints_csv(csv_path)), get_logger=lambda: log)
        StackGpsNode._setup_zones(node, lambda key: NS(value=values[key]))
        assert node.engine.avoid_ranges == ([(55, 191)] if route == '04' else [])
