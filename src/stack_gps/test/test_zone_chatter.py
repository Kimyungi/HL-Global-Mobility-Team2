"""Measure existing spatial classifier chatter; synthetic coordinates only.

100 samples at the existing GPS 10Hz rate represent 10 seconds. No hysteresis
or debounce is added: these are measurements, not invented acceptance limits.
"""
import random
import pytest
from stack_gps.path_engine import PathEngine
from stack_gps.zones import ZoneMap
from test_path_engine import en_to_latlon, make_track


def memberships(track, span, positions):
    engine = PathEngine(make_track(track), gps_only_ranges=[span])
    zones = ZoneMap.from_engine(engine)
    return [zones.snapshot(engine.snapshot(*en_to_latlon(x, y), heading=0)['idx'])[0][1]
            for x, y in positions]


def edges(levels):
    previous = False
    entries = exits = 0
    for level in levels:
        entries += not previous and level
        exits += previous and not level
        previous = level
    return entries, exits


@pytest.mark.parametrize('name,track,span,positions', [
    ('stopped_boundary_1cm', [(i*.2, 0) for i in range(50)], (10, 20),
     [(1.9 + (.01 if i%2==0 else -.01), 0) for i in range(100)]),
    ('parallel_tracks_20cm_apart', [(x, 0) for x in (-2,-1,0,1,2)] +
     [(x, .2) for x in (2,1,0,-1,-2)], (0, 4),
     [(0, .1 + (-.01 if i%2==0 else .01)) for i in range(100)]),
    ('crossing_1_1cm_jitter', [(i*.02, 0) for i in range(-10,11)] +
     [(0, i*.02) for i in range(-10,11)], (0, 20),
     [(.011, 0) if i%2==0 else (0, .011) for i in range(100)]),
])
def test_existing_nearest_index_can_chatter(name, track, span, positions):
    levels = memberships(track, span, positions)
    entry, exit = edges(levels)
    print(f'{name}: samples=100 duration=10s entries={entry} exits={exit}')
    assert (entry, exit) == (50, 50)


def test_seeded_position_noise_near_boundary_reports_transitions():
    rng = random.Random(20260911)
    positions = [(1.9+rng.gauss(0,.015), rng.gauss(0,.015)) for _ in range(100)]
    levels = memberships([(i*.2,0) for i in range(50)], (10,20), positions)
    entry, exit = edges(levels)
    print(f'gaussian_noise_sigma_1_5cm: samples=100 duration=10s entries={entry} exits={exit}')
    assert entry > 1 and exit > 1


def test_same_noise_away_from_boundary_is_stable():
    rng = random.Random(20260911)
    positions = [(3+rng.gauss(0,.015), rng.gauss(0,.015)) for _ in range(100)]
    assert edges(memberships([(i*.2,0) for i in range(50)], (10,20), positions)) == (1,0)
