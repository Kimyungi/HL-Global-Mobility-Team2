"""Native packaging, exact geometry and boundary decisions; no hardware."""
import math
import random
import struct

import numpy as np
import pytest

from stack_avoid.compute_backend import compute_functions
from stack_avoid.gps_cubic_path import GpsCubicPlanner
from stack_avoid.surfaces import Surface
from test_surface_node import node, scene_scan  # noqa: F401


def test_native_connector_preserves_float64_points():
    reference, _ = compute_functions('python')
    native, _ = compute_functions('native')
    rng = random.Random(2141)
    for i in range(258):
        start = (rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-math.pi, math.pi), 0.)
        goal = (rng.uniform(-3, 8), rng.uniform(-3, 3), rng.uniform(-math.pi, math.pi), 0.)
        args = (start, goal, .05, (1., 1.25, 1.5, 1.75)[i % 4])
        old, new = reference(*args), native(*args)
        assert len(old) == len(new)
        assert all(struct.pack('dddd', *a) == struct.pack('dddd', *b) for a, b in zip(old, new))


def test_native_collision_decisions_and_touching_boundaries():
    _, reference = compute_functions('python')
    _, native = compute_functions('native')
    rng = random.Random(1500)
    cases = []
    for _ in range(200):
        poses = [(rng.uniform(-4, 4), rng.uniform(-4, 4), rng.uniform(-math.pi, math.pi), 0.)
                 for _ in range(rng.randrange(1, 50))]
        surfaces = [Surface(tuple((rng.uniform(-6, 6), rng.uniform(-6, 6))
                                 for _ in range(rng.randrange(1, 8))))
                    for _ in range(rng.randrange(0, 12))]
        cases.append((poses, surfaces))
    for axis, bounds in [(0, (-.09-.15, .76+.15)), (1, (-.62/2-.15, .62/2+.15))]:
        for bound in bounds:
            for value in (math.nextafter(bound, -math.inf), bound, math.nextafter(bound, math.inf)):
                edge = ((value, -1.), (value, 1.)) if axis == 0 else ((-1., value), (1., value))
                cases.append(([(0., 0., 0., 0.)], [Surface(edge)]))
    for poses, surfaces in cases:
        args = (poses, surfaces, .62, .76, .09, .15)
        assert reference(*args) == native(*args)


@pytest.mark.parametrize('gap', [.7, 2., 3.3])
def test_native_scan_callback_matches_reference(node, gap):
    results = []
    for backend in ('python', 'native'):
        connector, checker = compute_functions(backend)
        node._planner = GpsCubicPlanner(width=.62, length=.85, front=.76, margin=.15,
                                       min_radius=1.15, connector=connector, collision_check=checker)
        node._completed = node._detected_prev = False
        node._path_last_scan = 0
        node.detect_range, node.offset_max = 3.5, 2.5
        node.on_scan(scene_scan([((gap, -.2), (gap, .2))]))
        msg = node.messages[-1]
        results.append((msg.points, msg.avoidable, msg.maneuver_done, node._planner.reason,
                        node._planner.path.points if node._planner.path else None))
    assert results[0] == results[1]


def test_unknown_backend_is_rejected():
    with pytest.raises(ValueError, match='python or native'):
        compute_functions('typo')


@pytest.mark.parametrize('bad', ['shape', 'dtype', 'strides'])
def test_native_binding_rejects_invalid_buffers(bad):
    from stack_avoid import _footprint_native
    poses = np.zeros((2, 4))
    if bad == 'shape':
        poses = np.zeros((2, 3))
    elif bad == 'dtype':
        poses = poses.astype(np.float32)
    else:
        poses = poses[:, ::-1]
    with pytest.raises((ValueError, BufferError)):
        _footprint_native.footprint(poses, np.ones(2), np.zeros(2), np.zeros((1, 2, 2)),
                                    .62, .76, .09, .15)
