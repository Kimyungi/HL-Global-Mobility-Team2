"""Explicit, read-only selection of the temporary v2 compute backend."""
import numpy as np


def compute_functions(name):
    if name == 'python':
        from stack_avoid.gps_cubic_path import cubic_connector
        from stack_avoid.station_path import footprint_clear
        return cubic_connector, footprint_clear
    if name != 'native':
        raise ValueError('avoid.compute_backend must be python or native')
    try:
        from stack_avoid import _footprint_native
    except ImportError as exc:
        raise RuntimeError('native 회피 모듈 로딩 실패: scripts/v2 build 후 런처를 재시작하세요. '
                           '기존 구현 선택: avoid_compute_backend:=python') from exc
    from stack_avoid.fast_cubic import cubic_connector

    def footprint_clear(points, surfaces, width, front, rear, margin):
        edges = [edge for surface in surfaces for edge in surface.segments()]
        if not edges or not points:
            return True
        poses = np.asarray(points, dtype=float)
        # Same NumPy trig functions as the original checker.
        return _footprint_native.footprint(
            poses, np.cos(poses[:, 2]), np.sin(poses[:, 2]), np.asarray(edges, dtype=float),
            width, front, rear, margin)

    return cubic_connector, footprint_clear
