"""Offline prototype, same slab intersection predicates; batched samples."""
import numpy as np

def footprint_clear(points, surfaces, width, front, rear, margin):
    """Check continuous measured edges against each sampled oriented vehicle box.

    Spacing/2 is included in margin by the caller to cover between-sample travel.
    """
    edges = [edge for s in surfaces for edge in s.segments()]
    if not edges or not points:
        return True
    edges = np.asarray(edges, dtype=float)
    # Chunk the path to bound memory even with dense full-resolution scans.
    chunk = min(512, max(1, 32768 // len(edges)))
    all_poses = np.asarray(points, dtype=float)
    all_c = np.cos(all_poses[:, 2])
    all_s = np.sin(all_poses[:, 2])
    for offset in range(0, len(points), chunk):
        poses = all_poses[offset:offset+chunk]
        relative = edges[None, :, :, :] - poses[:, None, None, :2]
        c, s = all_c[offset:offset+chunk, None, None], all_s[offset:offset+chunk, None, None]
        local = (c*relative[..., 0]+s*relative[..., 1],
                 -s*relative[..., 0]+c*relative[..., 1])
        near = np.zeros((len(poses), len(edges)))
        far = np.ones_like(near)
        possible = np.ones_like(near, dtype=bool)
        for coords, low, high in zip(local, (-rear-margin, -width/2-margin),
                                    (front+margin, width/2+margin)):
            a, delta = coords[..., 0], coords[..., 1]-coords[..., 0]
            parallel = np.abs(delta) < 1e-12
            possible &= ~parallel | ((a >= low) & (a <= high))
            denominator = np.where(parallel, 1., delta)
            t0, t1 = (low-a)/denominator, (high-a)/denominator
            near = np.maximum(near, np.where(parallel, -np.inf, np.minimum(t0, t1)))
            far = np.minimum(far, np.where(parallel, np.inf, np.maximum(t0, t1)))
        if np.any(possible & (near <= far)):
            return False
    return True

