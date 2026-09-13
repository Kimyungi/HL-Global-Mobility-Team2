"""Station preview on the fitted vehicle-frame centreline y = a*x² + b*x + c."""
from __future__ import annotations

import math

import numpy as np


# User-selected distance along the centreline, measured from the vehicle's projection.
CAMERA_PREVIEW_STATION_M = 2.5


def _arc_length(a: float, b: float, start_x: float, end_x: float) -> float:
    dx = end_x - start_x
    u0, u1 = 2.0 * a * start_x + b, 2.0 * a * end_x + b
    # Avoid cancellation in the antiderivative for a straight/nearly straight curve.
    # This is a numerical tolerance, not a driving-distance threshold.
    if abs(u1 - u0) <= 1e-6 * max(1.0, abs(u0), abs(u1)):
        return dx / 6.0 * (math.hypot(1.0, u0)
                           + 4.0 * math.hypot(1.0, (u0 + u1) / 2.0)
                           + math.hypot(1.0, u1))
    return (u1 * math.hypot(1.0, u1) - u0 * math.hypot(1.0, u0)
            + math.asinh(u1) - math.asinh(u0)) / (4.0 * a)


def station_preview_x(center_coeffs: np.ndarray) -> tuple[float, float]:
    """Return (vehicle projection x, x at station +2.5m), in increasing-x direction.

    The closest projection minimises x² + y(x)² over the fitted polynomial.
    Its stationary points solve x + y*y' = 0. The preview is evaluated on that
    same polynomial; neither x nor radial distance is clamped to 2.5m.
    """
    coeffs = np.asarray(center_coeffs, dtype=float)
    if coeffs.shape != (3,) or not np.isfinite(coeffs).all():
        raise ValueError('station preview requires three finite curve coefficients')
    a, b, c = map(float, coeffs)
    if a == 0.0:
        projection_x = -b * c / (1.0 + b * b)
    else:
        polynomial = [2.0 * a * a, 3.0 * a * b, 1.0 + b * b + 2.0 * a * c, b * c]
        if not np.isfinite(polynomial).all():
            raise ValueError('nonfinite projection polynomial')
        roots = np.roots(polynomial)
        candidates = [float(root.real) for root in roots
                      if np.isfinite(root) and abs(root.imag) <= 1e-8 * max(1.0, abs(root.real))]
        if not candidates:
            raise ValueError('no finite vehicle projection')
        projection_x = min(candidates, key=lambda x: (math.hypot(x, (a*x + b)*x + c), x))

    # ds/dx = sqrt(1 + y'²) >= 1, so +2.5 in x always brackets +2.5 in station.
    lo, hi = projection_x, projection_x + CAMERA_PREVIEW_STATION_M
    for _ in range(56):
        mid = (lo + hi) / 2.0
        length = _arc_length(a, b, projection_x, mid)
        if not math.isfinite(length):
            raise ValueError('nonfinite station length')
        if length < CAMERA_PREVIEW_STATION_M:
            lo = mid
        else:
            hi = mid
    target_x = (lo + hi) / 2.0
    if not math.isfinite(projection_x) or not math.isfinite(target_x):
        raise ValueError('nonfinite station preview')
    return projection_x, target_x
