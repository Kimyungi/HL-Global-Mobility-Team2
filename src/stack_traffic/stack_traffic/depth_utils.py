"""OAK 정렬 depth 영상에서 정지선 인접 노면 거리를 계산하는 순수 함수."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


BBox = Tuple[int, int, int, int]


@dataclass(frozen=True)
class StopLineDepthMeasurement:
    """정지선 가까운 경계의 카메라 optical-Z 거리 진단값."""

    camera_z_m: float
    raw_nonzero_count: int
    raw_nonzero_ratio: float
    valid_count: int
    total_count: int
    valid_ratio: float
    valid_row_count: int
    coherent_row_count: int
    coherent_pixel_ratio: float
    median_row_depth_mad_m: float
    fit_inlier_row_count: int
    inverse_depth_slope_per_px: float
    fit_residual_m: float
    accepted: bool
    sample_bbox: Optional[BBox]


def _empty_stopline_depth(
    sample_bbox: Optional[BBox] = None,
) -> StopLineDepthMeasurement:
    return StopLineDepthMeasurement(
        camera_z_m=math.nan,
        raw_nonzero_count=0,
        raw_nonzero_ratio=0.0,
        valid_count=0,
        total_count=0,
        valid_ratio=0.0,
        valid_row_count=0,
        coherent_row_count=0,
        coherent_pixel_ratio=0.0,
        median_row_depth_mad_m=math.nan,
        fit_inlier_row_count=0,
        inverse_depth_slope_per_px=math.nan,
        fit_residual_m=math.nan,
        accepted=False,
        sample_bbox=sample_bbox,
    )


def measure_stopline_depth(
    depth_mm: np.ndarray,
    sample_bbox: Optional[BBox],
    target_y_px: float,
    exclusion_mask: Optional[np.ndarray],
    minimum_depth_m: float,
    maximum_depth_m: float,
    minimum_valid_ratio: float,
    minimum_valid_pixels: int,
    minimum_valid_rows: int,
    maximum_row_depth_mad_m: float,
    coherence_absolute_tolerance_m: float,
    coherence_relative_tolerance: float,
    minimum_coherent_pixel_ratio: float,
    minimum_inverse_depth_slope_per_px: float,
    maximum_inverse_depth_slope_per_px: float,
    maximum_fit_residual_m: float,
    minimum_pixels_per_row: int = 3,
) -> StopLineDepthMeasurement:
    """정지선 주변 노면에서 가까운 도색 경계의 optical-Z를 추정한다.

    흰 페인트 픽셀은 stereo 대응점이 부족할 수 있어 ``exclusion_mask``로
    제외한다. 남은 노면의 행별 depth 중앙값에 ``1/Z = a*y + b``를 맞춘
    뒤 정지선의 차량 쪽 경계 y에서 평가한다. 이는 평평한 노면의 원근
    관계를 이용한 국소 보간이며, 차량 앞 범퍼 기준 거리는 아니다.
    """
    if sample_bbox is None or depth_mm.ndim != 2:
        return _empty_stopline_depth(sample_bbox)

    image_height, image_width = depth_mm.shape[:2]
    x1, y1, x2, y2 = sample_bbox
    x1 = max(0, min(int(x1), image_width))
    y1 = max(0, min(int(y1), image_height))
    x2 = max(0, min(int(x2), image_width))
    y2 = max(0, min(int(y2), image_height))
    clipped_bbox = (x1, y1, x2, y2)
    if x2 <= x1 or y2 <= y1:
        return _empty_stopline_depth(clipped_bbox)

    roi = np.asarray(depth_mm[y1:y2, x1:x2], dtype=np.float32)
    eligible = np.ones(roi.shape, dtype=bool)
    if exclusion_mask is not None:
        if exclusion_mask.shape[:2] != depth_mm.shape[:2]:
            return _empty_stopline_depth(clipped_bbox)
        eligible &= exclusion_mask[y1:y2, x1:x2] == 0

    total_count = int(np.count_nonzero(eligible))
    raw_nonzero = eligible & np.isfinite(roi) & (roi > 0.0)
    raw_nonzero_count = int(np.count_nonzero(raw_nonzero))
    raw_nonzero_ratio = raw_nonzero_count / max(1, total_count)

    minimum_depth_mm = minimum_depth_m * 1000.0
    maximum_depth_mm = maximum_depth_m * 1000.0
    valid = (
        eligible
        & np.isfinite(roi)
        & (roi >= minimum_depth_mm)
        & (roi <= maximum_depth_mm)
    )
    valid_count = int(np.count_nonzero(valid))
    valid_ratio = valid_count / max(1, total_count)

    row_positions = []
    row_depths_m = []
    row_depth_mads_m = []
    coherent_pixel_count = 0
    valid_row_count = 0
    for local_y in range(roi.shape[0]):
        values = roi[local_y][valid[local_y]]
        if values.size < minimum_pixels_per_row:
            continue
        valid_row_count += 1
        values_m = values.astype(np.float64) / 1000.0
        row_median_m = float(np.median(values_m))
        row_deviations_m = np.abs(values_m - row_median_m)
        row_mad_m = float(np.median(row_deviations_m))
        row_depth_mads_m.append(row_mad_m)
        coherence_tolerance_m = max(
            coherence_absolute_tolerance_m,
            coherence_relative_tolerance * row_median_m,
        )
        row_coherent_count = int(
            np.count_nonzero(row_deviations_m <= coherence_tolerance_m)
        )
        coherent_pixel_count += row_coherent_count
        row_coherent_ratio = row_coherent_count / float(values.size)
        if (
            row_mad_m > maximum_row_depth_mad_m
            or row_coherent_ratio < minimum_coherent_pixel_ratio
        ):
            continue
        row_positions.append(float(y1 + local_y))
        row_depths_m.append(row_median_m)

    coherent_row_count = len(row_positions)
    coherent_pixel_ratio = coherent_pixel_count / max(1, valid_count)
    median_row_depth_mad_m = (
        float(np.median(row_depth_mads_m))
        if row_depth_mads_m
        else math.nan
    )
    camera_z_m = math.nan
    fit_inlier_row_count = 0
    inverse_depth_slope_per_px = math.nan
    fit_residual_m = math.nan
    fit_prediction_valid = False
    if coherent_row_count > 0:
        camera_z_m = float(np.median(row_depths_m))

    if coherent_row_count >= 2:
        y_values = np.asarray(row_positions, dtype=np.float64)
        z_values = np.asarray(row_depths_m, dtype=np.float64)
        inverse_z = 1.0 / z_values
        coefficients = np.polyfit(y_values, inverse_z, 1)
        residuals = inverse_z - np.polyval(coefficients, y_values)
        residual_center = float(np.median(residuals))
        residual_mad = float(
            np.median(np.abs(residuals - residual_center))
        )
        residual_limit = max(0.005, 3.5 * 1.4826 * residual_mad)
        inliers = np.abs(residuals - residual_center) <= residual_limit
        fit_inlier_row_count = int(np.count_nonzero(inliers))
        if fit_inlier_row_count >= 2:
            coefficients = np.polyfit(
                y_values[inliers],
                inverse_z[inliers],
                1,
            )
            inverse_depth_slope_per_px = float(coefficients[0])
            predicted_inverse_z = float(
                np.polyval(coefficients, float(target_y_px))
            )
            if predicted_inverse_z > 0.0:
                predicted_z_m = 1.0 / predicted_inverse_z
                if minimum_depth_m <= predicted_z_m <= maximum_depth_m:
                    camera_z_m = float(predicted_z_m)
                    fit_prediction_valid = True
            fitted_inverse_z = np.polyval(
                coefficients,
                y_values[inliers],
            )
            if np.all(fitted_inverse_z > 0.0):
                fitted_z = 1.0 / fitted_inverse_z
                fit_residual_m = float(
                    np.median(np.abs(fitted_z - z_values[inliers]))
                )

    accepted = bool(
        math.isfinite(camera_z_m)
        and fit_prediction_valid
        and valid_count >= minimum_valid_pixels
        and valid_ratio >= minimum_valid_ratio
        and coherent_row_count >= minimum_valid_rows
        and coherent_pixel_ratio >= minimum_coherent_pixel_ratio
        and math.isfinite(median_row_depth_mad_m)
        and median_row_depth_mad_m <= maximum_row_depth_mad_m
        and fit_inlier_row_count >= minimum_valid_rows
        and math.isfinite(inverse_depth_slope_per_px)
        and minimum_inverse_depth_slope_per_px
        <= inverse_depth_slope_per_px
        <= maximum_inverse_depth_slope_per_px
        and math.isfinite(fit_residual_m)
        and fit_residual_m <= maximum_fit_residual_m
    )
    return StopLineDepthMeasurement(
        camera_z_m=camera_z_m,
        raw_nonzero_count=raw_nonzero_count,
        raw_nonzero_ratio=raw_nonzero_ratio,
        valid_count=valid_count,
        total_count=total_count,
        valid_ratio=valid_ratio,
        valid_row_count=valid_row_count,
        coherent_row_count=coherent_row_count,
        coherent_pixel_ratio=coherent_pixel_ratio,
        median_row_depth_mad_m=median_row_depth_mad_m,
        fit_inlier_row_count=fit_inlier_row_count,
        inverse_depth_slope_per_px=inverse_depth_slope_per_px,
        fit_residual_m=fit_residual_m,
        accepted=accepted,
        sample_bbox=clipped_bbox,
    )
