"""RGB 영상 하단에서 횡방향 흰색 정지선을 찾는 순수 함수."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

import cv2
import numpy as np


BBox = Tuple[int, int, int, int]


@dataclass(frozen=True)
class StopLineDetection:
    """정지선 후보와 진단용 흰색 마스크."""

    detected: bool
    bbox: Optional[BBox]
    near_edge_y_px: float
    maximum_edge_y_px: float
    score: float
    width_ratio: float
    aspect_ratio: float
    fill_ratio: float
    angle_deg: float
    roi_bbox: BBox
    white_mask: np.ndarray = field(repr=False, compare=False)


def _normalized_roi_to_bbox(
    frame_shape: Tuple[int, ...],
    x_min: float,
    y_min: float,
    x_max: float,
    y_max: float,
) -> BBox:
    frame_height, frame_width = frame_shape[:2]
    x1 = max(0, min(int(round(frame_width * x_min)), frame_width - 1))
    y1 = max(0, min(int(round(frame_height * y_min)), frame_height - 1))
    x2 = max(x1 + 1, min(int(round(frame_width * x_max)), frame_width))
    y2 = max(y1 + 1, min(int(round(frame_height * y_max)), frame_height))
    return x1, y1, x2, y2


def _long_axis_angle(rect) -> float:
    """OpenCV minAreaRect 각도를 긴 변 기준 -90~90도로 정규화한다."""
    (_, _), (width, height), angle = rect
    if width < height:
        angle += 90.0
    while angle > 90.0:
        angle -= 180.0
    while angle <= -90.0:
        angle += 180.0
    return float(angle)


def _near_edge_y_at_center(rect) -> float:
    """회전 사각형 중앙 x에서 차량 쪽(영상 아래쪽) 경계 y를 구한다."""
    center_x = float(rect[0][0])
    points = cv2.boxPoints(rect).astype(np.float64)
    intersections = []
    for index in range(4):
        first = points[index]
        second = points[(index + 1) % 4]
        delta_x = float(second[0] - first[0])
        if abs(delta_x) < 1e-6:
            if abs(center_x - float(first[0])) < 1e-3:
                intersections.extend([float(first[1]), float(second[1])])
            continue
        interpolation = (center_x - float(first[0])) / delta_x
        if -1e-6 <= interpolation <= 1.0 + 1e-6:
            intersections.append(
                float(first[1] + interpolation * (second[1] - first[1]))
            )
    if not intersections:
        return float(rect[0][1])
    return max(intersections)


def _maximum_near_edge_y(rect) -> float:
    """회전 사각형에서 차량에 가장 가까운 최하단 끝점 y를 구한다."""
    points = cv2.boxPoints(rect).astype(np.float64)
    return float(np.max(points[:, 1]))


def _empty_detection(
    roi_bbox: BBox,
    white_mask: np.ndarray,
) -> StopLineDetection:
    return StopLineDetection(
        detected=False,
        bbox=None,
        near_edge_y_px=math.nan,
        maximum_edge_y_px=math.nan,
        score=0.0,
        width_ratio=0.0,
        aspect_ratio=0.0,
        fill_ratio=0.0,
        angle_deg=math.nan,
        roi_bbox=roi_bbox,
        white_mask=white_mask,
    )


def _result_class_name(names, class_id: int) -> str:
    if isinstance(names, dict):
        return str(names.get(class_id, class_id))
    if isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
        return str(names[class_id])
    return str(class_id)


def _normalized_class_name(value: str) -> str:
    return value.lower().replace("_", " ").replace("-", " ").strip()


def _result_scalar(values, index: int) -> float:
    value = values[index]
    if hasattr(value, "item"):
        value = value.item()
    return float(value)


def detect_stop_line_from_yolo_result(
    result,
    frame_shape: Tuple[int, ...],
    roi_bbox: BBox,
    confidence_threshold: float = 0.35,
    class_name: str = "stop_line",
) -> StopLineDetection:
    """YOLO segmentation 결과를 기존 거리 판단용 형식으로 변환한다.

    ``result``는 전체 영상 또는 ``roi_bbox``로 자른 영상에 대한 Ultralytics
    Result다. ``result.orig_shape``가 전체 영상 크기이면 전체 마스크 중 ROI와
    겹치는 부분만 사용한다. 모델은 정지선의 의미를 판별하고, 이 함수는
    마스크에서 차량 쪽 경계와 전체 영상 좌표를 계산한다. 여러 정지선이 있으면
    신뢰도와 영상 아래쪽 위치를 함께 보아 차량에 가까운 후보를 우선한다.
    """
    if len(frame_shape) < 2:
        raise ValueError("frame_shape에는 높이와 폭이 있어야 합니다.")
    if not 0.0 <= confidence_threshold <= 1.0:
        raise ValueError("confidence_threshold는 0~1이어야 합니다.")

    frame_height, frame_width = frame_shape[:2]
    roi_x1, roi_y1, roi_x2, roi_y2 = roi_bbox
    if not (
        0 <= roi_x1 < roi_x2 <= frame_width
        and 0 <= roi_y1 < roi_y2 <= frame_height
    ):
        raise ValueError("roi_bbox가 영상 범위를 벗어났습니다.")
    roi_width = roi_x2 - roi_x1
    roi_height = roi_y2 - roi_y1
    empty_mask = np.zeros((roi_height, roi_width), dtype=np.uint8)

    boxes = getattr(result, "boxes", None) if result is not None else None
    masks = getattr(result, "masks", None) if result is not None else None
    if boxes is None or masks is None:
        return _empty_detection(roi_bbox, empty_mask)

    classes = getattr(boxes, "cls", None)
    confidences = getattr(boxes, "conf", None)
    polygons = getattr(masks, "xy", None)
    if classes is None or confidences is None or polygons is None:
        return _empty_detection(roi_bbox, empty_mask)

    target_name = _normalized_class_name(class_name)
    names = getattr(result, "names", {})
    result_shape = tuple(getattr(result, "orig_shape", (roi_height, roi_width)))
    full_frame_coordinates = result_shape[:2] == (frame_height, frame_width)
    best = None
    best_selection_score = -1.0
    candidate_count = min(len(polygons), len(classes), len(confidences))
    for index in range(candidate_count):
        class_id = int(round(_result_scalar(classes, index)))
        detected_name = _normalized_class_name(
            _result_class_name(names, class_id)
        )
        confidence = _result_scalar(confidences, index)
        if detected_name != target_name or confidence < confidence_threshold:
            continue

        polygon = np.asarray(polygons[index], dtype=np.float32).copy()
        if polygon.ndim != 2 or polygon.shape[0] < 3 or polygon.shape[1] != 2:
            continue
        if not np.isfinite(polygon).all():
            continue
        coordinate_width = frame_width if full_frame_coordinates else roi_width
        coordinate_height = frame_height if full_frame_coordinates else roi_height
        polygon[:, 0] = np.clip(polygon[:, 0], 0, coordinate_width - 1)
        polygon[:, 1] = np.clip(polygon[:, 1], 0, coordinate_height - 1)
        raw_contour = np.rint(polygon).astype(np.int32).reshape((-1, 1, 2))
        if full_frame_coordinates:
            full_mask = np.zeros((frame_height, frame_width), dtype=np.uint8)
            cv2.fillPoly(full_mask, [raw_contour], 255)
            candidate_mask = np.ascontiguousarray(
                full_mask[roi_y1:roi_y2, roi_x1:roi_x2]
            )
            clipped_contours, _ = cv2.findContours(
                candidate_mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            if not clipped_contours:
                continue
            contour = max(clipped_contours, key=cv2.contourArea)
        else:
            contour = raw_contour
            candidate_mask = np.zeros_like(empty_mask)
            cv2.fillPoly(candidate_mask, [contour], 255)
        if not np.any(candidate_mask):
            continue

        x, y, width, height = cv2.boundingRect(contour)
        rect = cv2.minAreaRect(contour)
        (_, _), (rect_width, rect_height), _ = rect
        long_side = max(float(rect_width), float(rect_height))
        short_side = max(1.0, min(float(rect_width), float(rect_height)))
        width_ratio = long_side / float(max(1, roi_width))
        aspect_ratio = long_side / short_side
        fill_ratio = float(np.count_nonzero(candidate_mask)) / float(
            max(1.0, long_side * short_side)
        )
        angle_deg = _long_axis_angle(rect)
        near_edge_y = _near_edge_y_at_center(rect)
        maximum_edge_y = _maximum_near_edge_y(rect)
        near_edge_ratio = maximum_edge_y / float(max(1, roi_height))
        selection_score = 0.75 * confidence + 0.25 * near_edge_ratio
        if selection_score <= best_selection_score:
            continue
        best_selection_score = selection_score
        best = (
            x,
            y,
            width,
            height,
            confidence,
            width_ratio,
            aspect_ratio,
            fill_ratio,
            angle_deg,
            near_edge_y,
            maximum_edge_y,
            candidate_mask,
        )

    if best is None:
        return _empty_detection(roi_bbox, empty_mask)

    (
        x,
        y,
        width,
        height,
        confidence,
        width_ratio,
        aspect_ratio,
        fill_ratio,
        angle_deg,
        near_edge_y,
        maximum_edge_y,
        selected_mask,
    ) = best
    return StopLineDetection(
        detected=True,
        bbox=(
            roi_x1 + x,
            roi_y1 + y,
            roi_x1 + x + width,
            roi_y1 + y + height,
        ),
        near_edge_y_px=float(roi_y1 + near_edge_y),
        maximum_edge_y_px=float(roi_y1 + maximum_edge_y),
        score=float(confidence),
        width_ratio=float(width_ratio),
        aspect_ratio=float(aspect_ratio),
        fill_ratio=float(fill_ratio),
        angle_deg=float(angle_deg),
        roi_bbox=roi_bbox,
        white_mask=selected_mask,
    )


def make_stopline_depth_bbox(
    line_bbox: Optional[BBox],
    image_shape: Tuple[int, ...],
    inner_width_ratio: float,
    band_height_px: int,
    near_edge_y_px: Optional[float] = None,
) -> Optional[BBox]:
    """정지선 가까운 경계 주위의 중앙 노면 depth 표본 영역을 만든다."""
    if line_bbox is None or not 0.0 < inner_width_ratio <= 1.0:
        return None
    if band_height_px < 1:
        return None

    image_height, image_width = image_shape[:2]
    x1, _, x2, y2 = line_bbox
    if x2 <= x1 or y2 <= 0:
        return None
    center_x = 0.5 * (x1 + x2)
    sample_width = max(1.0, (x2 - x1) * inner_width_ratio)
    sample_x1 = max(0, int(math.floor(center_x - 0.5 * sample_width)))
    sample_x2 = min(
        image_width,
        int(math.ceil(center_x + 0.5 * sample_width)),
    )
    near_edge_y = (
        y2 - 1
        if near_edge_y_px is None or not math.isfinite(near_edge_y_px)
        else int(round(near_edge_y_px))
    )
    sample_y1 = max(0, near_edge_y - band_height_px)
    sample_y2 = min(image_height, near_edge_y + band_height_px + 1)
    if sample_x2 <= sample_x1 or sample_y2 <= sample_y1:
        return None
    return sample_x1, sample_y1, sample_x2, sample_y2


def stopline_mask_in_frame(
    detection: StopLineDetection,
    frame_shape: Tuple[int, ...],
) -> np.ndarray:
    """ROI 크기 흰색 마스크를 전체 영상 좌표로 확장한다."""
    frame_mask = np.zeros(frame_shape[:2], dtype=np.uint8)
    x1, y1, x2, y2 = detection.roi_bbox
    expected_shape = (y2 - y1, x2 - x1)
    if detection.white_mask.shape != expected_shape:
        return frame_mask
    frame_mask[y1:y2, x1:x2] = detection.white_mask
    return frame_mask


def stable_stopline_y(
    y_history,
    minimum_samples: int,
    maximum_fit_error_px: float,
    maximum_forward_step_px: float,
    maximum_backward_step_px: float,
) -> Tuple[float, int, bool]:
    """정지선 y의 일정한 전진 움직임까지 허용해 동일 후보인지 확인한다."""
    indexed_values = [
        (index, float(value))
        for index, value in enumerate(y_history)
        if math.isfinite(float(value)) and float(value) >= 0.0
    ]
    if len(indexed_values) < minimum_samples:
        return math.nan, len(indexed_values), False

    frame_indices = np.asarray(
        [item[0] for item in indexed_values],
        dtype=np.float64,
    )
    y_values = np.asarray(
        [item[1] for item in indexed_values],
        dtype=np.float64,
    )
    median_y = float(np.median(y_values))
    if y_values.size == 1:
        return median_y, 1, True

    slope, intercept = np.polyfit(frame_indices, y_values, 1)
    predicted = slope * frame_indices + intercept
    maximum_error = float(np.max(np.abs(y_values - predicted)))
    stable = bool(
        -maximum_backward_step_px <= slope <= maximum_forward_step_px
        and maximum_error <= maximum_fit_error_px
    )
    return median_y, int(y_values.size), stable
