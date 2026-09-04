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


def detect_stop_line(
    frame: np.ndarray,
    roi_x_min: float = 0.08,
    roi_y_min: float = 0.48,
    roi_x_max: float = 0.92,
    roi_y_max: float = 0.98,
    minimum_value: int = 145,
    maximum_saturation: int = 90,
    adaptive_percentile: float = 65.0,
    adaptive_margin: float = 12.0,
    local_contrast_enabled: bool = True,
    local_contrast_minimum_value: int = 60,
    local_contrast_delta: float = 25.0,
    local_contrast_background_ratio: float = 0.12,
    local_contrast_clahe_clip_limit: float = 2.0,
    edge_pair_enabled: bool = True,
    edge_pair_canny_low: int = 35,
    edge_pair_canny_high: int = 110,
    edge_pair_minimum_length_ratio: float = 0.35,
    edge_pair_maximum_angle_difference_deg: float = 4.0,
    edge_pair_minimum_interior_contrast: float = 8.0,
    horizontal_close_ratio: float = 0.015,
    minimum_width_ratio: float = 0.45,
    minimum_aspect_ratio: float = 6.0,
    minimum_fill_ratio: float = 0.30,
    minimum_row_coverage: float = 0.60,
    minimum_thickness_px: int = 3,
    maximum_thickness_ratio: float = 0.20,
    maximum_angle_deg: float = 12.0,
) -> StopLineDetection:
    """길고 연속적인 흰색 횡방향 도색을 정지선으로 선택한다.

    횡단보도의 각 무늬는 폭이 좁고 서로 분리돼 있으므로 하나의 성분 폭과
    가로세로비 조건에서 제외된다. 반환 bbox와 y 좌표는 전체 영상 기준이다.
    ``near_edge_y_px``는 중앙 x의 차량 쪽 경계이고 depth 표본에 사용한다.
    ``maximum_edge_y_px``는 정지 판단에 쓰는 최하단 끝점이다.
    """
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame은 BGR HxWx3 영상이어야 합니다.")

    roi_bbox = _normalized_roi_to_bbox(
        frame.shape,
        roi_x_min,
        roi_y_min,
        roi_x_max,
        roi_y_max,
    )
    roi_x1, roi_y1, roi_x2, roi_y2 = roi_bbox
    roi = np.ascontiguousarray(frame[roi_y1:roi_y2, roi_x1:roi_x2])
    roi_height, roi_width = roi.shape[:2]
    if roi_height < 2 or roi_width < 2:
        return _empty_detection(
            roi_bbox,
            np.zeros((roi_height, roi_width), dtype=np.uint8),
        )

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    adaptive_value = float(np.percentile(value, adaptive_percentile))
    value_threshold = int(
        round(max(minimum_value, min(235.0, adaptive_value + adaptive_margin)))
    )
    absolute_white_mask = np.where(
        (saturation <= maximum_saturation) & (value >= value_threshold),
        255,
        0,
    ).astype(np.uint8)

    # 야간에는 흰색 도색도 절대 밝기 145에 못 미친다. 단순히 그 하한을
    # 낮추면 헤드라이트 얼룩과 노면 전체가 흰색이 되므로, LAB 명도 CLAHE와
    # 넓은 배경 블러의 차이로 "주변 노면보다 밝은 부분"만 보조 후보로 쓴다.
    # 이후의 폭·각도·중앙 통과·연속 프레임 조건은 낮 후보와 동일하게 적용된다.
    local_contrast_mask = np.zeros_like(absolute_white_mask)
    if local_contrast_enabled:
        lightness = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)[:, :, 0]
        clahe = cv2.createCLAHE(
            clipLimit=float(local_contrast_clahe_clip_limit),
            tileGridSize=(8, 8),
        )
        enhanced = clahe.apply(lightness)
        background_kernel = max(
            9,
            int(round(roi_height * local_contrast_background_ratio)),
        )
        if background_kernel % 2 == 0:
            background_kernel += 1
        # 세로 방향 opening은 긴 가로선도 두께가 창보다 얇으면 제거해
        # 주변 노면 밝기를 복원한다. Gaussian blur는 선 중앙에서 배경이
        # 선 밝기에 가까워져 두꺼운 야간 도색을 두 조각으로 만들 수 있다.
        background = cv2.morphologyEx(
            enhanced,
            cv2.MORPH_OPEN,
            np.ones((background_kernel, 3), dtype=np.uint8),
        )
        contrast = cv2.subtract(enhanced, background)
        local_contrast_mask = np.where(
            (saturation <= maximum_saturation)
            & (lightness >= local_contrast_minimum_value)
            & (contrast >= local_contrast_delta),
            255,
            0,
        ).astype(np.uint8)

    white_mask = cv2.bitwise_or(absolute_white_mask, local_contrast_mask)

    # 색/명도 마스크가 약한 경우 정지선의 위·아래 경계 두 개를 보조 후보로
    # 만든다. 한 개짜리 그림자·차선은 제외하고, 서로 평행하며 충분히 길고
    # 그 사이가 위·아래 노면보다 밝은 쌍만 채운 마스크로 변환한다.
    if edge_pair_enabled:
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        edge_gray = cv2.createCLAHE(
            clipLimit=float(local_contrast_clahe_clip_limit),
            tileGridSize=(8, 8),
        ).apply(gray)
        edges = cv2.Canny(
            edge_gray,
            threshold1=int(edge_pair_canny_low),
            threshold2=int(edge_pair_canny_high),
        )
        minimum_edge_length = max(
            10,
            int(round(roi_width * edge_pair_minimum_length_ratio)),
        )
        lines = cv2.HoughLinesP(
            edges,
            rho=1.0,
            theta=np.pi / 180.0,
            threshold=max(20, minimum_edge_length // 4),
            minLineLength=minimum_edge_length,
            maxLineGap=max(8, int(round(roi_width * 0.04))),
        )
        edge_pair_mask = np.zeros_like(white_mask)
        normalized_lines = []
        if lines is not None:
            for raw_line in lines[:, 0, :]:
                x1, y1, x2, y2 = (float(v) for v in raw_line)
                if x2 < x1:
                    x1, x2, y1, y2 = x2, x1, y2, y1
                dx = x2 - x1
                if dx < minimum_edge_length:
                    continue
                angle = math.degrees(math.atan2(y2 - y1, dx))
                if abs(angle) > maximum_angle_deg:
                    continue
                normalized_lines.append((x1, y1, x2, y2, angle))

        maximum_pair_gap = max(
            minimum_thickness_px,
            int(round(roi_height * maximum_thickness_ratio)),
        )
        for first_index, first in enumerate(normalized_lines):
            for second in normalized_lines[first_index + 1:]:
                if abs(first[4] - second[4]) > edge_pair_maximum_angle_difference_deg:
                    continue
                overlap_x1 = max(first[0], second[0])
                overlap_x2 = min(first[2], second[2])
                if overlap_x2 - overlap_x1 < minimum_edge_length:
                    continue
                center_x = 0.5 * (overlap_x1 + overlap_x2)

                def y_at(line, x_value):
                    return line[1] + (
                        (x_value - line[0])
                        * (line[3] - line[1])
                        / max(1.0, line[2] - line[0])
                    )

                first_y = y_at(first, center_x)
                second_y = y_at(second, center_x)
                top, bottom = (
                    (first, second) if first_y <= second_y else (second, first)
                )
                gap = abs(second_y - first_y)
                if not minimum_thickness_px <= gap <= maximum_pair_gap:
                    continue
                xs = np.arange(
                    max(0, int(math.floor(overlap_x1))),
                    min(roi_width, int(math.ceil(overlap_x2)) + 1),
                )
                if xs.size < minimum_edge_length:
                    continue
                top_y = np.asarray([y_at(top, float(x)) for x in xs])
                bottom_y = np.asarray([y_at(bottom, float(x)) for x in xs])
                middle_y = np.clip(
                    np.rint(0.5 * (top_y + bottom_y)).astype(int),
                    0,
                    roi_height - 1,
                )
                band_offset = max(2, int(round(gap)))
                above_y = np.clip(np.rint(top_y).astype(int) - band_offset, 0, roi_height - 1)
                below_y = np.clip(np.rint(bottom_y).astype(int) + band_offset, 0, roi_height - 1)
                interior_mean = float(np.mean(gray[middle_y, xs]))
                exterior_mean = 0.5 * float(
                    np.mean(gray[above_y, xs]) + np.mean(gray[below_y, xs])
                )
                if interior_mean - exterior_mean < edge_pair_minimum_interior_contrast:
                    continue
                polygon = np.array(
                    [
                        [overlap_x1, y_at(top, overlap_x1)],
                        [overlap_x2, y_at(top, overlap_x2)],
                        [overlap_x2, y_at(bottom, overlap_x2)],
                        [overlap_x1, y_at(bottom, overlap_x1)],
                    ],
                    dtype=np.int32,
                )
                cv2.fillConvexPoly(edge_pair_mask, polygon, 255)
        white_mask = cv2.bitwise_or(white_mask, edge_pair_mask)

    white_mask = cv2.morphologyEx(
        white_mask,
        cv2.MORPH_OPEN,
        np.ones((3, 3), dtype=np.uint8),
    )
    close_width = max(3, int(round(roi_width * horizontal_close_ratio)))
    if close_width % 2 == 0:
        close_width += 1
    white_mask = cv2.morphologyEx(
        white_mask,
        cv2.MORPH_CLOSE,
        np.ones((3, close_width), dtype=np.uint8),
    )

    contours, _ = cv2.findContours(
        white_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    best = None
    best_score = -1.0
    for contour in contours:
        x, y, width, height = cv2.boundingRect(contour)
        if width <= 0 or height < minimum_thickness_px:
            continue

        rect = cv2.minAreaRect(contour)
        (_, _), (rect_width, rect_height), _ = rect
        long_side = max(rect_width, rect_height)
        short_side = max(1.0, min(rect_width, rect_height))
        width_ratio = long_side / float(roi_width)
        thickness_ratio = short_side / float(roi_height)
        aspect_ratio = long_side / short_side
        if width_ratio < minimum_width_ratio:
            continue
        if not x <= 0.5 * roi_width <= x + width:
            continue
        if aspect_ratio < minimum_aspect_ratio:
            continue
        if thickness_ratio > maximum_thickness_ratio:
            continue

        angle_deg = _long_axis_angle(rect)
        if abs(angle_deg) > maximum_angle_deg:
            continue

        contour_area = float(cv2.contourArea(contour))
        fill_ratio = contour_area / float(max(1.0, long_side * short_side))
        if fill_ratio < minimum_fill_ratio:
            continue

        component = white_mask[y:y + height, x:x + width]
        row_counts = np.count_nonzero(component, axis=1)
        row_coverage = float(np.max(row_counts)) / float(max(1, width))
        angle_radians = math.radians(abs(angle_deg))
        expected_row_coverage = min(
            1.0,
            short_side
            / max(
                1.0,
                short_side + long_side * math.sin(angle_radians),
            ),
        )
        adjusted_row_coverage = min(
            1.0,
            row_coverage / max(0.05, expected_row_coverage),
        )
        if adjusted_row_coverage < minimum_row_coverage:
            continue

        # 긴 폭과 화면 아래쪽 위치를 우선한다. aspect는 너무 큰 값이
        # 점수를 독점하지 않도록 12에서 포화시킨다.
        near_edge_y = _near_edge_y_at_center(rect)
        maximum_edge_y = _maximum_near_edge_y(rect)
        near_edge_ratio = near_edge_y / float(roi_height)
        aspect_score = min(1.0, aspect_ratio / 12.0)
        score = (
            0.35 * width_ratio
            + 0.20 * aspect_score
            + 0.15 * min(1.0, fill_ratio)
            + 0.10 * adjusted_row_coverage
            + 0.20 * near_edge_ratio
        )
        if score > best_score:
            best_score = score
            best = (
                x,
                y,
                width,
                height,
                width_ratio,
                aspect_ratio,
                fill_ratio,
                angle_deg,
                near_edge_y,
                maximum_edge_y,
            )

    if best is None:
        return _empty_detection(roi_bbox, white_mask)

    (
        x,
        y,
        width,
        height,
        width_ratio,
        aspect_ratio,
        fill_ratio,
        angle_deg,
        near_edge_y,
        maximum_edge_y,
    ) = best
    bbox = (
        roi_x1 + x,
        roi_y1 + y,
        roi_x1 + x + width,
        roi_y1 + y + height,
    )
    return StopLineDetection(
        detected=True,
        bbox=bbox,
        near_edge_y_px=float(roi_y1 + near_edge_y),
        maximum_edge_y_px=float(roi_y1 + maximum_edge_y),
        score=float(best_score),
        width_ratio=float(width_ratio),
        aspect_ratio=float(aspect_ratio),
        fill_ratio=float(fill_ratio),
        angle_deg=float(angle_deg),
        roi_bbox=roi_bbox,
        white_mask=white_mask,
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
