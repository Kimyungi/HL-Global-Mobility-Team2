"""YOLO 검출 사이의 짧은 공백을 잇는 경량 영상 추적기."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


BBox = Tuple[int, int, int, int]


def _clip_bbox(bbox: BBox, frame_shape: Tuple[int, ...]) -> BBox:
    frame_height, frame_width = frame_shape[:2]
    x1, y1, x2, y2 = bbox
    x1 = max(0, min(int(x1), frame_width - 1))
    y1 = max(0, min(int(y1), frame_height - 1))
    x2 = max(x1 + 1, min(int(x2), frame_width))
    y2 = max(y1 + 1, min(int(y2), frame_height))
    return x1, y1, x2, y2


def scale_bbox(
    bbox: BBox,
    frame_shape: Tuple[int, ...],
    scale: float,
) -> BBox:
    """bbox 중심은 유지한 채 폭과 높이를 배율만큼 확장한다."""
    if scale <= 0.0:
        raise ValueError("bbox scale은 0보다 커야 합니다.")
    x1, y1, x2, y2 = bbox
    center_x = 0.5 * (x1 + x2)
    center_y = 0.5 * (y1 + y2)
    half_width = max(0.5, 0.5 * (x2 - x1) * scale)
    half_height = max(0.5, 0.5 * (y2 - y1) * scale)
    return _clip_bbox(
        (
            int(round(center_x - half_width)),
            int(round(center_y - half_height)),
            int(round(center_x + half_width)),
            int(round(center_y + half_height)),
        ),
        frame_shape,
    )


def smooth_bbox(
    previous_bbox: Optional[BBox],
    detected_bbox: BBox,
    frame_shape: Tuple[int, ...],
    current_weight: float,
) -> BBox:
    """새 YOLO bbox를 이전 bbox와 EMA로 섞어 좌표 흔들림을 줄인다."""
    if not 0.0 < current_weight <= 1.0:
        raise ValueError("bbox smoothing weight는 0 초과 1 이하여야 합니다.")
    if previous_bbox is None:
        return _clip_bbox(detected_bbox, frame_shape)
    blended = tuple(
        int(round((1.0 - current_weight) * old + current_weight * new))
        for old, new in zip(previous_bbox, detected_bbox)
    )
    return _clip_bbox(blended, frame_shape)


@dataclass(frozen=True)
class TemplateTrackResult:
    bbox: Optional[BBox] = None
    score: float = 0.0


class ShortTermTemplateTracker:
    """고정 템플릿으로 YOLO miss 몇 프레임만 이어 주는 추적기.

    템플릿은 YOLO가 실제로 검출한 프레임에서만 갱신한다. 추적 결과로
    템플릿을 다시 만들지 않으므로 장시간 drift가 누적되지 않는다.
    """

    def __init__(
        self,
        context_scale: float = 1.8,
        search_scale: float = 2.5,
        minimum_score: float = 0.72,
        maximum_center_shift_ratio: float = 0.75,
        minimum_template_stddev: float = 5.0,
    ) -> None:
        if context_scale < 1.0:
            raise ValueError("template context scale은 1 이상이어야 합니다.")
        if search_scale <= 1.0:
            raise ValueError("template search scale은 1보다 커야 합니다.")
        if not 0.0 <= minimum_score <= 1.0:
            raise ValueError("template minimum score는 0~1이어야 합니다.")
        if maximum_center_shift_ratio <= 0.0:
            raise ValueError("template 최대 중심 이동 비율은 0보다 커야 합니다.")
        if minimum_template_stddev < 0.0:
            raise ValueError("template 표준편차 임계값은 0 이상이어야 합니다.")

        self.context_scale = float(context_scale)
        self.search_scale = float(search_scale)
        self.minimum_score = float(minimum_score)
        self.maximum_center_shift_ratio = float(
            maximum_center_shift_ratio
        )
        self.minimum_template_stddev = float(minimum_template_stddev)
        self.reset()

    @property
    def ready(self) -> bool:
        return self._template is not None

    def reset(self) -> None:
        self._template: Optional[np.ndarray] = None
        self._context_bbox: Optional[BBox] = None
        self._object_offset: Optional[BBox] = None
        self._last_bbox: Optional[BBox] = None

    def initialize(self, frame: np.ndarray, bbox: BBox) -> bool:
        if frame is None or frame.size == 0:
            self.reset()
            return False

        object_bbox = _clip_bbox(bbox, frame.shape)
        context_bbox = scale_bbox(
            object_bbox,
            frame.shape,
            self.context_scale,
        )
        context_x1, context_y1, context_x2, context_y2 = context_bbox
        context = frame[context_y1:context_y2, context_x1:context_x2]
        if context.shape[0] < 3 or context.shape[1] < 3:
            self.reset()
            return False

        gray = cv2.cvtColor(context, cv2.COLOR_BGR2GRAY)
        if float(np.std(gray)) < self.minimum_template_stddev:
            self.reset()
            return False

        x1, y1, x2, y2 = object_bbox
        self._template = np.ascontiguousarray(gray)
        self._context_bbox = context_bbox
        self._object_offset = (
            x1 - context_x1,
            y1 - context_y1,
            x2 - context_x1,
            y2 - context_y1,
        )
        self._last_bbox = object_bbox
        return True

    def track(self, frame: np.ndarray) -> TemplateTrackResult:
        if (
            not self.ready
            or self._context_bbox is None
            or self._object_offset is None
            or self._last_bbox is None
            or frame is None
            or frame.size == 0
        ):
            return TemplateTrackResult()

        search_bbox = scale_bbox(
            self._context_bbox,
            frame.shape,
            self.search_scale,
        )
        search_x1, search_y1, search_x2, search_y2 = search_bbox
        search = frame[search_y1:search_y2, search_x1:search_x2]
        search_gray = cv2.cvtColor(search, cv2.COLOR_BGR2GRAY)
        template_height, template_width = self._template.shape[:2]
        if (
            search_gray.shape[0] < template_height
            or search_gray.shape[1] < template_width
        ):
            return TemplateTrackResult()

        response = cv2.matchTemplate(
            search_gray,
            self._template,
            cv2.TM_CCOEFF_NORMED,
        )
        _, maximum_score, _, maximum_location = cv2.minMaxLoc(response)
        score = float(maximum_score)
        if not math.isfinite(score) or score < self.minimum_score:
            return TemplateTrackResult(score=max(0.0, score))

        context_x1 = search_x1 + maximum_location[0]
        context_y1 = search_y1 + maximum_location[1]
        new_context_bbox = (
            context_x1,
            context_y1,
            context_x1 + template_width,
            context_y1 + template_height,
        )
        offset_x1, offset_y1, offset_x2, offset_y2 = self._object_offset
        candidate_bbox = _clip_bbox(
            (
                context_x1 + offset_x1,
                context_y1 + offset_y1,
                context_x1 + offset_x2,
                context_y1 + offset_y2,
            ),
            frame.shape,
        )

        old_x1, old_y1, old_x2, old_y2 = self._last_bbox
        new_x1, new_y1, new_x2, new_y2 = candidate_bbox
        old_width = max(1, old_x2 - old_x1)
        old_height = max(1, old_y2 - old_y1)
        old_diagonal = max(1.0, math.hypot(old_width, old_height))
        center_shift_ratio = math.hypot(
            0.5 * (new_x1 + new_x2 - old_x1 - old_x2),
            0.5 * (new_y1 + new_y2 - old_y1 - old_y2),
        ) / old_diagonal
        if center_shift_ratio > self.maximum_center_shift_ratio:
            return TemplateTrackResult(score=score)

        self._context_bbox = new_context_bbox
        self._last_bbox = candidate_bbox
        return TemplateTrackResult(bbox=candidate_bbox, score=score)
