"""YOLO segmentation 결과를 기존 정지선 거리 파이프라인 형식으로 변환한다."""

from __future__ import annotations

import math
from typing import Iterable

import cv2
import numpy as np


from stack_traffic.stopline_detector import StopLineDetection


def _class_name(names, class_id: int) -> str:
    if isinstance(names, dict):
        return str(names.get(class_id, class_id))
    if isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
        return str(names[class_id])
    return str(class_id)


def stopline_class_ids(names) -> list[int]:
    """모델 metadata에서 stop_line 클래스 ID를 찾는다."""
    items: Iterable[tuple[int, object]]
    if isinstance(names, dict):
        items = names.items()
    else:
        items = enumerate(names)
    return [
        int(class_id)
        for class_id, name in items
        if str(name).lower().replace("-", "_").strip() == "stop_line"
    ]


def _empty(frame_shape, roi_bbox) -> StopLineDetection:
    x1, y1, x2, y2 = roi_bbox
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
        white_mask=np.zeros((y2 - y1, x2 - x1), dtype=np.uint8),
    )


def detect_stop_line_yolo(
    model,
    frame: np.ndarray,
    class_ids: list[int],
    confidence_threshold: float = 0.10,
    image_size: int = 640,
    roi_x_min: float = 0.08,
    roi_y_min: float = 0.48,
    roi_x_max: float = 0.92,
    roi_y_max: float = 0.98,
) -> StopLineDetection:
    """가장 가까운 stop_line mask를 ``StopLineDetection``으로 변환한다."""
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame은 BGR HxWx3 영상이어야 합니다.")
    if not class_ids:
        raise ValueError("YOLO 모델에 stop_line 클래스가 없습니다.")

    height, width = frame.shape[:2]
    x1 = max(0, min(int(round(width * roi_x_min)), width - 1))
    y1 = max(0, min(int(round(height * roi_y_min)), height - 1))
    x2 = max(x1 + 1, min(int(round(width * roi_x_max)), width))
    y2 = max(y1 + 1, min(int(round(height * roi_y_max)), height))
    roi_bbox = (x1, y1, x2, y2)

    result = model.predict(
        source=frame,
        imgsz=image_size,
        conf=confidence_threshold,
        classes=class_ids,
        device="cpu",
        retina_masks=True,
        verbose=False,
    )[0]
    if result.boxes is None or result.masks is None:
        return _empty(frame.shape, roi_bbox)

    masks = result.masks.data.cpu().numpy()
    confidences = result.boxes.conf.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy().astype(int)
    candidates = []
    for mask, confidence, class_id in zip(masks, confidences, classes):
        if class_id not in class_ids:
            continue
        if mask.shape != (height, width):
            mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
        binary = mask >= 0.5
        clipped = np.zeros((height, width), dtype=np.uint8)
        clipped[y1:y2, x1:x2] = binary[y1:y2, x1:x2].astype(np.uint8) * 255
        ys, xs = np.nonzero(clipped)
        if xs.size < 20:
            continue
        candidates.append((int(np.max(ys)), float(confidence), clipped, xs, ys))

    if not candidates:
        return _empty(frame.shape, roi_bbox)

    _, confidence, selected, xs, ys = max(candidates, key=lambda item: (item[0], item[1]))
    contours, _ = cv2.findContours(selected, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = max(contours, key=cv2.contourArea)
    bx, by, bw, bh = cv2.boundingRect(contour)
    rect = cv2.minAreaRect(contour)
    rect_width, rect_height = rect[1]
    long_side = max(rect_width, rect_height)
    short_side = max(1.0, min(rect_width, rect_height))
    angle = float(rect[2] + (90.0 if rect_width < rect_height else 0.0))
    while angle > 90.0:
        angle -= 180.0
    while angle <= -90.0:
        angle += 180.0

    center_x = int(round(float(np.median(xs))))
    center_band = selected[:, max(0, center_x - 2):min(width, center_x + 3)]
    center_ys = np.nonzero(center_band)[0]
    near_y = float(np.max(center_ys)) if center_ys.size else float(np.max(ys))
    fill_ratio = float(np.count_nonzero(selected[by:by + bh, bx:bx + bw])) / max(1, bw * bh)
    return StopLineDetection(
        detected=True,
        bbox=(bx, by, bx + bw, by + bh),
        near_edge_y_px=near_y,
        maximum_edge_y_px=float(np.max(ys)),
        score=confidence,
        width_ratio=float(long_side) / max(1, x2 - x1),
        aspect_ratio=float(long_side) / short_side,
        fill_ratio=fill_ratio,
        angle_deg=angle,
        roi_bbox=roi_bbox,
        white_mask=selected[y1:y2, x1:x2].copy(),
    )
