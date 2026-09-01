#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""재민님의 traffic_red_binary_test.py를 ROS 2에 연결한 신호등 정지 노드.

신호등 판정 로직은 가벼운 HSV 색상 판정을 사용한다.
- YOLOv8n으로 traffic light 위치 검출
- HSV로 red_raw/green_raw 판정
- 최근 5프레임 중 빨간불 3프레임 이상이면 red_active
- 하단 RGB에서 정지선을 찾고 정렬 depth로 차량 쪽 경계 거리를 직접 측정
- 확정 red_active를 fresh 초록까지 페이즈 래치
- red_phase_latched AND 활성화된 접근 임계값이면 정지 래치
- 정지 래치는 fresh bbox 또는 확정 적색 anchor 안의 초록색 3/5에서만 해제

결과는 /perception/traffic_stop (TrafficStop)으로 발행한다. ADAS MGM이
stop_required를 v_ref=0으로 병합하고 bridge_dspace가 CAN으로 전송한다.
"""

from __future__ import annotations

# ★ 반드시 torch/ultralytics(아래 cv2·YOLO) 보다 **먼저** — OpenMP 워커가 놀 때
#   도는 것을 막는다. 이유·실측은 omp_runtime 모듈 docstring 참조.
from stack_traffic import omp_runtime  # noqa: F401  (import 만으로 동작)

import math
import time
import traceback
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Optional, Tuple, Union

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from fma_interfaces.msg import TrafficStop
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from stack_traffic.depth_utils import (
    StopLineDepthMeasurement,
    measure_stopline_depth,
)
from stack_traffic.logic import (
    camera_poll_timed_out,
    classify_color_ratios,
    combine_stopline_proximity,
    frame_bbox_to_roi,
    is_red_clear_confirmed,
    is_stopline_approaching,
    is_stopline_y_approaching,
    normalized_roi_to_bbox,
    robust_nonnegative_median,
    roi_bbox_to_frame,
    select_horizontal_roi_tile,
    select_tracking_candidate,
    should_clear_visual_track,
    should_accept_anchored_green,
    should_record_color_vote,
    update_red_phase_latch,
    update_stop_latch,
)
from stack_traffic.oak_camera import (
    OakRgbdCamera,
    normalize_oak_usb_speed,
)
from stack_traffic.stopline_detector import (
    StopLineDetection,
    detect_stop_line,
    make_stopline_depth_bbox,
    stable_stopline_y,
    stopline_mask_in_frame,
)
from stack_traffic.visual_tracker import (
    ShortTermTemplateTracker,
    smooth_bbox,
)

YOLO_IMPORT_ERROR = None
try:
    from ultralytics import YOLO
except Exception as error:
    YOLO = None
    YOLO_IMPORT_ERROR = error


CameraSource = Union[int, str]
BBox = Tuple[int, int, int, int]


@dataclass(frozen=True)
class StopLineRuntime:
    """한 프레임의 정지선 검출·거리 필터 상태."""

    detection: Optional[StopLineDetection]
    depth: StopLineDepthMeasurement
    current_camera_z_m: float
    median_camera_z_m: float
    valid_distance_samples: int
    median_y_px: float
    current_y_ratio: float
    median_y_ratio: float
    valid_y_samples: int
    stable: bool
    depth_near: bool
    y_near: bool
    near: bool


def parse_camera_source(value: str) -> CameraSource:
    value = value.strip()
    if value.lstrip("-").isdigit():
        return int(value)
    return value


def should_run_yolo(frame_index: int, inference_interval: int) -> bool:
    """첫 프레임부터 일정 간격으로 YOLO를 실행한다."""
    if frame_index < 1 or inference_interval < 1:
        raise ValueError(
            "frame_index와 inference_interval은 1 이상이어야 합니다."
        )
    return (frame_index - 1) % inference_interval == 0


def get_class_name(names, class_id: int) -> str:
    if isinstance(names, dict):
        return str(names.get(class_id, class_id))
    if isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
        return str(names[class_id])
    return str(class_id)


def is_traffic_light_class(class_name: str) -> bool:
    normalized = class_name.lower().replace("_", " ").strip()
    return "traffic light" in normalized


def get_traffic_light_class_ids(names) -> list[int]:
    """모델 label에서 traffic-light class ID만 추출한다."""
    if isinstance(names, dict):
        items = names.items()
    elif isinstance(names, (list, tuple)):
        items = enumerate(names)
    else:
        return []
    return sorted(
        int(class_id)
        for class_id, class_name in items
        if is_traffic_light_class(str(class_name))
    )


def choose_target_traffic_light(
    results,
    frame_shape: Tuple[int, ...],
    confidence_threshold: float,
    minimum_box_area: int,
    previous_bbox: Optional[BBox] = None,
    tracking_confidence_threshold: float = 0.10,
    tracking_minimum_iou: float = 0.10,
    tracking_maximum_center_shift_ratio: float = 0.50,
    tracking_minimum_size_similarity: float = 0.50,
    minimum_box_width_height_ratio: float = 0.0,
) -> Tuple[Optional[BBox], float]:
    """신규 신호등을 선택하거나 이전 bbox와 이어지는 후보를 추적한다."""
    if not results:
        return None, 0.0

    result = results[0]
    if result.boxes is None:
        return None, 0.0

    frame_height, frame_width = frame_shape[:2]
    frame_area = float(frame_height * frame_width)
    frame_diagonal = max(1.0, math.hypot(frame_width, frame_height))

    best_bbox: Optional[BBox] = None
    best_confidence = 0.0
    best_score = -1.0
    tracking_candidates = []
    candidate_threshold = (
        confidence_threshold
        if previous_bbox is None
        else tracking_confidence_threshold
    )

    for box in result.boxes:
        confidence = float(box.conf[0].item())
        if confidence < candidate_threshold:
            continue

        class_id = int(box.cls[0].item())
        class_name = get_class_name(result.names, class_id)

        if not is_traffic_light_class(class_name):
            continue

        x1, y1, x2, y2 = (
            box.xyxy[0].detach().cpu().numpy().astype(int).tolist()
        )

        x1 = max(0, min(x1, frame_width - 1))
        y1 = max(0, min(y1, frame_height - 1))
        x2 = max(1, min(x2, frame_width))
        y2 = max(1, min(y2, frame_height))

        if x2 <= x1 or y2 <= y1:
            continue

        box_width = x2 - x1
        box_height = y2 - y1
        box_area = box_width * box_height

        if box_area < minimum_box_area:
            continue
        if (
            box_width / max(1, box_height)
            < minimum_box_width_height_ratio
        ):
            continue

        bbox = (x1, y1, x2, y2)
        if previous_bbox is not None:
            tracking_candidates.append((bbox, confidence))
            continue

        center_x = 0.5 * (x1 + x2)
        center_y = 0.5 * (y1 + y2)

        center_distance = math.hypot(
            center_x - frame_width * 0.5,
            center_y - frame_height * 0.35,
        )
        center_score = max(
            0.0,
            1.0 - center_distance / frame_diagonal,
        )
        upper_score = max(
            0.0,
            1.0 - center_y / frame_height,
        )
        area_score = min(
            1.0,
            math.sqrt(box_area / frame_area) * 8.0,
        )

        score = (
            0.65 * confidence
            + 0.15 * center_score
            + 0.10 * upper_score
            + 0.10 * area_score
        )

        if score > best_score:
            best_score = score
            best_bbox = bbox
            best_confidence = confidence

    if previous_bbox is not None:
        tracked = select_tracking_candidate(
            tracking_candidates,
            previous_bbox,
            minimum_iou=tracking_minimum_iou,
            maximum_center_shift_ratio=(
                tracking_maximum_center_shift_ratio
            ),
            minimum_size_similarity=tracking_minimum_size_similarity,
        )
        if tracked is None:
            return None, 0.0
        return tracked

    return best_bbox, best_confidence


def classify_signal_color(
    frame: np.ndarray,
    bbox: Optional[BBox],
    minimum_red_ratio: float,
    minimum_green_ratio: float,
    red_hue_upper: int = 25,
    red_hue_high_lower: int = 165,
    minimum_color_saturation: int = 45,
    minimum_color_value: int = 60,
) -> Tuple[
    int,
    int,
    float,
    float,
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    """
    반환값
    - red_raw: 빨간불이면 1, 아니면 0
    - green_raw: 초록불이면 1, 아니면 0
    - red_ratio: ROI 전체 중 빨간색 픽셀 비율
    - green_ratio: ROI 전체 중 초록색 픽셀 비율
    - crop: 신호등 검출 영역
    - red_mask: 빨간색 마스크
    - green_mask: 초록색 마스크
    """
    if bbox is None:
        empty_crop = frame[0:1, 0:1]
        empty_mask = np.zeros((1, 1), dtype=np.uint8)
        return 0, 0, 0.0, 0.0, empty_crop, empty_mask, empty_mask

    x1, y1, x2, y2 = bbox
    crop = frame[y1:y2, x1:x2]

    if crop.size == 0:
        empty_crop = frame[0:1, 0:1]
        empty_mask = np.zeros((1, 1), dtype=np.uint8)
        return 0, 0, 0.0, 0.0, empty_crop, empty_mask, empty_mask

    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

    red_mask_1 = cv2.inRange(
        hsv,
        np.array(
            [0, minimum_color_saturation, minimum_color_value],
            dtype=np.uint8,
        ),
        np.array([red_hue_upper, 255, 255], dtype=np.uint8),
    )
    red_mask_2 = cv2.inRange(
        hsv,
        np.array(
            [
                red_hue_high_lower,
                minimum_color_saturation,
                minimum_color_value,
            ],
            dtype=np.uint8,
        ),
        np.array([179, 255, 255], dtype=np.uint8),
    )
    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)
    green_mask = cv2.inRange(
        hsv,
        np.array(
            [35, minimum_color_saturation, minimum_color_value],
            dtype=np.uint8,
        ),
        np.array([95, 255, 255], dtype=np.uint8),
    )

    kernel = np.ones((3, 3), dtype=np.uint8)
    red_mask = cv2.morphologyEx(
        red_mask,
        cv2.MORPH_CLOSE,
        kernel,
    )
    green_mask = cv2.morphologyEx(
        green_mask,
        cv2.MORPH_CLOSE,
        kernel,
    )

    roi_pixels = max(1, crop.shape[0] * crop.shape[1])
    red_ratio = cv2.countNonZero(red_mask) / roi_pixels
    green_ratio = cv2.countNonZero(green_mask) / roi_pixels

    red_detected, green_detected = classify_color_ratios(
        red_ratio,
        green_ratio,
        minimum_red_ratio,
        minimum_green_ratio,
    )

    return (
        int(red_detected),
        int(green_detected),
        red_ratio,
        green_ratio,
        crop,
        red_mask,
        green_mask,
    )


class StackTrafficNode(Node):
    def __init__(self) -> None:
        super().__init__("stack_traffic_node")
        self._declare_parameters()
        self._load_parameters()

        if YOLO is None:
            raise RuntimeError(
                "YOLO 런타임을 불러오지 못했습니다. "
                "`ros2 run stack_traffic stack_traffic_ml_preflight`로 "
                "torch/torchvision/ultralytics 조합을 확인하세요. "
                f"원본 오류: {type(YOLO_IMPORT_ERROR).__name__}: "
                f"{YOLO_IMPORT_ERROR}"
            ) from YOLO_IMPORT_ERROR

        model_path = self._resolve_model_path()
        if not model_path.exists():
            raise FileNotFoundError(
                f"YOLO 모델 파일을 찾을 수 없습니다: {model_path}. "
                "models/yolov8n.pt에 배치하거나 "
                "-p model_path:=/path/to/model.pt로 지정하세요."
            )

        self.model = YOLO(str(model_path))
        self.traffic_light_class_ids = get_traffic_light_class_ids(
            self.model.names
        )
        if not self.traffic_light_class_ids:
            self.get_logger().warning(
                "모델 label에서 traffic light class를 찾지 못해 "
                "전체 class 추론을 사용합니다."
            )
        self.camera_source = parse_camera_source(self.camera_source_text)
        self.capture = None
        self.oak_camera = None
        self._open_camera()

        self.publisher = self.create_publisher(
            TrafficStop, "/perception/traffic_stop", 1
        )

        self.red_history: Deque[int] = deque(maxlen=self.vote_window)
        self.green_history: Deque[int] = deque(maxlen=self.vote_window)
        self.bbox_observed_history: Deque[int] = deque(
            maxlen=self.vote_window
        )
        self.red_fresh_seeded = False
        self.stopline_y_history: Deque[float] = deque(
            [math.nan] * self.stopline_detection_window,
            maxlen=self.stopline_detection_window,
        )
        self.stopline_distance_history: Deque[float] = deque(
            [math.nan] * self.stopline_depth_window,
            maxlen=self.stopline_depth_window,
        )
        # YOLO miss에는 단순 stale 좌표가 아니라 검증된 짧은 template
        # 추적 결과만 색 판정과 bbox 표시를 이어 가는 데 사용한다.
        self.tracked_bbox: Optional[BBox] = None
        self.stop_target_bbox: Optional[BBox] = None
        self.red_phase_target_bbox: Optional[BBox] = None
        self.tracking_missed_frames = 0
        self.tracking_age_frames = 0
        self.template_tracking_failed_frames = 0
        self.template_tracker = ShortTermTemplateTracker(
            context_scale=self.template_tracking_context_scale,
            search_scale=self.template_tracking_search_scale,
            minimum_score=self.template_tracking_minimum_score,
            maximum_center_shift_ratio=(
                self.template_tracking_maximum_center_shift_ratio
            ),
        )
        self.stop_required_latched = False
        self.red_phase_latched = False
        self.camera_fault_latched = False
        self.startup_hold_latched = bool(
            self.stopline_detection_enabled
            and (
                self.stopline_stop_y_ratio > 0.0
                or self.stopline_stop_distance_m > 0.0
            )
        )
        self.startup_yolo_runs = 0
        self.startup_minimum_frames = max(
            self.stopline_detection_window,
            (
                self.stopline_depth_window
                if self.stopline_stop_distance_m > 0.0
                else 0
            ),
        )
        self.frame_index = 0
        self.detection_scan_index = 0
        self.last_detection_tile_bbox: Optional[BBox] = None
        self.last_camera_success_monotonic = time.monotonic()
        self.depth_resize_logged = False
        self.previous_time = time.perf_counter()
        self.filtered_fps = 0.0
        self.timer = self.create_timer(self.process_period_sec, self.tick)

        self.get_logger().info(
            "traffic_red_binary ROS 2 started | "
            f"model={model_path} camera={self._camera_description()} "
            f"red_vote={self.vote_window}/{self.minimum_red_votes} "
            f"green_vote={self.vote_window}/{self.minimum_green_votes} "
            f"stopline={int(self.stopline_detection_enabled)} "
            f"stopline_roi=[{self.stopline_roi_x_min:.2f},"
            f"{self.stopline_roi_y_min:.2f}-"
            f"{self.stopline_roi_x_max:.2f},"
            f"{self.stopline_roi_y_max:.2f}] "
            f"stopline_stop={self.stopline_stop_distance_m:.2f}m "
            f"stopline_y_stop={self.stopline_stop_y_ratio:.3f} "
            f"stopline_slope="
            f"{self.stopline_minimum_inverse_depth_slope_per_px:.4f}-"
            f"{self.stopline_maximum_inverse_depth_slope_per_px:.3f} "
            f"stopline_fit_err="
            f"{self.stopline_maximum_fit_residual_m:.2f}m "
            f"yolo_imgsz={self.yolo_image_size} "
            f"yolo_interval={self.yolo_inference_interval} "
            f"yolo_classes={self.traffic_light_class_ids or 'all'} "
            f"detect_roi={self.detection_roi_enabled}["
            f"{self.detection_roi_x_min:.2f},"
            f"{self.detection_roi_y_min:.2f}-"
            f"{self.detection_roi_x_max:.2f},"
            f"{self.detection_roi_y_max:.2f}] "
            f"detect_tile_width={self.detection_tile_width_ratio:.2f} "
            f"min_box_ratio={self.minimum_box_width_height_ratio:.2f} "
            f"red_hue=0-{self.red_hue_upper}/"
            f"{self.red_hue_high_lower}-179 "
            f"oak_depth={int(self.oak_depth_enabled)} "
            f"oak_depth_cfg=conf{self.oak_depth_confidence_threshold}/"
            f"lr{int(self.oak_depth_left_right_check)}/"
            f"subpixel{int(self.oak_depth_subpixel)}/"
            f"median{self.oak_depth_median_filter_size}/"
            f"decimation{self.oak_depth_decimation_factor}/"
            f"speckle{int(self.oak_depth_speckle_filter)}/"
            f"spatial{int(self.oak_depth_spatial_filter)}/"
            f"temporal{int(self.oak_depth_temporal_filter)} "
            f"detect_conf={self.confidence_threshold:.2f} "
            f"track_conf={self.tracking_confidence_threshold:.2f} "
            f"track_miss_max={self.tracking_max_missed_frames} "
            f"bbox_ema={self.bbox_smoothing_current_weight:.2f} "
            f"template_track={int(self.template_tracking_enabled)}/"
            f"{self.template_tracking_max_age_frames} "
            f"template_fail_max="
            f"{self.template_tracking_max_consecutive_failures} "
            f"template_score={self.template_tracking_minimum_score:.2f} "
            f"stopped_reacquire_shift="
            f"{self.stopped_reacquire_maximum_center_shift_ratio:.2f} "
            f"aux_debug={int(self.show_auxiliary_debug)} "
            f"startup_hold={int(self.startup_hold_latched)} "
            f"startup_yolo_required={self.vote_window} "
            f"resume_on_green={self.resume_on_green} "
            f"resume_on_red_clear={self.resume_on_red_clear}"
        )

    def _open_camera(self) -> None:
        if self.camera_backend == "oak":
            self.oak_camera = OakRgbdCamera(
                width=self.oak_width,
                height=self.oak_height,
                fps=self.oak_fps,
                depth_enabled=self.oak_depth_enabled,
                depth_confidence_threshold=(
                    self.oak_depth_confidence_threshold
                ),
                depth_left_right_check=self.oak_depth_left_right_check,
                depth_subpixel=self.oak_depth_subpixel,
                depth_median_filter_size=(
                    self.oak_depth_median_filter_size
                ),
                depth_decimation_factor=(
                    self.oak_depth_decimation_factor
                ),
                depth_speckle_filter=self.oak_depth_speckle_filter,
                depth_spatial_filter=self.oak_depth_spatial_filter,
                depth_temporal_filter=self.oak_depth_temporal_filter,
                minimum_depth_m=self.minimum_depth_m,
                maximum_depth_m=self.maximum_depth_m,
                mxid=self.oak_mxid,
                usb_speed=self.oak_usb_speed,
            )
            return

        backend = (
            cv2.CAP_V4L2
            if isinstance(self.camera_source, int)
            or str(self.camera_source).startswith("/dev/video")
            else cv2.CAP_ANY
        )
        self.capture = cv2.VideoCapture(self.camera_source, backend)
        if not self.capture.isOpened():
            raise RuntimeError(
                f"카메라를 열 수 없습니다: {self.camera_source}"
            )
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def _camera_description(self) -> str:
        if self.camera_backend == "oak":
            depth_mode = "rgbd" if self.oak_depth_enabled else "rgb-only"
            usb_speed = getattr(self.oak_camera, "usb_speed", "unknown")
            connected_mxid = getattr(
                self.oak_camera,
                "mxid",
                self.oak_mxid or "unknown",
            )
            return (
                f"oak:{self.oak_width}x{self.oak_height}@"
                f"{self.oak_fps:g}/{depth_mode}/mxid={connected_mxid}/"
                f"usb_requested={self.oak_usb_speed.upper()}/"
                f"usb_actual={usb_speed}"
            )
        return f"opencv:{self.camera_source}"

    def _declare_parameters(self) -> None:
        self.declare_parameter("model_path", "")
        self.declare_parameter("camera_backend", "opencv")
        # ROS CLI의 `camera_source:=2`는 정수, `/dev/video2`는 문자열이다.
        # 두 형식을 모두 허용하고 _load_parameters에서 문자열로 정규화한다.
        self.declare_parameter(
            "camera_source",
            "2",
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("oak_width", 640)
        self.declare_parameter("oak_height", 360)
        self.declare_parameter("oak_fps", 10.0)
        # 신호등용 OAK-D MxID (CLAUDE.md §6, 2026-08-12 실측 확정) — 2대 운용 시
        # 핀닝 없으면 차선용 카메라를 잡을 수 있다. 빈 문자열이면 첫 가용 장치.
        self.declare_parameter("oak_mxid", "14442C10B167CFD200")
        # USB 링크 속도 상한: 'super'(제한 없음) | 'high'(USB2 강제, 기본).
        # 'high'는 GPS 간섭 대책 — oak_camera.py 주석 참조 (2026-08-14).
        # ★ 기본값을 'high'/10 으로 뒤집었다 (2026-08-24, 인수인계).
        #   USB3 로 열거되면 GNSS L1 이 덮여 RTK 가 죽는다 — 그걸 피하려면
        #   매 launch 마다 인자를 손으로 붙여야 했는데, 한 번 잊으면 위성 수도
        #   HDOP 도 RTCM 도 정상으로 보이는 채 FIXED 만 안 잡혀 원인을 찾기 어렵다.
        #   안전한 쪽을 기본으로 두고, USB3 가 필요하면 그때 명시적으로 올린다.
        self.declare_parameter("oak_usb_speed", "high")
        self.declare_parameter("oak_depth_enabled", True)
        # 작은 물체를 후처리가 지우는지 확인하는 raw 진단 기본값.
        self.declare_parameter("oak_depth_confidence_threshold", 245)
        self.declare_parameter("oak_depth_left_right_check", True)
        self.declare_parameter("oak_depth_subpixel", True)
        self.declare_parameter("oak_depth_median_filter_size", 0)
        self.declare_parameter("oak_depth_decimation_factor", 1)
        self.declare_parameter("oak_depth_speckle_filter", False)
        self.declare_parameter("oak_depth_spatial_filter", False)
        self.declare_parameter("oak_depth_temporal_filter", False)
        self.declare_parameter("yolo_image_size", 640)
        self.declare_parameter("yolo_inference_interval", 1)
        self.declare_parameter("detection_roi_enabled", False)
        # ROI를 켜면 화면 중앙선을 기준으로 상단 전체 폭을 검색한다.
        self.declare_parameter("detection_roi_x_min", 0.00)
        self.declare_parameter("detection_roi_y_min", 0.00)
        self.declare_parameter("detection_roi_x_max", 1.00)
        self.declare_parameter("detection_roi_y_max", 0.50)
        self.declare_parameter("detection_tile_width_ratio", 1.00)
        self.declare_parameter("process_period_sec", 0.10)
        self.declare_parameter("camera_timeout_sec", 0.50)
        self.declare_parameter("confidence_threshold", 0.20)
        self.declare_parameter("tracking_confidence_threshold", 0.10)
        self.declare_parameter("tracking_max_missed_frames", 5)
        self.declare_parameter("tracking_minimum_iou", 0.10)
        self.declare_parameter("tracking_maximum_center_shift_ratio", 0.50)
        self.declare_parameter("tracking_minimum_size_similarity", 0.50)
        self.declare_parameter("bbox_smoothing_current_weight", 0.65)
        self.declare_parameter("template_tracking_enabled", True)
        self.declare_parameter("template_tracking_max_age_frames", 4)
        self.declare_parameter(
            "template_tracking_max_consecutive_failures",
            3,
        )
        self.declare_parameter("template_tracking_context_scale", 1.8)
        self.declare_parameter("template_tracking_search_scale", 2.5)
        self.declare_parameter("template_tracking_minimum_score", 0.72)
        self.declare_parameter(
            "template_tracking_maximum_center_shift_ratio",
            0.75,
        )
        # 정지 중 장시간 놓친 동일 신호등만 더 넓은 범위에서 복구한다.
        self.declare_parameter(
            "stopped_reacquire_maximum_center_shift_ratio",
            3.0,
        )
        self.declare_parameter(
            "stopped_reacquire_minimum_size_similarity",
            0.50,
        )
        self.declare_parameter("minimum_box_area", 24)
        self.declare_parameter("minimum_box_width_height_ratio", 0.0)
        self.declare_parameter("minimum_red_ratio", 0.004)
        self.declare_parameter("minimum_green_ratio", 0.004)
        self.declare_parameter("red_hue_upper", 25)
        self.declare_parameter("red_hue_high_lower", 165)
        self.declare_parameter("minimum_color_saturation", 45)
        self.declare_parameter("minimum_color_value", 60)
        self.declare_parameter("vote_window", 5)
        self.declare_parameter("minimum_red_votes", 3)
        self.declare_parameter("minimum_green_votes", 3)
        self.declare_parameter("minimum_red_clear_bbox_observations", 4)
        self.declare_parameter("minimum_depth_m", 0.30)
        self.declare_parameter("maximum_depth_m", 20.0)
        self.declare_parameter("minimum_depth_valid_ratio", 0.10)
        self.declare_parameter("minimum_depth_valid_pixels", 10)
        # 하단 RGB 정지선 + OAK 정렬 depth 진단. 정지 임계값 0은 출력만 한다.
        self.declare_parameter("stopline_detection_enabled", False)
        self.declare_parameter("stopline_roi_x_min", 0.08)
        self.declare_parameter("stopline_roi_y_min", 0.48)
        self.declare_parameter("stopline_roi_x_max", 0.92)
        self.declare_parameter("stopline_roi_y_max", 0.98)
        self.declare_parameter("stopline_minimum_value", 145)
        self.declare_parameter("stopline_maximum_saturation", 90)
        self.declare_parameter("stopline_adaptive_percentile", 65.0)
        self.declare_parameter("stopline_adaptive_margin", 12.0)
        self.declare_parameter("stopline_local_contrast_enabled", True)
        self.declare_parameter("stopline_local_contrast_minimum_value", 60)
        self.declare_parameter("stopline_local_contrast_delta", 25.0)
        self.declare_parameter("stopline_local_contrast_background_ratio", 0.12)
        self.declare_parameter("stopline_local_contrast_clahe_clip_limit", 2.0)
        self.declare_parameter("stopline_edge_pair_enabled", True)
        self.declare_parameter("stopline_edge_pair_canny_low", 35)
        self.declare_parameter("stopline_edge_pair_canny_high", 110)
        self.declare_parameter("stopline_edge_pair_minimum_length_ratio", 0.35)
        self.declare_parameter("stopline_edge_pair_maximum_angle_difference_deg", 4.0)
        self.declare_parameter("stopline_edge_pair_minimum_interior_contrast", 8.0)
        self.declare_parameter("stopline_horizontal_close_ratio", 0.015)
        self.declare_parameter("stopline_minimum_width_ratio", 0.45)
        self.declare_parameter("stopline_minimum_aspect_ratio", 6.0)
        self.declare_parameter("stopline_minimum_fill_ratio", 0.30)
        self.declare_parameter("stopline_minimum_row_coverage", 0.60)
        self.declare_parameter("stopline_minimum_thickness_px", 3)
        self.declare_parameter("stopline_maximum_thickness_ratio", 0.20)
        self.declare_parameter("stopline_maximum_angle_deg", 12.0)
        self.declare_parameter("stopline_detection_window", 5)
        self.declare_parameter("stopline_minimum_detections", 3)
        self.declare_parameter("stopline_maximum_y_residual_ratio", 0.012)
        self.declare_parameter("stopline_maximum_y_step_ratio", 0.08)
        self.declare_parameter("stopline_maximum_backward_step_ratio", 0.005)
        self.declare_parameter("stopline_depth_inner_width_ratio", 0.50)
        self.declare_parameter("stopline_depth_band_height_px", 16)
        self.declare_parameter("stopline_depth_window", 5)
        self.declare_parameter("stopline_minimum_depth_samples", 3)
        self.declare_parameter("stopline_minimum_depth_rows", 6)
        self.declare_parameter("stopline_maximum_row_depth_mad_m", 0.20)
        self.declare_parameter(
            "stopline_depth_coherence_absolute_tolerance_m",
            0.20,
        )
        self.declare_parameter(
            "stopline_depth_coherence_relative_tolerance",
            0.08,
        )
        self.declare_parameter(
            "stopline_minimum_coherent_pixel_ratio",
            0.60,
        )
        self.declare_parameter(
            "stopline_minimum_inverse_depth_slope_per_px",
            0.0001,
        )
        self.declare_parameter(
            "stopline_maximum_inverse_depth_slope_per_px",
            0.02,
        )
        self.declare_parameter("stopline_maximum_fit_residual_m", 0.25)
        self.declare_parameter("stopline_stop_distance_m", 0.0)
        self.declare_parameter("stopline_stop_y_ratio", 0.0)
        self.declare_parameter("resume_on_green", True)
        self.declare_parameter("resume_on_red_clear", False)
        self.declare_parameter("show_debug", False)
        self.declare_parameter("show_auxiliary_debug", False)
        self.declare_parameter("print_every", 10)

    def _load_parameters(self) -> None:
        self.model_path = str(self.get_parameter("model_path").value)
        self.camera_backend = str(
            self.get_parameter("camera_backend").value
        ).strip().lower()
        self.camera_source_text = str(
            self.get_parameter("camera_source").value
        )
        self.camera_width = int(self.get_parameter("camera_width").value)
        self.camera_height = int(self.get_parameter("camera_height").value)
        self.oak_width = int(self.get_parameter("oak_width").value)
        self.oak_height = int(self.get_parameter("oak_height").value)
        self.oak_fps = float(self.get_parameter("oak_fps").value)
        self.oak_mxid = str(self.get_parameter("oak_mxid").value or "").strip()
        # 허용값(high|super) 밖이면 여기서 ValueError — 오타가 조용히
        # SUPER 로 떨어져 USB3 가 GNSS 를 덮는 것을 막는다 (CLAUDE.md §6).
        self.oak_usb_speed = normalize_oak_usb_speed(
            self.get_parameter("oak_usb_speed").value or "high"
        )
        self.oak_depth_enabled = bool(
            self.get_parameter("oak_depth_enabled").value
        )
        self.oak_depth_confidence_threshold = int(
            self.get_parameter("oak_depth_confidence_threshold").value
        )
        self.oak_depth_left_right_check = bool(
            self.get_parameter("oak_depth_left_right_check").value
        )
        self.oak_depth_subpixel = bool(
            self.get_parameter("oak_depth_subpixel").value
        )
        self.oak_depth_median_filter_size = int(
            self.get_parameter("oak_depth_median_filter_size").value
        )
        self.oak_depth_decimation_factor = int(
            self.get_parameter("oak_depth_decimation_factor").value
        )
        self.oak_depth_speckle_filter = bool(
            self.get_parameter("oak_depth_speckle_filter").value
        )
        self.oak_depth_spatial_filter = bool(
            self.get_parameter("oak_depth_spatial_filter").value
        )
        self.oak_depth_temporal_filter = bool(
            self.get_parameter("oak_depth_temporal_filter").value
        )
        self.yolo_image_size = int(
            self.get_parameter("yolo_image_size").value
        )
        self.yolo_inference_interval = int(
            self.get_parameter("yolo_inference_interval").value
        )
        self.detection_roi_enabled = bool(
            self.get_parameter("detection_roi_enabled").value
        )
        self.detection_roi_x_min = float(
            self.get_parameter("detection_roi_x_min").value
        )
        self.detection_roi_y_min = float(
            self.get_parameter("detection_roi_y_min").value
        )
        self.detection_roi_x_max = float(
            self.get_parameter("detection_roi_x_max").value
        )
        self.detection_roi_y_max = float(
            self.get_parameter("detection_roi_y_max").value
        )
        self.detection_tile_width_ratio = float(
            self.get_parameter("detection_tile_width_ratio").value
        )
        self.process_period_sec = float(
            self.get_parameter("process_period_sec").value
        )
        self.camera_timeout_sec = float(
            self.get_parameter("camera_timeout_sec").value
        )
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.tracking_confidence_threshold = float(
            self.get_parameter("tracking_confidence_threshold").value
        )
        self.tracking_max_missed_frames = int(
            self.get_parameter("tracking_max_missed_frames").value
        )
        self.tracking_minimum_iou = float(
            self.get_parameter("tracking_minimum_iou").value
        )
        self.tracking_maximum_center_shift_ratio = float(
            self.get_parameter(
                "tracking_maximum_center_shift_ratio"
            ).value
        )
        self.tracking_minimum_size_similarity = float(
            self.get_parameter("tracking_minimum_size_similarity").value
        )
        self.bbox_smoothing_current_weight = float(
            self.get_parameter("bbox_smoothing_current_weight").value
        )
        self.template_tracking_enabled = bool(
            self.get_parameter("template_tracking_enabled").value
        )
        self.template_tracking_max_age_frames = int(
            self.get_parameter(
                "template_tracking_max_age_frames"
            ).value
        )
        self.template_tracking_max_consecutive_failures = int(
            self.get_parameter(
                "template_tracking_max_consecutive_failures"
            ).value
        )
        self.template_tracking_context_scale = float(
            self.get_parameter("template_tracking_context_scale").value
        )
        self.template_tracking_search_scale = float(
            self.get_parameter("template_tracking_search_scale").value
        )
        self.template_tracking_minimum_score = float(
            self.get_parameter("template_tracking_minimum_score").value
        )
        self.template_tracking_maximum_center_shift_ratio = float(
            self.get_parameter(
                "template_tracking_maximum_center_shift_ratio"
            ).value
        )
        self.stopped_reacquire_maximum_center_shift_ratio = float(
            self.get_parameter(
                "stopped_reacquire_maximum_center_shift_ratio"
            ).value
        )
        self.stopped_reacquire_minimum_size_similarity = float(
            self.get_parameter(
                "stopped_reacquire_minimum_size_similarity"
            ).value
        )
        self.minimum_box_area = int(
            self.get_parameter("minimum_box_area").value
        )
        self.minimum_box_width_height_ratio = float(
            self.get_parameter("minimum_box_width_height_ratio").value
        )
        self.minimum_red_ratio = float(
            self.get_parameter("minimum_red_ratio").value
        )
        self.minimum_green_ratio = float(
            self.get_parameter("minimum_green_ratio").value
        )
        self.red_hue_upper = int(
            self.get_parameter("red_hue_upper").value
        )
        self.red_hue_high_lower = int(
            self.get_parameter("red_hue_high_lower").value
        )
        self.minimum_color_saturation = int(
            self.get_parameter("minimum_color_saturation").value
        )
        self.minimum_color_value = int(
            self.get_parameter("minimum_color_value").value
        )
        self.vote_window = int(self.get_parameter("vote_window").value)
        self.minimum_red_votes = int(
            self.get_parameter("minimum_red_votes").value
        )
        self.minimum_green_votes = int(
            self.get_parameter("minimum_green_votes").value
        )
        self.minimum_red_clear_bbox_observations = int(
            self.get_parameter(
                "minimum_red_clear_bbox_observations"
            ).value
        )
        self.minimum_depth_m = float(
            self.get_parameter("minimum_depth_m").value
        )
        self.maximum_depth_m = float(
            self.get_parameter("maximum_depth_m").value
        )
        self.minimum_depth_valid_ratio = float(
            self.get_parameter("minimum_depth_valid_ratio").value
        )
        self.minimum_depth_valid_pixels = int(
            self.get_parameter("minimum_depth_valid_pixels").value
        )
        self.stopline_detection_enabled = bool(
            self.get_parameter("stopline_detection_enabled").value
        )
        self.stopline_roi_x_min = float(
            self.get_parameter("stopline_roi_x_min").value
        )
        self.stopline_roi_y_min = float(
            self.get_parameter("stopline_roi_y_min").value
        )
        self.stopline_roi_x_max = float(
            self.get_parameter("stopline_roi_x_max").value
        )
        self.stopline_roi_y_max = float(
            self.get_parameter("stopline_roi_y_max").value
        )
        self.stopline_minimum_value = int(
            self.get_parameter("stopline_minimum_value").value
        )
        self.stopline_maximum_saturation = int(
            self.get_parameter("stopline_maximum_saturation").value
        )
        self.stopline_adaptive_percentile = float(
            self.get_parameter("stopline_adaptive_percentile").value
        )
        self.stopline_adaptive_margin = float(
            self.get_parameter("stopline_adaptive_margin").value
        )
        self.stopline_local_contrast_enabled = bool(
            self.get_parameter("stopline_local_contrast_enabled").value
        )
        self.stopline_local_contrast_minimum_value = int(
            self.get_parameter("stopline_local_contrast_minimum_value").value
        )
        self.stopline_local_contrast_delta = float(
            self.get_parameter("stopline_local_contrast_delta").value
        )
        self.stopline_local_contrast_background_ratio = float(
            self.get_parameter(
                "stopline_local_contrast_background_ratio"
            ).value
        )
        self.stopline_local_contrast_clahe_clip_limit = float(
            self.get_parameter(
                "stopline_local_contrast_clahe_clip_limit"
            ).value
        )
        self.stopline_edge_pair_enabled = bool(
            self.get_parameter("stopline_edge_pair_enabled").value
        )
        self.stopline_edge_pair_canny_low = int(
            self.get_parameter("stopline_edge_pair_canny_low").value
        )
        self.stopline_edge_pair_canny_high = int(
            self.get_parameter("stopline_edge_pair_canny_high").value
        )
        self.stopline_edge_pair_minimum_length_ratio = float(
            self.get_parameter(
                "stopline_edge_pair_minimum_length_ratio"
            ).value
        )
        self.stopline_edge_pair_maximum_angle_difference_deg = float(
            self.get_parameter(
                "stopline_edge_pair_maximum_angle_difference_deg"
            ).value
        )
        self.stopline_edge_pair_minimum_interior_contrast = float(
            self.get_parameter(
                "stopline_edge_pair_minimum_interior_contrast"
            ).value
        )
        self.stopline_horizontal_close_ratio = float(
            self.get_parameter("stopline_horizontal_close_ratio").value
        )
        self.stopline_minimum_width_ratio = float(
            self.get_parameter("stopline_minimum_width_ratio").value
        )
        self.stopline_minimum_aspect_ratio = float(
            self.get_parameter("stopline_minimum_aspect_ratio").value
        )
        self.stopline_minimum_fill_ratio = float(
            self.get_parameter("stopline_minimum_fill_ratio").value
        )
        self.stopline_minimum_row_coverage = float(
            self.get_parameter("stopline_minimum_row_coverage").value
        )
        self.stopline_minimum_thickness_px = int(
            self.get_parameter("stopline_minimum_thickness_px").value
        )
        self.stopline_maximum_thickness_ratio = float(
            self.get_parameter("stopline_maximum_thickness_ratio").value
        )
        self.stopline_maximum_angle_deg = float(
            self.get_parameter("stopline_maximum_angle_deg").value
        )
        self.stopline_detection_window = int(
            self.get_parameter("stopline_detection_window").value
        )
        self.stopline_minimum_detections = int(
            self.get_parameter("stopline_minimum_detections").value
        )
        self.stopline_maximum_y_residual_ratio = float(
            self.get_parameter("stopline_maximum_y_residual_ratio").value
        )
        self.stopline_maximum_y_step_ratio = float(
            self.get_parameter("stopline_maximum_y_step_ratio").value
        )
        self.stopline_maximum_backward_step_ratio = float(
            self.get_parameter(
                "stopline_maximum_backward_step_ratio"
            ).value
        )
        self.stopline_depth_inner_width_ratio = float(
            self.get_parameter("stopline_depth_inner_width_ratio").value
        )
        self.stopline_depth_band_height_px = int(
            self.get_parameter("stopline_depth_band_height_px").value
        )
        self.stopline_depth_window = int(
            self.get_parameter("stopline_depth_window").value
        )
        self.stopline_minimum_depth_samples = int(
            self.get_parameter("stopline_minimum_depth_samples").value
        )
        self.stopline_minimum_depth_rows = int(
            self.get_parameter("stopline_minimum_depth_rows").value
        )
        self.stopline_maximum_row_depth_mad_m = float(
            self.get_parameter("stopline_maximum_row_depth_mad_m").value
        )
        self.stopline_depth_coherence_absolute_tolerance_m = float(
            self.get_parameter(
                "stopline_depth_coherence_absolute_tolerance_m"
            ).value
        )
        self.stopline_depth_coherence_relative_tolerance = float(
            self.get_parameter(
                "stopline_depth_coherence_relative_tolerance"
            ).value
        )
        self.stopline_minimum_coherent_pixel_ratio = float(
            self.get_parameter(
                "stopline_minimum_coherent_pixel_ratio"
            ).value
        )
        self.stopline_minimum_inverse_depth_slope_per_px = float(
            self.get_parameter(
                "stopline_minimum_inverse_depth_slope_per_px"
            ).value
        )
        self.stopline_maximum_inverse_depth_slope_per_px = float(
            self.get_parameter(
                "stopline_maximum_inverse_depth_slope_per_px"
            ).value
        )
        self.stopline_maximum_fit_residual_m = float(
            self.get_parameter("stopline_maximum_fit_residual_m").value
        )
        self.stopline_stop_distance_m = float(
            self.get_parameter("stopline_stop_distance_m").value
        )
        self.stopline_stop_y_ratio = float(
            self.get_parameter("stopline_stop_y_ratio").value
        )
        self.resume_on_green = bool(
            self.get_parameter("resume_on_green").value
        )
        self.resume_on_red_clear = bool(
            self.get_parameter("resume_on_red_clear").value
        )
        self.show_debug = bool(self.get_parameter("show_debug").value)
        self.show_auxiliary_debug = bool(
            self.get_parameter("show_auxiliary_debug").value
        )
        self.print_every = max(
            1, int(self.get_parameter("print_every").value)
        )

        if self.camera_backend not in ("opencv", "oak"):
            raise ValueError("camera_backend은 opencv 또는 oak여야 합니다.")
        if self.camera_width < 1 or self.camera_height < 1:
            raise ValueError("camera_width와 camera_height는 1 이상이어야 합니다.")
        if self.oak_width < 1 or self.oak_height < 1:
            raise ValueError("oak_width와 oak_height는 1 이상이어야 합니다.")
        if self.oak_fps <= 0.0:
            raise ValueError("oak_fps는 0보다 커야 합니다.")
        if self.process_period_sec <= 0.0:
            raise ValueError("process_period_sec는 0보다 커야 합니다.")
        if self.camera_timeout_sec <= 0.0:
            raise ValueError("camera_timeout_sec는 0보다 커야 합니다.")
        if not 0 <= self.oak_depth_confidence_threshold <= 255:
            raise ValueError(
                "oak_depth_confidence_threshold는 0 이상 255 이하여야 합니다."
            )
        if self.oak_depth_median_filter_size not in (0, 3, 5, 7):
            raise ValueError(
                "oak_depth_median_filter_size는 0, 3, 5, 7 중 하나여야 합니다."
            )
        if self.oak_depth_decimation_factor not in (1, 2, 3, 4):
            raise ValueError(
                "oak_depth_decimation_factor는 1~4 중 하나여야 합니다."
            )
        if self.yolo_image_size < 320:
            raise ValueError("yolo_image_size는 320 이상이어야 합니다.")
        if self.yolo_inference_interval < 1:
            raise ValueError("yolo_inference_interval은 1 이상이어야 합니다.")
        if not (
            0.0
            <= self.detection_roi_x_min
            < self.detection_roi_x_max
            <= 1.0
        ):
            raise ValueError(
                "detection_roi_x_min/x_max는 0~1 안에서 min < max여야 합니다."
            )
        if not (
            0.0
            <= self.detection_roi_y_min
            < self.detection_roi_y_max
            <= 1.0
        ):
            raise ValueError(
                "detection_roi_y_min/y_max는 0~1 안에서 min < max여야 합니다."
            )
        if not 0.0 < self.detection_tile_width_ratio <= 1.0:
            raise ValueError(
                "detection_tile_width_ratio는 0 초과 1 이하여야 합니다."
            )
        if self.vote_window < 1:
            raise ValueError("vote_window은 1 이상이어야 합니다.")
        if not 0.0 < self.confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold는 0 초과 1 이하여야 합니다.")
        if not (
            0.0 < self.tracking_confidence_threshold
            <= self.confidence_threshold
        ):
            raise ValueError(
                "tracking_confidence_threshold는 0 초과이고 "
                "confidence_threshold 이하여야 합니다."
            )
        if self.tracking_max_missed_frames < 1:
            raise ValueError("tracking_max_missed_frames는 1 이상이어야 합니다.")
        if not 0.0 <= self.tracking_minimum_iou <= 1.0:
            raise ValueError("tracking_minimum_iou는 0 이상 1 이하여야 합니다.")
        if self.tracking_maximum_center_shift_ratio <= 0.0:
            raise ValueError(
                "tracking_maximum_center_shift_ratio는 0보다 커야 합니다."
            )
        if not 0.0 < self.tracking_minimum_size_similarity <= 1.0:
            raise ValueError(
                "tracking_minimum_size_similarity는 0 초과 1 이하여야 합니다."
            )
        if not 0.0 < self.bbox_smoothing_current_weight <= 1.0:
            raise ValueError(
                "bbox_smoothing_current_weight는 0 초과 1 이하여야 합니다."
            )
        if self.template_tracking_max_age_frames < 1:
            raise ValueError(
                "template_tracking_max_age_frames는 1 이상이어야 합니다."
            )
        if self.template_tracking_max_consecutive_failures < 1:
            raise ValueError(
                "template_tracking_max_consecutive_failures는 "
                "1 이상이어야 합니다."
            )
        if self.template_tracking_context_scale < 1.0:
            raise ValueError(
                "template_tracking_context_scale은 1 이상이어야 합니다."
            )
        if self.template_tracking_search_scale <= 1.0:
            raise ValueError(
                "template_tracking_search_scale은 1보다 커야 합니다."
            )
        if not 0.0 <= self.template_tracking_minimum_score <= 1.0:
            raise ValueError(
                "template_tracking_minimum_score는 0~1이어야 합니다."
            )
        if self.template_tracking_maximum_center_shift_ratio <= 0.0:
            raise ValueError(
                "template_tracking_maximum_center_shift_ratio는 "
                "0보다 커야 합니다."
            )
        if self.stopped_reacquire_maximum_center_shift_ratio <= 0.0:
            raise ValueError(
                "stopped_reacquire_maximum_center_shift_ratio는 "
                "0보다 커야 합니다."
            )
        if not (
            0.0
            < self.stopped_reacquire_minimum_size_similarity
            <= 1.0
        ):
            raise ValueError(
                "stopped_reacquire_minimum_size_similarity는 "
                "0 초과 1 이하여야 합니다."
            )
        if not 1 <= self.minimum_red_votes <= self.vote_window:
            raise ValueError(
                "minimum_red_votes는 1 이상 vote_window 이하여야 합니다."
            )
        if not 1 <= self.minimum_green_votes <= self.vote_window:
            raise ValueError(
                "minimum_green_votes는 1 이상 vote_window 이하여야 합니다."
            )
        if not (
            1
            <= self.minimum_red_clear_bbox_observations
            <= self.vote_window
        ):
            raise ValueError(
                "minimum_red_clear_bbox_observations는 1 이상 "
                "vote_window 이하여야 합니다."
            )
        if self.minimum_red_ratio < 0.0:
            raise ValueError("minimum_red_ratio는 0 이상이어야 합니다.")
        if self.minimum_green_ratio < 0.0:
            raise ValueError("minimum_green_ratio는 0 이상이어야 합니다.")
        if not 0 <= self.red_hue_upper < 35:
            raise ValueError("red_hue_upper는 0 이상 35 미만이어야 합니다.")
        if not 95 < self.red_hue_high_lower <= 179:
            raise ValueError(
                "red_hue_high_lower는 95 초과 179 이하여야 합니다."
            )
        if not 0 <= self.minimum_color_saturation <= 255:
            raise ValueError(
                "minimum_color_saturation은 0 이상 255 이하여야 합니다."
            )
        if not 0 <= self.minimum_color_value <= 255:
            raise ValueError(
                "minimum_color_value는 0 이상 255 이하여야 합니다."
            )
        if self.minimum_box_width_height_ratio < 0.0:
            raise ValueError(
                "minimum_box_width_height_ratio는 0 이상이어야 합니다."
            )
        if self.minimum_depth_m <= 0.0:
            raise ValueError("minimum_depth_m은 0보다 커야 합니다.")
        if self.maximum_depth_m <= self.minimum_depth_m:
            raise ValueError(
                "maximum_depth_m은 minimum_depth_m보다 커야 합니다."
            )
        if self.maximum_depth_m > 65.0:
            raise ValueError("maximum_depth_m은 65 이하여야 합니다.")
        if not 0.0 <= self.minimum_depth_valid_ratio <= 1.0:
            raise ValueError(
                "minimum_depth_valid_ratio는 0 이상 1 이하여야 합니다."
            )
        if self.minimum_depth_valid_pixels < 1:
            raise ValueError("minimum_depth_valid_pixels는 1 이상이어야 합니다.")
        if not (
            0.0
            <= self.stopline_roi_x_min
            < self.stopline_roi_x_max
            <= 1.0
        ):
            raise ValueError(
                "stopline_roi_x_min/x_max는 0~1 안에서 min < max여야 합니다."
            )
        if not (
            0.0
            <= self.stopline_roi_y_min
            < self.stopline_roi_y_max
            <= 1.0
        ):
            raise ValueError(
                "stopline_roi_y_min/y_max는 0~1 안에서 min < max여야 합니다."
            )
        if not 0 <= self.stopline_minimum_value <= 255:
            raise ValueError("stopline_minimum_value는 0~255여야 합니다.")
        if not 0 <= self.stopline_maximum_saturation <= 255:
            raise ValueError(
                "stopline_maximum_saturation은 0~255여야 합니다."
            )
        if not 0.0 <= self.stopline_adaptive_percentile <= 100.0:
            raise ValueError(
                "stopline_adaptive_percentile은 0~100이어야 합니다."
            )
        if self.stopline_adaptive_margin < 0.0:
            raise ValueError("stopline_adaptive_margin은 0 이상이어야 합니다.")
        if not 0 <= self.stopline_local_contrast_minimum_value <= 255:
            raise ValueError(
                "stopline_local_contrast_minimum_value는 0~255여야 합니다."
            )
        if not 0.0 < self.stopline_local_contrast_delta <= 255.0:
            raise ValueError(
                "stopline_local_contrast_delta는 0보다 크고 255 이하여야 합니다."
            )
        if not 0.03 <= self.stopline_local_contrast_background_ratio <= 0.50:
            raise ValueError(
                "stopline_local_contrast_background_ratio는 0.03~0.50이어야 합니다."
            )
        if self.stopline_local_contrast_clahe_clip_limit <= 0.0:
            raise ValueError(
                "stopline_local_contrast_clahe_clip_limit는 0보다 커야 합니다."
            )
        if not 0 <= self.stopline_edge_pair_canny_low <= 255:
            raise ValueError("stopline_edge_pair_canny_low는 0~255여야 합니다.")
        if not self.stopline_edge_pair_canny_low < self.stopline_edge_pair_canny_high <= 255:
            raise ValueError(
                "stopline_edge_pair_canny_high는 low보다 크고 255 이하여야 합니다."
            )
        if not 0.10 <= self.stopline_edge_pair_minimum_length_ratio <= 1.0:
            raise ValueError(
                "stopline_edge_pair_minimum_length_ratio는 0.10~1.0이어야 합니다."
            )
        if not 0.0 <= self.stopline_edge_pair_maximum_angle_difference_deg <= 15.0:
            raise ValueError(
                "stopline_edge_pair_maximum_angle_difference_deg는 0~15여야 합니다."
            )
        if not 0.0 <= self.stopline_edge_pair_minimum_interior_contrast <= 255.0:
            raise ValueError(
                "stopline_edge_pair_minimum_interior_contrast는 0~255여야 합니다."
            )
        if not 0.0 < self.stopline_horizontal_close_ratio <= 0.10:
            raise ValueError(
                "stopline_horizontal_close_ratio는 0 초과 0.10 이하여야 합니다."
            )
        if not 0.0 < self.stopline_minimum_width_ratio <= 1.0:
            raise ValueError(
                "stopline_minimum_width_ratio는 0 초과 1 이하여야 합니다."
            )
        if self.stopline_minimum_aspect_ratio <= 1.0:
            raise ValueError(
                "stopline_minimum_aspect_ratio는 1보다 커야 합니다."
            )
        if not 0.0 <= self.stopline_minimum_fill_ratio <= 1.0:
            raise ValueError(
                "stopline_minimum_fill_ratio는 0~1이어야 합니다."
            )
        if not 0.0 <= self.stopline_minimum_row_coverage <= 1.0:
            raise ValueError(
                "stopline_minimum_row_coverage는 0~1이어야 합니다."
            )
        if self.stopline_minimum_thickness_px < 1:
            raise ValueError(
                "stopline_minimum_thickness_px은 1 이상이어야 합니다."
            )
        if not 0.0 < self.stopline_maximum_thickness_ratio <= 1.0:
            raise ValueError(
                "stopline_maximum_thickness_ratio는 0 초과 1 이하여야 합니다."
            )
        if not 0.0 <= self.stopline_maximum_angle_deg <= 45.0:
            raise ValueError(
                "stopline_maximum_angle_deg는 0~45여야 합니다."
            )
        if self.stopline_detection_window < 1:
            raise ValueError("stopline_detection_window은 1 이상이어야 합니다.")
        if not (
            1
            <= self.stopline_minimum_detections
            <= self.stopline_detection_window
        ):
            raise ValueError(
                "stopline_minimum_detections는 1 이상 detection_window "
                "이하여야 합니다."
            )
        if self.stopline_maximum_y_residual_ratio < 0.0:
            raise ValueError(
                "stopline_maximum_y_residual_ratio는 0 이상이어야 합니다."
            )
        if self.stopline_maximum_y_step_ratio <= 0.0:
            raise ValueError(
                "stopline_maximum_y_step_ratio는 0보다 커야 합니다."
            )
        if self.stopline_maximum_backward_step_ratio < 0.0:
            raise ValueError(
                "stopline_maximum_backward_step_ratio는 0 이상이어야 합니다."
            )
        if not 0.0 < self.stopline_depth_inner_width_ratio <= 1.0:
            raise ValueError(
                "stopline_depth_inner_width_ratio는 0 초과 1 이하여야 합니다."
            )
        if self.stopline_depth_band_height_px < 1:
            raise ValueError(
                "stopline_depth_band_height_px은 1 이상이어야 합니다."
            )
        if self.stopline_depth_window < 1:
            raise ValueError("stopline_depth_window은 1 이상이어야 합니다.")
        if not (
            1
            <= self.stopline_minimum_depth_samples
            <= self.stopline_depth_window
        ):
            raise ValueError(
                "stopline_minimum_depth_samples는 1 이상 depth_window "
                "이하여야 합니다."
            )
        if self.stopline_minimum_depth_rows < 2:
            raise ValueError(
                "stopline_minimum_depth_rows는 2 이상이어야 합니다."
            )
        if self.stopline_maximum_row_depth_mad_m <= 0.0:
            raise ValueError(
                "stopline_maximum_row_depth_mad_m은 0보다 커야 합니다."
            )
        if self.stopline_depth_coherence_absolute_tolerance_m <= 0.0:
            raise ValueError(
                "stopline_depth_coherence_absolute_tolerance_m은 "
                "0보다 커야 합니다."
            )
        if not (
            0.0
            <= self.stopline_depth_coherence_relative_tolerance
            <= 1.0
        ):
            raise ValueError(
                "stopline_depth_coherence_relative_tolerance는 "
                "0~1이어야 합니다."
            )
        if not 0.0 < self.stopline_minimum_coherent_pixel_ratio <= 1.0:
            raise ValueError(
                "stopline_minimum_coherent_pixel_ratio는 "
                "0 초과 1 이하여야 합니다."
            )
        if self.stopline_minimum_inverse_depth_slope_per_px <= 0.0:
            raise ValueError(
                "stopline_minimum_inverse_depth_slope_per_px는 "
                "0보다 커야 합니다."
            )
        if (
            self.stopline_maximum_inverse_depth_slope_per_px
            <= self.stopline_minimum_inverse_depth_slope_per_px
        ):
            raise ValueError(
                "stopline_maximum_inverse_depth_slope_per_px는 "
                "minimum보다 커야 합니다."
            )
        if self.stopline_maximum_fit_residual_m <= 0.0:
            raise ValueError(
                "stopline_maximum_fit_residual_m은 0보다 커야 합니다."
            )
        if self.stopline_stop_distance_m < 0.0:
            raise ValueError("stopline_stop_distance_m은 0 이상이어야 합니다.")
        if not 0.0 <= self.stopline_stop_y_ratio <= 1.10:
            raise ValueError(
                "stopline_stop_y_ratio는 0 이상 1.10 이하여야 합니다."
            )
        if (
            self.stopline_stop_y_ratio > 0.0
            and not self.stopline_detection_enabled
        ):
            raise ValueError(
                "stopline_stop_y_ratio를 사용하려면 정지선 검출이 켜져야 합니다."
            )
        if self.stopline_stop_distance_m > 0.0 and (
            not self.stopline_detection_enabled
            or self.camera_backend != "oak"
            or not self.oak_depth_enabled
        ):
            raise ValueError(
                "stopline_stop_distance_m을 사용하려면 정지선 검출과 "
                "OAK depth가 켜져야 합니다."
            )

    def _resolve_model_path(self) -> Path:
        if self.model_path.strip():
            return Path(self.model_path).expanduser().resolve()

        share_dir = Path(get_package_share_directory("stack_traffic"))
        return share_dir / "models" / "yolov8n.pt"

    def _read_camera(
        self,
    ) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray]]:
        if self.camera_backend == "oak":
            return self.oak_camera.read()
        success, frame = self.capture.read()
        return success, frame, None

    def _prepare_detection_frame(
        self,
        frame: np.ndarray,
    ) -> Tuple[np.ndarray, BBox]:
        search_roi_bbox = self._detection_roi_bbox(frame)
        if not self.detection_roi_enabled:
            self.last_detection_tile_bbox = search_roi_bbox
            return frame, search_roi_bbox
        tile_width_px = max(
            1,
            int(round(frame.shape[1] * self.detection_tile_width_ratio)),
        )
        roi_bbox = select_horizontal_roi_tile(
            search_roi=search_roi_bbox,
            tile_width_px=tile_width_px,
            tracked_bbox=self.tracked_bbox,
            scan_index=self.detection_scan_index,
        )
        if self.tracked_bbox is None:
            self.detection_scan_index += 1
        self.last_detection_tile_bbox = roi_bbox
        x1, y1, x2, y2 = roi_bbox
        detection_frame = np.ascontiguousarray(frame[y1:y2, x1:x2])
        return detection_frame, roi_bbox

    def _detection_roi_bbox(self, frame: np.ndarray) -> BBox:
        frame_height, frame_width = frame.shape[:2]
        if not self.detection_roi_enabled:
            return 0, 0, frame_width, frame_height
        return normalized_roi_to_bbox(
            frame.shape,
            self.detection_roi_x_min,
            self.detection_roi_y_min,
            self.detection_roi_x_max,
            self.detection_roi_y_max,
        )

    def _choose_detection_target(
        self,
        results,
        detection_frame: np.ndarray,
        detection_roi_bbox: BBox,
        previous_frame_bbox: Optional[BBox],
        tracking_confidence_threshold: Optional[float] = None,
        tracking_maximum_center_shift_ratio: Optional[float] = None,
        tracking_minimum_size_similarity: Optional[float] = None,
    ) -> Tuple[Optional[BBox], float]:
        previous_detection_bbox = None
        if previous_frame_bbox is not None:
            previous_detection_bbox = frame_bbox_to_roi(
                previous_frame_bbox,
                detection_roi_bbox,
            )
            if previous_detection_bbox is None:
                return None, 0.0

        detection_bbox, confidence = choose_target_traffic_light(
            results=results,
            frame_shape=detection_frame.shape,
            confidence_threshold=self.confidence_threshold,
            minimum_box_area=self.minimum_box_area,
            previous_bbox=previous_detection_bbox,
            tracking_confidence_threshold=(
                self.tracking_confidence_threshold
                if tracking_confidence_threshold is None
                else tracking_confidence_threshold
            ),
            tracking_minimum_iou=self.tracking_minimum_iou,
            tracking_maximum_center_shift_ratio=(
                self.tracking_maximum_center_shift_ratio
                if tracking_maximum_center_shift_ratio is None
                else tracking_maximum_center_shift_ratio
            ),
            tracking_minimum_size_similarity=(
                self.tracking_minimum_size_similarity
                if tracking_minimum_size_similarity is None
                else tracking_minimum_size_similarity
            ),
            minimum_box_width_height_ratio=(
                self.minimum_box_width_height_ratio
            ),
        )
        if detection_bbox is None:
            return None, confidence
        return (
            roi_bbox_to_frame(detection_bbox, detection_roi_bbox),
            confidence,
        )

    def _empty_stopline_depth(
        self,
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

    def _empty_stopline_runtime(self) -> StopLineRuntime:
        return StopLineRuntime(
            detection=None,
            depth=self._empty_stopline_depth(),
            current_camera_z_m=math.nan,
            median_camera_z_m=math.nan,
            valid_distance_samples=0,
            median_y_px=math.nan,
            current_y_ratio=math.nan,
            median_y_ratio=math.nan,
            valid_y_samples=0,
            stable=False,
            depth_near=False,
            y_near=False,
            near=False,
        )

    def _process_stopline(
        self,
        frame: np.ndarray,
        depth_mm: Optional[np.ndarray],
    ) -> StopLineRuntime:
        if not self.stopline_detection_enabled:
            return self._empty_stopline_runtime()

        detection = detect_stop_line(
            frame=frame,
            roi_x_min=self.stopline_roi_x_min,
            roi_y_min=self.stopline_roi_y_min,
            roi_x_max=self.stopline_roi_x_max,
            roi_y_max=self.stopline_roi_y_max,
            minimum_value=self.stopline_minimum_value,
            maximum_saturation=self.stopline_maximum_saturation,
            adaptive_percentile=self.stopline_adaptive_percentile,
            adaptive_margin=self.stopline_adaptive_margin,
            local_contrast_enabled=self.stopline_local_contrast_enabled,
            local_contrast_minimum_value=(
                self.stopline_local_contrast_minimum_value
            ),
            local_contrast_delta=self.stopline_local_contrast_delta,
            local_contrast_background_ratio=(
                self.stopline_local_contrast_background_ratio
            ),
            local_contrast_clahe_clip_limit=(
                self.stopline_local_contrast_clahe_clip_limit
            ),
            edge_pair_enabled=self.stopline_edge_pair_enabled,
            edge_pair_canny_low=self.stopline_edge_pair_canny_low,
            edge_pair_canny_high=self.stopline_edge_pair_canny_high,
            edge_pair_minimum_length_ratio=(
                self.stopline_edge_pair_minimum_length_ratio
            ),
            edge_pair_maximum_angle_difference_deg=(
                self.stopline_edge_pair_maximum_angle_difference_deg
            ),
            edge_pair_minimum_interior_contrast=(
                self.stopline_edge_pair_minimum_interior_contrast
            ),
            horizontal_close_ratio=(
                self.stopline_horizontal_close_ratio
            ),
            minimum_width_ratio=self.stopline_minimum_width_ratio,
            minimum_aspect_ratio=self.stopline_minimum_aspect_ratio,
            minimum_fill_ratio=self.stopline_minimum_fill_ratio,
            minimum_row_coverage=(
                self.stopline_minimum_row_coverage
            ),
            minimum_thickness_px=self.stopline_minimum_thickness_px,
            maximum_thickness_ratio=(
                self.stopline_maximum_thickness_ratio
            ),
            maximum_angle_deg=self.stopline_maximum_angle_deg,
        )
        self.stopline_y_history.append(
            detection.maximum_edge_y_px
            if detection.detected
            else math.nan
        )
        median_y_px, valid_y_samples, history_stable = stable_stopline_y(
            self.stopline_y_history,
            minimum_samples=self.stopline_minimum_detections,
            maximum_fit_error_px=(
                frame.shape[0] * self.stopline_maximum_y_residual_ratio
            ),
            maximum_forward_step_px=(
                frame.shape[0] * self.stopline_maximum_y_step_ratio
            ),
            maximum_backward_step_px=(
                frame.shape[0]
                * self.stopline_maximum_backward_step_ratio
            ),
        )
        stable = bool(detection.detected and history_stable)
        frame_height = max(1, frame.shape[0])
        current_y_ratio = (
            detection.maximum_edge_y_px / float(frame_height)
            if detection.detected
            else math.nan
        )
        median_y_ratio = (
            median_y_px / float(frame_height)
            if math.isfinite(median_y_px)
            else math.nan
        )

        sample_bbox = make_stopline_depth_bbox(
            detection.bbox,
            frame.shape,
            inner_width_ratio=self.stopline_depth_inner_width_ratio,
            band_height_px=self.stopline_depth_band_height_px,
            near_edge_y_px=detection.near_edge_y_px,
        )
        depth = self._empty_stopline_depth(sample_bbox)
        if depth_mm is not None and detection.detected:
            exclusion_mask = stopline_mask_in_frame(
                detection,
                frame.shape,
            )
            depth = measure_stopline_depth(
                depth_mm=depth_mm,
                sample_bbox=sample_bbox,
                target_y_px=detection.near_edge_y_px,
                exclusion_mask=exclusion_mask,
                minimum_depth_m=self.minimum_depth_m,
                maximum_depth_m=self.maximum_depth_m,
                minimum_valid_ratio=self.minimum_depth_valid_ratio,
                minimum_valid_pixels=self.minimum_depth_valid_pixels,
                minimum_valid_rows=self.stopline_minimum_depth_rows,
                maximum_row_depth_mad_m=(
                    self.stopline_maximum_row_depth_mad_m
                ),
                coherence_absolute_tolerance_m=(
                    self.stopline_depth_coherence_absolute_tolerance_m
                ),
                coherence_relative_tolerance=(
                    self.stopline_depth_coherence_relative_tolerance
                ),
                minimum_coherent_pixel_ratio=(
                    self.stopline_minimum_coherent_pixel_ratio
                ),
                minimum_inverse_depth_slope_per_px=(
                    self.stopline_minimum_inverse_depth_slope_per_px
                ),
                maximum_inverse_depth_slope_per_px=(
                    self.stopline_maximum_inverse_depth_slope_per_px
                ),
                maximum_fit_residual_m=(
                    self.stopline_maximum_fit_residual_m
                ),
            )

        current_camera_z_m = depth.camera_z_m
        current_distance_accepted = bool(
            stable
            and depth.accepted
            and math.isfinite(current_camera_z_m)
        )
        if valid_y_samples >= self.stopline_minimum_detections and not stable:
            self.stopline_distance_history.clear()
            self.stopline_distance_history.extend(
                [math.nan] * self.stopline_depth_window
            )
        self.stopline_distance_history.append(
            current_camera_z_m
            if current_distance_accepted
            else math.nan
        )
        median_camera_z_m, valid_distance_samples = (
            robust_nonnegative_median(
                self.stopline_distance_history,
                self.stopline_minimum_depth_samples,
            )
        )
        depth_near = is_stopline_approaching(
            median_distance_m=median_camera_z_m,
            stop_distance_m=self.stopline_stop_distance_m,
            current_line_detected=detection.detected,
            current_depth_accepted=depth.accepted,
            line_stable=stable,
            valid_distance_samples=valid_distance_samples,
            minimum_distance_samples=(
                self.stopline_minimum_depth_samples
            ),
        )
        y_near = is_stopline_y_approaching(
            median_y_px=median_y_px,
            frame_height_px=frame_height,
            stop_y_ratio=self.stopline_stop_y_ratio,
            current_line_detected=detection.detected,
            line_stable=stable,
            valid_y_samples=valid_y_samples,
            minimum_y_samples=self.stopline_minimum_detections,
        )
        near = combine_stopline_proximity(
            depth_near=depth_near,
            y_near=y_near,
            depth_threshold_m=self.stopline_stop_distance_m,
            y_threshold_ratio=self.stopline_stop_y_ratio,
        )
        return StopLineRuntime(
            detection=detection,
            depth=depth,
            current_camera_z_m=current_camera_z_m,
            median_camera_z_m=median_camera_z_m,
            valid_distance_samples=valid_distance_samples,
            median_y_px=median_y_px,
            current_y_ratio=current_y_ratio,
            median_y_ratio=median_y_ratio,
            valid_y_samples=valid_y_samples,
            stable=stable,
            depth_near=depth_near,
            y_near=y_near,
            near=near,
        )

    def _stopline_log_text(self, runtime: StopLineRuntime) -> str:
        if runtime.detection is None:
            return "stopline=off"
        detection = runtime.detection
        y_text = (
            f"{detection.maximum_edge_y_px:.0f}px"
            if detection.detected
            else "missing"
        )
        raw_z_text = (
            f"{runtime.depth.camera_z_m:.2f}m"
            if math.isfinite(runtime.depth.camera_z_m)
            else "invalid"
        )
        median_z_text = (
            f"{runtime.median_camera_z_m:.2f}m"
            if math.isfinite(runtime.median_camera_z_m)
            else "pending"
        )
        slope_text = (
            f"{runtime.depth.inverse_depth_slope_per_px:.4f}"
            if math.isfinite(runtime.depth.inverse_depth_slope_per_px)
            else "invalid"
        )
        residual_text = (
            f"{runtime.depth.fit_residual_m:.2f}m"
            if math.isfinite(runtime.depth.fit_residual_m)
            else "invalid"
        )
        row_mad_text = (
            f"{runtime.depth.median_row_depth_mad_m:.2f}m"
            if math.isfinite(runtime.depth.median_row_depth_mad_m)
            else "invalid"
        )
        current_y_ratio_text = (
            f"{runtime.current_y_ratio:.3f}"
            if math.isfinite(runtime.current_y_ratio)
            else "invalid"
        )
        median_y_ratio_text = (
            f"{runtime.median_y_ratio:.3f}"
            if math.isfinite(runtime.median_y_ratio)
            else "pending"
        )
        if (
            self.stopline_stop_distance_m > 0.0
            and self.stopline_stop_y_ratio > 0.0
        ):
            gate_text = "z&y"
        elif self.stopline_stop_distance_m > 0.0:
            gate_text = "z"
        elif self.stopline_stop_y_ratio > 0.0:
            gate_text = "y"
        else:
            gate_text = "off"
        return (
            f"stopline={int(detection.detected)} "
            f"stable={int(runtime.stable)} "
            f"y_raw={y_text} y_ratio={current_y_ratio_text} "
            f"y_med={median_y_ratio_text} "
            f"y_thr={self.stopline_stop_y_ratio:.3f} "
            f"y_ok={int(runtime.y_near)} score={detection.score:.2f} "
            f"width={detection.width_ratio:.0%} "
            f"line_z={raw_z_text} "
            f"z_med={median_z_text} "
            f"accepted={int(runtime.depth.accepted)} "
            f"valid={runtime.depth.valid_ratio:.0%} "
            f"coherent={runtime.depth.coherent_pixel_ratio:.0%} "
            f"rows={runtime.depth.fit_inlier_row_count}/"
            f"{runtime.depth.coherent_row_count}/"
            f"{runtime.depth.valid_row_count} "
            f"slope={slope_text} "
            f"row_mad={row_mad_text} "
            f"fit_err={residual_text} "
            f"samples={runtime.valid_distance_samples}/"
            f"{self.stopline_depth_window} "
            f"z_thr={self.stopline_stop_distance_m:.2f}m "
            f"z_ok={int(runtime.depth_near)} gate={gate_text} "
            f"line_near={int(runtime.near)}"
        )

    def _log_error_noexcept(self, message: str) -> None:
        """안전 복구 경로에서 로깅 장애가 timer를 종료하지 않게 한다."""
        try:
            self.get_logger().error(
                message,
                throttle_duration_sec=2.0,
            )
        except Exception:
            pass

    def tick(self) -> None:
        """인지 예외를 정지 래치로 바꾸고 timer를 유지한다."""
        try:
            self._tick_impl()
        except Exception:
            error_trace = traceback.format_exc()
            self.camera_fault_latched = True
            self.stop_required_latched = True
            try:
                self._publish(True, -1.0)
            except Exception as publish_error:
                self._log_error_noexcept(
                    "fail-safe 정지 발행도 실패: "
                    f"{type(publish_error).__name__}: {publish_error}"
                )
            self._log_error_noexcept(
                "인지 timer 예외; 정지를 래치하며 노드 재시작이 "
                f"필요합니다.\n{error_trace}"
            )

    def _tick_impl(self) -> None:
        success, frame, depth_mm = self._read_camera()
        if not success or frame is None:
            read_status = (
                getattr(self.oak_camera, "last_read_status", "error")
                if self.camera_backend == "oak"
                else "error"
            )
            silence_sec = (
                time.monotonic() - self.last_camera_success_monotonic
            )
            if not camera_poll_timed_out(
                read_status,
                silence_sec,
                self.camera_timeout_sec,
            ):
                return

            self.camera_fault_latched = True
            self.get_logger().error(
                "카메라 프레임 수신 실패: "
                f"status={read_status} silence={silence_sec:.2f}s; "
                "안전을 위해 정지하며 노드를 재시작해야 해제됩니다.",
                throttle_duration_sec=2.0,
            )
            self._publish(True, -1.0)
            return

        self.last_camera_success_monotonic = time.monotonic()
        if (
            self.camera_backend == "oak"
            and getattr(self.oak_camera, "depth_resized", False)
            and not self.depth_resize_logged
        ):
            self.get_logger().info(
                "OAK depth를 RGB 크기로 nearest resize: "
                f"{self.oak_camera.depth_native_shape} -> "
                f"{frame.shape[:2]}"
            )
            self.depth_resize_logged = True

        self.frame_index += 1
        processing_started = time.perf_counter()
        now = time.perf_counter()
        elapsed = now - self.previous_time
        self.previous_time = now
        if elapsed > 0.0:
            instant_fps = 1.0 / elapsed
            self.filtered_fps = (
                instant_fps
                if self.filtered_fps == 0.0
                else 0.9 * self.filtered_fps + 0.1 * instant_fps
            )

        # 정지선 인지는 확정 적색 이후에만 시작한다. 적색과 같은 프레임에서는 아직
        # latch가 갱신되기 전이므로 다음 카메라 프레임(통상 100ms 뒤)부터 시작한다.
        # 비적색 동안의 흰 선 이력은 적색 진입에 섞이지 않도록 비운다.
        if self.red_phase_latched:
            stopline_runtime = self._process_stopline(frame, depth_mm)
        else:
            self.stopline_y_history.clear()
            self.stopline_y_history.extend(
                [math.nan] * self.stopline_detection_window
            )
            self.stopline_distance_history.clear()
            self.stopline_distance_history.extend(
                [math.nan] * self.stopline_depth_window
            )
            stopline_runtime = self._empty_stopline_runtime()

        # CPU 환경에서는 YOLO가 가장 비싸다. 첫 프레임과 지정 간격의
        # 프레임에서만 추론하고, 사이 프레임은 아래 template 추적기로
        # 이어 간다. 건너뛴 프레임은 YOLO miss로 세지 않는다.
        yolo_ran = should_run_yolo(
            self.frame_index,
            self.yolo_inference_interval,
        )
        if yolo_ran:
            detection_frame, detection_roi_bbox = (
                self._prepare_detection_frame(frame)
            )
        else:
            # skip 프레임에서는 ROI 복사도 피한다. 아래 추론·선택 경로는
            # yolo_ran일 때만 detection_frame을 사용한다.
            detection_frame = frame
            detection_roi_bbox = (
                self.last_detection_tile_bbox
                or self._detection_roi_bbox(frame)
            )
        search_roi_bbox = self._detection_roi_bbox(frame)
        results = []
        yolo_inference_ms = 0.0
        if yolo_ran:
            yolo_started = time.perf_counter()
            results = self.model.predict(
                source=detection_frame,
                # 낮은 threshold 후보까지 받은 뒤 신규 검출은 0.20,
                # 기존 target 주변의 연속 후보만 0.10까지 허용한다.
                conf=self.tracking_confidence_threshold,
                imgsz=self.yolo_image_size,
                classes=self.traffic_light_class_ids or None,
                max_det=10,
                rect=True,
                verbose=False,
            )
            yolo_inference_ms = (
                time.perf_counter() - yolo_started
            ) * 1000.0
            if self.startup_hold_latched:
                self.startup_yolo_runs += 1
        tracking_recovered = False
        detection_fresh = False
        bbox_source = "none"
        template_score = 0.0
        detected_bbox: Optional[BBox] = None
        confidence = 0.0
        if yolo_ran:
            detected_bbox, confidence = self._choose_detection_target(
                results,
                detection_frame,
                detection_roi_bbox,
                self.tracked_bbox,
            )
        bbox: Optional[BBox] = None
        if detected_bbox is not None:
            bbox = smooth_bbox(
                self.tracked_bbox,
                detected_bbox,
                frame.shape,
                self.bbox_smoothing_current_weight,
            )
            detection_fresh = True
            bbox_source = "yolo"
            self.tracked_bbox = bbox
            self.tracking_missed_frames = 0
            self.tracking_age_frames = 0
            self.template_tracking_failed_frames = 0
            if self.template_tracking_enabled:
                self.template_tracker.initialize(frame, bbox)
            if (
                self.stop_required_latched
                and self.stop_target_bbox is not None
            ):
                self.stop_target_bbox = bbox
        elif self.tracked_bbox is not None:
            self.tracking_age_frames += 1
            if yolo_ran:
                self.tracking_missed_frames += 1
            if (
                self.template_tracking_enabled
                and self.tracking_age_frames
                <= self.template_tracking_max_age_frames
            ):
                template_result = self.template_tracker.track(frame)
                template_score = template_result.score
                if template_result.bbox is not None:
                    bbox = template_result.bbox
                    bbox_source = "template"
                    self.tracked_bbox = bbox
                    self.template_tracking_failed_frames = 0
                else:
                    self.template_tracking_failed_frames += 1
            else:
                self.template_tracking_failed_frames += 1
            if (
                yolo_ran
                and should_clear_visual_track(
                    yolo_missed_frames=self.tracking_missed_frames,
                    template_failed_frames=(
                        self.template_tracking_failed_frames
                    ),
                    maximum_yolo_misses=self.tracking_max_missed_frames,
                    maximum_template_failures=(
                        self.template_tracking_max_consecutive_failures
                    ),
                )
            ):
                if (
                    self.stop_required_latched
                    and self.stop_target_bbox is not None
                ):
                    # 일반 추적보다 넓되 기존 정지 bbox와 크기·위치가
                    # 이어지는 고신뢰 후보만 복구한다. 복구 뒤 색상 이력을
                    # 비워 새 후보의 연속 관측 없이는 출발하지 못하게 한다.
                    recovery_bbox, recovery_confidence = (
                        self._choose_detection_target(
                            results,
                            detection_frame,
                            detection_roi_bbox,
                            self.stop_target_bbox,
                            tracking_confidence_threshold=(
                                self.confidence_threshold
                            ),
                            tracking_maximum_center_shift_ratio=(
                                self
                                .stopped_reacquire_maximum_center_shift_ratio
                            ),
                            tracking_minimum_size_similarity=(
                                self.stopped_reacquire_minimum_size_similarity
                            ),
                        )
                    )
                    if recovery_bbox is not None:
                        bbox = smooth_bbox(
                            self.stop_target_bbox,
                            recovery_bbox,
                            frame.shape,
                            self.bbox_smoothing_current_weight,
                        )
                        confidence = recovery_confidence
                        self.tracked_bbox = bbox
                        self.stop_target_bbox = bbox
                        self.tracking_missed_frames = 0
                        self.tracking_age_frames = 0
                        self.template_tracking_failed_frames = 0
                        tracking_recovered = True
                        detection_fresh = True
                        bbox_source = "yolo_recovered"
                        if self.template_tracking_enabled:
                            self.template_tracker.initialize(frame, bbox)
                        self._reset_target_histories()
                    else:
                        # 적합 후보가 없으면 정지 래치와 기존 anchor 유지.
                        bbox = None
                        bbox_source = "none"
                        self.tracked_bbox = self.stop_target_bbox
                        self.tracking_missed_frames = (
                            self.tracking_max_missed_frames
                        )
                        self.template_tracker.reset()
                else:
                    # 주행 중 오래 놓친 target은 history를 초기화한 뒤
                    # 정상 confidence 기준으로 새 target을 획득한다.
                    self.tracked_bbox = None
                    self.template_tracker.reset()
                    self.tracking_missed_frames = 0
                    self.tracking_age_frames = 0
                    self.template_tracking_failed_frames = 0
                    self._reset_target_histories()
                    reacquired_bbox, confidence = (
                        self._choose_detection_target(
                            results,
                            detection_frame,
                            detection_roi_bbox,
                            None,
                        )
                    )
                    if reacquired_bbox is not None:
                        bbox = reacquired_bbox
                        detection_fresh = True
                        bbox_source = "yolo_reacquired"
                        self.tracked_bbox = bbox
                        self.tracking_age_frames = 0
                        self.template_tracking_failed_frames = 0
                        if self.template_tracking_enabled:
                            self.template_tracker.initialize(frame, bbox)
                    else:
                        bbox = None
                        bbox_source = "none"
        color_bbox = bbox
        anchored_color = False
        if (
            color_bbox is None
            and self.red_phase_latched
            and self.red_phase_target_bbox is not None
        ):
            # 적색으로 확정했던 동일 housing 위치만 최신 프레임에서 다시 본다.
            # 초록 화살표처럼 점등 모양이 바뀌어 YOLO/template이 target을 놓쳐도
            # 화면의 다른 초록 물체가 아니라 이 anchor 안의 초록만 해제 후보다.
            color_bbox = self.red_phase_target_bbox
            anchored_color = True
        (
            hsv_red_raw,
            hsv_green_raw,
            red_ratio,
            green_ratio,
            crop,
            red_mask,
            green_mask,
        ) = classify_signal_color(
            frame=frame,
            bbox=color_bbox,
            minimum_red_ratio=self.minimum_red_ratio,
            minimum_green_ratio=self.minimum_green_ratio,
            red_hue_upper=self.red_hue_upper,
            red_hue_high_lower=self.red_hue_high_lower,
            minimum_color_saturation=self.minimum_color_saturation,
            minimum_color_value=self.minimum_color_value,
        )
        red_raw = int(bool(hsv_red_raw))
        green_raw = int(bool(hsv_green_raw))
        anchored_green_fresh = should_accept_anchored_green(
            red_phase_latched=self.red_phase_latched,
            anchor_available=anchored_color,
            green_raw=bool(green_raw),
        )
        green_observation_fresh = bool(
            detection_fresh or anchored_green_fresh
        )
        color_source = (
            "hsv_anchor" if anchored_color
            else "hsv" if color_bbox is not None
            else "none"
        )
        # 미검출/unknown을 0표로 넣으면 간헐적인 YOLO miss마다 투표가
        # 씻긴다. 유효 색 관측만 누적하고 target을 잃을 때 전체를 비운다.
        # template 적색은 같은 target에서 fresh YOLO 적색을 최소 한 번
        # 확인한 뒤에만 투표한다. 초록은 fresh YOLO 또는 확정 적색 anchor의
        # 최신 영상에서만, clear는 항상 fresh YOLO에서만 진행한다.
        if detection_fresh and red_raw:
            self.red_fresh_seeded = True
        elif green_observation_fresh and green_raw:
            self.red_fresh_seeded = False
        vote_observation_valid = bool(
            color_bbox is not None
            and should_record_color_vote(
                detection_fresh=green_observation_fresh,
                red_raw=bool(red_raw),
                green_raw=bool(green_raw),
                red_fresh_seeded=bool(
                    self.red_fresh_seeded
                    or self.stop_required_latched
                ),
            )
        )
        if vote_observation_valid:
            self.red_history.append(red_raw)
            self.green_history.append(
                green_raw if green_observation_fresh else 0
            )
            self.bbox_observed_history.append(
                int(green_observation_fresh)
            )
        red_votes = sum(self.red_history)
        green_votes = sum(self.green_history)
        red_active = int(red_votes >= self.minimum_red_votes)
        green_active = int(green_votes >= self.minimum_green_votes)

        # 확정 적색을 실제로 보던 housing 위치를 저장한다. 적색 추적이 이어지는
        # 동안 갱신해 차량 접근에 따른 화면상 이동을 따라가고, stopline 프레임에서
        # bbox가 사라져도 같은 위치의 비원형 초록 점등을 확인할 수 있게 한다.
        if red_active and bbox is not None:
            self.red_phase_target_bbox = bbox

        # 적색과 정지선이 서로 다른 프레임에서 안정 검출되는 실차 패턴을 허용한다.
        # 한 번 확정한 적색은 bbox/YOLO 일시 소실로 해제하지 않고 fresh YOLO 또는
        # 확정 적색 anchor의 초록 3/5만 전환 근거로 쓴다. 동시 활성에서는 적색 우선.
        was_red_phase = self.red_phase_latched
        self.red_phase_latched = update_red_phase_latch(
            current=self.red_phase_latched,
            red_active=bool(red_active),
            green_active=bool(green_active),
        )

        proximity_reached = bool(stopline_runtime.near)
        clear_bbox_observations = sum(self.bbox_observed_history)
        red_clear_active = int(
            is_red_clear_confirmed(
                self.red_history,
                self.bbox_observed_history,
                window_size=self.vote_window,
                minimum_bbox_observations=(
                    self.minimum_red_clear_bbox_observations
                ),
            )
        )
        # 적색 정지 래치:
        # - 진입에는 확정 적색 페이즈 + 정지선 근접 gate 도달이 모두 필요하다.
        # - 신호등 bbox 미검출만으로는 해제하지 않는다.
        # - fresh bbox 또는 확정 적색 anchor 안의 초록 3/5에서 재출발한다.
        was_stopped = self.stop_required_latched
        self.stop_required_latched = update_stop_latch(
            current=self.stop_required_latched,
            red_active=self.red_phase_latched,
            pixel_approaching=proximity_reached,
            green_active=bool(green_active),
            resume_on_green=self.resume_on_green,
            red_clear_active=bool(red_clear_active),
            resume_on_red_clear=self.resume_on_red_clear,
        )
        if not was_stopped and self.stop_required_latched:
            self.stop_target_bbox = (
                bbox if bbox is not None
                else self.tracked_bbox if self.tracked_bbox is not None
                else self.red_phase_target_bbox
            )
        elif was_stopped and not self.stop_required_latched:
            self.stop_target_bbox = None
        if was_red_phase and not self.red_phase_latched:
            self.red_phase_target_bbox = None

        if (
            self.startup_hold_latched
            and self.startup_yolo_runs >= self.vote_window
            and self.frame_index >= self.startup_minimum_frames
        ):
            self.startup_hold_latched = False
            self.get_logger().info(
                "신호등·정지선 시작 판단창 준비 완료; "
                "startup 정지 hold를 해제합니다."
            )

        final_stop = int(
            self.stop_required_latched
            or self.camera_fault_latched
            or self.startup_hold_latched
        )
        published_stopline_distance = (
            stopline_runtime.median_camera_z_m
            if (
                stopline_runtime.stable
                and stopline_runtime.depth.accepted
                and math.isfinite(
                    stopline_runtime.median_camera_z_m
                )
            )
            else -1.0
        )
        metric_stopline_detected = bool(
            stopline_runtime.stable
            and stopline_runtime.depth.accepted
            and math.isfinite(published_stopline_distance)
        )
        self._publish(
            bool(final_stop),
            published_stopline_distance,
            red_active=bool(self.red_phase_latched),
            green_active=bool(green_active and not self.red_phase_latched),
            stopline_detected=metric_stopline_detected,
            fail_safe_stop=bool(
                self.camera_fault_latched or self.startup_hold_latched
            ),
        )

        if self.frame_index % self.print_every == 0:
            processing_ms = (
                time.perf_counter() - processing_started
            ) * 1000.0
            stopline_log = self._stopline_log_text(stopline_runtime)
            self.get_logger().info(
                f"frame={self.frame_index:06d} | "
                f"yolo_run={int(yolo_ran)} "
                f"yolo={int(detection_fresh)} "
                f"yolo_ms={yolo_inference_ms:.1f} "
                f"conf={confidence:.2f} "
                f"bbox_src={bbox_source} "
                f"tmpl={template_score:.2f} | "
                f"track={int(self.tracked_bbox is not None)} "
                f"track_miss={self.tracking_missed_frames}/"
                f"{self.tracking_max_missed_frames} "
                f"track_age={self.tracking_age_frames} "
                f"template_fail="
                f"{self.template_tracking_failed_frames}/"
                f"{self.template_tracking_max_consecutive_failures} "
                f"track_recovered={int(tracking_recovered)} | "
                f"color_src={color_source} "
                f"red_anchor={int(self.red_phase_target_bbox is not None)} "
                f"hsv_red={hsv_red_raw} "
                f"hsv_green={hsv_green_raw} | "
                f"red_raw={red_raw} "
                f"red_votes={red_votes}/{self.vote_window} "
                f"red_active={red_active} "
                f"red_phase={int(self.red_phase_latched)} "
                f"green_raw={green_raw} "
                f"green_votes={green_votes}/{self.vote_window} "
                f"green_active={green_active} | "
                f"red_clear={red_clear_active} | "
                f"clear_bbox={clear_bbox_observations}/"
                f"{self.vote_window} | "
                f"startup_hold={int(self.startup_hold_latched)} "
                f"warmup_yolo={self.startup_yolo_runs}/"
                f"{self.vote_window} | "
                f"{stopline_log} | "
                f"FINAL_STOP={final_stop} | "
                f"proc_ms={processing_ms:.1f} "
                f"fps={self.filtered_fps:.1f}"
            )

        if self.show_debug:
            self._show_debug(
                frame,
                bbox,
                confidence,
                bbox_source,
                template_score,
                red_raw,
                green_raw,
                red_ratio,
                green_ratio,
                hsv_red_raw,
                hsv_green_raw,
                color_source,
                red_votes,
                green_votes,
                search_roi_bbox,
                self.last_detection_tile_bbox or search_roi_bbox,
                depth_mm,
                stopline_runtime,
                final_stop,
                crop,
                red_mask,
                green_mask,
            )

    def _reset_target_histories(self) -> None:
        """서로 다른 신호등의 색 투표가 섞이지 않게 한다."""
        self.red_history.clear()
        self.green_history.clear()
        self.bbox_observed_history.clear()
        self.red_fresh_seeded = False

    def _publish(
        self,
        stop_required: bool,
        stop_distance_m: float,
        red_active: bool = False,
        green_active: bool = False,
        stopline_detected: bool = False,
        fail_safe_stop: bool = True,
    ) -> None:
        msg = TrafficStop()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = (
            "oak_rgb_optical_frame"
            if stop_distance_m >= 0.0
            else "base_link"
        )
        msg.stop_required = stop_required
        msg.stop_distance = float(stop_distance_m)
        msg.red_active = red_active
        msg.green_active = green_active
        msg.stopline_detected = stopline_detected
        msg.fail_safe_stop = fail_safe_stop
        self.publisher.publish(msg)

    def _show_debug(
        self,
        frame: np.ndarray,
        bbox: Optional[BBox],
        confidence: float,
        bbox_source: str,
        template_score: float,
        red_raw: int,
        green_raw: int,
        red_ratio: float,
        green_ratio: float,
        hsv_red_raw: int,
        hsv_green_raw: int,
        color_source: str,
        red_votes: int,
        green_votes: int,
        search_roi_bbox: BBox,
        active_detection_roi_bbox: BBox,
        depth_mm: Optional[np.ndarray],
        stopline_runtime: StopLineRuntime,
        final_stop: int,
        crop: np.ndarray,
        red_mask: np.ndarray,
        green_mask: np.ndarray,
    ) -> None:
        if self.detection_roi_enabled:
            roi_x1, roi_y1, roi_x2, roi_y2 = search_roi_bbox
            full_width_upper_area = (
                roi_x1 == 0
                and roi_y1 == 0
                and roi_x2 == frame.shape[1]
                and roi_y2 < frame.shape[0]
            )
            if full_width_upper_area:
                cv2.line(
                    frame,
                    (0, roi_y2 - 1),
                    (frame.shape[1] - 1, roi_y2 - 1),
                    (255, 0, 255),
                    2,
                )
                roi_label = "YOLO SEARCH: UPPER AREA"
            else:
                cv2.rectangle(
                    frame,
                    (roi_x1, roi_y1),
                    (roi_x2 - 1, roi_y2 - 1),
                    (255, 0, 255),
                    2,
                )
                roi_label = "YOLO SEARCH AREA"
            cv2.putText(
                frame,
                roi_label,
                (roi_x1 + 5, max(22, roi_y1 + 22)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 0, 255),
                2,
            )
            if active_detection_roi_bbox != search_roi_bbox:
                tile_x1, tile_y1, tile_x2, tile_y2 = (
                    active_detection_roi_bbox
                )
                cv2.rectangle(
                    frame,
                    (tile_x1, tile_y1),
                    (tile_x2 - 1, tile_y2 - 1),
                    (255, 255, 0),
                    1,
                )
                cv2.putText(
                    frame,
                    "ACTIVE YOLO TILE",
                    (tile_x1 + 5, max(44, tile_y1 + 44)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50,
                    (255, 255, 0),
                    1,
                )
        if bbox is not None:
            x1, y1, x2, y2 = bbox
            if red_raw:
                box_color = (0, 0, 255)
                color_label = "red"
            elif green_raw:
                box_color = (0, 255, 0)
                color_label = "green"
            else:
                box_color = (180, 180, 180)
                color_label = "unknown"
            bbox_score = (
                confidence
                if bbox_source.startswith("yolo")
                else template_score
            )
            cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
            cv2.putText(
                frame,
                (
                    f"traffic light {bbox_source} "
                    f"{bbox_score:.2f} "
                    f"{color_label} {color_source}"
                ),
                (x1, max(20, y1 - 7)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                box_color,
                2,
            )
        stopline_detection = stopline_runtime.detection
        if stopline_detection is not None:
            line_roi_x1, line_roi_y1, line_roi_x2, line_roi_y2 = (
                stopline_detection.roi_bbox
            )
            cv2.rectangle(
                frame,
                (line_roi_x1, line_roi_y1),
                (line_roi_x2 - 1, line_roi_y2 - 1),
                (255, 255, 0),
                1,
            )
            cv2.putText(
                frame,
                "STOPLINE SEARCH ROI",
                (line_roi_x1 + 5, line_roi_y1 + 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 0),
                2,
            )
            if stopline_detection.bbox is not None:
                line_x1, line_y1, line_x2, line_y2 = (
                    stopline_detection.bbox
                )
                line_color = (
                    (0, 255, 255)
                    if stopline_runtime.stable
                    else (0, 165, 255)
                )
                cv2.rectangle(
                    frame,
                    (line_x1, line_y1),
                    (line_x2, line_y2),
                    line_color,
                    2,
                )
                near_y = int(round(stopline_detection.maximum_edge_y_px))
                cv2.line(
                    frame,
                    (line_x1, near_y),
                    (line_x2, near_y),
                    line_color,
                    3,
                )
                cv2.putText(
                    frame,
                    (
                        f"stopline score={stopline_detection.score:.2f} "
                        f"y_max={near_y}px "
                        f"stable={int(stopline_runtime.stable)}"
                    ),
                    (line_x1, max(20, line_y1 - 7)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    line_color,
                    2,
                )
        if stopline_runtime.depth.sample_bbox is not None:
            sample_x1, sample_y1, sample_x2, sample_y2 = (
                stopline_runtime.depth.sample_bbox
            )
            cv2.rectangle(
                frame,
                (sample_x1, sample_y1),
                (sample_x2, sample_y2),
                (255, 128, 0),
                2,
            )
        lines = [
            (
                f"red_raw={red_raw} "
                f"red_votes={red_votes}/{self.vote_window}"
            ),
            (
                f"green_raw={green_raw} "
                f"green_votes={green_votes}/{self.vote_window}"
            ),
            f"color={color_source}",
            (
                f"hsv_red={hsv_red_raw} hsv_green={hsv_green_raw} "
                f"ratios={red_ratio:.4f}/{green_ratio:.4f}"
            ),
            (
                f"track={int(self.tracked_bbox is not None)} "
                f"miss={self.tracking_missed_frames}/"
                f"{self.tracking_max_missed_frames} "
                f"age={self.tracking_age_frames} "
                f"src={bbox_source} tmpl={template_score:.2f}"
            ),
            self._stopline_debug_line(stopline_runtime),
            f"FINAL_STOP={final_stop} FPS={self.filtered_fps:.1f}",
        ]
        for index, line in enumerate(lines):
            color = (255, 255, 255)
            if index == 0 and red_raw:
                color = (0, 0, 255)
            elif index == 1 and green_raw:
                color = (0, 255, 0)
            elif index == len(lines) - 1 and final_stop:
                color = (0, 0, 255)
            cv2.putText(
                frame,
                line,
                (10, 28 + index * 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56,
                color,
                2,
            )

        cv2.imshow("traffic_red_binary_test", frame)
        if self.show_auxiliary_debug:
            cv2.imshow("traffic_light_crop", crop)
            cv2.imshow("red_mask", red_mask)
            cv2.imshow("green_mask", green_mask)
        if self.show_auxiliary_debug and stopline_detection is not None:
            cv2.imshow("stopline_white_mask", stopline_detection.white_mask)
        if (
            self.show_auxiliary_debug
            and self.camera_backend == "oak"
            and depth_mm is not None
        ):
            depth_view = self._colorize_depth(depth_mm)
            if stopline_runtime.depth.sample_bbox is not None:
                sample_x1, sample_y1, sample_x2, sample_y2 = (
                    stopline_runtime.depth.sample_bbox
                )
                cv2.rectangle(
                    depth_view,
                    (sample_x1, sample_y1),
                    (sample_x2, sample_y2),
                    (255, 255, 255),
                    2,
                )
            cv2.imshow("oak_aligned_depth", depth_view)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            rclpy.shutdown()

    def _stopline_debug_line(self, runtime: StopLineRuntime) -> str:
        if runtime.detection is None:
            return "stopline=off"
        y_text = (
            f"{runtime.detection.maximum_edge_y_px:.0f}px"
            if runtime.detection.detected
            else "missing"
        )
        z_text = (
            f"{runtime.depth.camera_z_m:.2f}m"
            if math.isfinite(runtime.depth.camera_z_m)
            else "invalid"
        )
        median_z_text = (
            f"{runtime.median_camera_z_m:.2f}m"
            if math.isfinite(runtime.median_camera_z_m)
            else "pending"
        )
        median_y_ratio_text = (
            f"{runtime.median_y_ratio:.3f}"
            if math.isfinite(runtime.median_y_ratio)
            else "pending"
        )
        return (
            f"stopline={int(runtime.detection.detected)} "
            f"stable={int(runtime.stable)} y={y_text} "
            f"y_med={median_y_ratio_text}/"
            f"{self.stopline_stop_y_ratio:.3f} "
            f"y_ok={int(runtime.y_near)} "
            f"z={z_text} z_med={median_z_text} "
            f"valid={runtime.depth.valid_ratio:.0%} "
            f"coh={runtime.depth.coherent_pixel_ratio:.0%} "
            f"samples={runtime.valid_distance_samples}/"
            f"{self.stopline_depth_window} "
            f"z_ok={int(runtime.depth_near)} "
            f"near={int(runtime.near)}"
        )

    def _colorize_depth(self, depth_mm: np.ndarray) -> np.ndarray:
        depth_m = np.asarray(depth_mm, dtype=np.float32) / 1000.0
        valid = (
            np.isfinite(depth_m)
            & (depth_m >= self.minimum_depth_m)
            & (depth_m <= self.maximum_depth_m)
        )
        normalized = np.zeros(depth_m.shape, dtype=np.uint8)
        if np.any(valid):
            depth_span = self.maximum_depth_m - self.minimum_depth_m
            normalized[valid] = np.clip(
                (self.maximum_depth_m - depth_m[valid])
                / depth_span
                * 255.0,
                0.0,
                255.0,
            ).astype(np.uint8)
        colorized = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        colorized[~valid] = 0
        return colorized

    def destroy_node(self) -> bool:
        if self.capture is not None:
            self.capture.release()
        if self.oak_camera is not None:
            self.oak_camera.release()
        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[StackTrafficNode] = None
    try:
        node = StackTrafficNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
