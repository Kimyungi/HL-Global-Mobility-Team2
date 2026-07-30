#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""재민님의 traffic_red_binary_test.py를 ROS 2에 연결한 신호등 정지 노드.

신호등 판정 로직은 원본과 동일하다.
- YOLOv8n으로 traffic light 위치 검출
- HSV 빨간색 픽셀 비율로 red_raw 0/1 판정
- 최근 5프레임 중 빨간불 3프레임 이상이면 red_active
- red_active AND 정지선 거리 임계값 이내이면 stop_required 래치
- 래치 후에는 정지선이 사라져도 유지하고 red_active=False일 때만 해제

정지선 거리는 이현준의 /perception/stopline (StopLine)을 입력받고,
결과는 /perception/traffic_stop (TrafficStop)으로 발행한다.
"""

from __future__ import annotations

import math
import time
from collections import deque
from pathlib import Path
from typing import Deque, Optional, Tuple, Union

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from fma_interfaces.msg import StopLine, TrafficStop
from rclpy.node import Node

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


CameraSource = Union[int, str]
BBox = Tuple[int, int, int, int]


def parse_camera_source(value: str) -> CameraSource:
    value = value.strip()
    if value.lstrip("-").isdigit():
        return int(value)
    return value


def get_class_name(names, class_id: int) -> str:
    if isinstance(names, dict):
        return str(names.get(class_id, class_id))
    if isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
        return str(names[class_id])
    return str(class_id)


def is_traffic_light_class(class_name: str) -> bool:
    normalized = class_name.lower().replace("_", " ").strip()
    return "traffic light" in normalized


def choose_target_traffic_light(
    results,
    frame_shape: Tuple[int, ...],
    confidence_threshold: float,
    minimum_box_area: int,
) -> Tuple[Optional[BBox], float]:
    """여러 신호등 중 confidence와 화면 위치를 이용해 하나를 선택합니다."""
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

    for box in result.boxes:
        confidence = float(box.conf[0].item())
        if confidence < confidence_threshold:
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
            best_bbox = (x1, y1, x2, y2)
            best_confidence = confidence

    return best_bbox, best_confidence


def classify_red_binary(
    frame: np.ndarray,
    bbox: Optional[BBox],
    minimum_red_ratio: float,
) -> Tuple[int, float, np.ndarray, np.ndarray]:
    """
    반환값
    - red_raw: 빨간불이면 1, 아니면 0
    - red_ratio: ROI 전체 중 빨간색 픽셀 비율
    - crop: 신호등 검출 영역
    - red_mask: 빨간색 마스크
    """
    if bbox is None:
        empty_crop = frame[0:1, 0:1]
        empty_mask = np.zeros((1, 1), dtype=np.uint8)
        return 0, 0.0, empty_crop, empty_mask

    x1, y1, x2, y2 = bbox
    crop = frame[y1:y2, x1:x2]

    if crop.size == 0:
        empty_crop = frame[0:1, 0:1]
        empty_mask = np.zeros((1, 1), dtype=np.uint8)
        return 0, 0.0, empty_crop, empty_mask

    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

    red_mask_1 = cv2.inRange(
        hsv,
        np.array([0, 60, 60], dtype=np.uint8),
        np.array([10, 255, 255], dtype=np.uint8),
    )
    red_mask_2 = cv2.inRange(
        hsv,
        np.array([170, 60, 60], dtype=np.uint8),
        np.array([179, 255, 255], dtype=np.uint8),
    )
    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

    kernel = np.ones((3, 3), dtype=np.uint8)
    red_mask = cv2.morphologyEx(
        red_mask,
        cv2.MORPH_OPEN,
        kernel,
    )

    roi_pixels = max(1, crop.shape[0] * crop.shape[1])
    red_ratio = cv2.countNonZero(red_mask) / roi_pixels

    red_raw = int(red_ratio >= minimum_red_ratio)

    return red_raw, red_ratio, crop, red_mask


def update_stop_latch(
    current: bool,
    red_active: bool,
    stopline_approaching: bool,
) -> bool:
    """적색 해제만 기존 정지 래치를 해제할 수 있다."""
    if not red_active:
        return False
    if stopline_approaching:
        return True
    return current


class StackTrafficNode(Node):
    def __init__(self) -> None:
        super().__init__("stack_traffic_node")
        self._declare_parameters()
        self._load_parameters()

        if YOLO is None:
            raise RuntimeError(
                "ultralytics가 설치되어 있지 않습니다: "
                "python3 -m pip install ultralytics"
            )

        model_path = self._resolve_model_path()
        if not model_path.exists():
            raise FileNotFoundError(
                f"YOLO 모델 파일을 찾을 수 없습니다: {model_path}. "
                "models/yolov8n.pt에 배치하거나 "
                "-p model_path:=/path/to/model.pt로 지정하세요."
            )

        self.model = YOLO(str(model_path))
        self.camera_source = parse_camera_source(self.camera_source_text)
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

        self.publisher = self.create_publisher(
            TrafficStop, "/perception/traffic_stop", 1
        )
        self.stopline_subscription = self.create_subscription(
            StopLine,
            self.stopline_topic,
            self._on_stopline,
            1,
        )

        self.red_history: Deque[int] = deque(maxlen=self.vote_window)
        self.stop_required_latched = False
        self.stopline_detected = False
        self.stopline_distance_m = -1.0
        self.stopline_stamp_ns: Optional[int] = None
        self.frame_index = 0
        self.previous_time = time.perf_counter()
        self.filtered_fps = 0.0
        self.timer = self.create_timer(self.process_period_sec, self.tick)

        self.get_logger().info(
            "traffic_red_binary ROS 2 started | "
            f"model={model_path} camera={self.camera_source} "
            f"vote={self.vote_window}/{self.minimum_red_votes} "
            f"stop_threshold={self.stop_trigger_distance_m:.2f}m "
            f"stopline_topic={self.stopline_topic}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("model_path", "")
        self.declare_parameter("camera_source", "2")
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("process_period_sec", 0.10)
        self.declare_parameter("confidence_threshold", 0.20)
        self.declare_parameter("minimum_box_area", 24)
        self.declare_parameter("minimum_red_ratio", 0.004)
        self.declare_parameter("vote_window", 5)
        self.declare_parameter("minimum_red_votes", 3)
        self.declare_parameter(
            "stopline_topic", "/perception/stopline"
        )
        self.declare_parameter("stop_trigger_distance_m", 0.5)
        self.declare_parameter("stopline_timeout_sec", 0.5)
        self.declare_parameter("show_debug", False)
        self.declare_parameter("print_every", 10)

    def _load_parameters(self) -> None:
        self.model_path = str(self.get_parameter("model_path").value)
        self.camera_source_text = str(
            self.get_parameter("camera_source").value
        )
        self.camera_width = int(self.get_parameter("camera_width").value)
        self.camera_height = int(self.get_parameter("camera_height").value)
        self.process_period_sec = float(
            self.get_parameter("process_period_sec").value
        )
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.minimum_box_area = int(
            self.get_parameter("minimum_box_area").value
        )
        self.minimum_red_ratio = float(
            self.get_parameter("minimum_red_ratio").value
        )
        self.vote_window = int(self.get_parameter("vote_window").value)
        self.minimum_red_votes = int(
            self.get_parameter("minimum_red_votes").value
        )
        self.stopline_topic = str(
            self.get_parameter("stopline_topic").value
        )
        self.stop_trigger_distance_m = float(
            self.get_parameter("stop_trigger_distance_m").value
        )
        self.stopline_timeout_sec = float(
            self.get_parameter("stopline_timeout_sec").value
        )
        self.show_debug = bool(self.get_parameter("show_debug").value)
        self.print_every = max(
            1, int(self.get_parameter("print_every").value)
        )

        if self.vote_window < 1:
            raise ValueError("vote_window은 1 이상이어야 합니다.")
        if not 1 <= self.minimum_red_votes <= self.vote_window:
            raise ValueError(
                "minimum_red_votes는 1 이상 vote_window 이하여야 합니다."
            )
        if self.stop_trigger_distance_m < 0.0:
            raise ValueError(
                "stop_trigger_distance_m은 0 이상이어야 합니다."
            )
        if self.stopline_timeout_sec <= 0.0:
            raise ValueError("stopline_timeout_sec은 0보다 커야 합니다.")

    def _resolve_model_path(self) -> Path:
        if self.model_path.strip():
            return Path(self.model_path).expanduser().resolve()

        share_dir = Path(get_package_share_directory("stack_traffic"))
        return share_dir / "models" / "yolov8n.pt"

    def _on_stopline(self, msg: StopLine) -> None:
        self.stopline_detected = bool(msg.detected)
        self.stopline_distance_m = float(msg.distance)
        self.stopline_stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )

    def _get_stopline_distance(self) -> float:
        if self.stopline_stamp_ns is None or not self.stopline_detected:
            return -1.0
        age_sec = (
            self.get_clock().now().nanoseconds - self.stopline_stamp_ns
        ) / 1_000_000_000.0
        distance = self.stopline_distance_m
        if (
            age_sec < 0.0
            or age_sec > self.stopline_timeout_sec
            or not math.isfinite(distance)
            or distance < 0.0
        ):
            return -1.0
        return distance

    def tick(self) -> None:
        success, frame = self.capture.read()
        if not success or frame is None:
            self.get_logger().error("카메라 프레임 수신 실패")
            # 안전 원칙: 적색 정지 중 카메라가 죽어도 재출발시키지 않는다.
            self._publish(
                self.stop_required_latched,
                self._get_stopline_distance(),
            )
            return

        self.frame_index += 1
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

        results = self.model.predict(
            source=frame,
            conf=self.confidence_threshold,
            verbose=False,
        )
        bbox, confidence = choose_target_traffic_light(
            results=results,
            frame_shape=frame.shape,
            confidence_threshold=self.confidence_threshold,
            minimum_box_area=self.minimum_box_area,
        )
        red_raw, red_ratio, crop, red_mask = classify_red_binary(
            frame=frame,
            bbox=bbox,
            minimum_red_ratio=self.minimum_red_ratio,
        )

        # traffic_red_binary_test.py 원본과 동일한 0/1 투표 로직
        self.red_history.append(red_raw)
        red_votes = sum(self.red_history)
        red_active = int(red_votes >= self.minimum_red_votes)

        stopline_distance_m = self._get_stopline_distance()
        stopline_approaching = int(
            stopline_distance_m >= 0.0
            and stopline_distance_m <= self.stop_trigger_distance_m
        )
        # 적색 정지 래치:
        # - 진입에는 적색 + 정지선 접근이 모두 필요하다.
        # - 진입 후 정지선 미검출/stale은 해제 조건이 아니다.
        # - 정상 처리 프레임에서 적색 투표가 해제될 때만 출발을 허용한다.
        self.stop_required_latched = update_stop_latch(
            current=self.stop_required_latched,
            red_active=bool(red_active),
            stopline_approaching=bool(stopline_approaching),
        )

        final_stop = int(self.stop_required_latched)
        self._publish(bool(final_stop), stopline_distance_m)

        if self.frame_index % self.print_every == 0:
            self.get_logger().info(
                f"frame={self.frame_index:06d} | "
                f"yolo={int(bbox is not None)} conf={confidence:.2f} | "
                f"red_raw={red_raw} "
                f"red_votes={red_votes}/{self.vote_window} "
                f"red_active={red_active} | "
                f"stopline={stopline_distance_m:.2f}m "
                f"approaching={stopline_approaching} | "
                f"FINAL_STOP={final_stop} | fps={self.filtered_fps:.1f}"
            )

        if self.show_debug:
            self._show_debug(
                frame,
                bbox,
                confidence,
                red_raw,
                red_ratio,
                red_votes,
                stopline_distance_m,
                final_stop,
                crop,
                red_mask,
            )

    def _publish(
        self, stop_required: bool, stop_distance: float
    ) -> None:
        msg = TrafficStop()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.stop_required = stop_required
        msg.stop_distance = float(stop_distance)
        self.publisher.publish(msg)

    def _show_debug(
        self,
        frame: np.ndarray,
        bbox: Optional[BBox],
        confidence: float,
        red_raw: int,
        red_ratio: float,
        red_votes: int,
        stopline_distance_m: float,
        final_stop: int,
        crop: np.ndarray,
        red_mask: np.ndarray,
    ) -> None:
        if bbox is not None:
            x1, y1, x2, y2 = bbox
            box_color = (0, 0, 255) if red_raw else (180, 180, 180)
            cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
            cv2.putText(
                frame,
                f"traffic light {confidence:.2f}",
                (x1, max(20, y1 - 7)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                box_color,
                2,
            )

        lines = [
            (
                f"red_raw={red_raw} "
                f"red_votes={red_votes}/{self.vote_window}"
            ),
            f"red_ratio={red_ratio:.4f}",
            (
                f"stopline={stopline_distance_m:.2f}m "
                f"threshold={self.stop_trigger_distance_m:.2f}m"
            ),
            f"FINAL_STOP={final_stop} FPS={self.filtered_fps:.1f}",
        ]
        for index, line in enumerate(lines):
            color = (0, 0, 255) if final_stop else (255, 255, 255)
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
        cv2.imshow("traffic_light_crop", crop)
        cv2.imshow("red_mask", red_mask)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            rclpy.shutdown()

    def destroy_node(self) -> bool:
        if hasattr(self, "capture"):
            self.capture.release()
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
