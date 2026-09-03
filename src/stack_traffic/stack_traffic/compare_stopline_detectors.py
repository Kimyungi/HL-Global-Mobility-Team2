#!/usr/bin/env python3
"""OAK-D 영상에서 YOLO와 기존 정지선 검출기를 한 화면으로 비교한다.

이 도구는 ROS publisher와 CAN을 만들지 않는 인지 전용 진단 프로그램이다.
빨강은 YOLOv8s-seg stop_line mask, 파랑은 기존 영상처리 검출 결과다.
"""

from __future__ import annotations

from stack_traffic import omp_runtime  # noqa: F401

import argparse
import time
from pathlib import Path

import cv2
import depthai as dai
import numpy as np
from ultralytics import YOLO

from stack_traffic.stopline_detector import detect_stop_line


RED = (0, 0, 255)
BLUE = (255, 0, 0)
WHITE = (255, 255, 255)


def _default_model_path() -> Path:
    from ament_index_python.packages import get_package_share_directory

    return (
        Path(get_package_share_directory("stack_traffic"))
        / "models"
        / "stopline_yolov8s_seg.pt"
    )


def _overlay_mask(
    frame: np.ndarray,
    mask: np.ndarray,
    color: tuple[int, int, int],
    alpha: float = 0.35,
) -> None:
    selected = mask.astype(bool)
    if not np.any(selected):
        return
    color_array = np.asarray(color, dtype=np.float32)
    frame[selected] = np.clip(
        (1.0 - alpha) * frame[selected].astype(np.float32)
        + alpha * color_array,
        0,
        255,
    ).astype(np.uint8)


def _draw_status(
    frame: np.ndarray,
    yolo_detected: bool,
    yolo_confidence: float,
    classic_detected: bool,
    classic_score: float,
    inference_ms: float,
) -> None:
    rows = (
        (f"1 YOLO (RED): {'DETECTED' if yolo_detected else 'no'} "
         f"conf={yolo_confidence:.2f}", RED if yolo_detected else WHITE),
        (f"2 CLASSIC (BLUE): {'DETECTED' if classic_detected else 'no'} "
         f"score={classic_score:.2f}", BLUE if classic_detected else WHITE),
        (f"YOLO inference={inference_ms:.0f} ms | q/ESC: quit", WHITE),
    )
    for index, (text, color) in enumerate(rows):
        origin = (18, 34 + index * 30)
        cv2.putText(
            frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.68,
            (0, 0, 0), 4, cv2.LINE_AA,
        )
        cv2.putText(
            frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.68,
            color, 2, cv2.LINE_AA,
        )


def render_comparison(
    frame: np.ndarray,
    model: YOLO,
    confidence_threshold: float,
    image_size: int,
) -> np.ndarray:
    """한 프레임에 두 검출 결과를 겹쳐 그린다."""
    classic = detect_stop_line(frame)

    started = time.perf_counter()
    result = model.predict(
        frame,
        imgsz=image_size,
        conf=confidence_threshold,
        classes=[0],
        device="cpu",
        retina_masks=True,
        verbose=False,
    )[0]
    inference_ms = (time.perf_counter() - started) * 1000.0

    output = frame.copy()
    yolo_confidence = 0.0
    yolo_detected = bool(result.boxes is not None and len(result.boxes) > 0)
    if yolo_detected:
        yolo_confidence = max(float(value) for value in result.boxes.conf)
    if result.masks is not None:
        for mask in result.masks.data.cpu().numpy():
            if mask.shape != frame.shape[:2]:
                mask = cv2.resize(
                    mask,
                    (frame.shape[1], frame.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                )
            binary = mask >= 0.5
            _overlay_mask(output, binary, RED)
            contours, _ = cv2.findContours(
                binary.astype(np.uint8), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            cv2.drawContours(output, contours, -1, RED, 2)

    if classic.detected and classic.bbox is not None:
        x1, y1, x2, y2 = classic.bbox
        cv2.rectangle(output, (x1, y1), (x2, y2), BLUE, 3)
        cv2.line(
            output,
            (x1, int(round(classic.near_edge_y_px))),
            (x2, int(round(classic.near_edge_y_px))),
            BLUE,
            2,
        )

    _draw_status(
        output,
        yolo_detected,
        yolo_confidence,
        classic.detected,
        classic.score,
        inference_ms,
    )
    return output


def main() -> None:
    parser = argparse.ArgumentParser(
        description="OAK-D YOLO(빨강)/기존 로직(파랑) 정지선 비교"
    )
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--conf", type=float, default=0.10)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--camera-fps", type=float, default=10.0)
    args = parser.parse_args()

    model_path = (args.model or _default_model_path()).expanduser().resolve()
    if not model_path.is_file():
        parser.error(f"모델 파일을 찾을 수 없습니다: {model_path}")
    if not 0.0 < args.conf <= 1.0:
        parser.error("--conf는 0보다 크고 1 이하여야 합니다.")

    model = YOLO(str(model_path))
    with dai.Pipeline() as pipeline:
        camera = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_A
        )
        queue = camera.requestOutput(
            size=(args.width, args.height),
            type=dai.ImgFrame.Type.BGR888p,
            resizeMode=dai.ImgResizeMode.CROP,
            fps=args.camera_fps,
        ).createOutputQueue()
        pipeline.start()
        print("정지선 비교 실행 | YOLO=빨강, 기존 로직=파랑 | q/ESC 종료")
        try:
            while pipeline.isRunning():
                packet = queue.get()
                if packet is None:
                    continue
                output = render_comparison(
                    packet.getCvFrame(), model, args.conf, args.imgsz
                )
                cv2.imshow("Stop-line comparison: YOLO RED / classic BLUE", output)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
        finally:
            pipeline.stop()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
