"""YOLOPv2 주행가능영역/차선 세그멘테이션 육안 확인용 스크립트.

목적: ref point 추출 로직(호모그래피, BEV, 슬라이딩 윈도우 등)을 만들기 전에
"모델이 실제로 차선을 잡아내는지"부터 눈으로 확인한다. ROS 노드가 아니고
MGM 파이프라인과 무관한 독립 개발 도구다.

실제 추론 로직(전처리/후처리)은 stack_lane.yolopv2_infer로 옮겨서
visualize_lane_fit.py와 공유한다 — 이 파일은 프레임 소스 + 오버레이만 담당.

사용 예:
  정적 이미지로 파이프라인 검증:
    python3 visualize_yolopv2.py --source ../models/../sample.jpg --device cpu
  OAK-D Pro 라이브:
    python3 visualize_yolopv2.py --source oak --device 0
  일반 웹캠:
    python3 visualize_yolopv2.py --source 0 --device 0
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from stack_lane.yolopv2_infer import (  # noqa: E402
    DEFAULT_WEIGHTS,
    infer,
    load_model,
    preprocess,
    resolve_device,
)


def overlay(canvas: np.ndarray, da_mask: np.ndarray, ll_mask: np.ndarray) -> np.ndarray:
    color = np.zeros_like(canvas)
    color[da_mask == 1] = (0, 255, 0)  # 주행가능영역 = 초록
    color[ll_mask == 1] = (0, 0, 255)  # 차선 = 빨강 (BGR)
    hit = color.any(axis=2)
    blended = canvas.copy()
    blended[hit] = (canvas[hit] * 0.5 + color[hit] * 0.5).astype(np.uint8)
    return blended


def oak_frames():
    import depthai as dai

    pipeline = dai.Pipeline()
    cam = pipeline.createColorCamera()
    cam.setPreviewSize(1280, 720)
    cam.setInterleaved(False)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setFps(30)
    xout = pipeline.createXLinkOut()
    xout.setStreamName("rgb")
    cam.preview.link(xout.input)

    with dai.Device(pipeline) as device:
        q = device.getOutputQueue("rgb", maxSize=4, blocking=False)
        while True:
            yield q.get().getCvFrame()


def file_or_webcam_frames(source: str):
    idx_or_path = int(source) if source.isdigit() else source
    cap = cv2.VideoCapture(idx_or_path)
    if not cap.isOpened():
        raise RuntimeError(f"소스를 열 수 없음: {source}")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            yield frame
    finally:
        cap.release()


def frame_source(source: str):
    if source == "oak":
        yield from oak_frames()
    else:
        yield from file_or_webcam_frames(source)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", default=str(DEFAULT_WEIGHTS), help="YOLOPv2 TorchScript(.pt) 경로")
    parser.add_argument("--source", default="oak", help="'oak' | 웹캠 인덱스 | 이미지/동영상 경로")
    parser.add_argument("--img-size", type=int, default=640, help="official demo 기준값(변경 시 크롭 상수 12:372도 같이 바꿔야 함)")
    parser.add_argument("--device", default="0", help="'cpu' 또는 cuda 인덱스(기본 '0')")
    parser.add_argument("--save", default=None, help="주석 처리된 결과를 저장할 mp4/이미지 경로 (선택)")
    parser.add_argument("--headless", action="store_true", help="창 없이 --save로만 저장 (원격 환경용)")
    parser.add_argument("--max-frames", type=int, default=None, help="이 프레임 수만큼만 처리 후 종료 (라이브 소스 스모크 테스트용)")
    args = parser.parse_args()

    device, half = resolve_device(args.device)
    model = load_model(args.weights, device, half)

    writer = None
    frame_count = 0
    t_start = time.perf_counter()

    for frame in frame_source(args.source):
        canvas, tensor = preprocess(frame, args.img_size, device, half)

        t0 = time.perf_counter()
        da_mask, ll_mask = infer(model, tensor)
        if device.type == "cuda":
            import torch
            torch.cuda.synchronize()
        dt_ms = (time.perf_counter() - t0) * 1000

        vis = overlay(canvas, da_mask, ll_mask)

        frame_count += 1
        elapsed = time.perf_counter() - t_start
        fps = frame_count / elapsed if elapsed > 0 else 0.0
        cv2.putText(vis, f"{dt_ms:.1f} ms/frame  ({fps:.1f} fps avg)", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        if args.save:
            if writer is None:
                h, w = vis.shape[:2]
                if str(args.save).lower().endswith((".jpg", ".png")):
                    cv2.imwrite(args.save, vis)
                    writer = "image_written"
                else:
                    writer = cv2.VideoWriter(args.save, cv2.VideoWriter_fourcc(*"mp4v"), 15, (w, h))
            if writer not in (None, "image_written"):
                writer.write(vis)

        if not args.headless:
            cv2.imshow("YOLOPv2 lane check (q to quit)", vis)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        elif args.save and writer == "image_written":
            break  # 단일 이미지 소스 + headless 저장이면 첫 프레임 후 종료

        if args.max_frames is not None and frame_count >= args.max_frames:
            break

    if isinstance(writer, cv2.VideoWriter):
        writer.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
